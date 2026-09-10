"""Presentation presets must display real motion before a physical change."""

from dataclasses import replace
import math
import unittest

import numpy as np

from simulator.platforms import car_damage, quadruped, warehouse


class ShowcaseProfileTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.runs = {}
        for name, module in (("car_demo", car_damage), ("quadruped_demo", quadruped),
                             ("warehouse_demo", warehouse)):
            config = module.Config(**module.PRESETS[name])
            actual = module.Simulation(config)
            healthy = module.Simulation(replace(config, fault="healthy"))
            initial = np.array(actual.observe()["position"])
            fault_event = "structural_damage" if module is car_damage else config.fault
            prefix_equal = True
            before_fault = initial.copy()
            fault_time = None
            max_error = 0.
            post_fault_distance = 0.
            previous = initial.copy()
            for _ in range(math.ceil(config.duration / config.timestep)):
                healthy.step()
                actual.step()
                position = np.array(actual.observe()["position"])
                events = actual.diagnostics()["events"]
                faults = [event for event in events if event["event"] == fault_event]
                if faults:
                    fault_time = faults[0]["time"]
                    repositioned = any(event["event"] == "recovery_reposition" and
                                       abs(event["time"] - actual.elapsed) < config.timestep / 2
                                       for event in events)
                    if not repositioned:  # Declared fixture placement is not physical driving.
                        post_fault_distance += float(np.linalg.norm(position[:2] - previous[:2]))
                else:
                    before_fault = position.copy()
                    prefix_equal &= bool(np.array_equal(actual.data.qpos, healthy.data.qpos))
                    prefix_equal &= bool(np.array_equal(actual.data.qvel, healthy.data.qvel))
                max_error = max(max_error, float(np.linalg.norm(position - healthy.observe()["position"])))
                previous = position
            cls.runs[name] = {"actual": actual, "healthy": healthy, "prefix_equal": prefix_equal,
                              "prefix_distance": float(np.linalg.norm(before_fault[:2] - initial[:2])),
                              "post_fault_distance": post_fault_distance,
                              "fault_time": fault_time, "max_error": max_error}

    def test_showcases_have_visible_healthy_motion_and_complete_without_solver_warnings(self):
        for name, run in self.runs.items():
            with self.subTest(scenario=name):
                self.assertTrue(run["prefix_equal"], "Faults must not alter the healthy approach")
                self.assertGreater(run["prefix_distance"], .5)
                self.assertIsNotNone(run["fault_time"])
                self.assertGreater(run["post_fault_distance"], .5)
                for sim in (run["healthy"], run["actual"]):
                    self.assertTrue(sim.finished)
                    self.assertTrue(np.isfinite(sim.data.qpos).all())
                    self.assertTrue(np.isfinite(sim.data.qvel).all())
                    self.assertFalse(sim.data.warning.number.any())
                    self.assertFalse(any(sim.summary()["warnings"]))
                self.assertTrue(run["healthy"].summary()["public"]["safe"])

    def test_car_demo_uses_contact_damage_then_a_changed_steering_probe(self):
        run = self.runs["car_demo"]
        sim = run["actual"]
        diagnostics = sim.diagnostics()
        self.assertGreater(run["fault_time"], sim.config.fault_at)
        self.assertGreater(diagnostics["peak_impact_force"], sim.config.impact_threshold)
        self.assertEqual([event["event"] for event in diagnostics["events"]],
                         ["barrier_impact", "structural_damage", "recovery_reposition"])
        self.assertAlmostEqual(diagnostics["rack_gain"], sim.config.steering_gain)
        self.assertAlmostEqual(diagnostics["rack_bias"], sim.config.steering_bias)
        self.assertGreater(run["max_error"], 1)
        self.assertGreater(sim.summary()["public"]["metrics"]["probe_elapsed"], 6)
        self.assertEqual(sim.summary()["public"]["outcome"], "probe_outside_envelope")

    def test_dog_demo_walks_then_falls_and_replays_the_aftermath(self):
        run = self.runs["quadruped_demo"]
        sim, healthy = run["actual"], run["healthy"]
        actual = sim.summary()["public"]
        nominal = healthy.summary()["public"]
        self.assertGreater(run["prefix_distance"], 1)
        self.assertAlmostEqual(run["fault_time"], sim.config.fault_at)
        self.assertTrue(nominal["safe"])
        self.assertFalse(actual["safe"])
        self.assertEqual(actual["outcome"], "fell")
        self.assertGreater(actual["metrics"]["fall_time"], run["fault_time"] + 1)
        self.assertLess(actual["metrics"]["fall_time"], sim.config.duration - 2)
        self.assertLess(actual["metrics"]["min_body_height_m"], .15)
        self.assertGreater(actual["metrics"]["max_tilt_deg"], 90)
        self.assertAlmostEqual(actual["metrics"]["duration_s"], sim.config.duration)
        self.assertEqual(sim.diagnostics()["actuator_strength"]["FL_knee"], sim.config.strength)
        self.assertGreater(run["max_error"], .5)
        np.testing.assert_array_equal(sim.data.qfrc_applied, np.zeros(sim.model.nv))
        np.testing.assert_array_equal(sim.data.xfrc_applied, np.zeros((sim.model.nbody, 6)))

        final_position, final_velocity = sim.data.qpos.copy(), sim.data.qvel.copy()
        expected = sim.summary()
        sim.reset_full()
        at_fall = None
        while not sim.finished:
            sim.step()
            if at_fall is None and sim.elapsed >= actual["metrics"]["fall_time"]:
                at_fall = sim.data.qpos.copy()
        self.assertIsNotNone(at_fall)
        self.assertGreater(np.linalg.norm(sim.data.qpos[:3] - at_fall[:3]), .05)
        np.testing.assert_array_equal(sim.data.qpos, final_position)
        np.testing.assert_array_equal(sim.data.qvel, final_velocity)
        self.assertEqual(sim.summary(), expected)

    def test_warehouse_demo_physically_shifts_cargo_during_turns(self):
        run = self.runs["warehouse_demo"]
        sim = run["actual"]
        diagnostics = sim.diagnostics()
        self.assertAlmostEqual(run["fault_time"], sim.config.fault_at)
        self.assertTrue(sim.summary()["public"]["safe"])
        self.assertFalse(diagnostics["cargo_latched"])
        self.assertAlmostEqual(diagnostics["cargo_displacement"], sim.config.shift_distance, delta=.01)
        self.assertAlmostEqual(sim.model.body_mass.sum(), run["healthy"].model.body_mass.sum())
        self.assertGreater(run["max_error"], .5)
        self.assertGreater(sim.summary()["public"]["metrics"]["path_length_m"], 3)


if __name__ == "__main__":
    unittest.main()
