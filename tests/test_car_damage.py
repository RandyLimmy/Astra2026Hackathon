"""Physical contact gates, persistent damage, matched probes, and reset replay."""

from dataclasses import fields, replace
import json
import unittest

import numpy as np

from simulator.platforms.car_damage import Config, PRESETS, Simulation


def advance(sim, until):
    while sim.elapsed < until - sim.config.timestep / 2 and not sim.finished:
        sim.step()


class CarDamageTests(unittest.TestCase):
    def test_config_and_commands_reject_invalid_inputs_without_advancing(self):
        config = Config()
        for field in fields(config):
            if isinstance(getattr(config, field.name), (int, float)):
                with self.subTest(field=field.name), self.assertRaises(ValueError):
                    replace(config, **{field.name: float("nan")})
        for kwargs in ({"timestep": 0}, {"probe": "magic"}, {"fault": "magic"},
                       {"suspension_scale": 0}, {"pressure_radius_scale": 1.1}):
            with self.assertRaises(ValueError):
                Config(**kwargs)
        sim = Simulation(Config())
        before = sim.data.qpos.copy()
        for control in ({"steering": 1}, {"brake": -1}, {"throttle": float("inf")},
                        {"fault": "healthy"}, {"brake": True}):
            with self.assertRaises(ValueError):
                sim.step(control)
        np.testing.assert_array_equal(before, sim.data.qpos)
        self.assertEqual(sim.elapsed, 0)

    def test_damage_requires_armed_actual_barrier_contact(self):
        for kwargs in ({"approach_y": 8}, {"barrier_x": 100}, {"fault_at": 20},
                       {"impact_speed": 0}):
            with self.subTest(kwargs=kwargs):
                sim = Simulation(Config(duration=3, **kwargs))
                advance(sim, 3)
                self.assertFalse(sim.diagnostics()["damage_active"])
                self.assertIsNone(sim.diagnostics()["impact_time"])
                self.assertEqual(sim.summary()["events"], [])
                self.assertFalse(sim.summary()["public"]["safe"])

    def test_impact_damage_preserves_physical_state_and_identical_healthy_approach(self):
        for fault in ("steering", "alignment", "suspension", "pressure"):
            with self.subTest(fault=fault):
                actual = Simulation(Config(fault=fault, duration=2))
                nominal = Simulation(replace(actual.config, fault="healthy"))
                while actual.diagnostics()["impact_time"] is None:
                    actual.step()
                    nominal.step()
                    # The damage update may change forces, never position or
                    # velocity: mj_setConst scratch state must not teleport.
                    np.testing.assert_array_equal(actual.data.qpos, nominal.data.qpos)
                    np.testing.assert_array_equal(actual.data.qvel, nominal.data.qvel)
                d = actual.diagnostics()
                self.assertTrue(d["damage_active"])
                self.assertGreaterEqual(d["peak_impact_force"], actual.config.impact_threshold)
                self.assertGreater(d["impact_time"], 0.9)
                self.assertEqual(d["events"][1]["gate"], "measured_barrier_contact")

    def test_trial_reset_retains_damage_and_full_reset_replays_original_config(self):
        for fault, field in (("steering", "rack_gain"), ("alignment", "toe_quaternion"),
                             ("suspension", "spring_stiffness"), ("pressure", "tire_radius")):
            with self.subTest(fault=fault):
                sim = Simulation(Config(fault=fault, duration=4))
                advance(sim, 2)
                value, before = sim.diagnostics()[field], sim.elapsed
                sim.reset_trial()
                self.assertEqual(sim.elapsed, before)
                self.assertEqual(sim.data.time, before)
                self.assertEqual(sim.diagnostics()[field], value)
                self.assertTrue(sim.diagnostics()["damage_active"])
                self.assertTrue(sim.diagnostics()["events"][-1]["retained_damage"])
                self.assertEqual(sim.observe()["phase"], "controlled_probe")
                advance(sim, 2.2)
                self.assertEqual(sim.diagnostics()[field], value)
                sim.reset_full()
                self.assertEqual(sim.elapsed, 0)
                self.assertFalse(sim.diagnostics()["damage_active"])
                self.assertEqual(sim.config.fault, fault)
                replay = Simulation(sim.config)
                np.testing.assert_array_equal(sim.data.qpos, replay.data.qpos)
                advance(sim, 2)
                advance(replay, 2)
                np.testing.assert_array_equal(sim.data.qpos, replay.data.qpos)
                np.testing.assert_array_equal(sim.data.qvel, replay.data.qvel)

    def test_matching_probes_expose_distinct_physical_damage_without_warnings(self):
        comparisons = {}
        for name, kwargs in PRESETS.items():
            if name.endswith("_demo"):
                continue  # Separate presentation calibration lives in test_showcase_profiles.
            if kwargs["fault"] == "healthy":
                continue
            config = Config(**kwargs)
            results = []
            for fault in ("healthy", config.fault):
                sim = Simulation(replace(config, fault=fault))
                advance(sim, config.duration)
                result = sim.summary()
                json.dumps(result, allow_nan=False)
                self.assertFalse(any(result["warnings"]), name)
                self.assertTrue(np.isfinite(sim.data.qpos).all())
                self.assertEqual(len([e for e in result["events"] if e["event"] == "barrier_impact"]), 1)
                self.assertEqual(len([e for e in result["events"] if e["event"] == "recovery_reposition"]), 1)
                impact = next(e for e in result["events"] if e["event"] == "barrier_impact")
                self.assertGreater(impact["position"][0], 5, "The car must drive into the barrier")
                self.assertGreater(result["public"]["metrics"]["distance_traveled"], 10,
                                   "Inspection travel excludes the declared recovery placement")
                self.assertLess(result["public"]["metrics"]["final_speed"], .1)
                self.assertTrue(sim.finished)
                results.append(result["public"])
            comparisons[config.fault] = results
            self.assertTrue(results[0]["safe"], name)
        nominal, damaged = comparisons["steering"]
        self.assertLess(nominal["metrics"]["max_lateral_displacement"], 2)
        self.assertGreater(damaged["metrics"]["max_lateral_displacement"], 20)
        self.assertFalse(damaged["safe"])
        nominal, damaged = comparisons["alignment"]
        self.assertGreater(damaged["metrics"]["max_lateral_displacement"],
                           nominal["metrics"]["max_lateral_displacement"] + 8)
        self.assertFalse(damaged["safe"])
        nominal, damaged = comparisons["suspension"]
        self.assertGreater(damaged["metrics"]["max_suspension_travel"],
                           nominal["metrics"]["max_suspension_travel"] + 0.08)
        self.assertFalse(damaged["safe"])
        nominal, damaged = comparisons["pressure"]
        self.assertGreater(damaged["metrics"]["braking_distance"],
                           nominal["metrics"]["braking_distance"] * 1.08)
        self.assertGreater(damaged["metrics"]["max_roll_rad"], 0.01)
        self.assertTrue(damaged["safe"], "Longer braking alone does not mean the envelope was exceeded")

    def test_demo_full_reset_replays_motion_contact_failure_and_completed_stop(self):
        sim = Simulation(Config(**PRESETS["car_demo"]))

        def replay():
            snapshots = []
            for target in (1, 3, 4, 6, 9, 12):
                advance(sim, target)
                snapshots.append((sim.data.qpos.copy(), sim.data.qvel.copy()))
                self.assertFalse(sim.data.qfrc_applied.any())
                self.assertFalse(sim.data.xfrc_applied.any())
            return snapshots, sim.summary()

        first, result = replay()
        self.assertGreater(first[0][0][0], 4, "The approach must visibly move before damage")
        self.assertGreater(abs(first[0][1][sim._spin[0]]), 5, "Physical wheels must rotate")
        self.assertEqual([event["event"] for event in result["events"]],
                         ["barrier_impact", "structural_damage", "recovery_reposition"])
        self.assertGreater(result["public"]["metrics"]["distance_traveled"], 20)
        self.assertLess(result["public"]["metrics"]["final_speed"], .1)
        self.assertEqual(result["public"]["outcome"], "probe_outside_envelope")
        self.assertFalse(any(result["warnings"]))
        self.assertTrue(sim.finished)

        sim.reset_full()
        second, repeated = replay()
        for original, replayed in zip(first, second, strict=True):
            np.testing.assert_array_equal(original[0], replayed[0])
            np.testing.assert_array_equal(original[1], replayed[1])
        self.assertEqual(result, repeated)
        self.assertTrue(sim.finished)

    def test_observations_omit_labels_and_commands_use_only_public_inputs(self):
        actual = Simulation(Config(duration=4))
        nominal = Simulation(Config(fault="healthy", duration=4))
        advance(actual, 3)
        advance(nominal, 3)
        # Equal public speed and trial time produce identical probe commands
        # even though their hidden rack hardware and vehicle poses differ.
        nominal.data.qvel[:2] = actual.data.qvel[:2]
        self.assertEqual(actual._nominal_command(), nominal._nominal_command())
        observation = actual.observe()
        for private in ("fault", "damage_active", "rack_gain", "rack_bias", "events", "config",
                        "toe_quaternion", "spring_stiffness", "tire_friction", "tire_radius"):
            self.assertNotIn(private, observation)
            self.assertNotIn(private, actual.summary()["public"])
        self.assertIn("wheel_axle_directions", observation)
        self.assertIn("suspension_travel", observation)

    def test_manual_probe_reset_does_not_trigger_unobserved_damage(self):
        sim = Simulation(Config(duration=1))
        sim.reset_trial()
        advance(sim, 1)
        self.assertFalse(sim.diagnostics()["damage_active"])
        self.assertIsNone(sim.diagnostics()["impact_time"])
        self.assertFalse(sim.summary()["public"]["safe"])

    def test_trial_reset_restarts_completed_probe_with_monotonic_clock(self):
        sim = Simulation(Config(duration=3))
        advance(sim, 3)
        self.assertTrue(sim.finished)
        self.assertTrue(sim.diagnostics()["damage_active"])
        sim.reset_trial()
        self.assertFalse(sim.finished)
        self.assertEqual(sim.elapsed, 3)
        advance(sim, 6)
        self.assertTrue(sim.finished)
        self.assertEqual(sim.elapsed, 6)
        self.assertTrue(sim.diagnostics()["damage_active"])


if __name__ == "__main__":
    unittest.main()
