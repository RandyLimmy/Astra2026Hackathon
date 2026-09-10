"""Physics and experiment contract regressions; no graphics context required."""

from dataclasses import replace
import math
import unittest

import numpy as np

from simulator.config import Experiment, WHEELS
from simulator.model import build_model
from simulator.private import thermal
from simulator.runner import Simulator


def advance(simulator, seconds, throttle=0.0, brake=0.0):
    for _ in range(round(seconds / simulator.config.timestep)):
        simulator.step(throttle, brake)


class ExperimentTests(unittest.TestCase):
    def test_rejects_nonfinite_parameters(self):
        fields = ("initial_speed", "brake_at", "brake", "throttle", "duration",
                  "wall_x", "timestep", "warmup_cycles", "recovery", "detach_at",
                  "wet_friction", "wet_start", "wet_end", "payload",
                  "brake_efficiency", "weak_at", "lag")
        for name in fields:
            for value in (float("nan"), float("inf"), -float("inf")):
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    Experiment(**{name: value})

    def test_rejects_out_of_range_parameters(self):
        invalid = {
            "initial_speed": (-1, 36), "brake_at": (-1, 201),
            "brake": (-0.1, 1.1), "throttle": (-0.1, 1.1),
            "duration": (0, 121), "wall_x": (9, 251), "timestep": (0, 0.006),
            "warmup_cycles": (-1, 13, 1.5), "recovery": (-1, 601),
            "payload": (-1, 601), "brake_efficiency": (-0.1, 1.1),
            "lag": (-0.1, 2.1), "wet_friction": (0, 1.6),
            "wet_start": (-400, 120), "wet_end": (40, 900),
            "detach_at": (-1,), "weak_at": (-1,),
            "detach_wheel": ("front",), "weak_wheel": ("left",),
        }
        for name, values in invalid.items():
            for value in values:
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    Experiment(**{name: value})

    def test_command_validation_and_json_round_trip(self):
        invalid = (
            ((0, 1),), ((0, 0, 0, 0),), ((-1, 0, 0),),
            ((0, 0, 0), (0, 1, 0)), ((1, 0, 0), (0, 1, 0)),
            ((0, -0.1, 0),), ((0, 0, 1.1),), ((0, float("nan"), 0),),
            ((float("inf"), 0, 0),),
        )
        for commands in invalid:
            with self.subTest(commands=commands), self.assertRaises(ValueError):
                Experiment(commands=commands)
        config = Experiment.from_dict({"commands": [[0.2, 0.5, 0], [0.8, 0, 1]]})
        self.assertEqual(config.commands, ((0.2, 0.5, 0), (0.8, 0, 1)))
        self.assertEqual(Experiment.from_dict(config.to_dict()), config)


class SimulatorTests(unittest.TestCase):
    def test_step_and_trial_reset_reject_invalid_inputs(self):
        sim = Simulator(Experiment(initial_speed=0, wall=False))
        initial = sim.data.qpos.copy()
        for value in (-0.1, 1.1, float("nan"), float("inf")):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    sim.step(value, 0)
                with self.assertRaises(ValueError):
                    sim.step(0, value)
        for speed in (-1, 36, float("nan"), float("inf")):
            with self.subTest(speed=speed), self.assertRaises(ValueError):
                sim.reset_trial(speed)
        np.testing.assert_array_equal(sim.data.qpos, initial)
        self.assertEqual(sim.elapsed, 0)

    def test_command_schedule_uses_trial_clock_and_holds_last_value(self):
        sim = Simulator(Experiment(initial_speed=0, wall=False,
                                   commands=((0.1, 0.5, 0), (0.2, 0, 0.75))))
        self.assertEqual(sim.command(), (0, 0))
        advance(sim, 0.1)
        self.assertEqual(sim.command(), (0.5, 0))
        advance(sim, 0.1)
        self.assertEqual(sim.command(), (0, 0.75))
        advance(sim, 0.1)
        self.assertEqual(sim.command(), (0, 0.75))
        sim.reset_trial(0)
        self.assertEqual(sim.command(), (0, 0))
        self.assertAlmostEqual(sim.data.time, 0.3)

    def test_trial_reset_preserves_faults_and_full_reset_restores_defaults(self):
        config = Experiment(initial_speed=10, thermal=True, payload=300,
                            wet_friction=0.3, detach_wheel="FR", detach_at=0,
                            weak_wheel="FL", brake_efficiency=0.2)
        sim = Simulator(config)
        advance(sim, 0.4, brake=1)
        sim.temperature[:] = [350, 300, 200, 100]
        temperature = sim.temperature.copy()
        efficiency = sim.efficiency.copy()
        detached_joint = sim.model.joint("carrier_FR_free")
        qa = int(detached_joint.qposadr[0])
        relative_position = sim.data.qpos[qa:qa + 3] - sim.data.qpos[:3]
        orientation = sim.data.qpos[qa + 3:qa + 7].copy()
        elapsed = sim.elapsed
        sim.reset_trial(7)
        np.testing.assert_array_equal(sim.temperature, temperature)
        np.testing.assert_array_equal(sim.efficiency, efficiency)
        np.testing.assert_array_equal(sim.data.eq_active[sim.attach], [1, 0, 1, 1])
        np.testing.assert_allclose(sim.data.qpos[qa:qa + 3] - sim.data.qpos[:3],
                                   relative_position, atol=1e-12)
        np.testing.assert_array_equal(sim.data.qpos[qa + 3:qa + 7], orientation)
        self.assertAlmostEqual(sim.data.qvel[0], 7)
        self.assertEqual(sim.elapsed, elapsed)
        self.assertEqual(sim.data.time, elapsed)
        self.assertEqual(sim.trial_time, 0)
        self.assertEqual(sim.activation, 0)
        self.assertEqual(sim.last_command, (0, 0))
        np.testing.assert_array_equal(sim.model.dof_frictionloss[sim.spin], np.zeros(4))
        self.assertAlmostEqual(sim.model.body_mass.sum(), 1500)
        self.assertAlmostEqual(sim.model.geom("road_patch").friction[0], 0.3)

        sim.reset_full()
        self.assertEqual(sim.config, Experiment(timestep=config.timestep))
        np.testing.assert_array_equal(sim.temperature, np.full(4, thermal.AMBIENT))
        np.testing.assert_array_equal(sim.efficiency, np.ones(4))
        np.testing.assert_array_equal(sim.data.eq_active[sim.attach], np.ones(4))
        self.assertAlmostEqual(sim.model.body_mass.sum(), 1200)
        self.assertAlmostEqual(sim.model.geom("road_patch").friction[0], 1.1)
        self.assertGreater(sim.model.geom_contype[sim.wall], 0)
        self.assertGreater(sim.model.geom_conaffinity[sim.wall], 0)
        self.assertEqual(sim.model.geom_rgba[sim.wall, 3], 1)
        self.assertEqual(sim.events, [])
        self.assertEqual(sim.elapsed, 0)
        self.assertFalse(sim.collision)

    def test_brakes_oppose_both_spin_directions(self):
        sim = Simulator(Experiment(initial_speed=10, wall=False))
        for direction in (-1, 1):
            sim.reset_trial(10)
            # Reverse the complete rolling state to avoid introducing tire slip.
            sim.data.qvel[:] *= direction
            before = sim.data.qvel[sim.spin].copy()
            sim.step(0, 1)
            self.assertTrue(np.all(sim.last_brake_torque * before < 0))
            np.testing.assert_array_equal(sim.data.ctrl, np.zeros(4))

    def test_stationary_brake_hold_does_not_heat_or_drive(self):
        sim = Simulator(Experiment(initial_speed=0, thermal=True, wall=False))
        initial = sim.data.xpos[sim.chassis].copy()
        advance(sim, 2, brake=1)
        self.assertLess(float(np.max(sim.temperature - thermal.AMBIENT)), 0.25)
        self.assertLess(float(np.linalg.norm(sim.data.xpos[sim.chassis] - initial)), 0.01)
        self.assertLess(sim.speed, 0.02)
        self.assertLess(float(np.max(np.abs(sim.data.qvel[sim.spin]))), 0.1)

    def test_wheel_rotates_attached_then_physically_separates(self):
        sim = Simulator(Experiment(initial_speed=10, wall=False,
                                   detach_wheel="FR", detach_at=0.1))
        angle_adr = int(sim.model.joint("spin_FR").qposadr[0])
        initial_angle = sim.data.qpos[angle_adr]
        advance(sim, 0.1, brake=1)
        self.assertTrue(sim.data.eq_active[sim.attach[1]])
        self.assertGreater(abs(sim.data.qpos[angle_adr] - initial_angle), 0.5)
        self.assertLess(max(sim.diagnostics()["carrier_separation"]), 0.01)
        sim.step(0, 1)
        self.assertFalse(sim.data.eq_active[sim.attach[1]])
        self.assertEqual(sim.data.ctrl[1], 0)
        self.assertEqual(sim.events[0]["event"], "wheel_release")
        self.assertAlmostEqual(sim.events[0]["trial_time"], 0.1)
        advance(sim, 2, brake=1)
        self.assertGreater(sim.diagnostics()["carrier_separation"][1], 0.5)
        np.testing.assert_array_equal(sim.data.eq_active[sim.attach], [1, 0, 1, 1])
        self.assertEqual(len(sim.events), 1)
        self.assertFalse(sim.data.warning.number.any())

    def test_selected_wheel_loss_case_converges_when_timestep_is_halved(self):
        distances = []
        for timestep in (0.002, 0.001):
            sim = Simulator(Experiment(initial_speed=17, wall=False, detach_wheel="FR",
                                       detach_at=3.1, timestep=timestep, duration=12))
            separation_at_two_seconds = []

            def sample(simulator, phase):
                if phase == "trial" and abs(simulator.trial_time - 5.1) < timestep / 2:
                    separation_at_two_seconds.append(
                        simulator.diagnostics()["carrier_separation"][1])

            result = sim.run(sample)
            self.assertTrue(result["stopped"])
            self.assertFalse(any(result["warnings"]))
            self.assertEqual(len(separation_at_two_seconds), 1)
            self.assertGreater(separation_at_two_seconds[0], 0.5)
            self.assertFalse(sim.data.eq_active[sim.attach[1]])
            distances.append(result["stopping_distance"])
        self.assertAlmostEqual(distances[0], distances[1], delta=0.01 * distances[1])

    def test_wet_road_changes_effective_tire_contact_friction(self):
        for wet_start, wet_end, expected in ((-20, 20, 0.3), (40, 120, 1.1)):
            sim = Simulator(Experiment(initial_speed=0, wall=False, wet_friction=0.3,
                                       wet_start=wet_start, wet_end=wet_end))
            tire_ids = {sim.model.geom(f"tire_{wheel}").id for wheel in WHEELS}
            contacts = [contact for contact in sim.data.contact
                        if any(int(geom) in tire_ids for geom in contact.geom)]
            self.assertGreaterEqual(len(contacts), 4)
            for contact in contacts:
                self.assertAlmostEqual(contact.friction[0], expected)

    def test_payload_has_consistent_mass_and_positive_inertia(self):
        baseline = build_model(Experiment(wall=False))
        for payload in (150, 300, 450):
            with self.subTest(payload=payload):
                loaded = build_model(Experiment(wall=False, payload=payload))
                self.assertAlmostEqual(loaded.body_mass.sum() - baseline.body_mass.sum(), payload)
                self.assertAlmostEqual(loaded.body("payload").mass[0], payload)
                self.assertTrue(np.all(loaded.body("payload").inertia > 0))
                self.assertAlmostEqual(loaded.body_subtreemass[loaded.body("chassis").id],
                                       1100 + payload)

    def test_weak_brake_changes_one_wheel_and_never_releases_it(self):
        sim = Simulator(Experiment(initial_speed=10, wall=False, weak_wheel="FL",
                                   brake_efficiency=0.2, weak_at=0.1))
        advance(sim, 0.1)
        np.testing.assert_array_equal(sim.efficiency, np.ones(4))
        sim.step(0, 1)
        self.assertAlmostEqual(abs(sim.last_brake_torque[0] / sim.last_brake_torque[1]),
                               0.2, places=6)
        np.testing.assert_array_equal(sim.data.eq_active[sim.attach], np.ones(4))
        self.assertEqual(sim.events[0]["event"], "brake_efficiency")
        self.assertAlmostEqual(sim.events[0]["trial_time"], 0.1)

    def test_actuator_lag_reaches_first_order_response_and_releases(self):
        sim = Simulator(Experiment(initial_speed=10, wall=False, lag=0.1))
        advance(sim, 0.1, brake=1)
        self.assertAlmostEqual(sim.activation, 1 - math.exp(-1), places=12)
        advance(sim, 0.2, brake=1)
        self.assertAlmostEqual(sim.activation, 1 - math.exp(-3), places=12)
        advance(sim, 0.2, brake=1)
        self.assertGreater(sim.activation, 0.99)
        before = sim.activation
        advance(sim, 0.1)
        self.assertAlmostEqual(sim.activation, before * math.exp(-1), places=12)

    def test_neutral_fault_parameters_reproduce_baseline_trajectory(self):
        config = Experiment(initial_speed=10, wall=False, brake_at=0, duration=5)
        baseline = Simulator(config)
        reference = baseline.run()
        neutral = replace(config, thermal=True, weak_wheel="FL", brake_efficiency=1,
                          detach_wheel=None, detach_at=0, wet_friction=1.1,
                          wet_start=-20, wet_end=20, payload=0, lag=0)
        sim = Simulator(neutral)
        result = sim.run()
        self.assertTrue(result["stopped"])
        self.assertFalse(result["collision"])
        self.assertAlmostEqual(result["stopping_distance"], reference["stopping_distance"],
                               delta=0.01 * reference["stopping_distance"])
        self.assertAlmostEqual(sim.yaw, baseline.yaw, delta=math.radians(0.5))

    def test_baseline_stops_and_public_observations_exclude_diagnostics(self):
        sim = Simulator(Experiment(initial_speed=10, wall=False, brake_at=0, duration=5))
        result = sim.run()
        self.assertTrue(result["stopped"])
        self.assertFalse(result["censored"])
        self.assertFalse(result["collision"])
        self.assertGreater(result["stopping_distance"], 0)
        self.assertLess(result["stopping_distance"], 20)
        self.assertLess(result["max_lateral_displacement"], 0.25)
        self.assertFalse(any(result["warnings"]))
        for private in ("temperature", "efficiency", "attached", "config", "events",
                        "brake_activation", "contact_friction"):
            self.assertNotIn(private, sim.observe())
            self.assertNotIn(private, result["public"])


class ThermalTests(unittest.TestCase):
    def test_fade_is_bounded_and_monotonic(self):
        factors = thermal.fade(np.array([20, 180, 240, 300, 420, 1000]))
        self.assertTrue(np.all(np.diff(factors) <= 0))
        self.assertAlmostEqual(factors[0], 1)
        self.assertAlmostEqual(factors[-1], 0.45)
        self.assertTrue(np.all((factors >= 0.45 - 1e-12) & (factors <= 1)))

    def test_rest_cools_and_positive_dissipation_heats(self):
        ambient = np.full(4, thermal.AMBIENT)
        np.testing.assert_array_equal(thermal.advance(ambient, np.zeros(4), 30), ambient)
        hot = np.full(4, 400.0)
        short_rest = thermal.advance(hot, np.zeros(4), 30)
        long_rest = thermal.advance(hot, np.zeros(4), 180)
        self.assertTrue(np.all(ambient < long_rest))
        self.assertTrue(np.all(long_rest < short_rest))
        self.assertTrue(np.all(short_rest < hot))
        heated = thermal.advance(ambient, np.full(4, 1000.0), 1)
        self.assertTrue(np.all(heated > ambient))
        np.testing.assert_allclose(thermal.advance(short_rest, np.zeros(4), 150), long_rest)


if __name__ == "__main__":
    unittest.main()
