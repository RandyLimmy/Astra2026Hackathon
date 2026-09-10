"""Physical quadrotor experiments and their public/private observation boundary."""

from dataclasses import replace
import unittest

import mujoco
import numpy as np

from simulator.platforms.drone import (
    Config, DESCRIPTIONS, FAULTS, MAX_ROTOR_THRUST, NOMINAL_MASS, PRESETS, Simulation,
)


def advance(sim, seconds, control=None):
    for _ in range(round(seconds / sim.config.timestep)):
        sim.step(control)


class DroneConfigTests(unittest.TestCase):
    def test_incomplete_probe_and_reset_cannot_report_safety(self):
        sim = Simulation(Config(duration=0.1))
        self.assertFalse(sim.summary()["public"]["safe"])
        self.assertEqual(sim.summary()["public"]["outcome"], "incomplete_probe")
        advance(sim, 0.1)
        self.assertTrue(sim.summary()["public"]["safe"])
        sim.reset_trial()
        self.assertFalse(sim.summary()["public"]["safe"])
        self.assertEqual(sim.summary()["public"]["outcome"], "incomplete_probe")

    def test_rejects_nonfinite_and_out_of_range_config(self):
        invalid = {"duration": [0, 121], "timestep": [0, 0.006], "fault_at": [-1, 121],
                   "rotor_effectiveness": [-0.1, 1.1], "voltage_ratio": [0, 1.1],
                   "payload_mass": [-1, 6], "wind_force": [-1, 21], "control_delay": [-0.1, 1.1]}
        for field, values in invalid.items():
            for value in [*values, float("nan"), float("inf"), "1", True]:
                with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                    Config(**{field: value})
        for field, values in {"fault": ["broken", None], "probe": ["unknown"],
                              "rotor_index": [-1, 4, 1.2, True, float("nan")]}.items():
            for value in values:
                with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                    Config(**{field: value})
        self.assertEqual(set(PRESETS), set(DESCRIPTIONS))
        self.assertEqual({Config(**values).fault for values in PRESETS.values()}, set(FAULTS))

    def test_invalid_control_has_no_physical_side_effect(self):
        sim = Simulation(Config(fault="rotor_loss", fault_at=0))
        original = sim.data.qpos.copy()
        invalid = [{}, {"throttle": 1}, {"rotor_commands": [0, 1, 0]},
                   {"rotor_commands": [0, 0, 0, 1.1]}, {"rotor_commands": [0, 0, 0, float("nan")]},
                   {"rotor_commands": ["bad"] * 4}, {"target_position": [0, 0]},
                   {"target_position": [11, 0, 2]}, {"target_position": [0, 0, 0]},
                   {"target_position": [0, 0, float("inf")]},
                   {"rotor_commands": [0] * 4, "target_position": [0, 0, 2]}]
        for control in invalid:
            with self.subTest(control=control), self.assertRaises(ValueError):
                sim.step(control)
        self.assertEqual(sim.elapsed, 0)
        self.assertEqual(sim.events, [])
        np.testing.assert_array_equal(sim.data.qpos, original)
        np.testing.assert_array_equal(sim.effectiveness, np.ones(4))


class DronePhysicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.runs = {}
        cls.prefault = {}
        for name, values in PRESETS.items():
            # These regression assertions freeze the original 12 s / 3 s-onset
            # failure matrix; presentation routes have separate coverage.
            if name == "drone_demo":
                continue
            sim = Simulation(Config(**values))
            prefix = []
            while not sim.finished:
                sim.step()
                if sim.elapsed <= sim.config.fault_at + 1e-10:
                    prefix.append(sim.data.qpos.copy())
            cls.runs[name] = sim
            cls.prefault[name] = np.array(prefix)

    def test_nominal_hover_and_translation_are_stable(self):
        sim = self.runs["drone_hover"]
        public = sim.summary()["public"]
        metrics = public["metrics"]
        self.assertTrue(public["safe"])
        self.assertGreater(metrics["min_altitude"], 1.99)
        self.assertLess(metrics["max_altitude"], 2.01)
        self.assertLess(metrics["max_tilt_degrees"], 3)
        self.assertLess(metrics["max_tracking_error"], 0.6)
        self.assertGreater(metrics["max_horizontal_excursion"], 1)
        self.assertFalse(metrics["ground_contact"])
        self.assertEqual(sim.events, [])

    def test_each_fault_has_identical_healthy_interval_then_observable_mismatch(self):
        baseline = self.runs["drone_hover"]
        for name, sim in self.runs.items():
            with self.subTest(name=name):
                np.testing.assert_array_equal(self.prefault[name], self.prefault["drone_hover"])
                self.assertFalse(sim.data.warning.number.any())
                self.assertTrue(np.isfinite(sim.data.qpos).all())
                self.assertAlmostEqual(sim.elapsed, sim.config.duration, places=8)
                if name == "drone_hover":
                    continue
                self.assertEqual(len(sim.events), 1)
                self.assertAlmostEqual(sim.events[0]["time"], 3.0, places=8)
                self.assertFalse(sim.summary()["public"]["safe"])
                self.assertGreater(np.linalg.norm(sim.data.qpos[:3] - baseline.data.qpos[:3]), 0.7)

    def test_severe_faults_cause_real_ground_contact(self):
        for name in ("drone_rotor_loss", "drone_voltage_sag", "drone_payload", "drone_delay"):
            with self.subTest(name=name):
                sim = self.runs[name]
                metrics = sim.summary()["public"]["metrics"]
                self.assertTrue(metrics["ground_contact"])
                self.assertLess(metrics["final_altitude"], 0.2)
                self.assertGreater(metrics["contact_steps"], 1)
        self.assertGreater(self.runs["drone_rotor_loss"].max_tilt, 90)
        self.assertFalse(self.runs["drone_wind"].ground_contact)

    def test_thrust_applies_at_rotor_sites_and_induces_rotation(self):
        sim = Simulation()
        for index in range(4):
            actuator = sim.model.actuator(f"rotor_motor_{index}")
            self.assertEqual(int(actuator.trntype[0]), mujoco.mjtTrn.mjTRN_SITE)
            self.assertEqual(int(actuator.trnid[0]), sim.model.site(f"rotor_site_{index}").id)
        advance(sim, 0.08, {"rotor_commands": [1, 0, 0, 0]})
        self.assertGreater(sim.data.qvel[3], 0.3)
        self.assertLess(sim.data.qvel[4], -0.3)
        self.assertGreater(sim.data.qvel[5], 0.01)
        self.assertLess(sim.data.qpos[2], 2)
        self.assertTrue(np.all(sim.data.ctrl >= 0))
        self.assertTrue(np.all(sim.data.ctrl <= MAX_ROTOR_THRUST))

    def test_open_loop_motor_shutdown_falls_without_root_correction(self):
        sim = Simulation()
        advance(sim, 0.3, {"rotor_commands": [0, 0, 0, 0]})
        self.assertLess(sim.data.qvel[2], -2)
        self.assertLess(sim.data.qpos[2], 1.7)
        self.assertLess(float(sim.data.ctrl.max()), 0.001)
        np.testing.assert_array_equal(sim.data.qfrc_applied, np.zeros(sim.model.nv))

    def test_rotor_and_voltage_faults_have_distinct_physical_effects(self):
        control = {"rotor_commands": [0.5] * 4}
        rotor = Simulation(Config(fault="rotor_loss", fault_at=0, rotor_index=2))
        sag = Simulation(Config(fault="voltage_sag", fault_at=0))
        rotor.step(control)
        sag.step(control)
        self.assertAlmostEqual(rotor.data.ctrl[2] / rotor.data.ctrl[0], rotor.config.rotor_effectiveness)
        self.assertAlmostEqual(sag.data.ctrl[0] / rotor.data.ctrl[0], sag.config.voltage_ratio ** 2)
        self.assertGreater(abs(rotor.data.qvel[3]), 0)
        self.assertLess(abs(sag.data.qvel[3]), 1e-10)

    def test_payload_updates_mass_com_and_inertia_without_state_teleport(self):
        sim = Simulation(Config(fault="payload", fault_at=0.5))
        advance(sim, 0.5, {"target_position": [1, -0.5, 2.5]})
        qpos, qvel = sim.data.qpos.copy(), sim.data.qvel.copy()
        sim._apply_event()
        np.testing.assert_array_equal(sim.data.qpos, qpos)
        np.testing.assert_array_equal(sim.data.qvel, qvel)
        self.assertAlmostEqual(sim.model.body_mass[sim.focus_body], NOMINAL_MASS + sim.config.payload_mass)
        self.assertAlmostEqual(sim.model.body_subtreemass[sim.focus_body], NOMINAL_MASS + sim.config.payload_mass)
        self.assertTrue(np.all(sim.model.body_inertia[sim.focus_body] > 0))
        self.assertLess(sim.model.body_ipos[sim.focus_body, 2], 0)
        sim.step()
        self.assertLess(sim.data.qvel[2], qvel[2])

    def test_explicit_target_and_controller_do_not_read_fault_parameters(self):
        baseline = Simulation()
        damaged = Simulation(Config(fault="rotor_loss", fault_at=0))
        damaged._apply_event()
        target = np.array([0.5, -0.25, 2.5])
        np.testing.assert_array_equal(baseline._nominal_control(target), damaged._nominal_control(target))
        for _ in range(1000):
            baseline.step({"target_position": target.tolist()})
        self.assertGreater(baseline.data.qpos[0], 0.2)
        self.assertLess(baseline.data.qpos[1], -0.1)
        self.assertGreater(baseline.data.qpos[2], 2.3)
        self.assertEqual(baseline.observe()["command"]["mode"], "target_position")

    def test_transport_delay_uses_actual_command_history(self):
        sim = Simulation(Config(fault="delay", fault_at=0.1, control_delay=0.04))
        advance(sim, 0.08, {"rotor_commands": [0.3] * 4})
        advance(sim, 0.02, {"rotor_commands": [0.7] * 4})
        sim.step({"rotor_commands": [0.9] * 4})
        np.testing.assert_array_equal(sim.delayed_command, np.full(4, 0.3))
        advance(sim, 0.04, {"rotor_commands": [0.9] * 4})
        np.testing.assert_array_equal(sim.delayed_command, np.full(4, 0.9))

    def test_trial_reset_preserves_every_damage_type_and_monotonic_clock(self):
        for fault in FAULTS[1:]:
            with self.subTest(fault=fault):
                sim = Simulation(Config(fault=fault, fault_at=0.1, duration=0.2))
                advance(sim, 0.2)
                before = sim.diagnostics()
                elapsed = sim.elapsed
                self.assertTrue(sim.finished)
                sim.reset_trial()
                self.assertFalse(sim.finished)
                self.assertEqual(sim.elapsed, elapsed)
                self.assertEqual(sim.data.time, elapsed)
                self.assertEqual(sim.trial_time, 0)
                for key in ("rotor_effectiveness", "voltage_ratio", "wind_force", "control_delay",
                            "mass", "inertia", "center_of_mass", "events"):
                    self.assertEqual(sim.diagnostics()[key], before[key])
                sim.step()
                self.assertEqual(len(sim.events), 1)
                self.assertGreater(sim.elapsed, elapsed)
                sim.reset_full()
                self.assertEqual(sim.elapsed, 0)
                self.assertEqual(sim.config.fault, fault)
                self.assertEqual(sim.events, [])
                self.assertEqual(sim.model.body_mass[sim.focus_body], NOMINAL_MASS)
                np.testing.assert_array_equal(sim.effectiveness, np.ones(4))
                self.assertEqual(sim.voltage, 1)
                self.assertEqual(sim.delay, 0)

    def test_deterministic_replay_and_full_reset(self):
        config = Config(fault="payload", fault_at=0.1, duration=0.5)
        first, second = Simulation(config), Simulation(config)
        for sim in (first, second):
            advance(sim, 0.5)
        np.testing.assert_array_equal(first.data.qpos, second.data.qpos)
        self.assertEqual(first.summary(), second.summary())
        first.reset_full()
        advance(first, 0.5)
        np.testing.assert_array_equal(first.data.qpos, second.data.qpos)
        self.assertEqual(first.summary(), second.summary())

    def test_neutral_fault_parameters_match_nominal(self):
        base = Config(duration=0.4, fault_at=0.1)
        baseline = Simulation(base)
        advance(baseline, base.duration)
        params = {"rotor_loss": {"rotor_effectiveness": 1}, "voltage_sag": {"voltage_ratio": 1},
                  "payload": {"payload_mass": 0}, "wind": {"wind_force": 0}, "delay": {"control_delay": 0}}
        for fault, overrides in params.items():
            with self.subTest(fault=fault):
                sim = Simulation(replace(base, fault=fault, **overrides))
                advance(sim, base.duration)
                np.testing.assert_allclose(sim.data.qpos, baseline.data.qpos, atol=1e-12, rtol=0)

    def test_public_telemetry_has_sensors_but_no_fault_labels_or_private_truth(self):
        sim = self.runs["drone_rotor_loss"]
        observation = sim.observe()
        public = sim.summary()["public"]
        for key in ("config", "fault", "events", "rotor_effectiveness", "voltage_ratio",
                    "wind_force", "control_delay", "mass", "inertia", "rotor_thrust"):
            self.assertNotIn(key, observation)
            self.assertNotIn(key, public)
        self.assertEqual(len(observation["specific_force"]), 3)
        self.assertEqual(len(observation["angular_velocity"]), 3)
        self.assertEqual(len(observation["command"]["rotor_commands"]), 4)
        self.assertIn(observation["phase"], ("hover", "translation_probe"))
        for camera in ("overview", "side", "chase"):
            self.assertGreaterEqual(sim.model.camera(camera).id, 0)


if __name__ == "__main__":
    unittest.main()
