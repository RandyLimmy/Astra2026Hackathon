"""Warehouse physics and intervention contract, with no renderer required."""
from dataclasses import replace
import json
import math
import unittest

import mujoco
import numpy as np

from simulator.platforms.warehouse import (
    Config, DESCRIPTIONS, FAULTS, PRESETS, PROBES, Simulation, model_xml, probe_command,
)


def advance(sim: Simulation, seconds: float | None = None) -> Simulation:
    count = math.ceil((sim.config.duration if seconds is None else seconds) / sim.config.timestep)
    for _ in range(count):
        if sim.finished:
            break
        sim.step()
    return sim


class ConfigTests(unittest.TestCase):
    def test_invalid_values_are_rejected_before_compilation(self):
        numeric = ("duration", "timestep", "fault_at", "payload_mass", "added_mass", "shift_distance",
                   "traction_scale", "resistance", "motor_scale", "fault_ramp", "drive_scale",
                   "latch_strength", "latch_dwell", "latch_arm_at")
        for name in numeric:
            for value in (float("nan"), float("inf"), -float("inf"), True, "1"):
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    Config(**{name: value})
        bad = {"duration": (0, 61), "timestep": (0, .006), "fault_at": (-1, 121),
               "payload_mass": (0, 31), "added_mass": (-1, 41), "shift_distance": (-1, .31),
               "traction_scale": (0, 1.1), "resistance": (-1, 6), "motor_scale": (-1, 1.1),
               "fault_ramp": (-1, 11), "drive_scale": (-1, 1.6), "latch_strength": (0, 10001),
               "latch_dwell": (-1, 11), "latch_arm_at": (-1, 121),
               "fault": ("wind",), "probe": ("unknown",)}
        for name, values in bad.items():
            for value in values:
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    Config(**{name: value})

    def test_invalid_controls_leave_state_unchanged(self):
        sim = Simulation(Config())
        initial = sim.data.qpos.copy()
        controls = [{"left": x} for x in (-1.1, 1.1, float("nan"), float("inf"), True, "1")]
        controls += [{"right": -1.1}, {"throttle": .5}, [1, 1]]
        for control in controls:
            with self.subTest(control=control), self.assertRaises(ValueError):
                sim.step(control)
        np.testing.assert_array_equal(sim.data.qpos, initial)
        self.assertEqual(sim.elapsed, 0)


class PhysicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Original probes share the nominal eight-kilogram load. Presentation
        # profiles have their own matched-load checks in test_showcase_profiles.
        cls.runs = {name: advance(Simulation(Config(**preset))) for name, preset in PRESETS.items()
                    if not name.endswith("_demo")}
        cls.healthy = {probe: advance(Simulation(Config(probe=probe))) for probe in PROBES}

    def test_presets_cameras_and_states(self):
        self.assertEqual(set(PRESETS), set(DESCRIPTIONS))
        for name, sim in self.runs.items():
            with self.subTest(name=name):
                self.assertTrue(sim.finished)
                self.assertTrue(np.isfinite(sim.data.qpos).all())
                self.assertTrue(np.isfinite(sim.data.qvel).all())
                self.assertFalse(sim.data.warning.number.any())
                self.assertTrue(sim.summary()["public"]["safe"])
                self.assertLess(sim.summary()["public"]["metrics"]["max_tilt_rad"], .1)
                for camera in ("overview", "side", "chase"):
                    self.assertGreaterEqual(sim.model.camera(camera).id, 0)
                self.assertEqual(float(np.abs(sim.data.xfrc_applied).sum()), 0)
        self.assertGreater(self.healthy["straight"].observe()["position"][0], 5)
        for sim in self.healthy.values():
            self.assertTrue(sim.summary()["public"]["safe"])
            self.assertGreater(sim.summary()["public"]["metrics"]["path_length_m"], .5)

    def test_faults_change_real_motion_under_matched_probes(self):
        for name, sim in self.runs.items():
            if sim.config.fault == "healthy":
                continue
            nominal = self.healthy[sim.config.probe]
            with self.subTest(name=name):
                displacement = np.linalg.norm(np.array(sim.observe()["position"]) -
                                              nominal.observe()["position"])
                # The opposed turns partially cancel the cargo-shift endpoint
                # error; its observable tilt is the stronger retained metric.
                self.assertGreater(displacement, .01 if sim.config.fault == "payload_shift" else .1)
                if sim.config.fault == "payload_shift":
                    self.assertGreater(sim.summary()["public"]["metrics"]["max_tilt_rad"],
                                       3 * nominal.summary()["public"]["metrics"]["max_tilt_rad"])
                if sim.config.fault in ("payload_mass", "traction", "rolling_resistance", "battery"):
                    self.assertLess(sim.observe()["position"][0], nominal.observe()["position"][0])
        heavy = self.runs["warehouse_payload"]
        self.assertAlmostEqual(heavy.model.body_mass.sum() - self.healthy["straight"].model.body_mass.sum(),
                               heavy.config.added_mass)
        shift = self.runs["warehouse_shift"]
        self.assertFalse(shift.diagnostics()["cargo_latched"])
        self.assertGreater(shift.diagnostics()["cargo_displacement"], .15)
        self.assertGreater(max(self.runs["warehouse_caster"].diagnostics()["caster_resistance"]), 1)

    def test_all_timed_faults_match_before_onset(self):
        nominal = advance(Simulation(Config(duration=2)), 1.5)
        for fault in FAULTS:
            if fault in ("payload_mass", "cargo_breakaway"):
                continue  # Loading and force-triggered failure have separate onset contracts.
            with self.subTest(fault=fault):
                actual = advance(Simulation(Config(fault=fault, duration=2)), 1.5)
                np.testing.assert_array_equal(actual.data.qpos, nominal.data.qpos)
                np.testing.assert_array_equal(actual.data.qvel, nominal.data.qvel)
                self.assertEqual(actual.observe(), nominal.observe())
                self.assertEqual(actual.events, [])

    def test_neutral_fault_parameters_match_healthy(self):
        neutral = {"payload_mass": {"added_mass": 0}, "payload_shift": {"shift_distance": 0},
                   "traction": {"traction_scale": 1}, "rolling_resistance": {"resistance": 0},
                   "caster_jam": {"resistance": 0}, "battery": {"motor_scale": 1}}
        nominal = advance(Simulation(Config(duration=3)))
        for fault, values in neutral.items():
            with self.subTest(fault=fault):
                actual = advance(Simulation(Config(fault=fault, duration=3, **values)))
                np.testing.assert_array_equal(actual.data.qpos, nominal.data.qpos)
                self.assertEqual(actual.observe(), nominal.observe())

    def test_payload_release_has_no_position_or_mass_teleport(self):
        sim = advance(Simulation(Config(fault="payload_shift", duration=3)), 1.5)
        masses, position = sim.model.body_mass.copy(), sim.data.qpos.copy()
        self.assertTrue(sim.diagnostics()["cargo_latched"])
        sim.step()
        self.assertFalse(sim.diagnostics()["cargo_latched"])
        self.assertLess(abs(sim.diagnostics()["cargo_displacement"]), .001)
        self.assertLess(float(np.linalg.norm(sim.data.qpos[:3] - position[:3])), .005)
        np.testing.assert_array_equal(sim.model.body_mass, masses)
        advance(sim, 1)
        self.assertGreater(sim.diagnostics()["cargo_displacement"], .15)
        np.testing.assert_array_equal(sim.model.body_mass, masses)

    def test_probes_commands_and_observable_slip(self):
        # Identical time-only commands for all faults; explicit intervention overrides them.
        for probe in PROBES:
            nominal = Simulation(Config(probe=probe, duration=2))
            actual = Simulation(Config(probe=probe, fault="battery", fault_at=0, duration=2))
            for _ in range(1000):
                expected = probe_command(probe, nominal.trial_time)
                nominal.step()
                actual.step()
                self.assertEqual(nominal.last_command, expected)
                self.assertEqual(actual.last_command, expected)
        sim = Simulation(Config(duration=3, fault="traction", fault_at=.5))
        peak_slip = 0.
        for _ in range(1500):
            sim.step({"left": .7, "right": .7})
            observation = sim.observe()
            peak_slip = max(peak_slip, abs(observation["encoder_velocity"]["forward"] -
                                           observation["linear_velocity"][0]))
        self.assertGreater(peak_slip, .3)
        sim.reset_trial()
        sim.step({"left": -.5})
        self.assertEqual(sim.last_command, {"left": -.5, "right": 0.})
        self.assertEqual(sim.data.ctrl.tolist(), [-.5, 0.])

    def test_deterministic_replay_and_half_timestep_sensitivity(self):
        for name, first in self.runs.items():
            with self.subTest(name=name):
                repeat = advance(Simulation(first.config))
                np.testing.assert_allclose(repeat.data.qpos, first.data.qpos, atol=1e-10, rtol=0)
                finer = advance(Simulation(replace(first.config, timestep=.001)))
                self.assertFalse(finer.data.warning.number.any())
                np.testing.assert_allclose(finer.observe()["position"], first.observe()["position"],
                                           atol=.12, rtol=0)
                a = first.summary()["public"]["metrics"]
                b = finer.summary()["public"]["metrics"]
                self.assertLess(abs(a["path_length_m"] - b["path_length_m"]), .12)
                if first.config.fault in ("payload_mass", "traction", "rolling_resistance", "battery"):
                    self.assertLess(finer.observe()["position"][0],
                                    self.healthy[first.config.probe].observe()["position"][0])

    def test_resets_retain_damage_and_full_reset_replays_original(self):
        for name, preset in PRESETS.items():
            if name.endswith("_demo"):
                continue  # The legacy three-second trial must have already reached fault onset.
            with self.subTest(name=name):
                cfg = Config(**{**preset, "duration": 3})
                sim = advance(Simulation(cfg))
                before = sim.diagnostics()
                elapsed, model = sim.elapsed, sim.model
                sim.reset_trial()
                after = sim.diagnostics()
                self.assertEqual(sim.elapsed, elapsed)
                self.assertEqual(sim.trial_time, 0)
                self.assertFalse(sim.finished)
                self.assertEqual(sim.last_command, {"left": 0., "right": 0.})
                for field in ("fault_applied", "total_mass_kg", "cargo_latched", "drive_friction",
                              "bearing_resistance", "caster_resistance", "motor_scale", "events"):
                    self.assertEqual(before[field], after[field])
                if cfg.fault == "payload_shift":
                    self.assertEqual(before["cargo_displacement"], after["cargo_displacement"])
                sim.step()
                self.assertGreater(sim.elapsed, elapsed)
                sim.reset_full()
                fresh = Simulation(cfg)
                self.assertIs(sim.model, model)
                self.assertEqual(sim.config, cfg)
                self.assertEqual(sim.elapsed, 0)
                self.assertEqual(sim.observe(), fresh.observe())
                self.assertEqual(sim.diagnostics(), fresh.diagnostics())
                advance(sim, .1)
                advance(fresh, .1)
                np.testing.assert_array_equal(sim.data.qpos, fresh.data.qpos)

    def test_public_records_hide_true_parameters_and_fault_labels(self):
        forbidden = ("fault", "config", "events", "mass", "resistance", "friction", "motor_scale",
                     "cargo_displacement", "center_of_mass", "warehouse.xml", str(__file__))
        for name, sim in self.runs.items():
            with self.subTest(name=name):
                encoded = json.dumps({"observation": sim.observe(), "summary": sim.summary()["public"]},
                                     allow_nan=False)
                for text in forbidden:
                    self.assertNotIn(text, encoded)
                self.assertIn("config", sim.diagnostics())

    def test_nominal_export_uses_the_healthy_physics(self):
        for preset in PRESETS.values():
            actual = Config(**preset)
            nominal = replace(actual, fault="healthy")
            exported = mujoco.MjModel.from_xml_string(model_xml(nominal))
            physical = Simulation(nominal).model
            np.testing.assert_array_equal(exported.body_mass, physical.body_mass)
            np.testing.assert_array_equal(exported.geom_friction, physical.geom_friction)
            np.testing.assert_array_equal(exported.actuator_gainprm, physical.actuator_gainprm)


if __name__ == "__main__":
    unittest.main()
