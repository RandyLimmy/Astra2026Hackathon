"""Force-triggered cargo failures stay physical, repeatable, and private."""
from dataclasses import replace
import json
import unittest

import mujoco
import numpy as np

from simulator.platforms.warehouse import (
    CARGO_PROBES, Config, PRESETS, Simulation, equality_force, probe_command,
)


def run(config: Config) -> Simulation:
    sim = Simulation(config)
    while not sim.finished:
        sim.step()
    return sim


class CargoFailureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.configs = {name: Config(**preset) for name, preset in PRESETS.items()
                       if name.startswith("warehouse_cargo_")}
        cls.runs = {name: run(config) for name, config in cls.configs.items()}
        cls.nominal = {name: run(replace(config, fault="healthy"))
                       for name, config in cls.configs.items()}

    def test_matched_models_travel_before_force_release_without_teleport(self):
        cfg = self.configs["warehouse_cargo_demo"]
        actual, nominal = Simulation(cfg), Simulation(replace(cfg, fault="healthy"))
        masses = actual.model.body_mass.copy()
        while True:
            np.testing.assert_array_equal(actual.data.qpos, nominal.data.qpos)
            np.testing.assert_array_equal(actual.data.qvel, nominal.data.qvel)
            self.assertEqual(actual.observe(), nominal.observe())
            position = actual.data.qpos.copy()
            velocity = actual.data.qvel.copy()
            actual._apply_fault()
            if actual.events:
                # The discrete transition changes a constraint, never state.
                np.testing.assert_array_equal(actual.data.qpos, position)
                np.testing.assert_array_equal(actual.data.qvel, velocity)
                np.testing.assert_array_equal(actual.model.body_mass, masses)
                self.assertGreater(np.linalg.norm(position[:2]), .5)
                self.assertGreater(actual.events[0]["time"], 3.)
                self.assertGreater(actual.events[0]["constraint_force_N"], cfg.latch_strength)
                self.assertFalse(actual.data.eq_active[actual._latch])
                break
            # The private check above must not count the same sample twice.
            actual._latch_overload = max(0., actual._latch_overload - cfg.timestep)
            actual.step()
            nominal.step()
        actual.step()
        self.assertLess(abs(actual.data.qpos[actual._cargo_qpos] - position[actual._cargo_qpos]), .001)
        self.assertLess(np.linalg.norm(actual.data.qpos[:3] - position[:3]), .01)
        np.testing.assert_array_equal(actual.model.body_mass, masses)

    def test_variants_have_visible_displacement_and_one_invariant_failure_law(self):
        law = set()
        for name, actual in self.runs.items():
            with self.subTest(name=name):
                cfg = actual.config
                law.add((cfg.latch_strength, cfg.latch_dwell, cfg.latch_arm_at))
                nominal = self.nominal[name]
                if cfg.probe == "cargo_gentle":
                    self.assertEqual(actual.events, [])
                    self.assertTrue(actual.diagnostics()["cargo_latched"])
                    np.testing.assert_array_equal(actual.data.qpos, nominal.data.qpos)
                    continue
                self.assertEqual(len(actual.events), 1)
                self.assertEqual(nominal.events, [])
                self.assertFalse(actual.diagnostics()["cargo_latched"])
                self.assertTrue(nominal.diagnostics()["cargo_latched"])
                self.assertGreater(actual.diagnostics()["max_cargo_displacement"], .2)
                self.assertGreater(np.linalg.norm(actual.data.qpos[:2] - nominal.data.qpos[:2]), .5)
                np.testing.assert_array_equal(actual.model.body_mass, nominal.model.body_mass)
                self.assertTrue(actual.summary()["public"]["safe"])
                self.assertTrue(np.isfinite(actual.data.qvel).all())
                self.assertFalse(actual.data.warning.number.any())
                self.assertEqual(float(np.abs(actual.data.xfrc_applied).sum()), 0.)
        self.assertEqual(law, {(5., .01, .5)})
        self.assertLess(self.runs["warehouse_cargo_strong_demo"].events[0]["time"],
                        self.runs["warehouse_cargo_demo"].events[0]["time"])
        first, mirror = self.runs["warehouse_cargo_demo"], self.runs["warehouse_cargo_mirror_demo"]
        np.testing.assert_allclose(first.data.qpos[:3], mirror.data.qpos[:3] * [1., -1., 1.], atol=1e-9)
        self.assertAlmostEqual(first.diagnostics()["cargo_displacement"],
                               -mirror.diagnostics()["cargo_displacement"], places=9)

    def test_force_uses_only_its_joint_equality_row(self):
        sim = Simulation(self.configs["warehouse_cargo_demo"])
        for _ in range(100):
            sim.step()
        mask = ((sim.data.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY)
                & (sim.data.efc_id == sim._latch))
        other = (sim.data.efc_id == sim._latch) & ~mask
        self.assertEqual(int(mask.sum()), 1)
        self.assertGreater(int(other.sum()), 0)  # Contacts reuse constraint-local ids.
        expected = abs(float(sim.data.efc_force[mask][0]))
        sim.data.efc_force[other] = 1e9
        self.assertEqual(equality_force(sim.model, sim.data, sim._latch), expected)
        sim.data.eq_active[sim._latch] = False
        mujoco.mj_forward(sim.model, sim.data)
        self.assertEqual(equality_force(sim.model, sim.data, sim._latch), 0.)

    def test_release_requires_continuous_armed_overload(self):
        sim = Simulation(self.configs["warehouse_cargo_demo"])
        row = ((sim.data.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY)
               & (sim.data.efc_id == sim._latch))
        sim.data.efc_force[row] = 100.
        sim._apply_fault()
        self.assertEqual(sim._latch_overload, 0.)
        sim.data.time = .6
        for _ in range(4):
            sim._apply_fault()
        self.assertEqual(sim.events, [])
        sim.data.efc_force[row] = sim.config.latch_strength - .01
        sim._apply_fault()
        self.assertEqual(sim._latch_overload, 0.)
        sim.data.efc_force[row] = sim.config.latch_strength
        for _ in range(4):
            sim._apply_fault()
        self.assertEqual(sim.events, [])
        sim._apply_fault()
        self.assertEqual(len(sim.events), 1)
        self.assertAlmostEqual(sim.events[0]["overload_duration_s"], .01)

    def test_no_turn_and_strong_restraint_remain_identical_to_nominal(self):
        cfg = replace(self.configs["warehouse_cargo_demo"], duration=5.)
        actual, nominal = Simulation(cfg), Simulation(replace(cfg, fault="healthy"))
        while not actual.finished:
            command = {"left": .6, "right": .6}
            actual.step(command)
            nominal.step(command)
        self.assertEqual(actual.events, [])
        np.testing.assert_array_equal(actual.data.qpos, nominal.data.qpos)
        cfg = replace(cfg, duration=11., latch_strength=10000.)
        restrained, nominal = run(cfg), run(replace(cfg, fault="healthy"))
        self.assertEqual(restrained.events, [])
        np.testing.assert_array_equal(restrained.data.qpos, nominal.data.qpos)

    def test_replay_reset_and_timestep_sensitivity(self):
        for name, baseline in self.runs.items():
            with self.subTest(name=name):
                actual = run(baseline.config)
                np.testing.assert_array_equal(actual.data.qpos, baseline.data.qpos)
                finer = run(replace(actual.config, timestep=.001))
                self.assertFalse(finer.data.warning.number.any())
                self.assertTrue(finer.summary()["public"]["safe"])
                if actual.events:
                    self.assertLess(abs(finer.events[0]["time"] - actual.events[0]["time"]), .01)
                else:
                    self.assertEqual(finer.events, [])
                np.testing.assert_allclose(finer.data.qpos[:3], actual.data.qpos[:3], atol=.08, rtol=0.)
                expected = actual.data.qpos.copy()
                events = list(actual.events)
                displacement = actual.diagnostics()["cargo_displacement"]
                actual.reset_trial()
                self.assertEqual(actual.diagnostics()["cargo_latched"], not bool(events))
                self.assertEqual(actual.diagnostics()["cargo_displacement"], displacement if events else 0.)
                self.assertEqual(actual.events, events)
                actual.step()
                actual.reset_full()
                fresh = Simulation(actual.config)
                self.assertEqual(actual.observe(), fresh.observe())
                self.assertEqual(actual.diagnostics(), fresh.diagnostics())
                while not actual.finished:
                    actual.step()
                np.testing.assert_array_equal(actual.data.qpos, expected)
                self.assertEqual(actual.events, events)

    def test_public_observations_do_not_expose_the_hidden_mechanism(self):
        for actual in self.runs.values():
            observation = json.dumps(actual.observe())
            summary = json.dumps(actual.summary()["public"])
            for forbidden in ("cargo", "latch", "force", "fault", "mass", "events", "config"):
                self.assertNotIn(forbidden, observation)
                self.assertNotIn(forbidden, summary)
            private = actual.diagnostics()
            for field in ("cargo_latch_force_N", "cargo_velocity", "wheel_normal_load_N"):
                self.assertIn(field, private)

    def test_control_scaling_preserves_explicit_interventions(self):
        for probe in CARGO_PROBES:
            sim = Simulation(Config(probe=probe, drive_scale=1.5, duration=4.))
            sim.data.time = 3.2
            sim.step()
            expected = {name: float(np.clip(value * 1.5, -1., 1.))
                        for name, value in probe_command(probe, 3.2).items()}
            self.assertEqual(sim.last_command, expected)
            sim.step({"left": .2, "right": -.3})
            self.assertEqual(sim.last_command, {"left": .2, "right": -.3})


if __name__ == "__main__":
    unittest.main()
