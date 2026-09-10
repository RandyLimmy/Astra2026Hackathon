"""Physical quadruped checks: gait, observable mismatch, component faults, resets."""
from dataclasses import replace
import json
import unittest

import mujoco
import numpy as np

from simulator.platforms.quadruped import Config, DESCRIPTIONS, JOINTS, LEGS, PRESETS, Simulation


def advance(sim: Simulation, seconds: float) -> None:
    for _ in range(round(seconds / sim.config.timestep)):
        sim.step()


class QuadrupedConfigTests(unittest.TestCase):
    def test_config_rejects_nonfinite_invalid_and_unknown_fields(self):
        fields = ("duration", "timestep", "fault_at", "strength", "foot_friction", "damage_stiffness",
                  "damage_rest_angle", "payload_offset", "speed", "gait_period")
        for name in fields:
            for value in (float("nan"), float("inf"), -float("inf"), True, "1"):
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    Config(**{name: value})
        for values in ({"duration": 0}, {"timestep": 0.01}, {"fault_at": -1}, {"strength": 1.1},
                       {"foot_friction": 0}, {"damage_stiffness": -1}, {"damage_rest_angle": 0},
                       {"payload_offset": 0.4}, {"speed": 0.5}, {"gait_period": 0},
                       {"fault": "invented"}, {"probe": "jump"}, {"affected_leg": "head"}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                Config(**values)
        self.assertEqual(PRESETS.keys(), DESCRIPTIONS.keys())


class QuadrupedScenarioTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.runs = {}
        cls.prefixes = {}
        for name, overrides in PRESETS.items():
            if name.endswith("_demo") or name == "quadruped_gait_failure":
                continue  # These regressions compare the original shared five-second prefix.
            sim = Simulation(Config(**overrides))
            advance(sim, sim.config.fault_at)
            cls.prefixes[name] = sim.data.qpos.copy()
            while not sim.finished:
                sim.step()
            cls.runs[name] = sim

    def test_healthy_crawl_is_unsupported_forward_locomotion(self):
        sim = self.runs["quadruped_walk"]
        result = sim.summary()["public"]
        self.assertTrue(result["safe"])
        self.assertEqual(result["outcome"], "upright")
        self.assertGreater(result["metrics"]["forward_distance_m"], 0.5)
        self.assertGreater(result["metrics"]["min_body_height_m"], 0.32)
        self.assertLess(result["metrics"]["max_tilt_deg"], 25)
        self.assertEqual(sim.model.jnt_type[0], mujoco.mjtJoint.mjJNT_FREE)
        self.assertEqual(sim.model.nu, 12)
        np.testing.assert_array_equal(sim.data.qfrc_applied, np.zeros(sim.model.nv))
        np.testing.assert_array_equal(sim.data.xfrc_applied, np.zeros((sim.model.nbody, 6)))
        self.assertGreater(np.linalg.norm(sim.data.ctrl), 1)

    def test_all_presets_share_exact_healthy_prefix_and_finite_physics(self):
        nominal = self.prefixes["quadruped_walk"]
        for name, sim in self.runs.items():
            with self.subTest(name=name):
                np.testing.assert_array_equal(self.prefixes[name], nominal)
                self.assertTrue(np.isfinite(sim.data.qpos).all())
                self.assertTrue(np.isfinite(sim.data.qvel).all())
                self.assertEqual(sim.summary()["warnings"], {})
                self.assertEqual(set(sim.observe()["joint_positions"]), set(JOINTS))
                self.assertEqual(set(sim.observe()["foot_contacts"]), set(LEGS))
                self.assertEqual(sim.observe()["position"], sim.observe()["pose"]["position"])
                self.assertEqual(sim.observe()["velocity"], sim.observe()["linear_velocity"])
                json.dumps(sim.observe(), allow_nan=False)
                json.dumps(sim.summary(), allow_nan=False)
                for camera in ("overview", "side", "chase"):
                    self.assertGreaterEqual(mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_CAMERA, camera), 0)
                if name != "quadruped_walk":
                    self.assertEqual(len(sim.events), 1)
                    self.assertAlmostEqual(sim.events[0]["time"], 5.0)
                    self.assertGreater(np.linalg.norm(sim.data.qpos[:3] - self.runs["quadruped_walk"].data.qpos[:3]), 0.025)

    def test_damage_changes_component_physics_without_visual_or_sensor_labels(self):
        nominal = self.runs["quadruped_walk"]
        weak = self.runs["quadruped_joint_weakness"]
        slip = self.runs["quadruped_foot_slip"]
        bent = self.runs["quadruped_leg_damage"]
        payload = self.runs["quadruped_payload_shift"]
        strength = weak.diagnostics()["actuator_strength"]
        self.assertEqual(strength["FL_knee"], 0.08)
        self.assertTrue(all(value == 1 for key, value in strength.items() if key != "FL_knee"))
        self.assertEqual(slip.diagnostics()["foot_friction"]["FL"], 0.025)
        self.assertEqual(bent.diagnostics()["joint_stiffness"]["FL_knee"], 90)
        self.assertAlmostEqual(bent.diagnostics()["joint_rest_angles"]["FL_knee"], -2.35)
        self.assertAlmostEqual(payload.diagnostics()["payload_target"], 0.29)
        self.assertGreater(payload.diagnostics()["payload_offset"], 0.25)
        self.assertAlmostEqual(payload.model.body_mass.sum(), nominal.model.body_mass.sum())
        for sim in (weak, slip, bent, payload):
            np.testing.assert_array_equal(sim.model.geom_rgba, nominal.model.geom_rgba)
            public = {"observation": sim.observe(), "summary": sim.summary()["public"]}
            text = json.dumps(public)
            for hidden in ("fault", "strength", "friction", "stiffness", "payload_offset", "events", "config"):
                self.assertNotIn(hidden, text)

    def test_stiff_knee_causes_physical_fall_after_healthy_walking(self):
        result = self.runs["quadruped_leg_damage"].summary()["public"]
        self.assertFalse(result["safe"])
        self.assertEqual(result["outcome"], "fell")
        self.assertGreater(result["metrics"]["fall_time"], 5.0)
        self.assertLess(result["metrics"]["fall_time"], 10.0)
        self.assertLess(result["metrics"]["min_body_height_m"], 0.15)
        self.assertGreater(result["metrics"]["max_tilt_deg"], 90)


class QuadrupedInterventionTests(unittest.TestCase):
    def test_reset_trial_retains_each_fault_and_full_reset_replays(self):
        for name, overrides in PRESETS.items():
            if name in ("quadruped_walk", "quadruped_gait_failure"):
                continue
            with self.subTest(name=name):
                config = Config(**{**overrides, "fault_at": 0.1, "duration": 0.5})
                sim = Simulation(config)
                advance(sim, 0.2)
                end = sim.elapsed
                damaged = sim.diagnostics()
                sim.reset_trial()
                self.assertAlmostEqual(sim.elapsed, end)
                self.assertFalse(sim.finished)
                self.assertEqual(sim.data.time, 0)
                self.assertTrue(sim.diagnostics()["fault_active"])
                for field in ("actuator_strength", "foot_friction", "joint_stiffness", "joint_rest_angles", "payload_target"):
                    self.assertEqual(sim.diagnostics()[field], damaged[field])
                sim.step()
                self.assertGreater(sim.elapsed, end)
                self.assertEqual(len(sim.events), 1)
                sim.reset_full()
                fresh = Simulation(config)
                self.assertEqual(sim.config, config)
                self.assertEqual(sim.elapsed, 0)
                self.assertEqual(sim.events, [])
                self.assertEqual(sim.diagnostics(), fresh.diagnostics())
                for _ in range(80):
                    sim.step()
                    fresh.step()
                np.testing.assert_array_equal(sim.data.qpos, fresh.data.qpos)

    def test_passive_probe_really_loses_support(self):
        sim = Simulation(Config(duration=1, probe="passive"))
        self.assertEqual(sim.data.qvel[0], 0)
        while not sim.finished:
            sim.step()
            np.testing.assert_array_equal(sim.data.ctrl, np.zeros(12))
            np.testing.assert_array_equal(sim.data.qfrc_applied, np.zeros(sim.model.nv))
        self.assertEqual(sim.summary()["public"]["outcome"], "fell")
        self.assertLess(sim.data.qpos[2], 0.2)
        self.assertEqual(sim.summary()["warnings"], {})

    def test_contact_friction_is_applied_by_contact_solver(self):
        sim = Simulation(Config(fault="foot_slip", fault_at=0.1, probe="stand", duration=0.5))
        advance(sim, 0.3)
        foot = sim.model.geom("FL_foot").id
        contacts = [contact for contact in sim.data.contact if foot in contact.geom]
        self.assertGreater(len(contacts), 0)
        for contact in contacts:
            self.assertAlmostEqual(contact.friction[0], sim.config.foot_friction)

    def test_explicit_commands_and_input_validation_are_atomic(self):
        sim = Simulation(Config())
        initial = sim.data.qpos.copy()
        invalid = ({"forward_speed": float("nan")}, {"yaw_rate": float("inf")},
                   {"forward_speed": 10}, {"yaw_rate": True}, {"motors_enabled": 1},
                   {"joint_targets": [0] * 11}, {"joint_targets": [float("nan")] * 12},
                   {"joint_targets": [9] * 12}, {"hidden_damage": 0.5})
        for command in invalid:
            with self.subTest(command=command), self.assertRaises(ValueError):
                sim.step(command)
            np.testing.assert_array_equal(sim.data.qpos, initial)
            self.assertEqual(sim.elapsed, 0)
        targets = list(sim.observe()["joint_positions"].values())
        targets[0] += 0.1
        sim.step({"joint_targets": targets})
        self.assertGreater(sim.data.ctrl[0], 0)
        self.assertEqual(sim.observe()["command"]["joint_targets"], targets)
        sim.step({"motors_enabled": False})
        np.testing.assert_array_equal(sim.data.ctrl, np.zeros(12))
        self.assertFalse(sim.summary()["public"]["safe"], "An unfinished probe is not a safety result")

    def test_standing_conservative_and_turning_heldout_probes_remain_stable(self):
        for probe in ("stand", "conservative", "turn"):
            with self.subTest(probe=probe):
                sim = Simulation(Config(duration=8, probe=probe))
                while not sim.finished:
                    sim.step()
                result = sim.summary()["public"]
                self.assertTrue(result["safe"])
                self.assertEqual(sim.summary()["warnings"], {})
                if probe == "stand":
                    self.assertLess(abs(result["metrics"]["forward_distance_m"]), 0.03)
                else:
                    self.assertGreater(result["metrics"]["forward_distance_m"], 0.3)
                if probe == "turn":
                    self.assertGreater(abs(sim.data.xmat[sim.focus_body].reshape(3, 3)[1, 0]), 0.2)

    def test_fault_free_alias_reproduces_nominal(self):
        healthy = Simulation(Config(duration=0.2))
        none = Simulation(replace(healthy.config, fault="none"))
        while not healthy.finished:
            healthy.step()
            none.step()
        np.testing.assert_array_equal(healthy.data.qpos, none.data.qpos)


if __name__ == "__main__":
    unittest.main()
