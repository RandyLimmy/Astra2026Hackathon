"""Measured speed-transition failure with healthy motors and a matched control."""
from dataclasses import replace
import json
import unittest

import numpy as np

from simulator.platforms.quadruped import Config, JOINTS, LEGS, PRESETS, Simulation


def finish(config, control=None):
    simulation = Simulation(config)
    while not simulation.finished:
        simulation.step(control)
    return simulation


class GaitFailureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = Config(**PRESETS["quadruped_gait_failure"])
        cls.failure = finish(cls.config)
        # This is a developer-authored feasibility control, never an Astra run.
        cls.control = finish(replace(cls.config, coordination_defect=False))
        cls.half_step = finish(replace(cls.config, timestep=cls.config.timestep / 2))
        cls.half_control = finish(replace(cls.config, timestep=cls.config.timestep / 2,
                                         coordination_defect=False))

    def test_walks_then_physically_loses_support_and_falls(self):
        result = self.failure.summary()["public"]
        metrics = result["metrics"]
        self.assertEqual(result["outcome"], "fell")
        self.assertFalse(result["safe"])
        self.assertFalse(result["task_complete"])
        self.assertGreater(metrics["distance_before_speed_transition_m"], 0.4)
        self.assertGreater(metrics["support_loss_time"], self.config.speed_transition_at)
        self.assertGreater(metrics["fall_time"], metrics["support_loss_time"])
        self.assertGreater(metrics["body_contact_time"], metrics["support_loss_time"])
        self.assertGreater(metrics["body_contact_duration_s"], 2)
        self.assertGreater(metrics["max_tilt_deg"], 65)
        self.assertLess(metrics["min_body_height_m"], 0.15)
        np.testing.assert_array_equal(self.failure.model.actuator_gainprm[:, 0], np.ones(12))
        np.testing.assert_array_equal(self.failure.data.qfrc_applied, np.zeros(self.failure.model.nv))
        np.testing.assert_array_equal(self.failure.data.xfrc_applied, np.zeros((self.failure.model.nbody, 6)))

    def test_identical_mechanics_and_speed_schedule_are_feasible(self):
        for sim in (self.control, self.half_control):
            result = sim.summary()["public"]
            self.assertEqual(result["outcome"], "upright")
            self.assertTrue(result["safe"])
            self.assertTrue(result["task_complete"])
            self.assertFalse(result["diagnostic_speed_override"])
            self.assertEqual(result["metrics"]["body_contact_duration_s"], 0)
            self.assertLess(result["metrics"]["max_tilt_deg"], 20)
            self.assertLess(result["metrics"]["max_lateral_distance_m"], 0.25)
            self.assertGreater(result["metrics"]["forward_distance_m"], 1.5)
        for field in ("body_mass", "body_inertia", "actuator_gainprm", "geom_friction", "jnt_stiffness"):
            np.testing.assert_array_equal(getattr(self.failure.model, field), getattr(self.control.model, field))
        for time in (0, 6, 7, 7.5, 8, 10):
            commands = []
            for defect in (True, False):
                sim = Simulation(replace(self.config, coordination_defect=defect))
                sim.data.time = time
                commands.append(sim._controls(None)["forward_speed"])
            self.assertEqual(commands[0], commands[1])
            self.assertGreaterEqual(commands[0], self.config.speed)
            self.assertLessEqual(commands[0], self.config.accelerated_speed)

    def test_halving_timestep_preserves_outcome_and_onset(self):
        for sim in (self.failure, self.half_step, self.control, self.half_control):
            self.assertEqual(sim.summary()["warnings"], {})
            self.assertTrue(np.isfinite(sim.data.qpos).all())
            self.assertTrue(np.isfinite(sim.data.qvel).all())
        a = self.failure.summary()["public"]["metrics"]
        b = self.half_step.summary()["public"]["metrics"]
        self.assertEqual(self.half_step.summary()["public"]["outcome"], "fell")
        self.assertLess(abs(a["fall_time"] - b["fall_time"]), 0.2)
        self.assertLess(abs(a["body_contact_time"] - b["body_contact_time"]), 0.2)
        self.assertLess(abs(a["distance_before_speed_transition_m"] - b["distance_before_speed_transition_m"]), 0.02)

    def test_neutral_observed_events_and_diagnostic_evidence(self):
        events = self.failure.public_events()
        self.assertEqual([event["event"] for event in events],
                         ["walking_started", "speed_transition", "support_loss", "fall", "body_contact"])
        self.assertEqual([event["time"] for event in events], sorted(event["time"] for event in events))
        self.assertAlmostEqual(events[1]["time"], self.config.speed_transition_at, places=6)
        self.assertTrue(all(set(event) == {"time", "event", "label"} for event in events))
        events[0]["event"] = "changed_by_caller"
        self.assertEqual(self.failure.public_events()[0]["event"], "walking_started")
        observed = self.failure.observe()
        self.assertEqual(observed["phase"], "fallen")
        self.assertEqual(set(observed["joint_positions"]), set(JOINTS))
        for key in ("foot_contacts", "foot_positions", "foot_targets", "commanded_stance"):
            self.assertEqual(set(observed[key]), set(LEGS))
        self.assertAlmostEqual(observed["requested_speed_mps"], self.config.accelerated_speed)
        self.assertTrue(np.isfinite(observed["actual_forward_speed_mps"]))
        serialized = json.dumps({"observation": observed, "events": self.failure.public_events(),
                                 "summary": self.failure.summary()["public"]}, allow_nan=False)
        for private in ("coordination_defect", "gait_coordination", "strength", "fault_at", "paired"):
            self.assertNotIn(private, serialized)

    def test_full_reset_repeats_actual_failure_deterministically(self):
        sim = Simulation(self.config)
        for _ in range(round(10 / self.config.timestep)):
            sim.step()
        position, velocity, events = sim.data.qpos.copy(), sim.data.qvel.copy(), sim.public_events()
        sim.reset_full()
        self.assertEqual(sim.elapsed, 0)
        self.assertEqual(sim.public_events(), [])
        self.assertFalse(sim.summary()["public"]["task_complete"])
        self.assertIsNone(sim.summary()["public"]["metrics"]["mean_speed_final_window_mps"])
        for _ in range(round(10 / self.config.timestep)):
            sim.step()
        np.testing.assert_array_equal(sim.data.qpos, position)
        np.testing.assert_array_equal(sim.data.qvel, velocity)
        self.assertEqual(sim.public_events(), events)

    def test_low_speed_probe_does_not_enter_faster_gait_mode(self):
        sim = Simulation(self.config)
        while not sim.finished:
            sim.step({"forward_speed": self.config.speed})
        self.assertEqual(sim.summary()["public"]["outcome"], "upright")
        self.assertEqual(sim.observe()["requested_speed_mps"], self.config.speed)
        self.assertEqual(sim.observe()["command_speed_mps"], self.config.speed)
        self.assertEqual(sim.observe()["task_requested_speed_mps"], self.config.accelerated_speed)
        self.assertFalse(sim.summary()["public"]["task_complete"])
        self.assertFalse(any(event["event"] in ("support_loss", "fall", "body_contact")
                             for event in sim.public_events()))

    def test_standing_cannot_complete_or_rewrite_the_walking_task(self):
        for config, command in ((self.config, {"forward_speed": 0}),
                                (replace(self.config, probe="stand"), None)):
            with self.subTest(command=command, probe=config.probe):
                sim = finish(config, command)
                public, observation = sim.summary()["public"], sim.observe()
                self.assertEqual(public["outcome"], "upright")
                self.assertTrue(public["safe"], "Retain legacy diagnostic balance semantics")
                self.assertFalse(public["task_complete"])
                self.assertEqual(observation["task_requested_speed_mps"], self.config.accelerated_speed)
                self.assertEqual(observation["command_speed_mps"], 0)
                self.assertEqual(observation["phase"], "standing")
                self.assertFalse(any(event["event"] == "walking_started" for event in sim.public_events()))
                self.assertAlmostEqual(public["metrics"]["task_requested_distance_m"],
                                       self.control.summary()["public"]["metrics"]["task_requested_distance_m"])

    def test_near_request_speed_override_is_diagnostic_even_with_acceptable_measured_speed(self):
        sim = finish(self.config, {"forward_speed": 0.12})
        public = sim.summary()["public"]
        metrics = public["metrics"]
        self.assertTrue(public["safe"])
        self.assertTrue(public["diagnostic_speed_override"])
        self.assertFalse(public["task_complete"])
        self.assertLess(abs(metrics["mean_speed_final_window_mps"] - self.config.accelerated_speed),
                        0.2 * self.config.accelerated_speed)
        sim.reset_full()
        self.assertFalse(sim.summary()["public"]["diagnostic_speed_override"])

    def test_external_joint_targets_do_not_count_ignored_speed_as_gait_override(self):
        sim = Simulation(self.config)
        while sim.data.time < 1.02:
            sim.step()
        targets = list(sim.observe()["joint_positions"].values())
        sim.step({"joint_targets": targets, "forward_speed": 0.12})
        self.assertEqual(sim.observe()["controller_mode"], "joint_targets")
        self.assertFalse(sim.summary()["public"]["diagnostic_speed_override"])
        sim.step({"forward_speed": sim.observe()["task_requested_speed_mps"]})
        self.assertFalse(sim.summary()["public"]["diagnostic_speed_override"])
        sim.step({"forward_speed": 0.12})
        self.assertTrue(sim.summary()["public"]["diagnostic_speed_override"])

    def test_walking_bookmark_requires_observed_forward_progress(self):
        sim = Simulation(self.config)
        while not any(event["event"] == "walking_started" for event in sim.public_events()):
            sim.step()
            self.assertLess(sim.elapsed, 4)
        observation = sim.observe()
        self.assertGreaterEqual(observation["position"][0], 0.04)
        self.assertGreater(observation["actual_forward_speed_mps"], 0.025)
        self.assertGreaterEqual(sum(observation["foot_contacts"].values()), 2)

    def test_external_targets_do_not_export_previous_gait_metadata(self):
        sim = Simulation(self.config)
        sim.step()
        self.assertEqual(sim.observe()["controller_mode"], "gait")
        self.assertIsInstance(sim.observe()["foot_targets"], dict)
        targets = list(sim.observe()["joint_positions"].values())
        sim.step({"joint_targets": targets})
        observation = sim.observe()
        self.assertEqual(observation["controller_mode"], "joint_targets")
        self.assertEqual(observation["command"]["joint_targets"], targets)
        self.assertIsNone(observation["foot_targets"])
        self.assertIsNone(observation["commanded_stance"])
        self.assertEqual(set(observation["foot_positions"]), set(LEGS))
        sim.step({"motors_enabled": False})
        self.assertEqual(sim.observe()["controller_mode"], "motors_disabled")
        self.assertIsNone(sim.observe()["commanded_stance"])
        sim.step()
        self.assertEqual(sim.observe()["controller_mode"], "gait")
        self.assertIsInstance(sim.observe()["commanded_stance"], dict)

    def test_schedule_validation_and_external_commands_are_atomic(self):
        for field in ("speed_transition_at", "speed_ramp_duration", "accelerated_speed"):
            for value in (True, "1", float("nan"), float("inf"), -1):
                with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                    replace(self.config, **{field: value})
        for values in ({"speed_transition_at": 111}, {"speed_ramp_duration": 0},
                       {"speed_ramp_duration": 11}, {"accelerated_speed": 0.3},
                       {"coordination_defect": 0}, {"coordination_defect": "false"}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                replace(self.config, **values)
        sim = Simulation(self.config)
        initial = sim.data.qpos.copy()
        for command in ({"joint_targets": [0] * 11}, {"forward_speed": float("nan")},
                        {"forward_speed": 0.3}, {"gait_phase": "invalid"},
                        {"task_requested_speed_mps": 0}):
            with self.subTest(command=command), self.assertRaises(ValueError):
                sim.step(command)
            np.testing.assert_array_equal(sim.data.qpos, initial)
            self.assertEqual(sim.elapsed, 0)
        sim.step({"forward_speed": 0.05})
        self.assertEqual(sim.observe()["command"]["forward_speed"], 0.05)


def test_epsilon_speed_command_uses_exact_task_speed_for_actuation():
    sim = Simulation(Config(**PRESETS["quadruped_gait_failure"]))
    while not sim.finished:
        previous_time = float(sim.data.time)
        task_speed = sim.observe()["task_requested_speed_mps"]
        sim.step({"forward_speed": task_speed - 1e-10})
        if previous_time >= 1:
            assert sim.observe()["command_speed_mps"] == task_speed
    result = sim.summary()["public"]
    assert not result["diagnostic_speed_override"]
    assert not result["task_complete"]
    assert result["outcome"] == "fell"
    assert result["metrics"]["body_contact_duration_s"] > 2


if __name__ == "__main__":
    unittest.main()
