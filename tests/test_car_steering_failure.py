"""F3 keeps the fault in the rack and measures physical lane departure."""

from dataclasses import fields, replace
import json
import math
import unittest

import mujoco
import numpy as np

from simulator.platforms.car_steering import Config, PRESETS, Simulation


def advance(sim, until=None):
    target = sim.config.duration if until is None else until
    while sim.elapsed < target - sim.config.timestep / 2 and not sim.finished:
        sim.step()


class CarSteeringFailureTests(unittest.TestCase):
    def test_initial_failure_is_only_steering_with_healthy_running_gear(self):
        sim = Simulation(Config(**PRESETS["car_steering_drift"]))
        healthy = Simulation(replace(sim.config, fault="healthy"))
        self.assertEqual(sim.observe()["phase"], "straight_entry")
        self.assertAlmostEqual(sim.observe()["speed"], 4)
        self.assertEqual([e["event"] for e in sim.public_events], ["task_start"])
        for name in ("geom_size", "geom_friction", "geom_solref", "jnt_stiffness",
                     "dof_damping", "body_quat", "body_mass", "body_inertia"):
            np.testing.assert_array_equal(getattr(sim.model, name), getattr(healthy.model, name))
        self.assertEqual(sim.diagnostics()["rack_gain"], sim.config.steering_gain)
        self.assertEqual(sim.diagnostics()["rack_bias"], sim.config.steering_bias)
        self.assertEqual(healthy.diagnostics()["rack_gain"], 1)
        self.assertEqual(healthy.diagnostics()["rack_bias"], 0)
        self.assertEqual(mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_GEOM, "impact_barrier"), -1)
        self.assertTrue(sim._roadside_geoms)
        for geom in sim._roadside_geoms:
            self.assertGreater(sim.model.geom_contype[geom], 0)
        self.assertGreaterEqual(mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_GEOM,
                                                "finish_gate_header"), 0)
        self.assertFalse(sim.observe()["goal_reached"])
        for camera in sim.task_metadata["cameras"]:
            self.assertGreaterEqual(mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_CAMERA, camera), 0)

    def test_failure_attempts_bend_departs_for_half_second_and_physically_stops(self):
        sim = Simulation(Config())
        previous = sim.data.qpos[:3].copy()
        while not sim.finished:
            sim.step()
            self.assertLess(np.linalg.norm(sim.data.qpos[:3] - previous), 0.025,
                            "Physics must not teleport or reposition the car")
            previous = sim.data.qpos[:3].copy()
            self.assertFalse(sim.data.qfrc_applied.any())
            self.assertFalse(sim.data.xfrc_applied.any())
        public = sim.summary()["public"]
        self.assertEqual(public["outcome"], "lane_departure")
        metrics = public["metrics"]
        self.assertGreater(metrics["distance_traveled"], 18)
        self.assertGreater(metrics["max_tire_departure"], .3)
        self.assertTrue(metrics["barrier_contact"])
        self.assertGreater(metrics["impact_speed"], 3)
        self.assertGreaterEqual(metrics["barrier_contact_count"], 1)
        self.assertFalse(metrics["goal_reached"])
        self.assertLess(metrics["final_speed"], .01)
        events = {event["event"]: event for event in sim.public_events}
        self.assertLess(events["bend_entry"]["time"], events["tire_boundary_crossing"]["time"])
        self.assertGreater(events["lane_exit"]["time"], 4)
        self.assertGreaterEqual(events["lane_exit"]["outside_duration"], .5)
        self.assertGreater(events["barrier_contact"]["time"], events["lane_exit"]["time"])
        self.assertGreater(events["barrier_contact"]["normal_force"], 0)
        self.assertEqual(events["brake_onset"]["time"], events["barrier_contact"]["time"])
        self.assertAlmostEqual(sim.elapsed - events["barrier_contact"]["time"], 2.0)
        self.assertLess(sim.elapsed, sim.config.duration)
        self.assertIn("FAILED", sim.presentation()["status"])
        self.assertIn("not reached", sim.presentation()["detail"])
        self.assertEqual(sim.observe()["command"]["throttle"], 0)
        self.assertFalse(any(event["event"] == "finish_line" for event in sim.public_events))
        self.assertFalse(any(sim.summary()["warnings"]))
        json.dumps(sim.summary(), allow_nan=False)

    def test_same_nominal_driver_finishes_healthy_course_with_frozen_bounds(self):
        sim = Simulation(Config(**PRESETS["car_steering_nominal"]))
        advance(sim)
        result = sim.summary()["public"]
        self.assertEqual(result["outcome"], "course_complete")
        self.assertTrue(result["safe"])
        metrics = result["metrics"]
        self.assertLessEqual(metrics["max_lateral_error"], .30)
        self.assertLess(metrics["max_heading_error_after_2s"], math.radians(5))
        self.assertEqual(metrics["max_tire_departure"], 0)
        self.assertLess(metrics["finish_time"], 15)
        self.assertLess(metrics["final_speed"], .01)
        self.assertFalse(metrics["barrier_contact"])
        self.assertEqual(metrics["barrier_contact_count"], 0)
        self.assertTrue(metrics["goal_reached"])
        self.assertIn("GATE REACHED", sim.presentation()["status"])
        self.assertAlmostEqual(sim.elapsed - metrics["finish_time"], 2.0)
        self.assertIn("developer", sim.task_metadata["provenance"])
        self.assertFalse(any(sim.summary()["warnings"]))

    def test_three_repetitions_and_half_timestep_preserve_outcome_and_metrics(self):
        primary = Simulation(Config())
        advance(primary)
        expected = primary.summary()
        for _ in range(2):
            repeated = Simulation(primary.config)
            advance(repeated)
            np.testing.assert_array_equal(primary.data.qpos, repeated.data.qpos)
            self.assertEqual(expected, repeated.summary())
        for fault in ("steering", "healthy"):
            nominal_step = primary if fault == "steering" else Simulation(Config(fault=fault))
            advance(nominal_step)
            half_step = Simulation(replace(nominal_step.config, timestep=.001))
            advance(half_step)
            a, b = [s.summary()["public"] for s in (nominal_step, half_step)]
            self.assertEqual(a["outcome"], b["outcome"])
            for key in ("distance_traveled", "max_lateral_error", "max_heading_error_after_2s"):
                self.assertLess(abs(a["metrics"][key] - b["metrics"][key]),
                                max(abs(a["metrics"][key]) * .02, 1e-8), key)
            self.assertEqual(a["metrics"]["barrier_contact"], b["metrics"]["barrier_contact"])
            if fault == "steering":
                self.assertLessEqual(abs(a["metrics"]["barrier_contact_time"]
                                         - b["metrics"]["barrier_contact_time"]), .002)
                self.assertLess(abs(a["metrics"]["impact_speed"] - b["metrics"]["impact_speed"]), .01)
            self.assertFalse(any(half_step.summary()["warnings"]))

    def test_gain_development_and_two_held_out_fixture_variants(self):
        configs = [Config(steering_gain=.15, steering_bias=0),
                   Config(steering_bias=.02, bend_offset=-3.5),
                   Config(probe_speed=4.4, bend_offset=3.0)]
        for config in configs:
            with self.subTest(config=config):
                failed, healthy = Simulation(config), Simulation(replace(config, fault="healthy"))
                advance(failed)
                advance(healthy)
                self.assertEqual(failed.summary()["public"]["outcome"], "lane_departure")
                self.assertEqual(healthy.summary()["public"]["outcome"], "course_complete")
                self.assertTrue(failed.observe()["barrier_contact"])
                self.assertFalse(healthy.observe()["barrier_contact"])
                self.assertFalse(any(failed.summary()["warnings"]))
                self.assertFalse(any(healthy.summary()["warnings"]))

    def test_reset_is_explicit_in_place_and_preserves_initial_mismatch(self):
        sim = Simulation(Config())
        model, data = sim.model, sim.data
        advance(sim)
        pose, result = sim.data.qpos.copy(), sim.summary()
        sim.reset_full()
        self.assertIs(sim.model, model)
        self.assertIs(sim.data, data)
        self.assertEqual(sim.elapsed, 0)
        advance(sim)
        np.testing.assert_array_equal(pose, sim.data.qpos)
        self.assertEqual(result, sim.summary())
        before, rack = sim.elapsed, sim.diagnostics()["rack_bias"]
        sim.reset_trial()
        self.assertEqual(sim.elapsed, before)
        self.assertEqual(sim.data.time, before)
        self.assertEqual(sim.diagnostics()["rack_bias"], rack)
        self.assertEqual(sim.public_events[-2]["event"], "trial_reset")
        self.assertFalse(sim.observe()["barrier_contact"])
        self.assertFalse(sim.observe()["goal_reached"])
        advance(sim, before + sim.config.duration)
        self.assertTrue(sim.finished)
        np.testing.assert_array_equal(pose, sim.data.qpos)
        self.assertEqual(sim.summary()["public"]["outcome"], "lane_departure")

    def test_terminal_pose_holds_and_presentation_stays_public_and_ascii(self):
        sim = Simulation(Config())
        advance(sim)
        pose, time, events = sim.data.qpos.copy(), sim.elapsed, sim.public_events
        for _ in range(10):
            sim.step()
        np.testing.assert_array_equal(pose, sim.data.qpos)
        self.assertEqual(time, sim.elapsed)
        self.assertEqual(events, sim.public_events)
        presentation = sim.presentation()
        self.assertEqual(set(presentation), {"objective", "status", "detail"})
        for text in presentation.values():
            self.assertTrue(text.isascii())
            self.assertLessEqual(len(text), 45)
        self.assertIn("green finish gate", presentation["objective"])

    def test_observations_and_driver_do_not_expose_or_read_hidden_rack_values(self):
        sim = Simulation(Config())
        advance(sim, 3)
        command = sim._nominal_command()
        sim._rack_gain, sim._rack_bias = 1, .2
        self.assertEqual(command, sim._nominal_command())
        public = json.dumps({"observation": sim.observe(), "events": sim.public_events,
                             "summary": sim.summary()["public"], "task": sim.task_metadata})
        for key in ("steering_gain", "steering_bias", "rack_gain", "rack_bias", '"fault"'):
            self.assertNotIn(key, public)
        for key in ("steering_angles", "command", "lateral_error", "heading_error",
                    "tire_departure", "progress", "wheel_positions"):
            self.assertIn(key, sim.observe())

    def test_invalid_configs_and_commands_never_advance_physics(self):
        config = Config()
        for field in fields(config):
            if isinstance(getattr(config, field.name), (int, float)):
                with self.subTest(field=field.name), self.assertRaises(ValueError):
                    replace(config, **{field.name: float("nan")})
        for kwargs in ({"fault": "pressure"}, {"probe": "braking"}, {"finish_x": 20}):
            with self.assertRaises(ValueError):
                Config(**kwargs)
        sim = Simulation(config)
        pose = sim.data.qpos.copy()
        for command in ({"steering": 1}, {"brake": -1}, {"throttle": float("inf")},
                        {"fault": "healthy"}, {"brake": True}):
            with self.assertRaises(ValueError):
                sim.step(command)
        self.assertEqual(sim.elapsed, 0)
        np.testing.assert_array_equal(pose, sim.data.qpos)

    def test_stopping_before_the_route_cannot_count_as_completion(self):
        sim = Simulation(Config(fault="healthy", duration=3))
        while not sim.finished:
            sim.step({"brake": 1.0})
        public = sim.summary()["public"]
        self.assertEqual(public["outcome"], "incomplete_course")
        self.assertFalse(public["safe"])
        self.assertLess(public["metrics"]["final_speed"], .01)
        self.assertLess(public["metrics"]["progress"], .1)


if __name__ == "__main__":
    unittest.main()
