"""Visible course flight, fixed route phases, and the unchanged nominal controller."""

from dataclasses import replace
import math
import unittest

import mujoco
import numpy as np

from simulator.platforms.drone import Config, PRESETS, Simulation, showcase_phase


def flight(sim):
    states = []
    commands = []
    while not sim.finished:
        sim.step()
        states.append(sim.data.qpos.copy())
        commands.append(sim.last_command.copy())
    return np.array(states), np.array(commands)


class DroneShowcaseCameraTests(unittest.TestCase):
    def test_tighter_side_camera_frames_course_and_aircraft_at_supported_distances(self):
        for distance in (2., 4., 8.):
            with self.subTest(distance=distance):
                sim = Simulation(Config(probe="showcase", flight_distance=distance))
                camera = sim.model.camera("side")
                rotation = sim.data.cam_xmat[camera.id].reshape(3, 3)
                position = sim.data.cam_xpos[camera.id]
                tangent = math.tan(math.radians(float(camera.fovy[0])) / 2)

                def project(point):
                    local = rotation.T @ (np.array(point) - position)
                    self.assertLess(local[2], 0)
                    return local[:2] / (-local[2] * tangent * np.array([16 / 9, 1.]))

                # Include the full floor pads and additional room beyond the
                # commanded route for the demonstrated degradation residual.
                for x in (-.55, distance + .48):
                    for y in (-.55, .4 * distance + .48):
                        self.assertLess(np.abs(project([x, y, .012])).max(), .98)
                for center in ([0., 0., 2.], [distance + .8, 0., 2.8],
                               [distance + .8, .4 * distance + .6, 2.6]):
                    for offset in (-.355, .355):
                        self.assertLess(np.abs(project([center[0] + offset, center[1], center[2]])).max(), .9)
                if distance == 4.:
                    # A 0.71 m rotor span remains over 100 px in the 1280x720
                    # presentation viewport at the representative changed pose.
                    left = project([4.65 - .355, .97, 2.62])
                    right = project([4.65 + .355, .97, 2.62])
                    self.assertGreater((right[0] - left[0]) * 640, 100.)
        original = Simulation()
        np.testing.assert_array_equal(original.model.camera("side").pos, [0., -4., 2.])
        self.assertEqual(original.model.camera("side").fovy[0], 40.)


class DroneShowcaseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        config = Config(**PRESETS["drone_demo"])
        cls.healthy = Simulation(replace(config, fault="healthy"))
        cls.changed = Simulation(config)
        cls.healthy_states, cls.healthy_commands = flight(cls.healthy)
        cls.changed_states, cls.changed_commands = flight(cls.changed)

    def test_course_moves_before_fault_then_turns_and_returns_to_hover(self):
        config = self.changed.config
        onset = round(config.fault_at / config.timestep)
        self.assertGreater(self.healthy_states[onset - 1, 0], 2.)
        np.testing.assert_array_equal(self.healthy_states[:onset], self.changed_states[:onset])
        np.testing.assert_array_equal(self.healthy_commands[:onset], self.changed_commands[:onset])
        self.assertGreater(np.max(self.healthy_states[:, 1]), 1.3)
        self.assertGreater(np.max(self.healthy_states[:, 2]), 2.7)
        self.assertGreater(np.linalg.norm(np.diff(self.healthy_states[:, :3], axis=0), axis=1).sum(), 8.)
        for sim, states in ((self.healthy, self.healthy_states), (self.changed, self.changed_states)):
            with self.subTest(fault=sim.config.fault):
                self.assertTrue(sim.finished)
                self.assertAlmostEqual(sim.elapsed, 20., places=8)
                self.assertTrue(np.isfinite(states).all())
                self.assertFalse(sim.data.warning.number.any())
                self.assertFalse(sim.ground_contact)
                self.assertGreater(sim.min_altitude, 1.5)
                self.assertLess(sim.max_tilt, 15.)
                self.assertLess(np.linalg.norm(sim.data.qvel[:3]), .05)
        self.assertLess(np.linalg.norm(self.healthy.data.qpos[:3] - [0, 0, 2]), .02)
        self.assertLess(np.linalg.norm(self.changed.data.qpos[:3] - [0, 0, 2]), 1.2)

    def test_degradation_leaves_a_measurable_residual_without_claiming_repair(self):
        self.assertTrue(self.healthy.summary()["public"]["safe"])
        self.assertFalse(self.changed.summary()["public"]["safe"])
        self.assertEqual(self.changed.summary()["public"]["outcome"], "outside_probe_envelope")
        self.assertGreater(self.changed.max_tracking_error, self.healthy.max_tracking_error + .3)
        self.assertGreater(np.linalg.norm(self.changed.data.qpos[:3] - self.healthy.data.qpos[:3]), .3)
        self.assertEqual(len(self.changed.events), 1)
        self.assertAlmostEqual(self.changed.events[0]["time"], 8., places=8)
        self.assertEqual(self.changed.effectiveness[0], .72)

    def test_route_levers_are_validated_and_scale_only_the_showcase(self):
        for name, bad in (("flight_distance", (1.99, 8.01)), ("flight_altitude", (1.99, 3.21))):
            for value in (*bad, float("nan"), float("inf"), True, "4", None):
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    Config(**{name: value})
        config = Config(probe="showcase", flight_distance=6., flight_altitude=3.1)
        sim = Simulation(config)
        for t, expected in ((0., [0, 0, 2]), (2., [0, 0, 2]), (8., [6, 0, 3.1]),
                            (12., [6, 2.4, 3.1]), (18., [0, 0, 2]), (25., [0, 0, 2])):
            sim.trial_time = t
            np.testing.assert_allclose(sim._probe_target(), expected, atol=1e-12)
        for probe in ("hover", "maneuver"):
            original = Simulation(Config(probe=probe))
            knobs = Simulation(Config(probe=probe, flight_distance=8., flight_altitude=3.2))
            for t in (0., 2., 4., 7., 12.):
                original.trial_time = knobs.trial_time = t
                np.testing.assert_array_equal(original._probe_target(), knobs._probe_target())

    def test_public_phase_is_the_commanded_schedule_and_never_reads_fault_state(self):
        healthy = Simulation(Config(probe="showcase"))
        changed = Simulation(Config(probe="showcase", fault="rotor_loss", fault_at=0))
        changed._apply_event()
        for t, expected in ((0., "hover_settle"), (2., "outbound_climb"), (8., "cross_course"),
                            (12., "return"), (18., "settle_hover"), (20., "settle_hover")):
            healthy.trial_time = changed.trial_time = t
            self.assertEqual(showcase_phase(t), expected)
            self.assertEqual(healthy.observe()["phase"], expected)
            self.assertEqual(changed.observe()["phase"], expected)
            target = healthy._probe_target()
            np.testing.assert_array_equal(target, changed._probe_target())
            np.testing.assert_array_equal(healthy._nominal_control(target), changed._nominal_control(target))

    def test_full_replay_restores_original_fault_schedule_and_trajectory(self):
        sim = Simulation(self.changed.config)
        for _ in range(round(9 / sim.config.timestep)):
            sim.step()
        self.assertTrue(sim.event_applied)
        model, data = sim.model, sim.data
        sim.reset_full()
        self.assertIs(sim.model, model)
        self.assertIs(sim.data, data)
        self.assertEqual(sim.elapsed, 0.)
        self.assertEqual(sim.events, [])
        np.testing.assert_array_equal(sim.effectiveness, np.ones(4))
        states, commands = flight(sim)
        np.testing.assert_array_equal(states, self.changed_states)
        np.testing.assert_array_equal(commands, self.changed_commands)
        self.assertEqual(sim.summary(), self.changed.summary())

    def test_course_marks_cannot_change_contacts_and_side_camera_faces_along_x(self):
        original = Simulation()
        course = self.healthy
        names = [mujoco.mj_id2name(course.model, mujoco.mjtObj.mjOBJ_GEOM, index)
                 for index in range(course.model.ngeom)]
        marks = [name for name in names if name is not None and name.startswith("course_")]
        self.assertGreaterEqual(len(marks), 5)
        for name in marks:
            self.assertEqual(original.model.geom(name).rgba[3], 0.)
            self.assertGreater(course.model.geom(name).rgba[3], 0.)
            self.assertEqual(int(course.model.geom(name).contype[0]), 0)
            self.assertEqual(int(course.model.geom(name).conaffinity[0]), 0)
        side = course.model.camera("side").id
        np.testing.assert_allclose(course.data.cam_xmat[side].reshape(3, 3)[:, 0], [1, 0, 0], atol=1e-12)
        np.testing.assert_array_equal(original.model.body_mass, course.model.body_mass)
        np.testing.assert_array_equal(original.model.actuator_gear, course.model.actuator_gear)
        np.testing.assert_array_equal(original.model.geom("ground").friction, course.model.geom("ground").friction)

    def test_original_presets_and_defaults_are_preserved(self):
        original = {"drone_hover": "healthy", "drone_rotor_loss": "rotor_loss",
                    "drone_voltage_sag": "voltage_sag", "drone_payload": "payload",
                    "drone_wind": "wind", "drone_delay": "delay"}
        for name, fault in original.items():
            self.assertEqual(PRESETS[name], {"fault": fault})
        self.assertEqual(Config().duration, 12.)
        self.assertEqual(Config().fault_at, 3.)
        self.assertEqual(Config().probe, "maneuver")
        self.assertEqual(Config().rotor_effectiveness, .12)


if __name__ == "__main__":
    unittest.main()
