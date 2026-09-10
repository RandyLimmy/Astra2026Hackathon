"""Delivery's failure cause, real latch, feasibility, numerical and replay contracts."""

from dataclasses import replace
import json
import unittest

import mujoco
import numpy as np

from simulator.platforms.drone import Config, MAX_ROTOR_THRUST, NOMINAL_MASS, PRESETS, Simulation
from simulator.platforms.drone_delivery import DeliverySimulation, EFFECT_SEED, IMPACT_SPEED_THRESHOLD


BASE = Config(**PRESETS["drone_delivery_imbalance"])


def fly(config=BASE, *, effects=False):
    sim = Simulation(config)
    states, commands = [], []
    scene = mujoco.MjvScene(sim.model, maxgeom=128) if effects else None
    while not sim.finished:
        sim.step()
        if scene is not None:
            scene.ngeom = 0
            sim.decorate_scene(scene)
        states.append(np.r_[sim.data.qpos, sim.data.qvel])
        commands.append(sim.last_command.copy())
    return sim, np.array(states), np.array(commands)


class DroneDeliveryControlBoundaryTests(unittest.TestCase):
    def snapshot(self, sim):
        # Include controller memory and mission bookkeeping alongside the plant
        # state: a rejected command must be a complete no-op at any boundary.
        return json.loads(json.dumps({
            "qpos": sim.data.qpos.tolist(), "qvel": sim.data.qvel.tolist(),
            "eq_active": sim.data.eq_active.tolist(), "ctrl": sim.data.ctrl.tolist(),
            "applied_forces": sim.data.xfrc_applied.tolist(), "data_time": sim.data.time,
            "elapsed": sim.elapsed, "trial_time": sim.trial_time,
            "phase": sim.phase, "phase_started": sim.phase_started,
            "target": sim.target.tolist(), "last_command": sim.last_command.tolist(),
            "command_mode": sim.command_mode, "hover_trim": sim.hover_trim.tolist(),
            "motor_state": sim.motor_state.tolist(), "support_time": sim.support_time,
            "release_time": sim.release_time, "summary": sim.summary(),
        }))

    def assert_rejections_are_noops(self, sim):
        before = self.snapshot(sim)
        invalid = [{"rotor_commands": [2, 0, 0, 0]},
                   {"target_position": [0, 0, float("nan")]},
                   {"rotor_commands": [True, 0, 0, 0]},
                   {"rotor_commands": [np.bool_(False), .2, .2, .2]},
                   {"rotor_commands": ["0.5", .5, .5, .5]},
                   {"rotor_commands": np.array([".5"] * 4)},
                   {"target_position": [0, "0", 2]},
                   {"target_position": [False, 0, 2]}]
        for control in invalid:
            with self.subTest(control=control):
                with self.assertRaises(ValueError):
                    sim.step(control)
                self.assertEqual(self.snapshot(sim), before)

    def test_rejected_control_cannot_enter_takeoff_or_change_controller_state(self):
        sim = Simulation(BASE)
        for _ in range(round(.6 / BASE.timestep)):
            sim.step()
        self.assertEqual(sim.phase, "ready")
        self.assertGreaterEqual(sim.trial_time, .6)
        self.assert_rejections_are_noops(sim)
        sim.step()
        self.assertEqual(sim.phase, "takeoff")
        self.assertGreater(sim.elapsed, .6)

    def test_rejected_control_cannot_release_a_contact_supported_parcel(self):
        sim = Simulation(replace(BASE, delivery_controller="feasibility"))
        while not (sim.phase == "placement" and sim.support_time >= .3):
            self.assertFalse(sim.finished)
            sim.step()
        self.assertTrue(sim.attached)
        self.assertTrue(sim._contacts()[1])
        self.assertIsNone(sim.release_time)
        self.assert_rejections_are_noops(sim)
        sim.step()
        self.assertFalse(sim.attached)
        self.assertEqual(sim.phase, "unloaded_climb")


class DroneDeliveryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.failure, cls.trace, cls.commands = fly()
        cls.reference, _, cls.reference_commands = fly(replace(BASE, delivery_controller="feasibility"))
        cls.centered, _, _ = fly(replace(BASE, fault="healthy"))

    def test_preset_is_additive_and_has_a_real_visible_package_before_takeoff(self):
        sim = Simulation(BASE)
        self.assertIsInstance(sim, DeliverySimulation)
        self.assertTrue(sim.attached)
        self.assertEqual(sim.model.nq, 14)
        self.assertEqual(sim.model.body_mass[sim.focus_body], NOMINAL_MASS)
        self.assertAlmostEqual(sim.model.body_mass[sim.parcel_body], .36)
        np.testing.assert_allclose(sim.data.xpos[sim.parcel_body] - sim.data.xpos[sim.focus_body], [.30, 0., -.20])
        np.testing.assert_array_equal(sim.model.actuator_ctrlrange, [[0., 6.]] * 4)
        self.assertLess(float(sim.model.body_mass.sum()) * 9.81, 4 * MAX_ROTOR_THRUST)
        self.assertGreater(sim.model.geom("parcel_box").rgba[3], 0.)
        self.assertFalse(sim.release_parcel())
        self.assertFalse(sim.summary()["public"]["safe"])

    def test_failure_is_an_airborne_outbound_imbalance_and_measured_impact(self):
        sim = self.failure
        events = {event["event"]: event for event in sim.public_events()}
        self.assertEqual(sim.summary()["public"]["outcome"], "crashed")
        self.assertTrue(sim.attached)
        self.assertGreater(sim.max_altitude, 2.)
        self.assertGreater(sim.max_forward_progress, 1.)
        self.assertGreater(sim.max_tilt, 60.)
        self.assertLess(events["liftoff"]["time"], events["outbound"]["time"])
        self.assertLess(events["outbound"]["time"], events["first_instability"]["time"])
        self.assertLess(events["first_instability"]["time"], events["crash"]["time"])
        self.assertGreater(events["crash"]["impact_speed"], IMPACT_SPEED_THRESHOLD)
        self.assertGreater(events["crash"]["position"][0], 1.)
        self.assertLess(events["crash"]["position"][2], .4)
        self.assertGreaterEqual(sim.elapsed - events["crash"]["time"], 2.5 - 1e-9)
        self.assertEqual(len([event for event in sim.events if event["event"] == "crash"]), 1)
        np.testing.assert_array_equal(sim.effectiveness, np.ones(4))
        self.assertEqual(sim.voltage, 1.)
        self.assertFalse(sim.data.warning.number.any())
        self.assertTrue(np.isfinite(self.trace).all())
        self.assertTrue(np.all((self.commands >= 0) & (self.commands <= 1)))

    def test_centering_only_the_load_removes_the_failure(self):
        sim = self.centered
        self.assertTrue(sim.completed)
        self.assertIsNone(sim.crash_event)
        self.assertLess(sim.max_tilt, 10.)
        self.assertEqual(sim.config.delivery_controller, "nominal")
        self.assertEqual(sim.config.payload_mass, self.failure.config.payload_mass)
        self.assertEqual(sim.summary()["public"]["provenance"], "centered_load_control")

    def test_same_physics_reference_delivers_returns_and_lands_within_limits(self):
        sim = self.reference
        self.assertTrue(sim.completed)
        self.assertLessEqual(sim.elapsed, 30.)
        self.assertIsNone(sim.crash_event)
        self.assertEqual(sim.in_flight_body_contacts, 0)
        self.assertTrue(sim.unloaded_flight)
        self.assertGreaterEqual(sim.parcel_settled_time, 2.)
        self.assertGreaterEqual(sim.landed_time, 1.)
        self.assertLess(np.linalg.norm(sim.data.qpos[:2]), .5)
        self.assertLess(np.linalg.norm(sim.data.qvel[:3]), .1)
        self.assertLess(np.linalg.norm(sim.data.xpos[sim.parcel_body, :2] - [4, 0]), .5)
        self.assertFalse(sim.attached)
        self.assertLess(sim.max_tilt, 10.)
        self.assertFalse(sim.data.warning.number.any())
        self.assertTrue(np.all((self.reference_commands >= 0) & (self.reference_commands <= 1)))
        np.testing.assert_array_equal(sim.model.body_mass, self.failure.model.body_mass)
        np.testing.assert_array_equal(sim.model.body_inertia, self.failure.model.body_inertia)
        np.testing.assert_array_equal(sim.model.actuator_gear, self.failure.model.actuator_gear)
        self.assertEqual(sim.summary()["public"]["provenance"], "developer_feasibility_control")

    def test_latch_release_preserves_package_mass_and_all_physical_state(self):
        sim = self.reference
        release = sim.release_state
        self.assertIsNotNone(release)
        self.assertTrue(release["contact_supported"])
        self.assertEqual(release["parcel_mass"], BASE.payload_mass)
        np.testing.assert_array_equal(release["qpos_before"], release["qpos_after"])
        np.testing.assert_array_equal(release["qvel_before"], release["qvel_after"])
        self.assertAlmostEqual(sim.model.body_mass[sim.parcel_body], BASE.payload_mass)
        self.assertEqual(sim.model.nq, 14)
        self.assertFalse(sim.release_parcel())
        events = [event["event"] for event in sim.events]
        self.assertLess(events.index("placement"), events.index("parcel_released"))
        self.assertLess(events.index("parcel_released"), events.index("return"))
        self.assertLess(events.index("return"), events.index("mission_complete"))

    def test_full_reset_replays_identical_trace_and_trial_reset_is_explicit(self):
        sim, trace, commands = fly()
        np.testing.assert_array_equal(trace, self.trace)
        np.testing.assert_array_equal(commands, self.commands)
        self.assertEqual(sim.summary(), self.failure.summary())
        model, data = sim.model, sim.data
        sim.reset_full()
        self.assertIs(sim.model, model)
        self.assertIs(sim.data, data)
        self.assertEqual(sim.elapsed, 0.)
        self.assertTrue(sim.attached)
        self.assertIsNone(sim.crash_event)
        self.assertIsNone(sim.release_state)
        self.assertEqual(sim.phase, "ready")
        reset_trace = []
        while not sim.finished:
            sim.step()
            reset_trace.append(np.r_[sim.data.qpos, sim.data.qvel])
        np.testing.assert_array_equal(reset_trace, self.trace)
        elapsed = sim.elapsed
        sim.reset_trial()
        self.assertEqual(sim.elapsed, elapsed)
        self.assertEqual(sim.data.time, elapsed)
        self.assertEqual(sim.trial_time, 0.)
        self.assertTrue(sim.attached)
        self.assertIsNone(sim.crash_event)
        self.assertEqual(sim.public_events()[0]["event"], "recovery_reposition")

    def test_half_timestep_preserves_failure_metrics_and_feasibility(self):
        fine, _, _ = fly(replace(BASE, timestep=.001))
        self.assertEqual(fine.summary()["public"]["outcome"], "crashed")
        for name in ("max_forward_progress", "max_altitude"):
            self.assertLess(abs(getattr(fine, name) / getattr(self.failure, name) - 1), .02)
        self.assertLess(abs(fine.crash_event["time"] / self.failure.crash_event["time"] - 1), .02)
        reference, _, _ = fly(replace(BASE, timestep=.001, delivery_controller="feasibility"))
        self.assertTrue(reference.completed)
        self.assertEqual(reference.in_flight_body_contacts, 0)
        self.assertLess(abs(reference.elapsed / self.reference.elapsed - 1), .02)
        np.testing.assert_allclose(reference.data.qpos[:3], self.reference.data.qpos[:3], atol=.01)
        np.testing.assert_allclose(reference.data.xpos[reference.parcel_body], self.reference.data.xpos[self.reference.parcel_body], atol=.01)

    def test_changed_mass_and_opposite_offset_are_still_physically_feasible(self):
        for values in ({"payload_offset": -.3}, {"payload_mass": .40}):
            with self.subTest(values=values):
                sim, _, _ = fly(replace(BASE, delivery_controller="feasibility", **values))
                self.assertTrue(sim.completed)
                self.assertEqual(sim.in_flight_body_contacts, 0)
                self.assertIsNone(sim.crash_event)
                self.assertLess(sim.elapsed, 30.)

    def test_effects_require_real_crash_and_are_deterministic_and_physics_free(self):
        sim = Simulation(BASE)
        scene = mujoco.MjvScene(sim.model, maxgeom=128)
        sim.decorate_scene(scene)
        self.assertEqual(scene.ngeom, 0)
        while sim.crash_event is None:
            sim.step()
            if sim.crash_event is None:
                scene.ngeom = 0
                sim.decorate_scene(scene)
                self.assertEqual(scene.ngeom, 0)
        for _ in range(30):
            sim.step()
        qpos, qvel, ctrl = sim.data.qpos.copy(), sim.data.qvel.copy(), sim.data.ctrl.copy()
        sim.decorate_scene(scene)
        self.assertGreater(scene.ngeom, 0)
        geoms = [(g.pos.copy(), g.size.copy(), g.rgba.copy()) for g in scene.geoms[:scene.ngeom]]
        scene.ngeom = 0
        sim.decorate_scene(scene)
        for expected, geom in zip(geoms, scene.geoms[:scene.ngeom]):
            for left, right in zip(expected, (geom.pos, geom.size, geom.rgba)):
                np.testing.assert_array_equal(left, right)
        np.testing.assert_array_equal(sim.data.qpos, qpos)
        np.testing.assert_array_equal(sim.data.qvel, qvel)
        np.testing.assert_array_equal(sim.data.ctrl, ctrl)
        self.assertEqual(sim.diagnostics()["effect"]["seed"], EFFECT_SEED)
        scene.ngeom = 0
        self.reference.decorate_scene(scene)
        self.assertEqual(scene.ngeom, 0)
        decorated, trace, commands = fly(effects=True)
        np.testing.assert_array_equal(trace, self.trace)
        np.testing.assert_array_equal(commands, self.commands)
        self.assertEqual(decorated.summary(), self.failure.summary())

    def test_direct_rotor_commands_reach_real_motors_and_are_recorded(self):
        sim = Simulation(BASE)
        for _ in range(100):
            sim.step({"rotor_commands": [.6, .5, .4, .3]})
        self.assertEqual(sim.command_mode, "rotor_commands")
        np.testing.assert_allclose(sim.last_command, [.6, .5, .4, .3])
        self.assertGreater(np.ptp(sim.data.ctrl), 1.)
        self.assertTrue(np.all(sim.data.ctrl <= MAX_ROTOR_THRUST))
        self.assertEqual(sim.observe()["command"]["rotor_commands"], [.6, .5, .4, .3])

    def test_public_evidence_is_neutral_and_config_rejects_impossible_payloads(self):
        forbidden = {"fault", "parcel_mass", "payload_mass", "parcel_offset", "payload_offset", "hover_trim",
                     "rotor_effectiveness", "release_state", "controller", "rotor_thrust", "motor_state"}

        def check(value):
            if isinstance(value, dict):
                self.assertFalse(forbidden.intersection(value))
                for item in value.values():
                    check(item)
            elif isinstance(value, list):
                for item in value:
                    check(item)

        for evidence in (self.failure.observe(), self.failure.public_events(), self.failure.summary()["public"]):
            check(evidence)
            json.dumps(evidence, allow_nan=False)
        for values in ({"payload_mass": 1.4}, {"fault": "rotor_loss"}, {"payload_mass": 0.},
                       {"delivery_controller": "astra"}, {"payload_offset": float("nan")}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                Simulation(replace(BASE, **values))


if __name__ == "__main__":
    unittest.main()
