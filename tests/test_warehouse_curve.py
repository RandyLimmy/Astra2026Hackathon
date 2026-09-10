"""A wheel-driven cart turns through a declared bend and can physically spill cargo."""
from dataclasses import replace
import math
import unittest

import mujoco
import numpy as np

from simulator.platforms.warehouse import (
    Config, PRESETS, Simulation, route_command, route_geometry,
)


ROUTE = np.array(route_geometry()["centerline"])


def rollout(config: Config) -> dict:
    sim = Simulation(config)
    contact_time = None
    max_route_error = 0.
    max_cargo_step = 0.
    previous = sim.data.xpos[sim._cargo_body].copy()
    before_impact = None
    while not sim.finished:
        sim.step()
        position = sim.data.xpos[sim.focus_body]
        cargo = sim.data.xpos[sim._cargo_body]
        max_route_error = max(max_route_error, float(np.min(np.linalg.norm(ROUTE - position[:2], axis=1))))
        max_cargo_step = max(max_cargo_step, float(np.linalg.norm(cargo - previous)))
        previous = cargo.copy()
        if sim.observe()["cargo_has_touched_floor"] and contact_time is None:
            contact_time = sim.elapsed
        if abs(sim.trial_time - 6.) < config.timestep / 2:
            before_impact = np.r_[position.copy(), cargo.copy()]
    return {"sim": sim, "contact_time": contact_time, "route_error": max_route_error,
            "cargo_step": max_cargo_step, "before_impact": before_impact}


class CurveTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = Config(**PRESETS["warehouse_curve_demo"])
        cls.actual = rollout(cls.config)
        cls.nominal = rollout(replace(cls.config, fault="healthy"))
        cls.slow = rollout(Config(**PRESETS["warehouse_curve_control_demo"]))

    def test_declared_right_angle_route_and_clean_nominal_tracking(self):
        geometry = route_geometry()
        np.testing.assert_array_equal(ROUTE[0], [0., 0.])
        np.testing.assert_allclose(ROUTE[-1], [4.2, -3.6])
        self.assertEqual(geometry["bend_angle_deg"], -90.)
        self.assertEqual(geometry["bend_radius"], 1.2)
        for record in (self.nominal, self.slow):
            sim = record["sim"]
            with self.subTest(probe=sim.config.probe):
                self.assertLess(record["route_error"], .15)
                self.assertLess(abs(sim.observe()["heading"] + math.pi / 2), .08)
                self.assertLess(np.linalg.norm(sim.data.xpos[sim.focus_body, :2] - ROUTE[-1]), .1)
                self.assertFalse(sim.observe()["cargo_has_touched_floor"])
                self.assertTrue(sim.diagnostics()["cargo_on_deck"])
                self.assertEqual(sim.summary()["public"]["outcome"], "cargo_retained")
                self.assertTrue(sim.summary()["public"]["metrics"]["vehicle_upright"])
        # Geometry remains a predeclared route after all physical rollouts.
        np.testing.assert_array_equal(np.array(route_geometry()["centerline"]), ROUTE)

    def test_sharp_turn_spills_real_cargo_with_mass_conservation_and_no_teleport(self):
        sim = self.actual["sim"]
        nominal = self.nominal["sim"]
        self.assertEqual(len(sim.events), 1)
        self.assertGreater(sim.events[0]["time"], 4.)
        self.assertLess(sim.events[0]["time"], 5.5)
        self.assertGreater(self.actual["contact_time"], sim.events[0]["time"] + .5)
        self.assertLess(self.actual["contact_time"], 7.)
        self.assertLess(self.actual["cargo_step"], .02)
        self.assertLess(self.actual["route_error"], .15)
        self.assertLess(abs(sim.observe()["heading"] + math.pi / 2), .1)
        self.assertGreater(np.linalg.norm(np.array(sim.observe()["cargo_position"]) -
                                          np.array(nominal.observe()["cargo_position"])), .5)
        np.testing.assert_array_equal(sim.model.body_mass, nominal.model.body_mass)
        self.assertEqual(sim.summary()["public"]["outcome"], "cargo_spilled")
        self.assertTrue(sim.summary()["public"]["safe"])  # Vehicle stability remains separate.
        self.assertTrue(sim.diagnostics()["cargo_dropped"])
        self.assertFalse(sim.diagnostics()["cargo_latched"])
        self.assertFalse(sim.data.warning.number.any())
        self.assertTrue(np.isfinite(sim.data.qpos).all())
        self.assertTrue(np.isfinite(sim.data.qvel).all())
        self.assertEqual(float(np.abs(sim.data.xfrc_applied).sum()), 0.)
        self.assertEqual(float(np.abs(sim.data.qfrc_applied).sum()), 0.)

    def test_free_relative_motion_and_solid_stack_have_physical_contacts(self):
        sim = self.actual["sim"]
        joint_ids = np.flatnonzero(sim.model.jnt_bodyid == sim._cargo_body)
        self.assertEqual(sim.model.jnt_type[joint_ids].tolist(),
                         [mujoco.mjtJoint.mjJNT_SLIDE] * 3 + [mujoco.mjtJoint.mjJNT_BALL])
        self.assertFalse(sim.model.jnt_limited[joint_ids].any())
        np.testing.assert_array_equal(sim.model.actuator_gear[:, 0], [9., 9.])
        pairs = {frozenset((int(first), int(second)))
                 for first, second in zip(sim.model.pair_geom1, sim.model.pair_geom2)}
        for cargo in sim._cargo_geoms:
            self.assertEqual(sim.model.geom_contype[cargo], 1)
            self.assertIn(frozenset((cargo, sim._deck)), pairs)
            self.assertIn(frozenset((cargo, sim.model.geom("chassis_geom").id)), pairs)
        # Overview and side are attached to world; chase remains attached to cart.
        self.assertEqual(sim.model.cam_bodyid[sim.model.camera("overview").id], 0)
        self.assertEqual(sim.model.cam_bodyid[sim.model.camera("side").id], 0)
        self.assertEqual(sim.model.cam_bodyid[sim.model.camera("chase").id], sim.focus_body)

    def test_same_observed_state_produces_same_controls_until_physical_release(self):
        actual = Simulation(self.config)
        nominal = Simulation(replace(self.config, fault="healthy"))
        while not actual.events:
            np.testing.assert_array_equal(actual.data.qpos, nominal.data.qpos)
            self.assertEqual(actual.observe(), nominal.observe())
            command = route_command(actual.config.probe, actual.observe(), actual.config.drive_scale)
            actual.step()
            nominal.step()
            self.assertEqual(actual.last_command, command)
            self.assertEqual(actual.last_command, nominal.last_command)
        while actual.trial_time < 6.:
            actual.step()
            nominal.step()
        self.assertNotEqual(actual.last_command, nominal.last_command)
        # Explicit torque interventions still bypass automatic route following.
        actual.step({"left": .2, "right": -.1})
        self.assertEqual(actual.last_command, {"left": .2, "right": -.1})

    def test_slow_turn_has_same_latch_strength_and_retains_load(self):
        slow = self.slow["sim"]
        self.assertEqual(slow.events, [])
        self.assertTrue(slow.diagnostics()["cargo_latched"])
        for field in ("latch_strength", "latch_dwell", "latch_arm_at"):
            self.assertEqual(getattr(slow.config, field), getattr(self.config, field))
        nominal = rollout(replace(slow.config, fault="healthy"))["sim"]
        np.testing.assert_array_equal(slow.data.qpos, nominal.data.qpos)
        self.assertEqual(slow.observe(), nominal.observe())

    def test_half_timestep_preserves_release_impact_and_route_outcome(self):
        finer = rollout(replace(self.config, timestep=.001))
        actual = self.actual["sim"]
        self.assertLess(abs(finer["sim"].events[0]["time"] - actual.events[0]["time"]), .01)
        self.assertLess(abs(finer["contact_time"] - self.actual["contact_time"]), .04)
        self.assertLess(finer["route_error"], .15)
        self.assertEqual(finer["sim"].summary()["public"]["outcome"], "cargo_spilled")
        self.assertTrue(finer["sim"].summary()["public"]["safe"])
        self.assertFalse(finer["sim"].data.warning.number.any())
        np.testing.assert_allclose(finer["before_impact"], self.actual["before_impact"], atol=.03, rtol=0.)
        # Resting pose after wheel/crate collisions is contact-sensitive;
        # assert convergence before impact and the same physical outcome.

    def test_resets_preserve_damage_and_replay_deterministically(self):
        sim = rollout(self.config)["sim"]
        expected = sim.data.qpos.copy()
        cargo_pose = sim.data.qpos[sim._cargo_qpos:].copy()
        events = list(sim.events)
        sim.reset_trial()
        self.assertFalse(sim.diagnostics()["cargo_latched"])
        self.assertEqual(sim.events, events)
        np.testing.assert_array_equal(sim.data.qpos[sim._cargo_qpos:], cargo_pose)
        sim.reset_full()
        fresh = Simulation(self.config)
        self.assertEqual(sim.observe(), fresh.observe())
        self.assertEqual(sim.diagnostics(), fresh.diagnostics())
        while not sim.finished:
            sim.step()
        np.testing.assert_array_equal(sim.data.qpos, expected)
        self.assertEqual(sim.events, events)

    def test_contact_history_is_observable_without_hidden_failure_parameters(self):
        observed = self.actual["sim"].observe()
        self.assertTrue(observed["cargo_has_touched_floor"])
        self.assertIn("cargo_floor_contact", observed)
        for private in ("fault", "latch_strength", "latch_dwell", "latch_arm_at", "events", "config"):
            self.assertNotIn(private, observed)
        legacy = Simulation(Config()).observe()
        for field in ("cargo_position", "cargo_orientation", "cargo_floor_contact", "cargo_has_touched_floor"):
            self.assertNotIn(field, legacy)


if __name__ == "__main__":
    unittest.main()
