"""A released parcel clears the wheels, tips, lands and comes to rest."""
from dataclasses import replace
import unittest

import mujoco
import numpy as np

from simulator.platforms.warehouse import Config, PRESETS, Simulation


def cargo_motion(config):
    sim = Simulation(config)
    rows = []
    tire_contact = False
    previous = sim.data.xpos[sim._cargo_body].copy()
    max_step = 0.
    while not sim.finished:
        sim.step()
        velocity = np.zeros(6)
        mujoco.mj_objectVelocity(sim.model, sim.data, mujoco.mjtObj.mjOBJ_BODY,
                                sim._cargo_body, velocity, 0)
        position = sim.data.xpos[sim._cargo_body].copy()
        max_step = max(max_step, float(np.linalg.norm(position - previous)))
        previous = position
        cargo_contacts = [c for c in sim.data.contact
                          if {c.geom1, c.geom2} & sim._cargo_geoms]
        tire_contact |= any({c.geom1, c.geom2} & set(sim._tires) for c in cargo_contacts)
        tilt = np.arccos(np.clip(sim.data.xmat[sim._cargo_body, 8], -1., 1.))
        rows.append([sim.elapsed, *position, *velocity, tilt,
                     sim._cargo_floor_contact, sim._cargo_on_deck, len(cargo_contacts)])
    return sim, np.asarray(rows), tire_contact, max_step


class NaturalCargoTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = Config(**PRESETS["warehouse_curve_demo"])
        cls.fast = cargo_motion(cls.config)

    def test_parcel_clears_wheels_and_settles_separately_after_landing(self):
        sim, rows, tire_contact, max_step = self.fast
        impact = rows[rows[:, 11] > 0][0, 0]
        settled = rows[rows[:, 0] > impact + 1.5]
        self.assertFalse(tire_contact, "the driven tires must not recapture spilled cargo")
        self.assertLess(np.linalg.norm(settled[:, 7:10], axis=1).max(), .05)
        self.assertLess(np.linalg.norm(settled[:, 4:7], axis=1).max(), .15)
        self.assertLess(np.linalg.norm(settled[-1, 1:4] - settled[0, 1:4]), .02)
        self.assertGreater(np.linalg.norm(rows[-1, 1:3] - sim.data.xpos[sim.focus_body, :2]), 1.)
        self.assertLess(max_step, .02)
        self.assertFalse(sim.data.warning.number.any())
        self.assertEqual(np.abs(sim.data.qfrc_applied).sum(), 0.)
        self.assertEqual(np.abs(sim.data.xfrc_applied).sum(), 0.)

    def test_release_has_a_slide_tip_fall_and_real_floor_contact(self):
        sim, rows, _, _ = self.fast
        release = sim.events[0]["time"]
        impact = rows[rows[:, 11] > 0][0, 0]
        falling = rows[(rows[:, 0] > release) & (rows[:, 0] < impact)]
        self.assertEqual(len(sim.events), 1)
        self.assertGreater(release, 4.)
        self.assertLess(release, 5.5)
        self.assertGreater(impact - release, .5)
        self.assertTrue(np.any(falling[:, 12] > 0), "slide starts with deck support")
        self.assertGreater(falling[:, 10].max(), np.radians(20.))
        self.assertTrue(np.any((falling[:, 13] == 0) & (falling[:, 9] < -.2)))
        self.assertEqual(sim.summary()["public"]["outcome"], "cargo_spilled")
        self.assertEqual(sim.model.body_mass[sim._cargo_body], sim.config.payload_mass)

    def test_slow_control_retains_identical_parcel(self):
        slow, rows, tires, _ = cargo_motion(Config(**PRESETS["warehouse_curve_control_demo"]))
        self.assertEqual(slow.events, [])
        self.assertFalse(rows[:, 11].any())
        self.assertFalse(tires)
        self.assertTrue(slow.diagnostics()["cargo_on_deck"])
        self.assertEqual(slow.summary()["public"]["outcome"], "cargo_retained")
        np.testing.assert_array_equal(slow.model.body_mass, self.fast[0].model.body_mass)

    def test_repeated_and_finer_runs_preserve_natural_spill(self):
        repeated = cargo_motion(self.config)
        np.testing.assert_array_equal(repeated[1], self.fast[1])
        finer = cargo_motion(replace(self.config, timestep=.001))
        self.assertFalse(finer[2])
        impact = self.fast[1][self.fast[1][:, 11] > 0][0, 0]
        finer_impact = finer[1][finer[1][:, 11] > 0][0, 0]
        self.assertLess(abs(impact - finer_impact), .04)
        self.assertLess(abs(self.fast[0].events[0]["time"] - finer[0].events[0]["time"]), .01)
        self.assertLess(np.linalg.norm(finer[1][-1, 7:10]), .05)
        self.assertEqual(finer[0].summary()["public"]["outcome"], "cargo_spilled")


if __name__ == "__main__":
    unittest.main()
