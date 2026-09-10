"""Regressions for timing and coordinate semantics independent of scenario calibration."""
import math
import unittest

import mujoco
import numpy as np

from simulator.config import Experiment
from simulator.runner import Simulator


class RunContractTest(unittest.TestCase):
    def test_json_strings_are_not_boolean_flags(self):
        for field in ("wall", "thermal"):
            with self.assertRaises(ValueError):
                Experiment.from_dict({field: "false"})

    def test_failed_preparation_restores_wall(self):
        sim = Simulator(Experiment(warmup_cycles=1))
        before = (sim.model.geom_contype[sim.wall], sim.model.geom_conaffinity[sim.wall],
                  sim.model.geom_rgba[sim.wall, 3])

        def fail(current, phase):
            raise RuntimeError("recording interrupted")

        with self.assertRaisesRegex(RuntimeError, "recording interrupted"):
            sim.prepare(fail)
        after = (sim.model.geom_contype[sim.wall], sim.model.geom_conaffinity[sim.wall],
                 sim.model.geom_rgba[sim.wall, 3])
        self.assertEqual(before, after)

    def test_future_command_is_not_skipped_while_stationary(self):
        sim = Simulator(Experiment(initial_speed=0, wall=False, duration=2,
                                   commands=((0, 0, 1), (1, 1, 0))))
        result = sim.run()
        self.assertGreaterEqual(result['trial_duration'], 1.99)
        self.assertGreater(result['final_speed'], 1)

    def test_stationary_without_braking_is_a_stop_not_a_distance(self):
        result = Simulator(Experiment(initial_speed=0, wall=False)).run()
        self.assertTrue(result['stopped'])
        self.assertFalse(result['censored'])
        self.assertIsNone(result['stopping_distance'])

    def test_yaw_rate_is_derivative_of_world_yaw(self):
        sim = Simulator(Experiment(wall=False))
        angle = math.pi / 4
        sim.data.qpos[3:7] = [math.cos(angle / 2), 0, math.sin(angle / 2), 0]
        sim.data.qvel[:] = 0
        sim.data.qvel[5] = 1
        mujoco.mj_forward(sim.model, sim.data)
        before = sim.yaw
        reported = sim.observe()['yaw_rate']
        mujoco.mj_integratePos(sim.model, sim.data.qpos, sim.data.qvel, 1e-6)
        mujoco.mj_forward(sim.model, sim.data)
        self.assertAlmostEqual(reported, (sim.yaw - before) / 1e-6, places=5)

    def test_vertical_heading_marks_yaw_rate_undefined(self):
        sim = Simulator(Experiment(wall=False))
        sim.data.qpos[3:7] = [np.sqrt(.5), 0, np.sqrt(.5), 0]
        mujoco.mj_forward(sim.model, sim.data)
        self.assertIsNone(sim.observe()['yaw_rate'])


if __name__ == '__main__':
    unittest.main()
