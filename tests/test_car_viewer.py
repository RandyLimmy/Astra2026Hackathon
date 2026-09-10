"""Original four-wheel car replay keeps the native viewer and physical model alive."""

import contextlib
import io
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import mujoco.viewer
import numpy as np

from simulator import __main__ as cli
from simulator.config import Experiment
from simulator.runner import Simulator


class CarViewerTests(unittest.TestCase):
    def test_full_replay_restores_heat_attachments_and_declared_preparation_in_place(self):
        config = Experiment(initial_speed=5, wall=False, duration=.3, thermal=True,
                            detach_wheel="FR", detach_at=.02, warmup_cycles=1,
                            weak_wheel="FL", weak_at=.01, brake_efficiency=.4)
        sim = Simulator(config)
        sim.prepare()
        model, data = sim.model, sim.data
        expected_qpos, expected_qvel = data.qpos.copy(), data.qvel.copy()
        expected_temperature, expected_time = sim.temperature.copy(), sim.elapsed
        expected_history = list(sim.preparation_history)
        for _ in range(30):
            sim.step(0, 1)
        self.assertFalse(sim.data.eq_active[sim.attach[1]])
        self.assertEqual(sim.efficiency[0], .4)
        sim.reset_replay()
        self.assertIs(sim.model, model)
        self.assertIs(sim.data, data)
        np.testing.assert_allclose(sim.data.qpos, expected_qpos, atol=1e-10, rtol=0)
        np.testing.assert_allclose(sim.data.qvel, expected_qvel, atol=1e-10, rtol=0)
        np.testing.assert_allclose(sim.temperature, expected_temperature, atol=1e-10, rtol=0)
        np.testing.assert_array_equal(sim.data.eq_active[sim.attach], True)
        np.testing.assert_array_equal(sim.efficiency, 1)
        self.assertEqual(sim.events, [])
        self.assertAlmostEqual(sim.elapsed, expected_time)
        self.assertEqual(sim.trial_time, 0)
        self.assertEqual(sim.preparation_history, expected_history)

    def test_space_and_n_replay_the_car_without_relaunching(self):
        sim = Simulator(Experiment(initial_speed=5, wall=False, duration=.1,
                                   detach_wheel="FR", detach_at=.03))
        model, data = sim.model, sim.data
        callback = None
        closed = False
        stage = 0
        first_end = None
        window = SimpleNamespace(cam=SimpleNamespace(lookat=np.zeros(3)), set_texts=lambda texts: None)

        def close():
            nonlocal closed
            closed = True

        def sync():
            nonlocal stage, first_end
            if stage == 0:
                self.assertEqual(sim.trial_time, 0)
                callback(32)
                stage = 1
            elif stage == 1 and sim.trial_time >= sim.config.duration - 1e-9:
                first_end = sim.data.qpos.copy()
                self.assertTrue(sim.events)
                callback(32)
                stage = 2
            elif stage == 2:
                self.assertLess(sim.trial_time, .03)
                self.assertEqual(sim.events, [])
                self.assertIs(sim.model, model)
                self.assertIs(sim.data, data)
                stage = 3
            elif stage == 3 and sim.trial_time >= sim.config.duration - 1e-9:
                np.testing.assert_allclose(sim.data.qpos, first_end, atol=1e-10, rtol=0)
                callback(ord("N"))
                stage = 4
            elif stage == 4:
                self.assertLess(sim.trial_time, .03)
                self.assertEqual(sim.events, [])
                callback(256)
                stage = 5

        window.lock = contextlib.nullcontext
        window.is_running = lambda: not closed
        window.close = close
        window.sync = sync

        @contextlib.contextmanager
        def launch(current_model, current_data, *, key_callback, show_left_ui=False, show_right_ui=False):
            nonlocal callback
            callback = key_callback
            try:
                yield window
            finally:
                close()

        ticks = iter(np.arange(0, 100, .02))
        args = cli._parser().parse_args(["view", "wheel_loss"])
        with (patch.object(cli, "Simulator", return_value=sim) as create,
              patch.object(mujoco.viewer, "launch_passive", side_effect=launch) as launched,
              patch.object(mujoco.viewer, "_MJPYTHON", object()),
              patch.object(cli.time, "monotonic", side_effect=lambda: next(ticks)),
              patch.object(cli.time, "sleep"), contextlib.redirect_stdout(io.StringIO())):
            cli._view(args)
        self.assertEqual(stage, 5)
        self.assertEqual(create.call_count, 1)
        self.assertEqual(launched.call_count, 1)


if __name__ == "__main__":
    unittest.main()
