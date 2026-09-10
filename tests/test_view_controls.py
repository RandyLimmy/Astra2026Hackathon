"""Settings precedence and presentation controls must not change physical state."""

import contextlib
from dataclasses import replace
import io
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import mujoco
import mujoco.viewer
import numpy as np

from simulator import __main__ as cli
from simulator import lab
from simulator.platforms import catalog, operator
from simulator.view_controls import ViewControls, setting_overrides, settings_lines


class ViewControlTests(unittest.TestCase):
    def test_slow_display_frames_do_not_limit_physics_to_one_step_per_frame(self):
        for scenario in ("warehouse_shift", "wheel_loss"):
            with self.subTest(scenario=scenario):
                clock = 0.
                closed = False
                frames = 0
                args = cli._parser().parse_args(["view", scenario, "--autoplay", "--duration", "0.3"])
                if catalog.is_platform(scenario):
                    sim = catalog.create(scenario, {"duration": .3, "fault_at": .05})
                    factory = patch.object(catalog, "create", return_value=sim)
                else:
                    from simulator.config import Experiment
                    sim = cli.Simulator(Experiment(initial_speed=5, wall=False, duration=.3,
                                                   detach_wheel="FR", detach_at=.05))
                    factory = patch.object(cli, "Simulator", return_value=sim)
                window = SimpleNamespace(cam=mujoco.MjvCamera(), lock=contextlib.nullcontext,
                                         set_texts=lambda texts: None)

                def close():
                    nonlocal closed
                    closed = True

                def sleep(seconds):
                    nonlocal clock
                    clock += max(seconds, .0001)

                def sync():
                    nonlocal clock, frames
                    frames += 1
                    trial_time = sim.observe().get("trial_time", sim.trial_time)
                    # At 20 Hz rendering, physics should still keep its 500 Hz
                    # schedule rather than accumulating one frame's delay each sync.
                    self.assertGreaterEqual(trial_time + .004, min(clock, .3))
                    clock += .05
                    if trial_time >= .3 - 1e-9:
                        close()
                    self.assertLess(frames, 12)

                window.sync, window.close = sync, close
                window.is_running = lambda: not closed

                @contextlib.contextmanager
                def launch(*args, **kwargs):
                    try:
                        yield window
                    finally:
                        close()

                with (factory, patch.object(cli, "_config", return_value=sim.config),
                      patch.object(mujoco.viewer, "_MJPYTHON", object()),
                      patch.object(mujoco.viewer, "launch_passive", side_effect=launch),
                      patch.object(operator.time, "monotonic", side_effect=lambda: clock),
                      patch.object(operator.time, "sleep", side_effect=sleep),
                      contextlib.redirect_stdout(io.StringIO())):
                    cli._view(args)
                self.assertGreater(frames, 2)
                self.assertTrue(sim.events)

    def test_settings_parse_typed_values_and_reject_bad_assignments(self):
        self.assertEqual(setting_overrides(["probe=showcase", "rotor_effectiveness=0.8", "wall=false",
                                           "commands=[[0,0,1]]"]),
                         {"probe": "showcase", "rotor_effectiveness": .8,
                          "wall": False, "commands": [[0, 0, 1]]})
        for value in ("missing", "=1", "name=", "a.b=2", "duration=NaN", "duration=Infinity"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                setting_overrides([value])

    def test_config_then_settings_then_explicit_flag_precedence_for_both_families(self):
        with TemporaryDirectory() as temporary:
            config = Path(temporary) / "config.json"
            config.write_text(json.dumps({"duration": .4}))
            for scenario in ("baseline", "drone_demo"):
                args = cli._parser().parse_args(["view", scenario, "--config", str(config),
                                                 "--set", "duration=0.3", "--duration", ".2"])
                result = (catalog.create(scenario, operator.overrides(args)).config
                          if catalog.is_platform(scenario) else cli._config(args))
                self.assertEqual(result.duration, .2)
            with contextlib.redirect_stdout(io.StringIO()):
                lab.main(["run", "drone_hover", "--config", str(config), "--set", "duration=0.3",
                          "--duration", ".2", "--output", str(Path(temporary) / "run")])
            saved = json.loads((Path(temporary) / "run/private/summary.json").read_text())
            self.assertEqual(saved["config"]["duration"], .2)
        with self.assertRaises(ValueError):
            catalog.create("drone_demo", setting_overrides(["not_a_setting=1"]))
        with self.assertRaises(ValueError):
            catalog.create("drone_demo", setting_overrides(["rotor_effectiveness=2"]))

    def test_camera_and_speed_only_affect_presentation(self):
        sim = catalog.create("drone_hover", {"duration": .2})
        control = ViewControls(sim.model, scenario="drone_hover", config=sim.config)
        window = SimpleNamespace(cam=mujoco.MjvCamera(), lock=contextlib.nullcontext)
        position = sim.observe()["position"]
        before = sim.data.qpos.copy(), sim.data.qvel.copy(), sim.model.opt.timestep
        control.apply_camera(window, position)
        first_camera = window.cam.fixedcamid
        self.assertTrue(control.handle_key(ord("C"), window, position))
        self.assertNotEqual(window.cam.fixedcamid, first_camera)
        self.assertTrue(control.handle_key(ord("+"), window, position))
        self.assertEqual(control.speedup, 2)
        self.assertTrue(control.handle_key(ord("-"), window, position))
        self.assertEqual(control.speedup, 1)
        self.assertFalse(control.handle_key(32, window, position))
        np.testing.assert_array_equal(sim.data.qpos, before[0])
        np.testing.assert_array_equal(sim.data.qvel, before[1])
        self.assertEqual(sim.model.opt.timestep, before[2])
        reference = catalog.create("drone_hover", {"duration": .2})
        for _ in range(50):
            sim.step()
            reference.step()
        np.testing.assert_array_equal(sim.data.qpos, reference.data.qpos)

    def test_overlay_status_and_only_relevant_operator_settings(self):
        sim = catalog.create("drone_demo")
        control = ViewControls(sim.model, scenario="drone_demo", config=sim.config)
        for elapsed, paused, finished, expected in ((0, True, False, "READY"),
                (2, False, False, "PLAYING"), (3, True, False, "PAUSED"), (20, False, True, "COMPLETE")):
            text = "\n".join(row[2] for row in control.texts(elapsed=elapsed, duration=20,
                                                          phase="outbound_climb", paused=paused,
                                                          finished=finished))
            self.assertIn(expected, text)
            self.assertIn("C: camera", text)
            self.assertIn("Rotor output", text)
            self.assertNotIn("Wind force", text)
        wind_config = replace(sim.config, fault="wind")
        self.assertIn("Wind force", "\n".join(settings_lines("drone_demo", wind_config)))
        self.assertNotIn("Rotor output", "\n".join(settings_lines("drone_demo", wind_config)))

    def test_autoplay_and_camera_flag_use_the_same_viewer(self):
        sim = catalog.create("drone_hover", {"duration": .2})
        window = SimpleNamespace(cam=mujoco.MjvCamera(), lock=contextlib.nullcontext,
                                 set_texts=lambda texts: None)
        closed = False
        callback = None

        def close():
            nonlocal closed
            closed = True

        def sync():
            self.assertGreater(sim.elapsed, 0)
            self.assertEqual(window.cam.fixedcamid, sim.model.camera("side").id)
            callback(256)

        window.sync = sync
        window.close = close
        window.is_running = lambda: not closed

        @contextlib.contextmanager
        def launch(model, data, *, key_callback, show_left_ui=False, show_right_ui=False):
            nonlocal callback
            callback = key_callback
            try:
                yield window
            finally:
                close()

        args = cli._parser().parse_args(["view", "drone_hover", "--autoplay", "--camera", "side"])
        with (patch.object(catalog, "create", return_value=sim),
              patch.object(mujoco.viewer, "_MJPYTHON", object()),
              patch.object(mujoco.viewer, "launch_passive", side_effect=launch) as launched,
              contextlib.redirect_stdout(io.StringIO())):
            operator.view(args)
        self.assertEqual(launched.call_count, 1)

    def test_slow_display_refresh_does_not_limit_physics_to_one_step_per_frame(self):
        sim = catalog.create("drone_hover", {"duration": 2})
        clock = {"now": 0., "closed": False, "frames": 0}
        window = SimpleNamespace(cam=mujoco.MjvCamera(), lock=contextlib.nullcontext,
                                 set_texts=lambda texts: None)

        def close():
            clock["closed"] = True

        def sync():
            clock["frames"] += 1
            clock["now"] += .1  # slow rendering / HUD upload
            if clock["now"] >= 1:
                close()

        def sleep(duration):
            clock["now"] += duration

        window.is_running = lambda: not clock["closed"]
        window.sync = sync
        window.close = close

        @contextlib.contextmanager
        def launch(*args, **kwargs):
            try:
                yield window
            finally:
                close()

        args = cli._parser().parse_args(["view", "drone_hover", "--autoplay"])
        with (patch.object(catalog, "create", return_value=sim),
              patch.object(mujoco.viewer, "_MJPYTHON", object()),
              patch.object(mujoco.viewer, "launch_passive", side_effect=launch),
              patch.object(operator.time, "monotonic", side_effect=lambda: clock["now"]),
              patch.object(operator.time, "sleep", side_effect=sleep),
              contextlib.redirect_stdout(io.StringIO())):
            operator.view(args)
        self.assertGreater(sim.elapsed, .5)
        self.assertGreater(sim.elapsed / sim.config.timestep, clock["frames"] * 10)


if __name__ == "__main__":
    unittest.main()
