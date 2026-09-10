"""Recorded replay controls, tested with actual scene placement and a fake UI clock."""

import contextlib
from copy import deepcopy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import mujoco.viewer
import numpy as np

from demo import viewer as replay
from sim import cli


class ReplayViewerTests(unittest.TestCase):
    def replay(self, on_frame, *, duration_s=None, **options):
        candidate = {"probe": [{"t_s": 0., "x_m": 0., "v_mps": 2.},
                               {"t_s": .1, "x_m": .2, "v_mps": 1.},
                               {"t_s": .2, "x_m": .3, "v_mps": 0.}],
                     "summary": {"stopping_distance_m": .3, "wall_crossed": False}}
        reference = {"probe": [{"t_s": 0., "x_m": 0., "v_mps": 2.},
                               {"t_s": .1, "x_m": .15, "v_mps": 0.}],
                     "summary": {"stopping_distance_m": .15, "wall_crossed": False}}
        recorded_runs = deepcopy((candidate, reference))
        clock = 0.
        frames = []
        window = SimpleNamespace(cam=SimpleNamespace(lookat=np.zeros(3)), closed=False, texts=[])
        window.lock = contextlib.nullcontext
        window.is_running = lambda: not window.closed
        window.set_texts = lambda texts: setattr(window, "texts", texts)

        def sleep(_seconds):
            nonlocal clock
            clock += .05

        @contextlib.contextmanager
        def launch(model, data, *, key_callback, **options):
            def sync():
                self.assertLess(len(frames), 250, "Playback did not respond to the exit command")
                frame = {"time": data.time, "wall_time": clock, "position": data.qpos.copy(),
                         "camera": (window.cam.azimuth, window.cam.elevation, window.cam.distance),
                         "lookat": window.cam.lookat.copy(),
                         "text": "\n".join(text[2] for text in window.texts)}
                self.assertTrue(all(text[0] == mujoco.mjtFont.mjFONT_SHADOW for text in window.texts))
                frames.append(frame)
                on_frame(frame, key_callback)
            window.sync = sync
            try:
                yield window
            finally:
                window.closed = True

        with (patch.object(mujoco.viewer, "launch_passive", side_effect=launch) as launched,
              patch.object(replay.time, "monotonic", side_effect=lambda: clock),
              patch.object(replay.time, "sleep", side_effect=sleep),
              patch.object(mujoco, "mj_step") as physics_step):
            replay.show_comparison(candidate, reference, wall_distance_m=1., duration_s=duration_s, **options)
        self.assertEqual(launched.call_count, 1)
        self.assertTrue(window.closed)
        physics_step.assert_not_called()
        self.assertEqual((candidate, reference), recorded_runs, "Playback must not mutate saved trajectories")
        for frame in frames:
            self.assertIn("Recorded synthetic trajectories", frame["text"])
            self.assertIn("Visual barrier only; contact does not alter the recorded motion.", frame["text"])
        return frames

    def test_autoplay_speed_changes_wall_clock_duration_without_changing_recorded_motion(self):
        def on_frame(frame, key):
            if frame["time"] == .2:
                key(256)

        normal = self.replay(on_frame, autoplay=True)
        accelerated = self.replay(on_frame, autoplay=True, speedup=2.)
        self.assertIn("Replaying |", accelerated[0]["text"])
        self.assertIn("2x playback", accelerated[0]["text"])
        self.assertAlmostEqual(normal[-1]["wall_time"], 2 * accelerated[-1]["wall_time"])
        for frame in accelerated:
            matching = next(sample for sample in normal if abs(sample["time"] - frame["time"]) < 1e-12)
            np.testing.assert_allclose(frame["position"], matching["position"], atol=1e-12)

    def test_speed_keys_change_playback_rate_and_keep_recorded_positions(self):
        commands = iter((ord(" "), ord("+"), ord("-"), 256))
        frames = self.replay(lambda frame, key: key(next(commands)))
        np.testing.assert_allclose([frame["time"] for frame in frames], [0., .05, .15, .2])
        np.testing.assert_allclose([frame["position"] for frame in frames],
                                   [[0., 0.], [.1, .075], [.25, .15], [.3, .15]])
        self.assertIn("2x playback", frames[2]["text"])
        self.assertIn("1x playback", frames[3]["text"])

    def test_camera_cycle_preserves_fitted_initial_view_and_paused_trajectory(self):
        commands = iter((ord("C"), ord("c"), ord("C"), 256))
        frames = self.replay(lambda frame, key: key(next(commands)))
        self.assertEqual([frame["camera"] for frame in frames],
                         [(90., -53., 26.), (90., -10., 26.), (90., -80., 26.), (130., -25., 26.)])
        for frame, camera in zip(frames, ("free", "side", "overview", "free")):
            self.assertIn(f"{camera} camera", frame["text"])
            self.assertIn("C: cycle camera | - / +: playback speed", frame["text"])
            self.assertEqual(frame["time"], 0.)
            np.testing.assert_array_equal(frame["position"], [0., 0.])
            np.testing.assert_array_equal(frame["lookat"], [7.5, 0., 0.])

    def test_invalid_speed_is_rejected_before_opening_a_window(self):
        for speedup in (0., -1., float("inf"), float("nan"), True):
            with self.subTest(speedup=speedup), self.assertRaisesRegex(ValueError, "speedup"):
                self.replay(lambda frame, key: self.fail("Invalid speed opened the replay"), speedup=speedup)

    def test_view_cli_forwards_playback_options(self):
        bundle = {"title": "Replay test", "config": {"wall_distance_m": 12.},
                  "candidate": {"source": "candidate"}, "reference": {"source": "reference"}}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "replay.json"
            path.write_text(json.dumps(bundle))
            with (patch("sys.argv", ["sim.cli", "view", str(path), "--speedup", "2", "--autoplay", "--duration", "3"]),
                  patch.object(replay, "show_comparison") as show):
                cli.main()
        show.assert_called_once_with(bundle["candidate"], bundle["reference"],
                                     title="Replay test · synthetic replay", duration_s=3.,
                                     speedup=2., autoplay=True, wall_distance_m=12.)

    def test_space_starts_pauses_resumes_and_replays_without_automatic_loop(self):
        stage = 0
        initial_frames = paused_frames = completed_frames = 0
        paused_at = 0.
        terminal_position = None

        def on_frame(frame, key):
            nonlocal stage, initial_frames, paused_frames, completed_frames, paused_at, terminal_position
            if stage == 0:
                self.assertEqual(frame["time"], 0., "Playback must wait for Space")
                initial_frames += 1
                if initial_frames == 3:
                    key(32)
                    stage = 1
            elif stage == 1 and frame["time"] >= .1:
                paused_at = frame["time"]
                key(32)
                stage = 2
            elif stage == 2:
                self.assertEqual(frame["time"], paused_at)
                self.assertIn("Paused |", frame["text"])
                paused_frames += 1
                if paused_frames == 3:
                    key(32)
                    stage = 3
            elif stage == 3 and frame["time"] == .2:
                terminal_position = frame["position"].copy()
                np.testing.assert_allclose(terminal_position, [.3, .15], atol=1e-12)
                stage = 4
            elif stage == 4:
                self.assertEqual(frame["time"], .2, "The final frame must remain until a replay command")
                np.testing.assert_array_equal(frame["position"], terminal_position)
                self.assertIn("Replay complete |", frame["text"])
                completed_frames += 1
                # Hold longer than the previous 2.5-second automatic-loop delay.
                if completed_frames == 70:
                    key(32)
                    stage = 5
            elif stage == 5:
                self.assertEqual(frame["time"], 0.)
                np.testing.assert_array_equal(frame["position"], [0., 0.])
                self.assertIn("Replaying |", frame["text"])
                stage = 6
            elif stage == 6 and frame["time"] == .2:
                np.testing.assert_array_equal(frame["position"], terminal_position)
                key(256)
                stage = 7

        self.replay(on_frame)
        self.assertEqual(stage, 7)

    def test_r_and_n_restart_and_play_when_paused_in_the_same_window(self):
        keys = iter((ord("R"), ord("r"), ord("N"), ord("n")))
        stage = 0
        paused_at = 0.
        restarts = 0

        def on_frame(frame, key):
            nonlocal stage, paused_at, restarts
            if stage == 0:
                self.assertIn("Paused |", frame["text"])
                self.assertEqual(frame["time"], paused_at)
                next_key = next(keys, 256)
                key(next_key)
                stage = 1 if next_key != 256 else 3
            elif stage == 1:
                self.assertEqual(frame["time"], 0.)
                self.assertIn("Replaying |", frame["text"])
                restarts += 1
                stage = 2
            elif stage == 2 and frame["time"] >= .1:
                key(32)
                paused_at = frame["time"]
                stage = 0

        self.replay(on_frame)
        self.assertEqual(stage, 3)
        self.assertEqual(restarts, 4)

    def test_watchdog_closes_while_waiting_to_start_or_holding_final_frame(self):
        for start in (False, True):
            with self.subTest(start=start):
                started = False

                def on_frame(frame, key):
                    nonlocal started
                    if start and not started:
                        key(32)
                        started = True

                frames = self.replay(on_frame, duration_s=.75)
                self.assertEqual(frames[-1]["time"], .2 if start else 0.)
                self.assertGreater(len(frames), 5)

    def test_escape_closes_the_initial_paused_window(self):
        frames = self.replay(lambda frame, key: key(256))
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0]["time"], 0.)


if __name__ == "__main__":
    unittest.main()
