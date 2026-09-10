"""Viewer resets must not relaunch while macOS tears down its UI thread."""

import argparse
import contextlib
from copy import deepcopy
import io
from itertools import count
import math
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import mujoco.viewer
import numpy as np

from simulator.platforms import catalog, operator


class PlatformViewerTests(unittest.TestCase):
    def test_overlay_reports_completed_events_and_observed_outcomes_then_clears_on_replay(self):
        for scenario in ("drone_rotor_loss", "quadruped_joint_weakness", "warehouse_shift",
                         "car_steering_damage"):
            with self.subTest(scenario=scenario):
                sim = catalog.create(scenario, {"duration": 6 if scenario.startswith("car_") else .2,
                                                "fault_at": .002})
                self.assertNotIn("Event:", operator.view_detail(sim, sim.observe()))
                self.assertNotIn("Outcome:", operator.view_detail(sim, sim.observe()))
                while not sim.finished:
                    operator.checked_step(sim)
                text = operator.view_detail(sim, sim.observe())
                self.assertIn("Position (m):", text)
                event = "structural damage" if scenario.startswith("car_") else sim.config.fault.replace("_", " ")
                self.assertIn(f"Event: {event.capitalize()} at", text)
                self.assertNotIn("Recovery reposition", text)
                self.assertIn("Outcome: " + sim.summary()["public"]["outcome"].replace("_", " "), text)
                sim.reset_full()
                text = operator.view_detail(sim, sim.observe())
                self.assertNotIn("Event:", text)
                self.assertNotIn("Outcome:", text)

    def args(self, scenario):
        return argparse.Namespace(scenario=scenario, config=None, duration=.2,
                                  timestep=None, fault_at=.002, probe=None, speedup=1)

    def test_repeated_trial_and_full_resets_keep_one_window_and_model(self):
        for scenario in ("drone_rotor_loss", "warehouse_battery", "quadruped_joint_weakness",
                         "car_steering_damage"):
            with self.subTest(scenario=scenario):
                sim = catalog.create(scenario, {"duration": .2, "fault_at": .002})
                model, data = sim.model, sim.data
                keys = iter((ord("R"), ord("N"), ord("r"), ord("n"), 256))
                closed = False
                locked = False
                callback = None
                window = SimpleNamespace(cam=SimpleNamespace(lookat=np.zeros(3)), set_texts=lambda texts: None)

                @contextlib.contextmanager
                def lock():
                    nonlocal locked
                    locked = True
                    try:
                        yield
                    finally:
                        locked = False

                def close():
                    nonlocal closed
                    closed = True

                def sync():
                    self.assertIs(sim.model, model)
                    self.assertIs(sim.data, data)
                    callback(next(keys, 256))

                window.lock = lock
                window.close = close
                window.is_running = lambda: not closed
                window.sync = sync

                @contextlib.contextmanager
                def launch(current_model, current_data, *, key_callback, show_left_ui=False, show_right_ui=False):
                    nonlocal callback
                    self.assertIs(current_model, model)
                    self.assertIs(current_data, data)
                    callback = key_callback
                    try:
                        yield window
                    finally:
                        close()

                trial_reset, full_reset = sim.reset_trial, sim.reset_full

                def trial():
                    self.assertTrue(locked, "Reset mutates shared physics and must hold the viewer lock")
                    trial_reset()

                def full():
                    self.assertTrue(locked, "Reset mutates shared physics and must hold the viewer lock")
                    full_reset()

                # Advance wall time on each call to exercise sync without real sleeps.
                ticks = iter(np.arange(0, 100, .02))
                with (patch.object(catalog, "create", return_value=sim) as create,
                      patch.object(mujoco.viewer, "launch_passive", side_effect=launch) as launched,
                      patch.object(mujoco.viewer, "_MJPYTHON", object()),
                      patch.object(sim, "reset_trial", side_effect=trial) as reset_trial,
                      patch.object(sim, "reset_full", side_effect=full) as reset_full,
                      patch.object(operator.time, "monotonic", side_effect=lambda: next(ticks)),
                      patch.object(operator.time, "sleep"), contextlib.redirect_stdout(io.StringIO())):
                    operator.view(self.args(scenario))
                self.assertEqual(create.call_count, 1)
                self.assertEqual(launched.call_count, 1)
                # Some full resets internally invoke reset_trial too.
                self.assertGreaterEqual(reset_trial.call_count, 2)
                self.assertEqual(reset_full.call_count, 2)
                self.assertTrue(closed)

    def test_space_starts_pauses_resumes_and_replays_the_complete_scenario(self):
        scenarios = ("drone_hover", "drone_rotor_loss", "warehouse_healthy", "warehouse_battery",
                     "quadruped_walk", "quadruped_joint_weakness", "car_postcrash_healthy",
                     "car_steering_damage", "car_tire_pressure")
        for scenario in scenarios:
            with self.subTest(scenario=scenario):
                # Cars need actual barrier contact, recovery, and at least four
                # seconds of inspection. Timed faults can use shorter probes.
                is_car = scenario.startswith("car_")
                sim = catalog.create(scenario, {"duration": 6. if is_car else 1.5, "fault_at": .5})
                args = self.args(scenario)
                args.duration, args.fault_at = sim.config.duration, sim.config.fault_at
                model, data = sim.model, sim.data
                initial_diagnostics = deepcopy(sim.diagnostics())
                callback = None
                closed = False
                locked = False
                stage = 0
                paused_at = 0.
                ready_frames = 0
                paused_frames = 0
                finished_frames = 0
                sync_count = 0
                first_result = None
                first_events = None
                first_summary = None
                finished_at = 0.
                window = SimpleNamespace(cam=SimpleNamespace(lookat=np.zeros(3)), set_texts=lambda texts: None)

                @contextlib.contextmanager
                def lock():
                    nonlocal locked
                    locked = True
                    try:
                        yield
                    finally:
                        locked = False

                def close():
                    nonlocal closed
                    closed = True

                def completed_events():
                    events = sim.diagnostics()["events"]
                    names = [event["event"] for event in events]
                    if is_car:
                        self.assertIn("barrier_impact", names)
                        self.assertIn("recovery_reposition", names)
                        self.assertGreaterEqual(sim.summary()["public"]["metrics"]["probe_elapsed"], 4)
                    fault_event = "structural_damage" if is_car else sim.config.fault
                    if sim.config.fault == "healthy":
                        if is_car:
                            self.assertNotIn(fault_event, names)
                        else:
                            self.assertEqual(names, [])
                    else:
                        self.assertIn(fault_event, names, "Complete playback must reach the physical fault")
                        for event in events:
                            if event["event"] == fault_event:
                                self.assertGreaterEqual(event["time"] + sim.config.timestep, sim.config.fault_at)
                    return names

                def sync():
                    nonlocal stage, paused_at, ready_frames, paused_frames, finished_frames, sync_count
                    nonlocal first_result, first_events, first_summary, finished_at
                    sync_count += 1
                    self.assertLess(sync_count, 3 * math.ceil(sim.config.duration / sim.config.timestep) + 100,
                                    "Viewer must complete both runs without stalling")
                    self.assertIs(sim.model, model)
                    self.assertIs(sim.data, data)
                    if stage == 0:
                        self.assertEqual(sim.elapsed, 0, "Wait for Space before advancing the scenario")
                        ready_frames += 1
                        if ready_frames == 3:
                            callback(32)
                            stage = 1
                    elif stage == 1 and sim.elapsed >= sim.config.fault_at / 2:
                        paused_at = sim.elapsed
                        callback(32)
                        stage = 2
                    elif stage == 2:
                        self.assertEqual(sim.elapsed, paused_at)
                        paused_frames += 1
                        if paused_frames == 3:
                            callback(32)
                            stage = 3
                    elif stage == 3 and sim.finished:
                        first_events = completed_events()
                        first_result = list(sim.observe()["position"])
                        first_summary = sim.summary()["public"]
                        finished_at = sim.elapsed
                        stage = 4
                    elif stage == 4:
                        self.assertEqual(sim.elapsed, finished_at, "Completion waits for explicit replay")
                        finished_frames += 1
                        if finished_frames == 3:
                            callback(32)
                            stage = 5
                    elif stage == 5:
                        self.assertLess(sim.elapsed, sim.config.fault_at)
                        self.assertFalse(sim.finished)
                        self.assertEqual(sim.diagnostics()["events"], [])
                        stage = 6
                    elif stage == 6 and sim.finished:
                        self.assertEqual(completed_events(), first_events)
                        self.assertEqual(sim.summary()["public"]["outcome"], first_summary["outcome"])
                        self.assertEqual(sim.summary()["public"]["safe"], first_summary["safe"])
                        # Playback tests compare observable endpoints at a
                        # centimetre scale, allowing harmless contact-solver drift.
                        np.testing.assert_allclose(sim.observe()["position"], first_result, atol=.01, rtol=0)
                        callback(256)
                        stage = 7

                window.lock = lock
                window.is_running = lambda: not closed
                window.close = close
                window.sync = sync

                @contextlib.contextmanager
                def launch(current_model, current_data, *, key_callback, show_left_ui=False, show_right_ui=False):
                    nonlocal callback
                    self.assertIs(current_model, model)
                    self.assertIs(current_data, data)
                    callback = key_callback
                    try:
                        yield window
                    finally:
                        close()

                full_reset = sim.reset_full

                def replay():
                    self.assertTrue(locked, "Space replay must reset under the viewer lock")
                    full_reset()
                    self.assertIs(sim.model, model)
                    self.assertIs(sim.data, data)
                    self.assertEqual(sim.elapsed, 0)
                    self.assertFalse(sim.finished)
                    self.assertEqual(sim.diagnostics(), initial_diagnostics,
                                     "Replay restores physical fault parameters and the event schedule")

                ticks = count(step=.02)
                with (patch.object(catalog, "create", return_value=sim) as create,
                      patch.object(mujoco.viewer, "launch_passive", side_effect=launch) as launched,
                      patch.object(mujoco.viewer, "_MJPYTHON", object()),
                      patch.object(sim, "reset_full", side_effect=replay) as reset_full,
                      patch.object(operator.time, "monotonic", side_effect=lambda: next(ticks)),
                      patch.object(operator.time, "sleep"), contextlib.redirect_stdout(io.StringIO())):
                    operator.view(args)
                self.assertEqual(stage, 7)
                self.assertEqual(create.call_count, 1)
                self.assertEqual(launched.call_count, 1)
                self.assertEqual(reset_full.call_count, 1)


if __name__ == "__main__":
    unittest.main()
