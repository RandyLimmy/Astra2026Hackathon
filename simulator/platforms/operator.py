"""Trusted-operator run, matched-probe comparison, export and viewer commands."""
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
from queue import Empty, SimpleQueue
import sys
import time
from uuid import uuid4

import mujoco
import numpy as np

from ..recording import Recorder
from ..view_controls import ViewControls, setting_overrides
from . import catalog


def overrides(args):
    values = {}
    if args.config:
        values = json.loads(args.config.read_text())
        if not isinstance(values, dict):
            raise ValueError("configuration must be a JSON object")
        values = values.get("config", values)
        if not isinstance(values, dict):
            raise ValueError("config field must be an object")
    values = dict(values)
    values.update(setting_overrides(getattr(args, "settings", None)))
    for name in ("duration", "timestep", "fault_at", "probe"):
        value = getattr(args, name, None)
        if value is not None:
            values[name] = value
    for name in ("initial_speed", "brake", "brake_at", "wall_x", "recovery", "warmup_cycles"):
        if getattr(args, name, None) is not None:
            raise ValueError(f"{name} is a legacy car option; use --config for platform-specific controls")
    if getattr(args, "no_wall", False):
        raise ValueError("--no-wall belongs to the legacy braking track; use a platform diagnostic probe")
    return values


def destination(args, suffix=""):
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return args.output or Path("runs") / f"{args.scenario}{suffix}-{stamp}-{uuid4().hex[:8]}"


def checked_step(sim, control=None):
    before = sim.elapsed
    sim.step(control)
    if not sim.elapsed > before:
        raise RuntimeError("platform step did not advance the experiment clock")
    if not np.isfinite(sim.data.qpos).all() or not np.isfinite(sim.data.qvel).all():
        raise RuntimeError("platform returned non-finite physics state")
    if sim.data.warning.number.any():
        raise RuntimeError(f"MuJoCo warning counts: {sim.data.warning.number.tolist()}")


def execute(sim, output, *, frames=False, camera="chase", fps=30):
    positions: list[list[float]] = []
    times: list[float] = []
    with Recorder(output, sim.model, frames=frames, camera=camera, fps=fps) as recorder:
        def capture():
            observation = sim.observe()
            recorder.record(sim.data, observation, sim.diagnostics())
            # Bounded 50 Hz trajectory for the evaluator's matched-probe comparison.
            if not times or sim.elapsed - times[-1] >= .02 - 1e-10 or sim.finished:
                times.append(float(sim.elapsed))
                positions.append(list(observation["position"]))
        capture()
        # Modules may end early on a physical fall/impact. A broken endpoint must
        # not make the CLI loop forever.
        maximum_steps = math.ceil(sim.config.duration / sim.config.timestep) + 2
        for _ in range(maximum_steps):
            if sim.finished:
                break
            checked_step(sim)
            capture()
        if not sim.finished:
            raise RuntimeError("platform exceeded its declared experiment duration")
        summary = sim.summary()
        recorder.finish(summary, summary["public"])
    return summary, {"time": times, "position": positions}


def run(args):
    if args.counterfactual:
        raise ValueError("Use compare for platform healthy/fault probes; --counterfactual is for the wall track")
    sim = catalog.create(args.scenario, overrides(args))
    output = destination(args)
    print(f"Running {args.scenario}; artifacts: {output.resolve()}", flush=True)
    summary, _ = execute(sim, output, frames=args.frames, camera=args.camera, fps=args.fps)
    print(json.dumps(summary["public"], indent=2, allow_nan=False), flush=True)


def compare(args):
    """Freeze a nominal closed-loop probe result before running the changed world.

    These are matched controller/probe rollouts, not source-code repair or a claim
    that the controller knows an arbitrary future fault. Their commands can differ
    because the same nominal feedback law sees different observed states.
    """
    values = overrides(args)
    output = destination(args, "-comparison")
    output.mkdir(parents=True, exist_ok=False)
    nominal = catalog.create(args.scenario, values, healthy=True)
    print("Running nominal model before the damaged-world probe...", flush=True)
    baseline, baseline_trace = execute(nominal, output / "nominal", frames=args.frames,
                                       camera=args.camera, fps=args.fps)
    prediction = {"kind": "nominal closed-loop probe prediction", "summary": baseline["public"],
                  "trajectory": baseline_trace}
    encoded = (json.dumps(prediction, sort_keys=True, allow_nan=False) + "\n").encode()
    digest = hashlib.sha256(encoded).hexdigest()
    (output / "prediction.json").write_bytes(encoded)
    (output / "prediction.sha256").write_text(digest + "\n")
    print(f"Nominal result locked: {digest[:12]}. Running changed world...", flush=True)
    actual = catalog.create(args.scenario, values)
    observed, actual_trace = execute(actual, output / "actual", frames=args.frames,
                                    camera=args.camera, fps=args.fps)
    # Compare common simulation times only, never extrapolate an early fall.
    times = np.array(actual_trace["time"])
    ref_times = np.array(baseline_trace["time"])
    keep = (times >= ref_times[0]) & (times <= ref_times[-1])
    reference = np.asarray(baseline_trace["position"])
    interpolated = np.column_stack([np.interp(times[keep], ref_times, reference[:, i]) for i in range(3)])
    distance = np.linalg.norm(np.asarray(actual_trace["position"])[keep] - interpolated, axis=1)
    report = {"scenario": args.scenario, "prediction_sha256": digest,
              "comparison": "matched nominal feedback controller and probe; no automatic model repair",
              "nominal": baseline["public"], "actual": observed["public"],
              "common_time_end": float(times[keep][-1]),
              "position_rmse_m": float(np.sqrt(np.mean(distance ** 2))),
              "max_position_error_m": float(distance.max()),
              "nominal_config": catalog.config_dict(nominal),
              "actual_config": catalog.config_dict(actual)}
    (output / "comparison.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(json.dumps(report, indent=2, allow_nan=False), flush=True)
    print(f"Comparison artifacts: {output.resolve()}", flush=True)


def view_detail(sim, observation):
    """Show measured motion and completed events in the trusted operator HUD."""
    lines = ["Position (m): " + ", ".join(f"{v:.2f}" for v in observation["position"])]
    events = [event for event in sim.diagnostics()["events"]
              if event["event"] != "recovery_reposition"]
    if events:
        event = events[-1]
        label = event["event"].replace("_", " ").capitalize()
        lines.append(f"Event: {label} at {event['time']:.1f} s")
    if sim.finished:
        outcome = sim.summary()["public"]["outcome"].replace("_", " ")
        lines.append(f"Outcome: {outcome}")
    return "\n".join(lines)


def view(args):
    import mujoco.viewer
    if not math.isfinite(args.speedup) or args.speedup <= 0:
        raise ValueError("--speedup must be positive and finite")
    if sys.platform == "darwin" and not getattr(mujoco.viewer, "_MJPYTHON", None):
        raise ValueError("On macOS use .venv/bin/mjpython -m simulator view " + args.scenario)
    values = overrides(args)
    sim = catalog.create(args.scenario, values)
    camera = getattr(args, "camera", None) or ("side" if sim.config.probe == "showcase" else "chase")
    presentation = ViewControls(sim.model, scenario=args.scenario, config=sim.config,
                                speedup=args.speedup, camera=camera,
                                distance=float(getattr(sim, "camera_distance", 6.0)))
    print("Space: play/pause/replay | R: repeat retaining damage | N: full replay | Esc: close", flush=True)
    keys: SimpleQueue[int] = SimpleQueue()
    # All platforms reset their existing model/data in place. Keep one viewer:
    # closing a macOS window does not immediately release mjpython's UI thread.
    native_ui = getattr(args, "native_ui", False)
    with mujoco.viewer.launch_passive(sim.model, sim.data, key_callback=keys.put,
                                     show_left_ui=native_ui, show_right_ui=native_ui) as viewer:
        with viewer.lock():
            presentation.apply_camera(viewer, sim.data.xpos[sim.focus_body])
        paused = not getattr(args, "autoplay", False)
        announced = False
        playback_origin = sim.elapsed
        deadline = time.monotonic()
        last_sync = 0.0
        print("Ready. Press Space in the viewer to play the complete scenario.", flush=True)
        while viewer.is_running():
            while True:
                try:
                    key = keys.get_nowait()
                except Empty:
                    break
                if key == 32:
                    if sim.finished:
                        with viewer.lock():
                            sim.reset_full()
                        paused = announced = False
                        playback_origin = sim.elapsed
                        print("Replaying the complete scenario from the beginning.", flush=True)
                    else:
                        paused = not paused
                        print("Paused. Space resumes." if paused else "Playing scenario.", flush=True)
                    deadline = time.monotonic()
                elif key in (ord("R"), ord("r"), ord("N"), ord("n")):
                    with viewer.lock():
                        if key in (ord("R"), ord("r")):
                            sim.reset_trial()
                        else:
                            sim.reset_full()
                    paused = announced = False
                    playback_origin = sim.elapsed
                    deadline = time.monotonic()
                elif key == 256:
                    viewer.close()
                    break
                elif presentation.handle_key(key, viewer, sim.data.xpos[sim.focus_body]):
                    deadline = time.monotonic()
            if not viewer.is_running():
                break
            now = time.monotonic()
            if not paused and not sim.finished and now >= deadline:
                with viewer.lock():
                    # A slow render can leave several physics steps due. Advance
                    # them together; cap each batch so input still gets serviced.
                    for _ in range(64):
                        if sim.finished or now < deadline:
                            break
                        checked_step(sim)
                        deadline += sim.config.timestep / presentation.speedup
                if now - deadline > .25:
                    deadline = now
            if sim.finished and not announced:
                print(json.dumps(sim.summary()["public"], allow_nan=False), flush=True)
                print("Scenario complete. Space or N replays from the beginning; R retains damage.", flush=True)
                announced = True
            if now - last_sync >= 1 / 60:
                observation = sim.observe()
                phase_time = observation.get("phase_time", observation.get("trial_time", sim.elapsed - playback_origin))
                phase = observation["phase"]
                if args.scenario == "quadruped_demo" and phase == "probe":
                    phase = ("fall aftermath" if sim.summary()["public"]["outcome"] == "fell"
                             else "walking probe")
                elif args.scenario == "warehouse_demo" and phase == "drive":
                    command = observation["command"]
                    phase = "turn with cargo" if command["left"] != command["right"] else "drive with cargo"
                elif phase == "controlled_probe":
                    phase = "steering and braking inspection"
                presentation.update(viewer, position=observation["position"], elapsed=phase_time,
                                    duration=sim.config.duration, phase=phase,
                                    paused=paused, finished=sim.finished,
                                    detail=view_detail(sim, observation))
                viewer.sync()
                # Text upload/rendering can block until the next display frame.
                # Start the refresh interval after it completes, so the next loop
                # can advance physics instead of immediately uploading text again.
                last_sync = time.monotonic()
            time.sleep(.01 if paused or sim.finished else max(0, min(deadline - time.monotonic(), .005)))


def export(args):
    sim = catalog.create(args.scenario, healthy=True)
    output = args.output
    output.mkdir(parents=True, exist_ok=False)
    mujoco.mj_saveLastXML(str(output / "model.xml"), sim.model)
    observation = sim.observe()
    (output / "observation-example.json").write_text(json.dumps(observation, indent=2, allow_nan=False) + "\n")
    (output / "README.md").write_text(
        "# Nominal platform asset\n\n"
        "Self-contained MuJoCo model, compiled with healthy parameters. No private fault rules "
        "are included. The example observation shows the public sensor fields.\n\n"
        "The asset needs an external controller; loading MJCF alone does not reproduce the "
        "nominal probe controller. This is a synthetic mechanics asset, not a repaired twin.\n"
    )
    print(f"Nominal platform export: {output.resolve()}")
