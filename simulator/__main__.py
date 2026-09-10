"""Trusted-operator CLI for running, viewing, and exporting simulator experiments."""

import argparse
from dataclasses import replace
from datetime import datetime, timezone
import json
import math
from pathlib import Path
from queue import Empty, SimpleQueue
import sys
import time
from uuid import uuid4

import mujoco

from .config import Experiment
from .model import model_xml
from .recording import Recorder
from .runner import BRAKE_TORQUES, Simulator
from . import scenarios


def _physics_arguments(parser):
    parser.add_argument("scenario", help="private operator preset; use list to see names")
    parser.add_argument("--config", type=Path, help="JSON object overlaid on the selected preset")
    for flag, destination, help_text in (
        ("speed", "initial_speed", "initial speed in m/s"),
        ("brake", "brake", "brake pedal fraction [0,1]"),
        ("brake-at", "brake_at", "front-bumper x coordinate for braking, in metres"),
        ("duration", "duration", "maximum trial duration in seconds"),
        ("wall-x", "wall_x", "wall face x coordinate in metres"),
        ("timestep", "timestep", "physics timestep in seconds"),
        ("rest", "recovery", "recovery time after conditioning, in seconds"),
    ):
        parser.add_argument(f"--{flag}", dest=destination, type=float, help=help_text)
    parser.add_argument("--cycles", dest="warmup_cycles", type=int, help="conditioning cycles")
    parser.add_argument("--no-wall", action="store_true", help="measure an uncensored free stop")


def _parser():
    parser = argparse.ArgumentParser(prog="python -m simulator", description=__doc__)
    commands = parser.add_subparsers(dest="action", required=True)
    commands.add_parser("list", help="list available private operator scenarios")
    run = commands.add_parser("run", help="run deterministically and record public/private artifacts")
    _physics_arguments(run)
    run.add_argument("--output", type=Path, help="new recording directory (default: runs/<unique name>)")
    run.add_argument("--frames", action="store_true", help="render timestamped PNG frames")
    run.add_argument("--camera", default="overview", help="model camera: overview, side, or chase")
    run.add_argument("--fps", type=int, default=30, help="frame sample rate (default: 30)")
    run.add_argument("--counterfactual", action="store_true", help="also record a private wall-free replay")
    view = commands.add_parser("view", help="open the live MuJoCo viewer (macOS: use mjpython)")
    _physics_arguments(view)
    view.add_argument("--speedup", type=float, default=1.0, help="playback speed multiplier (default: 1)")
    export = commands.add_parser("export-baseline", help="export nominal MJCF and public sensor contract")
    export.add_argument("output", type=Path)
    return parser


def _config(args):
    values = scenarios.load(args.scenario).to_dict()
    if args.config:
        overlay = json.loads(args.config.read_text())
        if not isinstance(overlay, dict):
            raise ValueError("--config must contain a JSON object")
        # A saved operator manifest and a bare Experiment object are both accepted.
        overlay = overlay.get("config", overlay)
        if not isinstance(overlay, dict):
            raise ValueError("the config field must contain a JSON object")
        values.update(overlay)
    for name in ("initial_speed", "brake", "brake_at", "duration", "wall_x", "timestep",
                 "recovery", "warmup_cycles"):
        value = getattr(args, name)
        if value is not None:
            values[name] = value
    if args.no_wall:
        values["wall"] = False
    return Experiment.from_dict(values)


def _record(config, output, args):
    sim = Simulator(config)
    with Recorder(output, sim.model, frames=args.frames, camera=args.camera, fps=args.fps) as recorder:
        def capture(current, phase):
            recorder.record(current.data, current.observe(phase), current.diagnostics())

        summary = sim.run(capture)
        recorder.finish(summary, summary["public"])
    return summary


def _outcome(summary):
    if summary["collision"]:
        return f"wall contact at {summary['impact_speed']:.2f} m/s; stopping distance censored"
    if summary["stopped"]:
        if summary["stopping_distance"] is None:
            return "stationary; no braking-distance measurement"
        return f"stopped; braking distance {summary['stopping_distance']:.2f} m"
    return f"trial timed out at {summary['final_speed']:.2f} m/s; stopping distance censored"


def _run(args):
    config = _config(args)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output = args.output or Path("runs") / f"{args.scenario}-{stamp}-{uuid4().hex[:8]}"
    print(f"Running {args.scenario}; artifacts: {output.resolve()}", flush=True)
    summary = _record(config, output, args)
    print(_outcome(summary), flush=True)
    if args.counterfactual:
        paired_output = output / "counterfactual"
        paired = _record(replace(config, wall=False), paired_output, args)
        print(f"Evaluator-only wall-free replay: {_outcome(paired)}", flush=True)
        print(f"Counterfactual artifacts: {paired_output.resolve()}", flush=True)


def _view(args):
    # Importing the viewer here leaves headless run/export paths independent of GLFW.
    import mujoco.viewer

    if not math.isfinite(args.speedup) or args.speedup <= 0:
        raise ValueError("--speedup must be finite and positive")
    if sys.platform == "darwin" and not getattr(mujoco.viewer, "_MJPYTHON", None):
        raise ValueError("On macOS launch with .venv/bin/mjpython -m simulator view " + args.scenario)
    config = _config(args)
    print("Space: pause/resume | R: repeat trial retaining heat/faults | "
          "N: reset and replay full experiment | Esc: close", flush=True)
    while True:
        sim = Simulator(config)
        print("Preparing declared conditioning and recovery history...", flush=True)
        sim.prepare()
        print(f"Conditioning complete ({sim.elapsed:.2f} simulated seconds). Starting trial.", flush=True)
        keys = SimpleQueue()
        restart = False
        with mujoco.viewer.launch_passive(sim.model, sim.data, key_callback=keys.put) as viewer:
            with viewer.lock():
                viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
                viewer.cam.distance = 10
                viewer.cam.azimuth = 135
                viewer.cam.elevation = -25
                viewer.cam.lookat[:] = sim.data.xpos[sim.chassis]
            paused = False
            finished = False
            stop_ticks = 0
            last_sync = 0.0
            next_step = time.monotonic()
            while viewer.is_running():
                while True:
                    try:
                        key = keys.get_nowait()
                    except Empty:
                        break
                    if key == 32:
                        paused = not paused
                        next_step = time.monotonic()
                    elif key in (ord("R"), ord("r")):
                        with viewer.lock():
                            sim.reset_trial()
                        paused = finished = False
                        stop_ticks = 0
                        next_step = time.monotonic()
                        print("Trial reset; temperature and completed faults retained.", flush=True)
                    elif key in (ord("N"), ord("n")):
                        restart = True
                    elif key == 256:
                        viewer.close()
                if restart or not viewer.is_running():
                    break
                now = time.monotonic()
                if not paused and not finished and now >= next_step:
                    with viewer.lock():
                        throttle, brake = sim.command()
                        sim.step(throttle, brake)
                    stop_ticks = stop_ticks + 1 if sim.speed < 0.1 else 0
                    finished = sim.trial_complete(stop_ticks)
                    next_step += config.timestep / args.speedup
                    # Avoid an unbounded catch-up burst after a user/window stall.
                    if now - next_step > 0.25:
                        next_step = now
                    if finished:
                        label = "wall contact" if sim.collision else "stopped" if sim.speed < 0.1 else "timeout"
                        print(f"Trial complete: {label}; R repeats, N starts a full reset.", flush=True)
                if now - last_sync >= 1 / 60:
                    with viewer.lock():
                        viewer.cam.lookat[:] = sim.data.xpos[sim.chassis]
                    viewer.sync()
                    last_sync = now
                if paused or finished:
                    time.sleep(0.01)
                else:
                    time.sleep(max(0, min(next_step - time.monotonic(), 0.01)))
        if not restart:
            return


def _schema():
    vector = {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3}
    properties = {
        "time": {"type": "number", "minimum": 0, "description": "Monotonic experiment time in seconds"},
        "phase": {"type": "string", "description": "conditioning_N, recovery, or trial"},
        "phase_time": {"type": "number", "minimum": 0},
        "position": {**vector, "description": "World chassis position, metres"},
        "velocity": {**vector, "description": "World translational velocity, m/s"},
        "yaw": {"type": "number", "description": "World yaw, radians"},
        "yaw_rate": {"type": ["number", "null"],
                     "description": "World heading derivative, rad/s; null for vertical heading"},
        "wheel_speed": {"type": "array", "items": {"type": "number"}, "minItems": 4, "maxItems": 4,
                        "description": "FL, FR, RL, RR joint angular speeds, rad/s"},
        "throttle": {"type": "number", "minimum": 0, "maximum": 1},
        "brake": {"type": "number", "minimum": 0, "maximum": 1},
        "front_x": {"type": "number", "description": "Leading chassis collision-box x, metres"},
        "wall_contact": {"type": "boolean"},
        "lane_departure": {"type": "boolean"},
    }
    return {"$schema": "https://json-schema.org/draft/2020-12/schema", "type": "object",
            "additionalProperties": False, "required": list(properties), "properties": properties}


def _export(output):
    output.mkdir(parents=True, exist_ok=True)
    files = ("model.xml", "observations.schema.json", "README.md")
    if any((output / name).exists() for name in files):
        raise FileExistsError("Baseline export files already exist; choose a new output directory")
    xml = model_xml(Experiment())
    mujoco.MjModel.from_xml_string(xml)  # Validate the self-contained export before writing.
    (output / "model.xml").write_text(xml + "\n")
    (output / "observations.schema.json").write_text(json.dumps(_schema(), indent=2) + "\n")
    (output / "README.md").write_text(
        "# Nominal simulator baseline\n\n"
        "This is a self-contained synthetic car-and-track MJCF, plus the public observation schema. "
        "It is not a validated vehicle safety model. No private profiles or failure rules are included.\n\n"
        "Coordinates and units: x is forward, y is left, z is up; SI units; wheel order FL, FR, RL, RR. "
        "Named cameras are overview, side, chase. Default timestep is 0.002 s.\n\n"
        "The nominal controller applies 500 * throttle N m to each wheel motor. Disc braking uses "
        f"the spin_FL/spin_FR joint frictionloss capacities {BRAKE_TORQUES[0]:g} * brake N m "
        f"and spin_RL/spin_RR capacities {BRAKE_TORQUES[2]:g} * brake N m. "
        "MuJoCo's bounded friction constraint opposes rotation and holds "
        "at rest. The baseline scene alone does not execute pedal schedules. "
        "Settle contacts before motion and initialize wheel spin to vehicle speed / 0.34 m.\n\n"
        "Public observations are JSONL sampled at 100 Hz. Time is monotonic experiment time; "
        "phase_time resets on declared repositioning. Conditioning and recovery are observable "
        "history. PNG frames are indexed by time and relative file in frames.jsonl, normally at "
        "30 Hz. Frame times are actual physics times, within one physics step of the sampling grid.\n\n"
        "Public summary fields describe collision/impact speed, stop status, censoring, braking "
        "distance, final motion, wall clearance, yaw/lateral displacement, and lane departure. "
        "A collision or timeout has null stopping_distance; a stop requires speed below 0.1 m/s "
        "for 0.5 s. Wall-free paired replays are evaluator-only artifacts.\n\n"
        "A public directory is an export convention, not a security boundary. The integration "
        "must isolate the hidden simulator and private records using account/process/container "
        "permissions before exposing observations to an agent.\n"
    )
    print(f"Baseline export: {output.resolve()}")


def main(argv=None):
    parser = _parser()
    args = parser.parse_args(argv)
    try:
        if args.action == "list":
            for name in scenarios.names():
                print(f"{name:16} {scenarios.description(name)}")
        elif args.action == "run":
            _run(args)
        elif args.action == "view":
            _view(args)
        elif args.action == "export-baseline":
            _export(args.output)
    except (OSError, ValueError, TypeError) as error:
        parser.exit(2, f"error: {error}\n")
    except KeyboardInterrupt:
        parser.exit(130, "Interrupted; partial recording files were closed.\n")


if __name__ == "__main__":
    main()
