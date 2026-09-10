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
from .view_controls import ViewControls, setting_overrides, viewer_arguments
from .platforms import catalog as platforms
from .platforms import operator as platform_operator


def _physics_arguments(parser):
    parser.add_argument("scenario", help="private operator preset; use list to see names")
    parser.add_argument("--config", type=Path, help="JSON object overlaid on the selected preset")
    parser.add_argument("--controller", type=Path, help="dog gait controller JSON; changes control only")
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
    parser.add_argument("--probe", help="platform diagnostic probe (see platform documentation)")
    parser.add_argument("--fault-at", type=float, help="platform fault onset / impact arming time in seconds")
    parser.add_argument("--set", dest="settings", action="append", metavar="NAME=VALUE",
                        help="override a model setting (JSON values or unquoted text); repeat as needed")


def _parser():
    parser = argparse.ArgumentParser(prog="python -m simulator", description=__doc__)
    commands = parser.add_subparsers(dest="action", required=True)
    commands.add_parser("list", help="list available private operator scenarios")
    commands.add_parser("demos", help="show the four presentation demos and launch commands")
    run = commands.add_parser("run", help="run deterministically and record public/private artifacts")
    _physics_arguments(run)
    run.add_argument("--output", type=Path, help="new recording directory (default: runs/<unique name>)")
    run.add_argument("--frames", action="store_true", help="render timestamped PNG frames")
    run.add_argument("--camera", default="overview", help="model camera: overview, side, or chase")
    run.add_argument("--fps", type=int, default=30, help="frame sample rate (default: 30)")
    run.add_argument("--counterfactual", action="store_true", help="also record a private wall-free replay")
    compare = commands.add_parser("compare", help="compare car components or nominal and damaged platform probes")
    _physics_arguments(compare)
    compare.add_argument("--candidate", type=Path, help="editable wheel_v2 Python component")
    compare.add_argument("--developer-check", action="store_true", help="include a manually authored stateful check")
    compare.add_argument("--output", type=Path, help="new comparison directory")
    compare.add_argument("--frames", action="store_true", help="record PNG frames for each run")
    compare.add_argument("--camera", default="overview")
    compare.add_argument("--fps", type=int, default=30)
    view = commands.add_parser("view", help="open the live MuJoCo viewer (macOS: use mjpython)")
    _physics_arguments(view)
    view.add_argument("--speedup", type=float, default=1.0, help="playback speed multiplier (default: 1)")
    viewer_arguments(view)
    export = commands.add_parser("export-baseline", help="export nominal MJCF and public sensor contract")
    export.add_argument("output", type=Path)
    task = commands.add_parser("export-task", help="export editable Python and neutral wheel_v2 interface only")
    task.add_argument("output", type=Path)
    platform_export = commands.add_parser("export-platform", help="export healthy platform MJCF and sensor example")
    platform_export.add_argument("scenario")
    platform_export.add_argument("output", type=Path)
    return parser


def _config(args):
    if getattr(args, "controller", None) is not None:
        raise ValueError("--controller applies to quadruped_gait_failure")
    if args.probe is not None or args.fault_at is not None:
        raise ValueError("--probe and --fault-at apply to car-damage, quadruped, drone and warehouse platforms")
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
    values.update(setting_overrides(getattr(args, "settings", None)))
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
    if platforms.is_platform(args.scenario):
        return platform_operator.run(args)
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


def _compare(args):
    if platforms.is_platform(args.scenario):
        if args.candidate is not None or args.developer_check:
            raise ValueError("--candidate and --developer-check apply to the car wheel-actuator comparison")
        return platform_operator.compare(args)
    from .comparison import DEFAULT_CANDIDATE, compare
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output = args.output or Path("runs") / f"comparison-{args.scenario}-{stamp}-{uuid4().hex[:8]}"
    result = compare(_config(args), output, candidate=args.candidate or DEFAULT_CANDIDATE,
                     developer_check=args.developer_check, frames=args.frames,
                     camera=args.camera, fps=args.fps)
    print("Reference: " + _outcome(result["reference"]), flush=True)
    for name, prediction in result["predictions"].items():
        print(f"{name}: " + _outcome(prediction["summary"]), flush=True)
    print(f"Comparison chart and records: {output.resolve()}", flush=True)


def _view(args):
    if platforms.is_platform(args.scenario):
        return platform_operator.view(args)
    # Importing the viewer here leaves headless run/export paths independent of GLFW.
    import mujoco.viewer

    if not math.isfinite(args.speedup) or args.speedup <= 0:
        raise ValueError("--speedup must be finite and positive")
    if sys.platform == "darwin" and not getattr(mujoco.viewer, "_MJPYTHON", None):
        raise ValueError("On macOS launch with .venv/bin/mjpython -m simulator view " + args.scenario)
    config = _config(args)
    print("Space: play/pause/replay | R: repeat retaining heat/faults | "
          "N: full replay | Esc: close", flush=True)
    sim = Simulator(config)
    presentation = ViewControls(sim.model, scenario=args.scenario, config=config, speedup=args.speedup,
                                camera=getattr(args, "camera", None) or "chase", distance=10)
    print("Preparing declared conditioning and recovery history...", flush=True)
    sim.prepare()
    print(f"Ready after {sim.elapsed:.2f} simulated seconds of preparation. "
          "Press Space in the viewer to play.", flush=True)
    keys = SimpleQueue()
    native_ui = getattr(args, "native_ui", False)
    with mujoco.viewer.launch_passive(sim.model, sim.data, key_callback=keys.put,
                                     show_left_ui=native_ui, show_right_ui=native_ui) as viewer:
        with viewer.lock():
            presentation.apply_camera(viewer, sim.data.xpos[sim.chassis])
        paused = not getattr(args, "autoplay", False)
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
                    if finished:
                        print("Replaying the full experiment and its declared preparation...", flush=True)
                        with viewer.lock():
                            sim.reset_replay()
                        paused = finished = False
                        stop_ticks = 0
                    else:
                        paused = not paused
                        print("Paused. Space resumes." if paused else "Playing scenario.", flush=True)
                    next_step = time.monotonic()
                elif key in (ord("R"), ord("r"), ord("N"), ord("n")):
                    with viewer.lock():
                        if key in (ord("R"), ord("r")):
                            sim.reset_trial()
                        else:
                            sim.reset_replay()
                    paused = finished = False
                    stop_ticks = 0
                    next_step = time.monotonic()
                elif key == 256:
                    viewer.close()
                    break
                elif presentation.handle_key(key, viewer, sim.data.xpos[sim.chassis]):
                    next_step = time.monotonic()
            if not viewer.is_running():
                break
            now = time.monotonic()
            if not paused and not finished and now >= next_step:
                with viewer.lock():
                    # Catch up between display frames without starving input.
                    for _ in range(64):
                        if finished or now < next_step:
                            break
                        throttle, brake = sim.command()
                        sim.step(throttle, brake)
                        stop_ticks = stop_ticks + 1 if sim.speed < 0.1 else 0
                        finished = sim.trial_complete(stop_ticks)
                        next_step += config.timestep / presentation.speedup
                # Avoid an unbounded catch-up burst after a user/window stall.
                if now - next_step > 0.25:
                    next_step = now
                if finished:
                    label = "wall contact" if sim.collision else "stopped" if sim.speed < 0.1 else "timeout"
                    print(f"Trial complete: {label}; Space or N replays fully; R retains faults.", flush=True)
            if now - last_sync >= 1 / 60:
                phase = "wall contact" if sim.collision else "braking" if sim.last_command[1] else "approach"
                presentation.update(viewer, position=sim.data.xpos[sim.chassis], elapsed=sim.trial_time,
                                    duration=config.duration, phase=phase, paused=paused, finished=finished,
                                    detail=f"Speed: {sim.speed:.2f} m/s\nWheel/road contact physics")
                viewer.sync()
                last_sync = time.monotonic()
            time.sleep(.01 if paused or finished else max(0, min(next_step - time.monotonic(), .01)))


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
        if args.action == "demos":
            descriptions = platforms.entries()
            for name in ("drone_demo", "car_demo", "quadruped_demo", "warehouse_demo"):
                print(f"{name}: {descriptions[name]}")
                print(f"  .venv/bin/mjpython -m simulator view {name}")
            print("Space plays the full scenario; C changes camera; -/+ changes playback speed.")
            print("Use --set NAME=VALUE for model settings; see simulator/DEMO_GUIDE.md.")
        elif args.action == "list":
            for name in scenarios.names():
                print(f"{name:16} {scenarios.description(name)}")
            for name, description in platforms.entries().items():
                print(f"{name:28} {description}")
        elif args.action == "run":
            _run(args)
        elif args.action == "compare":
            _compare(args)
        elif args.action == "view":
            _view(args)
        elif args.action == "export-baseline":
            _export(args.output)
        elif args.action == "export-task":
            from .comparison import export_task
            export_task(args.output)
            print(f"Editable component package: {args.output.resolve()}")
        elif args.action == "export-platform":
            platform_operator.export(args)
    except (OSError, ValueError, TypeError, RuntimeError) as error:
        parser.exit(2, f"error: {error}\n")
    except KeyboardInterrupt:
        parser.exit(130, "Interrupted; partial recording files were closed.\n")


if __name__ == "__main__":
    main()
