"""Timed warehouse/drone interventions and locked nominal trajectory comparisons.

Trusted-operator experiments, not an inference agent. Run with
``python -m simulator.lab --help``. The normal simulator CLI supplies the viewer.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
from importlib import import_module
import json
import math
from pathlib import Path
from typing import Any
from uuid import uuid4

import mujoco
import numpy as np

from .recording import Recorder
from .platforms import catalog
from .view_controls import setting_overrides


PLATFORMS = ("warehouse", "drone")


def create(scenario: str, overrides: dict | None = None, *, healthy: bool = False):
    platform = scenario.split("_", 1)[0]
    if platform not in PLATFORMS:
        raise ValueError(f"Expected a warehouse_* or drone_* preset, got {scenario!r}")
    return catalog.create(scenario, overrides, healthy=healthy)


def load_controls(path: Path | None, duration: float) -> list[tuple[float, dict]] | None:
    """Commands start at t=0 and hold until the next timestamp, in trial seconds."""
    if path is None:
        return None
    rows = json.loads(path.read_text())
    if not isinstance(rows, list) or not rows:
        raise ValueError("--controls must contain a nonempty list of [time, command_object] pairs")
    result: list[tuple[float, dict]] = []
    previous = -1.0
    for row in rows:
        if not isinstance(row, list) or len(row) != 2:
            raise ValueError("Each control entry must be [time, command_object]")
        when, command = row
        if (isinstance(when, bool) or not isinstance(when, (int, float))
                or not math.isfinite(when) or not previous < when < duration):
            raise ValueError("Control times must be finite, strictly increasing and inside the trial")
        if not result and when != 0:
            raise ValueError("The first control must start at time 0")
        if not isinstance(command, dict) or not command:
            raise ValueError("Each command must be a nonempty object")
        # Python's JSON reader accepts NaN/Infinity; the intervention format does not.
        json.dumps(command, allow_nan=False)
        result.append((float(when), command))
        previous = when
    return result


def record_trial(sim, output: Path, *, controls=None, frames=False, camera="chase", fps=30) -> dict:
    origin = sim.elapsed
    index = 0
    with Recorder(output, sim.model, frames=frames, camera=camera, fps=fps) as recorder:
        recorder.record(sim.data, sim.observe(), sim.diagnostics())
        maximum_steps = math.ceil(sim.config.duration / sim.config.timestep) + 2
        for _ in range(maximum_steps):
            if sim.finished:
                break
            command = None
            if controls is not None:
                trial_time = sim.elapsed - origin
                while index + 1 < len(controls) and controls[index + 1][0] <= trial_time + 1e-10:
                    index += 1
                command = controls[index][1]
            before = sim.elapsed
            sim.step(command)
            if not sim.elapsed > before:
                raise RuntimeError("Simulation clock failed to advance")
            if (not np.isfinite(sim.data.qpos).all() or not np.isfinite(sim.data.qvel).all()
                    or sim.data.warning.number.any()):
                raise RuntimeError("Invalid numerical simulation; inspect partial private diagnostics")
            recorder.record(sim.data, sim.observe(), sim.diagnostics())
        if not sim.finished:
            raise RuntimeError("Simulation exceeded its declared duration")
        summary = {**sim.summary(), "interventions": controls}
        recorder.finish(summary, summary["public"])
    return summary


def _trajectory(output: Path) -> dict:
    rows = [json.loads(line) for line in (output / "public/observations.jsonl").read_text().splitlines()]
    return {"time": [row["time"] for row in rows], "position": [row["position"] for row in rows]}


def compare(scenario: str, overrides: dict, output: Path, *, controls=None,
            frames=False, camera="chase", fps=30) -> dict:
    # Validate the supplied config through the healthy constructor before writing.
    nominal = create(scenario, overrides, healthy=True)
    output.mkdir(parents=True, exist_ok=False)
    options = {"controls": controls, "frames": frames, "camera": camera, "fps": fps}
    baseline = record_trial(nominal, output / "nominal", **options)
    prediction = {"kind": "fixed command schedule" if controls else "nominal feedback probe",
                  "public": baseline["public"], "trajectory": _trajectory(output / "nominal")}
    encoded = (json.dumps(prediction, sort_keys=True, allow_nan=False) + "\n").encode()
    digest = hashlib.sha256(encoded).hexdigest()
    (output / "prediction.json").write_bytes(encoded)
    (output / "prediction.sha256").write_text(digest + "\n")
    actual = create(scenario, overrides)
    observed = record_trial(actual, output / "actual", **options)
    actual_trace = _trajectory(output / "actual")
    reference = prediction["trajectory"]
    times = np.asarray(actual_trace["time"])
    reference_times = np.asarray(reference["time"])
    common = (times >= reference_times[0]) & (times <= reference_times[-1])
    if not common.any():
        raise RuntimeError("No common recorded timestamps to compare")
    positions = np.asarray(reference["position"])
    interpolated = np.column_stack([
        np.interp(times[common], reference_times, positions[:, axis]) for axis in range(3)
    ])
    residuals = np.linalg.norm(np.asarray(actual_trace["position"])[common] - interpolated, axis=1)
    report = {
        "scenario": scenario, "prediction_sha256": digest, "comparison": prediction["kind"],
        "nominal": baseline["public"], "actual": observed["public"],
        "common_samples": int(common.sum()), "common_time_end": float(times[common][-1]),
        "position_rmse_m": float(np.sqrt(np.mean(residuals ** 2))),
        "maximum_position_error_m": float(residuals.max()),
        "nominal_recorded_duration": float(reference_times[-1]),
        "actual_recorded_duration": float(times[-1]),
        "nominal_config": asdict(nominal.config), "actual_config": asdict(actual.config),
        "interventions": controls,
        "interpretation": "Measured nominal-model mismatch; diagnosis and model repair are external.",
    }
    (output / "comparison.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    return report


def export(scenario: str, output: Path) -> None:
    sim = create(scenario, healthy=True)
    output.mkdir(parents=True, exist_ok=False)
    mujoco.mj_saveLastXML(str(output / "model.xml"), sim.model)
    mujoco.MjModel.from_xml_path(str(output / "model.xml"))
    (output / "observation-example.json").write_text(
        json.dumps(sim.observe(), indent=2, allow_nan=False) + "\n")
    (output / "README.md").write_text(
        "# Nominal platform model\n\n"
        "Healthy, self-contained MJCF and public observation example. SI units; x forward, "
        "y left, z up. Cameras: overview, side, chase. An external controller is required.\n\n"
        "Warehouse: left/right commands in [-1,1] request normalized wheel torque. "
        "The nominal warehouse motor gear is 3 N m per unit command, as encoded in MJCF. "
        "Drone: rotor_commands contains four normalized commands in [0,1], ordered "
        "front-left, front-right, rear-right, rear-left. Each rotor's nominal thrust is "
        "6 N times its normalized motor state. Update that state with a first-order "
        "filter of time constant 0.035 s, initialized to 1.2*9.81/(4*6) for a level hover. "
        "Apply force through the corresponding MJCF site motor to include lever-arm "
        "and rotor reaction torque. The drone also has fixed world force -0.10*v "
        "and world moment -0.006*omega_world. Full probe/controller descriptions "
        "are in the operator's simulator/LAB.md handoff.\n\n"
        "Only public observations should be supplied to the inference agent. Enforce "
        "filesystem/process isolation outside this tool. Synthetic model; not validated "
        "against real hardware. No private fault rules or repaired model are included.\n"
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="action", required=True)
    sub.add_parser("list", help="list warehouse and drone presets")
    for action in ("run", "compare"):
        command = sub.add_parser(action, help="record a probe" if action == "run" else "lock and compare a probe")
        command.add_argument("scenario")
        command.add_argument("--config", type=Path, help="configuration JSON, bare or under config")
        command.add_argument("--controls", type=Path, help="JSON list of [time, command] interventions")
        command.add_argument("--probe", help="named platform maneuver")
        command.add_argument("--duration", type=float)
        command.add_argument("--timestep", type=float)
        command.add_argument("--fault-at", type=float)
        command.add_argument("--set", dest="settings", action="append", metavar="NAME=VALUE")
        command.add_argument("--output", type=Path, help="new directory")
        command.add_argument("--frames", action="store_true")
        command.add_argument("--camera", choices=("overview", "side", "chase"), default="chase")
        command.add_argument("--fps", type=int, default=30)
    command = sub.add_parser("export-baseline", help="export healthy MJCF without fault rules")
    command.add_argument("scenario")
    command.add_argument("output", type=Path)
    return parser


def main(argv=None) -> None:
    parser = _parser()
    args = parser.parse_args(argv)
    try:
        if args.action == "list":
            for platform in PLATFORMS:
                module = import_module(f"simulator.platforms.{platform}")
                for preset in module.PRESETS:
                    print(f"{preset:28} {module.DESCRIPTIONS[preset]}")
            return
        if args.action == "export-baseline":
            export(args.scenario, args.output)
            print(f"Nominal export: {args.output.resolve()}")
            return
        values: dict[str, Any] = {}
        if args.config:
            values = json.loads(args.config.read_text())
            if not isinstance(values, dict):
                raise ValueError("--config must contain an object")
            values = values.get("config", values)
            if not isinstance(values, dict):
                raise ValueError("config must be an object")
        values = dict(values)
        values.update(setting_overrides(args.settings))
        for name in ("probe", "duration", "timestep", "fault_at"):
            value = getattr(args, name)
            if value is not None:
                values[name] = value
        sim = create(args.scenario, values)
        controls = load_controls(args.controls, sim.config.duration)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        output = args.output or Path("runs") / f"{args.scenario}-{stamp}-{uuid4().hex[:8]}"
        options = {"controls": controls, "frames": args.frames, "camera": args.camera, "fps": args.fps}
        if args.action == "compare":
            report = compare(args.scenario, values, output, **options)
            result = {key: report[key] for key in ("position_rmse_m", "maximum_position_error_m",
                                                  "common_time_end", "prediction_sha256")}
        else:
            output.mkdir(parents=True, exist_ok=False)
            result = record_trial(sim, output, **options)["public"]
        print(json.dumps(result, indent=2, allow_nan=False))
        print(f"Artifacts: {output.resolve()}")
    except (OSError, ValueError, TypeError, RuntimeError) as error:
        parser.exit(2, f"error: {error}\n")
    except KeyboardInterrupt:
        parser.exit(130, "Interrupted; partial recording files were closed.\n")


if __name__ == "__main__":
    main()
