"""Record the F3/F4 physical failures and export an offline, immutable player.

No candidate controller or model calls run here. Public frames and measured
observations are separate from private configuration/plant diagnostics.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
from importlib import import_module
import math
from pathlib import Path
import os
from uuid import uuid4

import mujoco
import numpy as np

from .recording import Recorder, _json, _write_png
from .platforms.operator import checked_step
from .platforms.catalog import REPLAY_TASKS


SCENARIOS = {
    "car_steering_drift": "car_steering",
    "car_auto_brake_failure": "car_braking",
}
TEMPLATE = Path(__file__).parent / "assets/failure_replay.html"


def _write_json(path: Path, value) -> None:
    with path.open("x", encoding="utf-8") as stream:
        stream.write(_json(value) + "\n")


def _events(sim) -> list[dict]:
    events = getattr(sim, "public_events", [])
    return [dict(event) for event in (events() if callable(events) else events)]


def record_failure(sim, scenario: str, output: Path, *, fps: int = 24,
                   width: int = 960, height: int = 540,
                   cameras: tuple[str, ...] = ("chase", "overview")) -> dict:
    """Capture all camera images before advancing physics; never rerun a view."""
    for key, value in (("fps", fps), ("width", width), ("height", height)):
        if type(value) is not int or value <= 0:
            raise ValueError(f"{key} must be a positive integer")
    if not cameras or len(set(cameras)) != len(cameras):
        raise ValueError("At least one unique camera is required")
    for camera in cameras:
        if mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_CAMERA, camera) < 0:
            raise ValueError(f"Unknown camera: {camera}")
    if sim.finished:
        raise ValueError("Recording requires a fresh, unfinished simulation")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    public = output / "public"
    private = output / "private"
    public.mkdir()
    private.mkdir()
    (public / "frames").mkdir()

    module = import_module(type(sim).__module__)
    if module.__file__ is None:
        raise ValueError("Controller module must have a source file")
    source = Path(module.__file__)
    signature = mujoco.mjtState.mjSTATE_INTEGRATION
    state = np.empty(mujoco.mj_stateSize(sim.model, signature))
    mujoco.mj_getState(sim.model, sim.data, state, signature)
    np.save(private / "initial_state.npy", state)
    mujoco.mj_saveLastXML(str(private / "model.xml"), sim.model)
    config = asdict(sim.config)
    _write_json(private / "config.json", config)
    initial_diagnostics = sim.diagnostics()
    _write_json(private / "initial_diagnostics.json", initial_diagnostics)
    model_hash = hashlib.sha256((private / "model.xml").read_bytes()).hexdigest()
    initial_hash = hashlib.sha256(state.tobytes() + _json(config).encode()
                                  + _json(initial_diagnostics).encode() + model_hash.encode()).hexdigest()
    sources = {source, Path(__file__).parent / "platforms/car_damage.py",
               Path(__file__).parent / "runner.py", Path(__file__).parent / "private/thermal.py"}
    source_records = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in sorted(sources)}
    _write_json(private / "source_hashes.json", source_records)
    source_hash = hashlib.sha256(_json(source_records).encode()).hexdigest()
    history = getattr(sim, "preparation_history", [])
    _write_json(public / "preparation.json", history)
    start = float(sim.elapsed)
    frames: list[dict] = []
    frame_tick = 0
    sim.model.vis.global_.offwidth = max(width, sim.model.vis.global_.offwidth)
    sim.model.vis.global_.offheight = max(height, sim.model.vis.global_.offheight)

    with Recorder(output, sim.model) as recorder:
        # Conditioning is observable history, including explicit reset records.
        # Cached observations are sampled by the same 100 Hz recorder, without
        # pretending the approach pose is an image of an earlier conditioning step.
        preparations = getattr(sim, "preparation_observations", [])
        diagnostics = getattr(sim, "preparation_diagnostics", [])
        if len(preparations) != len(diagnostics):
            raise ValueError("Preparation observations/diagnostics must be paired")
        for observation, diagnostic in zip(preparations, diagnostics, strict=True):
            recorder.record(sim.data, observation, diagnostic)

        with mujoco.Renderer(sim.model, height=height, width=width) as renderer:
            def capture(*, final=False):
                nonlocal frame_tick
                observation = sim.observe()
                recorder.record(sim.data, observation, sim.diagnostics())
                elapsed = float(sim.elapsed) - start
                if elapsed + 1e-9 < frame_tick / fps and not final:
                    return
                if frames and abs(frames[-1]["time"] - float(sim.elapsed)) < 1e-9:
                    return
                files = {}
                for camera in cameras:
                    renderer.update_scene(sim.data, camera=camera)
                    pixels = renderer.render()
                    name = f"frames/{camera}_{len(frames):06d}.png"
                    _write_png(public / name, pixels)
                    files[camera] = name
                frames.append({"time": float(sim.elapsed), "offset": elapsed,
                               "files": files, "observation": observation,
                               "presentation": sim.presentation() if hasattr(sim, "presentation") else None,
                               "t_s": elapsed, "views": files, "evidence": files})
                frame_tick = math.floor((elapsed + 1e-9) * fps) + 1

            capture()
            maximum_steps = math.ceil(sim.config.duration / sim.config.timestep) + 2
            for _ in range(maximum_steps):
                if sim.finished:
                    break
                checked_step(sim)
                capture()
            if not sim.finished:
                raise RuntimeError("Failure scenario exceeded its declared duration")
            capture(final=True)
        summary = sim.summary()
        recorder.finish(summary, summary["public"])

    all_events = _events(sim)
    events = [{**event, "experiment_time": event["time"], "time": max(0., event["time"] - start),
               "label": event["event"].replace("_", " ").capitalize(),
               "is_failure": event["event"] in {"lane_exit", "roadside_contact", "barrier_contact"}}
              for event in all_events if event["time"] >= start - 1e-8]
    _write_json(public / "history_events.json", all_events)
    _write_json(public / "events.json", events)
    manifest = {
        "schema_version": 1, "run_id": output.name, "platform": "car",
        "scenario": scenario, "task": getattr(sim, "task_metadata", {}),
        "scenario_id": scenario, **REPLAY_TASKS.get(scenario, {}),
        "objective": getattr(sim, "task_metadata", {}).get("objective", REPLAY_TASKS.get(scenario, {}).get("objective", "")),
        "created_at": datetime.now(timezone.utc).isoformat(),
        "provenance": "healthy_reference" if config.get("fault") == "healthy" else "original_attempt",
        "execution": {"kind": "recorded_physics", "controller": "original_failure_controller",
                       "astra_repair": "not_run", "seed": 0,
                       "randomness": "No stochastic forces or presentation effects"},
        "engine": {"name": "MuJoCo", "version": mujoco.__version__,
                   "timestep": sim.config.timestep, "asset_sha256": model_hash},
        "initial_state_sha256": initial_hash, "controller_source_sha256": source_hash,
        "source_sha256": source_hash, "physics_sha256": initial_hash,
        "time_range": [start, float(sim.elapsed)], "fps": fps,
        "duration_s": float(sim.elapsed) - start,
        "width": width, "height": height,
        "cameras": [{"id": camera, "label": camera.capitalize()} for camera in cameras],
        "frames": frames, "events": events, "outcome": summary["public"],
        "samples": [{**frame["observation"], "t_s": frame["offset"]} for frame in frames],
        "summary": summary["public"], "repair_status": "not_run", "effects": {"enabled": False},
        "conclusion": sim.presentation() if hasattr(sim, "presentation") else None,
        "artifacts": {"observations": "observations.jsonl", "events": "events.json",
                      "preparation": "preparation.json", "summary": "summary.json"},
        "preparation": {"recorded_steps": len(preparations), "approach_start_time": start,
                        "frames": "Approach and aftermath only; conditioning has telemetry and reset history"},
    }
    (public / "task.md").write_text(
        "# Original car control attempt\n\n" + manifest.get("objective", "") + "\n\n"
        "Inspect the paired unannotated frames and observations.jsonl (SI units, x forward, y left, z up). "
        "Frame t_s/event time are relative to approach; observation time is absolute experiment time. "
        "preparation.json and history_events.json preserve imposed reset and conditioning boundaries. "
        "Diagnose the response, choose probes and change the controller before testing on the same physical setup. "
        "The original controller and neutral interface are documented in simulator/CAR_FAILURE_DEMOS.md. "
        "Private evaluator records are outside this public evidence bundle. GPT-6 repair has not run.\n"
    )
    _write_json(public / "manifest.json", manifest)
    # Inline data allows file:// playback without a server or network access.
    # Escaping '<' prevents even unexpected task text from closing the data tag.
    encoded = _json(manifest).replace("<", "\\u003c")
    html = TEMPLATE.read_text().replace("__REPLAY_MANIFEST__", encoded)
    with (public / "replay.html").open("x", encoding="utf-8") as stream:
        stream.write(html)
    return manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenario", choices=["all", *SCENARIOS], nargs="?", default="all")
    parser.add_argument("--output", type=Path, help="new artifact directory; existing runs are never overwritten")
    parser.add_argument("--fps", type=int, default=24)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=540)
    args = parser.parse_args(argv)
    output = args.output or Path("runs") / ("car-failures-" + datetime.now().strftime("%Y%m%d-%H%M%S"))
    names = list(SCENARIOS) if args.scenario == "all" else [args.scenario]
    try:
        if output.exists():
            raise FileExistsError(f"Choose a new directory; {output} already exists")
        output.mkdir(parents=True)
        links = []
        for name in names:
            print(f"Preparing and recording {name}…", flush=True)
            module = import_module(f"simulator.platforms.{SCENARIOS[name]}")
            sim = module.Simulation(module.Config(**module.PRESETS[name]))
            recording = (output / name if args.output else
                         Path("runs/scenario-replays") / name / (datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "-" + uuid4().hex[:8]))
            manifest = record_failure(sim, name, recording, fps=args.fps,
                                      width=args.width, height=args.height)
            page = os.path.relpath(recording / "public/replay.html", output)
            links.append(f'<li><a href="{page}">{name.replace("_", " ").title()}</a></li>')
            print(f"{manifest['outcome']}\nReplay: {(output / page).resolve()}", flush=True)
        (output / "index.html").write_text(
            '<!doctype html><html lang="en"><meta charset="utf-8"><title>RealityPatch car failures</title>'
            '<style>body{background:#0d1520;color:#e9eff7;font:18px system-ui;margin:8vw}'
            'a{color:#80ddd0}li{margin:24px 0}</style><h1>RealityPatch · Car failures</h1>'
            '<p>Original physical failures. GPT-6 repair: not run.</p><ul>' + "".join(links) + '</ul></html>'
        )
        print(f"Open {(output / 'index.html').resolve()}", flush=True)
    except (OSError, ValueError, RuntimeError) as error:
        parser.exit(2, f"error: {error}\n")


if __name__ == "__main__":
    main()
