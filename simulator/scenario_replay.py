"""Record the dog and delivery-drone tasks once, then replay their saved evidence.

This command runs no model/API calls. Public RGB/telemetry remain separate from
private fixture configuration and from optional impact presentation effects.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import inspect
import json
import math
from pathlib import Path
import re
from uuid import uuid4

import mujoco
from PIL import Image

from .platforms import catalog
from .platforms.operator import checked_step
from .recording import Recorder, _json
from .view_controls import setting_overrides


SCENARIOS = {name: catalog.REPLAY_TASKS[name]
             for name in ("quadruped_gait_failure", "drone_delivery_imbalance")}


FAILURE_EVENTS = {"fall", "fell", "body_contact", "support_loss", "lost_balance", "stumble", "impact", "crash"}


def _write_json(path: Path, value) -> None:
    with path.open("x", encoding="utf-8") as stream:
        stream.write(_json(value) + "\n")


def _events(sim, origin: float) -> list[dict]:
    """Only explicitly public observations can become replay bookmarks."""
    source: list[dict] = getattr(sim, "public_events", lambda: [])()
    events = []
    for event in source:
        if not isinstance(event, dict) or not isinstance(event.get("event"), str):
            raise ValueError("Public events require an event name")
        at = event.get("time")
        if isinstance(at, bool) or not isinstance(at, (int, float)) or not math.isfinite(at):
            raise ValueError("Public events require a finite simulation time")
        relative = float(at) - origin
        if relative < -1e-8 or relative > float(sim.elapsed) - origin + 1e-8:
            raise ValueError("Public event lies outside the recorded trial")
        events.append({**event, "time": max(0., relative),
                       "label": event.get("label", event["event"].replace("_", " ").capitalize()),
                       "is_failure": bool(event.get("is_failure", event["event"] in FAILURE_EVENTS))})
    return sorted(events, key=lambda event: event["time"])


def _task_text(scenario: str) -> str:
    objective = SCENARIOS[scenario]["objective"]
    return f"""# Recorded control task

{objective}

Inspect the unannotated RGB sequences in evidence/ and timestamped observations
in observations.jsonl. The manifest pairs camera views at the same simulation
time. Units are SI; positions use the world frame, with z upward. Commands and
phase labels describe observable task history. The fixture remains unchanged
between controller attempts. Playback only reads this record.

Use measured motion to propose a hypothesis, select a diagnostic intervention,
change your controller and test it on a fresh copy of the same task. Record the
source of the change and evaluate the complete mission. A different prediction
or a fault-free reference is not evidence that a controller solved this task.

The model/control interface is documented in CONTROL_INTERFACE.md beside this task. The
host must expose only public evidence and the neutral controller interface when
integrating GPT-6; private/ contains evaluator data and must not be supplied.

This recording contains no GPT-6-authored correction or claimed successful repair.
"""


def export_player(public: Path, manifest: dict, dist: Path | None = None) -> bool:
    """Inline the existing React player for file:// playback with no requests.

    A frontend build is optional for physics recording. The dashboard can still
    display the immutable manifest if no static build is available yet.
    """
    dist = dist or Path(__file__).resolve().parents[1] / "frontend/dist"
    index = dist / "index.html"
    if not index.is_file():
        return False
    html = index.read_text()

    def asset(url: str) -> str:
        path = dist / url.lstrip("/")
        if not path.resolve().is_relative_to(dist.resolve()):
            raise ValueError("Frontend build contains an invalid asset path")
        return path.read_text()

    def script(match):
        code = re.sub(r"</script", r"<\\/script", asset(match[1]), flags=re.IGNORECASE)
        return '<script type="module">' + code + '</script>'

    html = re.sub(r'<script\b[^>]*\bsrc="([^"]+)"[^>]*></script>', script, html)
    html = re.sub(r'<link\b[^>]*\bhref="([^"]+\.css)"[^>]*>',
                  lambda match: '<style>' + asset(match[1]) + '</style>', html)
    encoded = _json(manifest).replace("<", "\\u003c")
    html = html.replace('<head>', '<head><script>window.__SCENARIO_REPLAY__=' + encoded + ';</script>', 1)
    with (public / "replay.html").open("x", encoding="utf-8") as stream:
        stream.write(html)
    return True


def record_scenario(scenario: str, output: Path | None = None, *, overrides: dict | None = None,
                    cameras: tuple[str, ...] = ("side", "overview"), fps: int = 30,
                    width: int = 960, height: int = 540, effects: bool = True,
                    controller_file: Path | None = None, sim=None, renderer_factory=None) -> dict:
    """Publish a manifest only after a complete, warning-free physical run.

    Frames, including all camera views, are captured before the next physics
    step. ``sim``/``renderer_factory`` allow contract tests with controlled clocks;
    ordinary CLI callers always use the registered physical platform.
    """
    if scenario not in SCENARIOS:
        raise ValueError(f"Choose one of {', '.join(SCENARIOS)}")
    if controller_file is not None:
        if scenario != "quadruped_gait_failure" or sim is not None:
            raise ValueError("--controller requires a fresh quadruped_gait_failure task")
        from .dog_task import load_controller
        if overrides and "controller_parameters" in overrides:
            raise ValueError("Choose --controller or controller_parameters, not both")
        overrides = {**(overrides or {}), "controller_parameters": load_controller(controller_file)}
    if scenario in {"car_steering_drift", "car_auto_brake_failure"}:
        # Cars retain full conditioning history and also export a file:// player.
        from .failure_replays import record_failure
        car = sim if sim is not None else catalog.create(scenario, overrides)
        run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "-" + uuid4().hex[:8]
        destination = output or Path("runs/scenario-replays") / scenario / run_id
        return record_failure(car, scenario, destination, cameras=cameras, fps=fps,
                              width=width, height=height)
    for name, value, low, high in (("fps", fps, 1, 60), ("width", width, 160, 1920),
                                   ("height", height, 90, 1080)):
        if isinstance(value, bool) or not isinstance(value, int) or not low <= value <= high:
            raise ValueError(f"{name} must be an integer in [{low}, {high}]")
    if not isinstance(effects, bool):
        raise ValueError("effects must be boolean")
    if not cameras or len(cameras) != len(set(cameras)) or any(camera not in {"side", "overview", "chase"}
                                                             for camera in cameras):
        raise ValueError("Choose unique side, overview or chase cameras")
    sim = sim if sim is not None else catalog.create(scenario, overrides)
    for camera in cameras:
        if mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_CAMERA, camera) < 0:
            raise ValueError(f"Camera {camera} is unavailable")
    if sim.finished:
        raise ValueError("Recording requires a fresh, unfinished simulation")
    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "-" + uuid4().hex[:8]
    output = Path(output) if output is not None else Path("runs/scenario-replays") / scenario / run_id
    output.mkdir(parents=True, exist_ok=False)
    public = output / "public"
    private = output / "private"
    public.mkdir()
    private.mkdir()
    (public / "frames").mkdir()
    (public / "evidence").mkdir()
    config = catalog.config_dict(sim)
    initial = {"config": config, "qpos": sim.data.qpos.copy(), "qvel": sim.data.qvel.copy(),
               "mujoco": mujoco.__version__}
    _write_json(private / "initial_state.json", initial)
    mujoco.mj_saveLastXML(str(private / "model.xml"), sim.model)
    controller_settings = {key: config[key] for key in ("coordination_defect", "delivery_controller", "controller_parameters") if key in config}
    if hasattr(sim, "effective_controller_parameters"):
        controller_settings["controller_parameters"] = sim.effective_controller_parameters
    fixture = {**initial, "config": {key: value for key, value in config.items() if key not in controller_settings}}
    physics_hash = hashlib.sha256((private / "model.xml").read_bytes() + _json(fixture).encode()).hexdigest()
    modules = {Path(inspect.getfile(type(sim))), Path(inspect.getfile(catalog.module_for(scenario)))}
    if scenario == "quadruped_gait_failure" and hasattr(sim, "effective_controller_parameters"):
        from .platforms import quadruped_controller
        modules.add(Path(inspect.getfile(quadruped_controller)))
    source_hash = hashlib.sha256(b"".join(path.read_bytes() for path in sorted(modules))).hexdigest()
    controller_hash = hashlib.sha256(source_hash.encode() + _json(controller_settings).encode()).hexdigest()
    origin = float(sim.elapsed)
    frames: list[dict] = []
    samples: list[dict] = []
    next_tick = 0
    renderer_factory = renderer_factory or mujoco.Renderer
    sim.model.vis.global_.offwidth = max(width, sim.model.vis.global_.offwidth)
    sim.model.vis.global_.offheight = max(height, sim.model.vis.global_.offheight)
    try:
        with Recorder(output, sim.model) as recorder, renderer_factory(sim.model, height=height, width=width) as renderer:
            def capture(final=False):
                nonlocal next_tick
                observation = sim.observe()
                recorder.record(sim.data, observation, sim.diagnostics())
                at = float(sim.elapsed) - origin
                if frames and at + 1e-9 < next_tick / fps and not final:
                    return
                if frames and at <= frames[-1]["t_s"] + 1e-10:
                    return
                frame = {"t_s": at, "views": {}, "evidence": {}}
                index = len(frames)
                for camera in cameras:
                    renderer.update_scene(sim.data, camera=camera)
                    raw = renderer.render()
                    evidence = f"evidence/{camera}_{index:06d}.jpg"
                    Image.fromarray(raw).save(public / evidence, quality=88)
                    filename = f"frames/{camera}_{index:06d}.jpg"
                    if effects and hasattr(sim, "decorate_scene"):
                        sim.decorate_scene(renderer.scene)
                        Image.fromarray(renderer.render()).save(public / filename, quality=88)
                    else:
                        # The two paths carry explicit provenance even when the
                        # presentation has no visual effects.
                        (public / filename).write_bytes((public / evidence).read_bytes())
                    frame["views"][camera] = filename
                    frame["evidence"][camera] = evidence
                frames.append(frame)
                samples.append({**observation, "t_s": at})
                next_tick = math.floor((at + 1e-9) * fps) + 1

            capture()
            maximum_steps = math.ceil(sim.config.duration / sim.config.timestep) + 2
            for _ in range(maximum_steps):
                if sim.finished:
                    break
                checked_step(sim)
                capture(final=sim.finished)
            if not sim.finished:
                raise RuntimeError("Scenario exceeded its declared duration")
            summary = sim.summary()
            events = _events(sim, origin)
            recorder.finish(summary, summary["public"])
        provenance = "original_attempt"
        if controller_file is not None or config.get("controller_parameters") is not None:
            provenance = "candidate_attempt"
        elif config.get("delivery_controller") == "feasibility" or config.get("coordination_defect") is False:
            provenance = "developer_control"
        elif config.get("fault") in {"healthy", "none"}:
            provenance = "healthy_reference"
        manifest = {"schema_version": 1, "scenario_id": scenario, "run_id": run_id,
                    **SCENARIOS[scenario], "provenance": provenance,
                    "duration_s": float(sim.elapsed) - origin, "fps": fps,
                    "width": width, "height": height,
                    "cameras": [{"id": camera, "label": camera.capitalize()} for camera in cameras],
                    "frames": frames, "samples": samples, "events": events, "summary": summary["public"],
                    "source_sha256": source_hash, "physics_sha256": physics_hash,
                    "controller_sha256": controller_hash,
                    "effects": {"enabled": effects and hasattr(sim, "decorate_scene"),
                                "description": "Impact flash and smoke are presentation only; evidence frames are unannotated."},
                    "repair_status": "not_run"}
        if hasattr(sim, "effective_controller_parameters"):
            _write_json(public / "controller.json", sim.effective_controller_parameters)
            controller_source = Path(inspect.getfile(quadruped_controller))
            (public / "controller_source.py").write_text(controller_source.read_text(), encoding="utf-8")
            contract = Path(__file__).resolve().parents[1] / "contracts/QUADRUPED_CONTROLLER.md"
            (public / "CONTROLLER.md").write_text(contract.read_text(), encoding="utf-8")
            manifest["controller"] = {"file": "controller.json", "source": "controller_source.py",
                                      "contract": "CONTROLLER.md", "parameters": sim.effective_controller_parameters}
            if controller_file is not None:
                manifest["controller_artifact_sha256"] = hashlib.sha256(Path(controller_file).read_bytes()).hexdigest()
        (public / "task.md").write_text(_task_text(scenario), encoding="utf-8")
        interface = Path(__file__).with_name("public") / "CONTROL_TASKS.md"
        (public / "CONTROL_INTERFACE.md").write_text(interface.read_text(), encoding="utf-8")
        _write_json(public / "events.json", events)
        manifest["standalone_player"] = export_player(public, manifest)
        _write_json(public / "manifest.json", manifest)
        print(f"Saved replay: {output.resolve()}", flush=True)
        return manifest
    except Exception:
        # Keep partial evidence for diagnostics. Absence of manifest.json makes
        # it ineligible for the dashboard's completed-recording list.
        _write_json(private / "recording_failed.json", {"status": "incomplete", "frames": len(frames)})
        raise


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenario", choices=SCENARIOS)
    parser.add_argument("--output", type=Path, help="new directory; default publishes to the dashboard replay library")
    parser.add_argument("--set", dest="settings", action="append", metavar="NAME=VALUE")
    parser.add_argument("--controller", type=Path, help="dog gait controller JSON; leaves task and physics fixed")
    parser.add_argument("--camera", dest="cameras", choices=("side", "overview", "chase"), action="append")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=540)
    parser.add_argument("--no-effects", action="store_true")
    args = parser.parse_args(argv)
    result = record_scenario(args.scenario, args.output, overrides=setting_overrides(args.settings),
                             controller_file=args.controller,
                             cameras=tuple(args.cameras or ("side", "overview")), fps=args.fps,
                             width=args.width, height=args.height, effects=not args.no_effects)
    print(json.dumps({"scenario": args.scenario, "frames": len(result["frames"]),
                      "duration_s": result["duration_s"], "summary": result["summary"]}, indent=2))


if __name__ == "__main__":
    main()
