"""Unannotated camera evidence and telemetry from real, complete control tasks."""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
from uuid import uuid4

import mujoco
import numpy as np
from PIL import Image

from .platform_story import atomic_json


def numeric_public(value):
    if isinstance(value, dict):
        return {str(key): numeric_public(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, np.ndarray)):
        return [numeric_public(item) for item in value]
    if isinstance(value, (float, np.floating)):
        if not math.isfinite(value):
            raise ValueError("Non-finite task observation")
        return round(float(value), 7)
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.bool_):
        return bool(value)
    if value is None or isinstance(value, (bool, int, str)):
        return value
    raise ValueError("Unsupported task observation")


def record_task(adapter, candidate, workdir, *, frames=True, kind="observed"):
    candidate = adapter.validate_candidate(candidate)
    capabilities = adapter.capabilities()
    workdir = Path(workdir)
    identifier = "run_" + uuid4().hex[:16]
    directory = workdir / identifier
    directory.mkdir(parents=True, exist_ok=False)
    sim = adapter.create_sim(deepcopy(candidate))
    cameras = tuple(adapter.cameras)
    observations, images = [], {camera: [] for camera in cameras}
    renderer = None
    origin = float(adapter.observe(sim).get("time", 0))
    source_hash = hashlib.sha256(json.dumps(candidate, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
    next_observation = next_frame = next_secondary = 0.0
    try:
        if frames:
            (directory / "frames").mkdir()
            sim.model.vis.global_.offwidth = max(800, sim.model.vis.global_.offwidth)
            sim.model.vis.global_.offheight = max(450, sim.model.vis.global_.offheight)
            renderer = mujoco.Renderer(sim.model, width=800, height=450)

        def capture(final=False):
            nonlocal next_observation, next_frame, next_secondary
            elapsed = float(sim.elapsed if hasattr(sim, "elapsed") else adapter.observe(sim)["time"]) - origin
            if not final and elapsed + 1e-8 < next_observation and (renderer is None or elapsed + 1e-8 < next_frame):
                return
            observed = numeric_public(adapter.observe(sim))
            t = round(float(observed.get("time", 0)) - origin, 7)
            if final or t + 1e-8 >= next_observation:
                if not observations or observations[-1]["t_s"] != t:
                    observations.append({**observed, "t_s": t})
                next_observation = t + .02
            if renderer is not None and (final or t + 1e-8 >= next_frame):
                for index, camera in enumerate(cameras):
                    if index and not final and t + 1e-8 < next_secondary:
                        continue
                    if images[camera] and images[camera][-1]["t_s"] == t:
                        continue
                    renderer.update_scene(sim.data, camera=camera)
                    filename = f"{camera}_{len(images[camera]):06d}.jpg"
                    relative = Path(identifier) / "frames" / filename
                    Image.fromarray(renderer.render()).save(workdir / relative, quality=85)
                    images[camera].append({"t_s": t, "file": relative.as_posix(), "camera": camera})
                next_frame = t + .1
                if t + 1e-8 >= next_secondary:
                    next_secondary = t + .5

        capture()
        limit = math.ceil(adapter.duration_s / float(sim.model.opt.timestep)) + 10
        for _ in range(limit):
            if sim.finished:
                break
            sim.step()
            capture()
        if not sim.finished:
            raise RuntimeError("Task exceeded its fixed simulation step budget")
        capture(final=True)
        outcome = numeric_public(adapter.outcome(sim))
        summary = outcome.get("summary", {})
        if not isinstance(summary, dict):
            raise ValueError("Task summary must be a mapping")
        summary = {**summary, "task_complete": outcome.get("goal_achieved") is True}
        events = numeric_public(adapter.public_events(sim))
        record = {"id": identifier, "kind": kind, "platform": capabilities["platform"],
                  "probe": capabilities["scenario"], "duration_s": adapter.duration_s,
                  "actual_duration_s": observations[-1]["t_s"], "summary": summary,
                  "goal_achieved": outcome.get("goal_achieved") is True,
                  "partial_success": outcome.get("partial_success") is True,
                  "source_sha256": source_hash, "candidate": candidate,
                  "observations": observations, "events": events,
                  "frames": images[cameras[0]], "evidence_frames": images,
                  "provenance": "full physical task with unchanged world and goal; recorded controller parameters",
                  "camera_evidence": "unannotated RGB from the physical state"}
        atomic_json(directory / "record.json", record)
        return record
    finally:
        if renderer is not None:
            renderer.close()
