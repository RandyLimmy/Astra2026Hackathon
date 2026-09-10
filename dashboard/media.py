"""Render completed evaluation rollouts with MuJoCo for the local dashboard.

The original recordings remain immutable. Each video sequence is a new rollout
of the saved commands and frozen source, published only if its outcome matches
the recorded evaluation. These images never enter an investigator's context.
"""

import argparse
from contextlib import nullcontext
import hashlib
import json
import math
from pathlib import Path
import shutil
from uuid import uuid4

import mujoco
from PIL import Image

from component_worker import WheelActuatorWorker
from investigation.physics import PhysicsService, _history
from simulator.runner import Simulator


TRACKS = ("reference", "candidate", "original")
CASES = ("reserved_1", "reserved_2", "reserved_3")
ROOT = Path(__file__).resolve().parents[1]


def _read(path):
    return json.loads(Path(path).read_text())


def outcomes_match(actual, expected):
    for key in ("stopped", "collision", "censored", "lane_departure"):
        if actual.get(key) != expected.get(key):
            return False
    for key in ("stopping_distance", "trial_duration", "impact_speed", "final_front_x", "final_speed"):
        a, b = actual.get(key), expected.get(key)
        if a is None or b is None:
            if a != b:
                return False
        elif not math.isclose(a, b, rel_tol=1e-8, abs_tol=1e-7):
            return False
    return True


def render_track(run_dir, case_id, track, *, fps=15, width=960, height=300, cache_dir=None):
    run_dir = Path(run_dir).resolve()
    if case_id not in CASES or track not in TRACKS:
        raise ValueError("Select a reserved case and a reference, candidate, or original track")
    if not 1 <= fps <= 30 or not 320 <= width <= 1280 or not 200 <= height <= 720:
        raise ValueError("Render dimensions or frame rate are out of bounds")
    if _read(run_dir / "metadata.json").get("status") != "completed":
        raise ValueError("Only a completed frozen evaluation can be rendered")
    evaluation = _read(run_dir / "evaluation/result.json")
    record_path = run_dir / "evaluation/cases" / case_id / f"{track}.json"
    record = _read(record_path)
    if "error" in record:
        raise ValueError("This track failed during evaluation and has no renderable rollout")
    source = None
    source_hash = None
    if track != "reference":
        source = run_dir / "evaluation" / ("frozen_candidate.py" if track == "candidate" else "original_candidate.py")
        source_hash = hashlib.sha256(source.read_bytes()).hexdigest()
        expected_hash = evaluation["source_sha256" if track == "candidate" else "original_source_sha256"]
        if source_hash != expected_hash or record.get("source_sha256") != source_hash:
            raise ValueError("Saved source does not match the frozen prediction")
    config, history = record["config"], record["preparation_history"]
    fingerprint_files = [ROOT / name for name in (
        "simulator/runner.py", "simulator/model.py", "simulator/config.py",
        "simulator/private/thermal.py", "investigation/physics.py", "dashboard/media.py")]
    fingerprint_files.extend(sorted((ROOT / "simulator/assets").glob("*.xml")))
    identity = {"config": config, "history": history, "source_sha256": source_hash,
                "reference": track == "reference", "expected_summary": record["summary"],
                "fps": fps, "width": width, "height": height, "mujoco": mujoco.__version__,
                "files": {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in fingerprint_files}}
    digest = hashlib.sha256(json.dumps(identity, sort_keys=True).encode()).hexdigest()
    target = run_dir / "dashboard_media" / case_id / track
    if (target / "manifest.json").is_file():
        manifest = _read(target / "manifest.json")
        if manifest.get("render_sha256") == digest:
            return manifest
        raise ValueError("Existing media belongs to a different renderer or recording")
    cache_dir = Path(cache_dir) if cache_dir else ROOT / "runs/dashboard-media-cache"
    cache = cache_dir / digest
    if (cache / "manifest.json").is_file():
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copytree(cache, target)
        return _read(target / "manifest.json")
    temporary = cache_dir / f".render-{uuid4().hex}"
    (temporary / "frames").mkdir(parents=True)
    frames = []
    worker_context = WheelActuatorWorker(source) if source is not None else nullcontext(None)
    try:
        with worker_context as worker:
            simulator = Simulator(PhysicsService._experiment(config, candidate=track != "reference"), actuator=worker)
            simulator.prepare(history=_history(history, public=False))
            PhysicsService._align_wall(simulator, config)
            simulator.model.vis.global_.offwidth = max(width, simulator.model.vis.global_.offwidth)
            simulator.model.vis.global_.offheight = max(height, simulator.model.vis.global_.offheight)
            with mujoco.Renderer(simulator.model, height=height, width=width) as renderer:
                next_frame = 0

                def capture(current, phase):
                    nonlocal next_frame
                    t = current.trial_time
                    if phase != "trial" or t + 1e-9 < next_frame / fps:
                        return
                    renderer.update_scene(current.data, camera="chase")
                    filename = f"frames/frame_{len(frames):06d}.jpg"
                    Image.fromarray(renderer.render()).save(temporary / filename, quality=82)
                    frames.append({"t_s": t, "file": filename})
                    next_frame = math.floor((t + 1e-9) * fps) + 1

                actual = simulator.run(capture, prepared=True)["public"]
                # Preserve the actual final state rather than inventing a final frame.
                if not frames or actual["trial_duration"] - frames[-1]["t_s"] > 1e-9:
                    next_frame = 0
                    capture(simulator, "trial")
        if not outcomes_match(actual, record["summary"]):
            raise ValueError("Rendered rollout differs from the recorded outcome; media was not published")
        manifest = {"kind": "mujoco_rendered_replay", "render_sha256": digest,
                    "source_sha256": source_hash, "record_sha256": hashlib.sha256(record_path.read_bytes()).hexdigest(),
                    "metrics_match": True, "summary": actual, "config": config,
                    "duration_s": actual["trial_duration"], "fps": fps, "width": width, "height": height,
                    "camera": "chase", "frames": frames,
                    "description": "MuJoCo rerender of saved commands and frozen source; outcome verified against the recording."}
        (temporary / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
        cache.parent.mkdir(parents=True, exist_ok=True)
        if cache.exists():
            shutil.rmtree(temporary)
        else:
            temporary.rename(cache)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copytree(cache, target)
        return manifest
    except BaseException:
        shutil.rmtree(temporary, ignore_errors=True)
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--case", choices=CASES, action="append")
    parser.add_argument("--track", choices=TRACKS, action="append")
    args = parser.parse_args()
    failed = 0
    for case in args.case or CASES:
        for track in args.track or TRACKS:
            print(f"Rendering {case} / {track}...", flush=True)
            try:
                manifest = render_track(args.run_dir, case, track)
            except Exception as error:
                failed += 1
                print(f"No verified replay for {case} / {track}: {type(error).__name__}.", flush=True)
                continue
            print(f"Verified {len(manifest['frames'])} MuJoCo frames.", flush=True)
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
