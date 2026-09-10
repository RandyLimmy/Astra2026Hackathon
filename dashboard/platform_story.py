"""Compact public story projections for saved platform investigations.

This module deliberately does not import the simulator or start computations.
All replays and adjustments come from recorded host artifacts.
"""

import difflib
from copy import deepcopy
import hashlib
import re


CATALOG = {
    "quadruped": {"label": "Robot dog", "default_scenario": "quadruped_gait_failure",
                  "goal": "Complete the requested pace transition upright along the marked walking strip.",
                  "scenarios": [("quadruped_gait_failure", "A faster walk",
                                 "Adjust the gait controller to complete the original speed transition without falling.")]},
    "drone": {"label": "Drone", "default_scenario": "drone_delivery_imbalance",
              "goal": "Carry the parcel from A to B, place and release it, then return to A and land.",
              "scenarios": [("drone_delivery_imbalance", "An uneven load",
                             "Adjust flight control to complete the same loaded delivery and unloaded return.")]},
    "warehouse": {"label": "Warehouse trolley", "default_scenario": "warehouse_curve_demo",
                  "goal": "Complete the marked 90-degree route with cargo aboard and no ground impact.",
                  "scenarios": [("warehouse_curve_demo", "The warehouse bend",
                                 "Adjust the trolley route controller to carry its load around the same bend.")]},
    "car": {"label": "Car braking", "default_scenario": "car_auto_brake_failure",
            "goal": "Stop the conditioned car in the marked target zone with at least 2 m of clearance before the barrier.",
            "scenarios": [("car_auto_brake_failure", "Stop before the wall",
                           "Adjust braking control to stop in the target zone after the same brake-conditioning history.")]},
}
IDENTIFIER = re.compile(r"[A-Za-z0-9][A-Za-z0-9_-]{0,127}\Z")
IMAGE = re.compile(r"[A-Za-z0-9_-]+\.(?:jpg|jpeg|png|webp)\Z")
DROP_KEYS = {"observations", "trajectory", "trajectories", "candidate_state", "frames"}


def scenarios():
    return {"platforms": [{"id": platform, **{key: value for key, value in data.items() if key != "scenarios"},
                           "scenarios": [{"id": key, "label": label, "description": description}
                                         for key, label, description in data["scenarios"]]}
                          for platform, data in CATALOG.items()]}


def valid_scenario(platform, scenario):
    return (isinstance(platform, str) and isinstance(scenario, str) and platform in CATALOG
            and any(row[0] == scenario for row in CATALOG[platform]["scenarios"]))


def compact(value):
    """Keep metrics and evidence without embedding frame/trajectory arrays."""
    if isinstance(value, dict):
        return {key: compact(item) for key, item in value.items() if key not in DROP_KEYS}
    if isinstance(value, list):
        return [compact(item) for item in value]
    return value


def record_parts(value):
    if not isinstance(value, str) or "\\" in value:
        return None
    parts = value.split("/")
    if (len(parts) == 3 and parts[0] == "physics" or
            len(parts) == 4 and parts[:2] == ["broker", "verification"]):
        if IDENTIFIER.fullmatch(parts[-2]) and parts[-1] == "record.json":
            return parts
    return None


def frame_parts(value):
    if not isinstance(value, str) or "\\" in value:
        return None
    parts = value.split("/")
    if (len(parts) == 4 and parts[0] == "physics" or
            len(parts) == 5 and parts[:2] == ["broker", "verification"]):
        if IDENTIFIER.fullmatch(parts[-3]) and parts[-2] == "frames" and IMAGE.fullmatch(parts[-1]):
            return parts
    return None


def _records(app, run_id):
    """Inspect only the two known host record directories, never recursive globs."""
    from .server import read_json
    records = {}
    for prefix in (("physics",), ("broker", "verification")):
        directory = app.artifact(run_id, *prefix)
        if not directory.is_dir():
            continue
        for child in sorted(directory.iterdir()):
            if not child.is_dir() or child.is_symlink() or not IDENTIFIER.fullmatch(child.name):
                continue
            path_parts = (*prefix, child.name, "record.json")
            record = read_json(app.artifact(run_id, *path_parts))
            if isinstance(record, dict):
                records["/".join(path_parts)] = record
    return records


def replay(app, run_id, item, records):
    """A frame URL is emitted only when its recorded owner and path agree."""
    if not isinstance(item, dict):
        return None
    path = item.get("record_path") or item.get("path")
    parts = record_parts(path)
    if parts is None or path not in records:
        return None
    record = records[path]
    result = {**compact(item), "record_path": path,
              "id": item.get("id") or record.get("id") or parts[-2],
              "kind": item.get("kind") or record.get("kind"),
              "probe": record.get("probe"), "duration_s": record.get("duration_s"),
              "actual_duration_s": record.get("actual_duration_s"),
              "summary": compact(record.get("summary")),
              "frames": []}
    base = parts[:-2]
    for frame in record.get("frames", []):
        if not isinstance(frame, dict):
            continue
        relative = frame.get("file") or frame.get("path")
        if not isinstance(relative, str):
            continue
        # Physics stores file names relative to its own workdir, not the child run.
        full = "/".join([*base, relative])
        allowed = frame_parts(full)
        if allowed is None or allowed[-3] != parts[-2]:
            continue
        candidate = app.artifact(run_id, *allowed)
        if candidate.is_file():
            result["frames"].append({"t_s": frame.get("t_s", frame.get("time_s", 0)),
                                     "url": f"/api/runs/{run_id}/platform-media/{full}"})
    result["media_status"] = "available" if result["frames"] else "unavailable"
    return result


def _verification(value, prior):
    aggregate = value.get("aggregate") or {}
    prior_cases = {case.get("probe"): case for case in prior.get("cases", []) if isinstance(case, dict)}
    cases = [{**prior_cases.get(case.get("probe"), {}), **compact(case)}
             for case in value.get("cases", []) if isinstance(case, dict)]
    return {**prior, "status": "completed",
            "goal_achieved": aggregate.get("goal_achieved", prior.get("goal_achieved")),
            "predictive_success": aggregate.get("predictive_success", prior.get("predictive_success")),
            "partial_success": aggregate.get("partial_success", prior.get("partial_success")),
            "aggregate": compact(aggregate), "cases": cases}


def _reassessment(app, run_id, records):
    """Accept a host recheck only when it names this exact frozen run and replay."""
    from .server import MAX_FILE_BYTES, RequestError, read_json
    warning = "Saved scorer reassessment was ignored because its provenance or owned recording could not be verified."
    try:
        path = app.artifact(run_id, "evaluation", "reassessment.json")
        if not path.is_file():
            return None, None, None
        saved = read_json(path)
        if not isinstance(saved, dict) or saved.get("kind") != "control_task_reassessment":
            return None, None, warning
        def digest(*parts):
            artifact = app.artifact(run_id, *parts)
            if not artifact.is_file() or artifact.stat().st_size > MAX_FILE_BYTES:
                return None
            return hashlib.sha256(artifact.read_bytes()).hexdigest()
        original_hash = digest("evaluation", "result.json")
        controller_hash = digest("broker", "submission", "controller.json")
        original_result = read_json(app.artifact(run_id, "evaluation", "result.json"))
        updated = saved.get("result")
        descriptor = saved.get("replay")
        if (original_hash is None or controller_hash is None or not isinstance(original_result, dict)
                or saved.get("original_result_sha256") != original_hash
                or saved.get("controller_sha256") != controller_hash
                or not isinstance(updated, dict) or updated.get("source_sha256") != controller_hash
                or not isinstance(updated.get("aggregate"), dict)
                or type(updated["aggregate"].get("goal_achieved")) is not bool
                or not isinstance(updated.get("cases"), list)
                or not updated["cases"] or any(not isinstance(case, dict) for case in updated["cases"])
                or not isinstance(descriptor, dict) or descriptor.get("kind") != "after"):
            return None, None, warning
        parts = record_parts(descriptor.get("record_path"))
        if parts is None or parts[:2] != ["broker", "verification"]:
            return None, None, warning
        record = records.get(descriptor["record_path"], {})
        if (record.get("source_sha256") != controller_hash
                or descriptor.get("id") != record.get("id")
                or descriptor.get("probe") != record.get("probe")):
            return None, None, warning
        resolved = replay(app, run_id, descriptor, records)
        return (saved, resolved, None) if resolved is not None else (None, None, warning)
    except (OSError, RequestError, ValueError, TypeError):
        return None, None, warning


def story(app, run_id, summary):
    from .server import read_json, read_text, read_events
    saved = read_json(app.artifact(run_id, "story.json"))
    saved = saved if isinstance(saved, dict) else {}
    result = compact(deepcopy(saved))
    result.update(id=run_id, metadata=summary["metadata"], status=summary["status"], active=summary["active"])
    result.setdefault("goal", {"status": "pending", "description": CATALOG.get(summary["metadata"].get("platform"), {}).get("goal")})
    result.setdefault("checkpoints", [])
    result["events"] = compact(read_events(app.artifact(run_id, "events.jsonl")))
    verification = read_json(app.artifact(run_id, "evaluation", "result.json"))
    if not isinstance(verification, dict):
        verification = read_json(app.artifact(run_id, "broker", "verification_result.json"))
    if isinstance(verification, dict):
        prior = result.get("verification") or {}
        result["verification"] = _verification(verification, prior)
    else:
        result.setdefault("verification", {"status": "pending", "goal_achieved": None})
    records = _records(app, run_id)
    entries = saved.get("replays", [])
    if isinstance(entries, dict):
        entries = [{"kind": key, **value} for key, value in entries.items() if isinstance(value, dict)]
    if not isinstance(entries, list):
        entries = []
    result["replays"] = [value for entry in entries if (value := replay(app, run_id, entry, records))]
    # Live recordings are useful before the final host story has been written.
    if not result["replays"]:
        result["replays"] = [value for path, record in records.items()
                             if (value := replay(app, run_id, {"record_path": path, "kind": record.get("kind")}, records))]
    reassessment, reassessed_replay, warning = _reassessment(app, run_id, records)
    if reassessment is not None:
        result["original_verification"] = deepcopy(result["verification"])
        updated = reassessment["result"]
        result["verification"] = _verification(updated, result["verification"])
        original_cases = verification.get("cases", []) if isinstance(verification, dict) else []
        original_after = original_cases[0].get("after", {}) if original_cases and isinstance(original_cases[0], dict) else {}
        original_after = original_after if isinstance(original_after, dict) else {}
        result["reassessment"] = {
            "reason": reassessment.get("reason"), "evaluated_at": reassessment.get("evaluated_at"),
            "original_outcome": compact(verification.get("aggregate", {})),
            "original_termination": original_after.get("summary", {}).get("outcome"),
        }
        if isinstance(updated.get("reassessment_criteria"), dict):
            result["reassessment"]["criteria"] = compact(updated["reassessment_criteria"])
        result["replays"] = [item for item in result["replays"] if item.get("kind") != "after"] + [reassessed_replay]
        if isinstance(updated.get("action_summary"), dict):
            result["action_summary"] = compact(updated["action_summary"])
    elif warning:
        result["reassessment_warning"] = warning
    filename = "controller.json" if summary["metadata"].get("task_kind") == "controller_repair" else "model.py"
    original = read_text(app.artifact(run_id, "broker", "versions", "v000", filename))
    final = read_text(app.artifact(run_id, "broker", "submission", filename))
    if final is None:
        versions = app.artifact(run_id, "broker", "versions")
        if versions.is_dir():
            for version in sorted(versions.iterdir(), reverse=True):
                if re.fullmatch(r"v\d{3}", version.name) and not version.is_symlink():
                    final = read_text(app.artifact(run_id, "broker", "versions", version.name, filename))
                    if final is not None:
                        break
    result["source"] = {"original": original, "current": final, "diff": "", "filename": filename}
    if original is not None and final is not None:
        result["source"]["diff"] = "".join(difflib.unified_diff(original.splitlines(keepends=True),
            final.splitlines(keepends=True), fromfile=f"original/{filename}", tofile=f"current/{filename}"))
    result["metrics"] = {"api_requests": summary["api_requests"], "tool_calls": summary["tool_calls"],
                         "duration_s": summary["metadata"].get("duration_s"),
                         "usage": summary["metadata"].get("usage"),
                         "estimated_cost_usd": summary["metadata"].get("estimated_cost_usd"),
                         "cost": summary["metadata"].get("cost")}
    return result
