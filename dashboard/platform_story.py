"""Compact public story projections for saved platform investigations.

This module deliberately does not import the simulator or start computations.
All replays and adjustments come from recorded host artifacts.
"""

import difflib
from copy import deepcopy
import re


CATALOG = {
    "car": {"label": "New car", "default_scenario": "car_wheel_misalignment",
            "goal": "Restore controlled driving and verify the same maneuver after repairs.",
            "scenarios": [
                ("car_postcrash_healthy", "Healthy control", "Barrier impact, recovery, and healthy controlled steering reference."),
                ("car_steering_damage", "Steering damage", "Barrier impact leaves reduced steering rack response and a steering bias."),
                ("car_wheel_misalignment", "Wheel misalignment", "A bent left front wheel mount changes toe during a slalom probe."),
                ("car_suspension_damage", "Suspension damage", "Impact weakens the left front spring before a one-wheel bump probe."),
                ("car_tire_pressure", "Tire pressure", "Impact changes left front tire radius/contact compliance: synthetic pressure proxy."),
                ("car_demo", "Crash demo", "Visible approach, measured barrier impact, then a moderately weakened steering inspection."),
            ]},
    "drone": {"label": "Drone", "default_scenario": "drone_rotor_loss",
              "goal": "Complete the flight maneuver while stable and airborne, then verify the repair.",
              "scenarios": [
                  ("drone_hover", "Healthy flight", "Healthy hover followed by a small, controlled translation probe."),
                  ("drone_rotor_loss", "Rotor loss", "One rotor loses thrust after healthy flight; bounded control loses attitude."),
                  ("drone_voltage_sag", "Voltage sag", "Supply voltage drops; all four rotors lose available thrust."),
                  ("drone_payload", "Added payload", "An explicit co-moving payload pickup increases real mass and inertia."),
                  ("drone_wind", "Wind", "A sustained crosswind force challenges nominal position prediction."),
                  ("drone_delay", "Command delay", "Motor command transport delay appears before the translation probe."),
                  ("drone_demo", "Flight demo", "Fly a visible course, lose some rotor thrust, then return to hover with a tracking residual."),
              ]},
    "quadruped": {"label": "Robot dog", "default_scenario": "quadruped_joint_weakness",
                  "goal": "Complete the walking maneuver upright and verify restored support and motion.",
                  "scenarios": [
                      ("quadruped_walk", "Healthy walking", "Healthy articulated dog performs a controlled forward crawl."),
                      ("quadruped_joint_weakness", "Joint weakness", "One knee actuator loses torque after normal walking."),
                      ("quadruped_foot_slip", "Foot slip", "One foot loses contact friction during the same walking probe."),
                      ("quadruped_leg_damage", "Leg damage", "One knee gains a stiff, bent rest configuration after damage."),
                      ("quadruped_payload_shift", "Shifted payload", "An onboard payload slides sideways, shifting the center of mass."),
                      ("quadruped_demo", "Walking and fall demo", "Walk a metre, lose one knee's support, stumble and fall, then simulate the aftermath."),
                  ]},
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
        aggregate = verification.get("aggregate") or {}
        prior = result.get("verification") or {}
        prior_cases = {case.get("probe"): case for case in prior.get("cases", []) if isinstance(case, dict)}
        cases = [{**prior_cases.get(case.get("probe"), {}), **compact(case)}
                 for case in verification.get("cases", []) if isinstance(case, dict)]
        result["verification"] = {**prior, "status": "completed",
                                  "goal_achieved": aggregate.get("goal_achieved", prior.get("goal_achieved")),
                                  "predictive_success": aggregate.get("predictive_success", prior.get("predictive_success")),
                                  "aggregate": compact(aggregate), "cases": cases}
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
    original = read_text(app.artifact(run_id, "broker", "versions", "v000", "model.py"))
    final = read_text(app.artifact(run_id, "broker", "submission", "model.py"))
    if final is None:
        versions = app.artifact(run_id, "broker", "versions")
        if versions.is_dir():
            for version in sorted(versions.iterdir(), reverse=True):
                if re.fullmatch(r"v\d{3}", version.name) and not version.is_symlink():
                    final = read_text(app.artifact(run_id, "broker", "versions", version.name, "model.py"))
                    if final is not None:
                        break
    result["source"] = {"original": original, "current": final, "diff": ""}
    if original is not None and final is not None:
        result["source"]["diff"] = "".join(difflib.unified_diff(original.splitlines(keepends=True),
            final.splitlines(keepends=True), fromfile="original/model.py", tofile="current/model.py"))
    result["metrics"] = {"api_requests": summary["api_requests"], "tool_calls": summary["tool_calls"],
                         "duration_s": summary["metadata"].get("duration_s"),
                         "usage": summary["metadata"].get("usage"),
                         "estimated_cost_usd": summary["metadata"].get("estimated_cost_usd"),
                         "cost": summary["metadata"].get("cost")}
    return result
