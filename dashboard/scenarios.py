"""Read only completed public dog/drone recordings; never launch a simulation."""

import re


from simulator.platforms.catalog import REPLAY_TASKS as SCENARIOS


def media_path(app, scenario, run_id, parts):
    from .server import IDENTIFIER, RequestError, safe_path

    if scenario not in SCENARIOS or not IDENTIFIER.fullmatch(run_id):
        raise RequestError(404, "Scenario replay not found.")
    if parts in (["manifest.json"], ["events.json"], ["task.md"], ["CONTROL_INTERFACE.md"], ["observations.jsonl"],
                 ["controller.json"], ["controller_source.py"], ["CONTROLLER.md"],
                 ["preparation.json"], ["history_events.json"]):
        pass
    elif (len(parts) == 2 and parts[0] in {"frames", "evidence"}
          and re.fullmatch(r"(?:side|overview|chase)_\d{6}\.(?:jpg|png)", parts[1])):
        pass
    else:
        raise RequestError(404, "Scenario replay not found.")
    return safe_path(app.root, "runs", "scenario-replays", scenario, run_id, "public", *parts)


def listing(app):
    from .server import IDENTIFIER, RequestError, read_json, safe_path

    entries = []
    for scenario, details in SCENARIOS.items():
        item = {"id": scenario, **details, "status": "missing", "manifest_url": None}
        try:
            root = safe_path(app.root, "runs", "scenario-replays", scenario)
            recordings = sorted(root.iterdir(), key=lambda path: path.name, reverse=True) if root.is_dir() else []
            for recording in recordings:
                if recording.is_symlink() or not recording.is_dir() or not IDENTIFIER.fullmatch(recording.name):
                    continue
                path = media_path(app, scenario, recording.name, ["manifest.json"])
                manifest = read_json(path)
                if (not isinstance(manifest, dict) or manifest.get("schema_version") != 1
                        or manifest.get("scenario_id") != scenario or not manifest.get("frames")
                        or manifest.get("provenance") != "original_attempt"):
                    continue
                item.update(status="ready", run_id=manifest.get("run_id"),
                            manifest_url=f"/api/scenarios/{scenario}/recordings/{recording.name}/manifest.json")
                break
        except (OSError, RequestError):
            pass
        entries.append(item)
    return {"scenarios": entries}
