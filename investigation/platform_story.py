"""Host-owned action evidence and replay index for the investigation UI."""

from datetime import datetime, timezone
import json
from pathlib import Path
from uuid import uuid4


GOALS = {
    "car": "Complete the selected driving maneuver and a control probe within the declared limits.",
    "drone": "Complete hover and maneuver probes while remaining stable and airborne.",
    "quadruped": "Complete walking and turning probes while remaining upright and controlled.",
}
KINDS = {"apply_repair": "repair", "replace_model_source": "model_edit",
         "restore_model_version": "model_edit", "run_experiment": "experiment",
         "run_model": "prediction", "check_repair": "verification",
         "run_regression_suite": "verification", "submit_result": "submission"}


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{uuid4().hex}.tmp")
    try:
        temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def criteria(platform, probes, duration):
    return {"goal": GOALS[platform], "probes": list(probes), "duration_s": duration,
            "physical_goal": "Every verification probe completes and its public safe flag is true.",
            "prediction_goal": "Every frozen prediction overlaps the full observed probe and has position RMSE at most 0.5 m.",
            "prediction_rmse_max_m": 0.5, "declared_before_run": True,
            "scope": "These controlled synthetic probes only."}


class Story:
    def __init__(self, root, platform, predeclared):
        self.root = Path(root).resolve()
        self.path = self.root / "story.json"
        self.goal = {"title": GOALS[platform],
                     "criteria": [predeclared["physical_goal"], predeclared["prediction_goal"]]}
        self.platform = platform
        self.checkpoints = []
        self.replays = []
        self.verification = {"status": "pending", "goal_achieved": None, "cases": []}
        self.explanations = []
        self.initial_mismatch = False
        self.publish()

    def replay(self, physics, record, kind, label):
        # A descriptor points to an actual recording; rendering never invents frames.
        base = getattr(physics, "workdir", None)
        if base is None or not isinstance(record, dict) or not record.get("id"):
            return
        path = (Path(base).resolve() / record["id"] / "record.json").resolve()
        if not path.is_relative_to(self.root):
            return
        entry = {"id": record["id"], "kind": kind, "label": label,
                 "probe": record.get("probe"), "record_path": path.relative_to(self.root).as_posix()}
        if not any(item["id"] == entry["id"] for item in self.replays):
            self.replays.append(entry)

    def begin(self, tool, arguments):
        args = arguments if isinstance(arguments, dict) else {}
        kind = KINDS.get(tool, "inspection")
        action = str(args.get("action", tool))
        target = args.get("target") if isinstance(args.get("target"), str) else None
        text = lambda value: value if isinstance(value, str) else None
        titles = {"inspect_system": "Inspect system capabilities", "observe_system": "Inspect component measurements",
                  "inspect_model": "Inspect prediction source", "observe_run": "Inspect recorded measurements",
                  "run_experiment": "Run diagnostic experiment", "run_model": "Run predictive model",
                  "replace_model_source": "Replace predictive source", "restore_model_version": "Restore predictive source",
                  "check_repair": "Check repair against healthy behavior", "run_regression_suite": "Check multiple maneuvers",
                  "submit_result": "Submit diagnosis and changes"}
        title = (str(action).replace("_", " ").capitalize() + (f" · {target}" if target else "")) if kind == "repair" else titles.get(tool, str(tool))
        entry = {"id": f"checkpoint_{len(self.checkpoints) + 1:03d}", "tool": tool,
                 "kind": kind, "stage": "attempted", "title": title, "action": action, "target": target,
                 "probe": text(args.get("probe")), "reason": text(args.get("rationale") or args.get("hypothesis") or args.get("diagnosis")),
                 "expected_effect": text(args.get("expected_effect") or args.get("expected_observation")),
                 "timestamp": datetime.now(timezone.utc).isoformat(), "changes": []}
        self.checkpoints.append(entry)
        self.publish()
        return entry

    def finish(self, entry, result, *, before=None, after=None, source_diff=None):
        if not result.get("ok"):
            entry.update(stage="rejected", error=result.get("error"))
        elif entry["kind"] in {"repair", "model_edit"}:
            entry["stage"] = "applied"
        elif entry["kind"] == "verification":
            cases = result.get("cases", [result])
            checked = [case.get("difference", {}).get("observed_within_envelope") for case in cases]
            entry["stage"] = "verified" if checked and all(value is True for value in checked) else "failed"
        else:
            entry["stage"] = "completed"
        entry["result"] = {key: result[key] for key in
                           ("receipt", "accepted", "source_sha256", "version_id", "difference", "submitted", "meaning", "summary")
                           if key in result}
        if "cases" in result:
            entry["result"]["cases"] = [{"difference": case.get("difference")} for case in result["cases"]]
        if isinstance(before, dict) and isinstance(after, dict):
            entry["changes"] = [{"parameter": key, "before": before.get(key), "after": after.get(key)}
                                for key in sorted(set(before) | set(after)) if before.get(key) != after.get(key)]
            entry["before_parameters"], entry["after_parameters"] = before, after
        if source_diff is not None:
            entry["source_diff"] = source_diff
        self.publish()

    def explanation(self, text):
        if isinstance(text, str) and text.strip():
            self.explanations.append({"text": text, "timestamp": datetime.now(timezone.utc).isoformat(),
                                      "meaning": "Model statement; not an executed repair."})
            self.publish()

    def action_summary(self):
        repairs = [row for row in self.checkpoints if row["kind"] == "repair"]
        edits = [row for row in self.checkpoints if row["kind"] == "model_edit"]
        applied = lambda rows: sum(row["stage"] == "applied" for row in rows)
        attempted = bool(repairs or edits)
        changed = bool(applied(repairs) + applied(edits))
        if not attempted:
            status = "diagnosed_but_no_fix_attempted" if self.initial_mismatch and (self.explanations or any(
                row["kind"] == "submission" and row["stage"] == "completed" for row in self.checkpoints)) else "no_fix_attempted"
        elif not changed:
            status = "fix_attempted_but_not_applied"
        elif self.verification["status"] != "completed":
            status = "applied_unverified"
        else:
            status = "goal_achieved" if self.verification["goal_achieved"] else "applied_but_goal_not_achieved"
        return {"repair_attempts": len(repairs), "repairs_applied": applied(repairs),
                "model_edit_attempts": len(edits), "model_edits_applied": applied(edits),
                "failed_attempts": sum(row["stage"] in {"rejected", "failed"} for row in self.checkpoints),
                "fix_attempted": attempted, "change_applied": changed, "status": status}

    def publish(self):
        atomic_json(self.path, {"schema_version": 1, "platform": self.platform, "goal": self.goal,
                               "checkpoints": self.checkpoints, "replays": self.replays,
                               "explanations": self.explanations, "verification": self.verification,
                               "action_summary": self.action_summary()})
