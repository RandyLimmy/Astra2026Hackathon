"""Recheck a saved, frozen controller locally without making model API calls."""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path

from .platform_story import atomic_json
from .runner import timestamp
from .task_recording import record_task
from .tasks import TASKS, adapter_for

ROOT = Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def recheck(run_dir, *, reason, frames=True):
    run_dir = Path(run_dir).resolve()
    output = run_dir / "evaluation/reassessment.json"
    if output.exists():
        raise ValueError("A reassessment already exists; preserve it instead of overwriting it.")
    original_path = run_dir / "evaluation/result.json"
    original = json.loads(original_path.read_text())
    metadata = json.loads((run_dir / "metadata.json").read_text())
    controller_path = run_dir / "broker/submission/controller.json"
    controller = json.loads(controller_path.read_text())
    source_hash, original_hash = sha(controller_path), sha(original_path)
    if (original.get("task_kind") != "controller_repair"
            or original.get("scenario") not in TASKS
            or metadata.get("status") != "completed"
            or source_hash != original.get("source_sha256")
            or controller != original.get("controller")):
        raise ValueError("Rechecking requires a completed task with its original frozen controller intact.")
    adapter = adapter_for(original["scenario"])
    adapter.validate_candidate(controller)
    caps = adapter.capabilities()
    criteria = {"goal": caps["goal"], "task_criteria": caps.get("success_criteria", caps.get("criteria")),
                "duration_s": adapter.duration_s, "declared_before_reassessment": True}
    plan = {"kind": "control_task_reassessment", "reason": reason,
            "original_result_sha256": original_hash, "controller_sha256": source_hash,
            "scorer_sources_sha256": {p: sha(ROOT / p) for p in adapter.source_paths},
            "criteria": criteria, "started_at": timestamp(), "model_api_calls": 0}
    plan_path = run_dir / "evaluation/reassessment_plan.json"
    if plan_path.exists():
        raise ValueError("A reassessment plan already exists; preserve the earlier attempt.")
    atomic_json(plan_path, plan)
    record = record_task(adapter, controller, run_dir / "broker/verification",
                         frames=frames, kind="controller_reassessment")
    if sha(original_path) != original_hash or sha(controller_path) != source_hash:
        raise RuntimeError("Original result or frozen controller changed during reassessment.")
    if any(sha(ROOT / p) != digest for p, digest in plan["scorer_sources_sha256"].items()):
        raise RuntimeError("Scorer sources changed during reassessment; do not publish mixed-version results.")
    result = deepcopy(original)
    result["kind"] = "control_task_reassessment_result"
    result["reassessment_criteria"] = criteria
    result["aggregate"] = {"goal_achieved": record["goal_achieved"],
                           "partial_success": record["partial_success"], "predictive_success": None}
    case = result["cases"][0]
    case.update(goal_achieved=record["goal_achieved"], partial_success=record["partial_success"])
    case["after"] = {"summary": record["summary"], "duration_s": adapter.duration_s,
                     "goal_achieved": record["goal_achieved"], "partial_success": record["partial_success"]}
    case["after_difference"] = {"observed_outcome": record["summary"]["outcome"],
                                "observed_within_envelope": record["goal_achieved"],
                                "common_duration_s": adapter.duration_s, "position_rmse_m": None}
    result["action_summary"]["status"] = (
        "goal_achieved" if record["goal_achieved"] else "applied_but_goal_not_achieved"
        if result["action_summary"].get("change_applied") else "no_change_applied")
    replay = {"kind": "after", "id": record["id"], "probe": original["scenario"],
              "record_path": f"broker/verification/{record['id']}/record.json",
              "summary": record["summary"], "duration_s": adapter.duration_s,
              "provenance": "Frozen original controller rechecked after a scoring correction; no new model call."}
    assessment = {**plan, "evaluated_at": timestamp(), "result": result, "replay": replay}
    atomic_json(output, assessment)
    return assessment


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", required=True, type=Path)
    parser.add_argument("--reason", required=True)
    parser.add_argument("--no-frames", action="store_true")
    args = parser.parse_args(argv)
    result = recheck(args.run, reason=args.reason, frames=not args.no_frames)
    print(json.dumps({"run": str(args.run), "model_api_calls": 0,
                      "result": result["result"]["aggregate"],
                      "artifact": str(args.run / 'evaluation/reassessment.json')}))


if __name__ == "__main__":
    main()
