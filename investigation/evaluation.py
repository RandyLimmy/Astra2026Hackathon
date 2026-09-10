"""Host-only frozen evaluation. No reserved outcomes are exposed before locks."""

import ast
from datetime import datetime, timezone
import difflib
import hashlib
import math
from pathlib import Path

from .physics import ORIGINAL_SOURCE, PhysicsService, _digest, _write_json, normalize_config


RESERVED_CONFIGS = (
    {"speed_mps": 22, "preparation_cycles": 0, "wait_s": 0},
    {"speed_mps": 22, "preparation_cycles": 2, "wait_s": 0},
    {"speed_mps": 22, "preparation_cycles": 2, "wait_s": 45},
)
PREDECLARED_CRITERIA = {
    "maximum_candidate_to_original_mae_ratio": 0.5,
    "minimum_original_mae_for_ratio_m": 1e-6,
    "per_case_absolute_floor_m": 1.0,
    "per_case_relative_tolerance": 0.10,
    "cold_control_absolute_floor_m": 1.0,
    "cold_control_relative_tolerance": 0.05,
    "require_matching_collision_outcomes": True,
    "require_all_reserved_cases_unseen": True,
    "require_uncensored_stops": True,
}


def _now():
    return datetime.now(timezone.utc).isoformat()


def _error(model, reference):
    a, b = model["summary"], reference["summary"]
    values = (a.get("stopping_distance"), b.get("stopping_distance"))
    if not a.get("stopped") or not b.get("stopped") or any(
        value is None or not math.isfinite(value) for value in values
    ):
        return None
    return abs(values[0] - values[1])


def _function_shapes(source):
    try:
        tree = ast.parse(source)
    except (SyntaxError, ValueError):
        return {}
    return {node.name: ast.dump(node, include_attributes=False)
            for node in tree.body if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))}


def _source_indicators(original, candidate, original_records, candidate_records):
    before, after = _function_shapes(original), _function_shapes(candidate)
    keys = lambda records: sorted({key for record in records for key in record.get("candidate_state", {})})
    original_keys, candidate_keys = keys(original_records), keys(candidate_records)
    return {
        "source_changed": original != candidate,
        "init_state_changed": before.get("init_state") != after.get("init_state"),
        "advance_state_changed": before.get("advance_state") != after.get("advance_state"),
        "force_rule_changed": before.get("compute_brake_torque_limits") != after.get("compute_brake_torque_limits"),
        "reset_rule_changed": before.get("on_trial_reset") != after.get("on_trial_reset"),
        "original_state_keys": original_keys, "candidate_state_keys": candidate_keys,
        "added_state_keys": sorted(set(candidate_keys) - set(original_keys)),
        "candidate_has_nonempty_state": bool(candidate_keys),
        "interpretation": "Source/runtime indicators only; they do not establish authorship, novelty, or physical identity.",
    }


def evaluate_frozen(service: PhysicsService, source: Path, output: Path, emit=None) -> dict:
    """Snapshot source, lock every prediction, then run all reserved probe truths.

    Reference preparation may run before prediction to obtain the permitted
    command/reset timeline. Candidate state evolves only from its own physics.
    Full-resolution records are saved under cases/; result.json is compact.
    """
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    candidate_bytes, original_bytes = Path(source).read_bytes(), ORIGINAL_SOURCE.read_bytes()
    frozen = output / "frozen_candidate.py"
    original_path = output / "original_candidate.py"
    frozen.write_bytes(candidate_bytes)
    original_path.write_bytes(original_bytes)
    source_hash = hashlib.sha256(candidate_bytes).hexdigest()
    original_hash = hashlib.sha256(original_bytes).hexdigest()
    frozen_at = _now()
    _write_json(output / "criteria.json", {"declared_at": frozen_at, "criteria": PREDECLARED_CRITERIA})
    original_text = original_bytes.decode("utf-8", errors="replace")
    candidate_text = candidate_bytes.decode("utf-8", errors="replace")
    (output / "source.diff").write_text("".join(difflib.unified_diff(
        original_text.splitlines(keepends=True), candidate_text.splitlines(keepends=True),
        fromfile="original_candidate.py", tofile="frozen_candidate.py",
    )))

    def progress(stage, **details):
        if emit is not None:
            emit({"type": "evaluation_progress", "stage": stage, **details})

    def model_record(config, path, history, digest):
        try:
            return service.model_record(config, path, history)
        except Exception as error:
            return {"config": config, "preparation_history": history, "observations": [],
                    "summary": {"stopped": False, "censored": True, "stopping_distance": None},
                    "source_sha256": digest, "error": f"{type(error).__name__}: {error}"}

    progress("source_frozen", source_sha256=source_hash)
    pending = []
    for index, values in enumerate(RESERVED_CONFIGS, 1):
        config = normalize_config(values)
        case_id = f"reserved_{index}"
        prepared = service.prepare_reference(config)
        original = model_record(config, original_path, prepared.history, original_hash)
        candidate = model_record(config, frozen, prepared.history, source_hash)
        case_dir = output / "cases" / case_id
        _write_json(case_dir / "original.json", original)
        _write_json(case_dir / "candidate.json", candidate)
        locked_at = _now()
        _write_json(case_dir / "prediction_lock.json", {
            "config": config, "source_sha256": source_hash,
            "original_source_sha256": original_hash, "locked_at": locked_at,
        })
        pending.append((case_id, prepared, original, candidate, locked_at))
        progress("prediction_locked", case_id=case_id, completed=index, total=len(RESERVED_CONFIGS))

    all_locked_at = _now()
    _write_json(output / "predictions_locked.json", {
        "source_sha256": source_hash, "original_source_sha256": original_hash,
        "source_frozen_at": frozen_at, "all_predictions_saved_at": all_locked_at,
        "cases": [{"case_id": case_id, "config": prepared.config, "locked_at": locked_at}
                  for case_id, prepared, _, _, locked_at in pending],
    })
    progress("all_predictions_locked", count=len(pending))

    cases = []
    for case_id, prepared, original, candidate, locked_at in pending:
        reference = service.finish_reference(prepared)
        _write_json(output / "cases" / case_id / "reference.json", reference)
        case = {
            "case_id": case_id, "config": prepared.config, "prediction_locked_at": locked_at,
            "reference_revealed_at": _now(),
            "original": service._public(original), "candidate": service._public(candidate),
            "reference": service._public(reference),
            "original_error_m": _error(original, reference),
            "candidate_error_m": _error(candidate, reference),
            "previously_observed": _digest(prepared.config) in service.development_configs,
            "full_records_directory": f"cases/{case_id}",
        }
        if "error" in candidate:
            case["candidate_error"] = candidate["error"]
        if "error" in original:
            case["original_error"] = original["error"]
        reference_summary, candidate_summary = reference["summary"], candidate["summary"]
        distance = reference_summary.get("stopping_distance")
        comparable_reference = bool(reference_summary.get("stopped")) and distance is not None
        case["distance_tolerance_m"] = (
            max(PREDECLARED_CRITERIA["per_case_absolute_floor_m"],
                PREDECLARED_CRITERIA["per_case_relative_tolerance"] * abs(distance))
            if comparable_reference else None
        )
        case["within_tolerance"] = (
            case["candidate_error_m"] is not None and case["candidate_error_m"] <= case["distance_tolerance_m"]
            if comparable_reference else None
        )
        known_collision = all(summary.get("collision") is not None
                              for summary in (reference_summary, candidate_summary))
        case["collision_correct"] = (candidate_summary["collision"] == reference_summary["collision"]
                                     if known_collision else None)
        if case_id == "reserved_1":
            case["cold_control_tolerance_m"] = (
                max(PREDECLARED_CRITERIA["cold_control_absolute_floor_m"],
                    PREDECLARED_CRITERIA["cold_control_relative_tolerance"] * abs(distance))
                if comparable_reference else None
            )
            case["cold_control_pass"] = (
                case["candidate_error_m"] is not None
                and case["candidate_error_m"] <= case["cold_control_tolerance_m"]
                if comparable_reference else None
            )
        cases.append(case)
        progress("reference_revealed", case_id=case_id)

    eligible = [case for case in cases if not case["previously_observed"]]
    scored = [case for case in eligible
              if case["original_error_m"] is not None and case["candidate_error_m"] is not None]
    original_mae = sum(case["original_error_m"] for case in scored) / len(scored) if scored else None
    candidate_mae = sum(case["candidate_error_m"] for case in scored) / len(scored) if scored else None
    aggregate = {
        "original_mae_m": original_mae, "candidate_mae_m": candidate_mae,
        "improvement_fraction": (1 - candidate_mae / original_mae) if original_mae else None,
        "scored_cases": len(scored), "eligible_cases": len(eligible),
        "previously_observed_cases": sum(case["previously_observed"] for case in cases),
        "failed_stop_predictions": sum(case["reference"]["summary"].get("stopped", False)
                                       and not case["candidate"]["summary"].get("stopped", False)
                                       for case in eligible),
        "original_failed_stop_predictions": sum(case["reference"]["summary"].get("stopped", False)
                                                and not case["original"]["summary"].get("stopped", False)
                                                for case in eligible),
    }
    improvement_pass = (
        candidate_mae <= PREDECLARED_CRITERIA["maximum_candidate_to_original_mae_ratio"] * original_mae
        if original_mae is not None and original_mae > PREDECLARED_CRITERIA["minimum_original_mae_for_ratio_m"]
        else None
    )
    per_case_pass = all(case["within_tolerance"] is True for case in eligible)
    collision_pass = all(case["collision_correct"] is True for case in eligible)
    cold_control_pass = cases[0]["cold_control_pass"] if not cases[0]["previously_observed"] else None
    complete = len(scored) == len(RESERVED_CONFIGS) and len(eligible) == len(RESERVED_CONFIGS)
    known_failure = any(case["within_tolerance"] is False or case["collision_correct"] is False
                        for case in eligible)
    success = (False if known_failure else
               bool(improvement_pass and per_case_pass and collision_pass and cold_control_pass)
               if complete and improvement_pass is not None else None)
    aggregate.update(predictive_success=success, improvement_pass=improvement_pass,
                     per_case_pass=per_case_pass, cold_control_pass=cold_control_pass,
                     collision_outcomes_match=collision_pass, scoring_complete=complete)
    result = {
        "kind": "frozen_synthetic_evaluation", "source_sha256": source_hash,
        "original_source_sha256": original_hash, "source_frozen_at": frozen_at,
        "all_predictions_saved_at": all_locked_at, "predeclared_criteria": dict(PREDECLARED_CRITERIA),
        "aggregate": aggregate, "cases": cases,
        "state_extension": _source_indicators(original_text, candidate_text,
                                               [entry[2] for entry in pending], [entry[3] for entry in pending]),
    }
    _write_json(output / "result.json", result)
    progress("evaluation_complete", aggregate=aggregate)
    return result
