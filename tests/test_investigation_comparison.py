import hashlib
import json

import pytest

from investigation.comparison import ComparisonError, export_comparison, main


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value))


def digest(value):
    return hashlib.sha256(value).hexdigest()


def fixture_run(root, profile, distances):
    original = b"def init_state():\n    return {}\n"
    candidate = f"def init_state():\n    return {{'memory': {1 if profile == 'astra-medium' else 2}.0}}\n".encode()
    for relative, data in (("broker/submission/actuator.py", candidate),
                           ("evaluation/frozen_candidate.py", candidate),
                           ("evaluation/original_candidate.py", original)):
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
    manifest = {"schema_version": 1, "source_sha256": {"shared_protocol.py": "source-fixture"},
                "caps": {"max_api_requests": 12, "max_seconds": 1800}}
    fingerprint = digest(json.dumps(manifest, sort_keys=True, separators=(",", ":")).encode())
    metadata = {
        "status": "completed", "profile": profile,
        "model": "gpt-6-astra" if profile == "astra-medium" else "gpt-5.6-sol",
        "reasoning_effort": "medium" if profile == "astra-medium" else "high",
        "source_hash": digest(candidate), "frozen_source_file": "broker/submission/actuator.py",
        "protocol_fingerprint": fingerprint, "protocol_manifest": manifest,
        "agent_submitted": profile == "sol-high", "freeze_reason": "agent_submission" if profile == "sol-high" else "request_or_time_budget",
        "api_requests": 8, "duration_s": 125.0, "usage": {"total_tokens": 42000, "reasoning_tokens": 3000},
        "budgets": {name: {"used": used, "limit": limit} for name, used, limit in (
            ("tool_calls", 9, 30), ("run_experiment", 2, 6), ("patch_model", 2, 3),
            ("run_model", 3, 12), ("run_regression_suite", 1, 3))},
    }
    cases = []
    for index, (measured, predicted) in enumerate(zip((30.0, 40.0), distances), 1):
        def run(distance):
            return {"summary": {"stopping_distance": distance, "stopped": distance is not None,
                                "collision": False, "censored": distance is None}}
        cases.append({
            "case_id": f"reserved_{index}",
            "config": {"speed_mps": 22.0, "brake_strength": 1.0, "preparation_cycles": index - 1,
                       "wait_s": 0.0, "wall_distance_m": None},
            "original": run(20.0), "candidate": run(predicted), "reference": run(measured),
            "original_error_m": measured - 20.0,
            "candidate_error_m": abs(measured - predicted) if predicted is not None else None,
            "previously_observed": False, "collision_correct": True,
        })
    scored = [case for case in cases if case["candidate_error_m"] is not None]
    evaluation = {
        "source_sha256": digest(candidate), "original_source_sha256": digest(original), "cases": cases,
        "aggregate": {"original_mae_m": 15.0,
                      "candidate_mae_m": sum(case["candidate_error_m"] for case in scored) / len(scored) if scored else None,
                      "scored_cases": len(scored), "eligible_cases": 2,
                      "failed_stop_predictions": len(cases) - len(scored),
                      "predictive_success": True if len(scored) == 2 else None, "scoring_complete": len(scored) == 2},
        "predeclared_criteria": {"maximum_candidate_to_original_mae_ratio": .5, "require_all_reserved_cases_unseen": True},
        "state_extension": {"source_changed": True, "init_state_changed": True, "advance_state_changed": True},
    }
    write_json(root / "metadata.json", metadata)
    write_json(root / "evaluation/result.json", evaluation)
    return metadata, evaluation, candidate


def pair(tmp_path):
    first, second = tmp_path / "first", tmp_path / "second"
    fixture_run(first, "astra-medium", (29.0, 37.0))
    fixture_run(second, "sol-high", (30.0, 39.0))
    return first, second


def test_matched_export_copies_exact_sources_and_reports_metrics_usage_and_limits(tmp_path):
    first, second = pair(tmp_path)
    before = {path: digest(path.read_bytes()) for root in (first, second) for path in root.rglob("*") if path.is_file()}
    destination = tmp_path / "comparison"
    report = export_comparison(first, second, destination)
    bundle = json.loads((destination / "comparison.json").read_text())
    assert bundle["comparison_valid"] is True
    assert all(bundle["validation"].values())
    assert bundle["protocol_fingerprint"] == json.loads((first / "metadata.json").read_text())["protocol_fingerprint"]
    assert bundle["runs_per_profile"] == 1 and bundle["is_model_ranking"] is False
    assert bundle["prior_cases_known_to_builder"] is True and bundle["fresh_model_contexts"] is True
    for label, root in (("run_a", first), ("run_b", second)):
        assert (destination / label / "actuator.py").read_bytes() == (root / "evaluation/frozen_candidate.py").read_bytes()
        assert (destination / label / "original_actuator.py").read_bytes() == (root / "evaluation/original_candidate.py").read_bytes()
    text = report.read_text()
    assert "gpt-6-astra / medium" in text and "gpt-5.6-sol / high" in text
    assert "| Frozen candidate stopping MAE (m) | 2.000 | 0.500 |" in text
    assert "| Patch attempts | 2 | 2 |" in text
    assert "| Extra reference attempts | 2 | 2 |" in text
    assert "| API requests | 8 | 8 |" in text
    assert "| Explicit agent submission | no | yes |" in text
    assert "not a model ranking" in text
    assert "fresh API contexts" in text and "builder had seen" in text
    assert "maximum_candidate_to_original_mae_ratio" in text
    assert "[astra-medium: frozen component](run_a/actuator.py)" in text
    assert all(digest(path.read_bytes()) == fingerprint for path, fingerprint in before.items())


@pytest.mark.parametrize("mismatch", ["protocol", "config", "case_id", "criteria", "eligibility", "reference"])
def test_mismatched_inputs_are_explicitly_flagged_instead_of_silently_paired(tmp_path, mismatch):
    first, second = pair(tmp_path)
    metadata = json.loads((second / "metadata.json").read_text())
    evaluation = json.loads((second / "evaluation/result.json").read_text())
    expected = {"protocol": "protocol_fingerprints_match", "config": "case_configs_match", "case_id": "case_ids_match",
                "criteria": "predeclared_criteria_match", "eligibility": "case_eligibility_matches", "reference": "reference_summaries_match"}[mismatch]
    if mismatch == "protocol":
        metadata["protocol_manifest"]["caps"]["max_api_requests"] = 10
        metadata["protocol_fingerprint"] = digest(json.dumps(metadata["protocol_manifest"], sort_keys=True, separators=(",", ":")).encode())
    elif mismatch == "config":
        evaluation["cases"][0]["config"]["speed_mps"] = 23.0
    elif mismatch == "case_id":
        evaluation["cases"][0]["case_id"] = "different_case"
    elif mismatch == "criteria":
        evaluation["predeclared_criteria"]["maximum_candidate_to_original_mae_ratio"] = .9
    elif mismatch == "eligibility":
        evaluation["cases"][0]["previously_observed"] = True
    else:
        evaluation["cases"][0]["reference"]["summary"]["stopping_distance"] = 31.0
    write_json(second / "metadata.json", metadata)
    write_json(second / "evaluation/result.json", evaluation)
    report = export_comparison(first, second, tmp_path / "comparison")
    bundle = json.loads(report.with_suffix(".json").read_text())
    assert bundle["comparison_valid"] is False
    assert bundle["validation"][expected] is False
    assert "MISMATCH" in report.read_text()
    assert bundle["warnings"]


@pytest.mark.parametrize("tamper", ["frozen_source", "submitted_source", "original_source", "manifest", "incomplete"])
def test_integrity_or_completion_failure_refuses_to_create_export(tmp_path, tamper):
    first, second = pair(tmp_path)
    if tamper == "frozen_source":
        (second / "evaluation/frozen_candidate.py").write_text("changed after evaluation")
    elif tamper == "submitted_source":
        (second / "broker/submission/actuator.py").write_text("different submitted code")
    elif tamper == "original_source":
        (second / "evaluation/original_candidate.py").write_text("different original code")
    else:
        metadata = json.loads((second / "metadata.json").read_text())
        if tamper == "manifest":
            metadata["protocol_manifest"]["caps"]["max_api_requests"] = 1
        else:
            metadata["status"] = "running"
        write_json(second / "metadata.json", metadata)
    destination = tmp_path / "comparison"
    with pytest.raises(ComparisonError):
        export_comparison(first, second, destination)
    assert not destination.exists()


def test_censored_predictions_remain_missing_and_do_not_become_zero(tmp_path):
    first, second = tmp_path / "first", tmp_path / "second"
    fixture_run(first, "astra-medium", (None, None))
    fixture_run(second, "sol-high", (30.0, 39.0))
    report = export_comparison(first, second, tmp_path / "comparison")
    text = report.read_text()
    assert "| Frozen candidate stopping MAE (m) | — | 0.500 |" in text
    assert "| Failed stop predictions | 2 | 0 |" in text
    assert "not fully scored" in text
    bundle = json.loads(report.with_suffix(".json").read_text())
    assert bundle["runs"][0]["cases"][0]["candidate_error_m"] is None


def test_no_input_overwrite_or_path_escape_and_cli_works(tmp_path):
    first, second = pair(tmp_path)
    with pytest.raises(ComparisonError):
        export_comparison(first, second, first / "new-comparison")
    with pytest.raises(ComparisonError):
        export_comparison(first, second, first)
    destination = tmp_path / "via-cli"
    assert main([str(first), str(second), str(destination)]) == 0
    assert (destination / "comparison.md").is_file()
    with pytest.raises(ComparisonError):
        export_comparison(first, second, destination)
    metadata = json.loads((second / "metadata.json").read_text())
    metadata["frozen_source_file"] = "../outside/actuator.py"
    write_json(second / "metadata.json", metadata)
    with pytest.raises(ComparisonError):
        export_comparison(first, second, tmp_path / "escape")
