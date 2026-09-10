import hashlib
import json

from investigation.report import build_report


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value))


def case(case_id, original, candidate, reference, *, observed=False, censored=False):
    def run(distance):
        return {"summary": {"stopping_distance": distance, "stopped": distance is not None,
                            "censored": censored and distance is None}}
    return {"case_id": case_id, "original": run(original), "candidate": run(candidate),
            "reference": run(reference),
            "original_error_m": abs(original - reference) if original is not None and reference is not None else None,
            "candidate_error_m": abs(candidate - reference) if candidate is not None and reference is not None else None,
            "previously_observed": observed}


def test_completed_report_uses_measured_cases_attributed_actions_and_frozen_sources(tmp_path):
    write_json(tmp_path / "metadata.json", {
        "model": "gpt-6-astra", "reasoning_effort": "medium", "status": "completed",
        "agent_submitted": True, "api_requests": 3,
        "usage": {"input_tokens": 1000, "output_tokens": 400, "reasoning_tokens": 80, "total_tokens": 1400},
    })
    events = [
        {"type": "assistant_message", "timestamp": "12:00", "text": "I expect the transient response to depend on prior use."},
        {"type": "reasoning_summary", "timestamp": "12:01", "text": "A matched rest experiment would distinguish the hypotheses."},
        {"type": "tool_call", "timestamp": "12:02", "name": "run_experiment", "arguments": {"case_id": "probe_1"}},
        {"type": "tool_result", "timestamp": "12:03", "name": "run_experiment", "result": {"stopping_distance": 31.0}},
        {"type": "api_response", "response_id": "resp_fixture_123", "model": "gpt-6-astra", "reasoning_effort": "medium"},
    ]
    (tmp_path / "events.jsonl").write_text("\n".join(json.dumps(event) for event in events) + "\n")
    evaluation = {
        "kind": "frozen_synthetic_evaluation", "source_sha256": "abc123",
        "aggregate": {"original_mae_m": 8.0, "candidate_mae_m": 1.5, "scored_cases": 2, "failed_stop_predictions": 0},
        "cases": [case("reserved_1", 20, 26, 25), case("reserved_2", 20, 29, 31)],
        "state_extension": {"has_persistent_state": True},
    }
    write_json(tmp_path / "evaluation/result.json", evaluation)
    original = tmp_path / "evaluation/original_candidate.py"
    candidate = tmp_path / "evaluation/frozen_candidate.py"
    original.write_text("def init_state():\n    return {}\n")
    candidate.write_text("def init_state():\n    return {'memory': 0.0}\n")
    (tmp_path / "prompts").mkdir()
    (tmp_path / "prompts/system.md").write_text("Exact system prompt fixture.")
    (tmp_path / "prompts/task.md").write_text("Exact task prompt fixture.")
    before = {path: hashlib.sha256(path.read_bytes()).hexdigest() for path in tmp_path.rglob("*") if path.is_file()}

    output = build_report(tmp_path)
    text = output.read_text()
    assert "**8.000 m**" in text and "**1.500 m**" in text
    assert "`gpt-6-astra`" in text and "`medium`" in text
    assert "| reserved_1 | 20.000 | 26.000 | 25.000 | 5.000 | 1.000 | no |" in text
    assert "Candidate error decreased on 2 of 2 comparable cases" in text
    assert "Brief API-provided reasoning summary" in text
    assert "These are the investigator’s stated explanations and brief API-provided summaries." in text
    assert "> A matched rest experiment would distinguish the hypotheses." in text
    assert "Tool call: `run_experiment`" in text
    assert "[Frozen candidate](evaluation/frozen_candidate.py)" in text
    assert "+    return {'memory': 0.0}" in text
    assert "[Exact system prompt](prompts/system.md)" in text
    assert "usage.reasoning_tokens | 80" in text
    assert "resp_fixture_123" in text
    assert (tmp_path / "evaluation.png").read_bytes().startswith(b"\x89PNG")
    assert all(hashlib.sha256(path.read_bytes()).hexdigest() == digest for path, digest in before.items())


def test_api_failure_and_truncated_log_produce_honest_partial_report(tmp_path):
    write_json(tmp_path / "metadata.json", {"status": "failed", "model": "gpt-6-astra", "reasoning_effort": "medium",
                                           "agent_submitted": False, "freeze_reason": "API request failed"})
    (tmp_path / "events.jsonl").write_text(json.dumps({"type": "error", "message": "Request timed out."}) + '\n{"type":')
    text = build_report(tmp_path).read_text()
    assert "No completed case evaluation is recorded" in text
    assert "did **not** explicitly submit" in text
    assert "Request timed out." in text
    assert "Skipped malformed events.jsonl line 2" in text
    assert "No frozen source comparison was recorded" in text
    assert not (tmp_path / "evaluation.png").exists()


def test_censoring_and_worsening_are_preserved_without_zero_imputation(tmp_path):
    write_json(tmp_path / "evaluation/result.json", {
        "cases": [case("censored", 22, None, None, censored=True),
                  case("worse", 29, 20, 30, observed=True)],
        "aggregate": {"failed_stop_predictions": 1},
    })
    text = build_report(tmp_path).read_text()
    assert "| censored | 22.000 | — (censored) | — (censored) | — | — | no |" in text
    assert "increased on 1" in text
    assert "1 cases lack comparable uncensored" in text
    assert "Cases marked previously observed cannot support an unseen-case claim" in text
    assert "Recorded failed stop predictions: 1" in text


def test_event_evaluation_fallback_works_when_file_is_missing(tmp_path):
    result = {"cases": [case("event_only", None, None, None, censored=True)]}
    (tmp_path / "events.jsonl").write_text(json.dumps({"type": "evaluation", "result": result}) + "\n")
    text = build_report(tmp_path).read_text()
    assert "event_only" in text
    assert "Evaluation recorded 1 cases" in text
    assert not (tmp_path / "evaluation.png").exists()


def test_empty_run_is_partial_and_does_not_invent_model_or_activity(tmp_path):
    text = build_report(tmp_path).read_text()
    assert "**Model:** `not recorded`" in text
    assert "No visible investigation activity was recorded" in text
    assert "No completed case evaluation is recorded" in text


def test_predeclared_failure_is_clear_even_when_aggregate_error_improves(tmp_path):
    write_json(tmp_path / "evaluation/result.json", {
        "predeclared_criteria": {"minimum_improvement_fraction": 0.5, "cold_control_tolerance_m": 0.5},
        "aggregate": {"original_mae_m": 5.0, "candidate_mae_m": 2.0, "scored_cases": 1,
                      "predictive_success": False, "scoring_complete": True,
                      "improvement_pass": True, "per_case_pass": True,
                      "cold_control_pass": False, "collision_outcomes_match": True},
        "cases": [case("reserved_1", 25, 28, 30)],
        "state_extension": {"has_persistent_state": True},
    })
    text = build_report(tmp_path).read_text()
    assert "**Predeclared prediction criteria: failed.**" in text
    assert "| Cold control retained | failed |" in text
    assert "| Aggregate error reduction | passed |" in text
    assert "Prediction checks and source-extension indicators are reported separately." in text
    assert "minimum improvement fraction | 0.5" in text


def test_partial_run_links_saved_versions_without_claiming_they_were_frozen(tmp_path):
    original = tmp_path / "broker/versions/v000/actuator.py"
    latest = tmp_path / "broker/versions/v001/actuator.py"
    original.parent.mkdir(parents=True)
    latest.parent.mkdir(parents=True)
    original.write_text("def init_state():\n    return {}\n")
    latest.write_text("def init_state():\n    return {'memory': 0.0}\n")
    text = build_report(tmp_path).read_text()
    assert "[Latest saved candidate (not frozen)](broker/versions/v001/actuator.py)" in text
    assert "+    return {'memory': 0.0}" in text
    assert "No completed case evaluation is recorded" in text


def test_per_case_runtime_failures_and_full_trace_links_are_visible(tmp_path):
    failed = case("reserved_failure", None, None, 30, censored=True)
    failed.update(candidate_error="WorkerError: component update failed at source line 12",
                  original_error="WorkerTimeout: original model exceeded its deadline",
                  full_records_directory="cases/reserved_failure")
    write_json(tmp_path / "evaluation/result.json", {"cases": [failed]})
    for name in ("original", "candidate", "reference"):
        write_json(tmp_path / "evaluation/cases/reserved_failure" / f"{name}.json", failed[name])
    text = build_report(tmp_path).read_text()
    assert "**reserved_failure · Candidate runtime failure:** WorkerError: component update failed at source line 12" in text
    assert "**reserved_failure · Original runtime failure:** WorkerTimeout: original model exceeded its deadline" in text
    assert "[Candidate](evaluation/cases/reserved_failure/candidate.json)" in text
    assert "[Reference](evaluation/cases/reserved_failure/reference.json)" in text
    assert "| reserved_failure | — (censored) | — (censored) | 30.000 | — | — | no |" in text


def test_trace_links_do_not_escape_run_directory_or_invent_missing_files(tmp_path):
    outside = case("outside", None, None, None)
    outside["full_records_directory"] = "../../../unrelated"
    missing = case("missing", None, None, None)
    missing["full_records_directory"] = "cases/missing"
    write_json(tmp_path / "evaluation/result.json", {"cases": [outside, missing]})
    text = build_report(tmp_path).read_text()
    assert "Full recorded traces" not in text
    assert "unrelated" not in text


def test_sol_report_uses_recorded_profile_without_astra_attribution(tmp_path):
    write_json(tmp_path / "metadata.json", {
        "model": "gpt-5.6-sol", "reasoning_effort": "high", "profile": "sol-high",
        "protocol_fingerprint": "same-protocol-fixture", "status": "completed", "agent_submitted": True,
    })
    (tmp_path / "events.jsonl").write_text(json.dumps({"type": "assistant_message", "text": "I checked the measured response."}) + "\n")
    text = build_report(tmp_path).read_text()
    assert text.startswith("# Investigator report\n")
    assert "`gpt-5.6-sol`" in text and "`high`" in text
    assert "| profile | sol-high |" in text
    assert "same-protocol-fixture" in text
    assert "the investigator’s stated explanations" in text
    assert "Astra" not in text
