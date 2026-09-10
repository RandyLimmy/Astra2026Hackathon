"""API configuration, stateless tool continuity and secret-safe pilot records."""

from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from investigation.api import MODEL, Settings, request_response
from investigation.runner import protocol_identity, run_session, safe_api_error


class Item(SimpleNamespace):
    def model_dump(self, **kwargs):
        def encode(value):
            if isinstance(value, SimpleNamespace):
                return {key: encode(item) for key, item in vars(value).items()}
            if isinstance(value, list):
                return [encode(item) for item in value]
            return value
        return encode(self)


class Client:
    def __init__(self, replies):
        self.replies = iter(replies)
        self.requests = []
        self.responses = self

    def create(self, **kwargs):
        self.requests.append(deepcopy(kwargs))
        reply = next(self.replies)
        if isinstance(reply, BaseException):
            raise reply
        return reply


class Broker:
    def __init__(self, root):
        self.source = root / "broker" / "actuator.py"
        self.source.parent.mkdir(parents=True)
        self.source.write_text("def init_state():\n    return {}\n")
        self.frozen_source = None
        self.calls = []

    def initial_evidence(self):
        return {"cases": [], "budget": self.budget_status()}

    def dispatch(self, name, args):
        self.calls.append((name, args))
        if name == "submit_prediction":
            self.frozen_source = self.source.resolve()
        return {"ok": True, "source": self.source.read_text()}

    def freeze(self):
        self.frozen_source = self.source.resolve()

    def budget_status(self):
        return {"tool_calls": {"used": len(self.calls), "limit": 30}}


def response(output, *, status="completed", model=MODEL, reasoning_effort="medium"):
    return SimpleNamespace(id="response_test", model=model, reasoning=SimpleNamespace(effort=reasoning_effort),
                           status=status, output=output, usage=None)


def call(name, arguments):
    return Item(type="function_call", name=name, arguments=json.dumps(arguments), call_id="call_" + name)


def test_every_request_pins_astra_medium_and_preserves_stateless_reasoning():
    client = Client([None])
    request_response(client, [{"role": "user", "content": "test"}], [])
    actual = client.requests[0]
    assert actual["model"] == "gpt-6-astra"
    assert actual["reasoning"] == {"effort": "medium", "summary": "auto"}
    assert actual["store"] is False
    assert actual["include"] == ["reasoning.encrypted_content"]
    assert actual["parallel_tool_calls"] is False
    assert "temperature" not in actual


def test_config_uses_one_key_without_exposing_it_in_repr(tmp_path, monkeypatch):
    for name in ("OPENAI_API_KEY", "ASTRA_MODEL", "ASTRA_REASONING_EFFORT"):
        monkeypatch.delenv(name, raising=False)
    path = tmp_path / ".env"
    path.write_text("OPENAI_API_KEY=synthetic-secret-only\nASTRA_MODEL=gpt-6-astra\nASTRA_REASONING_EFFORT=medium\n")
    settings = Settings.load(path)
    assert settings.api_key == "synthetic-secret-only"
    assert "synthetic-secret-only" not in repr(settings)
    path.write_text("OPENAI_API_KEY=synthetic-secret-only\nASTRA_REASONING_EFFORT=low\n")
    with pytest.raises(ValueError, match="medium"):
        Settings.load(path)


def test_remote_text_cannot_leak_through_api_error_logging():
    class Failure(Exception):
        status_code = 401
    safe = safe_api_error(Failure("Incorrect key: synthetic-secret-only"))
    assert "synthetic-secret-only" not in safe
    assert "401" in safe


def test_fresh_session_replays_tool_outputs_and_encrypted_items_without_logging_blob(tmp_path):
    broker = Broker(tmp_path)
    reasoning = Item(type="reasoning", id="reasoning_test", encrypted_content="ENCRYPTED_SENTINEL",
                     summary=[Item(type="summary_text", text="I will compare the available measurements.")])
    client = Client([response([reasoning, call("inspect_model", {})]),
                     response([call("submit_prediction", {"rationale": "Ready for a new test."})])])
    metadata = run_session(client, broker, tmp_path, max_api_requests=2)
    assert metadata["agent_submitted"] is True
    assert metadata["freeze_reason"] == "agent_submission"
    assert metadata["api_requests"] == 2
    assert metadata["frozen_source_file"] == "broker/actuator.py"
    assert "ENCRYPTED_SENTINEL" in json.dumps(client.requests[1]["input"])
    assert any(item.get("type") == "function_call_output" for item in client.requests[1]["input"])
    persisted = "\n".join(path.read_text() for path in tmp_path.rglob("*") if path.is_file())
    assert "ENCRYPTED_SENTINEL" not in persisted
    assert "I will compare the available measurements." in persisted
    assert (tmp_path / "prompts/system.md").read_text() == client.requests[0]["input"][0]["content"]
    assert (tmp_path / "prompts/task.md").read_text() == client.requests[0]["input"][1]["content"]


def test_budget_freeze_is_not_reported_as_agent_submission(tmp_path):
    broker = Broker(tmp_path)
    metadata = run_session(Client([response([call("inspect_model", {})])]), broker, tmp_path,
                           max_api_requests=1)
    assert broker.frozen_source is not None
    assert metadata["agent_submitted"] is False
    assert metadata["freeze_reason"] == "request_or_time_budget"


def test_api_budget_reaches_agent_and_log_without_changing_broker_results(tmp_path):
    class BudgetBroker(Broker):
        def __init__(self, root):
            super().__init__(root)
            self.original_result = {"ok": True, "summary": {"distance": 12.0},
                                    "budget": {"tool_calls": {"used": 7, "limit": 30},
                                               "run_model": {"used": 2, "limit": 12}}}

        def dispatch(self, name, args):
            super().dispatch(name, args)
            return self.original_result

    broker = BudgetBroker(tmp_path)
    original = deepcopy(broker.original_result)
    client = Client([response([call("inspect_model", {})]),
                     response([call("run_model", {"config": {}, "rationale": "Measure a prediction."})]),
                     response([call("inspect_model", {})]),
                     response([call("submit_prediction", {"rationale": "Ready to submit."})])])
    metadata = run_session(client, broker, tmp_path, max_api_requests=4)
    saved_task = (tmp_path / "prompts/task.md").read_text()
    assert saved_task == client.requests[0]["input"][1]["content"]
    assert "at most 4 API requests" in saved_task
    assert "separate from the broker's tool-call and experiment budgets" in saved_task
    assert "submit_prediction" in saved_task
    logged = [json.loads(line) for line in (tmp_path / "events.jsonl").read_text().splitlines()]
    outputs = [event["result"] for event in logged if event["type"] == "tool_result"]
    assert len(outputs) == 4
    for index, output in enumerate(outputs):
        assert output["api_request_budget"] == {"used": index + 1, "remaining": 3 - index}
        for key, value in original.items():
            assert output[key] == value
        if index == 0:
            assert "submission_reminder" not in output
        elif index < 3:
            assert "submit_prediction" in output["submission_reminder"]
        else:
            assert "source is frozen" in output["submission_reminder"]
        if index < 3:
            sent = [json.loads(item["output"]) for item in client.requests[index + 1]["input"]
                    if item.get("type") == "function_call_output"]
            assert sent[-1] == output
    assert broker.original_result == original
    assert metadata["agent_submitted"] is True
    assert metadata["api_requests"] == 4


def test_incomplete_tool_call_is_not_executed(tmp_path):
    broker = Broker(tmp_path)
    metadata = run_session(Client([response([call("patch_model", {"diff": "incomplete"})], status="incomplete")]),
                           broker, tmp_path, max_api_requests=1)
    assert broker.calls == []
    assert metadata["freeze_reason"] == "api_response_incomplete"
    assert metadata["agent_submitted"] is False


def test_prompts_do_not_disclose_private_backend_or_solution():
    root = Path(__file__).resolve().parents[1]
    text = "\n".join(path.read_text() for path in (root / "investigation/prompts").glob("*.md")).lower()
    for hint in ("mujoco", "temperature", "thermal", "brake fade", "developer_wheel_actuator", "reference_host"):
        assert hint not in text


def evaluated_result():
    """Host records contain extra fields that must not enter the debrief."""
    return {
        "aggregate": {"candidate_mae_m": 0.75, "original_mae_m": 9.0, "scoring_complete": True},
        "source_sha256": "HOST_SOURCE_HASH_SENTINEL",
        "state_extension": {"private_record": "PRIVATE_STATE_SENTINEL"},
        "cases": [{
            "case_id": "reserved_public_case",
            "config": {"speed_mps": 22.0, "brake_strength": 1.0, "preparation_cycles": 2,
                       "wait_s": 0.0, "wall_distance_m": None},
            "reference": {"summary": {"stopped": True, "stopping_distance": 47.25, "collision": False},
                          "diagnostics": {"private_value": "PRIVATE_REFERENCE_SENTINEL"}},
            "candidate": {"summary": {"stopped": True, "stopping_distance": 46.5, "collision": False},
                          "candidate_state": {"value": "PRIVATE_CANDIDATE_SENTINEL"}},
            "original": {"summary": {"stopping_distance": 38.25}, "internal": "PRIVATE_ORIGINAL_SENTINEL"},
            "original_error_m": 9.0, "candidate_error_m": 0.75, "previously_observed": False,
            "candidate_error": "/host/PRIVATE_ERROR_SENTINEL",
            "full_records_directory": "/host/PRIVATE_PATH_SENTINEL",
        }],
    }


def evaluator_saving(result, broker):
    def evaluate(source, output):
        assert broker.frozen_source is not None
        assert source == broker.frozen_source
        output.mkdir(parents=True)
        (output / "result.json").write_text(json.dumps(result))
        return result
    return evaluate


def test_debrief_only_receives_frozen_public_feedback_with_tools_disabled(tmp_path):
    broker = Broker(tmp_path)
    initial_source = broker.source.read_bytes()
    reasoning = Item(type="reasoning", id="initial_reasoning", encrypted_content="INITIAL_ENCRYPTED_SENTINEL",
                     summary=[Item(type="summary_text", text="The component is ready to freeze.")])
    debrief_reasoning = Item(type="reasoning", id="debrief_reasoning", encrypted_content="DEBRIEF_ENCRYPTED_SENTINEL",
                             summary=[Item(type="summary_text", text="I will interpret the observed errors.")])
    debrief_text = Item(type="message", content=[Item(type="output_text", text="The measured prediction error was 0.75 m.")])
    client = Client([
        response([reasoning, call("submit_prediction", {"rationale": "Freeze this source."})]),
        # Even an unexpected provider tool item must not reopen execution.
        response([debrief_reasoning, debrief_text, call("patch_model", {"diff": "must not execute"})]),
    ])
    result = evaluated_result()
    metadata = run_session(client, broker, tmp_path, max_api_requests=2,
                           evaluate=evaluator_saving(result, broker))
    assert metadata["status"] == "completed"
    assert metadata["debrief_status"] == "completed"
    assert metadata["agent_submitted"] is True
    assert metadata["api_requests"] == 2
    assert broker.calls == [("submit_prediction", {"rationale": "Freeze this source."})]
    assert broker.source.read_bytes() == initial_source
    final = client.requests[1]
    assert final["tool_choice"] == "none"
    assert final["max_output_tokens"] == 2048
    for request in client.requests:
        assert request["model"] == "gpt-6-astra"
        assert request["reasoning"] == {"effort": "medium", "summary": "auto"}
        assert request["store"] is False
        assert request["parallel_tool_calls"] is False
    content = final["input"][-1]["content"]
    feedback, _ = json.JSONDecoder().raw_decode(content[content.index("{"):])
    case = result["cases"][0]
    assert feedback == {"aggregate": result["aggregate"], "cases": [{
        "id": case["case_id"], "config": case["config"], "reference": case["reference"]["summary"],
        "prediction": case["candidate"]["summary"], "original_error_m": 9.0,
        "candidate_error_m": 0.75, "previously_observed": False,
    }]}
    assert "PRIVATE_" not in json.dumps(final["input"])
    assert "HOST_SOURCE_HASH_SENTINEL" not in json.dumps(final["input"])
    assert "reserved_public_case" not in json.dumps(client.requests[0]["input"])
    assert "INITIAL_ENCRYPTED_SENTINEL" in json.dumps(final["input"])
    persisted = "\n".join(path.read_text() for path in tmp_path.rglob("*") if path.is_file())
    assert "INITIAL_ENCRYPTED_SENTINEL" not in persisted
    assert "DEBRIEF_ENCRYPTED_SENTINEL" not in persisted
    assert "The measured prediction error was 0.75 m." in persisted
    assert json.loads((tmp_path / "evaluation/result.json").read_text()) == result


@pytest.mark.parametrize("failure_kind", ["api", "wrong_model", "wrong_effort"])
def test_debrief_failure_preserves_completed_evaluation_and_final_metadata(tmp_path, failure_kind):
    broker = Broker(tmp_path)
    if failure_kind == "api":
        from openai import APIError
        debrief = APIError("PRIVATE_PROVIDER_ERROR_SENTINEL",
                           request=SimpleNamespace(method="POST", url="https://example.invalid/responses"), body=None)
        expected_status = "api_error"
    else:
        debrief = response([Item(type="message", content=[Item(type="output_text", text="UNVERIFIED_DEBRIEF_SENTINEL")])])
        if failure_kind == "wrong_model":
            debrief.model = "unexpected-model"
        else:
            debrief.reasoning.effort = "low"
        expected_status = "error"
    client = Client([response([call("submit_prediction", {"rationale": "Ready."})]), debrief])
    result = evaluated_result()
    metadata = run_session(client, broker, tmp_path, max_api_requests=2,
                           evaluate=evaluator_saving(result, broker))
    assert metadata["status"] == "completed"
    assert metadata["debrief_status"] == expected_status
    assert metadata["api_requests"] == 2
    assert metadata["end_at"] is not None
    assert json.loads((tmp_path / "metadata.json").read_text()) == metadata
    assert json.loads((tmp_path / "evaluation/result.json").read_text()) == result
    logs = (tmp_path / "events.jsonl").read_text()
    assert "PRIVATE_PROVIDER_ERROR_SENTINEL" not in logs
    assert "UNVERIFIED_DEBRIEF_SENTINEL" not in logs
    assert len(broker.calls) == 1


@pytest.mark.parametrize("malformation", ["missing_summary", "nonfinite_feedback"])
def test_malformed_debrief_feedback_cannot_discard_completed_evaluation(tmp_path, malformation):
    broker = Broker(tmp_path)
    result = evaluated_result()
    if malformation == "missing_summary":
        del result["cases"][0]["candidate"]["summary"]
    else:
        result["aggregate"]["candidate_mae_m"] = float("nan")
    client = Client([response([call("submit_prediction", {"rationale": "Ready."})])])
    # Custom log permits the intentionally malformed feedback fixture; production
    # evaluation artifacts are separately validated before reaching this point.
    events = []
    metadata = run_session(client, broker, tmp_path, max_api_requests=2,
                           evaluate=evaluator_saving(result, broker),
                           log=lambda kind, **values: events.append({"type": kind, **values}))
    assert metadata["status"] == "completed"
    assert metadata["debrief_status"] == "error"
    assert metadata["end_at"] is not None
    assert metadata["api_requests"] == 1
    assert len(client.requests) == 1
    assert (tmp_path / "evaluation/result.json").exists()
    assert json.loads((tmp_path / "metadata.json").read_text()) == metadata
    assert any(event["type"] == "error" and "evaluation results are preserved" in event.get("message", "") for event in events)


def test_debrief_respects_exhausted_request_budget(tmp_path):
    broker = Broker(tmp_path)
    client = Client([response([call("submit_prediction", {"rationale": "Ready."})])])
    metadata = run_session(client, broker, tmp_path, max_api_requests=1,
                           evaluate=evaluator_saving(evaluated_result(), broker))
    assert metadata["status"] == "completed"
    assert metadata["api_requests"] == 1
    assert "debrief_status" not in metadata
    assert len(client.requests) == 1


def test_sol_profile_uses_matching_environment_names_and_one_shared_key(tmp_path, monkeypatch):
    for name in ("OPENAI_API_KEY", "SOL_MODEL", "SOL_REASONING_EFFORT"):
        monkeypatch.delenv(name, raising=False)
    monkeypatch.setenv("ASTRA_MODEL", "ignored-other-profile")
    monkeypatch.setenv("ASTRA_REASONING_EFFORT", "low")
    path = tmp_path / "profiles.env"
    path.write_text("OPENAI_API_KEY=synthetic-shared-key\nASTRA_MODEL=also-ignored\n")
    settings = Settings.load(path, profile="sol-high")
    assert settings.profile == "sol-high"
    assert settings.model == "gpt-5.6-sol"
    assert settings.reasoning_effort == "high"
    assert settings.api_key == "synthetic-shared-key"
    assert "synthetic-shared-key" not in repr(settings)
    assert Settings("synthetic-shared-key", profile="sol-high").model == "gpt-5.6-sol"
    path.write_text("OPENAI_API_KEY=synthetic-shared-key\nSOL_REASONING_EFFORT=medium\n")
    with pytest.raises(ValueError, match="high"):
        Settings.load(path, profile="sol-high")
    with pytest.raises(ValueError, match="gpt-5.6-sol"):
        Settings("synthetic-shared-key", model=MODEL, reasoning_effort="high", profile="sol-high")


def test_sol_request_pins_high_without_changing_stateless_or_tool_policy():
    client = Client([None])
    request_response(client, [{"role": "user", "content": "test"}], [], profile="sol-high", final=True)
    actual = client.requests[0]
    assert actual["model"] == "gpt-5.6-sol"
    assert actual["reasoning"] == {"effort": "high", "summary": "auto"}
    assert actual["tool_choice"] == "none"
    assert actual["store"] is False
    assert actual["include"] == ["reasoning.encrypted_content"]
    assert actual["parallel_tool_calls"] is False
    with pytest.raises(ValueError, match="profile"):
        request_response(client, [], [], profile="unknown")
    assert len(client.requests) == 1


def test_cli_explicit_sol_profile_reaches_settings_and_request(tmp_path, monkeypatch, capsys):
    from contextlib import nullcontext
    from investigation.__main__ import main
    for name in ("SOL_MODEL", "SOL_REASONING_EFFORT"):
        monkeypatch.delenv(name, raising=False)
    path = tmp_path / "settings.env"
    path.write_text("OPENAI_API_KEY=synthetic-shared-key\nASTRA_REASONING_EFFORT=invalid-for-astra\n")
    reply = response([], model="gpt-5.6-sol", reasoning_effort="high")
    reply.output_text = "API_READY"
    client = Client([reply])
    monkeypatch.setattr(Settings, "client", lambda self: nullcontext(client))
    assert main(["--profile", "sol-high", "--check-key", "--env-file", str(path)]) == 0
    assert client.requests[0]["model"] == "gpt-5.6-sol"
    assert client.requests[0]["reasoning"]["effort"] == "high"
    printed = json.loads(capsys.readouterr().out)
    assert printed["profile"] == "sol-high"
    assert printed["reply"] == "API_READY"
    assert "synthetic-shared-key" not in json.dumps(printed)


def test_sol_session_and_debrief_verify_selected_pair_and_use_generic_labels(tmp_path, capsys):
    broker = Broker(tmp_path)
    sol_response = lambda output: response(output, model="gpt-5.6-sol", reasoning_effort="high")
    client = Client([sol_response([call("submit_prediction", {"rationale": "Ready."})]),
                     sol_response([Item(type="message", content=[Item(type="output_text", text="Measured results reviewed.")])])])
    metadata = run_session(client, broker, tmp_path, max_api_requests=2, profile="sol-high",
                           evaluate=evaluator_saving(evaluated_result(), broker))
    assert metadata["status"] == "completed"
    assert metadata["debrief_status"] == "completed"
    assert metadata["profile"] == "sol-high"
    assert metadata["model"] == "gpt-5.6-sol"
    assert metadata["reasoning_effort"] == "high"
    assert len(metadata["protocol_fingerprint"]) == 64
    for request in client.requests:
        assert request["model"] == "gpt-5.6-sol"
        assert request["reasoning"]["effort"] == "high"
    assert client.requests[1]["tool_choice"] == "none"
    printed = capsys.readouterr().out
    assert "Investigator tool:" in printed
    assert "Investigator:" in printed
    assert "Astra" not in printed


@pytest.mark.parametrize("returned_model,returned_effort", [(MODEL, "medium"), ("gpt-5.6-sol", "medium")])
def test_sol_session_rejects_wrong_model_or_reasoning_without_fallback(tmp_path, returned_model, returned_effort):
    broker = Broker(tmp_path)
    client = Client([response([call("submit_prediction", {"rationale": "Do not execute."})],
                              model=returned_model, reasoning_effort=returned_effort)])
    metadata = run_session(client, broker, tmp_path, max_api_requests=1, profile="sol-high")
    assert metadata["status"] == "error"
    assert metadata["freeze_reason"] == "host_error"
    assert broker.calls == []
    assert broker.frozen_source is None
    assert len(client.requests) == 1
    assert client.requests[0]["model"] == "gpt-5.6-sol"
    assert client.requests[0]["reasoning"]["effort"] == "high"


def test_protocol_fingerprint_matches_across_profiles_and_excludes_run_identity(tmp_path):
    results = []
    for profile, model, effort in (("astra-medium", MODEL, "medium"), ("sol-high", "gpt-5.6-sol", "high")):
        directory = tmp_path / profile
        broker = Broker(directory)
        client = Client([response([call("submit_prediction", {"rationale": "Ready."})],
                                  model=model, reasoning_effort=effort)])
        results.append(run_session(client, broker, directory, max_api_requests=1, profile=profile))
    assert results[0]["profile"] != results[1]["profile"]
    assert results[0]["protocol_fingerprint"] == results[1]["protocol_fingerprint"]
    assert results[0]["protocol_manifest"] == results[1]["protocol_manifest"]
    serialized = json.dumps(results[0]["protocol_manifest"])
    assert "astra-medium" not in serialized
    assert "sol-high" not in serialized
    assert str(tmp_path) not in serialized
    assert "run_id" not in serialized
    hashes = results[0]["protocol_manifest"]["source_sha256"]
    assert {"investigation/prompts/system.md", "investigation/prompts/task.md", "contracts/WHEEL_ACTUATOR.md",
            "investigation/api.py", "investigation/broker.py", "investigation/runner.py", "investigation/physics.py",
            "investigation/evaluation.py", "candidate/wheel_actuator.py"} <= set(hashes)
    assert results[0]["protocol_manifest"]["runtime_versions"]["openai"]
    assert protocol_identity(max_api_requests=2)["fingerprint"] != protocol_identity(max_api_requests=1)["fingerprint"]
    assert protocol_identity(max_seconds=1801)["fingerprint"] != protocol_identity(max_seconds=1800)["fingerprint"]
