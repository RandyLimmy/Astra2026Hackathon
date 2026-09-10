"""No-network checks for the Astra platform tool entrypoint and audit records."""

from copy import deepcopy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import sys

import httpx2 as httpx
from openai import APIConnectionError
import pytest

from investigation import platform_run


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


def response(output, *, status="completed", model="gpt-6-astra", effort="xhigh"):
    return SimpleNamespace(id="response_test", model=model, reasoning=SimpleNamespace(effort=effort),
                           status=status, output=output, usage=None)


def call(name, arguments):
    return Item(type="function_call", name=name, arguments=json.dumps(arguments), call_id="call_" + name)


SUBMISSION = {"diagnosis": "The observed output differs from the model.",
              "evidence": "One probe was recorded.", "remaining_uncertainty": "Other inputs remain untested."}


class Broker:
    tool_schemas = [{"type": "function", "name": name, "strict": True,
                     "parameters": {"type": "object", "properties": {}, "required": [],
                                    "additionalProperties": False}}
                    for name in ("inspect_model", "run_model", "submit_result")]

    def __init__(self, root):
        self.current_source = root / "broker" / "model.py"
        self.current_source.parent.mkdir(parents=True)
        self.current_source.write_text("def init_state():\n    return {}\n")
        self.submitted = False
        self.calls = []
        self.initial_calls = 0
        self.finalizations = []
        self.physics = SimpleNamespace(scenario="PRIVATE_PRESET_SENTINEL", private_state="PRIVATE_STATE_SENTINEL")
        self.tool_result = {"ok": True, "summary": {"rmse": 0.3},
                            "budget": {"tool_calls": {"used": 1, "limit": 30}}}
        self.final_result = {"agent_submitted": False, "maintenance": [
            {"action": "service_component", "target": "unit_a", "receipt": {"status": "applied"}}],
            "cases": [{"probe": "diagnostic",
                       "before_difference": {"position_rmse_m": 0.3, "observed_outcome": "outside_probe_envelope",
                                             "observed_within_envelope": False},
                       "after_difference": {"position_rmse_m": 0.2, "observed_outcome": "outside_probe_envelope",
                                            "observed_within_envelope": False},
                       "prediction_difference": {"position_rmse_m": 0.1},
                       "after": {"observations": [{"sensor": "TELEMETRY_SENTINEL"}] * 200}}]}

    def initial_evidence(self):
        self.initial_calls += 1
        return {"capabilities": {"probe": "bounded numerical command"},
                "observed": {"summary": {"rmse": 0.3}}, "budget": self.budget_status()}

    def dispatch(self, name, args):
        self.calls.append((name, args))
        if name == "submit_result":
            self.submitted = True
        return self.tool_result

    def budget_status(self):
        return {"tool_calls": {"used": len(self.calls), "limit": 30}}

    def finalize(self, reason):
        self.finalizations.append(reason)
        return {**self.final_result, "agent_submitted": self.submitted}


@pytest.fixture(autouse=True)
def public_contract(tmp_path, monkeypatch):
    path = tmp_path / "public_contract.md"
    path.write_text("Public contract: model parameters and an executable state update.")
    monkeypatch.setattr(platform_run, "CONTRACT", path)


def events(root):
    return [json.loads(line) for line in (root / "events.jsonl").read_text().splitlines()]


def test_pinned_fresh_context_exact_prompts_and_encrypted_continuity(tmp_path):
    broker = Broker(tmp_path)
    reasoning = Item(type="reasoning", id="r1", encrypted_content="ENCRYPTED_SENTINEL",
                     summary=[Item(type="summary_text", text="I will inspect the available evidence.")])
    client = Client([response([reasoning, call("inspect_model", {})]),
                     response([call("submit_result", SUBMISSION)])])
    metadata = platform_run.run_platform_session(client, broker, tmp_path, platform="drone", max_api_requests=2)
    assert broker.initial_calls == 1
    assert metadata["kind"] == "platform_investigation"
    assert metadata["profile"] == "astra-xhigh"
    assert metadata["platform"] == "drone"
    assert metadata["status"] == "completed"
    assert metadata["agent_submitted"] is True
    assert broker.finalizations == ["agent_submission"]
    for request in client.requests:
        assert request["model"] == "gpt-6-astra"
        assert request["reasoning"] == {"effort": "xhigh", "summary": "auto"}
        assert request["store"] is False
        assert request["parallel_tool_calls"] is False
        assert request["tools"] == broker.tool_schemas
        assert "PRIVATE_PRESET_SENTINEL" not in json.dumps(request)
        assert "PRIVATE_STATE_SENTINEL" not in json.dumps(request)
    assert len(client.requests[0]["input"]) == 2
    assert "bounded numerical command" in client.requests[0]["input"][1]["content"]
    assert "ENCRYPTED_SENTINEL" in json.dumps(client.requests[1]["input"])
    persisted = "\n".join(path.read_text() for path in tmp_path.rglob("*") if path.is_file())
    assert "ENCRYPTED_SENTINEL" not in persisted
    assert "I will inspect the available evidence." in persisted
    for index, name in enumerate(("system", "task")):
        saved = (tmp_path / f"prompts/{name}.md").read_text()
        assert saved == client.requests[0]["input"][index]["content"]
        assert metadata[f"{name}_prompt_sha256"] == hashlib.sha256(saved.encode()).hexdigest()
    assert json.loads((tmp_path / "prompts/tools.json").read_text()) == broker.tool_schemas
    assert metadata["protocol_manifest"]["tool_caps"] == {"tool_calls": 30}
    assert len(metadata["protocol_fingerprint"]) == 64
    assert json.loads((tmp_path / "evaluation/result.json").read_text())["cases"][0]["after_difference"]["observed_within_envelope"] is False
    report = (tmp_path / "report.md").read_text()
    assert "not a claim that maintenance or predictions passed" in report
    assert "| diagnostic | 0.3000 | 0.2000 |" in report
    assert "False → False | 0.1000" in report
    assert "| service_component | unit_a | applied |" in report
    assert "TELEMETRY_SENTINEL" not in report
    assert "evaluation/result.json" in report
    sources = metadata["protocol_manifest"]["source_sha256"]
    assert sources["investigation/platform_run.py"] == hashlib.sha256(Path(platform_run.__file__).read_bytes()).hexdigest()
    assert "investigation/platform_physics.py" in sources
    assert "component_worker/platform_runtime.py" in sources
    assert "simulator/platforms/drone.py" in sources
    assert "simulator/assets/platforms/quadruped.xml" in sources
    assert set(metadata["protocol_manifest"]["runtime_versions"]) == {"python", "mujoco", "numpy", "openai", "Pillow"}
    assert "platform_physics.py" not in json.dumps(client.requests[0]["input"])


def test_api_budget_and_exact_arguments_are_sent_and_logged_without_mutation(tmp_path):
    broker = Broker(tmp_path)
    original = deepcopy(broker.tool_result)
    first = call("run_model", {"config": {"speed": 0.4}, "rationale": "Try another command."})
    first.arguments = '{ "config": {"speed": 0.4}, "rationale": "Try another command." }'
    client = Client([response([first]), response([call("submit_result", SUBMISSION)])])
    metadata = platform_run.run_platform_session(client, broker, tmp_path, platform="car", max_api_requests=2)
    assert "at most 2 API requests" in (tmp_path / "prompts/task.md").read_text()
    logged = events(tmp_path)
    assert [event for event in logged if event["type"] == "tool_call"][0]["arguments_json"] == first.arguments
    results = [event["result"] for event in logged if event["type"] == "tool_result"]
    assert results[0]["api_request_budget"] == {"used": 1, "remaining": 1}
    assert "submit_result" in results[0]["submission_reminder"]
    sent = [json.loads(item["output"]) for item in client.requests[1]["input"]
            if item.get("type") == "function_call_output"]
    assert sent == [results[0]]
    assert results[0]["budget"] == original["budget"]
    assert broker.tool_result == original
    assert metadata["api_requests"] == 2


def test_budget_exhaustion_finalizes_without_agent_submission(tmp_path):
    broker = Broker(tmp_path)
    client = Client([response([call("inspect_model", {})])])
    metadata = platform_run.run_platform_session(client, broker, tmp_path, platform="quadruped", max_api_requests=1)
    assert metadata["agent_submitted"] is False
    assert metadata["status"] == "completed"
    assert broker.finalizations == ["api_request_budget"]
    assert len(client.requests) == 1


@pytest.mark.parametrize("model,effort", [("gpt-5.6-sol", "high"), ("gpt-6-astra", "high")])
def test_unconfirmed_profile_cannot_run_tools_or_log_visible_output(tmp_path, model, effort):
    broker = Broker(tmp_path)
    text = Item(type="message", content=[Item(type="output_text", text="WRONG_MODEL_SENTINEL")])
    client = Client([response([text, call("submit_result", SUBMISSION)], model=model, effort=effort)])
    metadata = platform_run.run_platform_session(client, broker, tmp_path, platform="car", max_api_requests=1)
    assert broker.calls == []
    assert broker.finalizations == ["host_error"]
    assert metadata["status"] == "error"
    assert "WRONG_MODEL_SENTINEL" not in (tmp_path / "events.jsonl").read_text()


def test_incomplete_response_does_not_execute_maintenance_or_submission(tmp_path):
    broker = Broker(tmp_path)
    client = Client([response([call("submit_result", SUBMISSION)], status="incomplete")])
    metadata = platform_run.run_platform_session(client, broker, tmp_path, platform="car", max_api_requests=1)
    assert broker.calls == []
    assert metadata["agent_submitted"] is False
    assert broker.finalizations == ["api_response_incomplete"]


def test_api_error_is_neutral_and_completed_verification_is_preserved(tmp_path):
    broker = Broker(tmp_path)
    error = APIConnectionError(message="PRIVATE_API_KEY_SENTINEL", request=httpx.Request("POST", "https://api.openai.com/v1/responses"))
    metadata = platform_run.run_platform_session(Client([error]), broker, tmp_path,
                                                 platform="car", max_api_requests=1)
    assert metadata["status"] == "api_error"
    assert metadata["verification_status"] == "completed"
    assert broker.finalizations == ["api_error"]
    assert (tmp_path / "evaluation/result.json").is_file()
    assert "PRIVATE_API_KEY_SENTINEL" not in (tmp_path / "events.jsonl").read_text()


def test_failed_finalization_is_not_reported_as_success(tmp_path):
    class FailingBroker(Broker):
        def finalize(self, reason):
            raise RuntimeError("PRIVATE_ENGINE_PATH_SENTINEL")
    broker = FailingBroker(tmp_path)
    metadata = platform_run.run_platform_session(Client([response([call("submit_result", SUBMISSION)])]),
                                                 broker, tmp_path, platform="car", max_api_requests=1)
    assert metadata["status"] == "evaluation_error"
    assert metadata["verification_status"] == "error"
    assert not (tmp_path / "evaluation/result.json").exists()
    assert "Fresh verification did not complete" in (tmp_path / "report.md").read_text()
    assert "PRIVATE_ENGINE_PATH_SENTINEL" not in (tmp_path / "events.jsonl").read_text()


def test_malformed_tool_arguments_are_recorded_but_not_dispatched(tmp_path):
    broker = Broker(tmp_path)
    malformed = call("inspect_model", {})
    malformed.arguments = "{not JSON}"
    client = Client([response([malformed])])
    platform_run.run_platform_session(client, broker, tmp_path, platform="car", max_api_requests=1)
    assert broker.calls == []
    record = [event for event in events(tmp_path) if event["type"] == "tool_call"][0]
    assert record["arguments_json"] == "{not JSON}"
    result = [event for event in events(tmp_path) if event["type"] == "tool_result"][0]["result"]
    assert result["ok"] is False


def test_text_only_proposal_gets_one_nudge_then_truthful_host_finalization(tmp_path):
    broker = Broker(tmp_path)
    message = Item(type="message", content=[Item(type="output_text", text="I propose a change.")])
    client = Client([response([message]), response([message])])
    metadata = platform_run.run_platform_session(client, broker, tmp_path, platform="car", max_api_requests=3)
    assert len(client.requests) == 2
    assert metadata["stop_reason"] == "agent_finished_without_submission"
    assert broker.submitted is False
    assert client.requests[1]["input"][-1]["role"] == "user"
    assert "submit_result" in client.requests[1]["input"][-1]["content"]


def test_existing_records_are_not_overwritten(tmp_path):
    broker = Broker(tmp_path)
    path = tmp_path / "metadata.json"
    path.write_text('{"historical":true}')
    client = Client([])
    with pytest.raises(ValueError, match="existing records"):
        platform_run.run_platform_session(client, broker, tmp_path, platform="car")
    assert path.read_text() == '{"historical":true}'
    assert client.requests == []


def test_cli_constructs_host_preset_without_feeding_it_to_session(tmp_path, monkeypatch):
    output = tmp_path / "new-run"
    seen = {}
    class Physics:
        def __init__(self, platform, workdir, **kwargs):
            seen["physics"] = (platform, workdir, kwargs)
    class CLI_Broker:
        def __init__(self, workdir, physics):
            seen["broker"] = (workdir, physics)
    class ContextClient:
        def __enter__(self): return self
        def __exit__(self, *args): pass
    def load(**kwargs):
        seen["settings"] = kwargs
        return SimpleNamespace(client=ContextClient)
    def run(client, broker, run_dir, **kwargs):
        seen["session"] = (run_dir, kwargs)
        return {"status": "completed"}
    monkeypatch.setattr(platform_run.Settings, "load", load)
    monkeypatch.setattr(platform_run, "run_platform_session", run)
    monkeypatch.setitem(sys.modules, "investigation.platform_physics", SimpleNamespace(PlatformPhysics=Physics))
    monkeypatch.setitem(sys.modules, "investigation.platform_broker", SimpleNamespace(PlatformBroker=CLI_Broker))
    assert platform_run.main(["--platform", "drone", "--scenario", "PRIVATE_PRESET_SENTINEL",
                              "--output", str(output), "--no-frames"]) == 0
    assert seen["settings"] == {"profile": "astra-xhigh"}
    assert seen["physics"] == ("drone", output / "physics", {"scenario": "PRIVATE_PRESET_SENTINEL", "record_frames": False})
    assert seen["broker"][0] == output / "broker"
    assert seen["session"] == (output, {"platform": "drone", "max_api_requests": 16, "max_seconds": 1800})


@pytest.mark.parametrize("request_count", [0, 21])
def test_cli_rejects_out_of_range_budgets_before_loading_credentials(tmp_path, monkeypatch, request_count):
    def forbidden(**kwargs):
        raise AssertionError("Configuration must not be loaded")
    monkeypatch.setattr(platform_run.Settings, "load", forbidden)
    with pytest.raises(SystemExit) as error:
        platform_run.main(["--platform", "car", "--output", str(tmp_path / "new"),
                           "--max-api-requests", str(request_count)])
    assert error.value.code == 1
