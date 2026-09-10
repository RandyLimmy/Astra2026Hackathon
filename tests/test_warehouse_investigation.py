"""Declarative repair tests; the solution below is a DEVELOPER FIXTURE, not GPT output."""
import json
from copy import deepcopy
from dataclasses import replace

import numpy as np
import pytest

from investigation import warehouse as repair


@pytest.fixture
def developer_fixture():
    """Known physical law used only to test expressiveness and evaluation plumbing."""
    return {"schema_version": 1, "model_edits": [], "rules": [{
        "when": {"signal": "equality_force", "name": "cargo_latch", "absolute": True,
                 "comparison": "gte", "threshold": 5., "after_s": .5, "sustained_s": .01},
        "updates": [{"target": "equality", "name": "cargo_latch", "field": "active", "value": False}],
        "once": True}]}


def test_seed_is_unrepaired():
    assert repair.load_candidate(repair.SEED) == repair.EMPTY_MODEL


@pytest.mark.parametrize("edit", [
    {"target": "body", "name": "cargo", "field": "pos", "value": [0, 1, 0]},
    {"target": "equality", "name": "cargo_latch", "field": "active", "value": False},
    {"target": "joint", "name": "cargo_slide", "field": "damping", "value": -1},
    {"target": "joint", "name": "cargo_slide", "field": "damping", "value": True},
    {"target": "joint", "name": "cargo_slide", "field": "axis", "value": [0, 0, 0]},
    {"target": "joint", "name": "cargo_slide", "field": "range", "value": [1, 2]},
    {"target": "motor", "name": "left", "field": "gear", "value": 100},
    {"target": "include", "name": "/etc/passwd", "field": "file", "value": "/etc/passwd"},
    {"target": "geom", "name": "cargo_box", "field": "mass", "value": float("nan")},
])
def test_candidate_rejects_unsafe_edits(edit):
    candidate = deepcopy(repair.EMPTY_MODEL)
    candidate["model_edits"] = [edit]
    with pytest.raises(ValueError):
        repair.candidate_xml(repair.public_config({}), candidate)


def test_allowed_numeric_edits_only_affect_candidate():
    candidate = deepcopy(repair.EMPTY_MODEL)
    candidate["model_edits"] = [
        {"target": "joint", "name": "cargo_slide", "field": "damping", "value": 7},
        {"target": "geom", "name": "cargo_box", "field": "mass", "value": 20},
        {"target": "motor", "name": "left", "field": "gear", "value": 4}]
    config = repair.public_config({})
    simulation = repair.CandidateSimulation(config, candidate)
    untouched = repair.CandidateSimulation(config, repair.EMPTY_MODEL)
    assert simulation.model.body("cargo").mass[0] == 20
    assert untouched.model.body("cargo").mass[0] == 16
    assert simulation.model.actuator("left").gear[0] == 4
    assert untouched.model.actuator("left").gear[0] == 3
    assert candidate["model_edits"][0]["value"] == 7


@pytest.mark.parametrize("change", [
    {"once": False},
    {"when": {"signal": "hidden_fault"}},
    {"updates": [{"target": "body", "name": "cargo", "field": "qpos", "value": 0}]},
])
def test_rejects_nonphysical_rules(developer_fixture, change):
    developer_fixture["rules"][0].update(change)
    with pytest.raises(ValueError):
        repair.candidate_xml(repair.public_config({}), developer_fixture)


def test_rejects_multirow_force_signal(monkeypatch, developer_fixture):
    original = repair.warehouse.model_xml
    monkeypatch.setattr(repair.warehouse, "model_xml", lambda config: original(config).replace(
        '<joint name="cargo_latch" joint1="cargo_slide" polycoef="0 0 0 0 0" solref="0.005 1" />',
        '<weld name="cargo_latch" body1="cargo" body2="chassis"/>'))
    with pytest.raises(ValueError, match="scalar joint"):
        repair.candidate_xml(repair.public_config({}), developer_fixture)


def test_rule_uses_own_force_with_consecutive_dwell_and_full_reset(monkeypatch, developer_fixture):
    simulation = repair.CandidateSimulation(repair.public_config({}), developer_fixture)
    force = [10.]
    monkeypatch.setattr(repair, "scalar_equality_force", lambda *args: force[0])
    simulation.data.time = .49
    for _ in range(8):
        simulation._apply_fault()
    assert simulation.data.eq_active[simulation._latch]
    simulation.data.time = .5
    for _ in range(4):
        simulation._apply_fault()
    assert simulation.data.eq_active[simulation._latch]
    force[0] = 0
    simulation._apply_fault()
    force[0] = 5  # inclusive threshold
    for _ in range(4):
        simulation._apply_fault()
    assert simulation.data.eq_active[simulation._latch]
    simulation._apply_fault()
    assert not simulation.data.eq_active[simulation._latch]
    assert simulation._rule_fired == [True]
    simulation.reset_full()
    assert simulation.data.eq_active[simulation._latch]
    assert simulation._rule_fired == [False]
    assert simulation._rule_dwell == [0.]


def test_public_controls_cannot_select_fault():
    for config in ({"fault": "healthy"}, {"fault_at": 3.5}, {"latch_strength": 5}, {"probe": "payload_shift"}):
        with pytest.raises(ValueError):
            repair.public_config(config)


def test_fresh_candidate_forces_healthy_model(developer_fixture):
    config = replace(repair.public_config({}), fault="payload_mass", added_mass=40)
    simulation = repair.CandidateSimulation(config, developer_fixture)
    assert simulation.config.fault == "healthy"
    assert simulation.model.body("cargo").mass[0] == 16


def test_developer_fixture_solves_physics_without_teleport(developer_fixture):
    config = repair.public_config({"duration": 8})
    reference = repair.rollout(config, reference=True)
    nominal = repair.rollout(config)
    candidate = repair.rollout(config, developer_fixture)
    assert repair.prediction_error(nominal, reference)["position_rmse_m"] > .1
    assert repair.prediction_error(candidate, reference)["position_rmse_m"] < 1e-9
    assert candidate["summary"]["safe"]
    assert reference["summary"]["safe"]


def test_tool_outputs_never_include_reference_internals(tmp_path):
    broker = repair.WarehouseBroker(tmp_path)
    evidence = broker.initial_evidence()
    serialized = json.dumps(evidence)
    for forbidden in ("cargo_breakaway", "latch_strength", "latch_dwell", "fault_at", "fault_applied",
                      "cargo_displacement", "cargo_latched", "cargo_latch_overload", '"diagnostics"', '"events"'):
        assert forbidden not in serialized
    assert "model_signals" in serialized  # explicitly the nominal model's own signals
    assert "model_signals" not in json.dumps(evidence["cases"][0]["measured"])
    assert evidence["cases"][1]["original_error"]["position_rmse_m"] == 0
    result = broker.dispatch("run_experiment", {"config": {"fault": "healthy"}, "hypothesis": "control"})
    assert "error" in result
    assert broker.used["run_experiment"] == 1


def test_candidate_patch_hash_and_freeze_are_persistent(tmp_path, developer_fixture):
    broker = repair.WarehouseBroker(tmp_path)
    result = broker.dispatch("patch_model", {"candidate_json": json.dumps(developer_fixture),
                                             "rationale": "Developer fixture tests only"})
    assert result["accepted"]
    assert result["candidate_sha256"] == repair.digest(developer_fixture)
    result = broker.dispatch("submit_prediction", {"explanation": "Developer fixture tests only"})
    assert result["frozen"]
    developer_fixture["rules"] = []
    assert broker.frozen["rules"]
    result = broker.dispatch("patch_model", {"candidate_json": json.dumps(repair.EMPTY_MODEL), "rationale": "late"})
    assert "error" in result
    assert json.loads((tmp_path / "frozen_candidate.json").read_text())["rules"]


def test_all_predictions_locked_before_any_heldout_reference(tmp_path, monkeypatch):
    broker = repair.WarehouseBroker(tmp_path)
    seen = repair.config_dict(repair.public_config({"probe": "cargo_mirror", "duration": 11.,
                                                   "drive_scale": .95, "payload_mass": 18.}))
    broker.development_configs.append(seen)
    calls = []

    def fake_rollout(config, candidate=None, *, reference=False, **kwargs):
        calls.append("reference" if reference else "prediction")
        if reference:
            assert (tmp_path / "evaluation" / "predictions.json").is_file()
            assert (tmp_path / "evaluation" / "freeze.json").is_file()
            assert calls[:8] == ["prediction"] * 8
        return {"observations": [{"time": 0., "position": [0., 0., 0.], "heading": 0.}],
                "summary": {"safe": True}}

    monkeypatch.setattr(repair, "rollout", fake_rollout)
    report = broker.evaluate()
    assert all(repair.control_identity(case["config"]) != repair.control_identity(seen)
               for case in report["cases"])
    assert not report["passed"]  # a noninformative trial cannot produce a fake success
    assert report["frozen_predictions"]["all_predictions_locked_before_measurements"]
    with pytest.raises(ValueError, match="overwritten"):
        broker.evaluate()


def test_frozen_evaluation_rejects_seed_and_accepts_developer_fixture(tmp_path, developer_fixture):
    baseline = repair.WarehouseBroker(tmp_path / "baseline").evaluate()
    assert not baseline["passed"]
    repaired = repair.WarehouseBroker(tmp_path / "fixture", developer_fixture,
                                     origin={"kind": "developer_fixture"}).evaluate()
    assert repaired["passed"]
    assert repaired["provenance"]["kind"] == "developer_fixture"
    assert not repaired["provenance"]["source_changed"]
    assert repaired["provenance"]["initial_candidate_origin"] == "supplied_artifact"
    assert repaired["provenance"]["accepted_patch_count"] == 0
    assert not repaired["agent_submitted"]
    assert sum(not case["healthy_control"] for case in repaired["cases"]) >= 2
    assert all(case["candidate_error"]["position_rmse_m"] < 1e-9 for case in repaired["cases"])
    assert np.isfinite([case["nominal_error"]["position_rmse_m"] for case in repaired["cases"]]).all()


def test_optional_api_loop_records_verified_profile_and_submission(tmp_path, monkeypatch, developer_fixture):
    from types import SimpleNamespace

    from investigation import api

    class Call(SimpleNamespace):
        def model_dump(self, **kwargs):
            return vars(self)

    responses = iter([
        Call(type="function_call", name="patch_model", call_id="call1",
             arguments=json.dumps({"candidate_json": json.dumps(developer_fixture),
                                   "rationale": "Synthetic API adapter fixture"})),
        Call(type="function_call", name="submit_prediction", call_id="call2",
             arguments=json.dumps({"explanation": "Synthetic API adapter fixture"})),
    ])

    def fake_request(client, conversation, tools, **kwargs):
        assert kwargs["profile"] == api.DEFAULT_PROFILE
        assert {tool["name"] for tool in tools} == set(repair.LIMITS)
        return SimpleNamespace(id="synthetic-response", model="gpt-6-astra", status="completed",
                               reasoning=SimpleNamespace(effort=api.get_profile().reasoning_effort), usage=None, output=[next(responses)])

    broker = repair.WarehouseBroker(tmp_path)
    monkeypatch.setattr(broker, "initial_evidence", lambda: {"cases": []})
    monkeypatch.setattr(broker, "evaluate", lambda: {"passed": True, "candidate_sha256": repair.digest(broker.frozen)})
    monkeypatch.setattr(api, "request_response", fake_request)
    result = repair.run_session(object(), broker, max_api_requests=2)
    assert result["status"] == "evaluated"
    assert result["agent_submitted"]
    assert result["api_requests"] == 2
    assert result["candidate_sha256"] == repair.digest(developer_fixture)
    events = [json.loads(line) for line in (tmp_path / "tools.jsonl").read_text().splitlines()]
    verified = [event for event in events if event["tool"] == "api_response"]
    assert len(verified) == 2
    assert all(event["result"]["model"] == "gpt-6-astra" for event in verified)


def test_api_profile_mismatch_fails_without_executing_calls(tmp_path, monkeypatch):
    from types import SimpleNamespace

    from investigation import api

    broker = repair.WarehouseBroker(tmp_path)
    monkeypatch.setattr(broker, "initial_evidence", lambda: {"cases": []})
    monkeypatch.setattr(api, "request_response", lambda *args, **kwargs: SimpleNamespace(
        model="unrequested-model", reasoning=SimpleNamespace(effort="medium"), status="completed"))
    result = repair.run_session(object(), broker, max_api_requests=1)
    assert result["status"] == "error"
    assert not result["agent_submitted"]
    assert broker.frozen == repair.EMPTY_MODEL
    assert result["error_type"] == "RuntimeError"


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
def test_nonfinite_tool_arguments_are_rejected_without_audit_crash(tmp_path, bad):
    broker = repair.WarehouseBroker(tmp_path)
    result = broker.dispatch("run_experiment", {"config": {"duration": bad}, "hypothesis": "probe"})
    assert "error" in result
    audit = json.loads((tmp_path / "tools.jsonl").read_text())
    assert "rejected_arguments" in audit["arguments"]


def test_deep_json_and_nonfinite_constants_are_rejected():
    for source in ("[" * 2000 + "0" + "]" * 2000, "NaN", "Infinity", "-Infinity"):
        with pytest.raises(ValueError):
            repair.parse_json(source)
