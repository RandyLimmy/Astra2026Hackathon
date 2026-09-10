"""Curve/drop benchmark checks; the known repair below is a developer fixture only."""
import json
import math
from copy import deepcopy
from types import SimpleNamespace

import pytest

from investigation import warehouse as repair


@pytest.fixture
def developer_fixture():
    return {"schema_version": 1, "model_edits": [], "rules": [{
        "when": {"signal": "equality_force", "name": "cargo_latch", "absolute": True,
                 "comparison": "gte", "threshold": 5., "after_s": .5, "sustained_s": .01},
        "updates": [{"target": "equality", "name": "cargo_latch", "field": "active", "value": False}],
        "once": True}]}


def observed_row(*, cargo_z=.5, quaternion=None, touched_floor=False):
    return {"time": 0., "position": [0., 0., .24], "heading": 0.,
            "cargo_position": [0., 0., cargo_z],
            "cargo_orientation": quaternion or [1., 0., 0., 0.],
            "cargo_floor_contact": touched_floor, "cargo_has_touched_floor": touched_floor}


def test_drop_detection_uses_public_contact_history_in_any_orientation():
    upright = {"observations": [observed_row()]}
    side = {"observations": [observed_row(cargo_z=.32,
                              quaternion=[math.sqrt(.5), 0., math.sqrt(.5), 0.], touched_floor=True)]}
    assert not repair.cargo_dropped(upright)
    assert repair.cargo_dropped(side)
    assert side["observations"][0]["cargo_position"][2] > .25


def test_identical_chassis_paths_do_not_hide_wrong_cargo_prediction():
    measured = {"observations": [observed_row(cargo_z=.13, touched_floor=True)]}
    nominal = {"observations": [observed_row()]}
    error = repair.prediction_error(nominal, measured)
    assert error["position_rmse_m"] == 0
    assert error["heading_rmse_rad"] == 0
    assert error["cargo_position_rmse_m"] == pytest.approx(.37)
    assert repair.cargo_dropped(measured)
    assert not repair.cargo_dropped(nominal)


def test_curve_task_cannot_select_rail_or_hidden_controls():
    config = repair.public_config({}, task="curve")
    assert config.probe == "cargo_curve"
    assert config.payload_mass == 8.
    assert config.duration == 17.
    for value in ({"probe": "cargo_turn"}, {"fault": "healthy"}, {"latch_strength": 5.}):
        with pytest.raises(ValueError):
            repair.public_config(value, task="curve")
    with pytest.raises(ValueError):
        repair.public_config({"probe": "cargo_curve"})


def test_curve_evidence_is_neutral_and_contains_external_cargo_pose(tmp_path):
    broker = repair.WarehouseBroker(tmp_path, task="curve")
    evidence = broker.initial_evidence()
    assert evidence["benchmark"] == "curve"
    assert len(evidence["cases"]) == 2
    serialized = json.dumps(evidence)
    for forbidden in ("cargo_breakaway", "latch_strength", "fault_applied", '"events"',
                      "cargo_latched", "cargo_latch_overload", '"diagnostics"'):
        assert forbidden not in serialized
    first, control = evidence["cases"]
    assert "cargo_position" in first["measured"]["observations"][0]
    assert "cargo_orientation" in first["measured"]["observations"][0]
    assert repair.cargo_dropped(first["measured"])
    assert not repair.cargo_dropped(first["original_prediction"])
    assert not repair.cargo_dropped(control["measured"])
    assert first["original_error"]["cargo_position_rmse_m"] > .15
    assert control["original_error"]["cargo_position_rmse_m"] == 0
    inspected = broker.dispatch("inspect_model", {})
    assert "cargo" in inspected["nominal_xml"]
    assert '"fault"' not in inspected["nominal_xml"]


def test_curve_holdouts_freeze_predictions_and_exclude_observed_duration_prefix(tmp_path, monkeypatch):
    broker = repair.WarehouseBroker(tmp_path, task="curve")
    observed = repair.config_dict(repair.public_config({"probe": "cargo_curve", "duration": 20.,
                                                       "drive_scale": .95, "payload_mass": 9.}, task="curve"))
    broker.development_configs.append(observed)
    calls = []

    def fake_rollout(config, candidate=None, *, reference=False, **kwargs):
        calls.append("reference" if reference else "prediction")
        if reference:
            assert calls[:6] == ["prediction"] * 6
            assert (tmp_path / "evaluation" / "freeze.json").is_file()
        cargo_z = .13 if reference and config.probe == "cargo_curve" else .5
        return {"observations": [observed_row(cargo_z=cargo_z, touched_floor=cargo_z < .2)], "summary": {"safe": True}}

    monkeypatch.setattr(repair, "rollout", fake_rollout)
    result = broker.evaluate()
    assert result["task"] == "curve"
    assert len(result["cases"]) == 3
    assert not result["passed"]
    assert all(repair.control_identity(case["config"]) != repair.control_identity(observed)
               for case in result["cases"])
    for case in result["cases"]:
        assert case["candidate_error"]["position_rmse_m"] == 0
        if not case["healthy_control"]:
            assert not case["passed"]
            assert case["cargo_outcome"] == {"measured_dropped": True, "candidate_dropped": False,
                                             "nominal_dropped": False}


def test_curve_seed_fails_and_developer_fixture_predicts_drop_and_control(tmp_path, developer_fixture):
    seed = repair.WarehouseBroker(tmp_path / "seed", task="curve").evaluate()
    assert not seed["passed"]
    fixed = repair.WarehouseBroker(tmp_path / "fixture", developer_fixture, task="curve",
                                   origin={"kind": "developer_fixture"}).evaluate()
    assert fixed["passed"]
    assert sum(not case["healthy_control"] for case in fixed["cases"]) == 2
    for case in fixed["cases"]:
        assert case["candidate_error"]["cargo_position_rmse_m"] < 1e-9
        assert case["candidate_error"]["position_rmse_m"] < 1e-9
        assert case["cargo_outcome"]["candidate_dropped"] == (not case["healthy_control"])
    assert fixed["provenance"]["kind"] == "developer_fixture"
    assert not fixed["agent_submitted"]


def test_curve_api_uses_curve_tools_without_mutating_rail_schema(tmp_path, monkeypatch):
    from investigation import api

    original = deepcopy(repair.TOOL_SCHEMAS)
    broker = repair.WarehouseBroker(tmp_path, task="curve")
    monkeypatch.setattr(broker, "initial_evidence", lambda: {"benchmark": "curve", "cases": []})

    def fake_request(client, conversation, schemas, **kwargs):
        for schema in schemas:
            if "config" in schema["parameters"]["properties"]:
                assert schema["parameters"]["properties"]["config"]["properties"]["probe"]["enum"] == [
                    "cargo_curve", "cargo_curve_slow"]
        return SimpleNamespace(model="wrong-profile", status="completed", reasoning=None)

    monkeypatch.setattr(api, "request_response", fake_request)
    result = repair.run_session(object(), broker, max_api_requests=1)
    assert result["task"] == "curve"
    assert result["status"] == "error"
    assert repair.TOOL_SCHEMAS == original


def test_prepare_cli_selects_curve(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(repair.WarehouseBroker, "initial_evidence", lambda self: {"benchmark": self.task})
    assert repair.main(["prepare", "--task", "curve", "--output", str(tmp_path)]) == 0
    assert json.loads(capsys.readouterr().out)["api_requests"] == 0
    assert json.loads((tmp_path / "provenance.json").read_text())["task"] == "curve"


def test_curve_novelty_collapses_equivalent_speed_settings():
    fast = {"probe": "cargo_curve", "duration": 17., "drive_scale": .3, "payload_mass": 8.}
    slow = {"probe": "cargo_curve_slow", "duration": 20., "drive_scale": .8, "payload_mass": 8.}
    assert repair.control_identity(fast) == repair.control_identity(slow)
