"""The tool workflow enforces ownership, source isolation and measured repairs."""

import json
from pathlib import Path

import pytest

from investigation.platform_broker import PlatformBroker, compare_records


class FakePhysics:
    platform = "car"
    scenario = "private_test_preset"
    default_probe = "steering"
    default_duration_s = 2.0
    record_frames = False

    def __init__(self, platform="car", workdir=None, scenario=None, record_frames=False):
        self.workdir = Path(workdir or ".")
        self.offset = 1.0
        self.number = 0
        self.log = []

    def capabilities(self):
        return {"platform": "car", "probes": ["steering", "braking"],
                "model_parameters": {"gain": {"nominal": 1, "min": 0, "max": 1.5}},
                "components": [{"id": "wheel_FL"}, {"id": "wheel_FR"}],
                "repair_actions": [{"action": "align_wheel", "targets": ["wheel_FL", "wheel_FR"]}]}

    def record(self, kind, probe="steering", duration=2.0, offset=0.0):
        self.number += 1
        return {"id": f"r_{self.number}", "probe": probe, "duration_s": duration,
                "summary": {"safe": offset == 0, "outcome": "within_envelope" if offset == 0 else "outside_envelope"},
                "observations": [{"t_s": t, "time": t + 12, "phase": "probe",
                                  "position": [t, offset, 0.0]} for t in [0, duration]],
                "frames": [{"file": "private/path.jpg"}], "fault": "private_secret",
                "diagnostics": {"true_parameter": 5}, "kind": kind}

    def initial_evidence(self):
        return {"healthy": self.record("healthy"), "observed": self.record("actual", offset=1)}

    def observe(self, component_id=None):
        return {"component": component_id, "position": [0, self.offset, 0]}

    def run_experiment(self, probe, duration_s):
        self.log.append("experiment")
        return self.record("observed", probe, duration_s, self.offset)

    def reference_probe(self, probe, duration_s):
        return self.record("healthy", probe, duration_s)

    def run_model(self, probe, duration_s, parameters):
        self.log.append("prediction")
        parameters({"phase": "probe", "position": [0, 0, 0]}, 0)
        values = parameters({"phase": "probe", "position": [0.02, 0, 0]}, .02)
        return self.record("model", probe, duration_s, 1 - values.get("gain", 1))

    def apply_repair(self, action, target):
        if action != "align_wheel" or target not in {"wheel_FL", "wheel_FR"}:
            raise ValueError("private fault context must not leak")
        if target == "wheel_FL":
            self.offset = 0
        self.log.append("repair")
        return {"action": action, "target": target, "status": "applied", "verification_required": True}


@pytest.fixture
def broker(tmp_path):
    return PlatformBroker(tmp_path / "broker", FakePhysics())


def test_tool_records_are_owned_and_projected(broker):
    initial = broker.initial_evidence()
    encoded = json.dumps(initial)
    assert "private_secret" not in encoded and "private/path" not in encoded and "diagnostics" not in encoded
    known = initial["observed"]["id"]
    assert broker.dispatch("observe_run", {"id": known})["ok"]
    assert not broker.dispatch("observe_run", {"id": "other_session_record"})["ok"]
    assert not broker.dispatch("observe_run", {"id": "../.env"})["ok"]


def test_repair_receipt_is_not_evidence_of_recovery(broker):
    receipt = broker.dispatch("apply_repair", {"action": "align_wheel", "target": "wheel_FR",
                               "rationale": "Test the right wheel", "expected_effect": "Reduced drift"})
    assert receipt["ok"] and receipt["receipt"]["verification_required"]
    check = broker.dispatch("check_repair", {"probe": "steering", "duration_s": 2, "rationale": "Measure drift"})
    assert check["difference"]["position_rmse_m"] == 1
    assert check["difference"]["observed_within_envelope"] is False
    broker.dispatch("apply_repair", {"action": "align_wheel", "target": "wheel_FL",
                    "rationale": "Align the other wheel", "expected_effect": "Reduced drift"})
    check = broker.dispatch("check_repair", {"probe": "steering", "duration_s": 2, "rationale": "Verify"})
    assert check["difference"]["position_rmse_m"] == 0


def test_bad_target_error_does_not_disclose_private_details(broker):
    result = broker.dispatch("apply_repair", {"action": "magic", "target": "everything",
                             "rationale": "Test", "expected_effect": "Test"})
    assert not result["ok"]
    assert "private fault context" not in json.dumps(result)


def test_real_source_replacement_state_and_stale_hash(broker):
    original = broker.dispatch("inspect_model", {})
    assert original["ok"]
    source = ('def init_state():\n    return {"elapsed": 0.0}\n'
              'def predict_parameters(state):\n    return {"gain": max(0.0, 1.0 - state["elapsed"])}\n'
              'def advance_state(state, observation, dt_s):\n    state["elapsed"] += dt_s\n    return state\n')
    accepted = broker.dispatch("replace_model_source", {"source": source,
                                "expected_sha256": original["source_sha256"], "rationale": "Model gradual weakening"})
    assert accepted["ok"] and accepted["accepted"]
    assert broker.current_source.read_bytes() == source.encode()
    assert not broker.dispatch("replace_model_source", {"source": source,
                              "expected_sha256": original["source_sha256"], "rationale": "Stale change"})["ok"]
    prediction = broker.dispatch("run_model", {"probe": "steering", "duration_s": 2,
                                 "rationale": "Exercise the new state evolution"})
    assert prediction["ok"]
    assert prediction["observations"][0]["position"][1] == pytest.approx(.02)
    restored = broker.dispatch("restore_model_version", {"version_id": "v000",
                              "expected_sha256": accepted["source_sha256"], "rationale": "Restore baseline"})
    assert restored["ok"]
    assert broker.dispatch("inspect_model", {})["source_sha256"] == original["source_sha256"]


def test_rejected_edits_share_budget(broker):
    for _ in range(5):
        assert not broker.dispatch("replace_model_source", {"source": "bad", "expected_sha256": "stale", "rationale": "Test"})["ok"]
    result = broker.dispatch("restore_model_version", {"version_id": "v000", "expected_sha256": "stale", "rationale": "Test"})
    assert not result["ok"] and "shared" in result["error"]
    assert result["budget"]["model_edits"]["used"] == 5


@pytest.mark.parametrize("arguments", [
    {"probe": "private_fault", "duration_s": 2, "hypothesis": "Test", "expected_observation": "Test"},
    {"probe": "steering", "duration_s": float("nan"), "hypothesis": "Test", "expected_observation": "Test"},
    {"probe": "steering", "duration_s": True, "hypothesis": "Test", "expected_observation": "Test"},
    {"probe": "steering", "duration_s": 2, "hypothesis": "Test", "expected_observation": "Test", "fault": "healthy"},
])
def test_invalid_probe_rejected_before_simulation(broker, arguments):
    assert not broker.dispatch("run_experiment", arguments)["ok"]
    assert not broker.physics.log


def test_freeze_closes_tools_and_verification_replays_actions_on_fresh_specimen(broker, monkeypatch):
    import investigation.platform_physics as module
    created = []
    def factory(*args, **kwargs):
        physics = FakePhysics(*args, **kwargs)
        created.append(physics)
        return physics
    monkeypatch.setattr(module, "PlatformPhysics", factory)
    broker.initial_evidence()
    broker.dispatch("apply_repair", {"action": "align_wheel", "target": "wheel_FL",
                    "rationale": "Measured left wheel misalignment", "expected_effect": "Normal path"})
    result = broker.dispatch("submit_result", {"diagnosis": "Alignment", "evidence": "Observed drift",
                             "remaining_uncertainty": "Only controlled probes"})
    assert result["ok"] and result["submitted"]
    assert not broker.dispatch("run_experiment", {"probe": "steering", "duration_s": 2,
                              "hypothesis": "Edit after freeze", "expected_observation": "None"})["ok"]
    final = broker.finalize("agent_submission")
    assert created[0] is not broker.physics
    assert created[0].log[:2] == ["prediction", "prediction"]
    assert len(final["cases"]) == 2
    assert all(case["before_difference"]["position_rmse_m"] == 1 for case in final["cases"])
    assert all(case["after_difference"]["position_rmse_m"] == 0 for case in final["cases"])
    assert (broker.workdir / "submission/predictions_locked.json").is_file()
    assert broker.finalize("repeat") is final


def test_metrics_compare_overlap_without_extrapolation():
    physics = FakePhysics()
    long = physics.record("healthy", duration=10)
    short = physics.record("observed", duration=2, offset=1)
    compared = compare_records(long, short)
    assert compared["common_duration_s"] == 2
    assert compared["position_rmse_m"] == 1
    assert compare_records({}, short)["position_rmse_m"] is None
