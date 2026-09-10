import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from investigation.evaluation import PREDECLARED_CRITERIA, evaluate_frozen
from investigation.physics import (
    ORIGINAL_SOURCE, PhysicsService, _digest, normalize_config, select_observations,
)


def test_public_configuration_defaults_and_types():
    assert normalize_config({}) == {
        "speed_mps": 25.0, "brake_strength": 1.0, "preparation_cycles": 0,
        "wait_s": 0.0, "wall_distance_m": None,
    }
    assert normalize_config({"speed_mps": 22})["speed_mps"] == 22.0


@pytest.mark.parametrize("config", [
    {"thermal": True}, {"speed_mps": True}, {"speed_mps": float("nan")},
    {"brake_strength": 0.1}, {"preparation_cycles": 1.0}, {"preparation_cycles": True},
    {"preparation_cycles": 6}, {"wait_s": 121}, {"wall_distance_m": 9},
])
def test_configuration_rejects_hidden_fields_and_invalid_controls(config):
    with pytest.raises(ValueError):
        normalize_config(config)


def test_payload_cap_preserves_phase_command_collision_boundaries_and_endpoints():
    rows = [
        {"time": index / 100, "phase": "preparation_1" if index < 500 else "trial",
         "phase_time": (index if index < 500 else index - 500) / 100,
         "throttle": 0.0, "brake": 0.0 if index < 300 else 1.0,
         "wall_contact": index >= 900, "value": 1.123456789}
        for index in range(1000)
    ]
    selected = select_observations(rows)
    assert len(selected) == 120
    times = {row["time"] for row in selected}
    assert {0, 2.99, 3.0, 4.99, 5.0, 8.99, 9.0, 9.99} <= times
    assert all(row["value"] == 1.123457 for row in selected)
    assert selected[-1]["wall_contact"] is True


def test_real_cold_run_model_never_calls_reference_and_reuses_exact_cache(tmp_path, monkeypatch):
    service = PhysicsService(tmp_path / "cache")
    config = {"speed_mps": 5}
    reference = service.reference(config)
    assert reference["summary"]["stopped"]
    assert reference["preparation_history"] == [
        {"kind": "reset", "speed_mps": 5.0, "phase": "trial"},
    ]

    def forbidden(*args, **kwargs):
        raise AssertionError("A reference was run from the model path or a cache was missed")

    monkeypatch.setattr(service, "prepare_reference", forbidden)
    monkeypatch.setattr(service, "finish_reference", forbidden)
    prediction = service.model(config, ORIGINAL_SOURCE, reference["preparation_history"])
    assert prediction["summary"] == reference["summary"]
    assert prediction["source_sha256"] == hashlib.sha256(ORIGINAL_SOURCE.read_bytes()).hexdigest()
    assert len(prediction["observations"]) <= 120
    for private in ("temperature", "mujoco", "thermal", "candidate_state", "source_path"):
        assert private not in json.dumps(prediction).lower()

    import investigation.physics as physics
    monkeypatch.setattr(physics, "WheelActuatorWorker", forbidden)
    assert service.model(config, ORIGINAL_SOURCE, reference["preparation_history"]) == prediction
    assert service.reference(config) == reference
    full = service.model_record(config, ORIGINAL_SOURCE, reference["preparation_history"])
    assert "candidate_state" in full
    assert _digest(normalize_config(config)) in service.development_configs


class EvaluationService:
    """Validate freeze/reveal orchestration without spending time on preparation."""

    _public = staticmethod(PhysicsService._public)

    def __init__(self, output, fail_candidate=False):
        self.output = output
        self.development_configs = set()
        self.log = []
        self.fail_candidate = fail_candidate

    def prepare_reference(self, config):
        assert (self.output / "frozen_candidate.py").exists()
        assert (self.output / "criteria.json").exists()
        self.log.append("prepare")
        history = [{"kind": "reset", "speed_mps": config["speed_mps"], "phase": "trial"}]
        return SimpleNamespace(config=config, history=history)

    @staticmethod
    def _record(config, history, distance, **extra):
        return {"config": config, "preparation_history": history, "observations": [],
                "summary": {"stopped": True, "censored": False, "collision": False,
                            "stopping_distance": distance}, **extra}

    def model_record(self, config, source, history):
        assert "reveal" not in self.log
        self.log.append("prediction")
        original = Path(source).name == "original_candidate.py"
        if self.fail_candidate and not original and config["preparation_cycles"]:
            raise RuntimeError("Candidate failed to finish")
        distance = 20.0 if original or config["preparation_cycles"] == 0 else (30.0 if config["wait_s"] else 39.0)
        return self._record(config, history, distance,
                            source_sha256=hashlib.sha256(Path(source).read_bytes()).hexdigest(),
                            candidate_state={} if original else {"memory": [1.0] * 4})

    def finish_reference(self, prepared):
        assert self.log.count("prediction") == 6
        assert (self.output / "predictions_locked.json").exists()
        self.log.append("reveal")
        config = prepared.config
        distance = 20.0 if config["preparation_cycles"] == 0 else (30.0 if config["wait_s"] else 40.0)
        return self._record(config, prepared.history, distance)


def test_frozen_evaluation_locks_all_predictions_before_any_reveal(tmp_path):
    source = tmp_path / "submitted.py"
    source.write_text(ORIGINAL_SOURCE.read_text() + "\n# Candidate version under evaluation.\n")
    output = tmp_path / "evaluation"
    service = EvaluationService(output)
    events = []
    result = evaluate_frozen(service, source, output, emit=events.append)
    assert result["aggregate"]["scored_cases"] == 3
    assert result["aggregate"]["original_mae_m"] == pytest.approx(10)
    assert result["aggregate"]["candidate_mae_m"] == pytest.approx(1 / 3)
    assert result["aggregate"]["predictive_success"] is True
    assert result["predeclared_criteria"] == PREDECLARED_CRITERIA
    assert result["state_extension"]["added_state_keys"] == ["memory"]
    assert service.log[-3:] == ["reveal"] * 3
    assert (output / "frozen_candidate.py").read_bytes() == source.read_bytes()
    assert (output / "source.diff").exists()
    assert json.loads((output / "result.json").read_text())["source_sha256"] == result["source_sha256"]
    assert [event["stage"] for event in events].index("all_predictions_locked") < [event["stage"] for event in events].index("reference_revealed")


def test_previously_observed_reserved_case_is_excluded_without_replacement(tmp_path):
    output = tmp_path / "evaluation"
    service = EvaluationService(output)
    service.development_configs.add(_digest(normalize_config({"speed_mps": 22, "preparation_cycles": 2})))
    result = evaluate_frozen(service, ORIGINAL_SOURCE, output)
    assert len(result["cases"]) == 3
    assert result["cases"][1]["previously_observed"]
    assert result["aggregate"]["scored_cases"] == 2
    assert result["aggregate"]["previously_observed_cases"] == 1
    assert result["aggregate"]["predictive_success"] is None


def test_candidate_failures_are_censored_and_fail_the_success_gate(tmp_path):
    output = tmp_path / "evaluation"
    result = evaluate_frozen(EvaluationService(output, fail_candidate=True), ORIGINAL_SOURCE, output)
    assert result["aggregate"]["failed_stop_predictions"] == 2
    assert result["aggregate"]["predictive_success"] is False
    assert result["cases"][1]["candidate_error_m"] is None
    assert result["cases"][1]["candidate"]["summary"]["censored"] is True
    assert "Candidate failed to finish" in result["cases"][1]["candidate_error"]
