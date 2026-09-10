from dataclasses import replace
import json
from pathlib import Path

import pytest

from contracts import ExperimentConfig
from sim.bridge import Mechanics
from sim.experiment import predict_from_history, run_reference


def test_force_changes_mujoco_mechanics_with_expected_mass():
    rig = Mechanics()
    rig.advance(1200.0, 0.0, 0.01)
    assert rig.model.nv == 1
    assert rig.speed == pytest.approx(0.01)
    assert rig.position == pytest.approx(0.00005)


def test_normal_stop_matches_analytic_solution_and_does_not_reverse():
    run = run_reference(ExperimentConfig(), fade=False)
    assert run.summary["stopping_distance_m"] == pytest.approx(20**2 / (2 * 7.5), abs=0.002)
    assert run.summary["status"] == "stopped"
    assert not run.summary["wall_crossed"]
    assert all(row["v_mps"] >= 0 for row in run.probe)
    assert all(a["x_m"] <= b["x_m"] for a, b in zip(run.probe, run.probe[1:]))


def test_history_changes_response_and_rest_recovers():
    config = ExperimentConfig(preparation_cycles=5)
    normal = run_reference(config, fade=False)
    repeated = run_reference(config)
    recovered = run_reference(replace(config, rest_time_s=300))
    cold = run_reference(ExperimentConfig())
    assert repeated.probe[0]["v_mps"] == pytest.approx(cold.probe[0]["v_mps"])
    assert repeated.summary["stopping_distance_m"] > 2 * normal.summary["stopping_distance_m"]
    assert repeated.summary["wall_crossed"]
    assert recovered.summary["stopping_distance_m"] == pytest.approx(cold.summary["stopping_distance_m"], abs=0.05)
    assert cold.summary["stopping_distance_m"] == pytest.approx(normal.summary["stopping_distance_m"], abs=0.002)
    assert {row["phase"] for row in repeated.history} == {"drive", "brake", "wait"}


def test_halving_timestep_preserves_stopping_distance():
    config = ExperimentConfig(preparation_cycles=4, rest_time_s=60)
    coarse = run_reference(config)
    fine = run_reference(replace(config, timestep_s=0.005))
    assert abs(coarse.summary["stopping_distance_m"] - fine.summary["stopping_distance_m"]) < 0.1


def test_zero_brake_is_a_bounded_non_stopping_case():
    run = run_reference(ExperimentConfig(brake_strength=0, probe_horizon_s=3))
    assert run.summary["status"] == "not_stopped"
    assert run.summary["stopping_distance_m"] is None
    assert run.summary["distance_at_horizon_m"] == pytest.approx(60)
    assert run.summary["wall_crossed"]


def test_barrier_changes_outcome_without_changing_trajectory():
    near = run_reference(ExperimentConfig(wall_distance_m=20), fade=False)
    far = run_reference(ExperimentConfig(wall_distance_m=40), fade=False)
    assert near.probe == far.probe
    assert near.summary["wall_crossed"] and not far.summary["wall_crossed"]


def test_fresh_runs_are_deterministic_and_observations_are_neutral():
    config = ExperimentConfig(preparation_cycles=1)
    first = run_reference(config).to_dict()
    assert first == run_reference(config).to_dict()
    payload = json.dumps(first).lower()
    for forbidden in ("temperature", "mujoco", "thermal", "effectiveness", "api_key"):
        assert forbidden not in payload


@pytest.mark.parametrize("kwargs", [
    {"speed_mps": float("nan")}, {"brake_strength": 1.1},
    {"rest_time_s": -1}, {"preparation_cycles": 9},
    {"preparation_cycles": True}, {"timestep_s": 0},
])
def test_invalid_configuration_is_rejected(kwargs):
    with pytest.raises(ValueError):
        ExperimentConfig(**kwargs)


def test_candidate_uses_observed_history_then_own_mujoco_rollout():
    from component_worker.client import ActuatorWorker
    root = Path(__file__).resolve().parents[1]
    config = ExperimentConfig(preparation_cycles=3)
    reference = run_reference(config)
    with ActuatorWorker(root / "candidate/actuator.py") as worker:
        prediction = predict_from_history(config, reference.history, worker)
        assert worker.inspect_state() == {}
    assert prediction.summary["stopping_distance_m"] == pytest.approx(26.6667, abs=0.002)
    assert reference.summary["stopping_distance_m"] > prediction.summary["stopping_distance_m"] + 10
