"""Integration checks for the trusted car comparison and neutral task export."""

from datetime import datetime
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import pytest

from component_worker import WheelActuatorWorker
from simulator import comparison
from simulator.config import Experiment
from simulator.runner import Simulator


def _require_worker():
    if sys.platform != "darwin" or not Path("/usr/bin/sandbox-exec").is_file():
        pytest.skip("This integration requires the enforced macOS component worker")


@pytest.fixture(scope="module")
def baseline_comparison(tmp_path_factory):
    """One real low-speed run also verifies source freezing and reveal order."""
    _require_worker()
    parent = tmp_path_factory.mktemp("car-comparison")
    output = parent / "recorded"
    source = parent / "editable.py"
    loaded_bytes = comparison.DEFAULT_CANDIDATE.read_bytes()
    source.write_bytes(loaded_bytes)
    sequence = []
    original_run = Simulator.run

    def start_worker_then_edit_source(path):
        worker = WheelActuatorWorker(path)
        # The active worker must continue executing the bytes it already loaded.
        source.write_text("raise RuntimeError('source changed after loading')\n")
        return worker

    def observe_run(self, callback=None, preparation_history=None, *, prepared=False):
        if self.actuator is None:
            assert prepared
            sequence.append("reference")
            frozen = json.loads((output / "predictions.json").read_text())
            saved = json.loads((output / "candidate/public/summary.json").read_text())
            assert frozen["candidate"]["summary"] == saved
            assert (output / "candidate/public/observations.jsonl").stat().st_size > 0
        else:
            sequence.append("candidate")
            assert preparation_history is not None
        return original_run(self, callback, preparation_history, prepared=prepared)

    with pytest.MonkeyPatch.context() as patch:
        patch.setattr(comparison, "WheelActuatorWorker", start_worker_then_edit_source)
        patch.setattr(Simulator, "run", observe_run)
        bundle = comparison.compare(
            Experiment(initial_speed=8, brake_at=0, wall=False, duration=4),
            output, candidate=source,
        )
    return {"bundle": bundle, "output": output, "source": source,
            "loaded_bytes": loaded_bytes, "sequence": sequence}


def test_real_nominal_candidate_matches_low_speed_reference(baseline_comparison):
    bundle = baseline_comparison["bundle"]
    reference = bundle["reference"]
    prediction = bundle["predictions"]["candidate"]["summary"]
    assert reference["stopped"] and prediction["stopped"]
    assert not reference["censored"] and not prediction["censored"]
    assert not reference["collision"] and not prediction["collision"]
    assert prediction["stopping_distance"] == pytest.approx(reference["stopping_distance"], abs=1e-10)
    assert bundle["errors"]["candidate"]["final_front_x_error_m"] < 1e-10
    assert bundle["agent_run"] is False
    assert bundle["final_holdout_evaluation"] is False
    assert (baseline_comparison["output"] / "comparison.png").is_file()


def test_predictions_are_saved_before_reference_probe_and_timestamped(baseline_comparison):
    assert baseline_comparison["sequence"] == ["candidate", "reference"]
    bundle = baseline_comparison["bundle"]
    completed = datetime.fromisoformat(bundle["predictions"]["candidate"]["prediction_completed_at"])
    revealed = datetime.fromisoformat(bundle["reference_trial_started_at"])
    assert completed <= revealed
    saved = json.loads((baseline_comparison["output"] / "predictions.json").read_text())
    assert saved == bundle["predictions"]


def test_prediction_hash_identifies_loaded_source_after_file_changes(baseline_comparison):
    prediction = baseline_comparison["bundle"]["predictions"]["candidate"]
    sha = prediction["source_sha256"]
    assert sha == hashlib.sha256(baseline_comparison["loaded_bytes"]).hexdigest()
    assert sha != hashlib.sha256(baseline_comparison["source"].read_bytes()).hexdigest()
    snapshot = baseline_comparison["output"] / prediction["source_file"]
    assert snapshot.read_bytes() == baseline_comparison["loaded_bytes"]
    assert hashlib.sha256(snapshot.read_bytes()).hexdigest() == sha


def test_comparison_refuses_to_overwrite_existing_artifacts(baseline_comparison):
    output = baseline_comparison["output"]
    original = (output / "comparison.json").read_bytes()
    with pytest.raises(FileExistsError):
        comparison.compare(Experiment(), output)
    assert (output / "comparison.json").read_bytes() == original


def test_nominal_config_preserves_declared_controls_and_strips_hidden_parameters():
    config = Experiment(
        initial_speed=8, brake_at=5, brake=0.8, throttle=0.2, duration=4,
        wall=False, wall_x=50, timestep=0.002, warmup_cycles=3, recovery=12,
        commands=((0, 0.2, 0), (0.4, 0, 0.8)), thermal=True,
        detach_wheel="FR", detach_at=1.5, wet_friction=0.2,
        wet_start=12, wet_end=30, payload=240, weak_wheel="RL",
        brake_efficiency=0.2, weak_at=0.6, lag=0.4,
    )
    public = comparison.public_config(config)
    hidden = {"thermal", "detach_wheel", "detach_at", "wet_friction", "wet_start",
              "wet_end", "payload", "weak_wheel", "brake_efficiency", "weak_at", "lag"}
    assert not hidden.intersection(public)
    nominal = comparison.nominal_config(config)
    for name, value in public.items():
        assert getattr(nominal, name) == value
    for name in hidden:
        assert getattr(nominal, name) == getattr(Experiment(), name)


def test_export_contains_only_neutral_source_and_interface_and_never_overwrites(tmp_path):
    output = tmp_path / "task"
    comparison.export_task(output)
    assert {path.name for path in output.iterdir()} == {"actuator.py", "INTERFACE.md", "TASK.md"}
    assert (output / "actuator.py").read_bytes() == comparison.DEFAULT_CANDIDATE.read_bytes()
    text = "\n".join(path.read_text() for path in output.iterdir()).lower()
    for private_hint in ("mujoco", "temperature", "thermal", "brake fade", "developer_wheel", "reference_host"):
        assert private_hint not in text
    before = {path.name: path.read_bytes() for path in output.iterdir()}
    with pytest.raises(FileExistsError):
        comparison.export_task(output)
    assert {path.name: path.read_bytes() for path in output.iterdir()} == before


def test_timeout_remains_censored_in_comparison_error():
    public = Simulator(Experiment(initial_speed=8, brake_at=0, wall=False, duration=0.02)).run()["public"]
    assert public["censored"]
    assert not public["stopped"]
    assert public["stopping_distance"] is None
    assert comparison.outcome_error(public, public)["stopping_distance_error_m"] is None


def test_actual_worker_replays_shared_commands_without_reference_state():
    _require_worker()
    config = Experiment(initial_speed=8, brake_at=0, wall=False, thermal=True)
    history = [
        {"kind": "reset", "speed_mps": 5.0, "phase": "conditioning_1"},
        {"kind": "step", "throttle": 0.2, "brake": 0.0, "dt_s": 0.002, "steps": 20, "phase": "conditioning_1"},
        {"kind": "step", "throttle": 0.0, "brake": 1.0, "dt_s": 0.002, "steps": 60, "phase": "conditioning_1"},
        {"kind": "reset", "speed_mps": 0.0, "phase": "recovery"},
        {"kind": "step", "throttle": 0.0, "brake": 0.0, "dt_s": 0.002, "steps": 10, "phase": "recovery"},
        {"kind": "reset", "speed_mps": 8.0, "phase": "trial"},
    ]
    reference = Simulator(config)
    reference_observations = []
    reference.prepare(lambda current, phase: reference_observations.append(current.observe(phase)),
                      history=history)
    with WheelActuatorWorker(comparison.DEVELOPER_CHECK) as worker:
        candidate = Simulator(comparison.nominal_config(config), actuator=worker)
        assert worker.inspect_state() == {"recent_work_j": [0.0] * 4}
        candidate_observations = []
        candidate.prepare(lambda current, phase: candidate_observations.append(current.observe(phase)),
                          history=reference.preparation_history)
        assert candidate.preparation_history == reference.preparation_history == history
        assert candidate.elapsed == pytest.approx(reference.elapsed)
        assert all(work > 0 for work in worker.inspect_state()["recent_work_j"])
        assert candidate_observations == reference_observations
        # Low-work preparations produce the same mechanics, but the candidate
        # accumulated its own component state rather than copying hidden state.
        np.testing.assert_allclose(candidate.data.qpos, reference.data.qpos, atol=1e-10)
        np.testing.assert_allclose(candidate.data.qvel, reference.data.qvel, atol=1e-10)
        assert max(reference.temperature) > 20
