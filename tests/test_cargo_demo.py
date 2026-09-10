"""Presentation comparisons must replay measured states and identical commands."""

from dataclasses import replace
import hashlib
import json
from unittest.mock import patch

import numpy as np
import pytest

from simulator import cargo_demo
from simulator.platforms.warehouse import Config, Simulation


def test_capture_replays_actual_pose_and_includes_final_sample():
    sim = Simulation(Config(duration=.113))
    record = cargo_demo.capture(sim, fps=15)
    final = record["samples"][-1]
    assert final["observation"]["time"] == sim.elapsed
    np.testing.assert_array_equal(final["qpos"], sim.data.qpos)
    nominal = Simulation(sim.config)
    cargo_demo._restore(nominal, final)
    np.testing.assert_array_equal(nominal.data.qpos, sim.data.qpos)
    np.testing.assert_allclose(nominal.data.xpos, sim.data.xpos, atol=1e-10)
    assert len(record["samples"]) == 3


def test_comparison_rejects_time_or_command_mismatch():
    config = Config(duration=.2)
    a = cargo_demo.capture(Simulation(config), fps=15)
    b = cargo_demo.capture(Simulation(config), fps=15)
    assert cargo_demo.comparison(a, b)["position_rmse_m"] == 0
    b["samples"][1]["observation"]["command"]["left"] = .5
    with pytest.raises(ValueError, match="identical"):
        cargo_demo.comparison(a, b)
    with pytest.raises(ValueError, match="synchronized"):
        cargo_demo.comparison(a, {"samples": []})


def test_feedback_comparison_keeps_time_alignment_and_reports_changed_commands():
    config = Config(duration=.2)
    a = cargo_demo.capture(Simulation(config), fps=15)
    b = cargo_demo.capture(Simulation(config), fps=15)
    b["samples"][1]["observation"]["command"]["left"] = .1
    result = cargo_demo.comparison(a, b, feedback=True)
    assert result["identical_commands"] is False
    assert result["control_comparison"] == "same route feedback controller"
    b["samples"][1]["observation"]["time"] += .05
    with pytest.raises(ValueError, match="identical times"):
        cargo_demo.comparison(a, b, feedback=True)


def test_cargo_mismatch_is_visible_even_when_vehicle_paths_match():
    config = Config(duration=.2)
    a = cargo_demo.capture(Simulation(config), fps=15)
    b = cargo_demo.capture(Simulation(config), fps=15)
    for row in a["samples"]:
        row["cargo_position"] = [0, 0, .5]
    for row in b["samples"]:
        row.update(cargo_position=[0, 1, .2], cargo_ground_contact=True, cargo_dropped=True)
    result = cargo_demo.comparison(a, b, feedback=True)
    assert result["position_rmse_m"] == 0
    assert result["cargo_position_rmse_m"] > 1
    assert result["cargo_dropped"] is True
    assert cargo_demo.stage(b["samples"][0], 0, .2, curve=True)[0] == "03 / CARGO LOST"


def test_curve_replay_captures_physical_spill_and_predeclared_route(tmp_path):
    output = tmp_path / "curve"
    report = cargo_demo.build(output, fps=5, media=False)
    assert report["scenario"] == "warehouse_curve_demo"
    assert len(report["intended_route"]["centerline"]) > 10
    assert report["baseline"]["cargo_dropped"] is True
    assert report["baseline"]["cargo_position_rmse_m"] > .3
    assert report["baseline"]["first_recorded_ground_contact_s"] is not None
    assert report["baseline"]["control_comparison"] == "same route feedback controller"


def test_predictions_are_locked_before_measured_rollout(tmp_path):
    output = tmp_path / "replay"
    original = cargo_demo.capture
    calls = []

    def capture(sim, fps):
        calls.append(sim.config.fault)
        if sim.config.fault == "payload_shift":
            locks = json.loads((output / "predictions-locked.json").read_text())
            assert locks["nominal_sha256"] == hashlib.sha256(
                (output / "nominal-prediction.json").read_bytes()).hexdigest()
        return original(sim, fps)

    with patch.object(cargo_demo, "capture", side_effect=capture):
        report = cargo_demo.build(output, scenario="warehouse_shift", overrides={"duration": .2}, media=False)
    assert calls == ["healthy", "payload_shift"]
    assert report["candidate_sha256"] is None
    assert not (output / "candidate-prediction.json").exists()
    assert not (output / "animation.gif").exists()


def test_operator_comparison_exposes_actual_load_movement():
    config = Config(fault="payload_shift", duration=3, fault_at=1)
    a = cargo_demo.capture(Simulation(replace(config, fault="healthy")), fps=10)
    b = cargo_demo.capture(Simulation(config), fps=10)
    report = cargo_demo.comparison(a, b)
    assert report["maximum_cargo_travel_m"] > .15
    assert report["first_recorded_release_s"] >= 1
    assert report["maximum_position_error_m"] > .001
    assert cargo_demo.stage(b["samples"][-1], 1, 3)[0] == "04 / RESULT"


def test_replay_candidate_identity_matches_investigation_and_copied_source(tmp_path):
    from investigation.warehouse import digest
    source = tmp_path / "model.json"
    artifact = {"schema_version": 1, "model_edits": [], "rules": []}
    source.write_text(json.dumps(artifact, indent=4) + "\n")
    output = tmp_path / "candidate-replay"
    report = cargo_demo.build(output, scenario="warehouse_shift", overrides={"duration": .2},
                              candidate=source, media=False)
    assert report["candidate_sha256"] == digest(artifact)
    assert report["candidate_source_bytes_sha256"] == hashlib.sha256(source.read_bytes()).hexdigest()
    assert (output / "candidate-model.json").read_bytes() == source.read_bytes()
    locks = json.loads((output / "predictions-locked.json").read_text())
    assert locks["candidate_prediction_sha256"] == hashlib.sha256(
        (output / "candidate-prediction.json").read_bytes()).hexdigest()
    assert report["candidate"]["position_rmse_m"] == report["baseline"]["position_rmse_m"]


def test_large_candidate_is_rejected_before_artifacts(tmp_path):
    source = tmp_path / "large.json"
    source.write_text(" " * 32_769)
    output = tmp_path / "large-replay"
    with pytest.raises(ValueError, match="32 KiB"):
        cargo_demo.build(output, candidate=source, media=False)
    assert not output.exists()


@pytest.mark.parametrize("fps", [0, 61, True, 1.5])
def test_invalid_frame_rates_create_no_artifacts(tmp_path, fps):
    output = tmp_path / "invalid"
    with pytest.raises(ValueError, match="fps"):
        cargo_demo.build(output, fps=fps)
    assert not output.exists()
