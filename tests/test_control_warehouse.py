"""Focused contract and physical feasibility checks, without rendering or API calls."""
from copy import deepcopy
import math

import numpy as np
import pytest

from investigation.tasks.warehouse import Adapter, controller
from simulator.platforms import warehouse


def rollout(adapter, candidate):
    sim = adapter.create_sim(candidate)
    while not sim.finished:
        sim.step()
    return sim, adapter.outcome(sim)


@pytest.fixture(scope="module")
def physical_results():
    adapter = Adapter()
    initial = adapter.initial_candidate()
    before, failure = rollout(adapter, initial)
    candidate = {**initial, "target_speed_m_s": .6}
    after, success = rollout(adapter, candidate)
    stopped, parking = rollout(adapter, {**initial, "target_speed_m_s": 0.})
    return adapter, before, failure, after, success, stopped, parking


def test_speed_fix_retains_cargo_and_finishes_original_route(physical_results):
    adapter, before, failure, after, success, _, _ = physical_results
    assert not failure["goal_achieved"]
    assert failure["summary"]["metrics"]["cargo_has_touched_floor"]
    assert success["goal_achieved"]
    assert not success["partial_success"]
    assert success["summary"]["metrics"]["route_complete"]
    assert success["summary"]["metrics"]["cargo_retained"]
    assert before.config == after.config == warehouse.Config(**warehouse.PRESETS["warehouse_curve_demo"])
    np.testing.assert_array_equal(before.model.body_mass, after.model.body_mass)
    np.testing.assert_array_equal(before.model.geom_friction, after.model.geom_friction)
    np.testing.assert_array_equal(before.model.actuator_gear, after.model.actuator_gear)
    assert any(event["event"] == "cargo_ground_impact" for event in adapter.public_events(before))
    assert not any(event["event"] == "cargo_ground_impact" for event in adapter.public_events(after))


def test_parking_and_unfinished_trials_cannot_pass(physical_results):
    adapter, _, _, _, _, stopped, parking = physical_results
    assert parking["summary"]["metrics"]["cargo_retained"]
    assert not parking["summary"]["metrics"]["route_complete"]
    assert not parking["goal_achieved"]
    assert not parking["partial_success"]
    fresh = adapter.create_sim(adapter.initial_candidate())
    assert not adapter.outcome(fresh)["goal_achieved"]
    assert not adapter.outcome(fresh)["partial_success"]


def test_initial_controller_matches_original_measured_command():
    adapter = Adapter()
    original = warehouse.Simulation(warehouse.Config(**warehouse.PRESETS["warehouse_curve_demo"]))
    observed = original.observe()
    for when, position, progress, heading in ((0., [0., 0., .3], 0., 0.),
                                             (3., [2., .02, .3], 2., -.01),
                                             (5., [3.8, -.4, .3], 3.9, -.6),
                                             (14., [4.2, -3.6, .3], warehouse.ROUTE_LENGTH, -math.pi / 2)):
        observed.update(trial_time=when, position=position, route_progress=progress, heading=heading)
        actual = controller(adapter.initial_candidate(), observed)
        expected = warehouse.route_command("cargo_curve", observed)
        assert actual == pytest.approx(expected, abs=1e-13)


def test_revised_parcel_evidence_and_tracks_are_in_protocol():
    adapter = Adapter()
    sim = adapter.create_sim(adapter.initial_candidate())
    sim.step()
    observed, diagnostics = adapter.observe(sim), sim.diagnostics()
    for name in ("cargo_linear_velocity", "cargo_angular_velocity", "cargo_tilt_rad"):
        assert name in adapter.capabilities()["observables"]
        assert observed[name] == diagnostics[name]
    assert adapter.cameras[0] == "side"
    assert "simulator/platforms/warehouse_tracks.py" in adapter.source_paths
    assert sim.model.geom("cargo_deck").pos[2] == .20
    assert adapter.capabilities()["controller_context"]["deck_center_limits_in_chassis_m"]["z"][0] == .22


@pytest.mark.parametrize("change", [
    {"target_speed_m_s": float("nan")}, {"target_speed_m_s": float("inf")},
    {"target_speed_m_s": True}, {"target_speed_m_s": -1}, {"target_speed_m_s": 3},
    {"acceleration_m_s2": 0}, {"lookahead_m": "0.6"}, {"schema_version": True},
    {"latch_strength": 10000}, {"duration": 30},
])
def test_candidate_validation_rejects_nonfinite_and_world_edits(change):
    adapter = Adapter()
    with pytest.raises(ValueError):
        adapter.validate_candidate({**adapter.initial_candidate(), **change})


def test_contract_is_complete_and_contains_no_hidden_failure_parameters(physical_results):
    adapter, before, _, _, _, _, _ = physical_results
    with pytest.raises(ValueError):
        adapter.validate_candidate({"schema_version": 1, "target_speed_m_s": .6})
    capabilities = adapter.capabilities()
    assert capabilities["scenario"] == "warehouse_curve_demo"
    assert capabilities["duration_s"] == 17.
    assert set(capabilities["candidate_fields"]) == {"target_speed_m_s", "acceleration_m_s2", "lookahead_m"}
    exposed = repr([capabilities, adapter.observe(before), adapter.public_events(before)])
    for hidden in ("latch_strength", "latch_dwell", "latch_arm_at", "constraint_force_N", "cargo_curve_slow"):
        assert hidden not in exposed
    candidate = adapter.initial_candidate()
    sim = adapter.create_sim(candidate)
    saved = deepcopy(sim.controller_candidate)
    candidate["target_speed_m_s"] = 0.
    assert sim.controller_candidate == saved
