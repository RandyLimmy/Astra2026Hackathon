"""Focused adapter checks; no model APIs and no rendered frames."""

from copy import deepcopy
import json

import numpy as np
import pytest

from investigation.tasks.drone import Adapter, SCENARIO, controller
from simulator.platforms.drone import Config, PRESETS
from simulator.platforms.drone_delivery import DeliverySimulation


@pytest.fixture(scope="module")
def runs():
    adapter = Adapter()
    baseline = adapter.initial_candidate()
    correction = dict(baseline, trim_during_route=True, route_attitude_gain=3.5,
                      trim_limit_nm=1.4, reset_trim_on_release=True,
                      reset_trim_on_support=True)
    simulations = []
    for candidate in (baseline, correction):
        sim = adapter.create_sim(candidate)
        while not sim.finished:
            sim.step()
            assert np.isfinite(sim.data.qpos).all()
            assert np.all((sim.last_command >= 0.) & (sim.last_command <= 1.))
        simulations.append(sim)
    return adapter, simulations[0], simulations[1]


def test_baseline_retains_original_outbound_failure_on_original_physical_mission(runs):
    adapter, baseline, _ = runs
    original = DeliverySimulation(Config(**PRESETS[SCENARIO]))
    assert baseline.config == original.config
    np.testing.assert_array_equal(baseline.model.body_mass, original.model.body_mass)
    np.testing.assert_array_equal(baseline.model.body_inertia, original.model.body_inertia)
    np.testing.assert_array_equal(baseline.model.actuator_gear, original.model.actuator_gear)
    result = adapter.outcome(baseline)
    assert result["goal_achieved"] is False
    assert result["summary"]["outcome"] == "crashed"
    names = [item["event"] for item in baseline.public_events()]
    assert names.index("liftoff") < names.index("outbound") < names.index("first_instability") < names.index("crash")
    assert baseline.max_altitude > 2.
    assert baseline.max_forward_progress > 1.
    assert baseline.crash_event["impact_speed"] > 1.
    assert "not a bit-identical" in adapter.capabilities()["control_interface"]["baseline_provenance"]


def test_controller_adjustment_completes_delivery_release_return_and_landing(runs):
    adapter, baseline, corrected = runs
    result = adapter.outcome(corrected)
    assert result["goal_achieved"] is True
    assert result["partial_success"] is False
    assert corrected.elapsed <= adapter.duration_s
    assert corrected.config == baseline.config
    assert corrected.config.delivery_controller == "nominal"
    np.testing.assert_array_equal(corrected.model.body_mass, baseline.model.body_mass)
    np.testing.assert_array_equal(corrected.model.body_inertia, baseline.model.body_inertia)
    assert corrected.in_flight_body_contacts == 0
    assert corrected.crash_event is None
    assert corrected.unloaded_flight
    assert not corrected.attached
    assert corrected.parcel_settled_time >= 2.
    assert corrected.landed_time >= 1.
    assert np.linalg.norm(corrected.data.qpos[:2]) < .5
    assert np.linalg.norm(corrected.data.qvel[:3]) < .1
    release = corrected.release_state
    np.testing.assert_array_equal(release["qpos_before"], release["qpos_after"])
    np.testing.assert_array_equal(release["qvel_before"], release["qvel_after"])
    assert not corrected.data.warning.number.any()


def test_candidates_are_complete_finite_and_cannot_change_host_configuration():
    adapter = Adapter()
    initial = adapter.initial_candidate()
    for bad in (dict(initial, fault="healthy"), dict(initial, trim_limit_nm=float("nan")),
                dict(initial, trim_limit_nm=True), dict(initial, trim_limit_nm="1.4"),
                dict(initial, trim_limit_nm=2.), dict(initial, trim_during_route=1)):
        with pytest.raises(ValueError):
            adapter.validate_candidate(bad)
    incomplete = dict(initial)
    del incomplete["trim_limit_nm"]
    with pytest.raises(ValueError):
        adapter.validate_candidate(incomplete)
    returned = adapter.validate_candidate(initial)
    returned["trim_limit_nm"] = 0.
    assert initial == adapter.initial_candidate()


def test_public_interface_omits_private_plant_state_and_builder_reference():
    adapter = Adapter()
    sim = adapter.create_sim(adapter.initial_candidate())
    capabilities = adapter.capabilities()
    forbidden = {"fault", "payload_mass", "payload_offset", "parcel_mass", "parcel_offset",
                 "delivery_controller", "release_state", "rotor_effectiveness", "motor_state"}

    def check(value):
        if isinstance(value, dict):
            assert not forbidden.intersection(value)
            for child in value.values():
                check(child)
        elif isinstance(value, list):
            for child in value:
                check(child)

    for public in (capabilities, adapter.observe(sim), adapter.outcome(sim), adapter.public_events(sim)):
        check(public)
        json.dumps(public, allow_nan=False)
    for hidden in ("payload_mass", "payload_offset", "feasibility", "self.config", "self.data", "self.model"):
        assert hidden not in capabilities["controller_source"]


def test_controller_uses_only_public_input_and_memory_resets_at_full_boundary():
    adapter = Adapter()
    sim = adapter.create_sim(adapter.initial_candidate())
    for _ in range(400):
        sim.step()
    observation = adapter.observe(sim)
    before = deepcopy(observation)
    command = controller(observation, observation["command"]["target_position"],
                         adapter.initial_candidate(), {}, .002)
    assert len(command) == 4
    assert all(0. <= value <= 1. for value in command)
    assert observation == before
    assert sim.controller_memory
    sim.reset_full()
    assert sim.controller_memory == {}
    assert sim.elapsed == 0.
    assert sim.attached
