"""Regression for gentle pad-B departure being mislabeled as an in-flight impact."""

import numpy as np
import pytest

from investigation.tasks.drone import Adapter
from simulator.platforms.drone_delivery import _is_initial_departure_support


@pytest.fixture(scope="module")
def runs():
    adapter = Adapter()
    # Frozen Sol controller from the completed 2026-09-10 comparison. Its
    # commands are unchanged; only the host contact classification is repaired.
    sol = dict(adapter.initial_candidate(), attitude_damping=1.2,
               hover_attitude_gain=7., route_attitude_gain=3.5,
               trim_integral_gain=6., trim_limit_nm=1.2,
               trim_during_route=True, reset_trim_on_release=True,
               reset_trim_on_support=False)
    completed = []
    for candidate in (adapter.initial_candidate(), sol):
        sim = adapter.create_sim(candidate)
        while not sim.finished:
            sim.step()
        completed.append(sim)
    return adapter, completed[0], completed[1]


def test_frozen_sol_controller_completes_without_penalizing_gentle_pad_departure(runs):
    adapter, _, sol = runs
    assert adapter.outcome(sol)["goal_achieved"] is True
    assert sol.summary()["public"]["outcome"] == "mission_complete"
    assert sol.controlled_departure_contact_steps == 9
    assert sol.in_flight_body_contacts == 0
    assert sol.first_forbidden_contact is None
    assert sol.crash_event is None
    assert sol.unloaded_departed and sol.unloaded_flight
    assert sol.unloaded_liftoff_time > sol.release_time + 1.
    assert sol.parcel_settled_time >= 2.
    assert sol.landed_time >= 1.
    assert sol.elapsed <= adapter.duration_s
    assert np.linalg.norm(sol.data.qpos[:2]) < .5
    assert np.linalg.norm(sol.data.qvel[:3]) < .1
    observed = adapter.observe(sol)
    assert observed["unloaded_departed"] is True
    assert observed["controlled_departure_contact_steps"] == 9
    assert observed["in_flight_body_contacts"] == 0
    events = sol.public_events()
    clearance = next(event for event in events if event["event"] == "unloaded_liftoff")
    assert clearance["phase"] == "unloaded_climb"
    assert clearance["time"] == sol.unloaded_liftoff_time


def test_original_outbound_crash_remains_failed_and_has_contact_cause(runs):
    adapter, baseline, _ = runs
    assert adapter.outcome(baseline)["goal_achieved"] is False
    assert baseline.summary()["public"]["outcome"] == "crashed"
    assert baseline.crash_event is not None
    assert baseline.in_flight_body_contacts > 0
    assert baseline.controlled_departure_contact_steps == 0
    contact = baseline.first_forbidden_contact
    assert contact["event"] == "first_forbidden_contact"
    assert contact["phase"] == "outbound"
    assert contact["time"] <= baseline.crash_event["time"]
    assert contact["speed_m_s"] > 1.
    assert contact["contact_geoms"]
    assert len([event for event in baseline.public_events()
                if event["event"] == "first_forbidden_contact"]) == 1


SUPPORT = dict(phase="unloaded_climb", parcel_attached=False, cleared=False,
               position=[3.7, 0., .29], destination=[4., 0.], speed=.037,
               tilt=.86, contact_geoms=["leg_2", "leg_3"])


def test_allowance_is_for_initial_gentle_landing_gear_support_only():
    assert _is_initial_departure_support(**SUPPORT)
    assert _is_initial_departure_support(**dict(SUPPORT, position=[4., .65, .29], speed=.15, tilt=10.))


@pytest.mark.parametrize("change", [
    {"cleared": True}, {"phase": "return"}, {"parcel_attached": True},
    {"position": [4., .651, .29]}, {"speed": .151}, {"tilt": 10.01},
    {"contact_geoms": ["leg_2", "drone_body"]}, {"contact_geoms": []},
])
def test_recontact_off_pad_impact_tilt_and_airframe_contacts_are_not_exempt(change):
    assert not _is_initial_departure_support(**dict(SUPPORT, **change))


def test_departure_clearance_and_contact_evidence_reset_at_mission_boundaries():
    adapter = Adapter()
    sim = adapter.create_sim(adapter.initial_candidate())
    for reset in (sim.reset_full, sim.reset_trial):
        sim.unloaded_departed = True
        sim.unloaded_liftoff_time = 5.
        sim.controlled_departure_contact_steps = 9
        sim.first_forbidden_contact = {"time": 6., "event": "first_forbidden_contact"}
        sim.in_flight_body_contacts = 1
        reset()
        assert sim.unloaded_departed is False
        assert sim.unloaded_liftoff_time is None
        assert sim.controlled_departure_contact_steps == 0
        assert sim.first_forbidden_contact is None
        assert sim.in_flight_body_contacts == 0


def test_finished_contact_violation_is_distinguished_from_incomplete_mission():
    adapter = Adapter()
    sim = adapter.create_sim(adapter.initial_candidate())
    # Classification only: do not rerun a full mission to exercise the labels.
    sim.trial_time = sim.config.duration
    assert sim.summary()["public"]["outcome"] == "mission_timeout"
    sim.in_flight_body_contacts = 1
    assert sim.summary()["public"]["outcome"] == "contact_violation"
    assert adapter.outcome(sim)["goal_achieved"] is False
    sim.crash_event = {"time": 2., "impact_speed": 3.}
    assert sim.summary()["public"]["outcome"] == "crashed"
