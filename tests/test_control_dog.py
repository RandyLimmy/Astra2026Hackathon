"""Focused boundary checks; full walking feasibility is an explicit engineering run."""

from dataclasses import asdict
import json

import numpy as np
import pytest

from investigation.tasks.dog import Adapter


@pytest.mark.parametrize("candidate", [None, [], {}, {"rear_cadence_gain": 1, "speed": 0},
    {"rear_cadence_gain": True}, {"rear_cadence_gain": "1"}, {"rear_cadence_gain": float("nan")},
    {"rear_cadence_gain": float("inf")}, {"rear_cadence_gain": -0.01}, {"rear_cadence_gain": 4.01},
    {"rear_cadence_gain": 10 ** 400}])
def test_rejects_incomplete_or_non_controller_candidate(candidate):
    with pytest.raises(ValueError):
        Adapter().validate_candidate(candidate)


def test_attempts_keep_task_and_physics_and_copy_candidate():
    adapter = Adapter()
    candidate = {"rear_cadence_gain": 0.5}
    original = adapter.create_sim(adapter.initial_candidate())
    changed = adapter.create_sim(candidate)
    candidate["rear_cadence_gain"] = 4.0
    assert changed.effective_controller_parameters == {"rear_cadence_gain": 0.5}
    a, b = asdict(original.config), asdict(changed.config)
    a.pop("controller_parameters")
    b.pop("controller_parameters")
    assert a == b
    assert a["probe"] == "walk" and a["duration"] == 18
    for field in ("body_mass", "body_inertia", "actuator_gainprm", "geom_friction", "jnt_stiffness"):
        np.testing.assert_array_equal(getattr(original.model, field), getattr(changed.model, field))
    changed.step()
    assert original.elapsed == 0
    assert not adapter.outcome(original)["goal_achieved"]
    assert not adapter.outcome(original)["partial_success"]
    for camera in adapter.cameras:
        assert original.model.camera(camera).id >= 0


def test_public_interface_contains_controller_and_measurements_without_private_diagnostics():
    adapter = Adapter()
    sim = adapter.create_sim(adapter.initial_candidate())
    sim.step()
    capabilities = adapter.capabilities()
    assert "class GaitController" in capabilities["controller_source"]
    assert capabilities["success_criteria"]["authoritative_field"] == "task_complete"
    evidence = {"observation": adapter.observe(sim), "outcome": adapter.outcome(sim), "events": adapter.public_events(sim)}
    serialized = json.dumps(evidence, allow_nan=False)
    for private in ("coordination_defect", "gait_coordination", "fault_at", "diagnostics", "controller_parameters"):
        assert private not in serialized
    assert "foot_contacts" in evidence["observation"]
    assert "joint_torque_commands" in evidence["observation"]["command"]
