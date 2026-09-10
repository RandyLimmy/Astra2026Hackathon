"""Neutral maintenance bridge, actual component repair, and independent model rollouts."""

from copy import deepcopy
import json

import numpy as np
import pytest

from investigation.platform_physics import PlatformPhysics


@pytest.mark.parametrize("platform", ["car", "drone", "quadruped"])
def test_capabilities_and_initial_records_are_neutral(platform, tmp_path):
    physics = PlatformPhysics(platform, tmp_path, record_frames=False)
    caps = physics.capabilities()
    assert caps["default_probe"] in caps["probes"]
    assert caps["repair_actions"] and caps["model_parameters"] and caps["components"]
    assert all(spec["min"] <= spec["nominal"] <= spec["max"] for spec in caps["model_parameters"].values())
    assert all(spec["units"] for spec in caps["model_parameters"].values())
    evidence = physics.initial_evidence()
    assert set(evidence) == {"healthy", "observed"}
    for record in evidence.values():
        assert record["probe"] == physics.default_probe
        assert record["duration_s"] == physics.default_duration_s
        assert 2 <= len(record["observations"]) <= 200
        assert record["observations"][0]["t_s"] == 0
        assert record["observations"][-1]["t_s"] == physics.default_duration_s
        assert record["frames"] == []
        assert json.loads((tmp_path / record["id"] / "record.json").read_text()) == record
        for private in ('"fault"', '"scenario"', '"diagnostics"', '"config"', '"events"',
                        '"rotor_effectiveness"', '"rack_gain"', '"actuator_strength"'):
            assert private not in json.dumps(record)
    assert evidence["healthy"]["summary"]["safe"] is True
    assert evidence["healthy"]["summary"] != evidence["observed"]["summary"]
    assert physics.initial_evidence() == evidence
    component = caps["components"][0]["id"]
    observed = physics.observe(component)
    assert [row["id"] for row in observed["components"]] == [component]
    assert "fault" not in json.dumps(observed)


def test_car_wrong_target_is_not_a_repair_or_diagnostic_oracle(tmp_path):
    physics = PlatformPhysics("car", tmp_path, record_frames=False)
    evidence = physics.initial_evidence()
    before = physics._actual.model.body("toe_FL").quat.copy()
    wrong = physics.apply_repair("align_wheel", "wheel_FR")
    np.testing.assert_array_equal(physics._actual.model.body("toe_FL").quat, before)
    still_changed = physics.run_experiment("slalom", 10)
    right = physics.apply_repair("align_wheel", "wheel_FL")
    fixed = physics.run_experiment("slalom", 10)
    assert set(right) == set(wrong)
    assert right["message"] == wrong["message"] and right["verification_required"] is True
    assert still_changed["summary"]["metrics"]["max_lateral_displacement"] > 5
    assert fixed["summary"]["safe"] is True
    assert fixed["summary"]["metrics"]["max_lateral_displacement"] == pytest.approx(
        evidence["healthy"]["summary"]["metrics"]["max_lateral_displacement"], abs=1e-7)
    assert physics._actual._damaged is True
    assert len([event for event in physics._actual._events if event["event"] == "structural_damage"]) == 1


@pytest.mark.parametrize("scenario,action,target,parameter", [
    ("car_steering_damage", "calibrate_steering", "steering_rack", "steering_gain"),
    ("car_suspension_damage", "service_suspension", "wheel_FL", "spring_FL_scale"),
    ("car_tire_pressure", "replace_tire", "wheel_FL", "tire_FL_radius_scale"),
])
def test_other_car_maintenance_actions_restore_selected_parts(tmp_path, scenario, action, target, parameter):
    physics = PlatformPhysics("car", tmp_path, scenario=scenario, record_frames=False)
    physics._prepare_actual()
    # Independent alignment is retained by unrelated maintenance.
    physics._actual.model.body("toe_RR").quat[:] = [.99, 0, 0, .1]
    retained = physics._actual.model.body("toe_RR").quat.copy()
    physics.apply_repair(action, target)
    np.testing.assert_array_equal(physics._actual.model.body("toe_RR").quat, retained)
    actual, nominal = physics._actual, physics._nominal
    if parameter == "steering_gain":
        assert actual._rack_gain == 1 and actual._rack_bias == 0
    elif parameter == "spring_FL_scale":
        joint = actual.model.joint("suspension_FL").id
        assert actual.model.jnt_stiffness[joint] == nominal.model.jnt_stiffness[joint]
        assert actual.model.dof_damping[actual.model.jnt_dofadr[joint]] == nominal.model.dof_damping[nominal.model.jnt_dofadr[joint]]
    else:
        for attr in ("size", "friction", "solref"):
            np.testing.assert_array_equal(getattr(actual.model.geom("tire_FL"), attr), getattr(nominal.model.geom("tire_FL"), attr))
    assert actual._damaged


def test_complete_wheel_replacement_targets_only_the_selected_assembly(tmp_path):
    physics = PlatformPhysics("car", tmp_path, record_frames=False)
    physics._prepare_actual()
    model, nominal = physics._actual.model, physics._nominal.model
    damaged_alignment = model.body("toe_FL").quat.copy()
    physics.apply_repair("replace_wheel", "wheel_FR")
    np.testing.assert_array_equal(model.body("toe_FL").quat, damaged_alignment)
    joint = model.joint("suspension_FL").id
    dof = model.jnt_dofadr[joint]
    model.jnt_stiffness[joint] *= .4
    model.dof_damping[dof] *= .5
    model.geom("tire_FL").friction[0] = .2
    model.geom("tire_FL").size[[0, 2]] *= .8
    model.geom("tire_FL").solref[0] = .045
    physics._actual._rack_gain = .6
    model.body("toe_RR").quat[:] = [np.cos(.1), 0, 0, np.sin(.1)]
    unrelated = model.body("toe_RR").quat.copy()
    physics.apply_repair("replace_wheel", "wheel_FL")
    assert model.jnt_stiffness[joint] == nominal.jnt_stiffness[joint]
    assert model.dof_damping[dof] == nominal.dof_damping[dof]
    np.testing.assert_array_equal(model.body("toe_FL").quat, nominal.body("toe_FL").quat)
    for attr in ("size", "friction", "solref"):
        np.testing.assert_array_equal(getattr(model.geom("tire_FL"), attr), getattr(nominal.geom("tire_FL"), attr))
    np.testing.assert_array_equal(model.body("toe_RR").quat, unrelated)
    assert physics._actual._rack_gain == .6 and physics._actual._damaged


@pytest.mark.parametrize("scenario,action,target", [
    ("drone_rotor_loss", "replace_rotor", "rotor_FL"),
    ("drone_voltage_sag", "replace_battery", "battery"),
    ("drone_payload", "unload_payload", "payload"),
    ("drone_wind", "shelter_from_wind", "environment"),
    ("drone_delay", "service_command_link", "command_link"),
])
def test_drone_maintenance_restores_flight_and_does_not_reinject_fault(tmp_path, scenario, action, target):
    physics = PlatformPhysics("drone", tmp_path, scenario=scenario, record_frames=False)
    receipt = physics.apply_repair(action, target)
    assert receipt["status"] == "applied"
    record = physics.run_experiment("hover", 2)
    assert record["summary"]["safe"] is True
    assert record["summary"]["metrics"]["final_altitude"] == pytest.approx(2, abs=0.001)
    assert physics._actual.event_applied
    assert len(physics._actual.events) == 1


def test_wrong_drone_rotor_replacement_retains_failed_rotor(tmp_path):
    physics = PlatformPhysics("drone", tmp_path, record_frames=False)
    physics.apply_repair("replace_rotor", "rotor_FR")
    assert physics._actual.effectiveness[0] == .12
    assert physics.run_experiment("hover", 2)["summary"]["safe"] is False


@pytest.mark.parametrize("scenario,action,target", [
    ("quadruped_joint_weakness", "replace_actuator", "FL_knee"),
    ("quadruped_foot_slip", "replace_footpad", "foot_FL"),
    ("quadruped_leg_damage", "service_joint", "FL_knee"),
    ("quadruped_payload_shift", "secure_payload", "payload"),
])
def test_quadruped_maintenance_restores_named_parameters_and_retains_guard(tmp_path, scenario, action, target):
    physics = PlatformPhysics("quadruped", tmp_path, scenario=scenario, record_frames=False)
    physics.apply_repair(action, target)
    actual, nominal = physics._actual.model, physics._nominal.model
    for field in ("actuator_gainprm", "geom_friction", "jnt_stiffness", "qpos_spring"):
        np.testing.assert_array_equal(getattr(actual, field), getattr(nominal, field))
    result = physics.run_experiment("stand", 2)
    assert result["summary"]["safe"] is True
    assert physics._actual._fault_active and len(physics._actual.events) == 1


@pytest.mark.parametrize("platform,parameters", [
    ("car", {"toe_FL_rad": .2, "spring_FL_scale": .3, "tire_FL_radius_scale": .8}),
    ("drone", {"rotor_FL_gain": .8, "payload_kg": .1, "delay_s": .02}),
    ("quadruped", {"motor_FL_knee_gain": .8, "foot_FL_friction": .2}),
])
def test_fixed_parameters_and_constant_own_observation_callback_match(tmp_path, platform, parameters):
    physics = PlatformPhysics(platform, tmp_path, record_frames=False)
    fixed = physics.run_model(physics.default_probe, 2, parameters)
    feedback = []
    def model(observation, dt):
        feedback.append((deepcopy(observation), dt))
        return parameters
    dynamic = physics.run_model(physics.default_probe, 2, model)
    assert dynamic["summary"] == fixed["summary"]
    assert dynamic["observations"] == fixed["observations"]
    assert feedback[0][1] == 0 and feedback[1][1] == 0
    assert all(dt == pytest.approx(.02, abs=1e-7) for _, dt in feedback[2:])
    assert len(feedback) == 101
    assert physics._incident_ready is False  # model never executes reference preparation
    assert all('"fault"' not in json.dumps(obs) for obs, _ in feedback)


def test_model_prediction_does_not_copy_hidden_reference_properties(tmp_path):
    physics = PlatformPhysics("drone", tmp_path, record_frames=False)
    physics._prepare_actual()
    before = physics._actual.data.qpos.copy()
    hidden = physics._actual.effectiveness.copy()
    healthy = physics.reference_probe("hover", 2)
    model = physics.run_model("hover", 2, {})
    assert model["summary"] == healthy["summary"]
    np.testing.assert_array_equal(physics._actual.effectiveness, hidden)
    np.testing.assert_array_equal(physics._actual.data.qpos, before)


def test_public_validation_precedes_actual_mutation(tmp_path):
    physics = PlatformPhysics("car", tmp_path, record_frames=False)
    for call in (lambda: physics.run_experiment("unknown", 2),
                 lambda: physics.run_experiment("slalom", True),
                 lambda: physics.apply_repair("repair_all", "wheel_FL"),
                 lambda: physics.apply_repair("align_wheel", "body"),
                 lambda: physics.observe("not_a_component"),
                 lambda: physics.run_model("slalom", 2, {"fault": 0}),
                 lambda: physics.run_model("slalom", 2, {"toe_FL_rad": True}),
                 lambda: physics.run_model("slalom", 2, {"toe_FL_rad": float("nan")}),
                 lambda: physics.run_model("slalom", 2, lambda obs, dt: {"steering_gain": 100})):
        with pytest.raises(ValueError):
            call()
    assert physics._actual.elapsed == 0 and physics._incident_ready is False


def test_invalid_host_scenario_is_rejected(tmp_path):
    with pytest.raises(ValueError):
        PlatformPhysics("car", tmp_path, scenario="drone_rotor_loss", record_frames=False)


def test_frame_records_are_relative_jpeg_artifacts(tmp_path, monkeypatch):
    class Renderer:
        def __init__(self, model, width, height):
            self.width, self.height = width, height
        def update_scene(self, data, camera):
            assert camera in {"side", "chase"}
        def render(self):
            return np.zeros((self.height, self.width, 3), dtype=np.uint8)
        def close(self):
            pass
    monkeypatch.setattr("investigation.platform_physics.mujoco.Renderer", Renderer)
    physics = PlatformPhysics("drone", tmp_path, record_frames=True)
    record = physics.reference_probe("hover", 2)
    assert 30 <= len(record["frames"]) <= 32
    assert record["frames"][0]["t_s"] == 0 and record["frames"][-1]["t_s"] == 2
    for frame in record["frames"]:
        assert frame["file"].startswith(record["id"] + "/frames/")
        assert (tmp_path / frame["file"]).read_bytes().startswith(b"\xff\xd8")
