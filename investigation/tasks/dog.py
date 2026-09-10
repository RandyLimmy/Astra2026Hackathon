"""Public controller-editing adapter for the original walking task.

Candidates affect only the documented gait parameter. The task, physical model,
speed request and success evaluation remain the scenario author's originals.
"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import replace
from pathlib import Path
from typing import Any

from simulator.platforms import quadruped_controller
from simulator.platforms.quadruped import (
    Config,
    JOINTS,
    LEGS,
    PRESETS,
    Simulation,
    TASK_MAX_LATERAL_M,
    TASK_MAX_TILT_DEG,
    TASK_MIN_PRE_TRANSITION_DISTANCE_M,
    TASK_SPEED_TOLERANCE,
    TASK_SPEED_WINDOW_S,
)


SCENARIO = "quadruped_gait_failure"
PLATFORM = "quadruped"
LABEL = "Robot dog"


class Adapter:
    """Create isolated attempts of the same fixed, full-duration dog task."""

    scenario = SCENARIO
    platform = PLATFORM
    label = LABEL
    duration_s = float(PRESETS[SCENARIO]["duration"])
    cameras = ("side", "overview")
    source_paths = (
        "investigation/tasks/dog.py",
        "simulator/platforms/quadruped.py",
        "simulator/platforms/quadruped_controller.py",
        "simulator/assets/platforms/quadruped_gait.xml",
        "simulator/assets/platforms/quadruped.xml",
        "contracts/QUADRUPED_CONTROLLER.md",
    )

    def __init__(self) -> None:
        self._config = Config(**PRESETS[SCENARIO])

    def initial_candidate(self) -> dict[str, float]:
        return self.validate_candidate(dict(quadruped_controller.DEFAULT_PARAMETERS))

    def validate_candidate(self, candidate: dict) -> dict[str, float]:
        if not isinstance(candidate, dict) or set(candidate) != set(quadruped_controller.DEFAULT_PARAMETERS):
            raise ValueError("Dog controller must contain exactly rear_cadence_gain")
        # The controller validator rejects booleans, non-finite and out-of-range
        # numbers. Unlike the simulator's internal defaults, this public editing
        # interface requires every documented field to be present.
        try:
            return quadruped_controller.validate_parameters(candidate)
        except OverflowError as error:
            raise ValueError("rear_cadence_gain must be a finite number between 0 and 4") from error

    def capabilities(self) -> dict[str, Any]:
        config = self._config
        return {
            "scenario": SCENARIO,
            "platform": PLATFORM,
            "label": LABEL,
            "goal": "Follow the original walking-speed schedule along the strip while remaining upright.",
            "duration_s": self.duration_s,
            "cameras": list(self.cameras),
            "editable_artifact": "controller.json",
            "candidate_parameters": {
                "rear_cadence_gain": {
                    "type": "number", "required": True, "minimum": 0.0, "maximum": 4.0,
                    "units": "dimensionless",
                    "description": "Scales the rear legs' cadence response to changes in requested speed.",
                },
            },
            "controller_source": Path(quadruped_controller.__file__).read_text(encoding="utf-8"),
            "task": {
                "movement_start_s": 1.0,
                "initial_speed_mps": config.speed,
                "speed_transition_at_s": config.speed_transition_at,
                "speed_ramp_duration_s": config.speed_ramp_duration,
                "final_speed_mps": config.accelerated_speed,
                "controller_edit_scope": "Gait-clock and foot-target parameters only; physical actuation is unchanged.",
            },
            "success_criteria": {
                "authoritative_field": "task_complete",
                "full_duration_required": True,
                "no_fall": True,
                "no_body_ground_contact": True,
                "no_simulation_warnings": True,
                "no_diagnostic_speed_override": True,
                "max_tilt_deg_exclusive": TASK_MAX_TILT_DEG,
                "max_lateral_distance_m_exclusive": TASK_MAX_LATERAL_M,
                "minimum_distance_before_speed_transition_m": TASK_MIN_PRE_TRANSITION_DISTANCE_M,
                "minimum_requested_distance_fraction": 1 - TASK_SPEED_TOLERANCE,
                "final_speed_window_s": TASK_SPEED_WINDOW_S,
                "final_speed_relative_tolerance": TASK_SPEED_TOLERANCE,
                "partial_success": "Completed upright with forward progress, but failed one or more original task criteria.",
            },
            "observable_interface": {
                "units": "Seconds, metres, metres per second, radians for joint angles; body_tilt_deg is degrees.",
                "leg_order": list(LEGS),
                "joints": list(JOINTS),
                "body": ["position", "quaternion", "linear_velocity", "imu", "body_tilt_deg", "body_contact"],
                "gait": ["joint_positions", "joint_velocities", "foot_positions", "foot_velocities_mps",
                         "foot_contacts", "commanded_leg_phase", "foot_targets", "commanded_stance"],
                "task": ["task_requested_speed_mps", "command_speed_mps", "actual_forward_speed_mps", "phase"],
                "actuation": "command.joint_torque_commands records the bounded motor torque commands.",
                "interpretation": "Foot contacts and body motion are measurements; commanded phases, stance and foot targets are controller outputs.",
            },
        }

    def create_sim(self, candidate: dict) -> Simulation:
        parameters = self.validate_candidate(candidate)
        return Simulation(replace(self._config, controller_parameters=parameters))

    def observe(self, sim: Simulation) -> dict[str, Any]:
        return deepcopy(sim.observe())

    def outcome(self, sim: Simulation) -> dict[str, Any]:
        result = sim.summary()
        summary = deepcopy(result["public"])
        summary["simulation_warnings"] = deepcopy(result["warnings"])
        achieved = bool(summary["task_complete"])
        partial = bool(not achieved and sim.finished and summary["safe"]
                       and not summary["diagnostic_speed_override"]
                       and summary["metrics"]["forward_distance_m"] >= TASK_MIN_PRE_TRANSITION_DISTANCE_M)
        return {"goal_achieved": achieved, "partial_success": partial, "summary": summary}

    def public_events(self, sim: Simulation) -> list[dict[str, Any]]:
        return sim.public_events()
