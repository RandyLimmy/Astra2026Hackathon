"""Public-feedback controller boundary for the original parcel delivery mission.

The adapter owns the immutable physical task. A candidate changes only the
bounded feedback policy; no private loading parameter or builder policy is
available through the candidate or its source description.
"""

from copy import deepcopy
import inspect
import math

import numpy as np

from simulator.platforms.drone import Config, PRESETS
from simulator.platforms.drone_delivery import DeliverySimulation


SCENARIO = "drone_delivery_imbalance"
PLATFORM = "drone"
LABEL = "Drone"

_BOUNDS = {
    "position_gain_xy": (0.1, 8.),
    "position_gain_z": (0.1, 20.),
    "velocity_gain_xy": (0.1, 10.),
    "velocity_gain_z": (0.1, 12.),
    "hover_attitude_gain": (0.1, 8.),
    "route_attitude_gain": (0.1, 8.),
    "attitude_damping": (0.01, 2.),
    "yaw_gain": (0.05, 3.),
    "yaw_damping": (0.01, 1.),
    "trim_integral_gain": (0., 8.),
    "trim_limit_nm": (0., 1.4),
    "vertical_integral_gain": (0., 10.),
    "vertical_integral_limit_n": (0., 10.),
}
_BOOLEAN_FIELDS = ("trim_during_route", "reset_trim_on_release", "reset_trim_on_support")
_BASELINE = {
    "position_gain_xy": 3., "position_gain_z": 8.,
    "velocity_gain_xy": 3.2, "velocity_gain_z": 5.,
    "hover_attitude_gain": 3.5, "route_attitude_gain": .9,
    "attitude_damping": .48, "yaw_gain": .8, "yaw_damping": .18,
    "trim_integral_gain": 2., "trim_limit_nm": .95,
    "vertical_integral_gain": 3., "vertical_integral_limit_n": 8.,
    "trim_during_route": False, "reset_trim_on_release": False, "reset_trim_on_support": False,
}


def controller(observation: dict, target_position: list, candidate: dict,
               memory: dict, dt: float) -> list:
    """Map public measurements + mission target to four bounded rotor commands.

    ``memory`` starts empty on each full mission. It is controller-owned state,
    never a physics object. The host supplies the current mission target before
    advancing physics; candidates cannot change mission sequencing or release
    criteria. Quaternion order is w,x,y,z and angular velocity is body-frame.
    """
    phase = observation["phase"]
    if phase in ("ready", "crash_aftermath", "mission_complete"):
        return [0., 0., 0., 0.]
    position = np.asarray(observation["position"])
    velocity = np.asarray(observation["velocity"])
    omega = np.asarray(observation["angular_velocity"])
    w, x, y, z = observation["quaternion"]
    rotation = np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])
    error_position = np.asarray(target_position) - position
    acceleration = np.array([candidate["position_gain_xy"]] * 2 +
                            [candidate["position_gain_z"]]) * error_position
    acceleration -= np.array([candidate["velocity_gain_xy"]] * 2 +
                             [candidate["velocity_gain_z"]]) * velocity
    acceleration = np.clip(acceleration, [-3., -3., -4.], [3., 3., 6.])
    attached = observation["parcel_attached"]
    if memory.get("attached", attached) and not attached:
        memory["vertical_integral"] = 0.
        if candidate["reset_trim_on_release"]:
            memory["attitude_integral"] = np.zeros(3)
    memory["attached"] = attached
    supported = phase == "placement"
    vertical_integral = float(memory.get("vertical_integral", 0.))
    if supported:
        vertical_integral = 0.
    else:
        vertical_integral = float(np.clip(
            vertical_integral + candidate["vertical_integral_gain"] * error_position[2] * dt,
            -candidate["vertical_integral_limit_n"], candidate["vertical_integral_limit_n"]))
    memory["vertical_integral"] = vertical_integral
    # The nominal bare airframe mass is part of the public actuator interface.
    force = 1.2 * (acceleration + [0., 0., 9.81]) + [0., 0., vertical_integral]
    desired_z = force / np.linalg.norm(force)
    desired_y = np.cross(desired_z, [1., 0., 0.])
    desired_y /= np.linalg.norm(desired_y)
    desired = np.column_stack((np.cross(desired_y, desired_z), desired_y, desired_z))
    error_matrix = (desired.T @ rotation - rotation.T @ desired) / 2
    error = np.array([error_matrix[2, 1], error_matrix[0, 2], error_matrix[1, 0]])
    hover = phase in ("takeoff", "unloaded_climb", "landing")
    gain = candidate["hover_attitude_gain"] if hover else candidate["route_attitude_gain"]
    torque = -np.array([gain, gain, candidate["yaw_gain"]]) * error
    torque -= np.array([candidate["attitude_damping"]] * 2 + [candidate["yaw_damping"]]) * omega
    trim = np.asarray(memory.get("attitude_integral", np.zeros(3))).copy()
    if supported and candidate["reset_trim_on_support"]:
        trim[:] = 0.
    elif hover or candidate["trim_during_route"]:
        trim = np.clip(trim - candidate["trim_integral_gain"] * error * dt,
                       -candidate["trim_limit_nm"], candidate["trim_limit_nm"])
        torque += trim
    else:
        trim[:] = 0.
    memory["attitude_integral"] = trim
    torque = np.clip(torque, [-1.4, -1.4, -.15], [1.4, 1.4, .15])
    collective = max(0., float(force @ rotation[:, 2]))
    # Public rotor order FL, FR, RR, RL; arms .24 m, reaction ratio .018 m.
    mixer = np.array([[1., 1., 1., 1.], [.24, -.24, -.24, .24],
                      [-.24, -.24, .24, .24], [.018, -.018, .018, -.018]])
    commands = np.clip(np.linalg.solve(mixer, np.r_[collective, torque]) / 6., 0., 1.)
    if phase == "landing" and observation["ground_contact"] and np.linalg.norm(velocity) < .15:
        commands[:] = 0.
    return commands.tolist()


class CandidateSimulation(DeliverySimulation):
    def __new__(cls, config: Config, candidate: dict):
        return object.__new__(cls)

    def __init__(self, config: Config, candidate: dict):
        self.candidate = deepcopy(candidate)
        self.controller_memory: dict = {}
        super().__init__(config)

    def _reset_mission(self):
        self.controller_memory = {}
        super()._reset_mission()
        self.command_mode = "candidate_feedback_control"

    def _delivery_control(self, target: np.ndarray) -> np.ndarray:
        commands = controller(self.observe(), target.tolist(), self.candidate,
                              self.controller_memory, self.config.timestep)
        # Reuse the physical actuator validator: policies cannot exceed limits.
        _, validated = self._parse_delivery_control({"rotor_commands": commands})
        return validated


class Adapter:
    duration_s = 30.
    cameras = ("side", "overview")
    source_paths = (
        "investigation/tasks/drone.py", "simulator/platforms/drone_delivery.py",
        "simulator/platforms/drone.py", "simulator/assets/platforms/drone_delivery.xml",
        "simulator/public/CONTROL_TASKS.md",
    )

    def initial_candidate(self) -> dict:
        return deepcopy(_BASELINE)

    def validate_candidate(self, candidate: dict) -> dict:
        expected = set(_BOUNDS) | set(_BOOLEAN_FIELDS)
        if not isinstance(candidate, dict) or set(candidate) != expected:
            raise ValueError(f"candidate must contain exactly {sorted(expected)}")
        for field, (low, high) in _BOUNDS.items():
            value = candidate[field]
            if (type(value) not in (int, float) or not math.isfinite(value)
                    or not low <= value <= high):
                raise ValueError(f"{field} must be a finite number in [{low}, {high}]")
        for field in _BOOLEAN_FIELDS:
            if type(candidate[field]) is not bool:
                raise ValueError(f"{field} must be boolean")
        return deepcopy(candidate)

    def capabilities(self) -> dict:
        return {
            "platform": PLATFORM, "scenario": SCENARIO, "label": LABEL,
            "goal": "Take off at A with the parcel, release it supported at B, return unloaded, and land at A.",
            "criteria": [
                "Complete the unchanged mission within 30 simulation seconds without an in-flight body contact or crash.",
                "Controlled landing and initial gear support while departing B are allowed. Before unloaded clearance at 0.55 m, only landing-gear contact within 0.65 m of B, at speed at most 0.15 m/s and tilt at most 10 degrees is permitted; off-pad contact, airframe contact and contact after clearance remain violations.",
                "Parcel is released through its physical latch, within 0.5 m of B, supported and settled for at least 2 seconds.",
                "Drone flies unloaded, then lands within 0.5 m of A at less than 0.1 m/s for at least 1 second.",
            ],
            "duration_s": self.duration_s,
            "control_interface": {
                "output": "Four normalized rotor commands in [0,1], ordered FL, FR, RR, RL.",
                "input": "Public pose, velocities, IMU, contact and parcel measurements, current mission phase and target.",
                "memory": "Controller memory persists between mission phases and resets on a new physical run.",
                "task_boundary": "Mission schedule, release conditions, physical world and initial state are fixed by the host.",
                "baseline_provenance": "Public-feedback port of the nominal phase-based attitude loop; vertical integral compensation replaces access to private load mass. The initial policy retains the recorded loss of control after takeoff but is not a bit-identical stock-controller replay.",
            },
            "parameter_bounds": {key: {"minimum": low, "maximum": high} for key, (low, high) in _BOUNDS.items()},
            "boolean_parameters": list(_BOOLEAN_FIELDS),
            "initial_candidate": self.initial_candidate(),
            "controller_source": inspect.getsource(controller),
        }

    def create_sim(self, candidate: dict) -> CandidateSimulation:
        validated = self.validate_candidate(candidate)
        return CandidateSimulation(Config(**PRESETS[SCENARIO]), validated)

    def observe(self, sim: CandidateSimulation) -> dict:
        return sim.observe()

    def outcome(self, sim: CandidateSimulation) -> dict:
        summary = deepcopy(sim.summary()["public"])
        summary["provenance"] = "candidate_feedback_control"
        return {"goal_achieved": bool(sim.completed),
                "partial_success": bool(sim.release_time is not None and not sim.completed and sim.crash_event is None),
                "summary": summary}

    def public_events(self, sim: CandidateSimulation) -> list:
        return sim.public_events()
