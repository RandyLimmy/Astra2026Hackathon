"""Free-flight quadrotor with physical faults and an intentionally nominal controller.

Rotor thrust/reaction torque use MuJoCo site transmissions. A motor state models
rotor spool time; degradation, supply voltage and transport delay act before the
physical motors. No pose or velocity correction is applied during simulation.
"""

from collections import deque
from dataclasses import asdict, dataclass
import math
from pathlib import Path

import mujoco
import numpy as np


NOMINAL_MASS = 1.2
NOMINAL_INERTIA = np.array([0.022, 0.022, 0.042])
MAX_ROTOR_THRUST = 6.0
NOMINAL_MOTOR_TAU = 0.035
ROTOR_POSITIONS = np.array([[0.24, 0.24], [0.24, -0.24], [-0.24, -0.24], [-0.24, 0.24]])
MIXER = np.vstack((np.ones(4), ROTOR_POSITIONS[:, 1], -ROTOR_POSITIONS[:, 0],
                   np.array([0.018, -0.018, 0.018, -0.018])))
INVERSE_MIXER = np.linalg.inv(MIXER)
FAULTS = ("healthy", "rotor_loss", "voltage_sag", "payload", "wind", "delay")

# Fixed commanded route: time, x/distance, y/distance, fraction of the climb
# from the initial 2 m hover to flight_altitude. Segment ends ease the commanded
# velocity over 10% of their duration. These are targets, never imposed poses.
SHOWCASE_ROUTE = ((0., 0., 0., 0.), (2., 0., 0., 0.), (8., 1., 0., 1.),
                  (12., 1., .4, 1.), (18., 0., 0., 0.), (20., 0., 0., 0.))
SHOWCASE_PHASES = ((0., "hover_settle"), (2., "outbound_climb"), (8., "cross_course"),
                   (12., "return"), (18., "settle_hover"))


def showcase_phase(trial_time: float) -> str:
    """Public route stage, independent of the fault setting or observed outcome."""
    return next((label for start, label in reversed(SHOWCASE_PHASES) if trial_time + 1e-10 >= start),
                SHOWCASE_PHASES[0][1])


@dataclass(frozen=True)
class Config:
    fault: str = "healthy"
    duration: float = 12.0
    timestep: float = 0.002
    fault_at: float = 3.0
    probe: str = "maneuver"
    rotor_index: int = 0
    rotor_effectiveness: float = 0.12
    voltage_ratio: float = 0.60
    payload_mass: float = 1.4
    wind_force: float = 3.0
    control_delay: float = 0.22
    flight_distance: float = 4.0
    flight_altitude: float = 2.8
    payload_offset: float = 0.30
    delivery_controller: str = "nominal"

    def __post_init__(self):
        if self.fault not in FAULTS:
            raise ValueError(f"fault must be one of {FAULTS}")
        if self.probe not in ("hover", "maneuver", "showcase", "delivery"):
            raise ValueError("probe must be hover, maneuver, showcase or delivery")
        if self.delivery_controller not in ("nominal", "feasibility"):
            raise ValueError("delivery_controller must be nominal or feasibility")
        bounds = {"duration": (0.05, 120), "timestep": (0.0002, 0.005),
                  "fault_at": (0, 120), "rotor_effectiveness": (0, 1),
                  "voltage_ratio": (0.1, 1), "payload_mass": (0, 5),
                  "wind_force": (0, 20), "control_delay": (0, 1),
                  "flight_distance": (2, 8), "flight_altitude": (2, 3.2),
                  "payload_offset": (-0.35, 0.35)}
        for name, (low, high) in bounds.items():
            value = getattr(self, name)
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or not low <= value <= high):
                raise ValueError(f"{name} must be finite and in [{low}, {high}]")
        if self.timestep > self.duration:
            raise ValueError("timestep must not exceed duration")
        if (isinstance(self.rotor_index, bool) or not isinstance(self.rotor_index, int)
                or self.rotor_index not in range(4)):
            raise ValueError("rotor_index must be an integer in [0,3]")


PRESETS = {
    "drone_hover": {"fault": "healthy"},
    "drone_rotor_loss": {"fault": "rotor_loss"},
    "drone_voltage_sag": {"fault": "voltage_sag"},
    "drone_payload": {"fault": "payload"},
    "drone_wind": {"fault": "wind"},
    "drone_delay": {"fault": "delay"},
    "drone_demo": {"fault": "rotor_loss", "probe": "showcase", "duration": 20.,
                   "fault_at": 8., "rotor_effectiveness": .72},
    "drone_delivery_imbalance": {"fault": "payload", "probe": "delivery", "duration": 30.,
                                 "payload_mass": .36, "payload_offset": .30,
                                 "flight_altitude": 2.2},
}
DESCRIPTIONS = {
    "drone_hover": "Healthy hover followed by a small, controlled translation probe.",
    "drone_rotor_loss": "One rotor loses thrust after healthy flight; bounded control loses attitude.",
    "drone_voltage_sag": "Supply voltage drops; all four rotors lose available thrust.",
    "drone_payload": "An explicit co-moving payload pickup increases real mass and inertia.",
    "drone_wind": "A sustained crosswind force challenges nominal position prediction.",
    "drone_delay": "Motor command transport delay appears before the translation probe.",
    "drone_demo": "Fly a visible course, lose some rotor thrust, then return to hover with a tracking residual.",
    "drone_delivery_imbalance": "Carry an uneven parcel from A toward B; lose balance and crash.",
}


class Simulation:
    def __new__(cls, config: Config = Config()):
        if cls is Simulation and config.probe == "delivery":
            from .drone_delivery import DeliverySimulation
            return object.__new__(DeliverySimulation)
        return object.__new__(cls)

    def __init__(self, config: Config = Config()):
        self.config = config
        path = Path(__file__).resolve().parents[1] / "assets" / "platforms" / "drone.xml"
        self.model = mujoco.MjModel.from_xml_path(str(path))
        self.model.opt.timestep = config.timestep
        if config.probe == "showcase":
            self._configure_showcase_scene()
        self.data = mujoco.MjData(self.model)
        self.focus_body = self.model.body("drone").id
        self.ground = self.model.geom("ground").id
        self.events: list[dict] = []
        self.command_history: deque[tuple[float, np.ndarray]] = deque()
        self.reset_full()

    def _configure_showcase_scene(self):
        """Reveal nominal, noncolliding route marks and frame the larger course."""
        distance = self.config.flight_distance
        width = .4 * distance
        positions = {"course_outbound_pad": [distance, 0], "course_turn_pad": [distance, width],
                     "course_outbound_path": [distance / 2, 0],
                     "course_cross_path": [distance, width / 2],
                     "course_return_path": [distance / 2, width / 2]}
        for name, position in positions.items():
            geom = self.model.geom(name)
            geom.pos[:2] = position
            geom.rgba[3] = .8
        self.model.geom("course_outbound_path").size[0] = distance / 2
        self.model.geom("course_cross_path").size[1] = width / 2
        diagonal = self.model.geom("course_return_path")
        diagonal.size[0] = math.hypot(distance, width) / 2
        angle = math.atan2(width, distance)
        diagonal.quat[:] = [math.cos(angle / 2), 0, 0, math.sin(angle / 2)]
        for index in range(1, 4):
            marker = self.model.geom(f"course_distance_{index}")
            marker.pos[0] = distance * index / 4
            marker.rgba[3] = .9
        for name, position, target, fovy in (
            ("side", [distance / 2, -max(6.4, 1.6 * distance), 3.8], [distance / 2, width / 2, 1.7], 36),
            ("overview", [distance / 2 + 4, -6, 5.2], [distance / 2, width / 2, 1.5], 45),
        ):
            camera = self.model.camera(name)
            camera.pos[:] = position
            backward = np.array(position) - target
            backward /= np.linalg.norm(backward)
            right = np.cross([0, 0, 1], backward)
            right /= np.linalg.norm(right)
            rotation = np.column_stack((right, np.cross(backward, right), backward))
            mujoco.mju_mat2Quat(camera.quat, rotation.ravel())
            camera.fovy[0] = fovy
        self.camera_distance = max(7., 1.5 * distance)

    def reset_full(self):
        """Restore this experiment's original healthy start and event schedule."""
        self.elapsed = 0.0
        self.events = []
        self.event_applied = False
        self.effectiveness = np.ones(4)
        self.voltage = 1.0
        self.wind = np.zeros(3)
        self.delay = 0.0
        self.model.body_mass[self.focus_body] = NOMINAL_MASS
        self.model.body_inertia[self.focus_body] = NOMINAL_INERTIA
        self.model.body_ipos[self.focus_body] = 0
        self.model.geom("payload_visual").rgba[3] = 0
        mujoco.mj_setConst(self.model, self.data)
        self.reset_trial()

    def reset_trial(self):
        """Explicit reposition and rotor preparation; completed faults remain."""
        mujoco.mj_resetData(self.model, self.data)
        self.data.time = self.elapsed
        self.trial_time = 0.0
        self.target = np.array([0.0, 0.0, 2.0])
        self.last_command = np.full(4, NOMINAL_MASS * 9.81 / (4 * MAX_ROTOR_THRUST))
        self.motor_state = self.last_command.copy()
        self.command_history.clear()
        # A constant prehistory avoids inventing a command jump when resetting a
        # damaged vehicle with persistent transport delay.
        self.delayed_command = self.last_command.copy()
        self.command_mode = "nominal_probe"
        self.min_altitude = 2.0
        self.max_altitude = 2.0
        self.max_tracking_error = 0.0
        self.max_horizontal_excursion = 0.0
        self.max_tilt = 0.0
        self.ground_contact = False
        self.contact_steps = 0
        self.data.ctrl[:] = (MAX_ROTOR_THRUST * self.motor_state
                             * self.effectiveness * self.voltage ** 2)
        mujoco.mj_forward(self.model, self.data)

    @property
    def finished(self) -> bool:
        return self.trial_time + 1e-10 >= self.config.duration

    def _probe_target(self) -> np.ndarray:
        if self.config.probe == "showcase":
            for left, right in zip(SHOWCASE_ROUTE, SHOWCASE_ROUTE[1:]):
                if self.trial_time <= right[0]:
                    u = max(0., (self.trial_time - left[0]) / (right[0] - left[0]))
                    # A short acceleration/deceleration ramp preserves a long,
                    # visible transit without sharp target-velocity changes.
                    if u < .1:
                        progress = u * u / .18
                    elif u > .9:
                        progress = 1 - (1 - u) ** 2 / .18
                    else:
                        progress = (u - .05) / .9
                    target = np.array(left[1:]) + progress * (np.array(right[1:]) - left[1:])
                    target[:2] *= self.config.flight_distance
                    target[2] = 2 + target[2] * (self.config.flight_altitude - 2)
                    return target
            return np.array([0., 0., 2.])
        t = max(0.0, self.trial_time - 4.0)
        if self.config.probe == "hover":
            return np.array([0.0, 0.0, 2.0])
        # Smooth onset, followed by a lateral translation and braking/reversal.
        return np.array([0.65 * (1 - math.cos(0.8 * t)), 0.25 * (1 - math.cos(0.6 * t)), 2.0])

    def _nominal_control(self, target: np.ndarray) -> np.ndarray:
        rotation = self.data.xmat[self.focus_body].reshape(3, 3)
        velocity = self.data.qvel[:3]
        desired_acceleration = np.array([2.8, 2.8, 5.0]) * (target - self.data.qpos[:3])
        desired_acceleration -= np.array([2.6, 2.6, 3.8]) * velocity
        desired_acceleration[:2] = np.clip(desired_acceleration[:2], -4.0, 4.0)
        desired_acceleration[2] = np.clip(desired_acceleration[2], -5.0, 8.0)
        desired_force = NOMINAL_MASS * (desired_acceleration + np.array([0.0, 0.0, 9.81]))
        desired_z = desired_force / np.linalg.norm(desired_force)
        desired_y = np.cross(desired_z, np.array([1.0, 0.0, 0.0]))
        desired_y /= np.linalg.norm(desired_y)
        desired_x = np.cross(desired_y, desired_z)
        desired_rotation = np.column_stack((desired_x, desired_y, desired_z))
        error_matrix = (desired_rotation.T @ rotation - rotation.T @ desired_rotation) / 2
        attitude_error = np.array([error_matrix[2, 1], error_matrix[0, 2], error_matrix[1, 0]])
        torque = (-np.array([1.8, 1.8, 0.8]) * attitude_error
                  - np.array([0.28, 0.28, 0.18]) * self.data.qvel[3:6])
        torque = np.clip(torque, [-0.65, -0.65, -0.15], [0.65, 0.65, 0.15])
        collective = max(0.0, float(desired_force @ rotation[:, 2]))
        rotor_forces = INVERSE_MIXER @ np.concatenate(([collective], torque))
        return np.clip(rotor_forces / MAX_ROTOR_THRUST, 0, 1)

    def _apply_event(self):
        if (self.event_applied or self.config.fault == "healthy"
                or self.elapsed + 1e-10 < self.config.fault_at):
            return
        config = self.config
        self.event_applied = True
        event = {"time": self.elapsed, "trial_time": self.trial_time, "event": config.fault}
        if config.fault == "rotor_loss":
            self.effectiveness[config.rotor_index] = config.rotor_effectiveness
            event["rotor_index"] = config.rotor_index
        elif config.fault == "voltage_sag":
            self.voltage = config.voltage_ratio
        elif config.fault == "payload":
            # Instantaneously capture a package moving with the vehicle: velocity
            # continuity is the declared external intervention. Recompute combined
            # COM/inertia using a centered box 0.08 m below the original COM.
            payload = config.payload_mass
            mass = NOMINAL_MASS + payload
            offset = -0.08
            com = payload * offset / mass
            half_size = np.array([0.09, 0.075, 0.035])
            box_inertia = payload / 3 * np.array([half_size[1] ** 2 + half_size[2] ** 2,
                                                   half_size[0] ** 2 + half_size[2] ** 2,
                                                   half_size[0] ** 2 + half_size[1] ** 2])
            inertia = NOMINAL_INERTIA + box_inertia
            inertia[:2] += NOMINAL_MASS * com ** 2 + payload * (offset - com) ** 2
            self.model.body_mass[self.focus_body] = mass
            self.model.body_ipos[self.focus_body] = [0, 0, com]
            self.model.body_inertia[self.focus_body] = inertia
            self.model.geom("payload_visual").rgba[3] = float(payload > 0)
            # mj_setConst computes at qpos0 and overwrites its data argument.
            # Use scratch data so a mid-maneuver pickup cannot teleport the root.
            mujoco.mj_setConst(self.model, mujoco.MjData(self.model))
            mujoco.mj_forward(self.model, self.data)
            event["intervention"] = "co-moving external payload pickup; position and velocity retained"
            event["added_mass"] = payload
        elif config.fault == "wind":
            self.wind[:] = [config.wind_force, 0, 0]
        elif config.fault == "delay":
            self.delay = config.control_delay
        self.events.append(event)

    def _parse_control(self, control: dict | None) -> tuple[np.ndarray, np.ndarray, str]:
        target = self._probe_target()
        if control is None:
            return self._nominal_control(target), target, "nominal_probe"
        if not isinstance(control, dict) or set(control) not in ({"rotor_commands"}, {"target_position"}):
            raise ValueError("control must contain only rotor_commands or target_position")
        key = next(iter(control))
        try:
            values = np.asarray(control[key], dtype=float)
        except (ValueError, TypeError) as error:
            raise ValueError("control entries must be finite numeric values") from error
        if key == "rotor_commands":
            if values.shape != (4,) or not np.isfinite(values).all() or np.any((values < 0) | (values > 1)):
                raise ValueError("rotor_commands must contain four values in [0,1]")
            return values.copy(), target, "rotor_commands"
        if (values.shape != (3,) or not np.isfinite(values).all()
                or np.any(np.abs(values[:2]) > 10) or not 0.3 <= values[2] <= 5):
            raise ValueError("target_position must be [x,y,z], x/y in [-10,10], z in [0.3,5]")
        return self._nominal_control(values), values.copy(), "target_position"

    def step(self, control: dict | None = None):
        command, target, mode = self._parse_control(control)
        self._apply_event()
        dt = self.config.timestep
        self.target = target
        self.last_command = command
        self.command_mode = mode
        self.command_history.append((self.elapsed, command.copy()))
        while self.command_history[0][0] < self.elapsed - 1.0 - dt:
            self.command_history.popleft()
        # One second of prehistory supports any allowed delay, including onset
        # during an ongoing intervention. Delivered commands use zero-order hold.
        for timestamp, earlier_command in reversed(self.command_history):
            if timestamp <= self.elapsed - self.delay + 1e-10:
                self.delayed_command = earlier_command
                break
        self.motor_state += -math.expm1(-dt / NOMINAL_MOTOR_TAU) * (self.delayed_command - self.motor_state)
        self.data.ctrl[:] = MAX_ROTOR_THRUST * self.motor_state * self.effectiveness * self.voltage ** 2
        self.data.xfrc_applied[:] = 0
        # Linear parasitic drag is fixed across all configurations. Crosswind is
        # the prescribed external world-frame force, not a controller correction.
        self.data.xfrc_applied[self.focus_body, :3] = self.wind - 0.10 * self.data.qvel[:3]
        self.data.xfrc_applied[self.focus_body, 3:] = -0.006 * (
            self.data.xmat[self.focus_body].reshape(3, 3) @ self.data.qvel[3:6])
        mujoco.mj_step(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        self.elapsed += dt
        self.trial_time += dt
        if (not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all()
                or self.data.warning.number.any() or abs(self.data.time - self.elapsed) > dt / 2):
            raise RuntimeError(f"Invalid MuJoCo state or solver warning: {self.data.warning.number.tolist()}")
        altitude = float(self.data.qpos[2])
        self.min_altitude = min(self.min_altitude, altitude)
        self.max_altitude = max(self.max_altitude, altitude)
        self.max_tracking_error = max(self.max_tracking_error,
                                      float(np.linalg.norm(self.data.qpos[:3] - target)))
        self.max_horizontal_excursion = max(self.max_horizontal_excursion,
                                            float(np.linalg.norm(self.data.qpos[:2])))
        self.max_tilt = max(self.max_tilt, self._tilt())
        if any(self.ground in contact.geom for contact in self.data.contact):
            self.ground_contact = True
            self.contact_steps += 1

    def _tilt(self) -> float:
        cosine = float(np.clip(self.data.xmat[self.focus_body].reshape(3, 3)[2, 2], -1, 1))
        return math.degrees(math.acos(cosine))

    def observe(self) -> dict:
        phase = "hover" if self.trial_time < 4 or self.config.probe == "hover" else "translation_probe"
        if self.config.probe == "showcase":
            phase = showcase_phase(self.trial_time)
        return {"time": self.elapsed, "phase": phase, "phase_time": self.trial_time,
                "position": self.data.qpos[:3].tolist(), "quaternion": self.data.qpos[3:7].tolist(),
                "velocity": self.data.qvel[:3].tolist(),
                "angular_velocity": self.data.sensor("angular_velocity").data.tolist(),
                "specific_force": self.data.sensor("specific_force").data.tolist(),
                "command": {"mode": self.command_mode, "rotor_commands": self.last_command.tolist(),
                            "target_position": self.target.tolist()},
                "altitude": float(self.data.qpos[2]), "tilt_degrees": self._tilt(),
                "ground_contact": any(self.ground in contact.geom for contact in self.data.contact)}

    def diagnostics(self) -> dict:
        return {"time": self.elapsed, "rotor_effectiveness": self.effectiveness.tolist(),
                "voltage_ratio": self.voltage, "motor_state": self.motor_state.tolist(),
                "rotor_thrust": self.data.ctrl.tolist(), "wind_force": self.wind.tolist(),
                "control_delay": self.delay, "mass": float(self.model.body_mass[self.focus_body]),
                "inertia": self.model.body_inertia[self.focus_body].tolist(),
                "center_of_mass": self.model.body_ipos[self.focus_body].tolist(),
                "events": list(self.events), "warning_counts": self.data.warning.number.tolist()}

    def summary(self) -> dict:
        safe = (self.finished and not self.ground_contact and self.min_altitude >= 0.5 and self.max_altitude <= 3.5
                and self.max_tracking_error <= 0.8 and self.max_tilt <= 35)
        metrics = {"final_position": self.data.qpos[:3].tolist(), "final_altitude": float(self.data.qpos[2]),
                   "min_altitude": self.min_altitude, "max_altitude": self.max_altitude,
                   "max_tracking_error": self.max_tracking_error,
                   "max_horizontal_excursion": self.max_horizontal_excursion,
                   "max_tilt_degrees": self.max_tilt, "ground_contact": self.ground_contact,
                   "contact_steps": self.contact_steps, "trial_duration": self.trial_time}
        outcome = "incomplete_probe" if not self.finished else (
            "within_probe_envelope" if safe else "outside_probe_envelope")
        if self.ground_contact:
            outcome = "ground_contact"
        public = {"platform": "drone", "outcome": outcome, "safe": safe,
                  "safety_scope": "Only this synthetic probe: no contact, altitude 0.5–3.5 m, "
                                  "tracking error <=0.8 m, tilt <=35 deg.",
                  "metrics": metrics}
        return {"public": public, "config": asdict(self.config), "events": list(self.events),
                "warnings": self.data.warning.number.tolist(), "diagnostics": self.diagnostics()}
