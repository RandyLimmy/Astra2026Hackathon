"""Contact-triggered structural damage and repeatable car inspection probes.

The tire-pressure case is a synthetic radius/contact-compliance approximation,
not a pneumatic or finite-element tire model. The impact does not solve material
fracture: measured barrier contact activates a prescribed persistent parameter
change. No root forces or yaw impulses are used to manufacture the outcome.
"""

from dataclasses import asdict, dataclass
import math
from pathlib import Path

import mujoco
import numpy as np


WHEELS = ("FL", "FR", "RL", "RR")
FAULTS = ("healthy", "steering", "alignment", "suspension", "pressure")
PROBES = ("steering", "braking", "slalom", "bump")


@dataclass(frozen=True)
class Config:
    fault: str = "steering"
    duration: float = 12.0
    timestep: float = 0.002
    fault_at: float = 0.5
    probe: str = "steering"
    impact_speed: float = 6.5
    probe_speed: float = 5.5
    barrier_x: float = 9.0
    approach_y: float = 0.0
    impact_threshold: float = 500.0
    recovery_wait: float = 0.6
    steering_gain: float = 0.35
    steering_bias: float = 0.12
    toe_angle: float = 0.28
    suspension_scale: float = 0.20
    pressure_radius_scale: float = 0.80
    pressure_friction: float = 0.40
    pressure_contact_time: float = 0.045

    def __post_init__(self):
        bounds = {
            "duration": (0.1, 120), "timestep": (0.0005, 0.004),
            "fault_at": (0, 120), "impact_speed": (0, 15),
            "probe_speed": (0, 12), "barrier_x": (5, 100),
            "approach_y": (-30, 30), "impact_threshold": (1, 1e7),
            "recovery_wait": (0.1, 5), "steering_gain": (0, 1.5),
            "steering_bias": (-0.3, 0.3), "toe_angle": (-0.5, 0.5),
            "suspension_scale": (0.05, 1), "pressure_radius_scale": (0.6, 1),
            "pressure_friction": (0.05, 1.1), "pressure_contact_time": (0.008, 0.10),
        }
        for field, (low, high) in bounds.items():
            value = getattr(self, field)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{field} must be a finite number")
            if not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f"{field} must be finite and in [{low}, {high}]")
        if self.fault not in FAULTS:
            raise ValueError(f"fault must be one of {FAULTS}")
        if self.probe not in PROBES:
            raise ValueError(f"probe must be one of {PROBES}")


PRESETS = {
    "car_postcrash_healthy": {"fault": "healthy", "probe": "steering"},
    "car_steering_damage": {"fault": "steering", "probe": "steering"},
    "car_wheel_misalignment": {"fault": "alignment", "probe": "slalom"},
    "car_suspension_damage": {"fault": "suspension", "probe": "bump"},
    "car_tire_pressure": {"fault": "pressure", "probe": "braking"},
    "car_demo": {"fault": "steering", "probe": "steering", "duration": 12.0,
                 "fault_at": 1.0, "impact_speed": 4.5, "barrier_x": 14.0,
                 "recovery_wait": 0.8, "probe_speed": 4.0,
                 "steering_gain": 0.65, "steering_bias": 0.06},
}
DESCRIPTIONS = {
    "car_postcrash_healthy": "Barrier impact, recovery, and healthy controlled steering reference.",
    "car_steering_damage": "Barrier impact leaves reduced steering rack response and a steering bias.",
    "car_wheel_misalignment": "A bent left front wheel mount changes toe during a slalom probe.",
    "car_suspension_damage": "Impact weakens the left front spring before a one-wheel bump probe.",
    "car_tire_pressure": "Impact changes left front tire radius/contact compliance: synthetic pressure proxy.",
    "car_demo": "Visible approach, measured barrier impact, then a moderately weakened steering inspection.",
}


class Simulation:
    def __init__(self, config: Config):
        self.config = config
        path = Path(__file__).parents[1] / "assets/platforms/car_damage.xml"
        self.model = mujoco.MjModel.from_xml_path(str(path))
        self.model.opt.timestep = config.timestep
        self.data = mujoco.MjData(self.model)
        self.focus_body = self.model.body("chassis").id
        self._barrier = self.model.geom("impact_barrier").id
        self._bump = self.model.geom("probe_bump").id
        self.model.geom_pos[self._barrier, 0] = config.barrier_x
        self.model.geom("barrier_stripe").pos[0] = config.barrier_x - 0.38
        if config.probe != "bump":
            self.model.geom_contype[self._bump] = 0
            self.model.geom_conaffinity[self._bump] = 0
            self.model.geom_rgba[self._bump, 3] = 0
        self._spin = np.array([int(self.model.joint(f"spin_{w}").dofadr[0]) for w in WHEELS])
        self._suspension = np.array([self.model.joint(f"suspension_{w}").id for w in WHEELS])
        self._suspension_q = self.model.jnt_qposadr[self._suspension]
        self._steer_q = np.array([int(self.model.joint(f"steer_{w}").qposadr[0]) for w in WHEELS[:2]])
        self._tire = np.array([self.model.geom(f"tire_{w}").id for w in WHEELS])
        self._toe = self.model.body("toe_FL").id
        self._vehicle_geoms = {
            i for i in range(self.model.ngeom)
            if int(self.model.body_rootid[self.model.geom_bodyid[i]]) == self.focus_body
        }
        self._nominal = {
            key: getattr(self.model, key).copy()
            for key in ("jnt_stiffness", "dof_damping", "geom_size", "geom_friction",
                        "geom_solref", "body_quat")
        }
        self.reset_full()

    @property
    def finished(self) -> bool:
        return self.elapsed >= self._end_time - self.config.timestep / 2

    def _initialize_state(self, y: float, speed: float):
        """Declared fixture placement; physical dynamics own motion thereafter."""
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[1] = y
        self.data.qpos[self._suspension_q] = 0.067
        # Settle on the fixture before releasing. This preparation is not an
        # observation trial; the experiment clock remains monotonic unchanged.
        for _ in range(round(0.5 / self.config.timestep)):
            self._apply_controls({"steering": 0.0, "throttle": 0.0, "brake": 1.0})
            mujoco.mj_step(self.model, self.data)
        self.data.qvel[:] = 0
        self.data.qvel[0] = speed
        self.data.qvel[self._spin] = speed / self.model.geom_size[self._tire, 0]
        self.data.time = self.elapsed
        self.data.ctrl[:] = 0
        self.model.dof_frictionloss[self._spin] = 0
        mujoco.mj_forward(self.model, self.data)
        self._last_command = {"steering": 0.0, "throttle": 0.0, "brake": 0.0}

    def reset_full(self):
        """Restore the original Config and replay its fault-free approach."""
        for key, value in self._nominal.items():
            getattr(self.model, key)[:] = value
        mujoco.mj_setConst(self.model, self.data)
        self.elapsed = 0.0
        self._end_time = self.config.duration
        self._steps = 0
        self._damaged = False
        self._rack_gain, self._rack_bias = 1.0, 0.0
        self._impact_time: float | None = None
        self._probe_start: float | None = None
        self._impact_force = 0.0
        self._events: list[dict] = []
        self._warning_counts = np.zeros(len(self.data.warning), dtype=int)
        self._metrics = self._empty_metrics()
        self._initialize_state(self.config.approach_y, self.config.impact_speed)

    @staticmethod
    def _empty_metrics() -> dict:
        return {"probe_elapsed": 0.0, "max_lateral_displacement": 0.0,
                "max_roll_rad": 0.0, "max_suspension_travel": 0.0,
                "max_vertical_acceleration": 0.0, "braking_distance": None,
                "distance_traveled": 0.0, "final_speed": 0.0}

    def reset_trial(self):
        """Explicit recovery to the open lane; retain all completed damage."""
        self._end_time = self.elapsed + self.config.duration
        self._reposition_for_probe()

    def _reposition_for_probe(self):
        old_pose = self.data.qpos[:3].copy().tolist()
        self._warning_counts += self.data.warning.number
        self._initialize_state(-12.0, self.config.probe_speed)
        self._probe_start = self.elapsed
        self._metrics = self._empty_metrics()
        self._brake_position: np.ndarray | None = None
        self._previous_xy = self.data.qpos[:2].copy()
        self._events.append({"event": "recovery_reposition", "time": self.elapsed,
                             "from_position": old_pose, "to_position": self.data.qpos[:3].tolist(),
                             "retained_damage": self._damaged,
                             "fixture_settle_seconds": 0.5})

    def _apply_damage(self):
        config = self.config
        if config.fault == "healthy":
            return
        if config.fault == "steering":
            self._rack_gain, self._rack_bias = config.steering_gain, config.steering_bias
        elif config.fault == "alignment":
            self.model.body_quat[self._toe] = [math.cos(config.toe_angle / 2), 0, 0,
                                              math.sin(config.toe_angle / 2)]
        elif config.fault == "suspension":
            self.model.jnt_stiffness[self._suspension[0]] *= config.suspension_scale
            dof = self.model.jnt_dofadr[self._suspension[0]]
            self.model.dof_damping[dof] *= math.sqrt(config.suspension_scale)
        elif config.fault == "pressure":
            tire = self._tire[0]
            self.model.geom_size[tire, [0, 2]] *= config.pressure_radius_scale
            self.model.geom_friction[tire, 0] = config.pressure_friction
            self.model.geom_solref[tire, 0] = config.pressure_contact_time
        self._damaged = True
        self._events.append({"event": "structural_damage", "time": self.elapsed,
                             "fault": config.fault, "gate": "measured_barrier_contact",
                             "contact_force": self._impact_force})
        # mj_setConst uses qpos0 as scratch state. Preserve the complete dynamic
        # state so damage cannot accidentally teleport or re-accelerate the car.
        signature = mujoco.mjtState.mjSTATE_INTEGRATION
        state = np.empty(mujoco.mj_stateSize(self.model, signature))
        mujoco.mj_getState(self.model, self.data, state, signature)
        mujoco.mj_setConst(self.model, self.data)
        mujoco.mj_setState(self.model, self.data, state, signature)
        mujoco.mj_forward(self.model, self.data)

    def _nominal_command(self) -> dict:
        # This probe reads the experiment clock and public speed only. It never
        # reads fault state or true parameters to compensate for damage.
        if self._probe_start is None:
            return {"steering": 0.0, "throttle": 0.0,
                    "brake": 1.0 if self._impact_time is not None else 0.0}
        t = self.elapsed - self._probe_start
        steering = 0.0
        if self.config.probe == "steering" and 1 <= t < 6:
            steering = 0.10 if t < 2.25 or t >= 4.75 else -0.10
        elif self.config.probe == "slalom" and 1 <= t < 6:
            steering = 0.12 * math.sin(2 * math.pi * (t - 1) / 2.5)
        brake_at = 2.0 if self.config.probe == "braking" else 6.5
        speed = float(np.linalg.norm(self.data.qvel[:2]))
        throttle = float(np.clip((self.config.probe_speed - speed) * 0.2, 0, 0.3))
        return {"steering": steering, "throttle": throttle if t < brake_at else 0.0,
                "brake": 0.0 if t < brake_at else 0.75}

    @staticmethod
    def _validate_control(control: dict) -> dict:
        if not isinstance(control, dict) or set(control) - {"steering", "throttle", "brake"}:
            raise ValueError("control accepts steering (radians), throttle, and brake")
        result = {"steering": 0.0, "throttle": 0.0, "brake": 0.0}
        for key, value in control.items():
            low, high = (-0.5, 0.5) if key == "steering" else (0, 1)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{key} must be a finite number")
            if not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f"{key} must be in [{low}, {high}]")
            result[key] = float(value)
        return result

    def _apply_controls(self, command: dict):
        # The fault belongs to the steering hardware transfer function. Both the
        # external command and measured steering angles remain observable.
        target = self._rack_gain * command["steering"] + self._rack_bias
        self.data.ctrl[:2] = np.clip(target, -0.6, 0.6)
        self.data.ctrl[2:] = command["throttle"] * 700
        # MuJoCo joint friction opposes spin, including at zero speed, so brakes
        # never become reverse-driving torques as a wheel slows down.
        self.model.dof_frictionloss[self._spin] = command["brake"] * 850

    def step(self, control: dict | None = None):
        command = self._nominal_command() if control is None else self._validate_control(control)
        if self.finished:
            return
        self._last_command = command
        self._apply_controls(command)
        mujoco.mj_step(self.model, self.data)
        self._steps += 1
        self.elapsed = self._steps * self.config.timestep
        self.data.time = self.elapsed
        mujoco.mj_forward(self.model, self.data)
        if self._impact_time is None and self._probe_start is None:
            for i, contact in enumerate(self.data.contact):
                pair = {int(contact.geom1), int(contact.geom2)}
                if self._barrier not in pair or not pair.intersection(self._vehicle_geoms):
                    continue
                force = np.zeros(6)
                mujoco.mj_contactForce(self.model, self.data, i, force)
                normal_force = max(0.0, float(force[0]))
                self._impact_force = max(self._impact_force, normal_force)
                if self.elapsed >= self.config.fault_at and normal_force >= self.config.impact_threshold:
                    self._impact_time = self.elapsed
                    self._events.append({"event": "barrier_impact", "time": self.elapsed,
                                         "normal_force": normal_force,
                                         "position": self.data.qpos[:3].tolist()})
                    self._apply_damage()
                    break
        if (self._impact_time is not None and self._probe_start is None
                and self.elapsed >= self._impact_time + self.config.recovery_wait):
            self._reposition_for_probe()
            return
        if self._probe_start is not None:
            self._update_metrics(command)

    def _angles(self) -> tuple[float, float]:
        matrix = self.data.xmat[self.focus_body].reshape(3, 3)
        return math.atan2(matrix[1, 0], matrix[0, 0]), math.atan2(matrix[2, 1], matrix[2, 2])

    def _update_metrics(self, command: dict):
        m = self._metrics
        assert self._probe_start is not None
        m["probe_elapsed"] = self.elapsed - self._probe_start
        m["max_lateral_displacement"] = max(m["max_lateral_displacement"], abs(float(self.data.qpos[1]) + 12))
        m["max_roll_rad"] = max(m["max_roll_rad"], abs(self._angles()[1]))
        m["max_suspension_travel"] = max(m["max_suspension_travel"],
                                         float(np.max(np.abs(self.data.qpos[self._suspension_q]))))
        m["max_vertical_acceleration"] = max(m["max_vertical_acceleration"],
                                            abs(float(self.data.sensor("acceleration").data[2]) - 9.81))
        xy = self.data.qpos[:2]
        distance = float(np.linalg.norm(xy - self._previous_xy))
        m["distance_traveled"] += distance
        m["final_speed"] = float(np.linalg.norm(self.data.qvel[:2]))
        if command["brake"] > 0 and self._brake_position is None:
            self._brake_position = xy.copy()
            m["braking_distance"] = 0.0
        elif self._brake_position is not None and m["final_speed"] > 0.05:
            m["braking_distance"] += distance
        self._previous_xy = xy.copy()

    def observe(self) -> dict:
        yaw, roll = self._angles()
        phase = "approach"
        if self._probe_start is not None:
            phase = "controlled_probe"
        elif self._impact_time is not None:
            phase = "impact_recovery"
        return {"time": self.elapsed, "phase": phase,
                "position": self.data.qpos[:3].tolist(), "orientation": self.data.qpos[3:7].tolist(),
                "velocity": self.data.qvel[:3].tolist(), "yaw": yaw, "roll": roll,
                "angular_velocity": self.data.sensor("angular_velocity").data.tolist(),
                "acceleration": self.data.sensor("acceleration").data.tolist(),
                "command": dict(self._last_command),
                "steering_angles": self.data.qpos[self._steer_q].tolist(),
                "wheel_angular_velocity": self.data.qvel[self._spin].tolist(),
                "suspension_travel": self.data.qpos[self._suspension_q].tolist(),
                "wheel_positions": [self.data.xpos[self.model.body(f"wheel_{w}").id].tolist()
                                    for w in WHEELS],
                "wheel_axle_directions": [
                    self.data.xmat[self.model.body(f"wheel_{w}").id].reshape(3, 3)[:, 1].tolist()
                    for w in WHEELS]}

    def diagnostics(self) -> dict:
        return {"fault": self.config.fault, "damage_active": self._damaged,
                "impact_time": self._impact_time, "peak_impact_force": self._impact_force,
                "probe_started_at": self._probe_start,
                "rack_gain": self._rack_gain, "rack_bias": self._rack_bias,
                "toe_quaternion": self.model.body_quat[self._toe].tolist(),
                "spring_stiffness": self.model.jnt_stiffness[self._suspension].tolist(),
                "tire_radius": self.model.geom_size[self._tire, 0].tolist(),
                "tire_friction": self.model.geom_friction[self._tire, 0].tolist(),
                "tire_contact_time": self.model.geom_solref[self._tire, 0].tolist(),
                "events": list(self._events)}

    def summary(self) -> dict:
        metrics = dict(self._metrics)
        completed = self._probe_start is not None and self.finished and metrics["probe_elapsed"] >= 4
        safe = (completed and metrics["max_lateral_displacement"] < 4.5
                and metrics["max_roll_rad"] < 0.35 and metrics["final_speed"] < 0.5
                and metrics["max_suspension_travel"] < 0.20)
        outcome = "probe_within_envelope" if safe else "probe_outside_envelope"
        if not completed:
            outcome = "incomplete_probe" if self._probe_start is not None else "no_armed_impact"
        return {"public": {"platform": "car_damage", "outcome": outcome, "safe": bool(safe),
                           "metrics": metrics,
                           "safety_scope": "Only this synthetic low-speed maneuver; not roadworthiness."},
                "config": asdict(self.config), "events": list(self._events),
                "diagnostics": self.diagnostics(),
                "warnings": (self._warning_counts + self.data.warning.number).tolist()}
