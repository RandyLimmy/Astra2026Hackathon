"""A free-base, motor-driven dog for controlled model-mismatch experiments.

The nominal crawl uses joint torques and physical foot contacts only. Its balance
controller observes base pose/velocity and joint encoders; it never reads the
injected damage. Faults are synthetic component changes, not injury models.
"""
from dataclasses import asdict, dataclass
import math
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

LEGS = ("FL", "FR", "RL", "RR")
JOINTS = tuple(f"{leg}_{joint}" for leg in LEGS for joint in ("abduction", "hip_pitch", "knee"))
FAULTS = ("healthy", "none", "joint_weakness", "foot_slip", "leg_damage", "payload_shift")
PROBES = ("walk", "stand", "turn", "conservative", "passive")
HIP_OFFSETS = np.array(((0.255, 0.155, 0), (0.255, -0.155, 0),
                        (-0.255, 0.155, 0), (-0.255, -0.155, 0)))


@dataclass(frozen=True)
class Config:
    fault: str = "healthy"
    duration: float = 10.0
    timestep: float = 0.002
    fault_at: float = 5.0
    probe: str = "walk"
    affected_leg: str = "FL"
    strength: float = 0.08
    foot_friction: float = 0.025
    damage_stiffness: float = 90.0
    damage_rest_angle: float = -2.35
    payload_offset: float = 0.29
    speed: float = 0.10
    gait_period: float = 3.2

    def __post_init__(self) -> None:
        if self.fault not in FAULTS or self.probe not in PROBES or self.affected_leg not in LEGS:
            raise ValueError("Unknown quadruped fault, probe, or leg")
        limits = {"duration": (0.05, 120), "timestep": (0.0005, 0.004), "fault_at": (0, 120),
                  "strength": (0, 1), "foot_friction": (0.001, 1.5), "damage_stiffness": (0, 160),
                  "damage_rest_angle": (-2.6, -0.2), "payload_offset": (-0.3, 0.3),
                  "speed": (-0.15, 0.2), "gait_period": (2, 6)}
        for name, (low, high) in limits.items():
            value = getattr(self, name)
            if not isinstance(value, (float, int)) or isinstance(value, bool) or not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
            if not low <= value <= high:
                raise ValueError(f"{name} must be between {low} and {high}")


PRESETS: dict[str, dict[str, Any]] = {
    "quadruped_walk": {},
    "quadruped_joint_weakness": {"fault": "joint_weakness"},
    "quadruped_foot_slip": {"fault": "foot_slip"},
    "quadruped_leg_damage": {"fault": "leg_damage"},
    "quadruped_payload_shift": {"fault": "payload_shift"},
    "quadruped_demo": {"fault": "joint_weakness", "probe": "walk", "duration": 18.0,
                       "fault_at": 9.0, "speed": 0.12, "strength": 0.08},
}
DESCRIPTIONS = {
    "quadruped_walk": "Healthy articulated dog performs a controlled forward crawl.",
    "quadruped_joint_weakness": "One knee actuator loses torque after normal walking.",
    "quadruped_foot_slip": "One foot loses contact friction during the same walking probe.",
    "quadruped_leg_damage": "One knee gains a stiff, bent rest configuration after damage.",
    "quadruped_payload_shift": "An onboard payload slides sideways, shifting the center of mass.",
    "quadruped_demo": "Walk a metre, lose one knee's support, stumble and fall, then simulate the aftermath.",
}


class Simulation:
    def __init__(self, config: Config):
        self.config = config
        self.model = mujoco.MjModel.from_xml_path(str(Path(__file__).parents[1] / "assets/platforms/quadruped.xml"))
        self.model.opt.timestep = config.timestep
        self.data = mujoco.MjData(self.model)
        self.focus_body = int(self.model.body("torso").id)
        self._joints = np.array([self.model.joint(name).id for name in JOINTS])
        self._qa = self.model.jnt_qposadr[self._joints].copy()
        self._da = self.model.jnt_dofadr[self._joints].copy()
        self._toes = np.array([self.model.site(f"{leg}_toe").id for leg in LEGS])
        self._feet = np.array([self.model.geom(f"{leg}_foot").id for leg in LEGS])
        self._payload_joint = int(self.model.joint("payload_slide").id)
        self._payload_qa = int(self.model.jnt_qposadr[self._payload_joint])
        self._nominal_mass = float(self.model.body_mass.sum())
        self._baseline = {name: getattr(self.model, name).copy() for name in (
            "actuator_gainprm", "geom_friction", "jnt_stiffness", "qpos_spring")}
        self._jac = np.zeros((3, self.model.nv))
        self._jac_rot = np.zeros_like(self._jac)
        self.events: list[dict[str, Any]] = []
        self._fault_active = False
        self._offset = 0.0
        self._trial = 0
        self._initialize()

    @property
    def elapsed(self) -> float:
        return self._offset + float(self.data.time)

    @property
    def finished(self) -> bool:
        return self.data.time >= self.config.duration - 0.5 * self.config.timestep

    def _initialize(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:3] = (0, 0, 0.405)
        self.data.qpos[3:7] = (1, 0, 0, 0)
        self.data.qpos[self._payload_qa] = self.model.qpos_spring[self._payload_qa]
        for i in range(4):
            self.data.qpos[self._qa[3 * i:3 * i + 3]] = self._ik(np.array((0, 0.045 if i % 2 == 0 else -0.045, -0.371)))
        mujoco.mj_forward(self.model, self.data)
        self._anchors = self.data.site_xpos[self._toes].copy()
        self._anchors[:, 2] = 0.034
        self._swing_start = self._anchors.copy()
        self._swing_end = self._anchors.copy()
        self._active_slot = -1
        self._target_xy = np.zeros(2)
        self._heading = 0.0
        self._last_target = np.array((0, 0, 0.405))
        self._last_command: dict[str, Any] = {}
        self._max_tilt = 0.0
        self._min_height = 0.405
        self._fallen = False
        self._fall_time: float | None = None
        self._max_lateral = 0.0
        self._tracking_square = 0.0
        self._samples = 0

    @staticmethod
    def _ik(foot: np.ndarray) -> np.ndarray:
        x, y, z = foot
        abd = math.atan2(y, -z)
        length = min(0.438, max(0.13, math.sqrt(x * x + y * y + z * z)))
        knee = -2 * math.acos(length / 0.44)
        hip = -math.atan2(x, math.hypot(y, z)) - knee / 2
        return np.array((abd, hip, knee))

    def _controls(self, control: dict | None) -> dict[str, Any]:
        moving = self.config.probe in ("walk", "turn", "conservative")
        command: dict[str, Any] = {
            "forward_speed": self.config.speed * (0.5 if self.config.probe == "conservative" else 1) if moving else 0.0,
            "yaw_rate": 0.10 if self.config.probe == "turn" else 0.0,
            "motors_enabled": self.config.probe != "passive",
        }
        if control is not None:
            if not isinstance(control, dict) or set(control) - {"forward_speed", "yaw_rate", "motors_enabled", "joint_targets"}:
                raise ValueError("Unknown quadruped control; use forward_speed, yaw_rate, motors_enabled, joint_targets")
            command.update(control)
        for name, bound in (("forward_speed", 0.25), ("yaw_rate", 0.4)):
            value = command[name]
            if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or abs(value) > bound:
                raise ValueError(f"{name} must be finite with magnitude at most {bound}")
            command[name] = float(value)
        if not isinstance(command["motors_enabled"], bool):
            raise ValueError("motors_enabled must be boolean")
        if "joint_targets" in command:
            try:
                targets = np.asarray(command["joint_targets"], dtype=float)
            except (TypeError, ValueError) as exc:
                raise ValueError("joint_targets must contain twelve finite joint angles") from exc
            if targets.shape != (12,) or not np.isfinite(targets).all():
                raise ValueError("joint_targets must contain twelve finite joint angles")
            limits = self.model.jnt_range[self._joints]
            if np.any(targets < limits[:, 0]) or np.any(targets > limits[:, 1]):
                raise ValueError("joint_targets exceed physical joint limits")
            command["joint_targets"] = targets.tolist()
        return command

    def _inject_fault(self) -> None:
        if self._fault_active or self.config.fault in ("none", "healthy") or self.elapsed + 1e-10 < self.config.fault_at:
            return
        leg = LEGS.index(self.config.affected_leg)
        knee = int(self._joints[3 * leg + 2])
        if self.config.fault == "joint_weakness":
            self.model.actuator_gainprm[3 * leg + 2, 0] = self.config.strength
        elif self.config.fault == "foot_slip":
            self.model.geom_friction[self._feet[leg], 0] = self.config.foot_friction
        elif self.config.fault == "leg_damage":
            self.model.jnt_stiffness[knee] = self.config.damage_stiffness
            self.model.qpos_spring[self.model.jnt_qposadr[knee]] = self.config.damage_rest_angle
        elif self.config.fault == "payload_shift":
            # A physical internal slide moves the mass; the root pose is never set.
            self.model.qpos_spring[self._payload_qa] = self.config.payload_offset
        self._fault_active = True
        self.events.append({"event": self.config.fault, "time": self.elapsed, "leg": self.config.affected_leg})

    @staticmethod
    def _skew(vector: np.ndarray) -> np.ndarray:
        x, y, z = vector
        return np.array(((0, -z, y), (z, 0, -x), (-y, x, 0)))

    def _gait(self, command: dict[str, Any]) -> np.ndarray:
        time = float(self.data.time)
        rotation = self.data.xmat[self.focus_body].reshape(3, 3)
        pos = self.data.xpos[self.focus_body]
        velocity = self.data.qvel[:3]
        speed, yaw_rate = command["forward_speed"], command["yaw_rate"]
        active = time > 1.0 and (abs(speed) > 1e-6 or abs(yaw_rate) > 1e-6)
        self._heading += yaw_rate * self.config.timestep if active else 0
        desired_rotation = np.array(((math.cos(self._heading), -math.sin(self._heading), 0),
                                     (math.sin(self._heading), math.cos(self._heading), 0), (0, 0, 1)))
        desired_velocity = desired_rotation @ np.array((speed if active else 0, 0, 0))
        self._target_xy += desired_velocity[:2] * self.config.timestep
        desired_pos = np.array((*self._target_xy, 0.405))
        targets = self._anchors.copy()
        target_vel = np.zeros((4, 3))
        stance = np.ones(4, dtype=bool)
        if active:
            slot_length = self.config.gait_period / 4
            slot = int((time - 1) / slot_length)
            phase = ((time - 1) / slot_length) % 1
            order = (0, 3, 1, 2)
            leg = order[slot % 4]
            previous = order[(slot - 1) % 4]
            signs = np.array(((1, 1), (1, -1), (-1, 1), (-1, -1)))
            shift = -signs[leg] * np.array((0.06, 0.055))
            prev_shift = -signs[previous] * np.array((0.06, 0.055))
            blend = min(1.0, phase / 0.23)
            blend = blend * blend * (3 - 2 * blend)
            desired_pos[:2] += (desired_rotation[:2, :2] @ (prev_shift * (1 - blend) + shift * blend))
            if self._active_slot != slot:
                self._active_slot = slot
                self._swing_start[leg] = self._anchors[leg]
                local = HIP_OFFSETS[leg].copy()
                local[1] += 0.045 if leg % 2 == 0 else -0.045
                local[0] += speed * self.config.gait_period * 0.42
                self._swing_end[leg] = desired_rotation @ local + np.array((*self._target_xy, 0))
                self._swing_end[leg, 2] = 0.034
            if 0.25 < phase < 0.78:
                stance[leg] = False
                progress = (phase - 0.25) / 0.53
                blend = progress * progress * (3 - 2 * progress)
                rate = 6 * progress * (1 - progress) / (slot_length * 0.53)
                targets[leg] = self._swing_start[leg] * (1 - blend) + self._swing_end[leg] * blend
                targets[leg, 2] += 0.075 * math.sin(math.pi * progress)
                target_vel[leg] = (self._swing_end[leg] - self._swing_start[leg]) * rate
                target_vel[leg, 2] += 0.075 * math.pi * math.cos(math.pi * progress) / (slot_length * 0.53)
            elif phase >= 0.78:
                self._anchors[leg] = self._swing_end[leg]
                targets[leg] = self._anchors[leg]
        target_velocity = np.clip((desired_pos - self._last_target) / self.config.timestep, -0.6, 0.6)
        self._last_target = desired_pos.copy()
        # Allocate the desired observed-base wrench to feet. Only motor torques
        # are applied; root generalized forces remain zero for the entire run.
        force = self._nominal_mass * (np.array((0, 0, 9.81))
                + np.array((25, 30, 100)) * (desired_pos - pos)
                + np.array((8, 9, 16)) * (target_velocity - velocity))
        orientation_error = 0.5 * sum((np.cross(rotation[:, i], desired_rotation[:, i]) for i in range(3)), np.zeros(3))
        angular_world = rotation @ self.data.qvel[3:6]
        moment = np.array((45, 55, 20)) * orientation_error - np.array((5, 6, 3)) * (angular_world - np.array((0, 0, yaw_rate if active else 0)))
        indices = np.flatnonzero(stance)
        feet = self.data.site_xpos[self._toes]
        allocation = np.concatenate([np.vstack((np.eye(3), self._skew(feet[i] - pos))) for i in indices], axis=1)
        wrench = np.concatenate((force, moment))
        # Damped least squares makes diagonal/near-collinear contacts well posed.
        ground = allocation.T @ np.linalg.solve(allocation @ allocation.T + np.eye(6) * 0.002, wrench)
        support = np.zeros((4, 3))
        for k, stance_index in enumerate(indices):
            f = ground[3 * k:3 * k + 3]
            f[2] = np.clip(f[2], 0, 100)
            f[:2] = np.clip(f[:2], -0.7 * f[2], 0.7 * f[2])
            support[stance_index] = f
        torque = np.zeros(12)
        joint_targets = []
        for i, site in enumerate(self._toes):
            mujoco.mj_jacSite(self.model, self.data, self._jac, self._jac_rot, int(site))
            jac = self._jac[:, self._da[3 * i:3 * i + 3]]
            foot_velocity = self._jac @ self.data.qvel
            foot_error = targets[i] - feet[i]
            impedance = (350 if stance[i] else 500) * foot_error + (9 if stance[i] else 12) * (target_vel[i] - foot_velocity)
            torque[3 * i:3 * i + 3] = jac.T @ (impedance - support[i])
            joint_targets.extend(self._ik(rotation.T @ (targets[i] - pos) - HIP_OFFSETS[i]).tolist())
        self._last_command["joint_targets"] = joint_targets
        return torque

    def step(self, control: dict | None = None) -> None:
        command = self._controls(control)
        if self.finished:
            return
        self._inject_fault()
        self._last_command = command.copy()
        if not command["motors_enabled"]:
            torque = np.zeros(12)
        elif "joint_targets" in command:
            torque = 60 * (np.array(command["joint_targets"]) - self.data.qpos[self._qa]) - 2 * self.data.qvel[self._da]
        else:
            torque = self._gait(command)
        self.data.ctrl[:] = np.clip(torque, -35, 35)
        self._last_command["joint_torque_commands"] = dict(zip(JOINTS, self.data.ctrl.tolist()))
        mujoco.mj_step(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        rotation = self.data.xmat[self.focus_body].reshape(3, 3)
        tilt = math.acos(float(np.clip(rotation[2, 2], -1, 1)))
        height = float(self.data.xpos[self.focus_body, 2])
        self._max_tilt = max(self._max_tilt, tilt)
        self._min_height = min(self._min_height, height)
        self._max_lateral = max(self._max_lateral, abs(float(self.data.xpos[self.focus_body, 1])))
        self._tracking_square += float(np.linalg.norm(self.data.qpos[self._qa] - np.array(self._last_command.get("joint_targets", self.data.qpos[self._qa]))) ** 2)
        self._samples += 1
        if (height < 0.20 or tilt > math.radians(65)) and not self._fallen:
            self._fallen = True
            self._fall_time = self.elapsed

    def observe(self) -> dict[str, Any]:
        contacts = {leg: False for leg in LEGS}
        for contact in self.data.contact:
            for i, foot in enumerate(self._feet):
                if foot in contact.geom:
                    contacts[LEGS[i]] = True
        return {
            "time": self.elapsed, "trial_time": float(self.data.time), "trial": self._trial,
            "phase": "settling" if self.data.time < 1 else "probe",
            "position": self.data.xpos[self.focus_body].tolist(),
            "quaternion": self.data.xquat[self.focus_body].tolist(),
            "velocity": self.data.qvel[:3].tolist(),
            "pose": {"position": self.data.xpos[self.focus_body].tolist(), "quaternion": self.data.xquat[self.focus_body].tolist()},
            "linear_velocity": self.data.qvel[:3].tolist(),
            "imu": {"acceleration": self.data.sensor("accelerometer").data.tolist(), "angular_velocity": self.data.sensor("gyroscope").data.tolist()},
            "joint_positions": dict(zip(JOINTS, self.data.qpos[self._qa].tolist())),
            "joint_velocities": dict(zip(JOINTS, self.data.qvel[self._da].tolist())),
            "foot_contacts": contacts, "command": self._last_command.copy(),
        }

    def diagnostics(self) -> dict[str, Any]:
        return {"fault_active": self._fault_active, "fault": self.config.fault,
                "actuator_strength": dict(zip(JOINTS, self.model.actuator_gainprm[:, 0].tolist())),
                "foot_friction": dict(zip(LEGS, self.model.geom_friction[self._feet, 0].tolist())),
                "joint_stiffness": dict(zip(JOINTS, self.model.jnt_stiffness[self._joints].tolist())),
                "joint_rest_angles": dict(zip(JOINTS, self.model.qpos_spring[self._qa].tolist())),
                "payload_offset": float(self.data.qpos[self._payload_qa]),
                "payload_target": float(self.model.qpos_spring[self._payload_qa]),
                "events": [event.copy() for event in self.events]}

    def summary(self) -> dict[str, Any]:
        position = self.data.xpos[self.focus_body]
        warnings = {str(mujoco.mjtWarning(i).name): int(w.number) for i, w in enumerate(self.data.warning) if w.number}
        return {"public": {"platform": "quadruped", "outcome": "fell" if self._fallen else "upright",
                           "safe": bool(self.finished and not self._fallen and not warnings and self._max_tilt < math.radians(40)),
                           "metrics": {"forward_distance_m": float(position[0]), "lateral_distance_m": float(position[1]),
                                       "max_lateral_distance_m": self._max_lateral,
                                       "max_tilt_deg": math.degrees(self._max_tilt), "min_body_height_m": self._min_height,
                                       "final_body_height_m": float(position[2]), "fall_time": self._fall_time,
                                       "joint_tracking_rmse_rad": math.sqrt(self._tracking_square / max(1, 12 * self._samples)),
                                       "duration_s": float(self.data.time)}},
                "config": asdict(self.config), "events": [event.copy() for event in self.events],
                "diagnostics": self.diagnostics(), "warnings": warnings,
                "limitations": ["Primitive dog with a slow nominal crawl; not a validated hardware locomotion policy.",
                                "Damage uses actuator gain, foot friction, a knee spring, and an internal payload slide.",
                                "Safe refers only to the executed synthetic probe, not hardware safety."]}

    def reset_trial(self) -> None:
        self._offset = self.elapsed
        self._trial += 1
        self._initialize()

    def reset_full(self) -> None:
        for name, values in self._baseline.items():
            getattr(self.model, name)[:] = values
        self._offset = 0
        self._trial = 0
        self._fault_active = False
        self.events.clear()
        self._initialize()
