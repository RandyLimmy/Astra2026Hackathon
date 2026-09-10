"""Deterministic MuJoCo experiment runner. No agent or model-repair logic."""
from dataclasses import replace
import math
from typing import Callable

import mujoco
import numpy as np

from .config import Experiment, WHEELS
from .model import build_model
from .private import thermal

# Front bias accounts for load transfer. Values calibrated on this synthetic car.
BRAKE_TORQUES = np.array([835.0, 835.0, 557.0, 557.0])
WHEEL_RADIUS = 0.34


class Simulator:
    config: Experiment
    impact_speed: float | None
    collision_time: float | None

    def __init__(self, config: Experiment = Experiment()):
        self.elapsed = 0.0
        self.reset_full(config)

    def reset_full(self, config: Experiment | None = None):
        """Restore healthy defaults, or start a newly declared experiment."""
        self.config = config if config is not None else Experiment(timestep=self.config.timestep)
        self.model = build_model(self.config)
        self.data = mujoco.MjData(self.model)
        self.chassis = self.model.body("chassis").id
        self.chassis_geom = self.model.geom("chassis_geom").id
        self.wall = self.model.geom("wall").id if self.config.wall else -1
        self.spin = np.array([self.model.joint(f"spin_{w}").dofadr[0] for w in WHEELS])
        self.attach = np.array([self.model.equality(f"attach_{w}").id for w in WHEELS])
        self.carriers = [self.model.body(f"carrier_{w}").id for w in WHEELS]
        self.free_joints = [self.model.joint(name) for name in
                            ["chassis_free"] + [f"carrier_{w}_free" for w in WHEELS]]
        self.temperature = np.full(4, thermal.AMBIENT)
        self.activation = 0.0
        self.efficiency = np.ones(4)
        self.events: list[dict] = []
        self.elapsed = 0.0
        self.trial_time = 0.0
        self.brake_latched = False
        self.last_command = (0.0, 0.0)
        self.command_started_at = 0.0
        self.last_brake_torque = np.zeros(4)
        self.collision = False
        self.impact_speed = None
        self.collision_time = None
        # Settle static wheel contacts before taking the canonical reset snapshot.
        mujoco.mj_step(self.model, self.data, nstep=500)
        self.settled_qpos = self.data.qpos.copy()
        self.reset_trial(self.config.initial_speed)

    def reset_trial(self, speed: float | None = None):
        """Reposition the run; preserve heat, weakened brakes and released welds.

        All body positions translate with the car. Detached carriers retain their
        relative offsets/orientations; attached parts use settled canonical poses.
        Brake activation resets to zero as an explicit preparation intervention.
        """
        if speed is None:
            speed = self.config.initial_speed
        if not math.isfinite(speed) or not 0 <= speed <= 35:
            raise ValueError("reset speed must be [0,35]")
        active = self.data.eq_active.copy()
        previous = self.data.qpos.copy()
        old_origin = previous[:3].copy()
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:] = self.settled_qpos
        self.data.eq_active[:] = active
        for index, joint in enumerate(self.free_joints):
            qa, va = int(joint.qposadr[0]), int(joint.dofadr[0])
            if index and not active[self.attach[index - 1]]:
                self.data.qpos[qa:qa + 7] = previous[qa:qa + 7]
                self.data.qpos[qa:qa + 3] -= old_origin - self.settled_qpos[:3]
            self.data.qvel[va] = speed
        self.data.qvel[self.spin] = speed / WHEEL_RADIUS
        self.data.time = self.elapsed
        self.activation = 0.0
        self.trial_time = 0.0
        self.brake_latched = False
        self.collision = False
        self.impact_speed = None
        self.collision_time = None
        self.last_command = (0.0, 0.0)
        self.command_started_at = 0.0
        self.last_brake_torque[:] = 0
        self.model.dof_frictionloss[self.spin] = 0
        mujoco.mj_forward(self.model, self.data)

    def _events(self):
        config = self.config
        if config.detach_wheel and self.trial_time + 1e-10 >= config.detach_at:
            i = WHEELS.index(config.detach_wheel)
            if self.data.eq_active[self.attach[i]]:
                self.data.eq_active[self.attach[i]] = False
                self.events.append({"time": self.elapsed, "trial_time": self.trial_time,
                                    "event": "wheel_release", "wheel": config.detach_wheel})
        if config.weak_wheel and self.trial_time + 1e-10 >= config.weak_at:
            i = WHEELS.index(config.weak_wheel)
            if self.efficiency[i] != config.brake_efficiency:
                self.efficiency[i] = config.brake_efficiency
                self.events.append({"time": self.elapsed, "trial_time": self.trial_time,
                                    "event": "brake_efficiency", "wheel": config.weak_wheel})

    def command(self):
        if self.config.commands:
            command = (0.0, 0.0)
            for t, throttle, brake in self.config.commands:
                if t > self.trial_time + 1e-10:
                    break
                command = (throttle, brake)
            return command
        if self.front_x >= self.config.brake_at:
            self.brake_latched = True
        return (0.0, self.config.brake) if self.brake_latched else (self.config.throttle, 0.0)

    @property
    def front_x(self):
        # Leading corner of the oriented chassis collision box, including yaw/pitch.
        rotation = self.data.geom_xmat[self.chassis_geom].reshape(3, 3)
        halfsize = self.model.geom_size[self.chassis_geom]
        return float(self.data.geom_xpos[self.chassis_geom, 0] + abs(rotation[0]) @ halfsize)

    @property
    def speed(self):
        return float(np.linalg.norm(self.data.qvel[:3]))

    @property
    def yaw(self):
        rotation = self.data.xmat[self.chassis].reshape(3, 3)
        return float(math.atan2(rotation[1, 0], rotation[0, 0]))

    def step(self, throttle: float, brake: float, *, events: bool = True):
        if not 0 <= throttle <= 1 or not 0 <= brake <= 1:
            raise ValueError("throttle and brake must be [0,1]")
        dt = self.config.timestep
        if (throttle, brake) != self.last_command:
            self.command_started_at = self.trial_time
        if events:
            self._events()
        self.activation = brake if self.config.lag == 0 else (
            self.activation + (brake - self.activation) * -math.expm1(-dt / self.config.lag)
        )
        omega = self.data.qvel[self.spin].copy()
        capacity = BRAKE_TORQUES * self.activation * self.efficiency
        if self.config.thermal:
            capacity *= thermal.fade(self.temperature)
        attached = self.data.eq_active[self.attach]
        # MuJoCo's bounded joint-friction constraint is a physical disc brake:
        # it resists spin and can hold at rest without commanding reverse torque.
        self.model.dof_frictionloss[self.spin] = capacity * attached
        self.data.ctrl[:] = 500.0 * throttle * attached
        before_speed = self.speed
        mujoco.mj_step(self.model, self.data)
        brake_torque = np.zeros(4)
        for i, dof in enumerate(self.spin):
            rows = ((self.data.efc_type == mujoco.mjtConstraint.mjCNSTR_FRICTION_DOF)
                    & (self.data.efc_id == dof))
            brake_torque[i] = self.data.efc_force[rows].sum()
        # Refresh kinematics/contact buffers to the returned, post-step state.
        mujoco.mj_forward(self.model, self.data)
        power = np.maximum(-brake_torque * (omega + self.data.qvel[self.spin]) / 2, 0.0)
        if self.config.thermal:
            self.temperature = thermal.advance(self.temperature, power, dt)
        self.elapsed += dt
        self.trial_time += dt
        self.last_command = (throttle, brake)
        self.last_brake_torque = brake_torque
        if self.wall >= 0 and not self.collision:
            for contact in self.data.contact:
                if self.wall not in contact.geom:
                    continue
                other = int(contact.geom[0] if contact.geom[1] == self.wall else contact.geom[1])
                body = int(self.model.geom_bodyid[other])
                # A loose wheel hitting the wall is not a chassis collision.
                if body == self.chassis or self.model.body_rootid[body] == self.chassis:
                    self.collision = True
                    self.impact_speed = before_speed
                    self.collision_time = self.trial_time
                    break
        if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
            raise RuntimeError("non-finite MuJoCo state")
        if self.data.warning.number.any():
            raise RuntimeError(f"MuJoCo warning counts: {self.data.warning.number.tolist()}")
        if abs(self.data.time - self.elapsed) > dt / 2:
            raise RuntimeError("MuJoCo reset or simulation clock discontinuity")

    def observe(self, phase="trial"):
        throttle, brake = self.last_command
        rotation = self.data.xmat[self.chassis].reshape(3, 3)
        heading = rotation[:, 0]
        heading_dot = np.cross(rotation @ self.data.qvel[3:6], heading)
        horizontal = float(heading[0] ** 2 + heading[1] ** 2)
        yaw_rate = (float((heading[0] * heading_dot[1] - heading[1] * heading_dot[0]) / horizontal)
                    if horizontal > 1e-10 else None)
        return {"time": self.elapsed, "phase": phase, "phase_time": self.trial_time,
                "position": self.data.xpos[self.chassis].tolist(),
                "velocity": self.data.qvel[:3].tolist(),
                "yaw": self.yaw, "yaw_rate": yaw_rate,
                "wheel_speed": self.data.qvel[self.spin].tolist(),
                "throttle": throttle, "brake": brake,
                "front_x": self.front_x, "wall_contact": self.collision,
                "lane_departure": bool(abs(self.data.xpos[self.chassis, 1]) > 3.2)}

    def diagnostics(self):
        rotation = self.data.xmat[self.chassis].reshape(3, 3)
        offsets = np.array([[1.35, .98, -.24], [1.35, -.98, -.24],
                            [-1.35, .98, -.24], [-1.35, -.98, -.24]])
        expected = self.data.xpos[self.chassis] + offsets @ rotation.T
        separation = np.linalg.norm(self.data.xpos[self.carriers] - expected, axis=1)
        return {"time": self.elapsed, "temperature": self.temperature.tolist(),
                "brake_activation": self.activation, "brake_torque": self.last_brake_torque.tolist(),
                "efficiency": self.efficiency.tolist(),
                "attached": self.data.eq_active[self.attach].astype(bool).tolist(),
                "carrier_separation": separation.tolist(),
                "contact_friction": [float(c.friction[0]) for c in self.data.contact],
                "warning_counts": self.data.warning.number.tolist()}

    def trial_complete(self, stationary_ticks: int) -> bool:
        """Shared recording/viewer endpoint; scheduled future controls take priority over rest."""
        if self.trial_time + 1e-10 >= self.config.duration:
            return True
        if self.collision_time is not None and self.trial_time - self.collision_time >= 0.5:
            return True
        pending = any(t >= self.trial_time - 1e-10 for t, _, _ in self.config.commands)
        return (not pending and stationary_ticks * self.config.timestep >= 0.5
                and self.trial_time - self.command_started_at >= 0.5)

    def prepare(self, callback: Callable | None = None):
        """Measured accelerate/brake conditioning, then explicit recovery/reset."""
        wall_flags = None
        if self.wall >= 0:
            wall_flags = (self.model.geom_contype[self.wall], self.model.geom_conaffinity[self.wall],
                          self.model.geom_rgba[self.wall, 3])
            self.model.geom_contype[self.wall] = self.model.geom_conaffinity[self.wall] = 0
            self.model.geom_rgba[self.wall, 3] = 0
        try:
            for cycle in range(self.config.warmup_cycles):
                self.reset_trial(0)
                phase = f"conditioning_{cycle + 1}"
                if callback:
                    callback(self, phase)
                # Accelerate physically, then dissipate kinetic energy through brakes.
                while self.speed < 25 and self.trial_time < 15:
                    self.step(0.8, 0, events=False)
                    if callback:
                        callback(self, phase)
                while self.speed > 0.1 and self.trial_time < 35:
                    self.step(0, 1, events=False)
                    if callback:
                        callback(self, phase)
            if self.config.recovery:
                self.reset_trial(0)
                for _ in range(round(self.config.recovery / self.config.timestep)):
                    self.step(0, 0, events=False)
                    if callback:
                        callback(self, "recovery")
        finally:
            if wall_flags:
                self.model.geom_contype[self.wall], self.model.geom_conaffinity[self.wall], \
                    self.model.geom_rgba[self.wall, 3] = wall_flags
        self.reset_trial()

    def run(self, callback: Callable | None = None):
        self.prepare(callback)
        start_temperature = self.temperature.copy()
        brake_start_x = None
        brake_start_time = None
        stop_time = None
        stop_counter = 0
        max_yaw = 0.0
        max_lateral = 0.0
        max_separation = np.zeros(4)
        if callback:
            callback(self, "trial")
        for _ in range(math.ceil(self.config.duration / self.config.timestep)):
            throttle, brake = self.command()
            if brake > 0 and brake_start_x is None:
                brake_start_x, brake_start_time = self.front_x, self.trial_time
            self.step(throttle, brake)
            max_yaw = max(max_yaw, abs(math.degrees(self.yaw)))
            max_lateral = max(max_lateral, abs(float(self.data.xpos[self.chassis, 1])))
            if self.config.detach_wheel:
                max_separation = np.maximum(max_separation, self.diagnostics()["carrier_separation"])
            if callback:
                callback(self, "trial")
            stop_counter = stop_counter + 1 if self.speed < 0.1 else 0
            if self.trial_complete(stop_counter):
                if stop_counter * self.config.timestep >= 0.5:
                    stop_time = self.trial_time
                break
        stopped = stop_time is not None and not self.collision
        public = {"collision": self.collision, "impact_speed": self.impact_speed,
                  "collision_time": self.collision_time, "stopped": stopped,
                  "censored": not stopped, "brake_start_x": brake_start_x,
                  "brake_start_time": brake_start_time,
                  "stopping_distance": self.front_x - brake_start_x if stopped and brake_start_x is not None else None,
                  "final_front_x": self.front_x, "final_speed": self.speed,
                  "wall_clearance": self.config.wall_x - self.front_x if self.config.wall else None,
                  "max_yaw_degrees": max_yaw, "max_lateral_displacement": max_lateral,
                  "lane_departure": max_lateral > 3.2, "trial_duration": self.trial_time}
        return {**public, "config": self.config.to_dict(), "events": self.events,
                "initial_temperature": start_temperature.tolist(),
                "final_temperature": self.temperature.tolist(),
                "max_carrier_separation": max_separation.tolist(),
                "warnings": self.data.warning.number.tolist(),
                "mujoco_version": mujoco.__version__, "public": public}


def run_experiment(config: Experiment, callback=None):
    return Simulator(config).run(callback)


def counterfactual(config: Experiment, callback=None):
    """A separate wall-free run from the same declared conditioning history."""
    return run_experiment(replace(config, wall=False), callback)
