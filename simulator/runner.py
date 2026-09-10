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

    def __init__(self, config: Experiment = Experiment(), actuator=None):
        # Candidate implementations stay outside this process. The host sees
        # only the neutral wheel-component client interface.
        self.actuator = actuator
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
        self.preparation_history: list[dict] = []
        self.elapsed = 0.0
        self.trial_time = 0.0
        self.brake_latched = False
        self.last_command = (0.0, 0.0)
        self.command_started_at = 0.0
        self.last_intervention_at = 0.0
        self.last_brake_torque = np.zeros(4)
        self.collision = False
        self.impact_speed = None
        self.collision_time = None
        # Settle static wheel contacts before taking the canonical reset snapshot.
        mujoco.mj_step(self.model, self.data, nstep=500)
        self.settled_qpos = self.data.qpos.copy()
        if self.actuator is not None:
            self.actuator.reset()
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
        self.last_intervention_at = 0.0
        self.last_brake_torque[:] = 0
        self.model.dof_frictionloss[self.spin] = 0
        mujoco.mj_forward(self.model, self.data)
        if self.actuator is not None:
            self.actuator.reposition()

    def _events(self):
        config = self.config
        if config.detach_wheel and self.trial_time + 1e-10 >= config.detach_at:
            i = WHEELS.index(config.detach_wheel)
            if self.data.eq_active[self.attach[i]]:
                self.data.eq_active[self.attach[i]] = False
                self.last_intervention_at = self.trial_time
                self.events.append({"time": self.elapsed, "trial_time": self.trial_time,
                                    "event": "wheel_release", "wheel": config.detach_wheel})
        if config.weak_wheel and self.trial_time + 1e-10 >= config.weak_at:
            i = WHEELS.index(config.weak_wheel)
            if self.efficiency[i] != config.brake_efficiency:
                self.efficiency[i] = config.brake_efficiency
                self.last_intervention_at = self.trial_time
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
        omega = self.data.qvel[self.spin].copy()
        if self.actuator is None:
            self.activation = brake if self.config.lag == 0 else (
                self.activation + (brake - self.activation) * -math.expm1(-dt / self.config.lag)
            )
            capacity = BRAKE_TORQUES * self.activation * self.efficiency
            if self.config.thermal:
                capacity *= thermal.fade(self.temperature)
        else:
            capacity = np.asarray(self.actuator.torque_limits(brake, omega.tolist()), dtype=float)
            if capacity.shape != (4,) or not np.isfinite(capacity).all() or np.any(capacity < 0):
                raise ValueError("Candidate must return four finite nonnegative torque capacities")
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
        mean_omega = (omega + self.data.qvel[self.spin]) / 2
        if self.actuator is not None:
            self.actuator.advance(brake, mean_omega.tolist(), brake_torque.tolist(), dt)
        elif self.config.thermal:
            power = np.maximum(-brake_torque * mean_omega, 0.0)
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
        result = {"time": self.elapsed, "brake_torque": self.last_brake_torque.tolist(),
                  "efficiency": self.efficiency.tolist(),
                  "attached": self.data.eq_active[self.attach].astype(bool).tolist(),
                  "carrier_separation": separation.tolist(),
                  "contact_friction": [float(c.friction[0]) for c in self.data.contact],
                  "warning_counts": self.data.warning.number.tolist()}
        if self.actuator is None:
            result.update(temperature=self.temperature.tolist(), brake_activation=self.activation)
        return result

    def trial_complete(self, stationary_ticks: int) -> bool:
        """Do not end at rest before scheduled controls or physical fault events."""
        if self.trial_time + 1e-10 >= self.config.duration:
            return True
        if self.collision_time is not None and self.trial_time - self.collision_time >= 0.5:
            return True
        pending = any(t >= self.trial_time - 1e-10 for t, _, _ in self.config.commands)
        if self.config.detach_wheel:
            i = WHEELS.index(self.config.detach_wheel)
            pending |= (bool(self.data.eq_active[self.attach[i]])
                        and self.config.detach_at >= self.trial_time - 1e-10)
        if self.config.weak_wheel:
            i = WHEELS.index(self.config.weak_wheel)
            pending |= (self.efficiency[i] != self.config.brake_efficiency
                        and self.config.weak_at >= self.trial_time - 1e-10)
        return (not pending and stationary_ticks * self.config.timestep >= 0.5
                and self.trial_time - max(self.command_started_at, self.last_intervention_at) >= 0.5)

    def prepare(self, callback: Callable | None = None, history=None):
        """Record or replay the ordered, public preparation interventions.

        Reset events name the imposed speed; step events contain commands,
        timestep, phase, and a run-length count. Replay advances the candidate's
        own mechanics for those exact intervals, without reference states,
        forces, private parameters, or speed-dependent reference feedback.
        """
        replay = self._validated_preparation(history) if history is not None else None
        self.preparation_history = []

        def reset(speed, phase):
            self.reset_trial(speed)
            self.preparation_history.append({"kind": "reset", "speed_mps": float(speed), "phase": phase})
            if callback and phase != "trial":
                callback(self, phase)

        def step(throttle, brake, phase):
            self.step(throttle, brake, events=False)
            event = {"kind": "step", "throttle": float(throttle), "brake": float(brake),
                     "dt_s": self.config.timestep, "steps": 1, "phase": phase}
            previous = self.preparation_history[-1] if self.preparation_history else None
            if previous and all(previous.get(key) == event[key] for key in event if key != "steps"):
                previous["steps"] += 1
            else:
                self.preparation_history.append(event)
            if callback:
                callback(self, phase)

        wall_flags = None
        if self.wall >= 0:
            wall_flags = (self.model.geom_contype[self.wall], self.model.geom_conaffinity[self.wall],
                          self.model.geom_rgba[self.wall, 3])
            self.model.geom_contype[self.wall] = self.model.geom_conaffinity[self.wall] = 0
            self.model.geom_rgba[self.wall, 3] = 0
        try:
            if replay is not None:
                # The final probe reset happens after restoring the wall,
                # exactly as in the original preparation implementation.
                for event in replay[:-1]:
                    if event["kind"] == "reset":
                        reset(event["speed_mps"], event["phase"])
                    else:
                        for _ in range(event["steps"]):
                            step(event["throttle"], event["brake"], event["phase"])
            else:
                for cycle in range(self.config.warmup_cycles):
                    phase = f"conditioning_{cycle + 1}"
                    reset(0, phase)
                    # Accelerate physically, then dissipate kinetic energy through brakes.
                    while self.speed < 25 and self.trial_time < 15:
                        step(0.8, 0, phase)
                    while self.speed > 0.1 and self.trial_time < 35:
                        step(0, 1, phase)
                if self.config.recovery:
                    reset(0, "recovery")
                    for _ in range(round(self.config.recovery / self.config.timestep)):
                        step(0, 0, "recovery")
        finally:
            if wall_flags:
                self.model.geom_contype[self.wall], self.model.geom_conaffinity[self.wall], \
                    self.model.geom_rgba[self.wall, 3] = wall_flags
        reset(replay[-1]["speed_mps"] if replay is not None else self.config.initial_speed, "trial")

    def _validated_preparation(self, history):
        """Validate the neutral replay schema before changing the simulator."""
        if not isinstance(history, list) or not history:
            raise ValueError("Preparation history must end with a probe reset")
        result = []
        for event in history:
            if not isinstance(event, dict) or not isinstance(event.get("phase"), str):
                raise ValueError("Invalid preparation event")
            if event.get("kind") == "reset":
                if set(event) != {"kind", "speed_mps", "phase"}:
                    raise ValueError("Unexpected preparation reset fields")
                speed = event["speed_mps"]
                if isinstance(speed, bool) or not isinstance(speed, (float, int)) or not math.isfinite(speed) or not 0 <= speed <= 35:
                    raise ValueError("Invalid preparation reset speed")
            elif event.get("kind") == "step":
                if set(event) != {"kind", "throttle", "brake", "dt_s", "steps", "phase"}:
                    raise ValueError("Unexpected preparation step fields")
                if type(event["steps"]) is not int or event["steps"] < 1:
                    raise ValueError("Preparation step count must be a positive integer")
                for key in ("throttle", "brake", "dt_s"):
                    value = event[key]
                    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
                        raise ValueError("Preparation numeric inputs must be finite")
                if not 0 <= event["throttle"] <= 1 or not 0 <= event["brake"] <= 1:
                    raise ValueError("Preparation commands must be in [0,1]")
                if not math.isclose(event["dt_s"], self.config.timestep, rel_tol=0, abs_tol=1e-12):
                    raise ValueError("Preparation timestep must match the configured mechanics")
            else:
                raise ValueError("Unknown preparation event kind")
            result.append(dict(event))
        final = result[-1]
        if final["kind"] != "reset" or final["phase"] != "trial" or final["speed_mps"] != self.config.initial_speed:
            raise ValueError("Preparation history must end at the configured probe speed")
        return result

    def run(self, callback: Callable | None = None, preparation_history=None, *, prepared=False):
        if prepared and preparation_history is not None:
            raise ValueError("prepared=True cannot be combined with preparation_history")
        if not prepared:
            self.prepare(callback, history=preparation_history)
        start_temperature = self.temperature.copy() if self.actuator is None else None
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
        result = {**public, "config": self.config.to_dict(), "events": self.events,
                  "max_carrier_separation": max_separation.tolist(),
                  "warnings": self.data.warning.number.tolist(),
                  "mujoco_version": mujoco.__version__, "public": public}
        if self.actuator is None:
            result.update(initial_temperature=start_temperature.tolist(),
                          final_temperature=self.temperature.tolist())
        return result


def run_experiment(config: Experiment, callback=None):
    return Simulator(config).run(callback)


def counterfactual(config: Experiment, callback=None):
    """A separate wall-free run from the same declared conditioning history."""
    return run_experiment(replace(config, wall=False), callback)
