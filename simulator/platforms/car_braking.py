"""F4: a cold-calibrated brake trigger meets a physically conditioned car.

The unchanged four-wheel ``runner.Simulator`` owns every force, contact and
thermal update. Preparation physically accelerates and brakes the car; resets
are explicit interventions, never inferred animation events. No candidate brake
component or repaired controller is installed here.

``elapsed`` / observation ``time`` include all conditioning. ``trial_time`` and
``phase_time`` restart at the declared approach reset; Config.duration bounds
that approach, excluding preparation. Full replay repeats the conditioning on
the existing model/data; trial reset retains the current brake temperature.
"""

from dataclasses import asdict, dataclass
import math

import mujoco
import numpy as np

from ..config import Experiment
from ..private import thermal
from ..runner import Simulator


FAULTS = ("thermal_history", "healthy")
PROBES = ("braking", "wall_free")


@dataclass(frozen=True)
class Config:
    fault: str = "thermal_history"
    duration: float = 12.0
    timestep: float = 0.002
    probe: str = "braking"
    initial_speed: float = 25.0
    brake_at: float = 45.0
    wall_x: float = 100.0
    warmup_cycles: int = 4
    recovery: float = 0.0

    def __post_init__(self):
        if self.fault not in FAULTS:
            raise ValueError(f"fault must be one of {FAULTS}")
        if self.probe not in PROBES:
            raise ValueError(f"probe must be one of {PROBES}")
        for name, low, high in (
            ("duration", 0.1, 120), ("timestep", 0.0005, 0.004),
            ("initial_speed", 0.1, 35), ("brake_at", 3, 200),
            ("wall_x", 10, 250), ("recovery", 0, 600),
        ):
            value = getattr(self, name)
            if (isinstance(value, bool) or not isinstance(value, (float, int))
                    or not math.isfinite(value) or not low <= value <= high):
                raise ValueError(f"{name} must be finite and in [{low}, {high}]")
        if type(self.warmup_cycles) is not int or not 0 <= self.warmup_cycles <= 12:
            raise ValueError("warmup_cycles must be an integer in [0, 12]")
        if self.brake_at >= self.wall_x:
            raise ValueError("brake_at must precede the barrier")


PRESETS = {
    "car_auto_brake_failure": {},
    "car_auto_brake_cold_control": {"fault": "healthy"},
    "car_auto_brake_wall_free": {"probe": "wall_free"},
}
DESCRIPTIONS = {
    "car_auto_brake_failure": "Repeated braking precedes a barrier approach with a late automatic brake trigger.",
    "car_auto_brake_cold_control": "Developer control: the same automatic trigger with unconditioned brakes.",
    "car_auto_brake_wall_free": "Developer control: the conditioned car's uncensored stopping distance, without a wall.",
}


class Simulation:
    """Platform adapter, recording history and outcomes around the trusted plant."""

    camera_distance = 16.0
    task_metadata: dict = {
        "scenario_id": "F4",
        "title": "Automatic braking",
        "objective": "Approach the barrier at 25 m/s and stop before contact.",
        "repair_status": "not_run",
    }

    def __init__(self, config: Config = Config()):
        self.config = config
        self._plant = Simulator(Experiment(
            initial_speed=config.initial_speed, brake_at=config.brake_at,
            wall_x=config.wall_x, wall=config.probe != "wall_free",
            duration=config.duration, timestep=config.timestep, thermal=True,
            warmup_cycles=config.warmup_cycles if config.fault != "healthy" else 0,
            recovery=config.recovery,
        ))
        self.model, self.data = self._plant.model, self._plant.data
        self.focus_body = self._plant.chassis
        self.task_metadata = {
            **self.task_metadata,
            "objective": ("Stop in green zone before barrier"
                          if config.probe == "braking" else
                          f"Measure the free stopping distance from {config.initial_speed:g} m/s."),
        }
        self.goal_zone: dict | None = {
            "label": "STOP TARGET", "x_start": max(2.0, config.wall_x - 14.0),
            "x_end": config.wall_x - 2.0, "width": 8.0, "minimum_clearance": 2.0,
        } if self._plant.config.wall else None
        self.task_metadata.update(
            goal_label="Stop in green zone before barrier" if self.goal_zone else "Complete the open-track stop",
            goal_summary="Stop in the green zone, at least 2 m before the striped barrier."
                         if self.goal_zone else "Brake to a complete stop on the open track.",
            visible_zone=self.goal_zone,
        )
        self._decorate_goal()
        self._chase = self.model.camera("chase").id
        self.model.cam_mode[self._chase] = mujoco.mjtCamLight.mjCAMLIGHT_FIXED
        self._side = self.model.camera("side").id
        self._overview = self.model.camera("overview").id
        self.reset_full()

    def _decorate_goal(self) -> None:
        """Repurpose existing force-free distance ticks; physical geoms stay intact."""
        self._goal_geoms: list[int] = []
        if self.goal_zone is None:
            return
        self._goal_geoms = [
            i for i in range(self.model.ngeom)
            if self.model.geom_bodyid[i] == 0 and not self.model.geom_contype[i]
            and not self.model.geom_conaffinity[i]
            and mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, i) is None
        ]
        if len(self._goal_geoms) < 17:
            raise RuntimeError("The braking track needs its 17 visual distance markers")
        self._goal_geoms = self._goal_geoms[:17]
        start, end = self.goal_zone["x_start"], self.goal_zone["x_end"]
        center, half_length = (start + end) / 2, (end - start) / 2

        def marker(index, position, size, color, angle=0.0):
            geom = self._goal_geoms[index]
            self.model.geom_pos[geom] = position
            self.model.geom_size[geom] = size
            self.model.geom_rgba[geom] = color
            self.model.geom_quat[geom] = [math.cos(angle / 2), math.sin(angle / 2), 0, 0]

        green = [0.05, 0.72, 0.34, 1.0]
        white = [0.85, 1.0, 0.85, 1.0]
        marker(0, [center, 0, 0.014], [half_length, 3.9, 0.005], green)
        for i, x in enumerate((start, end), 1):
            marker(i, [x, 0, 0.028], [0.11, 3.9, 0.006], white)
        for i, y in enumerate((-3.9, 3.9), 3):
            marker(i, [center, y, 0.028], [half_length, 0.11, 0.006], white)
        self.model.geom_rgba[self._plant.wall] = [1.0, 0.72, 0.04, 1.0]
        for i in range(10):
            marker(i + 5, [self.config.wall_x - 0.025, -5.4 + 1.2 * i, 1.5],
                   [0.012, 0.26, 1.65], [0.055, 0.055, 0.06, 1.0], angle=-0.38)
        for i, y in enumerate((-4.35, 4.35), 15):
            marker(i, [end, y, 1.5], [0.18, 0.18, 1.5], green)

    @property
    def elapsed(self) -> float:
        return self._plant.elapsed

    @property
    def trial_time(self) -> float:
        return self._plant.trial_time

    @property
    def finished(self) -> bool:
        return self.trial_time >= self._finish_at - self.config.timestep / 2

    @property
    def preparation_history(self) -> list[dict]:
        return self._plant.preparation_history

    @property
    def public_events(self) -> list[dict]:
        return [dict(event) for event in self._events]

    def _event(self, name: str, **values) -> None:
        self._events.append({
            "id": f"F4-{len(self._events):04d}", "event": name,
            "time": self.elapsed, "trial_time": self.trial_time, **values,
        })

    def _prepare_capture(self, plant: Simulator, phase: str) -> None:
        # prepare() invokes this at every reset and every physics step. Keep all
        # records, including duplicate timestamps spanning a declared reset.
        if plant.trial_time == 0:
            self._event("preparation_reset", phase=phase, speed_mps=plant.speed,
                        preserves_component_state=True)
        observation = plant.observe(phase)
        observation.update(
            platform="car_braking", speed=plant.speed,
            command={"throttle": plant.last_command[0], "brake": plant.last_command[1]},
            wall_clearance=None, bumper_clearance=None, barrier_enabled=False,
        )
        self.preparation_observations.append(observation)
        diagnostic = plant.diagnostics()
        diagnostic["phase"] = phase
        self.preparation_diagnostics.append(diagnostic)

    def reset_full(self) -> None:
        """Repeat original conditioning, preserving native viewer object identity."""
        plant = self._plant
        mujoco.mj_resetData(self.model, self.data)
        plant.elapsed = 0.0
        plant.temperature.fill(thermal.AMBIENT)
        plant.efficiency.fill(1.0)
        plant.events.clear()
        self._events: list[dict] = []
        self.preparation_observations: list[dict] = []
        self.preparation_diagnostics: list[dict] = []
        plant.reset_trial()
        self._event("preparation_start", phase="preparation", component_state="initial")
        # Conditioning occurs on the declared open preparation track.
        if self._goal_geoms:
            self.model.geom_rgba[self._goal_geoms, 3] = 0
        plant.prepare(self._prepare_capture)
        if self._goal_geoms:
            self.model.geom_rgba[self._goal_geoms, 3] = 1
        self.conditioning_duration = self.elapsed
        self._start_approach("approach_reset")

    def reset_trial(self) -> None:
        """Explicit approach reposition with current component history retained."""
        plant = self._plant
        before = plant.observe("trial_end")
        # The previous approach also dissipated energy. Preserve its complete
        # observed motion and controls as history for this retained-state trial.
        self.preparation_observations.extend(self._trial_observations)
        self.preparation_diagnostics.extend(self._trial_diagnostics)
        plant.preparation_history.extend(self._trial_commands)
        plant.reset_trial()
        plant.preparation_history.append({"kind": "reset", "speed_mps": self.config.initial_speed,
                                          "phase": "trial"})
        self._start_approach("trial_reset", previous_position=before["position"])

    def _start_approach(self, reset_event: str, **values) -> None:
        self.approach_start_time = self.elapsed
        self._finish_at = self.config.duration
        self._stationary_time = 0.0
        self._stop_time: float | None = None
        self._brake_start_time: float | None = None
        self._brake_start_x: float | None = None
        self._brake_start_speed: float | None = None
        self._initial_temperature = self._plant.temperature.copy()
        self._initial_front_x = self._plant.front_x
        self._max_speed = self._plant.speed
        self._last_deceleration = 0.0
        self._event(reset_event, phase="approach", speed_mps=self.config.initial_speed,
                    preserves_component_state=True, **values)
        self._event("task_start", phase="approach", barrier_enabled=self._plant.config.wall)
        self._update_camera()
        self._trial_commands: list[dict] = []
        self._trial_observations = [self.observe()]
        self._trial_diagnostics = [self._plant.diagnostics()]

    def _update_camera(self) -> None:
        """Track the bumper and its remaining gap; camera updates exert no force."""
        x = float(self.data.xpos[self.focus_body, 0])
        # The road, green goal and striped barrier must be visible at time zero.
        self._point_camera(self._chase, np.array([x - 8.0, -1.5, 4.8]),
                           np.array([x + 14.0, 0.0, 0.8]), 56)
        self._point_camera(self._side, np.array([x + 3.0, -12.0, 5.5]),
                           np.array([x + 3.0, 0.0, 0.6]), 52)
        clearance = max(0.0, self.config.wall_x - x) if self._plant.config.wall else 20.0
        midpoint = x + clearance / 2
        self._point_camera(self._overview,
                           np.array([midpoint, -max(17.0, clearance * 0.40), max(21.0, clearance * 0.65)]),
                           np.array([midpoint, 0.0, 0.0]), 56)
        mujoco.mj_camlight(self.model, self.data)

    def _point_camera(self, camera: int, eye: np.ndarray, target: np.ndarray, fovy: float) -> None:
        z = eye - target
        z /= np.linalg.norm(z)
        right = np.cross(np.array([0.0, 0.0, 1.0]), z)
        right /= np.linalg.norm(right)
        up = np.cross(z, right)
        matrix = np.column_stack((right, up, z))
        body = self.model.cam_bodyid[camera]
        rotation = self.data.xmat[body].reshape(3, 3)
        self.model.cam_pos[camera] = rotation.T @ (eye - self.data.xpos[body])
        mujoco.mju_mat2Quat(self.model.cam_quat[camera], (rotation.T @ matrix).ravel())
        self.model.cam_fovy[camera] = fovy

    def step(self, control: dict | None = None) -> None:
        if self.finished:
            return
        plant = self._plant
        if control is None:
            throttle, brake = plant.command()
        else:
            if set(control) - {"throttle", "brake"}:
                raise ValueError("braking controls accept only throttle and brake")
            throttle, brake = control.get("throttle", 0.0), control.get("brake", 0.0)
            for value in (throttle, brake):
                if (isinstance(value, bool) or not isinstance(value, (int, float))
                        or not math.isfinite(value) or not 0 <= value <= 1):
                    raise ValueError("throttle and brake must be finite and in [0, 1]")
        if brake > 0 and self._brake_start_time is None:
            self._brake_start_time = self.trial_time
            self._brake_start_x = plant.front_x
            self._brake_start_speed = plant.speed
            self._event("brake_onset", phase="braking", speed=plant.speed,
                        bumper_clearance=self._clearance(), command={"throttle": throttle, "brake": brake})
        previous_speed = plant.speed
        had_collision = plant.collision
        plant.step(throttle, brake)
        self._last_deceleration = (previous_speed - plant.speed) / self.config.timestep
        self._max_speed = max(self._max_speed, plant.speed)
        if plant.collision and not had_collision:
            self._event("barrier_contact", phase="impact", impact_speed=plant.impact_speed,
                        bumper_clearance=self._clearance())
            # Full physical aftermath; do not freeze or reposition at impact.
            self._finish_at = min(self.config.duration, self.trial_time + 2.0)
        self._stationary_time = self._stationary_time + self.config.timestep if plant.speed < 0.1 else 0.0
        if plant.speed >= 0.1 and self._stop_time is not None and not plant.collision:
            self._stop_time = None
            self._finish_at = self.config.duration
        if self._stationary_time >= 0.5 - 1e-10 and self._stop_time is None and not plant.collision:
            self._stop_time = self.trial_time
            self._event("stopped", phase="stopped", speed=plant.speed, bumper_clearance=self._clearance())
            self._finish_at = min(self.config.duration, self.trial_time + 0.5)
        self._update_camera()
        if self.finished:
            self._event("outcome", phase=self.observe()["phase"], outcome=self._outcome())
        self._trial_observations.append(self.observe())
        self._trial_diagnostics.append(plant.diagnostics())
        entry = {"kind": "step", "throttle": float(throttle), "brake": float(brake),
                 "dt_s": self.config.timestep, "steps": 1, "phase": "trial"}
        previous = self._trial_commands[-1] if self._trial_commands else None
        if previous and all(previous[key] == value for key, value in entry.items() if key != "steps"):
            previous["steps"] += 1
        else:
            self._trial_commands.append(entry)

    def _clearance(self) -> float | None:
        return self.config.wall_x - self._plant.front_x if self._plant.config.wall else None

    def _outcome(self) -> str:
        if self._plant.collision:
            return "collision"
        if self._stop_time is not None:
            return "stopped"
        return "timeout" if self.finished else "running"

    @property
    def goal_reached(self) -> bool:
        if self._plant.collision or self._stop_time is None:
            return False
        return self.goal_zone is None or (
            self.goal_zone["x_start"] <= self._plant.front_x <= self.goal_zone["x_end"])

    def observe(self) -> dict:
        plant = self._plant
        phase = ("impact_aftermath" if plant.collision else "stopped" if self._stop_time is not None
                 else "braking" if self._brake_start_time is not None else "approach")
        result = plant.observe(phase)
        result.update(
            platform="car_braking", trial_time=self.trial_time, speed=plant.speed,
            deceleration=self._last_deceleration,
            command={"throttle": plant.last_command[0], "brake": plant.last_command[1]},
            wall_clearance=self._clearance(), bumper_clearance=self._clearance(),
            barrier_enabled=plant.config.wall, impact_speed=plant.impact_speed,
            collision=plant.collision, censored=plant.collision or self._stop_time is None,
            approach_start_time=self.approach_start_time,
            goal_reached=self.goal_reached, task_complete=self.goal_reached,
        )
        return result

    def presentation(self) -> dict:
        """Task and conclusion for viewers, derived entirely from public motion."""
        observation = self.observe()
        objective = self.task_metadata["objective"]
        clearance = observation["bumper_clearance"]
        if observation["collision"]:
            status = "FAILED - BARRIER COLLISION"
            detail = (f"Impact: {observation['impact_speed']:.1f} m/s at {self._plant.collision_time:.2f} s\n"
                      "The car missed the stop target.")
        elif self._stop_time is not None:
            if clearance is None:
                status, detail = "STOPPED - OPEN TRACK", "Free stopping distance measured."
            elif observation["goal_reached"]:
                status, detail = "SUCCESS - STOPPED IN TARGET", f"Stopped with {clearance:.1f} m before the barrier."
            else:
                status, detail = "FAILED - STOPPED OUTSIDE TARGET", f"No contact, but the target was missed.\nBarrier clearance: {clearance:.1f} m."
        elif self.finished:
            status = "FAILED - TIME LIMIT"
            detail = f"Stopping task incomplete.\nRemaining speed: {observation['speed']:.1f} m/s."
        else:
            status = ("READY - STOP TARGET AHEAD" if self.trial_time == 0 else
                      "BRAKING - TARGET AHEAD" if observation["brake"] > 0 else "APPROACHING STOP TARGET")
            detail = (f"Speed: {observation['speed']:.1f} m/s\n"
                      f"Barrier clearance: {clearance:.1f} m") if clearance is not None else (
                          f"Speed: {observation['speed']:.1f} m/s\nMeasure the complete physical stop.")
            if clearance is None:
                status = "READY - OPEN TRACK" if self.trial_time == 0 else "BRAKING - OPEN TRACK"
        return {"objective": objective, "status": status, "detail": detail}

    def diagnostics(self) -> dict:
        return {
            **self._plant.diagnostics(), "events": self.public_events,
            "fault": self.config.fault, "initial_temperature": self._initial_temperature.tolist(),
            "trusted_plant": "simulator.runner.Simulator", "candidate_actuator": False,
        }

    def summary(self) -> dict:
        plant = self._plant
        stopped = self._stop_time is not None and not plant.collision
        public = {
            "platform": "car_braking", "scenario_id": "F4", "outcome": self._outcome(),
            "safe": stopped, "goal_reached": self.goal_reached, "task_complete": self.goal_reached,
            "collision": plant.collision, "impact_speed": plant.impact_speed,
            "collision_time": plant.collision_time, "stopped": stopped, "censored": not stopped,
            "stopping_distance": (plant.front_x - self._brake_start_x
                                  if stopped and self._brake_start_x is not None else None),
            "brake_start_x": self._brake_start_x, "brake_start_time": self._brake_start_time,
            "brake_start_speed": self._brake_start_speed, "final_front_x": plant.front_x,
            "final_speed": plant.speed, "wall_clearance": self._clearance(),
            "bumper_clearance": self._clearance(), "max_speed": self._max_speed,
            "distance_traveled": plant.front_x - self._initial_front_x,
            "trial_duration": self.trial_time, "approach_start_time": self.approach_start_time,
            "conditioning_duration": self.conditioning_duration,
            "aftermath_duration": (self.trial_time - plant.collision_time
                                   if plant.collision_time is not None else 0.0),
            "events": self.public_events, "task": self.task_metadata,
            "provenance": "developer_control" if self.config.fault == "healthy" or self.config.probe == "wall_free"
                          else "failure_demonstration",
            "repair_status": "not_run",
        }
        return {
            "public": public, "config": asdict(self.config), "events": self.public_events,
            "preparation_history": self.preparation_history,
            "initial_temperature": self._initial_temperature.tolist(),
            "final_temperature": plant.temperature.tolist(),
            "warnings": self.data.warning.number.tolist(), "mujoco_version": mujoco.__version__,
        }
