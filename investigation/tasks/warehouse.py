"""Control the original warehouse bend without modifying its physical world.

Candidates change the route follower's numerical settings.  They cannot change
the route, trial duration, load, contact properties, or breakaway restraint.
"""
from __future__ import annotations

from copy import deepcopy
import inspect
import math
from typing import Any

import mujoco
import numpy as np

from simulator.platforms import warehouse


SCENARIO = "warehouse_curve_demo"
PLATFORM = "warehouse"
LABEL = "Warehouse trolley"
_BASE = {"schema_version": 1, "target_speed_m_s": 1.6,
         "acceleration_m_s2": .4, "lookahead_m": .65}
_BOUNDS = {"target_speed_m_s": (0., 2.4), "acceleration_m_s2": (.1, .8),
           "lookahead_m": (.35, 1.2)}
_ROUTE = np.asarray(warehouse.route_geometry()["centerline"])
_DISTANCES = np.linspace(0., warehouse.ROUTE_LENGTH, len(_ROUTE))
_END = np.asarray(warehouse.route_geometry()["end"])
_DURATION = float(warehouse.PRESETS[SCENARIO]["duration"])
# The revised fixture raises the deck top to .22 m in chassis coordinates.
_CARGO_HEIGHT_LIMITS = (.22, .84)


def controller(candidate: dict[str, Any], observation: dict[str, Any]) -> dict[str, float]:
    """Geometric pursuit using the fixed route and measured trolley state only."""
    time = float(observation["trial_time"])
    if time < .5 - 1e-10:
        return {"left": 0., "right": 0.}
    position = np.asarray(observation["position"][:2])
    nearest = int(np.argmin(np.linalg.norm(_ROUTE - position, axis=1)))
    progress = max(float(_DISTANCES[nearest]),
                   float(observation.get("route_progress", 0.)))
    vector = warehouse.route_point(progress + candidate["lookahead_m"]) - position
    heading = float(observation["heading"])
    lateral = -math.sin(heading) * vector[0] + math.cos(heading) * vector[1]
    curvature = 2 * lateral / max(.05, float(np.dot(vector, vector)))
    desired_speed = min(candidate["target_speed_m_s"],
                        candidate["acceleration_m_s2"] * max(0., time - .5),
                        math.sqrt(max(0., .5 * (warehouse.ROUTE_LENGTH - progress))))
    desired_yaw = desired_speed * curvature
    if progress >= warehouse.ROUTE_LENGTH - .05:
        desired_speed = 0.
        heading_error = math.atan2(math.sin(-math.pi / 2 - heading),
                                   math.cos(-math.pi / 2 - heading))
        desired_yaw = float(np.clip(heading_error, -.3, .3))
    velocity = observation["linear_velocity"]
    forward = velocity[0] * math.cos(heading) + velocity[1] * math.sin(heading)
    common = 2 * (desired_speed - forward) + (.025 if desired_speed > .03 else 0.)
    steering = 1.5 * (desired_yaw - observation["angular_velocity"][2])
    return {"left": float(np.clip(common - steering, -1., 1.)),
            "right": float(np.clip(common + steering, -1., 1.))}


def _cargo_position_on_deck(sim: warehouse.Simulation) -> np.ndarray:
    rotation = sim.data.xmat[sim.focus_body].reshape(3, 3)
    return rotation.T @ (sim.data.xpos[sim._cargo_body] - sim.data.xpos[sim.focus_body])


class _ControlledSimulation(warehouse.Simulation):
    def __init__(self, candidate: dict[str, Any]):
        self.controller_candidate = deepcopy(candidate)
        self.task_events: list[dict[str, Any]] = []
        self.max_route_error_m = 0.
        self.cargo_left_deck = False
        self._last_phase = "settling"
        self._floor_contact_seen = False
        self._route_completed_seen = False
        super().__init__(warehouse.Config(**warehouse.PRESETS[SCENARIO]))
        self._measure_task()

    def step(self) -> None:
        if self.finished:
            return
        super().step(controller(self.controller_candidate, super().observe()))
        self._measure_task()

    def _measure_task(self) -> None:
        observed = super().observe()
        route_error = float(np.min(np.linalg.norm(_ROUTE - np.asarray(observed["position"][:2]), axis=1)))
        self.max_route_error_m = max(self.max_route_error_m, route_error)
        cargo = _cargo_position_on_deck(self)
        # Public geometry: deck half extents .48/.31 m, with a 2 cm edge tolerance.
        aboard = (abs(cargo[0]) <= .50 and abs(cargo[1]) <= .33
                  and _CARGO_HEIGHT_LIMITS[0] <= cargo[2] <= _CARGO_HEIGHT_LIMITS[1])
        if not aboard and not self.cargo_left_deck:
            self.task_events.append({"time": self.elapsed, "event": "cargo_left_deck"})
        self.cargo_left_deck |= not aboard
        if observed["cargo_has_touched_floor"] and not self._floor_contact_seen:
            self.task_events.append({"time": self.elapsed, "event": "cargo_ground_impact"})
            self._floor_contact_seen = True
        if observed["phase"] != self._last_phase:
            self.task_events.append({"time": self.elapsed, "event": "phase_changed",
                                     "phase": observed["phase"]})
            self._last_phase = observed["phase"]
        if (observed["route_progress"] >= warehouse.ROUTE_LENGTH - .1
                and np.linalg.norm(np.asarray(observed["position"][:2]) - _END) <= .25
                and not self._route_completed_seen):
            self.task_events.append({"time": self.elapsed, "event": "route_end_reached"})
            self._route_completed_seen = True


class Adapter:
    platform = PLATFORM
    scenario = SCENARIO
    duration_s = _DURATION
    cameras = ("side", "overview", "chase")
    source_paths = ("investigation/tasks/warehouse.py", "simulator/platforms/warehouse.py",
                    "simulator/platforms/warehouse_tracks.py", "simulator/assets/warehouse.xml")

    def initial_candidate(self) -> dict[str, Any]:
        return deepcopy(_BASE)

    def validate_candidate(self, candidate: dict[str, Any]) -> dict[str, Any]:
        if not isinstance(candidate, dict) or set(candidate) != set(_BASE):
            raise ValueError(f"Controller requires exactly {sorted(_BASE)}")
        if type(candidate["schema_version"]) is not int or candidate["schema_version"] != 1:
            raise ValueError("schema_version must be the integer 1")
        validated = {"schema_version": 1}
        for name, (low, high) in _BOUNDS.items():
            value = candidate[name]
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or not low <= value <= high):
                raise ValueError(f"{name} must be a finite number in [{low}, {high}]")
            validated[name] = float(value)
        return validated

    def capabilities(self) -> dict[str, Any]:
        return {
            "platform": PLATFORM, "scenario": SCENARIO, "label": LABEL,
            "goal": "Complete the marked 90-degree route with the cargo aboard and no ground impact.",
            "duration_s": self.duration_s,
            "criteria": {
                "deadline_s": self.duration_s,
                "minimum_route_progress_m": warehouse.ROUTE_LENGTH - .1,
                "maximum_final_endpoint_error_m": .25,
                "maximum_final_heading_error_rad": .2,
                "maximum_route_error_m": .45,
                "maximum_vehicle_tilt_rad": .6,
                "cargo_center_must_remain_over_deck": True,
                "cargo_floor_contact_allowed": False,
                "cargo_must_rest_on_deck_at_finish": True,
                "full_trial_required": True,
                "finite_physics_without_warnings": True,
            },
            "candidate_fields": {
                name: {"minimum": low, "maximum": high, "initial": _BASE[name]}
                for name, (low, high) in _BOUNDS.items()
            },
            "candidate_schema": {"schema_version": 1, "requires_all_fields": True,
                                 "additional_fields_allowed": False},
            "controller_source": inspect.getsource(controller),
            "controller_context": {
                "route": {name: value for name, value in warehouse.route_geometry().items()
                          if name != "target_speed_m_s"},
                "deck_center_limits_in_chassis_m": {"x": [-.50, .50], "y": [-.33, .33], "z": list(_CARGO_HEIGHT_LIMITS)},
                "fixture": "Single parcel on a raised deck; visible tracks mark the route without physically guiding the trolley.",
                "command_units": "normalized wheel motor torque in [-1, 1]",
                "settling_duration_s": .5,
                "velocity_gain": 2., "yaw_gain": 1.5,
            },
            "observables": ["time", "trial_time", "phase", "position", "orientation", "heading",
                            "linear_velocity", "angular_velocity", "command", "wheel_speed",
                            "encoder_velocity", "accelerometer", "gyroscope", "cargo_position",
                            "cargo_orientation", "cargo_floor_contact", "cargo_has_touched_floor",
                            "cargo_linear_velocity", "cargo_angular_velocity", "cargo_tilt_rad",
                            "route_progress", "cargo_relative_position", "cargo_left_deck",
                            "route_error_m", "max_route_error_m"],
            "fixed_task": "The route, deadline, cargo, physical world, and restraint remain identical for every attempt.",
            "cameras": list(self.cameras),
        }

    def create_sim(self, candidate: dict[str, Any]) -> _ControlledSimulation:
        return _ControlledSimulation(self.validate_candidate(candidate))

    def observe(self, sim: _ControlledSimulation) -> dict[str, Any]:
        observed = sim.observe()
        cargo_velocity = np.zeros(6)
        mujoco.mj_objectVelocity(sim.model, sim.data, mujoco.mjtObj.mjOBJ_BODY,
                                sim._cargo_body, cargo_velocity, 0)
        observed.update(cargo_relative_position=_cargo_position_on_deck(sim).tolist(),
                        cargo_linear_velocity=cargo_velocity[3:].tolist(),
                        cargo_angular_velocity=cargo_velocity[:3].tolist(),
                        cargo_tilt_rad=math.acos(float(np.clip(sim.data.xmat[sim._cargo_body, 8], -1., 1.))),
                        cargo_left_deck=bool(sim.cargo_left_deck),
                        route_error_m=float(np.min(np.linalg.norm(
                            _ROUTE - np.asarray(observed["position"][:2]), axis=1))),
                        max_route_error_m=sim.max_route_error_m)
        return observed

    def outcome(self, sim: _ControlledSimulation) -> dict[str, Any]:
        summary = sim.summary()["public"]
        observed = sim.observe()
        endpoint_error = float(np.linalg.norm(np.asarray(observed["position"][:2]) - _END))
        heading_error = abs(math.atan2(math.sin(observed["heading"] + math.pi / 2),
                                      math.cos(observed["heading"] + math.pi / 2)))
        route_complete = (observed["route_progress"] >= warehouse.ROUTE_LENGTH - .1
                          and endpoint_error <= .25 and heading_error <= .2
                          and sim.max_route_error_m <= .45)
        cargo_retained = (not observed["cargo_has_touched_floor"] and not sim.cargo_left_deck
                          and bool(sim.diagnostics()["cargo_on_deck"]))
        achieved = bool(sim.finished and summary["safe"] and route_complete and cargo_retained)
        partial = bool(not achieved and sim.finished and summary["safe"]
                       and (route_complete or (cargo_retained and observed["route_progress"] >= warehouse.ROUTE_APPROACH)))
        summary["outcome"] = ("goal_achieved" if achieved else "unstable" if not summary["safe"]
                              else "cargo_spilled" if observed["cargo_has_touched_floor"]
                              else "cargo_left_deck" if sim.cargo_left_deck else "incomplete_route")
        summary["safe"] = bool(summary["safe"] and cargo_retained)
        summary["metrics"].update(route_complete=bool(route_complete), cargo_retained=bool(cargo_retained),
                                  final_endpoint_error_m=endpoint_error, final_heading_error_rad=heading_error,
                                  max_route_error_m=sim.max_route_error_m, full_trial_completed=bool(sim.finished))
        return {"goal_achieved": achieved, "partial_success": partial, "summary": summary}

    def public_events(self, sim: _ControlledSimulation) -> list[dict[str, Any]]:
        return deepcopy(sim.task_events)
