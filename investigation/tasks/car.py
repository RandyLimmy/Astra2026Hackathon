"""Bounded brake-controller repair on the existing conditioned four-wheel car.

The controller task fixes the approach speed at 22 m/s for both candidates. The
legacy 25 m/s failure replay is unchanged: at that speed even maximum braking
from the first step cannot avoid the wall on the conditioned physical plant.
The four preparation cycles, brake mechanics, barrier, target and deadline are
otherwise identical. Candidates can change only brake timing and pedal demand.
"""
from copy import deepcopy
import inspect
import math

import numpy as np

from simulator.platforms import car_braking


SCENARIO = "car_auto_brake_failure"
PLATFORM = "car"
LABEL = "Automatic braking"
TASK_CONFIG = car_braking.Config(initial_speed=22.0)
_BASELINE = {"brake_trigger_x_m": 45.0, "brake_command": 1.0}
_BOUNDS = {"brake_trigger_x_m": (2.0, 85.0), "brake_command": (0.1, 1.0)}


def controller(candidate: dict, observation: dict, memory: dict) -> dict:
    """Latch the brake pedal at a measured bumper position; use no private state."""
    if observation["front_x"] >= candidate["brake_trigger_x_m"]:
        memory["brake_latched"] = True
    return {"throttle": 0.0,
            "brake": candidate["brake_command"] if memory.get("brake_latched", False) else 0.0}


class CandidateSimulation(car_braking.Simulation):
    def __init__(self, candidate: dict):
        self.controller_candidate = deepcopy(candidate)
        self.controller_memory: dict = {}
        super().__init__(TASK_CONFIG)

    def _start_approach(self, reset_event: str, **values) -> None:
        self.controller_memory = {}
        super()._start_approach(reset_event, **values)

    def step(self) -> None:
        if not self.finished:
            super().step(controller(self.controller_candidate, self.observe(), self.controller_memory))


class Adapter:
    platform = PLATFORM
    scenario = SCENARIO
    duration_s = TASK_CONFIG.duration
    cameras = ("chase", "overview", "side")
    source_paths = (
        "investigation/tasks/car.py", "simulator/platforms/car_braking.py",
        "simulator/runner.py", "simulator/config.py", "simulator/model.py",
        "simulator/private/thermal.py",
        "simulator/assets/car.xml", "simulator/assets/track.xml",
        "simulator/public/CONTROL_TASKS.md",
    )

    def initial_candidate(self) -> dict:
        return deepcopy(_BASELINE)

    def validate_candidate(self, candidate: dict) -> dict:
        if not isinstance(candidate, dict) or set(candidate) != set(_BASELINE):
            raise ValueError(f"Controller requires exactly {sorted(_BASELINE)}")
        validated = {}
        for field, (low, high) in _BOUNDS.items():
            value = candidate[field]
            if (type(value) not in (int, float) or not math.isfinite(value)
                    or not low <= value <= high):
                raise ValueError(f"{field} must be a finite number in [{low}, {high}]")
            validated[field] = float(value)
        return validated

    def capabilities(self) -> dict:
        return {
            "platform": PLATFORM, "scenario": SCENARIO, "label": LABEL,
            "goal": "Stop the conditioned car in the green target before touching the barrier.",
            "duration_s": self.duration_s,
            "criteria": {
                "deadline_s": self.duration_s,
                "final_bumper_x_min_m": 86.0, "final_bumper_x_max_m": 98.0,
                "minimum_bumper_clearance_m": 2.0,
                "maximum_stopped_speed_m_s": 0.1,
                "minimum_stationary_duration_s": 1.0,
                "barrier_contact_allowed": False,
                "full_task_required": True,
                "finite_physics_without_warnings": True,
            },
            "candidate_fields": {
                field: {"minimum": low, "maximum": high, "initial": _BASELINE[field]}
                for field, (low, high) in _BOUNDS.items()
            },
            "candidate_schema": {"requires_all_fields": True, "additional_fields_allowed": False},
            "initial_candidate": self.initial_candidate(),
            "controller_source": inspect.getsource(controller),
            "controller_context": {
                "initial_speed_m_s": TASK_CONFIG.initial_speed,
                "wall_x_m": TASK_CONFIG.wall_x,
                "target_bumper_x_m": [86.0, 98.0],
                "command_units": "Normalized throttle and brake pedal in [0, 1]; this policy coasts until its brake trigger.",
                "memory": "Brake latch resets at each fresh approach.",
                "preparation": "Four physical acceleration/braking cycles to 25 m/s, followed by the declared 22 m/s approach reset; all candidates receive the same preparation.",
                "task_variant": "This controller task uses a fixed 22 m/s approach. The separate legacy failure replay retains its original 25 m/s approach, which cannot stop before the wall even with maximum braking from the start.",
            },
            "observables": ["time", "trial_time", "phase", "position", "velocity",
                            "speed", "deceleration", "front_x", "bumper_clearance",
                            "wall_clearance", "wheel_speed", "yaw", "yaw_rate",
                            "throttle", "brake", "command", "collision", "impact_speed",
                            "goal_reached", "task_complete"],
            "fixed_task": "Approach speed, preparation, physical brakes, road, barrier, target and deadline are fixed across every controller attempt. Stopping before the target is insufficient.",
            "cameras": list(self.cameras),
        }

    def create_sim(self, candidate: dict) -> CandidateSimulation:
        return CandidateSimulation(self.validate_candidate(candidate))

    def observe(self, sim: CandidateSimulation) -> dict:
        return sim.observe()

    def outcome(self, sim: CandidateSimulation) -> dict:
        summary = deepcopy(sim.summary()["public"])
        valid_physics = bool(np.isfinite(sim.data.qpos).all()
                             and np.isfinite(sim.data.qvel).all()
                             and not sim.data.warning.number.any())
        stationary = bool(sim._stationary_time >= 1.0 - 1e-8)
        achieved = bool(sim.finished and sim.goal_reached and stationary and valid_physics)
        safe_stop = bool(sim.finished and summary["stopped"] and valid_physics)
        summary.update(
            platform=PLATFORM,
            outcome="goal_achieved" if achieved else "stopped_outside_target" if safe_stop else summary["outcome"],
            safe=safe_stop, goal_reached=achieved, task_complete=achieved,
            provenance="candidate_feedback_control", repair_status="evaluated",
            initial_speed_m_s=TASK_CONFIG.initial_speed,
            events=self.public_events(sim),
        )
        summary["task"]["repair_status"] = "evaluated"
        summary["metrics"] = {
            key: summary[key] for key in (
                "collision", "impact_speed", "brake_start_x", "brake_start_time",
                "stopping_distance", "final_front_x", "final_speed", "bumper_clearance",
                "trial_duration", "conditioning_duration", "goal_reached",
            )
        }
        summary["metrics"].update(full_task_completed=bool(sim.finished),
                                  stationary_duration_s=sim._stationary_time,
                                  initial_speed_m_s=TASK_CONFIG.initial_speed)
        return {"goal_achieved": achieved, "partial_success": bool(safe_stop and not achieved),
                "summary": summary}

    def public_events(self, sim: CandidateSimulation) -> list:
        # Recordings start at the approach; use that same clock for replay events.
        return [{**event, "time": event["trial_time"]}
                for event in sim.public_events if event["time"] >= sim.approach_start_time]
