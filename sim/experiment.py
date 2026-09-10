"""Development experiments and predictions; no final holdout claims yet."""

from dataclasses import dataclass
from types import ModuleType
from .bridge import DRIVE_FORCE_N, MASS_KG, Mechanics, STOP_SPEED_MPS


class ReferenceActuator:
    """Trusted host reference only. Never load candidate paths in this class."""

    def __init__(self, module: ModuleType):
        self.module = module
        self.reset()

    def reset(self):
        self.state = self.module.init_state()

    def force(self, brake, velocity):
        return self.module.compute_force(self.state, brake, velocity)

    def advance(self, brake, velocity, applied_force, dt):
        self.state = self.module.advance_state(self.state, brake, velocity, applied_force, dt)


@dataclass
class RunResult:
    config: dict
    history: list
    probe: list
    summary: dict

    def to_dict(self):
        return {"schema_version": 1, "config": self.config,
                "history": self.history, "probe": self.probe, "summary": self.summary}


def _interval(mechanics, actuator, brake, drive, dt, phase):
    row, applied = mechanics.advance(drive, actuator.force(brake, mechanics.speed), dt)
    actuator.advance(brake, (row["v_mps"] + row["next_v_mps"]) / 2, applied, dt)
    row.update(brake=brake, phase=phase)
    return row


def prepare(config, actuator):
    """Reset both systems; record all drive/brake/wait history before probe."""
    actuator.reset()
    mechanics = Mechanics(config.timestep_s)
    history = []

    def drive():
        while config.speed_mps - mechanics.speed > 1e-8:
            remaining = (config.speed_mps - mechanics.speed) * MASS_KG / DRIVE_FORCE_N
            dt = min(config.timestep_s, remaining)
            history.append(_interval(mechanics, actuator, 0.0, DRIVE_FORCE_N, dt, "drive"))

    def wait(duration):
        target = mechanics.time + duration
        while target - mechanics.time > 1e-8:
            history.append(_interval(mechanics, actuator, 0.0, 0.0,
                                     min(config.timestep_s, target - mechanics.time), "wait"))

    for _ in range(config.preparation_cycles):
        drive()
        start = mechanics.time
        while mechanics.speed > STOP_SPEED_MPS:
            if mechanics.time - start > 90:
                raise RuntimeError("Preparation did not stop within its bounded horizon")
            history.append(_interval(mechanics, actuator, 1.0, 0.0, config.timestep_s, "brake"))
        wait(config.cycle_pause_s)
    wait(config.rest_time_s)
    drive()
    return mechanics, history


def probe(config, mechanics, actuator, history):
    origin_x, origin_t = mechanics.position, mechanics.time
    samples = [{"t_s": 0.0, "x_m": 0.0, "v_mps": mechanics.speed}]
    crossed_at = None
    impact_speed = None
    while mechanics.speed > STOP_SPEED_MPS:
        elapsed = mechanics.time - origin_t
        if config.probe_horizon_s - elapsed < 1e-8:
            break
        dt = min(config.timestep_s, config.probe_horizon_s - elapsed)
        _interval(mechanics, actuator, config.brake_strength, 0.0, dt, "probe")
        row = {"t_s": mechanics.time - origin_t,
               "x_m": mechanics.position - origin_x, "v_mps": mechanics.speed}
        previous = samples[-1]
        if crossed_at is None and row["x_m"] >= config.wall_distance_m:
            fraction = (config.wall_distance_m - previous["x_m"]) / (row["x_m"] - previous["x_m"])
            crossed_at = previous["t_s"] + fraction * (row["t_s"] - previous["t_s"])
            impact_speed = previous["v_mps"] + fraction * (row["v_mps"] - previous["v_mps"])
        samples.append(row)
    stopped = mechanics.speed <= STOP_SPEED_MPS
    summary = {
        "status": "stopped" if stopped else "not_stopped",
        "stopping_distance_m": samples[-1]["x_m"] if stopped else None,
        "stopping_time_s": samples[-1]["t_s"] if stopped else None,
        "distance_at_horizon_m": samples[-1]["x_m"],
        "wall_crossed": crossed_at is not None,
        "wall_crossing_time_s": crossed_at,
        "wall_crossing_speed_mps": impact_speed,
    }
    return RunResult(config.to_dict(), history, samples, summary)


def run_reference(config, *, fade=True):
    from reference_host import actuator as thermal
    from reference_host import normal_actuator as normal
    actuator = ReferenceActuator(thermal if fade else normal)
    mechanics, history = prepare(config, actuator)
    return probe(config, mechanics, actuator, history)


def predict_from_history(config, history, worker):
    """Use measured preparation, then roll out candidate without future truth.

    Braking force during preparation is derived from measured kinematics and
    known drive force/mass. No reference parameter or latent state is read.
    """
    worker.reset()
    for row in history:
        dt = row["dt_s"]
        observed_brake_force = max(0.0, row["drive_force_n"] - MASS_KG * (row["next_v_mps"] - row["v_mps"]) / dt)
        worker.advance(row["brake"], (row["v_mps"] + row["next_v_mps"]) / 2,
                       observed_brake_force, dt)
    mechanics = Mechanics(config.timestep_s)
    mechanics.data.qvel[0] = config.speed_mps
    return probe(config, mechanics, worker, history)


def stopping_error(candidate, reference):
    a, b = candidate.summary["stopping_distance_m"], reference.summary["stopping_distance_m"]
    return abs(a - b) if a is not None and b is not None else None
