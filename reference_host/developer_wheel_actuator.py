"""DEVELOPER-WRITTEN wheel-actuator solvability check, NOT an agent repair.

This builder-only approximation was calibrated with knowledge of the synthetic
reference. It demonstrates that persistent Python component state can explain
the reference's history dependence when coupled to the richer MuJoCo car. It is
not evidence of blind discovery, research novelty, or a hardware-calibrated
brake model. Keep this source outside the investigation agent's task package.

Each wheel remembers recent dissipated work in joules. A piecewise-linear
capacity curve and exponential recovery approximate the synthetic reference;
the component neither imports private code nor reads a reference temperature.
Run this source through the same isolated worker as editable candidates.
"""

import math


# FL, FR, RL, RR. Synthetic calibration, expressed as a work-memory model.
NOMINAL_TORQUE_LIMITS_NM = (835.0, 835.0, 557.0, 557.0)
MEMORY_RECOVERY_TIME_S = 92.0
WORK_RESPONSE = (
    (104000.0, 1.0),
    (143000.0, 0.915),
    (182000.0, 0.725),
    (221000.0, 0.535),
    (260000.0, 0.45),
)


def _finite(value, name):
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _four(values, name):
    result = [_finite(value, name) for value in values]
    if len(result) != 4:
        raise ValueError(f"{name} must contain four values in FL, FR, RL, RR order")
    return result


def _memory(state):
    memory = _four(state["recent_work_j"], "recent_work_j")
    if any(value < 0 for value in memory):
        raise ValueError("recent_work_j must be nonnegative")
    return memory


def _command(value):
    command = _finite(value, "brake_command")
    if not 0 <= command <= 1:
        raise ValueError("brake_command must be between zero and one")
    return command


def _effectiveness(recent_work_j):
    if recent_work_j <= WORK_RESPONSE[0][0]:
        return WORK_RESPONSE[0][1]
    for (left_work, left_gain), (right_work, right_gain) in zip(
        WORK_RESPONSE, WORK_RESPONSE[1:]
    ):
        if recent_work_j <= right_work:
            fraction = (recent_work_j - left_work) / (right_work - left_work)
            return left_gain + fraction * (right_gain - left_gain)
    return WORK_RESPONSE[-1][1]


def init_state():
    """A fresh specimen starts with independent, zeroed work memories."""
    return {"recent_work_j": [0.0, 0.0, 0.0, 0.0]}


def compute_brake_torque_limits(state, brake_command, wheel_speed_rad_s4):
    """Return nonnegative brake capacities; the host determines torque signs.

    Capacity is retained at zero speed so the mechanical brake constraint can
    hold a stationary wheel. Merely querying the component never updates state.
    """
    memory = _memory(state)
    command = _command(brake_command)
    _four(wheel_speed_rad_s4, "wheel_speed_rad_s4")
    return [command * nominal * _effectiveness(work)
            for nominal, work in zip(NOMINAL_TORQUE_LIMITS_NM, memory)]


def advance_state(state, brake_command, mean_wheel_speed_rad_s4,
                  applied_brake_torque_nm4, dt_s):
    """Evolve work memory once per supplied mechanical integration interval.

    Use actual signed brake torque and mean wheel angular speed over the step,
    not the requested torque capacity. Dissipation is positive when those signs
    oppose; drive torque or a stationary wheel adds no work. Rest intervals decay
    memory even with a zero pedal command. The input state is never mutated.
    """
    memory = _memory(state)
    _command(brake_command)
    speeds = _four(mean_wheel_speed_rad_s4, "mean_wheel_speed_rad_s4")
    torques = _four(applied_brake_torque_nm4, "applied_brake_torque_nm4")
    dt = _finite(dt_s, "dt_s")
    if dt < 0:
        raise ValueError("dt_s must be nonnegative")
    lost_fraction = -math.expm1(-dt / MEMORY_RECOVERY_TIME_S)
    result = []
    for work, speed, torque in zip(memory, speeds, torques):
        power = max(0.0, -torque * speed)
        next_work = work * (1.0 - lost_fraction) + (
            power * MEMORY_RECOVERY_TIME_S * lost_fraction
        )
        result.append(_finite(next_work, "next recent_work_j"))
    return {"recent_work_j": result}


def on_trial_reset(state):
    """Repositioning is an intervention on motion, not a fresh brake specimen."""
    return {"recent_work_j": _memory(state)}
