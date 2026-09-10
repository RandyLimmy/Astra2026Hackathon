"""Editable four-wheel component, protocol wheel_v2.

Wheel order: front-left, front-right, rear-left, rear-right.
Angular speeds are rad/s, braking torque capacities and applied torques are Nm.
"""


def init_state():
    return {"heat": [0.0, 0.0, 0.0, 0.0]}


def compute_brake_torque_limits(state, brake_command, wheel_speed_rad_s):
    import math
    limits = []
    for capacity, heat in zip((835.0, 835.0, 557.0, 557.0), state["heat"]):
        excess = max(0.0, (heat - 100000.0) / 90000.0)
        gain = 0.45 + 0.55 * math.exp(-excess * excess)
        limits.append(capacity * brake_command * gain)
    return limits


def advance_state(state, brake_command, mean_wheel_speed_rad_s, applied_brake_torque_nm, dt_s):
    import math
    cooling = math.exp(-dt_s / 90.0)
    interval = -90.0 * math.expm1(-dt_s / 90.0)
    heat = []
    for old, omega, torque in zip(state["heat"], mean_wheel_speed_rad_s, applied_brake_torque_nm):
        power = max(0.0, -torque * omega)
        heat.append(min(1e12, max(0.0, old * cooling + power * interval)))
    return {"heat": heat}


def on_trial_reset(state):
    return state
