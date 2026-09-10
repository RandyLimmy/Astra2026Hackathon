"""Editable four-wheel component, protocol wheel_v2.

Wheel order: front-left, front-right, rear-left, rear-right.
Angular speeds are rad/s, braking torque capacities and applied torques are Nm.
"""


def init_state():
    return {}


def compute_brake_torque_limits(state, brake_command, wheel_speed_rad_s):
    return [capacity * brake_command for capacity in (835.0, 835.0, 557.0, 557.0)]


def advance_state(state, brake_command, mean_wheel_speed_rad_s, applied_brake_torque_nm, dt_s):
    return state


def on_trial_reset(state):
    return state
