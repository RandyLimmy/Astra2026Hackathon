"""Editable component. Inputs use seconds, metres, and newtons.

The component returns a positive braking-force magnitude. The runner applies
its direction and calls advance_state once for every integration interval.
"""


def init_state():
    return {}


def compute_force(state, brake_command, velocity):
    return 9000.0 * brake_command


def advance_state(state, brake_command, velocity, applied_braking_force_n, dt_s):
    return state
