"""Host-only adequate-model control: fixed braking effectiveness."""


def init_state():
    return {}


def compute_force(state, brake_command, velocity):
    return 9000.0 * brake_command


def advance_state(state, brake_command, velocity, applied_braking_force_n, dt_s):
    return state
