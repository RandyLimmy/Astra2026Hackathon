"""Editable nominal platform predictor. Empty parameters retain host defaults.

Only public observations and this component's numeric state are available.
See the supplied platform capabilities for supported parameter names and units.
"""


def init_state():
    return {}


def predict_parameters(state):
    return {}


def advance_state(state, observation, dt_s):
    return state
