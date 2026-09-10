"""SYNTHETIC thermal-brake reference, for host use only.

These parameters describe a toy benchmark, not measured vehicle behavior.
The component intentionally contains the mechanism omitted from the initial
candidate. Keep its source, parameters, and state outside the agent workspace.

For the planned 1200 kg body at 20 m/s, one stop dissipates about 240 kJ.
Without cooling that raises this component by 53.3 K, leaving a fresh stop
below the 100 C fade onset. Repeated stops accumulate heat; rest removes it.
"""

import math


# SYNTHETIC benchmark constants; no external measurements are implied.
MAX_BRAKING_FORCE_N = 9000.0
AMBIENT_TEMPERATURE_C = 20.0
HEAT_CAPACITY_J_PER_K = 4500.0
COOLING_CONDUCTANCE_W_PER_K = 30.0
HEATING_FRACTION = 1.0
FADE_ONSET_C = 100.0
FULL_FADE_C = 220.0
MIN_EFFECTIVENESS = 0.35


def _finite(value, name):
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def init_state():
    """Start a fresh synthetic specimen at ambient temperature."""
    return {"temperature_c": AMBIENT_TEMPERATURE_C}


def compute_force(state, brake_command, velocity):
    """Return a nonnegative brake-force magnitude; the bridge supplies sign."""
    temperature = _finite(state["temperature_c"], "temperature_c")
    command = min(1.0, max(0.0, _finite(brake_command, "brake_command")))
    speed = _finite(velocity, "velocity")
    if speed <= 0.0:
        return 0.0
    fraction = min(
        1.0, max(0.0, (temperature - FADE_ONSET_C) / (FULL_FADE_C - FADE_ONSET_C))
    )
    # Smoothstep gives an exactly flat cold plateau and bounded faded force.
    transition = fraction * fraction * (3.0 - 2.0 * fraction)
    effectiveness = 1.0 - (1.0 - MIN_EFFECTIVENESS) * transition
    return command * MAX_BRAKING_FORCE_N * effectiveness


def advance_state(state, brake_command, velocity, applied_braking_force_n, dt_s):
    """Advance heat using actual applied brake power, including cooling at rest.

The update is exact for power held constant over the supplied timestep.
Passing interval-average velocity improves dissipated-work accuracy. The
bridge must call this once per physical timestep, including stationary waits.
The input state is never mutated.
"""
    temperature = _finite(state["temperature_c"], "temperature_c")
    _finite(brake_command, "brake_command")
    speed = max(0.0, _finite(velocity, "velocity"))
    force = _finite(applied_braking_force_n, "applied_braking_force_n")
    dt = _finite(dt_s, "dt_s")
    if force < 0.0 or dt < 0.0:
        raise ValueError("applied braking force and dt_s must be nonnegative")

    power_w = HEATING_FRACTION * force * speed
    cooling_fraction = -math.expm1(
        -COOLING_CONDUCTANCE_W_PER_K * dt / HEAT_CAPACITY_J_PER_K
    )
    excess_temperature = temperature - AMBIENT_TEMPERATURE_C
    next_temperature = AMBIENT_TEMPERATURE_C + (
        excess_temperature * (1.0 - cooling_fraction)
        + power_w / COOLING_CONDUCTANCE_W_PER_K * cooling_fraction
    )
    return {"temperature_c": max(AMBIENT_TEMPERATURE_C, next_temperature)}
