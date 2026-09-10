"""Synthetic brake-disc energy balance; coefficients are not hardware calibrated."""
import numpy as np

AMBIENT = 20.0
HEAT_CAPACITY = 650.0  # J / K per brake
COOLING_TIME = 90.0  # s


def fade(temperature):
    # Smooth transition from full capacity at 180 C to 45% at 420 C.
    fraction = np.clip((temperature - 180.0) / 240.0, 0.0, 1.0)
    smooth = fraction * fraction * (3.0 - 2.0 * fraction)
    return 1.0 - 0.55 * smooth


def advance(temperature, dissipated_power, dt):
    decay = np.exp(-dt / COOLING_TIME)
    # Exact update for power held constant during a physics step.
    return AMBIENT + (temperature - AMBIENT) * decay + (
        dissipated_power / HEAT_CAPACITY * COOLING_TIME * (1.0 - decay)
    )
