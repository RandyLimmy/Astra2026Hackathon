"""Fit an unchanged-structure baseline using development observations only.

The fitting objective uses analytic constant-force motion at the known
1200 kg mass. Scored predictions must execute the emitted Python component
through the isolated worker and mechanical runner, not this objective.
"""

import math

import numpy as np
from scipy.optimize import least_squares


MASS_KG = 1200.0
FORCE_BOUNDS_N = (1000.0, 15000.0)


def fit_fixed_force(reference_runs: list[dict]) -> dict:
    """Fit one shared force coefficient to all development speed samples.

Probe timestamps are seconds from the probe start. Every sample has equal
weight; longer recordings therefore contribute more observations. The caller
must supply development data, never final-suite outcomes.
"""
    if not reference_runs:
        raise ValueError("At least one development run is required")
    records = []
    informative = False
    for run in reference_runs:
        config = run["config"]
        speed = float(config["speed_mps"])
        brake = float(config["brake_strength"])
        times = np.asarray([row["t_s"] for row in run["probe"]], dtype=float)
        observed = np.asarray([row["v_mps"] for row in run["probe"]], dtype=float)
        if not (math.isfinite(speed) and speed >= 0 and math.isfinite(brake) and 0 <= brake <= 1):
            raise ValueError("Development speed/brake configuration is invalid")
        if (times.size < 2 or not np.all(np.isfinite(times))
                or not np.all(np.isfinite(observed)) or np.any(times < 0)
                or np.any(np.diff(times) <= 0) or np.any(observed < 0)):
            raise ValueError("Probe samples must have finite, increasing times and nonnegative speeds")
        informative |= brake > 0 and bool(np.any((times > 0) & (observed > 0)))
        records.append((speed, brake, times, observed))
    if not informative:
        raise ValueError("Need an observed moving interval with a positive brake command")

    def residual(parameters):
        force = parameters[0]
        return np.concatenate([
            np.maximum(0.0, speed - brake * force / MASS_KG * times) - observed
            for speed, brake, times, observed in records
        ])

    # Start at the lower bound: a large initial force can clip every later
    # speed to zero, giving a flat objective even when sparse data is useful.
    result = least_squares(residual, x0=[FORCE_BOUNDS_N[0]], bounds=FORCE_BOUNDS_N)
    return {
        "force_n": float(result.x[0]),
        "optimizer_success": bool(result.success),
        "fitting_split": "development",
        "speed_rmse_mps": float(np.sqrt(np.mean(result.fun ** 2))),
    }


def make_fixed_force_source(force_n: float) -> str:
    """Emit the three-function candidate API without state or host imports."""
    force = float(force_n)
    if not math.isfinite(force) or not FORCE_BOUNDS_N[0] <= force <= FORCE_BOUNDS_N[1]:
        raise ValueError("force_n must be finite and within the fitting bounds")
    return f'''"""Parameter-fitted development baseline; unchanged model structure."""

FORCE_N = {force!r}


def init_state():
    return {{}}


def compute_force(state, brake_command, velocity):
    command = min(1.0, max(0.0, float(brake_command)))
    return FORCE_N * command if velocity > 0.0 else 0.0


def advance_state(state, brake_command, velocity, applied_braking_force_n, dt_s):
    return {{}}
'''
