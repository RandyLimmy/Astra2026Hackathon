"""DEVELOPER-WRITTEN solvability check; this is NOT an agent-authored patch.

This independent approximate model remembers recent dissipated work and
reduces force with a rational response curve. It does not import the private
reference or receive its temperature. Its parameters are developer-selected
SYNTHETIC values, not fitted hardware measurements or a claimed discovery.

Run this source through the same isolated candidate worker as other candidate
components. Keep it out of the investigation agent's task package.
"""

import math


# SYNTHETIC developer-check parameters, intentionally approximate.
MAX_BRAKING_FORCE_N = 9000.0
MEMORY_RECOVERY_TIME_S = 160.0
RETAINED_WORK_FRACTION = 0.95
MEMORY_ONSET_J = 350000.0
RESPONSE_SCALE_J = 400000.0
MIN_EFFECTIVENESS = 0.37


def _finite(value, name):
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def init_state():
    """Initialize this model's own latent memory from a fresh specimen."""
    return {"recent_work_j": 0.0}


def compute_force(state, brake_command, velocity):
    """Map this model's memory to a bounded nonnegative brake magnitude."""
    memory = max(0.0, _finite(state["recent_work_j"], "recent_work_j"))
    command = min(1.0, max(0.0, _finite(brake_command, "brake_command")))
    speed = _finite(velocity, "velocity")
    if speed <= 0.0:
        return 0.0
    load = max(0.0, (memory - MEMORY_ONSET_J) / RESPONSE_SCALE_J)
    effectiveness = max(MIN_EFFECTIVENESS, 1.0 / (1.0 + load * load))
    return command * MAX_BRAKING_FORCE_N * effectiveness


def advance_state(state, brake_command, velocity, applied_braking_force_n, dt_s):
    """Update independent work memory from permitted history or own rollout.

Use the candidate's applied brake force and observed/candidate velocity, not
a private reference state. The exact constant-power decay also runs at rest.
The input state is never mutated.
"""
    memory = max(0.0, _finite(state["recent_work_j"], "recent_work_j"))
    _finite(brake_command, "brake_command")
    speed = max(0.0, _finite(velocity, "velocity"))
    force = _finite(applied_braking_force_n, "applied_braking_force_n")
    dt = _finite(dt_s, "dt_s")
    if force < 0.0 or dt < 0.0:
        raise ValueError("applied braking force and dt_s must be nonnegative")

    retained_power_w = RETAINED_WORK_FRACTION * force * speed
    decay_fraction = -math.expm1(-dt / MEMORY_RECOVERY_TIME_S)
    next_memory = (
        memory * (1.0 - decay_fraction)
        + retained_power_w * MEMORY_RECOVERY_TIME_S * decay_fraction
    )
    return {"recent_work_j": max(0.0, next_memory)}
