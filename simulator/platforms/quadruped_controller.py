"""Editable gait timing parameters for the articulated walking task.

Parameters affect foot/stance commands only. Body mechanics and task requests
remain owned by the simulation.
"""
from dataclasses import dataclass
import math

import numpy as np

DEFAULT_PARAMETERS = {"rear_cadence_gain": 4.0}


def validate_parameters(values: dict | None) -> dict[str, float]:
    """Return a copied complete configuration; this interface cannot alter physics."""
    if values is not None and (not isinstance(values, dict) or set(values) - set(DEFAULT_PARAMETERS)):
        raise ValueError("Unknown gait controller parameters")
    canonical = {**DEFAULT_PARAMETERS, **(values or {})}
    GaitParameters(**canonical)
    return {name: float(value) for name, value in canonical.items()}


@dataclass(frozen=True)
class GaitParameters:
    rear_cadence_gain: float = 4.0

    def __post_init__(self) -> None:
        value = self.rear_cadence_gain
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
            raise ValueError("rear_cadence_gain must be finite")
        if not 0 <= value <= 4:
            raise ValueError("rear_cadence_gain must be between 0 and 4")


class GaitController:
    """Continuous foot clocks with bounded torque-producing target trajectories."""
    def __init__(self, parameters: GaitParameters):
        self.parameters = parameters
        self.phase = np.array((0.0, -0.5, -0.75, -0.25))
        self.front_phase = 0.0
        self.swinging = np.zeros(4, dtype=bool)
        self.starts = np.zeros((4, 3))
        self.ends = np.zeros((4, 3))

    def targets(self, *, dt, speed, base_speed, period, rotation, target_xy, anchors, hip_offsets):
        response = max(0.0, speed / max(0.02, base_speed) - 1)
        cadence = 1 + 0.5 * response
        rates = np.array((cadence, cadence, 1 + 0.5 * self.parameters.rear_cadence_gain * response,
                          1 + 0.5 * self.parameters.rear_cadence_gain * response)) / period
        self.phase += rates * dt
        self.front_phase += cadence * dt / period
        cycle = self.phase % 1
        targets = anchors.copy()
        velocities = np.zeros((4, 3))
        stance = np.ones(4, dtype=bool)
        start, duration = 0.075, 0.13
        for leg in range(4):
            swinging = start < cycle[leg] < start + duration
            if swinging and not self.swinging[leg]:
                self.starts[leg] = anchors[leg]
                local = hip_offsets[leg].copy()
                local[1] += 0.045 if leg % 2 == 0 else -0.045
                local[0] += speed * period / cadence * 0.345
                self.ends[leg] = rotation @ local + np.array((*target_xy, 0))
                self.ends[leg, 2] = 0.034
            if swinging:
                stance[leg] = False
                u = (cycle[leg] - start) / duration
                blend = u**3 * (10 - 15*u + 6*u*u)
                rate = 30*u*u*(1-u)*(1-u) * rates[leg] / duration
                targets[leg] = self.starts[leg] * (1-blend) + self.ends[leg] * blend
                targets[leg, 2] += 0.055 * 16*u*u*(1-u)*(1-u)
                velocities[leg] = (self.ends[leg] - self.starts[leg]) * rate
                velocities[leg, 2] += 0.055 * 32*u*(1-u)*(1-2*u) * rates[leg] / duration
            elif self.swinging[leg]:
                anchors[leg] = self.ends[leg]
                targets[leg] = anchors[leg]
            self.swinging[leg] = swinging
        slot = int(self.front_phase * 4)
        phase = (self.front_phase * 4) % 1
        order = (0, 3, 1, 2)
        signs = np.array(((1, 1), (1, -1), (-1, 1), (-1, -1)))
        shift = -signs[order[slot % 4]] * np.array((0.0, 0.06))
        previous = -signs[order[(slot-1) % 4]] * np.array((0.0, 0.06))
        u = min(1.0, phase / 0.30)
        blend = u**3 * (10 - 15*u + 6*u*u)
        com_shift = rotation[:2, :2] @ (previous * (1-blend) + shift * blend)
        return targets, velocities, stance, com_shift
