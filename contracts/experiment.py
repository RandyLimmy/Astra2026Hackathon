"""Public numeric configuration; reference implementation settings stay private."""

from dataclasses import asdict, dataclass
import math


@dataclass(frozen=True)
class ExperimentConfig:
    speed_mps: float = 20.0
    brake_strength: float = 1.0
    wall_distance_m: float = 40.0
    preparation_cycles: int = 0
    rest_time_s: float = 0.0
    cycle_pause_s: float = 2.0
    timestep_s: float = 0.01
    probe_horizon_s: float = 30.0

    def __post_init__(self):
        bounds = {
            "speed_mps": (1.0, 40.0),
            "brake_strength": (0.0, 1.0),
            "wall_distance_m": (1.0, 500.0),
            "rest_time_s": (0.0, 600.0),
            "cycle_pause_s": (0.0, 30.0),
            "timestep_s": (0.001, 0.02),
            "probe_horizon_s": (1.0, 60.0),
        }
        for field, (low, high) in bounds.items():
            value = getattr(self, field)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{field} must be a number")
            if not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f"{field} must be between {low} and {high}")
        if type(self.preparation_cycles) is not int or not 0 <= self.preparation_cycles <= 8:
            raise ValueError("preparation_cycles must be an integer from 0 to 8")

    def to_dict(self):
        return asdict(self)
