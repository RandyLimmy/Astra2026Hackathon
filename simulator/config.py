"""Experiment input contract. All units are SI, temperatures in degrees C."""
from dataclasses import asdict, dataclass, fields
import math

WHEELS = ("FL", "FR", "RL", "RR")


@dataclass(frozen=True)
class Experiment:
    initial_speed: float = 25.0
    brake_at: float = 45.0
    brake: float = 1.0
    throttle: float = 0.0
    duration: float = 20.0
    wall: bool = True
    wall_x: float = 100.0
    timestep: float = 0.002
    warmup_cycles: int = 0
    recovery: float = 0.0
    thermal: bool = False
    detach_wheel: str | None = None
    detach_at: float = 2.2
    wet_friction: float = 1.1
    wet_start: float = 40.0
    wet_end: float = 120.0
    payload: float = 0.0
    weak_wheel: str | None = None
    brake_efficiency: float = 1.0
    weak_at: float = 0.0
    lag: float = 0.0
    # (time since measured trial start, throttle, brake). Overrides brake_at.
    commands: tuple[tuple[float, float, float], ...] = ()

    def __post_init__(self):
        for name in ("wall", "thermal"):
            if not isinstance(getattr(self, name), bool):
                raise ValueError(f"{name} must be a boolean")
        for field in fields(self):
            value = getattr(self, field.name)
            if isinstance(value, (int, float)) and not math.isfinite(value):
                raise ValueError(f"{field.name} must be finite")
        if not 0 <= self.initial_speed <= 35:
            raise ValueError("initial_speed must be between 0 and 35 m/s")
        if not 0 < self.duration <= 120 or not 0 < self.timestep <= 0.005:
            raise ValueError("duration must be (0,120], timestep (0,0.005]")
        if not isinstance(self.warmup_cycles, int) or not 0 <= self.warmup_cycles <= 12:
            raise ValueError("warmup_cycles must be an integer between 0 and 12")
        if not 0 <= self.recovery <= 600 or not 0 <= self.payload <= 600:
            raise ValueError("recovery and payload must be between 0 and 600")
        if not 0 <= self.brake <= 1 or not 0 <= self.throttle <= 1:
            raise ValueError("commands must be in [0,1]")
        if not 0 <= self.brake_efficiency <= 1 or not 0 <= self.lag <= 2:
            raise ValueError("brake_efficiency must be [0,1], lag [0,2]")
        if not 0.05 <= self.wet_friction <= 1.5 or not -400 < self.wet_start < self.wet_end < 900:
            raise ValueError("invalid wet road bounds or friction")
        if self.detach_at < 0 or self.weak_at < 0:
            raise ValueError("event times must be nonnegative")
        if not 0 <= self.brake_at <= 200 or not 10 <= self.wall_x <= 250:
            raise ValueError("brake_at must be [0,200], wall_x [10,250]")
        for wheel in (self.detach_wheel, self.weak_wheel):
            if wheel is not None and wheel not in WHEELS:
                raise ValueError(f"wheel must be one of {WHEELS}")
        previous = -1.0
        for command in self.commands:
            if len(command) != 3 or not all(math.isfinite(x) for x in command):
                raise ValueError("commands must contain finite (time, throttle, brake) triples")
            t, throttle, brake = command
            if t < 0 or t <= previous or not 0 <= throttle <= 1 or not 0 <= brake <= 1:
                raise ValueError("command times must increase and inputs must be [0,1]")
            previous = t

    def to_dict(self):
        return asdict(self)

    @classmethod
    def from_dict(cls, values):
        values = dict(values)
        if "commands" in values:
            values["commands"] = tuple(tuple(row) for row in values["commands"])
        return cls(**values)
