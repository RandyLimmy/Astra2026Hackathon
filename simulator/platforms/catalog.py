"""Lazy registry: legacy car profiles remain independent of new platform code."""
from dataclasses import asdict, fields, replace
from importlib import import_module

MODULES = ("car_damage", "car_steering", "car_braking", "quadruped", "drone", "warehouse")

REPLAY_TASKS = {
    "quadruped_gait_failure": {
        "title": "Dog · a faster walk",
        "description": "Watch the feet lose coordination as the requested pace increases.",
        "objective": "Complete the requested speed transition while walking upright along the marked strip.",
    },
    "drone_delivery_imbalance": {
        "title": "Drone · an uneven load",
        "description": "Follow the loaded drone from takeoff through loss of balance and impact.",
        "objective": "Carry the parcel from A to B, place and release it, then return unloaded to A and land.",
    },
    "car_steering_drift": {
        "title": "Car · steering off course",
        "description": "Follow the marked bend as the car crosses the lane boundary.",
        "objective": "Follow the marked lane through the bend and cross the finish line.",
    },
    "car_auto_brake_failure": {
        "title": "Car · braking too late",
        "description": "Watch the automatic brake trigger engage before a positive-speed barrier collision.",
        "objective": "Approach the barrier at the declared speed and stop before contact.",
    },
}


def is_platform(name: str) -> bool:
    return name.startswith(("car_", "quadruped_", "drone_", "warehouse_"))


def entries():
    result = {}
    for name in MODULES:
        module = import_module(f"simulator.platforms.{name}")
        for preset in module.PRESETS:
            result[preset] = module.DESCRIPTIONS[preset]
    return result


def module_for(preset: str):
    if preset.startswith("car_auto_brake_"):
        prefix = "car_braking"
    elif preset in ("car_steering_drift", "car_steering_nominal"):
        prefix = "car_steering"
    else:
        prefix = "car_damage" if preset.startswith("car_") else preset.split("_", 1)[0]
    if prefix not in MODULES:
        raise ValueError(f"Unknown platform preset: {preset}")
    module = import_module(f"simulator.platforms.{prefix}")
    if preset not in module.PRESETS:
        raise ValueError(f"Unknown {prefix} preset {preset!r}; choose {', '.join(module.PRESETS)}")
    return module


def create(preset: str, overrides: dict | None = None, *, healthy: bool = False):
    module = module_for(preset)
    values = dict(module.PRESETS[preset])
    values.update(overrides or {})
    known = {field.name for field in fields(module.Config)}
    unknown = set(values) - known
    if unknown:
        raise ValueError(f"Unknown {preset} configuration fields: {', '.join(sorted(unknown))}")
    config = module.Config(**values)
    if healthy:
        config = replace(config, fault="healthy")
    return module.Simulation(config)


def config_dict(sim):
    return asdict(sim.config)
