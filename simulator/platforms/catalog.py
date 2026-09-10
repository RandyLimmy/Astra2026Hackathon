"""Lazy registry: legacy car profiles remain independent of new platform code."""
from dataclasses import asdict, fields, replace
from importlib import import_module

MODULES = ("car_damage", "quadruped", "drone", "warehouse")


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
