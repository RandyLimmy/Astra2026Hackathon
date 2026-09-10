"""Trusted operator access to private, reproducible experiment presets."""
import json
from pathlib import Path

from .config import Experiment

DIRECTORY = Path(__file__).parent / "private" / "scenarios"


def names():
    return sorted(path.stem for path in DIRECTORY.glob("*.json"))


def load(name: str) -> Experiment:
    if name not in names():
        raise ValueError(f"Unknown scenario {name!r}. Available: {', '.join(names())}")
    manifest = json.loads((DIRECTORY / f"{name}.json").read_text())
    return Experiment.from_dict(manifest["config"])


def description(name: str) -> str:
    if name not in names():
        raise ValueError(f"Unknown scenario {name!r}")
    return json.loads((DIRECTORY / f"{name}.json").read_text())["description"]
