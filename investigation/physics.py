"""Trusted physics service; public results contain only the neutral contract."""

from dataclasses import dataclass, replace
import hashlib
import json
import math
from pathlib import Path
import uuid

import mujoco
import numpy as np

from component_worker import WheelActuatorWorker
from simulator.config import Experiment
from simulator.runner import Simulator


ROOT = Path(__file__).resolve().parents[1]
ORIGINAL_SOURCE = ROOT / "candidate" / "wheel_actuator.py"
OBSERVATION_LIMIT = 120
DEFAULT_CONFIG = {
    "speed_mps": 25.0, "brake_strength": 1.0, "preparation_cycles": 0,
    "wait_s": 0.0, "wall_distance_m": None,
}


def normalize_config(config: dict) -> dict:
    if not isinstance(config, dict) or set(config) - set(DEFAULT_CONFIG):
        raise ValueError("Configuration must contain only the five public control fields")
    result = {**DEFAULT_CONFIG, **config}
    for key, lower, upper in (("speed_mps", 5, 30), ("brake_strength", .2, 1),
                              ("wait_s", 0, 120), ("wall_distance_m", 10, 150)):
        value = result[key]
        if key == "wall_distance_m" and value is None:
            continue
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
            raise ValueError(f"{key} must be a finite number")
        if not lower <= value <= upper:
            raise ValueError(f"{key} must be between {lower} and {upper}")
        result[key] = float(value)
    count = result["preparation_cycles"]
    if type(count) is not int or not 0 <= count <= 5:
        raise ValueError("preparation_cycles must be an integer from 0 to 5")
    return result


def _encoded(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def _digest(value):
    return hashlib.sha256(_encoded(value)).hexdigest()


def _write_json(path: Path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
    try:
        temporary.write_bytes(_encoded(value) + b"\n")
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def _phase(value, *, public):
    if value == "trial":
        return value
    if value == ("recovery" if public else "wait"):
        return "wait" if public else "recovery"
    prefix = "conditioning_" if public else "preparation_"
    if isinstance(value, str) and value.startswith(prefix) and value[len(prefix):].isdigit():
        return ("preparation_" if public else "conditioning_") + value[len(prefix):]
    raise ValueError("Unknown preparation phase")


def _history(events, *, public):
    if not isinstance(events, list):
        raise ValueError("Preparation history must be a list")
    result = []
    for event in events:
        if not isinstance(event, dict):
            raise ValueError("Invalid preparation event")
        copied = dict(event)
        copied["phase"] = _phase(event.get("phase"), public=public)
        result.append(copied)
    return result


def _signature(row):
    return row["phase"], row["throttle"], row["brake"], row["wall_contact"]


class ObservationCollector:
    """Keep 100 Hz samples plus both sides of every phase/command transition."""

    def __init__(self):
        self.rows = []
        self.previous = None
        self.next_tick = 0.0

    def _append(self, row):
        if not self.rows or self.rows[-1] != row:
            self.rows.append(row)

    def __call__(self, simulator, phase):
        row = simulator.observe(phase)
        row["phase"] = _phase(phase, public=True)
        boundary = self.previous is not None and (
            _signature(row) != _signature(self.previous)
            or row["phase_time"] < self.previous["phase_time"]
        )
        if boundary:
            self._append(self.previous)
            self._append(row)
        if row["time"] + 1e-9 >= self.next_tick:
            self._append(row)
            self.next_tick = (math.floor((row["time"] + 1e-9) * 100) + 1) / 100
        self.previous = row

    def finish(self):
        if self.previous is not None:
            self._append(self.previous)
        return self.rows


def _round_observation(value):
    if isinstance(value, float):
        return round(value, 6)
    if isinstance(value, list):
        return [_round_observation(item) for item in value]
    if isinstance(value, dict):
        return {key: _round_observation(item) for key, item in value.items()}
    return value


def select_observations(rows, limit=OBSERVATION_LIMIT):
    """Cap tool payloads while retaining endpoints and every observable transition."""
    if len(rows) <= limit:
        return _round_observation(rows)
    keep = {0, len(rows) - 1}
    for index in range(1, len(rows)):
        if (_signature(rows[index]) != _signature(rows[index - 1])
                or rows[index]["phase_time"] < rows[index - 1]["phase_time"]):
            keep.update((index - 1, index))
    if len(keep) > limit:
        raise ValueError("Too many phase boundaries for the observation limit")
    remaining = [index for index in range(len(rows)) if index not in keep]
    count = limit - len(keep)
    if count:
        keep.update(remaining[int(position)] for position in np.linspace(0, len(remaining) - 1, count))
    return [_round_observation(rows[index]) for index in sorted(keep)]


@dataclass
class PreparedReference:
    config: dict
    simulator: Simulator
    collector: ObservationCollector
    history: list


class PhysicsService:
    def __init__(self, cache_dir: Path):
        self.cache_dir = Path(cache_dir)
        self.cache_dir.mkdir(parents=True, exist_ok=True)
        physics_files = [ROOT / "simulator" / name for name in
                         ("runner.py", "model.py", "config.py", "private/thermal.py")]
        physics_files.extend(sorted((ROOT / "simulator/assets").glob("*.xml")))
        physics_files.append(Path(__file__))
        self.physics_sha256 = _digest({
            "files": {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                      for path in physics_files},
            "engine_version": mujoco.__version__, "numeric_version": np.__version__,
        })
        self.development_configs = set()

    def _path(self, kind, config, **extra):
        key = _digest({"physics": self.physics_sha256, "config": config, **extra})
        return self.cache_dir / kind / f"{key}.json"

    @staticmethod
    def _read(path):
        return json.loads(path.read_text()) if path.exists() else None

    @staticmethod
    def _experiment(config, *, candidate):
        distance = config["wall_distance_m"]
        return Experiment(initial_speed=config["speed_mps"], brake_at=0,
                          brake=config["brake_strength"], duration=25, timestep=.002,
                          warmup_cycles=config["preparation_cycles"], recovery=config["wait_s"],
                          wall=distance is not None, wall_x=100 if distance is None else distance + 2,
                          thermal=not candidate)

    @staticmethod
    def _align_wall(simulator, config):
        distance = config["wall_distance_m"]
        if distance is not None:
            wall_x = simulator.front_x + distance
            simulator.config = replace(simulator.config, wall_x=wall_x)
            simulator.model.geom_pos[simulator.wall, 0] = wall_x + .5
            mujoco.mj_forward(simulator.model, simulator.data)

    @staticmethod
    def _public(record):
        result = {key: record[key] for key in ("config", "preparation_history", "summary")}
        result["observations"] = select_observations(record["observations"])
        if "source_sha256" in record:
            result["source_sha256"] = record["source_sha256"]
        return result

    def prepare_reference(self, config):
        """Trusted evaluation helper: preparation only, with no future probe."""
        config = normalize_config(config)
        simulator = Simulator(self._experiment(config, candidate=False))
        collector = ObservationCollector()
        simulator.prepare(collector)
        self._align_wall(simulator, config)
        return PreparedReference(config, simulator, collector,
                                 _history(simulator.preparation_history, public=True))

    def finish_reference(self, prepared):
        """Trusted reveal helper; callers must already have locked predictions."""
        summary = prepared.simulator.run(prepared.collector, prepared=True)["public"]
        record = {"config": prepared.config, "preparation_history": prepared.history,
                  "observations": prepared.collector.finish(), "summary": summary}
        _write_json(self._path("reference", prepared.config), record)
        return record

    def reference(self, config: dict) -> dict:
        config = normalize_config(config)
        self.development_configs.add(_digest(config))
        path = self._path("reference", config)
        record = self._read(path)
        if record is None:
            record = self.finish_reference(self.prepare_reference(config))
        return self._public(record)

    def _snapshot(self, source):
        content = Path(source).read_bytes()
        digest = hashlib.sha256(content).hexdigest()
        path = self.cache_dir / "sources" / f"{digest}.py"
        path.parent.mkdir(parents=True, exist_ok=True)
        if not path.exists():
            path.write_bytes(content)
        elif path.read_bytes() != content:
            raise RuntimeError("Candidate source cache failed its integrity check")
        return path, digest

    def model_record(self, config: dict, source: Path, history=None) -> dict:
        """Trusted full-resolution candidate run; this path never runs a reference."""
        config = normalize_config(config)
        replay = _history(history, public=False) if history is not None else None
        snapshot, digest = self._snapshot(source)
        path = self._path("model", config, source_sha256=digest, history=replay)
        cached = self._read(path)
        if cached is not None:
            return cached
        collector = ObservationCollector()
        with WheelActuatorWorker(snapshot) as worker:
            if worker.source_sha256 != digest:
                raise RuntimeError("Loaded candidate source does not match its snapshot")
            simulator = Simulator(self._experiment(config, candidate=True), actuator=worker)
            simulator.prepare(collector, history=replay)
            self._align_wall(simulator, config)
            summary = simulator.run(collector, prepared=True)["public"]
            record = {"config": config, "preparation_history": _history(simulator.preparation_history, public=True),
                      "observations": collector.finish(), "summary": summary,
                      "source_sha256": worker.source_sha256, "candidate_state": worker.inspect_state()}
        _write_json(path, record)
        return record

    def model(self, config: dict, source: Path, history=None) -> dict:
        return self._public(self.model_record(config, source, history))
