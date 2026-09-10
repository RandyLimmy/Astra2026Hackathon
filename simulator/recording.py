"""Simulation recordings with separate public sensor and private diagnostic exports."""

from __future__ import annotations

import json
import math
from pathlib import Path
import struct
from typing import Any, TextIO
import zlib

import mujoco
import numpy as np


def _json_default(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(f"Cannot serialize {type(value).__name__}")


def _json(value: Any) -> str:
    return json.dumps(value, default=_json_default, allow_nan=False, separators=(",", ":"))


def _write_png(path: Path, pixels: np.ndarray) -> None:
    """Write an RGB/RGBA uint8 image without an image-library dependency."""
    pixels = np.asarray(pixels)
    if pixels.dtype != np.uint8 or pixels.ndim != 3 or pixels.shape[2] not in (3, 4):
        raise ValueError("PNG pixels must be a uint8 array with three or four channels")
    height, width, channels = pixels.shape
    if not width or not height:
        raise ValueError("PNG dimensions must be positive")

    def chunk(kind: bytes, payload: bytes) -> bytes:
        return (struct.pack(">I", len(payload)) + kind + payload
                + struct.pack(">I", zlib.crc32(kind + payload) & 0xFFFFFFFF))

    # PNG filter 0 preserves the renderer's row order and channel values.
    scanlines = b"".join(b"\x00" + row.tobytes() for row in pixels)
    header = struct.pack(">IIBBBBB", width, height, 8, 2 if channels == 3 else 6, 0, 0, 0)
    with path.open("xb") as stream:
        stream.write(b"\x89PNG\r\n\x1a\n")
        stream.write(chunk(b"IHDR", header))
        stream.write(chunk(b"IDAT", zlib.compress(scanlines)))
        stream.write(chunk(b"IEND", b""))


class Recorder:
    """Sample caller-provided observations at 100 Hz and optional frames at ``fps``.

    Observations and public summaries must already contain only the agreed sensor
    fields. The recorder never copies diagnostic fields into a public artifact.
    Times must be nondecreasing across resets; the runner supplies experiment time
    in ``observation['time']``, independently of MuJoCo's per-trial ``data.time``.
    A gap emits one current sample, never fabricated intermediate observations.
    """

    def __init__(
        self,
        output_dir: Path,
        model: mujoco.MjModel,
        *,
        frames: bool = False,
        camera: str = "overview",
        fps: int = 30,
        width: int = 960,
        height: int = 540,
    ) -> None:
        for name, value in (("fps", fps), ("width", width), ("height", height)):
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        if not isinstance(camera, str) or not camera:
            raise ValueError("camera must be a nonempty model camera name")
        if frames and mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera) < 0:
            raise ValueError(f"Unknown model camera: {camera}")

        self.output_dir = Path(output_dir)
        self.public_dir = self.output_dir / "public"
        self.private_dir = self.output_dir / "private"
        self.public_dir.mkdir(parents=True, exist_ok=True)
        self.private_dir.mkdir(parents=True, exist_ok=True)
        self.camera = camera
        self.fps = fps
        self._renderer: mujoco.Renderer | None = None
        self._streams: list[TextIO] = []
        self._closed = False
        self._finished = False
        self._origin: float | None = None
        self._last_time: float | None = None
        self._telemetry_tick = 0
        self._frame_tick = 0
        self._frame_count = 0
        try:
            self._observations = self._open(self.public_dir / "observations.jsonl")
            self._diagnostics = self._open(self.private_dir / "diagnostics.jsonl")
            self._frames = self._open(self.public_dir / "frames.jsonl")
            if frames:
                (self.public_dir / "frames").mkdir(exist_ok=True)
                model.vis.global_.offwidth = max(width, model.vis.global_.offwidth)
                model.vis.global_.offheight = max(height, model.vis.global_.offheight)
                self._renderer = mujoco.Renderer(model, height=height, width=width)
        except BaseException:
            self.close()
            raise

    def _open(self, path: Path) -> TextIO:
        stream = path.open("x", encoding="utf-8")
        self._streams.append(stream)
        return stream

    def record(self, data: mujoco.MjData, observation: dict, private_state: dict) -> None:
        if self._closed:
            raise RuntimeError("Recorder is closed")
        time = float(observation["time"])
        if not math.isfinite(time) or time < 0:
            raise ValueError("Observation time must be finite and nonnegative")
        if self._last_time is not None and time < self._last_time:
            raise ValueError("Observation time must not move backward across trial resets")
        self._last_time = time
        if self._origin is None:
            self._origin = time
        elapsed = time - self._origin

        if elapsed + 1e-9 >= self._telemetry_tick / 100:
            diagnostics = {**private_state, "time": time}
            if "phase" in observation:
                diagnostics["phase"] = observation["phase"]
            # Serialize both before writing, so invalid values cannot split a pair.
            public_line, private_line = _json(observation), _json(diagnostics)
            self._observations.write(public_line + "\n")
            self._diagnostics.write(private_line + "\n")
            self._telemetry_tick = math.floor((elapsed + 1e-9) * 100) + 1

        if self._renderer is not None and elapsed + 1e-9 >= self._frame_tick / self.fps:
            self._renderer.update_scene(data, camera=self.camera)
            pixels = self._renderer.render()
            filename = f"frames/frame_{self._frame_count:06d}.png"
            _write_png(self.public_dir / filename, pixels)
            frame = {"time": time, "file": filename}
            if "phase" in observation:
                frame["phase"] = observation["phase"]
            self._frames.write(_json(frame) + "\n")
            self._frame_count += 1
            self._frame_tick = math.floor((elapsed + 1e-9) * self.fps) + 1

    def finish(self, summary: dict, public_summary: dict) -> None:
        """Write separately supplied summaries and release files/rendering resources."""
        if self._closed:
            raise RuntimeError("Recorder is closed")
        if self._finished:
            raise RuntimeError("Recorder is already finished")
        private_text, public_text = _json(summary), _json(public_summary)
        try:
            with (self.private_dir / "summary.json").open("x", encoding="utf-8") as stream:
                stream.write(private_text + "\n")
            with (self.public_dir / "summary.json").open("x", encoding="utf-8") as stream:
                stream.write(public_text + "\n")
            self._finished = True
        finally:
            self.close()

    def close(self) -> None:
        """Flush partial recordings too; safe to call repeatedly or during exceptions."""
        if self._closed:
            return
        self._closed = True
        try:
            for stream in self._streams:
                stream.close()
        finally:
            if self._renderer is not None:
                self._renderer.close()

    def __enter__(self) -> Recorder:
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()
