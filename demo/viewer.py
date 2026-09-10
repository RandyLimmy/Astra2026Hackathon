"""Native MuJoCo replay of the actual candidate and reference probe samples.

This scene is a presentation of saved trajectories, not a second physics engine.
It does not repair a model or attribute a developer baseline to an agent. Run the
interactive viewer with ``.venv/bin/mjpython`` on macOS.
"""

from __future__ import annotations

import math
from pathlib import Path
from queue import Empty, SimpleQueue
import time
from xml.sax.saxutils import escape

import mujoco
import numpy as np

from simulator.view_controls import ViewControls


_LANES = (("candidate", -3.0, "0.18 0.49 0.95 1"),
          ("reference", 3.0, "0.96 0.36 0.26 1"))


def _samples(run: dict) -> np.ndarray:
    try:
        rows = np.asarray([
            [sample["t_s"], sample["x_m"], sample["v_mps"]]
            for sample in run["probe"]
        ], dtype=float)
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("Run must contain probe samples with t_s, x_m and v_mps.") from exc
    if rows.ndim != 2 or rows.shape[0] < 1 or rows.shape[1] != 3:
        raise ValueError("A replay needs at least one probe sample.")
    if not np.isfinite(rows).all():
        raise ValueError("Replay samples must be finite.")
    if rows[0, 0] != 0 or np.any(np.diff(rows[:, 0]) <= 0):
        raise ValueError("Probe timestamps must start at zero and increase strictly.")
    return rows


def _scene(candidate: np.ndarray, reference: np.ndarray, wall_distance_m: float,
           title: str) -> tuple[mujoco.MjModel, mujoco.MjData, mujoco.MjvCamera]:
    if not math.isfinite(wall_distance_m) or wall_distance_m <= 0:
        raise ValueError("wall_distance_m must be finite and positive.")
    left = min(-3.0, float(candidate[:, 1].min()) - 3, float(reference[:, 1].min()) - 3)
    right = max(wall_distance_m + 4, float(candidate[:, 1].max()) + 4,
                float(reference[:, 1].max()) + 4, 18.0)
    span = right - left
    center = (left + right) / 2
    parts = [f"""<mujoco model="{escape(title, {'"': '&quot;'})}">
      <compiler angle="degree"/>
      <option gravity="0 0 0"/>
      <visual>
        <global offwidth="1440" offheight="900"/>
        <headlight ambient="0.45 0.45 0.48" diffuse="0.7 0.7 0.7" specular="0.3 0.3 0.3"/>
        <rgba haze="0.08 0.11 0.15 1"/>
        <quality shadowsize="2048"/>
      </visual>
      <asset>
        <texture type="skybox" builtin="gradient" rgb1="0.08 0.11 0.15"
                 rgb2="0.18 0.22 0.29" width="512" height="3072"/>
      </asset>
      <default><geom contype="0" conaffinity="0"/></default>
      <worldbody>
        <light pos="{center} -10 25" dir="0 0 -1" diffuse="0.8 0.8 0.8"/>
        <geom type="plane" pos="{center} 0 0" size="{span * 3} {span * 3} 0.1"
              rgba="0.10 0.14 0.19 1"/>
        <geom name="barrier" type="box" pos="{wall_distance_m} 0 0.65"
              size="0.16 5.1 0.65" rgba="0.98 0.72 0.22 0.6"/>
        <geom type="box" pos="0 0 0.018" size="0.065 5.1 0.018" rgba="0.72 0.8 0.86 1"/>
    """]
    marker_step = max(5, math.ceil(span / 14 / 5) * 5)
    for distance in range(marker_step, math.ceil(right), marker_step):
        parts.append(f'<geom type="box" pos="{distance} 0 0.02" '
                     'size="0.035 5.0 0.02" rgba="0.3 0.37 0.44 1"/>')
    for name, y, rgba in _LANES:
        parts.append(f"""
        <geom type="box" pos="{center} {y} 0.025" size="{span / 2} 1.45 0.025"
              rgba="0.17 0.22 0.29 1"/>
        <geom name="{name}_trail" type="box" pos="0 {y} 0.058"
              size="0.001 0.25 0.009" rgba="{rgba}"/>
        <body name="{name}" pos="0 {y} 0">
          <joint name="{name}_position" type="slide" axis="1 0 0"/>
          <geom type="box" pos="0 0 0.61" size="1.3 0.65 0.3" rgba="{rgba}"/>
          <geom type="box" pos="-0.22 0 1.06" size="0.67 0.54 0.23" rgba="{rgba}"/>
          <geom type="box" pos="0.46 0 1.07" size="0.012 0.48 0.18" rgba="0.13 0.2 0.27 1"/>
          <geom type="box" pos="1.31 -0.42 0.66" size="0.015 0.14 0.09" rgba="1 0.96 0.76 1"/>
          <geom type="box" pos="1.31 0.42 0.66" size="0.015 0.14 0.09" rgba="1 0.96 0.76 1"/>
        """)
        for x in (-0.84, 0.84):
            for side in (-0.70, 0.70):
                parts.append(f'<geom type="cylinder" pos="{x} {side} 0.33" '
                             'size="0.32 0.12" euler="90 0 0" rgba="0.055 0.065 0.08 1"/>')
        parts.append('</body>')
    parts.append('</worldbody></mujoco>')
    model = mujoco.MjModel.from_xml_string("".join(parts))
    data = mujoco.MjData(model)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [center, 0, 0]
    camera.azimuth = 90
    camera.elevation = -53
    camera.distance = max(26, span * 1.08)
    mujoco.mj_forward(model, data)
    return model, data, camera


def _place(model: mujoco.MjModel, data: mujoco.MjData,
           samples: tuple[np.ndarray, np.ndarray], probe_time: float) -> tuple[float, float]:
    speeds = []
    for (name, _, _), rows in zip(_LANES, samples):
        position = float(np.interp(probe_time, rows[:, 0], rows[:, 1]))
        speed = float(np.interp(probe_time, rows[:, 0], rows[:, 2]))
        data.joint(f"{name}_position").qpos[0] = position
        data.joint(f"{name}_position").qvel[0] = speed
        trail = model.geom(f"{name}_trail")
        trail.pos[0] = position / 2
        trail.size[0] = max(0.001, abs(position) / 2)
        speeds.append(speed)
    data.time = probe_time
    # Forward kinematics only: the physics was already computed in the saved run.
    mujoco.mj_forward(model, data)
    return speeds[0], speeds[1]


def _summary(label: str, run: dict) -> str:
    summary = run["summary"]
    distance = summary.get("stopping_distance_m")
    if distance is None:
        motion = f"not stopped; last x = {run['probe'][-1]['x_m']:.2f} m"
    else:
        motion = f"stopped at {distance:.2f} m"
    crossing = "crossed barrier" if summary["wall_crossed"] else "did not cross barrier"
    return f"{label}: {motion}; {crossing}"


def show_comparison(candidate: dict, reference: dict, *, wall_distance_m: float,
                    title: str = "Synthetic braking replay", duration_s: float | None = None,
                    speedup: float = 1., autoplay: bool = False) -> None:
    """Replay recorded probe trajectories in one window, initially paused.

    Space starts, pauses, resumes, or replays after completion. R/N restart and
    play; C cycles cameras, +/- changes playback speed, and Escape closes.
    Autoplay starts immediately. An optional wall-clock duration bounds smoke checks.
    """
    import mujoco.viewer

    if duration_s is not None and (not math.isfinite(duration_s) or duration_s <= 0):
        raise ValueError("duration_s must be finite and positive.")
    samples = (_samples(candidate), _samples(reference))
    model, data, camera = _scene(*samples, wall_distance_m, title)
    controls = ViewControls(model, scenario="recorded_car", speedup=speedup,
                            camera="free", distance=camera.distance)
    end_time = max(float(rows[-1, 0]) for rows in samples)
    keys: SimpleQueue[int] = SimpleQueue()

    def on_key(keycode: int) -> None:
        keys.put(keycode)

    _place(model, data, samples, 0)
    with mujoco.viewer.launch_passive(
        model, data, key_callback=on_key, show_left_ui=False, show_right_ui=False
    ) as viewer:
        with viewer.lock():
            viewer.cam.lookat[:] = camera.lookat
            viewer.cam.azimuth = camera.azimuth
            viewer.cam.elevation = camera.elevation
            viewer.cam.distance = camera.distance
        started = previous = time.monotonic()
        elapsed = 0.0
        paused = not autoplay
        while viewer.is_running():
            now = time.monotonic()
            if duration_s is not None and now - started >= duration_s:
                break
            restart = close = False
            while True:
                try:
                    key = keys.get_nowait()
                except Empty:
                    break
                if controls.handle_key(key, viewer, camera.lookat):
                    continue
                if key == ord(" "):
                    if elapsed >= end_time:
                        elapsed = 0.0
                        paused = False
                        restart = True
                    else:
                        paused = not paused
                elif key in (ord("R"), ord("r"), ord("N"), ord("n")):
                    elapsed = 0.0
                    paused = False
                    restart = True
                elif key == 256:
                    close = True
            if close:
                break
            if not paused and not restart:
                elapsed = min(end_time, elapsed + (now - previous) * controls.speedup)
            previous = now
            if elapsed >= end_time:
                paused = True
            probe_time = elapsed
            with viewer.lock():
                speeds = _place(model, data, samples, probe_time)
            state = "Replay complete" if elapsed >= end_time else "Paused" if paused else "Replaying"
            viewer.set_texts([
                (mujoco.mjtFont.mjFONT_SHADOW, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                 f"{title}\nRecorded synthetic trajectories | probe t = {probe_time:.2f} s\n"
                 + _summary("BLUE / original candidate", candidate) + "\n"
                 + _summary("CORAL / synthetic reference", reference), ""),
                (mujoco.mjtFont.mjFONT_SHADOW, mujoco.mjtGridPos.mjGRID_TOPRIGHT,
                 f"{controls.speedup:g}x playback | {controls.camera} camera\n"
                 "C: cycle camera | - / +: playback speed", ""),
                (mujoco.mjtFont.mjFONT_SHADOW, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
                 f"{state} | Space: start/pause/resume/replay | R/N: restart and play | Esc: close\n"
                 f"Barrier = {wall_distance_m:.2f} m; crossing uses the vehicle center.\n"
                 "Visual barrier only; contact does not alter the recorded motion.", ""),
                (mujoco.mjtFont.mjFONT_SHADOW, mujoco.mjtGridPos.mjGRID_BOTTOMRIGHT,
                 f"BLUE speed: {speeds[0]:.2f} m/s\nCORAL speed: {speeds[1]:.2f} m/s", ""),
            ])
            viewer.sync()
            time.sleep(1 / 60)


def render_comparison(candidate: dict, reference: dict, *, wall_distance_m: float,
                      output_path: Path) -> None:
    """Save an offscreen PNG of the final recorded positions, with honest labels."""
    from PIL import Image

    samples = (_samples(candidate), _samples(reference))
    model, data, camera = _scene(*samples, wall_distance_m, "Synthetic braking replay")
    _place(model, data, samples, max(float(rows[-1, 0]) for rows in samples))
    width, height = 1440, 900
    gl = mujoco.GLContext(width, height)
    context = None
    try:
        gl.make_current()
        context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, context)
        scene = mujoco.MjvScene(model, maxgeom=1000)
        mujoco.mjv_updateScene(model, data, mujoco.MjvOption(), None, camera,
                              mujoco.mjtCatBit.mjCAT_ALL, scene)
        viewport = mujoco.MjrRect(0, 0, width, height)
        mujoco.mjr_render(viewport, scene, context)
        mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                           viewport, "SYNTHETIC BRAKING | final positions from recorded probes\n"
                           + _summary("BLUE / original candidate", candidate) + "\n"
                           + _summary("CORAL / synthetic reference", reference), "", context)
        mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
                           viewport, f"Start: pale line at 0 m | Amber barrier: {wall_distance_m:.2f} m\n"
                           "Crossing uses the vehicle center. Visual barrier only; no crash dynamics.\n"
                           "Recorded MuJoCo trajectories; no agent-authored repair is claimed.", "", context)
        pixels = np.empty((height, width, 3), dtype=np.uint8)
        mujoco.mjr_readPixels(pixels, None, viewport, context)
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(np.flipud(pixels)).save(output_path, format="PNG")
    finally:
        if context is not None:
            context.free()
        gl.free()
