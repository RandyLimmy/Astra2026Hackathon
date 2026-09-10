"""Shared presentation controls; all effects stay outside physical integration."""

from dataclasses import asdict, is_dataclass
import json
import math

import mujoco


CAMERAS = ("chase", "side", "overview", "free")
SPEEDS = (.125, .25, .5, 1., 2., 4., 8.)


def setting_overrides(assignments: list[str] | None) -> dict:
    """Parse repeated NAME=JSON settings; unquoted text is a string value."""
    values = {}
    for assignment in assignments or ():
        name, separator, raw = assignment.partition("=")
        if not separator or not name.isidentifier() or not raw.strip():
            raise ValueError("--set expects NAME=VALUE, for example --set rotor_effectiveness=0.75")
        try:
            value = json.loads(raw)
        except json.JSONDecodeError:
            value = raw
        # Reject Python JSON's nonstandard NaN/Infinity before config validation.
        json.dumps(value, allow_nan=False)
        values[name] = value
    return values


def viewer_arguments(parser) -> None:
    parser.add_argument("--camera", choices=CAMERAS, help="initial camera (default: showcase side, otherwise chase)")
    parser.add_argument("--autoplay", action="store_true", help="start immediately instead of waiting for Space")
    parser.add_argument("--native-ui", action="store_true", help="also show MuJoCo's advanced side panels")


def settings_lines(scenario: str, config) -> list[str]:
    """Friendly names for operator settings; never used in public recordings."""
    values = asdict(config) if is_dataclass(config) and not isinstance(config, type) else {}
    platform = scenario.split("_", 1)[0]
    fault = values.get("fault", "healthy")
    common = {
        "drone": (("flight_distance", "Route", "m"), ("flight_altitude", "Cruise altitude", "m"))
                 if values.get("probe") == "showcase" else (),
        "quadruped": (("speed", "Walking speed", "m/s"), ("gait_period", "Gait period", "s")),
        "warehouse": (("payload_mass", "Base cargo", "kg"),),
        "car": (("impact_speed", "Approach speed", "m/s"), ("probe_speed", "Probe speed", "m/s")),
    }
    faults = {
        ("drone", "rotor_loss"): ("rotor_effectiveness", "Rotor output", "%"),
        ("drone", "voltage_sag"): ("voltage_ratio", "Voltage", "%"),
        ("drone", "payload"): ("payload_mass", "Added payload", "kg"),
        ("drone", "wind"): ("wind_force", "Wind force", "N"),
        ("drone", "delay"): ("control_delay", "Command delay", "s"),
        ("quadruped", "joint_weakness"): ("strength", "Leg output", "%"),
        ("quadruped", "foot_slip"): ("foot_friction", "Foot friction", ""),
        ("quadruped", "leg_damage"): ("damage_stiffness", "Joint stiffness", "Nm/rad"),
        ("quadruped", "payload_shift"): ("payload_offset", "Payload offset", "m"),
        ("warehouse", "payload_mass"): ("added_mass", "Added cargo", "kg"),
        ("warehouse", "payload_shift"): ("shift_distance", "Shift travel", "m"),
        ("warehouse", "traction"): ("traction_scale", "Grip scale", "%"),
        ("warehouse", "battery"): ("motor_scale", "Motor output", "%"),
        ("warehouse", "caster_jam"): ("resistance", "Caster resistance", "Nm"),
        ("warehouse", "rolling_resistance"): ("resistance", "Bearing resistance", "Nm"),
        ("car", "steering"): ("steering_bias", "Steering bias", "rad"),
        ("car", "misalignment"): ("toe_angle", "Wheel toe", "rad"),
    }
    fields = common.get(platform, (
        ("initial_speed", "Starting speed", "m/s"), ("brake", "Brake command", "%"),
        ("warmup_cycles", "Warm-up cycles", "")))
    if (platform, fault) in faults:
        fields = (*fields, faults[platform, fault])
    result = []
    if "fault" in values and fault in ("healthy", "none"):
        result.append("Healthy reference")
    elif platform == "warehouse" and fault == "cargo_breakaway":
        result.append("Force-triggered restraint release")
    elif "fault_at" in values:
        label = "Impact arming" if scenario.startswith("car_") else "Change scheduled"
        result.append(f"{label}: {values['fault_at']:g} s")
    for key, label, unit in fields:
        value = values.get(key)
        if isinstance(value, (float, int)):
            result.append(f"{label}: {value * 100 if unit == '%' else value:g} {unit}".rstrip())
    return result


class ViewControls:
    """Camera and wall-clock speed controls, plus a compact status overlay."""

    def __init__(self, model, *, scenario: str, config=None, speedup: float = 1,
                 camera: str = "chase", distance: float = 6):
        if isinstance(speedup, bool) or not math.isfinite(speedup) or speedup <= 0:
            raise ValueError("--speedup must be positive and finite")
        self.model = model
        self.scenario = scenario
        self.config = config
        self.speedup = float(speedup)
        self.distance = distance
        self.cameras = tuple(name for name in CAMERAS if name == "free" or
                             mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name) >= 0)
        if self.cameras == ("free",):
            self.cameras = ("free", "side", "overview")
        if camera not in self.cameras:
            raise ValueError(f"Camera {camera!r} is unavailable; choose {', '.join(self.cameras)}")
        self.camera = camera

    def apply_camera(self, viewer, position) -> None:
        camera_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, self.camera)
        if camera_id < 0:
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            viewer.cam.distance = self.distance
            viewer.cam.azimuth = 130 if self.camera == "free" else 90
            viewer.cam.elevation = {"free": -25, "side": -10, "overview": -80}[self.camera]
            viewer.cam.lookat[:] = position
        else:
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            viewer.cam.fixedcamid = camera_id

    def handle_key(self, key: int, viewer, position) -> bool:
        if key in (ord("C"), ord("c")):
            self.camera = self.cameras[(self.cameras.index(self.camera) + 1) % len(self.cameras)]
            with viewer.lock():
                self.apply_camera(viewer, position)
            return True
        if key in (ord("+"), ord("="), 334):
            self.speedup = next((rate for rate in SPEEDS if rate > self.speedup), max(SPEEDS[-1], self.speedup))
            return True
        if key in (ord("-"), 333):
            self.speedup = next((rate for rate in reversed(SPEEDS) if rate < self.speedup),
                                min(SPEEDS[0], self.speedup))
            return True
        return False

    def texts(self, *, elapsed: float, duration: float, phase: str, paused: bool,
              finished: bool, detail: str = "") -> list[tuple]:
        state = "COMPLETE" if finished else "READY" if paused and elapsed < 1e-8 else "PAUSED" if paused else "PLAYING"
        progress = min(1., max(0., elapsed / duration)) if duration > 0 and not finished else 1.
        filled = round(24 * progress)
        bar = "[" + "=" * filled + "." * (24 - filled) + "]"
        title = self.scenario.replace("_", " ").upper()
        stage = phase.replace("_", " ").capitalize()
        settings = "\n".join(settings_lines(self.scenario, self.config))
        action = "Space: replay from start" if finished else "Space: play the full scenario" if state == "READY" else "Space: pause / resume"
        return [
            (mujoco.mjtFont.mjFONT_SHADOW, mujoco.mjtGridPos.mjGRID_TOPLEFT,
             f"{title}\n{state}  |  {elapsed:.1f} / {duration:.1f} s\n{stage}\n{bar}", ""),
            (mujoco.mjtFont.mjFONT_SHADOW, mujoco.mjtGridPos.mjGRID_TOPRIGHT,
             f"{self.speedup:g}x playback  |  {self.camera} camera\n{settings}", ""),
            (mujoco.mjtFont.mjFONT_SHADOW, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
             f"{action}\nN: full replay  |  R: repeat with damage  |  Esc: close\nC: camera  |  - / +: playback speed", ""),
            (mujoco.mjtFont.mjFONT_SHADOW, mujoco.mjtGridPos.mjGRID_BOTTOMRIGHT,
             detail or "Physical simulation\nNominal controller; no automatic model repair", ""),
        ]

    def update(self, viewer, *, position, elapsed: float, duration: float, phase: str,
               paused: bool, finished: bool, detail: str = "") -> None:
        if mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, self.camera) < 0:
            with viewer.lock():
                viewer.cam.lookat[:] = position
        viewer.set_texts(self.texts(elapsed=elapsed, duration=duration, phase=phase,
                                  paused=paused, finished=finished, detail=detail))
