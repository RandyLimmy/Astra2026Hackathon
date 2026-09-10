"""Physical differential-drive probes with a hidden, changing warehouse load.

Commands are normalized wheel motor torques. The open-loop probe schedules are
deliberately ignorant of fault parameters. Payload mass is loaded at reset;
the other faults change joints, contact, or motor capacity at ``fault_at``.
The cargo rail slopes gently sideways, so releasing its latch permits a real
gravity/inertia-driven load shift, including momentum transfer at the end stop.
"""
from dataclasses import asdict, dataclass
import math
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ASSET = Path(__file__).resolve().parents[1] / "assets" / "warehouse.xml"
FAULTS = ("healthy", "none", "payload_mass", "payload_shift", "traction", "caster_jam",
          "rolling_resistance", "battery")
PROBES = ("straight", "turn", "pulses", "validation")
WHEEL_RADIUS = 0.20
TRACK_WIDTH = 0.71


@dataclass(frozen=True)
class Config:
    fault: str = "healthy"
    duration: float = 8.0
    timestep: float = 0.002
    fault_at: float = 1.5
    probe: str = "straight"
    payload_mass: float = 8.0
    added_mass: float = 18.0
    shift_distance: float = 0.24
    traction_scale: float = 0.05
    resistance: float = 0.65
    motor_scale: float = 0.42
    fault_ramp: float = 0.5

    def __post_init__(self) -> None:
        bounds = {"duration": (0.01, 60), "timestep": (0.00025, 0.005),
                  "fault_at": (0, 120), "payload_mass": (0.1, 30), "added_mass": (0, 40),
                  "shift_distance": (0, 0.3), "traction_scale": (0.01, 1),
                  "resistance": (0, 5), "motor_scale": (0, 1), "fault_ramp": (0, 10)}
        for name, (low, high) in bounds.items():
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{name} must be a finite number")
            if not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f"{name} must be between {low} and {high}")
        if self.fault not in FAULTS:
            raise ValueError(f"fault must be one of {FAULTS}")
        if self.probe not in PROBES:
            raise ValueError(f"probe must be one of {PROBES}")
        if self.duration < self.timestep:
            raise ValueError("duration must be at least one timestep")


PRESETS: dict[str, dict[str, Any]] = {
    "warehouse_healthy": {},
    "warehouse_payload": {"fault": "payload_mass"},
    "warehouse_shift": {"fault": "payload_shift", "probe": "turn"},
    "warehouse_slip": {"fault": "traction"},
    "warehouse_caster": {"fault": "caster_jam", "probe": "turn"},
    "warehouse_resistance": {"fault": "rolling_resistance"},
    "warehouse_battery": {"fault": "battery", "probe": "pulses"},
    "warehouse_demo": {"fault": "payload_shift", "probe": "validation", "duration": 9.0,
                       "fault_at": 3.2, "payload_mass": 16.0, "shift_distance": 0.28},
}
DESCRIPTIONS = {
    "warehouse_healthy": "Secured nominal cargo; straight acceleration and coast probe.",
    "warehouse_payload": "Extra cargo is loaded before the trial; compare acceleration and coast.",
    "warehouse_shift": "A cargo latch releases on an inclined lateral rail during the turn probe.",
    "warehouse_slip": "Drive-wheel contact grip decreases during an acceleration probe.",
    "warehouse_caster": "Caster rolling and swivel joints develop friction during turns.",
    "warehouse_resistance": "Drive-wheel bearing resistance increases; inspect coast and pulses.",
    "warehouse_battery": "Common motor torque capacity ramps down during low/high torque pulses.",
    "warehouse_demo": "Carry secured boxes into a turn, release the cargo latch, then continue the maneuver.",
}


def model_xml(config: Config) -> str:
    """Compile-time world; use ``replace(config, fault='healthy')`` for export."""
    root = ET.parse(ASSET).getroot()
    option = root.find("option")
    assert option is not None
    option.set("timestep", str(config.timestep))
    cargo = root.find(".//geom[@name='cargo_box']")
    assert cargo is not None
    mass = config.payload_mass + (config.added_mass if config.fault == "payload_mass" else 0)
    cargo.set("mass", str(mass))
    # Keep the inactive joint's nominal constraint identical for every preset.
    # The travel stop changes only when the latch releases at fault onset.
    return ET.tostring(root, encoding="unicode")


def probe_command(probe: str, time: float) -> dict[str, float]:
    """Nominal, time-only intervention sequence shared by all physics variants."""
    if probe not in PROBES:
        raise ValueError(f"unknown probe: {probe}")
    schedules = {
        "straight": ((0.5, 0., 0.), (3.5, .65, .65), (5., 0., 0.), (6., .35, .35)),
        "turn": ((0.5, 0., 0.), (1.5, .3, .3), (3.5, .15, .55),
                 (4., 0., 0.), (6., .55, .15)),
        "pulses": ((.5, 0., 0.), (1., .3, .3), (1.7, 0., 0.), (2.4, .7, .7),
                   (3.2, 0., 0.), (4.4, .35, .35), (5.2, 0., 0.), (6.4, .8, .8)),
        "validation": ((.7, 0., 0.), (2.2, .5, .5), (3., 0., 0.),
                       (4.2, .5, .22), (4.8, 0., 0.), (6.3, .25, .6)),
    }
    for end, left, right in schedules[probe]:
        if time < end - 1e-10:
            return {"left": left, "right": right}
    return {"left": 0., "right": 0.}


class Simulation:
    camera_distance = 4.5

    def __init__(self, config: Config):
        self.config = config
        self.model = mujoco.MjModel.from_xml_string(model_xml(config))
        self.data = mujoco.MjData(self.model)
        self.focus_body = self.model.body("chassis").id
        self._wheel_dofs = np.array([int(self.model.joint(f"wheel_{s}").dofadr[0])
                                     for s in ("left", "right")])
        self._tires = np.array([self.model.geom(f"tire_{s}").id for s in ("left", "right")])
        self._caster_dofs = np.array([int(self.model.joint(n).dofadr[0])
                                     for n in ("caster_swivel", "caster_roll")])
        self._slide = self.model.joint("cargo_slide").id
        self._cargo_qpos = int(self.model.jnt_qposadr[self._slide])
        self._swivel_qpos = int(self.model.joint("caster_swivel").qposadr[0])
        self._latch = self.model.equality("cargo_latch").id
        self._nominal_friction = self.model.geom_friction.copy()
        self._nominal_resistance = self.model.dof_frictionloss.copy()
        self._nominal_gain = self.model.actuator_gainprm.copy()
        self._nominal_range = self.model.jnt_range.copy()
        self.events: list[dict[str, Any]] = []
        self._fault_applied = False
        self._trial_start = 0.
        self._initialize_trial()
        self._record_loading()

    @property
    def elapsed(self) -> float:
        return float(self.data.time)

    @property
    def trial_time(self) -> float:
        return self.elapsed - self._trial_start

    @property
    def finished(self) -> bool:
        return self.trial_time >= self.config.duration - 1e-9

    def _initialize_trial(self) -> None:
        self.last_command = {"left": 0., "right": 0.}
        self._max_speed = 0.
        self._max_tilt = 0.
        self._distance = 0.
        mujoco.mj_forward(self.model, self.data)
        self._start_position = self.data.xpos[self.focus_body].copy()
        self._last_position = self._start_position.copy()

    def _record_loading(self) -> None:
        if self.config.fault == "payload_mass":
            self._fault_applied = True
            self.events.append({"time": 0., "event": "payload_loaded", "mass_kg":
                                self.config.payload_mass + self.config.added_mass})

    def _apply_fault(self) -> None:
        cfg = self.config
        if cfg.fault in ("healthy", "none", "payload_mass") or self.elapsed < cfg.fault_at - 1e-10:
            return
        if not self._fault_applied:
            self._fault_applied = True
            self.events.append({"time": self.elapsed, "event": cfg.fault})
            if cfg.fault == "payload_shift" and cfg.shift_distance > 0:
                self.model.jnt_range[self._slide] = [0, cfg.shift_distance]
                self.data.eq_active[self._latch] = False
            elif cfg.fault == "traction":
                self.model.geom_friction[self._tires, 0] *= cfg.traction_scale
            elif cfg.fault == "rolling_resistance":
                self.model.dof_frictionloss[self._wheel_dofs] += cfg.resistance
            elif cfg.fault == "caster_jam":
                self.model.dof_frictionloss[self._caster_dofs] += [5 * cfg.resistance,
                                                                 3 * cfg.resistance]
        if cfg.fault == "battery":
            fraction = min(1., (self.elapsed - cfg.fault_at) / cfg.fault_ramp) if cfg.fault_ramp else 1.
            self.model.actuator_gainprm[:, 0] = 1 - (1 - cfg.motor_scale) * max(0., fraction)

    def step(self, control: dict[str, float] | None = None) -> None:
        command = probe_command(self.config.probe, self.trial_time) if control is None else control
        if not isinstance(command, dict) or set(command) - {"left", "right"}:
            raise ValueError("control must contain only left and right motor commands")
        values = [command.get(name, 0.) for name in ("left", "right")]
        if any(isinstance(value, bool) or not isinstance(value, (float, int))
               or not math.isfinite(value) or not -1 <= value <= 1 for value in values):
            raise ValueError("left and right must be finite normalized torques between -1 and 1")
        if self.finished:
            return
        self._apply_fault()
        self.last_command = dict(zip(("left", "right"), map(float, values)))
        self.data.ctrl[:] = values
        mujoco.mj_step(self.model, self.data)
        # Recompute pose/sensors at the recorded time, after integration.
        mujoco.mj_forward(self.model, self.data)
        position = self.data.xpos[self.focus_body]
        self._distance += float(np.linalg.norm(position[:2] - self._last_position[:2]))
        self._last_position = position.copy()
        self._max_speed = max(self._max_speed, float(np.linalg.norm(self.data.qvel[:2])))
        self._max_tilt = max(self._max_tilt, self._tilt())

    def _tilt(self) -> float:
        return math.acos(float(np.clip(self.data.xmat[self.focus_body, 8], -1, 1)))

    def observe(self) -> dict[str, Any]:
        rotation = self.data.xmat[self.focus_body].reshape(3, 3)
        velocity = np.zeros(6)
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY,
                                self.focus_body, velocity, 0)
        wheel_speed = self.data.qvel[self._wheel_dofs]
        return {"time": self.elapsed, "trial_time": self.trial_time,
                "phase": "complete" if self.finished else
                         "coast" if not any(self.last_command.values()) else "drive",
                "position": self.data.xpos[self.focus_body].tolist(),
                "orientation": self.data.xquat[self.focus_body].tolist(),
                "heading": math.atan2(float(rotation[1, 0]), float(rotation[0, 0])),
                "linear_velocity": velocity[3:].tolist(), "angular_velocity": velocity[:3].tolist(),
                "command": self.last_command.copy(),
                "wheel_speed": wheel_speed.tolist(),
                "encoder_velocity": {"forward": float(wheel_speed.mean() * WHEEL_RADIUS),
                                     "yaw": float((wheel_speed[1] - wheel_speed[0]) *
                                                  WHEEL_RADIUS / TRACK_WIDTH)},
                "accelerometer": self.data.sensor("acceleration").data.tolist(),
                "gyroscope": self.data.sensor("angular_velocity").data.tolist()}

    def diagnostics(self) -> dict[str, Any]:
        return {"config": asdict(self.config), "events": list(self.events),
                "fault_applied": self._fault_applied,
                "total_mass_kg": float(self.model.body_mass.sum()),
                "center_of_mass": self.data.subtree_com[self.focus_body].tolist(),
                "cargo_displacement": float(self.data.qpos[self._cargo_qpos]),
                "cargo_latched": bool(self.data.eq_active[self._latch]),
                "drive_friction": self.model.geom_friction[self._tires, 0].tolist(),
                "bearing_resistance": self.model.dof_frictionloss[self._wheel_dofs].tolist(),
                "caster_resistance": self.model.dof_frictionloss[self._caster_dofs].tolist(),
                "motor_scale": self.model.actuator_gainprm[:, 0].tolist(),
                "warnings": self.data.warning.number.tolist()}

    def summary(self) -> dict[str, Any]:
        position = self.data.xpos[self.focus_body]
        finite = bool(np.isfinite(self.data.qpos).all() and np.isfinite(self.data.qvel).all())
        safe = finite and self._max_tilt < .6 and not self.data.warning.number.any()
        public = {"platform": "warehouse", "outcome": "upright" if safe else "unstable",
                  "safe": bool(safe),
                  "metrics": {"elapsed": self.elapsed, "trial_time": self.trial_time,
                              "position": position.tolist(), "path_length_m": self._distance,
                              "displacement_m": float(np.linalg.norm(position[:2] -
                                                                       self._start_position[:2])),
                              "max_speed_m_s": self._max_speed, "max_tilt_rad": self._max_tilt,
                              "heading_rad": self.observe()["heading"]}}
        return {"public": public, "config": asdict(self.config), "events": list(self.events),
                "warnings": self.data.warning.number.tolist(), "diagnostics": self.diagnostics()}

    def reset_trial(self) -> None:
        """Reposition with the same damaged physical parameters and experiment clock."""
        elapsed = self.elapsed
        latch = bool(self.data.eq_active[self._latch])
        cargo = float(self.data.qpos[self._cargo_qpos])
        swivel = float(self.data.qpos[self._swivel_qpos])
        mujoco.mj_resetData(self.model, self.data)
        self.data.time = elapsed
        self._trial_start = elapsed
        self.data.eq_active[self._latch] = latch
        if not latch:
            self.data.qpos[self._cargo_qpos] = cargo
        if self._fault_applied and self.config.fault == "caster_jam":
            self.data.qpos[self._swivel_qpos] = swivel
        self._initialize_trial()

    def reset_full(self) -> None:
        """Replay the initial configuration, including its original fault schedule."""
        self.model.geom_friction[:] = self._nominal_friction
        self.model.dof_frictionloss[:] = self._nominal_resistance
        self.model.actuator_gainprm[:] = self._nominal_gain
        self.model.jnt_range[:] = self._nominal_range
        mujoco.mj_resetData(self.model, self.data)
        self._trial_start = 0.
        self._fault_applied = False
        self.events.clear()
        self._initialize_trial()
        self._record_loading()
