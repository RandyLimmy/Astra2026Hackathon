"""Physical differential-drive probes with a hidden, changing warehouse load.

Commands are normalized wheel motor torques. The open-loop probe schedules are
deliberately ignorant of fault parameters. Payload mass is loaded at reset;
legacy faults change joints, contact, or motor capacity at ``fault_at``. The
new cargo probes use a horizontal rail and a restraint that breaks only after
sustained lateral constraint load. Legacy probes retain their inclined rail.
Released cargo moves through real dynamics, including momentum transfer at
the end stop.
"""
from dataclasses import asdict, dataclass
import math
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ASSET = Path(__file__).resolve().parents[1] / "assets" / "warehouse.xml"
FAULTS = ("healthy", "none", "payload_mass", "payload_shift", "cargo_breakaway", "traction", "caster_jam",
          "rolling_resistance", "battery")
CARGO_PROBES = ("cargo_turn", "cargo_mirror", "cargo_gentle", "cargo_strong")
# PROBES remains the historical time-only schedule interface. Curve commands
# additionally depend on observed vehicle motion through route_command.
PROBES = ("straight", "turn", "pulses", "validation", *CARGO_PROBES)
CURVE_PROBES = ("cargo_curve", "cargo_curve_slow")
ALL_PROBES = (*PROBES, *CURVE_PROBES)
WHEEL_RADIUS = 0.20
TRACK_WIDTH = 0.71
ROUTE_APPROACH = 3.
ROUTE_RADIUS = 1.2
ROUTE_ARC = ROUTE_RADIUS * math.pi / 2
ROUTE_LENGTH = ROUTE_APPROACH + ROUTE_ARC + 2.4


def route_point(distance: float) -> np.ndarray:
    """The declared aisle: straight approach, 90-degree right bend, exit."""
    distance = float(np.clip(distance, 0., ROUTE_LENGTH))
    if distance <= ROUTE_APPROACH:
        return np.array([distance, 0.])
    if distance < ROUTE_APPROACH + ROUTE_ARC:
        angle = (distance - ROUTE_APPROACH) / ROUTE_RADIUS
        return np.array([ROUTE_APPROACH + ROUTE_RADIUS * math.sin(angle),
                         ROUTE_RADIUS * (math.cos(angle) - 1)])
    return np.array([ROUTE_APPROACH + ROUTE_RADIUS,
                     -ROUTE_RADIUS - (distance - ROUTE_APPROACH - ROUTE_ARC)])


_ROUTE_DISTANCES = np.linspace(0., ROUTE_LENGTH, 400)
_ROUTE_POINTS = np.array([route_point(distance) for distance in _ROUTE_DISTANCES])


def route_geometry() -> dict[str, Any]:
    """Declared before the trial; never fitted to the resulting trajectory."""
    return {"centerline": _ROUTE_POINTS.tolist(), "lane_width": 1.4,
            "approach_end": [ROUTE_APPROACH, 0.],
            "bend_center": [ROUTE_APPROACH, -ROUTE_RADIUS], "bend_radius": ROUTE_RADIUS,
            "bend_angle_deg": -90., "exit_start": [ROUTE_APPROACH + ROUTE_RADIUS, -ROUTE_RADIUS],
            "end": route_point(ROUTE_LENGTH).tolist(), "length": ROUTE_LENGTH,
            "controller": "body velocity feedback with geometric path pursuit",
            "target_speed_m_s": {"cargo_curve": 1.6, "cargo_curve_slow": .6}}


def route_command(probe: str, observation: dict[str, Any], drive_scale: float = 1.) -> dict[str, float]:
    """Same controller for both models, using only their own public observations.

    The motors apply actual wheel torques. Nothing constrains the chassis to
    the painted route or assigns a pose, velocity, or cargo state.
    """
    if probe not in CURVE_PROBES:
        raise ValueError(f"unknown curve probe: {probe}")
    time = float(observation["trial_time"])
    if time < .5 - 1e-10:
        return {"left": 0., "right": 0.}
    position = np.asarray(observation["position"][:2])
    nearest = int(np.argmin(np.linalg.norm(_ROUTE_POINTS - position, axis=1)))
    progress = max(float(_ROUTE_DISTANCES[nearest]), float(observation.get("route_progress", 0.)))
    vector = route_point(progress + .65) - position
    heading = float(observation["heading"])
    lateral = -math.sin(heading) * vector[0] + math.cos(heading) * vector[1]
    curvature = 2 * lateral / max(.05, float(np.dot(vector, vector)))
    desired_speed = min((1.6 if probe == "cargo_curve" else .6) * drive_scale,
                        .4 * max(0., time - .5), math.sqrt(max(0., .5 * (ROUTE_LENGTH - progress))))
    desired_yaw = desired_speed * curvature
    if progress >= ROUTE_LENGTH - .05:
        desired_speed = 0.
        heading_error = math.atan2(math.sin(-math.pi / 2 - heading), math.cos(-math.pi / 2 - heading))
        desired_yaw = float(np.clip(heading_error, -.3, .3))
    velocity = observation["linear_velocity"]
    forward = velocity[0] * math.cos(heading) + velocity[1] * math.sin(heading)
    common = 2 * (desired_speed - forward) + (.025 if desired_speed > .03 else 0.)
    steering = 1.5 * (desired_yaw - observation["angular_velocity"][2])
    return {"left": float(np.clip(common - steering, -1., 1.)),
            "right": float(np.clip(common + steering, -1., 1.))}


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
    drive_scale: float = 1.0
    latch_strength: float = 5.0
    latch_dwell: float = 0.01
    latch_arm_at: float = 0.5

    def __post_init__(self) -> None:
        bounds = {"duration": (0.01, 60), "timestep": (0.00025, 0.005),
                  "fault_at": (0, 120), "payload_mass": (0.1, 30), "added_mass": (0, 40),
                  "shift_distance": (0, 0.3), "traction_scale": (0.01, 1),
                  "resistance": (0, 5), "motor_scale": (0, 1), "fault_ramp": (0, 10),
                  "drive_scale": (0, 1.5), "latch_strength": (0.01, 10000),
                  "latch_dwell": (0, 10), "latch_arm_at": (0, 120)}
        for name, (low, high) in bounds.items():
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{name} must be a finite number")
            if not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f"{name} must be between {low} and {high}")
        if self.fault not in FAULTS:
            raise ValueError(f"fault must be one of {FAULTS}")
        if self.probe not in ALL_PROBES:
            raise ValueError(f"probe must be one of {ALL_PROBES}")
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
    "warehouse_cargo_demo": {"fault": "cargo_breakaway", "probe": "cargo_turn", "duration": 11.,
                             "payload_mass": 16., "shift_distance": .28},
    "warehouse_cargo_mirror_demo": {"fault": "cargo_breakaway", "probe": "cargo_mirror", "duration": 11.,
                                    "payload_mass": 16., "shift_distance": .28},
    "warehouse_cargo_gentle_demo": {"fault": "cargo_breakaway", "probe": "cargo_gentle", "duration": 11.,
                                    "payload_mass": 16., "shift_distance": .28},
    "warehouse_cargo_strong_demo": {"fault": "cargo_breakaway", "probe": "cargo_strong", "duration": 11.,
                                    "payload_mass": 16., "shift_distance": .28},
    "warehouse_curve_demo": {"fault": "cargo_breakaway", "probe": "cargo_curve", "duration": 17.,
                             "payload_mass": 8.},
    "warehouse_curve_control_demo": {"fault": "cargo_breakaway", "probe": "cargo_curve_slow", "duration": 17.,
                                     "payload_mass": 8.},
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
    "warehouse_cargo_demo": "Carry a secured load into a turn until real lateral force breaks its restraint.",
    "warehouse_cargo_mirror_demo": "Repeat the cargo maneuver with an opposite-direction turn and the same restraint.",
    "warehouse_cargo_gentle_demo": "Use balanced wheel commands as a matched no-turn control for the same restraint.",
    "warehouse_cargo_strong_demo": "Apply a sharper turn to the same cargo restraint and measure earlier release.",
    "warehouse_curve_demo": "Follow a marked 90-degree bend at speed; a failed restraint lets cargo spill onto the floor.",
    "warehouse_curve_control_demo": "Follow the same marked 90-degree bend slowly enough to retain the load.",
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
    if config.probe in CARGO_PROBES:
        # This declared fixture is shared by the reference and nominal models.
        # A horizontal rail lets opposite turns move the load in opposite
        # directions; only the hidden runtime release law differs.
        slide = root.find(".//joint[@name='cargo_slide']")
        assert slide is not None
        slide.set("axis", "0 1 0")
        slide.set("range", f"{-config.shift_distance} {config.shift_distance}")
        slide.set("limited", "true" if config.shift_distance > 0 else "false")
        slide.set("damping", "3")
        for name, x in (("cargo_rail_rear", -.28), ("cargo_rail_front", .28)):
            rail = root.find(f".//geom[@name='{name}']")
            assert rail is not None
            rail.set("fromto", f"{x} -.49 .135 {x} .49 .135")
        for name in ("chase", "side"):
            camera = root.find(f".//camera[@name='{name}']")
            assert camera is not None
            camera.set("fovy", "36")
            if name == "side":
                camera.set("pos", "0 -3.2 1.2")
    if config.probe in CURVE_PROBES:
        cargo_body = root.find(".//body[@name='cargo']")
        slide = root.find(".//joint[@name='cargo_slide']")
        chassis = root.find(".//body[@name='chassis']")
        world = root.find("worldbody")
        assert cargo_body is not None and slide is not None and chassis is not None and world is not None
        # Carry one parcel rather than the legacy massless upper box fused to
        # the lower box. Keep the parcel's declared mass and inertia together.
        for parcel_geom in list(cargo_body.findall("geom")):
            if float(parcel_geom.get("pos", "0 0 0").split()[2]) > .14:
                cargo_body.remove(parcel_geom)
        deck = root.find(".//geom[@name='cargo_deck']")
        assert deck is not None
        # The old deck was below the tops of the driven tires: an outward
        # sliding parcel hit a tire and was thrown back onto the cart. Provide
        # physical clearance and start the box resting exactly on the deck.
        deck.set("pos", "0 0 .20")
        cargo_body.set("pos", "0 0 .35")
        slide.set("axis", "0 1 0")
        slide.set("limited", "false")
        slide.set("damping", "0")
        for attributes in ({"name": "cargo_longitudinal", "type": "slide", "axis": "1 0 0"},
                           {"name": "cargo_vertical", "type": "slide", "axis": "0 0 1"},
                           {"name": "cargo_rotation", "type": "ball"}):
            ET.SubElement(cargo_body, "joint", {**attributes, "damping": "0", "armature": "0",
                                                "limited": "false"})
        for name in ("cargo_box", "cargo_deck"):
            geom = root.find(f".//geom[@name='{name}']")
            assert geom is not None
            geom.set("contype", "1")
            geom.set("conaffinity", "1")
            geom.set("friction", ".06 .002 .0001")
        contact = ET.SubElement(root, "contact")
        for first in ("cargo_box",):
            for second in ("cargo_deck", "chassis_geom"):
                # Explicit pairs are necessary for collisions between the
                # cargo body and its direct parent. The floor uses ordinary
                # collision detection.
                ET.SubElement(contact, "pair", geom1=first, geom2=second,
                              friction=".06 .06 .002 .0001 .0001", solref=".006 1",
                              solimp=".95 .99 .001")
        # This cart needs enough wheel torque to brake yaw at the end of the
        # tight bend. The declared gearing is shared by all curve variants.
        for motor in root.findall("./actuator/motor"):
            motor.set("gear", "9")
        for name in ("cargo_rail_rear", "cargo_rail_front"):
            rail = root.find(f".//geom[@name='{name}']")
            assert rail is not None
            chassis.remove(rail)
        # Replace the old straight aisle marks with this route, before rollout.
        for geom in list(world.findall("geom")):
            if geom.get("class") == "visual" and geom.get("pos", "").endswith(("0.003", "0.004")):
                world.remove(geom)
        from .warehouse_tracks import add_tracks
        add_tracks(world, chassis, _ROUTE_POINTS, TRACK_WIDTH)
        for name in ("overview", "chase", "side"):
            camera = root.find(f".//camera[@name='{name}']")
            assert camera is not None
            if name == "chase":
                camera.set("fovy", "42")
                continue
            camera.set("mode", "fixed")
            if name == "overview":
                camera.set("pos", "2.1 -5.8 6.4")
                camera.set("xyaxes", "1 0 0 0 .81 .59")
            else:
                # Outside the bend: the falling parcel stays in front of the
                # trolley instead of disappearing behind the raised deck.
                eye = np.array([7.5, 2.5, 3.3])
                z = eye - np.array([3.5, -.9, .3])
                z /= np.linalg.norm(z)
                right = np.cross([0., 0., 1.], z)
                right /= np.linalg.norm(right)
                up = np.cross(z, right)
                camera.set("pos", " ".join(map(str, eye)))
                camera.set("xyaxes", " ".join(map(str, np.r_[right, up])))
            camera.set("fovy", "60")
            chassis.remove(camera)
            world.append(camera)
    # Legacy probes keep their original travel stop until the timed release.
    # Each cargo probe's declared fixture is identical in matched comparisons.
    return ET.tostring(root, encoding="unicode")


def probe_command(probe: str, time: float) -> dict[str, float]:
    """Nominal, time-only intervention sequence shared by all physics variants."""
    if probe not in PROBES:
        raise ValueError(f"unknown probe: {probe}")
    if probe in CARGO_PROBES:
        turn = {"cargo_turn": (.65, .05), "cargo_mirror": (.05, .65),
                "cargo_gentle": (.45, .45), "cargo_strong": (.7, -.1)}[probe]
        for end, left, right in ((.5, 0., 0.), (3., .65, .65), (6., *turn), (8., .5, .5)):
            if time < end - 1e-10:
                return {"left": left, "right": right}
        return {"left": 0., "right": 0.}
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


def equality_force(model: mujoco.MjModel, data: mujoco.MjData, equality_id: int) -> float:
    """Magnitude in newtons of a scalar joint equality, from its own EFC row.

    Constraint ids are local to their type. Filtering only ``efc_id`` would
    silently include contacts or limits with the same id. Forces here must be
    sampled after ``mj_forward`` and before assigning the next step's command.
    """
    if model.eq_type[equality_id] != mujoco.mjtEq.mjEQ_JOINT:
        raise ValueError("force monitoring requires a scalar joint equality")
    rows = ((data.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY)
            & (data.efc_id == equality_id))
    values = data.efc_force[rows]
    if len(values) > 1:
        raise ValueError("joint equality must have at most one constraint row")
    return abs(float(values[0])) if len(values) else 0.


def presentation_stage(probe: str, time: float) -> str:
    """Describe the declared maneuver stage without disclosing physical state."""
    if probe in CARGO_PROBES:
        for end, stage in ((.5, "settling"), (3., "approach"),
                           (6., "straight control" if probe == "cargo_gentle" else "turn"),
                           (8., "departure")):
            if time < end - 1e-10:
                return stage
        return "coast"
    return "drive" if any(probe_command(probe, time).values()) else "coast"


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
        self._cargo_dof = int(self.model.jnt_dofadr[self._slide])
        self._swivel_qpos = int(self.model.joint("caster_swivel").qposadr[0])
        self._latch = self.model.equality("cargo_latch").id
        self._cargo_body = self.model.body("cargo").id
        self._cargo_geoms = {self.model.geom(name).id for name in
                             (("cargo_box",) if config.probe in CURVE_PROBES else
                              ("cargo_box", "cargo_top"))}
        self._floor = self.model.geom("floor").id
        self._deck = self.model.geom("cargo_deck").id
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
        self._latch_overload = 0.
        self._max_cargo_displacement = abs(float(self.data.qpos[self._cargo_qpos]))
        self._route_progress = 0.
        self._cargo_floor_contact = False
        self._cargo_on_deck = False
        self._cargo_dropped = False
        mujoco.mj_forward(self.model, self.data)
        self._update_curve_observation()
        self._start_position = self.data.xpos[self.focus_body].copy()
        self._last_position = self._start_position.copy()

    def _record_loading(self) -> None:
        if self.config.fault == "payload_mass":
            self._fault_applied = True
            self.events.append({"time": 0., "event": "payload_loaded", "mass_kg":
                                self.config.payload_mass + self.config.added_mass})

    def _apply_fault(self) -> None:
        cfg = self.config
        if cfg.fault == "cargo_breakaway":
            if self._fault_applied or cfg.shift_distance == 0:
                return
            force = equality_force(self.model, self.data, self._latch)
            overloaded = self.trial_time >= cfg.latch_arm_at - 1e-10 and force >= cfg.latch_strength
            self._latch_overload = self._latch_overload + cfg.timestep if overloaded else 0.
            if overloaded and self._latch_overload >= cfg.latch_dwell - 1e-12:
                self._fault_applied = True
                # Change only the constraint. Pose, velocity, mass, wheel grip,
                # and actuator capacity are untouched by this transition.
                self.data.eq_active[self._latch] = False
                self.events.append({"time": self.elapsed, "event": "cargo_breakaway",
                                    "constraint_force_N": force,
                                    "overload_duration_s": self._latch_overload})
            return
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
        command = (route_command(self.config.probe, self.observe(), self.config.drive_scale)
                   if control is None and self.config.probe in CURVE_PROBES else
                   {name: float(np.clip(value * self.config.drive_scale, -1., 1.))
                    for name, value in probe_command(self.config.probe, self.trial_time).items()}
                   if control is None else control)
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
        self._max_cargo_displacement = max(self._max_cargo_displacement,
                                           abs(float(self.data.qpos[self._cargo_qpos])))
        self._update_curve_observation()

    def _update_curve_observation(self) -> None:
        if self.config.probe in CURVE_PROBES:
            position = self.data.xpos[self.focus_body]
            nearest = int(np.argmin(np.linalg.norm(_ROUTE_POINTS - position[:2], axis=1)))
            self._route_progress = max(self._route_progress, float(_ROUTE_DISTANCES[nearest]))
            self._cargo_floor_contact = False
            self._cargo_on_deck = False
            for contact in self.data.contact:
                pair = {int(contact.geom1), int(contact.geom2)}
                if pair & self._cargo_geoms:
                    self._cargo_floor_contact |= self._floor in pair
                    self._cargo_on_deck |= self._deck in pair
            self._cargo_dropped |= self._cargo_floor_contact

    def _tilt(self) -> float:
        return math.acos(float(np.clip(self.data.xmat[self.focus_body, 8], -1, 1)))

    def observe(self) -> dict[str, Any]:
        rotation = self.data.xmat[self.focus_body].reshape(3, 3)
        velocity = np.zeros(6)
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY,
                                self.focus_body, velocity, 0)
        wheel_speed = self.data.qvel[self._wheel_dofs]
        observation = {"time": self.elapsed, "trial_time": self.trial_time,
                "phase": "complete" if self.finished else
                         presentation_stage(self.config.probe, self.trial_time)
                         if self.config.probe in CARGO_PROBES else
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
        if self.config.probe in CURVE_PROBES:
            observation.update({"cargo_position": self.data.xpos[self._cargo_body].tolist(),
                                "cargo_orientation": self.data.xquat[self._cargo_body].tolist(),
                                "cargo_floor_contact": self._cargo_floor_contact,
                                "cargo_has_touched_floor": self._cargo_dropped,
                                "route_progress": self._route_progress})
            observation["phase"] = ("complete" if self.finished else "settling" if self.trial_time < .5 else
                                    "approach" if self._route_progress < ROUTE_APPROACH else
                                    "turn" if self._route_progress < ROUTE_APPROACH + ROUTE_ARC else "exit")
        return observation

    def diagnostics(self) -> dict[str, Any]:
        cargo_velocity = np.zeros(6)
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY,
                                self._cargo_body, cargo_velocity, 0)
        wheel_loads = np.zeros(2)
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            force = np.zeros(6)
            mujoco.mj_contactForce(self.model, self.data, index, force)
            for wheel, tire in enumerate(self._tires):
                if tire in (contact.geom1, contact.geom2):
                    wheel_loads[wheel] += max(0., float(force[0]))
        return {"config": asdict(self.config), "events": list(self.events),
                "fault_applied": self._fault_applied,
                "total_mass_kg": float(self.model.body_mass.sum()),
                "center_of_mass": self.data.subtree_com[self.focus_body].tolist(),
                "cargo_displacement": float(self.data.qpos[self._cargo_qpos]),
                "cargo_velocity": float(self.data.qvel[self._cargo_dof]),
                "max_cargo_displacement": self._max_cargo_displacement,
                "cargo_latched": bool(self.data.eq_active[self._latch]),
                "cargo_position": self.data.xpos[self._cargo_body].tolist(),
                "cargo_linear_velocity": cargo_velocity[3:].tolist(),
                "cargo_angular_velocity": cargo_velocity[:3].tolist(),
                "cargo_tilt_rad": math.acos(float(np.clip(self.data.xmat[self._cargo_body, 8], -1., 1.))),
                "cargo_ground_contact": self._cargo_floor_contact,
                "cargo_dropped": self._cargo_dropped,
                "cargo_on_deck": self._cargo_on_deck,
                "cargo_latch_force_N": equality_force(self.model, self.data, self._latch),
                "cargo_latch_overload_s": self._latch_overload,
                "wheel_normal_load_N": wheel_loads.tolist(),
                "drive_friction": self.model.geom_friction[self._tires, 0].tolist(),
                "bearing_resistance": self.model.dof_frictionloss[self._wheel_dofs].tolist(),
                "caster_resistance": self.model.dof_frictionloss[self._caster_dofs].tolist(),
                "motor_scale": self.model.actuator_gainprm[:, 0].tolist(),
                "warnings": self.data.warning.number.tolist()}

    def summary(self) -> dict[str, Any]:
        position = self.data.xpos[self.focus_body]
        finite = bool(np.isfinite(self.data.qpos).all() and np.isfinite(self.data.qvel).all())
        safe = finite and self._max_tilt < .6 and not self.data.warning.number.any()
        public: dict[str, Any] = {"platform": "warehouse", "outcome": "upright" if safe else "unstable",
                  "safe": bool(safe),
                  "metrics": {"elapsed": self.elapsed, "trial_time": self.trial_time,
                              "position": position.tolist(), "path_length_m": self._distance,
                              "displacement_m": float(np.linalg.norm(position[:2] -
                                                                       self._start_position[:2])),
                              "max_speed_m_s": self._max_speed, "max_tilt_rad": self._max_tilt,
                              "heading_rad": self.observe()["heading"]}}
        if self.config.probe in CURVE_PROBES:
            public["outcome"] = "cargo_spilled" if self._cargo_dropped else "cargo_retained"
            public["metrics"].update({"vehicle_upright": bool(safe),
                                      "cargo_has_touched_floor": self._cargo_dropped,
                                      "route_progress_m": self._route_progress})
        return {"public": public, "config": asdict(self.config), "events": list(self.events),
                "warnings": self.data.warning.number.tolist(), "diagnostics": self.diagnostics()}

    def reset_trial(self) -> None:
        """Reposition with the same damaged physical parameters and experiment clock."""
        elapsed = self.elapsed
        latch = bool(self.data.eq_active[self._latch])
        cargo = float(self.data.qpos[self._cargo_qpos])
        cargo_pose = self.data.qpos[self._cargo_qpos:].copy()
        swivel = float(self.data.qpos[self._swivel_qpos])
        mujoco.mj_resetData(self.model, self.data)
        self.data.time = elapsed
        self._trial_start = elapsed
        self.data.eq_active[self._latch] = latch
        if not latch:
            if self.config.probe in CURVE_PROBES:
                self.data.qpos[self._cargo_qpos:] = cargo_pose
            else:
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
