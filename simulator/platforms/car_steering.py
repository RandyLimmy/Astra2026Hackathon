"""F3: a lane-following attempt with an initial steering calibration mismatch.

The four-wheel/suspension/actuator plant is reused unchanged from car_damage.
Only the rack transfer function differs between the failure and the explicitly
labelled healthy developer control. Route marks and the finish gate are visual;
roadside rails are physical. All motion after fixture preparation comes from
MuJoCo contacts and bounded actuator commands.
"""

from dataclasses import asdict, dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from .car_damage import Simulation as CarSimulation, WHEELS


@dataclass(frozen=True)
class Config:
    fault: str = "steering"
    duration: float = 12.0
    timestep: float = 0.002
    probe: str = "steering"
    probe_speed: float = 4.0
    steering_gain: float = 0.45
    steering_bias: float = -0.02
    lane_half_width: float = 1.8
    bend_start: float = 10.0
    bend_length: float = 18.0
    bend_offset: float = 3.5
    finish_x: float = 32.0

    def __post_init__(self):
        bounds = {"duration": (0.1, 30), "timestep": (0.0005, 0.004),
                  "probe_speed": (1, 8), "steering_gain": (0.05, 1.5),
                  "steering_bias": (-0.3, 0.3), "lane_half_width": (1.3, 3),
                  "bend_start": (4, 20), "bend_length": (8, 30),
                  "bend_offset": (-5, 5), "finish_x": (20, 60)}
        for field, (low, high) in bounds.items():
            value = getattr(self, field)
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or not low <= value <= high):
                raise ValueError(f"{field} must be a finite number in [{low}, {high}]")
        if self.fault not in ("healthy", "steering"):
            raise ValueError("fault must be healthy or steering")
        if self.probe != "steering":
            raise ValueError("probe must be steering")
        if self.finish_x < self.bend_start + self.bend_length:
            raise ValueError("finish_x must follow the bend")


PRESETS = {
    "car_steering_drift": {"fault": "steering"},
    "car_steering_nominal": {"fault": "healthy"},
}
DESCRIPTIONS = {
    "car_steering_drift": "Reach the green finish gate; steering drift sends the car into a roadside rail.",
    "car_steering_nominal": "Developer feasibility control: healthy steering follows the same marked lane.",
}


class Simulation(CarSimulation):
    """Independent no-crash startup, reusing the existing car mechanics."""

    # The inherited mechanics read only timestep. Crash-specific operations are
    # replaced here, so this standalone task intentionally has a smaller Config.
    config: Config  # type: ignore[assignment]

    def __init__(self, config: Config):
        self.config = config
        path = Path(__file__).parents[1] / "assets/platforms/car_damage.xml"
        root = ET.parse(path).getroot()
        root.set("model", "RealityPatch steering lane trial")
        world = root.find("worldbody")
        assert world is not None
        for element in list(world):
            if element.tag == "camera" or (element.tag == "geom" and element.get("name") != "ground"):
                world.remove(element)
        ground_texture = root.find("asset/texture[@name='ground']")
        assert ground_texture is not None
        ground_texture.set("rgb1", "0.24 0.29 0.24")
        ground_texture.set("rgb2", "0.27 0.32 0.26")
        self._build_route(world)
        ET.SubElement(world, "camera", name="overview", pos="18 -23 29",
                      xyaxes="1 0 0 0 0.75 0.66", fovy="50")
        chassis = world.find("body[@name='chassis']")
        assert chassis is not None
        chase = chassis.find("camera[@name='chase']")
        assert chase is not None
        # Track translation while looking along the route, keeping the finish
        # portal ahead and the car large in the lower part of the frame.
        chase.set("pos", "-8 -3.8 4.5")
        chase.set("xyaxes", "0.30 -0.954 0 0.325 0.102 0.940")
        chase.set("fovy", "62")
        ET.SubElement(chassis, "camera", name="side", mode="track", pos="0 -6 6",
                      xyaxes="1 0 0 0 0.707 0.707", fovy="58")
        self.model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
        self.model.opt.timestep = config.timestep
        self.data = mujoco.MjData(self.model)
        self.focus_body = self.model.body("chassis").id
        self._spin = np.array([int(self.model.joint(f"spin_{w}").dofadr[0]) for w in WHEELS])
        self._suspension = np.array([self.model.joint(f"suspension_{w}").id for w in WHEELS])
        self._suspension_q = self.model.jnt_qposadr[self._suspension]
        self._steer_q = np.array([int(self.model.joint(f"steer_{w}").qposadr[0]) for w in WHEELS[:2]])
        self._tire = np.array([self.model.geom(f"tire_{w}").id for w in WHEELS])
        self._wheel_bodies = np.array([self.model.body(f"wheel_{w}").id for w in WHEELS])
        self._roadside_geoms = {i for i in range(self.model.ngeom)
                               if self.model.geom(i).name.startswith("roadside_rail_")}
        self._vehicle_geoms = {
            i for i in range(self.model.ngeom)
            if int(self.model.body_rootid[self.model.geom_bodyid[i]]) == self.focus_body
        }
        self.reset_full()

    def _route(self, x: float) -> tuple[float, float, float]:
        """Lane center, heading, and signed curvature at longitudinal x."""
        c = self.config
        u = float(np.clip((x - c.bend_start) / c.bend_length, 0, 1))
        y = c.bend_offset * u * u * (3 - 2 * u)
        slope = c.bend_offset * 6 * u * (1 - u) / c.bend_length
        second = (c.bend_offset * (6 - 12 * u) / c.bend_length ** 2
                  if 0 < u < 1 else 0.0)
        return y, math.atan(slope), second / (1 + slope * slope) ** 1.5

    def _build_route(self, world):
        for index, x in enumerate(np.arange(-6.0, self.config.finish_x + 10, 0.75)):
            y, heading, _ = self._route(float(x))
            for side, offset, width, rgba in (
                    ("road", 0.0, self.config.lane_half_width, "0.11 0.14 0.18 1"),
                    ("left", self.config.lane_half_width, 0.04, "0.95 0.96 0.85 1"),
                    ("right", -self.config.lane_half_width, 0.04, "0.95 0.96 0.85 1")):
                px, py = x - offset * math.sin(heading), y + offset * math.cos(heading)
                ET.SubElement(world, "geom", {"name": f"lane_{side}_{index}",
                    "class": "visual", "type": "box", "pos": f"{px} {py} {0.006 if side == 'road' else 0.014}",
                    "size": f"0.40 {width} 0.003", "euler": f"0 0 {heading}", "rgba": rgba})
            if index % 3 == 0:
                ET.SubElement(world, "geom", {"class": "visual", "type": "box",
                    "pos": f"{x} {y} 0.015", "size": "0.28 0.025 0.003",
                    "euler": f"0 0 {heading}", "rgba": "0.94 0.73 0.24 1"})
        finish_y = self._route(self.config.finish_x)[0]
        for index in range(10):
            ET.SubElement(world, "geom", {"class": "visual", "type": "box",
                "pos": f"{self.config.finish_x} {finish_y - self.config.lane_half_width + (index + .5) * self.config.lane_half_width / 5} 0.018",
                "size": f"0.22 {self.config.lane_half_width / 10} 0.003",
                "rgba": "0.95 0.96 0.92 1" if index % 2 else "0.04 0.05 0.07 1"})
        gate_half_width = self.config.lane_half_width + 0.6
        for shoulder in (-1, 1):
            ET.SubElement(world, "geom", {"name": f"finish_gate_post_{shoulder}",
                "class": "visual", "type": "box", "size": "0.20 0.20 1.80",
                "pos": f"{self.config.finish_x} {finish_y + shoulder * gate_half_width} 1.80",
                "rgba": "0.08 0.90 0.35 1"})
        ET.SubElement(world, "geom", {"name": "finish_gate_header", "class": "visual",
            "type": "box", "size": f"0.22 {gate_half_width + .2} 0.35",
            "pos": f"{self.config.finish_x} {finish_y} 3.45", "rgba": "0.08 0.90 0.35 1"})
        for index in range(12):
            ET.SubElement(world, "geom", {"class": "visual", "type": "box",
                "pos": f"{self.config.finish_x - .225} {finish_y - gate_half_width + (index + .5) * gate_half_width / 6} 3.45",
                "size": f"0.008 {gate_half_width / 12} 0.16",
                "rgba": "0.97 0.98 0.94 1" if index % 2 else "0.04 0.06 0.06 1"})

        # The inside rail face is 0.35 m beyond the paint. It cannot obstruct
        # an in-lane car; a departing tire/body must physically cross the shoulder.
        for index, x in enumerate(np.arange(self.config.bend_start, self.config.bend_start
                                           + self.config.bend_length + 0.75, 0.75)):
            y, heading, _ = self._route(float(x))
            for shoulder in (-1, 1):
                offset = shoulder * (self.config.lane_half_width + .50)
                px, py = x - offset * math.sin(heading), y + offset * math.cos(heading)
                ET.SubElement(world, "geom", {"name": f"roadside_rail_{shoulder}_{index}",
                    "type": "box", "pos": f"{px} {py} .42", "size": ".42 .15 .42",
                    "euler": f"0 0 {heading}", "rgba": "0.72 0.77 0.81 1",
                    "friction": "0.6 0.005 0.0001"})
                ET.SubElement(world, "geom", {"class": "visual", "type": "box",
                    "pos": f"{px} {py} .85", "size": ".40 .16 .025",
                    "euler": f"0 0 {heading}",
                    "rgba": "1.0 .68 .09 1" if index % 2 else ".10 .13 .16 1"})

    @property
    def task_metadata(self) -> dict:
        return {"scenario_id": "F3", "title": "Steering drift",
                "objective": "Reach the green finish gate without leaving the lane or hitting a roadside rail.",
                "cameras": ["overview", "chase", "side"],
                "route": {"lane_half_width": self.config.lane_half_width,
                          "finish_x": self.config.finish_x,
                          "finish_gate": [self.config.finish_x, self._route(self.config.finish_x)[0], 3.45],
                          "roadside_rail_clearance": 0.35,
                          "centerline": [[float(x), self._route(float(x))[0]]
                                         for x in np.linspace(0, self.config.finish_x, 65)]},
                "provenance": "developer feasibility control" if self.config.fault == "healthy" else "failure demonstration"}

    @property
    def public_events(self) -> list[dict]:
        return [dict(event) for event in self._events]

    def reset_full(self):
        self.elapsed = 0.0
        self._steps = 0
        self._end_time = self.config.duration
        self._events: list[dict] = []
        self._warning_counts = np.zeros(len(self.data.warning), dtype=int)
        self._rack_gain = self.config.steering_gain if self.config.fault == "steering" else 1.0
        self._rack_bias = self.config.steering_bias if self.config.fault == "steering" else 0.0
        self._initialize_trial()

    def _initialize_trial(self):
        self._trial_start = self.elapsed
        self._probe_start = self.elapsed
        self._impact_time = None
        self._outside_since: float | None = None
        self._outside_start_step = 0
        self._failure_time: float | None = None
        self._finish_time: float | None = None
        self._barrier_contact_time: float | None = None
        self._barrier_impact_speed = 0.0
        self._barrier_contact_count = 0
        self._touching_rail = False
        self._bend_entered = False
        self._brake_started = False
        self._max_lateral_error = 0.0
        self._max_heading_error = 0.0
        self._max_tire_departure = 0.0
        self._max_outside_duration = 0.0
        self._distance = 0.0
        self._initialize_state(0.0, self.config.probe_speed)
        self._previous_xy = self.data.qpos[:2].copy()
        self._events.append({"event": "task_start", "time": self.elapsed,
                             "position": self.data.qpos[:3].tolist(),
                             "fixture_settle_seconds": 0.5,
                             "initial_speed": self.config.probe_speed})

    def reset_trial(self):
        old_pose = self.data.qpos[:3].copy().tolist()
        self._warning_counts += self.data.warning.number
        self._end_time = self.elapsed + self.config.duration
        self._events.append({"event": "trial_reset", "time": self.elapsed,
                             "from_position": old_pose, "retains_initial_setup": True})
        self._initialize_trial()

    def _route_measurements(self) -> dict:
        y, heading, _ = self._route(float(self.data.qpos[0]))
        yaw, _ = self._angles()
        heading_error = math.atan2(math.sin(yaw - heading), math.cos(yaw - heading))
        departures = []
        for geom in self._tire:
            x, tire_y, _ = self.data.geom_xpos[geom]
            center, route_heading, _ = self._route(float(x))
            normal = np.array([-math.sin(route_heading), math.cos(route_heading), 0.0])
            rotation = self.data.geom_xmat[geom].reshape(3, 3)
            # Ellipsoid support radius projected onto the route normal measures
            # the outside of the physical tire, including steering/yaw rotation.
            support = float(np.linalg.norm(self.model.geom_size[geom] * (rotation.T @ normal)))
            error = (tire_y - center) * math.cos(route_heading)
            departures.append(max(0.0, abs(float(error)) + support - self.config.lane_half_width))
        return {"lateral_error": (float(self.data.qpos[1]) - y) * math.cos(heading),
                "heading_error": heading_error, "route_heading": heading,
                "tire_departure": departures, "any_tire_outside": any(d > 0 for d in departures),
                "progress": float(np.clip(self.data.qpos[0] / self.config.finish_x, 0, 1))}

    def _nominal_command(self) -> dict:
        measured = self._route_measurements()
        _, _, curvature = self._route(float(self.data.qpos[0]))
        steering = float(np.clip(math.atan(2.6 * curvature)
                                - 0.6 * measured["heading_error"]
                                - 0.08 * measured["lateral_error"], -0.5, 0.5))
        speed = float(np.linalg.norm(self.data.qvel[:2]))
        # The attempt continues until an observed collision or finish. A hidden
        # fault parameter or storyboard time can never trigger its conclusion.
        stop = self._barrier_contact_time is not None or self._finish_time is not None
        return {"steering": steering,
                "throttle": 0.0 if stop else float(np.clip((self.config.probe_speed - speed) * 0.25, 0, 0.4)),
                "brake": 0.65 if stop else 0.0}

    def step(self, control: dict | None = None):
        command = self._nominal_command() if control is None else self._validate_control(control)
        if self.finished:
            return
        self._last_command = command
        if command["brake"] > 0 and not self._brake_started:
            self._brake_started = True
            self._events.append({"event": "brake_onset", "time": self.elapsed,
                                 "command": dict(command)})
        self._apply_controls(command)
        pre_step_speed = float(np.linalg.norm(self.data.qvel[:2]))
        mujoco.mj_step(self.model, self.data)
        self._steps += 1
        self.elapsed = self._steps * self.config.timestep
        self.data.time = self.elapsed
        mujoco.mj_forward(self.model, self.data)
        touching_rail = False
        for index, contact in enumerate(self.data.contact):
            pair = {int(contact.geom1), int(contact.geom2)}
            if not pair.intersection(self._roadside_geoms) or not pair.intersection(self._vehicle_geoms):
                continue
            force = np.zeros(6)
            mujoco.mj_contactForce(self.model, self.data, index, force)
            if force[0] <= 0:
                continue
            touching_rail = True
            if self._barrier_contact_time is None:
                self._barrier_contact_time = self.elapsed
                self._barrier_impact_speed = pre_step_speed
                self._end_time = min(self._end_time, self.elapsed + 2.0)
                self._events.append({"event": "barrier_contact", "time": self.elapsed,
                                     "impact_speed": pre_step_speed, "normal_force": float(force[0]),
                                     "position": self.data.qpos[:3].tolist(),
                                     "contact_position": contact.pos.tolist()})
        if touching_rail and not self._touching_rail:
            self._barrier_contact_count += 1
        self._touching_rail = touching_rail
        measured = self._route_measurements()
        self._distance += float(np.linalg.norm(self.data.qpos[:2] - self._previous_xy))
        self._previous_xy = self.data.qpos[:2].copy()
        self._max_lateral_error = max(self._max_lateral_error, abs(measured["lateral_error"]))
        if self.elapsed - self._trial_start >= 2.0:
            self._max_heading_error = max(self._max_heading_error, abs(measured["heading_error"]))
        self._max_tire_departure = max(self._max_tire_departure, max(measured["tire_departure"]))
        if not self._bend_entered and self.data.qpos[0] >= self.config.bend_start:
            self._bend_entered = True
            self._events.append({"event": "bend_entry", "time": self.elapsed,
                                 "position": self.data.qpos[:3].tolist()})
        if measured["any_tire_outside"]:
            if self._outside_since is None:
                self._outside_since = self.elapsed
                self._outside_start_step = self._steps
                self._events.append({"event": "tire_boundary_crossing", "time": self.elapsed,
                                     "position": self.data.qpos[:3].tolist()})
            outside_duration = (self._steps - self._outside_start_step + 1) * self.config.timestep
            self._max_outside_duration = max(self._max_outside_duration, outside_duration)
            if self._failure_time is None and outside_duration >= 0.5 - 1e-9:
                self._failure_time = self.elapsed
                self._events.append({"event": "lane_exit", "time": self.elapsed,
                                     "outside_duration": outside_duration,
                                     "tire_departure": measured["tire_departure"],
                                     "position": self.data.qpos[:3].tolist()})
        else:
            self._outside_since = None
        if (self._finish_time is None and self._failure_time is None
                and self._barrier_contact_time is None
                and self._max_tire_departure == 0 and self._max_lateral_error <= .30
                and self._max_heading_error < math.radians(5)
                and self.data.qpos[0] >= self.config.finish_x):
            self._finish_time = self.elapsed
            self._end_time = min(self._end_time, self.elapsed + 2.0)
            self._events.append({"event": "finish_line", "time": self.elapsed,
                                 "position": self.data.qpos[:3].tolist()})
        if self.finished:
            self._events.append({"event": "outcome", "time": self.elapsed,
                                 "outcome": self._outcome()})

    def _outcome(self) -> str:
        if self._failure_time is not None or self._barrier_contact_time is not None:
            return "lane_departure"
        if self._finish_time is not None:
            within_bounds = (self._max_tire_departure == 0 and self._max_lateral_error <= 0.30
                             and self._max_heading_error < math.radians(5)
                             and self._finish_time - self._trial_start <= 15)
            return "course_complete" if within_bounds else "course_outside_envelope"
        return "incomplete_course"

    def observe(self) -> dict:
        observation = super().observe()
        observation.update(self._route_measurements())
        observation["phase"] = ("collision_aftermath" if self._barrier_contact_time is not None
                                else "lane_departure" if self._failure_time is not None
                                else "finish_hold" if self._finish_time is not None
                                else "bend" if self._bend_entered else "straight_entry")
        observation["speed"] = float(np.linalg.norm(self.data.qvel[:2]))
        observation["barrier_contact"] = self._barrier_contact_time is not None
        observation["impact_speed"] = self._barrier_impact_speed
        observation["goal_reached"] = self._finish_time is not None
        return observation

    def presentation(self) -> dict:
        objective = "Reach the green finish gate"
        if self._barrier_contact_time is not None:
            return {"objective": objective, "status": "FAILED: ROADSIDE COLLISION",
                    "detail": f"Impact {self._barrier_impact_speed:.1f} m/s; finish gate not reached"}
        if self._finish_time is not None:
            return {"objective": objective, "status": "FINISH GATE REACHED",
                    "detail": "Course complete; no roadside contact"}
        if self._failure_time is not None:
            return {"objective": objective, "status": "OFF COURSE: LANE EXIT",
                    "detail": "Outside lane; finish gate not reached"}
        if self.finished:
            return {"objective": objective, "status": "INCOMPLETE: GATE NOT REACHED",
                    "detail": "The attempt ended before the finish gate"}
        return {"objective": objective,
                "status": "FOLLOWING THE BEND" if self._bend_entered else "APPROACHING THE BEND",
                "detail": f"{max(0.0, self.config.finish_x - self.data.qpos[0]):.0f} m to finish; stay inside the white lines"}

    def diagnostics(self) -> dict:
        return {"fault": self.config.fault, "rack_gain": self._rack_gain,
                "rack_bias": self._rack_bias, "events": self.public_events,
                "failure_time": self._failure_time, "finish_time": self._finish_time,
                "tire_radius": self.model.geom_size[self._tire, 0].tolist(),
                "tire_friction": self.model.geom_friction[self._tire, 0].tolist(),
                "spring_stiffness": self.model.jnt_stiffness[self._suspension].tolist()}

    def summary(self) -> dict:
        metrics = {"max_lateral_error": self._max_lateral_error,
                   "max_heading_error_after_2s": self._max_heading_error,
                   "max_tire_departure": self._max_tire_departure,
                   "max_outside_duration": self._max_outside_duration,
                   "distance_traveled": self._distance,
                   "final_speed": float(np.linalg.norm(self.data.qvel[:2])),
                   "progress": self._route_measurements()["progress"],
                   "barrier_contact": self._barrier_contact_time is not None,
                   "barrier_contact_time": self._barrier_contact_time,
                   "barrier_contact_count": self._barrier_contact_count,
                   "impact_speed": self._barrier_impact_speed,
                   "goal_reached": self._finish_time is not None,
                   "end_time": self.elapsed,
                   "failure_time": self._failure_time, "finish_time": self._finish_time}
        return {"public": {"platform": "car_steering", "outcome": self._outcome(),
                           "safe": self._outcome() == "course_complete", "metrics": metrics},
                "config": asdict(self.config), "events": self.public_events,
                "diagnostics": self.diagnostics(),
                "warnings": (self._warning_counts + self.data.warning.number).tolist()}
