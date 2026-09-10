"""Physical parcel delivery: deliberately incomplete load control, replayable impact.

The developer feasibility controller is a calibration reference, not an Astra
repair. Both controllers act exclusively through the same four rotor motors.
"""

from dataclasses import asdict
import math
from pathlib import Path

import mujoco
import numpy as np

from .drone import Config, INVERSE_MIXER, MAX_ROTOR_THRUST, NOMINAL_MASS, NOMINAL_MOTOR_TAU, Simulation


IMPACT_SPEED_THRESHOLD = 1.0
EFFECT_SEED = 2048
PARCEL_DROP = .20
PARCEL_HALF_HEIGHT = .09


def _ease(value: float) -> float:
    value = float(np.clip(value, 0, 1))
    return value * value * (3 - 2 * value)


class DeliverySimulation(Simulation):
    """A separate asset and mission preserve every legacy drone preset."""

    def __init__(self, config: Config):
        if config.fault not in ("healthy", "payload"):
            raise ValueError("delivery supports only healthy (centered parcel) or payload (offset parcel)")
        if not .05 <= config.payload_mass <= .65:
            raise ValueError("delivery payload_mass must be in [.05,.65] kg")
        self.config = config
        self.model = mujoco.MjModel.from_xml_path(str(
            Path(__file__).resolve().parents[1] / "assets/platforms/drone_delivery.xml"))
        self.model.opt.timestep = config.timestep
        self.focus_body = self.model.body("drone").id
        self.parcel_body = self.model.body("parcel").id
        self.ground = self.model.geom("ground").id
        self.parcel_geom = self.model.geom("parcel_box").id
        self.latch = self.model.equality("parcel_latch").id
        self.parcel_qpos = int(self.model.joint("parcel_free").qposadr[0])
        self.parcel_dof = int(self.model.joint("parcel_free").dofadr[0])
        self.offset = np.array([config.payload_offset if config.fault == "payload" else 0., 0., -PARCEL_DROP])
        self.model.body_mass[self.parcel_body] = config.payload_mass
        self.model.body_inertia[self.parcel_body] *= config.payload_mass / .36
        self.model.body_pos[self.parcel_body] = self.model.body_pos[self.focus_body] + self.offset
        self.model.qpos0[self.parcel_qpos:self.parcel_qpos + 3] = self.model.body_pos[self.parcel_body]
        self.model.eq_data[self.latch, 3:6] = self.offset
        mount = self.model.geom("parcel_mount")
        mount.pos[:] = self.offset / 2
        mount.size[1] = np.linalg.norm(self.offset) / 2
        mujoco.mju_quatZ2Vec(mount.quat, self.offset / np.linalg.norm(self.offset))
        self._configure_scene()
        self.data = mujoco.MjData(self.model)
        mujoco.mj_setConst(self.model, self.data)
        self.camera_distance = 7.
        self.reset_full()

    def _configure_scene(self):
        distance = self.config.flight_distance
        self.model.geom("destination_pad").pos[0] = distance
        self.model.geom("delivery_route").pos[0] = distance / 2
        self.model.geom("delivery_route").size[0] = distance / 2
        for index in range(1, 4):
            self.model.geom(f"delivery_distance_{index}").pos[0] = distance * index / 4
        for index in range(5):
            self.model.geom(f"pad_B_{index}").pos[0] += distance - 4
        for name, position, target, fovy in (
            ("side", [distance / 2 + .65, -8.5, 3.5], [distance / 2 + .65, 0., 1.1], 27),
            ("overview", [distance / 2 + 4.5, -6.8, 5.5], [distance / 2 + .55, 0., 1.0], 32),
        ):
            camera = self.model.camera(name)
            camera.pos[:] = position
            backward = np.array(position) - target
            backward /= np.linalg.norm(backward)
            right = np.cross([0, 0, 1], backward)
            right /= np.linalg.norm(right)
            rotation = np.column_stack((right, np.cross(backward, right), backward))
            mujoco.mju_mat2Quat(camera.quat, rotation.ravel())
            camera.fovy[0] = fovy

    def reset_full(self):
        self.elapsed = 0.
        self.events: list[dict] = []
        self._reset_mission()

    def reset_trial(self):
        # Explicit recovery/reset boundary; cargo is reloaded, never resurrected
        # during flight. No damage state is added by the presentation effect.
        self.events = [{"time": self.elapsed, "event": "recovery_reposition",
                        "label": "Reset to A and reload parcel"}]
        self._reset_mission()

    def _reset_mission(self):
        mujoco.mj_resetData(self.model, self.data)
        self.data.time = self.elapsed
        self.data.eq_active[self.latch] = 1
        self.trial_time = 0.
        self.phase = "ready"
        self.phase_started = 0.
        self.target = np.array([0., 0., .30])
        self.last_command = np.zeros(4)
        self.motor_state = np.zeros(4)
        self.hover_trim = np.zeros(3)
        self.effectiveness = np.ones(4)
        self.voltage = 1.
        self.command_mode = "developer_feasibility" if self.config.delivery_controller == "feasibility" else "nominal_delivery"
        self.departed = False
        self.ground_contact = False
        self.crash_event: dict | None = None
        self.release_state: dict | None = None
        self.contact_steps = 0
        self.in_flight_body_contacts = 0
        self.max_tilt = 0.
        self.max_altitude = .30
        self.min_altitude = .30
        self.max_horizontal_excursion = 0.
        self.max_forward_progress = 0.
        self.max_tracking_error = 0.
        self.support_time = 0.
        self.parcel_settled_time = 0.
        self.landed_time = 0.
        self.release_time: float | None = None
        self.completed = False
        self.unloaded_flight = False
        mujoco.mj_forward(self.model, self.data)
        self.events.append({"time": self.elapsed, "event": "mission_ready", "label": "Carry parcel A → B"})

    @property
    def attached(self) -> bool:
        return bool(self.data.eq_active[self.latch])

    @property
    def finished(self) -> bool:
        return (self.trial_time + 1e-10 >= self.config.duration or self.completed
                or (self.crash_event is not None and self.elapsed >= self.crash_event["time"] + 2.5 - 1e-10))

    def _event(self, event: str, label: str, **details):
        record = {"time": float(self.elapsed), "event": event, "label": label, **details}
        if not any(item["event"] == event for item in self.events):
            self.events.append(record)
        return record

    def _enter(self, phase: str, label: str):
        self.phase = phase
        self.phase_started = self.trial_time
        self._event(phase, label)

    def _contacts(self) -> tuple[bool, bool]:
        body_contact = parcel_contact = False
        for contact in self.data.contact:
            if self.ground not in contact.geom:
                continue
            other = int(contact.geom[1] if contact.geom[0] == self.ground else contact.geom[0])
            body_contact |= int(self.model.geom_bodyid[other]) == self.focus_body
            parcel_contact |= int(self.model.geom_bodyid[other]) == self.parcel_body
        return body_contact, parcel_contact

    def release_parcel(self) -> bool:
        """Unlatch only supported, slow cargo at B; retain every pose and velocity.

        The package is an independent free body throughout the run. Releasing
        the weld changes only the constraint's active bit; no mass is removed.
        """
        _, contact = self._contacts()
        parcel_position = self.data.xpos[self.parcel_body]
        speed = np.linalg.norm(self.data.qvel[self.parcel_dof:self.parcel_dof + 3])
        at_destination = np.linalg.norm(parcel_position[:2] - [self.config.flight_distance, 0.]) < .45
        if not self.attached or not contact or speed > .15 or not at_destination:
            return False
        before_position = self.data.qpos.copy()
        before_velocity = self.data.qvel.copy()
        self.data.eq_active[self.latch] = 0
        self.release_state = {"qpos_before": before_position.tolist(), "qvel_before": before_velocity.tolist(),
                              "qpos_after": self.data.qpos.tolist(), "qvel_after": self.data.qvel.tolist(),
                              "contact_supported": True, "parcel_mass": float(self.model.body_mass[self.parcel_body])}
        self.release_time = self.elapsed
        self._event("parcel_released", "Parcel supported at B; latch released", position=parcel_position.tolist())
        return True

    def _mission_target(self) -> np.ndarray:
        elapsed = self.trial_time - self.phase_started
        distance, altitude = self.config.flight_distance, self.config.flight_altitude
        if self.phase == "ready":
            if elapsed >= .6:
                self._enter("takeoff", "Take off with parcel")
            return np.array([0., 0., .30])
        if self.phase == "takeoff":
            target = np.array([0., 0., .30 + (altitude - .30) * _ease(elapsed / 2.4)])
            if elapsed >= 3.3:
                self._enter("outbound", "Fly toward B with parcel")
            return target
        if self.phase == "outbound":
            target = np.array([distance * _ease(elapsed / 4.4), 0., altitude])
            if elapsed >= 5.5 and np.linalg.norm(self.data.qpos[:2] - [distance, 0.]) < .3:
                self._enter("approach", "Descend to place parcel at B")
            return target
        if self.phase in ("approach", "placement"):
            target = np.array([distance - self.offset[0], 0., altitude + (.278 - altitude) * _ease(elapsed / 4.)])
            if self.phase == "placement":
                target[2] = .278
            if self.support_time >= .3 and self.release_parcel():
                self._enter("unloaded_climb", "Parcel delivered; climb without load")
            return target
        if self.phase == "unloaded_climb":
            target = np.array([distance - self.offset[0], 0., .29 + (altitude - .29) * _ease(elapsed / 2.6)])
            if elapsed >= 3.2:
                self._enter("return", "Return unloaded to A")
            return target
        if self.phase == "return":
            target = np.array([(distance - self.offset[0]) * (1 - _ease(elapsed / 4.4)), 0., altitude])
            if elapsed >= 5.5 and np.linalg.norm(self.data.qpos[:2]) < .3:
                self._enter("landing", "Land at A")
            return target
        if self.phase == "landing":
            return np.array([0., 0., altitude + (.278 - altitude) * _ease(elapsed / 3.2)])
        return self.data.qpos[:3].copy()

    def _delivery_control(self, target: np.ndarray) -> np.ndarray:
        """The default lacks load-moment trim; the reference demonstrates feasibility."""
        if self.crash_event is not None or self.phase == "ready" or self.completed:
            return np.zeros(4)
        reference = self.config.delivery_controller == "feasibility"
        rotation = self.data.xmat[self.focus_body].reshape(3, 3)
        acceleration = np.array([3., 3., 8.]) * (target - self.data.qpos[:3])
        acceleration -= np.array([3.2, 3.2, 5.]) * self.data.qvel[:3]
        acceleration = np.clip(acceleration, [-3., -3., -4.], [3., 3., 6.])
        supported = self.phase == "placement"
        mass = NOMINAL_MASS + (self.config.payload_mass if self.attached and not supported else 0.)
        force = mass * (acceleration + [0., 0., 9.81])
        desired_z = force / np.linalg.norm(force)
        desired_y = np.cross(desired_z, [1., 0., 0.])
        desired_y /= np.linalg.norm(desired_y)
        desired = np.column_stack((np.cross(desired_y, desired_z), desired_y, desired_z))
        error_matrix = (desired.T @ rotation - rotation.T @ desired) / 2
        error = np.array([error_matrix[2, 1], error_matrix[0, 2], error_matrix[1, 0]])
        hover_control = self.phase in ("takeoff", "unloaded_climb", "landing")
        gain = 3.5 if reference or hover_control else .9
        torque = -np.array([gain, gain, .8]) * error - np.array([.48, .48, .18]) * self.data.qvel[3:6]
        collective = max(0., float(force @ rotation[:, 2]))
        if not reference and hover_control:
            self.hover_trim = np.clip(self.hover_trim - 2. * error * self.config.timestep, -.95, .95)
            torque += self.hover_trim
        elif not reference:
            # Incomplete mission handoff: route control discards the trim learned
            # by the takeoff loop. A centered package masks this defect.
            self.hover_trim[:] = 0
        if reference and self.attached and not supported:
            com = self.offset * self.config.payload_mass / mass
            torque += np.cross(com, [0., 0., collective])
        torque = np.clip(torque, [-1.4, -1.4, -.15], [1.4, 1.4, .15])
        commands = np.clip((INVERSE_MIXER @ np.concatenate(([collective], torque))) / MAX_ROTOR_THRUST, 0, 1)
        if self.phase == "landing" and self._contacts()[0] and np.linalg.norm(self.data.qvel[:3]) < .15:
            commands[:] = 0
        return commands

    @staticmethod
    def _parse_delivery_control(control: dict | None) -> tuple[str, np.ndarray] | None:
        """Validate before any mission, latch, controller or plant mutation."""
        if control is None:
            return None
        if not isinstance(control, dict) or set(control) not in ({"rotor_commands"}, {"target_position"}):
            raise ValueError("control must contain only rotor_commands or target_position")
        key = next(iter(control))
        try:
            # Keep each input's type until checked: float coercion would accept
            # strings and erase booleans embedded in an otherwise numeric list.
            entries = np.asarray(control[key], dtype=object)
            if any(isinstance(value, (bool, np.bool_))
                   or not isinstance(value, (int, float, np.integer, np.floating))
                   for value in entries.flat):
                raise ValueError("control entries must be numeric, excluding booleans and strings")
            values = np.asarray(entries, dtype=float)
        except (TypeError, ValueError, OverflowError) as error:
            raise ValueError("control entries must be finite numeric values") from error
        if key == "rotor_commands":
            if values.shape != (4,) or not np.isfinite(values).all() or np.any((values < 0) | (values > 1)):
                raise ValueError("rotor_commands must contain four values in [0,1]")
        elif (values.shape != (3,) or not np.isfinite(values).all()
              or np.any(np.abs(values[:2]) > 10) or not .15 <= values[2] <= 5):
            raise ValueError("target_position must be [x,y,z], x/y in [-10,10], z in [.15,5]")
        return key, values.copy()

    def step(self, control: dict | None = None):
        request = self._parse_delivery_control(control)
        target = self._mission_target()
        if request is None:
            command, mode = self._delivery_control(target), self.command_mode
        elif request[0] == "rotor_commands":
            command, mode = request[1], "rotor_commands"
        else:
            target = request[1]
            command, mode = self._delivery_control(target), "target_position"
        dt = self.config.timestep
        self.target, self.last_command, self.command_mode = target, command, mode
        self.motor_state += -math.expm1(-dt / NOMINAL_MOTOR_TAU) * (command - self.motor_state)
        self.data.ctrl[:] = MAX_ROTOR_THRUST * self.motor_state
        self.data.xfrc_applied[:] = 0
        self.data.xfrc_applied[self.focus_body, :3] = -.10 * self.data.qvel[:3]
        self.data.xfrc_applied[self.focus_body, 3:] = -.006 * (
            self.data.xmat[self.focus_body].reshape(3, 3) @ self.data.qvel[3:6])
        incoming_velocity = self.data.qvel[:3].copy()
        mujoco.mj_step(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        self.elapsed += dt
        self.trial_time += dt
        if (not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all()
                or self.data.warning.number.any() or abs(self.data.time - self.elapsed) > dt / 2):
            raise RuntimeError(f"Invalid MuJoCo delivery state: {self.data.warning.number.tolist()}")
        body_contact, parcel_contact = self._contacts()
        altitude, tilt = float(self.data.qpos[2]), self._tilt()
        self.max_altitude = max(self.max_altitude, altitude)
        self.min_altitude = min(self.min_altitude, altitude)
        self.max_tilt = max(self.max_tilt, tilt)
        self.max_horizontal_excursion = max(self.max_horizontal_excursion, float(np.linalg.norm(self.data.qpos[:2])))
        if self.crash_event is None:
            self.max_forward_progress = max(self.max_forward_progress, float(self.data.qpos[0]))
        self.max_tracking_error = max(self.max_tracking_error, float(np.linalg.norm(self.data.qpos[:3] - target)))
        if not self.departed and altitude > .55 and not parcel_contact and not body_contact:
            self.departed = True
            self._event("liftoff", "Parcel and drone clear the ground", position=self.data.qpos[:3].tolist())
        if self.departed and tilt > 30:
            self._event("first_instability", "Large visible tilt", is_failure=True, position=self.data.qpos[:3].tolist())
        if body_contact:
            self.contact_steps += 1
            self.ground_contact = True
            controlled_pad = (self.phase in ("approach", "placement") and abs(self.data.qpos[0] - self.config.flight_distance) < .65) or (self.phase == "landing" and np.linalg.norm(self.data.qpos[:2]) < .5)
            if self.departed and not controlled_pad:
                self.in_flight_body_contacts += 1
        impact_speed = float(np.linalg.norm(incoming_velocity))
        if (self.crash_event is None and self.departed and body_contact
                and impact_speed > IMPACT_SPEED_THRESHOLD and (tilt > 25 or incoming_velocity[2] < -.8)):
            self.crash_event = self._event("crash", "Drone ground impact", is_failure=True,
                                           impact_speed=impact_speed, position=self.data.qpos[:3].tolist())
            self.phase = "crash_aftermath"
        if self.phase == "approach" and parcel_contact and np.linalg.norm(self.data.qvel[:3]) < .2:
            self._enter("placement", "Parcel touches down at B")
        if self.phase in ("approach", "placement") and parcel_contact and np.linalg.norm(self.data.qvel[:3]) < .15:
            self.support_time += dt
        else:
            self.support_time = 0.
        if not self.attached:
            parcel_distance = np.linalg.norm(self.data.xpos[self.parcel_body, :2] - [self.config.flight_distance, 0.])
            parcel_speed = np.linalg.norm(self.data.qvel[self.parcel_dof:self.parcel_dof + 3])
            self.parcel_settled_time = self.parcel_settled_time + dt if parcel_contact and parcel_distance < .5 and parcel_speed < .1 else 0.
            self.unloaded_flight |= altitude > 1.
        if self.phase == "landing" and body_contact and np.linalg.norm(self.data.qpos[:2]) < .5 and np.linalg.norm(self.data.qvel[:3]) < .1:
            self.landed_time += dt
            if self.landed_time >= 1. and self.parcel_settled_time >= 2. and self.in_flight_body_contacts == 0:
                self.completed = True
                self._enter("mission_complete", "Parcel delivered; drone landed at A")
        else:
            self.landed_time = 0.

    def public_events(self) -> list[dict]:
        return [dict(event) for event in self.events]

    def observe(self) -> dict:
        body_contact, parcel_contact = self._contacts()
        return {"time": self.elapsed, "phase": self.phase, "phase_time": self.trial_time,
                "position": self.data.qpos[:3].tolist(), "quaternion": self.data.qpos[3:7].tolist(),
                "velocity": self.data.qvel[:3].tolist(),
                "angular_velocity": self.data.sensor("angular_velocity").data.tolist(),
                "specific_force": self.data.sensor("specific_force").data.tolist(),
                "command": {"mode": self.command_mode, "rotor_commands": self.last_command.tolist(),
                            "target_position": self.target.tolist()},
                "altitude": float(self.data.qpos[2]), "tilt_degrees": self._tilt(),
                "ground_contact": body_contact, "parcel_contact": parcel_contact,
                "parcel_position": self.data.xpos[self.parcel_body].tolist(), "parcel_attached": self.attached,
                "mission": {"origin": [0., 0., 0.], "destination": [self.config.flight_distance, 0., 0.],
                            "completed": self.completed}}

    def diagnostics(self) -> dict:
        return {"time": self.elapsed, "rotor_effectiveness": self.effectiveness.tolist(),
                "voltage_ratio": self.voltage, "motor_state": self.motor_state.tolist(),
                "rotor_thrust": self.data.ctrl.tolist(), "parcel_mass": self.config.payload_mass,
                "parcel_offset": self.offset.tolist(), "release_state": self.release_state,
                "controller": self.config.delivery_controller, "events": self.public_events(),
                "effect": {"impact_event": "crash" if self.crash_event else None, "seed": EFFECT_SEED,
                           "kind": "illustrative flash/sparks/smoke; no combustion physics"},
                "warning_counts": self.data.warning.number.tolist()}

    def summary(self) -> dict:
        outcome = "mission_complete" if self.completed else "crashed" if self.crash_event else "mission_timeout" if self.finished else "incomplete_mission"
        metrics = {"final_position": self.data.qpos[:3].tolist(), "final_altitude": float(self.data.qpos[2]),
                   "final_speed": float(np.linalg.norm(self.data.qvel[:3])),
                   "min_altitude": self.min_altitude, "max_altitude": self.max_altitude,
                   "max_tracking_error": self.max_tracking_error, "max_horizontal_excursion": self.max_horizontal_excursion,
                   "forward_progress_before_impact": self.max_forward_progress, "max_tilt_degrees": self.max_tilt,
                   "ground_contact": self.ground_contact, "contact_steps": self.contact_steps,
                   "in_flight_body_contacts": self.in_flight_body_contacts,
                   "trial_duration": self.trial_time, "parcel_delivered": self.release_time is not None,
                   "parcel_distance_to_B": float(np.linalg.norm(self.data.xpos[self.parcel_body, :2] - [self.config.flight_distance, 0.])),
                   "parcel_settled_seconds": self.parcel_settled_time, "unloaded_flight": self.unloaded_flight,
                   "landed_seconds": self.landed_time, "impact_time": self.crash_event["time"] if self.crash_event else None,
                   "impact_speed": self.crash_event["impact_speed"] if self.crash_event else None}
        public = {"platform": "drone", "task": "delivery", "outcome": outcome, "safe": self.completed,
                  "provenance": "developer_feasibility_control" if self.config.delivery_controller == "feasibility" else "centered_load_control" if self.config.fault == "healthy" else "failure_demonstration",
                  "metrics": metrics}
        return {"public": public, "config": asdict(self.config), "events": self.public_events(),
                "warnings": self.data.warning.number.tolist(), "diagnostics": self.diagnostics()}

    def decorate_scene(self, scene):
        """Append deterministic, impact-only illustration; caller resets the scene.

        Works with renderer.scene and viewer.user_scn. No model/data writes or
        random state advances occur, so repeated renders and seeks are identical.
        """
        if self.crash_event is None:
            return
        age = self.elapsed - self.crash_event["time"]
        if not 0 <= age <= 2.5:
            return
        origin = np.array(self.crash_event["position"])
        origin[2] = max(.08, origin[2])

        def sphere(position, size, rgba):
            if scene.ngeom >= scene.maxgeom:
                return
            mujoco.mjv_initGeom(scene.geoms[scene.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE,
                                np.full(3, size), np.asarray(position), np.eye(3).ravel(), np.asarray(rgba))
            scene.ngeom += 1

        if age < .28:
            sphere(origin, .10 + .35 * math.sin(math.pi * age / .28), [1., .65, .08, .85 * (1 - age / .28)])
        random = np.random.default_rng(EFFECT_SEED)
        for _ in range(14):
            velocity = random.uniform([-2.1, -2.1, 1.], [2.1, 2.1, 3.5])
            position = origin + age * velocity - np.array([0., 0., 3.8 * age * age])
            if age < .75 and position[2] > .025:
                sphere(position, .024 * (1 - age / 1.1), [1., .47, .06, max(0., 1 - age / .75)])
        if age > .12:
            for index in range(5):
                position = origin + [math.sin(index * 2.3) * .10 * age, math.cos(index * 2.3) * .10 * age,
                                     .25 * age + .075 * index]
                sphere(position, .10 + .11 * age, [.24, .25, .28, .22 * (1 - age / 2.5)])
