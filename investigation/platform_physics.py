"""Trusted maintenance experiments with a neutral, numeric agent boundary.

The platform modules and incident presets remain private to this adapter. Repairs
are explicit synthetic component replacements, not source edits or fault-label
queries. Hypothesized models run independently from a known nominal fixture.
"""

from copy import deepcopy
from dataclasses import replace
import json
import math
from pathlib import Path
from uuid import uuid4

import mujoco
import numpy as np
from PIL import Image

from simulator.platforms import car_damage, drone, quadruped


WHEELS = ("FL", "FR", "RL", "RR")
ROTORS = ("FL", "FR", "RR", "RL")
MODULES = {"car": car_damage, "drone": drone, "quadruped": quadruped}
DEFAULTS = {"car": "car_wheel_misalignment", "drone": "drone_rotor_loss",
            "quadruped": "quadruped_joint_weakness"}
SCENARIOS = {
    "car": ("car_postcrash_healthy", "car_steering_damage", "car_wheel_misalignment",
            "car_suspension_damage", "car_tire_pressure"),
    "drone": ("drone_hover", "drone_rotor_loss", "drone_voltage_sag", "drone_payload", "drone_wind", "drone_delay"),
    "quadruped": ("quadruped_walk", "quadruped_joint_weakness", "quadruped_foot_slip",
                  "quadruped_leg_damage", "quadruped_payload_shift"),
}
PROBES = {"car": ("steering", "braking", "slalom", "bump"), "drone": ("hover", "maneuver"),
          "quadruped": ("walk", "stand", "turn", "conservative", "passive")}
DEFAULT_PROBES = {"car": "slalom", "drone": "maneuver", "quadruped": "walk"}


def _number(value, low, high, label):
    if (isinstance(value, bool) or not isinstance(value, (int, float)) or
            not math.isfinite(value) or not low <= value <= high):
        raise ValueError(f"{label} must be a finite number in [{low}, {high}].")
    return float(value)


def _public(value):
    """Copy sensor values and trim numeric payload precision, never engine objects."""
    if isinstance(value, dict):
        return {str(key): _public(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_public(item) for item in value]
    if isinstance(value, (float, np.floating)):
        if not math.isfinite(value):
            raise ValueError("A sensor returned a non-finite measurement.")
        return round(float(value), 6)
    if isinstance(value, np.integer):
        return int(value)
    if value is None or isinstance(value, (bool, int, str)):
        return value
    raise ValueError("A sensor returned an unsupported value.")


def _sample(rows, limit=200):
    if len(rows) <= limit:
        return rows
    keep = {0, len(rows) - 1}
    for index in range(1, len(rows)):
        if rows[index].get("phase") != rows[index - 1].get("phase"):
            keep.update((index - 1, index))
    remaining = [index for index in range(len(rows)) if index not in keep]
    slots = limit - len(keep)
    if slots > 0:
        keep.update(remaining[index] for index in np.linspace(0, len(remaining) - 1, slots, dtype=int))
    return [rows[index] for index in sorted(keep)[:limit]]


class PlatformPhysics:
    default_duration_s = 10.0

    def __init__(self, platform, workdir, scenario=None, record_frames=True):
        if platform not in MODULES:
            raise ValueError("Choose platform car, drone, or quadruped.")
        self.platform = platform
        self.scenario = scenario or DEFAULTS[platform]  # Trusted host only.
        if self.scenario not in SCENARIOS[platform]:
            raise ValueError("The host scenario is not available for this platform.")
        self.workdir = Path(workdir)
        self.workdir.mkdir(parents=True, exist_ok=True)
        self.record_frames = bool(record_frames)
        self.default_probe = DEFAULT_PROBES[platform]
        self._module = MODULES[platform]
        settings = dict(self._module.PRESETS[self.scenario])
        settings.update(duration=12.0, timestep=0.002,
                        probe="bump" if platform == "car" else self.default_probe)
        self._actual = self._module.Simulation(self._module.Config(**settings))
        self._nominal = self._new_nominal()
        self._incident_ready = False
        self._initial = None
        self._receipts = []
        self._specs = {}
        self._components = []
        self._actions = []
        self._build_catalog()

    def _new_nominal(self):
        # Do not derive this Config from the damaged Config, including severity,
        # affected component, onset, or any actual dynamic state.
        return self._module.Simulation(self._module.Config(
            fault="healthy", duration=12.0, timestep=0.002,
            probe="bump" if self.platform == "car" else self.default_probe))

    def _spec(self, key, nominal, low, high, component, units=None):
        if units is None:
            units = "N m/rad" if key.endswith("_stiffness") else next(
                (unit for suffix, unit in (("_rad", "rad"), ("_s", "s"), ("_kg", "kg"),
                                           ("_n", "N"), ("_m", "m")) if key.endswith(suffix)),
                "dimensionless")
        self._specs[key] = {"nominal": float(nominal), "min": float(low), "max": float(high),
                            "component_id": component, "units": units}

    def _component(self, name, kind):
        self._components.append({"id": name, "kind": kind})

    def _action(self, name, targets, description):
        self._actions.append({"action": name, "targets": list(targets), "description": description})

    def _build_catalog(self):
        model = self._nominal.model
        if self.platform == "car":
            self._component("steering_rack", "steering")
            self._spec("steering_gain", 1, 0, 1.5, "steering_rack")
            self._spec("steering_bias_rad", 0, -.3, .3, "steering_rack")
            targets = [f"wheel_{wheel}" for wheel in WHEELS]
            for wheel, target in zip(WHEELS, targets):
                self._component(target, "wheel_assembly")
                tire = model.geom(f"tire_{wheel}")
                self._spec(f"toe_{wheel}_rad", 0, -.5, .5, target)
                self._spec(f"spring_{wheel}_scale", 1, .05, 1.5, target)
                self._spec(f"tire_{wheel}_radius_scale", 1, .6, 1.1, target)
                self._spec(f"tire_{wheel}_friction", tire.friction[0], .05, 1.5, target)
                self._spec(f"tire_{wheel}_contact_s", tire.solref[0], .008, .1, target)
            self._action("calibrate_steering", ["steering_rack"], "Restore nominal rack gain and center.")
            self._action("align_wheel", targets, "Restore the selected wheel mount alignment.")
            self._action("service_suspension", targets, "Replace the selected spring and damper with nominal parts.")
            self._action("replace_tire", targets, "Restore selected tire radius, contact compliance, and grip.")
            self._action("replace_wheel", targets, "Replace the selected complete wheel assembly: mount alignment, suspension, and tire. This does not reconnect detached wheels.")
        elif self.platform == "drone":
            for rotor in ROTORS:
                self._component(f"rotor_{rotor}", "rotor")
                self._spec(f"rotor_{rotor}_gain", 1, 0, 1.5, f"rotor_{rotor}")
            for name in ("battery", "payload", "command_link", "environment"):
                self._component(name, name)
            self._spec("voltage_ratio", 1, .1, 1, "battery")
            self._spec("payload_kg", 0, 0, 5, "payload")
            self._spec("wind_x_n", 0, -20, 20, "environment")
            self._spec("delay_s", 0, 0, 1, "command_link")
            self._action("replace_rotor", [f"rotor_{rotor}" for rotor in ROTORS], "Restore one nominal rotor's thrust capacity.")
            self._action("replace_battery", ["battery"], "Install a nominal charged supply.")
            self._action("unload_payload", ["payload"], "Remove the external package and its mass/inertia contribution.")
            self._action("service_command_link", ["command_link"], "Restore nominal command transport.")
            self._action("shelter_from_wind", ["environment"], "Environmental intervention: remove the imposed crosswind.")
        else:
            for joint in quadruped.JOINTS:
                self._component(joint, "joint")
                index = model.joint(joint).id
                actuator = quadruped.JOINTS.index(joint)
                self._spec(f"motor_{joint}_gain", model.actuator_gainprm[actuator, 0], 0, 1.5, joint)
                self._spec(f"joint_{joint}_stiffness", model.jnt_stiffness[index], 0, 160, joint)
                self._spec(f"joint_{joint}_rest_rad", model.qpos_spring[model.jnt_qposadr[index]], -2.8, 2.8, joint)
            for leg in WHEELS:
                target = f"foot_{leg}"
                self._component(target, "foot")
                self._spec(f"foot_{leg}_friction", model.geom(f"{leg}_foot").friction[0], .001, 1.5, target)
            self._component("payload", "movable_payload")
            self._spec("payload_offset_m", 0, -.3, .3, "payload")
            self._action("replace_actuator", quadruped.JOINTS, "Restore the selected nominal joint motor.")
            self._action("service_joint", quadruped.JOINTS, "Restore the selected joint's nominal spring and rest geometry.")
            self._action("replace_footpad", [f"foot_{leg}" for leg in WHEELS], "Restore the selected foot's nominal contact grip.")
            self._action("secure_payload", ["payload"], "Return the movable payload's fixture target to nominal center.")
        for component in self._components:
            component["nominal_parameters"] = {key: spec["nominal"] for key, spec in self._specs.items()
                                                if spec["component_id"] == component["id"]}

    def capabilities(self):
        sensors = list(self._nominal.observe())
        ordering = {"car": {"wheel_order": list(WHEELS)}, "drone": {"rotor_order": list(ROTORS)},
                    "quadruped": {"leg_order": list(WHEELS), "joint_order": list(quadruped.JOINTS)}}
        return deepcopy({"platform": self.platform, "components": self._components,
                         "model_parameters": self._specs, "probes": list(PROBES[self.platform]),
                         "repair_actions": self._actions, "sensors": sensors,
                         "sensor_order": ordering[self.platform],
                         "measurement_units": "Seconds, meters, meters/second, radians, radians/second, m/s^2; explicitly named degree sensors use degrees. Quaternions are w,x,y,z.",
                         "default_probe": self.default_probe, "default_duration_s": self.default_duration_s,
                         "duration_s": {"min": 2.0, "max": 20.0},
                         "specimen_policy": "Each probe explicitly repositions motion and controller state; completed damage and maintenance persist.",
                         "model_policy": "Models start independently from the nominal fixture. Partial parameter maps default unspecified values to nominal; updates use only the model's own sensors.",
                         "maintenance_policy": "A valid receipt confirms an intervention, not a diagnosis or successful repair. Verify with a completed probe.",
                         "safety_scope": "Synthetic probe outcomes only; no hardware safety certification."})

    def _validate_probe(self, probe, duration_s):
        if probe not in PROBES[self.platform]:
            raise ValueError("Choose one of the declared diagnostic probes.")
        return _number(duration_s, 2, 20, "duration_s")

    def _parameters(self, parameters):
        if not isinstance(parameters, dict) or set(parameters) - set(self._specs):
            raise ValueError("Model parameters must use only the declared numeric keys.")
        values = {key: spec["nominal"] for key, spec in self._specs.items()}
        for key, value in parameters.items():
            spec = self._specs[key]
            values[key] = _number(value, spec["min"], spec["max"], key)
        return values

    @staticmethod
    def _check(sim):
        if (not np.isfinite(sim.data.qpos).all() or not np.isfinite(sim.data.qvel).all() or
                sim.data.warning.number.any()):
            raise RuntimeError("The physical probe could not complete with valid numerical state.")

    def _prepare_actual(self):
        if self._incident_ready:
            return
        sim = self._actual
        maximum_steps = math.ceil(sim.config.duration / sim.config.timestep)
        for _ in range(maximum_steps):
            if self.platform == "car":
                if sim.observe()["phase"] == "controlled_probe":
                    break
            elif sim.elapsed >= sim.config.fault_at + sim.config.timestep / 2:
                break
            if sim.finished:
                raise RuntimeError("The specimen preparation could not complete.")
            sim.step()
            self._check(sim)
        else:
            raise RuntimeError("The specimen preparation could not complete.")
        self._incident_ready = True

    def _configure_probe(self, sim, probe, duration_s):
        sim.config = replace(sim.config, probe=probe, duration=duration_s)
        if self.platform == "car":
            bump = sim.model.geom("probe_bump")
            nominal = self._nominal.model.geom("probe_bump")
            bump.contype[:] = nominal.contype if probe == "bump" else 0
            bump.conaffinity[:] = nominal.conaffinity if probe == "bump" else 0
            bump.rgba[3] = nominal.rgba[3] if probe == "bump" else 0
        sim.reset_trial()
        self._check(sim)

    @staticmethod
    def _constants(sim):
        # mj_setConst writes its data argument at qpos0: use scratch data so
        # parameter updates never overwrite the simulated pose or velocity.
        mujoco.mj_setConst(sim.model, mujoco.MjData(sim.model))
        mujoco.mj_forward(sim.model, sim.data)

    def _payload(self, sim, payload):
        mass = drone.NOMINAL_MASS + payload
        offset = -.08
        com = payload * offset / mass
        half_size = np.array([.09, .075, .035])
        inertia = drone.NOMINAL_INERTIA + payload / 3 * np.array([
            half_size[1] ** 2 + half_size[2] ** 2, half_size[0] ** 2 + half_size[2] ** 2,
            half_size[0] ** 2 + half_size[1] ** 2])
        inertia[:2] += drone.NOMINAL_MASS * com ** 2 + payload * (offset - com) ** 2
        sim.model.body_mass[sim.focus_body] = mass
        sim.model.body_inertia[sim.focus_body] = inertia
        sim.model.body_ipos[sim.focus_body] = [0, 0, com]
        sim.model.geom("payload_visual").rgba[3] = float(payload > 0)

    def _apply_parameters(self, sim, values):
        model, nominal = sim.model, self._nominal.model
        if self.platform == "car":
            sim._rack_gain, sim._rack_bias = values["steering_gain"], values["steering_bias_rad"]
            for wheel in WHEELS:
                angle = values[f"toe_{wheel}_rad"]
                model.body(f"toe_{wheel}").quat[:] = [math.cos(angle / 2), 0, 0, math.sin(angle / 2)]
                joint = model.joint(f"suspension_{wheel}").id
                dof = model.jnt_dofadr[joint]
                scale = values[f"spring_{wheel}_scale"]
                model.jnt_stiffness[joint] = nominal.jnt_stiffness[joint] * scale
                model.dof_damping[dof] = nominal.dof_damping[dof] * math.sqrt(scale)
                tire, base = model.geom(f"tire_{wheel}"), nominal.geom(f"tire_{wheel}")
                tire.size[[0, 2]] = base.size[[0, 2]] * values[f"tire_{wheel}_radius_scale"]
                tire.friction[0] = values[f"tire_{wheel}_friction"]
                tire.solref[0] = values[f"tire_{wheel}_contact_s"]
        elif self.platform == "drone":
            sim.effectiveness[:] = [values[f"rotor_{rotor}_gain"] for rotor in ROTORS]
            sim.voltage = values["voltage_ratio"]
            sim.wind[:] = [values["wind_x_n"], 0, 0]
            sim.delay = values["delay_s"]
            self._payload(sim, values["payload_kg"])
        else:
            for index, joint in enumerate(quadruped.JOINTS):
                ident = model.joint(joint).id
                model.actuator_gainprm[index, 0] = values[f"motor_{joint}_gain"]
                model.jnt_stiffness[ident] = values[f"joint_{joint}_stiffness"]
                model.qpos_spring[model.jnt_qposadr[ident]] = values[f"joint_{joint}_rest_rad"]
            for leg in WHEELS:
                model.geom(f"{leg}_foot").friction[0] = values[f"foot_{leg}_friction"]
            model.qpos_spring[sim._payload_qa] = values["payload_offset_m"]
        self._constants(sim)

    def _observation(self, sim, origin):
        result = _public(sim.observe())
        result["t_s"] = round(sim.elapsed - origin, 6)
        result["phase_time"] = result["t_s"]
        return result

    def _record(self, sim, probe, duration_s, kind, parameter_model=None, parameters=None):
        duration_s = self._validate_probe(probe, duration_s)
        # Initial hypothesized parameters shape the declared preparation itself.
        # A stateful model gets a dt=0 initial sensor query, then elapsed feedback.
        initial_parameters = self._parameters(parameters or {}) if kind == "model" else None
        if parameter_model is not None:
            initial_parameters = self._parameters(parameter_model(self._observation(sim, sim.elapsed), 0.0))
        if initial_parameters is not None:
            self._apply_parameters(sim, initial_parameters)
        self._configure_probe(sim, probe, duration_s)
        origin = sim.elapsed
        identifier = "run_" + uuid4().hex[:16]
        directory = self.workdir / identifier
        directory.mkdir(exist_ok=False)
        frames, observations = [], []
        renderer = None
        last_parameters = initial_parameters
        model_tick = observation_tick = frame_tick = 0
        previous_model_time = 0.0
        try:
            if self.record_frames:
                (directory / "frames").mkdir()
                sim.model.vis.global_.offwidth = max(800, sim.model.vis.global_.offwidth)
                sim.model.vis.global_.offheight = max(360, sim.model.vis.global_.offheight)
                renderer = mujoco.Renderer(sim.model, width=800, height=360)

            def capture(final=False):
                nonlocal observation_tick, frame_tick
                elapsed = sim.elapsed - origin
                if final or elapsed + 1e-9 >= observation_tick / 50:
                    observation = self._observation(sim, origin)
                    if not observations or observation != observations[-1]:
                        observations.append(observation)
                    observation_tick = math.floor((elapsed + 1e-9) * 50) + 1
                if renderer is not None and (final or elapsed + 1e-9 >= frame_tick / 15):
                    if not frames or abs(frames[-1]["t_s"] - elapsed) > 1e-8:
                        renderer.update_scene(sim.data, camera="side" if self.platform == "car" else "chase")
                        relative = Path(identifier) / "frames" / f"frame_{len(frames):06d}.jpg"
                        Image.fromarray(renderer.render()).save(self.workdir / relative, quality=82)
                        frames.append({"t_s": round(elapsed, 6), "file": relative.as_posix()})
                    frame_tick = math.floor((elapsed + 1e-9) * 15) + 1

            capture()
            for _ in range(math.ceil(duration_s / sim.config.timestep) + 2):
                if sim.finished:
                    break
                elapsed = sim.elapsed - origin
                if parameter_model is not None and elapsed + 1e-9 >= model_tick / 50:
                    supplied = parameter_model(self._observation(sim, origin), max(0.0, elapsed - previous_model_time))
                    updated = self._parameters(supplied)
                    if updated != last_parameters:
                        self._apply_parameters(sim, updated)
                        last_parameters = updated
                    previous_model_time = elapsed
                    model_tick = math.floor((elapsed + 1e-9) * 50) + 1
                sim.step()
                self._check(sim)
                capture()
            if not sim.finished:
                raise RuntimeError("The probe exceeded its declared duration.")
            capture(final=True)
            summary = deepcopy(sim.summary()["public"])
            fall_time = summary.get("metrics", {}).get("fall_time")
            if fall_time is not None:
                summary["metrics"]["fall_time"] = fall_time - origin
            record = {"id": identifier, "platform": self.platform, "kind": kind, "probe": probe,
                      "duration_s": duration_s, "summary": summary, "observations": _sample(observations),
                      "frames": frames, "maintenance": deepcopy(self._receipts) if kind == "observed" else [],
                      "fixture": "Declared reposition and nominal controller preparation; persistent physical component properties retained."}
            if kind == "model":
                record["model_parameters"] = last_parameters
                record["parameter_updates"] = "own-sensor callback at 50 Hz" if parameter_model else "fixed parameter hypothesis"
            (directory / "record.json").write_text(json.dumps(record, indent=2, allow_nan=False) + "\n")
            return deepcopy(record)
        finally:
            if renderer is not None:
                renderer.close()

    def initial_evidence(self):
        if self._initial is None:
            self._prepare_actual()
            healthy = self.reference_probe(self.default_probe, self.default_duration_s)
            observed = self.run_experiment(self.default_probe, self.default_duration_s)
            self._initial = {"healthy": healthy, "observed": observed}
        return deepcopy(self._initial)

    def observe(self, component_id=None):
        selected = [item for item in self._components if component_id is None or item["id"] == component_id]
        if not selected:
            raise ValueError("Choose a declared component ID.")
        self._prepare_actual()
        return {"platform": self.platform, "components": deepcopy(selected),
                "measurements": _public(self._actual.observe()), "maintenance": deepcopy(self._receipts)}

    def run_experiment(self, probe, duration_s):
        self._validate_probe(probe, duration_s)
        self._prepare_actual()
        return self._record(self._actual, probe, duration_s, "observed")

    def reference_probe(self, probe, duration_s):
        self._validate_probe(probe, duration_s)
        return self._record(self._new_nominal(), probe, duration_s, "healthy")

    def run_model(self, probe, duration_s, parameters):
        self._validate_probe(probe, duration_s)
        if callable(parameters):
            return self._record(self._new_nominal(), probe, duration_s, "model", parameter_model=parameters)
        values = self._parameters(parameters)
        return self._record(self._new_nominal(), probe, duration_s, "model", parameters=values)

    def regression_cases(self):
        choices = {"car": ("slalom", "braking", "bump"), "drone": ("hover", "maneuver"),
                   "quadruped": ("walk", "stand", "turn")}
        return [{"probe": probe, "duration_s": self.default_duration_s} for probe in choices[self.platform]]

    def apply_repair(self, action, target):
        match = next((entry for entry in self._actions if entry["action"] == action), None)
        if match is None or target not in match["targets"]:
            raise ValueError("Choose a declared maintenance action and target.")
        self._prepare_actual()
        sim, nominal = self._actual, self._nominal.model
        model = sim.model
        if self.platform == "car":
            if action == "calibrate_steering":
                sim._rack_gain, sim._rack_bias = 1.0, 0.0
            else:
                wheel = target.removeprefix("wheel_")
                if action in {"align_wheel", "replace_wheel"}:
                    model.body(f"toe_{wheel}").quat[:] = nominal.body(f"toe_{wheel}").quat
                if action in {"service_suspension", "replace_wheel"}:
                    joint = model.joint(f"suspension_{wheel}").id
                    dof = model.jnt_dofadr[joint]
                    model.jnt_stiffness[joint] = nominal.jnt_stiffness[joint]
                    model.dof_damping[dof] = nominal.dof_damping[dof]
                if action in {"replace_tire", "replace_wheel"}:
                    tire, base = model.geom(f"tire_{wheel}"), nominal.geom(f"tire_{wheel}")
                    tire.size[:], tire.friction[:], tire.solref[:] = base.size, base.friction, base.solref
        elif self.platform == "drone":
            if action == "replace_rotor":
                sim.effectiveness[ROTORS.index(target.removeprefix("rotor_"))] = 1
            elif action == "replace_battery":
                sim.voltage = 1
            elif action == "unload_payload":
                self._payload(sim, 0)
            elif action == "service_command_link":
                sim.delay = 0
            elif action == "shelter_from_wind":
                sim.wind[:] = 0
        else:
            if action in {"replace_actuator", "service_joint"}:
                index = quadruped.JOINTS.index(target)
                joint = model.joint(target).id
                if action == "replace_actuator":
                    model.actuator_gainprm[index, :] = nominal.actuator_gainprm[index, :]
                else:
                    model.jnt_stiffness[joint] = nominal.jnt_stiffness[joint]
                    address = model.jnt_qposadr[joint]
                    model.qpos_spring[address] = nominal.qpos_spring[address]
            elif action == "replace_footpad":
                leg = target.removeprefix("foot_")
                model.geom(f"{leg}_foot").friction[:] = nominal.geom(f"{leg}_foot").friction
            elif action == "secure_payload":
                model.qpos_spring[sim._payload_qa] = nominal.qpos_spring[sim._payload_qa]
        self._constants(sim)
        receipt = {"id": "maintenance_" + uuid4().hex[:12], "action": action, "target": target,
                   "status": "applied", "verification_required": True,
                   "message": "The requested intervention was performed. Run a probe to measure its effect."}
        self._receipts.append(receipt)
        return deepcopy(receipt)
