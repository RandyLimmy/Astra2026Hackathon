"""Bounded cargo model repair with neutral evidence and frozen prediction trials.

The remote investigator can edit a small JSON physical model, never Python, XML,
files, simulator state, or the reference implementation. OpenAI is imported only
by the optional live runner, so preparation/evaluation work entirely offline.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import time
import xml.etree.ElementTree as ET
from copy import deepcopy
from dataclasses import replace
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import numpy as np

import mujoco
from simulator.platforms import warehouse

ROOT = Path(__file__).resolve().parents[1]
SEED = ROOT / "candidate" / "warehouse_model.json"
CONTRACT = ROOT / "contracts" / "WAREHOUSE_MODEL.md"
PUBLIC_PROBES = ("cargo_turn", "cargo_mirror", "cargo_gentle", "cargo_strong")
CURVE_PUBLIC_PROBES = ("cargo_curve", "cargo_curve_slow")
TASK_PROBES = {"rail": PUBLIC_PROBES, "curve": CURVE_PUBLIC_PROBES}
EMPTY_MODEL: dict[str, Any] = {"schema_version": 1, "model_edits": [], "rules": []}
THRESHOLDS = {"affected_min_nominal_rmse_m": .03, "affected_max_candidate_rmse_m": .05,
              "affected_max_error_ratio": .35, "max_heading_rmse_rad": .08,
              "healthy_max_candidate_rmse_m": .015, "healthy_max_degradation_m": .01,
              "minimum_affected_cases": 2}
CURVE_THRESHOLDS = {"minimum_affected_cases": 2, "affected_min_nominal_cargo_rmse_m": .15,
                    "max_candidate_cargo_rmse_m": .08, "affected_max_cargo_error_ratio": .35,
                    "max_candidate_position_rmse_m": .1, "max_heading_rmse_rad": .1,
                    "healthy_max_candidate_cargo_rmse_m": .02,
                    "healthy_max_candidate_position_rmse_m": .015}
LIMITS = {"inspect_model": 3, "run_experiment": 8, "run_model": 12,
          "patch_model": 8, "run_regression_suite": 4, "submit_prediction": 1}


def canonical(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


def digest(value: Any) -> str:
    return hashlib.sha256(canonical(value).encode()).hexdigest()


def save_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def number(value: Any, low: float, high: float, name: str) -> float:
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or not low <= value <= high):
        raise ValueError(f"{name} must be a finite number in [{low}, {high}]")
    return float(value)


def keys(value: Any, expected: set[str], name: str) -> None:
    if not isinstance(value, dict) or set(value) != expected:
        raise ValueError(f"{name} requires exactly {sorted(expected)}")


def public_config(value: Any, *, task: str = "rail") -> warehouse.Config:
    if task not in TASK_PROBES:
        raise ValueError("task must be rail or curve")
    probes = TASK_PROBES[task]
    if not isinstance(value, dict) or set(value) - {"probe", "duration", "drive_scale", "payload_mass"}:
        raise ValueError("config accepts only probe, duration, drive_scale, payload_mass")
    probe = value.get("probe", probes[0])
    if not isinstance(probe, str) or probe not in probes:
        raise ValueError(f"probe must be one of {probes}")
    return warehouse.Config(probe=probe, duration=number(value.get("duration", 11. if task == "rail" else 17.), 1., 12. if task == "rail" else 20., "duration"),
                            drive_scale=number(value.get("drive_scale", 1.), .2, 1., "drive_scale"),
                            payload_mass=number(value.get("payload_mass", 16. if task == "rail" else 8.), 8., 24., "payload_mass"),
                            shift_distance=.28)


def config_dict(config: warehouse.Config) -> dict[str, Any]:
    return {name: getattr(config, name) for name in ("probe", "duration", "drive_scale", "payload_mass")}


def control_identity(config: dict[str, Any]) -> str:
    """Treat every duration/prefix of the same physical intervention as observed."""
    controls = {key: value for key, value in config.items() if key != "duration"}
    if controls.get("probe") in CURVE_PUBLIC_PROBES:
        speed = warehouse.route_geometry()["target_speed_m_s"][controls.pop("probe")]
        controls["target_speed_m_s"] = round(speed * controls.pop("drive_scale"), 10)
    return canonical(controls)


def parse_json(source: str, maximum: int = 32_000) -> Any:
    if not isinstance(source, str) or len(source) > maximum:
        raise ValueError("JSON input is oversized or not text")
    try:
        value = json.loads(source)
        pending = [(value, 0)]
        while pending:
            item, depth = pending.pop()
            if depth > 32:
                raise ValueError("JSON nesting exceeds 32 levels")
            if isinstance(item, dict):
                pending.extend((child, depth + 1) for child in item.values())
            elif isinstance(item, list):
                pending.extend((child, depth + 1) for child in item)
        canonical(value)  # Reject nonfinite constants and numeric overflow such as 1e999.
        return value
    except RecursionError as error:
        raise ValueError("JSON nesting is too deep") from error


def load_candidate(candidate: dict[str, Any] | Path) -> dict[str, Any]:
    if isinstance(candidate, Path):
        if candidate.stat().st_size > 32_000:
            raise ValueError("candidate JSON exceeds 32000 bytes")
        value = parse_json(candidate.read_text())
    else:
        value = deepcopy(candidate)
    # Round-trip rules out NaN/Infinity and non-JSON values even for in-process clients.
    if len(canonical(value)) > 32_000:
        raise ValueError("candidate JSON exceeds 32000 bytes")
    keys(value, {"schema_version", "model_edits", "rules"}, "candidate")
    if type(value["schema_version"]) is not int or value["schema_version"] != 1:
        raise ValueError("schema_version must be 1")
    for name, maximum in (("model_edits", 24), ("rules", 8)):
        if not isinstance(value[name], list) or len(value[name]) > maximum:
            raise ValueError(f"{name} must be a list with at most {maximum} entries")
    return value


def candidate_xml(config: warehouse.Config, candidate: dict[str, Any] | Path) -> tuple[str, dict[str, Any]]:
    """Apply numeric allowlisted edits to host-owned healthy XML only."""
    candidate = load_candidate(candidate)
    root = ET.fromstring(warehouse.model_xml(replace(config, fault="healthy")))
    seen: set[tuple[str, str, str]] = set()
    for edit in candidate["model_edits"]:
        keys(edit, {"target", "name", "field", "value"}, "model edit")
        target, name, field, value = (edit[key] for key in ("target", "name", "field", "value"))
        if not all(isinstance(part, str) for part in (target, name, field)):
            raise ValueError("edit target, name, field must be strings")
        identity = (target, name, field)
        if identity in seen:
            raise ValueError("duplicate model edit")
        seen.add(identity)
        # Compare names directly: candidate strings never enter an XPath expression.
        element = next((node for node in root.iter(target) if node.get("name") == name), None)
        if element is None:
            raise ValueError("edit must refer to an existing named element")
        if target == "joint" and field in {"damping", "frictionloss"}:
            numeric = [number(value, 0., 30. if field == "damping" else 10., field)]
        elif target == "joint" and field == "axis":
            if not isinstance(value, list) or len(value) != 3:
                raise ValueError("axis requires three components")
            numeric = [number(v, -1., 1., "axis") for v in value]
            if not .9 <= float(np.linalg.norm(numeric)) <= 1.2:
                raise ValueError("axis norm must be between .9 and 1.2")
        elif target == "joint" and field == "range" and name == "cargo_slide":
            if not isinstance(value, list) or len(value) != 2:
                raise ValueError("cargo_slide range requires two components")
            numeric = [number(v, -.3, .3, "range") for v in value]
            if not numeric[0] <= 0 <= numeric[1] or numeric[0] >= numeric[1]:
                raise ValueError("range must contain zero and have positive width")
        elif target == "geom" and field == "mass" and name in {
                "cargo_box", "chassis_geom", "tire_left", "tire_right", "caster_tire"}:
            numeric = [number(value, .1, 60., "mass")]
        elif target == "geom" and field == "friction" and name in {
                "floor", "tire_left", "tire_right", "caster_tire"}:
            if not isinstance(value, list) or len(value) != 3:
                raise ValueError("friction requires three components")
            numeric = [number(v, 0., limit, "friction") for v, limit in zip(value, (2., .1, .1))]
        elif target == "motor" and field == "gear" and name in {"left", "right"}:
            numeric = [number(value, .1, 12., "gear")]
        else:
            raise ValueError("model edit field/target is outside the numeric allowlist")
        element.set(field, " ".join(str(v) for v in numeric))
    xml = ET.tostring(root, encoding="unicode")
    model = mujoco.MjModel.from_xml_string(xml)
    for rule in candidate["rules"]:
        keys(rule, {"when", "updates", "once"}, "rule")
        if rule["once"] is not True:
            raise ValueError("rules must have once=true")
        condition = rule["when"]
        keys(condition, {"signal", "name", "absolute", "comparison", "threshold", "after_s", "sustained_s"},
             "rule condition")
        if condition["signal"] != "equality_force" or condition["comparison"] != "gte":
            raise ValueError("only equality_force gte conditions are supported")
        if condition["absolute"] is not True:
            raise ValueError("absolute must be true")
        name = condition["name"]
        if not isinstance(name, str):
            raise TypeError("equality name must be a string")
        eq_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, name)
        if eq_id < 0 or model.eq_type[eq_id] != mujoco.mjtEq.mjEQ_JOINT:
            raise ValueError("signal must refer to an existing scalar joint equality")
        number(condition["threshold"], .1, 1000., "threshold")
        number(condition["after_s"], 0., 10., "after_s")
        number(condition["sustained_s"], 0., 1., "sustained_s")
        if not isinstance(rule["updates"], list) or len(rule["updates"]) != 1:
            raise ValueError("rule requires one update")
        update = rule["updates"][0]
        keys(update, {"target", "name", "field", "value"}, "rule update")
        if (update["target"] != "equality" or update["name"] != name
                or update["field"] != "active" or update["value"] is not False):
            raise ValueError("rule may only deactivate its observed equality")
    return xml, candidate


def scalar_equality_force(model: mujoco.MjModel, data: mujoco.MjData, eq_id: int) -> float:
    if not data.eq_active[eq_id]:
        return 0.
    rows = (data.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY) & (data.efc_id == eq_id)
    forces = data.efc_force[rows]
    if len(forces) > 1:
        raise ValueError("rule signal unexpectedly has multiple constraint rows")
    return abs(float(forces[0])) if len(forces) else 0.


class CandidateSimulation(warehouse.Simulation):
    """An independently reset healthy simulation with declarative physical edits."""

    def __init__(self, config: warehouse.Config, candidate: dict[str, Any] | Path):
        config = replace(config, fault="healthy")
        xml, self.candidate = candidate_xml(config, candidate)
        self.candidate_sha256 = digest(self.candidate)
        self._rule_dwell = [0.] * len(self.candidate["rules"])
        self._rule_fired = [False] * len(self.candidate["rules"])
        super().__init__(config)
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data = mujoco.MjData(self.model)
        self._nominal_friction = self.model.geom_friction.copy()
        self._nominal_resistance = self.model.dof_frictionloss.copy()
        self._nominal_gain = self.model.actuator_gainprm.copy()
        self._nominal_range = self.model.jnt_range.copy()
        self._initialize_trial()

    def _apply_fault(self) -> None:
        for index, rule in enumerate(self.candidate["rules"]):
            if self._rule_fired[index]:
                continue
            condition = rule["when"]
            eq_id = self.model.equality(condition["name"]).id
            active = (self.elapsed >= condition["after_s"] - 1e-10
                      and scalar_equality_force(self.model, self.data, eq_id) >= condition["threshold"])
            self._rule_dwell[index] = self._rule_dwell[index] + self.config.timestep if active else 0.
            if active and self._rule_dwell[index] >= condition["sustained_s"] - 1e-10:
                self.data.eq_active[eq_id] = False
                self._rule_fired[index] = True

    def reset_full(self) -> None:
        self._rule_dwell = [0.] * len(self.candidate["rules"])
        self._rule_fired = [False] * len(self.candidate["rules"])
        super().reset_full()


def rollout(config: warehouse.Config, candidate: dict[str, Any] | Path | None = None,
            *, reference: bool = False, sample_period: float = .1) -> dict[str, Any]:
    simulation = (warehouse.Simulation(replace(config, fault="cargo_breakaway")) if reference
                  else CandidateSimulation(config, EMPTY_MODEL if candidate is None else candidate))
    observations = [simulation.observe()]
    next_sample = sample_period
    while not simulation.finished:
        simulation.step()
        if simulation.trial_time >= next_sample - 1e-9 or simulation.finished:
            observation = simulation.observe()
            if not reference:
                observation["model_signals"] = {
                    "equality_force": {"cargo_latch": scalar_equality_force(
                        simulation.model, simulation.data, simulation.model.equality("cargo_latch").id)}}
            observations.append(observation)
            next_sample += sample_period
        if (not np.isfinite(simulation.data.qpos).all() or not np.isfinite(simulation.data.qvel).all()
                or simulation.data.warning.number.any()):
            raise ValueError("simulation became nonfinite or produced a physics warning")
    result: dict[str, Any] = {"observations": observations, "summary": simulation.summary()["public"]}
    if isinstance(simulation, CandidateSimulation):
        result["candidate_sha256"] = simulation.candidate_sha256
    return result


def prediction_error(prediction: dict[str, Any], measured: dict[str, Any]) -> dict[str, float]:
    lhs, rhs = prediction["observations"], measured["observations"]
    if len(lhs) != len(rhs) or any(abs(a["time"] - b["time"]) > 1e-8 for a, b in zip(lhs, rhs)):
        raise ValueError("prediction and measurement timestamps do not match")
    positions = np.array([a["position"][:2] for a in lhs]) - np.array([b["position"][:2] for b in rhs])
    headings = np.array([a["heading"] - b["heading"] for a, b in zip(lhs, rhs)])
    headings = np.arctan2(np.sin(headings), np.cos(headings))
    result = {"position_rmse_m": float(np.sqrt(np.mean(np.sum(positions ** 2, axis=1)))),
            "final_position_error_m": float(np.linalg.norm(positions[-1])),
            "heading_rmse_rad": float(np.sqrt(np.mean(headings ** 2)))}
    if "cargo_position" in lhs[0] or "cargo_position" in rhs[0]:
        cargo_error = np.array([a["cargo_position"] for a in lhs]) - np.array([b["cargo_position"] for b in rhs])
        result["cargo_position_rmse_m"] = float(np.sqrt(np.mean(np.sum(cargo_error ** 2, axis=1))))
        result["final_cargo_position_error_m"] = float(np.linalg.norm(cargo_error[-1]))
    return result


def cargo_dropped(trajectory: dict[str, Any]) -> bool:
    """The public contact-history sensor includes every collidable cargo geom."""
    return any(row["cargo_has_touched_floor"] for row in trajectory["observations"])


class WarehouseBroker:
    def __init__(self, run_dir: Path, candidate: dict[str, Any] | Path | None = None,
                 *, origin: dict[str, Any] | None = None, task: str = "rail"):
        public_config({}, task=task)  # Validate before creating an output directory.
        self.task = task
        self.thresholds = THRESHOLDS if task == "rail" else CURVE_THRESHOLDS
        self.run_dir = Path(run_dir)
        self.run_dir.mkdir(parents=True, exist_ok=True)
        if (self.run_dir / "candidate.json").exists():
            raise ValueError("output directory already contains a candidate; use a fresh run directory")
        self.candidate = load_candidate(EMPTY_MODEL if candidate is None else candidate)
        candidate_xml(public_config({}, task=self.task), self.candidate)
        self.origin = origin or {"kind": "prepared_baseline" if candidate is None else "provided_candidate"}
        self.initial_candidate_sha256 = digest(self.candidate)
        self.initial_candidate_origin = "shipped_nominal" if candidate is None else "supplied_artifact"
        self.accepted_patch_count = 0
        self.used = dict.fromkeys(LIMITS, 0)
        self.development_configs: list[dict[str, Any]] = []
        self.frozen: dict[str, Any] | None = None
        self.submission: str | None = None
        self._evidence: dict[str, Any] | None = None
        save_json(self.run_dir / "candidate.json", self.candidate)
        save_json(self.run_dir / "provenance.json", {"origin": self.origin,
                  "initial_sha256": digest(self.candidate), "task": self.task, "thresholds": self.thresholds})

    def _record(self, name: str, arguments: Any, result: Any) -> None:
        try:
            encoded = canonical(arguments)
            if len(encoded) > 40_000:
                raise ValueError("oversized audit arguments")
        except (ValueError, TypeError, RecursionError, OverflowError):
            arguments = {"rejected_arguments": "nonfinite, unsupported, oversized, or too deeply nested"}
        with (self.run_dir / "tools.jsonl").open("a") as stream:
            stream.write(canonical({"tool": name, "arguments": arguments, "result": result}) + "\n")

    def initial_evidence(self) -> dict[str, Any]:
        if self._evidence is None:
            cases: list[dict[str, Any]] = []
            for probe in (("cargo_turn", "cargo_gentle") if self.task == "rail" else CURVE_PUBLIC_PROBES):
                config = public_config({"probe": probe}, task=self.task)
                self.development_configs.append(config_dict(config))
                measured = rollout(config, reference=True)
                nominal = rollout(config)
                cases.append({"id": f"specimen_{len(cases) + 1:03d}", "config": config_dict(config),
                              "measured": measured, "original_prediction": nominal,
                              "original_error": prediction_error(nominal, measured)})
            self._evidence = {"task": "Repair the model's prediction of observed trolley and cargo motion. "
                              "Use interventions to distinguish physical hypotheses, then submit a persistent model.",
                              "cases": cases, "candidate": self.candidate, "budgets": LIMITS,
                              "benchmark": self.task, "evaluation_thresholds": self.thresholds}
            if self.task == "curve":
                self._evidence["public_route"] = {key: value for key, value in warehouse.route_geometry().items()
                                                   if key != "centerline"}
            save_json(self.run_dir / "evidence.json", self._evidence)
        return deepcopy(self._evidence)

    def dispatch(self, name: str, arguments: Any) -> dict[str, Any]:
        result: dict[str, Any]
        try:
            if self.frozen is not None:
                raise ValueError("candidate is frozen; no further tool calls are accepted")
            if name not in LIMITS:
                raise ValueError("unknown tool")
            if sum(self.used.values()) >= 30 or self.used[name] >= LIMITS[name]:
                raise ValueError("tool budget exhausted")
            self.used[name] += 1
            if not isinstance(arguments, dict):
                raise TypeError("tool arguments must be an object")
            if len(canonical(arguments)) > 40_000:
                raise ValueError("oversized tool arguments")
            if name == "inspect_model":
                keys(arguments, set(), name)
                result = {"nominal_xml": warehouse.model_xml(public_config({}, task=self.task)),
                          "contract": CONTRACT.read_text(), "candidate": self.candidate,
                          "candidate_sha256": digest(self.candidate)}
                if self.task == "curve":
                    result["public_route"] = {key: value for key, value in warehouse.route_geometry().items()
                                              if key != "centerline"}
            elif name in {"run_experiment", "run_model"}:
                keys(arguments, {"config", "hypothesis"}, name)
                self._text(arguments["hypothesis"], "hypothesis")
                config = public_config(arguments["config"], task=self.task)
                if name == "run_experiment":
                    self.development_configs.append(config_dict(config))
                result = {"config": config_dict(config), "result": rollout(
                    config, self.candidate, reference=name == "run_experiment")}
            elif name == "patch_model":
                keys(arguments, {"candidate_json", "rationale"}, name)
                self._text(arguments["rationale"], "rationale")
                source = arguments["candidate_json"]
                if not isinstance(source, str) or len(source) > 32_000:
                    raise ValueError("candidate_json must be a JSON string under 32000 characters")
                proposal = load_candidate(parse_json(source))
                candidate_xml(public_config({}, task=self.task), proposal)
                old_hash = digest(self.candidate)
                self.candidate = proposal
                self.accepted_patch_count += 1
                save_json(self.run_dir / "candidate.json", self.candidate)
                result = {"accepted": True, "previous_sha256": old_hash,
                          "candidate_sha256": digest(self.candidate)}
            elif name == "run_regression_suite":
                keys(arguments, set(), name)
                cases = self.initial_evidence()["cases"]
                result = {"cases": [{"id": case["id"], "config": case["config"],
                          "original_error": case["original_error"], "candidate_error": prediction_error(
                              rollout(public_config(case["config"], task=self.task), self.candidate), case["measured"])}
                                    for case in cases]}
            else:
                keys(arguments, {"explanation"}, name)
                self._text(arguments["explanation"], "explanation")
                self.submission = arguments["explanation"]
                result = self.freeze()
        except (ValueError, TypeError, KeyError, OverflowError, RecursionError):
            # Exceptions from model compilers may include internals; return neutral context.
            result = {"error": "Tool rejected: invalid arguments, unsupported model, unsafe simulation, or exhausted budget. "
                      "Check the model contract and tool schema."}
        result["budgets_used"] = self.used.copy()
        self._record(name, arguments, result)
        return result

    @staticmethod
    def _text(value: Any, name: str) -> None:
        if not isinstance(value, str) or not value.strip() or len(value) > 4000:
            raise ValueError(f"{name} must contain 1-4000 characters")

    def freeze(self) -> dict[str, Any]:
        if self.frozen is None:
            self.frozen = deepcopy(self.candidate)
            save_json(self.run_dir / "frozen_candidate.json", self.frozen)
            save_json(self.run_dir / "submission.json", {"candidate_sha256": digest(self.frozen),
                      "agent_submitted": self.submission is not None, "explanation": self.submission,
                      "origin": self.origin, "development_configs": self.development_configs,
                      "task": self.task, "thresholds": self.thresholds})
        return {"frozen": True, "candidate_sha256": digest(self.frozen)}

    def evaluate(self) -> dict[str, Any]:
        self.freeze()
        assert self.frozen is not None
        folder = self.run_dir / "evaluation"
        if folder.exists():
            raise ValueError("evaluation already exists; frozen evaluations cannot be overwritten")
        folder.mkdir()
        seen = {control_identity(config) for config in self.development_configs}
        if self.task == "curve":
            reserved = [public_config({"probe": probe, "duration": 16.8, "drive_scale": scale,
                                       "payload_mass": mass}, task="curve")
                        for probe, scale, mass in (("cargo_curve", .95, 9.), ("cargo_curve", .9, 10.),
                                                  ("cargo_curve_slow", .9, 9.), ("cargo_curve", .97, 8.5),
                                                  ("cargo_curve", .93, 9.5), ("cargo_curve", .99, 10.5),
                                                  ("cargo_curve_slow", .85, 8.5), ("cargo_curve", .96, 10.),
                                                  ("cargo_curve", .92, 8.5), ("cargo_curve_slow", .8, 10.),
                                                  ("cargo_curve", .94, 9.), ("cargo_curve_slow", .88, 8.))]
            control_probe = "cargo_curve_slow"
        else:
            reserved = [public_config({"probe": probe, "duration": 10.8, "drive_scale": scale,
                                       "payload_mass": mass})
                        for probe, scale, mass in (("cargo_mirror", .95, 18.), ("cargo_strong", .85, 20.),
                                                  ("cargo_turn", .9, 17.), ("cargo_gentle", .75, 12.),
                                                  ("cargo_mirror", .87, 17.), ("cargo_strong", .92, 16.),
                                                  ("cargo_gentle", .68, 14.), ("cargo_turn", .83, 18.),
                                                  ("cargo_mirror", .79, 19.), ("cargo_gentle", .6, 16.),
                                                  ("cargo_strong", .97, 17.), ("cargo_gentle", .7, 18.))]
            control_probe = "cargo_gentle"
        selected = [config for config in reserved if control_identity(config_dict(config)) not in seen]
        affected = [config for config in selected if config.probe != control_probe][:2 if self.task == "curve" else 3]
        controls = [config for config in selected if config.probe == control_probe][:1]
        if len(affected) < 2 or not controls:
            raise ValueError("insufficient unseen controls for evaluation")
        configs = affected + controls
        # Write EVERY candidate and nominal prediction before ANY reference rollout.
        predictions: list[dict[str, Any]] = [{"id": f"reserved_{index + 1:03d}", "config": config_dict(config),
                        "candidate": rollout(config, self.frozen), "nominal": rollout(config)}
                       for index, config in enumerate(configs)]
        prediction_path = folder / "predictions.json"
        save_json(prediction_path, predictions)
        freeze_manifest = {"candidate_sha256": digest(self.frozen), "task": self.task, "thresholds": self.thresholds,
                           "predictions_sha256": hashlib.sha256(prediction_path.read_bytes()).hexdigest(),
                           "all_predictions_locked_before_measurements": True,
                           "frozen_at": datetime.now(UTC).isoformat()}
        save_json(folder / "freeze.json", freeze_manifest)
        cases: list[dict[str, Any]] = []
        for prediction, config in zip(predictions, configs):
            measured = rollout(config, reference=True)
            nominal = prediction_error(prediction["nominal"], measured)
            candidate_error = prediction_error(prediction["candidate"], measured)
            control = config.probe == control_probe
            if self.task == "curve":
                measured_drop = cargo_dropped(measured)
                candidate_drop = cargo_dropped(prediction["candidate"])
                nominal_drop = cargo_dropped(prediction["nominal"])
                passed = (candidate_error["position_rmse_m"] <= CURVE_THRESHOLDS["max_candidate_position_rmse_m"]
                          and candidate_error["heading_rmse_rad"] <= CURVE_THRESHOLDS["max_heading_rmse_rad"]
                          and candidate_drop == measured_drop
                          and prediction["candidate"]["summary"]["safe"] and measured["summary"]["safe"])
                if control:
                    passed = (passed and not measured_drop and not nominal_drop
                              and nominal["cargo_position_rmse_m"] <= .001
                              and candidate_error["cargo_position_rmse_m"]
                              <= CURVE_THRESHOLDS["healthy_max_candidate_cargo_rmse_m"]
                              and candidate_error["position_rmse_m"]
                              <= CURVE_THRESHOLDS["healthy_max_candidate_position_rmse_m"])
                else:
                    passed = (passed and measured_drop and not nominal_drop
                              and nominal["cargo_position_rmse_m"]
                              >= CURVE_THRESHOLDS["affected_min_nominal_cargo_rmse_m"]
                              and candidate_error["cargo_position_rmse_m"]
                              <= CURVE_THRESHOLDS["max_candidate_cargo_rmse_m"]
                              and candidate_error["cargo_position_rmse_m"] <= nominal["cargo_position_rmse_m"]
                              * CURVE_THRESHOLDS["affected_max_cargo_error_ratio"])
            else:
                if control:
                    passed = (nominal["position_rmse_m"] <= .001
                              and candidate_error["position_rmse_m"] <= THRESHOLDS["healthy_max_candidate_rmse_m"]
                              and candidate_error["position_rmse_m"] - nominal["position_rmse_m"]
                              <= THRESHOLDS["healthy_max_degradation_m"])
                else:
                    passed = (nominal["position_rmse_m"] >= THRESHOLDS["affected_min_nominal_rmse_m"]
                              and candidate_error["position_rmse_m"] <= THRESHOLDS["affected_max_candidate_rmse_m"]
                              and candidate_error["position_rmse_m"] <= nominal["position_rmse_m"]
                              * THRESHOLDS["affected_max_error_ratio"])
                passed = (passed and candidate_error["heading_rmse_rad"] <= THRESHOLDS["max_heading_rmse_rad"]
                          and prediction["candidate"]["summary"]["safe"] and measured["summary"]["safe"])
            cases.append({"id": prediction["id"], "config": config_dict(config), "healthy_control": control,
                          "previously_observed": False, "nominal_error": nominal,
                          "candidate_error": candidate_error, "passed": bool(passed), "measured": measured})
            if self.task == "curve":
                cases[-1]["cargo_outcome"] = {"measured_dropped": measured_drop,
                                             "candidate_dropped": candidate_drop, "nominal_dropped": nominal_drop}
        report = {"schema_version": 1, "task": self.task, "candidate_sha256": digest(self.frozen),
                  "provenance": {**self.origin, "initial_candidate_origin": self.initial_candidate_origin,
                                 "initial_candidate_sha256": self.initial_candidate_sha256,
                                 "accepted_patch_count": self.accepted_patch_count,
                                 "source_changed": digest(self.frozen) != self.initial_candidate_sha256},
                  "agent_submitted": self.submission is not None,
                  "passed": bool(all(case["passed"] for case in cases)),
                  "frozen_predictions": freeze_manifest, "cases": cases}
        save_json(folder / "report.json", report)
        return report


def tool_schema(name: str, description: str, properties: dict[str, Any]) -> dict[str, Any]:
    return {"type": "function", "name": name, "description": description, "strict": True,
            "parameters": {"type": "object", "properties": properties, "required": list(properties),
                           "additionalProperties": False}}


CONFIG_SCHEMA = {"type": "object", "properties": {
    "probe": {"type": "string", "enum": list(PUBLIC_PROBES)}, "duration": {"type": "number"},
    "drive_scale": {"type": "number"}, "payload_mass": {"type": "number"}},
    "required": ["probe", "duration", "drive_scale", "payload_mass"], "additionalProperties": False}
TEXT_SCHEMA = {"type": "string"}
TOOL_SCHEMAS = [
    tool_schema("inspect_model", "Inspect nominal host model, editable artifact and bounded contract.", {}),
    tool_schema("run_experiment", "Measure a fresh specimen with the selected public controls.",
                {"config": CONFIG_SCHEMA, "hypothesis": TEXT_SCHEMA}),
    tool_schema("run_model", "Predict with the current candidate's own physics and force signals.",
                {"config": CONFIG_SCHEMA, "hypothesis": TEXT_SCHEMA}),
    tool_schema("patch_model", "Replace the complete candidate JSON with a validated physical model.",
                {"candidate_json": TEXT_SCHEMA, "rationale": TEXT_SCHEMA}),
    tool_schema("run_regression_suite", "Compare the candidate against the initial observed trials.", {}),
    tool_schema("submit_prediction", "Freeze the candidate before unseen predictions and evaluation.",
                {"explanation": TEXT_SCHEMA}),
]


def run_session(client: Any, broker: WarehouseBroker, *, max_api_requests: int = 16,
                profile: str | None = None, max_seconds: int = 1800) -> dict[str, Any]:
    from .api import DEFAULT_PROFILE, get_profile, request_response
    profile = profile or DEFAULT_PROFILE
    selected = get_profile(profile)
    if not 1 <= max_api_requests <= 30 or not 60 <= max_seconds <= 7200:
        raise ValueError("invalid API request or time budget")
    metadata: dict[str, Any] = {"status": "running", "model": selected.model,
                                "reasoning_effort": selected.reasoning_effort, "api_requests": 0,
                                "agent_submitted": False, "usage": {"input_tokens": 0, "output_tokens": 0}}
    system = ("You repair a physical simulation from measured experiments. Use only the supplied tools. "
              "The reference is inaccessible. Develop and execute a bounded JSON model, investigate competing "
              "hypotheses, and submit a persistent physical prediction. Observations do not identify causes. "
              "Do not claim success before frozen held-out evaluation. "
              f"You have at most {max_api_requests} API responses; reserve one to submit_prediction.")
    schemas = deepcopy(TOOL_SCHEMAS)
    for schema in schemas:
        if "config" in schema["parameters"]["properties"]:
            schema["parameters"]["properties"]["config"]["properties"]["probe"]["enum"] = list(TASK_PROBES[broker.task])
    metadata["task"] = broker.task
    evidence = broker.initial_evidence()
    task = CONTRACT.read_text() + "\nInitial evidence:\n" + canonical(evidence)
    save_json(broker.run_dir / "prompts.json", {"system": system, "task": task, "tools": schemas})
    conversation: list[Any] = [{"role": "system", "content": system}, {"role": "user", "content": task}]
    started = time.monotonic()
    try:
        while metadata["api_requests"] < max_api_requests and time.monotonic() - started < max_seconds:
            metadata["api_requests"] += 1
            save_json(broker.run_dir / "metadata.json", metadata)
            response = request_response(client, conversation, schemas, profile=profile)
            if (response.model != selected.model or not response.reasoning
                    or response.reasoning.effort != selected.reasoning_effort or response.status != "completed"):
                raise RuntimeError("API did not confirm requested model, reasoning, and completion")
            broker._record("api_response", {}, {"response_id": response.id, "model": response.model,
                           "reasoning_effort": response.reasoning.effort, "status": response.status})
            if response.usage:
                for key in metadata["usage"]:
                    metadata["usage"][key] += getattr(response.usage, key, 0) or 0
            conversation.extend(item.model_dump(exclude_none=True) for item in response.output)
            calls = []
            for item in response.output:
                if item.type == "function_call":
                    calls.append(item)
                elif item.type == "message":
                    for part in item.content:
                        if part.type == "output_text":
                            broker._record("assistant_message", {}, {"text": part.text})
            for call in calls:
                try:
                    if len(call.arguments) > 40_000:
                        raise ValueError("arguments too large")
                    arguments = parse_json(call.arguments, maximum=40_000)
                except (ValueError, TypeError):
                    result = {"error": "Invalid JSON tool arguments"}
                else:
                    result = broker.dispatch(call.name, arguments)
                conversation.append({"type": "function_call_output", "call_id": call.call_id,
                                     "output": canonical({**result, "api_requests_remaining":
                                                          max_api_requests - metadata["api_requests"]})})
                if broker.frozen is not None:
                    break
            if broker.frozen is not None:
                metadata["agent_submitted"] = broker.submission is not None
                break
            if not calls:
                conversation.append({"role": "user", "content": "Use tools to execute the model repair; "
                                     "call submit_prediction when ready. A textual proposal is not a patch."})
        broker.freeze()
        report = broker.evaluate()
        metadata["status"] = "evaluated"
        metadata["passed"] = report["passed"]
        metadata["candidate_sha256"] = report["candidate_sha256"]
    except Exception as error:  # noqa: BLE001 -- redact SDK errors at the API trust boundary
        # SDK exception text and request objects may contain credentials.
        metadata["status"] = "error"
        metadata["error_type"] = type(error).__name__
        broker.freeze()
    metadata["duration_s"] = time.monotonic() - started
    save_json(broker.run_dir / "metadata.json", metadata)
    return metadata


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("prepare", "run", "evaluate"))
    parser.add_argument("--output", type=Path, required=True, help="Fresh output directory")
    parser.add_argument("--candidate", type=Path)
    parser.add_argument("--task", choices=tuple(TASK_PROBES), default="rail")
    parser.add_argument("--max-api-requests", type=int, default=16)
    parser.add_argument("--profile", help="Model profile; defaults to the current API configuration")
    args = parser.parse_args(argv)
    if args.command == "run":
        from .api import DEFAULT_PROFILE, Settings, get_profile
        profile_name = args.profile or DEFAULT_PROFILE
        settings = Settings.load(profile=profile_name)
        profile = get_profile(profile_name)
        broker = WarehouseBroker(args.output, args.candidate, origin={"kind": "api_investigation",
                                 "model": profile.model, "reasoning_effort": profile.reasoning_effort}, task=args.task)
        result = run_session(settings.client(), broker, max_api_requests=args.max_api_requests,
                             profile=profile_name)
        print(canonical(result))
        return 0 if result["status"] == "evaluated" and result["agent_submitted"] and result["passed"] else 1
    broker = WarehouseBroker(args.output, args.candidate, task=args.task)
    if args.command == "prepare":
        broker.initial_evidence()
        print(canonical({"status": "prepared", "task": broker.task, "output": str(args.output),
                         "candidate_sha256": digest(broker.candidate), "api_requests": 0}))
        return 0
    result = broker.evaluate()
    print(canonical({"passed": result["passed"], "task": broker.task, "candidate_sha256": result["candidate_sha256"],
                     "report": str(args.output / "evaluation" / "report.json")}))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
