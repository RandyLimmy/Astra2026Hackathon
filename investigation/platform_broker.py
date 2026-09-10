"""Astra's bounded experiment, source-editing and maintenance tools.

The host owns platform construction and physical interventions. Candidate Python
is executed only by PlatformModelWorker; public observations are its only input.
"""

from __future__ import annotations

import ast
import difflib
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np

from component_worker import WorkerError


ROOT = Path(__file__).resolve().parents[1]
LIMITS = {"run_experiment": 8, "replace_model_source": 5, "restore_model_version": 5,
          "run_model": 10, "apply_repair": 4, "check_repair": 4, "run_regression_suite": 2}
MAX_EDITS = 5
VERIFICATION_PROBES = {"car": ("steering", "braking"), "drone": ("hover", "maneuver"),
                       "quadruped": ("walk", "turn")}
TEXT = {"type": "string"}


class PlatformToolError(ValueError):
    """A deliberately public, neutral tool error."""


def _sha(data):
    return hashlib.sha256(data).hexdigest()


def _save(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def _schema(name, description, properties):
    return {"type": "function", "name": name, "description": description, "strict": True,
            "parameters": {"type": "object", "properties": properties,
                           "required": list(properties), "additionalProperties": False}}


def _worker(source):
    from component_worker import PlatformModelWorker
    # Give isolated Python startup a separate, bounded allowance on a busy host.
    # Every subsequent component operation retains the normal two-second limit.
    worker = PlatformModelWorker(source, timeout_s=5.0)
    worker.timeout_s = 2.0
    return worker


def _public(record):
    """Never send host paths, frames, private diagnostics or construction config."""
    return {key: record[key] for key in
            ("id", "probe", "duration_s", "summary", "observations", "source_sha256") if key in record}


def compare_records(prediction, observation):
    """Compare overlapping measured trajectories; never extrapolate an early stop."""
    def trajectory(record):
        rows = record.get("observations", [])
        if not rows:
            return np.array([]), np.empty((0, 3))
        clock = "t_s" if "t_s" in rows[0] else "time"
        origin = rows[0].get(clock, 0)
        samples = [(row.get(clock, origin) - origin, row.get("position")) for row in rows]
        samples = [(t, p) for t, p in samples if isinstance(p, list) and len(p) == 3]
        return np.array([t for t, _ in samples]), np.array([p for _, p in samples])

    pt, pp = trajectory(prediction)
    ot, op = trajectory(observation)
    result = {"position_rmse_m": None, "maximum_position_error_m": None,
              "common_duration_s": 0.0, "samples": 0,
              "observed_outcome": observation.get("summary", {}).get("outcome"),
              "observed_within_envelope": observation.get("summary", {}).get("safe"),
              "reference_outcome": prediction.get("summary", {}).get("outcome")}
    if not len(pt) or not len(ot):
        return result
    keep = (ot >= pt[0]) & (ot <= pt[-1])
    if not keep.any():
        return result
    interpolated = np.column_stack([np.interp(ot[keep], pt, pp[:, axis]) for axis in range(3)])
    distance = np.linalg.norm(op[keep] - interpolated, axis=1)
    result.update(position_rmse_m=float(np.sqrt(np.mean(distance ** 2))),
                  maximum_position_error_m=float(distance.max()),
                  common_duration_s=float(ot[keep][-1]), samples=int(keep.sum()))
    return result


class PlatformBroker:
    def __init__(self, workdir, physics):
        self.workdir = Path(workdir).resolve()
        self.workdir.mkdir(parents=True, exist_ok=True)
        self.physics = physics
        self.capabilities = physics.capabilities()
        self._probes = tuple(self.capabilities["probes"])
        self._counts = {name: 0 for name in LIMITS}
        self._dispatches = 0
        self._edits = 0
        self._started = time.monotonic()
        self._records = {}
        self._initial = None
        self._last_check = None
        self.repair_history = []
        self.submitted = False
        self._frozen = False
        self._result = None
        self._versions = {}
        self.current_source = self._version((ROOT / "candidate/platform_model.py").read_bytes())
        self.tool_schemas = self._tools()
        _save(self.workdir / "verification_plan.json", {
            "probes": list(VERIFICATION_PROBES[self.physics.platform]),
            "duration_s": self.physics.default_duration_s,
            "fresh_specimen": True, "predictions_locked_before_probes": True,
            "measurement": "position error over overlapping recorded times and the platform's public probe envelope",
            "scope": "Repeated controlled synthetic probes, not necessarily unseen maneuvers or general safety."})

    def _tools(self):
        probe = {"type": "string", "enum": list(self._probes)}
        duration = {"type": "number", "minimum": 2, "maximum": 20,
                    "description": "Seconds of the diagnostic probe, after a declared fixture reset."}
        return [
            _schema("inspect_system", "Read nominal components, sensors, probe choices, repair actions and model parameter bounds. No hidden fault identity is supplied.", {}),
            _schema("observe_system", "Read current allowed sensor measurements, optionally for one named component. Measurements do not label which component is broken.",
                    {"component_id": {"type": ["string", "null"]}}),
            _schema("inspect_model", "Read editable Python source, current hash, state and available source versions.", {}),
            _schema("replace_model_source", "Replace the complete model.py source using its latest hash. Source runs in isolation and returns bounded physical-model parameters. This changes predictions, not the specimen.",
                    {"source": TEXT, "expected_sha256": TEXT, "rationale": TEXT}),
            _schema("restore_model_version", "Restore a previously accepted source version using the current hash. Shares five total attempted edits with source replacement.",
                    {"version_id": TEXT, "expected_sha256": TEXT, "rationale": TEXT}),
            _schema("run_experiment", "Test the current specimen with a named diagnostic probe. A fixture reset retains damage and completed repairs. Eight development probes maximum.",
                    {"probe": probe, "duration_s": duration, "hypothesis": TEXT, "expected_observation": TEXT}),
            _schema("observe_run", "Read a public recording belonging to this investigation by its opaque id.", {"id": TEXT}),
            _schema("run_model", "Execute current Python model and nominal MuJoCo mechanics. Its state receives only its own predicted observations, never hidden specimen state.",
                    {"probe": probe, "duration_s": duration, "rationale": TEXT}),
            _schema("apply_repair", "Perform one catalogued maintenance action on one component. A receipt only confirms the action; use check_repair to measure its effect. Four actions maximum.",
                    {"action": TEXT, "target": TEXT, "rationale": TEXT, "expected_effect": TEXT}),
            _schema("check_repair", "Run the current specimen and matching healthy control; return measured differences. A single probe is evidence, not a general safety guarantee.",
                    {"probe": probe, "duration_s": duration, "rationale": TEXT}),
            _schema("run_regression_suite", "Run two declared development probes against healthy controls, preserving the current repairs between probes. This checks the physical maintenance result.", {"rationale": TEXT}),
            _schema("submit_result", "Freeze the exact model and maintenance sequence for fresh-specimen verification. All further tools close. Distinguish observations, diagnosis and uncertainty.",
                    {"diagnosis": TEXT, "evidence": TEXT, "remaining_uncertainty": TEXT}),
        ]

    def _version(self, data):
        name = f"v{len(self._versions):03d}"
        path = self.workdir / "versions" / name / "model.py"
        path.parent.mkdir(parents=True, exist_ok=False)
        path.write_bytes(data)
        path.chmod(0o444)
        self._versions[name] = path
        return path

    def budget_status(self):
        return {"tool_calls": {"used": self._dispatches, "limit": 40},
                **{name: {"used": count, "limit": LIMITS[name]} for name, count in self._counts.items()},
                "model_edits": {"used": self._edits, "limit": MAX_EDITS},
                "elapsed_s": round(time.monotonic() - self._started, 2),
                "time_limit_s": 1800, "frozen": self._frozen}

    def _remember(self, record):
        public = _public(record)
        if not isinstance(public.get("id"), str):
            raise PlatformToolError("The experiment did not produce a recording id.")
        self._records[public["id"]] = public
        _save(self.workdir / "records" / (public["id"] + ".json"), public)
        return public

    def initial_evidence(self):
        if self._initial is None:
            initial = self.physics.initial_evidence()
            healthy, observed = (self._remember(initial[key]) for key in ("healthy", "observed"))
            self._initial = {"capabilities": self.capabilities, "healthy": healthy, "observed": observed,
                             "initial_difference": compare_records(healthy, observed),
                             "interpretation": "A physical incident or condition may have changed behavior. Infer causes from public measurements; maintenance and model editing are distinct actions."}
            _save(self.workdir / "initial_evidence.json", self._initial)
        return self._initial

    def _text(self, value, maximum=4000):
        if not isinstance(value, str) or not value.strip() or len(value) > maximum:
            raise PlatformToolError("Provide nonempty text within the permitted length.")
        return value

    def _probe(self, probe, duration_s):
        if not isinstance(probe, str) or probe not in self._probes:
            raise PlatformToolError("Choose a probe from inspect_system.")
        if (isinstance(duration_s, bool) or not isinstance(duration_s, (int, float))
                or not math.isfinite(duration_s) or not 2 <= duration_s <= 20):
            raise PlatformToolError("Probe duration must be between 2 and 20 seconds.")

    def _parameters(self, values):
        specs = self.capabilities["model_parameters"]
        if not isinstance(values, dict) or set(values) - set(specs):
            raise PlatformToolError("The model returned an unsupported parameter; inspect_system lists the allowed keys.")
        for key, value in values.items():
            spec = specs[key]
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or not spec["min"] <= value <= spec["max"]):
                raise PlatformToolError("A predicted parameter is outside its declared physical bounds.")
        return values

    def inspect_model(self):
        with _worker(self.current_source) as worker:
            state = worker.inspect_state()
            parameters = self._parameters(worker.parameters())
        return {"source": self.current_source.read_text(), "source_sha256": _sha(self.current_source.read_bytes()),
                "state": state, "initial_parameters": parameters,
                "versions": [{"id": name, "source_sha256": _sha(path.read_bytes())} for name, path in self._versions.items()],
                "contract": (ROOT / "contracts/PLATFORM_MODEL.md").read_text()}

    def _replace(self, source, expected_sha256, rationale):
        self._text(rationale)
        current = self.current_source.read_bytes()
        if not isinstance(expected_sha256, str) or expected_sha256 != _sha(current):
            raise PlatformToolError("Source changed; inspect_model and use its current source_sha256.")
        if not isinstance(source, str) or len(source.encode()) > 65_536:
            raise PlatformToolError("Send complete Python source within 65,536 bytes.")
        attempt = self.workdir / "edit_attempts" / f"attempt_{self._edits:03d}"
        attempt.mkdir(parents=True, exist_ok=False)
        _save(attempt / "request.json", {"source": source, "expected_sha256": expected_sha256, "rationale": rationale})
        try:
            tree = ast.parse(source)
        except (SyntaxError, ValueError):
            raise PlatformToolError("Python syntax is invalid; inspect the source and send a corrected complete file.") from None
        names = {node.name for node in tree.body if isinstance(node, ast.FunctionDef)}
        if not {"init_state", "predict_parameters", "advance_state"} <= names:
            raise PlatformToolError("Keep init_state, predict_parameters and advance_state from the model contract.")
        test_source = attempt / "model.py"
        test_source.write_text(source)
        with _worker(test_source) as worker:
            self._parameters(worker.parameters())
            # Only neutral nominal observations are supplied during interface validation.
            first = self.initial_evidence()["healthy"]["observations"][0]
            worker.advance(first, 0.02)
            self._parameters(worker.parameters())
        self.current_source = self._version(source.encode())
        difference = "".join(difflib.unified_diff(current.decode().splitlines(True), source.splitlines(True),
                                                fromfile="previous/model.py", tofile="current/model.py"))
        (attempt / "source.diff").write_text(difference)
        return {"accepted": True, "version_id": next(reversed(self._versions)),
                "source_sha256": _sha(self.current_source.read_bytes()),
                "meaning": "The predictive component changed. No physical repair was performed."}

    def _model(self, physics, probe, duration_s):
        with _worker(self.current_source) as worker:
            def parameters(observation, dt):
                worker.advance(observation, dt)
                return self._parameters(worker.parameters())
            record = physics.run_model(probe, duration_s, parameters)
            record["source_sha256"] = worker.source_sha256
            return record

    def _check(self, probe, duration_s):
        actual = self._remember(self.physics.run_experiment(probe, duration_s))
        healthy = self._remember(self.physics.reference_probe(probe, duration_s))
        result = {"observed": actual, "healthy": healthy, "difference": compare_records(healthy, actual),
                  "maintenance_actions": len(self.repair_history),
                  "interpretation": "These measurements describe this probe only. A maintenance receipt alone does not demonstrate recovery."}
        self._last_check = result
        return result

    def dispatch(self, name, arguments):
        try:
            if self._frozen:
                raise PlatformToolError("The result is frozen; development tools are closed.")
            if self._dispatches >= 40 or time.monotonic() - self._started >= 1800:
                raise PlatformToolError("The investigation tool budget is exhausted.")
            self._dispatches += 1
            schemas = {entry["name"]: entry["parameters"] for entry in self.tool_schemas}
            if name not in schemas:
                raise PlatformToolError("Unknown tool. Use the declared capabilities.")
            if name in LIMITS:
                if self._counts[name] >= LIMITS[name]:
                    raise PlatformToolError("This tool's attempt budget is exhausted.")
                self._counts[name] += 1
            if name in {"replace_model_source", "restore_model_version"}:
                if self._edits >= MAX_EDITS:
                    raise PlatformToolError("The shared source-edit budget is exhausted.")
                self._edits += 1
            if not isinstance(arguments, dict) or set(arguments) != set(schemas[name]["required"]):
                raise PlatformToolError("Tool arguments must match the declared fields exactly.")
            for key in ("rationale", "hypothesis", "expected_observation", "expected_effect", "diagnosis", "evidence", "remaining_uncertainty"):
                if key in arguments:
                    self._text(arguments[key])
            if "probe" in arguments:
                self._probe(arguments["probe"], arguments["duration_s"])
            if name == "inspect_system":
                result = self.capabilities
            elif name == "observe_system":
                component = arguments["component_id"]
                if component is not None and (not isinstance(component, str) or len(component) > 80):
                    raise PlatformToolError("Choose a component id from inspect_system, or null.")
                result = self.physics.observe(component)
            elif name == "inspect_model":
                result = self.inspect_model()
            elif name in {"replace_model_source", "restore_model_version"}:
                source = arguments.get("source")
                if name == "restore_model_version":
                    version = arguments["version_id"]
                    if not isinstance(version, str) or version not in self._versions:
                        raise PlatformToolError("Choose a saved version id from inspect_model.")
                    source = self._versions[version].read_text()
                result = self._replace(source, arguments["expected_sha256"], arguments["rationale"])
            elif name == "run_experiment":
                result = self._remember(self.physics.run_experiment(arguments["probe"], arguments["duration_s"]))
            elif name == "observe_run":
                identifier = arguments["id"]
                if not isinstance(identifier, str) or identifier not in self._records:
                    raise PlatformToolError("That recording does not belong to this investigation.")
                result = self._records[identifier]
            elif name == "run_model":
                result = self._remember(self._model(self.physics, arguments["probe"], arguments["duration_s"]))
            elif name == "apply_repair":
                self._text(arguments["action"], 80)
                self._text(arguments["target"], 80)
                receipt = self.physics.apply_repair(arguments["action"], arguments["target"])
                self.repair_history.append({**arguments, "receipt": receipt})
                _save(self.workdir / "maintenance.json", self.repair_history)
                result = {"receipt": receipt, "next_step": "Run check_repair; the action alone does not establish recovery."}
            elif name == "check_repair":
                result = self._check(arguments["probe"], arguments["duration_s"])
            elif name == "run_regression_suite":
                probes = list(dict.fromkeys([self.physics.default_probe, *self._probes]))[:2]
                result = {"cases": [self._check(probe, self.physics.default_duration_s) for probe in probes]}
            elif name == "submit_result":
                self.submitted = True
                self._freeze("agent_submission", arguments)
                result = {"submitted": True, "source_sha256": _sha(self.current_source.read_bytes()),
                          "next_step": "The host will verify the frozen maintenance sequence on a fresh specimen. No outcomes are known yet."}
            else:
                raise PlatformToolError("Unsupported operation.")
            # A final numeric serialization check prevents malformed tool results.
            result = json.loads(json.dumps(result, allow_nan=False))
            return {"ok": True, **result, "budget": self.budget_status()}
        except (PlatformToolError, WorkerError) as error:
            return {"ok": False, "error": str(error), "budget": self.budget_status()}
        except (ValueError, TypeError, KeyError, OSError, RuntimeError):
            # Platform exception strings can contain private config. Never forward them.
            return {"ok": False, "error": "The operation could not complete within the public interface. Inspect the declared controls, targets and parameter bounds.", "budget": self.budget_status()}

    def _freeze(self, reason, diagnosis=None):
        if self._frozen:
            return
        self._frozen = True
        frozen = self.workdir / "submission" / "model.py"
        frozen.parent.mkdir(exist_ok=False)
        frozen.write_bytes(self.current_source.read_bytes())
        frozen.chmod(0o444)
        self.current_source = frozen
        _save(frozen.parent / "submission.json", {"reason": reason, "agent_submitted": self.submitted,
              "source_sha256": _sha(frozen.read_bytes()), "maintenance": self.repair_history,
              "diagnosis": diagnosis, "verification_probes": list(VERIFICATION_PROBES[self.physics.platform])})

    def finalize(self, reason):
        """Freeze once; assess the same action sequence on a fresh host specimen."""
        if self._result is not None:
            return self._result
        initial = self.initial_evidence()
        self._freeze(reason)
        from .platform_physics import PlatformPhysics
        reserved = PlatformPhysics(self.physics.platform, self.workdir / "verification",
                                   scenario=self.physics.scenario,
                                   record_frames=getattr(self.physics, "record_frames", True))
        cases = []
        probes = list(VERIFICATION_PROBES[self.physics.platform])
        duration = self.physics.default_duration_s
        predictions = []
        for probe in probes:
            try:
                predictions.append(_public(self._model(reserved, probe, duration)))
            except (WorkerError, ValueError, RuntimeError, OSError):
                predictions.append({"probe": probe, "error": "The frozen model could not produce this prediction."})
        _save(self.workdir / "submission" / "predictions_locked.json", predictions)
        before = [reserved.run_experiment(probe, duration) for probe in probes]
        for action in self.repair_history:
            reserved.apply_repair(action["action"], action["target"])
        for index, probe in enumerate(probes):
            healthy = reserved.reference_probe(probe, duration)
            after = reserved.run_experiment(probe, duration)
            case = {"probe": probe, "duration_s": duration, "before": _public(before[index]),
                    "after": _public(after), "healthy": _public(healthy), "prediction": predictions[index],
                    "before_difference": compare_records(healthy, before[index]),
                    "after_difference": compare_records(healthy, after),
                    "prediction_difference": compare_records(predictions[index], after)}
            cases.append(case)
        self._result = {"kind": "platform_repair_verification", "platform": self.physics.platform,
                        "source_sha256": _sha(self.current_source.read_bytes()), "agent_submitted": self.submitted,
                        "maintenance": self.repair_history, "initial_difference": initial["initial_difference"],
                        "cases": cases,
                        "interpretation": "The model and maintenance sequence were frozen, then checked on a fresh synthetic specimen. Before/after differences and probe envelopes are evidence, not general physical validity or safety claims. Physical maintenance is distinct from Python source extension."}
        _save(self.workdir / "verification_result.json", self._result)
        return self._result
