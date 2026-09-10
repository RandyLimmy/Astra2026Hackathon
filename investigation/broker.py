"""Neutral investigation tools over host-owned physics and source versions."""

from __future__ import annotations

import ast
from datetime import datetime, timezone
import difflib
import hashlib
import json
from pathlib import Path
import time
from uuid import uuid4

from component_worker import WheelActuatorWorker, WorkerError

from .patching import MAX_SOURCE_BYTES, PatchError, apply_actuator_diff


DEFAULTS = {"speed_mps": 25.0, "brake_strength": 1.0, "preparation_cycles": 0,
            "wait_s": 0.0, "wall_distance_m": None}
CONFIG_SCHEMA = {
    "type": "object", "additionalProperties": False, "required": list(DEFAULTS),
    "properties": {
        "speed_mps": {"type": ["number", "null"], "description": "Speed in m/s, 5–30; null selects 25."},
        "brake_strength": {"type": ["number", "null"], "description": "Pedal fraction, 0.2–1; null selects 1."},
        "preparation_cycles": {"type": ["integer", "null"], "description": "Preparation repetitions, 0–5; null selects 0."},
        "wait_s": {"type": ["number", "null"], "description": "Waiting duration, 0–120 seconds; null selects 0."},
        "wall_distance_m": {"type": ["number", "null"], "description": "Barrier distance, 10–150 metres; null removes the barrier."},
    },
}


def _schema(name, description, properties):
    return {"type": "function", "name": name, "description": description, "strict": True,
            "parameters": {"type": "object", "properties": properties,
                           "required": list(properties), "additionalProperties": False}}


TEXT = {"type": "string"}
TOOL_SCHEMAS = [
    _schema("inspect_model", "Read the current editable source, its hash and freshly initialized own state.", {}),
    _schema("replace_model_source", "Replace all of actuator.py using the current source_sha256 from inspect_model as expected_sha256. Preferred for source edits; shares three total attempts with patch_model, including rejections. Keep the required interface.",
            {"source": TEXT, "expected_sha256": TEXT, "rationale": TEXT}),
    _schema("patch_model", "Apply a unified diff to actuator.py only. Shares three total edit attempts with replace_model_source, including rejections; full-source replacement avoids diff hunk counts.",
            {"diff": TEXT, "rationale": TEXT}),
    _schema("run_experiment", "Test a fresh specimen after recording a hypothesis and expected observation; six additional attempts maximum.",
            {"config": CONFIG_SCHEMA, "hypothesis": TEXT, "expected_observation": TEXT}),
    _schema("observe_run", "Read a permitted run owned by this investigation using its opaque id.", {"id": TEXT}),
    _schema("run_model", "Predict with the current source; reuse matching observed preparation if available. Twelve attempts maximum.",
            {"config": CONFIG_SCHEMA, "rationale": TEXT}),
    _schema("run_regression_suite", "Check the current source against the two cached initial development cases; no new reference experiments. Three calls maximum.",
            {"rationale": TEXT}),
    _schema("submit_prediction", "Freeze the current source for separate reserved evaluation. No reserved outcomes are returned; subsequent edits and runs close.",
            {"rationale": TEXT}),
]
ARGUMENTS = {entry["name"]: set(entry["parameters"]["required"]) for entry in TOOL_SCHEMAS}
MODEL_EDIT_LIMIT = 3
EDIT_TOOLS = frozenset({"patch_model", "replace_model_source"})
LIMITS = {"run_experiment": 6, "patch_model": MODEL_EDIT_LIMIT,
          "replace_model_source": MODEL_EDIT_LIMIT, "run_model": 12, "run_regression_suite": 3}
SUMMARY_FIELDS = {"collision", "impact_speed", "collision_time", "stopped", "censored", "brake_start_x",
                  "brake_start_time", "stopping_distance", "final_front_x", "final_speed", "wall_clearance",
                  "max_yaw_degrees", "max_lateral_displacement", "lane_departure", "trial_duration"}
OBSERVATION_FIELDS = {"time", "phase", "phase_time", "position", "velocity", "yaw", "yaw_rate", "wheel_speed",
                      "throttle", "brake", "front_x", "wall_contact", "lane_departure"}


class BrokerError(ValueError):
    pass


def _hash(source: bytes) -> str:
    return hashlib.sha256(source).hexdigest()


def _stamp() -> str:
    return datetime.now(timezone.utc).isoformat()


def _write(path: Path, payload) -> None:
    with path.open("x", encoding="utf-8") as stream:
        json.dump(payload, stream, allow_nan=False, indent=2)
        stream.write("\n")


class InvestigationBroker:
    """Host-owned development broker; candidate code is never imported here.

    ``physics`` supplies synchronous reference(config) and
    model(config, source, history=None) operations. Logs and source paths stay
    host-side. The initial fixture does not consume additional-tool budgets.
    """

    def __init__(self, workdir: Path, physics, log=None):
        self.workdir = Path(workdir).resolve()
        self.workdir.mkdir(parents=True, exist_ok=True)
        self.versions = self.workdir / "versions"
        self.versions.mkdir(exist_ok=False)
        self.attempts = self.workdir / "patch_attempts"
        self.attempts.mkdir(exist_ok=False)
        self.physics = physics
        self._logger = log
        self._started = time.monotonic()
        self._counts = {name: 0 for name in LIMITS}
        self._dispatches = 0
        self._runs = {}
        self._reference_by_config = {}
        self._initial = None
        self._development = []
        self._frozen_source = None
        self._submission = None
        baseline = Path(__file__).resolve().parents[1] / "candidate" / "wheel_actuator.py"
        self._current_source = self._version("v000", baseline.read_bytes())
        self._initial_source = self._current_source
        self._log("broker_started", {"source_sha256": _hash(self._source_bytes())})

    @property
    def current_source(self) -> Path:
        return self._current_source

    @property
    def frozen_source(self) -> Path | None:
        return self._frozen_source

    def _version(self, name, source):
        directory = self.versions / name
        directory.mkdir(exist_ok=False)
        path = directory / "actuator.py"
        with path.open("xb") as stream:
            stream.write(source)
        path.chmod(0o444)
        return path

    def _source_bytes(self):
        if self.current_source.is_symlink() or not self.current_source.is_file():
            raise BrokerError("The current source version is unavailable.")
        content = self.current_source.read_bytes()
        if len(content) > 65_536:
            raise BrokerError("The current source version exceeds its size limit.")
        return content

    def _log(self, event, details):
        record = {"event": event, "timestamp": _stamp(), **details}
        with (self.workdir / "broker_events.jsonl").open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, allow_nan=False) + "\n")
        if self._logger is not None:
            self._logger(record)

    def budget_status(self):
        return {"tool_calls": {"used": self._dispatches, "limit": 30},
                **{name: {"used": count, "limit": LIMITS[name]} for name, count in self._counts.items()},
                "model_edits": {"used": self._edit_count(), "limit": MODEL_EDIT_LIMIT,
                                "tools": sorted(EDIT_TOOLS)},
                "elapsed_s": round(time.monotonic() - self._started, 2), "time_limit_s": 1800,
                "frozen": self.frozen_source is not None}

    def _edit_count(self):
        return sum(self._counts[name] for name in EDIT_TOOLS)

    def _config(self, config):
        if not isinstance(config, dict) or set(config) - set(DEFAULTS):
            raise BrokerError("Configuration contains unsupported fields; each experiment uses a fresh specimen.")
        values = dict(DEFAULTS)
        for key, value in config.items():
            if value is not None or key == "wall_distance_m":
                values[key] = value
        from .physics import normalize_config
        try:
            return normalize_config(values)
        except (ValueError, TypeError, OverflowError):
            raise BrokerError("Configuration values are outside the permitted ranges.") from None

    def _key(self, config):
        return json.dumps(config, sort_keys=True, allow_nan=False)

    def _text(self, value, *, maximum=4000):
        if not isinstance(value, str) or not value.strip() or len(value) > maximum:
            raise BrokerError("Required explanation must be nonempty text within its size limit.")
        return value

    def _project(self, result, config, *, source=None):
        # Project before any tool can return a host result or cached recording.
        if not isinstance(result, dict) or not isinstance(result.get("summary"), dict):
            raise BrokerError("The run did not produce the required public result.")
        observations = result.get("observations", [])
        if not isinstance(observations, list) or len(observations) > 400 or not all(isinstance(row, dict) for row in observations):
            raise BrokerError("The run exceeded the public observation limit.")
        history = result.get("preparation_history", [])
        if not isinstance(history, list) or len(history) > 1000:
            raise BrokerError("The run produced invalid preparation history.")
        for event in history:
            if not isinstance(event, dict):
                raise BrokerError("The run produced invalid preparation history.")
            allowed = {"kind", "phase", "speed_mps"} if event.get("kind") == "reset" else {
                "kind", "phase", "throttle", "brake", "dt_s", "steps"}
            if set(event) - allowed or event.get("kind") not in {"reset", "step"}:
                raise BrokerError("The run produced unsupported preparation fields.")
        public = {"config": config, "preparation_history": history,
                  "observations": [{key: value for key, value in row.items() if key in OBSERVATION_FIELDS} for row in observations],
                  "summary": {key: value for key, value in result["summary"].items() if key in SUMMARY_FIELDS}}
        if source is not None:
            expected = _hash(source.read_bytes())
            if result.get("source_sha256") != expected:
                raise BrokerError("The prediction source could not be verified.")
            public["source_sha256"] = expected
        encoded = json.dumps(public, allow_nan=False)
        if len(encoded.encode()) > 1_000_000:
            raise BrokerError("The public result exceeds its size limit.")
        return json.loads(encoded)

    def _remember(self, result, *, reference=False):
        identifier = "run_" + uuid4().hex[:16]
        self._runs[identifier] = result
        if reference:
            self._reference_by_config[self._key(result["config"])] = result
        return identifier

    def initial_evidence(self):
        if self._initial is not None:
            return self._initial
        cases = []
        for cycles in (0, 4):
            config = self._config({"preparation_cycles": cycles})
            reference = self._project(self.physics.reference(config), config)
            reference_id = self._remember(reference, reference=True)
            prediction = self._project(self.physics.model(config, self._initial_source,
                                                          history=reference["preparation_history"]),
                                       config, source=self._initial_source)
            prediction_id = self._remember(prediction)
            self._development.append({"config": config, "reference": reference, "baseline": prediction})
            # Full permitted preparations remain available via observe_run;
            # initial model summaries avoid duplicating the reference telemetry.
            cases.append({"config": config, "reference_id": reference_id,
                          "reference_summary": reference["summary"],
                          "preparation_history": reference["preparation_history"],
                          "reference_observations": reference["observations"],
                          "initial_prediction_id": prediction_id,
                          "initial_prediction_summary": prediction["summary"]})
        self._initial = {"cases": cases, "source_sha256": _hash(self._source_bytes()),
                         "specimen_policy": "Every experiment begins with a fresh specimen; continuation is unsupported.",
                         "budget": self.budget_status()}
        _write(self.workdir / "initial_evidence.json", self._initial)
        self._log("initial_evidence_ready", {"reference_runs": 2, "initial_predictions": 2})
        return self._initial

    def dispatch(self, name, args):
        try:
            if self._dispatches >= 30:
                raise BrokerError("The total tool-call budget is exhausted.")
            self._dispatches += 1
            if name not in ARGUMENTS:
                raise BrokerError("Unknown investigation tool.")
            if time.monotonic() - self._started >= 1800:
                raise BrokerError("The investigation time budget is exhausted.")
            if self.frozen_source is not None and name in {*LIMITS, "submit_prediction"}:
                raise BrokerError("The source is frozen; further edits and executions are closed.")
            if name in LIMITS:
                if name in EDIT_TOOLS and self._edit_count() >= MODEL_EDIT_LIMIT:
                    raise BrokerError("The shared source-edit attempt budget is exhausted.")
                if self._counts[name] >= LIMITS[name]:
                    raise BrokerError("This tool's attempt budget is exhausted.")
                self._counts[name] += 1
            if not isinstance(args, dict) or set(args) != ARGUMENTS[name]:
                raise BrokerError("Tool arguments do not match the required fields.")
            if len(json.dumps(args, allow_nan=False).encode()) > 65_536:
                raise BrokerError("Tool arguments exceed the message-size limit.")
            self._log("tool_started", {"tool": name, "tool_call": self._dispatches})
            result = getattr(self, "_" + name)(**args)
            self._log("tool_completed", {"tool": name, "tool_call": self._dispatches})
            return {"ok": True, **result, "budget": self.budget_status()}
        except (BrokerError, PatchError, WorkerError) as error:
            payload = {"code": "request_rejected", "message": str(error)}
            if isinstance(error, WorkerError) and error.source_line is not None:
                payload["source_line"] = error.source_line
            self._log("tool_rejected", {"tool": str(name)[:80], "error": payload})
            return {"ok": False, "error": payload, "budget": self.budget_status()}
        except Exception as error:
            self._log("host_tool_error", {"tool": str(name)[:80], "exception_type": type(error).__name__, "detail": str(error)})
            return {"ok": False, "error": {"code": "execution_failed", "message": "The requested operation could not complete."},
                    "budget": self.budget_status()}

    def _inspect_model(self):
        source = self._source_bytes()
        with WheelActuatorWorker(self.current_source) as worker:
            state = worker.inspect_state()
            fingerprint = worker.source_sha256
        return {"source": source.decode("utf-8"), "source_sha256": fingerprint,
                "state": state, "state_status": "freshly_initialized"}

    def _patch_model(self, diff, rationale):
        self._text(rationale)
        before = self._source_bytes().decode("utf-8")
        directory = self._edit_attempt("patch_model", rationale, {"diff": diff})
        updated = apply_actuator_diff(before, diff)
        return self._validate_edit(updated, before, directory, "patch_model", difference=diff)

    def _replace_model_source(self, source, expected_sha256, rationale):
        self._text(rationale)
        before = self._source_bytes()
        directory = self._edit_attempt("replace_model_source", rationale,
                                       {"source": source, "expected_sha256": expected_sha256})
        if (not isinstance(expected_sha256, str) or len(expected_sha256) != 64
                or any(char not in "0123456789abcdef" for char in expected_sha256)):
            raise BrokerError("expected_sha256 must be the current 64-character source_sha256 from inspect_model.")
        if expected_sha256 != _hash(before):
            raise BrokerError("The source has changed; call inspect_model and retry with its source_sha256.")
        if (not isinstance(source, str) or not source.strip()
                or len(source.encode("utf-8")) > MAX_SOURCE_BYTES):
            raise BrokerError("Replacement source must be nonempty UTF-8 text of at most 65536 bytes.")
        if "\r" in source or "\x00" in source:
            raise BrokerError("Replacement source must use ordinary UTF-8 text with Unix newlines.")
        if source.encode("utf-8") == before:
            raise BrokerError("Replacement does not change the actuator source.")
        return self._validate_edit(source, before.decode("utf-8"), directory, "replace_model_source")

    def _edit_attempt(self, tool, rationale, request):
        attempt = self._edit_count()
        directory = self.attempts / f"attempt_{attempt:03d}"
        directory.mkdir(exist_ok=False)
        _write(directory / "rationale.json", {"rationale": rationale})
        _write(directory / "request.json", {"tool": tool, **request})
        return directory

    def _validate_edit(self, updated, before, directory, tool, *, difference=None):
        # Both edit paths use the same source/interface limits and OS-isolated
        # execution. Candidate code is parsed here, never imported or executed.
        attempt = self._edit_count()
        if difference is None:
            difference = "".join(
                line if line.endswith("\n") else line + "\n\\ No newline at end of file\n"
                for line in difflib.unified_diff(
                    before.splitlines(keepends=True), updated.splitlines(keepends=True),
                    fromfile="a/actuator.py", tofile="b/actuator.py"))
        (directory / "patch.diff").write_text(difference, encoding="utf-8")
        path = directory / "actuator.py"
        path.write_bytes(updated.encode("utf-8"))
        path.chmod(0o444)
        try:
            tree = ast.parse(updated, filename="actuator.py")
            if sum(1 for _ in ast.walk(tree)) > 20_000:
                raise BrokerError("Edited source exceeds its structural complexity limit.")
            compile(tree, "actuator.py", "exec")
        except SyntaxError as error:
            line = error.lineno
            if type(line) is int and 1 <= line <= updated.count("\n") + 1:
                raise WorkerError("Actuator source contains a syntax error.", source_line=line) from None
            raise BrokerError("Actuator source contains a syntax error.") from None
        required = {"init_state", "compute_brake_torque_limits", "advance_state", "on_trial_reset"}
        declared = [node.name for node in tree.body if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))]
        if any(declared.count(name) != 1 for name in required):
            raise BrokerError("The edited source must define each required component function once.")
        with WheelActuatorWorker(path) as worker:
            torque = worker.torque_limits(0.5, [10.0] * 4)
            worker.advance(0.5, [10.0] * 4, [-value for value in torque], 0.002)
            worker.reposition()
            worker.inspect_state()
            fingerprint = worker.source_sha256
        self._current_source = self._version(f"v{attempt:03d}", updated.encode())
        self._log("patch_validated", {"attempt": attempt, "tool": tool, "source_sha256": fingerprint})
        return {"source_sha256": fingerprint, "version": f"v{attempt:03d}",
                "validation": "Syntax, required functions and isolated interface smoke check passed; predictive quality is not yet established."}

    def _run_experiment(self, config, hypothesis, expected_observation):
        normalized = self._config(config)
        self._text(hypothesis)
        self._text(expected_observation)
        self._log("experiment_expectation", {"config": normalized, "hypothesis": hypothesis,
                                              "expected_observation": expected_observation,
                                              "attempt": self._counts["run_experiment"]})
        result = self._project(self.physics.reference(normalized), normalized)
        identifier = self._remember(result, reference=True)
        return {"id": identifier, "summary": result["summary"]}

    def _observe_run(self, id):
        if not isinstance(id, str) or len(id) > 80 or id not in self._runs:
            raise BrokerError("That run is not available to this investigation.")
        return {"id": id, "run": self._runs[id]}

    def _run_model(self, config, rationale):
        normalized = self._config(config)
        self._text(rationale)
        permitted = self._reference_by_config.get(self._key(normalized))
        history = permitted["preparation_history"] if permitted is not None else None
        self._log("prediction_requested", {"config": normalized, "rationale": rationale})
        result = self._project(self.physics.model(normalized, self.current_source, history=history),
                               normalized, source=self.current_source)
        identifier = self._remember(result)
        return {"id": identifier, "summary": result["summary"], "source_sha256": result["source_sha256"],
                "preparation": "matching_permitted_history" if history is not None else "own_model_history"}

    def _run_regression_suite(self, rationale):
        self._text(rationale)
        if len(self._development) != 2:
            raise BrokerError("Initial development evidence is not ready.")
        results = []
        for case in self._development:
            reference = case["reference"]["summary"]
            current = self._project(self.physics.model(case["config"], self.current_source,
                                                       history=case["reference"]["preparation_history"]),
                                    case["config"], source=self.current_source)["summary"]
            baseline = case["baseline"]["summary"]

            def error(summary):
                a, b = summary.get("stopping_distance"), reference.get("stopping_distance")
                return abs(a - b) if a is not None and b is not None else None

            baseline_error, current_error = error(baseline), error(current)
            correct = current.get("collision") == reference.get("collision")
            tolerance = max(0.5, 0.05 * abs(reference.get("stopping_distance") or 0))
            no_regression = (current_error is not None and baseline_error is not None
                             and current_error <= baseline_error + tolerance and correct)
            results.append({"config": case["config"], "baseline_stopping_error_m": baseline_error,
                            "current_stopping_error_m": current_error, "collision_prediction_correct": correct,
                            "passed": no_regression})
        return {"suite": "initial_development_v1", "cases": results,
                "passed": all(case["passed"] for case in results), "new_reference_runs": 0}

    def _freeze(self, *, agent_submitted, rationale):
        if self._submission is not None:
            return dict(self._submission)
        source = self._source_bytes()
        directory = self.workdir / "submission"
        directory.mkdir(exist_ok=False)
        path = directory / "actuator.py"
        path.write_bytes(source)
        path.chmod(0o444)
        self._frozen_source = path
        self._submission = {"id": "submission_" + uuid4().hex[:16], "source_sha256": _hash(source),
                            "status": "agent_submitted" if agent_submitted else "host_frozen",
                            "agent_submitted": agent_submitted, "frozen_at": _stamp(), "reserved_outcomes_revealed": False}
        _write(directory / "manifest.json", {**self._submission, "rationale": rationale})
        self._log("source_frozen", self._submission)
        return dict(self._submission)

    def _submit_prediction(self, rationale):
        self._text(rationale)
        return self._freeze(agent_submitted=True, rationale=rationale)

    def freeze(self):
        """Host fallback after early stopping; never label it agent submission."""
        return self._freeze(agent_submitted=False, rationale="Host ended the development investigation.")


Broker = InvestigationBroker
