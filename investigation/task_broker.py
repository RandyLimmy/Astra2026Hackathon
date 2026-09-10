"""Bounded, versioned controller edits with actual RGB evidence and full trials."""
from copy import deepcopy
import difflib
import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace
import time

from .platform_broker import _schema, TEXT
from .platform_story import Story, atomic_json
from .task_recording import record_task

ROOT = Path(__file__).resolve().parents[1]


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


def digest(value):
    return hashlib.sha256(canonical(value).encode()).hexdigest()


class TaskBroker:
    task_kind = "controller_repair"
    system_prompt = (ROOT / "investigation/prompts/control_task_system.md").read_text()
    contract_text = ("The editable controller is a complete JSON object matching inspect_controller. "
                     "Only declared controller settings may change. Full trials retain the same physical world, "
                     "initial conditions, mission and deadline. Use view_frames for actual RGB images, "
                     "observe_run for public telemetry, and run_trial for measured outcomes.")
    task_instruction = ("Investigate the supplied failed control task. Inspect original images and telemetry, "
                        "install a warranted controller correction, and run the complete task to measure its effect. "
                        "Do not substitute a predicted success for an actual successful task.")

    def __init__(self, workdir, adapter, *, frames=True):
        self.workdir = Path(workdir).resolve()
        self.workdir.mkdir(parents=True, exist_ok=False)
        self.root = self.workdir.parent
        self.adapter = adapter
        self.capabilities = adapter.capabilities()
        self.platform = self.capabilities["platform"]
        self.scenario = self.capabilities["scenario"]
        self.frames = frames
        self._started = time.monotonic()
        self._calls = self._edits = self._trials = self._views = 0
        self._records = {}
        self._baseline = None
        self._initial = None
        self._result = None
        self._frozen = False
        self.submitted = False
        self._pending_images = []
        self.candidate = adapter.validate_candidate(adapter.initial_candidate())
        self.current_source = self._version(self.candidate, 0)
        self.predeclared_criteria = {"goal": self.capabilities["goal"],
            "physical_goal": "Complete the entire original mission under its declared task criteria; stability alone is insufficient.",
            "task_criteria": self.capabilities.get("success_criteria", self.capabilities.get("criteria", {})),
            "duration_s": adapter.duration_s, "probes": [self.scenario], "declared_before_run": True,
            "scope": "One fixed synthetic control task; no broader reliability claim."}
        atomic_json(self.workdir / "verification_plan.json", self.predeclared_criteria)
        self.story = Story(self.root, self.platform, self.predeclared_criteria)
        self.story.goal["criteria"].append(canonical(self.predeclared_criteria["task_criteria"]))
        self.story.publish()
        self.protocol_paths = {"investigation/task_broker.py", "investigation/task_recording.py",
                               "investigation/task_run.py", "investigation/tasks/__init__.py",
                               "investigation/prompts/control_task_system.md", *adapter.source_paths}
        cameras = {"type": "string", "enum": list(adapter.cameras)}
        self.tool_schemas = [
            _schema("inspect_system", "Read public task, controller scope, sensors, cameras and unchanged success criteria.", {}),
            _schema("inspect_controller", "Read the complete editable controller, current hash, public source and bounds.", {}),
            _schema("observe_run", "Read public measurements and observed events from one owned recording. At most80 samples per read; use a time interval to inspect motion closely.",
                    {"id": TEXT, "start_s": {"type": "number", "minimum": 0},
                     "end_s": {"type": "number", "minimum": 0}, "max_samples": {"type": "integer", "minimum": 2, "maximum": 80}}),
            _schema("view_frames", "Inspect actual unannotated RGB images from an owned recording. Image content follows this tool result; choose up to4 times and a camera.",
                    {"id": TEXT, "times_s": {"type": "array", "items": {"type": "number", "minimum": 0}, "minItems": 1, "maxItems": 4}, "camera": cameras}),
            _schema("replace_controller", "Install a complete controller JSON object using the latest hash. This is an actual controller edit; run_trial measures whether it works. No world/task edits allowed.",
                    {"controller_json": TEXT, "expected_sha256": TEXT, "rationale": TEXT, "expected_effect": TEXT}),
            _schema("run_trial", "Run the complete task from its original initial conditions with the current controller; record motion, images and the unchanged task outcome.",
                    {"rationale": TEXT, "expected_observation": TEXT}),
            _schema("submit_result", "Freeze this controller for a fresh complete host trial. State actual edits, observations and remaining uncertainty.",
                    {"diagnosis": TEXT, "evidence": TEXT, "remaining_uncertainty": TEXT}),
        ]

    def _version(self, candidate, number):
        path = self.workdir / "versions" / f"v{number:03d}" / "controller.json"
        path.parent.mkdir(parents=True, exist_ok=False)
        # Canonical on disk: the tool hash, recording identity and frozen hash agree.
        path.write_text(canonical(candidate))
        path.chmod(0o444)
        return path

    def budget_status(self):
        return {"tool_calls": {"used": self._calls, "limit": 40},
                "controller_edits": {"used": self._edits, "limit": 6},
                "full_trials": {"used": self._trials, "limit": 6},
                "image_reads": {"used": self._views, "limit": 8},
                "elapsed_s": round(time.monotonic() - self._started, 2), "frozen": self._frozen}

    @staticmethod
    def _sample(rows, count):
        if len(rows) <= count:
            return rows
        return [rows[round(i * (len(rows) - 1) / (count - 1))] for i in range(count)]

    def _remember(self, record):
        identifier = f"run_{len(self._records) + 1:04d}"
        self._records[identifier] = record
        return self._public(identifier)

    def _public(self, identifier, count=16):
        record = self._records[identifier]
        return {"id": identifier, **{key: record[key] for key in ("probe", "duration_s", "actual_duration_s", "summary", "goal_achieved", "partial_success", "source_sha256", "events")},
                "observations": self._sample(record["observations"], count),
                "cameras": list(self.adapter.cameras), "images_available": bool(record["frames"])}

    def _record(self, candidate, kind, label, *, final=False):
        base = self.workdir / "verification" if final else self.root / "physics"
        record = record_task(self.adapter, candidate, base, frames=self.frames, kind=kind)
        self.story.replay(SimpleNamespace(workdir=base), record, kind, label)
        self.story.publish()
        return record

    def initial_evidence(self):
        if self._initial is None:
            self._baseline = self._record(self.adapter.initial_candidate(), "incident", "Original full task")
            self.story.replay(SimpleNamespace(workdir=self.root / "physics"), self._baseline, "before", "Original full task")
            # A single actual record can serve both the original scene and the baseline.
            if not any(item["kind"] == "before" for item in self.story.replays):
                first = next(item for item in self.story.replays if item["id"] == self._baseline["id"])
                self.story.replays.append({**first, "kind": "before", "label": "Original full task"})
            observed = self._remember(self._baseline)
            self.story.initial_mismatch = not self._baseline["goal_achieved"]
            self._initial = {"capabilities": self.capabilities, "observed": observed,
                             "initial_controller": deepcopy(self.candidate),
                             "initial_difference": {"observed_outcome": observed["summary"].get("outcome"),
                                                    "observed_within_envelope": observed["goal_achieved"]},
                             "interpretation": "This is a full failed task. Camera images require view_frames; filenames alone are not image evidence."}
            atomic_json(self.workdir / "initial_evidence.json", self._initial)
            self.story.publish()
        return deepcopy(self._initial)

    def _freeze(self, reason, submission=None):
        if not self._frozen:
            self._frozen = True
            target = self.workdir / "submission" / "controller.json"
            target.parent.mkdir(exist_ok=False)
            target.write_bytes(self.current_source.read_bytes())
            target.chmod(0o444)
            self.current_source = target
            atomic_json(target.parent / "submission.json", {"reason": reason, "controller": self.candidate,
                "source_sha256": digest(self.candidate), "agent_submitted": self.submitted, "submission": submission})

    def dispatch(self, name, arguments):
        entry = self.story.begin(name, arguments)
        before = deepcopy(self.candidate)
        try:
            if self._frozen or self._calls >= 40:
                raise ValueError("The controller is frozen or the tool budget is exhausted.")
            self._calls += 1
            schemas = {tool["name"]: tool["parameters"] for tool in self.tool_schemas}
            if name not in schemas or not isinstance(arguments, dict) or set(arguments) != set(schemas[name]["required"]):
                raise ValueError("Arguments must match the declared tool fields exactly.")
            for key in ("rationale", "expected_effect", "expected_observation", "diagnosis", "evidence", "remaining_uncertainty"):
                if key in arguments and (not isinstance(arguments[key], str) or not arguments[key].strip() or len(arguments[key]) > 12000):
                    raise ValueError("Provide a nonempty concise explanation.")
            if name == "inspect_system":
                result = deepcopy(self.capabilities)
            elif name == "inspect_controller":
                result = {"controller": deepcopy(self.candidate), "source_sha256": digest(self.candidate),
                          "controller_source": self.capabilities["controller_source"], "capabilities": deepcopy(self.capabilities)}
            elif name in {"observe_run", "view_frames"}:
                identifier = arguments["id"]
                if not isinstance(identifier, str) or identifier not in self._records:
                    raise ValueError("Choose a recording ID returned in this investigation.")
                record = self._records[identifier]
                if name == "observe_run":
                    start, end, count = arguments["start_s"], arguments["end_s"], arguments["max_samples"]
                    if (any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in (start, end))
                            or not 0 <= start <= end <= self.adapter.duration_s or type(count) is not int or not 2 <= count <= 80):
                        raise ValueError("Choose an interval within the full task and2–80 samples.")
                    rows = [row for row in record["observations"] if start <= row["t_s"] <= end]
                    result = {**self._public(identifier), "observations": self._sample(rows, count)}
                else:
                    self._views += 1
                    if self._views > 8:
                        raise ValueError("Image-read budget exhausted.")
                    times, camera = arguments["times_s"], arguments["camera"]
                    if (camera not in self.adapter.cameras or not isinstance(times, list) or not 1 <= len(times) <= 4
                            or any(isinstance(t, bool) or not isinstance(t, (int, float)) or not math.isfinite(t) or not 0 <= t <= self.adapter.duration_s for t in times)):
                        raise ValueError("Choose a declared camera and1–4 times within the recording.")
                    frames = record["evidence_frames"][camera]
                    if not frames:
                        raise ValueError("This run has no image recording.")
                    selected = [min(frames, key=lambda row: abs(row["t_s"] - t)) for t in times]
                    validated_images = []
                    image_receipts = []
                    for frame in selected:
                        base = self.root / "physics"
                        original_path = base / frame["file"]
                        path = original_path.resolve()
                        owner = (base / record["id"] / "frames").resolve()
                        if (not owner.is_relative_to(base.resolve()) or not path.is_relative_to(owner)
                                or not path.is_file() or original_path.is_symlink()):
                            raise ValueError("Recorded image unavailable.")
                        image_receipts.append({"t_s": frame["t_s"], "sha256": hashlib.sha256(path.read_bytes()).hexdigest()})
                        validated_images.append({"path": path, "label": f"{identifier}, {camera}, t={frame['t_s']:.3f}s; unannotated recorded RGB"})
                    self._pending_images.extend(validated_images)
                    result = {"id": identifier, "camera": camera,
                              "images": image_receipts,
                              "meaning": "Actual image content accompanies these observations."}
            elif name == "replace_controller":
                self._edits += 1
                if self._edits > 6:
                    raise ValueError("Controller edit budget exhausted.")
                if arguments["expected_sha256"] != digest(self.candidate):
                    raise ValueError("Stale controller hash; inspect the current controller first.")
                raw = arguments["controller_json"]
                if not isinstance(raw, str) or len(raw) > 16000:
                    raise ValueError("Controller must be bounded JSON text.")
                def unique(pairs):
                    value = {}
                    for key, item in pairs:
                        if key in value:
                            raise ValueError("Duplicate controller field.")
                        value[key] = item
                    return value
                candidate = self.adapter.validate_candidate(json.loads(raw, object_pairs_hook=unique))
                self.current_source = self._version(candidate, self._edits)
                self.candidate = candidate
                result = {"accepted": True, "changed": candidate != before, "source_sha256": digest(candidate), "controller": deepcopy(candidate),
                          "meaning": "Controller installed. Run the task to measure its effect." if candidate != before else
                                     "Controller is identical; no effective change was applied."}
            elif name == "run_trial":
                self._trials += 1
                if self._trials > 6:
                    raise ValueError("Full-trial budget exhausted.")
                result = self._remember(self._record(self.candidate, "diagnostic", "Controller development trial"))
            elif name == "submit_result":
                self.submitted = True
                self._freeze("agent_submission", arguments)
                result = {"submitted": True, "source_sha256": digest(self.candidate), "meaning": "Host final verification is pending."}
            result = {"ok": True, **result, "budget": self.budget_status()}
        except (ValueError, TypeError, KeyError) as error:
            result = {"ok": False, "error": str(error), "budget": self.budget_status()}
        except (RuntimeError, OSError):
            result = {"ok": False, "error": "The full physical trial or owned artifact could not complete.", "budget": self.budget_status()}
        installed = name == "replace_controller" and result["ok"]
        diff = "".join(difflib.unified_diff((json.dumps(before, indent=2) + "\n").splitlines(True),
                                          (json.dumps(self.candidate, indent=2) + "\n").splitlines(True),
                                          fromfile="before/controller.json", tofile="after/controller.json")) if installed else None
        self.story.finish(entry, result, before=before if installed else None,
                          after=self.candidate if installed else None, source_diff=diff)
        if installed and before == self.candidate:
            entry["stage"] = "unchanged"
            entry["result"]["meaning"] = "The controller was identical; no effective change was applied."
            self.story.publish()
        if name == "run_trial" and result.get("ok"):
            entry["result"].update(goal_achieved=result["goal_achieved"], partial_success=result["partial_success"])
            self.story.publish()
        return result

    def pop_images(self):
        images, self._pending_images = self._pending_images, []
        return images

    def record_explanation(self, text):
        self.story.explanation(text)

    def publish_story(self):
        self.story.publish()

    def action_summary(self):
        return self.story.action_summary()

    def finalize(self, reason):
        if self._result is not None:
            return self._result
        self.initial_evidence()
        self._freeze(reason)
        final = self._record(self.candidate, "after", "Fresh final task with frozen controller", final=True)
        def difference(record):
            return {"observed_outcome": record["summary"].get("outcome"),
                    "observed_within_envelope": record["goal_achieved"],
                    "common_duration_s": record["actual_duration_s"], "position_rmse_m": None}
        case = {"probe": self.scenario, "duration_s": self.adapter.duration_s,
                "goal_achieved": final["goal_achieved"], "partial_success": final["partial_success"],
                "before": {k: self._baseline[k] for k in ("summary", "duration_s", "goal_achieved", "partial_success")},
                "after": {k: final[k] for k in ("summary", "duration_s", "goal_achieved", "partial_success")},
                "before_difference": difference(self._baseline), "after_difference": difference(final)}
        self.story.verification = {"status": "completed", "goal_achieved": final["goal_achieved"],
                                   "partial_success": final["partial_success"], "predictive_success": None, "cases": [case]}
        self._result = {"kind": "control_task_verification", "task_kind": self.task_kind, "platform": self.platform,
                        "scenario": self.scenario, "source_sha256": digest(self.candidate),
                        "goal": self.story.goal, "predeclared_criteria": self.predeclared_criteria,
                        "agent_submitted": self.submitted, "controller": deepcopy(self.candidate),
                        "aggregate": {"goal_achieved": final["goal_achieved"], "partial_success": final["partial_success"],
                                      "predictive_success": None}, "action_summary": self.action_summary(), "cases": [case]}
        atomic_json(self.workdir / "verification_result.json", self._result)
        self.story.publish()
        return self._result
