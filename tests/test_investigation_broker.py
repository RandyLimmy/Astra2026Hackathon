"""Broker budgets, prospective records, isolated patches, and freeze boundary."""

import difflib
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest

from component_worker import WheelActuatorWorker
from investigation.broker import InvestigationBroker, TOOL_SCHEMAS
from investigation.patching import PatchError, apply_actuator_diff


def diff_for(before, after):
    return "".join(difflib.unified_diff(before.splitlines(keepends=True), after.splitlines(keepends=True),
                                         fromfile="a/actuator.py", tofile="b/actuator.py"))


class FakePhysics:
    def __init__(self):
        self.references = []
        self.models = []
        self.on_reference = None

    def result(self, config):
        return {"config": dict(config),
                "preparation_history": [{"kind": "reset", "phase": "trial", "speed_mps": config["speed_mps"]}],
                "observations": [{"time": 0.0, "phase": "trial", "phase_time": 0.0,
                                  "velocity": [config["speed_mps"], 0.0, 0.0]}],
                "summary": {"collision": False, "stopped": True, "censored": False,
                            "stopping_distance": 45.0 if config["preparation_cycles"] else 35.0,
                            "final_front_x": 37.0, "final_speed": 0.0}}

    def reference(self, config):
        if self.on_reference:
            self.on_reference()
        self.references.append(dict(config))
        return self.result(config)

    def model(self, config, source, history=None):
        self.models.append({"config": dict(config), "source": source, "history": history})
        result = self.result(config)
        result["summary"]["stopping_distance"] = 35.0
        result["source_sha256"] = hashlib.sha256(source.read_bytes()).hexdigest()
        return result


class PatchingTests(unittest.TestCase):
    def test_multiple_hunks_apply_exactly(self):
        before = "".join(f"line_{i}\n" for i in range(30))
        after = before.replace("line_2\n", "changed_2\n").replace("line_27\n", "new_27\nextra\n")
        self.assertEqual(apply_actuator_diff(before, diff_for(before, after)), after)

    def test_rejects_traversal_extra_files_symlink_metadata_and_bad_context(self):
        valid = diff_for("old\n", "new\n")
        malicious = [valid.replace("a/actuator.py", "a/../secret.py"),
                     valid + valid.replace("actuator.py", "another.py"),
                     "diff --git a/actuator.py b/actuator.py\nnew file mode 120000\n" + valid,
                     valid.replace("-old\n", "-different\n"),
                     valid.replace("--- a/actuator.py", "--- /dev/null")]
        for text in malicious:
            with self.subTest(text=text[:60]):
                with self.assertRaises(PatchError):
                    apply_actuator_diff("old\n", text)

    def test_no_newline_markers_preserve_content(self):
        patch = "--- a/actuator.py\n+++ b/actuator.py\n@@ -1 +1 @@\n-old\n\\ No newline at end of file\n+new\n\\ No newline at end of file\n"
        self.assertEqual(apply_actuator_diff("old", patch), "new")


@unittest.skipUnless(sys.platform == "darwin" and Path("/usr/bin/sandbox-exec").is_file(), "macOS component isolation")
class BrokerTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.directory = Path(self.temporary.name)
        self.physics = FakePhysics()
        self.events = []
        self.broker = InvestigationBroker(self.directory / "run", self.physics, log=self.events.append)

    def tearDown(self):
        self.temporary.cleanup()

    def experiment(self, config=None):
        return self.broker.dispatch("run_experiment", {"config": config or {},
                                    "hypothesis": "Prior operation may change the response.",
                                    "expected_observation": "The measured stopping distance changes."})

    def replace(self, source, expected_sha256=None):
        if expected_sha256 is None:
            expected_sha256 = hashlib.sha256(self.broker.current_source.read_bytes()).hexdigest()
        return self.broker.dispatch("replace_model_source", {
            "source": source, "expected_sha256": expected_sha256,
            "rationale": "Test a source-level component extension."})

    def test_initial_evidence_is_cached_and_does_not_spend_extra_budget(self):
        evidence = self.broker.initial_evidence()
        self.assertIs(self.broker.initial_evidence(), evidence)
        self.assertEqual(len(self.physics.references), 2)
        self.assertEqual(len(self.physics.models), 2)
        self.assertEqual([case["config"]["preparation_cycles"] for case in evidence["cases"]], [0, 4])
        self.assertEqual(self.broker.budget_status()["run_experiment"]["used"], 0)
        self.assertNotIn("initial_prediction_observations", evidence["cases"][0])

    def test_expectation_is_logged_before_reference_execution(self):
        def verify_order():
            self.assertEqual(self.events[-1]["event"], "experiment_expectation")
            saved = [json.loads(line) for line in (self.directory / "run" / "broker_events.jsonl").read_text().splitlines()]
            self.assertEqual(saved[-1]["event"], "experiment_expectation")
        self.physics.on_reference = verify_order
        result = self.experiment()
        self.assertTrue(result["ok"])
        self.assertNotIn("observations", result)
        observed = self.broker.dispatch("observe_run", {"id": result["id"]})
        self.assertTrue(observed["ok"])
        self.assertIn("preparation_history", observed["run"])

    def test_owned_ids_and_host_fields_are_not_exposed(self):
        result = self.experiment()
        other = InvestigationBroker(self.directory / "other", FakePhysics())
        self.assertFalse(other.dispatch("observe_run", {"id": result["id"]})["ok"])
        self.assertFalse(self.broker.dispatch("observe_run", {"id": "../private"})["ok"])
        self.assertNotIn(str(self.directory), json.dumps(result))

    def test_reference_budget_counts_failed_attempts_and_rejects_continuation(self):
        for _ in range(6):
            self.assertFalse(self.experiment({"continue": True})["ok"])
        self.assertFalse(self.experiment()["ok"])
        self.assertEqual(self.broker.budget_status()["run_experiment"]["used"], 6)
        self.assertEqual(self.physics.references, [])

    def test_model_reuses_only_matching_permitted_history_without_reference_calls(self):
        self.broker.initial_evidence()
        count = len(self.physics.references)
        matched = self.broker.dispatch("run_model", {"config": {"preparation_cycles": 4}, "rationale": "Check the observed case."})
        self.assertTrue(matched["ok"])
        self.assertIsNotNone(self.physics.models[-1]["history"])
        unmatched = self.broker.dispatch("run_model", {"config": {"speed_mps": 18}, "rationale": "Predict another input."})
        self.assertTrue(unmatched["ok"])
        self.assertIsNone(self.physics.models[-1]["history"])
        self.assertEqual(len(self.physics.references), count)

    def test_successful_patch_is_isolated_versioned_and_keeps_old_source(self):
        old_path = self.broker.current_source
        before = old_path.read_text()
        after = before.replace("835.0", "800.0")
        result = self.broker.dispatch("patch_model", {"diff": diff_for(before, after), "rationale": "Test a changed capacity."})
        self.assertTrue(result["ok"], result)
        self.assertNotEqual(self.broker.current_source, old_path)
        self.assertEqual(old_path.read_text(), before)
        self.assertEqual(self.broker.current_source.read_text(), after)
        self.assertEqual(self.broker.current_source.stat().st_mode & 0o222, 0)
        self.assertEqual(result["source_sha256"], hashlib.sha256(after.encode()).hexdigest())

    def test_replacement_installs_real_state_update_and_preserves_exact_versions(self):
        inspected = self.broker.dispatch("inspect_model", {})
        old_path = self.broker.current_source
        before = inspected["source"]
        # No final newline: exact replacement bytes and its generated diff must agree.
        after = '''def init_state():
    return {"elapsed_s": 0.0}

def compute_brake_torque_limits(state, brake_command, wheel_speed_rad_s):
    return [100.0 * brake_command / (1.0 + state["elapsed_s"])] * 4

def advance_state(state, brake_command, mean_wheel_speed_rad_s, applied_brake_torque_nm, dt_s):
    return {"elapsed_s": state["elapsed_s"] + dt_s}

def on_trial_reset(state):
    return state'''
        result = self.replace(after, inspected["source_sha256"])
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["version"], "v001")
        self.assertEqual(result["source_sha256"], hashlib.sha256(after.encode()).hexdigest())
        self.assertEqual(old_path.read_text(), before)
        self.assertEqual(self.broker.current_source.read_bytes(), after.encode())
        self.assertEqual(self.broker.current_source.stat().st_mode & 0o222, 0)
        attempt = self.broker.attempts / "attempt_001"
        self.assertEqual(apply_actuator_diff(before, (attempt / "patch.diff").read_text()), after)
        self.assertEqual(json.loads((attempt / "request.json").read_text())["source"], after)
        with WheelActuatorWorker(self.broker.current_source) as worker:
            self.assertEqual(worker.torque_limits(1.0, [10.0] * 4), [100.0] * 4)
            worker.advance(1.0, [10.0] * 4, [-100.0] * 4, 0.25)
            worker.reposition()
            self.assertEqual(worker.inspect_state(), {"elapsed_s": 0.25})
            self.assertEqual(worker.torque_limits(1.0, [10.0] * 4), [80.0] * 4)
        self.assertEqual(self.physics.references, [])

    def test_stale_replacement_cannot_overwrite_new_version_and_is_recorded(self):
        before = self.broker.current_source.read_text()
        old_hash = hashlib.sha256(before.encode()).hexdigest()
        first = self.replace(before.replace("835.0", "800.0"), old_hash)
        current_path = self.broker.current_source
        stale = self.replace(before.replace("835.0", "700.0"), old_hash)
        self.assertFalse(stale["ok"])
        self.assertIn("source has changed", stale["error"]["message"])
        self.assertEqual(self.broker.current_source, current_path)
        self.assertEqual(json.loads((self.broker.attempts / "attempt_002/request.json").read_text())["expected_sha256"], old_hash)
        current = self.broker.dispatch("inspect_model", {})
        self.assertEqual(current["source_sha256"], first["source_sha256"])
        retry = self.replace(current["source"].replace("800.0", "790.0"), current["source_sha256"])
        self.assertTrue(retry["ok"], retry)
        self.assertEqual(retry["version"], "v003")
        self.assertEqual(retry["budget"]["model_edits"]["used"], 3)

    def test_patch_and_replacement_share_attempt_budget_and_unique_history(self):
        before = self.broker.current_source.read_text()
        self.assertFalse(self.broker.dispatch("patch_model", {"diff": "bad", "rationale": "Try."})["ok"])
        replacement = self.replace(before.replace("835.0", "800.0"))
        self.assertTrue(replacement["ok"], replacement)
        self.assertEqual(replacement["version"], "v002")
        source = self.broker.current_source.read_text()
        patched = self.broker.dispatch("patch_model", {
            "diff": diff_for(source, source.replace("800.0", "790.0")), "rationale": "Try another capacity."})
        self.assertTrue(patched["ok"], patched)
        self.assertEqual(patched["version"], "v003")
        self.assertFalse(self.replace(source)["ok"])
        self.assertFalse(self.broker.dispatch("patch_model", {"diff": "bad", "rationale": "Try."})["ok"])
        budget = self.broker.budget_status()
        self.assertEqual(budget["model_edits"]["used"], 3)
        self.assertEqual(budget["patch_model"]["used"], 2)
        self.assertEqual(budget["replace_model_source"]["used"], 1)
        self.assertEqual(sorted(path.name for path in self.broker.attempts.iterdir()),
                         ["attempt_001", "attempt_002", "attempt_003"])

    def test_invalid_replacement_syntax_and_interface_keep_current_source(self):
        current_path = self.broker.current_source
        before = current_path.read_text()
        invalid = [before.replace("def init_state():", "def init_state(:"),
                   before.replace("def advance_state(", "def missing_function("),
                   before.replace("return {}", "return {'bad': object()}")]
        for index, source in enumerate(invalid):
            result = self.replace(source)
            self.assertFalse(result["ok"], result)
            self.assertEqual(self.broker.current_source, current_path)
            self.assertNotIn(str(self.directory), json.dumps(result))
            if index == 0:
                self.assertIn("source_line", result["error"])

    def test_replacement_executes_only_in_worker_and_cannot_read_host_files(self):
        before = self.broker.current_source.read_text()
        secret = self.directory / "private_reference.txt"
        secret.write_text("private sentinel value")
        hostile = before + f"\nwith open({str(secret)!r}) as stream:\n    assert stream.read()\n"
        result = self.replace(hostile)
        self.assertFalse(result["ok"])
        self.assertEqual(self.broker.current_source.read_text(), before)
        self.assertNotIn(str(secret), json.dumps(result))
        self.assertNotIn("private sentinel value", json.dumps(result))

    def test_replacement_rejects_invalid_hash_unchanged_and_oversized_source(self):
        before = self.broker.current_source.read_text()
        result = self.replace(before + "\n", "../other.py")
        self.assertFalse(result["ok"])
        self.assertIn("64-character", result["error"]["message"])
        unchanged = self.replace(before)
        self.assertFalse(unchanged["ok"])
        self.assertIn("does not change", unchanged["error"]["message"])
        oversized = self.replace(before + "#" * 65536)
        self.assertFalse(oversized["ok"])
        self.assertIn("size limit", oversized["error"]["message"])
        self.assertEqual(self.broker.current_source.read_text(), before)

    def test_syntax_and_execution_failures_keep_prior_version_and_hide_host_paths(self):
        old_path = self.broker.current_source
        before = old_path.read_text()
        syntax = before.replace("def init_state():", "def init_state(:")
        result = self.broker.dispatch("patch_model", {"diff": diff_for(before, syntax), "rationale": "Attempt one."})
        self.assertFalse(result["ok"])
        self.assertIn("source_line", result["error"])
        sentinel = self.directory / "must_not_exist.txt"
        hostile = before + f"\nopen({str(sentinel)!r}, 'w').write('unsafe')\n"
        result = self.broker.dispatch("patch_model", {"diff": diff_for(before, hostile), "rationale": "Attempt two."})
        self.assertFalse(result["ok"])
        self.assertFalse(sentinel.exists())
        self.assertNotIn(str(self.directory), json.dumps(result))
        self.assertEqual(self.broker.current_source, old_path)

    def test_symlink_source_is_rejected_without_reading_target(self):
        target = self.directory / "outside.py"
        target.write_text("outside data")
        self.broker.current_source.unlink()
        self.broker.current_source.symlink_to(target)
        result = self.broker.dispatch("inspect_model", {})
        self.assertFalse(result["ok"])
        self.assertNotIn("outside data", json.dumps(result))

    def test_three_patch_attempts_include_malformed_diffs(self):
        for _ in range(3):
            self.assertFalse(self.broker.dispatch("patch_model", {"diff": "bad patch", "rationale": "Try."})["ok"])
        before = self.broker.current_source.read_text()
        result = self.broker.dispatch("patch_model", {"diff": diff_for(before, before.replace("835.0", "834.0")), "rationale": "Try again."})
        self.assertFalse(result["ok"])
        self.assertEqual(self.broker.budget_status()["patch_model"]["used"], 3)

    def test_regression_uses_only_initial_cached_references(self):
        self.broker.initial_evidence()
        self.experiment({"speed_mps": 20})
        references = len(self.physics.references)
        models = len(self.physics.models)
        result = self.broker.dispatch("run_regression_suite", {"rationale": "Check earlier cases."})
        self.assertTrue(result["ok"])
        self.assertEqual(len(result["cases"]), 2)
        self.assertEqual(result["new_reference_runs"], 0)
        self.assertEqual(len(self.physics.references), references)
        self.assertEqual(len(self.physics.models), models + 2)

    def test_submission_locks_source_and_returns_no_reserved_outcomes(self):
        result = self.broker.dispatch("submit_prediction", {"rationale": "Submit this version."})
        self.assertTrue(result["ok"])
        self.assertTrue(result["agent_submitted"])
        self.assertFalse(result["reserved_outcomes_revealed"])
        self.assertIsNotNone(self.broker.frozen_source)
        self.assertEqual(self.broker.frozen_source.read_bytes(), self.broker.current_source.read_bytes())
        self.assertFalse(self.experiment()["ok"])
        self.assertFalse(self.broker.dispatch("run_model", {"config": {}, "rationale": "More."})["ok"])
        self.assertFalse(self.broker.dispatch("patch_model", {"diff": "bad", "rationale": "More."})["ok"])
        self.assertFalse(self.replace(self.broker.current_source.read_text() + "\n")["ok"])
        self.assertEqual(self.physics.references, [])
        self.assertEqual(self.broker.freeze()["status"], "agent_submitted")

    def test_host_freeze_is_never_labeled_agent_submission(self):
        result = self.broker.freeze()
        self.assertFalse(result["agent_submitted"])
        self.assertEqual(result["status"], "host_frozen")
        self.assertEqual(self.broker.freeze(), result)

    def test_total_dispatch_cap_and_strict_tool_schema(self):
        for _ in range(30):
            self.broker.dispatch("unknown", {})
        self.assertFalse(self.broker.dispatch("inspect_model", {})["ok"])
        self.assertEqual(self.broker.budget_status()["tool_calls"]["used"], 30)
        self.assertEqual({tool["name"] for tool in TOOL_SCHEMAS}, {
            "inspect_model", "replace_model_source", "patch_model", "run_experiment", "observe_run", "run_model", "run_regression_suite", "submit_prediction"})
        for schema in TOOL_SCHEMAS:
            self.assertTrue(schema["strict"])
            self.assertFalse(schema["parameters"]["additionalProperties"])
            self.assertEqual(set(schema["parameters"]["required"]), set(schema["parameters"]["properties"]))

    def test_unexpected_physics_errors_are_neutral(self):
        def fail():
            raise RuntimeError("/host/private/reference secret mechanism")
        self.physics.on_reference = fail
        result = self.experiment()
        self.assertFalse(result["ok"])
        self.assertEqual(result["error"]["message"], "The requested operation could not complete.")


if __name__ == "__main__":
    unittest.main()
