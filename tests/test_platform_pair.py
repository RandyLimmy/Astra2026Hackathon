"""Tiny no-network checks for simultaneous, isolated child-process orchestration."""
import json
from types import SimpleNamespace

import pytest

from investigation import platform_pair


@pytest.fixture(autouse=True)
def no_credentials(monkeypatch):
    monkeypatch.setattr(platform_pair.Settings, "load", lambda **kwargs: SimpleNamespace())


def test_both_children_launch_before_either_is_polled_and_outputs_are_isolated(tmp_path):
    launched = []
    class Process:
        pid = 100
        def poll(self):
            assert len(launched) == 2, "waiting on the first child would serialize this comparison"
            return 0
    def launch(command, **kwargs):
        launched.append((command, kwargs))
        return Process()
    result = platform_pair.run_pair(tmp_path / "pair", platform="drone", scenario="drone_rotor_loss",
                                    no_frames=True, launcher=launch)
    assert result["status"] == "completed"
    assert result["comparison_valid"] is False  # Successful processes without evidence prove nothing.
    assert result["supervisor_pid"]
    assert len(launched) == 2
    for (command, kwargs), label in zip(launched, ("astra", "sol")):
        assert command[command.index("--profile") + 1] == label + "-max"
        assert command[command.index("--output") + 1] == str(tmp_path / f"pair-{label}")
        assert command[command.index("--comparison-id") + 1] == "pair"
        assert "--no-frames" in command
        assert kwargs["start_new_session"] is True
    saved = json.loads((tmp_path / "pair/comparison.json").read_text())
    assert saved == result
    assert not list((tmp_path / "pair").glob("*.tmp"))


def test_launch_failure_preserves_first_child_outcome(tmp_path):
    calls = []
    class Process:
        pid = 100
        def poll(self): return 0
        def wait(self, timeout=None): return 0
    def launch(command, **kwargs):
        calls.append(command)
        if len(calls) == 2:
            raise OSError("launcher unavailable")
        return Process()
    result = platform_pair.run_pair(tmp_path / "pair", platform="car", launcher=launch)
    assert result["status"] == "failed"
    assert result["runs"][0]["exit_code"] == 0
    assert result["runs"][1]["exit_code"] is None
    assert result["comparison_valid"] is False
    assert (tmp_path / "pair/comparison.json").is_file()


def test_existing_child_is_never_overwritten(tmp_path):
    (tmp_path / "pair-sol").mkdir()
    with pytest.raises(ValueError, match="already exists"):
        platform_pair.run_pair(tmp_path / "pair", platform="quadruped", launcher=lambda *a, **k: pytest.fail())
    assert not (tmp_path / "pair").exists()


@pytest.mark.parametrize("name", ["pair.bad", "_pair", "x" * 121])
def test_unsafe_or_oversize_basename_rejected_before_output_creation(tmp_path, name):
    with pytest.raises(ValueError, match="directory name"):
        platform_pair.run_pair(tmp_path / name, platform="drone", launcher=lambda *a, **k: pytest.fail())
    assert not (tmp_path / name).exists()


def test_comparison_verifies_real_prompt_files_and_rejects_tampering(tmp_path):
    from investigation.api import get_profile
    output = tmp_path / "pair"
    output.mkdir()
    entries = []
    for label in ("astra", "sol"):
        run = tmp_path / f"pair-{label}"
        (run / "prompts").mkdir(parents=True)
        (run / "evaluation").mkdir()
        (run / "broker/submission").mkdir(parents=True)
        (run / "prompts/system.md").write_text("Identical system prompt")
        (run / "prompts/task.md").write_text("Identical evidence and task")
        platform_pair.atomic_json(run / "prompts/tools.json", [])
        platform_pair.atomic_json(run / "prompts/initial_evidence.json", {"observed": 1})
        (run / "broker/submission/model.py").write_text("pass\n")
        manifest = {"fixture": "same-initial-conditions"}
        profile = get_profile(label + "-max")
        metadata = {"status": "completed", "model": profile.model, "reasoning_effort": profile.reasoning_effort,
                    "model_effort_confirmed": True, "protocol_manifest": manifest,
                    "protocol_fingerprint": platform_pair._sha(manifest),
                    "system_prompt_sha256": platform_pair._file_sha(run / "prompts/system.md"),
                    "task_prompt_sha256": platform_pair._file_sha(run / "prompts/task.md"),
                    "tool_schema_sha256": platform_pair._sha([]),
                    "initial_evidence_sha256": platform_pair._sha({"observed": 1}),
                    "source_hash": platform_pair._file_sha(run / "broker/submission/model.py"),
                    "frozen_source_file": "broker/submission/model.py"}
        evaluation = {"source_sha256": metadata["source_hash"], "predeclared_criteria": {"goal": "same"},
                      "cases": [{"probe": "hover", "duration_s": 10, "before_difference": {"safe": False}}]}
        platform_pair.atomic_json(run / "metadata.json", metadata)
        platform_pair.atomic_json(run / "evaluation/result.json", evaluation)
        entries.append({"run_id": run.name, "profile": profile.name, "status": "completed", "exit_code": 0})
    comparison = {"runs": entries}
    assert platform_pair.validate_comparison(output, comparison)["comparison_valid"] is True
    (tmp_path / "pair-sol/prompts/task.md").write_text("A different task was substituted")
    result = platform_pair.validate_comparison(output, comparison)
    assert result["comparison_valid"] is False
    assert result["validation"]["task_prompts_verified_and_match"] is False
    assert result["validation"]["system_prompts_verified_and_match"] is True
