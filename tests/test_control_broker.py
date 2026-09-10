"""Controller action/evidence contracts without simulator or model API calls."""

from copy import deepcopy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

from PIL import Image
import pytest

from investigation.platform_story import atomic_json
from investigation.task_broker import TaskBroker, canonical, digest
from investigation.task_recording import record_task


class FakeAdapter:
    duration_s = 10.
    cameras = ("side", "overview")
    source_paths = ()

    def capabilities(self):
        return {"platform": "drone", "scenario": "fixed_delivery",
                "goal": "Deliver the parcel and return to the original landing pad.",
                "criteria": ["Deliver and return; remaining stable is insufficient."],
                "controller_source": "def controller(observation, candidate): return candidate['gain']"}

    def initial_candidate(self):
        return {"gain": .5}

    def validate_candidate(self, candidate):
        if (not isinstance(candidate, dict) or set(candidate) != {"gain"}
                or type(candidate["gain"]) not in (int, float)
                or not 0. <= candidate["gain"] <= 4.):
            raise ValueError("Only a finite bounded controller gain may change.")
        return deepcopy(candidate)


@pytest.fixture
def broker_setup(tmp_path, monkeypatch):
    recordings = []

    def fake_record_task(adapter, candidate, workdir, *, frames=True, kind="observed"):
        candidate = adapter.validate_candidate(candidate)
        identifier = f"record_{len(recordings) + 1:04d}"
        workdir = Path(workdir)
        directory = workdir / identifier
        directory.mkdir(parents=True)
        evidence = {camera: [] for camera in adapter.cameras}
        if frames:
            (directory / "frames").mkdir()
            for camera in adapter.cameras:
                for index, timestamp in enumerate((0., 5., 10.)):
                    relative = Path(identifier) / "frames" / f"{camera}_{index}.jpg"
                    Image.new("RGB", (2, 2), (index * 80, 20, 40)).save(workdir / relative)
                    evidence[camera].append({"t_s": timestamp, "file": relative.as_posix(), "camera": camera})
        goal = candidate["gain"] >= 2.
        record = {"id": identifier, "probe": adapter.capabilities()["scenario"], "kind": kind,
                  "duration_s": adapter.duration_s, "actual_duration_s": adapter.duration_s,
                  "summary": {"safe": True, "outcome": "mission_complete" if goal else "standing_still"},
                  "goal_achieved": goal, "partial_success": False,
                  "source_sha256": digest(candidate), "candidate": deepcopy(candidate),
                  "events": [{"time": 0., "event": "ready"}],
                  "observations": [{"t_s": t, "position": [t, 0., 1.]} for t in range(11)],
                  "frames": evidence[adapter.cameras[0]], "evidence_frames": evidence}
        atomic_json(directory / "record.json", record)
        recordings.append({"candidate": deepcopy(candidate), "kind": kind,
                           "workdir": workdir, "record": record})
        return record

    monkeypatch.setattr("investigation.task_broker.record_task", fake_record_task)
    broker = TaskBroker(tmp_path / "run" / "broker", FakeAdapter())
    broker.initial_evidence()
    return broker, recordings


def replace(broker, candidate, *, expected=None):
    return broker.dispatch("replace_controller", {
        "controller_json": json.dumps(candidate),
        "expected_sha256": expected if expected is not None else digest(broker.candidate),
        "rationale": "Adjust feedback after inspecting recorded motion.",
        "expected_effect": "The complete mission may improve; a measured trial is still required.",
    })


@pytest.mark.parametrize("candidate,expected", [
    ({"gain": 2.}, "stale-hash"),
    ({"gain": 2., "world": "healthy"}, None),
    ({"gain": float("nan")}, None),
])
def test_stale_invalid_and_unchanged_controllers_do_not_count_as_applied_fix(broker_setup, candidate, expected):
    broker, _ = broker_setup
    before_source = broker.current_source
    result = replace(broker, candidate, expected=expected)
    assert result["ok"] is False
    assert broker.candidate == {"gain": .5}
    assert broker.current_source == before_source
    assert broker.story.checkpoints[-1]["stage"] == "rejected"
    actions = broker.action_summary()
    assert actions["controller_edit_attempts"] == 1
    assert actions["controller_edits_applied"] == 0
    assert actions["change_applied"] is False
    assert actions["status"] == "fix_attempted_but_not_applied"


def test_unchanged_controller_is_explicit_and_does_not_count_as_applied_fix(broker_setup):
    broker, _ = broker_setup
    before_hash = digest(broker.candidate)
    result = replace(broker, {"gain": .5})
    assert result["ok"] is True
    assert digest(broker.candidate) == before_hash
    assert broker.story.checkpoints[-1]["stage"] == "unchanged"
    assert broker.story.checkpoints[-1]["changes"] == []
    assert broker.action_summary()["controller_edits_applied"] == 0
    assert broker.action_summary()["change_applied"] is False
    assert broker.action_summary()["status"] == "fix_attempted_but_not_applied"


def test_actual_edit_is_versioned_then_verified_by_fresh_complete_trial(broker_setup):
    broker, recordings = broker_setup
    original_hash = digest(broker.candidate)
    response = replace(broker, {"gain": 2.})
    assert response["ok"] is True
    assert response["source_sha256"] != original_hash
    assert broker.current_source.read_text() == canonical({"gain": 2.})
    checkpoint = broker.story.checkpoints[-1]
    assert checkpoint["stage"] == "applied"
    assert checkpoint["changes"] == [{"parameter": "gain", "before": .5, "after": 2.}]
    assert '"gain": 0.5' in checkpoint["source_diff"]
    assert '"gain": 2.0' in checkpoint["source_diff"]
    assert broker.action_summary()["status"] == "applied_unverified"
    assert broker.story.verification["goal_achieved"] is None

    trial = broker.dispatch("run_trial", {"rationale": "Measure the installed controller.",
                                         "expected_observation": "Observe complete delivery and return."})
    assert trial["ok"] is True and trial["goal_achieved"] is True
    assert broker.story.verification["status"] == "pending"
    submission = broker.dispatch("submit_result", {
        "diagnosis": "The observed feedback setting was insufficient.",
        "evidence": "The complete development trial met the unchanged goal.",
        "remaining_uncertainty": "Only this fixed task has been examined.",
    })
    assert submission["ok"] is True
    assert broker.story.verification["status"] == "pending"
    result = broker.finalize("agent_submission")
    assert [item["kind"] for item in recordings] == ["incident", "diagnostic", "after"]
    assert [item["candidate"] for item in recordings] == [{"gain": .5}, {"gain": 2.}, {"gain": 2.}]
    assert recordings[-1]["workdir"] == broker.workdir / "verification"
    assert result["aggregate"]["goal_achieved"] is True
    assert result["action_summary"]["status"] == "goal_achieved"
    assert result["source_sha256"] == hashlib.sha256(broker.current_source.read_bytes()).hexdigest()
    assert result["cases"][0]["before"]["goal_achieved"] is False
    assert result["cases"][0]["after"]["goal_achieved"] is True
    assert broker.finalize("repeat") is result
    assert len(recordings) == 3
    assert replace(broker, {"gain": 3.})["ok"] is False


def test_explanation_and_safe_flag_are_not_controller_action_or_task_success(broker_setup):
    broker, recordings = broker_setup
    broker.record_explanation("The controller needs a correction; I expect that correction to succeed.")
    actions = broker.action_summary()
    assert actions["fix_attempted"] is False
    assert actions["controller_edits_applied"] == 0
    assert actions["status"] == "diagnosed_but_no_fix_attempted"
    result = broker.finalize("model_stopped_without_action")
    assert recordings[-1]["record"]["summary"]["safe"] is True
    assert result["aggregate"]["goal_achieved"] is False
    assert result["cases"][0]["after_difference"]["observed_within_envelope"] is False
    assert result["action_summary"]["status"] == "diagnosed_but_no_fix_attempted"


def test_view_frames_returns_real_owned_images_for_runner_content_injection(broker_setup):
    broker, recordings = broker_setup
    response = broker.dispatch("view_frames", {"id": "run_0001", "times_s": [.1, 9.9], "camera": "side"})
    assert response["ok"] is True
    assert [item["t_s"] for item in response["images"]] == [0., 10.]
    images = broker.pop_images()
    assert len(images) == 2
    allowed = recordings[0]["workdir"] / recordings[0]["record"]["id"] / "frames"
    for item, metadata in zip(images, response["images"]):
        assert item["path"].resolve().is_relative_to(allowed.resolve())
        assert hashlib.sha256(item["path"].read_bytes()).hexdigest() == metadata["sha256"]
        assert Image.open(item["path"]).format == "JPEG"
        assert "run_0001" in item["label"]
    assert broker.pop_images() == []
    unknown = broker.dispatch("view_frames", {"id": "../some-other-run", "times_s": [0.], "camera": "side"})
    assert unknown["ok"] is False
    assert broker.pop_images() == []


def test_failed_frame_selection_cannot_queue_partial_images_or_escape_record(broker_setup, tmp_path):
    broker, recordings = broker_setup
    outsider = tmp_path / "outside.jpg"
    Image.new("RGB", (1, 1)).save(outsider)
    record = recordings[0]["record"]
    record["evidence_frames"]["side"][1]["file"] = str(outsider)
    response = broker.dispatch("view_frames", {"id": "run_0001", "times_s": [0., 5.], "camera": "side"})
    assert response["ok"] is False
    assert broker.pop_images() == []


def test_frame_cannot_be_relabelled_as_a_different_owned_record(broker_setup):
    broker, recordings = broker_setup
    broker.dispatch("run_trial", {"rationale": "Measure the current controller.", "expected_observation": "Inspect its actual result."})
    first, second = (item["record"] for item in recordings)
    first["evidence_frames"]["side"][0]["file"] = second["evidence_frames"]["side"][0]["file"]
    response = broker.dispatch("view_frames", {"id": "run_0001", "times_s": [0.], "camera": "side"})
    assert response["ok"] is False
    assert broker.pop_images() == []


def test_recording_uses_adapter_contract_and_explicit_task_goal_without_rendering(tmp_path):
    class FakeSimulation:
        def __init__(self):
            self.steps = 0
            self.model = SimpleNamespace(opt=SimpleNamespace(timestep=.01))

        @property
        def finished(self):
            return self.steps == 4

        def step(self):
            self.steps += 1

    class RecordingAdapter(FakeAdapter):
        duration_s = .04

        def create_sim(self, candidate):
            self.validate_candidate(candidate)
            return FakeSimulation()

        def observe(self, sim):
            return {"time": sim.steps * .01, "position": [0., 0., 0.]}

        def outcome(self, sim):
            return {"goal_achieved": False, "partial_success": False,
                    "summary": {"safe": True, "outcome": "standing_still"}}

        def public_events(self, sim):
            return [{"time": 0., "event": "ready"}]

    adapter = RecordingAdapter()
    record = record_task(adapter, adapter.initial_candidate(), tmp_path, frames=False)
    assert record["platform"] == adapter.capabilities()["platform"]
    assert record["probe"] == adapter.capabilities()["scenario"]
    assert record["goal_achieved"] is False
    assert record["summary"]["safe"] is True
    assert record["summary"]["task_complete"] is False
    assert record["actual_duration_s"] == adapter.duration_s
    assert [row["t_s"] for row in record["observations"]] == [0., .02, .04]
    assert record["frames"] == []
    assert record["source_sha256"] == digest(adapter.initial_candidate())
    assert json.loads((tmp_path / record["id"] / "record.json").read_text())["goal_achieved"] is False
