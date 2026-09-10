"""No-API checks for real image inputs and parallel control-task orchestration."""

import base64
from copy import deepcopy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

from PIL import Image
import pytest

from investigation import platform_run, task_batch
from investigation.platform_story import atomic_json
from investigation.tasks import TASKS


class FunctionCall(SimpleNamespace):
    def model_dump(self, **kwargs):
        return vars(self).copy()


def reply(name):
    call = FunctionCall(type="function_call", name=name, arguments="{}", call_id=f"call_{name}")
    return SimpleNamespace(id=f"response_{name}", model="gpt-6-astra",
                           reasoning=SimpleNamespace(effort="max"), status="completed",
                           usage=None, output=[call])


@pytest.mark.parametrize("max_api_requests", [1, 2])
def test_owned_rgb_is_counted_only_when_a_later_request_receives_it(tmp_path, max_api_requests):
    image_path = tmp_path / "physics/owned_run/frames/side_000.jpg"
    image_path.parent.mkdir(parents=True)
    Image.new("RGB", (3, 2), (37, 129, 211)).save(image_path)
    image_bytes = image_path.read_bytes()

    class ImageBroker:
        system_prompt = "Use measured task evidence and report the actual outcome."
        contract_text = "view_frames supplies owned RGB images; submit_result freezes the controller."
        task_instruction = "Inspect the failed task image before submitting an outcome."
        task_kind = "controller_repair"
        tool_schemas = [{"type": "function", "name": name, "strict": True,
                         "parameters": {"type": "object", "properties": {}, "required": [],
                                        "additionalProperties": False}}
                        for name in ("view_frames", "submit_result")]

        def __init__(self):
            self.current_source = tmp_path / "broker/controller.json"
            self.current_source.parent.mkdir()
            self.current_source.write_text('{"gain":1}')
            self.submitted = False
            self.pending = []
            self.calls = []

        def initial_evidence(self):
            return {"capabilities": {"duration_s": 10.}, "observed": {"id": "run_0001", "goal_achieved": False}}

        def budget_status(self):
            return {"tool_calls": {"used": len(self.calls), "limit": 40}}

        def dispatch(self, name, arguments):
            self.calls.append(name)
            if name == "view_frames":
                self.pending.append({"path": image_path, "label": "run_0001, side, t=0.000s; unannotated recorded RGB"})
                return {"ok": True, "id": "run_0001", "images": [{"t_s": 0., "sha256": hashlib.sha256(image_bytes).hexdigest()}]}
            self.submitted = True
            return {"ok": True, "submitted": True}

        def pop_images(self):
            images, self.pending = self.pending, []
            return images

        def finalize(self, reason):
            return {"agent_submitted": self.submitted, "aggregate": {"goal_achieved": False}, "cases": []}

    class Client:
        def __init__(self):
            self.requests = []
            self.responses = self
            self.replies = iter((reply("view_frames"), reply("submit_result")))

        def create(self, **kwargs):
            self.requests.append(deepcopy(kwargs))
            return next(self.replies)

    broker, client = ImageBroker(), Client()
    metadata = platform_run.run_platform_session(client, broker, tmp_path, platform="drone", max_api_requests=max_api_requests)
    assert metadata["status"] == "completed"
    assert len(client.requests) == max_api_requests
    assert "input_image" not in json.dumps(client.requests[0]["input"])
    prompt = client.requests[0]["input"][1]["content"]
    assert "Start by inspecting the system and controller." in prompt
    assert "Start by inspecting the system and model." not in prompt
    events = [json.loads(line) for line in (tmp_path / "events.jsonl").read_text().splitlines()]
    visual = [item for item in events if item["type"] == "visual_evidence"]
    if max_api_requests == 1:
        assert broker.calls == ["view_frames"]
        assert metadata["image_inputs"] == 0
        assert visual == []
        assert metadata["stop_reason"] == "api_request_budget"
        assert json.loads((tmp_path / "metadata.json").read_text())["image_inputs"] == 0
        return
    assert metadata["image_inputs"] == 1
    assert broker.calls == ["view_frames", "submit_result"]
    content = [part for item in client.requests[1]["input"] if isinstance(item.get("content"), list)
               for part in item["content"]]
    actual_images = [part for part in content if part["type"] == "input_image"]
    assert len(actual_images) == 1
    data_url = actual_images[0]["image_url"]
    assert data_url.startswith("data:image/jpeg;base64,")
    assert base64.b64decode(data_url.split(",", 1)[1], validate=True) == image_bytes
    assert actual_images[0]["detail"] == "high"
    assert any(part["type"] == "input_text" and "run_0001, side" in part["text"] for part in content)
    assert all(request["model"] == "gpt-6-astra" and request["reasoning"]["effort"] == "max"
               for request in client.requests)
    assert len(visual) == 1 and visual[0]["image_sha256"] == hashlib.sha256(image_bytes).hexdigest()
    text_artifacts = "\n".join(path.read_text() for path in tmp_path.rglob("*")
                               if path.is_file() and path.suffix in {".json", ".jsonl", ".md"})
    assert data_url not in text_artifacts
    assert base64.b64encode(image_bytes).decode("ascii") not in text_artifacts


def test_all_task_pairs_launch_before_polling_with_same_max_profile_budgets(tmp_path, monkeypatch):
    loaded_profiles, launches, events = [], [], []
    monkeypatch.setattr(task_batch.Settings, "load", lambda *, profile: loaded_profiles.append(profile))

    class Process:
        def __init__(self, index):
            self.pid = 3000 + index

        def poll(self):
            assert len(launches) == len(TASKS), "Polling before all launches would serialize scenarios."
            events.append("poll")
            return 0

    def launch(command, **kwargs):
        launches.append((command, kwargs))
        events.append("launch")
        output = Path(command[command.index("--output") + 1])
        atomic_json(output / "comparison.json", {"comparison_valid": True})
        return Process(len(launches))

    output = tmp_path / "all-tasks"
    result = task_batch.run_batch(output, max_api_requests=12, max_seconds=900, launcher=launch)
    assert loaded_profiles == ["astra-max", "sol-max"]
    assert events[:len(TASKS)] == ["launch"] * len(TASKS)
    assert result["status"] == "completed"
    assert result["comparison_valid"] is True
    assert result["scenario_count"] == len(TASKS)
    assert result["model_sessions"] == len(TASKS) * len(task_batch.PAIR_PROFILES)
    assert result["all_scenarios_parallel"] is True
    assert set(result["comparisons"]) == {"quadruped", "drone", "warehouse", "car"}
    for (command, kwargs), (scenario, task) in zip(launches, TASKS.items()):
        assert command[1:3] == ["-m", "investigation.task_pair"]
        assert command[command.index("--platform") + 1] == task["platform"]
        assert command[command.index("--scenario") + 1] == scenario
        assert command[command.index("--max-api-requests") + 1] == "12"
        assert command[command.index("--max-seconds") + 1] == "900"
        assert command[command.index("--output") + 1] == str(tmp_path / f"all-tasks-{task['platform']}")
        assert "--no-frames" not in command
        assert kwargs["cwd"] == task_batch.ROOT
        assert kwargs["start_new_session"] is True
    assert json.loads((output / "batch.json").read_text()) == result
