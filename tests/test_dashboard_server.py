"""Saved-artifact and local HTTP contracts; no API calls or real child launches."""

import http.client
import json
from pathlib import Path
from types import SimpleNamespace
import threading

import pytest

from dashboard.server import Dashboard, DashboardServer, RequestError


def save(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value))


@pytest.fixture
def recorded(tmp_path):
    run = tmp_path / "runs" / "recorded-run"
    save(run / "metadata.json", {"model": "gpt-6-astra", "reasoning_effort": "medium", "status": "completed",
                                "start_at": "2026-09-10T17:03:00Z", "api_requests": 12})
    observations = [{"time": 50.0, "phase": "trial", "phase_time": 0.0, "position": [0, 0, 0.5],
                     "velocity": [22, 0, 0], "front_x": 2, "brake": 0}]
    summary = {"stopping_distance": 62.4, "stopped": True, "censored": False}
    record = {"config": {"speed_mps": 22}, "summary": summary, "observations": observations,
              "candidate_state": {"PRIVATE_BUILDER_ONLY": 123}}
    for track in ("original", "candidate", "reference"):
        save(run / "evaluation" / "cases" / "reserved_1" / (track + ".json"), record)
    save(run / "evaluation" / "result.json", {"aggregate": {"predictive_success": False, "candidate_mae_m": 2.93},
        "cases": [{"case_id": "reserved_1", "config": {"speed_mps": 22}, "original": record,
                   "candidate": record, "reference": record, "candidate_error_m": 6.9}]})
    events = [{"type": "assistant_message", "timestamp": "2026-09-10T17:04:00Z", "text": "Compare recovery."},
              {"type": "tool_call", "timestamp": "2026-09-10T17:04:01Z", "name": "run_experiment",
               "arguments": {"hypothesis": "The state recovers.", "expected_observation": "Shorter stop."}},
              {"type": "tool_result", "timestamp": "2026-09-10T17:04:02Z", "name": "run_experiment",
               "result": {"ok": True, "summary": summary}}]
    (run / "events.jsonl").write_text("\n".join(json.dumps(event) for event in events) + "\n")
    (run / "prompts").mkdir()
    (run / "prompts" / "system.md").write_text("Exact system prompt.\n")
    (run / "prompts" / "task.md").write_text("Exact task prompt.\n")
    save(run / "prompts" / "tools.json", [{"name": "inspect_model"}])
    for version, text in (("v000", "def init_state(): return {}\n"), ("v002", "def init_state(): return {'work': 0}\n")):
        path = run / "broker" / "versions" / version / "actuator.py"
        path.parent.mkdir(parents=True)
        path.write_text(text)
    return tmp_path, run


class Launcher:
    def __init__(self):
        self.calls = []
        self.returncode = None

    def __call__(self, args, **kwargs):
        self.calls.append((args, kwargs))
        return SimpleNamespace(poll=lambda: self.returncode)


@pytest.fixture
def http_server(tmp_path):
    launcher = Launcher()
    dashboard = Dashboard(tmp_path, launcher=launcher)
    server = DashboardServer(("127.0.0.1", 0), dashboard=dashboard)
    thread = threading.Thread(target=lambda: server.serve_forever(poll_interval=0.01), daemon=True)
    thread.start()

    def request(method, path, body=None, headers=None):
        connection = http.client.HTTPConnection("127.0.0.1", server.server_port, timeout=2)
        connection.request(method, path, body=body, headers=headers or {})
        response = connection.getresponse()
        data = response.read()
        connection.close()
        return response.status, response.getheader("Content-Type"), data

    yield dashboard, launcher, server, request
    server.shutdown()
    server.server_close()
    thread.join(timeout=2)


def test_saved_run_detail_preserves_public_evidence_and_current_source(recorded):
    root, run = recorded
    dashboard = Dashboard(root)
    listing = dashboard.list_runs()
    assert listing["active_run_id"] is None
    assert listing["runs"][0]["aggregate"]["predictive_success"] is False
    detail = dashboard.detail(run.name)
    assert detail["artifacts"]["system_prompt"] == "Exact system prompt.\n"
    assert detail["artifacts"]["task_prompt"] == "Exact task prompt.\n"
    assert "'work'" in detail["artifacts"]["current_source"]
    assert "+def init_state(): return {'work': 0}" in detail["artifacts"]["source_diff"]
    assert detail["events"][1]["arguments"]["expected_observation"] == "Shorter stop."
    assert "observations" not in detail["evaluation"]["cases"][0]["candidate"]
    assert "PRIVATE_BUILDER_ONLY" not in json.dumps(detail)
    trace = dashboard.trace(run.name, "reserved_1", "reference")
    assert trace["observations"][0]["phase_time"] == 0
    assert "PRIVATE_BUILDER_ONLY" not in json.dumps(trace)
    assert dashboard.detail(run.name)["media"] == []


def test_live_status_counts_and_partial_files_are_pollable(recorded):
    root, run = recorded
    (run / "evaluation" / "result.json").write_text('{"aggregate":')
    (run / "metadata.json").write_text('{"status":')
    with (run / "events.jsonl").open("a") as stream:
        stream.write(json.dumps({"type": "status", "timestamp": "2026-09-10T17:06:00Z",
                                 "message": "API request 4/12: Astra / xhigh."}) + "\n")
        stream.write('{"type": "tool_call"')
    dashboard = Dashboard(root)
    summary = dashboard.summary(run.name)
    assert summary["api_requests"] == 4
    assert summary["tool_calls"] == 1
    assert summary["status"] == "running" and summary["active"]
    assert summary["latest_status"] == "API request 4/12: Astra / xhigh."
    assert dashboard.detail(run.name)["evaluation"] is None
    with (run / "events.jsonl").open("a") as stream:
        stream.write('\n' + json.dumps({"type": "status", "message": "Evaluation: prediction_locked"}) + '\n')
    assert dashboard.summary(run.name)["status"] == "evaluating"


def test_cache_directories_and_symlink_runs_are_excluded(recorded):
    root, run = recorded
    (root / "runs" / "investigation-cache").mkdir()
    (root / "runs" / "linked-run").symlink_to(run, target_is_directory=True)
    assert [item["id"] for item in Dashboard(root).list_runs()["runs"]] == [run.name]


@pytest.mark.parametrize("run_id", ["../recorded-run", "..", "recorded-run/../../hidden", "linked-run"])
def test_run_paths_reject_escape_and_symlinks(recorded, run_id):
    root, run = recorded
    (root / "runs" / "linked-run").symlink_to(run, target_is_directory=True)
    with pytest.raises(RequestError):
        Dashboard(root).detail(run_id)


def test_nested_artifact_symlink_is_not_followed(recorded):
    root, run = recorded
    secret = root / "hidden.txt"
    secret.write_text("PRIVATE_SENTINEL")
    (run / "prompts" / "system.md").unlink()
    (run / "prompts" / "system.md").symlink_to(secret)
    with pytest.raises(RequestError):
        Dashboard(root).detail(run.name)


def test_fixed_profile_launch_is_single_and_does_not_precreate_run_directory(tmp_path):
    launcher = Launcher()
    dashboard = Dashboard(tmp_path, launcher, monitor_jobs=False)
    result = dashboard.start_run({"profile": "sol-high"})
    args, options = launcher.calls[0]
    assert args[:5] == [str(tmp_path / ".venv/bin/python"), "-m", "investigation", "--profile", "sol-high"]
    assert args[-4:] == ["--max-api-requests", "12", "--max-seconds", "1800"]
    assert "shell" not in options and "env" not in options
    assert options["cwd"] == str(tmp_path)
    assert not (tmp_path / "runs" / result["id"]).exists()
    assert dashboard.list_runs()["active_run_id"] == result["id"]
    assert dashboard.detail(result["id"])["metadata"]["reasoning_effort"] == "high"
    with pytest.raises(RequestError) as error:
        dashboard.start_run({"profile": "astra-xhigh"})
    assert error.value.status == 409
    launcher.returncode = 1
    assert dashboard.summary(result["id"])["status"] == "failed"
    assert dashboard.list_runs()["active_run_id"] is None
    dashboard.start_run({"profile": "astra-xhigh"})
    assert len(launcher.calls) == 2


@pytest.mark.parametrize("body", [{"profile": "anything"}, {"profile": []}, {}, {"profile": "astra-xhigh", "output": "/tmp/no"}, []])
def test_launcher_rejects_arbitrary_arguments(tmp_path, body):
    launcher = Launcher()
    with pytest.raises(RequestError) as error:
        Dashboard(tmp_path, launcher).start_run(body)
    assert error.value.status == 400
    assert not launcher.calls


def test_existing_external_run_prevents_new_launch(recorded):
    root, run = recorded
    (run / "evaluation/result.json").unlink()
    save(run / "metadata.json", {"status": "running"})
    launcher = Launcher()
    with pytest.raises(RequestError) as error:
        Dashboard(root, launcher).start_run({"profile": "astra-xhigh"})
    assert error.value.status == 409 and not launcher.calls


def test_http_serves_built_spa_and_rejects_private_paths(http_server):
    dashboard, _, _, request = http_server
    dist = dashboard.root / "frontend/dist"
    dist.mkdir(parents=True)
    (dist / "index.html").write_text("<html>dashboard</html>")
    (dist / "app.js").write_text("window.ready = true;")
    assert request("GET", "/")[0] == 200
    assert b"dashboard" in request("GET", "/view/recorded-run")[2]
    assert request("GET", "/app.js")[0] == 200
    for path in ("/.env", "/%2e%2e/hidden.txt", "/api/runs/../metadata.json", "/api/files/hidden.txt"):
        assert request("GET", path)[0] == 404
    assert request("GET", "/api/runs", headers={"Host": "attacker.example"})[0] == 403


def test_http_post_checks_origin_json_and_body_limit(http_server):
    _, launcher, server, request = http_server
    body = json.dumps({"profile": "astra-xhigh"})
    headers = {"Content-Type": "application/json", "Origin": f"http://127.0.0.1:{server.server_port}"}
    assert request("POST", "/api/runs", body, {**headers, "Origin": "https://elsewhere.example"})[0] == 403
    assert request("POST", "/api/runs", body, {**headers, "Sec-Fetch-Site": "cross-site"})[0] == 403
    assert request("POST", "/api/runs", body, {"Content-Type": "text/plain"})[0] == 415
    assert request("POST", "/api/runs", "x" * 1025, headers)[0] == 413
    assert request("POST", "/api/runs", "{", headers)[0] == 400
    assert request("POST", "/api/runs", body, headers)[0] == 202
    assert request("POST", "/api/runs", body, headers)[0] == 409
    assert len(launcher.calls) == 1


def test_http_trace_and_media_only_serve_named_public_artifacts(http_server):
    dashboard, _, _, request = http_server
    run = dashboard.root / "runs/test-run"
    save(run / "metadata.json", {"status": "completed"})
    save(run / "evaluation/cases/reserved_1/reference.json", {"config": {}, "summary": {}, "observations": []})
    manifest = run / "dashboard_media/reserved_1/reference/manifest.json"
    save(manifest, {"frames": [{"t_s": 0, "url": "frames/frame_00000.jpg"}]})
    frame = manifest.parent / "frames/frame_00000.jpg"
    frame.parent.mkdir()
    frame.write_bytes(b"fixture-jpeg")
    prefix = "/api/runs/test-run"
    assert request("GET", prefix + "/trace?case=reserved_1&track=reference")[0] == 200
    assert request("GET", prefix + "/trace?case=reserved_1&track=developer")[0] == 400
    assert request("GET", prefix + "/trace?case=reserved_1&track=reference&track=candidate")[0] == 400
    assert request("GET", prefix + "/trace?case=reserved_2&track=reference")[0] == 404
    assert request("GET", prefix + "/media/reserved_1/reference/manifest.json")[0] == 200
    status, mime, payload = request("GET", prefix + "/media/reserved_1/reference/frames/frame_00000.jpg")
    assert (status, mime, payload) == (200, "image/jpeg", b"fixture-jpeg")
    assert request("GET", prefix + "/media/reserved_1/reference/source.py")[0] == 404
    assert len(dashboard.detail("test-run")["media"]) == 1
    frame.unlink()
    frame.symlink_to(run / "metadata.json")
    assert request("GET", prefix + "/media/reserved_1/reference/frames/frame_00000.jpg")[0] == 404


def test_server_refuses_non_loopback_binding():
    with pytest.raises(ValueError):
        DashboardServer(("0.0.0.0", 8765))


def test_media_queue_is_separate_from_api_job_and_serialized(tmp_path):
    class Process:
        returncode = None
        def poll(self):
            return self.returncode

    children, commands = [], []
    def launch(args, **kwargs):
        child = Process()
        children.append(child)
        commands.append(args)
        return child

    dashboard = Dashboard(tmp_path, launch, monitor_jobs=False)
    first = dashboard.start_run({"profile": "astra-xhigh"})["id"]
    save(tmp_path / "runs" / first / "metadata.json", {"status": "completed"})
    children[0].returncode = 0
    dashboard.advance_jobs()
    assert commands[1][1:3] == ["-m", "dashboard.media"]
    assert dashboard.summary(first)["media_status"] == "rendering"
    assert dashboard.list_runs()["active_run_id"] is None
    second = dashboard.start_run({"profile": "sol-high"})["id"]
    save(tmp_path / "runs" / second / "metadata.json", {"status": "completed"})
    children[2].returncode = 0
    dashboard.advance_jobs()
    assert len(commands) == 3
    assert dashboard.summary(second)["media_status"] == "queued"
    children[1].returncode = 1
    dashboard.advance_jobs()
    assert dashboard.summary(first)["media_status"] == "failed"
    assert dashboard.summary(first)["status"] == "completed"
    assert dashboard.summary(second)["media_status"] == "rendering"
    children[3].returncode = 0
    dashboard.advance_jobs()
    assert dashboard.summary(second)["media_status"] == "completed"
