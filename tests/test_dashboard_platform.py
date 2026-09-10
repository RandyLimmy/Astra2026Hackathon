"""Fast saved-artifact checks: no simulator, network model calls, or real child launches."""

import http.client
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import threading

import pytest

from dashboard.server import Dashboard, DashboardServer, RequestError
from dashboard.platform_story import CATALOG


def save(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value))


class Launcher:
    returncode = None

    def __init__(self):
        self.calls = []

    def __call__(self, args, **options):
        self.calls.append((args, options))
        return SimpleNamespace(poll=lambda: self.returncode)


def fixture_pair(tmp_path):
    pair_id = "pair-drone-saved"
    save(tmp_path / "runs" / pair_id / "comparison.json", {
        "kind": "platform_parallel_comparison", "task_kind": "controller_repair", "comparison_id": pair_id, "platform": "drone",
        "scenario": "drone_delivery_imbalance", "status": "completed", "start_at": "2026-09-11T01:00:00Z",
        "runs": [{"label": name, "run_id": pair_id + "-" + name, "status": "completed"}
                 for name in ("astra", "sol")]})
    for label in ("astra", "sol"):
        run = tmp_path / "runs" / (pair_id + "-" + label)
        save(run / "metadata.json", {"kind": "platform_investigation", "task_kind": "controller_repair", "platform": "drone", "status": "completed",
                                    "model": "gpt-6-astra" if label == "astra" else "gpt-5.6-sol",
                                    "reasoning_effort": "max", "api_requests": 3, "duration_s": 5,
                                    "usage": {"total_tokens": 120}, "estimated_cost_usd": .02})
        save(run / "story.json", {"goal": {"title": "Remain airborne", "criteria": ["Airborne for full maneuver"]},
            "checkpoints": [{"id": "step-1", "kind": "repair", "stage": "applied", "reason": "Thrust is weak",
                             "changes": [{"parameter": "rotor_FL_efficiency", "before": .25, "after": 1}]}],
            "replays": [{"kind": "after", "record_path": "broker/verification/run_test/record.json"}],
            "verification": {"status": "completed", "goal_achieved": label == "astra",
                             "cases": [{"probe": "maneuver", "goal_achieved": label == "astra"}]},
            "action_summary": {"repairs_applied": 1 if label == "astra" else 0}})
        save(run / "evaluation/result.json", {"kind": "platform_repair_verification",
            "aggregate": {"goal_achieved": label == "astra", "predictive_success": None, "partial_success": label == "sol"},
            "cases": [{"probe": "maneuver", "after": {"summary": {"airborne": True},
                        "observations": [{"position": [1, 2, 3]}]}}]})
        record = run / "broker/verification/run_test"
        save(record / "record.json", {"id": "run_test", "kind": "observed", "probe": "maneuver", "duration_s": 2,
              "summary": {"airborne": True}, "observations": [{"position": [1, 2, 3]}],
              "frames": [{"t_s": 0, "file": "run_test/frames/frame_000000.jpg"}]})
        (record / "frames").mkdir()
        (record / "frames/frame_000000.jpg").write_bytes(b"owned-image")
        for name, source in (("versions/v000", '{"gain": 1}\n'), ("submission", '{"gain": 2}\n')):
            path = run / "broker" / name / "controller.json"
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(source)
    return pair_id


def fixture_reassessment(tmp_path):
    pair_id = fixture_pair(tmp_path)
    run = tmp_path / "runs" / (pair_id + "-sol")
    original = json.loads((run / "evaluation/result.json").read_text())
    original["cases"][0]["after"]["summary"]["outcome"] = "mission_timeout"
    save(run / "evaluation/result.json", original)
    controller_hash = hashlib.sha256((run / "broker/submission/controller.json").read_bytes()).hexdigest()
    updated = {"kind": "control_task_verification", "source_sha256": controller_hash,
        "aggregate": {"goal_achieved": True, "partial_success": False, "predictive_success": None},
        "action_summary": {"controller_edits_applied": 1, "change_applied": True, "status": "goal_achieved"},
        "cases": [{"probe": "maneuver", "goal_achieved": True, "after": {"summary": {"outcome": "mission_complete"}}}]}
    record = run / "broker/verification/run_rechecked"
    save(record / "record.json", {"id": "run_rechecked", "kind": "after", "probe": "maneuver", "duration_s": 30,
          "actual_duration_s": 29.15, "source_sha256": controller_hash,
          "summary": {"outcome": "mission_complete"},
          "frames": [{"t_s": 29.15, "file": "run_rechecked/frames/frame_000000.jpg"}]})
    (record / "frames").mkdir()
    (record / "frames/frame_000000.jpg").write_bytes(b"rechecked-owned-image")
    sidecar = {"kind": "control_task_reassessment", "reason": "Corrected supported departure contact scoring.",
               "evaluated_at": "2026-09-11T02:00:00Z",
               "original_result_sha256": hashlib.sha256((run / "evaluation/result.json").read_bytes()).hexdigest(),
               "controller_sha256": controller_hash, "result": updated,
               "replay": {"kind": "after", "id": "run_rechecked", "probe": "maneuver",
                          "record_path": "broker/verification/run_rechecked/record.json"}}
    save(run / "evaluation/reassessment.json", sidecar)
    return pair_id, run, sidecar


def test_verified_reassessment_projects_new_outcome_without_changing_original_artifacts(tmp_path):
    pair_id, run, _ = fixture_reassessment(tmp_path)
    immutable = {name: (run / name).read_bytes() for name in
                 ("evaluation/result.json", "story.json", "metadata.json", "broker/submission/controller.json")}
    detail = Dashboard(tmp_path).comparison_detail(pair_id)["runs"]["sol"]
    story = detail["story"]
    assert story["verification"]["goal_achieved"] is True
    assert story["verification"]["partial_success"] is False
    assert story["original_verification"]["goal_achieved"] is False
    assert story["reassessment"]["original_outcome"]["partial_success"] is True
    assert story["reassessment"]["original_termination"] == "mission_timeout"
    assert story["action_summary"]["status"] == "goal_achieved"
    assert story["metadata"]["api_requests"] == detail["summary"]["api_requests"] == 3
    assert story["checkpoints"][0]["changes"][0]["before"] == .25
    assert len(story["replays"]) == 1 and story["replays"][0]["id"] == "run_rechecked"
    assert story["replays"][0]["actual_duration_s"] == 29.15
    assert story["replays"][0]["frames"][0]["url"].endswith("/run_rechecked/frames/frame_000000.jpg")
    assert all((run / name).read_bytes() == contents for name, contents in immutable.items())


@pytest.mark.parametrize("invalid", ["original_hash", "controller_hash", "result_hash", "record_hash", "escape", "kind"])
def test_unverified_reassessment_preserves_original_outcome_and_footage(tmp_path, invalid):
    pair_id, run, sidecar = fixture_reassessment(tmp_path)
    if invalid == "original_hash":
        sidecar["original_result_sha256"] = "0" * 64
    elif invalid == "controller_hash":
        sidecar["controller_sha256"] = "0" * 64
    elif invalid == "result_hash":
        sidecar["result"]["source_sha256"] = "0" * 64
    elif invalid == "record_hash":
        record = json.loads((run / "broker/verification/run_rechecked/record.json").read_text())
        record["source_sha256"] = "0" * 64
        save(run / "broker/verification/run_rechecked/record.json", record)
    elif invalid == "escape":
        sidecar["replay"]["record_path"] = "../../other/record.json"
    else:
        sidecar["kind"] = "unrelated_evaluation"
    save(run / "evaluation/reassessment.json", sidecar)
    story = Dashboard(tmp_path).comparison_detail(pair_id)["runs"]["sol"]["story"]
    assert story["verification"]["goal_achieved"] is False
    assert "reassessment" not in story and "original_verification" not in story
    assert story["reassessment_warning"]
    assert story["replays"][0]["id"] == "run_test"


def test_catalog_contains_four_selected_controller_tasks():
    catalog = Dashboard(Path(__file__).resolve().parents[1]).scenario_catalog()["platforms"]
    assert {row["id"]: tuple(s["id"] for s in row["scenarios"]) for row in catalog} == {
        "quadruped": ("quadruped_gait_failure",), "drone": ("drone_delivery_imbalance",),
        "warehouse": ("warehouse_curve_demo",), "car": ("car_auto_brake_failure",),
    }
    assert set(CATALOG) == {"drone", "quadruped", "warehouse", "car"}


def test_pair_launch_is_bounded_and_only_one_supervisor(tmp_path):
    launcher = Launcher()
    app = Dashboard(tmp_path, launcher, monitor_jobs=False)
    result = app.start_comparison({"platform": "drone", "scenario": "drone_delivery_imbalance"})
    args, options = launcher.calls[0]
    assert args[1:7] == ["-m", "investigation.task_pair", "--platform", "drone", "--scenario", "drone_delivery_imbalance"]
    assert args[-4:] == ["--max-api-requests", "16", "--max-seconds", "1800"]
    assert "shell" not in options and not (tmp_path / "runs" / result["id"]).exists()
    assert result["runs"] == {name: result["id"] + "-" + name for name in ("astra", "sol")}
    assert set(app.comparison_detail(result["id"])["runs"]) == {"astra", "sol"}
    with pytest.raises(RequestError) as error:
        app.start_comparison({"platform": "warehouse", "scenario": "warehouse_curve_demo"})
    assert error.value.status == 409 and len(launcher.calls) == 1
    launcher.returncode = 1
    assert app.comparison_detail(result["id"])["status"] == "failed"
    assert app.list_comparisons()["active_comparison_id"] is None


@pytest.mark.parametrize("body", [[], {}, {"platform": "warehouse", "scenario": "warehouse_demo"},
    {"platform": "car", "scenario": "drone_demo"}, {"platform": "drone", "scenario": "drone_demo", "output": "/tmp/x"}])
def test_pair_rejects_arbitrary_or_cross_platform_arguments(tmp_path, body):
    launcher = Launcher()
    with pytest.raises(RequestError) as error:
        Dashboard(tmp_path, launcher).start_comparison(body)
    assert error.value.status == 400 and not launcher.calls


def test_story_uses_recorded_changes_results_sources_and_owned_frames(tmp_path):
    pair_id = fixture_pair(tmp_path)
    app = Dashboard(tmp_path)
    detail = app.comparison_detail(pair_id)
    astra, sol = (detail["runs"][label]["story"] for label in ("astra", "sol"))
    assert astra["verification"]["goal_achieved"] is True
    assert sol["verification"]["goal_achieved"] is False
    assert astra["verification"]["cases"][0]["goal_achieved"] is True
    assert sol["verification"]["cases"][0]["goal_achieved"] is False
    assert astra["verification"]["cases"][0]["after"]["summary"]["airborne"] is True
    assert astra["checkpoints"][0]["changes"][0]["before"] == .25
    assert '-{"gain": 1}' in astra["source"]["diff"] and '+{"gain": 2}' in astra["source"]["diff"]
    assert astra["source"]["filename"] == "controller.json"
    assert astra["verification"]["predictive_success"] is None
    assert sol["verification"]["partial_success"] is True
    replay = astra["replays"][0]
    assert replay["duration_s"] == 2 and replay["kind"] == "after"
    assert replay["frames"][0]["url"].endswith("/broker/verification/run_test/frames/frame_000000.jpg")
    assert astra["metrics"]["estimated_cost_usd"] == .02
    assert "observations" not in json.dumps(detail)
    listing = json.dumps(app.list_comparisons())
    assert "frames" not in listing and "observations" not in listing and "source_diff" not in listing


def test_media_rejects_unrecorded_files_escape_and_symlinks(tmp_path):
    pair_id = fixture_pair(tmp_path)
    app = Dashboard(tmp_path)
    run_id = pair_id + "-astra"
    parts = ["broker", "verification", "run_test", "frames", "frame_000000.jpg"]
    assert app.platform_media_path(run_id, parts).read_bytes() == b"owned-image"
    for invalid in (["physics", "..", "frames", "frame_000000.jpg"],
                    ["broker", "verification", "run_test", "frames", "unrecorded.jpg"],
                    ["broker", "submission", "model.py"]):
        with pytest.raises(RequestError):
            app.platform_media_path(run_id, invalid)
    frame = app.platform_media_path(run_id, parts)
    frame.unlink()
    frame.symlink_to(tmp_path / "runs" / run_id / "metadata.json")
    with pytest.raises(RequestError):
        app.platform_media_path(run_id, parts)


def test_restart_and_partial_story_remain_pollable_without_active_orphan(tmp_path):
    pair_id = fixture_pair(tmp_path)
    manifest_path = tmp_path / "runs" / pair_id / "comparison.json"
    manifest = json.loads(manifest_path.read_text())
    manifest.update(status="running", supervisor_pid=2147483647)
    save(manifest_path, manifest)
    story_path = tmp_path / "runs" / (pair_id + "-sol") / "story.json"
    story_path.write_text('{"verification":')
    app = Dashboard(tmp_path)
    detail = app.comparison_detail(pair_id)
    assert detail["status"] == "interrupted" and not detail["active"]
    assert detail["runs"]["astra"]["summary"]["status"] == "completed"
    assert detail["runs"]["sol"]["story"]["verification"]["goal_achieved"] is False
    save(story_path, {"checkpoints": [{"id": "newly-written"}], "replays": []})
    assert app.comparison_detail(pair_id)["runs"]["sol"]["story"]["checkpoints"][0]["id"] == "newly-written"


def test_new_http_routes_and_origin_guard(tmp_path):
    pair_id = fixture_pair(tmp_path)
    launcher = Launcher()
    app = Dashboard(tmp_path, launcher, monitor_jobs=False)
    server = DashboardServer(("127.0.0.1", 0), dashboard=app)
    thread = threading.Thread(target=lambda: server.serve_forever(poll_interval=.01), daemon=True)
    thread.start()
    def request(method, path, body=None, headers=None):
        conn = http.client.HTTPConnection("127.0.0.1", server.server_port, timeout=2)
        conn.request(method, path, body, headers or {})
        response = conn.getresponse()
        result = response.status, response.read()
        conn.close()
        return result
    try:
        catalog = json.loads(request("GET", "/api/scenarios")[1])
        assert {row["id"] for row in catalog["platforms"]} == {"warehouse", "drone", "quadruped", "car"}
        assert sum(len(row["scenarios"]) for row in catalog["platforms"]) == 4
        assert {row["id"] for row in catalog["scenarios"]} == {
            "quadruped_gait_failure", "drone_delivery_imbalance", "warehouse_curve_demo", "car_auto_brake_failure"}
        for path in ("/api/scenarios", "/api/comparisons", "/api/comparisons/" + pair_id,
                     "/api/runs/" + pair_id + "-astra/story"):
            assert request("GET", path)[0] == 200
        path = "/api/runs/" + pair_id + "-astra/platform-media/broker/verification/run_test/frames/frame_000000.jpg"
        assert request("GET", path) == (200, b"owned-image")
        body = json.dumps({"platform": "drone", "scenario": "drone_delivery_imbalance"})
        assert request("POST", "/api/comparisons", body,
                       {"Content-Type": "application/json", "Origin": "https://untrusted.example"})[0] == 403
        assert not launcher.calls
        assert request("POST", "/api/comparisons", body, {"Content-Type": "application/json"})[0] == 202
        assert len(launcher.calls) == 1
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def test_legacy_comparison_artifacts_are_not_platform_comparisons(tmp_path):
    save(tmp_path / "runs/old-braking/comparison.json", {"aggregate": {"success": True}})
    save(tmp_path / "runs/old-braking/metadata.json", {"status": "completed", "model": "gpt-6-astra"})
    app = Dashboard(tmp_path)
    assert app.list_comparisons()["comparisons"] == []
    with pytest.raises(RequestError):
        app.comparison_detail("old-braking")
    assert app.list_runs()["runs"][0]["id"] == "old-braking"


def test_failed_child_is_visible_while_the_other_child_continues(tmp_path):
    launcher = Launcher()
    app = Dashboard(tmp_path, launcher, monitor_jobs=False)
    result = app.start_comparison({"platform": "drone", "scenario": "drone_delivery_imbalance"})
    save(tmp_path / "runs" / result["id"] / "comparison.json", {
        "kind": "platform_parallel_comparison", "task_kind": "controller_repair", "platform": "drone", "status": "running",
        "runs": [{"label": "astra", "status": "running"}, {"label": "sol", "status": "failed"}]})
    detail = app.comparison_detail(result["id"])
    assert detail["active"] and detail["runs"]["astra"]["summary"]["active"]
    assert not detail["runs"]["sol"]["summary"]["active"]
    assert detail["runs"]["sol"]["summary"]["status"] == "failed"
