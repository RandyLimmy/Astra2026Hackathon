"""Fixed task-batch launch and saved preview contracts; no real processes."""
import json
import http.client
import threading

import pytest

from dashboard.server import Dashboard, DashboardServer, RequestError
from test_dashboard_platform import Launcher, fixture_pair, save


def test_batch_starts_one_fixed_supervisor_and_blocks_extra_jobs(tmp_path):
    launcher = Launcher()
    app = Dashboard(tmp_path, launcher, monitor_jobs=False)
    batch = app.start_batch({})
    assert batch["active"] and batch["status"] == "launching"
    assert batch["comparisons"] == {platform: batch["id"] + "-" + platform
                                    for platform in ("quadruped", "drone", "warehouse", "car")}
    args, options = launcher.calls[0]
    assert args[1:3] == ["-m", "investigation.task_batch"]
    assert args[-4:] == ["--max-api-requests", "16", "--max-seconds", "1800"]
    assert "shell" not in options and not app.run_path(batch["id"]).exists()
    for function, body in ((app.start_batch, {}),
                           (app.start_comparison, {"platform": "drone", "scenario": "drone_delivery_imbalance"}),
                           (app.start_run, {"profile": "astra-max"})):
        with pytest.raises(RequestError) as error:
            function(body)
        assert error.value.status == 409
    assert len(launcher.calls) == 1
    assert app.list_batches()["active_batch_id"] == batch["id"]
    launcher.returncode = 1
    assert app.batch_summary(batch["id"])["status"] == "failed"
    assert app.list_batches()["active_batch_id"] is None


@pytest.mark.parametrize("body", [None, [], {"platform": "car"}, {"max_api_requests": 1000}])
def test_batch_rejects_custom_arguments(tmp_path, body):
    launcher = Launcher()
    with pytest.raises(RequestError) as error:
        Dashboard(tmp_path, launcher).start_batch(body)
    assert error.value.status == 400 and not launcher.calls


def test_saved_batch_uses_only_owned_pair_ids_and_recovers_completed_state(tmp_path):
    batch_id = "batch-saved"
    save(tmp_path / "runs" / batch_id / "batch.json", {
        "kind": "control_task_batch", "task_kind": "controller_repair", "status": "completed",
        "comparisons": {"drone": "../../private"}, "start_at": "2026-09-11T00:00:00Z"})
    app = Dashboard(tmp_path)
    batch = app.batch_summary(batch_id)
    assert batch["status"] == "completed" and not batch["active"]
    assert batch["comparisons"]["drone"] == "batch-saved-drone"
    assert app.list_batches()["batches"][0]["id"] == batch_id


def test_active_pair_blocks_new_batch(tmp_path):
    launcher = Launcher()
    app = Dashboard(tmp_path, launcher)
    app.start_comparison({"platform": "warehouse", "scenario": "warehouse_curve_demo"})
    with pytest.raises(RequestError) as error:
        app.start_batch({})
    assert error.value.status == 409 and len(launcher.calls) == 1


def test_old_platform_pairs_are_preserved_but_excluded_from_primary_listing(tmp_path):
    pair_id = fixture_pair(tmp_path)
    manifest = tmp_path / "runs" / pair_id / "comparison.json"
    old = json.loads(manifest.read_text())
    del old["task_kind"]
    save(manifest, old)
    app = Dashboard(tmp_path)
    assert app.list_comparisons()["comparisons"] == []
    assert app.platform_story(pair_id + "-astra")["replays"]


def test_catalog_preview_is_saved_original_footage_and_not_an_investigation(tmp_path):
    run = tmp_path / "runs" / "preview-control-warehouse"
    save(run / "metadata.json", {"kind": "control_task_preview", "task_kind": "controller_repair",
                                "platform": "warehouse", "status": "completed"})
    save(run / "story.json", {"replays": [{"kind": "incident", "record_path": "physics/original/record.json"}],
                             "verification": {"status": "pending", "goal_achieved": None}, "checkpoints": []})
    save(run / "physics/original/record.json", {"id": "original", "probe": "warehouse_curve_demo", "duration_s": 17,
         "summary": {"outcome": "cargo_spilled"}, "frames": [{"t_s": 0, "file": "original/frames/frame_000000.jpg"}]})
    frame = run / "physics/original/frames/frame_000000.jpg"
    frame.parent.mkdir()
    frame.write_bytes(b"original")
    app = Dashboard(tmp_path, launcher=lambda *a, **kw: pytest.fail("Preview launched a process"))
    platforms = app.scenario_catalog()["platforms"]
    preview = next(item["preview"] for item in platforms if item["id"] == "warehouse")
    assert not preview["summary"]["active"]
    assert preview["story"]["verification"]["goal_achieved"] is None
    assert preview["story"]["replays"][0]["kind"] == "incident"
    assert preview["story"]["replays"][0]["frames"][0]["url"].startswith("/api/runs/preview-control-warehouse/")
    assert app.list_runs()["active_run_id"] is None


def test_batch_http_creation_polling_and_origin_guard(tmp_path):
    launcher = Launcher()
    app = Dashboard(tmp_path, launcher, monitor_jobs=False)
    server = DashboardServer(("127.0.0.1", 0), dashboard=app)
    thread = threading.Thread(target=lambda: server.serve_forever(poll_interval=.01), daemon=True)
    thread.start()
    def request(method, path, body=None, headers=None):
        connection = http.client.HTTPConnection("127.0.0.1", server.server_port, timeout=2)
        connection.request(method, path, body, headers or {})
        response = connection.getresponse()
        result = response.status, json.loads(response.read())
        connection.close()
        return result
    try:
        headers = {"Content-Type": "application/json"}
        assert request("POST", "/api/batches", "{}", {**headers, "Origin": "https://other.example"})[0] == 403
        assert not launcher.calls
        status, batch = request("POST", "/api/batches", "{}", headers)
        assert status == 202
        assert request("GET", "/api/batches")[1]["active_batch_id"] == batch["id"]
        assert request("GET", "/api/batches/list")[1]["batches"][0]["id"] == batch["id"]
        assert request("GET", "/api/batches/" + batch["id"])[1]["comparisons"] == batch["comparisons"]
        assert request("POST", "/api/batches", "{}", headers)[0] == 409
        assert len(launcher.calls) == 1
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
