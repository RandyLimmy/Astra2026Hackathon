"""Scenario replay HTTP reads cannot expose evaluator files or launch models."""

import http.client
import json
import threading

import pytest

from dashboard import scenarios
from dashboard.server import Dashboard, DashboardServer, RequestError


def fixture(root, *, run_id="20260910T180000Z-abc", provenance="original_attempt"):
    public = root / "runs/scenario-replays/quadruped_gait_failure" / run_id / "public"
    public.mkdir(parents=True)
    manifest = {"schema_version": 1, "scenario_id": "quadruped_gait_failure", "run_id": run_id,
                "provenance": provenance, "frames": [{"t_s": 0, "views": {"side": "frames/side_000000.jpg"}}]}
    (public / "manifest.json").write_text(json.dumps(manifest))
    (public / "frames").mkdir()
    (public / "frames/side_000000.jpg").write_bytes(b"recorded image")
    (public / "task.md").write_text("Neutral task")
    return public


def test_listing_reads_only_complete_original_recordings(tmp_path):
    app = Dashboard(tmp_path, launcher=lambda *a, **kw: pytest.fail("Replay launched a process"))
    assert all(entry["status"] == "missing" for entry in scenarios.listing(app)["scenarios"])
    fixture(tmp_path)
    fixture(tmp_path, run_id="20260910T190000Z-new", provenance="developer_control")
    entries = scenarios.listing(app)["scenarios"]
    assert entries[0]["status"] == "ready"
    assert "180000" in entries[0]["manifest_url"]
    assert entries[1]["status"] == "missing"


def test_media_allowlist_rejects_private_files_and_symlinks(tmp_path):
    app = Dashboard(tmp_path)
    public = fixture(tmp_path)
    for parts in (["private", "initial_state.json"], ["..", "manifest.json"], ["frames", "../secret.jpg"],
                  ["frames", "metadata.json"]):
        with pytest.raises(RequestError):
            scenarios.media_path(app, "quadruped_gait_failure", public.parent.name, parts)
    image = public / "frames/side_000000.jpg"
    image.unlink()
    image.symlink_to(public / "task.md")
    with pytest.raises(RequestError):
        scenarios.media_path(app, "quadruped_gait_failure", public.parent.name, ["frames", image.name])


def test_scenario_http_reads_work_without_an_investigation(tmp_path):
    public = fixture(tmp_path)
    app = Dashboard(tmp_path, launcher=lambda *a, **kw: pytest.fail("Unexpected process"))
    server = DashboardServer(("127.0.0.1", 0), dashboard=app)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    def get(path):
        connection = http.client.HTTPConnection("127.0.0.1", server.server_port)
        connection.request("GET", path)
        response = connection.getresponse()
        result = response.status, response.read()
        connection.close()
        return result
    try:
        status, body = get("/api/scenarios")
        assert status == 200
        url = json.loads(body)["scenarios"][0]["manifest_url"]
        assert get(url)[0] == 200
        assert get(url.replace("manifest.json", "frames/side_000000.jpg")) == (200, b"recorded image")
        assert get(url.replace("manifest.json", "private/initial_state.json"))[0] == 404
        assert get("/api/scenarios/not-a-scenario")[0] == 404
        assert public.exists()
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
