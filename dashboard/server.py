"""Loopback-only, standard-library dashboard and bounded investigation launcher."""

from datetime import datetime, timezone
import difflib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import mimetypes
from pathlib import Path
import re
import subprocess
import threading
import time
from urllib.parse import parse_qs, unquote, urlsplit
from uuid import uuid4


ROOT = Path(__file__).resolve().parents[1]
PROFILES = {"astra-medium": ("gpt-6-astra", "medium"), "sol-high": ("gpt-5.6-sol", "high")}
TRACKS = {"reference", "candidate", "original"}
IDENTIFIER = re.compile(r"[A-Za-z0-9][A-Za-z0-9_-]{0,127}\Z")
TERMINAL = {"completed", "submitted", "error", "api_error", "evaluation_error", "interrupted", "failed"}
MAX_FILE_BYTES = 32 * 1024 * 1024
MAX_BODY_BYTES = 1024


class RequestError(Exception):
    def __init__(self, status, message):
        self.status, self.message = status, message


def timestamp():
    return datetime.now(timezone.utc).isoformat()


def safe_path(base: Path, *parts: str) -> Path:
    """Reject traversal and every symlink component, including the allowlisted root."""
    path = base
    if path.is_symlink():
        raise RequestError(404, "Artifact not found.")
    for part in parts:
        if not part or part in {".", ".."} or "/" in part or "\\" in part or "\x00" in part:
            raise RequestError(404, "Artifact not found.")
        path = path / part
        if path.is_symlink():
            raise RequestError(404, "Artifact not found.")
    if not path.resolve().is_relative_to(base.resolve()):
        raise RequestError(404, "Artifact not found.")
    return path


def read_text(path: Path):
    try:
        if not path.is_file() or path.stat().st_size > MAX_FILE_BYTES:
            return None
        return path.read_text(encoding="utf-8")
    except (OSError, UnicodeError):
        return None


def read_json(path: Path):
    # Writers replace some JSON files in place. A partial write is pending data,
    # never a server error or an invitation to expose another host file.
    for attempt in range(2):
        text = read_text(path)
        if text is None:
            return None
        try:
            return json.loads(text)
        except ValueError:
            if not attempt:
                time.sleep(0.005)
    return None


def read_events(path: Path):
    text = read_text(path) or ""
    result = []
    for line in text.splitlines():
        try:
            event = json.loads(line)
        except ValueError:
            continue
        if isinstance(event, dict) and event.get("type") in {
            "status", "error", "api_response", "assistant_message", "reasoning_summary",
            "tool_call", "tool_result", "evaluation",
        }:
            if event["type"] == "evaluation":
                event = {**event, "result": compact_evaluation(event.get("result"))}
            result.append(event)
    return result


def compact_evaluation(value):
    if not isinstance(value, dict):
        return None
    result = {key: val for key, val in value.items() if key != "cases"}
    result["cases"] = []
    for case in value.get("cases", []):
        if not isinstance(case, dict):
            continue
        item = {key: val for key, val in case.items() if key not in TRACKS}
        for track in TRACKS:
            record = case.get(track)
            item[track] = ({key: record[key] for key in ("config", "summary", "source_sha256") if key in record}
                           if isinstance(record, dict) else None)
        result["cases"].append(item)
    return result


class Dashboard:
    """Read only named public run artifacts; launch one fixed-profile child at a time."""

    def __init__(self, root=ROOT, launcher=subprocess.Popen, *, monitor_jobs=True):
        self.root = Path(root).resolve()
        self.runs_dir = self.root / "runs"
        self.dist_dir = self.root / "frontend" / "dist"
        self.launcher = launcher
        self.jobs = {}
        self.lock = threading.RLock()
        self.monitor_jobs = monitor_jobs
        self.stop_monitor = threading.Event()
        self.monitor = None
        self.renderer = None

    def run_path(self, run_id):
        if not IDENTIFIER.fullmatch(run_id):
            raise RequestError(404, "Run not found.")
        return safe_path(self.runs_dir, run_id)

    def artifact(self, run_id, *parts):
        self.run_path(run_id)
        return safe_path(self.runs_dir, run_id, *parts)

    def _job(self, run_id):
        with self.lock:
            job = self.jobs.get(run_id)
            if job and job["process"].poll() is not None and job["ended_at"] is None:
                job["ended_at"] = timestamp()
                if job["process"].poll() == 0:
                    job["media_status"] = "queued"
            return dict(job) if job else None

    def advance_jobs(self):
        """Observe child completion and advance one independent media-render queue."""
        with self.lock:
            for run_id in self.jobs:
                self._job(run_id)
            if self.renderer:
                run_id, process = self.renderer
                code = process.poll()
                if code is None:
                    return
                self.jobs[run_id]["media_status"] = "completed" if code == 0 else "failed"
                self.renderer = None
            for run_id, job in self.jobs.items():
                if job["media_status"] != "queued":
                    continue
                args = [str(self.root / ".venv" / "bin" / "python"), "-m", "dashboard.media", str(self.run_path(run_id))]
                try:
                    process = self.launcher(args, cwd=str(self.root), stdin=subprocess.DEVNULL,
                                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
                except OSError:
                    job["media_status"] = "failed"
                    continue
                self.renderer = (run_id, process)
                job["media_status"] = "rendering"
                break

    def _monitor_jobs(self):
        while not self.stop_monitor.wait(0.25):
            self.advance_jobs()

    def close(self):
        # Existing paid investigation children continue independently if the UI
        # server closes. No restart or second API call is initiated here.
        self.stop_monitor.set()

    def summary(self, run_id):
        self.run_path(run_id)
        job = self._job(run_id)
        metadata = read_json(self.artifact(run_id, "metadata.json"))
        metadata = metadata if isinstance(metadata, dict) else {}
        events = read_events(self.artifact(run_id, "events.jsonl"))
        if not metadata and not events and not job:
            raise RequestError(404, "Run not found.")
        if job and not metadata:
            model, effort = PROFILES[job["profile"]]
            metadata = {"model": model, "reasoning_effort": effort, "profile": job["profile"],
                        "start_at": job["started_at"], "status": "starting", "max_api_requests": 12,
                        "max_seconds": 1800}
        evaluation = compact_evaluation(read_json(self.artifact(run_id, "evaluation", "result.json")))
        status = metadata.get("status", "running")
        last_status = next((e.get("message") for e in reversed(events) if e["type"] in {"status", "error"}), None)
        if status not in TERMINAL:
            if evaluation:
                status = "completed"
            elif any(e["type"] == "status" and ("Source frozen." in e.get("message", "") or
                                               e.get("message", "").startswith("Evaluation:")) for e in events):
                status = "evaluating"
            if job and job["ended_at"] and status not in TERMINAL:
                status = "failed"
                last_status = "The investigation process exited before recording completion."
        requests = sum(e["type"] == "api_response" for e in events)
        for event in events:
            match = re.search(r"API request (\d+)/", event.get("message", ""))
            if match:
                requests = max(requests, int(match.group(1)))
        requests = max(requests, metadata.get("api_requests", 0) or 0)
        calls = sum(e["type"] == "tool_call" for e in events)
        active = bool(job and not job["ended_at"]) or status in {"starting", "running", "evaluating"}
        return {"id": run_id, "metadata": metadata, "aggregate": evaluation.get("aggregate") if evaluation else None,
                "status": status, "active": active, "latest_status": last_status,
                "media_status": job["media_status"] if job else None,
                "last_event_at": events[-1].get("timestamp") if events else metadata.get("start_at"),
                "api_requests": requests, "tool_calls": calls}

    def list_runs(self):
        if self.runs_dir.is_symlink():
            raise RequestError(404, "Run directory unavailable.")
        with self.lock:
            ids = set(self.jobs)
        if self.runs_dir.is_dir():
            ids.update(path.name for path in self.runs_dir.iterdir()
                       if path.is_dir() and not path.is_symlink() and IDENTIFIER.fullmatch(path.name))
        runs = []
        for run_id in ids:
            try:
                runs.append(self.summary(run_id))
            except RequestError:
                continue
        runs.sort(key=lambda row: (row["metadata"].get("start_at") or row["last_event_at"] or "", row["id"]), reverse=True)
        return {"runs": runs, "active_run_id": next((row["id"] for row in runs if row["active"]), None)}

    def detail(self, run_id):
        result = self.summary(run_id)
        result["evaluation"] = compact_evaluation(read_json(self.artifact(run_id, "evaluation", "result.json")))
        result["events"] = read_events(self.artifact(run_id, "events.jsonl"))
        names = {"system_prompt": ("prompts", "system.md"), "task_prompt": ("prompts", "task.md"),
                 "original_source": ("evaluation", "original_candidate.py"),
                 "frozen_source": ("evaluation", "frozen_candidate.py"), "source_diff": ("evaluation", "source.diff")}
        result["artifacts"] = {key: read_text(self.artifact(run_id, *parts)) for key, parts in names.items()}
        result["artifacts"]["tools"] = read_json(self.artifact(run_id, "prompts", "tools.json"))
        if result["artifacts"]["original_source"] is None:
            result["artifacts"]["original_source"] = read_text(self.artifact(run_id, "broker", "versions", "v000", "actuator.py"))
        if result["artifacts"]["frozen_source"] is None:
            result["artifacts"]["frozen_source"] = read_text(self.artifact(run_id, "broker", "submission", "actuator.py"))
        result["artifacts"]["current_source"] = result["artifacts"]["frozen_source"]
        versions = self.artifact(run_id, "broker", "versions")
        if result["artifacts"]["current_source"] is None and versions.is_dir():
            for version in sorted(versions.iterdir(), reverse=True):
                if re.fullmatch(r"v\d{3}", version.name) and not version.is_symlink():
                    source = read_text(self.artifact(run_id, "broker", "versions", version.name, "actuator.py"))
                    if source is not None:
                        result["artifacts"]["current_source"] = source
                        break
        if result["artifacts"]["source_diff"] is None:
            original, current = (result["artifacts"][name] for name in ("original_source", "current_source"))
            if original is not None and current is not None:
                result["artifacts"]["source_diff"] = "".join(difflib.unified_diff(
                    original.splitlines(keepends=True), current.splitlines(keepends=True),
                    fromfile="original/actuator.py", tofile="current/actuator.py"))
        result["media"] = []
        media_dir = self.artifact(run_id, "dashboard_media")
        if media_dir.is_dir():
            for case in sorted(media_dir.iterdir()):
                if not case.is_dir() or case.is_symlink() or not IDENTIFIER.fullmatch(case.name):
                    continue
                for track in sorted(TRACKS):
                    try:
                        manifest = self.artifact(run_id, "dashboard_media", case.name, track, "manifest.json")
                        available = isinstance(read_json(manifest), dict)
                    except RequestError:
                        available = False
                    if available:
                        result["media"].append({"case_id": case.name, "track": track,
                            "manifest_url": f"/api/runs/{run_id}/media/{case.name}/{track}/manifest.json"})
        return result

    def trace(self, run_id, case, track):
        self.summary(run_id)
        if not IDENTIFIER.fullmatch(case) or track not in TRACKS:
            raise RequestError(400, "Choose a recorded case and valid track.")
        record = read_json(self.artifact(run_id, "evaluation", "cases", case, track + ".json"))
        if not isinstance(record, dict):
            raise RequestError(404, "Trace is not available yet.")
        return {"case_id": case, "track": track, **{key: record.get(key) for key in ("config", "summary", "observations")}}

    def media_path(self, run_id, case, track, parts):
        self.run_path(run_id)
        if not IDENTIFIER.fullmatch(case) or track not in TRACKS:
            raise RequestError(404, "Media not found.")
        if parts == ["manifest.json"]:
            pass
        elif (len(parts) == 2 and parts[0] == "frames" and
              re.fullmatch(r"[A-Za-z0-9_-]+\.(jpg|jpeg|png|webp)", parts[1])):
            pass
        else:
            raise RequestError(404, "Media not found.")
        return self.artifact(run_id, "dashboard_media", case, track, *parts)

    def start_run(self, body):
        if (not isinstance(body, dict) or set(body) != {"profile"} or
                not isinstance(body["profile"], str) or body["profile"] not in PROFILES):
            raise RequestError(400, "Choose profile astra-medium or sol-high.")
        with self.lock:
            if self.list_runs()["active_run_id"]:
                raise RequestError(409, "An investigation is already active.")
            self.runs_dir.mkdir(exist_ok=True)
            profile = body["profile"]
            stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            run_id = f"{profile}-{stamp}-{uuid4().hex[:8]}"
            output = self.run_path(run_id)
            args = [str(self.root / ".venv" / "bin" / "python"), "-m", "investigation", "--profile", profile,
                    "--output", str(output), "--max-api-requests", "12", "--max-seconds", "1800"]
            try:
                process = self.launcher(args, cwd=str(self.root), stdin=subprocess.DEVNULL,
                                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
            except OSError:
                raise RequestError(503, "The investigation process could not start.") from None
            self.jobs[run_id] = {"process": process, "profile": profile, "started_at": timestamp(),
                                 "ended_at": None, "media_status": None}
            if self.monitor_jobs and self.monitor is None:
                self.monitor = threading.Thread(target=self._monitor_jobs, daemon=True)
                self.monitor.start()
            return {"id": run_id, "status": "starting"}


class DashboardServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address=("127.0.0.1", 8765), *, dashboard=None):
        if address[0] != "127.0.0.1":
            raise ValueError("Dashboard must bind to 127.0.0.1")
        self.dashboard = dashboard or Dashboard()
        super().__init__(address, DashboardHandler)

    def server_close(self):
        self.dashboard.close()
        super().server_close()


class DashboardHandler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def _check_origin(self, mutation=False):
        host = self.headers.get("Host", "")
        try:
            parsed = urlsplit("http://" + host)
            valid = (parsed.hostname in {"127.0.0.1", "localhost", "::1"} and not parsed.username and
                     not parsed.password and not parsed.path and not parsed.query and not parsed.fragment)
            parsed.port
        except ValueError:
            valid = False
        if not valid or self.client_address[0] != "127.0.0.1":
            raise RequestError(403, "Use the local dashboard address.")
        if mutation:
            origin = self.headers.get("Origin")
            if origin and origin not in {"http://" + host, "https://" + host}:
                raise RequestError(403, "Run creation requires a same-origin request.")
            if self.headers.get("Sec-Fetch-Site") not in {None, "same-origin", "none"}:
                raise RequestError(403, "Run creation requires a same-origin request.")

    def _json(self, status, value):
        content = json.dumps(value, allow_nan=False, separators=(",", ":")).encode()
        self._send(status, content, "application/json; charset=utf-8")

    def _send(self, status, content, content_type):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "no-referrer")
        self.end_headers()
        self.wfile.write(content)

    def _file(self, path):
        try:
            if not path.is_file() or path.stat().st_size > MAX_FILE_BYTES:
                raise RequestError(404, "Artifact not found.")
            content = path.read_bytes()
        except OSError:
            raise RequestError(404, "Artifact not found.") from None
        self._send(200, content, mimetypes.guess_type(path.name)[0] or "application/octet-stream")

    def do_GET(self):
        try:
            self._check_origin()
            parsed = urlsplit(self.path)
            parts = unquote(parsed.path).strip("/").split("/")
            app = self.server.dashboard
            if parts[:2] == ["api", "scenarios"]:
                from .scenarios import listing, media_path
                if len(parts) == 2:
                    return self._json(200, listing(app))
                if len(parts) >= 6 and parts[3] == "recordings":
                    return self._file(media_path(app, parts[2], parts[4], parts[5:]))
                raise RequestError(404, "Scenario endpoint not found.")
            if parts == ["api", "runs"]:
                return self._json(200, app.list_runs())
            if parts[:2] == ["api", "runs"] and len(parts) >= 3:
                run_id = parts[2]
                if len(parts) == 3:
                    return self._json(200, app.detail(run_id))
                if len(parts) == 4 and parts[3] == "trace":
                    query = parse_qs(parsed.query)
                    if set(query) != {"case", "track"} or any(len(v) != 1 for v in query.values()):
                        raise RequestError(400, "Trace requires one case and one track.")
                    return self._json(200, app.trace(run_id, query["case"][0], query["track"][0]))
                if len(parts) >= 7 and parts[3] == "media":
                    return self._file(app.media_path(run_id, parts[4], parts[5], parts[6:]))
                raise RequestError(404, "Endpoint not found.")
            if parts[0] == "api":
                raise RequestError(404, "Endpoint not found.")
            if parts == [""]:
                parts = ["index.html"]
            if any(part.startswith(".") for part in parts):
                raise RequestError(404, "Artifact not found.")
            path = safe_path(app.root, "frontend", "dist", *parts)
            if not path.is_file() and not path.suffix:
                path = safe_path(app.root, "frontend", "dist", "index.html")
            return self._file(path)
        except RequestError as error:
            self._json(error.status, {"error": error.message})
        except (OSError, ValueError, TypeError):
            self._json(503, {"error": "Run data is being updated. Retry shortly."})

    def do_POST(self):
        try:
            self._check_origin(mutation=True)
            if self.path != "/api/runs":
                raise RequestError(404, "Endpoint not found.")
            if self.headers.get_content_type() != "application/json":
                raise RequestError(415, "Run creation requires JSON.")
            if self.headers.get("Transfer-Encoding"):
                raise RequestError(400, "Use a bounded JSON request.")
            try:
                length = int(self.headers.get("Content-Length", "0"))
            except ValueError:
                raise RequestError(400, "Invalid request length.") from None
            if not 0 < length <= MAX_BODY_BYTES:
                raise RequestError(413, "Run request is too large or empty.")
            try:
                body = json.loads(self.rfile.read(length))
            except (ValueError, UnicodeError):
                raise RequestError(400, "Invalid JSON request.") from None
            self._json(202, self.server.dashboard.start_run(body))
        except RequestError as error:
            self._json(error.status, {"error": error.message})
        except (OSError, ValueError, TypeError):
            self._json(503, {"error": "The investigation could not start."})


def main():
    server = DashboardServer()
    print("Dashboard: http://127.0.0.1:8765", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
