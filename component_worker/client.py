"""A persistent actuator process with an enforced macOS Seatbelt boundary.

Only the component and the fixed worker runtime are copied into its workspace.
The child gets no repository paths, engine objects, credentials, or site packages.
This implementation deliberately fails closed outside supported macOS hosts.
"""

from __future__ import annotations

import json
import ctypes
import math
import os
from pathlib import Path
import selectors
import shutil
import subprocess
import sys
import sysconfig
import tempfile
import threading
import time


MAX_MESSAGE_BYTES = 65_536
MAX_SOURCE_BYTES = 65_536
MAX_RESIDENT_BYTES = 256 * 1024 * 1024


class _TaskInfo(ctypes.Structure):
    _fields_ = [(name, ctypes.c_uint64) for name in (
        "virtual_size", "resident_size", "total_user", "total_system", "threads_user", "threads_system"
    )] + [("counts", ctypes.c_int32 * 12)]


class WorkerError(RuntimeError):
    """A neutral error safe to return in an agent-facing tool result."""

    def __init__(self, message: str, *, source_line: int | None = None):
        self.source_line = source_line
        location = f" [actuator.py:{source_line}]" if source_line is not None else ""
        super().__init__(message + location)


class WorkerIsolationError(WorkerError):
    """The required operating-system isolation could not be established."""


class WorkerTimeout(WorkerError):
    """The component exceeded its per-request wall-clock budget."""


def _finite(value: object, label: str, *, lower: float | None = None,
            upper: float = 1e9) -> float:
    if isinstance(value, bool) or not isinstance(value, (float, int)):
        raise WorkerError(f"{label} must be a finite number.")
    try:
        number = float(value)
    except (ValueError, OverflowError):
        raise WorkerError(f"{label} must be a finite number.") from None
    if not math.isfinite(number) or abs(number) > upper or (lower is not None and number < lower):
        raise WorkerError(f"{label} is outside the permitted numeric range.")
    return number


def _check_state(state: object) -> dict:
    """Validate again in the host; the editable code owns the child process."""
    remaining = 2048

    def walk(value: object, depth: int) -> None:
        nonlocal remaining
        remaining -= 1
        if remaining < 0 or depth > 8:
            raise WorkerError("Actuator state exceeds the permitted size.")
        if value is None or isinstance(value, bool):
            return
        if isinstance(value, (int, float)):
            _finite(value, "Actuator state", upper=1e15)
            return
        if isinstance(value, list):
            for item in value:
                walk(item, depth + 1)
            return
        if isinstance(value, dict):
            for key, item in value.items():
                if not isinstance(key, str) or not key.isidentifier() or len(key) > 64:
                    raise WorkerError("Actuator state keys must be short identifiers.")
                walk(item, depth + 1)
            return
        raise WorkerError("Actuator state must contain only numeric JSON values.")

    if not isinstance(state, dict):
        raise WorkerError("Actuator state must be a dictionary.")
    walk(state, 0)
    return state


def _profile(workspace: Path, executable: Path) -> str:
    """Allow Python itself and stdlib; exclude every site-packages directory."""
    stdlib = Path(sysconfig.get_path("stdlib")).resolve()
    base = Path(sys.base_prefix).resolve()
    literals = [executable, base / "Python", stdlib.parent / f"python{sys.version_info.major}{sys.version_info.minor}.zip"]
    rules = [
        "(version 1)",
        "(deny default)",
        '(allow file-read* (literal "/"))',
        "(allow file-read* (subpath \"/System/Library\") (subpath \"/usr/lib\") (subpath \"/Library/Apple/System/Library\"))",
        "(allow file-read* (literal \"/dev/null\") (literal \"/dev/urandom\") (literal \"/dev/random\"))",
        "(allow file-write* (literal \"/dev/null\"))",
        f"(allow file-read* (subpath {json.dumps(str(workspace))}) (subpath {json.dumps(str(stdlib))}))",
        "(allow file-read* " + " ".join(f"(literal {json.dumps(str(path))})" for path in literals) + ")",
        f"(allow process-exec (literal {json.dumps(str(executable))}))",
        "(deny process-fork)",
        "(deny network*)",
        '(deny file-read* (regex #"/site-packages(/|$)"))',
    ]
    return "\n".join(rules)


class ActuatorWorker:
    """Run one stateful actuator behind bounded JSON-lines messages.

    ``sandbox=False`` is intentionally rejected: a subprocess is not isolation.
    The current backend requires macOS and ``/usr/bin/sandbox-exec``. Each call
    has a wall-clock deadline; the child also has CPU, file-size and descriptor
    limits. A host watchdog checks RSS every 10 ms and kills above 256 MiB.
    macOS does not provide a hard address-space rlimit; transient overshoot is
    possible between samples. Any protocol failure terminates the child.
    """

    def __init__(self, source_path: Path, timeout_s: float = 2.0, sandbox: bool = True):
        self.timeout_s = _finite(timeout_s, "Request timeout", lower=0.01, upper=60)
        self._temporary = None
        self._process = None
        self._selector = None
        self._buffer = bytearray()
        self._sequence = 0
        self._resource_failure = None
        self._watchdog_stop = threading.Event()
        self._watchdog = None
        if not sandbox or sys.platform != "darwin" or not Path("/usr/bin/sandbox-exec").is_file():
            raise WorkerIsolationError("Required operating-system isolation is unavailable.")
        try:
            with Path(source_path).open("rb") as handle:
                source = handle.read(MAX_SOURCE_BYTES + 1)
        except OSError:
            raise WorkerError("Actuator source could not be read.") from None
        if len(source) > MAX_SOURCE_BYTES:
            raise WorkerError("Actuator source exceeds the permitted size.")
        self._source_line_count = max(1, source.count(b"\n") + 1)
        try:
            self._temporary = tempfile.TemporaryDirectory(prefix="actuator-")
            workspace = Path(self._temporary.name).resolve()
            (workspace / "actuator.py").write_bytes(source)
            shutil.copyfile(Path(__file__).with_name("runtime.py"), workspace / "runtime.py")
            executable = Path(getattr(sys, "_base_executable", sys.executable)).resolve()
            # Framework builds ship a launcher which posix_spawns this binary.
            # Execute it directly so process creation remains denied throughout.
            framework_binary = Path(sys.base_prefix) / "Resources/Python.app/Contents/MacOS/Python"
            if framework_binary.is_file():
                executable = framework_binary.resolve()
            profile = _profile(workspace, executable)
            command = ["/usr/bin/sandbox-exec", "-p", profile, str(executable), "-I", "-S", "-B", "runtime.py"]
            self._process = subprocess.Popen(
                command, cwd=workspace, env={"PATH": "/usr/bin:/bin", "LANG": "C", "LC_ALL": "C"},
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                bufsize=0, close_fds=True, start_new_session=True,
            )
            self._selector = selectors.DefaultSelector()
            self._selector.register(self._process.stdout, selectors.EVENT_READ)
            libproc = ctypes.CDLL("/usr/lib/libproc.dylib", use_errno=True)
            self._pidinfo = libproc.proc_pidinfo
            self._pidinfo.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_uint64, ctypes.c_void_p, ctypes.c_int]
            self._pidinfo.restype = ctypes.c_int
            probe = _TaskInfo()
            if self._pidinfo(os.getpid(), 4, 0, ctypes.byref(probe), ctypes.sizeof(probe)) != ctypes.sizeof(probe):
                raise WorkerIsolationError("Required worker resource monitoring is unavailable.")
            self._watchdog = threading.Thread(target=self._watch_resources, daemon=True, name="actuator-resource-watchdog")
            self._watchdog.start()
            self.reset()
        except WorkerError:
            self.close()
            raise
        except Exception:
            self.close()
            raise WorkerIsolationError("The isolated actuator worker could not start.") from None

    def __enter__(self) -> "ActuatorWorker":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def _watch_resources(self) -> None:
        process = self._process
        while not self._watchdog_stop.wait(0.01):
            if process.poll() is not None:
                return
            information = _TaskInfo()
            size = self._pidinfo(process.pid, 4, 0, ctypes.byref(information), ctypes.sizeof(information))
            if size != ctypes.sizeof(information):
                if process.poll() is not None:
                    return
                self._resource_failure = WorkerIsolationError("Worker resource monitoring became unavailable.")
            elif information.resident_size > MAX_RESIDENT_BYTES:
                self._resource_failure = WorkerError("Actuator exceeded its resident-memory budget.")
            if self._resource_failure is not None:
                try:
                    process.kill()
                except ProcessLookupError:
                    pass
                return

    def _read_message(self) -> dict:
        deadline = time.monotonic() + self.timeout_s
        while b"\n" not in self._buffer:
            if self._resource_failure is not None:
                raise self._resource_failure
            if len(self._buffer) > MAX_MESSAGE_BYTES:
                raise WorkerError("Actuator response exceeds the permitted size.")
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not self._selector.select(remaining):
                raise WorkerTimeout("Actuator exceeded its request time budget.")
            chunk = os.read(self._process.stdout.fileno(), min(4096, MAX_MESSAGE_BYTES + 1 - len(self._buffer)))
            if not chunk:
                if self._resource_failure is not None:
                    raise self._resource_failure
                raise WorkerError("Isolated actuator worker exited without a valid response.")
            self._buffer.extend(chunk)
        line, _, rest = self._buffer.partition(b"\n")
        self._buffer = bytearray(rest)
        if len(line) > MAX_MESSAGE_BYTES:
            raise WorkerError("Actuator response exceeds the permitted size.")
        try:
            message = json.loads(line)
        except (ValueError, UnicodeError, RecursionError):
            raise WorkerError("Actuator returned an invalid response.") from None
        if not isinstance(message, dict):
            raise WorkerError("Actuator returned an invalid response.")
        return message

    def _request(self, operation: str, **values: object) -> object:
        if self._process is None:
            raise WorkerError("Actuator worker is closed.")
        self._sequence += 1
        request = {"id": self._sequence, "operation": operation, **values}
        try:
            payload = json.dumps(request, allow_nan=False, separators=(",", ":")).encode() + b"\n"
            if len(payload) > MAX_MESSAGE_BYTES:
                raise WorkerError("Actuator request exceeds the permitted size.")
            self._process.stdin.write(payload)
            result = self._read_message()
            if result.get("id") != self._sequence or type(result.get("ok")) is not bool:
                raise WorkerError("Actuator returned an invalid response.")
            if not result["ok"]:
                # Never forward arbitrary exception text or tracebacks from code.
                reason = result.get("reason")
                safe_reasons = {
                    "invalid_state": "Actuator returned invalid numeric state.",
                    "invalid_force": "Actuator returned an invalid braking force.",
                    "state_mutation": "compute_force must not modify actuator state; use advance_state.",
                    "candidate_error": "Actuator computation failed.",
                    "runtime_error": "Actuator runtime rejected the request.",
                }
                source_line = result.get("source_line")
                if type(source_line) is not int or not 1 <= source_line <= self._source_line_count:
                    source_line = None
                raise WorkerError(safe_reasons.get(reason, "Actuator computation failed."), source_line=source_line)
            value = result.get("value")
            if operation == "force":
                return _finite(value, "Braking force", lower=0)
            if operation in {"reset", "inspect", "advance"}:
                return _check_state(value)
            raise WorkerError("Unknown actuator operation.")
        except WorkerError:
            self.close()
            raise
        except Exception:
            self.close()
            raise WorkerError("Isolated actuator communication failed.") from None

    def force(self, brake: float, velocity: float) -> float:
        return self._request("force", brake=_finite(brake, "Brake command", lower=0, upper=1),
                             velocity=_finite(velocity, "Velocity", upper=1e6))

    def advance(self, brake: float, velocity: float, applied_force: float, dt: float) -> None:
        self._request("advance", brake=_finite(brake, "Brake command", lower=0, upper=1),
                      velocity=_finite(velocity, "Velocity", upper=1e6),
                      applied_force=_finite(applied_force, "Applied braking force", lower=0),
                      dt=_finite(dt, "Timestep", lower=1e-9, upper=60))

    def reset(self) -> None:
        self._request("reset")

    def inspect_state(self) -> dict:
        return self._request("inspect")

    def close(self) -> None:
        self._watchdog_stop.set()
        if self._watchdog is not None and self._watchdog is not threading.current_thread():
            self._watchdog.join(timeout=1)
            self._watchdog = None
        process, self._process = self._process, None
        if process is not None:
            if process.poll() is None:
                process.kill()
            try:
                process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                pass
            for pipe in (process.stdin, process.stdout):
                if pipe is not None:
                    pipe.close()
        if self._selector is not None:
            self._selector.close()
            self._selector = None
        if self._temporary is not None:
            self._temporary.cleanup()
            self._temporary = None
