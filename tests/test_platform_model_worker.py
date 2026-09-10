"""Stateful platform predictions and their actual OS execution boundary."""

import hashlib
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

from component_worker import PlatformModelWorker, WorkerError, WorkerIsolationError, WorkerTimeout


REPOSITORY = Path(__file__).resolve().parents[1]
BASELINE = (REPOSITORY / "candidate" / "platform_model.py").read_text()
STATEFUL = """
def init_state():
    return {"distance": 0.0, "samples": 0, "trial": False}
def predict_parameters(state):
    return {"gain": 1.0 / (1.0 + state["distance"])}
def advance_state(state, observation, dt_s):
    return {"distance": state["distance"] + observation["speed"] * dt_s,
            "samples": state["samples"] + 1,
            "trial": observation.get("phase") == "trial"}
"""


@unittest.skipUnless(sys.platform == "darwin" and Path("/usr/bin/sandbox-exec").is_file(), "macOS Seatbelt backend")
class PlatformModelWorkerTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix="platform-worker-test-", dir=REPOSITORY)
        self.source = Path(self.directory.name) / "candidate.py"

    def tearDown(self):
        self.directory.cleanup()

    def worker(self, source=BASELINE, **options):
        self.source.write_text(source)
        # Allow cold Seatbelt/Python startup on a busy simulation host. The
        # timeout test narrows the per-call budget after initialization.
        options.setdefault("timeout_s", 5.0)
        return PlatformModelWorker(self.source, **options)

    def test_nominal_component_returns_no_parameter_overrides(self):
        with self.worker() as worker:
            self.assertEqual(worker.parameters(), {})
            self.assertEqual(worker.advance({"phase": "trial", "pose": [0, 1, 2]}, 0), {})
            self.assertEqual(worker.inspect_state(), {})

    def test_public_observation_evolves_state_and_reset_creates_fresh_specimen(self):
        with self.worker(STATEFUL) as worker:
            self.assertEqual(worker.parameters(), {"gain": 1})
            self.assertEqual(worker.advance({"phase": "trial", "speed": 10}, 0),
                             {"distance": 0.0, "samples": 1, "trial": True})
            self.assertEqual(worker.advance({"phase": "trial", "speed": 10}, 0.2),
                             {"distance": 2.0, "samples": 2, "trial": True})
            self.assertAlmostEqual(worker.parameters()["gain"], 1 / 3)
            self.assertAlmostEqual(worker.parameters()["gain"], 1 / 3)
            self.assertEqual(worker.inspect_state()["samples"], 2)
            worker.reset()
            self.assertEqual(worker.inspect_state(), {"distance": 0.0, "samples": 0, "trial": False})

    def test_parameter_query_rejects_nested_mutation_and_original_alias_mutation(self):
        source = BASELINE + """
original = None
def init_state():
    global original
    original = {"memory": [0]}
    return original
"""
        for target in ("state", "original"):
            with self.subTest(target=target):
                query = f"\ndef predict_parameters(state):\n    {target}['memory'][0] += 1\n    return {{}}\n"
                with self.worker(source + query) as worker:
                    with self.assertRaisesRegex(WorkerError, "predict_parameters must not modify"):
                        worker.parameters()
                    with self.assertRaisesRegex(WorkerError, "closed"):
                        worker.inspect_state()

    def test_invalid_parameters_fail_closed(self):
        values = ("[]", "{'gain': float('nan')}", "{'gain': float('inf')}", "{'gain': 1e10}",
                  "{'gain': True}", "{'gain': '1'}", "{'gain': [1]}", "{'bad key': 1}",
                  "{'x' * 65: 1}", "{'k' + str(i): i for i in range(65)}")
        for value in values:
            with self.subTest(value=value):
                source = BASELINE + f"\ndef predict_parameters(state):\n    return {value}\n"
                with self.worker(source) as worker:
                    with self.assertRaisesRegex(WorkerError, "invalid parameters"):
                        worker.parameters()
                    self.assertIsNone(worker._process)

    def test_state_validation_at_initialization_and_advance(self):
        for value in ("[]", "{'text': 'unexpected'}", "{'x': float('nan')}", "{'x': [0] * 2048}", "{'bad key': 0}"):
            with self.subTest(value=value):
                with self.assertRaisesRegex(WorkerError, "invalid numeric state"):
                    self.worker(BASELINE + f"\ndef init_state():\n    return {value}\n")
        with self.worker(BASELINE + "\ndef advance_state(state, observation, dt_s):\n    return {'x': float('inf')}\n") as worker:
            with self.assertRaisesRegex(WorkerError, "invalid numeric state"):
                worker.advance({}, 0.01)

    def test_required_functions_are_checked(self):
        for function in ("init_state", "predict_parameters", "advance_state"):
            with self.subTest(function=function):
                with self.assertRaises(WorkerError):
                    self.worker(BASELINE.replace(f"def {function}", f"def missing_{function}"))

    def test_host_rejects_bad_observation_and_time_without_advancing(self):
        values = ([], {"x": float("nan")}, {"x": b"bad"}, {"phase": "x" * 4001},
                  {"bad key": 0}, {"x": [0] * 2048}, {"phase": "😀" * 2000})
        with self.worker(STATEFUL) as worker:
            for value in values:
                with self.subTest(observation=type(value).__name__):
                    with self.assertRaisesRegex(WorkerError, "Public observation"):
                        worker.advance(value, 0.1)
            for dt in (-0.1, 60.1, float("inf"), True, "0.1"):
                with self.subTest(dt=dt):
                    with self.assertRaisesRegex(WorkerError, "Timestep"):
                        worker.advance({"speed": 1}, dt)
            self.assertEqual(worker.inspect_state()["samples"], 0)
            self.assertEqual(worker.parameters(), {"gain": 1})

    def test_runtime_revalidates_observations_and_protocol(self):
        for values in ({"observation": {"phase": "x" * 4001}, "dt": 0.1},
                       {"observation": {}, "dt": -1}):
            with self.subTest(values=list(values)):
                with self.worker() as worker:
                    with self.assertRaisesRegex(WorkerError, "runtime rejected"):
                        worker._request("advance", **values)
        with self.worker() as worker:
            worker._protocol = "wheel_v2"
            with self.assertRaisesRegex(WorkerError, "runtime rejected"):
                worker.parameters()

    def test_host_revalidates_forged_output(self):
        for operation, value in (("parameters", {"gain": [1]}), ("inspect", {"text": "bad"})):
            with self.subTest(operation=operation):
                with self.worker() as worker:
                    response = {"id": 2, "ok": True, "value": value}
                    with patch.object(worker, "_read_message", return_value=response):
                        with self.assertRaises(WorkerError):
                            worker._request(operation)
                    self.assertIsNone(worker._process)

    def test_hash_records_loaded_source_and_only_fixed_runtime_is_copied(self):
        expected = hashlib.sha256(BASELINE.encode()).hexdigest()
        with self.worker() as worker:
            self.source.write_text("an unrelated later edit")
            self.assertEqual(worker.source_sha256, expected)
            workspace = Path(worker._temporary.name)
            self.assertEqual({path.name for path in workspace.iterdir()}, {"actuator.py", "runtime.py"})
            self.assertEqual((workspace / "runtime.py").read_bytes(),
                             (REPOSITORY / "component_worker" / "platform_runtime.py").read_bytes())
            self.assertEqual(worker.parameters(), {})

    def test_filesystem_environment_engine_and_network_restrictions(self):
        private_fixture = self.source.parent / "synthetic-private.txt"
        private_fixture.write_text("synthetic fixture only")
        source = BASELINE + f"""
import errno
import os
import socket
def init_state():
    state = {{"clean_environment": "PLATFORM_SECRET_SENTINEL" not in os.environ}}
    try:
        open({str(private_fixture)!r}).read()
        state["read_denied"] = False
    except OSError as error:
        state["read_denied"] = error.errno in (errno.EACCES, errno.EPERM)
    try:
        open("unauthorized.txt", "w").write("test")
        state["write_denied"] = False
    except OSError as error:
        state["write_denied"] = error.errno in (errno.EACCES, errno.EPERM)
    try:
        import mujoco
        state["engine_absent"] = False
    except ImportError:
        state["engine_absent"] = True
    try:
        connection = socket.socket()
        connection.settimeout(0.1)
        connection.connect(("127.0.0.1", 9))
        state["network_denied"] = False
    except OSError as error:
        state["network_denied"] = error.errno in (errno.EACCES, errno.EPERM)
    return state
"""
        with patch.dict(os.environ, {"PLATFORM_SECRET_SENTINEL": "synthetic fixture only"}):
            with self.worker(source) as worker:
                self.assertEqual(worker.inspect_state(), {"clean_environment": True, "read_denied": True,
                                                         "write_denied": True, "engine_absent": True,
                                                         "network_denied": True})

    def test_print_is_discarded_and_stdlib_math_is_available(self):
        source = BASELINE + "\nimport math\nprint('debug')\ndef predict_parameters(state):\n    print('query')\n    return {'gain': math.sqrt(4)}\n"
        with self.worker(source) as worker:
            self.assertEqual(worker.parameters(), {"gain": 2})

    def test_fixed_runtime_sets_persistent_resource_limits(self):
        source = BASELINE + """
import resource
def init_state():
    return {"cpu_s": resource.getrlimit(resource.RLIMIT_CPU)[1],
            "file_bytes": resource.getrlimit(resource.RLIMIT_FSIZE)[1],
            "descriptors": resource.getrlimit(resource.RLIMIT_NOFILE)[1],
            "core_bytes": resource.getrlimit(resource.RLIMIT_CORE)[1]}
"""
        with self.worker(source) as worker:
            self.assertEqual(worker.inspect_state(), {"cpu_s": 120, "file_bytes": 0,
                                                     "descriptors": 32, "core_bytes": 0})

    def test_nonterminating_query_is_killed(self):
        source = BASELINE + "\ndef predict_parameters(state):\n    while True: pass\n"
        with self.worker(source) as worker:
            worker.timeout_s = 0.3
            started = time.monotonic()
            with self.assertRaises(WorkerTimeout):
                worker.parameters()
            self.assertLess(time.monotonic() - started, 2)
            self.assertIsNone(worker._process)

    def test_error_message_is_neutral_and_reports_only_own_source_line(self):
        source = BASELINE + "\ndef predict_parameters(state):\n    raise ValueError('arbitrary disclosure attempt')\n"
        with self.worker(source) as worker:
            with self.assertRaises(WorkerError) as caught:
                worker.parameters()
        line = len(source.splitlines())
        self.assertEqual(str(caught.exception), f"Platform model computation failed. [actuator.py:{line}]")
        self.assertEqual(caught.exception.source_line, line)

    def test_disabling_os_isolation_is_rejected(self):
        with self.assertRaises(WorkerIsolationError):
            self.worker(sandbox=False)


if __name__ == "__main__":
    unittest.main()
