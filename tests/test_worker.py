"""Exercise both actuator history and the operating-system access boundary."""

import os
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

from component_worker import ActuatorWorker, WorkerError, WorkerIsolationError, WorkerTimeout


REPOSITORY = Path(__file__).resolve().parents[1]
BASELINE = """
def init_state():
    return {"work": 0.0}

def compute_force(state, brake_command, velocity):
    return 1000.0 * brake_command / (1.0 + state["work"])

def advance_state(state, brake_command, velocity, applied_braking_force_n, dt_s):
    return {"work": state["work"] + applied_braking_force_n * abs(velocity) * dt_s / 1000.0}
"""


@unittest.skipUnless(sys.platform == "darwin" and Path("/usr/bin/sandbox-exec").is_file(), "macOS Seatbelt backend")
class WorkerTests(unittest.TestCase):
    def setUp(self):
        # These contain synthetic fixtures only; never open the user's .env.
        self.directory = tempfile.TemporaryDirectory(prefix="worker-test-", dir=REPOSITORY)
        self.source = Path(self.directory.name) / "candidate.py"
        self.source.write_text(BASELINE)

    def tearDown(self):
        self.directory.cleanup()

    def worker(self, source=BASELINE, **options):
        self.source.write_text(source)
        return ActuatorWorker(self.source, **options)

    def test_state_persists_and_reset_is_explicit(self):
        with self.worker() as worker:
            self.assertEqual(worker.force(0.5, 20), 500.0)
            worker.advance(0.5, 20, 500, 0.1)
            self.assertEqual(worker.inspect_state(), {"work": 1.0})
            self.assertEqual(worker.force(0.5, 20), 250.0)
            worker.reset()
            self.assertEqual(worker.inspect_state(), {"work": 0.0})

    def test_stdlib_math_available_and_prints_do_not_corrupt_protocol(self):
        source = BASELINE + "\nimport math\nprint('candidate debug')\ndef compute_force(state, brake_command, velocity):\n    print('more debug')\n    return math.sqrt(100.0)\n"
        with self.worker(source) as worker:
            self.assertEqual(worker.force(1, 20), 10.0)

    def test_force_rejects_nested_state_mutation(self):
        source = BASELINE + """
def init_state():
    return {"history": {"work": [0.0]}}
def compute_force(state, brake_command, velocity):
    state["history"]["work"][0] += 1
    return 1000.0
"""
        with self.worker(source) as worker:
            with self.assertRaisesRegex(WorkerError, "compute_force must not modify actuator state"):
                worker.force(1, 20)

    def test_repository_and_synthetic_secret_and_reference_reads_are_denied(self):
        secret = self.source.parent / "sample.env"
        secret.write_text("OPENAI_API_KEY=synthetic-test-only")
        reference = self.source.parent / "private_reference.py"
        reference.write_text("PRIVATE_COEFFICIENT = 123")
        targets = [REPOSITORY / "idea1PLan.md", secret, reference]
        for target in targets:
            with self.subTest(target=target.name):
                source = BASELINE + f"""
import errno
def init_state():
    try:
        open({str(target)!r}).read()
    except OSError as error:
        return {{"denied": error.errno in (errno.EPERM, errno.EACCES)}}
    return {{"denied": False}}
"""
                with self.worker(source) as worker:
                    self.assertEqual(worker.inspect_state(), {"denied": True})

    def test_child_environment_has_no_host_credentials(self):
        source = BASELINE + "\nimport os\ndef init_state():\n    return {'clean': 'OPENAI_API_KEY' not in os.environ and 'SECRET_SENTINEL' not in os.environ, 'isolated': __import__('sys').flags.isolated}\n"
        with patch.dict(os.environ, {"OPENAI_API_KEY": "synthetic-test-only", "SECRET_SENTINEL": "synthetic"}):
            with self.worker(source) as worker:
                self.assertEqual(worker.inspect_state(), {"clean": True, "isolated": 1})

    def test_engine_is_not_importable(self):
        source = BASELINE + "\ndef init_state():\n    try:\n        import mujoco\n    except ImportError:\n        return {'denied': True}\n    return {'denied': False}\n"
        with self.worker(source) as worker:
            self.assertEqual(worker.inspect_state(), {"denied": True})

    def test_network_is_denied_by_os(self):
        source = BASELINE + """
import errno
import socket
def init_state():
    try:
        connection = socket.socket()
        connection.settimeout(0.1)
        connection.connect(("127.0.0.1", 9))
    except OSError as error:
        return {"denied": error.errno in (errno.EPERM, errno.EACCES)}
    return {"denied": False}
"""
        with self.worker(source) as worker:
            self.assertEqual(worker.inspect_state(), {"denied": True})

    def test_fork_is_denied_by_os(self):
        source = BASELINE + """
import os
import errno
def init_state():
    try:
        pid = os.fork()
    except OSError as error:
        return {"denied": error.errno in (errno.EPERM, errno.EACCES)}
    if pid == 0:
        os._exit(0)
    os.waitpid(pid, 0)
    return {"denied": False}
"""
        with self.worker(source) as worker:
            self.assertEqual(worker.inspect_state(), {"denied": True})

    def test_filesystem_writes_are_denied(self):
        source = BASELINE + """
import errno
def init_state():
    try:
        open("unauthorized.txt", "w").write("test")
    except OSError as error:
        return {"denied": error.errno in (errno.EPERM, errno.EACCES)}
    return {"denied": False}
"""
        with self.worker(source) as worker:
            self.assertEqual(worker.inspect_state(), {"denied": True})

    def test_nonterminating_force_is_killed(self):
        source = BASELINE + "\ndef compute_force(state, brake_command, velocity):\n    while True: pass\n"
        with self.worker(source, timeout_s=0.3) as worker:
            started = time.monotonic()
            with self.assertRaises(WorkerTimeout):
                worker.force(1, 20)
            self.assertLess(time.monotonic() - started, 2)
            with self.assertRaisesRegex(WorkerError, "closed"):
                worker.inspect_state()

    def test_resident_memory_watchdog(self):
        source = BASELINE + "\ndef compute_force(state, brake_command, velocity):\n    chunks = []\n    while True:\n        chunks.append(bytearray(8 * 1024 * 1024))\n"
        with self.worker(source, timeout_s=3) as worker:
            with self.assertRaisesRegex(WorkerError, "resident-memory budget"):
                worker.force(1, 20)

    def test_invalid_force_outputs_fail_closed(self):
        for expression in ("float('nan')", "float('inf')", "-1", "True", "'1000'", "1e30"):
            with self.subTest(expression=expression):
                source = BASELINE + f"\ndef compute_force(state, brake_command, velocity):\n    return {expression}\n"
                with self.worker(source) as worker:
                    with self.assertRaises(WorkerError):
                        worker.force(1, 20)

    def test_state_is_bounded_numeric_json(self):
        for expression in ("{'text': 'not numeric'}", "{'x': [0] * 10000}", "{'bad key': 1}", "[]"):
            with self.subTest(expression=expression):
                source = BASELINE + f"\ndef init_state():\n    return {expression}\n"
                with self.assertRaises(WorkerError):
                    self.worker(source)

    def test_errors_cannot_disclose_paths_or_arbitrary_text(self):
        marker = str(REPOSITORY / "synthetic-secret")
        source = BASELINE + f"\ndef compute_force(state, brake_command, velocity):\n    raise ValueError({marker!r})\n"
        with self.worker(source) as worker:
            with self.assertRaises(WorkerError) as context:
                worker.force(1, 20)
        expected_line = len(source.splitlines())
        self.assertEqual(context.exception.source_line, expected_line)
        self.assertEqual(str(context.exception), f"Actuator computation failed. [actuator.py:{expected_line}]")
        self.assertNotIn(str(REPOSITORY), str(context.exception))

    def test_syntax_error_reports_only_candidate_line(self):
        with self.assertRaises(WorkerError) as context:
            self.worker("def init_state():\n    return (\n")
        self.assertEqual(context.exception.source_line, 2)
        self.assertEqual(str(context.exception), "Actuator computation failed. [actuator.py:2]")

    def test_host_rejects_untrusted_source_locations(self):
        for location in (-1, 1000000, True, str(REPOSITORY)):
            with self.subTest(location=location):
                with self.worker() as worker:
                    response = {"id": 2, "ok": False, "reason": "candidate_error", "source_line": location}
                    with patch.object(worker, "_read_message", return_value=response):
                        with self.assertRaises(WorkerError) as context:
                            worker.force(1, 20)
                    self.assertIsNone(context.exception.source_line)
                    self.assertEqual(str(context.exception), "Actuator computation failed.")

    def test_oversized_protocol_output_is_rejected(self):
        source = BASELINE + "\nimport os\ndef compute_force(state, brake_command, velocity):\n    os.write(3, b'x' * 100000)\n    return 1\n"
        with self.worker(source) as worker:
            with self.assertRaises(WorkerError):
                worker.force(1, 20)

    def test_unsandboxed_execution_is_not_an_option(self):
        with self.assertRaises(WorkerIsolationError):
            self.worker(sandbox=False)


if __name__ == "__main__":
    unittest.main()
