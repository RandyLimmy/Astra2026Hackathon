"""Version-two wheel protocol, state lifetime, and execution boundary checks."""

import hashlib
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

from component_worker import ActuatorWorker, WheelActuatorWorker, WorkerError, WorkerTimeout


REPOSITORY = Path(__file__).resolve().parents[1]
BASELINE = (REPOSITORY / "candidate" / "wheel_actuator.py").read_text()
STATEFUL = """
def init_state():
    return {"work": [0.0, 0.0, 0.0, 0.0], "activation": 0.0}
def compute_brake_torque_limits(state, brake_command, wheel_speed_rad_s):
    return [1000.0 * brake_command / (1 + work) for work in state["work"]]
def advance_state(state, brake_command, mean_wheel_speed_rad_s, applied_brake_torque_nm, dt_s):
    return {"work": [work + max(-torque * omega, 0) * dt_s / 1000.0
                     for work, torque, omega in zip(state["work"], applied_brake_torque_nm, mean_wheel_speed_rad_s)],
            "activation": brake_command}
def on_trial_reset(state):
    return {"work": list(state["work"]), "activation": 0.0}
"""


@unittest.skipUnless(sys.platform == "darwin" and Path("/usr/bin/sandbox-exec").is_file(), "macOS Seatbelt backend")
class WheelWorkerTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix="wheel-worker-test-", dir=REPOSITORY)
        self.source = Path(self.directory.name) / "candidate.py"

    def tearDown(self):
        self.directory.cleanup()

    def worker(self, source=BASELINE, **options):
        self.source.write_text(source)
        return WheelActuatorWorker(self.source, **options)

    def test_baseline_wheel_order_and_stationary_capacity(self):
        with self.worker() as worker:
            self.assertEqual(worker.torque_limits(0.5, [0.0] * 4), [417.5, 417.5, 278.5, 278.5])
            worker.advance(0.5, [0.0] * 4, [0.0] * 4, 0.002)
            worker.reposition()
            self.assertEqual(worker.inspect_state(), {})

    def test_signed_torques_and_history_survive_reposition_only_reset_clears(self):
        with self.worker(STATEFUL) as worker:
            worker.advance(0.5, [10.0, -10.0, 0.0, 20.0], [-1000.0, 1000.0, 0.0, -500.0], 0.1)
            self.assertEqual(worker.inspect_state(), {"work": [1.0, 1.0, 0.0, 1.0], "activation": 0.5})
            worker.reposition()
            self.assertEqual(worker.inspect_state(), {"work": [1.0, 1.0, 0.0, 1.0], "activation": 0.0})
            self.assertEqual(worker.torque_limits(1, [10.0] * 4), [500.0, 500.0, 1000.0, 500.0])
            worker.reset()
            self.assertEqual(worker.inspect_state(), {"work": [0.0] * 4, "activation": 0.0})

    def test_nested_state_mutation_in_torque_query_is_rejected(self):
        source = STATEFUL + "\ndef compute_brake_torque_limits(state, brake_command, wheel_speed_rad_s):\n    state['work'][0] += 1\n    return [1.0] * 4\n"
        with self.worker(source) as worker:
            with self.assertRaisesRegex(WorkerError, "compute_brake_torque_limits must not modify"):
                worker.torque_limits(1, [10.0] * 4)

    def test_invalid_torque_outputs_fail_closed(self):
        expressions = ("1", "[1, 2, 3]", "[1] * 5", "[1, 1, 1, -1]", "[1, 1, 1, True]",
                       "[1, 1, 1, float('nan')]", "[1, 1, 1, float('inf')]", "[1, 1, 1, 1e10]")
        for expression in expressions:
            with self.subTest(expression=expression):
                source = BASELINE + f"\ndef compute_brake_torque_limits(state, brake_command, wheel_speed_rad_s):\n    return {expression}\n"
                with self.worker(source) as worker:
                    with self.assertRaisesRegex(WorkerError, "invalid wheel braking torque limits"):
                        worker.torque_limits(1, [10.0] * 4)

    def test_host_validates_inputs_and_does_not_advance_on_rejection(self):
        vectors = ([1, 2, 3], [1] * 5, [1, 1, 1, True], [1, 1, 1, float("inf")], [1, 1, 1, "2"])
        with self.worker(STATEFUL) as worker:
            for vector in vectors:
                with self.subTest(vector=vector):
                    with self.assertRaises(WorkerError):
                        worker.torque_limits(1, vector)
                    with self.assertRaises(WorkerError):
                        worker.advance(1, [10.0] * 4, vector, 0.01)
            self.assertEqual(worker.inspect_state(), {"work": [0.0] * 4, "activation": 0.0})

    def test_runtime_independently_validates_vectors(self):
        with self.worker() as worker:
            with self.assertRaises(WorkerError):
                worker._request("torque_limits", brake=1.0, omega=[1.0] * 3)
            with self.assertRaisesRegex(WorkerError, "closed"):
                worker.inspect_state()

    def test_host_independently_validates_worker_vectors(self):
        with self.worker() as worker:
            response = {"id": 2, "ok": True, "value": [1.0, 2.0, 3.0, -4.0]}
            with patch.object(worker, "_read_message", return_value=response):
                with self.assertRaises(WorkerError):
                    worker.torque_limits(1, [10.0] * 4)

    def test_required_reset_hook_and_invalid_reset_state(self):
        with self.assertRaises(WorkerError):
            self.worker(BASELINE.replace("def on_trial_reset", "def missing_hook"))
        with self.worker(BASELINE + "\ndef on_trial_reset(state):\n    return []\n") as worker:
            with self.assertRaisesRegex(WorkerError, "invalid numeric state"):
                worker.reposition()

    def test_protocol_cannot_change_after_initialization(self):
        with self.worker() as worker:
            worker._protocol = "scalar_v1"
            with self.assertRaises(WorkerError):
                worker.reset()

    def test_hash_tracks_loaded_bytes_and_legacy_worker_exposes_hash(self):
        expected = hashlib.sha256(BASELINE.encode()).hexdigest()
        with self.worker() as worker:
            self.source.write_text("unrelated later edit")
            self.assertEqual(worker.source_sha256, expected)
            self.assertEqual(worker.torque_limits(1, [10.0] * 4), [835.0, 835.0, 557.0, 557.0])
        scalar = REPOSITORY / "candidate" / "actuator.py"
        with ActuatorWorker(scalar) as worker:
            self.assertEqual(worker.source_sha256, hashlib.sha256(scalar.read_bytes()).hexdigest())

    def test_wheel_worker_preserves_filesystem_engine_and_environment_boundary(self):
        secret = self.source.parent / "synthetic-private.txt"
        secret.write_text("not-a-real-secret")
        source = BASELINE + f"""
import os
import errno
def init_state():
    state = {{"clean_environment": "OPENAI_API_KEY" not in os.environ}}
    try:
        open({str(secret)!r}).read()
        state["read_denied"] = False
    except OSError as error:
        state["read_denied"] = error.errno in (errno.EACCES, errno.EPERM)
    try:
        import mujoco
        state["engine_absent"] = False
    except ImportError:
        state["engine_absent"] = True
    return state
"""
        with patch.dict(os.environ, {"OPENAI_API_KEY": "synthetic-test-only"}):
            with self.worker(source) as worker:
                self.assertEqual(worker.inspect_state(), {"clean_environment": True, "read_denied": True, "engine_absent": True})

    def test_nonterminating_wheel_query_is_killed(self):
        source = BASELINE + "\ndef compute_brake_torque_limits(state, brake_command, wheel_speed_rad_s):\n    while True: pass\n"
        with self.worker(source) as worker:
            # The short deadline targets the query, after normal initialization.
            worker.timeout_s = 0.3
            with self.assertRaises(WorkerTimeout):
                worker.torque_limits(1, [10.0] * 4)


if __name__ == "__main__":
    unittest.main()
