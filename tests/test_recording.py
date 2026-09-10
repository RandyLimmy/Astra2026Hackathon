"""Recording tests avoid creating a graphics context."""

import json
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch
import zlib

import mujoco
import numpy as np

from simulator.recording import Recorder, _write_png


class RecordingTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name)
        self.model = mujoco.MjModel.from_xml_string(
            '<mujoco><worldbody><camera name="overview" pos="0 0 3"/>'
            '<body><freejoint/><geom size="0.1"/></body></worldbody></mujoco>'
        )
        self.data = mujoco.MjData(self.model)

    def rows(self, relative):
        return [json.loads(line) for line in (self.path / relative).read_text().splitlines()]

    def test_cadence_and_private_separation(self):
        with Recorder(self.path, self.model) as recorder:
            for tick in range(501):
                recorder.record(self.data, {"time": tick * 0.002, "speed": np.float64(3),
                                            "phase": "trial"},
                                {"temperature": 350, "scenario": "secret"})
            recorder.finish({"temperature": 350}, {"stopped": True})
        observations = self.rows("public/observations.jsonl")
        diagnostics = self.rows("private/diagnostics.jsonl")
        self.assertEqual(len(observations), 101)
        self.assertEqual([row["time"] for row in observations],
                         [row["time"] for row in diagnostics])
        self.assertEqual(diagnostics[0]["phase"], "trial")
        self.assertEqual(diagnostics[0]["temperature"], 350)
        for path in (self.path / "public").rglob("*"):
            if path.is_file():
                self.assertNotIn("temperature", path.read_text())
                self.assertNotIn("secret", path.read_text())
        self.assertEqual(json.loads((self.path / "public/summary.json").read_text()),
                         {"stopped": True})

    def test_reset_clock_duplicate_ticks_and_gaps(self):
        with Recorder(self.path, self.model) as recorder:
            for time in (7.0, 7.0, 7.001, 7.01, 8.0):
                self.data.time = 0
                recorder.record(self.data, {"time": time}, {})
            with self.assertRaisesRegex(ValueError, "backward"):
                recorder.record(self.data, {"time": 0}, {})
        self.assertEqual([row["time"] for row in self.rows("public/observations.jsonl")],
                         [7.0, 7.01, 8.0])

    def test_exception_flushes_partial_records(self):
        with self.assertRaisesRegex(RuntimeError, "interrupted"):
            with Recorder(self.path, self.model) as recorder:
                recorder.record(self.data, {"time": 0}, {})
                raise RuntimeError("interrupted")
        self.assertEqual(self.rows("public/observations.jsonl"), [{"time": 0}])
        recorder.close()
        with self.assertRaisesRegex(RuntimeError, "closed"):
            recorder.record(self.data, {"time": 1}, {})

    def test_existing_recording_is_not_overwritten(self):
        with Recorder(self.path, self.model) as recorder:
            recorder.record(self.data, {"time": 0}, {})
        with self.assertRaises(FileExistsError):
            Recorder(self.path, self.model)
        self.assertEqual(self.rows("public/observations.jsonl"), [{"time": 0}])

    def test_option_and_time_validation(self):
        for options in ({"fps": 0}, {"width": -1}, {"height": 1.2}, {"fps": True},
                        {"frames": True, "camera": "missing"}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                Recorder(self.path, self.model, **options)
        with Recorder(self.path, self.model) as recorder:
            for time in (float("nan"), float("inf"), -1):
                with self.assertRaises(ValueError):
                    recorder.record(self.data, {"time": time}, {})

    def test_frame_sampling_and_physics_state_unchanged(self):
        initial = self.data.qpos.copy(), self.data.qvel.copy(), self.data.time
        with patch("simulator.recording.mujoco.Renderer") as renderer_class:
            renderer = renderer_class.return_value
            renderer.render.return_value = np.zeros((2, 3, 3), dtype=np.uint8)
            with Recorder(self.path, self.model, frames=True, fps=30, width=3, height=2) as recorder:
                for tick in range(501):
                    recorder.record(self.data, {"time": tick * 0.002}, {})
            self.assertEqual(renderer.render.call_count, 31)
            renderer.close.assert_called_once()
            renderer.update_scene.assert_called_with(self.data, camera="overview")
        frames = self.rows("public/frames.jsonl")
        self.assertEqual(len(frames), 31)
        for index, frame in enumerate(frames):
            self.assertLessEqual(abs(frame["time"] - index / 30), 0.002 + 1e-12)
            self.assertTrue((self.path / "public" / frame["file"]).is_file())
        np.testing.assert_array_equal(self.data.qpos, initial[0])
        np.testing.assert_array_equal(self.data.qvel, initial[1])
        self.assertEqual(self.data.time, initial[2])

    def test_png_structure_crc_and_pixel_round_trip(self):
        pixels = np.array([[[255, 0, 1], [2, 3, 4]], [[5, 6, 7], [8, 9, 10]]], dtype=np.uint8)
        path = self.path / "image.png"
        _write_png(path, pixels)
        payload = path.read_bytes()
        self.assertEqual(payload[:8], b"\x89PNG\r\n\x1a\n")
        offset, chunks = 8, {}
        while offset < len(payload):
            length = struct.unpack(">I", payload[offset:offset + 4])[0]
            kind = payload[offset + 4:offset + 8]
            body = payload[offset + 8:offset + 8 + length]
            crc = struct.unpack(">I", payload[offset + 8 + length:offset + 12 + length])[0]
            self.assertEqual(crc, zlib.crc32(kind + body) & 0xFFFFFFFF)
            chunks[kind] = body
            offset += length + 12
        self.assertEqual(struct.unpack(">IIBBBBB", chunks[b"IHDR"]), (2, 2, 8, 2, 0, 0, 0))
        self.assertEqual(zlib.decompress(chunks[b"IDAT"]),
                         b"".join(b"\x00" + row.tobytes() for row in pixels))
        self.assertEqual(chunks[b"IEND"], b"")


if __name__ == "__main__":
    unittest.main()
