"""Immutable multi-camera evidence, public boundaries and preparation history."""
from dataclasses import dataclass
import json
from unittest.mock import patch

import mujoco
import numpy as np
import pytest

from simulator.failure_replays import record_failure
from simulator.platforms import catalog


@dataclass
class Config:
    duration: float = .2
    timestep: float = .002
    fault: str = "private_fixture"


class Fixture:
    def __init__(self):
        self.config = Config()
        self.model = mujoco.MjModel.from_xml_string(
            '<mujoco><option timestep=".002"/><worldbody>'
            '<camera name="overview" pos="0 0 3"/><camera name="chase" pos="0 -3 1"/>'
            '<body pos="0 0 1"><freejoint/><geom size=".1"/></body></worldbody></mujoco>'
        )
        self.data = mujoco.MjData(self.model)
        self.elapsed = .5
        self.data.time = .5
        self.preparation_observations = [{"time": .1, "phase": "conditioning_1", "speed": 1.0},
                                         {"time": .3, "phase": "conditioning_1", "speed": 0.0}]
        self.preparation_diagnostics = [{"temperature": 400}, {"temperature": 450}]
        self.preparation_history = [{"kind": "reset", "phase": "trial", "speed_mps": 0}]
        self.public_events = [{"event": "preparation_reset", "time": .1},
                              {"event": "task_start", "time": .5}]
        self.task_metadata = {"title": "Controlled fixture", "objective": "Observe falling body"}

    @property
    def finished(self):
        return self.elapsed >= .7 - 1e-9

    def step(self, control=None):
        mujoco.mj_step(self.model, self.data)
        self.elapsed = self.data.time
        if self.elapsed >= .6 and len(self.public_events) == 2:
            self.public_events.append({"event": "barrier_contact", "time": self.elapsed})

    def observe(self):
        return {"time": self.elapsed, "phase": "approach", "position": self.data.qpos[:3].tolist(),
                "speed": float(np.linalg.norm(self.data.qvel[:3]))}

    def diagnostics(self):
        return {"temperature": 470, "private_fixture": 123}

    def summary(self):
        return {"private_fixture": 123, "public": {"outcome": "contact"}}

    def presentation(self):
        contact = any(event["event"] == "barrier_contact" for event in self.public_events)
        return {"objective": self.task_metadata["objective"],
                "status": "FAILED - BARRIER COLLISION" if contact else "READY - APPROACH",
                "detail": "Stop target missed" if contact else "Stop before the barrier"}


def mocked_rendering():
    return patch("simulator.failure_replays.mujoco.Renderer")


def test_camera_alignment_conditioning_and_private_boundary(tmp_path):
    sim = Fixture()
    captures = []
    with mocked_rendering() as factory:
        renderer = factory.return_value.__enter__.return_value
        renderer.update_scene.side_effect = lambda data, camera: captures.append((float(data.time), camera))
        renderer.render.return_value = np.zeros((2, 3, 3), dtype=np.uint8)
        result = record_failure(sim, "car_auto_brake_failure", tmp_path / "run", fps=20, width=3, height=2)
    assert result["repair_status"] == "not_run"
    assert result["provenance"] == "original_attempt"
    assert result["objective"] == sim.task_metadata["objective"]
    assert result["frames"][0]["presentation"]["status"] == "READY - APPROACH"
    assert result["frames"][-1]["presentation"]["status"] == "FAILED - BARRIER COLLISION"
    assert result["conclusion"] == result["frames"][-1]["presentation"]
    assert len(result["frames"]) == 5
    for index, frame in enumerate(result["frames"]):
        assert captures[2*index:2*index+2] == [(frame["time"], "chase"), (frame["time"], "overview")]
        assert frame["t_s"] == pytest.approx(index / 20)
        for filename in frame["files"].values():
            assert (tmp_path / "run/public" / filename).read_bytes().startswith(b"\x89PNG")
    public = tmp_path / "run/public"
    rows = [json.loads(line) for line in (public / "observations.jsonl").read_text().splitlines()]
    assert rows[0]["phase"] == "conditioning_1"
    assert rows[0]["time"] == .1
    assert all(a["time"] <= b["time"] for a, b in zip(rows, rows[1:]))
    assert result["events"][0]["time"] == 0
    assert result["events"][1]["time"] == pytest.approx(.1)
    assert result["events"][1]["experiment_time"] == pytest.approx(.6)
    assert result["events"][1]["is_failure"]
    assert json.loads((public / "history_events.json").read_text())[0]["time"] == .1
    for path in public.iterdir():
        if path.is_file():
            content = path.read_text()
            assert "private_fixture" not in content
            assert '"temperature"' not in content
    html = (public / "replay.html").read_text()
    assert "__REPLAY_MANIFEST__" not in html
    assert "fetch(" not in html
    assert (tmp_path / "run/private/initial_state.npy").exists()


def test_rendering_and_playback_export_do_not_change_physics(tmp_path):
    recorded, bare = Fixture(), Fixture()
    while not bare.finished:
        bare.step()
    with mocked_rendering() as factory:
        factory.return_value.__enter__.return_value.render.return_value = np.zeros((2, 3, 3), dtype=np.uint8)
        record_failure(recorded, "car_auto_brake_failure", tmp_path / "run", fps=17, width=3, height=2)
    np.testing.assert_array_equal(recorded.data.qpos, bare.data.qpos)
    np.testing.assert_array_equal(recorded.data.qvel, bare.data.qvel)
    assert recorded.public_events == bare.public_events
    with pytest.raises(FileExistsError):
        record_failure(Fixture(), "car_auto_brake_failure", tmp_path / "run")
    with pytest.raises(ValueError, match="unfinished"):
        record_failure(recorded, "car_auto_brake_failure", tmp_path / "other")


def test_incomplete_capture_never_publishes_manifest(tmp_path):
    with mocked_rendering() as factory:
        factory.return_value.__enter__.return_value.render.side_effect = RuntimeError("renderer failed")
        with pytest.raises(RuntimeError, match="renderer failed"):
            record_failure(Fixture(), "car_auto_brake_failure", tmp_path / "run")
    assert not (tmp_path / "run/public/manifest.json").exists()


def test_car_presets_coexist_with_legacy_and_other_platforms():
    assert catalog.module_for("car_steering_drift").__name__.endswith("car_steering")
    assert catalog.module_for("car_auto_brake_failure").__name__.endswith("car_braking")
    assert catalog.module_for("car_demo").__name__.endswith("car_damage")
    assert catalog.module_for("drone_demo").__name__.endswith("drone")
    assert set(catalog.REPLAY_TASKS) >= {"car_steering_drift", "car_auto_brake_failure",
                                        "quadruped_gait_failure", "drone_delivery_imbalance"}
