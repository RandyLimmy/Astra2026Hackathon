"""Recording contracts: same-time views, untouched evidence, immutable artifacts."""

from dataclasses import dataclass
import json
import sys
from types import SimpleNamespace

import mujoco
import numpy as np
from PIL import Image
import pytest

from simulator import scenario_replay as replay


@dataclass
class Config:
    duration: float = .4
    timestep: float = .1
    fault: str = "coordination"
    coordination_defect: bool = True


class SmallSimulation:
    def __init__(self):
        self.config = Config()
        self.model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
          <camera name="side" pos="0 -3 2"/><camera name="overview" pos="0 0 5"/>
          <body pos="0 0 1"><freejoint/><geom type="sphere" size=".1"/></body>
          </worldbody></mujoco>''')
        self.model.opt.timestep = self.config.timestep
        self.data = mujoco.MjData(self.model)
        self.elapsed = 0.

    @property
    def finished(self):
        return self.elapsed >= self.config.duration - 1e-9

    def step(self, control=None):
        self.elapsed += self.config.timestep
        self.data.time = self.elapsed
        self.data.qpos[0] = self.elapsed

    def observe(self):
        return {"time": self.elapsed, "phase": "walking", "position": self.data.qpos[:3].tolist(),
                "velocity": [1., 0., 0.], "command": {"forward_speed": 1.}}

    def diagnostics(self):
        return {"PRIVATE_PARAMETER": 123, "events": []}

    def summary(self):
        return {"public": {"outcome": "fell", "distance": float(self.data.qpos[0])}, "private": 123}

    def public_events(self):
        return [{"time": .3, "event": "fall"}] if self.elapsed >= .3 else []

    def decorate_scene(self, scene):
        scene.effect = self.elapsed >= .3


class FakeRenderer:
    def __init__(self, model, height, width):
        self.width, self.height = width, height
        self.scene = SimpleNamespace(effect=False)

    def update_scene(self, data, camera):
        self.scene.effect = False
        self.time = float(data.time)

    def render(self):
        return np.full((self.height, self.width, 3), 220 if self.scene.effect else round(50 + self.time * 10),
                       dtype=np.uint8)

    def __enter__(self):
        return self

    def __exit__(self, *args):
        pass


@pytest.fixture
def record(monkeypatch):
    monkeypatch.setattr(replay.catalog, "module_for", lambda scenario: sys.modules[__name__])
    def run(output, **options):
        sim = SmallSimulation()
        result = replay.record_scenario("quadruped_gait_failure", output, sim=sim,
                                        renderer_factory=FakeRenderer, fps=5, width=160, height=90, **options)
        return result, sim
    return run


def test_views_share_timestamps_and_frames_preserve_the_final_state(tmp_path, record):
    manifest, sim = record(tmp_path / "recording")
    assert [frame["t_s"] for frame in manifest["frames"]] == pytest.approx([0., .2, .4])
    assert manifest["frames"][-1]["t_s"] == manifest["duration_s"]
    assert [sample["position"][0] for sample in manifest["samples"]] == pytest.approx([0., .2, .4])
    assert manifest["events"] == [{"time": .3, "event": "fall", "label": "Fall", "is_failure": True}]
    for frame in manifest["frames"]:
        for camera in ("side", "overview"):
            assert (tmp_path / "recording/public" / frame["views"][camera]).is_file()
            assert (tmp_path / "recording/public" / frame["evidence"][camera]).is_file()
    assert sim.finished


def test_effects_preserve_raw_images_physics_and_public_separation(tmp_path, record):
    with_fx, first = record(tmp_path / "with")
    plain, second = record(tmp_path / "plain", effects=False)
    np.testing.assert_array_equal(first.data.qpos, second.data.qpos)
    assert with_fx["samples"] == plain["samples"]
    assert with_fx["events"] == plain["events"]
    raw = tmp_path / "with/public" / with_fx["frames"][-1]["evidence"]["side"]
    presentation = tmp_path / "with/public" / with_fx["frames"][-1]["views"]["side"]
    assert np.array(Image.open(raw)).mean() < 60
    assert np.array(Image.open(presentation)).mean() > 200
    observations = (tmp_path / "with/public/observations.jsonl").read_text()
    assert "PRIVATE_PARAMETER" not in observations
    assert "PRIVATE_PARAMETER" not in json.dumps(with_fx)
    assert "PRIVATE_PARAMETER" in (tmp_path / "with/private/diagnostics.jsonl").read_text()
    assert (tmp_path / "with/public/task.md").is_file()
    interface = (tmp_path / "with/public/CONTROL_INTERFACE.md").read_text()
    assert "joint_targets" in interface and "rotor_commands" in interface
    assert "coordination_defect" not in interface and "feasibility" not in interface


def test_existing_recording_cannot_be_overwritten(tmp_path, record):
    path = tmp_path / "recording"
    record(path)
    original = (path / "public/manifest.json").read_bytes()
    with pytest.raises(FileExistsError):
        record(path)
    assert (path / "public/manifest.json").read_bytes() == original


def test_failed_run_never_publishes_complete_manifest(tmp_path, monkeypatch):
    monkeypatch.setattr(replay.catalog, "module_for", lambda scenario: sys.modules[__name__])
    sim = SmallSimulation()
    sim.step = lambda control=None: None
    with pytest.raises(RuntimeError, match="did not advance"):
        replay.record_scenario("quadruped_gait_failure", tmp_path / "broken", sim=sim,
                               renderer_factory=FakeRenderer, width=160, height=90)
    assert not (tmp_path / "broken/public/manifest.json").exists()
    assert (tmp_path / "broken/private/recording_failed.json").is_file()


@pytest.mark.parametrize("options", [{"fps": 0}, {"fps": True}, {"width": 0}, {"height": 4000},
                                    {"cameras": ("side", "side")}, {"effects": "yes"}])
def test_invalid_recording_options_fail_before_creating_output(tmp_path, options):
    with pytest.raises(ValueError):
        replay.record_scenario("quadruped_gait_failure", tmp_path / "invalid", **options)
    assert not (tmp_path / "invalid").exists()


def test_standalone_player_inlines_assets_and_escapes_manifest_text(tmp_path):
    dist, public = tmp_path / "dist", tmp_path / "public"
    (dist / "assets").mkdir(parents=True)
    public.mkdir()
    (dist / "index.html").write_text('<html><head><script type="module" src="/assets/app.js"></script>'
                                    '<link rel="stylesheet" href="/assets/app.css"></head><body></body></html>')
    (dist / "assets/app.js").write_text('const title = "</script>";')
    (dist / "assets/app.css").write_text('body{color:blue}')
    assert replay.export_player(public, {"title": "</script><script>bad()</script>"}, dist)
    html = (public / "replay.html").read_text()
    assert 'src="/assets/' not in html and 'href="/assets/' not in html
    assert "window.__SCENARIO_REPLAY__=" in html
    assert '<script>bad()' not in html
    assert '<style>body{color:blue}</style>' in html
    assert not replay.export_player(public, {}, tmp_path / "no-build")


def test_controller_selection_does_not_change_physical_fixture_identity(tmp_path, monkeypatch):
    monkeypatch.setattr(replay.catalog, "module_for", lambda scenario: sys.modules[__name__])
    records = []
    for enabled in (True, False):
        sim = SmallSimulation()
        sim.config.coordination_defect = enabled
        records.append(replay.record_scenario("quadruped_gait_failure", tmp_path / str(enabled), sim=sim,
                                              renderer_factory=FakeRenderer, width=160, height=90, fps=5))
    original, control = records
    assert original["physics_sha256"] == control["physics_sha256"]
    assert original["controller_sha256"] != control["controller_sha256"]
    assert original["provenance"] == "original_attempt"
    assert control["provenance"] == "developer_control"
