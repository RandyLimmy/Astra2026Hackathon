"""Decorative train tracks follow the route and cannot alter the rollout."""
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from simulator.platforms.warehouse import Config, PRESETS, Simulation, model_xml


def test_track_geometry_is_visual_and_matches_wheel_gauge():
    sim = Simulation(Config(**PRESETS["warehouse_curve_demo"]))
    rails = [i for i in range(sim.model.ngeom) if sim.model.geom(i).name.startswith("track_rail_")]
    sleepers = [i for i in range(sim.model.ngeom) if sim.model.geom(i).name.startswith("track_sleeper_")]
    assert len(rails) > 100
    assert len(sleepers) > 20
    scenery = [i for i in range(sim.model.ngeom)
               if sim.model.geom(i).name.startswith(("track_", "deck_support_"))]
    assert not sim.model.geom_contype[scenery].any()
    assert not sim.model.geom_conaffinity[scenery].any()
    left, right = sim.model.geom("track_rail_-1_2"), sim.model.geom("track_rail_1_2")
    assert abs(np.linalg.norm(left.pos[:2] - right.pos[:2]) - .71) < 1e-10


def test_tracks_preserve_mass_and_actual_physics():
    config = Config(**{**PRESETS["warehouse_curve_demo"], "duration": 7.})
    with_tracks = Simulation(config)
    root = ET.fromstring(model_xml(config))
    for parent in root.iter():
        for child in list(parent):
            if child.get("name", "").startswith(("track_", "deck_support_")):
                parent.remove(child)
    bare = Simulation(config)
    # Recompile the identical physical model with scenery omitted. All joints,
    # body IDs and the explicitly named deck pairs keep their original topology.
    bare.model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    bare.data = mujoco.MjData(bare.model)
    for name, attr in (("cargo_deck", "_deck"), ("floor", "_floor")):
        setattr(bare, attr, bare.model.geom(name).id)
    bare._cargo_geoms = {bare.model.geom("cargo_box").id}
    bare._tires = np.array([bare.model.geom(f"tire_{side}").id for side in ("left", "right")])
    bare._initialize_trial()
    np.testing.assert_array_equal(with_tracks.model.body_mass, bare.model.body_mass)
    while not with_tracks.finished:
        with_tracks.step()
        bare.step()
        np.testing.assert_allclose(with_tracks.data.qpos, bare.data.qpos, atol=1e-10, rtol=0)
    assert with_tracks.summary()["public"]["outcome"] == "cargo_spilled"
    assert bare.summary()["public"]["outcome"] == "cargo_spilled"
