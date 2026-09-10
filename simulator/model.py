"""Compile the common physical car, with scenario parameters set before a run."""
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco

from .config import Experiment

ASSETS = Path(__file__).parent / "assets"


def _expanded(path: Path) -> ET.Element:
    root = ET.parse(path).getroot()
    for include in list(root.findall("include")):
        offset = list(root).index(include)
        root.remove(include)
        for child in _expanded(path.parent / include.attrib["file"]):
            root.insert(offset, child)
            offset += 1
    return root


def model_xml(config: Experiment) -> str:
    root = _expanded(ASSETS / ("track.xml" if config.wall else "track_no_wall.xml"))
    option = root.find("option")
    assert option is not None
    option.set("timestep", str(config.timestep))
    geoms = {g.get("name"): g for g in root.iter("geom")}
    segments = (("road_before", -500, config.wet_start),
                ("road_patch", config.wet_start, config.wet_end),
                ("road_after", config.wet_end, 1000))
    for name, start, end in segments:
        geoms[name].set("pos", f"{(start + end) / 2} 0 -0.1")
        geoms[name].set("size", f"{(end - start) / 2} 20 0.1")
    geoms["road_patch"].set("friction", f"{config.wet_friction} 0.005 0.0001")
    if config.wet_friction < 1:
        geoms["road_patch"].set("rgba", "0.10 0.29 0.38 1")
    if config.wall:
        geoms["wall"].set("pos", f"{config.wall_x + 0.5} 0 1.5")
    geoms["brake_line"].set("pos", f"{config.brake_at} 0 0.008")
    chassis = next(b for b in root.iter("body") if b.get("name") == "chassis")
    if config.payload:
        # Fixed child body: compiler handles COM and inertia consistently.
        cargo = ET.SubElement(chassis, "body", name="payload", pos="0 0 0.65")
        ET.SubElement(cargo, "geom", name="cargo", type="box", size="0.65 0.55 0.2",
                      mass=str(config.payload), rgba="0.8 0.56 0.23 1")
    # Cosmetic distance ticks never contribute contact forces.
    world = root.find("worldbody")
    assert world is not None
    for x in range(0, 161, 10):
        ET.SubElement(world, "geom", type="box", pos=f"{x} -3.6 0.006",
                      size="0.035 0.35 0.005", rgba="0.65 0.7 0.74 1",
                      contype="0", conaffinity="0")
    return ET.tostring(root, encoding="unicode")


def build_model(config: Experiment) -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_string(model_xml(config))
