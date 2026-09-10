"""Train-style route scenery; all rails are flush, force-free floor inlays."""
import math
import xml.etree.ElementTree as ET

import numpy as np


def add_tracks(world: ET.Element, chassis: ET.Element, centerline: np.ndarray, gauge: float) -> None:
    """Match the declared route and wheel spacing without guiding the vehicle."""
    points = np.asarray(centerline, dtype=float)[::4]
    if not np.array_equal(points[-1], centerline[-1]):
        points = np.vstack((points, centerline[-1]))
    first = (points[1] - points[0]) / np.linalg.norm(points[1] - points[0])
    last = (points[-1] - points[-2]) / np.linalg.norm(points[-1] - points[-2])
    points = np.vstack((points[0] - first * .8, points, points[-1] + last * .5))

    def box(parent, name, position, size, color, angle=0.0):
        ET.SubElement(parent, "geom", {
            "name": name, "class": "visual", "type": "box",
            "pos": " ".join(map(str, position)), "size": " ".join(map(str, size)),
            "rgba": color, "euler": f"0 0 {angle}",
        })

    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    distances = np.r_[0., np.cumsum(lengths)]
    # Rails are inlaid at floor height; they cannot snag a wheel or a parcel.
    for index, (a, b, length) in enumerate(zip(points, points[1:], lengths)):
        tangent = (b - a) / length
        normal = np.array([-tangent[1], tangent[0]])
        center = (a + b) / 2
        heading = math.atan2(tangent[1], tangent[0])
        box(world, f"track_bed_{index}", [*center, .0005],
            [length / 2 + .003, .56, .0004], ".17 .19 .20 1", heading)
        for side in (-1, 1):
            rail = center + side * gauge / 2 * normal
            box(world, f"track_rail_base_{side}_{index}", [*rail, .004],
                [length / 2 + .003, .037, .001], ".23 .27 .29 1", heading)
            box(world, f"track_rail_{side}_{index}", [*rail, .006],
                [length / 2 + .003, .022, .001], ".64 .69 .72 1", heading)

    for index, at in enumerate(np.arange(.05, distances[-1], .25)):
        center = np.array([np.interp(at, distances, points[:, axis]) for axis in (0, 1)])
        segment = min(len(lengths) - 1, np.searchsorted(distances, at, side="right") - 1)
        tangent = (points[segment + 1] - points[segment]) / lengths[segment]
        heading = math.atan2(tangent[1], tangent[0])
        box(world, f"track_sleeper_{index}", [*center, .002], [.065, .51, .001],
            ".32 .23 .16 1" if index % 2 else ".38 .28 .20 1", heading)

    # Show the structure carrying the raised deck; these never add mass/contact.
    for x in (-.37, .37):
        for y in (-.22, .22):
            box(chassis, f"deck_support_{x}_{y}", [x, y, .115], [.025, .025, .065], ".18 .23 .26 1")
