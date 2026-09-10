"""Render synchronized, measured trolley predictions and cargo-failure animation.

Operator presentation artifacts include cargo diagnostics. They are deliberately
separate from the neutral evidence exported by ``investigation.warehouse``.
"""

from __future__ import annotations

import argparse
from dataclasses import replace
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
from uuid import uuid4

import mujoco
import numpy as np

from .platforms import warehouse
from .view_controls import setting_overrides


def _write(path: Path, value) -> str:
    encoded = (json.dumps(value, sort_keys=True, allow_nan=False, indent=2) + "\n").encode()
    with path.open("xb") as stream:
        stream.write(encoded)
    return hashlib.sha256(encoded).hexdigest()


def capture(sim, fps: int = 15) -> dict:
    """Record actual integrated states; presentation never synthesizes motion."""
    if type(fps) is not int or not 1 <= fps <= 60:
        raise ValueError("fps must be an integer between 1 and 60")
    rows = []
    tick = 0
    maximum = math.ceil(sim.config.duration / sim.config.timestep) + 2
    for _ in range(maximum):
        if sim.elapsed + 1e-9 >= tick / fps or sim.finished:
            observation = sim.observe()
            diagnostics = sim.diagnostics()
            rows.append({"observation": observation,
                         "cargo_displacement": diagnostics["cargo_displacement"],
                         "cargo_latched": diagnostics["cargo_latched"],
                         "center_of_mass": diagnostics["center_of_mass"],
                         "qpos": sim.data.qpos.tolist(), "qvel": sim.data.qvel.tolist(),
                         "eq_active": sim.data.eq_active.tolist(),
                         "joint_range": sim.model.jnt_range.tolist()})
            for key in ("cargo_position", "cargo_ground_contact", "cargo_dropped", "cargo_on_deck",
                        "cargo_linear_velocity", "cargo_angular_velocity", "cargo_tilt_rad"):
                if key in diagnostics:
                    rows[-1][key] = diagnostics[key]
            tick = math.floor((sim.elapsed + 1e-9) * fps) + 1
        if sim.finished:
            break
        before = sim.elapsed
        sim.step()
        if (sim.elapsed <= before or not np.isfinite(sim.data.qpos).all()
                or not np.isfinite(sim.data.qvel).all() or sim.data.warning.number.any()):
            raise RuntimeError("Cargo replay encountered invalid physics")
    if not sim.finished:
        raise RuntimeError("Cargo replay exceeded the declared trial length")
    return {"samples": rows, "summary": sim.summary()["public"],
            "events": sim.diagnostics()["events"]}


def comparison(nominal: dict, actual: dict, *, feedback=False) -> dict:
    left, right = nominal["samples"], actual["samples"]
    if len(left) != len(right) or not left:
        raise ValueError("Comparison requires synchronized nonempty trajectories")
    same_commands = all(a["observation"]["command"] == b["observation"]["command"]
                        for a, b in zip(left, right))
    if not same_commands and not feedback:
        raise ValueError("Comparison requires identical times and motor commands")
    if any(abs(a["observation"]["time"] - b["observation"]["time"]) > 1e-8 for a, b in zip(left, right)):
        raise ValueError("Comparison requires identical times and motor commands")
    delta = np.asarray([a["observation"]["position"] for a in left]) - np.asarray(
        [b["observation"]["position"] for b in right])
    error = np.linalg.norm(delta[:, :2], axis=1)
    release = next((row["observation"]["time"] for row in right if not row["cargo_latched"]), None)
    result: dict = {"position_rmse_m": float(np.sqrt(np.mean(error ** 2))),
            "maximum_position_error_m": float(error.max()),
            "final_position_error_m": float(error[-1]),
            "maximum_cargo_travel_m": max(abs(row["cargo_displacement"]) for row in right),
            "first_recorded_release_s": release,
            "actual_outcome": actual["summary"]["outcome"],
            "identical_commands": same_commands,
            "control_comparison": "same route feedback controller" if feedback else "identical motor commands"}
    if all("cargo_position" in row for row in left + right):
        cargo_error = np.linalg.norm(np.asarray([row["cargo_position"] for row in left]) -
                                     np.asarray([row["cargo_position"] for row in right]), axis=1)
        result.update(cargo_position_rmse_m=float(np.sqrt(np.mean(cargo_error ** 2))),
                      final_cargo_position_error_m=float(cargo_error[-1]),
                      cargo_dropped=any(row.get("cargo_dropped", False) for row in right),
                      first_recorded_ground_contact_s=next((row["observation"]["time"] for row in right
                                                           if row.get("cargo_ground_contact")), None))
    return result


def stage(row: dict, release: float | None, duration: float, *, curve=False) -> tuple[str, str]:
    when = row["observation"]["time"]
    if when >= duration - 1e-8:
        if curve:
            if row.get("cargo_dropped"):
                return "04 / DELIVERY FAILED", "The parcel remains on the floor; the trolley finished without its load"
            if row["observation"].get("route_progress", 0) >= warehouse.ROUTE_LENGTH - .05:
                return "04 / DELIVERY COMPLETE", "The parcel stayed on the deck through the entire route"
            return "04 / ATTEMPT ENDED", "The delivery route is not complete"
        return "04 / RESULT", "Measure the prediction error"
    if curve:
        if row.get("cargo_dropped"):
            if ("cargo_linear_velocity" in row
                    and np.linalg.norm(row["cargo_linear_velocity"]) < .05
                    and np.linalg.norm(row["cargo_angular_velocity"]) < .15):
                return "03 / PARCEL SETTLED", "The parcel rests on the floor as the trolley continues"
            return "03 / CARGO LOST", "The load leaves the deck and hits the ground"
        if not row["cargo_latched"]:
            if row.get("cargo_on_deck") is False:
                return "02 / PARCEL FALLING", "The parcel clears the deck and falls under gravity"
            if row.get("cargo_tilt_rad", 0.) > math.radians(12):
                return "02 / PARCEL TIPPING", "The parcel tips naturally over the deck edge"
            return "02 / PARCEL SLIDING", "The released parcel slides outward through the bend"
        if "bend" in row["observation"]["phase"] or "turn" in row["observation"]["phase"]:
            return "02 / SHARP BEND", "The trolley follows the marked right turn"
        if any(word in row["observation"]["phase"] for word in ("exit", "stop", "finish")):
            return "03 / LOAD RETAINED", "The trolley exits the bend with its cargo aboard"
        return "01 / APPROACH", "Follow the marked track into the right-angle bend"
    if not row["cargo_latched"]:
        if release is not None and when < release + 1.2:
            return "02 / LOAD MOVES", "The restraint releases; cargo slides"
        return "03 / PATHS DIVERGE", "Changed weight distribution changes the motion"
    if when < .5:
        return "01 / READY", "Same trolley. Same load. Same motor commands."
    return "01 / SECURED APPROACH", "The original model and measured motion agree"


def _restore(sim, row):
    sim.data.time = row["observation"]["time"]
    sim.data.qpos[:] = row["qpos"]
    sim.data.qvel[:] = row["qvel"]
    sim.data.eq_active[:] = row["eq_active"]
    sim.model.jnt_range[:] = row["joint_range"]
    mujoco.mj_forward(sim.model, sim.data)


def _font(size: int):
    from PIL import ImageFont
    for name in ("/System/Library/Fonts/Supplemental/Arial.ttf", "DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            pass
    return ImageFont.load_default(size=size)


def _view(renderer, sim, row):
    _restore(sim, row)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = sim.data.xpos[sim.focus_body] + [0, 0, .15]
    camera.distance = 2.65
    camera.azimuth = 135
    camera.elevation = -28
    if sim.config.probe in getattr(warehouse, "CURVE_PROBES", ()):
        cargo = np.asarray(row.get("cargo_position", sim.data.xpos[sim.focus_body]))
        cart = sim.data.xpos[sim.focus_body]
        route_points = np.asarray(warehouse.route_geometry()["centerline"])
        route_center = (route_points.min(axis=0) + route_points.max(axis=0)) / 2
        wide_center = np.array([*route_center[:2], .15])
        close_center = (cart + cargo) / 2 + [0, 0, .15]
        fraction = float(np.clip((row["observation"]["time"] - 1) / 3, 0, 1))
        blend = fraction * fraction * (3 - 2 * fraction)
        camera.lookat[:] = (1 - blend) * wide_center + blend * close_center
        camera.distance = (1 - blend) * 8.5 + blend * max(4.0, float(np.linalg.norm(cart - cargo)) + 2.7)
        camera.azimuth = 215
        camera.elevation = (1 - blend) * -50 + blend * -30
    renderer.update_scene(sim.data, camera=camera)
    # This marker is an operator visualization of the measured whole-trolley COM.
    scene = renderer.scene
    if sim.config.probe not in getattr(warehouse, "CURVE_PROBES", ()) and scene.ngeom + 1 < scene.maxgeom:
        center = np.asarray(row["center_of_mass"])
        raised = center + [0, 0, .62]
        mujoco.mjv_initGeom(scene.geoms[scene.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE,
                           [.027, .027, .027], raised,
                           np.eye(3).ravel(), np.array([1., .3, .65, 1.]))
        scene.ngeom += 1
        mujoco.mjv_initGeom(scene.geoms[scene.ngeom], mujoco.mjtGeom.mjGEOM_CAPSULE,
                           [.003, .003, .003], center, np.eye(3).ravel(),
                           np.array([1., .3, .65, .7]))
        mujoco.mjv_connector(scene.geoms[scene.ngeom], mujoco.mjtGeom.mjGEOM_CAPSULE,
                             .003, center, raised)
        scene.ngeom += 1
    return renderer.render().copy()


def _draw_paths(draw, traces, index: int, colors, font, route=None):
    all_points = np.concatenate([np.asarray([row["observation"]["position"][:2]
                                           for row in trace["samples"]]) for trace in traces])
    if route is not None:
        all_points = np.concatenate([all_points, np.asarray(route)[:, :2], np.asarray(
            [row["cargo_position"][:2] for row in traces[1]["samples"]])])
    lower, upper = all_points.min(axis=0) - .35, all_points.max(axis=0) + .35
    span = np.maximum(upper - lower, .7)
    # x-forward points right, y-left points up; both axes use the same metre scale.
    scale = min(650 / span[0], 135 / span[1])
    center = (lower + upper) / 2

    def point(value):
        x, y = (np.asarray(value[:2]) - center) * scale
        return float(370 + x), float(671 - y)

    draw.rounded_rectangle((24, 570, 760, 776), radius=12, fill="#172431")
    draw.text((44, 585), "PATH IN WORLD COORDINATES", font=font, fill="#adc0d0")
    if route is not None:
        draw.line([point(value) for value in route], fill="#6b7b88", width=8)
    for trace, color in zip(traces, colors):
        points = [point(row["observation"]["position"]) for row in trace["samples"][:index + 1]]
        if len(points) > 1:
            draw.line(points, fill=color, width=3)
        x, y = points[-1]
        draw.ellipse((x - 5, y - 5, x + 5, y + 5), fill=color)
    if route is not None:
        cargo_path = [point(row["cargo_position"]) for row in traces[1]["samples"][:index + 1]
                      if not row["cargo_latched"]]
        if len(cargo_path) > 1:
            draw.line(cargo_path, fill="#ff7180", width=2)
            x, y = cargo_path[-1]
            draw.rectangle((x - 4, y - 4, x + 4, y + 4), fill="#ff7180")
    start = point(traces[0]["samples"][0]["observation"]["position"])
    draw.text((start[0] - 10, start[1] + 8), "Start", font=font, fill="#adc0d0")
    draw.line((626, 755, 626 + scale, 755), fill="#adc0d0", width=2)
    draw.text((626, 732), "1 m", font=font, fill="#adc0d0")


def render(output: Path, nominal_sim, actual_sim, traces: list[dict], report: dict, *, fps: int):
    """Export a GIF and, when locally available, an MP4 using identical frames."""
    from PIL import Image, ImageDraw

    nominal, actual = traces[:2]
    curve = actual_sim.config.probe in getattr(warehouse, "CURVE_PROBES", ())
    route = warehouse.route_geometry()["centerline"] if curve else None
    font, small, title = _font(21), _font(15), _font(34)
    colors = ["#68cced", "#ffbd69", "#98e1b0"]
    frames = []
    frames_dir = output / "frames"
    frames_dir.mkdir()
    mp4 = shutil.which("ffmpeg")
    encoder = None
    encoder_log = None
    if mp4:
        encoder_log = (output / "encoder.log").open("wb")
        encoder = subprocess.Popen(
            [mp4, "-v", "error", "-f", "rawvideo", "-pixel_format", "rgb24", "-video_size", "1280x832",
             "-framerate", str(fps), "-i", "-", "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p",
             "-movflags", "+faststart", str(output / "animation.mp4")],
            stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=encoder_log)
    count = len(actual["samples"])
    release = report["baseline"]["first_recorded_release_s"]
    duration = actual["samples"][-1]["observation"]["time"]
    representative = {0, count - 1, count // 2}
    if release is not None:
        representative.update(min(range(count), key=lambda i: abs(
            actual["samples"][i]["observation"]["time"] - when))
            for when in (max(0, release - .2), release + .7))
    ground_time = report["baseline"].get("first_recorded_ground_contact_s")
    if ground_time is not None:
        representative.update(min(range(count), key=lambda i: abs(
            actual["samples"][i]["observation"]["time"] - when))
            for when in (ground_time - .3, ground_time + .3))
    try:
        with mujoco.Renderer(nominal_sim.model, width=600, height=330) as left, \
                mujoco.Renderer(actual_sim.model, width=600, height=330) as right:
            for index, (a, b) in enumerate(zip(nominal["samples"], actual["samples"])):
                canvas = Image.new("RGB", (1280, 832), "#0d1722")
                draw = ImageDraw.Draw(canvas)
                when = b["observation"]["time"]
                label, explanation = stage(b, release, duration, curve=curve)
                draw.text((24, 17), "REALITYPATCH / CARGO DYNAMICS", font=small, fill="#9ab1c4")
                draw.text((24, 43), "A sharp turn. A lost load." if curve else "When cargo breaks loose",
                          font=title, fill="#f4f7fa")
                subtitle = "Train-style track  /  Physical slide, tip and landing" if curve else "Identical motor commands  /  Physical MuJoCo replay"
                draw.text((24, 91), subtitle, font=small, fill="#b4c5d3")
                draw.text((792, 29), label, font=font, fill=colors[1])
                draw.text((792, 64), f"{when:04.1f} / {duration:.1f} seconds", font=font, fill="#f4f7fa")
                for x, row, caption, subtitle, color, pixels in (
                    (24, a, "ORIGINAL MODEL", "Predicts cargo remains secured", colors[0], _view(left, nominal_sim, a)),
                    (656, b, "MEASURED SIMULATION", "Real cargo, deck and ground contacts" if curve else
                     "Physical load and wheel contacts", colors[1], _view(right, actual_sim, b)),
                ):
                    draw.rounded_rectangle((x, 128, x + 600, 552), radius=12, fill="#172431")
                    draw.text((x + 18, 142), caption, font=font, fill=color)
                    draw.text((x + 18, 171), subtitle, font=small, fill="#b4c5d3")
                    canvas.paste(Image.fromarray(pixels), (x, 198))
                    latched = "Cargo on ground" if row.get("cargo_dropped") else "Secured" if row["cargo_latched"] else "Cargo released"
                    caption = (f"{latched}   /   Cargo height {row['cargo_position'][2]:.2f} m" if curve else
                               f"{latched}   /   Cargo travel {abs(row['cargo_displacement']) * 100:.1f} cm")
                    draw.text((x + 18, 530), caption,
                              font=small, fill=color)
                _draw_paths(draw, traces, index, colors, small, route=route)
                expected_position = a["cargo_position"] if curve else a["observation"]["position"][:2]
                observed_position = b["cargo_position"] if curve else b["observation"]["position"][:2]
                error = float(np.linalg.norm(np.asarray(expected_position) - observed_position))
                draw.text((788, 580), "CARGO PREDICTION ERROR" if curve else "PREDICTION ERROR", font=small, fill="#adc0d0")
                draw.text((788, 603), f"{error:.2f} m", font=title, fill=colors[1])
                draw.text((788, 656), "Original", font=small, fill=colors[0])
                draw.text((874, 656), "Measured", font=small, fill=colors[1])
                if len(traces) == 3:
                    c = traces[2]["samples"][index]
                    c_position = c["cargo_position"] if curve else c["observation"]["position"][:2]
                    candidate_error = float(np.linalg.norm(np.asarray(c_position) - observed_position))
                    draw.text((788, 686), f"Candidate: {candidate_error:.2f} m", font=font, fill=colors[2])
                    draw.text((788, 720), "Model " + report["candidate_sha256"][:16], font=small, fill="#adc0d0")
                else:
                    draw.text((788, 689), "GPT-6 investigation ready", font=font, fill="#f4f7fa")
                    draw.text((788, 722), "No candidate repair evaluated in this replay", font=small, fill="#adc0d0")
                legend = "Grey: intended route   /   Pink: released parcel" if curve else "Pink marker: center of mass (raised for visibility)"
                draw.text((44, 755), legend, font=small, fill="#adc0d0")
                draw.text((24, 793), explanation, font=font, fill="#f4f7fa")
                draw.rectangle((24, 825, 24 + round(1232 * when / duration), 829), fill=colors[1])
                if index in representative:
                    canvas.save(frames_dir / f"frame-{index:04d}.png")
                if encoder is not None and encoder.stdin is not None:
                    encoder.stdin.write(canvas.tobytes())
                # Independent palettes avoid keeping all full RGB frames in memory.
                frames.append(canvas.quantize(colors=160))
                if index % max(1, fps * 2) == 0:
                    print(f"Rendering {when:.1f} / {duration:.1f} s", flush=True)
        # GIF stores centiseconds: distribute 30/40 ms holds at 30 fps rather
        # than truncating every frame to 30 ms and speeding up the physical fall.
        delays = [10 * (round((i + 1) * 100 / fps) - round(i * 100 / fps)) for i in range(len(frames))]
        delays[-1] = 2200
        frames[0].save(output / "animation.gif", save_all=True, append_images=frames[1:],
                       duration=delays, loop=0, optimize=False, disposal=2)
        if encoder is not None and encoder.stdin is not None:
            # Match the GIF's final hold without adding any simulated motion.
            for _ in range(max(0, round(2.2 * fps) - 1)):
                encoder.stdin.write(canvas.tobytes())
    finally:
        if encoder is not None:
            if encoder.stdin is not None:
                encoder.stdin.close()
            code = encoder.wait(timeout=30)
            if encoder_log is not None:
                encoder_log.close()
            if code:
                (output / "animation.mp4").unlink(missing_ok=True)
                print("MP4 encoder failed; inspect encoder.log. GIF is the primary replay.", flush=True)
    return output / "animation.gif"


def build(output: Path, *, scenario="warehouse_curve_demo", overrides=None,
          candidate: Path | None = None, fps=30, media=True) -> dict:
    if scenario not in warehouse.PRESETS or not scenario.startswith("warehouse_"):
        raise ValueError("Choose an existing warehouse scenario")
    if type(fps) is not int or not 1 <= fps <= 60:
        raise ValueError("fps must be an integer between 1 and 60")
    config = warehouse.Config(**{**warehouse.PRESETS[scenario], **(overrides or {})})
    curve = config.probe in getattr(warehouse, "CURVE_PROBES", ())
    if config.duration > 20:
        raise ValueError("Presentation replays are limited to 20 seconds")
    nominal_sim = warehouse.Simulation(replace(config, fault="healthy"))
    actual_sim = warehouse.Simulation(config)
    candidate_sim = None
    candidate_source = None
    if candidate is not None:
        from investigation.warehouse import CandidateSimulation
        with Path(candidate).open("rb") as stream:
            candidate_source = stream.read(32_769)
        if len(candidate_source) > 32_768:
            raise ValueError("Candidate model exceeds 32 KiB")
        try:
            candidate_value = json.loads(candidate_source)
        except (ValueError, RecursionError):
            raise ValueError("Candidate model must contain a bounded JSON object") from None
        candidate_sim = CandidateSimulation(config, candidate_value)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    nominal = capture(nominal_sim, fps)
    nominal_hash = _write(output / "nominal-prediction.json", nominal)
    traces = [nominal]
    candidate_trace = None
    candidate_hash = None
    candidate_bytes_hash = None
    candidate_prediction_hash = None
    if candidate_sim is not None:
        candidate_trace = capture(candidate_sim, fps)
        candidate_prediction_hash = _write(output / "candidate-prediction.json", candidate_trace)
        # Copy the artifact into the replay so its identity survives later edits.
        assert candidate_source is not None
        (output / "candidate-model.json").write_bytes(candidate_source)
        candidate_hash = candidate_sim.candidate_sha256
        candidate_bytes_hash = hashlib.sha256(candidate_source).hexdigest()
    _write(output / "predictions-locked.json", {"nominal_sha256": nominal_hash,
                                               "candidate_sha256": candidate_hash,
                                               "candidate_source_bytes_sha256": candidate_bytes_hash,
                                               "candidate_prediction_sha256": candidate_prediction_hash})
    actual = capture(actual_sim, fps)
    _write(output / "measured-operator-trace.json", actual)
    traces.append(actual)
    report = {"scenario": scenario, "fps": fps, "baseline": comparison(nominal, actual, feedback=curve),
              "candidate_sha256": candidate_hash,
              "candidate_source_bytes_sha256": candidate_bytes_hash,
              "interpretation": "Synthetic physical experiment; model repair success requires separate unseen evaluation."}
    if candidate_trace is not None:
        report["candidate"] = comparison(candidate_trace, actual, feedback=curve)
        traces.append(candidate_trace)
    if curve:
        report["intended_route"] = warehouse.route_geometry()
    _write(output / "comparison.json", report)
    if media:
        render(output, nominal_sim, actual_sim, traces, report, fps=fps)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", default="warehouse_curve_demo")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--candidate", type=Path, help="execute this model artifact; never assumes it is repaired")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--set", dest="settings", action="append", metavar="NAME=VALUE")
    parser.add_argument("--no-media", action="store_true", help="record and compare physics without rendering")
    args = parser.parse_args(argv)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output = args.output or Path("runs") / f"cargo-demo-{stamp}-{uuid4().hex[:8]}"
    try:
        report = build(output, scenario=args.scenario, overrides=setting_overrides(args.settings),
                       candidate=args.candidate, fps=args.fps, media=not args.no_media)
    except (ValueError, OSError, RuntimeError) as error:
        parser.exit(1, f"Cargo demo failed: {error}\n")
    print(json.dumps({key: value for key, value in report.items() if key != "intended_route"}, indent=2), flush=True)
    print(f"Replay artifacts: {output.resolve()}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
