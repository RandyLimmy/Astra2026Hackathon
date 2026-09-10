"""Local builder commands. These are not the seven final Astra tools."""

import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path

from contracts import ExperimentConfig
from .experiment import predict_from_history, run_reference, stopping_error

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CANDIDATE = ROOT / "candidate" / "actuator.py"


def write_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")


def _config(args):
    return ExperimentConfig(speed_mps=args.speed, brake_strength=args.brake,
                            wall_distance_m=args.wall, preparation_cycles=args.cycles,
                            rest_time_s=args.rest, timestep_s=args.timestep)


def _pair(config, case_id, title, worker, fade=True):
    reference = run_reference(config, fade=fade)
    prediction = predict_from_history(config, reference.history, worker)
    return {"schema_version": 1, "case_id": case_id, "title": title,
            "experiment_kind": "synthetic_development", "config": config.to_dict(),
            "reference": reference.to_dict(), "candidate": prediction.to_dict(),
            "original_error_m": stopping_error(prediction, reference)}


def _print_bundle(bundle):
    def number(value):
        return "no stop" if value is None else f"{value:.2f} m"
    print(f"{bundle['title']}: original {number(bundle['candidate']['summary']['stopping_distance_m'])}; "
          f"reference {number(bundle['reference']['summary']['stopping_distance_m'])}; "
          f"wall crossed: {bundle['reference']['summary']['wall_crossed']}", flush=True)


def run_single(args):
    from component_worker.client import ActuatorWorker
    config = _config(args)
    with ActuatorWorker(args.candidate) as worker:
        bundle = _pair(config, "custom", "Custom development run", worker, fade=not args.normal_control)
    bundle["candidate_source_sha256"] = hashlib.sha256(args.candidate.read_bytes()).hexdigest()
    write_json(args.output, bundle)
    _print_bundle(bundle)
    print(f"Saved {args.output.resolve()}")


def run_demo(args):
    import mujoco
    from component_worker.client import ActuatorWorker
    from demo.plots import plot_development
    from evaluation.baselines import fit_fixed_force, make_fixed_force_source

    base = _config(args)
    if base.brake_strength == 0:
        raise ValueError("The comparison demo needs --brake > 0 to fit braking force; use run --brake 0 for a coasting experiment")
    definitions = [
        ("fresh", "Fresh brakes", replace(base, preparation_cycles=0, rest_time_s=0), True),
        ("repeated", "Repeated braking", replace(base, rest_time_s=0), True),
        ("recovered", "After recovery", replace(base, rest_time_s=args.recovery), True),
        ("normal_control", "No-fade control", replace(base, rest_time_s=0), False),
    ]
    args.output.mkdir(parents=True, exist_ok=True)
    bundles = []
    with ActuatorWorker(DEFAULT_CANDIDATE) as worker:
        for case_id, title, config, fade in definitions:
            bundle = _pair(config, case_id, title, worker, fade)
            bundle["candidate_source_sha256"] = hashlib.sha256(DEFAULT_CANDIDATE.read_bytes()).hexdigest()
            bundles.append(bundle)
            _print_bundle(bundle)
    # The adequate-model control is a separate reference world, not fit data.
    fit = fit_fixed_force([b["reference"] for b in bundles if b["case_id"] != "normal_control"])
    fit_path = args.output / "parameter_fit.py"
    fit_path.write_text(make_fixed_force_source(fit["force_n"]))
    with ActuatorWorker(fit_path) as worker:
        for bundle in bundles:
            config = ExperimentConfig(**bundle["config"])
            result = predict_from_history(config, bundle["reference"]["history"], worker)
            bundle["parameter_fit"] = result.to_dict()
    if args.developer_check:
        with ActuatorWorker(ROOT / "reference_host" / "developer_actuator.py") as worker:
            for bundle in bundles:
                config = ExperimentConfig(**bundle["config"])
                result = predict_from_history(config, bundle["reference"]["history"], worker)
                bundle["developer_check"] = result.to_dict()
        print("Included a DEVELOPER-WRITTEN stateful check, not an Astra repair.", flush=True)
    for bundle in bundles:
        write_json(args.output / f"{bundle['case_id']}.json", bundle)
    write_json(args.output / "manifest.json", {
        "schema_version": 1, "builder_only": True, "backend": "MuJoCo", "version": mujoco.__version__,
        "kind": "synthetic_development", "agent_run": False, "final_holdout_evaluation": False,
        "parameter_fit": fit, "cases": [b["case_id"] for b in bundles],
        "candidate_source_sha256": bundles[0]["candidate_source_sha256"],
    })
    plot_development(bundles, args.output / "comparison.png")
    print(f"Saved computed trajectories and chart in {args.output.resolve()}")
    print(f"View the native replay: .venv/bin/mjpython -m sim.cli view {args.output / 'repeated.json'}")


def show(args):
    from demo.viewer import render_comparison, show_comparison
    data = json.loads(args.bundle.read_text())
    kwargs = dict(wall_distance_m=data["config"]["wall_distance_m"])
    if args.command == "render":
        render_comparison(data["candidate"], data["reference"], output_path=args.output, **kwargs)
        print(f"Saved {args.output.resolve()}")
    else:
        show_comparison(data["candidate"], data["reference"], title=f"{data['title']} · synthetic replay",
                        duration_s=args.duration, **kwargs)


def export_task(args):
    """Export only approved candidate material; no builder repository mounts."""
    import shutil
    if args.output.exists() and any(args.output.iterdir()):
        raise ValueError("Task export directory must be empty")
    args.output.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(DEFAULT_CANDIDATE, args.output / "actuator.py")
    shutil.copyfile(ROOT / "contracts" / "ACTUATOR.md", args.output / "INTERFACE.md")
    (args.output / "TASK.md").write_text(
        "# Model investigation\n\nHere is an incomplete Python component. Use observed histories "
        "and experiments to improve its predictions. State expected observations before each "
        "experiment and report uncertainty.\n\nThis package is a starting source/interface "
        "fixture. An experiment broker is not connected in this milestone.\n")
    print(f"Exported only actuator.py, INTERFACE.md, and TASK.md to {args.output.resolve()}")


def main():
    parser = argparse.ArgumentParser(description="RealityPatch: synthetic MuJoCo braking foundation")
    commands = parser.add_subparsers(dest="command", required=True)
    for command in ("run", "demo"):
        sub = commands.add_parser(command)
        sub.add_argument("--speed", type=float, default=20.0, help="Initial probe speed in m/s")
        sub.add_argument("--brake", type=float, default=1.0, help="Brake command from 0 to 1")
        sub.add_argument("--wall", type=float, default=40.0, help="Wall distance from probe origin, metres")
        sub.add_argument("--cycles", type=int, default=5, help="Full braking preparation cycles")
        sub.add_argument("--timestep", type=float, default=0.01)
        sub.add_argument("--output", type=Path, default=Path("artifacts/latest") if command == "demo" else Path("artifacts/custom.json"))
        if command == "run":
            sub.add_argument("--rest", type=float, default=0.0, help="Rest after preparation, seconds")
            sub.add_argument("--normal-control", action="store_true", help="Use the fixed-effectiveness reference")
            sub.add_argument("--candidate", type=Path, default=DEFAULT_CANDIDATE)
        else:
            sub.set_defaults(rest=0.0)
            sub.add_argument("--recovery", type=float, default=300.0, help="Recovery-case wait in seconds")
            sub.add_argument("--developer-check", action="store_true")
    for command in ("view", "render"):
        sub = commands.add_parser(command)
        sub.add_argument("bundle", type=Path)
        if command == "render":
            sub.add_argument("--output", type=Path, default=Path("artifacts/scene.png"))
        else:
            sub.add_argument("--duration", type=float, default=None, help="Close after this many wall-clock seconds")
    sub = commands.add_parser("export-task")
    sub.add_argument("--output", type=Path, default=Path("artifacts/task-package"))
    args = parser.parse_args()
    try:
        if args.command == "run":
            run_single(args)
        elif args.command == "demo":
            run_demo(args)
        elif args.command == "export-task":
            export_task(args)
        else:
            show(args)
    except (ValueError, RuntimeError, OSError) as error:
        parser.exit(1, f"Error: {error}\n")


if __name__ == "__main__":
    main()
