"""Physics acceptance matrix; run with python -m simulator.validate."""
import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
from datetime import datetime, timezone
import json
from pathlib import Path

from .config import Experiment
from .runner import run_experiment
from .scenarios import load, names


def evaluate_case(item):
    name, values = item
    result = run_experiment(Experiment.from_dict(values))
    result.pop("public")
    return name, result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("runs/validation"))
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--full", action="store_true", help="add repeatability, timestep and private matrix cases")
    args = parser.parse_args(argv)
    if args.workers < 1:
        parser.error("--workers must be positive")
    cases = {}
    for name in names():
        cases[name] = load(name)
        cases[name + "_free"] = replace(load(name), wall=False)
    if args.full:
        for name in names():
            cases[name + "_fine"] = replace(load(name), wall=False, timestep=load(name).timestep / 2)
            cases[name + "_fine_wall"] = replace(load(name), timestep=load(name).timestep / 2)
            for repeat in (2, 3):
                cases[f"{name}_repeat{repeat}"] = replace(load(name), wall=False)
        matrix_path = Path(__file__).parent / "private" / "matrix.json"
        for split, rows in json.loads(matrix_path.read_text()).items():
            for row in rows:
                base = load(row["scenario"]).to_dict()
                base.update(row["overrides"])
                cases[f'{split}/{row["id"]}'] = Experiment.from_dict(base)
    args.output.mkdir(parents=True, exist_ok=True)
    results, errors = {}, {}
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        jobs = {name: pool.submit(evaluate_case, (name, config.to_dict())) for name, config in cases.items()}
        for name, job in jobs.items():
            try:
                _, results[name] = job.result()
                result = results[name]
                distance = result["stopping_distance"]
                print(f"{name:34} collision={str(result['collision']):5} "
                      f"stop={distance if distance is not None else 'censored'}", flush=True)
            except Exception as exc:
                errors[name] = f"{type(exc).__name__}: {exc}"
                print(f"{name}: ERROR {exc}", flush=True)
    checks = {}

    def check(name, condition):
        checks[name] = bool(condition)

    if not errors:
        baseline = results["baseline_free"]["stopping_distance"]
        check("baseline_46_to_48m", baseline is not None and 46 <= baseline <= 48)
        check("baseline_safe", not results["baseline"]["collision"] and results["baseline"]["stopped"])
        check("baseline_lateral_under_025m", results["baseline"]["max_lateral_displacement"] < .25)
        check("hot_at_least_20pct_longer", results["brake_fade_free"]["stopping_distance"] >= baseline * 1.2)
        check("hot_collides", results["brake_fade"]["collision"])
        check("recovery_within_5pct", abs(results["recovery_free"]["stopping_distance"] / baseline - 1) < .05)
        check("wet_at_least_20pct_longer", results["wet_road_free"]["stopping_distance"] >= baseline * 1.2)
        check("payload_at_least_15pct_longer", results["payload_free"]["stopping_distance"] >= baseline * 1.15)
        check("weak_brake_at_least_15pct_longer", results["weak_brake_free"]["stopping_distance"] >= baseline * 1.15)
        check("wheel_separates", results["wheel_loss_free"]["max_carrier_separation"][1] > .5)
        check("lag_increases_distance", results["actuator_lag_free"]["stopping_distance"] > baseline)
        check("no_solver_warnings", all(not any(r["warnings"]) for r in results.values()))
        if args.full:
            for name in names():
                reference = results[name + "_free"]
                distance = reference["stopping_distance"]
                fine = results[name + "_fine"]["stopping_distance"]
                check(name + "_timestep_under_1pct", distance is not None and fine is not None
                      and abs(fine / distance - 1) < .01)
                check(name + "_collision_timestep", results[name + "_fine_wall"]["collision"]
                      == results[name]["collision"])
                repeats = [results[name + suffix] for suffix in ("_repeat2", "_repeat3")]
                check(name + "_repeatable", all(
                    r["stopping_distance"] is not None and abs(r["stopping_distance"] - distance) < .1
                    and abs(r["max_yaw_degrees"] - reference["max_yaw_degrees"]) < .5 for r in repeats))
    passed = not errors and bool(checks) and all(checks.values())
    report = {"generated_at": datetime.now(timezone.utc).isoformat(), "passed": passed,
              "checks": checks, "errors": errors, "results": results,
              "scope": "synthetic simulator physics; no agent/candidate evaluation"}
    (args.output / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    lines = ["# Simulator validation", "", f"Result: {'PASS' if passed else 'FAIL'}", "",
             "All values below are measured synthetic MuJoCo outcomes.", "",
             "| Scenario | Wall collision | Wall-free stop (m) | Max yaw (deg) |",
             "|---|---|---:|---:|"]
    for name in names():
        if name not in results or name + "_free" not in results:
            continue
        r = results[name + "_free"]
        distance = r["stopping_distance"]
        stop = f"{distance:.2f}" if distance is not None else "censored"
        lines.append(f"| {name} | {results[name]['collision']} | {stop} | {r['max_yaw_degrees']:.2f} |")
    lines += ["", "## Checks", ""]
    lines += [f"- {'PASS' if ok else 'FAIL'}: {name}" for name, ok in checks.items()]
    lines += [f"- ERROR: {name}: {error}" for name, error in errors.items()]
    (args.output / "report.md").write_text("\n".join(lines) + "\n")
    print(f"{'PASS' if passed else 'FAIL'}: {sum(checks.values())}/{len(checks)} checks; {args.output / 'report.md'}")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
