"""Validate healthy/fault probes, determinism and reduced-timestep stability."""
import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
from datetime import datetime, timezone
import json
from pathlib import Path

import numpy as np

from . import catalog
from .operator import checked_step


def rollout(preset, *, healthy=False, fine=False, probe=None):
    values = {} if probe is None else {"probe": probe}
    sim = catalog.create(preset, values, healthy=healthy)
    if fine:
        module = catalog.module_for(preset)
        sim = module.Simulation(replace(sim.config, timestep=sim.config.timestep / 2))
    trajectory: list[list[float]] = []
    clocks: list[float] = []
    for _ in range(round(sim.config.duration / sim.config.timestep) + 2):
        observation = sim.observe()
        if not clocks or sim.elapsed - clocks[-1] >= .02 - 1e-10 or sim.finished:
            clocks.append(float(sim.elapsed))
            trajectory.append(observation["position"])
        if sim.finished:
            break
        checked_step(sim)
    if not sim.finished:
        raise RuntimeError("scenario failed to finish within its configured horizon")
    return {"summary": sim.summary(), "time": clocks, "position": trajectory,
            "diagnostics": sim.diagnostics(), "warnings": sim.data.warning.number.tolist()}


def _error(reference, actual, *, before=None):
    ta = np.asarray(actual["time"])
    tr = np.asarray(reference["time"])
    keep = (ta >= tr[0]) & (ta <= tr[-1])
    if before is not None:
        keep &= ta < before - .025
    if not keep.any():
        return None
    positions = np.asarray(reference["position"])
    interpolated = np.column_stack([np.interp(ta[keep], tr, positions[:, i]) for i in range(3)])
    errors = np.linalg.norm(np.asarray(actual["position"])[keep] - interpolated, axis=1)
    return {"rmse_m": float(np.sqrt(np.mean(errors ** 2))), "max_m": float(errors.max())}


def evaluate(preset):
    nominal = rollout(preset, healthy=True)
    actual = rollout(preset)
    repeat = rollout(preset)
    fine = rollout(preset, fine=True)
    config = catalog.config_dict(catalog.create(preset))
    checks = {
        "nominal_probe_safe": bool(nominal["summary"]["public"]["safe"]),
        "no_solver_warnings": not any(any(r["warnings"]) for r in (nominal, actual, repeat, fine)),
        "deterministic_repeat": actual["summary"]["public"] == repeat["summary"]["public"],
    }
    same_times = actual["time"] == repeat["time"]
    checks["deterministic_trajectory"] = same_times and bool(
        np.array_equal(actual["position"], repeat["position"]))
    changed = _error(nominal, actual)
    if config["fault"] in ("healthy", "none"):
        checks["healthy_matches_nominal"] = changed is not None and changed["max_m"] < 1e-8
    else:
        checks["fault_changes_observed_motion"] = changed is not None and changed["max_m"] > .001
    # Complex contact falls can be chaotic; this checks the coarse observable
    # outcome, not an unsupported claim of identical post-impact trajectories.
    checks["fine_step_safety_agrees"] = (actual["summary"]["public"]["safe"]
                                         == fine["summary"]["public"]["safe"])
    return preset, {"checks": checks, "passed": all(checks.values()), "config": config,
                    "nominal": nominal["summary"], "actual": actual["summary"],
                    "fine": fine["summary"], "nominal_motion_error": changed,
                    "timestep_motion_error": _error(actual, fine)}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("runs/platform-validation"))
    parser.add_argument("--workers", type=int, default=3)
    parser.add_argument("--scenario", action="append", help="limit to one or more named platform profiles")
    args = parser.parse_args(argv)
    if args.workers < 1:
        parser.error("workers must be positive")
    # The independently maintained warehouse lab has its own validation suite.
    presets = args.scenario or [name for name in catalog.entries() if not name.startswith("warehouse_")]
    results, errors = {}, {}
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        jobs = {name: pool.submit(evaluate, name) for name in presets}
        for name, job in jobs.items():
            try:
                _, result = job.result()
                results[name] = result
                print(f"{'PASS' if result['passed'] else 'FAIL'} {name}: "
                      f"{result['actual']['public']}", flush=True)
            except Exception as exc:
                errors[name] = f"{type(exc).__name__}: {exc}"
                print(f"ERROR {name}: {exc}", flush=True)
    passed = not errors and bool(results) and all(r["passed"] for r in results.values())
    report = {"passed": passed, "generated_at": datetime.now(timezone.utc).isoformat(),
              "results": results, "errors": errors,
              "scope": "Synthetic physical probes only; no autonomous model-repair claim"}
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    lines = ["# Platform scenario validation", "", f"Result: {'PASS' if passed else 'FAIL'}", "",
             "Nominal and changed worlds use the same controller and probe.", "",
             "| Profile | Healthy safe | Fault-world safe | Maximum nominal position mismatch (m) | Checks |",
             "|---|---|---|---:|---|"]
    for name, result in results.items():
        error = result["nominal_motion_error"]
        error_text = f"{error['max_m']:.4f}" if error else "n/a"
        lines.append(f"| {name} | {result['nominal']['public']['safe']} | "
                     f"{result['actual']['public']['safe']} | {error_text} | "
                     f"{sum(result['checks'].values())}/{len(result['checks'])} |")
    lines += ["", "Safety labels concern only the measured synthetic probe. Finer-timestep checks",
              "compare outcome classifications; post-fall trajectories need not be identical.", ""]
    for name, result in results.items():
        for check, ok in result["checks"].items():
            if not ok:
                lines.append(f"- FAIL {name}: {check}")
    lines += [f"- ERROR {name}: {error}" for name, error in errors.items()]
    (args.output / "report.md").write_text("\n".join(lines) + "\n")
    print(f"{'PASS' if passed else 'FAIL'}: {len(results)} profiles; report {args.output / 'report.md'}")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
