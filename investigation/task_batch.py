"""Launch all declared scenarios concurrently; each runs Astra and Sol in parallel."""
import argparse
from contextlib import ExitStack
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

from .api import Settings
from .platform_pair import PAIR_PROFILES, _read, _stop_children
from .platform_run import ROOT, _validate_options
from .platform_story import atomic_json
from .runner import timestamp
from .tasks import TASKS


def run_batch(output, *, max_api_requests=16, max_seconds=1800, launcher=subprocess.Popen):
    output = Path(output).resolve()
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,100}", output.name):
        raise ValueError("Use a short, safe new batch directory name.")
    _validate_options("drone", max_api_requests, max_seconds)
    for _, profile in PAIR_PROFILES:
        Settings.load(profile=profile)
    comparisons = {task["platform"]: f"{output.name}-{task['platform']}" for task in TASKS.values()}
    reserved_names = [name + suffix for name in comparisons.values()
                      for suffix in ("", *(f"-{label}" for label, _ in PAIR_PROFILES))]
    if any((output.parent / name).exists() for name in reserved_names):
        raise ValueError("A comparison or model directory already exists; choose a fresh batch.")
    output.mkdir(parents=True, exist_ok=False)
    manifest = {"schema_version": 1, "kind": "control_task_batch", "task_kind": "controller_repair",
                "id": output.name, "status": "launching", "start_at": timestamp(), "end_at": None,
                "supervisor_pid": os.getpid(), "comparisons": comparisons, "jobs": {},
                "max_api_requests_per_model": max_api_requests, "max_seconds_per_model": max_seconds,
                "scenario_count": len(TASKS), "model_sessions": len(TASKS) * len(PAIR_PROFILES),
                "all_scenarios_parallel": True}
    path = output / "batch.json"
    atomic_json(path, manifest)
    processes = []
    with ExitStack() as stack:
        try:
            # Start every pair supervisor before polling any of them.
            for scenario, task in TASKS.items():
                platform = task["platform"]
                stream = stack.enter_context((output / f"{platform}.log").open("w"))
                command = [sys.executable, "-m", "investigation.task_pair", "--platform", platform,
                           "--scenario", scenario, "--output", str(output.parent / comparisons[platform]),
                           "--max-api-requests", str(max_api_requests), "--max-seconds", str(max_seconds)]
                process = launcher(command, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT,
                                   start_new_session=True)
                processes.append(process)
                manifest["jobs"][platform] = {"pid": process.pid, "status": "running", "exit_code": None}
            manifest["status"] = "running"
            atomic_json(path, manifest)
            started = time.monotonic()
            while True:
                for platform, process in zip(comparisons, processes):
                    code = process.poll()
                    if code is not None:
                        manifest["jobs"][platform].update(exit_code=code, status="completed" if code == 0 else "failed")
                atomic_json(path, manifest)
                if all(process.poll() is not None for process in processes):
                    break
                if time.monotonic() - started > max_seconds + 420:
                    raise TimeoutError("Batch exceeded bounded verification grace period")
                time.sleep(.5)
        except (KeyboardInterrupt, InterruptedError, TimeoutError):
            _stop_children(processes)
            manifest["status"] = "interrupted"
        except Exception:
            _stop_children(processes)
            manifest["status"] = "failed"
            manifest["error"] = "A comparison process could not launch or finish. Recorded evidence is preserved."
    if manifest["status"] == "running":
        manifest["status"] = "completed" if all(process.poll() == 0 for process in processes) else "failed"
    manifest["comparison_valid"] = all(_read(output.parent / name / "comparison.json").get("comparison_valid") is True
                                       for name in comparisons.values())
    manifest["end_at"] = timestamp()
    atomic_json(path, manifest)
    return manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-api-requests", type=int, default=16)
    parser.add_argument("--max-seconds", type=int, default=1800)
    args = parser.parse_args(argv)
    def stop(signum, frame):
        raise InterruptedError("Batch stopped")
    signal.signal(signal.SIGTERM, stop)
    result = run_batch(args.output, max_api_requests=args.max_api_requests, max_seconds=args.max_seconds)
    print(f"{result['scenario_count']}-scenario batch: {result['status']} — {args.output / 'batch.json'}")
    return 0 if result["status"] == "completed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
