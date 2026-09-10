"""Launch Astra/max and Sol/max concurrently on isolated copies of one scenario.

python -m investigation.platform_pair --platform drone --scenario drone_rotor_loss \
    --output runs/drone-comparison

Each pair owns comparison.json and two sibling child directories. Repeating this
command with a fresh output creates another independent matched experiment.
"""
from __future__ import annotations

import argparse
from contextlib import ExitStack
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
from uuid import uuid4

from .api import Settings, get_profile
from .platform_run import PLATFORMS, ROOT, _validate_options
from .runner import timestamp

PAIR_PROFILES = (("astra", "astra-max"), ("sol", "sol-max"))
TERMINAL_STATUSES = {"completed", "failed", "interrupted"}
COMPARISON_ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9_-]{0,119}\Z")


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_name(f".{path.name}.{uuid4().hex}.tmp")
    try:
        temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def _read(path):
    try:
        value = json.loads(Path(path).read_text())
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError):
        return {}


def child_command(output, *, platform, scenario, label, profile, max_api_requests=16,
                  max_seconds=1800, no_frames=False):
    output = Path(output).resolve()
    command = [sys.executable, "-m", "investigation.platform_run", "--platform", platform,
               "--scenario", scenario, "--profile", profile, "--comparison-id", output.name,
               "--output", str(output.parent / f"{output.name}-{label}"),
               "--max-api-requests", str(max_api_requests), "--max-seconds", str(max_seconds)]
    if no_frames:
        command.append("--no-frames")
    return command


def _sha(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()


def _file_sha(path):
    try:
        return hashlib.sha256(path.read_bytes()).hexdigest()
    except OSError:
        return None


def _record(output, entry):
    directory = output.parent / entry["run_id"]
    metadata = _read(directory / "metadata.json")
    evaluation = _read(directory / "evaluation/result.json")
    events = []
    try:
        for line in (directory / "events.jsonl").read_text().splitlines():
            try:
                events.append(json.loads(line))
            except ValueError:
                pass  # A currently writing final line is not an event yet.
    except OSError:
        pass
    budget = metadata.get("budgets", {}).get("tool_calls", {})
    entry.update(status=metadata.get("status", entry["status"]),
                 metrics={"duration_s": metadata.get("duration_s"),
                          "api_requests": metadata.get("api_requests"),
                          "tool_calls": budget.get("used"),
                          "failed_tool_calls": sum(event.get("type") == "tool_result" and
                              event.get("result", {}).get("ok") is False for event in events) if events else None,
                          "usage": metadata.get("usage", {}),
                          "estimated_cost_usd": metadata.get("estimated_cost_usd"),
                          "cost_note": "No billing estimate recorded; token counts are available.",
                          "agent_submitted": metadata.get("agent_submitted"),
                          "stop_reason": metadata.get("stop_reason"),
                          "verification_status": metadata.get("verification_status"),
                          "aggregate": evaluation.get("aggregate", {}),
                          "goal": evaluation.get("goal"),
                          "action_summary": evaluation.get("action_summary", metadata.get("action_summary", {})),
                          "cases": [{key: case.get(key) for key in ("probe", "duration_s", "before_difference",
                                     "after_difference", "prediction_difference")} for case in evaluation.get("cases", [])]})
    return metadata, evaluation


def validate_comparison(output, manifest):
    """Recheck actual recorded inputs, protocol identities and frozen outcomes."""
    output = Path(output)
    records = [_record(output, entry) for entry in manifest["runs"]]
    a, b = (record[0] for record in records)
    ea, eb = (record[1] for record in records)
    checks = {"both_processes_completed": all(entry.get("exit_code") == 0 and meta.get("status") == "completed"
                    for entry, (meta, _) in zip(manifest["runs"], records)),
              "isolated_runs": len({entry["run_id"] for entry in manifest["runs"]}) == 2,
              "model_efforts_confirmed": all(meta.get("model_effort_confirmed") is True and
                    meta.get("model") == get_profile(entry["profile"]).model and
                    meta.get("reasoning_effort") == get_profile(entry["profile"]).reasoning_effort
                    for entry, (meta, _) in zip(manifest["runs"], records)),
              "protocol_manifests_verified": all(isinstance(meta.get("protocol_manifest"), dict) and
                    _sha(meta["protocol_manifest"]) == meta.get("protocol_fingerprint") for meta in (a, b)),
              "protocol_fingerprints_match": bool(a.get("protocol_fingerprint")) and
                    a.get("protocol_fingerprint") == b.get("protocol_fingerprint"),
              "predeclared_criteria_match": bool(ea.get("predeclared_criteria")) and
                    ea.get("predeclared_criteria") == eb.get("predeclared_criteria"),
              "verification_cases_match": bool(ea.get("cases")) and
                    [(c.get("probe"), c.get("duration_s")) for c in ea.get("cases", [])] ==
                    [(c.get("probe"), c.get("duration_s")) for c in eb.get("cases", [])],
              "initial_conditions_match": bool(ea.get("cases")) and
                    [c.get("before_difference") for c in ea.get("cases", [])] ==
                    [c.get("before_difference") for c in eb.get("cases", [])]}
    for key, relative in (("system_prompt_sha256", "prompts/system.md"),
                          ("task_prompt_sha256", "prompts/task.md")):
        hashes = [_file_sha(output.parent / entry["run_id"] / relative) for entry in manifest["runs"]]
        checks[key.replace("_sha256", "s_verified_and_match")] = bool(hashes[0]) and hashes[0] == hashes[1] and all(
            digest == meta.get(key) for digest, (meta, _) in zip(hashes, records))
    for key, relative in (("initial_evidence_sha256", "prompts/initial_evidence.json"),
                          ("tool_schema_sha256", "prompts/tools.json")):
        hashes = []
        for entry in manifest["runs"]:
            try:
                hashes.append(_sha(json.loads((output.parent / entry["run_id"] / relative).read_text())))
            except (OSError, ValueError):
                hashes.append(None)
        checks[key.replace("_sha256", "_verified_and_match")] = bool(hashes[0]) and hashes[0] == hashes[1] and all(
            digest == meta.get(key) for digest, (meta, _) in zip(hashes, records))
    checks["frozen_sources_verified"] = all(
        isinstance(meta.get("frozen_source_file"), str) and
        (output.parent / entry["run_id"] / meta["frozen_source_file"]).resolve().is_relative_to(
            (output.parent / entry["run_id"]).resolve()) and
        bool(meta.get("source_hash")) and meta["source_hash"] == result.get("source_sha256") ==
        _file_sha(output.parent / entry["run_id"] / meta["frozen_source_file"])
        for entry, (meta, result) in zip(manifest["runs"], records))
    manifest.update(validation=checks, comparison_valid=all(checks.values()),
                    warnings=[f"Comparison check failed: {name}." for name, passed in checks.items() if not passed])
    return manifest


def _stop_children(processes):
    for process in processes:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
    deadline = time.monotonic() + 20
    while any(process.poll() is None for process in processes) and time.monotonic() < deadline:
        time.sleep(.1)
    for process in processes:
        if process.poll() is None:
            process.terminate()
    for process in processes:
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)


def run_pair(output, *, platform, scenario=None, max_api_requests=16, max_seconds=1800,
             no_frames=False, launcher=None):
    _validate_options(platform, max_api_requests, max_seconds)
    from .platform_physics import DEFAULTS, SCENARIOS
    scenario = scenario or DEFAULTS[platform]
    if scenario not in SCENARIOS[platform]:
        raise ValueError("Choose a supported scenario for the selected platform")
    output = Path(output).resolve()
    if not COMPARISON_ID.fullmatch(output.name):
        raise ValueError("Use a comparison directory name of at most 120 letters, digits, underscores or hyphens, starting with a letter or digit")
    for label, profile in PAIR_PROFILES:
        if (output.parent / f"{output.name}-{label}").exists():
            raise ValueError("A child run directory already exists; choose a fresh comparison output")
        Settings.load(profile=profile)  # Validate both without making any API call.
    output.mkdir(parents=True, exist_ok=False)
    manifest = {"schema_version": 1, "kind": "platform_parallel_comparison",
                "comparison_id": output.name, "platform": platform, "scenario": scenario,
                "status": "launching", "supervisor_pid": os.getpid(), "start_at": timestamp(), "end_at": None,
                "requested_reasoning": "ultra", "api_reasoning_effort": "max",
                "reasoning_note": "User requested Ultra; both APIs are configured at documented highest effort max.",
                "max_api_requests": max_api_requests, "max_seconds": max_seconds,
                "record_frames": not no_frames, "fresh_model_contexts": True,
                "fresh_simulators": True, "parallel": True, "runs_per_profile": 1,
                "randomness": "Deterministic simulator fixtures; no random seed or inherited repairs.",
                "is_model_ranking": False, "comparison_valid": None, "validation": {}, "warnings": [],
                "runs": [{"label": label, "run_id": f"{output.name}-{label}", "profile": profile,
                          "model": get_profile(profile).model,
                          "reasoning_effort": get_profile(profile).reasoning_effort,
                          "status": "pending", "pid": None, "exit_code": None,
                          "metadata_file": f"../{output.name}-{label}/metadata.json", "metrics": {}}
                         for label, profile in PAIR_PROFILES]}
    path = output / "comparison.json"
    atomic_json(path, manifest)
    processes = []
    launcher = launcher or subprocess.Popen
    started = time.monotonic()
    failure = None
    with ExitStack() as stack:
        try:
            for entry in manifest["runs"]:
                stream = stack.enter_context((output / f"{entry['label']}.log").open("w"))
                command = child_command(output, platform=platform, scenario=scenario, label=entry["label"],
                                        profile=entry["profile"], max_api_requests=max_api_requests,
                                        max_seconds=max_seconds, no_frames=no_frames)
                process = launcher(command, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT,
                                   start_new_session=True)
                processes.append(process)
                entry.update(pid=process.pid, status="running", launched_at=timestamp())
                atomic_json(path, manifest)
            manifest["status"] = "running"
            atomic_json(path, manifest)
            # Both children are launched before waiting on either one.
            finished = set()
            while len(finished) < len(processes):
                for index, process in enumerate(processes):
                    code = process.poll()
                    if code is not None and index not in finished:
                        entry = manifest["runs"][index]
                        entry.update(exit_code=code, status="completed" if code == 0 else "failed", end_at=timestamp())
                        _record(output, entry)
                        finished.add(index)
                        atomic_json(path, manifest)
                if time.monotonic() - started > max_seconds + 300:
                    raise TimeoutError("Child runtime exceeded the investigation budget plus verification grace period")
                if len(finished) < len(processes):
                    time.sleep(.25)
        except (KeyboardInterrupt, InterruptedError):
            failure = "interrupted"
            _stop_children(processes)
        except Exception as error:
            failure = "failed"
            manifest["error"] = f"Comparison supervisor stopped: {type(error).__name__}. See preserved child logs."
            _stop_children(processes)
    for index, entry in enumerate(manifest["runs"]):
        if index < len(processes):
            entry["exit_code"] = processes[index].poll()
        entry["status"] = "completed" if entry["exit_code"] == 0 else "failed"
    validate_comparison(output, manifest)
    manifest.update(status=failure or ("completed" if all(e["exit_code"] == 0 for e in manifest["runs"]) else "failed"),
                    end_at=timestamp(), duration_s=time.monotonic() - started)
    atomic_json(path, manifest)
    return manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--platform", choices=PLATFORMS, required=True)
    parser.add_argument("--scenario")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-api-requests", type=int, default=16)
    parser.add_argument("--max-seconds", type=int, default=1800)
    parser.add_argument("--no-frames", action="store_true")
    args = parser.parse_args(argv)
    def interrupted(signum, frame):
        raise InterruptedError("Comparison was stopped")
    previous = signal.signal(signal.SIGTERM, interrupted)
    try:
        result = run_pair(args.output, platform=args.platform, scenario=args.scenario,
                          max_api_requests=args.max_api_requests, max_seconds=args.max_seconds,
                          no_frames=args.no_frames)
    except (ValueError, OSError):
        parser.exit(1, "Comparison setup failed. Check the platform, scenario, new output and API configuration.\n")
    finally:
        signal.signal(signal.SIGTERM, previous)
    print(f"Comparison status: {result['status']}\nManifest: {(args.output / 'comparison.json').resolve()}")
    return 0 if result["status"] == "completed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
