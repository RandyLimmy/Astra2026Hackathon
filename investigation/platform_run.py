"""Run the car, drone or quadruped tool workflow with pinned Astra/extra high."""

from __future__ import annotations

import argparse
import hashlib
from importlib.metadata import version
import json
import math
from pathlib import Path
import sys
import time

from openai import APIError

from .api import Settings, get_profile, request_response
from .runner import EventLog, add_usage, safe_api_error, save_json, timestamp, visible_output


PROFILE = "astra-xhigh"
PLATFORMS = ("car", "drone", "quadruped")
ROOT = Path(__file__).resolve().parents[1]
SYSTEM_PROMPT = Path(__file__).with_name("prompts") / "platform_system.md"
CONTRACT = ROOT / "contracts" / "PLATFORM_MODEL.md"
MAX_ARGUMENT_BYTES = 100_000


def _hash(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def _json(value) -> str:
    return json.dumps(value, allow_nan=False, sort_keys=True, separators=(",", ":"))


def _validate_options(platform, max_api_requests, max_seconds):
    if platform not in PLATFORMS:
        raise ValueError("Choose car, drone or quadruped")
    if (type(max_api_requests) is not int or not 1 <= max_api_requests <= 20
            or type(max_seconds) is not int or not 60 <= max_seconds <= 7200):
        raise ValueError("Use 1–20 API requests and a 60–7200 second time budget")


def _report(run_dir, metadata, result):
    def mapping(value):
        return value if isinstance(value, dict) else {}

    def cell(value):
        if value is None:
            return "not measured"
        return str(value).replace("|", "\\|").replace("\n", " ").replace("\r", " ")

    def metric(value):
        if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value):
            return f"{value:.4f}"
        return "not measured"

    lines = ["# Platform investigation report", "",
             f"Platform: **{metadata['platform']}**. Model: **{metadata['model']}**; "
             f"reasoning: **{metadata['reasoning_effort']}**.", "",
             f"Process status: **{metadata['status']}**. Stop reason: **{metadata['stop_reason']}**.",
             f"Explicit agent submission: **{'yes' if metadata['agent_submitted'] else 'no'}**. "
             f"API requests: **{metadata['api_requests']}/{metadata['max_api_requests']}**.",
             f"Elapsed time including tools and verification: **{metadata['duration_s']:.1f} s**.", "",
             "A completed process is not a claim that maintenance or predictions passed. "
             "Use the recorded verification outcomes below; these synthetic probes do not "
             "establish general safety or reliability.", "",
             "[Recorded events](events.jsonl) · [Run metadata](metadata.json) · "
             "[Exact task](prompts/task.md) · [Tool definitions](prompts/tools.json)", ""]
    if result is not None:
        lines += ["## Frozen result and fresh verification", "",
                  "[Full result and sampled observations](evaluation/result.json)", "",
                  f"Frozen source SHA256: `{result.get('source_sha256', metadata.get('source_hash', 'unavailable'))}`.", ""]
        if metadata.get("frozen_source_file"):
            lines += [f"[Exact frozen Python source]({metadata['frozen_source_file']})", ""]
        cases = result.get("cases", [])
        if cases:
            lines += ["Position RMSE compares overlapping recorded times only. Before/after "
                      "columns compare the specimen with its healthy control; model error "
                      "compares the frozen prediction with the specimen after maintenance.", "",
                      "| Probe | Before RMSE (m) | After RMSE (m) | Observed outcome: before → after | Within envelope: before → after | Frozen model RMSE (m) |",
                      "|---|---:|---:|---|---|---:|"]
            for case in cases:
                before = mapping(case.get("before_difference"))
                after = mapping(case.get("after_difference"))
                prediction = mapping(case.get("prediction_difference"))
                lines += [f"| {cell(case.get('probe'))} | {metric(before.get('position_rmse_m'))} "
                          f"| {metric(after.get('position_rmse_m'))} "
                          f"| {cell(before.get('observed_outcome'))} → {cell(after.get('observed_outcome'))} "
                          f"| {cell(before.get('observed_within_envelope'))} → {cell(after.get('observed_within_envelope'))} "
                          f"| {metric(prediction.get('position_rmse_m'))} |"]
            lines += [""]
        else:
            lines += ["No per-probe verification metrics were recorded; consult the full result.", ""]
        lines += ["## Recorded maintenance", ""]
        maintenance = result.get("maintenance", [])
        if maintenance:
            lines += ["| Action | Target | Receipt status |", "|---|---|---|"]
            for action in maintenance:
                lines += [f"| {cell(action.get('action'))} | {cell(action.get('target'))} "
                          f"| {cell(mapping(action.get('receipt')).get('status'))} |"]
            lines += ["", "Receipts record interventions; verification metrics measure their observed effect.", ""]
        else:
            lines += ["No physical maintenance actions were recorded.", ""]
        lines += ["Scope: frozen model and maintenance sequence, replayed on a fresh synthetic "
                  "specimen. Repeated controlled probes may use maneuvers already investigated; "
                  "they do not by themselves establish generalization or hardware safety.", ""]
    else:
        lines += ["Fresh verification did not complete. Existing tool records are preserved; "
                  "no successful final verification is claimed.", ""]
    lines += ["## Investigator's recorded public notes", "",
              "These are the investigator's stated explanations and brief API-provided summaries, "
              "not independently verified findings.", ""]
    path = run_dir / "events.jsonl"
    if path.is_file():
        for line in path.read_text().splitlines():
            event = json.loads(line)
            if event.get("type") in {"assistant_message", "reasoning_summary"}:
                lines += ["\n".join("> " + part for part in event.get("text", "").splitlines()), ""]
    (run_dir / "report.md").write_text("\n".join(lines), encoding="utf-8")


def run_platform_session(client, broker, run_dir: Path, *, platform,
                         max_api_requests=16, max_seconds=1800, log=None):
    """Run a fresh context with only the broker's public tools; no API fallback.

    A supplied client and broker make the full loop testable without credentials,
    network access or physical simulation. Final verification stays host-side.
    """
    _validate_options(platform, max_api_requests, max_seconds)
    selected = get_profile(PROFILE)
    run_dir = Path(run_dir)
    run_dir.mkdir(parents=True, exist_ok=True)
    if (run_dir / "metadata.json").exists():
        raise ValueError("Use a new investigation directory; existing records cannot be overwritten")
    log = log or EventLog(run_dir / "events.jsonl")
    started = time.monotonic()
    metadata = {"kind": "platform_investigation", "platform": platform,
                "profile": selected.name, "model": selected.model,
                "reasoning_effort": selected.reasoning_effort, "status": "running",
                "start_at": timestamp(), "end_at": None, "api_requests": 0,
                "max_api_requests": max_api_requests, "max_seconds": max_seconds,
                "agent_submitted": False, "stop_reason": None,
                "usage": {key: 0 for key in ("input_tokens", "output_tokens", "total_tokens",
                                             "reasoning_tokens", "cached_input_tokens")}}
    save_json(run_dir / "metadata.json", metadata)
    result = None
    try:
        log("status", message=f"Starting fresh {platform} investigation: {selected.model} / {selected.reasoning_effort}.")
        # Only these broker-projected public values enter the remote context.
        # Never serialize CLI arguments, a physics object's attributes or a preset.
        evidence = broker.initial_evidence()
        schemas = broker.tool_schemas
        system = SYSTEM_PROMPT.read_text()
        contract = CONTRACT.read_text()
        task = (f"Investigate the supplied {platform} system. Diagnose the observed mismatch, "
                "use maintenance when justified, and repair the predictive model if needed. "
                "State evidence and uncertainty separately from verified outcomes.\n\n"
                "## Public model contract\n\n" + contract +
                "\n\n## Public capabilities and initial evidence\n\n" + _json(evidence) +
                f"\n\n## API request budget\n\nThis session allows at most {max_api_requests} API requests "
                "(model responses), separate from the broker's tool and experiment budgets. "
                "Each response, including a tool call, uses one request. Tool results report "
                "the remaining API requests. Call submit_result with diagnosis, evidence and "
                "remaining_uncertainty before requests run out. Otherwise the host finalizes "
                "the current source and maintenance record without claiming that you submitted it. "
                "Start by inspecting the system and model.")
        prompts = run_dir / "prompts"
        prompts.mkdir(exist_ok=False)
        (prompts / "system.md").write_text(system)
        (prompts / "task.md").write_text(task)
        save_json(prompts / "tools.json", schemas)
        save_json(prompts / "initial_evidence.json", evidence)
        caps = broker.budget_status()
        paths = {"investigation/api.py", "investigation/runner.py", "investigation/platform_run.py",
                 "investigation/platform_broker.py", "investigation/platform_physics.py",
                 "candidate/platform_model.py", "contracts/PLATFORM_MODEL.md",
                 "investigation/prompts/platform_system.md"}
        paths.update(str(path.relative_to(ROOT)) for path in (ROOT / "component_worker").glob("*.py"))
        paths.update(f"simulator/platforms/{name}.py" for name in ("__init__", "car_damage", "drone", "quadruped"))
        paths.update(str(path.relative_to(ROOT)) for path in (ROOT / "simulator/assets/platforms").rglob("*.xml"))
        capabilities = evidence.get("capabilities", {})
        manifest = {"schema_version": 1, "platform": platform,
                    "source_sha256": {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
                                      for name in sorted(paths)},
                    "runtime_versions": {"python": ".".join(str(part) for part in sys.version_info[:3]),
                                         **{name: version(name) for name in ("mujoco", "numpy", "openai", "Pillow")}},
                    "system_prompt_sha256": _hash(system), "contract_sha256": _hash(contract),
                    "tool_schema_sha256": _hash(_json(schemas)),
                    "initial_source_sha256": hashlib.sha256(broker.current_source.read_bytes()).hexdigest(),
                    "max_api_requests": max_api_requests, "max_seconds": max_seconds,
                    "max_output_tokens": 8192,
                    "tool_caps": {key: value["limit"] for key, value in caps.items()
                                  if isinstance(value, dict) and "limit" in value},
                    "configuration_caps": {"model_parameters": capabilities.get("model_parameters", {}),
                                           "probe_duration_s": capabilities.get("duration_s", {}),
                                           "tools": {tool["name"]: {
                                               key: value for key, value in tool.get("parameters", {}).get("properties", {}).items()
                                               if key in {"config", "probe", "duration_s"}} for tool in schemas}}}
        metadata.update(system_prompt_sha256=_hash(system), task_prompt_sha256=_hash(task),
                        tool_schema_sha256=manifest["tool_schema_sha256"],
                        contract_sha256=manifest["contract_sha256"],
                        protocol_fingerprint=_hash(_json(manifest)), protocol_manifest=manifest)
        save_json(run_dir / "metadata.json", metadata)
        conversation = [{"role": "system", "content": system}, {"role": "user", "content": task}]
        allowed = {tool["name"] for tool in schemas}
        nudged = False
        while metadata["api_requests"] < max_api_requests and time.monotonic() - started < max_seconds:
            metadata["api_requests"] += 1
            save_json(run_dir / "metadata.json", metadata)
            log("status", message=f"API request {metadata['api_requests']}/{max_api_requests}: "
                f"{selected.model} / {selected.reasoning_effort}.")
            response = request_response(client, conversation, schemas, profile=PROFILE)
            add_usage(metadata["usage"], response)
            effort = response.reasoning.effort if response.reasoning else None
            log("api_response", response_id=response.id, model=response.model, reasoning_effort=effort,
                status=response.status, usage=response.usage.model_dump() if response.usage else None)
            if response.model != selected.model or effort != selected.reasoning_effort:
                raise RuntimeError("The API did not confirm the pinned model/reasoning pair")
            visible_output(response, log)
            if response.status != "completed":
                metadata["stop_reason"] = "api_response_incomplete"
                break
            # Encrypted continuity is retained in memory, never written to logs.
            conversation.extend(item.model_dump(exclude_none=True) for item in response.output)
            calls = [item for item in response.output if item.type == "function_call"]
            for call in calls:
                if time.monotonic() - started >= max_seconds:
                    metadata["stop_reason"] = "time_budget"
                    break
                raw = call.arguments
                bounded = isinstance(raw, str) and len(raw.encode("utf-8")) <= MAX_ARGUMENT_BYTES
                try:
                    if not bounded:
                        raise ValueError("oversized arguments")
                    arguments = json.loads(raw)
                    if not isinstance(arguments, dict) or call.name not in allowed:
                        raise ValueError("unsupported tool arguments")
                except (ValueError, TypeError):
                    arguments = {}
                    tool_result = {"ok": False, "error": {"code": "request_rejected",
                                   "message": "Invalid, oversized or unsupported tool request."}}
                    log("tool_call", name=call.name, arguments=arguments,
                        arguments_json=raw if bounded else "[omitted: invalid or oversized arguments]")
                else:
                    log("tool_call", name=call.name, arguments=arguments, arguments_json=raw)
                    tool_result = broker.dispatch(call.name, arguments)
                remaining = max_api_requests - metadata["api_requests"]
                tool_result = {**tool_result, "api_request_budget": {
                    "used": metadata["api_requests"], "remaining": remaining}}
                if remaining <= 2:
                    tool_result["submission_reminder"] = (
                        "Submission is recorded; the host will finalize and verify it." if broker.submitted else
                        f"{remaining} API requests remain. Call submit_result when ready; "
                        "the host finalizes without an agent submission if the budget runs out.")
                log("tool_result", name=call.name, result=tool_result)
                conversation.append({"type": "function_call_output", "call_id": call.call_id,
                                     "output": _json(tool_result)})
                if broker.submitted:
                    metadata["stop_reason"] = "agent_submission"
                    break
            if broker.submitted or metadata["stop_reason"] is not None:
                break
            if not calls:
                if nudged:
                    metadata["stop_reason"] = "agent_finished_without_submission"
                    break
                reminder = ("Use the available tools to execute the investigation, model edits or maintenance. "
                            "A textual proposal alone does not change either system. If finished, call "
                            "submit_result with diagnosis, evidence and remaining_uncertainty.")
                conversation.append({"role": "user", "content": reminder})
                log("host_message", text=reminder)
                nudged = True
        metadata["stop_reason"] = metadata["stop_reason"] or (
            "time_budget" if time.monotonic() - started >= max_seconds else "api_request_budget")
        metadata["status"] = "finalizing"
    except APIError as error:
        log("error", message=safe_api_error(error))
        metadata.update(status="api_error", stop_reason="api_error")
    except KeyboardInterrupt:
        log("error", message="Investigation interrupted; completed tool records are preserved.")
        metadata.update(status="interrupted", stop_reason="interrupted")
    except Exception as error:
        log("error", message=f"Investigation stopped: {type(error).__name__}. Completed tool records are preserved.")
        metadata.update(status="error", stop_reason="host_error")
    metadata["agent_submitted"] = bool(broker.submitted)
    save_json(run_dir / "metadata.json", metadata)
    try:
        log("status", message="Freezing the source and maintenance record for fresh verification.")
        finalized = broker.finalize(metadata["stop_reason"])
        if not isinstance(finalized, dict):
            raise ValueError("Final verification must return a result object")
        _json(finalized)
        (run_dir / "evaluation").mkdir(exist_ok=True)
        save_json(run_dir / "evaluation" / "result.json", finalized)
        result = finalized
        metadata["source_hash"] = hashlib.sha256(broker.current_source.read_bytes()).hexdigest()
        source_path = broker.current_source.resolve()
        if source_path.is_relative_to(run_dir.resolve()):
            metadata["frozen_source_file"] = source_path.relative_to(run_dir.resolve()).as_posix()
        metadata["verification_status"] = "completed"
        if metadata["status"] == "finalizing":
            metadata["status"] = "completed"
        log("evaluation", result=result)
    except Exception as error:
        log("error", message=f"Fresh verification could not complete: {type(error).__name__}. Existing records are preserved.")
        metadata["verification_status"] = "error"
        if metadata["status"] == "finalizing":
            metadata["status"] = "evaluation_error"
    metadata.update(end_at=timestamp(), duration_s=time.monotonic() - started,
                    budgets=broker.budget_status())
    save_json(run_dir / "metadata.json", metadata)
    _report(run_dir, metadata, result)
    return metadata


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--platform", required=True, choices=PLATFORMS)
    parser.add_argument("--scenario", help="host-only operator preset; never included in model prompts")
    parser.add_argument("--output", required=True, type=Path, help="new session directory")
    parser.add_argument("--max-api-requests", type=int, default=16)
    parser.add_argument("--max-seconds", type=int, default=1800)
    parser.add_argument("--no-frames", action="store_true", help="skip host frame recording")
    args = parser.parse_args(argv)
    try:
        _validate_options(args.platform, args.max_api_requests, args.max_seconds)
        if args.output.exists():
            raise ValueError("Output already exists; choose a new investigation directory")
        settings = Settings.load(profile=PROFILE)
        from .platform_broker import PlatformBroker
        from .platform_physics import PlatformPhysics

        args.output.mkdir(parents=True, exist_ok=False)
        # Keep operator choices reproducible without including them in the
        # model's capabilities, prompts or tool results. Null means the default
        # preset in the recorded adapter revision.
        save_json(args.output / "host_setup.json", {
            "platform": args.platform, "requested_scenario": args.scenario,
            "record_frames": not args.no_frames,
            "max_api_requests": args.max_api_requests, "max_seconds": args.max_seconds})
        physics = PlatformPhysics(args.platform, args.output / "physics",
                                  scenario=args.scenario, record_frames=not args.no_frames)
        broker = PlatformBroker(args.output / "broker", physics)
        with settings.client() as client:
            metadata = run_platform_session(client, broker, args.output, platform=args.platform,
                                            max_api_requests=args.max_api_requests,
                                            max_seconds=args.max_seconds)
        print(f"Process status: {metadata['status']}\nReport: {(args.output / 'report.md').resolve()}")
        return 0 if metadata["status"] == "completed" else 1
    except APIError as error:
        parser.exit(1, safe_api_error(error) + "\n")
    except (ValueError, OSError) as error:
        parser.exit(1, f"Setup failed: {type(error).__name__}. Check the platform, preset, output and API configuration.\n")


if __name__ == "__main__":
    raise SystemExit(main())
