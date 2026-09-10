"""Audited, bounded Responses API investigation and post-submission evaluation."""

from datetime import datetime, timezone
import hashlib
from importlib.metadata import version
import json
from pathlib import Path
import sys
import time

from openai import APIError

from .api import DEFAULT_PROFILE, get_profile, request_response


ROOT = Path(__file__).resolve().parents[1]
PROMPTS = Path(__file__).with_name("prompts")


def protocol_identity(max_api_requests=12, max_seconds=1800):
    """Hash the shared experiment protocol, excluding selected model and run ids.

    Expanded evidence contains opaque ids and elapsed times, so hash its source
    templates and generator code instead. This is a host audit, not agent input.
    """
    from .broker import LIMITS, TOOL_SCHEMAS

    paths = [
        "investigation/prompts/system.md", "investigation/prompts/task.md",
        "contracts/WHEEL_ACTUATOR.md", "candidate/wheel_actuator.py",
        "investigation/api.py", "investigation/broker.py", "investigation/patching.py", "investigation/runner.py",
        "investigation/physics.py", "investigation/evaluation.py",
        "component_worker/client.py", "component_worker/runtime.py",
        "simulator/config.py", "simulator/model.py", "simulator/runner.py",
        "simulator/private/thermal.py",
    ]
    paths.extend(str(path.relative_to(ROOT)) for path in sorted((ROOT / "simulator/assets").glob("*.xml")))
    manifest = {
        "schema_version": 1,
        "source_sha256": {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest() for name in paths},
        "tool_schema_sha256": hashlib.sha256(json.dumps(TOOL_SCHEMAS, sort_keys=True).encode()).hexdigest(),
        "caps": {"max_api_requests": max_api_requests, "max_seconds": max_seconds,
                 "max_tool_calls": 30, "tool_attempts": dict(LIMITS),
                 "max_output_tokens": 8192, "debrief_max_output_tokens": 2048},
        "runtime_versions": {"python": ".".join(str(part) for part in sys.version_info[:3]),
                             "mujoco": version("mujoco"), "numpy": version("numpy"), "openai": version("openai")},
    }
    fingerprint = hashlib.sha256(json.dumps(manifest, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
    return {"fingerprint": fingerprint, "manifest": manifest}


def timestamp():
    return datetime.now(timezone.utc).isoformat()


def save_json(path: Path, data):
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")


class EventLog:
    def __init__(self, path: Path):
        self.path = path

    def __call__(self, kind, **values):
        event = {"timestamp": timestamp(), "type": kind, **values}
        with self.path.open("a") as stream:
            stream.write(json.dumps(event, allow_nan=False) + "\n")
        if kind == "tool_call":
            arguments = values.get("arguments", {})
            note = arguments.get("hypothesis") or arguments.get("rationale") or ""
            print(f"Investigator tool: {values['name']} · {str(note)[:300]}", flush=True)
        elif kind in {"assistant_message", "reasoning_summary"}:
            label = "Investigator" if kind == "assistant_message" else "Investigator summary"
            print(f"{label}: {values.get('text', '')[:1200]}", flush=True)
        elif kind in {"status", "error"}:
            print(values.get("message", ""), flush=True)


def safe_api_error(error):
    """Do not echo provider exception messages, which may contain credentials."""
    status = getattr(error, "status_code", None)
    suffix = f" (HTTP {status})" if isinstance(status, int) else ""
    return f"OpenAI request failed: {type(error).__name__}{suffix}. No model fallback was used."


def visible_output(response, log):
    """Log public text/summary items; retain encrypted continuity only in memory."""
    for item in response.output:
        if item.type == "message":
            for part in item.content:
                if part.type == "output_text":
                    log("assistant_message", text=part.text)
                elif part.type == "refusal":
                    log("assistant_message", text=part.refusal)
        elif item.type == "reasoning":
            for part in item.summary:
                if part.type == "summary_text":
                    log("reasoning_summary", text=part.text)


def add_usage(total, response):
    if response.usage is None:
        return
    usage = response.usage.model_dump()
    for key in ("input_tokens", "output_tokens", "total_tokens"):
        total[key] += usage.get(key) or 0
    total["reasoning_tokens"] += (usage.get("output_tokens_details") or {}).get("reasoning_tokens") or 0
    total["cached_input_tokens"] += (usage.get("input_tokens_details") or {}).get("cached_tokens") or 0


def run_session(client, broker, run_dir: Path, *, max_api_requests=12, max_seconds=1800,
                log=None, evaluate=None, profile=DEFAULT_PROFILE):
    """Run a fresh API context. The broker is the only model-accessible capability.

    No shell, browser, filesystem, built-in code execution, or other connectors
    are supplied to the remote model. Candidate code uses the OS-isolated worker.
    """
    from .broker import TOOL_SCHEMAS

    if not 1 <= max_api_requests <= 20 or not 60 <= max_seconds <= 7200:
        raise ValueError("Invalid pilot request/time budget")
    selected = get_profile(profile)
    protocol = protocol_identity(max_api_requests, max_seconds)
    run_dir = Path(run_dir)
    run_dir.mkdir(parents=True, exist_ok=True)
    log = log or EventLog(run_dir / "events.jsonl")
    metadata = {"profile": selected.name, "model": selected.model, "reasoning_effort": selected.reasoning_effort,
                "protocol_fingerprint": protocol["fingerprint"], "protocol_manifest": protocol["manifest"],
                "status": "running", "start_at": timestamp(), "end_at": None,
                "api_requests": 0, "agent_submitted": False, "freeze_reason": None,
                "max_api_requests": max_api_requests, "max_seconds": max_seconds,
                "usage": {key: 0 for key in ("input_tokens", "output_tokens", "total_tokens",
                                             "reasoning_tokens", "cached_input_tokens")}}
    save_json(run_dir / "metadata.json", metadata)
    started = time.monotonic()
    log("status", message=f"Starting fresh {selected.model} investigation with {selected.reasoning_effort} reasoning.")
    try:
        log("status", message="Preparing initial measured evidence and baseline predictions...")
        evidence = broker.initial_evidence()
        system = (PROMPTS / "system.md").read_text()
        task = (PROMPTS / "task.md").read_text()
        task += "\n\n## Component interface\n\n" + (ROOT / "contracts" / "WHEEL_ACTUATOR.md").read_text()
        task += "\n\n## Initial evidence\n\n" + json.dumps(evidence, allow_nan=False, separators=(",", ":"))
        task += (f"\n\n## API request budget\n\nThis session allows at most {max_api_requests} API requests "
                 "(model responses), separate from the broker's tool-call and experiment budgets. "
                 "Each response uses one request, including responses containing tool calls. "
                 "Tool results report used and remaining API requests. Plan to call submit_prediction "
                 "before requests run out; otherwise the host freezes the current source and records "
                 "that it was not submitted by you. A post-evaluation debrief, if requested, shares this budget.")
        task += "\n\nInspect the current source and begin the investigation."
        (run_dir / "prompts").mkdir(exist_ok=True)
        (run_dir / "prompts" / "system.md").write_text(system)
        (run_dir / "prompts" / "task.md").write_text(task)
        save_json(run_dir / "prompts" / "tools.json", TOOL_SCHEMAS)
        metadata["system_prompt_sha256"] = hashlib.sha256(system.encode()).hexdigest()
        metadata["task_prompt_sha256"] = hashlib.sha256(task.encode()).hexdigest()
        conversation = [{"role": "system", "content": system}, {"role": "user", "content": task}]
        nudged = False
        while metadata["api_requests"] < max_api_requests and time.monotonic() - started < max_seconds:
            metadata["api_requests"] += 1
            save_json(run_dir / "metadata.json", metadata)
            log("status", message=f"API request {metadata['api_requests']}/{max_api_requests}: {selected.model} / {selected.reasoning_effort}.")
            response = request_response(client, conversation, TOOL_SCHEMAS, profile=selected.name)
            add_usage(metadata["usage"], response)
            log("api_response", response_id=response.id, model=response.model,
                reasoning_effort=response.reasoning.effort if response.reasoning else None,
                usage=response.usage.model_dump() if response.usage else None, status=response.status)
            if response.model != selected.model or not response.reasoning or response.reasoning.effort != selected.reasoning_effort:
                raise RuntimeError("The API did not confirm the requested model/reasoning profile")
            visible_output(response, log)
            if response.status != "completed":
                metadata["freeze_reason"] = "api_response_incomplete"
                log("error", message="The API response was incomplete; incomplete tool calls were not executed.")
                break
            # Preserve all response items, including encrypted reasoning continuity,
            # exactly as the Responses API documents for store=False sessions.
            conversation.extend(item.model_dump(exclude_none=True) for item in response.output)
            calls = [item for item in response.output if item.type == "function_call"]
            for call in calls:
                try:
                    if len(call.arguments) > 100_000:
                        raise ValueError("arguments too large")
                    arguments = json.loads(call.arguments)
                    if not isinstance(arguments, dict):
                        raise ValueError("arguments must be an object")
                except (ValueError, TypeError):
                    arguments = {}
                    result = {"error": "Invalid or oversized tool arguments"}
                    log("tool_call", name=call.name, arguments=arguments)
                else:
                    log("tool_call", name=call.name, arguments=arguments)
                    result = broker.dispatch(call.name, arguments)
                remaining = max_api_requests - metadata["api_requests"]
                result = {**result, "api_request_budget": {"used": metadata["api_requests"], "remaining": remaining}}
                if remaining <= 2:
                    if broker.frozen_source is not None:
                        reminder = "The source is frozen; no further submission is needed."
                    elif remaining:
                        reminder = (f"Only {remaining} API request{'s' if remaining != 1 else ''} remain. "
                                    "Call submit_prediction when ready to freeze the current source.")
                    else:
                        reminder = "No API requests remain; the host will freeze the source without an accepted submit_prediction."
                    result["submission_reminder"] = reminder
                log("tool_result", name=call.name, result=result)
                conversation.append({"type": "function_call_output", "call_id": call.call_id,
                                     "output": json.dumps(result, allow_nan=False, separators=(",", ":"))})
                if broker.frozen_source is not None:
                    metadata["agent_submitted"] = call.name == "submit_prediction"
                    metadata["freeze_reason"] = "agent_submission" if metadata["agent_submitted"] else "broker_freeze"
                    break
            if broker.frozen_source is not None:
                break
            if not calls:
                if nudged:
                    metadata["freeze_reason"] = "agent_finished_without_submission"
                    break
                conversation.append({"role": "user", "content":
                    "Use the available tools to execute your proposed investigation or repair. "
                    "If you are finished, call submit_prediction to freeze the current component. "
                    "A textual proposal alone does not change the executable model."})
                nudged = True
        if broker.frozen_source is None:
            metadata["freeze_reason"] = metadata["freeze_reason"] or "request_or_time_budget"
            broker.freeze()
        metadata["status"] = "evaluating"
    except APIError as error:
        log("error", message=safe_api_error(error))
        metadata["status"] = "api_error"
        metadata["freeze_reason"] = "api_error"
    except KeyboardInterrupt:
        log("error", message="Run interrupted; completed actions and current source are preserved.")
        metadata["status"] = "interrupted"
        metadata["freeze_reason"] = "interrupted"
    except Exception as error:
        # Builder exceptions can contain host internals. Preserve the type, not
        # arbitrary text or SDK objects; tool-specific neutral errors live in logs.
        log("error", message=f"Investigation stopped: {type(error).__name__}. See completed tool events.")
        metadata["status"] = "error"
        metadata["freeze_reason"] = "host_error"
    if metadata["status"] == "evaluating":
        source = broker.frozen_source
        metadata["source_hash"] = hashlib.sha256(source.read_bytes()).hexdigest()
        metadata["frozen_source_file"] = str(source.resolve().relative_to(run_dir.resolve()))
        save_json(run_dir / "metadata.json", metadata)
        if evaluate is not None:
            log("status", message="Source frozen. Predicting reserved cases before revealing their measured outcomes...")
            try:
                result = evaluate(source, run_dir / "evaluation")
                log("evaluation", result=result)
                metadata["status"] = "completed"
            except Exception as error:
                log("error", message=f"Reserved evaluation failed: {type(error).__name__}. Existing predictions are preserved.")
                metadata["status"] = "evaluation_error"
            if metadata["status"] == "completed" and metadata["api_requests"] < max_api_requests and time.monotonic() - started < max_seconds:
                # The source is locked. Reveal only observed metrics and public
                # controls for a human-readable debrief, with tools disabled.
                try:
                    feedback = {"aggregate": result.get("aggregate", {}), "cases": [
                        {"id": case["case_id"], "config": case["config"],
                         "reference": case["reference"]["summary"],
                         "prediction": case["candidate"]["summary"],
                         "original_error_m": case.get("original_error_m"),
                         "candidate_error_m": case.get("candidate_error_m"),
                         "previously_observed": case.get("previously_observed", False)}
                        for case in result.get("cases", [])]}
                    conversation.append({"role": "user", "content":
                        "The source is frozen and evaluation is complete. Here are the observed results: "
                        + json.dumps(feedback, allow_nan=False) +
                        "\nGive a concise human-readable debrief: what you changed and why, what was "
                        "correct or incorrect, and the remaining uncertainty. This small synthetic "
                        "test does not establish general physical validity or research novelty. No further edits or tools."})
                    metadata["api_requests"] += 1
                    log("status", message="Asking the investigator for a debrief of the measured results; tools are disabled.")
                    debrief = request_response(client, conversation, TOOL_SCHEMAS, max_output_tokens=2048,
                                               final=True, profile=selected.name)
                    add_usage(metadata["usage"], debrief)
                    log("api_response", response_id=debrief.id, model=debrief.model,
                        reasoning_effort=debrief.reasoning.effort if debrief.reasoning else None,
                        usage=debrief.usage.model_dump() if debrief.usage else None, status=debrief.status)
                    if debrief.model != selected.model or not debrief.reasoning or debrief.reasoning.effort != selected.reasoning_effort:
                        raise RuntimeError("Unexpected debrief model configuration")
                    visible_output(debrief, log)
                    metadata["debrief_status"] = debrief.status
                except APIError as error:
                    log("error", message=safe_api_error(error))
                    metadata["debrief_status"] = "api_error"
                except Exception:
                    log("error", message="The debrief could not complete; measured evaluation results are preserved.")
                    metadata["debrief_status"] = "error"
        else:
            metadata["status"] = "submitted"
    metadata["end_at"] = timestamp()
    metadata["duration_s"] = time.monotonic() - started
    metadata["budgets"] = broker.budget_status()
    save_json(run_dir / "metadata.json", metadata)
    return metadata
