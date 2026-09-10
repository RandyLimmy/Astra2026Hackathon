"""Build a readable report from recorded investigation evidence, without API calls.

Inputs remain unchanged. Only report.md and, when distance data exist,
evaluation.png are written. Visible assistant messages and API-provided brief
reasoning summaries are attributed as records, not treated as verified findings.
"""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import difflib
import json
import math
from pathlib import Path


def _read_json(path: Path) -> tuple[dict | None, str | None]:
    if not path.is_file():
        return None, None
    try:
        value = json.loads(path.read_text())
        if not isinstance(value, dict):
            raise ValueError("expected a JSON object")
        return value, None
    except (OSError, ValueError) as exc:
        return None, f"Could not read {path.name}: {type(exc).__name__}."


def _read_events(path: Path) -> tuple[list[dict], list[str]]:
    if not path.is_file():
        return [], []
    events, warnings = [], []
    try:
        lines = path.read_text().splitlines()
    except OSError as exc:
        return [], [f"Could not read events.jsonl: {type(exc).__name__}."]
    for number, line in enumerate(lines, 1):
        if not line.strip():
            continue
        try:
            event = json.loads(line)
            if not isinstance(event, dict):
                raise ValueError("expected an event object")
            events.append(event)
        except ValueError:
            warnings.append(f"Skipped malformed events.jsonl line {number}; earlier records are retained.")
    return events, warnings


def _number(value) -> float | None:
    if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value):
        return float(value)
    return None


def _cell(value) -> str:
    return str(value).replace("|", "\\|").replace("\n", " ").replace("\r", " ")


def _metric(value) -> str:
    number = _number(value)
    return f"{number:.3f}" if number is not None else "—"


def _summary(case: dict, name: str) -> dict:
    value = case.get(name)
    if not isinstance(value, dict):
        return {}
    summary = value.get("summary", value)
    return summary if isinstance(summary, dict) else {}


def _distance_cell(summary: dict) -> str:
    value = _number(summary.get("stopping_distance"))
    if value is not None:
        return f"{value:.3f}"
    if summary.get("censored"):
        return "— (censored)"
    if summary.get("stopped"):
        return "— (no braking-distance measurement)"
    return "— (not recorded)"


def _compact(value, limit: int = 1000) -> str:
    if isinstance(value, dict):
        value = dict(value)
        for key in ("source", "code", "diff", "patch"):
            if isinstance(value.get(key), str) and len(value[key]) > 160:
                value[key] = f"[{len(value[key])} characters; see raw log and saved source]"
        for key in ("observations", "trajectory", "frames", "probe"):
            if isinstance(value.get(key), list):
                value[key] = f"[{len(value[key])} recorded entries]"
    text = value if isinstance(value, str) else json.dumps(value, ensure_ascii=False, sort_keys=True)
    return text if len(text) <= limit else text[:limit] + " … [truncated; see raw log]"


def _quote(text: str, limit: int = 6000) -> str:
    if len(text) > limit:
        text = text[:limit] + "\n[Excerpt ends; see raw log for the full recorded text.]"
    return "\n".join("> " + line for line in text.splitlines())


def _source_section(run_dir: Path, evaluation: dict, metadata: dict) -> list[str]:
    lines = ["## Source change", ""]
    folder = run_dir / "evaluation"
    original = folder / "original_candidate.py"
    candidate = folder / "frozen_candidate.py"
    candidate_label = "Frozen candidate"
    diff_file = folder / "source.diff"
    if not original.is_file():
        original = run_dir / "broker/versions/v000/actuator.py"
    if not candidate.is_file():
        frozen_path = metadata.get("frozen_source_file")
        if isinstance(frozen_path, str):
            saved = (run_dir / frozen_path).resolve()
            if saved.is_relative_to(run_dir.resolve()) and saved.is_file():
                candidate = saved
        if not candidate.is_file():
            versions = sorted((run_dir / "broker/versions").glob("v[0-9][0-9][0-9]/actuator.py"))
            if versions:
                candidate = versions[-1]
                candidate_label = "Latest saved candidate (not frozen)"
    links = []
    for path, label in ((original, "Original component"), (candidate, candidate_label),
                        (diff_file, "Full source diff")):
        if path.is_file():
            links.append(f"[{label}]({path.resolve().relative_to(run_dir.resolve()).as_posix()})")
    if links:
        lines.extend([" · ".join(links), ""])
    difference = None
    if diff_file.is_file():
        difference = diff_file.read_text()
    elif original.is_file() and candidate.is_file():
        difference = "".join(difflib.unified_diff(
            original.read_text().splitlines(keepends=True),
            candidate.read_text().splitlines(keepends=True),
            fromfile="original_candidate.py", tofile="candidate.py"))
    if difference:
        diff_lines = difference.splitlines()
        lines.extend(["```diff", *diff_lines[:140], "```", ""])
        if len(diff_lines) > 140:
            lines.extend(["Diff excerpt: first 140 lines; use the saved sources or full diff above for the remainder.", ""])
    elif difference == "":
        lines.extend(["The saved original and candidate sources are identical.", ""])
    else:
        lines.extend(["No frozen source comparison was recorded.", ""])
    indicators = evaluation.get("state_extension")
    if indicators:
        lines.extend(["Recorded source indicators (syntactic checks; not proof of the inferred mechanism):", "",
                      "```json", _compact(indicators, 2500), "```", ""])
    return lines


def _plot_distances(run_dir: Path, cases: list[dict]) -> Path | None:
    if not any(_number(_summary(case, name).get("stopping_distance")) is not None
               for case in cases for name in ("original", "candidate", "reference")):
        return None
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    positions = np.arange(len(cases))
    fig, axis = plt.subplots(figsize=(max(8, len(cases) * 1.7), 5.2), layout="constrained")
    width = 0.25
    for offset, name, label, color in (
        (-width, "original", "Original model", "#4d84b4"),
        (0, "candidate", "Frozen candidate", "#398b71"),
        (width, "reference", "Synthetic reference", "#cc6d58"),
    ):
        values = [_number(_summary(case, name).get("stopping_distance")) for case in cases]
        axis.bar(positions + offset, [value if value is not None else np.nan for value in values],
                 width=width, color=color, label=label)
        for index, value in enumerate(values):
            if value is None:
                axis.text(index + offset, 0, "×", ha="center", va="bottom", color=color, fontsize=14)
    axis.set_xticks(positions, [str(case.get("case_id", index + 1)) for index, case in enumerate(cases)])
    axis.set_ylabel("Stopping distance (m)")
    axis.set_title("Recorded frozen evaluation · synthetic braking")
    axis.spines[["top", "right"]].set_visible(False)
    axis.grid(axis="y", alpha=0.2)
    axis.set_axisbelow(True)
    axis.legend(frameon=False)
    fig.text(0.01, -0.03, "× = censored or missing measurement; it does not mean zero distance.", fontsize=9)
    output = run_dir / "evaluation.png"
    fig.savefig(output, dpi=160, bbox_inches="tight")
    plt.close(fig)
    return output


def build_report(run_dir: Path) -> Path:
    """Create report.md from the exact saved prompts, events and evaluation.

    Missing or interrupted API/evaluation records produce an explicitly partial
    report. The function never contacts an API or reads a credential file.
    """
    run_dir = Path(run_dir)
    if not run_dir.is_dir():
        raise FileNotFoundError(f"Investigation directory does not exist: {run_dir}")
    with ThreadPoolExecutor(max_workers=3) as pool:
        metadata_future = pool.submit(_read_json, run_dir / "metadata.json")
        events_future = pool.submit(_read_events, run_dir / "events.jsonl")
        evaluation_future = pool.submit(_read_json, run_dir / "evaluation" / "result.json")
        metadata, metadata_warning = metadata_future.result()
        events, warnings = events_future.result()
        evaluation, evaluation_warning = evaluation_future.result()
    metadata = metadata or {}
    warnings.extend(warning for warning in (metadata_warning, evaluation_warning) if warning)
    if evaluation is None:
        evaluation = next((event["result"] for event in reversed(events)
                           if event.get("type") == "evaluation" and isinstance(event.get("result"), dict)), {})
    cases = [case for case in evaluation.get("cases", []) if isinstance(case, dict)]
    aggregate = evaluation.get("aggregate") if isinstance(evaluation.get("aggregate"), dict) else {}
    status = metadata.get("status", "not recorded")
    model = metadata.get("model", "not recorded")
    effort = metadata.get("reasoning_effort", "not recorded")
    errors = [event for event in events if event.get("type") == "error"]
    lines = ["# Astra investigation report", "", f"**Status:** {_cell(status)}. "
             f"**Model:** `{_cell(model)}`. **Configured reasoning effort:** `{_cell(effort)}`.", ""]
    if "predictive_success" in aggregate:
        verdict = aggregate["predictive_success"]
        label = "passed" if verdict is True else "failed" if verdict is False else "not fully scored"
        lines.extend([f"**Predeclared prediction criteria: {label}.**", ""])
    if cases:
        original_mae = aggregate.get("original_mae_m")
        candidate_mae = aggregate.get("candidate_mae_m")
        if _number(original_mae) is not None and _number(candidate_mae) is not None:
            lines.extend([f"Recorded mean absolute stopping-distance error: **{_metric(original_mae)} m** "
                          f"for the original model and **{_metric(candidate_mae)} m** for the frozen candidate "
                          f"across **{_cell(aggregate.get('scored_cases', 'unspecified'))} scored cases**.", ""])
        else:
            lines.extend([f"Evaluation recorded {len(cases)} cases; aggregate comparable stopping-distance errors were not recorded.", ""])
    else:
        lines.extend(["**No completed case evaluation is recorded.** The activity below does not establish an improved prediction.", ""])
    if metadata.get("agent_submitted") is True:
        lines.extend(["The agent explicitly submitted the candidate for freezing.", ""])
    elif metadata.get("agent_submitted") is False:
        reason = _cell(metadata.get("freeze_reason", "not recorded"))
        lines.extend([f"The agent did **not** explicitly submit a candidate. Freeze reason: {reason}.", ""])
    if errors:
        lines.extend([f"**Recorded failure:** {_cell(errors[-1].get('message', 'Details not recorded.'))}", ""])
    if cases:
        lines.extend(["## Measured evaluation", "", "Distances and absolute errors are in metres. "
                      "A dash retains a missing or censored measurement; it is never scored as zero.", "",
                      "| Case | Original stop | Candidate stop | Reference stop | Original error | Candidate error | Previously observed |",
                      "| --- | ---: | ---: | ---: | ---: | ---: | --- |"])
        for index, case in enumerate(cases):
            observed = case.get("previously_observed")
            observed_label = "yes" if observed is True else "no" if observed is False else "not recorded"
            lines.append("| " + " | ".join([
                _cell(case.get("case_id", index + 1)),
                _distance_cell(_summary(case, "original")), _distance_cell(_summary(case, "candidate")),
                _distance_cell(_summary(case, "reference")), _metric(case.get("original_error_m")),
                _metric(case.get("candidate_error_m")), observed_label,
            ]) + " |")
        lines.append("")
        case_details = []
        for index, case in enumerate(cases):
            label = _cell(case.get("case_id", index + 1))
            for name in ("original", "candidate"):
                error = case.get(f"{name}_error")
                if error:
                    case_details.append(f"- **{label} · {name.capitalize()} runtime failure:** {_cell(error)}")
            directory = case.get("full_records_directory")
            if isinstance(directory, str):
                traces = (run_dir / "evaluation" / directory).resolve()
                if traces.is_relative_to(run_dir.resolve()):
                    links = []
                    for name in ("original", "candidate", "reference"):
                        path = traces / f"{name}.json"
                        if path.is_file():
                            relative = path.relative_to(run_dir.resolve()).as_posix()
                            links.append(f"[{name.capitalize()}]({relative})")
                    if links:
                        case_details.append(f"- **{label} · Full recorded traces:** " + " · ".join(links))
        if case_details:
            lines.extend([*case_details, ""])
        criteria = evaluation.get("predeclared_criteria")
        if isinstance(criteria, dict):
            lines.extend(["### Predeclared scoring", "", "| Criterion | Recorded rule |", "| --- | --- |"])
            for name, value in criteria.items():
                lines.append(f"| {_cell(name.replace('_', ' '))} | {_cell(_compact(value, 800))} |")
            lines.append("")
        scoring_fields = (
            ("scoring_complete", "All planned cases scored"),
            ("improvement_pass", "Aggregate error reduction"),
            ("per_case_pass", "Per-case distance tolerance"),
            ("cold_control_pass", "Cold control retained"),
            ("collision_outcomes_match", "Collision outcomes"),
        )
        scoring_rows = [(key, label) for key, label in scoring_fields if key in aggregate]
        if scoring_rows:
            lines.extend(["| Prediction check | Result |", "| --- | --- |"])
            for key, label in scoring_rows:
                value = aggregate[key]
                result = "passed" if value is True else "failed" if value is False else "not fully scored"
                lines.append(f"| {label} | {result} |")
            lines.extend(["", "Prediction checks and source-extension indicators are reported separately.", ""])
        try:
            chart = _plot_distances(run_dir, cases)
        except Exception as exc:
            chart = None
            warnings.append(f"Evaluation chart unavailable: {type(exc).__name__}; recorded metrics are retained.")
        if chart:
            lines.extend(["![Recorded original, candidate and synthetic reference stopping distances](evaluation.png)", ""])
        comparable = [case for case in cases if _number(case.get("original_error_m")) is not None
                      and _number(case.get("candidate_error_m")) is not None]
        improved = sum(case["candidate_error_m"] < case["original_error_m"] for case in comparable)
        worsened = sum(case["candidate_error_m"] > case["original_error_m"] for case in comparable)
        unchanged = len(comparable) - improved - worsened
        lines.extend(["## What the evidence establishes", "",
                      f"- Candidate error decreased on {improved} of {len(comparable)} comparable cases, "
                      f"increased on {worsened}, and was unchanged on {unchanged}.",
                      f"- {len(cases) - len(comparable)} cases lack comparable uncensored stopping-distance errors."])
        if aggregate.get("failed_stop_predictions") is not None:
            lines.append(f"- Recorded failed stop predictions: {_cell(aggregate['failed_stop_predictions'])}.")
        if any(case.get("previously_observed") is True for case in cases):
            lines.append("- Cases marked previously observed cannot support an unseen-case claim.")
        lines.extend(["- These results concern the recorded synthetic suite. Source changes alone do not validate a physical mechanism or establish research novelty.", ""])
    lines.extend(_source_section(run_dir, evaluation, metadata))
    lines.extend(["## Recorded investigation", "", "These are Astra’s stated explanations and brief API-provided summaries. "
                  "They record its hypotheses, evidence, and reasons for choosing each action.", ""])
    if (run_dir / "events.jsonl").is_file():
        lines.extend(["[Full structured event log](events.jsonl). Tool payloads below are compact excerpts.", ""])
    rendered_events = 0
    for event in events:
        kind = event.get("type")
        timestamp = _cell(event.get("timestamp", "time not recorded"))
        if kind in ("assistant_message", "reasoning_summary") and event.get("text"):
            label = "Assistant message" if kind == "assistant_message" else "Brief API-provided reasoning summary"
            lines.extend([f"**{timestamp} · {label}**", "", _quote(str(event["text"])), ""])
        elif kind == "tool_call":
            lines.extend([f"**{timestamp} · Tool call: `{_cell(event.get('name', 'unknown'))}`**", "",
                          "```json", _compact(event.get("arguments", {})), "```", ""])
        elif kind == "tool_result":
            lines.extend([f"**{timestamp} · Tool result: `{_cell(event.get('name', 'unknown'))}`**", "",
                          "```json", _compact(event.get("result")), "```", ""])
        elif kind in ("status", "error"):
            lines.extend([f"**{timestamp} · {kind.capitalize()}**: {_cell(event.get('message', ''))}", ""])
        else:
            continue
        rendered_events += 1
    if not rendered_events:
        lines.extend(["No visible investigation activity was recorded.", ""])
    lines.extend(["## Technical record", ""])
    links = []
    for path, label in ((run_dir / "prompts/system.md", "Exact system prompt"),
                        (run_dir / "prompts/task.md", "Exact task prompt"),
                        (run_dir / "metadata.json", "Run metadata"),
                        (run_dir / "evaluation/result.json", "Evaluation records")):
        if path.is_file():
            links.append(f"[{label}]({path.relative_to(run_dir).as_posix()})")
    if links:
        lines.extend([" · ".join(links), ""])
    lines.extend(["| Field | Recorded value |", "| --- | --- |"])
    for key in ("model", "reasoning_effort", "status", "start_at", "end_at", "api_requests", "freeze_reason", "source_hash"):
        if key in metadata:
            lines.append(f"| {_cell(key)} | {_cell(metadata[key])} |")
    if isinstance(metadata.get("usage"), dict):
        for key in ("input_tokens", "output_tokens", "total_tokens", "reasoning_tokens", "cached_input_tokens"):
            if key in metadata["usage"]:
                lines.append(f"| usage.{key} | {_cell(metadata['usage'][key])} |")
    for key in ("source_sha256", "original_source_sha256"):
        if key in evaluation:
            lines.append(f"| evaluation.{key} | {_cell(evaluation[key])} |")
    responses = [event for event in events if event.get("type") == "api_response"]
    if responses:
        lines.extend(["", "API response IDs: " + ", ".join(f"`{_cell(event.get('response_id', 'not recorded'))}`" for event in responses) + "."])
    if warnings:
        lines.extend(["", "## Incomplete records", "", *[f"- {warning}" for warning in warnings]])
    output = run_dir / "report.md"
    output.write_text("\n".join(lines).rstrip() + "\n")
    return output
