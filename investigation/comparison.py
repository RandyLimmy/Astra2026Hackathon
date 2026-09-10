"""Export two completed investigation records without running models or physics.

Usage: python -m investigation.comparison RUN_A RUN_B NEW_OUTPUT_DIRECTORY
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re


class ComparisonError(ValueError):
    """Input records cannot support an integrity-checked comparison."""


def _sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _canonical(value) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def _file(root: Path, relative: str) -> Path:
    requested = root / relative
    resolved = requested.resolve()
    if Path(relative).is_absolute() or not resolved.is_relative_to(root) or not resolved.is_file():
        raise ComparisonError("A required recording file is missing or outside its run directory.")
    return resolved


def _read(root: Path, relative: str) -> dict:
    def reject_constant(_):
        raise ValueError("non-finite JSON")
    try:
        value = json.loads(_file(root, relative).read_text(), parse_constant=reject_constant)
    except (OSError, ValueError):
        raise ComparisonError(f"Invalid required recording: {relative}.") from None
    if not isinstance(value, dict):
        raise ComparisonError(f"Expected an object in {relative}.")
    return value


def _fingerprint(value):
    return isinstance(value, str) and re.fullmatch(r"[0-9a-f]{64}", value) is not None


def _record(directory: Path, label: str):
    root = Path(directory).resolve()
    metadata = _read(root, "metadata.json")
    if metadata.get("status") != "completed":
        raise ComparisonError("Both input runs must have completed evaluation.")
    evaluation = _read(root, "evaluation/result.json")
    candidate = _file(root, "evaluation/frozen_candidate.py").read_bytes()
    original = _file(root, "evaluation/original_candidate.py").read_bytes()
    digest, original_digest = _sha(candidate), _sha(original)
    if metadata.get("source_hash") != digest or evaluation.get("source_sha256") != digest:
        raise ComparisonError("A frozen candidate source hash does not match its recorded metadata/evaluation.")
    if evaluation.get("original_source_sha256") != original_digest:
        raise ComparisonError("An original component source hash does not match its evaluation.")
    submitted = metadata.get("frozen_source_file")
    if not isinstance(submitted, str) or Path(submitted).name != "actuator.py":
        raise ComparisonError("A run is missing its recorded submitted-component location.")
    if _file(root, submitted).read_bytes() != candidate:
        raise ComparisonError("The submitted component differs from the evaluated frozen component.")
    fingerprint = metadata.get("protocol_fingerprint")
    manifest = metadata.get("protocol_manifest")
    verified_manifest = False
    if _fingerprint(fingerprint) and isinstance(manifest, dict):
        if _sha(_canonical(manifest)) != fingerprint:
            raise ComparisonError("A protocol manifest does not match its recorded fingerprint.")
        verified_manifest = True
    cases = evaluation.get("cases")
    if not isinstance(cases, list) or not cases or not all(isinstance(case, dict) for case in cases):
        raise ComparisonError("Each run must contain nonempty recorded evaluation cases.")
    budgets = metadata.get("budgets", {})
    budgets = budgets if isinstance(budgets, dict) else {}

    def count(name):
        value = budgets.get(name)
        return value.get("used") if isinstance(value, dict) else None

    record = {
        "id": label, "profile": metadata.get("profile"), "model": metadata.get("model"),
        "reasoning_effort": metadata.get("reasoning_effort"), "status": metadata["status"],
        "source_sha256": digest, "original_source_sha256": original_digest,
        "source_file": f"{label}/actuator.py", "original_source_file": f"{label}/original_actuator.py",
        "protocol_fingerprint": fingerprint, "protocol_manifest_verified": verified_manifest,
        "agent_submitted": metadata.get("agent_submitted"), "freeze_reason": metadata.get("freeze_reason"),
        "debrief_status": metadata.get("debrief_status"), "duration_s": metadata.get("duration_s"),
        "api_requests": metadata.get("api_requests"),
        "tool_calls": count("tool_calls"), "extra_reference_attempts": count("run_experiment"),
        "patch_attempts": count("patch_model"), "model_attempts": count("run_model"),
        "regression_attempts": count("run_regression_suite"), "budgets": budgets,
        "usage": metadata.get("usage", {}), "aggregate": evaluation.get("aggregate", {}),
        "predeclared_criteria": evaluation.get("predeclared_criteria", {}),
        "state_extension": evaluation.get("state_extension", {}),
        "cases": [{key: case.get(key) for key in (
            "case_id", "config", "original_error_m", "candidate_error_m", "previously_observed",
            "collision_correct", "within_tolerance", "distance_tolerance_m", "cold_control_pass")}
            | {name: case.get(name, {}).get("summary", {}) for name in ("original", "candidate", "reference")}
            for case in cases],
    }
    return record, candidate, original, manifest


def _case_map(record):
    cases = record["cases"]
    ids = [case.get("case_id") for case in cases]
    if any(not isinstance(identifier, str) or not identifier for identifier in ids) or len(set(ids)) != len(ids):
        return None
    return {case["case_id"]: case for case in cases}


def _checks(first, second):
    a, b = _case_map(first), _case_map(second)
    same_ids = a is not None and b is not None and set(a) == set(b)
    same_configs = bool(same_ids and all(isinstance(a[key]["config"], dict) and a[key]["config"] == b[key]["config"] for key in a))
    same_eligibility = bool(same_ids and all(type(a[key]["previously_observed"]) is bool
                                            and type(b[key]["previously_observed"]) is bool
                                            and a[key]["previously_observed"] == b[key]["previously_observed"] for key in a))
    same_references = bool(same_configs and all(a[key]["reference"] == b[key]["reference"] for key in a))
    return {
        "frozen_source_hashes_verified": True,
        "protocol_manifests_verified": first["protocol_manifest_verified"] and second["protocol_manifest_verified"],
        "protocol_fingerprints_match": _fingerprint(first["protocol_fingerprint"]) and first["protocol_fingerprint"] == second["protocol_fingerprint"],
        "original_sources_match": first["original_source_sha256"] == second["original_source_sha256"],
        "distinct_profiles": first["profile"] != second["profile"] and bool(first["profile"] and second["profile"]),
        "case_ids_match": same_ids, "case_configs_match": same_configs,
        "case_eligibility_matches": same_eligibility, "reference_summaries_match": same_references,
        "predeclared_criteria_match": bool(first["predeclared_criteria"]) and first["predeclared_criteria"] == second["predeclared_criteria"],
    }


def _cell(value):
    return str(value).replace("|", "\\|").replace("\n", " ").replace("\r", " ")


def _metric(value):
    if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value):
        return f"{value:.3f}"
    return "—"


def _verdict(value):
    return "passed" if value is True else "failed" if value is False else "not fully scored"


def _markdown(bundle):
    first, second = bundle["runs"]
    labels = [f"{record['model']} / {record['reasoning_effort']}" for record in bundle["runs"]]
    lines = ["# Recorded investigator comparison", "",
             "One recorded run per model profile. This is a small synthetic experiment, not a model ranking or a general performance claim.", "",
             "Both investigators were started in fresh API contexts. The builder had seen the reserved cases in an earlier pilot; this is not a builder-blind benchmark.", "",
             f"**Comparison validation:** {'matched recorded protocol and cases' if bundle['comparison_valid'] else 'MISMATCH — do not treat these as a matched comparison'}.", ""]
    if bundle["protocol_fingerprint"]:
        lines.extend([f"**Declared shared protocol fingerprint:** `{bundle['protocol_fingerprint']}`.", ""])
    if bundle["warnings"]:
        lines.extend(["Validation flags:", "", *[f"- {_cell(warning)}" for warning in bundle["warnings"]], ""])
    lines.extend(["## Recorded outcomes and work", "",
                  "MAEs below are each run’s recorded eligible-case aggregates. Missing or censored values remain missing; no missing value is treated as zero.", "",
                  f"| Measure | {_cell(labels[0])} | {_cell(labels[1])} |", "| --- | ---: | ---: |"])
    measures = [
        ("Original stopping MAE (m)", lambda r: _metric(r["aggregate"].get("original_mae_m"))),
        ("Frozen candidate stopping MAE (m)", lambda r: _metric(r["aggregate"].get("candidate_mae_m"))),
        ("Scored / eligible cases", lambda r: f"{r['aggregate'].get('scored_cases', '—')} / {r['aggregate'].get('eligible_cases', '—')}"),
        ("Failed stop predictions", lambda r: r["aggregate"].get("failed_stop_predictions")),
        ("Predeclared prediction criteria", lambda r: _verdict(r["aggregate"].get("predictive_success"))),
        ("API requests", lambda r: r["api_requests"]), ("Tool calls", lambda r: r["tool_calls"]),
        ("Extra reference attempts", lambda r: r["extra_reference_attempts"]),
        ("Patch attempts", lambda r: r["patch_attempts"]), ("Model attempts", lambda r: r["model_attempts"]),
        ("Regression calls", lambda r: r["regression_attempts"]),
        ("Session duration, including evaluation (s)", lambda r: _metric(r["duration_s"])),
        ("Total tokens", lambda r: r["usage"].get("total_tokens")),
        ("Reasoning tokens", lambda r: r["usage"].get("reasoning_tokens")),
        ("Explicit agent submission", lambda r: "yes" if r["agent_submitted"] is True else "no" if r["agent_submitted"] is False else "not recorded"),
        ("Freeze reason", lambda r: r["freeze_reason"]),
        ("Changed source", lambda r: r["state_extension"].get("source_changed")),
        ("Changed state initialization", lambda r: r["state_extension"].get("init_state_changed")),
        ("Changed state evolution", lambda r: r["state_extension"].get("advance_state_changed")),
    ]
    for label, get in measures:
        values = [get(record) for record in bundle["runs"]]
        lines.append(f"| {label} | " + " | ".join(_cell(value) if value is not None else "—" for value in values) + " |")
    lines.extend(["", "Attempts include rejected requests. Source indicators are syntactic/runtime evidence, not proof of novelty or a uniquely correct physical explanation.", "",
                  "## Case records", "",
                  "Rows retain each run’s own case configuration and prior-observation status; validation flags above identify any mismatch.", "",
                  "| Run | Case | Speed (m/s) | Prep | Wait (s) | Brake | Wall (m) | Original stop (m) | Candidate stop (m) | Reference stop (m) | Original error (m) | Candidate error (m) | Previously observed |",
                  "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |"])
    for record in bundle["runs"]:
        for case in record["cases"]:
            config = case["config"] if isinstance(case["config"], dict) else {}
            observed = case["previously_observed"]
            cells = [_cell(record["profile"]), _cell(case["case_id"]), _metric(config.get("speed_mps")),
                     _cell(config.get("preparation_cycles", "—")), _metric(config.get("wait_s")),
                     _metric(config.get("brake_strength")), "none" if config.get("wall_distance_m") is None else _metric(config["wall_distance_m"]),
                     *[_metric(case[name].get("stopping_distance")) for name in ("original", "candidate", "reference")],
                     _metric(case["original_error_m"]), _metric(case["candidate_error_m"]),
                     "yes" if observed is True else "no" if observed is False else "not recorded"]
            lines.append("| " + " | ".join(cells) + " |")
    lines.extend(["", "## Recorded scoring rules", "", "| Criterion | Run A | Run B |", "| --- | --- | --- |"])
    for key in sorted(set(first["predeclared_criteria"]) | set(second["predeclared_criteria"])):
        lines.append("| " + _cell(key) + " | " + " | ".join(_cell(json.dumps(record["predeclared_criteria"].get(key))) for record in bundle["runs"]) + " |")
    lines.extend(["", "| Prediction check | Run A | Run B |", "| --- | --- | --- |"])
    for key in ("scoring_complete", "improvement_pass", "per_case_pass", "cold_control_pass", "collision_outcomes_match"):
        lines.append(f"| {_cell(key)} | " + " | ".join(_verdict(record["aggregate"].get(key)) for record in bundle["runs"]) + " |")
    lines.extend(["", "## Exact frozen components", "",
                  "These are byte-for-byte copies of the verified evaluated components. Passing interface validation or freezing a component does not imply it passed prediction criteria.", ""])
    for record in bundle["runs"]:
        lines.append(f"- [{_cell(record['profile'])}: frozen component]({record['source_file']}) · SHA-256 `{record['source_sha256']}`.")
        lines.append(f"- [{_cell(record['profile'])}: original component]({record['original_source_file']}) · SHA-256 `{record['original_source_sha256']}`.")
    lines.extend(["", "[Machine-readable comparison](comparison.json) · [Recorded protocol manifests](protocols.json)", "",
                  "This export only reads saved records. It makes no API requests, changes no source run, and does not generate replacement evaluation cases."])
    return "\n".join(lines) + "\n"


def export_comparison(run_a: Path, run_b: Path, output: Path) -> Path:
    """Validate frozen records, write a new comparison, and leave both inputs intact."""
    roots = [Path(run_a).resolve(), Path(run_b).resolve()]
    destination = Path(output).resolve()
    if roots[0] == roots[1]:
        raise ComparisonError("Select two distinct completed run directories.")
    if destination.exists() or any(destination.is_relative_to(root) for root in roots):
        raise ComparisonError("Comparison output must be a new directory outside both source runs.")
    loaded = [_record(root, label) for root, label in zip(roots, ("run_a", "run_b"))]
    records = [item[0] for item in loaded]
    checks = _checks(*records)
    warnings = [f"Validation mismatch: {name.replace('_', ' ')}." for name, passed in checks.items() if not passed]
    if any(case["previously_observed"] is True for record in records for case in record["cases"]):
        warnings.append("Previously observed cases cannot support an unseen-case claim; consult each run's eligible-case aggregate.")
    fingerprint = records[0]["protocol_fingerprint"] if checks["protocol_fingerprints_match"] else None
    bundle = {"schema_version": 1, "kind": "recorded_two_run_comparison",
              "created_at": datetime.now(timezone.utc).isoformat(), "comparison_valid": all(checks.values()),
              "protocol_fingerprint": fingerprint, "validation": checks, "warnings": warnings,
              "runs_per_profile": 1, "fresh_model_contexts": True, "prior_cases_known_to_builder": True,
              "is_model_ranking": False, "runs": records}
    # Finish validation/rendering before creating the new output directory.
    text = _markdown(bundle)
    encoded = json.dumps(bundle, indent=2, allow_nan=False) + "\n"
    protocols = {record["id"]: {"fingerprint": record["protocol_fingerprint"], "manifest": item[3]}
                 for record, item in zip(records, loaded)}
    destination.mkdir(parents=True, exist_ok=False)
    for record, (_, source, original, _) in zip(records, loaded):
        folder = destination / record["id"]
        folder.mkdir()
        (folder / "actuator.py").write_bytes(source)
        (folder / "original_actuator.py").write_bytes(original)
    (destination / "comparison.json").write_text(encoded)
    (destination / "protocols.json").write_text(json.dumps(protocols, indent=2, allow_nan=False) + "\n")
    report = destination / "comparison.md"
    report.write_text(text)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_a", type=Path)
    parser.add_argument("run_b", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args(argv)
    try:
        report = export_comparison(args.run_a, args.run_b, args.output)
    except (ComparisonError, OSError) as error:
        parser.exit(1, f"Comparison export failed: {error}\n")
    print(report)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
