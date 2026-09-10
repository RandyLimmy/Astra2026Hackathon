"""Run a complete controller task with recorded visual evidence."""
import argparse
import json
from pathlib import Path

from .api import DEFAULT_PROFILE, PROFILES, Settings
from .platform_run import run_platform_session, _validate_options
from .platform_story import atomic_json
from .runner import timestamp
from .task_broker import TaskBroker
from .tasks import TASKS, TASK_PLATFORMS, adapter_for


def write_report(output, metadata):
    result_path = output / "evaluation/result.json"
    result = json.loads(result_path.read_text()) if result_path.is_file() else {}
    aggregate = result.get("aggregate", {})
    verdict = "completed" if aggregate.get("goal_achieved") is True else "partial" if aggregate.get("partial_success") else "failed or unverified"
    source = metadata.get("frozen_source_file")
    lines = ["# Complete control-task investigation", "",
             f"Model: **{metadata['model']} / {metadata['reasoning_effort']}**. Task outcome: **{verdict}**.", "",
             f"API requests: {metadata['api_requests']}. Actual image inputs: {metadata.get('image_inputs', 0)}. "
             f"Elapsed time including simulations: {metadata['duration_s']:.1f}s.", "",
             "[Exact action checkpoints](story.json) · [Task and criteria](broker/verification_plan.json) · "
             "[Final verification](evaluation/result.json)", "",
             "A controller edit is separate from its measured task outcome. The fresh final run uses the "
             "frozen controller and the original physical world, initial conditions, mission and deadline.", ""]
    if source:
        lines += [f"[Frozen controller]({source})", ""]
    (output / "report.md").write_text("\n".join(lines))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", required=True, choices=TASKS)
    parser.add_argument("--platform", choices=TASK_PLATFORMS)
    parser.add_argument("--profile", choices=PROFILES, default=DEFAULT_PROFILE)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--comparison-id")
    parser.add_argument("--max-api-requests", type=int, default=16)
    parser.add_argument("--max-seconds", type=int, default=1800)
    parser.add_argument("--no-frames", action="store_true")
    parser.add_argument("--preview-only", action="store_true")
    args = parser.parse_args(argv)
    platform = TASKS[args.scenario]["platform"]
    if args.platform and args.platform != platform:
        parser.error("Scenario and platform must match.")
    _validate_options(platform, args.max_api_requests, args.max_seconds)
    if args.output.exists():
        parser.error("Choose a new output directory.")
    args.output.mkdir(parents=True)
    adapter = adapter_for(args.scenario)
    broker = TaskBroker(args.output / "broker", adapter, frames=not args.no_frames)
    atomic_json(args.output / "host_setup.json", {"scenario": args.scenario, "platform": platform,
                "task_kind": "controller_repair", "comparison_id": args.comparison_id})
    if args.preview_only:
        broker.initial_evidence()
        atomic_json(args.output / "metadata.json", {"kind": "control_task_preview", "task_kind": "controller_repair",
                    "platform": platform, "scenario": args.scenario, "status": "completed",
                    "start_at": timestamp(), "api_requests": 0, "image_inputs": 0,
                    "provenance": "Original engineering preview; no model intervention."})
        print(f"Original task preview: {args.output.resolve()}")
        return 0
    settings = Settings.load(profile=args.profile)
    with settings.client() as client:
        metadata = run_platform_session(client, broker, args.output, platform=platform,
                    profile=args.profile, comparison_id=args.comparison_id,
                    max_api_requests=args.max_api_requests, max_seconds=args.max_seconds)
    write_report(args.output, metadata)
    print(f"{platform} {args.profile}: {metadata['status']} — {args.output / 'report.md'}")
    return 0 if metadata["status"] == "completed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
