"""Two independent max-effort model calls on the same complete control task."""
import argparse
from pathlib import Path
import signal

from .platform_pair import run_pair
from .tasks import TASKS, TASK_PLATFORMS


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--platform", required=True, choices=TASK_PLATFORMS)
    parser.add_argument("--scenario", required=True, choices=TASKS)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--max-api-requests", type=int, default=16)
    parser.add_argument("--max-seconds", type=int, default=1800)
    parser.add_argument("--no-frames", action="store_true")
    args = parser.parse_args(argv)
    def stop(signum, frame):
        raise InterruptedError("Comparison stopped")
    signal.signal(signal.SIGTERM, stop)
    result = run_pair(args.output, platform=args.platform, scenario=args.scenario,
                      max_api_requests=args.max_api_requests, max_seconds=args.max_seconds,
                      no_frames=args.no_frames, control_task=True)
    print(f"{args.platform} comparison: {result['status']}, matched={result['comparison_valid']}")
    return 0 if result["status"] == "completed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
