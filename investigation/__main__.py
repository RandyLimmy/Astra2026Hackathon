"""Run the car investigation with Astra/medium using one local API key."""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
from uuid import uuid4

from openai import APIError

from .api import MODEL, REASONING_EFFORT, Settings, request_response
from .runner import EventLog, run_session, safe_api_error, save_json, timestamp


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--env-file", type=Path, default=Path(".env"))
    parser.add_argument("--output", type=Path, help="new session directory")
    parser.add_argument("--cache-dir", type=Path, default=Path("runs/investigation-cache"))
    parser.add_argument("--max-api-requests", type=int, default=12)
    parser.add_argument("--max-seconds", type=int, default=1800)
    parser.add_argument("--check-key", action="store_true", help="one tiny Astra/medium request only")
    args = parser.parse_args()
    try:
        settings = Settings.load(args.env_file)
        with settings.client() as client:
            if args.check_key:
                result = request_response(client, [{"role": "user", "content": "Reply exactly API_READY."}],
                                          [], max_output_tokens=512)
                print(json.dumps({"status": result.status, "model": result.model,
                                  "reasoning_effort": result.reasoning.effort, "reply": result.output_text}))
                return 0 if result.status == "completed" and result.model == MODEL and result.reasoning.effort == REASONING_EFFORT else 1
            from .broker import InvestigationBroker
            from .evaluation import PREDECLARED_CRITERIA, evaluate_frozen
            from .physics import PhysicsService
            from .report import build_report

            stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            output = args.output or Path("runs") / f"astra-{stamp}-{uuid4().hex[:8]}"
            output.mkdir(parents=True, exist_ok=False)
            save_json(output / "evaluation_criteria.json", {"declared_at": timestamp(),
                                                          "criteria": PREDECLARED_CRITERIA})
            log = EventLog(output / "events.jsonl")
            physics = PhysicsService(args.cache_dir)
            broker = InvestigationBroker(output / "broker", physics)
            metadata = run_session(client, broker, output, log=log,
                                   max_api_requests=args.max_api_requests, max_seconds=args.max_seconds,
                                   evaluate=lambda source, destination: evaluate_frozen(
                                       physics, source, destination,
                                       emit=lambda event: log("status", message="Evaluation: " + event["stage"])))
            report = build_report(output)
            print(f"Run status: {metadata['status']}\nReport: {report.resolve()}", flush=True)
            return 0 if metadata["status"] == "completed" else 1
    except APIError as error:
        parser.exit(1, safe_api_error(error) + "\n")
    except (ValueError, OSError) as error:
        # These errors arise from trusted argument/config validation. Credentials
        # and remote response bodies are never embedded in these messages.
        parser.exit(1, f"Setup failed: {error}\n")


if __name__ == "__main__":
    raise SystemExit(main())
