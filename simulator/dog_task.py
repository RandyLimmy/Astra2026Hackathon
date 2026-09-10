"""Bounded, editable dog-controller JSON and a neutral investigation export.

The same loader is used by native viewing and recorded candidate attempts.
This module contains no successful controller values or remote model calls.
"""

from __future__ import annotations

import argparse
import hashlib
import inspect
import json
from pathlib import Path
import shutil

from .platforms import quadruped_controller


MAX_CONTROLLER_BYTES = 16 * 1024


def _unique_fields(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"Duplicate controller field: {key}")
        result[key] = value
    return result


def load_controller(path: Path) -> dict[str, float]:
    """Read only controller parameters; unknown task/physics fields are rejected."""
    path = Path(path)
    if path.stat().st_size > MAX_CONTROLLER_BYTES:
        raise ValueError("Controller JSON exceeds 16 KiB")
    try:
        values = json.loads(path.read_text(encoding="utf-8"), object_pairs_hook=_unique_fields)
    except (UnicodeError, RecursionError, json.JSONDecodeError) as error:
        raise ValueError("Controller must be a small JSON object") from error
    if not isinstance(values, dict) or set(values) != set(quadruped_controller.DEFAULT_PARAMETERS):
        raise ValueError("Controller JSON must contain exactly the documented gait parameters")
    return quadruped_controller.validate_parameters(values)


def export_task(output: Path, *, evidence: Path | None = None) -> None:
    """Export the starter and raw evidence, never a developer reference policy."""
    source = None
    if evidence is not None:
        source = Path(evidence)
        if (source / "public").is_dir():
            source = source / "public"
        manifest = json.loads((source / "manifest.json").read_text())
        if manifest.get("scenario_id") != "quadruped_gait_failure" or manifest.get("provenance") != "original_attempt":
            raise ValueError("Use an original dog attempt for public task evidence")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    controller = quadruped_controller.validate_parameters(None)
    (output / "controller.json").write_text(json.dumps(controller, indent=2) + "\n")
    package = Path(__file__).resolve().parents[1]
    shutil.copyfile(package / "contracts/QUADRUPED_CONTROLLER.md", output / "CONTROLLER.md")
    shutil.copyfile(Path(inspect.getfile(quadruped_controller)), output / "controller_source.py")
    (output / "TASK.md").write_text(
        "# Walking control task\n\n"
        "Follow the declared walking-speed schedule along the strip while remaining upright. "
        "Inspect recorded feet, body motion, contacts, leg phases and controller source. "
        "Form a hypothesis, edit controller.json using only its documented fields, "
        "and test the changed controller on the same task.\n\n"
        "The host runs a candidate with: python -m simulator.scenario_replay "
        "quadruped_gait_failure --controller PATH/controller.json\n\n"
        "Task success requires the original progress/speed/balance criteria. "
        "A slowed diagnostic, changed world or changed task is not a successful repair. "
        "No reference solution is included. These files do not claim that GPT-6 has run.\n"
    )
    if source is not None:
        raw = output / "evidence"
        raw.mkdir()
        for name in ("observations.jsonl", "events.json", "summary.json"):
            shutil.copyfile(source / name, raw / name)
        # Only unannotated RGB, with an index stripped of presentation views and
        # any links to other attempts, enter the model's evidence package.
        shutil.copytree(source / "evidence", raw / "images")
        index = {"run_id": manifest["run_id"], "fps": manifest["fps"],
                 "duration_s": manifest["duration_s"], "physics_sha256": manifest["physics_sha256"],
                 "controller_sha256": manifest["controller_sha256"], "cameras": manifest["cameras"],
                 "frames": [{"t_s": frame["t_s"],
                             "images": {camera: "images/" + Path(path).name
                                        for camera, path in frame["evidence"].items()}}
                            for frame in manifest["frames"]]}
        (raw / "frames.json").write_text(json.dumps(index, separators=(",", ":")) + "\n")
    hashes = {path.name: hashlib.sha256(path.read_bytes()).hexdigest() for path in output.iterdir() if path.is_file()}
    (output / "package.json").write_text(json.dumps({"kind": "dog_controller_task", "sha256": hashes}, indent=2) + "\n")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="action", required=True)
    export = commands.add_parser("export", help="export starter controller, source and neutral task")
    export.add_argument("output", type=Path)
    export.add_argument("--evidence", type=Path, help="original recording directory or its public subdirectory")
    validate = commands.add_parser("validate", help="validate a controller JSON without running physics")
    validate.add_argument("controller", type=Path)
    args = parser.parse_args(argv)
    try:
        if args.action == "export":
            export_task(args.output, evidence=args.evidence)
            print(f"Public dog task: {args.output.resolve()}")
        else:
            print(json.dumps(load_controller(args.controller), indent=2))
    except (OSError, ValueError) as error:
        parser.exit(2, f"error: {error}\n")


if __name__ == "__main__":
    main()
