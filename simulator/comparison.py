"""Trusted development comparisons using the same car mechanics for both models.

Only commands and declared resets cross from preparation into candidate runs.
Each candidate completes its prediction before the reference trial is revealed.
This is not the reserved evaluation suite or an API-backed investigation.
"""

from contextlib import ExitStack
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

from component_worker import WheelActuatorWorker
from component_worker.client import MAX_SOURCE_BYTES

from .config import Experiment
from .recording import Recorder
from .runner import Simulator


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CANDIDATE = ROOT / "candidate" / "wheel_actuator.py"
DEVELOPER_CHECK = ROOT / "reference_host" / "developer_wheel_actuator.py"
PUBLIC_CONFIG_FIELDS = (
    "initial_speed", "brake_at", "brake", "throttle", "duration", "wall", "wall_x",
    "timestep", "warmup_cycles", "recovery", "commands",
)


def public_config(config: Experiment) -> dict:
    """Allowlist interventions; exclude operator-only mechanisms and parameters."""
    values = config.to_dict()
    return {key: values[key] for key in PUBLIC_CONFIG_FIELDS}


def nominal_config(config: Experiment) -> Experiment:
    return Experiment.from_dict(public_config(config))


def _write(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x") as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write("\n")


def _timestamp():
    return datetime.now(timezone.utc).isoformat()


def outcome_error(prediction: dict, reference: dict) -> dict:
    a, b = prediction["stopping_distance"], reference["stopping_distance"]
    return {
        "stopping_distance_error_m": abs(a - b) if a is not None and b is not None else None,
        "collision_prediction_correct": prediction["collision"] == reference["collision"],
        "final_front_x_error_m": abs(prediction["final_front_x"] - reference["final_front_x"]),
    }


def compare(config: Experiment, output: Path, *, candidate: Path = DEFAULT_CANDIDATE,
            developer_check: bool = False, frames: bool = False, camera: str = "overview",
            fps: int = 30) -> dict:
    """Record independent candidate/reference worlds with exact common preparation.

    The output is a builder directory, not an agent export. Each run contains
    explicitly separated public observations and private simulator diagnostics.
    """
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    sources = {"candidate": Path(candidate)}
    if developer_check:
        sources["developer_check"] = DEVELOPER_CHECK
    predictions = {}
    frozen = {}
    with ExitStack() as stack:
        reference = Simulator(config)
        recorder = stack.enter_context(Recorder(output / "reference", reference.model,
                                               frames=frames, camera=camera, fps=fps))

        def capture_reference(current, phase):
            recorder.record(current.data, current.observe(phase), current.diagnostics())

        print("Preparing reference; recording public commands and resets...", flush=True)
        reference.prepare(capture_reference)
        history = json.loads(json.dumps(reference.preparation_history, allow_nan=False))
        _write(output / "preparation.json", history)
        for name, source in sources.items():
            print(f"Predicting with {name} in the isolated worker...", flush=True)
            with source.open("rb") as stream:
                source_bytes = stream.read(MAX_SOURCE_BYTES + 1)
            if len(source_bytes) > MAX_SOURCE_BYTES:
                raise ValueError("Candidate source exceeds the permitted size")
            source_snapshot = output / name / "actuator.py"
            source_snapshot.parent.mkdir(parents=True, exist_ok=True)
            with source_snapshot.open("xb") as snapshot_stream:
                snapshot_stream.write(source_bytes)
            # The worker copies and hashes the exact source it executes. Never
            # import this source into the host or infer state from diagnostics.
            with WheelActuatorWorker(source_snapshot) as worker:
                model = Simulator(nominal_config(config), actuator=worker)
                with Recorder(output / name, model.model, frames=frames, camera=camera, fps=fps) as record:
                    def capture_model(current, phase):
                        record.record(current.data, current.observe(phase), current.diagnostics())

                    result = model.run(capture_model, preparation_history=history)
                    result["component_state"] = worker.inspect_state()
                    result["actuator_source_sha256"] = worker.source_sha256
                    record.finish(result, result["public"])
                predictions[name] = result["public"]
                frozen[name] = {"source_sha256": worker.source_sha256,
                                "source_file": f"{name}/actuator.py",
                                "prediction_completed_at": _timestamp(),
                                "summary": result["public"]}
        _write(output / "predictions.json", frozen)
        print("Predictions saved. Running the reference trial...", flush=True)
        reference_started = _timestamp()
        result = reference.run(capture_reference, prepared=True)
        recorder.finish(result, result["public"])
    bundle = {
        "schema_version": 2,
        "component_interface": "wheel_v2",
        "experiment_kind": "synthetic_development",
        "agent_run": False,
        "final_holdout_evaluation": False,
        "developer_check_is_manually_authored": developer_check,
        "public_config": public_config(config),
        "preparation_sha256": hashlib.sha256(json.dumps(history, sort_keys=True).encode()).hexdigest(),
        "reference_trial_started_at": reference_started,
        "reference": result["public"],
        "predictions": frozen,
        "errors": {name: outcome_error(summary, result["public"])
                   for name, summary in predictions.items()},
    }
    _write(output / "comparison.json", bundle)
    plot_comparison(output, bundle)
    return bundle


def plot_comparison(output: Path, bundle: dict) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8), layout="constrained")
    series = [("reference", "Synthetic reference", "#d76552"),
              ("candidate", "Editable Python model", "#367daf")]
    if "developer_check" in bundle["predictions"]:
        series.append(("developer_check", "Developer stateful check", "#23876f"))
    for name, label, color in series:
        with (output / name / "public" / "observations.jsonl").open() as stream:
            rows = [row for line in stream if (row := json.loads(line))["phase"] == "trial"]
        times = [row["phase_time"] for row in rows]
        axes[0].plot(times, [sum(v*v for v in row["velocity"]) ** .5 for row in rows],
                     color=color, label=label, linewidth=2)
        axes[1].plot(times, [row["front_x"] for row in rows], color=color, label=label, linewidth=2)
    axes[0].set(xlabel="Trial time (s)", ylabel="Speed (m/s)")
    axes[1].set(xlabel="Trial time (s)", ylabel="Leading chassis position (m)")
    if bundle["public_config"]["wall"]:
        axes[1].axhline(bundle["public_config"]["wall_x"], color="#aa8833", linestyle=":", label="Wall")
    for axis in axes:
        axis.spines[["top", "right"]].set_visible(False)
        axis.grid(alpha=.2)
        axis.legend(frameon=False, fontsize=8)
    fig.suptitle("RealityPatch · editable Python component + four-wheel car", fontsize=15)
    fig.text(.01, -.035, "Synthetic development run. Developer check is manually authored; no Astra repair or final evaluation.",
             fontsize=9)
    fig.savefig(output / "comparison.png", dpi=160, bbox_inches="tight")
    plt.close(fig)


def export_task(output: Path) -> None:
    """Export only the source template and neutral v2 contract."""
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    (output / "actuator.py").write_bytes(DEFAULT_CANDIDATE.read_bytes())
    (output / "INTERFACE.md").write_bytes((ROOT / "contracts" / "WHEEL_ACTUATOR.md").read_bytes())
    (output / "TASK.md").write_text(
        "# Component investigation\n\nImprove the Python component using permitted observations "
        "and experiments. State expected observations before testing a hypothesis, and report "
        "uncertainty. This source package is a fixture; the experiment broker is not connected yet.\n")
