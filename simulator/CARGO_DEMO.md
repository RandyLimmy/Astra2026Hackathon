# Cargo failure: animation and GPT-6 model repair

Double-click `launch_cargo_demo.command`, or run:

```sh
.venv/bin/mjpython -m simulator view warehouse_curve_demo --camera overview --speedup 0.75
```

Press **Space** to play. The trolley follows a marked straight approach into a
90-degree right bend, then follows the exit. A sharp turn overloads the restraint;
the released cargo slides across the deck, falls off, and hits the ground.
Space pauses or replays; C changes camera; +/- changes playback speed.

This is a synthetic differential-drive trolley, not a rail train. MuJoCo computes
the movement, cargo/deck contacts, weight transfer, and ground impacts. The latch is a
simplified breakaway restraint: it releases after a sustained force threshold.
There is no injected steering offset or position teleport.

## The failure cases

| Preset | Experiment |
|---|---|
| `warehouse_curve_demo` | Follow the marked bend at speed; cargo breaks loose and falls |
| `warehouse_curve_control_demo` | Follow the same bend more slowly; retain cargo |
| `warehouse_cargo_demo` | Straight approach, right turn, restraint release, departure and coast |
| `warehouse_cargo_mirror_demo` | Opposite turn with the same restraint and cargo |
| `warehouse_cargo_strong_demo` | Sharper turn with the same restraint |
| `warehouse_cargo_gentle_demo` | Straight control that stays below the release threshold |

The curve cases use the same route controller and restraint, with a speed change.
Their feedback motor commands respond to the measured state. Compare them in MuJoCo:

```sh
.venv/bin/mjpython -m simulator view warehouse_curve_demo --camera overview
.venv/bin/mjpython -m simulator view warehouse_curve_control_demo --camera overview
.venv/bin/python -m simulator compare warehouse_curve_demo
```

The earlier `warehouse_cargo_*` cases retain their constrained load-shift fixture
and time-only commands. They use the same hidden restraint strength. The last case retains the historical
`gentle` probe name, but deliberately commands a straight control. Compare any
case with identical commands and a secured reference:

```sh
.venv/bin/python -m simulator compare warehouse_cargo_demo
.venv/bin/mjpython -m simulator view warehouse_cargo_demo --set fault=healthy
```

The original `warehouse_demo` and `warehouse_shift` timed-release presets remain
available. The new cases use a measured force trigger, which can generalize to
unseen maneuvers.

## Synchronized comparison animation

With the Python dependencies in `requirements.txt` installed under Python 3.12+:

```sh
python -m simulator.cargo_demo --output runs/my-cargo-demo
```

Open `animation.gif`, or `animation.mp4` if FFmpeg is available locally. The
animation pairs close views of the original fixed-cargo prediction and measured
failure. A shared-scale path plot shows the divergence. Cargo travel, restraint
state, and the raised center-of-mass marker are operator presentation information.
These annotated artifacts are **not** the investigator's neutral evidence bundle.

For the earlier `--scenario warehouse_cargo_demo`, the 11-second case produces about **2.48 m final path error**, a maximum
**28.8 cm cargo displacement**, and an upright trolley. The first sampled release
is around **3.6 s**; exact physical onset is in the private simulator event log.
Those earlier trajectories receive identical motor commands. The new curve
comparison uses the same feedback controller, and prominently measures cargo
position error and ground contact, even when the trolley follows the route well.
The full intended track is drawn before motion begins. Replays retain source
snapshots and lock predictions before generating the measured trajectory.

`--scenario`, `--fps`, and repeated `--set NAME=VALUE` customize the recording.
Use `--no-media` to verify the physics without rendering. Outputs require a fresh
directory and contain the recorded states plus `comparison.json`.

## What GPT-6 repairs

The nominal model already contains cargo degrees of freedom held by a restraint.
Its missing behavior is **the restraint releasing under load**. The initial
editable artifact contains no transitions. GPT-6 can examine neutral sensor
records, request fresh experiments, edit physical parameters and add persistent
transition rules. Each prediction runs its edited model in a separate MuJoCo
simulation using only that model's own state and forces.

The patch is a validated **JSON physics model**, not arbitrary Python code. The
host accepts bounded numerical changes to existing model elements and a small
transition language. The investigator receives no hidden restraint settings,
reference diagnostics, or reference model access. See
[`WAREHOUSE_MODEL.md`](../contracts/WAREHOUSE_MODEL.md) for the exact interface.

Prepare the task without any API requests:

```sh
python -m investigation.warehouse prepare --task curve --output runs/cargo-curve-task
```

Start a fresh GPT-6 Astra investigation with the existing `.env` API configuration:

```sh
python -m investigation.warehouse run --task curve --output runs/cargo-curve-gpt6
```

This command makes API requests. The failure animation and task preparation do
not. Tool calls, artifact versions, prompts, submission, prediction locks and
evaluation results are saved in the run directory. Read the evaluation verdict
before claiming that the model repaired the behavior. Test fixtures demonstrate
that the mechanism is representable; they are not GPT-6 results.

Evaluate a supplied model separately, then render its actual prediction:

```sh
python -m investigation.warehouse evaluate --task curve --candidate path/to/model.json --output runs/cargo-curve-check
python -m simulator.cargo_demo --candidate path/to/model.json --output runs/cargo-candidate-replay
```

The candidate adds a third trajectory to the path plot and carries the exact
artifact hash. It is always labeled **Candidate prediction**. A replay alone
does not establish successful generalization. Evaluation freezes the model and
records all unseen predictions before producing their measured outcomes, with
predeclared path, heading, upright-outcome and straight-control checks.

The curve evaluator checks cargo trajectory and whether the load hit the ground,
alongside chassis path/heading and the slow control. Merely predicting the trolley's
path cannot pass while predicting that spilled cargo stayed aboard. Omit `--task curve`
to use the earlier load-shift investigation.

No live GPT-6 repair is included in the prepared failure animation.
