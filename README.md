# RealityPatch — MuJoCo scenarios

A synthetic car test track with seven scenario types and a recovery preset.
MuJoCo 3.13.0 runs the rigid bodies, rotating wheels, tire contact, brakes, and
wall collisions. This repository contains simulator tooling; agent integration,
model repair, and a dashboard are separate work.

## Open the live simulator

From this workspace on macOS:

```sh
.venv/bin/mjpython -m simulator view wheel_loss
```

Or double-click `launch_simulator.command` in Finder. The live viewer follows the
car. **Space** pauses, **R** repeats the trial retaining heat and completed faults,
**N** resets and replays the full experiment, and **Esc** closes it. The final pose
stays visible until you repeat or close. Heating/recovery runs prepare their
history before opening the trial; this can take a few minutes.

```sh
.venv/bin/python -m simulator list
.venv/bin/mjpython -m simulator view baseline
.venv/bin/mjpython -m simulator view brake_fade
.venv/bin/mjpython -m simulator view recovery
.venv/bin/mjpython -m simulator view wet_road
.venv/bin/mjpython -m simulator view payload
.venv/bin/mjpython -m simulator view weak_brake
.venv/bin/mjpython -m simulator view actuator_lag
```

On Linux, use `.venv/bin/python` in place of `mjpython` for the viewer.

## What each preset does

| Preset | Physical change | Expected demonstration |
|---|---|---|
| `baseline` | Healthy 1,200 kg car, cold brakes, dry road | Stops before the wall |
| `brake_fade` | Four physical accelerate/brake cycles heat the brake discs | Reduced capacity, collision |
| `recovery` | Same heating followed by 180 s rest | Cooling restores a safe stop |
| `wheel_loss` | Front-right carrier weld releases at 3.1 s | Wheel rolls away; car hits the nearer wall |
| `wet_road` | Friction drops from 1.1 to 0.32 over x = 40–120 m | Longer braking, collision |
| `payload` | Secured 300 kg cargo with physical mass/inertia | Longer torque-limited braking, collision |
| `weak_brake` | Front-left brake has 20% capacity | Longer braking, collision; modest yaw in this model |
| `actuator_lag` | 0.25 s first-order brake response | Later stop, still before the wall |

Most presets start at 25 m/s, brake at x = 45 m, and use a wall at x = 100 m.
Wheel loss uses 17 m/s and a wall at x = 70 m: that case was selected for numerical
stability, and a healthy car at the same speed clears that wall. Front-position
measurements account for chassis length and orientation.

These are deliberately controlled, synthetic phenomena. In particular, fixed
steering and simplified tires do not reproduce realistic automotive handling.
The weak-brake case produces only modest yaw; no lateral force is added to make it
look dramatic. Arbitrary high-speed wheel-release settings can be sensitive to
timestep, so use the calibrated preset for comparisons.

## Record a run

```sh
.venv/bin/python -m simulator run wheel_loss --frames --camera chase --counterfactual
.venv/bin/python -m simulator run baseline --no-wall --speed 20
```

Each invocation creates a unique directory under `runs/`:

- `public/observations.jsonl`: 100 Hz observable state and commands, including conditioning history.
- `public/frames/` and `frames.jsonl`: optional PNGs with simulation timestamps (30 Hz by default).
- `public/summary.json`: observed collision, stopping, and lane outcomes.
- `private/diagnostics.jsonl` and `summary.json`: temperatures, events, parameters, and solver checks.
- `counterfactual/`: optional evaluator-only replay with the wall removed.

Collision and timeout trajectories are censored: their stopping distance is
`null`. The wall-free replay measures a separate counterfactual stopping distance.
A stationary run without braking has no braking-distance measurement either.

Use `--output NEW_DIRECTORY` to choose a destination. Existing recordings are
protected from overwrite. `--config experiment.json` accepts an `Experiment`
object or a manifest containing `config`; explicit CLI flags take precedence.
The Python `Simulator` also supports individual steps and trial resets for custom
conditioning histories. See [the complete contract](simulator/contracts.md).

The wheel-loss recording from setup is in `runs/wheel-loss-demo/`, including
`wheel-loss.mp4`. PNG recording needs no extra dependencies. If FFmpeg is already
installed, convert another recording with:

```sh
ffmpeg -framerate 30 -i RUN_DIRECTORY/public/frames/frame_%06d.png \
  -c:v libx264 -pix_fmt yuv420p replay.mp4
```

## Validate

```sh
.venv/bin/python -m unittest discover -s tests -v
.venv/bin/python -m simulator.validate --full --workers 6
```

The first command tests mechanics, resets, timing, recording, and public-output
contracts. The second executes wall/free pairs, repeats, half-timestep checks, and
the private development/held-out matrix. Reports are written to
`runs/validation/report.md` and `report.json`; the larger run can take several
minutes. These validate the simulator, not any agent's ability to repair a model.

Existing local developer tools can additionally check:

```sh
ruff check simulator tests
mypy --python-executable .venv/bin/python simulator
```

## Handoff and environment

Clone this repository with `git clone --recurse-submodules` to include the pinned
upstream MuJoCo source. For an existing clone, run `git submodule update --init`.
The simulator itself uses the Python wheel installed from `requirements.txt`.

`simulator/public/` contains an exported nominal MJCF and observation schema, with
no hidden scenario rules. To generate another public export:

```sh
.venv/bin/python -m simulator export-baseline NEW_DIRECTORY
```

Share only that export and permitted `public/` recordings with an agent.
Directory names do not enforce isolation: deployment must restrict the agent's
filesystem/process access to the hidden simulator and private holdouts. This CLI
is for the trusted operator, not an agent-facing endpoint.

The engine source is cloned, unmodified, in `mujoco/` at release `3.13.0`, commit
`123347c0eeab7e13c8da0828ab593bbd95bcf335`. `.venv/` uses Python 3.11 and the
precompiled native engine; no C++ build or new dependencies were needed.

```sh
uv venv --python 3.11 .venv
uv pip install --python .venv/bin/python -r requirements.txt
```

Original scope: [.omx/plans/simulator-scenarios.md](.omx/plans/simulator-scenarios.md).
[Implementation decisions](simulator/IMPLEMENTATION.md) explain changes made
when physical tests contradicted the initial sketch.
