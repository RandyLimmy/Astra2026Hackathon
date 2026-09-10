# Simulator interface and recordings

This simulator supplies controlled synthetic car experiments, MJCF, telemetry, camera recordings, and comparisons with an isolated editable Python brake component. It does not yet implement an API-backed agent, automated missing-physics discovery, or a dashboard. Its simplified rigid car is not a validated vehicle safety model.

## Editable component integration

`python -m simulator compare brake_fade --developer-check` runs the four-wheel
component through the same car mechanics. `--candidate PATH` selects another
version-two source file. See [the neutral contract](../contracts/WHEEL_ACTUATOR.md).
The scalar component used by the original `sim/` rig is a separate interface.

`Simulator(config, actuator=worker)` accepts a `WheelActuatorWorker` with
`torque_limits`, `advance`, `reset`, and `reposition`. Its capacities replace
the built-in brake controller. The host solves actual braking torque, advances
mechanics and updates the component once per timestep. The comparison wrapper
constructs candidate mechanics from an allowlist of public controls, excluding
private fault parameters. Direct `Simulator` construction is a trusted-host API.

`prepare(callback=None, history=None)` records `preparation_history`: ordered
reset events and run-length-encoded command steps, including exact timestep,
phase and count. Supplying that history replays the same interventions using
the candidate's own mechanics and forces. `run(preparation_history=...)` prepares
and runs; `run(prepared=True)` runs the trial after explicit preparation. These
two options are mutually exclusive. Repositioning invokes `on_trial_reset`;
only a full specimen reset invokes `init_state` to discard long-term state.

The comparison command prepares the reference, records each candidate prediction
and source snapshot, then runs the prepared reference trial. Each component has
its own state and receives no private reference torque/state. Candidate private
summaries include its own `component_state`; unused reference-temperature fields
are omitted. Collision/timeout distances remain censored. These are development
records, not the protected final-suite service. The optional developer component
is explicitly reference-informed and manually authored.

`python -m simulator export-task NEW_DIRECTORY` exports Python and its neutral
contract without engine assets. The existing `export-baseline` MJCF export is
for builders and must not be supplied to the blinded investigation agent.

## Operator commands

Run commands from the workspace root with the existing Python environment:

```sh
.venv/bin/python -m simulator list
.venv/bin/python -m simulator run baseline --frames --counterfactual
.venv/bin/python -m simulator run brake_fade --cycles 3 --rest 60 --no-wall
.venv/bin/python -m simulator run wheel_loss --camera chase --frames
.venv/bin/python -m simulator export-baseline runs/nominal-export
```

The scenario name selects a private operator preset. `--config path.json` overlays a JSON Experiment object, or the `config` object of a saved manifest, on that preset. Explicit CLI physics flags override that JSON. Available flags are `--speed` (m/s), `--brake` (0–1), `--brake-at` (leading bumper x in metres), `--duration` (seconds), `--wall-x` (metres), `--no-wall`, `--timestep` (seconds), `--cycles`, and `--rest` (seconds). A config's `commands` list contains strictly increasing `[trial_time, throttle, brake]` triples and overrides position-triggered commands; the last command remains active until replaced.

`run` defaults to a unique directory under `runs/`. Use `--output DIR` to choose one. Existing recording files cause an error, protecting earlier runs. `--frames`, `--camera overview|side|chase`, and `--fps 30` control optional PNG capture; default recording is telemetry only. Rendering samples actual physics states and does not advance dynamics. Sampling occurs at the first physics tick on or after a sample deadline; gaps never fabricate observations.

`--counterfactual` replays the same declared experiment without the wall into `DIR/counterfactual/`, outside the main public export. It is an evaluator-only paired experiment, not the uncensored continuation of a collided trajectory. Do not expose this folder to the evaluated agent.

## Interactive viewer

On macOS, launch the viewer through MuJoCo's main-thread wrapper:

```sh
.venv/bin/mjpython -m simulator view baseline
.venv/bin/mjpython -m simulator view wheel_loss --speedup 0.5
```

On other systems, `.venv/bin/python -m simulator view baseline` is sufficient. The viewer accepts the same physics flags as `run`; `--speedup` changes wall-clock playback pacing only. Conditioning/recovery is simulated before opening the trial window. The free camera tracks the chassis; the standard MuJoCo mouse controls change its angle and distance.

Space pauses/resumes. **R** repositions and repeats the current trial while retaining heat and completed faults, including detached wheels. **N** rebuilds the original experiment, restores its initial fault state and ambient temperature, and replays its declared conditioning/recovery. Escape closes the viewer. A finished trial holds its final pose until a reset or close. Viewer runs do not automatically save telemetry; use `run --frames` for recorded experiments.

## Python execution

`Experiment` in `config.py` validates configuration and uses SI units, with temperature in degrees Celsius. `Simulator(config).run(callback)` prepares conditioning/recovery, runs the trial, and returns a private summary with a separately filtered `public` summary. The optional callback receives `(simulator, phase)` at initial phase boundaries and every physics step. Read `simulator.observe(phase)` for public sensor data and `simulator.diagnostics()` only for evaluator diagnostics. No global MuJoCo callbacks are installed.

`reset_full(config)` rebuilds the specified experiment from ambient state; `reset_full()` restores healthy defaults. `reset_trial(speed)` repositions the car and motion while retaining temperature, brake efficiency and released attachments. It explicitly resets brake activation to zero. Warm-up and recovery are declared history, and repositioning starts a new `phase_time`; experiment `time` remains monotonic. To replay a full configured experiment, construct a fresh Simulator and call `run` or `prepare`.

`Recorder(output_dir, model, frames=False, camera='overview', fps=30, width=960, height=540)` accepts `record(data, observation, private_state)` every physics tick. It samples observations/diagnostics at 100 Hz and frames at `fps`. Supply `observation['time']` in global experiment time across trial resets. `finish(private_summary, public_summary)` writes the separate summaries and closes resources. The context manager and `close()` flush partial recordings on interruption. Recorder trusts the caller to supply filtered public dictionaries; it never merges diagnostic fields into them.

## Public sensor export

Only `DIR/public/` is intended for the evaluated agent:

| File | Meaning |
|---|---|
| `observations.jsonl` | Public observations, normally 100 Hz |
| `frames.jsonl` | Actual experiment timestamp and relative PNG path, plus phase |
| `frames/frame_000000.png` | Optional RGB camera frames |
| `summary.json` | Observable trial outcomes, excluding hidden configuration and state |

The fixed observation fields are `time`, `phase`, `phase_time`, `position`, `velocity`, `yaw`, `yaw_rate`, `wheel_speed`, `throttle`, `brake`, `front_x`, `wall_contact`, and `lane_departure`. Position is chassis world position in metres; velocity is world translational velocity in m/s; yaw is radians; yaw_rate is the world heading derivative in rad/s, or null when heading is vertical and yaw is undefined. Wheel angular speeds are rad/s in FL, FR, RL, RR order. A declared conditioning phase is named `conditioning_N`; other phases are `recovery` and `trial`.

`front_x` measures the leading x extent of the oriented chassis collision box, not chassis center. Lane departure is chassis center exceeding 3.2 m lateral displacement. Wall contact tracks chassis collision rather than a loose wheel striking the wall. Public summaries include contact/impact speed, stopping status, censoring, brake onset, braking distance, final pose-derived metrics, wall clearance, maximum yaw/lateral displacement, and trial duration. A stop requires speed below 0.1 m/s for 0.5 s; a resting car continues until any remaining scheduled commands have executed, subject to duration and impact limits. A stationary case with no braking onset has no braking-distance measurement (`stopping_distance: null`). Collision and timeout outcomes have `censored: true` and `stopping_distance: null`.

`export-baseline OUTPUT` creates a self-contained nominal `model.xml`, `observations.schema.json`, and a public README documenting the nominal wheel-drive/brake controller. It includes no private manifests, hidden update code, or failure rules. The MJCF is a physical asset; pedal schedules and brake-capacity updates require a controller.

## Private evaluator records and isolation

`DIR/private/diagnostics.jsonl` retains temperatures, brake activation and torque, efficiencies, attachment state, separation, contact friction and solver warnings. `DIR/private/summary.json` retains configuration, events, temperatures, solver/version evidence and the outcome summary. Preset names and hidden parameters belong to the trusted operator interface and must not become agent-facing observation metadata.

The public/private folder split is an export convention, **not a security boundary**. An agent with unrestricted access to the workspace could read both. Integration must enforce separate process/account/container access and expose only the agreed public bundle. Conditioning commands and motion must be supplied to both evaluated agents, so hidden heat is not inferred from unavailable prehistory. A physical wheel loss is not an inherently predictable future event: evaluate motion after observation or after a declared intervention.
