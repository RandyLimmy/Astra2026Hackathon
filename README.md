# RealityPatch

The four-wheel MuJoCo car now runs with an **isolated, editable Python brake
component**. The car supplies rigid-body mechanics, rotating wheels, tire
contacts and collisions; the component supplies four brake torque limits and
maintains its own state. We extend the component's source, not MuJoCo's engine.

The trusted reference includes synthetic brake fade and the teammate's other
failure scenarios. Candidate predictions use nominal mechanics and their own
component state. Baseline and brake-fade comparisons are the first integrated
model-repair benchmark. The original one-axis rig remains available for fast,
controlled tests.

The **Astra investigation loop is implemented**: a fresh API session can inspect
the component, request experiments, patch its source, validate development cases,
and freeze a candidate for separate evaluation. Each run records what Astra said,
which tools it used, the exact source changes, and measured prediction errors.
Check that run's report for its outcome; having the loop implemented does not
establish that a repair succeeded.

The [first live Astra/medium experiment](docs/astra-pilot-20260910/README.md)
produced an actual stateful Python repair. Reserved-case mean error fell from
10.29 m to 2.93 m, but one case exceeded its tolerance, so the full prediction
criteria failed. The shared record includes Astra's unchanged source and results.

The optional `--developer-check` component in the simulator comparison remains a
**manually authored, reference-informed solvability check**. It is separate from
the actual Astra experiment below.

## Run an Astra investigation

The pilot uses exactly **`gpt-6-astra` with `medium` reasoning** through the OpenAI
Responses API. It requires one `OPENAI_API_KEY` with access to that model. Configure
the key in the existing ignored `.env` or the host environment; see
[.env.example](.env.example) for placeholders. The optional configuration values
must match this pilot:

```dotenv
OPENAI_API_KEY=your-project-key
ASTRA_MODEL=gpt-6-astra
ASTRA_REASONING_EFFORT=medium
```

The investigation loads `.env` itself. Values in the selected file take precedence
over shell variables; `--env-file path/to/file` selects another file. A different
model or reasoning effort is rejected. Credentials stay in the host API adapter
and are not supplied to the candidate worker. Simulator-only commands do not
load `.env` or make API requests.

```sh
# Make one small request to check the exact model and reasoning configuration.
.venv/bin/python -m investigation --check-key

# Run a fresh investigation; creates a unique runs/astra-... directory.
.venv/bin/python -m investigation

# Explicit request/time limits and a new output directory.
.venv/bin/python -m investigation --max-api-requests 12 --max-seconds 1800 --output runs/astra-pilot-001
```

The installed console entry point is `realitypatch-astra`. Defaults are 12 API
requests, a 1,800-second investigation budget, and at most 8,192 output tokens per
request. Each API request has a 180-second timeout and no automatic retry or model
fallback. Limits are checked between requests/tools; in-flight work and the final
frozen evaluation can extend the overall elapsed time. Use a new `--output`
directory for each run. Physics caching defaults to `runs/investigation-cache/`.

The broker supplies two initial measured cases and corresponding original-model
predictions. Astra then has these seven tools, with bounded attempts:

| Tool | Purpose and limit |
| --- | --- |
| `inspect_model` | Read its current source, hash, and freshly initialized own state |
| `run_experiment` | Record a hypothesis/expected observation and run a fresh measured experiment; 6 additional attempts |
| `observe_run` | Read a run belonging to this investigation by its opaque ID |
| `run_model` | Execute its current component and predict a trial; 12 attempts |
| `patch_model` | Apply a unified diff to the component and run isolated interface checks; 3 attempts, including rejected patches |
| `run_regression_suite` | Compare against the two initial development cases; 3 calls |
| `submit_prediction` | Freeze the current source and close further edits/executions |

There are at most 30 tool calls and a separate 1,800-second broker development
budget. Every experiment starts fresh and includes its full preparation/wait
history. Public controls are speed (5–30 m/s), brake strength (0.2–1), preparation
cycles (0–5), wait (0–120 s), and wall gap (10–150 m, or `null` for no wall).
The investigation pilot targets braking history; the other operator scenarios
remain available through the simulator CLI.

The exact prompts live in [system.md](investigation/prompts/system.md) and
[task.md](investigation/prompts/task.md). The runner appends the neutral component
contract and initial evidence, then saves the **exact messages sent** under the
run's `prompts/`. The remote model receives only these messages, source, and broker
results. Keep builder documents and private engine/scenario files outside its
investigation context.

Open `report.md` in the printed output directory to follow Astra's hypotheses,
stated reasons, experiments, edits, and measured outcomes. The main artifacts are:

| Artifact | Contents |
| --- | --- |
| `report.md` | Readable activity, prediction verdict, errors, source diff, and technical metadata |
| `evaluation.png` | Original/candidate/reference stopping distances, when measurements exist |
| `events.jsonl` | Visible messages, brief API-provided summaries, tool calls/results, errors, and usage |
| `metadata.json` | Exact model/effort, status, token counts, budgets, submission status, and frozen source hash |
| `prompts/` | Exact system/task messages and tool schemas sent to the API |
| `broker/versions/`, `broker/patch_attempts/`, `broker/submission/` | Saved component versions, attempted diffs, and frozen submission |
| `evaluation_criteria.json` | Prediction criteria recorded before the investigation |
| `evaluation/result.json` | Per-case outcomes, predeclared prediction checks, and separate source/runtime indicators |
| `evaluation/source.diff`, `evaluation/frozen_candidate.py` | Executable frozen source and its change from the original |
| `evaluation/predictions_locked.json`, `evaluation/cases/` | Prediction locks and full original/candidate/reference records |

All reserved predictions use the same frozen source. The evaluator saves **every
original and candidate prediction before revealing any reserved reference probe**;
only the permitted preparation timeline is available beforehand. Prediction
success requires at least a 50% reduction in mean absolute distance error,
per-case error within the larger of 1 m or 10%, a cold-control error within the
larger of 1 m or 5%, and matching collision outcomes. All reserved cases must be
unseen and have uncensored stops; previously observed cases do not become new
holdouts. Source-extension indicators are recorded separately from these
prediction criteria.

A `completed` run means the harness finished; read the report's predeclared
criteria verdict to determine predictive success. If Astra stops without
submission, the report distinguishes a host-frozen candidate from an explicit
agent submission. API failures and partial runs retain their recorded evidence
and source changes. The developer-written check is never substituted for an
Astra repair.

## Start on this Mac

The local `.venv` is installed. From this repository:

```sh
# Compare the isolated baseline component with the healthy car.
.venv/bin/python -m simulator compare baseline

# Repeated braking: include the developer-written state extension.
.venv/bin/python -m simulator compare brake_fade --developer-check

# Test recovery after the same conditioning.
.venv/bin/python -m simulator compare recovery --developer-check

# Open the live reference car (macOS requires mjpython).
.venv/bin/mjpython -m simulator view brake_fade
```

Each comparison prints its unique output directory under `runs/`. It contains
`comparison.png`, `comparison.json`, the exact preparation command/reset timeline,
and separate `candidate/`, `reference/` and optional `developer_check/` recordings.
Each recording has public observations/outcomes and private builder diagnostics.
`predictions.json` records candidate source hashes and completion timestamps
**before the reference trial starts**. This ordering is useful infrastructure;
the separate Astra investigation performs the frozen reserved evaluation
described above.

Use `--no-wall` to measure full stopping distances. With a wall, collision and
timeout stopping distances are `null`; the chart shows the observed trajectory
and contact instead of inventing a full stop. `--frames --camera chase` also
records timestamped PNGs, including conditioning. Long conditioning/recovery
runs and frame recording can take a few minutes.

## Edit and run the Python component

Start from `candidate/wheel_actuator.py`. Its initial state is empty and its
brake capacities are fixed. Keep the four functions in the
[wheel component contract](contracts/WHEEL_ACTUATOR.md): initialization, a
read-only torque-limit query, one state update per interval, and a declared
trial-reset intervention. Only numeric messages cross the worker boundary.

```sh
.venv/bin/python -m simulator compare brake_fade --candidate path/to/your_component.py
.venv/bin/python -m simulator compare brake_fade --cycles 2 --rest 30 --speed 20 --brake 0.8 --no-wall
.venv/bin/python -m simulator export-task runs/agent-source-package
```

The task export contains only `actuator.py`, `INTERFACE.md` and a neutral task
stub. It does not contain engine files, private rules, scenarios, model answers
or credentials. It is useful for inspecting the neutral component interface;
`python -m investigation` connects that interface to the working broker. The full
repository is for the builders.

The car applies nonnegative torque capacities through its wheel constraints and
reports actual signed brake torques and mean wheel speeds to the component.
Reference and candidate each evolve their own state. For preparation, the
candidate replays the exact observable command/reset timeline on its own car;
it receives neither reference temperature nor private torque diagnostics.
A reposition preserves the component's history through its explicit reset hook.

## Other car scenarios and live controls

```sh
.venv/bin/python -m simulator list
.venv/bin/mjpython -m simulator view wheel_loss
.venv/bin/python -m simulator run wet_road --frames --camera chase --counterfactual
```

| Preset | Implemented reference behavior |
| --- | --- |
| `baseline` | Healthy car, dry road, cold brakes |
| `brake_fade` | Four physical accelerate/brake cycles reduce later brake capacity |
| `recovery` | Same conditioning followed by 180 seconds at rest |
| `wheel_loss` | Front-right carrier detaches physically; calibrated 17 m/s preset |
| `wet_road` | Contact friction drops over a road patch |
| `payload` | An additional 300 kg body changes mass and inertia |
| `weak_brake` | One brake has reduced capacity; modest yaw |
| `actuator_lag` | First-order brake response, not a pure transport delay |

The comparison command can show a nominal candidate's error on these scenarios.
The editable brake component does not yet expose changes to chassis mass, tire
friction or attachment geometry; those repairs need further contracts. Lateral
disturbance is still unimplemented. Steering and tire behavior are simplified,
and arbitrary high-speed wheel releases are not guaranteed numerically stable.

Live viewer: **Space** pauses, **R** repeats retaining heat/completed faults,
**N** resets and replays the whole experiment, **Esc** closes. Double-clicking
`launch_simulator.command` also opens the reference viewer. CLI controls cover
speed, brake strength, brake onset, wall position, conditioning cycles and rest.
The browser control panel and human “inject failure” button remain future work.
See [the simulator contract](simulator/contracts.md) and
[physics implementation notes](simulator/IMPLEMENTATION.md) for recording,
reset, contact and wall-free counterfactual semantics.

## Setup and checks

Tested on Python 3.13.7, macOS arm64, MuJoCo 3.13.0. Use Python 3.12 or later:

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python -m pip install --no-deps -e .
.venv/bin/python -m pytest -q
```

`requirements.txt` delegates to the shared `requirements-lock.txt`; both
simulators use the same environment. The upstream `mujoco/` submodule is optional
for reading engine source. Execution uses the Python wheel and requires no C++
build or submodule initialization. On macOS, use `mjpython` for native viewing as
explained in the official [MuJoCo Python documentation](https://mujoco.readthedocs.io/en/stable/python.html).

The candidate worker currently requires macOS `sandbox-exec` and fails closed on
unsupported hosts. Reference-only car runs work with standard MuJoCo bindings;
Linux/Windows candidate execution needs an enforced isolation backend. See
[worker limits](component_worker/README.md). The worker cannot read the repository
or API credentials, import the engine, access the network, or create processes.
The investigation API exposes only the seven broker tools, with no remote shell
or direct filesystem tools. The local trusted host owns engine execution,
source versioning, private reference data, and evaluation.

For the more expensive reference-physics matrix:

```sh
.venv/bin/python -m simulator.validate --full --workers 4
```

## Original small rig

```sh
.venv/bin/python -m sim.cli demo --developer-check
.venv/bin/mjpython -m sim.cli view artifacts/latest/repeated.json
```

This uses `candidate/actuator.py` and the original scalar-force contract. Its
one-axis cart and geometric wall crossing remain a separate controlled
benchmark. Its stopping distances should not be compared directly with those
of the four-wheel car, which has different brake laws and contact dynamics.
The parameter-only fitted baseline currently belongs to this small rig.

## Parallel work

The Astra pilot above uses one key and its fixed model/effort configuration.
A Sol comparison remains later work. OpenAI account setup is described in the
[official quickstart](https://developers.openai.com/api/docs/quickstart).

**Person 1:** own car mechanics, private scenarios, observations, component
coupling, and physics validation. **Person 2:** own the investigation broker,
OpenAI adapter, prompts, source versions, report, and frozen evaluation. Review
actual run evidence together, separating a successful prediction from a confirmed
source extension. The remaining project scope is in [idea1PLan.md](idea1PLan.md).
