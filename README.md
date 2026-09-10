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

**The OpenAI investigation loop has not been built yet.** The optional stateful
component is a manually authored, reference-informed solvability check. Current
runs are development experiments, not evidence of an Astra-authored repair or
final held-out performance.

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
it is not yet the planned protected final-suite evaluation service.

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
or credentials. Start a fresh investigation context with that package when the
broker is ready; the full repository is for the builders.

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
The complete agent/broker deployment boundary still needs to be implemented.

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

## OpenAI configuration and parallel work

`OPENAI_API_KEY` is the single credential. `ASTRA_MODEL` and optional `SOL_MODEL`
select models; both use that key if the project has access. Sol is only needed
for the later comparison. See [.env.example](.env.example), the
[OpenAI quickstart](https://developers.openai.com/api/docs/quickstart) and
[model catalog](https://developers.openai.com/api/docs/models). The simulator does
not load `.env` or make API requests. The future backend must load the file or
receive exported environment variables. The real `.env` remains ignored.

**Person 1:** own car mechanics, private scenarios, observations, component
coupling and physics validation. **Person 2:** own the neutral experiment broker,
OpenAI adapter, seven public tools, candidate source versions and protected
prediction/reveal evaluation. Both agree the observation/reset contract and
verify the first actual source repair together. The remaining project scope is
in [idea1PLan.md](idea1PLan.md).
