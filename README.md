# RealityPatch

A working **synthetic MuJoCo braking foundation** for the hackathon plan in
[idea1PLan.md](idea1PLan.md). A Python actuator supplies braking force to a
1,200 kg body on one slide joint. MuJoCo advances position and velocity.

This milestone runs baseline braking, repeated-braking fade, recovery after
rest, and an adequate-model control. It produces actual trajectories, a chart,
and a native 3D replay. The initial editable model runs in a separate restricted
Python process; it is never imported into the MuJoCo host.

**No Astra investigation or agent-authored repair has run yet.** The optional
stateful comparison is a clearly labeled developer-written solvability check.
All current cases are development experiments, not final holdout evaluation.

## Start on this Mac

The project-local `.venv` is already installed. From the repository directory:

```sh
.venv/bin/python -m sim.cli demo --developer-check
.venv/bin/mjpython -m sim.cli view artifacts/latest/repeated.json
```

The viewer replays the computed original prediction in **blue** and synthetic
reference in **coral**, with an amber wall marker. Space pauses/resumes; R
restarts. The scene intentionally has no physical wall collision response:
wall crossing is recorded while the underlying stopping trajectory continues.
The marker refers to the cart's model coordinate, not a detailed bumper mesh.

The command also writes [the comparison chart](artifacts/latest/comparison.png)
and per-case JSON files under `artifacts/latest/`. Generated artifacts are
ignored by Git. Get a still image of the native scene with:

```sh
.venv/bin/python -m sim.cli render artifacts/latest/repeated.json --output artifacts/scene.png
```

## Change an experiment

```sh
.venv/bin/python -m sim.cli run --speed 20 --brake 1 --wall 40 --cycles 5 --rest 60
.venv/bin/mjpython -m sim.cli view artifacts/custom.json
```

Arguments are speed in m/s, brake command in `[0, 1]`, wall distance in metres,
number of preparation braking cycles, and rest in seconds. Preparation uses
real force-driven acceleration and full stops. Every phase preserves component
state; a new experiment resets the specimen. `--normal-control` selects the
host's fixed-effectiveness reference. Use `--cycles 0` for a fresh-brake probe.
Use `run --brake 0` for coasting without braking. The `demo` comparison requires
a positive brake command for parameter fitting and uses `--recovery` (default
300 seconds) to set the recovery case's rest time.

These are **builder CLI controls**. The planned browser control panel, failure
injection, seven Astra tools, and other six car scenarios are not implemented
yet. The one-axis scene does not model yaw, sideways motion, wheel detachment,
or road traction.

## Setup on another checkout

Use Python 3.12 or later. The tested setup is Python 3.13.7 / macOS arm64 with
MuJoCo 3.13.0. Runtime and development dependencies are pinned:

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-lock.txt
.venv/bin/python -m pip install --no-deps -e .
.venv/bin/python -m pytest -q
```

The current candidate-worker isolation backend requires macOS `sandbox-exec`.
It fails closed on unsupported hosts; another OS needs an enforced container
or equivalent worker backend before candidate execution. The host-only
reference simulator itself uses standard MuJoCo Python bindings.

The official [MuJoCo Python documentation](https://mujoco.readthedocs.io/en/stable/python.html)
describes installation and the native viewer. On macOS, use the package's
`mjpython` launcher for the passive viewer, as shown above.

## What is measured

The original candidate applies a fixed maximum brake force of 9,000 N. A fresh
20 m/s stop has an analytic stopping distance of approximately 26.67 m. The
host-only reference adds a deliberately synthetic heating/cooling rule and
bounded fade curve. Repeated stops can therefore change the next stop even
when starting speed and brake command are identical.

Candidate predictions ingest **past** observed preparation velocities and
commands, reconstructing past applied force from the specified known mass and
observed acceleration. They estimate only their own state, then generate the
future probe with their own force through MuJoCo. No reference temperature or
future probe samples enter candidate rollout.

The demo fits one fixed-force coefficient on development speed traces. That
parameter-only baseline is also executed in the restricted worker and MuJoCo.
The optional developer check uses a separate approximate work-memory model,
not the private temperature state. Its purpose is to confirm that an
executable state extension can solve the development task.

The `normal_control` is a separate no-fade reference and is excluded from
parameter fitting. A stateful developer model can fail that control by adding
unnecessary history dependence; the artifact preserves its result rather than
concealing it. There is no claim that this manually authored check passes the
planned adequate-model investigation test.

## Code ownership and next handoff

| Area | Purpose |
| --- | --- |
| `candidate/actuator.py` | Small incomplete editable Python component |
| `contracts/` | Neutral numeric configuration and component interface |
| `sim/` | Host-only scene, mechanics, experiments, and builder CLI |
| `reference_host/` | Private synthetic reference, normal control, developer check |
| `component_worker/` | Persistent pure-Python worker and macOS access boundary |
| `evaluation/` | Development parameter-only baseline fitting |
| `demo/` | Chart and native replay of computed trajectories |
| `tests/` | Physics/history, numerical convergence, fitting, and isolation checks |

Person 1's mechanics foundation is ready for integration. Person 2 can now
build the neutral experiment broker and seven-tool agent adapter around these
contracts. Start a fresh investigation context; do not pass this repository or
its planning documents to the blinded agent. Export the minimal source package:

```sh
.venv/bin/python -m sim.cli export-task --output artifacts/task-package
```

The export contains only `actuator.py`, `INTERFACE.md`, and a neutral task stub.
It is a handoff fixture, not a connected agent environment. The worker has
denied-access tests for host files, engine imports, credentials, writes,
network access, and process creation; see [its limits and supported backend](component_worker/README.md).

Next work is the API-backed investigation, prospective experiment records,
development regression service, frozen final-suite predictions and reveal,
and the operator UI. The local `.env` remains ignored; this physics milestone
does not read it or make OpenAI API requests.

Only **one API key** is needed: `OPENAI_API_KEY`. `ASTRA_MODEL` and the optional
`SOL_MODEL` select model IDs for the planned investigator and comparison runs;
they are not credentials. Both use the same key when its API project can access
the selected models. Sol is unnecessary for the first Astra-only loop. See
[.env.example](.env.example), the official
[API quickstart](https://developers.openai.com/api/docs/quickstart), and
[model IDs](https://developers.openai.com/api/docs/models). The future backend
must explicitly load `.env` or receive exported environment variables; merely
creating the file does not connect the current simulator to OpenAI.
