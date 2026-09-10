# Comparing the two simulator branches

**Integration update:** the branches are now combined. The car executes the
isolated `wheel_v2` Python component, replays an exact public preparation timeline,
and saves candidate predictions before running the reference trial. The
scheduled-fault completion bug below is fixed. See [README.md](README.md) for
current commands. The remainder records the pre-integration review and rationale.

Reviewed on September 10, 2026:

- `origin/main`: `4fce806` — the teammate's car/scenario simulator.
- `feat/mujoco-braking-foundation`: `13af8bb` — the editable-component foundation.

**Recommendation: use the teammate's car mechanics and recording tools as the
next host simulator, retaining our isolated Python component and model-evaluation
architecture.** First connect baseline and brake fade; keep the other scenarios
available for later expansion. Preserve our small rig as a fast controlled
benchmark while adapting the richer car. This review does not merge the branches.

## What each branch contributes

| Area | Teammate's `main` | Our foundation |
| --- | --- | --- |
| Mechanics | Free-body chassis, rotating wheels, tire contact, real wall collision, payload mass/inertia | One-axis 1,200 kg cart, applied braking force, geometric wall crossing |
| Scenarios | Baseline, brake fade, cooling recovery, wheel loss, wet road, payload, weak brake, first-order actuator lag | Baseline, fade, recovery, and a no-fade control |
| Viewing and evidence | Live native viewer, timestamped telemetry/PNG recordings, censored collision outcomes and separate wall-free counterfactuals | Side-by-side native replay, comparison chart, prediction/reference JSON |
| Editable source | Brake capacity and heat update are built into the trusted simulator | Small editable Python actuator executed through an isolated worker |
| Candidate evaluation | No candidate model or repair loop yet | Original model, parameter-only fit, manually written stateful solvability check |
| Isolation | Public/private export conventions; deployment boundary still needed | Restricted macOS component process; complete investigation-agent environment still needed |
| OpenAI integration | None | None; example configuration only |

The branches have different starting speeds, brake laws, geometry and stopping
definitions. Their stopping-distance numbers cannot be compared as a score of
which implementation is better. Both remain synthetic experiments.

Eight presets on `main` include recovery. They do **not** cover all eight requested
scenario types: lateral disturbance remains absent, and its actuator-lag model is
a first-order response rather than a pure transport delay. The implementation
notes also identify modest weak-brake yaw and a calibrated wheel-release preset;
avoid claiming realistic vehicle handling or arbitrary wheel-loss stability.

## Confirmed bug in the new simulator

[`trial_complete`](https://github.com/RandyLimmy/Astra2026Hackathon/blob/4fce806/simulator/runner.py#L224)
waits for future pedal commands but ignores pending fault events. Reproduction:
`Experiment(initial_speed=0, wall=False, duration=2, detach_wheel="FR", detach_at=1)`.
The run finishes at 0.5 seconds, emits no event, and leaves the front-right wheel
attached. Completion should account for pending configured wheel-release and
brake-failure events within the trial horizon. This was reproduced separately
from the passing tests; the review leaves the teammate's code unchanged.

## Integration work that matters

1. **Add a component hook to the car's controller.** In
   [`Simulator.step`](https://github.com/RandyLimmy/Astra2026Hackathon/blob/4fce806/simulator/runner.py#L138),
   brake capacity and thermal evolution are currently computed directly from
   private host state. Reference execution can retain that implementation.
   Candidate execution must obtain its brake behavior from the editable worker,
   maintain its own state, and run through the same mechanics without applying
   the reference's hidden fade a second time.
2. **Agree a versioned numeric interface before connecting the worker.** Our
   current component returns one force in newtons; the car uses four brake torque
   capacities in newton-metres. Define wheel order, units, observations, solved
   torque/work feedback, state-update timing and reset behavior. Do not equate
   torque capacity with actual dissipated torque. Our one-axis reconstruction of
   past braking force from mass and acceleration does not identify each wheel's
   brake torque in a contact/slip model. If adding a torque sensor, document it
   and give both evaluated agents the same observation access. Alternatively,
   replay the exact public command and reposition timeline through the candidate
   mechanics and update from its own solved torques. Never use private reference
   diagnostics as undeclared preparation observations.
3. **Keep operator controls and engine files on the host.**
   [`Experiment`](https://github.com/RandyLimmy/Astra2026Hackathon/blob/4fce806/simulator/config.py#L9)
   includes hidden mechanism settings such as thermal fade, payload and failure
   timing. The operator may use them; the agent's experiment interface must
   expose only permitted controls. The current
   [public MJCF export](https://github.com/RandyLimmy/Astra2026Hackathon/blob/4fce806/simulator/public/README.md)
   names MuJoCo and supplies engine assets. Keep it as a builder asset for our
   chosen blinded, Python-only investigation. Export the neutral component,
   interface and approved observations to the agent instead.
4. **Unify history and outcome semantics.** Preserve conditioning/recovery
   observations and declared resets. Use the same initial conditions and
   permitted history for candidate and reference. The recorder's 100 Hz output
   is not the same as the 500 Hz physics/control timeline; preserve exact command
   and reset events separately. The car resets activation while retaining heat:
   define that intervention without indiscriminately clearing candidate state.
   For the car, a collision or
   timeout has no measured full stopping distance; retain the teammate's
   censoring rules and keep wall-free counterfactuals evaluator-only. Adapt our
   plots and error metrics to these rules.
5. **Connect the seven planned tools and freeze/reveal evaluation.** Neither
   branch implements an API-backed investigation or agent-authored repair.
   Simulator validation and a developer-written stateful model are separate
   checks. They do not establish the research contribution.

## Suggested parallel ownership

**Person 1 / teammate:** own `simulator/`, the physical scene, private scenarios,
recordings and physics validation. Implement the controller hook and shared
observation/reset contract, starting with baseline and brake fade.

**Person 2 / you:** own the isolated candidate worker, neutral experiment broker,
OpenAI adapter, seven public tools, source-version tracking, and prediction lock
before reveal. Start the Astra-only loop with fake broker fixtures while Person 1
connects the car. Add the Sol comparison after the first working repair run.

Together, agree the torque/state contract first, then verify that the untouched
candidate matches the healthy car and that a clearly labeled developer state
extension can predict prepared/recovered cases through the new hook.

## Checks performed and merge preparation

- All **31 tests** in the teammate's branch passed when run from a separate
  checkout using our existing environment: Python 3.13.7, MuJoCo 3.13.0,
  NumPy 2.5.3. Its own requirements pin NumPy 2.4.6; this was a compatibility
  check, not a reproduction of every pinned dependency.
- Our foundation previously passed **38 tests and 17 subtests**. Its executable
  implementation was unchanged during this review.
- The longer `simulator.validate --full` matrix was not rerun in this review.
- A merge preview found add/add conflicts in `README.md` and `pyproject.toml`.
  The source directories otherwise coexist, but resolving those two files alone
  would still leave two independent simulators.
- Consolidate `requirements.txt` and `requirements-lock.txt`. Preserve our
  package configuration and the teammate's lint/type-check settings; explicitly
  include `simulator` and its required assets/presets if packaging the combined
  project. Keep both `runs/` and `artifacts/` ignored.
- `main` adds an upstream MuJoCo submodule; execution uses the installed Python
  wheel. The source checkout is not required to run the simulator and must not
  be included in the investigation package.

## One key, optional model selections

`OPENAI_API_KEY` is the only credential. `ASTRA_MODEL=gpt-6-astra` and
`SOL_MODEL=gpt-5.6-sol` identify which model a future request uses; both can use
the same key if the API project has access. Sol is for the planned comparison,
not a requirement to get the first Astra loop running. See the official
[OpenAI quickstart](https://developers.openai.com/api/docs/quickstart) and
[model catalog](https://developers.openai.com/api/docs/models).

`.env.example` now explains this and comments out the optional Sol setting.
The real `.env` remains ignored and was not opened or changed in this review.
Neither branch currently loads it or makes API calls. Key validity and model
access have not been tested here.
