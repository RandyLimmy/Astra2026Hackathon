# Astra and Sol control-task tooling

The active testing scope is **robot dog, drone delivery, warehouse bend, and car
braking**. Both models receive tools to inspect the actual failed task, edit a
bounded controller and measure the complete task again. They must complete the
original goal with the original world and deadline. The nineteen older
maintenance presets remain available as historical workflows below.

| Platform | Exact scenario | Goal and editable control |
| --- | --- | --- |
| `quadruped` | `quadruped_gait_failure` | Follow the requested pace transition upright along the strip. Edit the rear-leg cadence response. |
| `drone` | `drone_delivery_imbalance` | Carry and release the parcel at B, return unloaded and land at A. Edit bounded flight-feedback and integral-trim settings. |
| `warehouse` | `warehouse_curve_demo` | Complete the fixed 90-degree trolley route with its cargo retained. Edit route speed, acceleration and lookahead. |
| `car` | `car_auto_brake_failure` | Stop in the marked zone before the wall without collision. Edit brake-trigger position and normalized brake command. |

The warehouse vehicle is a synthetic wheel-driven trolley. Controller edits
cannot change the load, restraint, route, deadline or physical model. The dog
cannot lower the required speed schedule; the drone cannot skip release or the
return flight; stopping the trolley does not count as delivery. Car braking
retains the declared approach speed, wall, vehicle and physical conditions.

The car controller task uses a declared 22 m/s approach after the same four
25 m/s preparation cycles, a wall at 100 m, a front-bumper target zone of
86–98 m, and a 12 s deadline. This lower approach speed makes a controller-only
repair physically feasible; the retained legacy preset still uses 25 m/s.
The original preview, development trials and fresh final verification all use
the same declared task fixture. Editable fields are `brake_trigger_x_m`
(2–85 m) and `brake_command` (0.1–1.0); these cannot alter physical braking
capacity, the approach speed, the wall or the target zone.

## Preview only

These commands record the initial editable controller's physical task without
loading API credentials or calling Astra or Sol:

```sh
.venv/bin/python -m investigation.task_run --scenario quadruped_gait_failure --preview-only --output runs/preview-control-quadruped
.venv/bin/python -m investigation.task_run --scenario drone_delivery_imbalance --preview-only --output runs/preview-control-drone
.venv/bin/python -m investigation.task_run --scenario warehouse_curve_demo --preview-only --output runs/preview-control-warehouse
.venv/bin/python -m investigation.task_run --scenario car_auto_brake_failure --preview-only --output runs/preview-control-car
```

Use these fixed output IDs for dashboard previews; each directory must be new.
Already recorded previews are available immediately when the dashboard opens.
Preview metadata says `control_task_preview`, with zero API requests. It contains
only the original incident/baseline, with no model correction or final model
verification. The drone's initial editable controller is a public-feedback port
of the original flight policy; its capability record describes this provenance
rather than claiming a bit-identical stock-controller replay.

`--no-frames` is available for telemetry-only local work, but removes camera
images and dashboard footage. Keep frame recording enabled for the planned
model comparison.

## Run all four pairs concurrently

These commands make live API calls; opening saved records does not:

```sh
# Eight sessions: all four tasks concurrently, with Astra and Sol concurrent within each task.
.venv/bin/python -m investigation.task_batch --output runs/control-batch-001

# Explicit common limits, applied independently to every model session.
.venv/bin/python -m investigation.task_batch --output runs/control-batch-002 --max-api-requests 16 --max-seconds 1800
```

Each pair uses **`gpt-6-astra` / `max`** and **`gpt-5.6-sol` / `max`**, separate
API conversations and child processes, independent controller versions, fresh
physical simulations and separate run directories. The four pair supervisors
start before the batch waits for any one pair. Each pair starts both model
children before waiting. Models share the same task/evidence contract and limits
for their scenario; neither receives the other's messages or fixes.

The selected profiles pin the model and reasoning effort. Configure the existing
ignored `.env` or host environment as follows; the key needs access to both models:

```dotenv
OPENAI_API_KEY=your-project-key
ASTRA_MODEL=gpt-6-astra
ASTRA_REASONING_EFFORT=max
SOL_MODEL=gpt-5.6-sol
SOL_REASONING_EFFORT=max
```

The actual API setting is `max`. Conflicting selected-profile settings are
rejected; there is no silent model or effort fallback. Default limits are 16 API
responses, 1,800 seconds of investigation time and 16,384 output tokens per
response, independently for each model. Limits are checked between operations;
in-flight requests and final verification may extend elapsed time. The API
adapter has a 180-second request timeout and no automatic retry.

Use a new output directory each time. The dashboard's **Run all 4 · Astra + Sol**
button starts the same batch and shows each task and model's status. It blocks
additional model launches while a batch is active. Individual pairs remain
available from the terminal:

```sh
.venv/bin/python -m investigation.task_pair --platform quadruped --scenario quadruped_gait_failure --output runs/dog-control-pair-001
.venv/bin/python -m investigation.task_pair --platform drone --scenario drone_delivery_imbalance --output runs/drone-control-pair-001
.venv/bin/python -m investigation.task_pair --platform warehouse --scenario warehouse_curve_demo --output runs/warehouse-control-pair-001
.venv/bin/python -m investigation.task_pair --platform car --scenario car_auto_brake_failure --output runs/car-control-pair-001
```

For one model only, use `investigation.task_run` with the exact `--scenario`, a
fresh `--output`, and `--profile astra-max` or `--profile sol-max`.

## Model-facing tools

| Tool | What the model can actually do |
| --- | --- |
| `inspect_system()` | Read the goal, predeclared task criteria, available cameras, public measurements and bounded controller scope. |
| `inspect_controller()` | Read the complete current controller JSON, exact hash, public controller source and allowed bounds. |
| `observe_run(id, start_s, end_s, max_samples)` | Read public telemetry and observed events from an owned recording, with 2–80 samples per read. |
| `view_frames(id, times_s, camera)` | Receive actual unannotated RGB image inputs for up to four recorded times and a declared camera. A filename alone is not image evidence. |
| `replace_controller(controller_json, expected_sha256, rationale, expected_effect)` | Install a complete validated controller using its current hash. Record exact changed values and the stated purpose. |
| `run_trial(rationale, expected_observation)` | Execute the entire original task with the current controller and fresh original conditions; record motion, images and measured outcomes. |
| `submit_result(diagnosis, evidence, remaining_uncertainty)` | Freeze the current controller for a fresh final host trial; preserve the model's actual submission and uncertainty. |

Each session allows at most 40 tool calls, six controller-edit attempts, six
additional full trials and eight image reads. Rejected attempts consume their
budgets. Controller validation rejects unknown or missing fields, unsupported
values and stale hashes. Reinstalling an identical controller is recorded as
unchanged. Models have no remote shell, arbitrary file reader, repository access
or tool for changing the task/world.

The host records complete original and final trials; intermediate experiments
also run the full task. The frozen final controller is evaluated in a new
simulation, independent of the development runs. Controller JSON is a bounded
set of real feedback-control settings, not an arbitrary executable program or a
replacement physics prediction.

## Outcomes, recordings and fairness

Before any model request, `broker/verification_plan.json` records the original
goal and criteria. Overall success requires the complete task, not merely a
stable chassis or a predicted outcome. Dog checks include the requested pace,
progress, upright posture and walking-strip limits. Drone checks include the
physical delivery/release and settled return landing. Warehouse checks include
route progress, endpoint and heading, lane tracking, upright motion and cargo
retention without floor contact throughout the fixed trial. Car checks require a
settled stop before the wall, measured clearance, and no collision during the
unchanged braking task.

`goal_achieved`, `partial_success`, and the measured summary remain separate.
`predictive_success` is `null` for these controller tasks. A completed process
only means the harness finished. An explanation is a model statement, an accepted
edit is an executed change, and a passing final trial is a measured success.
Both models receive identical instructions and the same bounded no-tool-action
reminder, so describing a fix without attempting it remains visible.

The dashboard presents:

1. Original task footage and its actual outcome, available before any API run.
2. Astra and Sol checkpoint tracks with rationale, exact controller changes,
   rejected operations, JSON diffs and subsequent measured results.
3. Original/Astra/Sol footage synchronized by elapsed simulation time, plus final
   success/partial/failure, action counts, time, tokens and recorded cost estimates.

Short or failed clips visibly end; no footage is invented or stretched to imply
completion. Saved comparison lists include only these new controller tasks.

| Artifact | Purpose |
| --- | --- |
| `runs/<batch-id>/batch.json` | Batch status, scenario/session counts and the four concurrent pair IDs. |
| `runs/<batch-id>-<platform>/comparison.json` | Per-model status, exact model/effort and whether matched protocol checks passed. |
| `runs/<batch-id>-<platform>-astra/` and `-sol/` | Isolated model records and artifacts. |
| `metadata.json`, `events.jsonl`, `prompts/` | Exact public prompts/tools, model usage, tool calls/results and image-input evidence. |
| `story.json` | Recorded checkpoints, original/development/final replay references and actual verification flags. |
| `broker/versions/v000/controller.json`, later versions, `broker/submission/controller.json` | Original, accepted and frozen controller artifacts. |
| `broker/verification_plan.json`, `evaluation/result.json` | Criteria declared before investigation and final measured outcome. |
| `physics/<record-id>/record.json`, `broker/verification/<record-id>/record.json` | Complete public measurements, owned RGB-frame references and controller hash for each physical trial. |

Shared protocol fingerprints cover prompts, tools, initial evidence/controller,
runtime dependencies, criteria and budgets. A mismatch invalidates a claim of a
matched comparison. One batch describes eight particular runs; repeat matched
comparisons before drawing a general performance conclusion.

## Host-only solution references

Developer feasibility answers are stored separately in
[`reference_host/control_task_solutions.json`](../reference_host/control_task_solutions.json)
and [`reference_host/control_task_solutions.md`](../reference_host/control_task_solutions.md).
They distinguish validated reference adjustments from remaining uncertainty and
are not model-authored results. The seven tools cannot read those files; no answer
key is included in the public task contract, recorded model prompts or image
evidence. They must never be substituted for an Astra/Sol result. Repository
access by a human developer is separate from the remote investigator interface.

## Historical nineteen-preset maintenance workflow

The remainder preserves the earlier car/drone/robot-dog fault-maintenance broker
and its commands. These presets are outside the current four-task control batch.
Their prediction, physical-maintenance and reserved-probe semantics differ from
the complete controller tasks above. The original car code and saved experiments
remain available; warehouse control now uses the active adapter described above.

### Run it

Install the existing environment from the repository root:

```sh
.venv/bin/python -m pip install -e '.[dev]'
```

List the supported presets without starting a simulation or making API requests:

```sh
.venv/bin/python -m investigation.platform_run --list-scenarios
.venv/bin/python -m investigation.platform_run --platform drone --list-scenarios
```

Launch a matched parallel comparison with a new output directory:

```sh
.venv/bin/python -m investigation.platform_pair --platform drone --scenario drone_rotor_loss --output runs/drone-pair-001
.venv/bin/python -m investigation.platform_pair --platform car --scenario car_wheel_misalignment --output runs/car-pair-001
.venv/bin/python -m investigation.platform_pair --platform quadruped --scenario quadruped_joint_weakness --output runs/dog-pair-001
```

Each model has its own API conversation, child process, mutable simulation,
source versions and run directory. Both children start before either is awaited.
The pair records operator selections, per-side status and protocol fingerprints
in `comparison.json`; children live beside it as `<pair-id>-astra` and
`<pair-id>-sol`. The same initial public evidence, prompts, tools, source and
budgets must match before results count as a matched comparison. Neither model
receives the other's work. A failure on one side does not erase the other run.

For an individual investigation, select a profile explicitly when needed:

```sh
.venv/bin/python -m investigation.platform_run --platform car --output runs/tooling/car-001 --no-frames
.venv/bin/python -m investigation.platform_run --platform drone --output runs/tooling/drone-001 --no-frames
.venv/bin/python -m investigation.platform_run --platform quadruped --output runs/tooling/dog-001 --no-frames
.venv/bin/python -m investigation.platform_run --platform drone --profile sol-max --output runs/tooling/sol-drone-001 --no-frames
```

Use a new output directory for every investigation. Omit `--no-frames` to record
actual MuJoCo frames for dashboard replay. Each launch command makes real API
requests; listing presets or inspecting an existing report does not. The default budget is 16 model
responses and 1,800 seconds, checked between operations. In-flight operations and
final verification can extend total runtime. `--max-api-requests` and
`--max-seconds` select smaller or larger bounded budgets.

### Complete scenario coverage

The platform names are `car`, `drone`, and `quadruped`. `car` selects the
teammate's new post-crash car implementation. Use `--scenario PRESET` to choose
one of that platform's presets below. Omitting `--scenario` selects
`car_wheel_misalignment`, `drone_rotor_loss`, or `quadruped_joint_weakness`,
respectively. A preset from another platform is rejected.

| Platform | Preset | Host incident |
| --- | --- | --- |
| `car` | `car_postcrash_healthy` | Healthy post-crash control. |
| `car` | `car_steering_damage` | Reduced steering response and steering bias. |
| `car` | `car_wheel_misalignment` | Bent front-left wheel mount. |
| `car` | `car_suspension_damage` | Weakened front-left suspension. |
| `car` | `car_tire_pressure` | Front-left tire radius, compliance and grip changes. |
| `car` | `car_demo` | Demo impact with moderate steering damage. |
| `drone` | `drone_hover` | Healthy flight control. |
| `drone` | `drone_rotor_loss` | One rotor loses thrust. |
| `drone` | `drone_voltage_sag` | Supply voltage reduces available thrust. |
| `drone` | `drone_payload` | Added payload changes mass and inertia. |
| `drone` | `drone_wind` | Imposed crosswind. |
| `drone` | `drone_delay` | Delayed motor commands. |
| `drone` | `drone_demo` | Demo flight with partial rotor thrust loss. |
| `quadruped` | `quadruped_walk` | Healthy walking control. |
| `quadruped` | `quadruped_joint_weakness` | Reduced knee actuator torque. |
| `quadruped` | `quadruped_foot_slip` | Reduced grip at one foot. |
| `quadruped` | `quadruped_leg_damage` | Damaged knee spring and rest geometry. |
| `quadruped` | `quadruped_payload_shift` | Shifted onboard payload. |
| `quadruped` | `quadruped_demo` | Demo walking with severe knee torque loss. |

For example, select the demo incident explicitly:

```sh
.venv/bin/python -m investigation.platform_run --platform drone --scenario drone_demo --output runs/tooling/drone-demo-001 --no-frames
```

These 6 car, 7 drone and 6 robot-dog presets share the inspection, prediction,
source-editing, maintenance and verification tools below. Preset selection
configures the host incident; diagnostic probes remain explicit tool requests.
The demo defaults use 12 seconds of steering for the car, the full 20-second
`showcase` route for the drone, and 18 seconds of walking for the robot dog.
Healthy and predictive runs share the declared driving speed, flight-route
settings or walking controls, so those operating differences are not mistaken
for damage. `inspect_system` includes these public fixture controls.
Preset names and hidden fault settings are not included in Astra's context.
These are synthetic fault cases in the teammate's mechanics.

### What Astra can do

| Tool | Capability |
| --- | --- |
| `inspect_system()` | Discover components, nominal specifications, sensors, probes, repair actions, targets and model parameter bounds. |
| `observe_system(component_id)` | Read current public sensors, optionally alongside one component's nominal information. This does not reveal a hidden diagnosis. |
| `run_experiment(probe, duration_s, hypothesis, expected_observation)` | Run a diagnostic maneuver on the current specimen. A declared fixture reset preserves damage and completed maintenance. |
| `observe_run(id)` | Inspect a recording owned by this investigation. |
| `inspect_model()` | Read exact Python source, state, parameter output, version history and source hash. |
| `replace_model_source(source, expected_sha256, rationale)` | Install a complete Python file after stale-hash, syntax, interface, isolation and parameter checks. No unified-diff line counts are needed. |
| `restore_model_version(version_id, expected_sha256, rationale)` | Restore a previously accepted source version without losing its history. |
| `run_model(probe, duration_s, rationale)` | Execute the candidate component with independent nominal mechanics and its own predicted sensor feedback. |
| `apply_repair(action, target, rationale, expected_effect)` | Perform one declared maintenance operation on the selected component. It returns a receipt requiring verification. |
| `check_repair(probe, duration_s, rationale)` | Compare the current specimen's measured trajectory and outcome with a matching healthy probe. |
| `run_regression_suite(rationale)` | Check maintenance on two development probes, keeping completed repairs between tests. |
| `submit_result(diagnosis, evidence, remaining_uncertainty)` | Freeze the exact model and maintenance sequence before host verification on a fresh specimen. |

Probe duration is 2–20 seconds. The broker allows at most 40 tool calls, eight
additional experiments, ten candidate predictions, four maintenance actions,
four repair checks, two regression calls, and five source-edit attempts shared
between replacement and restoration. Rejected attempts count. Each tool result
returns the remaining limits; model responses have a separate request budget.

| Platform | Available diagnostic probes |
| --- | --- |
| `car` | `steering`, `braking`, `slalom`, `bump` |
| `drone` | `hover`, `maneuver`, `showcase` |
| `quadruped` | `walk`, `stand`, `turn`, `conservative`, `passive` |

### Supported maintenance

| Machine | Actions and targets |
| --- | --- |
| Car | Calibrate `steering_rack`; align, replace, or service suspension/tire on `wheel_FL`, `wheel_FR`, `wheel_RL`, or `wheel_RR`. Whole-wheel replacement restores that assembly's alignment, spring/damper and tire properties. |
| Drone | Replace a named rotor, replace the battery, unload payload, service the command link, or move out of imposed wind. Rotor order is FL, FR, RR, RL. Wind shelter is an environmental intervention. |
| Robot dog | Replace a named joint actuator, service its spring/rest geometry, replace a named foot pad, or secure the movable payload. Component IDs and motor ordering come from `inspect_system`. |

Valid maintenance on the wrong component produces the same kind of receipt as
maintenance on the damaged part. Astra has to measure the next probe to find out
whether its choice helped. Operations preserve unrelated damage and completed
fault-event guards. A probe reset is never used as an implicit repair.

This catalog covers the implemented platform faults. Whole-wheel replacement
does not add a detached-wheel reconnection model to the older braking track.
New failure mechanisms need a corresponding physical model and adapter before
Astra can test or repair them.

### Model edits and machine repairs are different

The editable component is [platform_model.py](../candidate/platform_model.py),
with its [three-function contract](../contracts/PLATFORM_MODEL.md):

```python
def init_state():
    return {}

def predict_parameters(state):
    return {}

def advance_state(state, observation, dt_s):
    return state
```

An empty parameter map retains nominal mechanics. Astra can return bounded
parameter hypotheses or add state and an evolution rule based on its own
predicted observations. The adapter applies those outputs to the independent
prediction model. The model receives an initial zero-time query before fixture
preparation and another at trial start, then feedback at approximately 50 Hz.
This diagnostic contract starts a new model state at each prediction; it does
not infer the specimen's private pre-incident state or secretly copy it.

Predictive edits do not physically fix the observed specimen. Maintenance does
not automatically repair or validate the predictive source. A static parameter
adjustment, a source extension with evolving state, and a physical replacement
are separate results; the existence of the tooling does not make any one of
them a novel research contribution.

Python runs behind the existing macOS sandbox with no key, repository, engine,
network or process-creation access. The host validates source hashes, finite
numeric state, allowed parameter keys and physical bounds. Parameter queries
cannot mutate state. Source versions are retained, and rejected edits leave the
last accepted version in place. Startup has a bounded five-second allowance;
subsequent worker operations have the usual two-second limit.

### Verification and records

The run saves exact prompts, public explanations, tool arguments/results,
component versions, maintenance receipts, predicted/observed trajectories,
budgets and source hashes. `host_setup.json` records operator selections and is
never included in Astra’s input. Read `report.md` for a compact result and
`evaluation/result.json` for the full record. These files stay in ignored `runs/`.

Before the first model request, the host declares the verification probes,
duration and criteria in `broker/verification_plan.json`. Physical success means
every declared probe runs to completion and reports its public `safe` flag.
Prediction success is reported separately: each frozen prediction must cover
the full corresponding observation and have position RMSE at most 0.5 m.
Per-probe outcomes remain visible even when the overall goal fails.
The first probe repeats the selected scenario's maneuver (including the drone
demo's full `showcase` route); the second is a distinct declared control probe.

After submission, development tools close. The host locks candidate predictions,
creates a fresh instance of the original incident, replays the same selected
maintenance actions, and measures two declared verification probes. It reports
before/after position error relative to healthy behavior, prediction error over
overlapping times, and each platform's public probe outcome. Early termination
is not extrapolated into an invented trajectory. These are repeated controlled
probes and are not necessarily unseen maneuvers or a general safety certificate.

`story.json` indexes the full original incident, actual tool checkpoints and
recorded replays. The incident preserves its original fault timing and route;
diagnostic and final comparison probes use the declared reproducible fixture.
Final playback compares the same probe and elapsed simulation time across
Original, Astra and Sol. A shorter recording ends visibly instead of inventing
frames or stretching its time axis.

The dashboard has three sections:

1. **Original scenario:** recorded incident, the task and original outcome.
2. **What each model changed:** parallel checkpoint tracks with the stated
   rationale, requested operation, exact parameter changes or source diff,
   rejection details and subsequent measurements.
3. **Results:** matching Original/Astra/Sol recordings, synchronized playback,
   physical goal and prediction outcomes, action counts, elapsed time, usage
   and recorded cost estimates when available.

Textual proposals are preserved as model statements. They do not count as
repair attempts or applied changes. The action summary distinguishes no attempt,
diagnosis without a fix attempt, rejected attempts, applied but unverified work,
and the final goal outcome. Both models receive the same tool-use instruction
and at most one identical reminder when they respond without a tool call.
This makes Sol's possible “described the fix but never attempted it” behavior
visible without awarding either model credit for prose.

A single pair describes those two runs. It does not establish a general
performance advantage. Repeat matched comparisons before making a broader
claim; compare success, actual actions, time and usage together.

The original brake-fade source-extension experiment remains available through
`python -m investigation`. It now also supports complete-source replacement
with a current hash; replacement and legacy `patch_model` share its original
three-attempt edit budget. Existing recorded Astra/Sol results remain unchanged.
