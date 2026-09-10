# Astra experiment and repair tools

The teammate's improved car, drone and robot dog are integrated with an Astra
tool broker. The model can inspect public measurements, run diagnostic probes,
edit an isolated Python prediction component, perform supported maintenance, and
verify the result. The new workflow uses **gpt-6-astra with extra high (`xhigh`) reasoning** and
the existing `OPENAI_API_KEY`. Frontend changes and further Sol work are deferred.

## Run it

Install the existing environment from the repository root:

```sh
.venv/bin/python -m pip install -e '.[dev]'

.venv/bin/python -m investigation.platform_run --platform car --output runs/tooling/car-001 --no-frames
.venv/bin/python -m investigation.platform_run --platform drone --output runs/tooling/drone-001 --no-frames
.venv/bin/python -m investigation.platform_run --platform quadruped --output runs/tooling/dog-001 --no-frames
```

Use a new output directory for every investigation. Omit `--no-frames` to record
actual MuJoCo frames for later replay work. The commands above each make real API
requests; inspecting an existing report does not. The default budget is 16 model
responses and 1,800 seconds, checked between operations. In-flight operations and
final verification can extend total runtime. `--max-api-requests` and
`--max-seconds` select smaller or larger bounded budgets.

The host can select another supported case with `--scenario PRESET`, for example
`--scenario car_tire_pressure`, `drone_voltage_sag`, or `quadruped_leg_damage`.
Preset names and hidden fault settings are not included in Astra's context.
The host defaults exercise wheel misalignment, rotor thrust loss and a weakened
robot knee, respectively. These are synthetic fault cases in the teammate's
mechanics, not real hardware.

## What Astra can do

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

## Supported maintenance

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

## Model edits and machine repairs are different

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

## Verification and records

The run saves exact prompts, public explanations, tool arguments/results,
component versions, maintenance receipts, predicted/observed trajectories,
budgets and source hashes. `host_setup.json` records operator selections and is
never included in Astra’s input. Read `report.md` for a compact result and
`evaluation/result.json` for the full record. These files stay in ignored `runs/`.

After submission, development tools close. The host locks candidate predictions,
creates a fresh instance of the original incident, replays the same selected
maintenance actions, and measures two declared verification probes. It reports
before/after position error relative to healthy behavior, prediction error over
overlapping times, and each platform's public probe outcome. Early termination
is not extrapolated into an invented trajectory. These are repeated controlled
probes and are not necessarily unseen maneuvers or a general safety certificate.

The original brake-fade source-extension experiment remains available through
`python -m investigation`. It now also supports complete-source replacement
with a current hash; replacement and legacy `patch_model` share its original
three-attempt edit budget. Existing recorded Astra/Sol results remain unchanged.
