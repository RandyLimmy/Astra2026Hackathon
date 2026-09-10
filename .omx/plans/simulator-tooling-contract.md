# Proposed simulator tools and controller contract

Status: **design specification, not an installed API**. Companion to [the scenario plan](simulator-scenarios.md). Every `sim_*` name below is proposed. Existing callable commands are identified separately at the end. No corrected controller or autonomous repair loop is supplied here.

Existing-source references describe baseline commit `803fa7f`, inspected before concurrent workspace edits; they do not assert the status of those later changes.

## Purpose and ownership

Expose useful operations so GPT-6 Astra can inspect the nominal engine and a failed animation, choose probes, write controller changes, and evaluate actual behavior. The host executes deterministic physics; the caller decides what to investigate or change. There is no `fix_scenario`, `auto_balance`, or hidden search routine.

Use a thin Python dispatcher plus machine-readable schemas over the existing platform/runner interfaces. Its contract is independent of a particular model transport. A later Astra adapter can register these functions and attach their visual evidence; launching that adapter is outside this planning pass. Preserve the seven existing braking investigation tools and do not silently rename or extend their semantics (`investigation/broker.py:39`).

## Common schema rules

- Publish a schema version, tool version, stable tool name, purpose, complete JSON input/output shape, units, limits, side effects, reset semantics, errors, and one success/error example per tool.
- Input objects reject unknown keys. Finite numbers only; booleans are not numeric values. Public IDs are opaque and scoped to one task/session. No arbitrary host paths or private preset names in agent requests.
- All time fields are simulation seconds and distinguish `experiment_time_s` from `trial_time_s`. APIs name units in field names or in their schema. Control intervals are explicit.
- Every result has `ok`, `request_id`, `schema_version`, and exactly one of `data` or `error`. Mutations return version/ID receipts; reads do not advance physics or consume a new experiment attempt.
- A run record includes task version, opaque world/fixture fingerprint, start-state fingerprint, controller hash, engine/build version, timestep, sensor contract version, mission/probe type, command history, recording metadata and terminal status. Do not publish private configuration behind a fingerprint.
- Duplicate mutating requests with the same `request_id` return the original receipt; a duplicate ID with different arguments is rejected. Expensive operations publish limits through discovery. Errors never silently substitute nominal controllers, a healthy world or a different task.
- Separate `completed`, `task_failed`, `controller_error`, `numerical_error`, `timeout`, and `interrupted`. A physical crash is an observable task failure; solver divergence is an engine error. Partial records remain retrievable and explicitly incomplete.

Proposed initial bounds: 60 simulated seconds per run; physics step fixed by the task; 100 Hz controller ticks unless the task specifies another value; up to 100 scheduled interventions; up to 2,000 observation rows per page; up to 12 frames per visual response; image sizes up to 1280×720. Declare compute/run/edit limits in discovery instead of copying the brake pilot's three-patch limit onto every platform. Calibrate resource limits before release; do not advertise these as measured throughput.

## Proposed tool catalog

Names and principal fields below define the intended public interface. Before implementation handoff, publish full JSON schemas for nested types and response fields in the proposed `simulator/public/tools.schema.json`.

| Proposed tool | Inputs | Returns | Effect / limit |
|---|---|---|---|
| `sim_list_tasks` | Optional platform filter | Public task IDs/titles, versions, availability, documentation IDs and resource limits | Read; titles describe missions, not hidden causes |
| `sim_read_document` | `document_id`, optional `version`, `section`, `cursor`, `limit` | Public document/schema content, content hash/version, section index and next cursor | Read; only indexed public bundle; no arbitrary paths; proposed limit 16,000 characters/page |
| `sim_describe_task` | `task_id` | Objective, fixed route/speed schedule, observation contract, allowed probes, immutable conditions, completion criteria, initial failed-run ID | Read; explicitly lists what controller may change |
| `sim_inspect_engine` | `task_id`, `section`, optional `component_id` | Nominal topology/model excerpt, bodies/joints/actuators, frames, physical units, limits, timing, nominal force/kinematics mapping, documentation refs | Read-only nominal engine; sections are an enum, no arbitrary source path |
| `sim_describe_controls` | `task_id` | Platform command schema, order/signs/ranges, defaults, saturation and hold behavior, controller interface version | Read; distinguishes direct actuation from built-in feedback helpers |
| `sim_inspect_controller` | `task_id`, `controller_id` | Exact allowed source/configuration, hash, interface, editable files/fields, initialized own state, validation status | Read; cannot inspect private fault logic |
| `sim_validate_controller` | `task_id`, candidate `source`, candidate `parameters` | Parse/interface/range/runtime-check report and bounded errors | Sandbox validation only; no world run, no install, no completion claim |
| `sim_patch_controller` | `task_id`, `controller_id`, `base_hash`, `diff`, `rationale`, `request_id` | New immutable controller ID/hash, accepted diff and validation receipt | Applies only allowed controller/config files; stale/invalid edits leave prior version intact |
| `sim_create_experiment` | `task_id`, `mode`, `probe_id`, optional typed `probe_parameters`, `controller_id` or `controls`, `capture`, `request_id` | Experiment ID, resolved public settings, initial fixture fingerprint, limits | Allocates a trial; mission settings immutable; exactly one control source |
| `sim_reset_experiment` | `experiment_id`, `reset_mode`, optional `source_run_id`, `request_id` | New trial ID, reset receipt, retained-state categories and opaque start-state fingerprint | Reset modes are capability-gated; prior recordings unchanged |
| `sim_run_experiment` | `experiment_id`, `hypothesis`, `expected_observation`, `request_id` | Run ID, terminal status, observable summary, artifact IDs, applied controller hash | Runs a bounded trial; hypothesis is caller text, never used to determine outcome |
| `sim_get_run` | `run_id` | Immutable manifest, status, actual summary, public event index, artifact metadata and pagination links | Read; includes contact/mission/censoring status |
| `sim_get_observations` | `run_id`, `fields`, `start_s`, `end_s`, `cursor`, `limit` | Timestamped observed/derived samples, units, applied-command changes, next cursor | Read; no synthetic interpolation presented as sensor readings |
| `sim_get_frames` | `run_id`, `camera`, `times_s`, `include_telemetry` | Actual image content/attachment references, frame IDs, actual/requested timestamps, telemetry windows and provenance | Read; up to 12 frames; deterministic nearest-frame selection |
| `sim_get_replay` | `run_id`, `view`, `start_s`, `end_s`, `speed`, `overlays` | Recorded replay artifact/manifest, frame index, event bookmarks and playback settings | Creates/retrieves derived media only; no physics advancement |
| `sim_compare_runs` | `before_run_id`, `after_run_id`, `alignment` | Compatibility check, metric differences, synchronized replay/telemetry artifacts, actual outcomes | Read/derived media; flags unmatched tasks/worlds instead of calling them repaired |
| `sim_evaluate_controller` | `task_id`, `controller_id`, public `suite_id`, `request_id` | Per-case actual runs, metrics and task completion verdicts | Executes only a declared development suite; no optimizer or edits |
| `sim_freeze_controller` | `task_id`, `controller_id`, `expected_hash`, `request_id` | Immutable submission receipt binding controller/task versions | Freezes submission for later independent evaluation; does not assert success or expose reserved cases |

Every tool page must document precise optional/null semantics; optional table entries are not permission to guess defaults. Mutating tools use a request ID either as a listed input or required common envelope field. Read tools accept an optional common request ID for traceability. Keep one convention in the final schemas.

## Experiment and reset semantics

`mode="mission"` executes the fixed objective from its task start. `mode="probe"` selects a public diagnostic recipe; recipe-specific bounds may permit a speed or pulse adjustment through a typed probe configuration. A mission cannot become easier because an experiment requests another target, duration, seed, or physical parameter.

`controller_id` selects a versioned feedback controller. `controls` is an explicit schedule of `[trial_time_s, command]` pairs, starts at zero, is strictly increasing, and holds until the next entry. The two are mutually exclusive. Mission-specific discrete commands, such as package release, are validated against task gates. Schedules support diagnostic experiments; using a schedule never earns a special success rule.

The first run after create uses the declared task/probe start. Each later run requires an explicit reset receipt. `reset_mode="task_start"` restores the task's original world, including its off-center payload or steering defect and required prehistory, and initializes candidate state. This is not an instruction to set `fault=healthy`.

`reset_mode="retained_state"` is available only where documented. It uses a completed source run in the same task, repositions for a probe, preserves declared physical damage/thermal/load state, and initializes the candidate's controller state according to the probe contract. The reset receipt identifies retained categories without exposing private numeric values. It cannot be used to claim a same-start before/after comparison. Unsupported reset combinations are rejected.

Implement this wrapper deliberately around current `reset_trial`/`reset_full`: their platform semantics currently retain damage or replay an original healthy-start fault schedule (`simulator/platforms/CONTRACT.md:20`), which is not automatically equivalent to the new task-start fixture.

Initial capture request: cameras from the task's available camera list, `fps`, width/height, and `public_overlays`. Capture all requested views from the same physics step; do not run one fresh simulation per camera. Observation and command recording remain active even with images disabled. Querying a camera not recorded must return unavailable or a separately verified reconstruction with provenance, never a substituted image.

## Editable controller contract

Proposed interface:

```python
def init(task, nominal_engine, parameters):
    # Return the candidate's JSON-serializable state.
    ...

def step(observation, goal, state, dt_s):
    # Return (validated platform command, updated candidate state).
    ...
```

These signatures are a proposal, not functions already implemented. The host supplies frozen public task/nominal data and the versioned candidate parameters at initialization, then public observations/fixed goal at each tick. Physics advances at its pinned step; the most recent command holds between controller ticks. Controller compute time never becomes simulation time. No model/API request occurs inside a physics step.

Each controller executes in a bounded worker with explicit available helpers/imports, deterministic randomness if permitted, state-size limits, and execution limits. Reuse the component worker design after checking whether its existing brake-specific operations fit (`component_worker/runtime.py`, `component_worker/client.py`); do not claim the current worker already supports this interface.

Source can read its own state and public nominal helpers, not live `MjModel`/`MjData`, private truth, evaluator files, filesystem/network credentials, or future sensor values. Interface violations terminate that candidate attempt with retained partial evidence. Keep the last complete controller version and make validation failures actionable.

### Platform control surfaces

| Platform | Existing command surface | Planned additions to expose |
|---|---|---|
| Dog | `forward_speed`, `yaw_rate`, `motors_enabled`, 12 `joint_targets`; direct targets select bounded PD (`simulator/platforms/QUADRUPED.md:61`) | Required candidate-owned gait/PD/balance source and parameters; direct 12-joint torque mode bounded to nominal ±35 N m; public nominal leg kinematics helpers |
| Drone | `rotor_commands` in [0,1] or `target_position`; rotor order FL, FR, RR, RL (`simulator/platforms/DRONE.md:39`) | Editable estimator/mixer/feedback controller; guarded `release_payload`; mission-owned target/landing phase separate from candidate commands |
| Steered car | `steering` ±0.5 rad, `throttle`/`brake` [0,1] (`simulator/platforms/CAR_DAMAGE.md:36`) | Versioned steering controller; fixed route goal and derived route-error observations |
| Brake car | Physical `Simulator.step(throttle, brake)` and separate wheel-capacity worker (`simulator/runner.py:167`, `simulator/contracts.md:12`) | Versioned automatic-brake policy; declared obstacle range/detection/relative-speed sensor; explicitly documented capacity-model separation |

Use strict tagged control modes where existing dictionaries are ambiguous. For example, a drone cannot send direct rotor commands and a target command in the same tick. Engine inspection supplies exact joint ordering and limits from the exported nominal asset.

The required dog v1 controller computes gait phases, desired feet/joints, PD/impedance and balance torques in its own editable source. It returns `{"mode":"joint_torques","joint_torques_nm":[...]}` with exactly 12 values in exported joint order, each within ±35 N m. The host validates and applies those commands through the existing physical actuators; it does not add hidden support or retune the gait. Existing speed/target helpers remain separately tagged probe/legacy modes and cannot be blended into a candidate torque tick.

Publish pure nominal forward/inverse leg kinematics and Jacobian helpers accepting explicit public joint/body pose arguments and returning documented frames/units. No helper reads private live physics. Supply the current nominal gait as inspectable starter source, with the new task's deliberately flawed speed transition clearly isolated in that candidate. Phase offsets, duty fraction, clearance, stride scaling, PD/impedance and balance gains are required editable parameters. Validate the starter/helper behavior at the published control cadence; if the existing gait needs a higher tick rate, declare it in the task and test it rather than silently changing integration time.

Frames: world axes, body axes, quaternion ordering, gyro frame and velocity units are mandatory documentation. Existing drone uses x forward, y left, z up, body gyro, and rotor order FL/FR/RR/RL (`simulator/LAB.md:115`). Export verified per-platform conventions rather than assuming every field matches. Requested rotor command is not a measurement of actual thrust; commanded joint torque is not a ground-contact force.

## Visual evidence transport

A `sim_get_frames` result needs two parts: machine-readable metadata and decodable image content. A local path string alone cannot give a remote model sight. The later transport adapter must attach actual images using its supported image-content mechanism, while the tool result associates frame IDs with timestamps/telemetry. Do not embed giant base64 payloads as ordinary text in the conversation.

Proposed metadata shape (illustrative IDs):

```json
{
  "ok": true,
  "request_id": "read-frames-01",
  "schema_version": "1.0-draft",
  "data": {
    "run_id": "run_001",
    "camera": "side",
    "frames": [
      {
        "frame_id": "frame_0144",
        "requested_time_s": 4.8,
        "actual_time_s": 4.8,
        "mime_type": "image/png",
        "attachment_id": "img_0144",
        "width": 960,
        "height": 540,
        "overlays": "none",
        "origin": "recorded_physics"
      }
    ],
    "telemetry_window_s": [4.6, 5.0]
  }
}
```

`attachment_id` is a proposed adapter-level reference; it is not an existing provider file ID. The final adapter contract must define how it resolves to attached bytes. Return actual sample times and missing-frame errors. Select nearest frame with an earlier-frame tie break, and report the difference; requests outside the recorded interval are rejected. Contact sheets preserve individual frame times and camera names.

Engine/mission event bookmarks are derived from allowed evidence, such as contact, lane crossing, low height, brake onset, or package/pad contact. The public stream cannot announce a hidden fault type at its injection time. Operator-only explanations use a separate overlay channel.

## Planned invocation walkthroughs

These are **illustrative future requests**. They are not executable against today's broker and do not contain corrected controllers.

### Common discovery and failure inspection

```json
{"tool":"sim_list_tasks","arguments":{}}
{"tool":"sim_read_document","arguments":{"document_id":"doc_getting_started"}}
{"tool":"sim_describe_task","arguments":{"task_id":"task_001"}}
{"tool":"sim_inspect_engine","arguments":{"task_id":"task_001","section":"actuators"}}
{"tool":"sim_describe_controls","arguments":{"task_id":"task_001"}}
{"tool":"sim_inspect_controller","arguments":{"task_id":"task_001","controller_id":"controller_original"}}
{"tool":"sim_get_run","arguments":{"run_id":"run_original"}}
{"tool":"sim_get_frames","arguments":{"run_id":"run_original","camera":"side","times_s":[2.0,4.0,6.0],"include_telemetry":true}}
```

IDs, available cameras, times and probes must come from discovery/run metadata. The literal example timestamps are not hidden failure-time hints and need not fit every task.

### Dog diagnostic

After reading the speed schedule, retrieve frames just before/after the observed stumble and the aligned joint/contact observations. Inspect nominal leg ordering and kinematics. Run a public speed-ramp or one-leg diagnostic before deciding which gait settings/source to edit.

```json
{"tool":"sim_create_experiment","arguments":{"task_id":"task_001","mode":"probe","probe_id":"probe_speed_ramp","controller_id":"controller_original","capture":{"cameras":["side","overview"],"fps":30,"width":960,"height":540,"public_overlays":"none"},"request_id":"dog-probe-01"}}
{"tool":"sim_run_experiment","arguments":{"experiment_id":"exp_001","hypothesis":"The support timing becomes inconsistent during the speed transition.","expected_observation":"Contact timing and joint tracking errors increase around the transition.","request_id":"dog-run-01"}}
```

The hypothesis is an example of test design, not a tool-supplied diagnosis. No assumption that it is true enters the run.

### Drone diagnostic

Inspect body/rotor axes and limit data, then obtain package/attitude frames. Run a loaded-hover probe followed by bounded collective and opposed roll/pitch pulse recipes. Compare signs of observed angular response and altitude loss. Run unloaded response only through a declared probe fixture or a legitimate delivery release; never drop cargo arbitrarily to make the mission pass. Editing rotor allocation is permitted; editing rotor strength or physical payload is not.

### Steering diagnostic

Retrieve the fixed route and steering encoders alongside overhead frames. Run zero-steering and equal/opposite steering pulse probes, holding the declared speed conditions fixed. Inspect the starter mapping and compare issued steering to measured wheel/rack angle. A later patch changes that mapping/feedback; it does not change the rack defect, lane geometry or evaluator reference.

### Automatic-brake diagnostic

Read the observation contract to learn when range first becomes valid. Compare brake request onset, wheel speeds, deceleration and gap. Use a wall-free development probe to measure an uncensored stop. A later patch changes the policy that issues brake commands. It cannot achieve success by changing a capacity-prediction component while leaving the executed commands unchanged.

### Candidate change and actual comparison

1. Use `sim_validate_controller` on caller-authored source/parameters; inspect its error report if invalid.
2. Submit an actual unified diff through `sim_patch_controller`, including the expected base hash. This step requires a caller-authored proposal; there is no bundled solution diff.
3. Create a fresh mission experiment with the returned candidate ID. Confirm the same fixture and start-state fingerprints as the original attempt.
4. Run it and retrieve frames/outcomes even when it fails.
5. Use `sim_compare_runs` with simulation-time alignment. A comparison may report “still fails.”
6. Run the public development suite, inspect per-case results, and optionally freeze a submission for later evaluation. Freezing does not change a failed result into success.

## Error documentation requirements

| Code | Meaning | Caller recovery |
|---|---|---|
| `INVALID_ARGUMENT` / `OUT_OF_RANGE` | Schema/type/limit violation | Read field detail and allowed range; retry valid input |
| `WRONG_PLATFORM` | Control/helper incompatible with task | Fetch the task's actual control schema |
| `UNKNOWN_ARTIFACT` | ID absent or inaccessible in this task scope | Use discovery/owned run IDs; do not leak whether a private ID exists |
| `STALE_VERSION` | Edit base hash differs | Reinspect controller and rebase the edit |
| `UNSUPPORTED_RESET` | Retention/probe combination unavailable | Select a documented reset mode |
| `CONTROLLER_ERROR` | Syntax/interface/worker execution failure | Inspect bounded public error; repair candidate; partial run retained |
| `NUMERICAL_FAILURE` | Engine state/warnings invalid | Mark run invalid; builder diagnostics required; not a successful failure demo |
| `RENDER_UNAVAILABLE` | Missing camera/frames/backend | Read available captured views; retain telemetry; do not claim visual evidence |
| `NOT_COMPARABLE` | World, mission or start conditions differ | Rerun a matched fixture; keep unmatched results labeled |
| `LIMIT_REACHED` | Published compute/artifact bound exceeded | Reduce request or use existing evidence; no silent truncation |

## Existing commands and Python calls today

The following commands already exist; they show the current demos, not the new four tasks. They are trusted operator commands and may reveal private preset names/settings (`simulator/__main__.py:26`). Do not copy this section into the restricted agent task bundle.

```sh
.venv/bin/python -m simulator list
.venv/bin/mjpython -m simulator view quadruped_demo
.venv/bin/mjpython -m simulator view drone_demo
.venv/bin/mjpython -m simulator view car_demo
.venv/bin/python -m simulator run brake_fade --frames --camera side --output runs/brake-inspect-unique
.venv/bin/python -m simulator.lab run drone_delay --controls simulator/experiments/drone-collective-pulses.json --duration 12 --frames --output runs/drone-probe-unique
```

`mjpython` is the documented macOS viewer launcher. Output directories must be fresh. These examples were checked against current parser/source; they were not executed for this planning pass. `simulator.lab` currently excludes dog and car, and live viewer restart is a new execution, not a universal saved-frame replay API (`simulator/lab.py:28`, `simulator/platforms/operator.py:155`).

Existing direct Python platform calls are `Simulation.step(control)`, `observe()`, `reset_trial()`, and `reset_full()` (`simulator/platforms/CONTRACT.md:10`). `diagnostics()` is private. The proposed tools wrap these boundaries rather than giving Astra unrestricted access to the simulator object.
