# Warehouse and drone intervention lab

This lab supplies physical failures, observable motion, controlled interventions,
nominal predictions and recorded comparisons for Astra's model-maintenance loop.
It does not itself infer a fault or edit a candidate dynamics model. Use the
separate agent workstream to select probes, patch its model, freeze a prediction,
and score a previously unused maneuver.

The warehouse platform is integrated with the shared car/quadruped/drone CLI.
The drone physics is reused from the other platform workstream; this lab adds
independent checks and explicit diagnostic command recipes. Upstream main was
integrated at `9fe0d19`, including the separate car Python-actuator comparison.
No new runtime dependencies were introduced by the warehouse/drone lab.

## Run and view

The existing `.venv` runs the warehouse/drone mechanics. The incoming main branch
requires Python 3.12 and the committed requirements lock for the full car worker,
plotting and pytest suite. The existing local environment was preserved.

```sh
.venv/bin/python -m simulator list
.venv/bin/mjpython -m simulator view warehouse_shift
.venv/bin/mjpython -m simulator view drone_rotor_loss --speedup 0.5
.venv/bin/python -m simulator compare warehouse_battery --camera chase --frames
.venv/bin/python -m simulator compare drone_wind --probe hover
```

On macOS use `mjpython` for the live viewer. It opens paused: press Space in the
viewer to play the full scenario. Space pauses/resumes during playback and starts
a full replay after completion. R repositions while retaining completed faults;
N replays the configured experiment; Escape closes.
`safe` only reports the platform's declared synthetic probe envelope.

The default fault presets intentionally show prediction failure. In particular,
`drone_rotor_loss` leaves one rotor at 12% effectiveness after three seconds;
the unchanged nominal controller then loses stability. A fall is expected for
that severe preset. It is not an automatic recovery demonstration.

Choose a healthy scenario to check normal motion, or a fault scenario to show
what the nominal controller can no longer predict:

| Object | Healthy scenario | Example failure scenario |
|---|---|---|
| Four-wheel car | `baseline` | `wheel_loss` |
| Post-crash car | `car_postcrash_healthy` | `car_steering_damage` |
| Robot dog | `quadruped_walk` | `quadruped_joint_weakness` |
| Drone | `drone_hover` | `drone_rotor_loss` |
| Warehouse robot | `warehouse_healthy` | `warehouse_shift` |

For a less severe drone failure, add
`--config simulator/experiments/drone-maintenance.json` to the viewer command.
Its tested challenge stays airborne while showing prediction error. The same
Space playback controls now apply to the original car and recorded car replay
as well as every platform. Close old viewer processes and relaunch to load updates.

For a fixed command schedule, use the lab adapter:

```sh
.venv/bin/python -m simulator.lab compare warehouse_battery \
  --controls simulator/experiments/warehouse-pulses.json --duration 8
.venv/bin/python -m simulator.lab compare warehouse_shift \
  --controls simulator/experiments/warehouse-turns.json --duration 8 --frames
.venv/bin/python -m simulator.lab compare drone_delay \
  --controls simulator/experiments/drone-collective-pulses.json --duration 12
.venv/bin/python -m simulator.lab compare drone_rotor_loss \
  --controls simulator/experiments/drone-attitude-pulses.json --duration 12
.venv/bin/python -m simulator.lab run drone_wind \
  --controls simulator/experiments/drone-flight-challenge.json --duration 12
.venv/bin/python -m simulator.lab export-baseline warehouse_healthy runs/warehouse-nominal
```

Schedules are nonempty JSON lists of `[trial_seconds, command_object]`, beginning
at zero with strictly increasing times less than the trial duration. Each command
holds until the next entry, applied at the first physics step at or after its
timestamp. Config JSON can be a bare object or an object under `config`; explicit
CLI duration/timestep/probe/fault-at flags take precedence. Invalid commands are
rejected by the platform. Existing output directories are never overwritten.

## Diagnostic design

| Platform / change | Useful experiment | Observable evidence and limits |
|---|---|---|
| Warehouse, extra mass | Low/high torque acceleration, then coast | Lower acceleration; one acceleration result confounds mass and motor gain |
| Warehouse, moving cargo | Left turn, coast, right turn | Asymmetric transient path/tilt; opposed turns can cancel final position error |
| Warehouse, slipping wheels | Acceleration at multiple torque levels | Wheel angular speed versus body translation, with possible lateral drift |
| Warehouse, caster resistance | Opposed turns plus straight coast | Heading-dependent resistance; compare with symmetric bearing drag |
| Warehouse, bearing resistance | Zero-command coast between pulses | Decay continues without drive; the simplified model uses joint friction |
| Warehouse, motor degradation | Small/large pulses and coast | Weaker driven response with unchanged passive resistance |
| Drone, rotor loss | Collective pulse plus opposed attitude pulses | Unequal thrust generates roll/pitch moments as well as altitude error |
| Drone, voltage sag or payload | Multiple collective levels and transients | Both can reduce altitude; a single hover residual does not uniquely identify them |
| Drone, crosswind | Hover and changed horizontal targets | Horizontal residual in world coordinates; distinguish from thrust loss using attitude/altitude |
| Drone, command delay | Short and long collective pulses | Response begins later; this core models transport delay and a fixed motor spool time |

Use `warehouse-challenge.json` and `drone-flight-challenge.json` as separate
prediction maneuvers after discovery. They are public development examples,
not secret heldouts. Evaluation must keep its actual held-out schedules/severities
outside agent access and freeze them before model tuning.

## Physical and control contracts

Warehouse presets: `warehouse_healthy`, `warehouse_payload`, `warehouse_shift`,
`warehouse_slip`, `warehouse_caster`, `warehouse_resistance`, `warehouse_battery`.
The robot has two driven rotating wheels, a swivel caster, and a latched cargo
slide on an inclined rail. Gravity/inertia move cargo after latch release; no
scripted chassis displacement or extra launch force is used. Additional mass is
compiled at loading. Other faults affect contact friction, joint resistance or
motor gain at their configured onset. No controller reads hidden fault state.

Warehouse controls are `{ "left": u, "right": u }`, normalized motor torques in
[-1,1], with nominal 3 N m per unit command; omitted sides command zero. Built-in probes are `straight`, `turn`,
`pulses`, `validation`. The reset retains completed damage, cargo displacement,
monotonic experiment time, and partially completed motor degradation.

Drone presets: `drone_hover`, `drone_rotor_loss`, `drone_voltage_sag`,
`drone_payload`, `drone_wind`, `drone_delay`. The nominal 1.2 kg body uses four
site-transmitted thrust forces, each up to 6 N, with lever-arm and rotor-reaction
torques. Controls contain exactly one of `rotor_commands` (four values in [0,1])
or `target_position` ([x,y,z], horizontal coordinates within ±10 m, altitude
0.3–5 m). Rotor order is front-left, front-right, rear-right, rear-left; x forward,
y left, z up. Public angular velocity is body-frame gyro output. The nominal
motor time constant is 0.035 s; voltage scales thrust by squared voltage ratio.

The drone payload preset is a declared co-moving external pickup at `fault_at`,
with mass, center of mass and inertia recomputed while preserving motion. It is
an idealized intervention, not a simulated package-grasping sequence. A prediction
must know a scheduled loading intervention, or start after observing it. The
transport-delay preset retains command prehistory; it is not an increased motor
time constant. These details differ from an initial sketch and are intentional
compatibility with the existing drone core.

## Prediction and export boundary

`simulator.lab compare` runs the healthy model first, saves `prediction.json` and
its exact-file SHA-256, then runs the changed physical world. With explicit rotor
or wheel commands both runs receive the same input schedule. Target-position
commands specify the same target schedule; feedback motor commands can differ
because the controller observes different states. Without `--controls`, both
runs use the same nominal probe/controller, again without fault-aware tuning.

The report measures position RMSE and maximum error over common recorded times.
It reports both recorded durations and does not extrapolate after either run ends.
A large residual demonstrates a stale nominal prediction, not a unique diagnosis,
a repaired model or successful autonomous flight recovery.

Each run uses the existing `Recorder`: 100 Hz public observations, optional PNG
frames, public outcome summary, and separate private diagnostics/configuration.
The comparison report and configuration are operator-only. Share only permitted
`public/` recordings and the healthy export; filesystem/process isolation is an
integration responsibility. Private folder names alone do not enforce it.

The nominal export is reloadable MJCF plus a public observation example. An
external controller is required; MJCF alone does not execute probe schedules.

## Verification

Focused checks:

```sh
.venv/bin/python -m unittest discover -s tests -p 'test_warehouse.py' -v
.venv/bin/python -m unittest discover -s tests -p 'test_drone_lab.py' -v
.venv/bin/python -m unittest discover -s tests -p 'test_lab_cli.py' -v
ruff check simulator tests
mypy --python-executable .venv/bin/python simulator
```

Full upstream integration tests require the Python 3.12 environment with
`requirements.txt`, then `python -m pytest tests` (the imported upstream suite
includes pytest functions that unittest discovery alone does not execute).
Physics checks include repeatability, half-timestep sensitivity, observable fault
effects, input validation, reset persistence, rotor force frames, command timing,
public/private separation and nominal export reload. Visual QA uses rendered
MuJoCo frames. Measurements and final verification status are recorded below.

## Measured delivery results

Final default-preset comparisons are in
`runs/warehouse-drone-final-validation/report.json` (13 paired experiments).
Healthy comparisons have zero position error. All six warehouse faults and all
five drone faults produce nonzero measured motion mismatch, with no solver
warnings. These are measurements at the shipped presets, not guarantees across
all possible configurations.

| Warehouse experiment (8 s) | Healthy x | Changed x |
|---|---:|---:|
| Extra cargo / straight | 8.901 m | 6.217 m |
| Traction loss / straight | 8.901 m | 7.318 m |
| Bearing resistance / straight | 8.901 m | 5.416 m |
| Reduced motor torque / pulses | 5.702 m | 2.868 m |

Cargo shift produces 0.173 m maximum trajectory mismatch; opposed turns cancel
much of its final-position difference. Whole-trajectory evidence matters here.

For continued-flight experiments, the milder operator configuration is:

```sh
.venv/bin/python -m simulator.lab compare drone_rotor_loss \
  --config simulator/experiments/drone-maintenance.json \
  --controls simulator/experiments/drone-flight-challenge.json
```

The same configuration works with voltage, payload, wind and delay presets.
All five changed-world challenge trials remained airborne, with minimum altitude
at least 1.533 m and zero ground contact. Four exceeded the nominal 0.8 m tracking
error envelope; the delay case stayed inside it. Do not equate airborne with
accurate tracking. Records: `runs/drone-maintenance-validation/report.json`.
Severe default voltage/payload faults can remove sufficient hover authority;
fixing a model cannot restore missing physical thrust.

Side-by-side H.264 replays (nominal left, changed world right):

- `runs/warehouse-shift-final-demo/comparison.mp4`: 1920×540, 97 frames, about 8 s.
- `runs/drone-rotor-final-demo/comparison.mp4`: 1920×540, 85 frames, about 7 s.

Rendered-frame QA passed for both, including visible shifted cargo and inverted
drone contact. Camera/lighting edits did not change the physical trajectories.

Verification on the integrated checkout:

- Full Python 3.12 pytest suite: 174 tests and 501 subtests passed.
- Final lab/shared-operator regression: 11 tests and 16 subtests passed.
- Incoming worker/comparison typing corrections: 46 tests and 30 subtests passed
  across the initial run and isolated rerun. Three constructor-startup tests
  timed out under concurrent load and passed unchanged when rerun alone.
- Focused warehouse physics: 12 tests pass; focused drone probes: 9 tests pass.
- Ruff passes. Mypy passes across 20 simulator source files with the existing
  Python 3.11 developer environment. The installed older mypy parser cannot read
  the newer NumPy 2.5 stub syntax in the Python 3.12 environment; runtime tests use
  the committed Python 3.12 dependency lock.
- No merge conflicts remain; HEAD and origin/main are both `9fe0d19`. Local
  implementation remains uncommitted. Pre-pull files are backed up under
  `/tmp/astra-prepull-20260910-warehouse-drone`.

The isolated full-test environment is `/tmp/astra2026-integration-py312`; use its
`bin/python -m pytest tests` to reproduce without modifying the original `.venv`.
The last full run predates the final type-only fixes; the affected worker/car
modules and final CLI were checked again afterward as listed above.
