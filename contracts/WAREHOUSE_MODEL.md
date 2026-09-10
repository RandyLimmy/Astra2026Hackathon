# Warehouse physical model repair

Repair the simulation so it predicts measured chassis and cargo motion under new public controls.
The initial artifact is a nominal model. It contains no fitted mechanism. The reference
specimen is available only through measured experiments. No hidden configuration,
event labels, internal restraint state, future event times, or reference constraint forces are exposed.
The curve benchmark includes externally observed cargo pose, as described below.

## Public experiments

The default `--task rail` benchmark preserves the original trolley experiment.
Every experiment creates a fresh specimen at rest. Controls are `probe` (`cargo_turn`,
`cargo_mirror`, `cargo_gentle`, `cargo_strong`), `duration` (1–12 seconds), `drive_scale`
(0.2–1), and the publicly weighed `payload_mass` (8–24 kg). These controls select only
wheel schedules, test length, motor command amplitude, and initial cargo mass.
The same controls and initial state drive predictions and measurements. The measured
signals are time, chassis position/orientation, velocity, wheel encoders and IMU.
`run_model` also exposes forces computed by the candidate itself in `model_signals`.

With `--task curve`, select `cargo_curve` or `cargo_curve_slow`, `duration` (1–20
seconds, default 17), `drive_scale` (0.2–1), and publicly weighed `payload_mass`
(8–24 kg, default 8). Both probes follow the same marked curve; the second uses a
lower speed. Predictions and measurements run the same route controller, which uses
public chassis state. Motor commands can diverge as their predicted/measured states
diverge. The task is to predict trolley and cargo motion under that policy, rather than
to reproduce a prescribed identical command trace.

Curve observations additionally contain `cargo_position` (world XYZ metres) and
`cargo_orientation` (world quaternion, WXYZ), which represent externally measurable
cargo pose. `cargo_floor_contact` reports a current contact between any collidable
cargo geometry and the floor. `cargo_has_touched_floor` persists after any such contact
is detected at a physics step, including brief impacts between sampled frames. These
external contact sensors and their history do not expose restraint activation, hidden transition parameters,
constraint forces from the specimen, or causal event annotations. The nominal curve
model includes three cargo translations, a ball joint, deck/floor collision geometry,
and the nominal restraint. Physical topology remains host-owned and identical between
reference and nominal models. The artifact can change bounded physical parameters
and persistent state transitions; experiments determine which changes are justified.

Use `inspect_model`, `run_experiment`, `run_model`, `patch_model`,
`run_regression_suite`, then `submit_prediction`. Tool budgets are finite and unsuccessful
attempts count. Submission locks the candidate; evaluation reveals outcomes only after
all nominal and candidate predictions for unseen control combinations have been saved.

## Editable artifact

`patch_model` accepts `candidate_json`, a string containing the entire JSON document,
and a short `rationale`. The only top-level fields are:

```json
{"schema_version": 1, "model_edits": [], "rules": []}
```

There are at most 24 model edits and 8 rules. Each model edit has exactly `target`,
`name`, `field`, `value`. Edits apply to an existing named element of host-owned nominal
XML before compilation. Nothing loads candidate XML, Python, arbitrary paths or poses.
No topology, initial equality activation, solver, timestep, sensor, control, or observation
edits are allowed. A duplicate `(target, name, field)` is rejected.

| Target | Field | Allowed values |
|---|---|---|
| Existing named joint | damping | Scalar 0–30 |
| Existing named joint | frictionloss | Scalar 0–10 |
| Existing named joint | axis | Three values in [-1, 1], norm 0.9–1.2 |
| cargo_slide joint | range | Two values in [-0.3, 0.3], increasing and containing zero |
| cargo_box, chassis_geom, tire_left, tire_right, caster_tire geom | mass | Scalar 0.1–60 kg |
| floor, tire_left, tire_right, caster_tire geom | friction | Three nonnegative values, maxima [2, 0.1, 0.1] |
| left or right motor | gear | Scalar 0.1–12 |

Only elements actually present in `inspect_model` may be edited. All numbers must be
finite and booleans are not numbers. Setting a geom mass replaces its public initial
mass for that prediction, so a fixed fitted mass must still generalize to public mass
interventions. The artifact is bounded to this model family; it cannot invent arbitrary
mechanisms or change the host simulator implementation.

## Persistent state transitions

A rule has exactly `when`, `updates`, and `once` (true). A condition has exactly:
`signal` (`equality_force`), `name` (an existing scalar joint equality), `absolute` (true),
`comparison` (`gte`), `threshold` (0.1–1000 N), `after_s` (0–10 seconds), and `sustained_s`
(0–1 seconds). The single update has `target` (`equality`), the same `name`, `field`
(`active`), and `value` (false). Choose these parameters from experimental evidence.

The rule observes the magnitude of the candidate's own scalar equality constraint
force, after the previous physics step's forward pass. Inactive equalities report zero.
Each pre-step evaluation adds one physics timestep to the dwell counter if time is at
or after `after_s` and the force meets the threshold; otherwise the counter resets.
After consecutive qualifying time reaches `sustained_s`, the equality is disabled
before the next physics step and stays disabled for that specimen. Zero dwell triggers
on the first qualifying step. Rules execute in listed order and do not write positions
or velocities. State resets on a new specimen or full reset. Multirow equality types
such as weld/connect cannot be used as scalar force signals.

## What counts as success

Evaluation uses control conditions never requested as measured development experiments,
including a gentle control. Changing only duration does not make a control condition
unseen: every prefix or extension of an observed probe/mass/drive-scale combination
is excluded. All predictions and their hashes are saved before any
held-out measurements are run. Rail criteria are declared before investigation: at least two
affected cases with nominal position RMSE ≥0.03 m, candidate RMSE ≤0.05 m and ≤35% of
nominal error; candidate heading RMSE ≤0.08 radians; gentle control candidate position
RMSE ≤0.015 m with degradation ≤0.01 m. Candidate and measured simulation must remain
finite, upright, and free of physics warnings. A failed candidate is reported as failed.
The curve benchmark instead requires at least two unseen cases in which the cargo
reaches the floor and one slow control in which it stays on the trolley. Cargo position
is evaluated in three dimensions: affected-case nominal RMSE must be at least 0.15 m;
candidate RMSE must be at most 0.08 m and 35% of nominal error. Candidate chassis
position RMSE must be at most 0.1 m and heading RMSE at most 0.1 radians. The slow
control allows at most 0.02 m cargo RMSE and 0.015 m chassis RMSE. All cases require
the correct cargo drop outcome and an upright, finite, warning-free trolley.

The drop outcome comes from the public `cargo_has_touched_floor` contact-history
sensor. It includes all collidable cargo pieces and every landing orientation, and is
updated at every physics step. The evaluator cannot be passed by a fixed-cargo
candidate merely because its route controller tracks the chassis path. Curve novelty
uses effective target speed and cargo mass, so equivalent speed settings across the two
curve probes and duration-only changes are also excluded. Curve criteria, predictions,
and outcomes are saved separately from the original rail benchmark.

An offline evaluation proves only the supplied artifact's performance; it does not
establish that GPT-6 authored it. Reports separately record the initial artifact origin
and hash, accepted patch count, whether the final source changed, and whether the agent
submitted. Submitting an unchanged supplied artifact does not establish patch authorship.
A live investigation additionally records the verified API model and reasoning profile. The crate is still physically loose: this repairs prediction, not hardware.
