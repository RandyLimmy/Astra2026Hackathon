# Articulated quadruped experiment

The quadruped is a free-base MuJoCo rigid-body model with twelve actuated leg
joints, four contacting feet, and a movable 1.2 kg ballast. Its torso is never
positioned by a trajectory and receives no artificial external support forces.
All walking and balance forces come through motor torques and contact with the
floor. Disabling motors makes it collapse under gravity.

The deterministic nominal controller performs a slow crawl. One foot swings at a
time while the controller shifts its desired center of mass into the supporting
triangle. An observed-pose balance controller allocates a desired body wrench
among the stance feet and maps those forces to joint torques. Foot impedance
controls the stance anchors and swing trajectories. This is a deliberately simple
synthetic test controller, not a policy validated on a robot.

The first second is a settling interval, included in the configured duration.
Default damage is injected at experiment time 5 s. The diagnostic presets follow
precisely the same physical trajectory before that instant. The nominal controller reads
pose, velocity and encoders; it does not read the injected fault parameters or
retune itself after damage. Its support-force calculation uses a fixed nominal
mass and friction assumption.

| Preset | Physical change at onset |
| --- | --- |
| `quadruped_walk` | Healthy reference |
| `quadruped_joint_weakness` | Front-left knee motor torque gain falls from 1 to 0.08 |
| `quadruped_foot_slip` | Front-left foot sliding friction falls from 0.9 to 0.025 |
| `quadruped_leg_damage` | Front-left knee gains a 90 N m/rad spring with bent rest angle -2.35 rad |
| `quadruped_payload_shift` | A spring-driven internal slide moves ballast sideways toward 0.29 m |
| `quadruped_demo` | After 9 s of walking, front-left knee torque gain falls to 0.08; simulation continues through the fall to 18 s |
| `quadruped_gait_failure` | Healthy mechanics; a modest speed increase exposes an incomplete stance/swing coordination policy |

The leg-damage approximation represents a joint that resists extension; it does
not model fracture mechanics. Payload shift moves an actual body with conserved
mass and its own inertia. Changing the slide's spring rest position creates an
internal transient as well as a later center-of-mass offset. Friction loss has no
artificial color marker, and joint weakness has no artificial visible damage
marker. The responsible part must be inferred from motion and allowed sensors.

On the default 10 s, 2 ms forward probe, the calibrated healthy dog travels about
0.89 m with a maximum body tilt of 12.1 degrees and minimum torso height of
0.394 m. Default knee weakness reaches 47.6 degrees of tilt. Default bent-knee
damage produces a physical fall around 8.91 s and torso contact with the ground.
Foot slip and shifted ballast remain upright on this particular probe while
producing different trajectories. These are measured synthetic outcomes, not
claims of real-world maneuvering safety. `public.safe` requires the probe to
finish without a fall, numerical warnings, or more than 40 degrees of tilt.

The presentation preset `quadruped_demo` walks about 1.06 m before its knee loses
support at 9 s. With the 2 ms timestep, it stumbles and falls around 13.68 s,
leaving over four seconds of physical aftermath before the run ends at 18 s.
Its healthy counterpart stays upright throughout the same 18 s walking probe.
The fall comes from the reduced motor torque and floor contacts; neither the
controller nor the viewer supplies a body trajectory or stops physics at failure.

Configuration supports `probe` values `walk`, `stand`, `turn`, `conservative`, and
`passive`; the conservative crawl halves commanded speed. `affected_leg` selects
`FL`, `FR`, `RL`, or `RR`; severity fields are `strength`, `foot_friction`,
`damage_stiffness`, `damage_rest_angle`, and `payload_offset`. The default fault
is `healthy` (`none` is accepted as an alias).

`step()` accepts optional commands with `forward_speed` in m/s, `yaw_rate` in
rad/s, `motors_enabled`, or `joint_targets` containing twelve joint angles in the
order given by `JOINTS`. Explicit joint targets use bounded joint PD torques.
Commands are validated before any clock or state change. Commanded torques are
reported in the public observation, along with pose, velocity, IMU measurements,
encoders, and foot-contact booleans. Exact component parameters and event labels
are available only through private diagnostics.

`reset_trial()` repositions the robot for a new probe while retaining damaged
actuator gains, contact friction, joint stiffness/rest geometry, and shifted
ballast. Experiment time remains monotonic. `reset_full()` restores the nominal
physical model and time zero, then replays the original configuration including
its scheduled fault. A new `Simulation(Config(...))` can run a different probe or
parameter hypothesis without modifying the existing damaged instance.

## F1: faster walking, loss of coordination, physical fall

Run the first failure task with:

```sh
.venv/bin/mjpython -m simulator view quadruped_gait_failure --camera side
.venv/bin/python -m simulator run quadruped_gait_failure --frames --camera overview
```

The task is to walk along the marked strip, starting at 0.10 m/s, then increase
the request to 0.13 m/s from simulation time 7 to 8 s. The 16 s run includes
settling, walking, the speed transition, a physical stumble/fall, and aftermath.
The requested speed increases by 30%; it is not a sudden high-speed jump.
Half-metre floor marks and closer `side`/`overview` cameras provide route and
limb views. Both cameras can render the same `model`/`data` state without stepping.
The scene includes the original articulated model; added marks cannot collide.

The incomplete controller's faster gait groups same-side legs into a shared
swing while its center-of-mass transfer still assumes a single lifted foot.
The mode is entered when the requested faster speed is reached. The resulting
support loss is produced by motor torques and actual ground contacts. It never
changes motor gains, joint limits, friction, stiffness, masses or inertias, and
it never writes a torso trajectory or root force. Existing damage scenarios and
their exact healthy prefix remain covered by their original regression tests.

Measured at a 2 ms timestep:

| Measurement | Failure task | Developer-authored feasibility control |
| --- | --- | --- |
| Distance before speed transition | 0.650 m | 0.650 m |
| First measured support loss | 8.480 s | None |
| Fall threshold crossing | 8.812 s | None |
| First body/floor contact | 9.344 s | None |
| Cumulative body contact | 6.022 s | 0 s |
| Maximum tilt | 179.23° | 14.08° |
| Minimum torso height | 0.074 m | 0.383 m |
| Final forward displacement | −0.219 m after the fall/slide | 1.758 m |

At a 1 ms timestep the failure still falls (8.743 s) and contacts the ground
(9.320 s), while the same control stays upright with maximum tilt 14.12°.
The final slide position changes after repeated impacts; the outcome and onset
are stable under timestep refinement. No MuJoCo numerical warnings occur.
These figures are synthetic-model calibration, not hardware validation.

The developer control changes only `coordination_defect=False`, retaining the
same speed schedule and mechanics. This is a feasibility check, **not an Astra
repair**. Its private summary identifies `controller_provenance` accordingly.
Do not place that control configuration, this private explanation, or diagnostic
fault fields into a future model's task/evidence bundle. The source boundary for
a later controller edit is `Simulation._gait`; restored individual-leg stance
timing is enough for the measured task. No model investigation or repair loop is
implemented by this scenario.

### Neutral actuator and observation contract

External controllers can already replace gait targets through
`step({"joint_targets": [...]})`. Its twelve entries follow `JOINTS`: front-left,
front-right, rear-left, rear-right; each leg has abduction, hip pitch, then knee.
Angles are radians. Abduction rotates about local +x, pitch/knee about local +y.
The XML supplies each joint's range; commands outside it fail before advancing
physics. A bounded PD motor controller converts targets to torques capped at
±35 N m per joint. `forward_speed` accepts ±0.25 m/s and `yaw_rate` accepts
±0.4 rad/s; `motors_enabled` is boolean. Explicit speed requests override the
controller's scheduled command. They cannot change the task's requested speed.
A low-speed command does not enter the faster gait mode.

Observations include world-frame +x actual speed, immutable
`task_requested_speed_mps` from the configured task schedule, and the issued
`command_speed_mps`. The older `requested_speed_mps` remains an alias of the
issued command for compatibility; task evaluation never uses that alias.
These fields distinguish an unmet task request from an intentionally slower probe.
Additional observations include pose/IMU,
joint encoders and commanded targets/torques, actual foot positions and floor
contacts, requested foot positions and stance flags, body tilt and body contact.
`controller_mode` distinguishes `not_started`, built-in `gait`, external
`joint_targets`, and `motors_disabled`. `foot_targets` and `commanded_stance`
are nullable: they are available only for the built-in gait, which produces
those commands. External joint control still reports actual foot positions,
contacts, joint targets and torques; it does not export stale gait metadata.
Presentation phases are `ready`, `standing`, `walking`, `accelerating`,
`stumbling`, `fallen`.
Public summary metrics distinguish pre-transition progress from post-fall
displacement and include support-loss, fall and body-contact times.

For F1, `public.task_complete` is separate from the legacy `safe` diagnostic.
It requires the entire configured run, at least five seconds after the speed
ramp, no fall/body contact/numerical warnings, maximum tilt below 20°, maximum
lateral displacement below 0.25 m, at least 0.4 m forward progress before the
speed transition, and at least 80% of the task's integrated requested forward
distance after the one-second settling interval. Mean actual speed over the
final five seconds must be within 20% of the task's mean request over that
window. Both measured and requested final-window means, evaluated duration,
requested distance and speed tolerance are exposed in public metrics.
After settling, the built-in gait must also receive the scheduled task speed.
Any different built-in speed command marks `public.diagnostic_speed_override`
and prevents task completion, even if its measured speed lies within the 20%
physical tolerance. This rules out avoiding the transition by commanding a
slightly slower gait. The marker persists for the trial and clears on reset.
Commands within 1e-9 m/s of the scheduled demand are normalized to that exact
demand before gait actuation and command recording, so numerical tolerance
cannot bypass a strict gait-speed threshold.
External `joint_targets` control is evaluated by physical task performance;
its unused `forward_speed` field does not count as a built-in gait override.
The calibrated developer feasibility control completes this task. Standing,
slower diagnostic probes, the failure run, and unfinished/short runs do not.
A completed standing probe can still report `safe: true` under the pre-existing
balance diagnostic; it reports `task_complete: false` for the walking task.

`public_events()` returns copied `{time,event,label}` entries for walking onset,
the speed request transition, measured support loss, fall and first body contact.
Walking onset requires measured forward displacement of at least 0.04 m,
forward velocity above 0.025 m/s, two or more contacting feet, and torso height
above 0.30 m after settling. A stationary robot emits no walking bookmark.
Support loss requires fewer than three contacting feet with tilt over 15°;
fall uses the existing height below 0.20 m or tilt above 65° criterion. Neither
event is forced at a storyboard timestamp. Public events contain no private
coordination/fault parameters. Full reset clears these events and deterministically
repeats the same physical run. Retained-condition trial reset starts a new gait
and speed schedule with monotonic experiment time.
