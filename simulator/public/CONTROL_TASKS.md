# Control tasks: observations and actuator interface

This is the neutral interface for the recorded dog and parcel-delivery tasks.
It contains no diagnosed cause, private fixture parameters, or corrected policy.
The host retains ownership of the physical world, initial conditions and goals.

## Time, frames and experiment boundaries

Positions/linear velocities use metres and metres per second; angular quantities
use radians unless their name explicitly says degrees. The world frame has z up.
Quaternions are ordered w, x, y, z. The default physical step is 0.002 s. A
controller supplies one command per step; it can hold commands across steps.

`observe()` returns current public measurements and command history metadata.
`step(control)` advances one physics step. `reset_full()` starts the same task
and configuration again; changing playback speed never changes this timestep.
`reset_trial()` is an explicit experimental reposition and is not continuous
physical motion. Preserve/reset controller memory according to the declared
experiment boundary and retain the full observed task history.

`observations.jsonl` contains samples at up to 100 Hz. Manifest frame `t_s` is
seconds since recording start, while observation `time` is the simulation clock.
Both recorded camera views come from the same physical state. Use each frame's
`evidence` paths for unannotated RGB; `views` paths may include an illustrative
impact effect. Public events are observations, not explanations of their cause.

## Dog control

Task: walk along the marked strip and follow the recorded requested-speed
schedule. Progress, requested speed and balance are evaluated together. Standing
still or reducing the requested task speed is not task completion.

Commands to `step` may contain these fields:

| Field | Shape / units / limits |
|---|---|
| `forward_speed` | scalar, −0.25 to 0.25 m/s |
| `yaw_rate` | scalar, −0.4 to 0.4 rad/s |
| `motors_enabled` | boolean |
| `joint_targets` | 12 finite joint angles in radians, in the order below |

Leg order: front-left, front-right, rear-left, rear-right (`FL`, `FR`, `RL`, `RR`).
Each leg has abduction, hip pitch, knee, in that order. Abduction acts about local
+x; hip/knee about local +y. Joint ranges are abduction [−0.8, 0.8], hip pitch
[−1.3, 1.5], knee [−2.7, −0.15] radians. A bounded PD actuator converts supplied
joint targets into torques limited to ±35 N m per joint. Invalid targets fail
before physics advances.

Supplying joint targets replaces the built-in gait target branch. Supplying only
speed/yaw uses that built-in branch and can be useful for diagnostic probes.
The controller can derive new joint targets from public measurements and its own
state; the complete task still uses its original goal and speed schedule.

Public observations include body pose, velocity, IMU readings, joint encoders,
foot positions and contacts, previous command, requested foot targets/stance,
requested and actual speed, body tilt and body contact. A foot-target/stance
field is controller metadata; it is not a measurement of actual support.
When an external controller supplies joint targets, built-in foot-target/stance
metadata is unavailable (`null`); use actual feet/joints/contacts instead.
`task_requested_speed_mps` retains the declared task schedule, independently of
`command_speed_mps` used in a diagnostic probe. Physical stability (`safe`) and
completion of the requested task (`task_complete`) are separate outcomes.
Replacing the built-in task speed after settling is a diagnostic intervention
(`diagnostic_speed_override`) and cannot claim task completion, even if a lower
speed lies within a measurement tolerance. External joint controllers are judged
by their actual progress, speed and balance against the unchanged task.

## Parcel-delivery drone control

Task: take off from A carrying the parcel, place/release it at B, return without
the parcel, and land at A. The package must remain at B. Use the manifest's
observed mission target/coordinates rather than assuming world geometry.

Each `step` accepts exactly one of:

- `{"rotor_commands": [u0, u1, u2, u3]}`: four finite normalized rotor commands
  in [0, 1]. This bypasses the built-in flight controller. It does not bypass
  bounded motors, spool dynamics or physical contact.
- `{"target_position": [x, y, z]}`: a position target for the built-in controller;
  x/y in [−10, 10] m and z in [0.15, 5] m. Useful for bounded diagnostic probes.

Rotor order is front-left, front-right, rear-right, rear-left. Nominal x/y rotor
locations relative to the body frame are (0.24, 0.24), (0.24, −0.24),
(−0.24, −0.24), (−0.24, 0.24) m. Each rotor provides at most 6 N along local +z;
nominal spool time constant is 0.035 s. Reaction-torque signs alternate
+/−/+/− with nominal torque/thrust ratio 0.018 m. Bare body mass is 1.2 kg;
the parcel contributes additional mass and inertia while attached. Infer the
response from observations rather than accessing private loading parameters.

`release_parcel()` returns a boolean. It unlatches only when cargo is supported
near B and moving slowly. It changes only the attachment constraint; package
pose, velocity and mass remain continuous. Failed release leaves it attached.

Public observations include pose, linear/angular velocity, specific force,
altitude, tilt, actual contact flags, previous rotor commands, target position,
parcel position/attachment state and mission state. Raw rotor control can use
these observations and controller-owned state. Mission progress does not excuse
an impact or permit teleporting the parcel.

## Evidence and later GPT-6 integration

The public recording is input for an investigation, not an implemented remote
tool broker. The integrating host should expose bounded reads, diagnostic runs
and a versioned controller patch interface around the controls above. Deliver
actual RGB image content to GPT-6, not only filenames. Keep private evaluator
data and builder calibration controls outside its context.

A claimed fix requires a GPT-6-authored change and a new physical run that meets
the unchanged mission criteria. Save the source identity, applied commands,
visual evidence and outcome, including unsuccessful attempts.
