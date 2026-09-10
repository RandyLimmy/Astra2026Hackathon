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
