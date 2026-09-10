# Quadrotor scenario platform

This is a physical synthetic test environment for model identification. It does
not implement Astra inference, model editing, or an aircraft safety certification.

`drone_hover` is the healthy control for `drone_rotor_loss`, `drone_voltage_sag`,
`drone_payload`, `drone_wind`, and `drone_delay`. Each uses the same nominal
controller and probe: hover at 2 m for 4 seconds, followed by a smooth lateral
translation and reversal. Faults activate at experiment time 3 seconds. The
duration is 12 seconds, timestep 2 ms; all are configurable. `probe="hover"`
provides a separate stationary experiment.

The 1.2 kg vehicle has one free joint and four site-transmission motors. Rotor
order is **front-left, front-right, rear-right, rear-left**. Their positions are
`(+.24,+.24)`, `(+.24,-.24)`, `(-.24,-.24)`, `(-.24,+.24)` m in the body frame.
Each produces up to 6 N along body z and a signed reaction torque of 0.018 times
thrust. Nominal motor spool time is 35 ms. The controller has fixed nominal mass,
fixed allocation, bounded torque/tilt authority, and no fault-state access.
MuJoCo integrates all free-body motion and contacts; no root stabilization,
teleport, or scripted crash occurs during a trial.

The faults deliberately produce different physical signatures:

| Fault | Default physical change | Observable result in the default probe |
|---|---|---|
| Rotor loss | Front-left thrust effectiveness becomes 0.12 | Attitude loss, inverted ground contact |
| Voltage sag | Supply ratio becomes 0.60; thrust capacity scales with its square | Mostly level descent and ground contact |
| Payload | An explicitly declared co-moving 1.4 kg package is captured | Mass, COM and inertia change; descent and ground contact |
| Wind | Constant 3 N world-x force | Position offset/drift while remaining airborne |
| Delay | Command transport delay becomes 220 ms | Hover initially holds; translation destabilizes attitude |

Payload pickup is an idealized external intervention: the package arrives with
the same rigid-body motion as the aircraft and is immediately latched. Position
and velocity remain continuous. Its box inertia and parallel-axis contributions
update the real body mass, center of mass, and inertia. `mj_setConst` receives
scratch data because it overwrites its data argument with the reference pose;
the live integration state is retained. Trial reset preserves the captured mass.

For deliberate experiments, pass one control dictionary per timestep:

```python
sim.step({"rotor_commands": [0.49, 0.49, 0.49, 0.49]})  # normalized requests [0,1]
sim.step({"target_position": [0.5, -0.25, 2.0]})       # nominal bounded controller
```

These commands permit thrust pulses, differential rotor tests, waypoint changes,
and braking/reversal without access to the hidden cause. A target must have
horizontal coordinates within ±10 m and altitude 0.3–5 m. Omitting the command
uses the configured deterministic probe. Direct motor requests bypass the pose
controller, while retaining the physical motor lag, transport delay and faults.

`observe()` exposes time, phase, pose, world linear velocity, body gyro, body
specific-force accelerometer, requested commands, target position, altitude,
tilt, and current ground contact. These idealized noiseless sensors are observable
quantities; they exclude fault identity, rotor effectiveness, actual thrust,
supply ratio, wind, mass, inertia, transport delay and event logs. Those fields
are available only through `diagnostics()` and private summaries.

`reset_trial()` explicitly repositions the vehicle at its initial hover pose,
zeros velocities, prepares nominal initial rotor speed, and resets probe metrics.
All completed faults persist and the experiment clock remains monotonic. Pending
events still use that experiment clock. `reset_full()` restores the original
configuration's healthy start and repeats its event schedule from time zero.

The public `safe` value requires a completed probe; before completion it is false
and the outcome is `incomplete_probe` (or already-observed `ground_contact`). It
only describes the executed synthetic probe: no ground
contact, altitude between 0.5 and 3.5 m, tracking error at most 0.8 m, and tilt at
most 35 degrees. It is not a general maneuverability claim. Healthy default
flight stays within 0.0002 m of its 2 m altitude, below 2.4 degrees tilt and below
0.49 m maximum translation tracking error. Fault cases deliberately fail that
envelope. Ground-contact outcomes are physical contact results; solver warnings
instead raise an error and must not be reported as a successful crash simulation.
All six default presets also ran at a halved 1 ms timestep with zero warnings and
the same airborne/contact and probe-envelope outcomes. Healthy maximum tracking
error changed from 0.4862 to 0.4868 m; wind error changed from 1.3640 to 1.3646 m.
Post-impact trajectories are sensitive, so exact crash-path identity across
timesteps is not asserted.

Simplifications: primitives instead of a manufacturer aircraft; prescribed
linear drag and crosswind force rather than aerodynamics/CFD; squared-voltage
thrust scaling rather than an electrical battery model; no propeller gyroscopic
or blade dynamics; perfectly known nominal state estimation; idealized package
capture. Multiple thrust levels and transient/attitude probes are needed to
distinguish extra mass from symmetric propulsion loss. No single hover trace
proves a unique diagnosis.

Focused verification: `python -m unittest discover -s tests -p 'test_drone.py' -v`.
The tests cover healthy flight, identical pre-event traces, all fault effects,
site-actuation signs, motor shutdown, payload state continuity, nominal-controller
blindness, real delay history, reset persistence, deterministic replay, neutral
fault settings and the public/private observation boundary.
