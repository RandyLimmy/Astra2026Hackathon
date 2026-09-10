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

## Uneven-load delivery failure animation

`drone_delivery_imbalance` is an additive task in `drone_delivery.py`, using its
own `drone_delivery.xml`. Legacy hover, pickup and rotor-loss presets retain their
original asset, controller and behavior. The new mission starts on pad A with a
**real, visible 0.36 kg parcel attached 0.30 m forward and 0.20 m below the drone**.
The 1.2 kg drone still has four healthy 6 N rotors and the same 35 ms motor lag.
Total weight is 15.30 N against 24 N maximum thrust; the package is physically
controllable with bounded differential thrust.

The incomplete nominal mission controller has a specific handoff defect: its
takeoff loop integrates attitude error to acquire hover trim, while its outbound
route loop discards that trim and uses a centered-load attitude tuning. The
offset remains present from the initial frame. MuJoCo determines the growing
pitch, descent and ground impact. No rotor fault, scheduled crash, root-pose
forcing or mass deletion creates the failure.

Frozen default calibration at 2 ms timestep:

| Observed event | Time / measurement |
|---|---|
| Takeoff begins | 0.600 s |
| Actual liftoff | 1.704 s |
| Enter outbound flight | 3.902 s |
| Tilt exceeds 30° | 4.344 s |
| Actual drone ground impact | 5.518 s; 5.57 m/s incoming body speed |
| Forward travel before impact | 4.849 m; maximum altitude 2.194 m |
| End after impact settles | 8.018 s |

A and B are 4 m apart. The fixed `side` camera makes pitch and parcel placement
clear; `overview` retains the route and pads. Public events describe observed
mission changes, large tilt and contact, and carry simulation timestamps.
`observe()` includes pose, velocity, gyro, accelerometer, requested rotor
commands, mission target, current body/parcel contacts and attachment state.
Private load parameters, controller reference and release validation stay in
`diagnostics()` and the trusted configuration.

The mission supports takeoff → outbound → approach → contact-supported placement
→ release → unloaded climb → return → landing. The parcel is an independent
free-jointed physical body from time zero, initially coupled by a weld. Release
requires actual ground contact near B and package speed below 0.15 m/s; changing
the weld's active bit preserves all positions, velocities, mass and inertia.
The package remains on B while the unloaded drone returns to A. Explicit full
and trial resets reload the package; no reset occurs within a flight.

For future controller work, normalized `step({"rotor_commands": [...]})`
requests bypass the nominal controller and drive the real motor states. The
alternative `target_position` request runs the current controller and allows
altitude 0.15–5 m. `release_parcel()` exposes the guarded physical latch command.
Control entries must be finite numbers; booleans and numeric-looking strings
are rejected. Validation precedes mission transitions and physical release, so
an invalid request leaves the clock, controller memory, latch and plant unchanged.
`_delivery_control(target)` is the present controller boundary. No Astra call,
patch execution or repair claim is implemented here.

For **private developer feasibility validation only**, set
`delivery_controller="feasibility"`. That reference uses explicit known load
trim through the same rotor limits. It must not be supplied as investigation
source or presented as an Astra repair. With identical offset/mass/mechanics it
places the parcel at 14.616 s, returns unloaded and completes a one-second
stationary landing at 28.618 s. The parcel finishes 2.6 mm from B and the drone
0.31 mm from A, with maximum tilt 6.19° and no body contacts outside controlled
pad placement/landing. Centering only the load (`fault="healthy"`) also allows
the unchanged nominal controller to complete, identifying the loading mismatch.
Opposite-side offset and 0.40 kg parcel reference trials also complete within
30 s. Halving the timestep retains failure and mission-complete outcomes; the
failure's forward travel changes by about 0.5%, well within the frozen 2% bound.

`decorate_scene(scene)` appends a deterministic brief flash, sparks and smoke
only after measured drone-body ground impact following departure, with incoming
speed above 1 m/s and substantial tilt or downward velocity. It accepts either
a renderer scene or a reset `viewer.user_scn`. Callers reset/rebuild the scene
before each decoration; repeated renders produce identical geometry. Takeoff,
parcel placement and controlled landing produce no effect. The effect has seed
2048, applies no forces and does not simulate combustion or a blast. Plain
physics evidence can omit the decorator. Decorated and undecorated runs have
identical controls, states, public events and outcomes.

Verification: `python -m pytest tests/test_drone_delivery.py tests/test_drone.py
 tests/test_drone_showcase.py tests/test_drone_lab.py -q`. Coverage includes the
new physical failure, same-physics feasibility, actual latch continuity, neutral
evidence, command bounds, held-out load variants, reset/determinism, halved
timestep and effect isolation. Visual storyboard evidence and a passing verdict
are retained under `.omx/state/drone-delivery-visual/`.
