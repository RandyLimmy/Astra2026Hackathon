# Car damage, quadruped and drone scenarios

These three platforms provide repeatable physical failures, nominal controllers,
controlled probes, public sensor recordings and private reference parameters.
They supply the experiments for a model-repair agent. The comparison command
measures an old-model mismatch; it does not diagnose damage or automatically
rewrite a digital twin.

## Open a scenario

```sh
.venv/bin/mjpython -m simulator view quadruped_leg_damage
.venv/bin/mjpython -m simulator view car_steering_damage
.venv/bin/mjpython -m simulator view drone_rotor_loss
```

Use `python` instead of `mjpython` for the viewer on Linux. The viewer starts
paused; **Space** plays the whole scenario, then pauses/resumes during playback.
After completion, **Space** starts a full replay. **R** repositions for another
probe while retaining completed damage, **N** replays the whole experiment from
a healthy start, and **Esc** closes the window. The final pose remains visible
until you replay or close.

## Scenario catalog

| Platform | Presets | Diagnostic probes |
|---|---|---|
| Post-crash car | `car_postcrash_healthy`, `car_steering_damage`, `car_wheel_misalignment`, `car_suspension_damage`, `car_tire_pressure` | `steering`, `braking`, `slalom`, `bump` |
| Quadruped | `quadruped_walk`, `quadruped_joint_weakness`, `quadruped_foot_slip`, `quadruped_leg_damage`, `quadruped_payload_shift` | `walk`, `stand`, `turn`, `conservative`, `passive` |
| Quadrotor | `drone_hover`, `drone_rotor_loss`, `drone_voltage_sag`, `drone_payload`, `drone_wind`, `drone_delay` | `hover`, `maneuver` |

The original brake-fade/wheel-loss track remains available through its existing
presets. `python -m simulator list` lists every registered platform, including
any independently maintained warehouse scenarios.

### Post-crash car

A four-wheel vehicle has front steering joints, independent spring/damper
suspension, and physical tire contact. It strikes a barrier before any damage is
applied. Measured, sufficiently strong barrier contact activates a prescribed
structural change; missing the barrier leaves the car unchanged.

After a visible recovery interval, an explicitly logged reposition places it on
an inspection lane. The car retains its rack bias, bent wheel mount, weakened
spring, or smaller/compliant tire and executes the selected probe. This reset is
a test-fixture intervention, not continuous autonomous recovery from the wall.

Measured default examples: steering drift increases from about 1.81 m to 26.33 m;
a bent wheel increases slalom drift from 2.70 m to 12.85 m; a weakened spring
increases bump travel from 0.139 m to 0.243 m. The tire-pressure approximation
increases braking distance from 2.38 m to 2.69 m while remaining within this probe's
limits. Material fracture and pneumatic tire dynamics are not modeled.

[Car controls, parameters and reset semantics](platforms/CAR_DAMAGE.md).

### Quadruped

The dog has a floating torso, twelve actuated leg joints, four contacting feet,
and a movable internal ballast. A slow crawl and balance controller act through
joint torques; there is no scripted root trajectory or artificial body support.
The first second settles the robot, and faults normally begin after five seconds.

The healthy dog travels about 0.886 m over ten seconds. A weak front-left knee
reaches 47.6 degrees of tilt; a stiff, bent knee produces a physical fall around
8.91 seconds. Foot slip and shifted ballast produce different motion while
remaining upright in their default probes. They are useful counterexamples to
assuming every mismatch must end in a fall.

[Quadruped gait, controls and synthetic damage details](platforms/QUADRUPED.md).

### Drone

Four rotor sites apply thrust and reaction torque to a free quadrotor. A bounded
nominal controller initially hovers, then performs a small translation probe.
It uses nominal mass and actuator assumptions throughout the experiment.

Default healthy flight stays near 2 m altitude with less than 2.4 degrees of tilt.
Rotor degradation, low supply voltage, added payload and transport delay can
produce ground contact. The wind case remains airborne but misses the nominal
position prediction. Voltage-to-thrust scaling, wind/drag, and payload capture
are explicit synthetic approximations. Payload capture is logged as an external
co-moving pickup with mass/inertia change; it is not concealed as gradual drift.

[Drone controls, physics, sensors and limitations](platforms/DRONE.md).

## Lock a nominal prediction, then reveal the changed world

```sh
.venv/bin/python -m simulator compare quadruped_leg_damage --output runs/dog-comparison
.venv/bin/python -m simulator compare car_steering_damage --probe steering
.venv/bin/python -m simulator compare drone_rotor_loss --probe maneuver
```

The command runs the healthy model first and saves `prediction.json` and its
exact-file SHA-256 before constructing/running the fault-world trial. It then
writes separate `nominal/` and `actual/` recordings plus `comparison.json` with
position errors and measured outcomes. Output directories cannot be overwritten.

Both worlds use the same nominal feedback law and probe. Their motor commands
can differ because the feedback law observes different motion. This is a
closed-loop prediction comparison, not an identical-input open-loop fit or
proof that an agent predicted an unknowable future failure. Future-event timing
belongs to the trusted scenario operator.

`--frames --camera chase --fps 12` adds PNG recordings. Configuration JSON can
change fault severity, affected components and trial conditions; `--probe`,
`--duration`, `--timestep` and `--fault-at` override shared fields. Per-platform
parameters are documented in the linked files.

## Controlled interventions and retained damage

Each `Simulation.step(control)` accepts an optional public command:

- Car: `steering` (radians), `throttle`, `brake`.
- Dog: `forward_speed`, `yaw_rate`, `motors_enabled`, or twelve `joint_targets`.
- Drone: four normalized `rotor_commands` or a three-axis `target_position`.

A caller can change commands at chosen simulation times, record observations,
and call `reset_trial()` to test the already-damaged body again. The reset
reinitializes pose/controller preparation while preserving completed faults.
`reset_full()` restores the original healthy start and schedules the configured
failure again. The experiment clock stays monotonic across trial resets.

## Public and private outputs

`public/` holds pose/velocity, commands, allowed joint/IMU/contact sensors,
timestamped frames and observed outcomes. `private/` holds true fault parameters,
event labels and complete configurations. Public videos have no artificial
fault-color indicators. Actual geometry changes or a visible load remain
legitimate observations.

A `safe` result means the *completed synthetic probe* stayed inside its declared
limits; it is not a general roadworthiness or flight-readiness determination.
The integration must enforce process/filesystem access restrictions before an
agent is given the public exports. Naming a directory private does not isolate it.

## Export and verify

```sh
.venv/bin/python -m simulator export-platform quadruped_walk exports/quadruped
.venv/bin/python -m simulator export-platform drone_hover exports/drone
.venv/bin/python -m simulator.platforms.validate --workers 3
.venv/bin/python -m pytest -q
```

The full repository suite needs the Python 3.12 environment and dependencies from
`requirements-lock.txt`, as described in the main README. The original Python
3.11 simulator environment can run these platform demos but lacks the newer
repository's pytest tools; use a separate test environment when keeping it open.

Exports contain a self-contained healthy MJCF and an example public observation.
The nominal probe controllers still require an external implementation; the MJCF
alone is not a complete walking/flying program. A repair workstream can use these
assets and sensor contracts as its editable starting point.

Validation covers all sixteen profiles: healthy versus changed worlds, exact
repeats, finite states, warnings, and outcome agreement at half the timestep.
The report records measured differences; it does not demand identical chaotic
trajectories after a fall. Focused tests additionally cover impact gating,
healthy pre-fault histories, physical component changes, retained damage,
controller inputs, reset safety and contact behavior.

Example local replays are generated under `runs/quadruped-demo/`,
`runs/postcrash-car-demo/`, and `runs/drone-demo/`; generated recordings are ignored
by Git. The validation report is `runs/platform-validation/report.md`.
