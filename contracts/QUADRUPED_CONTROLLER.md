# Walking controller interface

The editable artifact is a JSON object. It supplies parameters to the same
continuous gait-clock, foot-trajectory and bounded motor-control path used by
the original attempt. Edits cannot change the physical model or task.

| Field | Units / range | Effect |
|---|---|---|
| `rear_cadence_gain` | dimensionless, finite number in [0, 4] | Scales the rear legs' cadence response to changes in requested speed |

All fields are required. Unknown fields, duplicate fields, booleans, non-finite
values and out-of-range values are rejected before creating a simulation.

`controller_source.py` describes how this parameter affects the continuously
integrated leg clocks and smooth foot targets. Leg order is front-left,
front-right, rear-left, rear-right. Phases are measured in cycles; joint angles
are in radians, world positions in metres and time in seconds. Foot contacts
are actual observations; requested phases and stance flags are controller
metadata. Use them together with the unannotated camera sequences.

The engine keeps the original masses, contacts, actuator limits, target speed
schedule and evaluation criteria. It converts the gait's target trajectories
into joint torques through its existing support/impedance controller. The
parameter edit does not bypass physical actuation or reposition the dog.

Run a changed file through the same interface:

```sh
python -m simulator.scenario_replay quadruped_gait_failure --controller controller.json
```

For interactive inspection:

```sh
mjpython -m simulator view quadruped_gait_failure --controller controller.json --camera side
```

The result contains the loaded controller, its identity, raw evidence and
`task_complete`. Compare attempts only when the physical/task identity matches.
Do not count standing, lowering the requested task speed or changing the world
as completion. A candidate can fail; its measured result remains in the record.

This interface includes no reference correction and makes no claim that GPT-6
has authored or verified a change. That requires a recorded model interaction,
its actual controller edit and a passing fresh physical run.
