# Editable wheel component: version 2

The editable source implements four functions. Every vector has exactly four
entries in wheel order **FL, FR, RL, RR** (front-left, front-right, rear-left,
rear-right). Only this Python component is editable.

```python
init_state() -> dict
compute_brake_torque_limits(state, brake_command, wheel_speed_rad_s) -> list[float]
advance_state(state, brake_command, mean_wheel_speed_rad_s, applied_brake_torque_nm, dt_s) -> dict
on_trial_reset(state) -> dict
```

`brake_command` is the requested pedal fraction in `[0, 1]`. Wheel angular
velocities are signed radians/second. `compute_brake_torque_limits` returns
**nonnegative torque capacities in Nm**, independently for each wheel. It must
not modify state. A capacity bounds available resistance; it is not a command
to reverse a stopped wheel and need not equal the torque actually applied.

`advance_state` receives the **signed braking torques actually applied** during
that candidate interval, in Nm, plus mean interval angular velocities and a
positive timestep in seconds. Positive torque acts in the positive wheel-spin
coordinate. These are solved candidate quantities, not private reference
forces. State advances exactly once per physics timestep, including driving,
stationary waiting, and preparation. Component state is a bounded numeric JSON
dictionary; lists, booleans and null are allowed, strings as values are not.

`on_trial_reset` handles a declared pedal-release/reposition intervention: the
host repositions motion and releases the pedal, while accumulated long-term
component state must be retained. A component may clear transient command
activation during this intervention. The function receives and returns the
existing state; it must not silently create a fresh specimen. Only a full reset
calls `init_state` and discards accumulated state.

Preparation uses the exact public command and reposition timeline, replayed in
the candidate's own mechanics. Each candidate state update uses its own solved
torques and motion. No private reference state, private applied torque, or future
outcome initializes or advances candidate state. Public past observations can
inform investigation, but the scalar version-one mass-times-acceleration force
reconstruction is not used for this wheel model.

The starting component has empty state and fixed capacities
`[835, 835, 557, 557] * brake_command`. All outputs must be finite. Angular-speed
magnitudes are bounded by `1e6 rad/s`, torque magnitudes by `1e9 Nm`, and each
timestep by `[1e-9, 60] s`; host experiment limits may be tighter. Invalid
vectors, state mutation, time/resource overruns and malformed output terminate
the isolated component. Only the standard library is available; the component
cannot access reference internals, credentials, the engine or the host project.

`WheelActuatorWorker.source_sha256` identifies the exact source bytes loaded by
that instance, so later edits cannot relabel an earlier prediction. The scalar
version-one contract remains separately available for the original experiment.
