# Editable component interface

The current component predicts a positive braking-force magnitude in newtons.
You may edit `actuator.py`; keep these three functions callable:

```python
init_state() -> dict
compute_force(state, brake_command, velocity) -> float
advance_state(state, brake_command, velocity, applied_braking_force_n, dt_s) -> dict
```

`brake_command` lies in `[0, 1]`. Velocity is in metres/second; time is in
seconds. A fresh specimen calls `init_state`. State persists through driving,
braking, and waiting, and belongs only to this component instance. The force
function must not mutate state. State is a finite JSON object.

`advance_state` is called once per interval, using the mean interval velocity
and the braking force applied after the runner's non-reversal guard. During
history ingestion, force can be reconstructed from the observed velocity
change and known drive force for the specified 1200 kg mechanical body. This
uses past observations, not future outcomes. The future rollout uses only the
component's own computed force and state.

A returned force must be finite and nonnegative. Outputs and execution time
are bounded. Only Python's standard library is available in the component
environment. Reference internals, other runs, credentials, and final outcomes
are unavailable. Candidate state is never initialized from reference state.
