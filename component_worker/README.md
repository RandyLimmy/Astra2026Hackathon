# Actuator execution boundary

`ActuatorWorker(Path("candidate/actuator.py"))` copies only that source and the
fixed runtime into a temporary directory. It starts a persistent pure-Python
worker with macOS Seatbelt (`sandbox-exec`), `-I -S -B`, and a new environment
containing no host credentials. The host never imports candidate source.

The filesystem policy permits the copied files, Python's standard library,
specific Python runtime files, and required OS libraries. Site packages,
repository/reference files, writes, network access, and process creation are
denied. The sandbox permits executing the selected Python binary to start the
worker; the same sandbox remains in force if candidate code re-executes it.

Requests and responses are at most 64 KiB. State consists of a bounded numeric
dictionary with identifier keys, lists, booleans, and null values; numbers must
be finite. `compute_force` receives a detached copy and any state mutation is
rejected; history changes belong in `advance_state`. Force must be nonnegative
and at most 1e9 N. Candidate error details are replaced by fixed neutral messages
with an optional host-validated `actuator.py` line number. Each request has a 2-second default
deadline. CPU, file-size, descriptor and core-dump resource limits are set
before loading candidate source. A host watchdog checks resident memory every
10 ms and terminates above 256 MiB. macOS rejects hard address-space/data/RSS
rlimit reductions on this machine, so transient memory overshoot between
samples is possible. This is a local prototype boundary, not a claim that
general hostile native code is safe to execute on a personal computer.

The backend fails closed if macOS Seatbelt or its resource monitor is
unavailable; `sandbox=False` is rejected. Linux/Windows support needs a separate
container or equivalent enforced backend. Keep scored investigation sessions
separate from the builder's repository and conversation: this worker boundary
does not by itself create that separate investigation environment.

`WheelActuatorWorker` uses the same execution boundary for the version-two
four-wheel contract in `contracts/WHEEL_ACTUATOR.md`. The initial JSON request
selects `wheel_v2`; that protocol cannot change during a worker session. Inputs
and outputs are validated as finite four-element numeric vectors on both sides.
`torque_limits` returns nonnegative Nm capacities, while `advance` receives
signed applied Nm torques and mean rad/s wheel speeds. `reposition` invokes the
explicit trial-reset hook without replacing accumulated state. Both worker
classes expose `source_sha256` for the exact source bytes copied at startup.

Run the history, access-boundary, malformed-output and resource-budget checks:

```sh
.venv/bin/python -m pytest tests/test_worker.py tests/test_wheel_worker.py -q
```
