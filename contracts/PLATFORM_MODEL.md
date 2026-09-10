# Editable platform prediction component

The component is pure Python with three required functions:

```python
def init_state() -> dict:
    return {}

def predict_parameters(state: dict) -> dict:
    return {}

def advance_state(state: dict, observation: dict, dt_s: float) -> dict:
    return state
```

`init_state` creates a fresh specimen's memory. The host worker's `reset()` calls
it again; the component must put all evolving memory in the returned state.
`advance_state` receives only public observations from its own predicted rollout
and elapsed seconds. The first callback may have `dt_s = 0` to supply initial
context without elapsed time. Subsequent updates evolve the state. Observations
may include a string phase and nested public pose/sensor values; the host's
platform capabilities describe their exact schema. Trial preparation is explicit
in those observations. There is no implicit reset hook.

`predict_parameters` reads state and returns a flat dictionary of parameter names
to finite numbers. Empty or partial maps retain nominal values for unspecified
parameters. The supplied `capabilities()['model_parameters']` describes each
platform's allowed keys, units and physical bounds; the host checks these before
applying a prediction. The generic worker permits at most 64 identifier keys of
up to 64 characters and numeric magnitudes at most `1e9`. Booleans, strings, nested
values and nonfinite numbers are not parameter values.

A parameter query must leave state unchanged, including nested objects and any
retained aliases. It receives a detached state copy; the worker checks that both
the copy and original state are unchanged. Put evolution in `advance_state`, not
in parameter queries or module globals. Repeated queries with unchanged state
must produce the same result.

State is a dictionary of numeric JSON values, nested lists/dictionaries, booleans
or null. String values are reserved for observations, not state. All dictionary
keys are identifiers of at most 64 characters. State and observations have at
most 2,048 values and nesting depth 8; numbers must be finite with magnitude at
most `1e15`. Observation strings are at most 4,000 characters each, and the full
observation is at most 16,384 UTF-8 bytes when serialized as compact JSON. Time
increments range from 0 through 60 seconds. Host input rejection does not advance
the component. Each complete protocol message is limited to 65,536 bytes.
Invalid component output or protocol failure closes the worker.

The host API is `PlatformModelWorker(source_path)`, with `parameters()`,
`advance(observation, dt)`, `inspect_state()`, `reset()` and `close()` methods.
`advance` returns the resulting state. The `source_sha256` property identifies
the exact source bytes copied into the isolated process. It supports a context
manager. Errors expose fixed messages and, when available, an own-source line
number using the isolated filename `actuator.py`.

Execution uses the existing enforced macOS sandbox: a read-only temporary
directory containing only the source and fixed runtime, standard-library access,
no repository or site packages, no inherited credentials, network or process
creation, and no filesystem writes. A call has a wall-time budget (2 seconds by
default); the persistent process has a cumulative 120-second CPU limit, a
256 MiB resident-memory watchdog, and file/descriptor limits. The watchdog samples
every 10 ms, so brief memory overshoot is possible. Unsupported OS isolation
fails closed. Printing is discarded. The host never passes engine objects,
private reference state, or future reference outcomes to the component.
