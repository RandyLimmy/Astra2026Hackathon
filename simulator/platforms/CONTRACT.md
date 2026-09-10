# Platform module contract

Each `car_damage`, `quadruped`, and `drone` module exposes:

- `Config`: frozen dataclass. Shared fields: `fault: str`, `duration: float`,
  `timestep: float`, `fault_at: float`, `probe: str`. Validate finite values/ranges.
  Additional fields are platform-specific and can be supplied through JSON.
- `PRESETS: dict[str, dict]`: stable scenario names to Config keyword overrides.
- `DESCRIPTIONS: dict[str, str]`: same names to short human descriptions.
- `Simulation(config: Config)`: physical MuJoCo model; no agent integration.
- Properties: `model`, `data`, `config`, `elapsed: float`, `focus_body: int`,
  `finished: bool`. Named cameras `overview`, `side`, `chase` in model.
- `step(control: dict | None = None)`: one timestep. None selects a deterministic
  nominal probe controller. Control dictionaries permit explicit interventions.
- `observe() -> dict`: includes monotonic `time`, descriptive phase (never hidden
  fault labels), pose, observable velocities, command and platform sensors.
- `diagnostics() -> dict`: private parameters, events, true component state.
- `summary() -> dict`: includes `public` (platform/outcome/safe/metrics) and private
  config/events/warnings. `public.safe` describes only the tested synthetic probe.
- `reset_trial()`: reposition/reinitialize controller for another probe while
  retaining completed damage. Preserve monotonic experiment clock. No hidden repair.
- `reset_full()`: restore initial Config and replay original experiment on same
  model if possible. A fresh Simulation(config) is always a full replay alternative.

Controls and controllers must not read hidden fault parameters to secretly
compensate for them. Fault effects belong in physical force/contact/joint/inertia
updates. All presets need nominal/healthy controls with identical probes and a
fault-free interval before onset. If a phenomenon is simplified (e.g. tire pressure
as contact compliance), label that approximation explicitly.

No new dependencies, no edits to original car modules or the upstream MuJoCo
submodule. Primitive geometry is preferred over downloaded assets. Parent owns
CLI/registry/validation/docs. Platform implementers own their module, unique
assets, and focused tests. Do not recursively delegate.
