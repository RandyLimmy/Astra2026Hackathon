# Car failure animations: F3 and F4

These are original physical failures for a later GPT-6 investigation. No repaired
controller, model calls or claimed GPT-6 success is included. Existing post-crash
car presets remain separate.

## Watch the recordings

```sh
.venv/bin/python -m simulator.failure_replays all
```

This records both cars at 24 fps with synchronized chase/overview images. Open the
printed `index.html` directly in a browser; no server or network is needed. The
same recordings appear in the dashboard's Failure lab. Files default to
`runs/scenario-replays/<scenario>/<run-id>/public/`. Use `--output NEW_DIRECTORY`
for a separate bundle, or replace `all` with either scenario name.

The player opens paused. Play/pause, restart, scrub, frame-step and 0.25×/0.5×/1×/2×
are available. Camera switching preserves time. **Inspect failure** loops from
two seconds before the measured failure to one second after. Whole-run playback
includes aftermath and holds the final image. Playback never reruns the engine.

For live MuJoCo reruns on macOS:

```sh
.venv/bin/mjpython -m simulator view car_steering_drift
.venv/bin/mjpython -m simulator view car_auto_brake_failure
```

Space plays/pauses, N resets the original experiment, R repeats while retaining
the current component state, C switches cameras, and −/+ changes speed. N/R are
new physical attempts; they are distinct from immutable recorded playback.

## F3: steering leaves the lane

The car starts at 4 m/s in a marked lane. The original route controller attempts
a straight entry and gentle bend toward a raised green/checkered finish gate.
Its steering response differs from the assumed mapping; tires, brakes and
suspension stay healthy. It crosses the tire boundary, remains outside for
0.5 s, and makes physical contact with the roadside rail. Contact triggers
braking and two seconds of physical aftermath, then the run holds with
`FAILED: ROADSIDE COLLISION`. There is no prior crash or recovery teleport.

Public evidence includes route/finish geometry, heading and lateral errors,
steering commands, measured front wheel angles, wheel motion and tire-boundary
events. `Simulation.step({"steering": radians, "throttle": pedal, "brake": pedal})`
accepts bounded controls: steering −0.5…0.5 rad, pedals 0…1. Omitted controls use
the original route controller in `car_steering.py`.

Measured revised default: bend entry **2.512 s**, sustained lane exit **4.122 s**,
roadside collision **4.790 s** at **3.978 m/s**, terminal hold at **6.790 s**.
The raised finish gate remains visible ahead and is never reached. The explicitly labeled
`car_steering_nominal` developer control verifies the route is physically feasible
with healthy steering. It is not a repair of the faulty car.

## F4: automatic braking hits the barrier

Four actual accelerate/brake cycles condition the existing thermal brake plant.
The approach resets position and speed, retains the brake state, then releases
the car at 25 m/s. Its original position-based automatic trigger applies full
braking at the same point as the cold calibration. Reduced actual braking causes
positive-speed barrier contact and 2 s of physical aftermath. The forward-facing
chase view shows the striped wall from the opening frame. A green target zone
marks 86–98 m on the default track: stop there with at least 2 m clearance.
The collision run holds with `FAILED - BARRIER COLLISION` at **6.332 s**.

Public evidence includes speed/deceleration, wheel speeds, pedal commands, ideal
simulated bumper-to-barrier range, contact and complete conditioning/reset history.
`Simulation.step({"throttle": pedal, "brake": pedal})` accepts pedals 0…1;
omitting controls uses the original trigger. The plant in `simulator/runner.py`
is unchanged and no candidate actuator replaces its brake capacity.

Measured default: **51.730 s** conditioning, brake onset **1.722 s** into the
approach, contact **4.332 s** at **17.177 m/s**. A collision is censored: free
stopping distance is null. Developer controls `car_auto_brake_cold_control` and
`car_auto_brake_wall_free` stop 7.57 m clear and measure a 104.51 m conditioned
free stopping distance respectively. These are diagnostic controls, not fixes.

## Evidence and later controller work

The public manifest pairs the actual images and frame-aligned samples. Frame
`t_s` and timeline events use approach-relative seconds; observations retain
absolute experiment time. `history_events.json` retains absolute reset/event
times, `preparation.json` preserves ordered commands and resets, and the 100 Hz
`observations.jsonl` includes conditioning. Conditioning is explicitly labeled
but has no video chapter in this milestone; the animation begins at approach.

Private records contain actual component diagnostics, initial engine state,
configuration and source hashes. Keep them outside the model's evidence bundle.
Unannotated camera images are the evidence; no image-generation model invents
poses or failures. The manifest identifies the original attempt and records
`repair_status: not_run`.

A later integration can expose the neutral observations/control bounds above,
let GPT-6 inspect the starter controller, choose probes and patch its code, then
record a fresh run with the **same** steering response or brake-conditioning
history. The corrected run must physically complete the lane or avoid the wall;
changing the plant, changing the task or predicting a better result is not a pass.

## Verification

`tests/test_car_steering_failure.py` and `tests/test_car_braking_failure.py` cover
physical outcomes, diagnostic controls, repeatability, half-timestep checks,
retained-state resets and bounded commands. `tests/test_car_failure_replays.py`
covers synchronized cameras, unchanged dynamics, conditioning history, private
data separation and rejection of overwrites/incomplete recordings.

## Verification record — 2026-09-10

- F3: 9 focused tests passed; F4 plus legacy actuation: 24 passed. Both include
  repeated identical failures and a half-timestep comparison. Car recorder and
  shared replay/API tests: 17 passed. Five frontend playback tests passed.
- Chromium verified both exported players: paused start, synchronized camera
  switch, scrubbing, frame stepping, inspection loop, playback rates, final-frame
  hold, 390 px mobile layout, no JavaScript errors and no network requests.
  The dashboard loaded and played both cars from its four-scenario registry.
- `npm run build` passed. Ruff passed on the changed car/replay/API files;
  targeted mypy passed on the two car platforms, registry and recorder.
- Broader Python verification ran 357 cases excluding `test_baselines.py`:
  351 passed, two failed, four had setup errors. Five failures/errors came from
  incompatible installed NumPy/Matplotlib binaries; the remaining failure was
  in concurrently modified warehouse physics. Full collection also encounters
  an installed NumPy/SciPy incompatibility in `test_baselines.py`.
- Full-repository lint reported two pre-existing test-style issues; full mypy
  encountered seven errors in concurrent `investigation/warehouse.py` work.
  Those unrelated changes and the shared Python environment were not modified.

The venv has the working MuJoCo/NumPy runtime but lacks pytest. Verification used
its interpreter with the already installed Anaconda pytest directory appended to
`sys.path`; no dependencies were added or installed. This exposes the legacy
plotting/scientific binary incompatibilities above, which do not affect the
recorded car simulations or browser playback.

## Goal and conclusion revision

Both live MuJoCo windows now display GOAL and measured mission status throughout
the attempt. Terminal attempts hold with an explicit result rather than a generic
completion label. The same public-state presentation is captured per frame for
browser replays, so scrubbing cannot show a future conclusion on an earlier frame.

The steering defect stays fixed; new roadside rails are physical geometry outside
the valid lane, and the healthy developer control still crosses the finish gate
without contact. Braking visual props and cameras do not change its underlying
trajectory or thermal response; an exact-state regression checks this. Neither
revision adds a GPT-6 repair. Old recordings are retained; use the latest replay
from the dashboard or the new index printed by the recorder.

Revised-run verification: 10 steering physics tests passed. The braking suite
passed 19 checks, followed by 5 fresh checks after adding explicit target-zone
completion (20 distinct tests total). Replay/native-HUD tests passed 12 checks;
legacy platform viewer/operator tests passed 8. Six browser playback unit tests,
frontend build, changed-file Ruff and targeted mypy passed. Chromium verified
both fresh recordings' controls and held collision conclusions. Native MuJoCo
rendering verified that GOAL, collision result and the vehicle are visible.

Primary files changed for this revision: `platforms/car_steering.py`,
`platforms/car_braking.py`, `view_controls.py`, `platforms/operator.py`,
`failure_replays.py`, `assets/failure_replay.html`, and the frontend
`ScenarioReplay.jsx` / `scenario-replay.css`, with focused tests. The changes
reuse the original controllers and braking plant. F4's scene props apply no
forces; F3's roadside rails intentionally add real contact outside the valid lane.
