# Dog and drone failure demos

These are reproducible physical failures with recordings for GPT-6 to inspect
and repair later. No model calls or GPT-6-authored repair run when opening them.

## Open a live failure

From the repository root on macOS:

```sh
.venv/bin/mjpython -m simulator view quadruped_gait_failure --camera side
.venv/bin/mjpython -m simulator view drone_delivery_imbalance --camera side
```

Open one at a time. Space starts/pauses and replays after completion. N restarts
the full configured experiment; R starts a retained-condition trial. C switches
cameras; +/− changes pacing only. The drone's live impact flash/smoke is a
presentation effect triggered by contact and applies no physical force.

## Generate immutable recordings

Install the repository's existing frontend dependencies/build first so the
recorder can include a standalone player:

```sh
npm --prefix frontend ci
npm --prefix frontend run build
.venv/bin/python -m simulator.scenario_replay quadruped_gait_failure
.venv/bin/python -m simulator.scenario_replay drone_delivery_imbalance
.venv/bin/python -m dashboard
```

Open `http://127.0.0.1:8765/?view=scenarios`, or use the dashboard's **Failure demos**
link. Each recorder command prints its new directory under
`runs/scenario-replays/<scenario>/<run-id>/`. Open `public/replay.html` directly
for offline playback; keep its adjacent `frames/` folder. The standalone page
embeds the same React player and manifest and makes no HTTP/model calls.

Replay includes play/pause, restart, timeline scrubbing, previous/next frame,
0.25×/0.5×/1×/2× playback, two synchronized cameras and event bookmarks.
**Inspect failure** loops from two seconds before the first observed failure to
one second after. Whole-run replay includes the aftermath. The final frame holds
until replay/restart. These controls do not rerun or change the physical result.

Defaults: 960×540, 30 fps, side and overview cameras captured from each identical
state. `--fps`, `--width`, `--height`, repeated `--camera`, `--no-effects`, and
validated `--set NAME=VALUE` options are available. `--output DIR` uses a new
explicit directory rather than publishing it into the default library. Existing
recordings are never overwritten. Incomplete runs do not publish a manifest.

## What the two failures show

| Scenario | Recorded behavior |
|---|---|
| Dog | Walks about 0.65 m, requests 0.10 → 0.13 m/s at 7–8 s, loses support near 8.48 s and falls near 8.81 s |
| Drone | Carries an offset parcel, takes off near 1.70 s, flies outbound, loses balance, and records impact near 5.52 s |

The dog retains full actuator strength; its faster gait coordinates stance/swing
poorly. The drone retains healthy rotors and a physically attached parcel; its
outbound control mishandles load compensation. Neither outcome is an animated
root trajectory, disabled limb, deleted payload, or scheduled crash.

Development-only feasibility checks establish that the unchanged hardware/task
can be controlled. They are labeled developer controls and excluded from the
failure library's original-attempt selection. They are not GPT-6 repairs. The
drone feasibility check physically places/releases its free-body parcel at B,
returns unloaded and lands; release preserves pose/velocity/mass. Its failed
attempt correctly aborts delivery on impact.

## Handoff to the GPT-6 workstream

Every `public/` directory contains unannotated evidence images, presentation
frames, 100 Hz observations, observed events, the manifest, a task and a neutral
control interface. The manifest includes source/physical-state identities and
timestamps. `private/` contains the exact fixture, compiled model and diagnostics.

Pass the public task, [neutral interface](public/CONTROL_TASKS.md), actual image
content and telemetry to GPT-6. Keep this builder guide, platform implementation
notes and private calibration data outside that investigation context. The
integrating host still needs to expose inspection, bounded experiments and
versioned controller editing; this change supplies the physical tasks/control
surfaces and recordings, not a remote repair broker.

Do not substitute a healthy-world run or better prediction for task repair.
Verify the GPT-6 controller in the same faulty setup and preserve its patch and
executed result. The UI intentionally reports **Astra repair: Not run** until
that workstream supplies such evidence.

## Validation

Focused physical tests cover pre-failure motion, real contact, feasibility,
deterministic reset, timestep sensitivity, parcel release and effect isolation.
Recording/HTTP tests cover same-time cameras, raw/effect separation, immutable
artifacts, incomplete-run handling and public-file boundaries. Node playback
tests and browser checks cover controls and offline playback.

On this checkout the existing complete Python 3.12 test environment is
`/tmp/realitypatch-test-venv/bin/python`; the shared `.venv` is Python 3.11
and lacks pytest. Use a supported Python ≥3.12 environment with the existing
locked dependencies for the complete regression suite. No dependency was added.
