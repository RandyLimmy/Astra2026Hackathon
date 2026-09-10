# RealityPatch: four failure animations and replays

Status: planning only, 2026-09-10. Define all four requested simulations before implementation. No simulator, animation, controller, or Astra solution code is changed in this planning pass.

## Requirements and delivery boundary

The immediate deliverable is this plan. The next implementation milestone is **four clear failure animations with recorded replay**. Finish and validate that milestone across every scenario before implementing Astra's repair loop.

The eventual presentation is: watch the failure, inspect its replay, let GPT-6 Astra examine visual and engine evidence, observe its actual tool calls and source changes, then replay the corrected task.

| ID / proposed preset | Required failure story | Eventual corrected task |
|---|---|---|
| F1 / `quadruped_gait_failure` | Dog walks, speeds up slightly, loses limb coordination, stumbles and falls | Corrected limb timing and feedback keep it walking at the requested speed |
| F2 / `drone_delivery_imbalance` | Drone carries an off-center parcel toward B, tilts, loses control, crashes and shows an impact explosion effect | Balance the load, fly A → B, deliver the parcel, return unloaded to A and land |
| F3 / `car_steering_drift` | Car tries to follow the lane but steering miscalibration sends it off course | Correct steering response and follow the same course |
| F4 / `car_auto_brake_failure` | Car approaches a barrier and brakes too late for its actual capability, causing a collision | Learn braking response and automatically brake early enough to stop with clearance |

“Learns” means inference and an executable controller/component change during the investigation, followed by measured verification. It does not imply retraining model weights. In F1, the incomplete gait controller is responsible for coordination; MuJoCo supplies the physical joint/contact response.

Reuse existing MuJoCo assets, Python runtime, recorder, native controls and React dashboard. Add no dependencies and make no upstream `mujoco/` edits. Keep existing warehouse and legacy car fault presets available; they are outside these four headline demos. The historical S0–S6 car plan is preserved in [the car foundation archive](simulator-scenarios-car-foundation-20260910.md). This document supersedes its delivery order.

## Current repository evidence and gaps

References describe inspected source, not fresh simulation results. Existing test coverage has not been rerun for this document.

| Area | Reusable foundation | Gap |
|---|---|---|
| Dog | `simulator/platforms/quadruped.py:60` defines the existing collapse demo; `:140` accepts external controls; `:278` computes nominal gait targets | Existing fall comes from severe knee weakness. Add a speed transition and controller coordination failure with functioning limbs |
| Drone | `simulator/platforms/drone.py:80` defines the moderate flight demo; `:197` defines its route; `:256` handles faults | Existing showcase uses rotor loss and stays airborne. Add a physical offset parcel, delivery/release/return mission and impact effect |
| Steering | `simulator/platforms/car_damage.py:190` applies steering damage; `:217` supplies nominal control; `:235` accepts commands | Current demo requires impact/repositioning. Add an independent lane trial starting with a steering mismatch |
| Braking | `simulator/runner.py:139` selects scheduled commands or a position-based brake trigger; `:167` applies controls | Later add an observation-driven automatic-braking interface |
| Recorder | `simulator/recording.py:51` separates public observations/frames from private diagnostics | Add shared mission events, camera tracks and a manifest for all four demos |
| Native viewer | `simulator/platforms/operator.py:155`, `simulator/view_controls.py:118` supply replay, camera and speed controls | Keep full rerun and retained-damage trial distinct from immutable recorded playback |
| Browser replay | `frontend/src/components/Replay.jsx:16` loads reserved cases; `:91` seeks; `:119` supplies playback controls | Currently car-specific. Add failure recordings independent of API investigations, platform metrics and later paired replays |
| Media | `dashboard/media.py:25` fixes three car cases/tracks; `:48` reconstructs verified car rollouts | Add platform manifests/cameras while retaining source/outcome verification |
| Astra | `investigation/broker.py:39` exposes seven actuator tools; `investigation/runner.py:139` builds text evidence | Existing images never enter the investigator (`dashboard/media.py:1`). Visual evidence and controller execution need later integration |

## Shared animation and replay design

Every demo opens paused with a one-sentence objective and visible route/goal. Make the subject large enough to see the fault while retaining route context. Show concise mission/outcome labels; keep detailed diagnostics in a separate operator view.

First milestone:

`Ready → normal motion → visible loss of control → physical failure → hold final frame → replay`

Later:

`Replay evidence → Astra inspects/tests → source patch → fresh run on the same faulty setup → measured outcome → synchronized before/after replay`

Show an explicit reset between attempts. A wrecked drone does not resurrect mid-flight. Corrected attempts retain the original payload offset, steering defect or braking history. For the dog, mechanics and speed history stay fixed while its controller changes.

### Playback contract

- Play/pause, restart, scrub, single-frame stepping and 0.25× / 0.5× / 1× / 2× playback. Preserve native Space, N, R, C and speed keys. Scrubbing/event jumps operate on recordings; native N reruns the experiment and R explicitly retains damage.
- At least two recorded camera views per scenario: context and fault-revealing. Capture both from the same physics state before advancing. Switching cameras preserves time.
- Timeline markers for task start, command transition, first instability, lane exit/contact, delivery and outcome as applicable. Derive observed events from recorded state; never force an impact at a storyboard timestamp.
- “Replay failure” plays the whole run. “Inspect failure” loops 2 s before the first failure marker to 1 s after, clipped to available footage. Whole-run replay includes aftermath.
- Playback reads immutable frames/telemetry, makes no model calls and cannot change commands, physics or results. Cached replay works after closing the simulator and without network access.
- Later “Before” and “After” share a simulation-time clock and initial task. A shorter run holds its final frame with its own end time visible. Optional event alignment must be labeled.
- Initial cards show “Failure replay ready” or a concrete missing-media state. Astra repair remains “Not run” until actual evidence exists. Developer control, model prediction and executed repaired run retain separate provenance.

### Recording contract

Extend the recorder with a versioned manifest: opaque run ID, platform, task, initial-state identity, seed, physics/asset version, controller source hash, simulation-time range, camera IDs, timestamped frames, telemetry, public events, outcome and provenance. Keep private fault parameters and validation diagnostics in private artifacts.

Persist actual controls and every reset/conditioning boundary. Public events describe observations such as “lost balance” or “contact”; revealing fault labels/source paths/private overlays never enter Astra's input. Effects also record their impact event ID and random seed.

## F1 — Dog: faster walking breaks limb coordination

**Task:** walk along a marked strip. Starting calibration proposal: increase requested speed from 0.10 to 0.16 m/s over 1 s, with healthy joint strength and unchanged ground contact. These are proposed settings, not measured stable/failing outcomes.

| Approximate simulation time | Visible action | Camera / evidence |
|---|---|---|
| 0–3 s | Stand, shift weight, start walking | Side three-quarter view; all feet visible |
| 3–7 s | Steady walking; requested speed increases near the end | Ground marks and requested/actual speed |
| 7–11 s | Stance/swing timing stops coordinating; foot scuffs, support shifts, torso pitches | Side and front-quarter views; measured foot-contact highlights |
| 11–15 s | Stumble, body contact, settle and hold | Keep feet and torso visible through the fall |

**Failure mechanism:** add a bounded speed schedule and an intentionally incomplete gait controller whose stance/swing transition or phase update fails during acceleration. Select one defect after a feasibility probe and freeze its source/configuration. Do not also inject the existing 8% knee-strength collapse, which tells a different story and may be impossible to repair in software. Commands act through joint actuators; no torso dragging or animated root translation.

**Later evidence:** synchronized frames, joint positions/velocities, commanded targets, declared foot-contact signals, body orientation/velocity, requested speed, joint limits and neutral actuator interface. Highlight the stumbling limb from observations without supplying a private cause label.

**Planned repair:** inspect gait source and limb mapping, request low-speed/transition probes, correct stance timing, foot clearance and feedback within actuator bounds, then rerun. Success means sustained walking.

**Acceptance targets:** at least 0.4 m forward walking before the transition; a recorded scuff/support-loss event followed by sustained torso contact or tilt above 45° in the failure. A separately labeled feasibility control completes the identical speed schedule without falling. Later repaired execution: 10 s after the transition without torso contact, roll/pitch below 20°, mean speed within 20% of the request over the final 5 s, lateral deviation below 0.25 m. Confirm model axis/sign conventions during calibration.

**Variants:** primary transition; second development transition time; held-out intermediate speeds and a different transition time. Actuator weakness remains a distinct optional extension.

## F2 — Drone: uneven delivery load causes a crash

**Task:** carry a visible parcel from A to B, set it down, release it, return unloaded to A and land. Start with pads approximately 4 m apart and 2–3 m cruise altitude; calibrate against current scale and rotor authority.

| Approximate simulation time | Failure animation | Eventual successful replay |
|---|---|---|
| 0–3 s | Parcel mounted visibly to one side before takeoff; A/B and route visible | Identical parcel mass/offset |
| 3–6 s | Takeoff with uneven attitude; leave A | Bounded differential rotor thrust balances load |
| 6–11 s | Translate toward B, growing roll, altitude loss and physical ground impact | Reach B, stabilize and descend |
| Impact + 0–2 s | Brief flash/fire/smoke at contact; wreck settles; mission fails | Parcel supported at B before release; parcel stays there |
| 13–24 s, success only | Failed mission has ended | Rebalance after release, climb, return to A, land and hold |

**Failure mechanism:** healthy rotors plus a rigid parcel attached off-center. The nominal controller assumes centered loading or lacks adequate trim adaptation. Choose mass/offset that produces visible instability and a reproducible crash while remaining recoverable by bounded control. Do not substitute rotor loss.

**Parcel/mission physics:** create the parcel before the run with physical mass, inertia and attachment. Contact supports it at B; releasing the attachment creates independent motion with continuous pose/velocity. Never delete mass at a waypoint or teleport cargo. Mission states: takeoff, outbound, approach, stable placement, release, return and landing. Crash/timeout aborts the mission.

**Explosion:** deterministic presentation triggered once by a recorded crash impact after departure, above a frozen severity threshold. Takeoff contact, gentle parcel placement and successful landing cannot trigger it. Use a short flash/sparks then smoke without hiding the impact. Fire/smoke is an illustrative effect, not simulated combustion/blast physics. Effects apply no forces or outcome changes.

**Later evidence:** side/front images showing parcel and lean, attitude/rates, position, mission target, motor commands, allowed rotor-response measurements, contact observations and attachment command state. Supply neutral body/actuator mechanics and units without giving the true private mass offset.

**Planned repair:** inspect rotor mapping, compare hover and short-motion probes, infer load compensation, patch thrust allocation/attitude feedback, then verify loaded outbound and unloaded return. Default to differential thrust. Moving or adding counterweights requires an explicitly modeled mechanism and is outside this first design.

**Acceptance targets:** failed run moves at least 1 m horizontally toward B, visibly tilts, loses altitude and impacts; effect appears only on that impact. A feasibility control completes A → B → A with identical offset/limits. Later repaired execution: no drone-body ground contacts except controlled pad landings, parcel within 0.5 m of B for 2 s after release, final position within 0.5 m of A, final speed below 0.1 m/s for 1 s, full mission within 30 s. Verify parcel mass and pose continuity across release.

**Variants:** centered-load control; offset-load primary; held-out opposite-side offset and changed parcel mass. Returning unloaded tests the load-removal transition, preventing a fixed outbound trim from counting as a full solution.

## F3 — Car: steering sends it off course

**Task:** follow a marked lane with a straight entry, gentle bend and finish line. Separate scenario card and replay from braking.

| Approximate simulation time | Visible action | Camera / evidence |
|---|---|---|
| 0–3 s | Enter straight lane | Chase view with lane/route |
| 3–7 s | Attempt bend; steering differs from intended response; error grows | Overhead path view; commanded/observed wheel-angle inset |
| 7–11 s | Cross boundary onto shoulder or into a physical roadside barrier | Wide view through departure/contact and aftermath |

**Failure mechanism:** steering bias/gain mismatch applied as a declared initial condition. Nominal route control assumes healthy steering. Keep tires, brakes and wheel attachment healthy. Reuse car-damage mechanics but add start-in-probe behavior; this scenario needs no unrelated crash/reposition first.

**Later evidence:** route, frames, speed, yaw/rate, lateral error, steering command and observed wheel angle. Wheel angle is a declared sensor/visual measurement, not a private bias parameter.

**Planned repair:** inspect units/sign/mapping, run small left/right probes, infer bias/gain, patch calibrated steering plus heading/lateral feedback, then execute the same route and defect.

**Acceptance targets:** failure leaves at least one tire outside the lane for 0.5 s, visibly recorded. Later repair completes the course within 15 s, maximum centerline error at most 0.30 m, heading error below 5° after the first 2 s, no tire outside lane, no barrier contact. Stopping indefinitely is not a pass. Calibrate lane width/curvature with a bounded feasibility control.

**Variants:** primary rightward bias; gain-error development probe; held-out opposite bias and changed bend/speed within the frozen feasible range.

## F4 — Car: automatic braking learns stopping response

**Task:** approach a stationary barrier at a declared speed and stop before it. Reuse the existing four-wheel braking platform and thermal-history experiment as the primary mismatch.

| Approximate simulation time | Failure animation | Eventual successful replay |
|---|---|---|
| 0–3 s | Approach; barrier and bumper clearance visible | Same speed, barrier and conditioning |
| 3–6 s | Cold-calibrated trigger engages brakes, but actual deceleration is weaker | Controller brakes earlier using observed/estimated response |
| 6–9 s | Positive-speed barrier contact and aftermath | Stop with visible gap and hold |

**Failure mechanism:** repeatable heating reduces brake torque; the old cold-calibrated trigger becomes too late. Conditioning is a labeled replay chapter with complete telemetry for Astra; the main animation can start at approach. Never hide resets or cool brakes for the successful run. Use cold and wall-free controls.

**Later evidence:** frames, bumper clearance, speed/deceleration, wheel speeds, pedal commands and full conditioning/rest history. A collision censors free stopping distance; use a labeled paired wall-free probe to measure it.

**Planned repair:** inspect the brake component, compare cold/hot/rested probes, repair its response estimate if needed, then build automatic braking that uses speed and remaining distance to select onset/strength. It emits bounded throttle/brake each control interval. A better predicted stop alone is insufficient: the next physical approach must avoid impact.

**Plant boundary:** the final avoidance run keeps the trusted thermal/brake plant and conditioning unchanged. Candidate brake-response code supplies estimates to the controller; only its bounded pedal commands reach the reference plant. The existing `Simulator(actuator=worker)` path replaces brake capacities and remains a prediction path, not proof that the unchanged car avoided the barrier (`simulator/runner.py:167`).

**Acceptance targets:** baseline has positive-speed barrier contact. Later correction under identical heating/approach has zero barrier contacts, final bumper clearance at least 2 m, speed below 0.1 m/s for 0.5 s, completion within 15 s excluding conditioning. Require forward travel before braking and freeze an upper clearance bound after calibration to exclude stopping immediately/far away. Freeze all bounds before Astra runs.

**Variants:** cold control; hot primary failure; recovery control; held-out intermediate speed and unseen heating/rest history. Wet-road and lag variants remain in the legacy suite until this core story works.

## Later Astra integration — design now, implement after all failures

Reuse source versioning, bounded experiments, isolated execution, event logs and frozen evaluation. Existing model profile: `investigation/api.py:11`. Proposed extensions below are not current capabilities:

1. **Inspect:** extend `inspect_model` or supply neutral engine-interface documentation: component source, joint/rotor mapping, units, sensors and command limits. Exclude private faults/held-out outcomes.
2. **Observe visuals:** extend `observe_run` with bounded time-window/camera/frame requests. Supply pre-failure, onset, instability and outcome images plus a short adjacent sequence. Deliver actual image content to the multimodal request; paths/narration do not count. Save precisely which images/telemetry were sent.
3. **Probe:** extend `run_experiment` with platform-specific bounded maneuvers selected from the agent's hypothesis and evidence.
4. **Patch:** extend patch/worker contracts to one scoped controller per platform, plus brake-response code where needed. Proposed interface: initialized state, observations + declared task target/history in, bounded commands + updated state out. Allow observable conditioning/rest/reset history for diagnosis. Forbid access to private truth and mutation of task goals, reset records or evaluator thresholds.
5. **Execute/verify:** run patched controls in the physical plant under the same fault; record commands, source hash and outcomes. Keep prediction separate. Freeze before held-out evaluation and replay actual results, including unsuccessful attempts.

The activity panel uses real logged actions: inspecting stumble images, probing rotor response, changing steering mapping. “Astra fixed it” requires an API-authored artifact and a passing executed run. Feasibility controls stay labeled developer-authored. Investigation latency is separate from simulation-time playback.

## Future implementation sequence and file ownership

Only this plan and its historical archive are written now.

| Order | Work / files | Completion gate |
|---|---|---|
| 1 | Freeze IDs, recording/events, sensors and criteria in `simulator/contracts.md` and `simulator/platforms/CONTRACT.md`; retain current preset tests | Four explicit tasks, failure predicates, reset rules and proposed success bounds |
| 2 | Prototype failure mechanisms in existing platform modules/assets and `simulator/runner.py`; additive registry entries in `simulator/platforms/catalog.py` and `simulator/scenarios.py`, with matching CLI dispatch in `simulator/__main__.py` | All four fail for their stated cause and have a same-physics feasibility control; no Astra repair code |
| 3 | Complete drone parcel/mission in `simulator/platforms/drone.py` and `simulator/assets/platforms/drone.xml`; dog speed schedule; independent steering course; conditioned braking trial | Full physical timelines; drone control proves real delivery and unloaded return |
| 4 | Extend `simulator/recording.py`, `simulator/platforms/operator.py`, `simulator/view_controls.py`; impact-only effects and camera/route assets | Every failure recording includes context, onset and aftermath; effects never alter traces |
| 5 | Adapt `dashboard/media.py`, `dashboard/server.py`, `frontend/src/components/Replay.jsx`, `frontend/src/components/SpeedChart.jsx` and supporting API/UI files | All four cards open without investigation; scrub, frame-step, loop, speed and camera controls work offline |
| 6 | Calibration matrix, numerical checks, saved visual review, replay tests; update `simulator/DEMO_GUIDE.md` | **All four failure animations and replays pass together before the solution milestone starts** |
| 7, deferred | Extend `investigation/broker.py`, `investigation/runner.py`, worker/contracts and `frontend/src/components/Investigator.jsx`; paired playback | Actual Astra evidence → patch → same-fault rerun → held-out result for each scenario |

Dispatch detail: `simulator/platforms/catalog.py:8` and `:21` currently route every `car_` prefix to the car-damage platform. Add an explicit mapping for `car_auto_brake_failure` to the legacy four-wheel `Simulator`, consistently across list/run/view/compare and the replay loader; preserve all existing car-damage routes. Test this distinction so F4 cannot accidentally run on the steering platform.

During implementation, independent ownership can be dog, drone, car scenarios and shared replay/integration. Settle shared contracts before integration. Preserve the current car prediction benchmark and warehouse behavior.

## Verification and acceptance gates

All numerical values above are proposed calibration targets. Record final settings and any revised threshold with its reason **before** running Astra. Do not tune thresholds to fit the model's result.

- **Coverage:** four primary failures, four feasibility controls and at least two held-out cases per scenario. Recalibrate physically impossible fixtures before integration.
- **Causality:** disable the selected mismatch and recover control behavior; verify the declared mismatch exists in failures. Do not combine hidden faults to manufacture a crash.
- **Repeatability:** three runs per frozen case agree on event times within one physics step and final position within 1 cm / attitude within 0.5°. Reject NaN/Inf, automatic resets and MuJoCo warnings.
- **Numerics:** halving timestep preserves outcome labels and changes primary distance/tracking metrics by at most 2%. Move marginal fixtures away from thresholds. Capture rate must not change physics.
- **Replay:** frames/telemetry/events align within one frame; capture at 30 fps by default. Seeking/restarting/camera changes reproduce the same pose and event. Exercise final-frame hold, unavailable media and offline reload for all platforms.
- **Visual review:** inspect full motion plus saved context/onset/failure/final frames. Dog feet/torso, drone parcel/lean/impact, car lane exit and bumper contact/clearance must be legible. During implementation use `$visual-verdict` against the storyboard criteria and save JSON to `.omx/state/failure-scenarios-visual/ralph-progress.json` before the next visual edit.
- **Effects:** gentle landing/parcel contacts never trigger explosion; scrubbing does not duplicate it; replay reconstructs the same effect; on/off runs have identical physics.
- **Later integration:** actual image content enters Astra's request, controller outputs reach the plant, hashes match the replay, and success labels use executed criteria. Test observation filtering and separation of development/held-out evidence.

Reuse/extend `tests/test_quadruped.py`, `tests/test_drone.py`, `tests/test_drone_showcase.py`, `tests/test_car_damage.py`, `tests/test_car_actuation.py`, `tests/test_showcase_profiles.py`, `tests/test_recording.py`, `tests/test_platform_viewer.py`, `tests/test_view_controls.py`, `tests/test_dashboard_media.py` and `tests/test_dashboard_server.py`.

Proposed new tests: `tests/test_failure_scenarios.py` for physical/mission outcomes and `tests/test_scenario_replay.py` for cross-platform replay. Frontend has a build script but no configured test runner (`frontend/package.json:6`); use available browser/manual QA and existing server tests without implicitly adding dependencies.

After implementation run focused tests, full Python suite, configured Ruff checks, mypy for `simulator`, and frontend production build. Fix relevant regressions and capture evidence. This planning pass checks documents/references; it does not claim runtime checks were executed.

## Risks and mitigations

| Risk | Mitigation |
|---|---|
| Dog falls from an unrepairable broken limb | Healthy mechanics, isolated coordination defect, same-physics feasibility control, fixed speed requirement |
| Drone load exceeds thrust authority or delivery is merely a waypoint | Bounded feasibility calibration; physical parcel/contact/release; verify loaded outbound and unloaded return |
| Existing car flow conflates steering, crash damage and braking | Separate cards; initial steering fault; braking stays on established brake platform |
| Explosion hides cause or invents outcome | Trigger after measured crash impact; independent physics/outcome; retain visible contact |
| Reset removes fault | Saved task/state/conditioning identity, same-fault rerun, explicit reset events, immutable failure recording |
| Prediction or builder control is presented as Astra repair | Separate provenance; actual images, tool log, source patch and passing executed run required |
| Solution work leaves animations unfinished | Gate repair integration on four complete failure animations and replays |

Planning completion: all four stories, causes, animations, replay behavior, later repair boundaries, file ownership, outcomes and verification are specified. Implementation remains deferred.

Review note: independent read-only review found no material issues after the history-access, dispatch and unchanged-brake-plant corrections. The shared `simulator-scenarios.md` changed concurrently after that review; this separate file preserves the reviewed animation plan. The concurrent plan is left untouched.
