# Simulator failure demos and Astra tooling plan

Status: **planning only**, 2026-09-10. This revision specifies four simulations, their animations/replays, and the documented tools through which GPT-6 Astra can later investigate and correct behavior. No simulator, controller, agent integration, or animation is implemented by this revision. All new numerical targets below are proposed calibration targets, not measured results.

## Requirements and scope

Build the experimental playground: reproducible physical failures, readable animations, replayable evidence, engine/control documentation, editable controller interfaces, and tool calls for inspection, experiments, changes, and verification. Astra later chooses hypotheses, implements corrections, and tests them. Here, “learns” means that investigation process; model training is not required.

The four primary stories are:

1. A robot dog tries to walk slightly faster, coordinates its limbs badly, and stumbles. Astra can inspect feet/joints/body motion and change the gait/controller so it completes the faster walk.
2. A delivery drone carries a visibly off-center package from A toward B, becomes unstable, crashes, and shows a brief explosion effect after impact. Astra can inspect flight/engine evidence and change thrust allocation/control so the same loaded drone delivers at B and returns to A.
3. A car steers incorrectly and drifts out of its lane. Astra can inspect commanded versus actual steering and adjust the controller to complete the route.
4. A separate car experiment brakes too late or incorrectly and strikes an obstacle. Astra can inspect approach/braking evidence and implement automatic braking that completes the approach and stops with clearance.

**A better dynamics prediction alone does not satisfy these stories.** The new editable artifact must command the simulated robot/car in the unchanged task world. Existing brake-model repair experiments remain available as a separate capability.

Deliverables in this planning pass:

- This implementation plan and acceptance matrix.
- [Animation and replay storyboards](simulator-replay-storyboards.md).
- [Proposed tool and controller contract](simulator-tooling-contract.md), including example invocation sequences.
- [Documentation and handoff specification](simulator-documentation-map.md).

The [original S0–S6 car plan](archive/simulator-scenarios-car-original.md) is archived. Its brake fade, wheel loss, wet road, payload, weak brake, and actuator lag families remain regression/extension cases. Existing warehouse scenarios also remain available. Neither warehouse expansion nor new scenario families displace these four deliverables. Earlier implementation notes describe existing work; this revision controls the next scope.

## What exists, and what is missing

These are inspected baseline facts, not new verification results. Source references use repository-relative `path:line` notation at commit `803fa7f`, which was HEAD during initial inspection. Other simulator source edits appeared concurrently in the shared workspace; this planning pass does not implement or assess those edits. Recheck source offsets and reconcile those changes when implementation begins.

| Area | Existing foundation | Gap for this request |
|---|---|---|
| Dog | Twelve-joint physical dog; commandable speed/yaw/joint targets; current demo injects 8% knee strength (`simulator/platforms/QUADRUPED.md:23`, `simulator/platforms/quadruped.py:278`) | Faster-gait coordination failure, editable gait configuration/source, progress-based completion |
| Drone | Four bounded rotor commands and target-position control; demo has moderate rotor degradation; payload capture is idealized (`simulator/platforms/DRONE.md:13`, `simulator/DEMO_GUIDE.md:14`) | Lateral payload offset, visible physical package/latch, delivery/drop/return mission, load-aware editable controller, impact-triggered effect |
| Steering | Physical steering/suspension, impact-gated damage, recovery reposition, public steering/throttle/brake (`simulator/platforms/CAR_DAMAGE.md:8`, `simulator/platforms/CAR_DAMAGE.md:36`) | Standalone lane-following task, fixed route, inspectable and editable steering controller, lane completion metrics |
| Braking | Physical wheel braking, histories, wall collision and censored stop reporting (`simulator/contracts.md:12`, `simulator/contracts.md:89`) | Live automatic-brake controller driven by allowed observations; obstacle detection/range contract and task progress gates |
| Experiments | Shared platform interface and recorder; timed lab schedules only for warehouse/drone (`simulator/platforms/CONTRACT.md:3`, `simulator/lab.py:28`, `simulator/lab.py:38`) | Uniform agent experiment contract across all four demos; controller execution and same-world comparisons |
| Replay | PNGs/timestamps; live restart/pause/camera/speed controls (`simulator/recording.py:51`, `simulator/view_controls.py:118`) | Immutable multi-camera replay, seek/frame-step/event bookmarks, evidence alignment and comparison of actual controller attempts |
| Agent tools | Seven braking-model tools, including editing only `actuator.py` (`investigation/broker.py:39`) | General simulator/controller tools, visual retrieval, platform schemas; existing tools do not supply these four repairs |
| Images | Recorder saves frames, while investigation tool results are serialized to JSON; dashboard media explicitly excludes investigator context (`investigation/runner.py:204`, `dashboard/media.py:1`) | A documented image-content delivery interface; a file path or text description alone is insufficient visual input |

No upstream `mujoco/` changes or new dependencies are planned. Reuse platform physics, validation, worker isolation patterns, recorder, and viewers. Avoid a second simulator or a new autonomous agent framework.

## Shared experimental design

### Fixed task, editable controller

Each scenario publishes a task card, nominal engine description, allowed observations, control schema, bounded diagnostic probes, starter controller, mission criteria, and recorded failed run. The host owns physical parameters, contacts, obstacles, mission schedule, scoring, fixture versions, and reserved cases. Controller changes cannot modify those.

Three run labels have distinct meanings:

- **Reference:** a labeled nominal physical/control example, useful for orientation. It is never presented as an Astra repair.
- **Original attempt:** the flawed starter controller in the target task world.
- **Candidate attempt:** a versioned controller supplied later through the tools, in the same target task world. It can fail and its real outcome remains visible.

Every before/after comparison must match fixture, initial state/conditioning, mission, world version, and observation contract. Only controller source/configuration changes. Comparing a fault-free world to a damaged world is still an existing nominal-mismatch experiment, not repaired behavior.

Controller execution is a simulator capability. Building a controller tool adapter and a deliberately imperfect starter is in the eventual tooling scope; authoring the corrected controller, diagnosis script, automatic optimizer, or model-driven orchestration is outside it.

### Public knowledge and observations

Expose the nominal engine in enough detail to reason: coordinate frames, mass/inertia conventions, actuator mapping/signs, joint order/limits, control cadence, latency, motor limits, contact model, integration step, reset behavior, and limitations. Expose nominal model assets and controller source through bounded tool reads. Where useful, add nominal kinematics/actuation query operations under engine inspection.

Document every sensor as measured, derived, or command metadata. Keep exact injected fault parameters, private state snapshots, and evaluator schedules outside the public tool output. A visible off-center package is legitimate visual evidence; an overlay announcing its hidden COM is not. Existing operator `--set` commands remain builder tools and must not become the agent's way to erase a fault.

### Common timeline and replay

Each failed demo shows setup/goal, normal progress, observable loss of control, physical outcome, and a paused final result. Requested times are storyboard targets; actual event times come from the simulation. Capture 30 fps at a proposed 960×540 minimum per view, public observations at the existing 100 Hz, and commands at their actual application times. Physics starts from the existing 2 ms step and is checked at 1 ms.

Use a wide view plus a diagnostic view per scenario. Public RGB is the source evidence for Astra. Operator annotations can explain the story, but do not replace unannotated frames. Provide actual image content to a future model adapter, with run ID, time, camera, and nearby telemetry. See the storyboard document for playback, checkpoints, event effects, and storage requirements.

Replaying saved frames is read-only. Rerunning creates a new run. A reset is an explicit experiment intervention; it cannot silently cure damage or switch to a healthy world. Fresh target runs reinstantiate the same task-specific fault/load and controller state policy.

## Scenario 1: faster dog, poor limb coordination

**Primary cause:** a starter gait/controller that mishandles a modest speed increase on an otherwise healthy physical dog. Do not reuse injected knee weakness as if it were a gait timing bug. Existing joint weakness/foot slip remain separate diagnostic/regression cases.

Proposed task: settle, walk at 0.12 m/s, then request 0.15 m/s along a marked route. Keep the request within supported speed limits. Show at least two complete slow gait cycles before the transition; allow a roughly 24–30 s demonstration if needed. Calibrate a flawed speed-to-gait transition, such as an inconsistent swing/stance phase update, that visibly causes mistimed support or toe scuffing. The body must move and stumble through torques and contacts.

Evidence: front-quarter footage with all four feet distinguishable; side close-up of swing clearance and torso pitch; requested versus measured speed; joint angles/velocities; commanded targets/torques; foot contacts; IMU/torso height. Add world foot positions and planned swing/stance metadata as documented derived/public controller information where needed.

Tools must permit speed-ramp and individual-leg probes, nominal joint/kinematics inspection, editing gait parameters or source, and rerunning with unchanged mechanics. Planned editable parameters: phase offsets, stance duty fraction, swing clearance, stride scaling, joint PD gains, and balance gains, all bounded and separately documented. Do not represent direct joint targets as the existing crawl controller: today they select a different PD branch (`simulator/platforms/quadruped.py:284`).

Proposed gates:

- Original attempt completes two slow cycles upright, then has an observable stumble after the speed transition: tilt above 25° for at least 0.1 s plus loss of expected support, or torso contact. Record the actual trigger and do not force a fall at a timestamp.
- Candidate evaluation requires the full speed schedule, no torso contact/fall, maximum tilt below 25°, and mean speed within 10% of 0.15 m/s (0.135–0.165 m/s) after a 2 s transition allowance. It must complete the manifest's route distance within its horizon; derive that minimum distance from the fixed speed schedule and allowances before freezing the task.
- Standing still, continuing at the original 0.12 m/s, halving the task speed, disabling motors, or shortening the run cannot pass. Test each explicitly. The current `safe` flag alone lacks a progress criterion (`simulator/platforms/quadruped.py:336`).
- Development probes: stand, slow walk, speed ramp, one-leg swing/target checks. Reserve two intermediate speed ramps and one small heading change after fixtures are frozen.

## Scenario 2: unbalanced drone delivery

**Primary cause:** a lateral package offset combined with a starter controller that assumes symmetric loading. This is distinct from rotor failure and from insufficient total lift.

Proposed mission: visibly loaded drone at pad A → take off → travel about 4 m to B → place/release the package inside B → return unloaded → settle/land at A. Mark both pads and the route. The package starts secured to one side; preserve its mass and geometry throughout the loaded phase. A physical latch release at B creates a separate package body with continuous pose/velocity. Record delivery only when that package is actually deposited and remains at B.

Use a moderate load as a calibration starting point (for example 0.35 kg and 0.08–0.16 m lateral mounting offset). These values are not promised to create a crash. Before accepting the fixture, verify loaded and unloaded thrust/torque feasibility per rotor with at least 20% upper thrust headroom at equilibrium. Total lift alone is insufficient; the rotor allocation must also balance moments. The current 1.4 kg payload default added to the nominal 1.2 kg body exceeds four 6 N rotors' total hover authority (`simulator/platforms/DRONE.md:13`, `simulator/platforms/DRONE.md:28`), so it cannot serve as this controller-repair task.

Calibrate a documented imperfect symmetric-load starter and mission that physically produces unstable attitude, ground impact, and aftermath. If the selected offset is naturally stabilized, revise the starter/fixture openly before freezing it; do not prescribe an impact trajectory or exceed recoverable hardware limits merely to obtain a crash.

Evidence: wide A/B route, oblique underside view that shows the offset package, roll/pitch and altitude history, requested rotor commands, saturation, world velocity, gyro, and mission milestones. Nominal engine docs explain rotor locations/order/signs and how unequal vertical thrust creates correcting moments. Actual load/COM can be inferred through visual and pulse evidence without exposing the private answer.

Astra's later correction surface is the controller's mass/COM estimate, feedforward, rotor allocation/trims, attitude/position gains, and load-transition state. Use differential thrust for counterbalancing in the first version. A movable ballast would require an additional physical mechanism and bounded actuator; it is an optional extension, not a hidden mass slider. Package release is a documented command permitted only within the delivery envelope.

Proposed gates:

- Original attempt visibly transports the package away from A, then loses control and physically impacts. Camera captures imbalance, descent, contact, and at least 2 s of aftermath.
- Explosion/sparks/smoke start only after a qualifying collision event. This is a labeled presentation effect, with no physics forces or effect on task scores. Clean impact footage is also available.
- Candidate evaluation requires ordered A→B→A milestones, package deposited within 0.4 m of B and stationary for 1 s, return within 0.4 m of A, terminal speed below 0.15 m/s for 1 s, and completion within the fixed mission horizon (initial target 35–45 s).
- Exclude declared low-speed pad takeoff/landing contacts from crash classification; include high-speed or uncontrolled chassis contacts. Define and freeze impact thresholds in the fixture before evaluation.
- Require no crash, no package release outside B, transit tilt below 30°, and fixed route tracking bounds. Hovering indefinitely, changing the goal, dumping the package at A, or increasing motor limits cannot pass.
- Development probes: loaded hover, collective pulse, opposed roll/pitch pulses, loaded translation, permitted release/unloaded response. Reserve mirrored offsets, another feasible load, and a different outbound/return route. Engine authority must hold for every reserved case.

## Scenario 3: car steering drift

**Primary cause:** biased/miscalibrated steering response that a controller must compensate. Build on the existing steered car. Keep the current impact→reposition inspection demo separate; the primary new animation begins on a clearly marked driving course.

Declare the damaged initial steering fixture or show a labeled preparation replay. Do not silently transplant a healthy car for the candidate attempt. Proposed task: drive a 30 m straight/gentle-curve route at 4 m/s, with a 4 m lane and visible centerline. The unchanged starter's neutral or nominal steering command creates sustained lateral drift and lane departure; a barrier at the outer edge is optional.

Evidence: overhead trajectory and lane boundaries; front wheel/rack close-up; commanded and actual steering; yaw rate; lateral/heading error to the fixed route; wheel speeds and velocity. Separate target centerline from actual traveled path.

Tools must permit small positive/negative steering pulses, zero-steering/coast probes, fixed-speed route trials, and edits to steering sign/scale/offset estimates and lateral/heading feedback. Keep the task route and physical rack bias outside the editable artifact. Exact steering mapping is inferred; nominal mapping and units are documented.

Proposed gates:

- Original attempt travels at least 5 m before its body footprint crosses a lane boundary; video and geometry-derived event agree. No preassigned sideways chassis translation.
- Candidate evaluation completes the full route with mean moving speed at least 3.2 m/s, centerline RMS error at most 0.25 m and peak at most 0.5 m, no footprint departure/contact, and a controlled stop at the marked finish. Freeze acceleration/finish allowances in the task card.
- A parked car does not pass. Rewriting the centerline to follow the car does not pass. Current post-crash `safe` metrics are insufficient for this mission (`simulator/platforms/CAR_DAMAGE.md:54`).
- Development probes: straight, left/right pulses, slow slalom, coast. Reserve a modest speed change, reverse-signed steering bias, and a gentle curve within remaining steering authority.

## Scenario 4: automatic braking before an obstacle

**Primary cause:** an inadequate brake-trigger/modulation policy, initially on a dry, physically stoppable track. Keep stronger friction/thermal variants as later tests instead of confounding the first animation.

Reuse the original longitudinal car mechanics, wheel torques, wall/contact logic, and histories. Add a controller callback for throttle/brake at a declared rate; the existing editable wheel-capacity model is a different interface. Start at a manifest-defined approach speed and reveal a stationary obstacle through a declared camera/range sensor with enough distance for a feasible stop. The flawed starter triggers too late or modulates poorly and contacts the obstacle.

Proposed first calibration: 12 m/s approach, obstacle initially about 25 m ahead of the front bumper. Validate actual stopping capability, sensor availability/latency, actuator lag, and margin before freezing either number. Sensors must make successful intervention possible; no trial may require braking before the obstacle becomes observable.

Evidence: side-wide view containing car, obstacle, and physical gap; chase view; speed, front-bumper gap, range validity, closing speed, commanded brake/throttle, wheel angular speeds, and actual collision/stop event. State whether obstacle range is an ideal simulated sensor; do not attribute it to vision inference.

Tools must permit brake step/pulse/coast experiments, matched wall-free diagnostic stopping tests, and edits to a stateful automatic-brake controller. The documentation should explain reaction/actuation delay and braking-distance estimation principles without supplying tuned scenario answers. An editable brake-capacity model can be an optional estimator inside the controller later, but predicting a stop does not actuate the brakes.

Proposed gates:

- Original attempt physically contacts the obstacle with positive pre-impact speed; collision records are censored and never labeled free stopping distance.
- Candidate evaluation reaches the approach zone at the required speed, brakes after detection using allowed signals, then stops with 1–5 m front-bumper clearance, no contact, and speed below 0.1 m/s for 0.5 s before timeout.
- Stopping immediately at launch, changing obstacle position, increasing friction/brake hardware limits, or reporting only a prediction cannot pass. The approach-zone/speed gate prevents these shortcuts.
- Development probes: coast, partial/full braking, brake pulse, wall-free stop. Reserve different feasible speeds/gaps and one declared reduced-grip or warm-brake case after the basic policy interface is proven. Score censored/timeout results explicitly.

## Planned implementation sequence

All new paths in this table are **future deliverables**. Existing source anchors above identify reuse points. Implementation begins only in a later implementation task.

| Order | Work and ownership boundary | Files/surfaces | Exit evidence |
|---|---|---|---|
| 1 | Freeze task cards, editable boundaries, proposed numbers after physical calibration, and scenario matrix | New `simulator/tasks/`; existing platform assets/modules and `simulator/config.py` | Four manifests distinguish mission, physics, starter controller, observations, visual checkpoints and validation cases |
| 2 | Add controller execution contract and one common tool dispatcher; reuse existing worker mechanisms where applicable | New `contracts/CONTROLLER.md`, `simulator/tooling.py`, `simulator/public/tools.schema.json`; bounded adapters in platform modules/runner; existing worker runtime | All four platforms run validated public controllers and schedules; rejects preserve state; private world inaccessible |
| 3 | Build four original failed scenarios and generic diagnostic probes | Existing `quadruped.py`, `drone.py`, `car_damage.py`, `runner.py`; platform XML; new starter-controller/task assets | Every failure arises physically, is reproducible, has remaining control authority, and has the intended observable signature |
| 4 | Add multi-camera recording, visual tool outputs and immutable replay/comparison | Extend `recording.py`, `view_controls.py`, platform operator; proposed `simulator/replay.py` and `simulator/visual_evidence.py` | Seek, frame-step, replay, bookmarks, synchronized plots, drone impact effect, image-content response contract pass |
| 5 | Complete public docs, JSON examples, diagnostic walkthroughs and builder guide | Planned `simulator/tooling/` documentation tree described in companion document | A fresh caller uses only the public bundle to discover tools, get images, run a probe, validate an edit and compare actual runs |
| 6 | Validate whole simulator/tooling package and hand off | Extend focused tests; new contract/mission/replay tests; build/runtime checks and validation report | All four coverage rows pass; legacy scenarios remain working; measured artifacts accompany docs; no claim of Astra-authored success without such a run |

Steps 3's platform implementations can run independently once contracts are fixed. Step 4 can prepare the common replay layer alongside platform work, then perform visual QA on each finished scenario. A future implementation lane should own one platform at a time; shared schema/recorder integration has one owner to avoid competing interfaces.

## Acceptance matrix and verification

| Requirement | Dog | Delivery drone | Steering | Auto brake |
|---|---|---|---|---|
| Failed physical animation and visible objective | Faster walk → mistimed legs → stumble | Loaded A→B flight → tilt → crash/effect | Lane route → drift → departure | Approach → late braking → impact |
| Primary diagnostic view | Feet, joints, support | Package, rotors, body attitude | Wheels and lane path | Gap, speed and wheel response |
| Editable capability | Gait/joint/balance controller | Load estimate and thrust/flight controller | Steering/route controller | Stateful trigger/modulation controller |
| Replay deliverables | Wide + limb view + gait/contact trace | Route + package view + rotor/attitude trace | Overhead + wheel view + route errors | Side + chase + brake/range trace |
| Anti-shortcut gate | Required speed and distance | Deposit at B and return A | Required route/speed | Approach at speed then stop near obstacle |
| Evidence of later repair | Same-world candidate completes faster walk | Same-world candidate delivers and returns | Same-world candidate stays in lane | Same-world candidate actually stops |

Verification for later implementation:

1. Validate all documented schemas/examples, units, ranges, cadence, resets, partial/error results and artifact permissions; include invalid command, stale edit, unknown run, wrong-platform and cross-task access cases.
2. Run three identical original attempts per fixture with identical pre-impact outcomes and event timing within one physics step. Pre-impact pose tolerance starts at 1 mm and attitude at 0.1° in the same pinned environment; calibrate and freeze any justified change. Do not demand identical post-impact trajectories across different timesteps.
3. Halve timestep: retain failure classifications, mission feasibility and pre-contact metric differences within 2%. No NaN/Inf, solver warnings, or automatic engine resets accepted as physical failures.
4. Prove rendering, playback speed, seeking and effects do not change recorded physics; frame/telemetry alignment within one captured frame. Reject a mismatched rerender rather than passing off a new run as a replay.
5. Use rejection/control candidates to test success rules: standing dog, hovering drone, parked car, immediate-stop car, and mission/physics mutation attempts all fail appropriately. These are contract tests, not authored successful solutions.
6. Visually inspect setup, failure onset, physical event and aftermath for each scenario using `$visual-verdict` during implementation iterations; persist verdict JSON under `.omx/state/simulator-failure-demos/ralph-progress.json`. Require passing visibility checks for both human presentation and clean agent frames.
7. Run relevant tests and the full Python suite at integration (`.venv/bin/python -m pytest tests`), lint (`ruff check simulator contracts component_worker tests`), type/static checks (`mypy --python-executable .venv/bin/python simulator`). Run frontend build only if frontend code changes. Save exact commands, tool versions, outcomes and verification gaps; do not inherit old “passed” counts as new evidence.
8. Handoff a simulator/tooling readiness report. At this stage the new corrected replay slot can legitimately say “No candidate run yet.” A later successful Astra-authored run needs tool transcript, controller hash/diff, unchanged-world manifest and measured completion; a developer reference never substitutes for it.

## Risks and mitigations

| Risk | Mitigation |
|---|---|
| Polished footage still tells the wrong failure story | Gate each primary fixture on the exact cause/observable sequence above; retain old demos under their old names |
| Failure cannot be corrected with permitted controls | Check steering, gait and per-rotor authority plus braking feasibility before freezing; reduce severity or redesign flawed starter openly |
| Controller wrapper accidentally solves the task | Keep adaptation in the candidate; baseline helper/control behavior is disclosed; no hidden load-aware tuning |
| Scenario passes by doing nothing or changing goals | Host-owned speed/progress/milestone/approach criteria and explicit negative tests |
| Agent receives only text while demo claims visual reasoning | Image retrieval returns decodable content with timestamps; future adapter must attach that content, verified with a transport test |
| Replay is a fresh simulation or manufactured outcome | Immutable captured media/state provenance; label reruns and verify rerenders; no generated-image substitutes for physical evidence |
| Documentation reveals the answer or is too vague to invoke | Publish nominal mechanics, APIs and diagnostic methods; keep exact scenario truth/reference corrections private; test examples against schemas |
| Multiplatform expansion disturbs existing work | Add task-specific adapters and preserve existing preset behavior; extend existing tests; no upstream engine rewrite or dependencies |

## Completion of this planning pass

The planning output is complete when these four scenario specifications, companion storyboards/tool contract/documentation map, file ownership, acceptance criteria and risks are written and cross-checked against the checkout. This is not an implementation approval gate or a request to begin coding: the user explicitly requested planning first.

Planning verification completed: four active documents; eight local links; 38 source references checked against baseline `803fa7f`; 11 parseable JSON request/response examples; 18 unique proposed tools; original car-plan body preserved in the archive; no whitespace errors. An independent read-only review identified and resolved the faster-walk tolerance, required dog actuation surface, and missing public-document retrieval operation. Runtime tests and visual validation were not run for this documentation-only change. Scenario numbers still require physical calibration during later implementation.
