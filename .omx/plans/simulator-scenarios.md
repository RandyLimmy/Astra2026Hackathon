# RealityPatch simulator scenario plan

Status: planning only. No scenario models, runtime code, agent code, or UI implemented by this plan.

## Scope and ownership

Build a reusable MuJoCo car test environment and a reproducible collection of synthetic failures. Our eventual responsibility is the simulator: MJCF assets, hidden physical behavior, experiment execution, resets, sensor recordings, replay, and scenario validation. Another workstream owns Astra/Sol integration, candidate-model editing, prediction locking, scoring dashboards, and the self-repair loop.

The user requested planning without coding. Only this document is created now. Later, dynamic heating and timed failures require a small scenario runner in addition to MJCF; they cannot all be delivered as static XML scenes. No new dependencies are planned.

Existing setup: MuJoCo 3.13.0 and its Python bindings are installed; the source checkout is untouched. The verified setup is documented in `README.md:3` and `README.md:38`. No car model exists yet.

## Shared scene

Use one flat, straight track, one simple rigid car, four rotating physical wheels, a wall, ground markings, and fixed side/overhead cameras. Allow chassis translation, yaw, roll, and pitch so later failures can affect its motion. Start with fixed steering and no suspension, ABS, aerodynamics, or detailed tire model. This is a controlled synthetic experiment, not a validated vehicle safety simulator.

Proposed starting dimensions and calibration targets, not measured results:

- Total healthy vehicle mass: 1,200 kg, including wheels and carriers.
- Track: 160 m long, 8 m wide. Wall face at x = 100 m for the demonstration.
- Approach speed: 25 m/s. Brake trigger: front bumper reaches x = 45 m.
- Cold stopping distance after trigger: approximately 46–48 m, giving 7–9 m clearance.
- Physics timestep: start at 0.002 s; compare with 0.001 s before freezing scenarios.
- Telemetry: 100 Hz; fixed-camera frames: 30 Hz, timestamped in simulation time.

Measure wall clearance from the car's leading collision geometry, not the chassis center. For straight-line baseline calculations, 55 m of available braking distance and a 46 m stop imply about 6.8 m/s² mean deceleration. A claimed 54.6 m stop has only 0.4 m clearance, not a generous safety margin. All distances and failure outcomes in the pasted brief are illustrative until calibrated.

### Common mechanics

Apply propulsion/braking at wheel joints so tire-ground contacts transmit force. Bound brake torque and oppose wheel motion; prevent the brake controller from becoming reverse propulsion near zero speed. Avoid an unlimited chassis force: it would bypass traction and undermine the wet-road scenario. MuJoCo supports joint/body force application (`mujoco/include/mujoco/mjdata.h:186`, `mujoco/include/mujoco/mujoco.h:625`); choosing wheel torque here is a design decision.

For detachable wheels, propose a world-level free carrier welded to the chassis, with a rotating wheel hinged under the carrier. Disabling that carrier's weld releases the assembly while preserving the tire's spin. Do not weld the rotating wheel directly: welds suppress all relative degrees of freedom (`mujoco/doc/computation/index.rst:859`). Runtime equality activation is exposed through `mjData.eq_active` (`mujoco/include/mujoco/mjdata.h:189`). Validate this assembly before building the full track.

Use physical contact and the existing momentum when releasing a wheel; do not add an unexplained launch impulse. Disable its drive/brake commands upon release. Verify collision filtering allows the released assembly to contact the ground and appropriate car geometry without artificial overlap impulses. If the simplified assembly cannot produce a reliable visible effect, revise the scenario instead of faking a crash.

## Scenario catalog

Each row is a family of cases, not one scripted trajectory. Build the first three as the initial deliverable; retain the same platform for the remaining four.

| ID | Scenario | Hidden change | Observable evidence | Controlled probes and held-out variants |
|---|---|---|---|---|
| S0 | Healthy car | None | Repeatable straight braking and safe stop | 15/20/25 m/s, partial/full braking, coast-only; reserve intermediate speeds |
| S1 | Brake fade and recovery | Temperature-dependent brake torque capacity | Repeated braking worsens stopping; rest restores it | Vary braking energy and rest independently; reserve mixed warm-up/recovery histories |
| S2 | Front-right wheel detaches | Release one wheel carrier | Visible separation, changed support and braking, possible yaw | Coast versus brake after release; reserve speed, release time, and combined moderate heat |
| S3 | Wet road patch | Lower tire-road contact friction over a spatial interval | Braking weakens upon entering that location | Brake before/inside/after the patch; reserve patch position/length and approach speed |
| S4 | Additional payload | Add a secured, centered 300 kg load | Lower acceleration and, in a torque-limited regime, lower deceleration | Compare coast, drive, partial brake, full brake; reserve 150/450 kg loads |
| S5 | One brake weakens | Front-left brake retains 20% capacity | Braking-dependent asymmetry with the wheel still attached | Coast and multiple brake strengths; reserve onset time and severity |
| S6 | Brake actuator lag | Brake torque command follows first-order dynamics | Gradual onset with approximately unchanged settled response | Short pulses versus long steps; reserve pulse durations and time constants |

### S0: healthy reference

Calibrate cold brake torque on a dry, high-friction track so braking is torque-limited rather than contact-saturated. Use identical nominal mechanics for hidden reality and the candidate's baseline asset. Record initial wheel spin consistent with chassis speed to avoid a startup slip artifact. Settling must happen before the measured run.

Gate: three identical runs stop within 0.1 m of each other; the 25 m/s demonstration stops in the target interval without wall contact; lateral drift stays below 0.25 m. Lower brake input must not shorten the stop in the selected torque-limited operating range.

### S1: heating, fade, recovery

Model brake-disc heat from actual dissipated brake power: heat capacity times temperature rate equals a fraction of brake torque times wheel angular speed, minus cooling to ambient. Use documented units, positive heat capacity, positive cooling time, and a smooth bounded fade curve. Calibrate a synthetic temperature range; do not claim the constants describe a real brake system.

MuJoCo's built-in DC motor thermal feature models winding temperature and electrical resistance (`mujoco/doc/XMLreference.rst:6977`). It is not automatically a friction-brake fade model. A small hidden thermal update is therefore part of the eventual simulator runner.

Warm-up means repeated accelerate/decelerate cycles on a wall-free version of the track. Do not count stationary brake-command time as heating: once the wheel stops, dissipated brake power is near zero. Expose the complete commands and motion during conditioning, followed by rest. Reposition the car for the wall trial while retaining thermal state.

Gate: cold baseline passes; a fixed hot history increases wall-free stopping distance by at least 20% and the selected wall case collides; longer rest monotonically reduces excess distance for the chosen calibration. A long-rest case returns within 5% of baseline. Coast-only or stationary pedal-hold histories must not produce comparable heating. These are acceptance targets, not results already observed.

### S2: physical wheel loss

Trigger release at a private, deterministic simulation time or track coordinate. Use side/front-quarter footage that actually shows the separation. Log wheel presence and release events privately for validation; neither becomes a privileged sensor returned to the agent. A vision-based inference from frames is allowed.

Gate: while attached, carrier position error stays below 1 cm and the wheel rotates; after release, the assembly separates by at least 0.5 m within 2 s in the chosen demo case and its attachment remains inactive. The resulting motion must differ reproducibly from the healthy case without numerical warnings. Select a physically produced collision or lane departure for the demo if available; detachment alone does not guarantee either. Report the observed result honestly.

The candidate cannot predict an arbitrary unobserved future wheel failure. Test prediction of motion after observing failure, or provide a declared external release intervention. Reserve the release schedule itself for diagnosis runs; do not score its exact prediction as missing-physics discovery.

### S3–S6: extension requirements

- Wet road: use adjoining ground segments without an overlapping dry ground collider. Give ground friction suitable priority or specify wheel-road contact pairs. Equal-priority geoms use the maximum friction coefficient, so merely lowering the road value may do nothing (`mujoco/doc/modeling.rst:425–449`). Verify effective contact friction. In a selected braking case the wet patch must increase wall-free distance by at least 20%, while a path avoiding it matches dry baseline within 5%.
- Payload: load before reset/compilation, with consistent total mass, center of mass, and inertia. No unexplained mid-run mass teleportation. In friction-limited braking, extra mass need not increase stopping distance because available friction scales with normal force. Demonstrate the load effect in the calibrated torque-limited regime, targeting at least 15% longer stopping distance. Keep cargo secured and centered to isolate mass from shifting-load dynamics.
- Weak brake: verify affected-wheel torque is 20% of its healthy value for the same activation; all wheels remain attached. Select cases where brake-dependent yaw exceeds healthy yaw by at least 3°, while coast trajectories differ by less than 1° over the same interval. Use this as evidence supporting an asymmetric braking hypothesis, not proof that excludes every other cause.
- Actuator lag: use a first-order filter, initially around 0.25 s, with the same torque saturation as baseline. `filterexact` integrates that activation filter analytically (`mujoco/doc/XMLreference.rst:5757`, `mujoco/doc/computation/index.rst:363`). It is not pure transport delay. Gate: activation reaches approximately 63% at one time constant and 95% by three, within 5 percentage points; steady torque agrees within 2%. A true dead-time scenario would need a separate command-history mechanism and is deferred.

## Experiment and reset contract

Define simulator-facing records independently of the agent's tool implementation:

1. A request specifies initial observable pose/speed, throttle/brake schedules, warm-up/recovery history, run duration, camera choice, and reset mode. Commands are bounded and timestamps use simulation time.
2. A full reset restores ambient temperature, healthy wheel/brake state, default mass/surface, inactive failure schedules, and clean controller state. Each independent experiment begins with a full reset and replays its declared conditioning history.
3. A trial reset repositions/reinitializes chassis and attached-wheel motion while explicitly retaining declared persistent state. Hidden temperature and retained faults must not disappear through a generic `mj_resetData`. Completed detachments are repaired only by full reset or an explicit intervention.
4. Warm-up without the wall and the wall trial share persistent state. Teleport/reposition is a declared experimental reset, never concealed as continuous driving.
5. A result returns time, observable pose/velocity, yaw/yaw rate, wheel speeds if enabled in the fixed sensor set, commands, camera frames, wall contact, impact speed, lane departure, and observed stop status. Direct force/temperature readings are not enabled by default.
6. Evaluator-only records retain parameters, event schedule, true temperature, contact forces, seed, model version, solver settings, and reset provenance. No hidden parameter names, revealing scenario IDs, source paths, or diagnostic overlays enter agent-facing outputs.

The public sensor set is fixed before evaluation and shared by Astra and Sol. Heat cannot be inferred from an unreported prehistory fairly: both get the same available history, even when temperature itself is hidden.

For actual isolation, the hidden simulator runs under a separate process/account or container boundary with restricted filesystem access. A `reality/` directory beside candidate code is not isolation if the agent has unrestricted shell access. The simulator workstream supplies an explicit public export; integration work owns enforcing the deployment boundary.

## Validation and handoff

Keep two track variants: wall-present for visible collision outcomes and wall-free for stopping-distance measurements. After a wall collision, the measured trajectory is censored; do not call the collision position a free stopping distance. A paired evaluator-only replay from the same pre-run state can measure the counterfactual stop. Report timeouts as censored, too. Define a stop as speed below 0.1 m/s for 0.5 s.

Provide a development matrix and a private held-out matrix. Initial matrix: 3 healthy cases, 3 heat/recovery cases, and 3 wheel-loss cases; holdouts: 2 intermediate-speed baseline cases, 2 unseen heat histories, and 2 post-observation wheel-loss predictions including moderate heat. Later add at least 2 development and 2 held-out cases per extension. Freeze parameter ranges, seeds, and splits before model repair; do not tune cases after seeing agent predictions.

Every scenario must pass:

- Three same-seed runs within 0.1 m stopping distance and 0.5° final pre-impact yaw; event timing within one physics step.
- No NaN/Inf state, automatic numerical resets, or MuJoCo solver warnings in accepted runs.
- Halving timestep changes wall-free distance by less than 1% and preserves selected collision labels. Move marginal cases away from the wall threshold if numerical sensitivity changes the label.
- Physics advances without rendering; changing video capture rate does not change trajectories beyond the repeatability tolerance.
- Full reset reproduces cold baseline after each failure; trial reset preserves exactly the declared persistent states.
- Neutral settings for each fault reproduce the baseline within 1% stopping distance.
- Video timestamps match logs within one video frame; wheel-loss footage visibly includes the detached wheel.

The simulator supplies raw outcomes and reference replays. The other workstream locks candidate predictions, runs repaired-model regressions, and computes prediction error. A simulator validation pass is not evidence that Astra has repaired anything.

## Planned delivery sequence and file ownership

All paths below are future deliverables, not files already created:

1. **Mechanical prototype:** `simulator/assets/car.xml`, `track.xml`, `track_no_wall.xml`. Prove rolling, braking, wall contact, and one releasable wheel on a small test before expanding assets.
2. **Experiment foundation:** `simulator/runner.py`, `recording.py`, and `contracts.md`. Establish deterministic stepping, reset semantics, observation filtering, camera replay, and evaluator-only logs. Runtime code remains deferred until implementation is requested.
3. **Initial scenarios:** `simulator/private/thermal.py`, `events.py`, and `simulator/private/scenarios/` data manifests for S0–S2. Calibrate and freeze discovery/holdout cases, then meet the gates above.
4. **Simulator handoff:** `simulator/public/` baseline assets and observation schema; private validation report and replay artifacts. Export a clean baseline with matching nominal mechanics but without failure rules. Do not put hidden implementation in the public bundle.
5. **Extensions:** add S3–S6 one at a time through the same runner and validation contract. Re-run all prior simulator cases and neutral-fault controls after each addition.

No changes are planned to the upstream `mujoco/` source. No agent tools, candidate patch logic, dashboard, or benchmark orchestration are included in simulator ownership.

## Main risks and mitigations

| Risk | Mitigation |
|---|---|
| Car/wheel constraints consume the build effort | Prototype one releasable rolling wheel first; use simple primitive geometry and low center of mass |
| Road friction has no effect | Use wheel torques, inspect effective contact friction, exclude overlapping dry colliders |
| Every perturbation is explained as heat | Isolate faults first; include coast-only, stationary-brake, recovery, and spatial-control experiments |
| Scripted numbers masquerade as results | Treat all example values as targets; save measured manifests and raw traces before the demo |
| Hidden state or held-out answers leak | Separate public exports from private evaluator storage and require deployment access restrictions |
| Collision hides stopping error | Use matched wall-free replay and explicitly label it counterfactual |
| Unknown future events make prediction impossible | Evaluate post-observation behavior or declared interventions, not clairvoyance |

Official references: [contact parameters](https://mujoco.readthedocs.io/en/stable/modeling.html#contact-parameters), [actuation and equality constraints](https://mujoco.readthedocs.io/en/stable/computation/index.html), [simulation lifecycle](https://mujoco.readthedocs.io/en/latest/programming/simulation.html). Local source references above are pinned to the installed 3.13.0 checkout.
