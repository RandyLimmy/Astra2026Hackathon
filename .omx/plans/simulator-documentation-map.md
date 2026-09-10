# Documentation and handoff plan for simulator tooling

Status: **planned documentation package**, with detailed design in [the tool contract](simulator-tooling-contract.md). This map specifies what must ship with the future tooling layer. The public package does not exist yet; current commands and limitations are listed in the contract's final section.

Existing-source references describe initial inspection of baseline commit `803fa7f`; implementation must reconcile any concurrent workspace changes.

## What Astra needs to know

The first document should enable a caller to answer: What is the mission? Which tools are actually installed? What can I observe? How do I obtain real images? What engine/control assumptions are nominal? What may I change? How do I run the same task again and prove my controller helped?

Use layered documentation: a short starting guide, complete generated tool reference, platform mechanics/control guides, scenario task cards, and targeted diagnostic walkthroughs. “Tons of documentation” should mean complete coverage and useful examples, not repeated descriptions with drifting parameter names.

The builder plan explains intended causes so implementers can create the fixtures. The public agent bundle teaches mechanics and diagnostic methods without stating each task's hidden answer. Keep both audiences explicit. Sharing public nominal engine knowledge is intended; sharing private fault tables/evaluation schedules is not.

## Proposed documentation tree

All paths in this tree are future files. Reuse/extract facts from existing platform docs; validate examples against the same schemas used by runtime tools.

```text
simulator/tooling/
  README.md                         Start here: task discovery and one public probe
  tool-reference.md                 Every tool's complete invocation/result/errors
  engine-guide.md                   Nominal physics, frames, integration and limitations
  controller-guide.md               Editable files, state, cadence, helper APIs, validation
  observations-and-units.md          Sensor provenance, names, units, timestamps and nulls
  experiments-and-resets.md          Task start, retained state, schedules and probe design
  visual-evidence-and-replay.md      Image delivery, frame windows, playback and comparisons
  errors-and-limits.md               Error recovery and published resource limits
  platforms/
    dog.md                          Joints, gait, contact/support and limb probes
    drone.md                        Rotor geometry, thrust moments, load and release
    steering-car.md                 Steering mapping, contact, lane/heading measurements
    braking-car.md                  Range/detection, braking actuation and stop metrics
  tasks/
    dog-walk.md                     Public objective, allowed edits, probes, acceptance
    drone-delivery.md                Fixed A/B mission, deposit/return rules, evidence
    lane-driving.md                 Fixed route/speed, lane boundaries, finish rules
    obstacle-stop.md                 Approach/detection/stop contract and censoring
  examples/
    discover-and-observe.jsonl       Tool calls that retrieve a real failed-run frame batch
    dog-probe.jsonl                 Speed/limb probe requests; no successful gait supplied
    drone-probe.jsonl               Hover/collective/attitude probes; no tuned mixer
    steering-probe.jsonl            Opposed steering probes; no correction constants
    braking-probe.jsonl             Brake/coast/wall-free probes; no solved trigger policy
    candidate-workflow.md            Validate, edit, execute, compare, evaluate, freeze
  builder/
    scenario-authoring.md            Fixture calibration, hidden causes, solvability checks
    recording-validation.md          Physical/visual acceptance and reproducibility
    release-checklist.md             Evidence and public-bundle assembly
simulator/public/tools.schema.json   Machine-readable public tool definitions
contracts/CONTROLLER.md              Canonical executable controller interface
```

If code and docs coexist, keep the dispatcher at proposed `simulator/tooling.py` and the documentation directory without a Python `__init__.py`; alternatively choose `simulator/docs/tooling/` at implementation time. Do not create a conflicting second Python package for markdown.

## Required content by document

| Document | Must explain | Acceptance evidence |
|---|---|---|
| Start guide | Installation status, tool discovery, task selection, reading original failure, fetching images, one probe, next controller-edit steps | A fresh caller reaches a decoded image and recorded probe without private files or undocumented arguments |
| Tool reference | All proposed 18 tools; exact schema/default/null behavior; source/side effects; output fields; success/error examples; resource limits | Every exported tool has a page; examples schema-validate and dispatcher replay succeeds |
| Engine guide | Nominal mass/inertia/contact/actuation model; x/y/z and body/world transforms; joint/rotor signs; step/control/sensor clocks; simplifications | Facts checked against exported nominal assets and platform code; no private fault fields |
| Controller guide | Interface version, editable allowlist, state lifecycle, helper availability, saturated controls, immutable world, atomic patches, invalid candidate handling | Valid no-op edit executes; deliberately invalid edit leaves original version intact; no solution policy supplied |
| Observation guide | Fields, units, frame/quaternion order, measured versus derived versus command values, absent sensors, null/censoring rules, timestamps | Full field coverage with small actual response examples from each platform |
| Experiment guide | Mission versus probe, matching fixture/start state, control schedules, feedback versus identical-input tests, reset retention and pending events | Reset tests reproduce declared state categories; comparison rejects mismatches |
| Visual/replay guide | Real image attachment transport, recorded frame provenance, camera availability, frame/telemetry alignment, replay versus rerun, effects and incomplete runs | Decodable image-content test and read-only replay test; no claimed vision from JSON paths |
| Errors/limits | Invalid input, wrong platform, stale source, unavailable view, timeout, controller failure, numerical failure, unknown run and bounds | One meaningful recovery example per error family |
| Platform guides | Actuator order/limits, nominal controller assumptions, expected response to basic probes, ambiguity of diagnosis, feasible control authority | Each guide includes a mapping table and at least three probe recipes with observations to inspect |
| Task cards | Objective, fixed initial/preparation conditions, allowed sensors/edits, exact progress/success/failure rules, development probes and known limitations | Standing/hovering/parking/early-stop shortcut candidates fail mission checks |
| Builder guide | Why each failure is constructed, physical calibration, selected parameters, developer-only diagnostics, no manufactured crash | Three repeated physical failures and authority checks per frozen fixture |
| Release report | Current versus planned tools, exact test commands/results, artifact locations, unresolved limitations | No stale historical test counts or unobserved Astra-success claims |

## Platform diagnostic teaching requirements

**Dog:** include a labeled joint/leg order map; stance versus swing; expected support contacts; gait period/duty fraction/stride/clearance units; relation of joint target, PD torque and physical contact; speed tracking and torso stability. Show how to compare frames and contact traces across a speed ramp. Explain that actuator weakness, friction loss and poor coordination can produce similar symptoms. The active task card should not give a preselected diagnosis or tuned phase offsets.

**Drone:** include rotor positions/order/spin signs, body/world force frames, normalized commands versus thrust, spool/delay effects, thrust/torque saturation, nominal load/moment relationships, and guarded release semantics. Explain how collective versus differential pulses help distinguish altitude deficits from roll/pitch imbalance, while a single trace rarely identifies a unique cause. Explain per-rotor authority and loaded-to-unloaded dynamics; do not supply the task's true COM or corrected mixer weights.

**Steering:** include commanded rack angle versus measured steering, yaw/heading signs, wheel speeds, lane footprint and path error; steering gain/bias estimation through opposed pulses; distinction between control calibration and physical tire/suspension damage. Provide the fixed route and finish rules, not the hidden rack offset or a solved feedback gain table.

**Braking:** include throttle/brake units, command-to-wheel torque path, wheel slip/contact limits, sensor detection/latency, gap reference at the bumper, closing speed, stopping-distance estimation, and collision/timeout censoring. Explain why a wall-free experiment measures what a collided experiment cannot. Do not provide a tuned emergency-braking trigger or confuse capacity prediction with control output.

Each walkthrough should follow: observed symptom → plausible hypotheses → discriminating probe → expected evidence under each hypothesis → how to validate a caller-authored change. Hypotheses remain possibilities, not injected labels disguised as observations.

## Public bundle and builder bundle

The public bundle contains nominal assets/helpers, public controllers, schemas, task cards, allowed observations/replays, documentation and examples. Package a generated index listing every exposed tool/document/artifact with version/hash. Discovery returns document IDs, and `sim_read_document` returns the actual indexed guide/schema content in bounded pages. Relative links resolve through this index; no references require the full repository or `.omx/plans/` to understand the task.

The builder bundle contains hidden scenario configuration, exact fault/load truth, calibration records, complete engine snapshots, private evaluator cases and reference checks. The current `public/` and `private/` split is only a convention (`simulator/contracts.md:95`); any later restricted agent deployment must enforce an actual access boundary. This is a tooling requirement, not a reason to ask permission during planning.

Do not inject these planning documents wholesale into Astra's task context: they name intended causes and proposed fixture values. Assemble the neutral public bundle from the documented allowlist. Human demo overlays with private settings also stay outside the agent's RGB stream.

## Documentation verification and release order

1. Establish canonical controller/tool/observation schemas with platform adapters. Mark all examples draft until those exist.
2. Generate the common reference from schemas; author explanatory platform/task guides around actual engine behavior. Keep nominal versus measured versus target values labeled.
3. Record real original attempts and generic probes. Replace example IDs/times with documented sample response bundles; retain their run/source provenance.
4. Run all JSON requests against a local dispatcher test harness, including documented error cases. Use a transport fixture to confirm that frame results contain decodable image content and metadata. This does not require an API call or an agent-authored repair.
5. Have a fresh documentation-only consumer discover the four tasks, read controls, inspect a failed frame sequence, run a probe, make/validate a harmless controller edit, and compare actual runs. It must not need hidden source paths or an implementer's verbal instructions.
6. Verify every local link and generated field/tool reference; verify public exports exclude private overlays, scenario truth and reserved cases. Record any unavailable optional encoder/viewer backend explicitly.
7. Handoff four failed demos, saved replay bundles, schemas, runnable diagnostic examples, starter controllers and the readiness report. Later agent integration supplies its own decision process and corrected controllers.

Completion of the tooling package is not contingent on inventing four successful repairs. The package must expose sufficient bounded control authority and evidence, preserve fixed mission rules, and let a caller implement and verify a real attempt. A later claimed success requires that caller's versioned source and an actual passing run.
