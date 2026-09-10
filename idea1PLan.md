# RealityPatch: brake-fade-first hackathon plan

Based on [idea1.md](idea1.md) and [idea1UseCases.md](idea1UseCases.md).

This is a **builder-facing plan**, incorporating the later bumper-impact walkthrough and requirement to keep the reference implementation hidden. Do not include this document, the original ideas, or the pasted walkthrough in the investigation agent's context. Brake fade remains the first committed build; the bumper example below is a separate optional scenario, not an additional first-demo requirement.

## 1. What we are building

Start with **use case 3: brake fade**. RealityPatch investigates why a simple braking simulator predicts the wrong stopping distance, requests experiments, adds missing history-dependent behavior to the simulator's code, and predicts an unseen braking trial before its result is revealed.

**Required implementation choice: use the second approach from `idea1.md`—a small editable Python actuator model coupled to MuJoCo's mechanics.** Astra must add missing state and its evolution to that Python component. For brake fade, this component models brake force and its history dependence; for the later robot use case, it models motor torque. **We are repairing the actuator model, not rewriting MuJoCo's engine.** MuJoCo is part of the first working demo, not a later optional upgrade.

The coupling is implemented behind a neutral interface. Astra sees the incomplete Python component and experiment observations; it is not told that MuJoCo is the backend and cannot inspect the engine, bridge, scene, or complete reference model. Its existing knowledge of MuJoCo and possible physical mechanisms is legitimate background knowledge; we cannot erase that knowledge or promise it cannot infer the backend. The test is whether observations support a useful repair, not whether it reproduces our exact hidden code.

This approach best matches our intended **source-code extension** contribution. Enabling an existing thermal option would demonstrate configuration repair; it does not fulfill the source-extension goal we have chosen. Requiring extra code does not itself make the research novel. Our claim must rest on the connection from evidence and experiments to a tested executable extension and improved unseen predictions.

**Demo pitch:** “The simulator says the car will stop before the barrier. After repeated braking, the synthetic reference crosses it. RealityPatch investigates the mismatch, repairs its model, and predicts the next stop.”

The repair improves the prediction; it does not physically improve the brakes. On the final trial, show the original prediction, repaired prediction, and actual reference trajectory for the same commands. A later feature could use the improved model to recommend earlier braking.

The current repository contains the two idea documents and no implementation. This plan assumes two builders and uses a **five-hour working timebox**, not a confirmed event deadline. If there is more time, expand validation before adding another use case. Person 1 and Person 2 below are interchangeable role assignments based on your strengths.

### Required setup: obtain an OpenAI API key

**Person 2 owns getting a working OpenAI API key before the live agent loop can run.** Create a key in the team's authorized OpenAI API project, or obtain the event-provided credential through the organizers' approved process. Follow the [official OpenAI quickstart](https://developers.openai.com/api/docs/quickstart) to configure `OPENAI_API_KEY` in the host backend's environment; the OpenAI SDK reads that variable.

- Confirm the project's API credits/billing, usage limits, and actual access to the intended Astra and Sol models. A key alone does not demonstrate model access; make a small test request with each configured model ID.
- Keep the key in the host environment or a local ignored `.env` loaded by the backend. The repository's `.gitignore` excludes `.env` and `.env.*` while allowing a placeholder-only `.env.example`; Person 2 owns maintaining those rules and creating the example. Keep real keys out of Git, frontend code, run logs, and the generated-code sandbox, following [OpenAI's authentication guidance](https://developers.openai.com/api/reference/overview#authentication).
- Record the working model IDs and a successful request/tool-response check, without recording the credential. Person 1 can keep building the simulator with fake fixtures while access is being configured.

These are implementation prerequisites assigned to the team; this planning update does not create a credential or verify this project's API access.

## 2. Scope and completion criteria

The first version has:

- A synthetic, one-dimensional vehicle/braking reference using MuJoCo mechanics and a private Python actuator with an internal thermal state.
- An editable Python candidate actuator coupled to the same MuJoCo mechanics; it initially applies fixed braking effectiveness and has no thermal state.
- An investigation loop that lets Astra inspect the candidate, request bounded experiments, modify its code, and validate the result.
- An original-model baseline and a parameter-fitted baseline using the same development data.
- A frozen prediction on new operating histories, followed by a visible reveal.
- A compact demo showing motion, stopping distance, evidence, a code diff, and measured errors.
- A Sol comparison using the same task and resources, if both models are available through the event environment.

**Done means an actual agent-authored Python actuator extension adds state and its evolution, runs coupled to MuJoCo, and is evaluated on withheld outcomes.** A written explanation, a thermal configuration toggle, or a prerecorded animation alone does not meet that bar. Show the changed Python source and its executed effect on MuJoCo trajectories. A manually written improved model is useful to validate the harness, but must be labeled as a developer baseline.

Defer physical hardware, full vehicle dynamics, ABS, steering, tire slip, realistic collision simulation, app accounts, and deployment. Keep the required MuJoCo scene minimal: one body moving along one slide joint. A barrier crossing is a visualization of stopping-distance error, not a crash simulation. Keep all reference results labeled **synthetic**.

## 3. Minimum physical model

Use **Python + MuJoCo + NumPy**, with SciPy for parameter fitting. Install the official `mujoco` Python package following the [MuJoCo Python documentation](https://mujoco.readthedocs.io/en/latest/python.html#installation), then pin versions that work on both laptops. MuJoCo advances mechanical position and velocity; the editable Python actuator computes braking force and evolves any state Astra adds. A standalone numerical model can be a development cross-check, but the submitted demo must use the isolated Python actuator's force output in the host's actual MuJoCo rollout.

### Mechanical engine and editable actuator boundary

Person 1 builds `sim/brake_scene.xml` with one slide joint and an effective vehicle mass, plus a fixed `sim/mujoco_bridge.py`. The bridge reads mechanical state, calls the actuator, applies the returned braking force along the slide coordinate, and steps MuJoCo. Keep candidate and reference mechanics, timestep, command handling, and stop handling identical; their Python actuator models are the intended difference.

Expose a small editable `candidate/actuator.py` with these proposed functions:

```text
init_state() -> actuator_state
compute_force(actuator_state, brake_command, velocity) -> braking_force_n
advance_state(actuator_state, brake_command, velocity, applied_braking_force_n, dt_s)
    -> next_actuator_state
```

The starting implementation has an empty actuator state, a fixed command-to-force mapping, and a no-op state update. Astra can extend these functions with a persistent state, its update rule, and its effect on force. Keep all three in the editable Python component. It uses ordinary numeric inputs and its own state, with no MuJoCo imports or engine objects. The host handles mechanical initialization and passes observed preparation history through the same actuator interface.

For the one-axis prototype, the host-only bridge can apply net drive/brake force through `data.qfrc_applied` before calling `mujoco.mj_step(model, data)`. MuJoCo documents this force-input and stepping interface in its [main simulation API](https://mujoco.readthedocs.io/en/latest/APIreference/APIfunctions.html#main-simulation). Set the force afresh each step and evolve Python state once per physics timestep, using the braking force actually applied after any stop handling. Freeze the bridge and scene during agent runs; restrict source edits to the actuator component. Execute that component in a separate isolated worker and exchange numeric messages with the host; do not import agent-authored code into the host's MuJoCo process. The candidate worker receives only candidate-rollout quantities or allowed preparation observations, never reference latent state.

The following equations describe the intended model; MuJoCo integrates the mechanical part rather than duplicating it in a separate Python motion solver.

For the initial candidate, use fixed braking effectiveness:

```text
state = [x, v]
dx/dt = v
m * dv/dt = F_drive - b * F_max
```

For the hidden reference, add a deliberately simplified thermal mechanism:

```text
state = [x, v, T]
F_brake = b * F_max * effectiveness(T)
dx/dt = v
m * dv/dt = F_drive - F_brake
C * dT/dt = eta * F_brake * v - h * (T - T_ambient)
```

Here `x` is distance in metres, `v` is speed in metres/second, `m` is mass in kilograms, `b` is brake command in `[0, 1]`, and forces are in newtons. `C` is effective heat capacity in joules/kelvin, `h` is cooling conductance in watts/kelvin, and `eta` is the fraction of dissipated braking power heating the modeled component. Temperature differences use kelvin. Choose a smooth, bounded effectiveness curve that is approximately constant when cool and decreases above a chosen threshold.

These are **our synthetic benchmark equations and tunable parameters**, not a calibrated model of a particular vehicle or brake compound. The physical motivation is that excess brake temperature can reduce braking performance; [Brembo's discussion of heat dissipation and fading](https://www.brembo.com/en/motorsport/formula1/ventilation-holes) supports that mechanism, not our chosen curve or numerical values.

Implementation rules:

- Detect stopping at the shared velocity threshold and resolve the final substep consistently; do not let a braking force numerically drive the car backward. Continue Python actuator-state updates while stationary so cooling persists.
- Integrate across command changes explicitly and use the same observation timestamps for comparisons.
- Repeated runs within an experiment preserve actuator state. A full reset resets both MuJoCo and Python state, with the reference at ambient conditions. Use distinct reference and candidate instances to prevent state leakage.
- Record acceleration/repositioning phases too: returning to the same visible speed must not secretly reset the thermal state.
- Let the candidate infer its own added state from observable history. Never initialize it with the reference temperature.
- Check smaller MuJoCo timesteps and convergence of the Python state update on representative runs. Numerical changes must be much smaller than the mismatch being demonstrated.

Before spending time on agent prompting, Person 1 should verify that a small developer-written stateful candidate improves predictions on separate development checks. This establishes that the task and interface are solvable. Keep that candidate outside the agent workspace and do not use final holdout outcomes to tune it.

## 4. The experiment that makes the idea convincing

### Development evidence and agent experiments

Start with two probe stops at the same initial speed and brake command, with distance measured from the probe start:

1. **Rested history:** ambient reset, accelerate to the probe speed, then brake.
2. **Repeated-use history:** several acceleration/braking cycles, return to the same probe speed, then apply the same brake command.

Show that the candidate treats these probe starts as equivalent, while the reference produces different trajectories. Provide the complete observable preparation history so a repaired model can estimate its own internal state.

Give the agent room to investigate: vary the number of prior stops, braking intensity, and rest duration. A useful next experiment compares similar prior braking with short versus long recovery. Recovery supports a reversible hidden-state explanation over permanent wear, although motion alone does not establish that the state is physically temperature.

Before every additional reference experiment, require the agent to record the requested sequence, the competing hypotheses it is testing, the observable result expected under each hypothesis, and what would weaken its current explanation. Qualitative expectations are sufficient; use numerical ranges when justified. The broker saves this record before executing the request. For example, recovery after waiting would support a reversible effect; no recovery over the tested interval would favor a more persistent effect without proving permanent damage. Keep these prospective expectations separate from the explanation written after results arrive.

Define reset actions precisely. Starting a **fresh specimen** resets both mechanics and the component's history state. Repositioning/reaccelerating the same specimen changes only the specified visible mechanical state while preserving component history. Waiting advances time and the component's update rule. Never treat “wait,” “prepare another trial,” and “start fresh” as interchangeable. Within an experiment, each preparation and wait remains in the recorded history.

Use an initial task such as: “Here is an incomplete Python model and access to an unknown reference system. Use observations and experiments to produce a better model. State expected observations before requesting an experiment, and report uncertainty.” Do not tell it to “add temperature,” name the backend, expose reference equations, or give it this planning document. Log its stated hypotheses, chosen experiments, evidence, and explanation; do not require a transcript of private internal reasoning.

### Held-out prediction and reveal

Person 1 reserves at least three probe scenarios with **new combinations of preparation, recovery, speed, or brake command**, plus a cool-brake check. Freeze their specifications before the first scored agent run; keep their future probe results outside the agent workspace.

1. End development and freeze the submitted candidate source, fitted parameters, and file hash.
2. Give that frozen candidate each holdout's observable preparation history, probe starting state, and future brake commands. The preparation observations are inputs; the future probe trajectory is withheld.
3. Execute the frozen model without further agent edits or fitting. Its latent state must come from the provided history, not a hidden-state measurement.
4. Save predictions, source hash, scenario IDs, and timestamps in a host-owned artifact before revealing reference probe results.
5. Score all reserved probes. Designate one reserved scenario for the stage reveal before final evaluation, using lessons from development trials to design it. It must remain a held-out probe; do not select it based on final scores.

Reset the reference independently between scenarios, while preserving state within each scenario. Development, baseline fitting, and final evaluation must agree on these semantics.

## 5. Architecture and the interface to agree first

Keep the builders' implementation in one Python project, but export only an allowlisted task package to the investigation workspace. The fixed MuJoCo bridge stays host-side and the editable Python actuator runs in an isolated worker. Use JSON artifacts to connect the simulation, agent, evaluation, and presentation. Start the presentation as a simple local page/report that reads saved MuJoCo trajectories; live rendering is optional. Builder/demo metadata is separate from agent-visible tool results.

```text
Investigation workspace                 Host-controlled services
-----------------------                 ------------------------
Incomplete Python source + traces <----> Neutral experiment/validation broker
                                                |
                         +----------------------+----------------------+
                         |                                             |
              Private reference + MuJoCo                  Candidate MuJoCo bridge
                                                                       |
                                              numeric messages only <-> isolated
                                                                       Python worker

Frozen candidate -> host-only evaluation -> prediction/reveal artifacts -> demo
```

Before splitting work, agree on these contracts and commit one fake fixture that exercises them. The names below are proposed interfaces to implement, not existing commands.

| Contract | Minimum inputs | Minimum outputs / behavior |
| --- | --- | --- |
| Actuator interface above | Opaque actuator state; command; velocity; applied force; timestep | Force and next state; original and patched implementations both work with the same fixed MuJoCo bridge |
| `run_experiment(spec)` | Request ID; bounded drive/brake/wait sequence; explicit fresh/continue semantics; hypotheses and expected observations | Neutral experiment ID and observed samples: `t_s`, `x_m`, `v_mps`, `brake`, `drive_force_n`, phase boundaries; no hidden state or backend metadata |
| `predict(candidate, history, probe)` | Candidate version; preparation samples; visible initial probe state; future commands | Probe samples, predicted stopping distance/time, completion status |
| `validate_candidate(candidate)` | Candidate version | Neutral status, candidate-code errors, finite-output checks and development metrics; host executes the actual MuJoCo rollout without exposing backend details |
| `submit_candidate(candidate)` | Candidate version and concise explanation | Immutable source/parameter hash; closes editing for that evaluation |
| Host-only `evaluate(submission, suite)` | Frozen submission and reserved scenarios | Per-scenario scores, prediction/reveal artifacts, aggregate scores |

Contract details to resolve in the first 20 minutes:

- Define each phase's termination rule: target speed, full stop, or elapsed duration, always with a time cap. A host driver may accelerate to a target; record the actual applied force.
- Make every experiment's specimen lifecycle explicit: either start fresh and include the full preparation sequence, or continue the caller's identified specimen. Isolate specimen sessions across models/runs; never reuse another run's hidden or candidate state.
- Use SI units, monotonically increasing timestamps, a shared sampling interval, and relative distance at the start of each probe.
- The candidate interface separates history ingestion from future rollout and allows additional internal state without changing its public output fields.
- History ingestion uses a frozen state-update rule. It may estimate state from past observations, but must not fit new parameters or an arbitrary initial state separately for each holdout.
- Stop when speed falls below one fixed small threshold. If a probe does not stop within the horizon, return `not_stopped`; do not invent a stopping distance or discard the case.
- Use explicit error/status fields for malformed requests, invalid patches, timeouts, and non-stopping trajectories. Expose actionable candidate-file errors; keep host stack traces, paths, engine symbols, and internal configuration in builder-only logs.
- The host audit artifact includes schema version, model identifier, actuator source hash, MuJoCo version, scene/bridge configuration hash, timestep, scenario IDs, tool counts, timing, prediction arrays, and metrics. The separate agent-visible projection contains only permitted observations, candidate errors, budgets/status, and development results.
- Person 1 owns schema changes; both agree before an interface changes. Person 2 can build against fake JSON immediately.

### Agent execution boundary

The investigation loop is: inspect evidence → state hypotheses and expected outcomes → request experiment → inspect results → patch candidate → validate → submit.

| Astra may access | Keep inaccessible to Astra and its generated code |
| --- | --- |
| Incomplete Python component and its own patches/state | Complete reference model, parameters, hidden temperature/damage, or developer solution |
| Neutral function contracts, units, allowed inputs and observation definitions | MuJoCo modules, bridge/scene files, engine configuration and engine-identifying metadata |
| Its requested experiment observations and development validation | Reserved test specifications during development and future probe outcomes before prediction freeze |
| Its previous expectations, results, and candidate-code errors | Planning docs, answer-bearing filenames/comments, Git history, host logs, credentials, and other agents' runs |

Expose only source read/edit tools and neutral operations such as `run_experiment`, `validate_candidate`, and `submit_candidate`. Use neutral case IDs and action names such as `wait` or `new_specimen`; do not expose names such as `test_damaged_bumper`, `thermal_reference`, or fields such as `damage_level`. Do not feed builder/demo labels into the agent context.

Use a separate reference service and a separate candidate-code worker with enforced filesystem/process/network boundaries. Neither the investigation workspace nor that worker mounts the host repository or installs/loads MuJoCo. A persistent worker exchanges bounded numeric messages with the host bridge and holds only its own component state. This preserves Python-to-MuJoCo coupling without giving the submitted code engine access through imports or Python introspection. Keep `OPENAI_API_KEY` in the host model adapter. An instruction not to read a file, or a directory named `hidden`, is insufficient.

Start a fresh investigation session from the allowlisted package; do not reuse a builder-assistant conversation that already read the reference or this plan. Before scored runs, verify that the workspace cannot read the reference/bridge/engine, and inspect success/error tool payloads for engine names, hidden-state fields, solution hints, and host paths. These checks establish the access boundary; they cannot establish that the model has no prior physics knowledge or cannot guess the backend. Accept an alternative internal representation if it satisfies the source-extension and predictive criteria.

Start with a provisional budget of **six additional experiments, three patch attempts, and eight minutes per agent run**, plus the same initial evidence. Also cap phase count, total simulated duration, force/speed ranges, and returned sample count per experiment so one request cannot contain unlimited trials. Person 1 sets those limits from the development rig before comparison runs. Count failed requests and attempts consistently. Confirm that this fits actual model latency and event limits before freezing the comparison protocol.

Use one thin model adapter with configurable model IDs for Astra and Sol. Verify actual access and tool behavior in the first checkpoint. Record the identifiers used; do not assume display names are API IDs. If one model is unavailable, report that comparison as unavailable and complete the working single-model demo.

## 6. Person 1: build the MuJoCo simulation and evaluation

**Your job:** make the missing dynamics measurable and provide a trustworthy way to execute and score the Python actuator repair. Person 1 also owns integration. One teammate takes this entire workstream; the other takes section 7.

**Own these files:** `sim/`, the initial `candidate/actuator.py` template, `reference_host/`, `component_worker/`, `evaluation/`, `contracts/`, `fixtures/`, simulator/boundary tests, and the dependency manifest. After handing off the candidate template, freeze it as the starting baseline; agent patches go into separate per-run copies.

Do these tasks in order:

1. **Install and verify MuJoCo.** Load the minimal slide-joint scene from Python and prove that a known applied force changes its position/velocity without opening a viewer. Share the working dependency versions.
2. **Agree the contract with Person 2.** Commit the actuator interface, units, reset/phase semantics, one fake JSON run artifact, and candidate stub. This is the early handoff that lets Person 2 proceed independently.
3. **Build the fixed mechanics bridge and both actuator paths.** Couple the editable Python candidate worker to host-side MuJoCo through numeric messages. Run the richer reference privately with identical mechanics and separate state. Enforce the filesystem/process/network boundary; no generated code is imported into the engine process. Preserve history across drive/brake/rest phases.
4. **Produce the mismatch and verify solvability.** Generate rested and repeated-use probes; check a developer-written stateful actuator on development scenarios. Keep that solution private from the investigation agent.
5. **Build the experiment broker and checks.** Return only the neutral observation schema and save expected outcomes before executing requests. Check resets, force signs, stopping behavior, state persistence, timestep convergence, and denied access to reference/engine files. Reserve final scenario specifications before the first scored agent run.
6. **Build the evaluator.** Add parameter-only fitting, the no-fade control, immutable submission hashes, withheld prediction/reveal, and per-scenario metrics. All scored candidate rollouts must use MuJoCo.
7. **Integrate and verify the final run.** Help connect Person 2's patch runner, verify the executed actuator diff and metrics, and document the reproduction command.

**Hand to Person 2:** first the fixture + interface + candidate stub; next the real broker/runner + example traces; finally frozen predictions + reference results + score artifacts. Each handoff includes a runnable command and expected output.

**Your completion gate:** an original and a stateful Python actuator run through the same MuJoCo mechanics, the submitted actuator has no access to engine/reference internals, and withheld outcomes remain protected until prediction is frozen.

## 7. Person 2: get the API key, build the agent, and present the demo

**Your job:** make Astra investigate, edit the Python actuator source, and submit an executable extension; then make that process and its measured outcome visible.

**Own these files:** `agent/`, `demo/`, agent tests, `.gitignore`, `.env.example`, and per-run patch/event artifacts. Coordinate dependency additions with Person 1. Do not edit Person 1's fixed MuJoCo bridge or baseline template during agent runs.

Do these tasks in order:

1. **Get and configure the OpenAI API key.** Complete the required setup in section 1, configure `OPENAI_API_KEY` only in the backend, and verify a small authenticated request. Check Astra and Sol access using the actual configured model IDs. Create ignored local configuration and a placeholder-only example file.
2. **Build against Person 1's fake fixture.** Implement the adapter, structured tool requests/results, and a simple run display. Start a fresh investigation context with only the allowlisted task package; exclude builder conversations, engine names, and mechanism hints.
3. **Implement the investigation tools.** Allow source reads/edits and neutral experiment/validation/submission operations. Require hypotheses, expected observations, and a disconfirming result before each additional experiment. Return useful candidate-code errors while keeping host paths, stack traces, and engine metadata private.
4. **Require a real source extension.** Save each patch to a fresh run workspace, execute the changed `candidate/actuator.py` through the fixed bridge, and show whether added state/update code improves development predictions. A configuration toggle or coefficient-only change is not the required extension.
5. **Connect the real broker and evaluator.** Replace the fixture with Person 1's service without changing the schema. Keep the API key outside the candidate worker. Log expectations before experiments and findings afterward. With Person 1, inspect agent-visible success/error payloads for leaks before scored runs.
6. **Run the model comparison.** Use fresh Astra/Sol runs with matched starting conditions and budgets. Hand the submitted actuator versions to Person 1 for final evaluation; do not patch after seeing holdout outcomes.
7. **Build and rehearse the demo.** Display the actual Python source diff, original/repaired MuJoCo predictions, reference reveal, errors, synthetic labels, and clearly marked live/replay status. Save a replay of a real completed run.

**Hand to Person 1:** first a working adapter/tool call using the fixture; next a submitted Python actuator patch + run log; finally a demo that consumes the evaluator's artifacts and a short presentation script. API credentials stay local and are never a handoff artifact.

**Your completion gate:** Astra uses tools and authors persistent-state/evolution code in the actuator, the patched component actually drives a MuJoCo rollout, and the demo shows a frozen prediction followed by the reference result.

## 8. Shared five-hour schedule and integration

This is an aggressive timebox with MuJoCo now required. Cut presentation polish and extra repetitions if needed; preserve the Python-actuator-to-MuJoCo execution path.

| Elapsed time | Person 1 | Person 2 | Shared checkpoint / exit condition |
| --- | --- | --- | --- |
| 0:00–0:20 | Verify MuJoCo force-to-motion; agree neutral contracts and isolation boundary | Obtain/configure API key; verify request; agree allowlisted task package | MuJoCo smoke test and API request work; commit contract, fixture, ownership, dependencies |
| 0:20–1:20 | Build host bridge, isolated candidate worker, private reference, matched probes; reserve holdouts | Build adapter, neutral dispatcher, expectations log, result view against fixture | Real mismatch and developer solvability check; Person 2 can complete a tool call and render a run |
| 1:20–2:00 | Connect broker; add parameter fitting and boundary/reset checks | Connect source editing and neutral validation; inspect payloads for leaks | Blind boundary verified; first complete evidence → expected outcome → experiment → Python extension → MuJoCo score |
| 2:00–3:00 | Implement freeze/reveal and no-fade control | Improve patch feedback; show real code diff and prediction artifacts | Working frozen prediction before reference reveal |
| 3:00–4:00 | Run baselines, control, and solver checks; collect per-case scores | Run Astra/Sol with equal budgets; handle failures and prepare replay | Results saved with provenance; no edits based on final holdout results |
| 4:00–5:00 | Verify clean-checkout instructions and review results | Polish visualization and rehearse the short demo | Joint rehearsal, honest labels, final integrated commit |

### Keep integration inexpensive

1. Both start from the same contract commit. Use separate clones/worktrees and branches such as `feat/brake-sim-eval` and `feat/agent-demo`; do not switch branches in one shared working directory.
2. Person 1 owns shared schemas and the dependency manifest. Person 2 requests additions in a short message instead of editing the same files concurrently.
3. Integrate a small working slice at each checkpoint. Person 1 merges; the other person updates from `main` before continuing.
4. Each handoff includes one command, one sample artifact, and the expected output. Agree the exact commands at the first checkpoint and put them in the implementation README.
5. Use a five-minute sync at each checkpoint: what works, what changed in the contract, and the next blocker. Avoid waiting until the final hour to join the systems.

The initial fixture is the key to parallel work: Person 2 does not need to wait for the simulator, and Person 1 does not need to wait for model access or UI work.

## 9. Evaluation: prove the repair helped

Compare these approaches on the same final scenarios:

| Approach | Purpose |
| --- | --- |
| Original fixed-effectiveness candidate | Establish the initial failure |
| Parameter-fitted candidate with unchanged state structure | Test whether ordinary coefficient tuning is sufficient |
| Astra's submitted code patch | Measure executable model extension and predictive improvement |
| Sol's submitted code patch, when available | Compare the same investigation/repair workflow across models |

Use bounded [`scipy.optimize.least_squares`](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.least_squares.html) to fit the fixed-effectiveness baseline on development observations. For each agent, fit a matched baseline on the initial evidence plus the additional development data that agent acquired; never fit on final outcomes. Do not allow scenario IDs or separate per-history constants to substitute for modeling state.

Primary metric: **mean absolute stopping-distance error in metres**, reported per scenario and across the reserved probes. Secondary metrics: speed-trajectory RMSE on a common time grid, stopping-time error, barrier-crossing prediction correctness, patch validity, experiment count, runtime, and token/cost usage when available. Report non-stopping cases separately and count them as failed stop predictions; never silently drop them from the scorecard.

Set these provisional engineering targets before final evaluation; they are goals, not claimed results:

- The original simulator has a clear history-dependent error on development probes.
- The submitted source diff adds persistent actuator state and its evolution in Python, and the resulting force is executed through the fixed MuJoCo bridge. Parameter edits or configuration toggles alone do not satisfy this requirement.
- The repaired model cuts mean held-out stopping-distance error by at least **50% versus the original** and beats its matched parameter-only baseline.
- The cool-brake check does not regress materially; use a tolerance of the larger of **0.5 m or 5% of reference stopping distance**.
- A separate **no-fade reference** is already explainable by the simple model. Run the investigation there too and check whether the agent avoids claiming unsupported missing physics or making predictions worse.
- The frozen candidate executes without invalid states, hidden-data access, or evaluation edits.
- The investigation receives a neutral task/interface, with no disclosed engine identity or reference implementation; candidate execution cannot inspect the engine process or files. Record the access checks with the host audit artifacts.
- Every additional experiment has a timestamped specification and expected-outcomes record saved before observations are returned.
- Predictions are saved before future reference results are available to the agent or demo audience.

The source-extension requirement applies to the deliberately incomplete brake-fade case. On the no-fade control, retaining the adequate original model is a valid outcome; do not force unnecessary state into that control. A repaired model need not copy the hidden equations, coefficients, variable names, or code. Score its valid executable structure and unseen predictions, with uncertainty about physical interpretation.

Both models get the same prompt, initial candidate, initial evidence, tools, allowed actions, holdout suite, and budget ceilings. Their chosen experiments may differ—that is part of the task. Start each run fresh with no other model's patches or transcript. If time allows, run three paired repetitions using the same scenario seeds; otherwise explicitly label the comparison as one exploratory run per model. Show failures and ties as well as successes; one demo does not establish general model superiority.

Motion evidence may support a useful latent state without uniquely identifying temperature. Describe the contribution as **recovery of deliberately omitted dynamics through experiments and executable model repair**. A separate temperature measurement would be a later physical-interpretation check.

## 10. What the audience sees

Keep the presentation to roughly three minutes:

1. **Mismatch, 30 seconds:** cold and repeated-use stops start at the same speed. Show the original stopping marker and the synthetic reference crossing it.
2. **Investigation, 45 seconds:** show the agent's competing hypotheses and expected outcomes recorded before its selected experiment, then show the observed result and interpretation.
3. **Repair, 30 seconds:** show the actual Python actuator diff introducing state/evolution and its executed MuJoCo validation. Explain that the actuator was repaired while MuJoCo supplies the mechanics.
4. **Prediction, 45 seconds:** display the frozen original and repaired stopping predictions for the reserved trial; then reveal the reference trajectory.
5. **Evidence, 30 seconds:** display errors for all reserved probes, the parameter-fit baseline, and Astra/Sol results if available.

Render three distinguishable tracks or traces: original prediction, repaired prediction, and synthetic reference. Fix the barrier position from development trials before final evaluation. Show units and a persistent “Synthetic benchmark” label. Display inferred state as “inferred state,” not a measured temperature. Use recorded structured events for the investigation view rather than inventing dialogue.

Record a replay of a real run once the loop works. Label replay versus live execution clearly. If a repair fails, show the failure and measured scores; do not substitute a developer-written patch and attribute it to the agent.

## 11. Cut lines and risks

| Trigger | Action |
| --- | --- |
| No model tool call works by minute 30 | Person 2 resolves access while Person 1 continues with fixtures. A scripted driver can test plumbing, but cannot count as the agent result. |
| MuJoCo coupling is not working by hour 1 | Simplify the scene to one slide joint and run without a viewer. Cut visual polish; keep MuJoCo mechanics and the editable Python actuator as required scope. |
| Isolation or neutral-payload checks fail | Fix the service/worker boundary and leaked context before scored runs. A run with disclosed internals must be labeled assisted and cannot count as the blind experiment. |
| No clear physics mismatch by hour 1 | Simplify to one braking axis and deterministic observations; adjust synthetic parameters using development scenarios only. |
| No complete executable repair loop by hour 2 | Stop UI polish and extra comparisons; both debug the smallest evidence → patch → validation path. |
| Generated patches fail | Return concrete syntax/runtime/interface errors within the fixed attempt budget. Retain the last valid version and report submission failure if needed. |
| The repair fits examples but fails holdouts | Report that outcome. Debug with development cases; a new iteration needs fresh reserved cases for a new final claim. |
| Model latency threatens the stage demo | Use the recorded real run and its frozen prediction/reveal artifacts, clearly marked as replay. |
| Core demo is stable with time remaining | Add repetitions, noise robustness, or stronger controls before starting a second use case. |

## 12. Move on only after brake fade works

Suggested continuation order after use case 3 is **robot arm/gripper (1)**, then **phone thermal throttling (2)**, because the original idea emphasizes robotics. This is a proposed follow-on order; brake fade is the committed first priority.

| Phase | Keep | Change | Gate |
| --- | --- | --- | --- |
| 1: Brake fade | Establish Python actuator + MuJoCo coupling, experiment broker, source repair, freeze/reveal, scoring | Build the smallest synthetic braking system | Reproducible actuator-state extension executed through MuJoCo with honest holdout results |
| 2: Robot arm/gripper | Reuse MuJoCo coupling, agent tools, artifact format, isolation, model comparison | Replace the slide scene with a one-joint robot scene and the brake component with an editable Python motor actuator | Predict new joint-motion histories using source-code extension of the actuator |
| 3: Phone throttling | Investigation and submission workflow | Replace commands/observations with workload and performance; add a synthetic thermal reference | Predict unseen workload/rest sequences and demonstrate model reuse |

### Optional bumper-impact experiment from the walkthrough

This is a separate example to build after the brake-fade milestone if the team chooses it. It does not replace the first use case or require building realistic vehicle collisions. It uses the same principle of an editable Python force component coupled to hidden MuJoCo mechanics; here that component represents a bumper/contact law rather than a brake actuator.

Person 1 would own the cart/wall scene, bumper reference, reset semantics, and impact metrics. Person 2 would reuse the neutral tools, expectation log, source patching, and reveal display with the new observation schema.

| Step | What happens | What the experiment establishes |
| --- | --- | --- |
| 1. Build the toy system | A cart hits a wall; the Python bumper component supplies contact force while MuJoCo advances motion. Disable duplicate native contact response for that modeled interaction in both reference and candidate paths. | Exactly one modeled bumper force acts; the engine still supplies mechanics. |
| 2. Define the omission | The private reference accumulates an internal history variable and is deliberately designed to soften and dissipate more energy after sufficiently strong impacts. The visible Python component uses fixed properties. | These are toy-reference assumptions, not general claims about real bumpers. |
| 3. Observe a fresh impact | Start a fresh specimen and record time, position, velocity, compression, and contact force at a chosen starting speed. | A good fit on one fresh collision alone does not establish missing state. |
| 4. Repeat on the same specimen | Reposition the cart to the same visible starting conditions without resetting component history; repeat the impact. | Different compression/rebound despite matched visible conditions exposes a history-dependent mismatch. |
| 5. Investigate | Before each request, state expected outcomes for a fresh specimen, waiting before reusing a specimen, or changing prior impact severity. Then reveal the observed response. | Fresh-specimen recovery, lack of recovery over the tested wait, and severity dependence can support a persistent mechanism; none proves a unique material explanation. |
| 6. Extend source | The agent adds its own state, an update rule driven by prior impacts, and an effect on the force law. Numerical tools fit coefficients on development data. | State comes from observed history, never the private reference damage value or one answer per recording. |
| 7. Freeze and predict | Freeze code/parameters, then supply a new preparation history such as two gentle impacts, one stronger impact, and a pause. Predict the final impact at a different speed before revealing its outcome. | The frozen model must maintain its estimated state and generalize to a new sequence. |
| 8. Reveal and score | Compare peak compression, rebound velocity, and force-trace error for the original, parameter-fit baseline, and repaired component. Use actual run values. | Predictive improvement is measured independently of whether the state is named “damage.” |

Define impact-specific phase endings and validation: rebound needs signed velocity, so the brake scenario's no-backward-motion guard must not suppress it. Check single contact-force application and physically sensible energy behavior in development. Use an unchanged-property bumper as the adequate-model control. Keep bumper scores separate from braking-distance scores, and reserve its final sequences independently.

Define a small scenario adapter only when adding the second use case: observation fields, experiment actions, candidate entry point, metrics, and renderer. Avoid building a generic plugin framework during the first demo. Hardware measurements and stronger novelty comparisons belong after the synthetic workflow is established.

## 13. First actions when implementation starts

- [ ] Assign Person 1 to section 6 and Person 2 to section 7; confirm actual build time.
- [ ] Person 2 obtains/configures the OpenAI API key and verifies model access with a small request.
- [ ] Person 1 installs MuJoCo and verifies that the Python actuator's force drives the minimal mechanical scene.
- [ ] Commit the shared schema, one fake run artifact, and the dependency setup.
- [ ] Verify that the investigation task package and candidate worker cannot access engine/reference internals, and that tool results contain only the neutral schema.
- [ ] Require a saved expected-outcomes record before each additional experiment executes.
- [ ] Branch into simulation/evaluation and agent/demo workstreams.
- [ ] Person 1 produces the two matched probe traces; Person 2 completes one tool call against the fixture.
- [ ] Integrate the first complete Python actuator source-extension → MuJoCo rollout → evaluation loop before polishing the presentation.
- [ ] Freeze predictions, reveal all reserved results, and save a labeled replay.
- [ ] Document setup/run instructions, review the final artifact set, and rehearse together.
