# RealityPatch: brake-fade-first hackathon plan

Based on [idea1.md](idea1.md) and [idea1UseCases.md](idea1UseCases.md).

## 1. What we are building

Start with **use case 3: brake fade**. RealityPatch investigates why a simple braking simulator predicts the wrong stopping distance, requests experiments, adds missing history-dependent behavior to the simulator's code, and predicts an unseen braking trial before its result is revealed.

**Demo pitch:** “The simulator says the car will stop before the barrier. After repeated braking, the synthetic reference crosses it. RealityPatch investigates the mismatch, repairs its model, and predicts the next stop.”

The repair improves the prediction; it does not physically improve the brakes. On the final trial, show the original prediction, repaired prediction, and actual reference trajectory for the same commands. A later feature could use the improved model to recommend earlier braking.

The current repository contains the two idea documents and no implementation. This plan assumes two builders and uses a **five-hour working timebox**, not a confirmed event deadline. If there is more time, expand validation before adding another use case. Person A and Person B below are interchangeable role assignments based on your strengths.

## 2. Scope and completion criteria

The first version has:

- A synthetic, one-dimensional vehicle/braking reference with an internal thermal state.
- An editable Python candidate model that initially knows position and speed but lacks that state.
- An investigation loop that lets Astra inspect the candidate, request bounded experiments, modify its code, and validate the result.
- An original-model baseline and a parameter-fitted baseline using the same development data.
- A frozen prediction on new operating histories, followed by a visible reveal.
- A compact demo showing motion, stopping distance, evidence, a code diff, and measured errors.
- A Sol comparison using the same task and resources, if both models are available through the event environment.

**Done means an actual agent-authored patch runs and is evaluated on withheld outcomes.** A written explanation, a manually selected thermal model, or a prerecorded animation alone does not meet that bar. A manually written improved model is useful to validate the harness, but must be labeled as a developer baseline.

Defer MuJoCo, physical hardware, full vehicle dynamics, ABS, steering, tire slip, realistic collision simulation, accounts, and deployment. A barrier crossing is a visualization of stopping-distance error, not a crash simulation. Keep all reference results labeled **synthetic**.

## 3. Minimum physical model

Use Python with NumPy and SciPy for the first simulation. SciPy's [`solve_ivp`](https://docs.scipy.org/doc/scipy/reference/generated/scipy.integrate.solve_ivp.html) integrates systems of differential equations and supports event detection for stopping. Pin the versions that actually work on both laptops.

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

- Stop motion at `v = 0`; do not let numerical integration drive the car backward. Continue cooling while stationary.
- Integrate across command changes explicitly and use the same observation timestamps for comparisons.
- Repeated runs within an experiment preserve temperature. A full reset starts a new experiment at ambient conditions.
- Record acceleration/repositioning phases too: returning to the same visible speed must not secretly reset the thermal state.
- Let the candidate infer its own added state from observable history. Never initialize it with the reference temperature.
- Check tighter solver tolerances and smaller maximum steps on representative runs. Numerical changes must be much smaller than the mismatch being demonstrated.

Before spending time on agent prompting, Person A should verify that a small developer-written stateful candidate improves predictions on separate development checks. This establishes that the task and interface are solvable. Keep that candidate outside the agent workspace and do not use final holdout outcomes to tune it.

## 4. The experiment that makes the idea convincing

### Development evidence and agent experiments

Start with two probe stops at the same initial speed and brake command, with distance measured from the probe start:

1. **Rested history:** ambient reset, accelerate to the probe speed, then brake.
2. **Repeated-use history:** several acceleration/braking cycles, return to the same probe speed, then apply the same brake command.

Show that the candidate treats these probe starts as equivalent, while the reference produces different trajectories. Provide the complete observable preparation history so a repaired model can estimate its own internal state.

Give the agent room to investigate: vary the number of prior stops, braking intensity, and rest duration. A useful next experiment compares similar prior braking with short versus long recovery. Recovery supports a reversible hidden-state explanation over permanent wear, although motion alone does not establish that the state is physically temperature.

The agent's initial task should say that the simulator may be incomplete and should report uncertainty. Do not tell it to “add temperature,” expose the reference equations, or give it this planning document. Log its stated hypotheses, chosen experiments, evidence, and explanation; do not require a transcript of private internal reasoning.

### Held-out prediction and reveal

Person A reserves at least three probe scenarios with **new combinations of preparation, recovery, speed, or brake command**, plus a cool-brake check. Freeze their specifications before the first scored agent run; keep their future probe results outside the agent workspace.

1. End development and freeze the submitted candidate source, fitted parameters, and file hash.
2. Give that frozen candidate each holdout's observable preparation history, probe starting state, and future brake commands. The preparation observations are inputs; the future probe trajectory is withheld.
3. Execute the frozen model without further agent edits or fitting. Its latent state must come from the provided history, not a hidden-state measurement.
4. Save predictions, source hash, scenario IDs, and timestamps in a host-owned artifact before revealing reference probe results.
5. Score all reserved probes. Designate one reserved scenario for the stage reveal before final evaluation, using lessons from development trials to design it. It must remain a held-out probe; do not select it based on final scores.

Reset the reference independently between scenarios, while preserving state within each scenario. Development, baseline fitting, and final evaluation must agree on these semantics.

## 5. Architecture and the interface to agree first

Keep the first version in one Python project. Use JSON artifacts to connect the simulation, agent, evaluation, and presentation. Start the presentation as a simple local page/report that reads saved artifacts; live updates are optional. This avoids making the physics depend on a frontend framework.

```text
Agent workspace                         Host-controlled environment
-----------------------------           ---------------------------
Editable candidate + observations <---> Experiment broker -> hidden reference
          |
          +--> bounded patch runner ---> validation metrics
          |
          +--> frozen candidate --------> evaluator -> saved results -> demo
```

Before splitting work, agree on these contracts and commit one fake fixture that exercises them. The names below are proposed interfaces to implement, not existing commands.

| Contract | Minimum inputs | Minimum outputs / behavior |
| --- | --- | --- |
| `run_experiment(spec)` | Request ID; sequence of bounded drive, brake, and rest phases; full-reset flag | Experiment ID and observed samples: `t_s`, `x_m`, `v_mps`, `brake`, `drive_force_n`, phase boundaries; no hidden state |
| `predict(candidate, history, probe)` | Candidate version; preparation samples; visible initial probe state; future commands | Probe samples, predicted stopping distance/time, completion status |
| `validate_candidate(candidate)` | Candidate version | Import/interface checks, finite-output checks, development errors, bounded runtime result |
| `submit_candidate(candidate)` | Candidate version and concise explanation | Immutable source/parameter hash; closes editing for that evaluation |
| Host-only `evaluate(submission, suite)` | Frozen submission and reserved scenarios | Per-scenario scores, prediction/reveal artifacts, aggregate scores |

Contract details to resolve in the first 20 minutes:

- Define each phase's termination rule: target speed, full stop, or elapsed duration, always with a time cap. A host driver may accelerate to a target; record the actual applied force.
- Use SI units, monotonically increasing timestamps, a shared sampling interval, and relative distance at the start of each probe.
- The candidate interface separates history ingestion from future rollout and allows additional internal state without changing its public output fields.
- History ingestion uses a frozen state-update rule. It may estimate state from past observations, but must not fit new parameters or an arbitrary initial state separately for each holdout.
- Stop when speed falls below one fixed small threshold. If a probe does not stop within the horizon, return `not_stopped`; do not invent a stopping distance or discard the case.
- Use explicit error/status fields for malformed requests, invalid patches, timeouts, and non-stopping trajectories.
- A run artifact includes schema version, model identifier, source hash, scenario IDs, tool counts, timing, prediction arrays, and metrics.
- Person A owns schema changes; both agree before an interface changes. Person B can build against fake JSON immediately.

### Agent execution boundary

The investigation loop is: inspect evidence → state hypotheses → request experiment → inspect results → patch candidate → validate → submit.

Expose only candidate reads/edits, the experiment broker, and the bounded candidate runner. Run generated code in an isolated workspace with time limits and no access to the reference source, evaluator, hidden fixtures, project planning docs, Git history, or service credentials. A folder named `hidden` inside an otherwise readable checkout is not isolation. Use an allowlisted workspace/container and a broker outside it; keep credentials in the host adapter.

Start with a provisional budget of **six additional experiments, three patch attempts, and eight minutes per agent run**, plus the same initial evidence. Also cap phase count, total simulated duration, force/speed ranges, and returned sample count per experiment so one request cannot contain unlimited trials. Person A sets those limits from the development rig before comparison runs. Count failed requests and attempts consistently. Confirm that this fits actual model latency and event limits before freezing the comparison protocol.

Use one thin model adapter with configurable model IDs for Astra and Sol. Verify actual access and tool behavior in the first checkpoint. Record the identifiers used; do not assume display names are API IDs. If one model is unavailable, report that comparison as unavailable and complete the working single-model demo.

## 6. How the two of you work in parallel

**Suggested split:** Person A takes simulation and evaluation; Person B takes agent integration and the demo. Swap people if your strengths suggest it. Person A also serves as integration owner so shared decisions have one owner.

| Area | Person A: simulation and evaluation | Person B: agent and demo |
| --- | --- | --- |
| Main responsibility | Make the mismatch measurable and the score trustworthy | Make an agent produce a working repair and explain the result |
| Owns | `sim/`, `reference_host/`, `evaluation/`, `contracts/`, `fixtures/`, simulator tests | `agent/`, `demo/`, agent tests, run-event rendering |
| First independent deliverable | Cool/repeated-use traces and candidate stub matching the contract | Model access check, tool loop, demo rendering the shared fake fixture |
| Next deliverable | Real broker, parameter-fit baseline, reserved scenarios | Experiment requests, source patching, validation feedback, submission |
| Integration responsibility | Reference isolation, metric correctness, dependency manifest | Adapter errors, artifact handling, UI labels and replay behavior |
| Final deliverable | Reproducible evaluation command and scored artifacts | Complete demo flow, code diff, run replay, short presentation script |

### Five-hour schedule

| Elapsed time | Person A | Person B | Shared checkpoint / exit condition |
| --- | --- | --- | --- |
| 0:00–0:20 | Agree equations, units, experiment semantics, and output schema | Check model access and agree tools/artifacts | Commit contract, fixture, ownership, and dependency choices |
| 0:20–1:20 | Build reference, original candidate, and matched probes; reserve holdout specs | Build against fixture: adapter, tool dispatcher, result view | Real mismatch and a developer stateful-model solvability check; B can complete a tool call and render a run |
| 1:20–2:00 | Connect real broker; add parameter fitting and basic checks | Connect tools to candidate editing and validation | First complete run: evidence → experiment → executable patch → development score |
| 2:00–3:00 | Implement freeze/reveal and no-fade control | Improve patch feedback; show real code diff and prediction artifacts | Working frozen prediction before reference reveal |
| 3:00–4:00 | Run baselines, control, and solver checks; collect per-case scores | Run Astra/Sol with equal budgets; handle failures and prepare replay | Results saved with provenance; no edits based on final holdout results |
| 4:00–5:00 | Verify clean-checkout instructions and review results | Polish visualization and rehearse the short demo | Joint rehearsal, honest labels, final integrated commit |

### Keep integration inexpensive

1. Both start from the same contract commit. Use separate clones/worktrees and branches such as `feat/brake-sim-eval` and `feat/agent-demo`; do not switch branches in one shared working directory.
2. Person A owns shared schemas and the dependency manifest. Person B requests additions in a short message instead of editing the same files concurrently.
3. Integrate a small working slice at each checkpoint. Person A merges; the other person updates from `main` before continuing.
4. Each handoff includes one command, one sample artifact, and the expected output. Agree the exact commands at the first checkpoint and put them in the implementation README.
5. Use a five-minute sync at each checkpoint: what works, what changed in the contract, and the next blocker. Avoid waiting until the final hour to join the systems.

The initial fixture is the key to parallel work: Person B does not need to wait for the simulator, and Person A does not need to wait for model access or UI work.

## 7. Evaluation: prove the repair helped

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
- The repaired model cuts mean held-out stopping-distance error by at least **50% versus the original** and beats its matched parameter-only baseline.
- The cool-brake check does not regress materially; use a tolerance of the larger of **0.5 m or 5% of reference stopping distance**.
- A separate **no-fade reference** is already explainable by the simple model. Run the investigation there too and check whether the agent avoids claiming unsupported missing physics or making predictions worse.
- The frozen candidate executes without invalid states, hidden-data access, or evaluation edits.
- Predictions are saved before future reference results are available to the agent or demo audience.

Both models get the same prompt, initial candidate, initial evidence, tools, allowed actions, holdout suite, and budget ceilings. Their chosen experiments may differ—that is part of the task. Start each run fresh with no other model's patches or transcript. If time allows, run three paired repetitions using the same scenario seeds; otherwise explicitly label the comparison as one exploratory run per model. Show failures and ties as well as successes; one demo does not establish general model superiority.

Motion evidence may support a useful latent state without uniquely identifying temperature. Describe the contribution as **recovery of deliberately omitted dynamics through experiments and executable model repair**. A separate temperature measurement would be a later physical-interpretation check.

## 8. What the audience sees

Keep the presentation to roughly three minutes:

1. **Mismatch, 30 seconds:** cold and repeated-use stops start at the same speed. Show the original stopping marker and the synthetic reference crossing it.
2. **Investigation, 45 seconds:** show the agent's stated competing hypotheses, selected recovery experiment, and observed result.
3. **Repair, 30 seconds:** show the actual source diff introducing state/evolution and the successful executable validation.
4. **Prediction, 45 seconds:** display the frozen original and repaired stopping predictions for the reserved trial; then reveal the reference trajectory.
5. **Evidence, 30 seconds:** display errors for all reserved probes, the parameter-fit baseline, and Astra/Sol results if available.

Render three distinguishable tracks or traces: original prediction, repaired prediction, and synthetic reference. Fix the barrier position from development trials before final evaluation. Show units and a persistent “Synthetic benchmark” label. Display inferred state as “inferred state,” not a measured temperature. Use recorded structured events for the investigation view rather than inventing dialogue.

Record a replay of a real run once the loop works. Label replay versus live execution clearly. If a repair fails, show the failure and measured scores; do not substitute a developer-written patch and attribute it to the agent.

## 9. Cut lines and risks

| Trigger | Action |
| --- | --- |
| No model tool call works by minute 30 | Person B resolves access while Person A continues with fixtures. A scripted driver can test plumbing, but cannot count as the agent result. |
| No clear physics mismatch by hour 1 | Simplify to one braking axis and deterministic observations; adjust synthetic parameters using development scenarios only. |
| No complete executable repair loop by hour 2 | Stop UI polish and extra comparisons; both debug the smallest evidence → patch → validation path. |
| Generated patches fail | Return concrete syntax/runtime/interface errors within the fixed attempt budget. Retain the last valid version and report submission failure if needed. |
| The repair fits examples but fails holdouts | Report that outcome. Debug with development cases; a new iteration needs fresh reserved cases for a new final claim. |
| Model latency threatens the stage demo | Use the recorded real run and its frozen prediction/reveal artifacts, clearly marked as replay. |
| Core demo is stable with time remaining | Add repetitions, noise robustness, or stronger controls before starting a second use case. |

## 10. Move on only after brake fade works

Suggested continuation order after use case 3 is **robot arm/gripper (1)**, then **phone thermal throttling (2)**, because the original idea emphasizes robotics. This is a proposed follow-on order; brake fade is the committed first priority.

| Phase | Keep | Change | Gate |
| --- | --- | --- | --- |
| 1: Brake fade | Establish experiment broker, code repair, freeze/reveal, scoring | Build the smallest synthetic braking system | Reproducible end-to-end run with an executable repair and honest holdout results |
| 2: Robot arm/gripper | Agent tools, artifact format, isolation, model comparison | Add an editable actuator with omitted history dependence; optionally couple to a one-joint MuJoCo scene | Verify installed engine capabilities; predict new joint-motion histories |
| 3: Phone throttling | Investigation and submission workflow | Replace commands/observations with workload and performance; add a synthetic thermal reference | Predict unseen workload/rest sequences and demonstrate model reuse |

Define a small scenario adapter only when adding the second use case: observation fields, experiment actions, candidate entry point, metrics, and renderer. Avoid building a generic plugin framework during the first demo. Hardware measurements and stronger novelty comparisons belong after the synthetic workflow is established.

## 11. First actions when implementation starts

- [ ] Assign Person A and Person B; confirm actual build time and model access.
- [ ] Commit the shared schema, one fake run artifact, and the dependency setup.
- [ ] Branch into simulation/evaluation and agent/demo workstreams.
- [ ] Person A produces the two matched probe traces; Person B completes one tool call against the fixture.
- [ ] Integrate the first complete repair loop before polishing the presentation.
- [ ] Freeze predictions, reveal all reserved results, and save a labeled replay.
- [ ] Document setup/run instructions, review the final artifact set, and rehearse together.
