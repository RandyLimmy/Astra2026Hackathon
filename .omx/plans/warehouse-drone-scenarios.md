# Warehouse payload physics and drone degradation

Status: delivered and verified, 2026-09-10. User authorized planning and building,
then requested pulling main and continuing. HEAD now matches origin/main 9fe0d19.

## Requirements and scope

Provide physical warehouse payload/drive failures and drone rotor/power/load/wind
failures. Supply observable motion, controlled probes, nominal predictions,
recordings, resets and usable nominal exports for Astra model maintenance.
The existing simulator contract and README assign inference, candidate editing,
protected held-out evaluation and the autonomous agent loop to another workstream.
This delivery supplies simulator evidence and interventions, not a fictional
claim that Astra has repaired a model. Spacecraft was comparison context.

## Implementation and ownership

1. Warehouse physical platform: simulator/platforms/warehouse.py and
   simulator/assets/warehouse.xml. Physical driven wheels, caster and latched
   cargo rail; seven presets and straight/turn/pulse/challenge probes. Extra cargo
   loaded at reset; contact, joint resistance and motor gain generate other faults.
2. Drone platform: reuse simulator/platforms/drone.py and assets/platforms/drone.xml
   from the concurrent platform workstream. Add three diagnostic/challenge schedules
   in simulator/experiments/drone-*.json and independent tests/test_drone_lab.py.
   A concurrent write collision was recovered from the original author's patches;
   no duplicate engine remains. Only camera/lighting changes were retained in the
   shared drone asset by this session.
3. Intervention lab: simulator/lab.py reuses the shared platform catalog and
   existing Recorder. Explicit timestamped controls replay the same commands in
   both worlds; nominal prediction bytes and SHA-256 are saved before constructing
   the changed world. Compare position trajectories only over common recorded time.
4. Shared CLI integration: warehouse added to simulator/platforms/catalog.py.
   Pull brought three commits and isolated car component comparisons. Resolve
   simulator/__main__.py by retaining one compare parser with platform/car dispatch
   plus both export-task and export-platform. Preserve other agents' code.
5. Handoff and distribution: simulator/LAB.md, README link, package-data includes
   nested MJCF and experiment JSON. No runtime dependencies added by this work.
6. Verification: tests/test_warehouse.py, tests/test_drone_lab.py,
   tests/test_lab_cli.py, incoming full pytest suite, Ruff, mypy, physical matched
   runs and rendered replay inspection. Minimal typing corrections in incoming
   worker/client, runner and comparison code preserve behavior and OS isolation.

## Acceptance criteria and measured evidence

- 13 preset pairs complete without solver warnings. Healthy traces match exactly;
  every default fault changes observable motion. Results:
  runs/warehouse-drone-final-validation/report.json.
- Warehouse straight x at 8 s: healthy 8.901 m, extra mass 6.217 m, slip 7.318 m,
  resistance 5.416 m. Pulse x: healthy 5.702 m, motor degradation 2.868 m.
- Cargo moves physically and produces 0.173 m peak path mismatch. Opposed turns
  partly cancel final error; assess the transient trajectory as well as endpoint.
- Warehouse same-config state repeatability within 1e-10; calibrated half-step
  endpoints/path length within 0.12 m. Drone force-frame tests match rotor forces,
  lever arms and reaction torques; short pre-contact half-step agreement within
  5 mm. These bounds apply to tested configurations only.
- Trial reset retains damage and monotonic experiment time; full reset replays
  original configuration. Public records exclude hidden values and event labels.
- Invalid configs/controls and overwrites rejected. Prediction hash matches exact
  file bytes; construction/run ordering tested. Healthy exports reload in MuJoCo.
- Three healthy drone diagnostic recipes remain within the synthetic flight
  envelope. Five milder fault challenge runs stay airborne; four exceed nominal
  tracking tolerance. See runs/drone-maintenance-validation/report.json.
- Final rendered warehouse/drone MP4 comparisons verified, nominal left and
  changed world right. Visual verdicts saved under .omx/state/*-visual/.
- Full post-pull pytest: 174 tests +501 subtests pass. Final CLI/operator checks:
  11 tests +16 subtests pass. Final affected worker/car checks: 46 tests +30
  subtests pass, including unchanged isolated reruns of three startup timeouts.
  Ruff and mypy pass; full details in simulator/LAB.md.

## Risks and mitigations

- Cross-session edits: shared ownership note in warehouse-drone-coordination.md;
  pre-pull backup retained, conflicting CLI merged, no external code discarded.
- Confounded physics: vary torque, direction and pulse timing; one hover or
  acceleration residual does not uniquely identify mass versus motor gain.
- Drone payload is an explicit co-moving pickup, not a simulated grasp. Predict
  declared pickup schedules or post-observation motion; no clairvoyance claim.
- Core drone delay is transport delay plus fixed spool time, not changing spool
  constant. Maintain those labels in experiments.
- Severe faults can remove physical maneuver capability; a model patch cannot
  restore actuator capacity. Milder presets support airborne diagnosis.
- Public folders are an export convention. Integration must enforce actual
  process/filesystem isolation and private held-out maneuvers.
- Worker startup tests are load-sensitive; isolated reruns passed unchanged.
