# Cargo failure animation and model repair

Build a clear physical trolley cargo failure and a separate executable model-repair experiment for GPT-6. Existing drone/quadruped work and original warehouse presets remain intact.

Requirements:
1. A staged trolley animation has visible secured-load travel, turning, physical cargo motion and observable path mismatch. Derive all motion from MuJoCo. Show synchronized nominal/reference traces and explicit labels.
2. Include matched healthy control and multiple cargo test schedules/load severities. Keep an invariant hidden mechanism across development and held-out cases. Prefer a breakaway latch driven by real constraint load, avoiding arbitrary unknown future failure time; preserve the older timed release.
3. Export a neutral model/evidence bundle. Hidden configuration, fault labels, and private telemetry stay outside model tools. A candidate must be executed only in its own nominal simulation.
4. Provide a bounded declarative model artifact (JSON) that GPT-6 can edit to change physical model parameters and add persistent constraint-release transitions. This is model repair of a missing release mechanism, not a claim of invented degrees of freedom or Python code authorship.
5. Provide tools/CLI to inspect, experiment, patch, predict, freeze, and evaluate the candidate on unseen controls. Persist candidate hashes and predictions before revealing held-out reality. Reuse API profile configuration, but avoid altering the brake investigation protocol.
6. Animation can demonstrate original mismatch immediately. A repaired track only appears after executing a supplied/evaluated candidate. Never invent GPT-6 success. No paid API run needed to prepare the user's scenarios for later inference.

Acceptance: deterministic replay, matching pre-event state and commands, mass conservation and no pose teleport, meaningful path error, actual cargo travel, finite states/no warnings, timestep sensitivity, no private tool leaks, validated candidate edits, frozen unseen evaluation. Render and inspect representative frames and a playable animation. Tests/lint/type checks must pass for changed code.
