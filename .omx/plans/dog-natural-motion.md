# Natural dog gait and a demonstrable controller repair

User correction: replace the abrupt walk-then-fall with natural motion and a
clear correction GPT-6 can infer. This work changes the dog only; preserve the
concurrent drone, car and warehouse work.

1. Smooth the headline dog's foot arcs and body transfer. Keep the original
   physical-fault presets on their legacy gait.
2. Replace the paired-leg switch with continuously evolving cadence error.
   Require multiple measured missteps/recoveries spanning at least two seconds
   before terminal instability; no imposed root motion or scheduled fall.
3. Expose bounded gait-controller JSON through the existing support-torque path.
   The reference correction must use exactly that public edit/load interface.
   Changing world physics, task pace, duration or a defect flag is not a repair.
4. Record the original and developer-authored corrected run on identical task
   fixtures. Show provenance clearly and keep reference values out of the GPT-6
   export. Add observed foot contacts and a reference replay selector.
5. Verify the reference on the main task and at least two independent speed/onset
   variants, deterministic resets and a half timestep. Inspect motion at normal
   and slow speed, retain visual-verdict evidence, run relevant tests/build/lint.

Ownership: dog executor owns gait/controller module/assets/physical tests;
replay executor owns UI; parent owns JSON loader, public export, replay/CLI
integration, validation artifacts and final handoff. No GPT-6 API calls are
part of this revision; a passing developer correction proves physical
solvability, not that GPT-6 has already authored a fix.
