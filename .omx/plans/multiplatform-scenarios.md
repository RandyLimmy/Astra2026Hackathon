# Multi-platform simulator extension

User authorized implementation after the original car scenario suite. Preserve
that suite and add three independent simulator platforms to the existing CLI.
Scope: physics, nominal probe controllers, reproducible faults, telemetry/video,
healthy comparisons, and scenario validation. Agent inference, model patching and
real-world safety certification are not supplied by these scenarios.

1. Post-crash car: physical impact gates retained steering bias, wheel toe error,
   suspension softening or tire contact degradation. Reposition explicitly for
   steering/braking probes and measure resulting path/heading/braking changes.
2. Quadruped: stable visible locomotion from an articulated physical dog, followed
   by actuator weakness, foot slip, leg damage or shifted payload. Evaluate the
   same nominal gait/controller before and after the change and on separate probes.
3. Drone: physical rotor forces and bounded nominal flight control, followed by
   thrust loss, voltage sag, payload change, wind or lag. Report measured tracking,
   attitude, altitude and contacts on hover/flight probes.

Each platform must expose the platform contract, reject invalid inputs, keep
hidden diagnostics outside public observations, preserve damage on trial reset,
replay healthy/fault cases deterministically, and run without MuJoCo warnings.
Use tests of physical effect, not tests that simply mirror constants. Validate
recordings with rendered frames and compare actual output to healthy runs.

Parent integrates CLI run/view/list/compare/export and validation. Independent
implementation ownership: car_damage module/assets/tests; quadruped module/assets/tests;
drone module/assets/tests. No changes to existing car physics are required.
