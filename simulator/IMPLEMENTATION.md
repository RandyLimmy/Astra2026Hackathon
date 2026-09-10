# Implementation decisions

The initial plan is retained as a design record. The following changes came from
running the actual MuJoCo mechanics:

- The first cylinder tires and `implicitfast` integration produced unstable
  rotation after high-speed carrier release. Rounded ellipsoid tire contacts
  and full `implicit` integration handle the coupled gyroscopic terms. The
  17 m/s, 3.1 s release preset passes the paired timestep check; arbitrary
  release configurations are not promised to converge to the same trajectory.
- An explicit signed motor torque chattered around zero wheel speed and heated
  stationary brakes numerically. Disc brakes instead use MuJoCo's bounded DOF
  friction constraints. They hold at rest without driving wheels backwards.
  Heat uses the solved brake torque and midpoint angular velocity.
- The four synthetic brake temperatures integrate dissipated energy and
  exponential cooling. This models brake-disc fade; it does not repurpose the
  engine's electrical-winding thermal feature. A 650 J/K per-brake heat capacity
  and 90 s cooling constant are synthetic calibration choices.
- No extra impulse is applied when a weld releases. A carrier is a world-level
  free body attached to the chassis by a weld; its wheel rotates on a child hinge.
- The fixed-steering vehicle develops only modest yaw with one weakened brake.
  Its verified effect is longer braking with all wheels still attached. The
  original >3 degree yaw target was replaced by a measured stopping-distance
  increase and direct capacity checks, and is not reported as passed.
- The wheel-loss preset uses a 70 m wall instead of the general 100 m wall.
  Healthy controls must use the same 17 m/s starting speed and wall position.
- The road patch has higher contact priority than tire geoms and does not overlap
  a second dry ground collider. Payload is added before compilation so MuJoCo
  computes the added body's inertia consistently.
- First-order actuator lag is integrated analytically in the brake controller.
  It is not a pure command transport delay.
- Logs and frames use a monotonic experiment clock across repositioning resets.
  Conditioning steps remain observable. Hidden event/state diagnostics are
  exported separately; actual OS/process isolation belongs to deployment.
- `run` and `view` share a terminal policy. Future scheduled commands execute even
  if the vehicle is currently stationary. World yaw rate accounts for chassis
  pitch and roll and is null when the projected heading is undefined.

Evidence: `runs/validation/report.json`, focused tests under `tests/`, and the
rendered wheel-loss replay under `runs/wheel-loss-demo/`. Numeric outcomes in the
validation report are measurements, not hard-coded scenario results.
