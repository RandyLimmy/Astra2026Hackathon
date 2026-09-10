# Post-crash car inspection

This platform owns a separate steered, suspended, four-wheel MuJoCo car. Motor
torques and joint-friction brakes act on physical wheel joints; all chassis
translation and yaw result from contact forces. It does not alter the original
brake-fade car.

The initial 6.5 m/s rolling fixture release reaches the barrier at about 1.036 s.
An armed car–barrier contact above 500 N activates prescribed retained damage.
Time alone, missing the barrier, and resetting into the inspection lane cannot
activate damage. The impact is real rigid-body contact; structural fracture is
represented by a parameter change gated by that contact, not material simulation.

After a 0.6 s recovery hold, an explicitly logged `recovery_reposition` moves the
car to the open inspection lane at y = -12 m. A 0.5 s unobserved fixture settling
preparation then releases it at 5.5 m/s. Reset preparation preserves the experiment
clock and every damaged model parameter. Only `reset_full()` repairs the vehicle
and replays the original Config. The automatic probe uses the same commanded
steering schedule and speed regulator for healthy and damaged cars.
An explicit `reset_trial()` starts a fresh `duration`-second probe even after a
run finishes; automatic post-impact recovery keeps the original run horizon.

| Preset | Probe | Persistent physical change |
| --- | --- | --- |
| car_postcrash_healthy | steering | Same impact and recovery, no damage |
| car_steering_damage | steering | Rack gain 0.35; center bias 0.12 rad |
| car_wheel_misalignment | slalom | Left-front wheel mount toe +0.28 rad |
| car_suspension_damage | one-wheel bump | Left-front spring 40,000 → 8,000 N/m; damping scaled by √0.2 |
| car_tire_pressure | braking | Left-front rolling radius 0.34 → 0.272 m; friction 1.1 → 0.4; contact time 0.008 → 0.045 s |

Tire pressure is a **synthetic radius/contact-compliance approximation**, not a
pneumatic or finite-element tire model. Wheel mass/inertia remain unchanged;
this isolates radius and traction/contact effects. The springs are vertical
linear suspensions, with no deformable chassis or full steering linkage.

`step({"steering": 0.1, "throttle": 0.2, "brake": 0.0})` permits explicit
experiments. Steering is radians in [-0.5, 0.5]; throttle/brake are [0, 1].
Available Config probes are `steering`, `braking`, `slalom`, and `bump`. Any
damage can use any probe, and `fault="healthy"` supplies its matched reference.

Public observations expose body pose/velocity/IMU, issued commands, steering
encoders, suspension displacement, wheel speed/position/axle direction. Damage
parameters and the contact activation gate appear only in private diagnostics.

Default 12 s measurements at a 0.002 s timestep:

| Probe metric | Matched healthy | Damaged |
| --- | ---: | ---: |
| Steering max lateral displacement | 1.81 m | 26.33 m |
| Misaligned-wheel slalom max lateral displacement | 2.70 m | 12.85 m |
| Suspension peak travel on bump | 0.139 m | 0.243 m |
| Tire-pressure braking distance | 2.38 m | 2.69 m |

The first three damaged presets exceed the synthetic diagnostic envelope. The
pressure case remains inside it while braking roughly 13% farther. Safe means
only: completed probe, final speed under 0.5 m/s, lateral displacement under
4.5 m, absolute roll under 0.35 rad, and suspension travel under 0.20 m. It is
neither roadworthiness nor an assertion that all other maneuvers are safe.
