# Nominal simulator baseline

This is a self-contained synthetic car-and-track MJCF, plus the public observation schema. It is not a validated vehicle safety model. No private profiles or failure rules are included.

Coordinates and units: x is forward, y is left, z is up; SI units; wheel order FL, FR, RL, RR. Named cameras are overview, side, chase. Default timestep is 0.002 s.

The nominal controller applies 500 * throttle N m to each wheel motor. Disc braking uses the spin_FL/spin_FR joint frictionloss capacities 835 * brake N m and spin_RL/spin_RR capacities 557 * brake N m. MuJoCo's bounded friction constraint opposes rotation and holds at rest. The baseline scene alone does not execute pedal schedules. Settle contacts before motion and initialize wheel spin to vehicle speed / 0.34 m.

Public observations are JSONL sampled at 100 Hz. Time is monotonic experiment time; phase_time resets on declared repositioning. Conditioning and recovery are observable history. PNG frames are indexed by time and relative file in frames.jsonl, normally at 30 Hz. Frame times are actual physics times, within one physics step of the sampling grid.

Public summary fields describe collision/impact speed, stop status, censoring, braking distance, final motion, wall clearance, yaw/lateral displacement, and lane departure. A collision or timeout has null stopping_distance; a stop requires speed below 0.1 m/s for 0.5 s. Wall-free paired replays are evaluator-only artifacts.

A public directory is an export convention, not a security boundary. The integration must isolate the hidden simulator and private records using account/process/container permissions before exposing observations to an agent.
