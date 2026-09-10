# Drone: Astra/max and Sol/max

One matched parallel comparison on `drone_rotor_loss`. Both models used 11 API
requests, applied one physical repair and two predictive-source edits, and
submitted their results. All recorded comparison-integrity checks passed.

| Result | Astra | Sol |
| --- | ---: | ---: |
| Physical verification probes passed | 2/2 | 2/2 |
| Frozen prediction agreement | Passed | Passed |
| Elapsed time, including tools and verification | 273.2 s | 198.5 s |
| Final mean position error against healthy control | 0 m | 0 m |

Both replaced `rotor_FL`, restoring its recorded thrust gain from **0.12 to 1.0**.
Both first modeled the reduced thrust, then revised the predictive source after
maintenance. Sol performed the repair; this run did not exhibit prose-only
inactivity. The original drone contacted the ground. Repaired maneuver and hover
probes each completed the full 10 seconds within their declared envelope.

[Recorded comparison, exact changes and protocol hashes](comparison.json)
contains the compact evidence. Full prompts, recordings and checkpoints remain
in ignored `runs/paired-drone-20260910-001*` and are available in the dashboard.
Cost was not recorded; token usage is included.

This run predates integration of the friend’s `4e2359c` changes. The recorded
protocol hashes identify the implementation used. It is one synthetic scenario,
not evidence of a general performance advantage.
