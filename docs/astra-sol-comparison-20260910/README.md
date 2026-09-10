# Astra/medium and Sol/high: one matched experiment

In this pair, **`gpt-6-astra` with `medium` reasoning** produced a working stateful
Python component and passed the predeclared prediction criteria on all three
reserved cases. **`gpt-5.6-sol` with `high` reasoning** proposed a stateful repair,
but all three patch attempts were rejected; its submitted component remained
identical to the original. Mean absolute stopping-distance error was **0.405 m
for Astra and 10.292 m for Sol**, versus **10.292 m for the original component**.

Both runs started in fresh model contexts, with the same initial evidence,
editable component, tools, mechanics, evaluation rules and budgets. Their
identical recorded protocol fingerprint is:

```text
b7d205a5aff97bf7cf0d7832d43aff52939a667ead3537671109db9a113335d6
```

Each used 12 API requests and explicitly froze its submission before reserved
outcomes were revealed. Astra used all 12 requests during investigation; Sol
used 11 during investigation and one for a tool-free debrief after evaluation.
The [complete comparison](comparison.md), [machine-readable results](comparison.json)
and [protocol manifests](protocols.json) preserve the recorded evidence.

## What the predictions show

All three cases start at **22 m/s**, use full braking and have no wall.
Distances below are measured from braking onset, in metres.

| Reserved case | Preparation / wait | Reference | Original / Sol | Astra |
|---|---|---:|---:|---:|
| 1: cold control | 0 cycles / 0 s | 36.875 | 36.875 | 36.875 |
| 2: repeated braking | 2 cycles / 0 s | 62.403 | 36.875 | 62.526 |
| 3: braking after rest | 2 cycles / 45 s | 42.223 | 36.875 | 43.314 |

Astra retained the cold prediction, then captured both the longer stop after
repeated braking and the shorter stop after resting. Its errors in cases 2 and
3 were 0.123 m and 1.091 m. It passed the aggregate improvement, per-case
tolerance, cold-control and collision checks. Sol retained the cold prediction
in every case and failed the aggregate improvement and per-case criteria.
All three cases were eligible, unseen by each investigator before freezing,
and produced uncensored stops.

## What changed in the code

Astra's [accepted component](run_a/actuator.py) adds four persistent `heat`
values. Each accumulates its own wheel's dissipated braking work, decays with a
90-second timescale, and reduces that wheel's braking capacity above a threshold.
The state survives repositioning between trials. This is an inferred heat/work
proxy, not an observed temperature or proof that the hidden mechanism was uniquely
identified. The extension changes the editable actuator and its state evolution;
the mechanics engine remains unchanged.

Both models encountered patch-format errors. Astra's first two attempts had
incorrect unified-diff hunk counts; its third was accepted. Sol first supplied
an unsupported patch wrapper, then twice supplied mismatched hunk counts.
Its [frozen component](run_b/actuator.py) is therefore unchanged. Sol's proposed
work-dependent weakening and recovery mechanism never ran: this result records
a failure to deliver an executable patch under this interface and budget, not
a test disproving that physics hypothesis.

## Limits of this comparison

This is **one run per model/effort profile**, not a general model ranking.
The builder had seen these reserved cases in the earlier pilot; fresh model
contexts do not make this a builder-blind benchmark. Session durations were
1,193 s for Astra and 502 s for Sol, including simulation, evaluation and host
load. They are not a model latency benchmark. The earlier
[Astra pilot](../astra-pilot-20260910/README.md) used an earlier protocol and is
kept as a separate record.
