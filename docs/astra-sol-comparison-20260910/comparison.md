# Recorded investigator comparison

One recorded run per model profile. This is a small synthetic experiment, not a model ranking or a general performance claim.

Both investigators were started in fresh API contexts. The builder had seen the reserved cases in an earlier pilot; this is not a builder-blind benchmark.

**Comparison validation:** matched recorded protocol and cases.

**Declared shared protocol fingerprint:** `b7d205a5aff97bf7cf0d7832d43aff52939a667ead3537671109db9a113335d6`.

## Recorded outcomes and work

MAEs below are each run’s recorded eligible-case aggregates. Missing or censored values remain missing; no missing value is treated as zero.

| Measure | gpt-6-astra / medium | gpt-5.6-sol / high |
| --- | ---: | ---: |
| Original stopping MAE (m) | 10.292 | 10.292 |
| Frozen candidate stopping MAE (m) | 0.405 | 10.292 |
| Scored / eligible cases | 3 / 3 | 3 / 3 |
| Failed stop predictions | 0 | 0 |
| Predeclared prediction criteria | passed | failed |
| API requests | 12 | 12 |
| Tool calls | 12 | 11 |
| Extra reference attempts | 3 | 4 |
| Patch attempts | 3 | 3 |
| Model attempts | 1 | 0 |
| Regression calls | 1 | 1 |
| Session duration, including evaluation (s) | 1193.209 | 502.253 |
| Total tokens | 668816 | 628680 |
| Reasoning tokens | 1466 | 7416 |
| Explicit agent submission | yes | yes |
| Freeze reason | agent_submission | agent_submission |
| Changed source | True | False |
| Changed state initialization | True | False |
| Changed state evolution | True | False |

Attempts include rejected requests. Source indicators are syntactic/runtime evidence, not proof of novelty or a uniquely correct physical explanation.

## Case records

Rows retain each run’s own case configuration and prior-observation status; validation flags above identify any mismatch.

| Run | Case | Speed (m/s) | Prep | Wait (s) | Brake | Wall (m) | Original stop (m) | Candidate stop (m) | Reference stop (m) | Original error (m) | Candidate error (m) | Previously observed |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| astra-medium | reserved_1 | 22.000 | 0 | 0.000 | 1.000 | none | 36.875 | 36.875 | 36.875 | 0.000 | 0.000 | no |
| astra-medium | reserved_2 | 22.000 | 2 | 0.000 | 1.000 | none | 36.875 | 62.526 | 62.403 | 25.527 | 0.123 | no |
| astra-medium | reserved_3 | 22.000 | 2 | 45.000 | 1.000 | none | 36.875 | 43.314 | 42.223 | 5.348 | 1.091 | no |
| sol-high | reserved_1 | 22.000 | 0 | 0.000 | 1.000 | none | 36.875 | 36.875 | 36.875 | 0.000 | 0.000 | no |
| sol-high | reserved_2 | 22.000 | 2 | 0.000 | 1.000 | none | 36.875 | 36.875 | 62.403 | 25.527 | 25.527 | no |
| sol-high | reserved_3 | 22.000 | 2 | 45.000 | 1.000 | none | 36.875 | 36.875 | 42.223 | 5.348 | 5.348 | no |

## Recorded scoring rules

| Criterion | Run A | Run B |
| --- | --- | --- |
| cold_control_absolute_floor_m | 1.0 | 1.0 |
| cold_control_relative_tolerance | 0.05 | 0.05 |
| maximum_candidate_to_original_mae_ratio | 0.5 | 0.5 |
| minimum_original_mae_for_ratio_m | 1e-06 | 1e-06 |
| per_case_absolute_floor_m | 1.0 | 1.0 |
| per_case_relative_tolerance | 0.1 | 0.1 |
| require_all_reserved_cases_unseen | true | true |
| require_matching_collision_outcomes | true | true |
| require_uncensored_stops | true | true |

| Prediction check | Run A | Run B |
| --- | --- | --- |
| scoring_complete | passed | passed |
| improvement_pass | passed | failed |
| per_case_pass | passed | failed |
| cold_control_pass | passed | passed |
| collision_outcomes_match | passed | passed |

## Exact frozen components

These are byte-for-byte copies of the verified evaluated components. Passing interface validation or freezing a component does not imply it passed prediction criteria.

- [astra-medium: frozen component](run_a/actuator.py) · SHA-256 `b88fc154047af1bc3d630258b9862f83d9683e8170a49eac479d5e42ced2bb4d`.
- [astra-medium: original component](run_a/original_actuator.py) · SHA-256 `7451a16e73d0652bff21620fdbea938b304bc64ce139577da4af67cc3467b03c`.
- [sol-high: frozen component](run_b/actuator.py) · SHA-256 `7451a16e73d0652bff21620fdbea938b304bc64ce139577da4af67cc3467b03c`.
- [sol-high: original component](run_b/original_actuator.py) · SHA-256 `7451a16e73d0652bff21620fdbea938b304bc64ce139577da4af67cc3467b03c`.

[Machine-readable comparison](comparison.json) · [Recorded protocol manifests](protocols.json)

This export only reads saved records. It makes no API requests, changes no source run, and does not generate replacement evaluation cases.
