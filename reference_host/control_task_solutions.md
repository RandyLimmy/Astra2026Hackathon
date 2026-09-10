# Reference answers for the four controller tasks

The exact, complete candidates are saved in [control_task_solutions.json](control_task_solutions.json). Each succeeded in a local physics check during implementation. These are developer reference answers, **not Astra or Sol results**.

| Task | Exact correction | Previously verified result |
| --- | --- | --- |
| Robot dog (`quadruped_gait_failure`) | `rear_cadence_gain`: **4.0 → 1.8** | Completes the original 18-second speed schedule upright; 1.960 m forward progress, 14.818° maximum tilt, no body-ground contact. |
| Drone (`drone_delivery_imbalance`) | `route_attitude_gain`: **0.9 → 3.5**; `trim_limit_nm`: **0.95 → 1.4**; enable `trim_during_route`, `reset_trim_on_release`, and `reset_trim_on_support` | Delivers and releases the parcel, returns unloaded and lands in approximately 26.49 seconds, without in-flight body contact. |
| Warehouse trolley (`warehouse_curve_demo`) | `target_speed_m_s`: **1.6 → 0.6**; keep acceleration **0.4 m/s²** and lookahead **0.65 m** | Completes the original route in the 17-second trial with the cargo retained. |
| Automatic braking (`car_auto_brake_failure`) | `brake_trigger_x_m`: **45.0 → 10.0**; keep `brake_command`: **1.0** | In the fixed **22 m/s controller task**, stops at bumper x **91.217 m**, with **8.783 m** wall clearance, in **9.142 s**. |

The dog correction reduces the rear gait clock's reaction to acceleration. The drone correction sustains attitude compensation during travel and resets it when support or payload release changes the loading. The trolley correction reduces turn speed enough to retain cargo without abandoning the route.

The warehouse answer was **reverified after teammate commit `99534df`** raised the deck and changed the parcel geometry. The unchanged 0.6 m/s reference still succeeds: it reaches the route end at **13.824 s**, finishes the full **17 s** trial with cargo aboard and no cargo-floor contact, and ends **0.00643 m** from the endpoint. The new 1.6 m/s baseline spills its cargo at **5.920 s**. The [saved verification](../runs/reference-warehouse-99534df/verification.json) includes exact metrics, source hashes, and paths to complete baseline/reference telemetry; this check used no model API calls.

The drone controller exposed to both models is a public-feedback port of the original nominal controller. It uses the same world and mission and exhibits the outbound crash, but its trajectory is not a bit-identical replay of the stock controller. Both models receive this same baseline.

The car controller task declares a **22 m/s approach for every candidate** after the same four physical conditioning cycles. Its wall remains at **100 m**, its green bumper target remains **86–98 m**, and its deadline remains **12 s**. The late 45 m trigger collides at **12.433 m/s**; applying the same full brake at 10 m stops inside the target and remains below 0.1 m/s for one second. The separate legacy **25 m/s** failure replay is unchanged: a feasibility check found that even full braking from its first step still hits the wall at **6.303 m/s**. The calibrated approach speed is therefore part of the fixed new task, not an editable controller parameter or a change between baseline and solution. [Car verification and saved corrected telemetry](../runs/reference-car-green-zone-20260910/verification.json) record the engineering result and source hashes; no model API calls were used.

## Evaluation and separation

- Keep these answers out of prompts, public capabilities, original preview stories, and model tool responses. The task broker exposes its explicit public interface; it does not offer arbitrary repository-file access.
- The model must apply its own controller changes. Describing a fix without applying it does not count as an applied change.
- Evaluate the frozen model candidate in a fresh execution against the fixed task criteria. A candidate does not have to match this answer key to succeed.
- The reference candidates change only the controller. They preserve the physical world, mission, deadline, and success criteria. Standing still or parking is insufficient.
- The JSON identifies the provenance of each engineering check. Full corrected telemetry is retained for the revised warehouse and car; the earlier dog/drone corrected recordings were not retained. Reference rechecks or renders must be labeled as developer references, never model successes.

The first three-scenario Astra/Sol batch was interrupted when the revised train arrived. These references establish feasible solutions for the current tasks; they do not establish either model's performance. Fresh batch artifacts record the subsequent comparison separately.
