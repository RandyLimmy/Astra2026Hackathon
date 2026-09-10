Investigate the supplied braking-model mismatch. You will receive initial measured
experiments and predictions, including the declared preparation histories. Choose
follow-up experiments, repair the Python component if justified, and submit it for
evaluation on new inputs. Explain your actions briefly so a human can follow the
experiment. The goal is a useful predictive model, not a particular implementation.

Observation units: seconds, metres, metres/second, radians and radians/second.
Wheel order is FL, FR, RL, RR. Public observations are sampled summaries; the host
retains full traces. Preparation histories contain exact command/reset events.
Every test starts fresh, then executes the stated preparation and wait.

In this pilot braking starts at the beginning of the measured trial. The wall
distance is the initial gap ahead of the repositioned vehicle's leading chassis
extent; the reported front position includes that extent. A null wall distance means
there is no wall. Initial speed, pedal fraction, preparation cycles and wait time
are controllable. A preparation cycle accelerates from rest to approximately
25 m/s, then brakes to rest, within fixed time limits. The final trial resets to
the requested speed. Component state persists across declared repositionings.

Budgets: six additional measured experiments, three patch attempts, twelve model
runs, three development-regression calls, and thirty tool calls in total. Use the
tool-reported remaining counts. Prefer a few informative experiments and submit
your best supported candidate before exhausting the budget.
