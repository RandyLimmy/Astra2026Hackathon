2. RealityPatch uses Astra to discover what a simulator is missing, add that missing behavior to its code, and check whether it now predicts the system correctly.
Imagine a motor simulator that remembers two things: the motor’s position and speed.
You give it a command, and it predicts how the motor will move.
But the physical motor behaves differently depending on what happened earlier. After resting, it responds quickly. After prolonged use, it responds slowly. Imagine that heating causes this difference.
The simulator has no concept of temperature. It therefore predicts the same movement whenever the starting position, speed, and command are identical—even when the motor’s actual response differs.
Astra’s job is to recognize that the simulator needs additional information.
RealityPatch gives Astra the simulator’s source code, recorded behavior, and tools to run experiments. It then tries to:
Find the contradiction: two situations the simulator treats as identical produce different outcomes.
Investigate possible explanations: temperature, stored energy, or another effect related to operating history.
Choose an experiment: for example, vary the operating and recovery periods to help distinguish those explanations.
Extend the simulator: add a candidate temperature variable, heating and cooling rules, and its effect on movement.
Predict a new situation: test an operating sequence it has not used during the repair.
Temperature remains a hypothesis until the evidence supports it. The system should report uncertainty when several explanations remain possible.
The demo’s payoff is a prediction made before revealing the result. Astra updates the simulator, predicts how the motor will behave after a new sequence of operation and rest, and we compare that prediction with the reference.
For the hackathon, a hidden simulator can supply the reference behavior, clearly labeled as synthetic. Later, the same workflow could use measurements from hardware.
The potential robotics application is a robot that can investigate why its predictions fail and improve its model of its own body. Astra would provide the investigation and model-repair component.
The proposed contribution is the complete connection from evidence of missing information to a tested, executable model extension. That contribution is potentially novel; discovering hidden states and repairing models already have prior work. Sol and Astra would attempt the same workflow to measure whether Astra makes a meaningful improvement.
Compare to gpt 5.6 sol vs Astra on this task. 


I’d use an existing physics engine and deliberately give Astra an incomplete model within it. Synthetic data and an established simulator work together: we control what information is missing while using documented physics for the reference.
MuJoCo is my first choice—and I found something particularly relevant.
Its current documentation includes a DC motor actuator with optional temperature, electrical dynamics, and history-dependent friction. Thermal modeling adds a winding-temperature state; disabling it removes that state. That closely matches the experiment we’ve been discussing. MuJoCo actuator documentation.
Here are the options I would seriously consider:
Option
How we would use it
My assessment
MuJoCo
A motor driving a single joint, with a richer reference model and an incomplete candidate model.
Best fit for a robotics-facing demo.
Small Python model using SciPy
Implement documented motor equations and integrate them numerically.
Fastest way to test the core idea and make every equation inspectable.
Drake
Compose a mechanical system with additional actuator dynamics and compare against a reduced model.
Strong modeling framework; I’d choose it if someone on the team already knows it.

SciPy provides numerical integration through solve_ivp. Drake supports composing custom dynamical systems and includes examples such as a controlled pendulum. Drake tutorial.
The first MuJoCo experiment could be very concrete.
Build a motor driving a pointer or a single arm joint:
Reference model: includes winding temperature and its effect on motor response.
Candidate model: includes the same mechanics but uses a simpler motor model without thermal state.
Agent observations: commands, position, velocity, and timestamps.
Hidden information: reference configuration, internal temperature, and reference source details.
Both Sol and Astra can request operating sequences and observe the response. They can inspect and modify the candidate model.
We then test whether each can recognize that prior operating history matters, obtain supporting evidence, and produce a model that predicts new histories.
For a later physical interpretation check, we could reveal a separate temperature measurement. Predicting motion alone would not prove that an inferred variable truly represents temperature.
There is one important implication for our novelty claim.
Because MuJoCo already supports thermal state, simply having Astra enable the thermal option would demonstrate model selection and configuration repair. We should describe that accurately.
If we want to demonstrate source-code extension, use a small editable Python actuator model coupled to MuJoCo’s mechanics. Astra would add the state and its evolution to that component. We would be repairing the actuator model—not rewriting MuJoCo’s engine.
Either approach can be useful. The second better matches our proposed code-extension contribution, but requiring extra code does not itself make the research novel.
Yes, deliberately omitting information is a reasonable starting experiment.
The omission should be a physically motivated simplification:
Treating temperature as constant.
Assuming motor current responds instantaneously.
Approximating history-dependent friction with a fixed friction law.
MuJoCo’s technical note describes these mechanisms and their equations, giving us a documented basis for constructing the experiments. DC motor technical note.
To keep the result meaningful:
Keep reference internals outside the agent’s accessible environment.
Include a case where the simpler model is already adequate.
Test genuinely new operating histories.
Check that numerical integration error is not creating the apparent mismatch.
Report the experiment as recovery of deliberately omitted dynamics, rather than discovery of unknown physical laws.
MuJoCo also has a system-identification toolbox for fitting parameters to recorded data. That gives us a useful conventional comparison: what can parameter fitting accomplish before a model extension is needed? Its repository documentation describes the fitting workflow; we’d check compatibility with our installed version before relying on it. MuJoCo system identification.
My build recommendation: start with a small Python motor model using documented equations, then connect the working investigation loop to a one-joint MuJoCo scene. That lets us establish whether the idea works before spending time on robot assets and rendering. The reference can remain synthetic for the hackathon, with hardware measurements as the next validation step
