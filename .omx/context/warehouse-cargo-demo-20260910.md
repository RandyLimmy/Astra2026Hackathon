# Warehouse cargo failure demo

Task: Build the trolley cargo-shift animation and failure scenarios so GPT-6 can infer and repair its predictive model.

Outcome: Physical, reproducible cargo failures; a clear replay; neutral measured evidence and an editable nominal model with held-out evaluation. A failure animation must never imply that GPT-6 has already repaired the model.

Evidence: simulator/platforms/warehouse.py has a physical inclined slide and timed latch release. warehouse_demo is a 9 s matched-command probe. simulator.lab exports nominal MJCF and public recordings but has no candidate repair path. investigation currently isolates editable Python brake components and is specific to the original car. Native viewers already support Space/replay and cameras.

Constraints: No dependencies; preserve unrelated dirty drone/quadruped files and existing presets; never fake trajectories or silently provide a developer patch as model output. Hide reference faults and private diagnostics from the investigation interface. Use existing worker isolation if evaluating generated Python. Do not change upstream MuJoCo. API usage is not necessary to stage the failure scenarios.

Open choices: Smallest safe trolley model patch interface; best calibrated motion and camera for visible deviation; scope of reusable existing API runner.

Touchpoints: simulator/platforms/warehouse.py, simulator/assets/warehouse.xml, simulator/platforms/operator.py, simulator/view_controls.py, simulator/lab.py, tests/test_warehouse.py, a dedicated warehouse repair/export module and candidate source.
