"""Wheel-component coupling and observable preparation replay regressions."""

from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from simulator.config import Experiment
from simulator.runner import BRAKE_TORQUES, Simulator


class TrackingActuator:
    """Trusted test double for interface call counts and state lifecycle."""

    def __init__(self, scale=1.0):
        self.scale = scale
        self.resets = self.repositions = self.advances = 0
        self.work = 0.0
        self.last_interval = None

    def reset(self):
        self.resets += 1
        self.work = 0.0

    def reposition(self):
        self.repositions += 1

    def torque_limits(self, brake, omega):
        assert len(omega) == 4
        return (BRAKE_TORQUES * brake * self.scale).tolist()

    def advance(self, brake, mean_omega, applied_torque, dt):
        self.advances += 1
        self.last_interval = (brake, mean_omega, applied_torque, dt)
        self.work += float(np.maximum(-np.asarray(mean_omega) * applied_torque, 0).sum()) * dt


def test_candidate_controls_capacities_without_private_lag_or_fade():
    actuator = TrackingActuator()
    sim = Simulator(Experiment(initial_speed=10, wall=False, thermal=True, lag=1,
                               weak_wheel="FL", brake_efficiency=0.2), actuator=actuator)
    sim.temperature[:] = 500
    sim.step(0, 1)
    np.testing.assert_array_equal(sim.model.dof_frictionloss[sim.spin], BRAKE_TORQUES)
    np.testing.assert_array_equal(sim.temperature, np.full(4, 500.0))
    assert sim.activation == 0
    assert actuator.advances == 1
    brake, omega, torque, dt = actuator.last_interval
    assert brake == 1 and dt == sim.config.timestep
    assert len(omega) == len(torque) == 4
    assert np.all(np.asarray(torque) * omega <= 0)
    assert actuator.work > 0


def test_reposition_preserves_component_memory_but_full_reset_clears_it():
    actuator = TrackingActuator()
    sim = Simulator(Experiment(initial_speed=10, wall=False), actuator=actuator)
    assert (actuator.resets, actuator.repositions) == (1, 1)
    sim.step(0, 1)
    work = actuator.work
    sim.reset_trial(7)
    assert actuator.work == work > 0
    assert (actuator.resets, actuator.repositions) == (1, 2)
    sim.reset_full()
    assert actuator.work == 0
    assert (actuator.resets, actuator.repositions) == (2, 3)


def test_isolated_normal_component_matches_nominal_reference():
    from component_worker import WheelActuatorWorker

    config = Experiment(initial_speed=10, brake_at=0, wall=False, duration=5)
    reference = Simulator(config)
    expected = reference.run()
    source = Path(__file__).resolve().parents[1] / "candidate" / "wheel_actuator.py"
    with WheelActuatorWorker(source) as worker:
        candidate = Simulator(config, actuator=worker)
        result = candidate.run(preparation_history=reference.preparation_history)
        assert worker.inspect_state() == {}
    assert result["public"] == expected["public"]
    np.testing.assert_allclose(candidate.data.qpos, reference.data.qpos, atol=1e-12)
    assert "initial_temperature" not in result and "final_temperature" not in result
    assert "temperature" not in candidate.diagnostics()
    assert "brake_activation" not in candidate.diagnostics()
    assert "initial_temperature" in expected and "final_temperature" in expected
    assert "temperature" in reference.diagnostics()


def test_replay_uses_exact_preparation_commands_not_candidate_speed_thresholds():
    config = Experiment(initial_speed=10, wall=False, warmup_cycles=1, recovery=0.02)
    reference = Simulator(config)
    reference.prepare()
    history = reference.preparation_history
    expected_steps = sum(event.get("steps", 0) for event in history)
    assert history[-1] == {"kind": "reset", "speed_mps": 10.0, "phase": "trial"}
    assert {event["phase"] for event in history} == {"conditioning_1", "recovery", "trial"}
    assert all(set(event) <= {"kind", "phase", "speed_mps", "throttle", "brake", "dt_s", "steps"}
               for event in history)

    actuator = TrackingActuator(scale=0.0)
    candidate = Simulator(replace(config, warmup_cycles=0, recovery=0), actuator=actuator)
    candidate.prepare(history=history)
    assert candidate.preparation_history == history
    assert actuator.advances == expected_steps
    assert candidate.elapsed == pytest.approx(expected_steps * config.timestep)
    assert candidate.speed == pytest.approx(reference.speed)
    assert candidate.trial_time == 0
    assert actuator.work == 0  # Own zero torque, never the reference's dissipated work.


def test_prepared_run_does_not_repeat_conditioning_or_reset_component():
    actuator = TrackingActuator()
    sim = Simulator(Experiment(initial_speed=0, wall=False, duration=0.1), actuator=actuator)
    sim.prepare()
    resets = actuator.repositions
    sim.run(prepared=True)
    assert actuator.repositions == resets
    with pytest.raises(ValueError, match="cannot be combined"):
        sim.run(prepared=True, preparation_history=sim.preparation_history)


@pytest.mark.parametrize("fault,event", [
    ({"detach_wheel": "FR", "detach_at": 1}, "wheel_release"),
    ({"weak_wheel": "FL", "weak_at": 1, "brake_efficiency": 0.2}, "brake_efficiency"),
])
def test_stationary_trial_runs_pending_fault_and_observes_its_aftermath(fault, event):
    sim = Simulator(Experiment(initial_speed=0, wall=False, duration=2, **fault))
    result = sim.run()
    assert result["trial_duration"] >= 1.5 - 1e-9
    assert [entry["event"] for entry in result["events"]] == [event]
    assert result["events"][0]["trial_time"] == pytest.approx(1.0)


def test_replay_rejects_private_fields_before_changing_state():
    sim = Simulator(Experiment(initial_speed=10, wall=False))
    initial = sim.data.qpos.copy()
    with pytest.raises(ValueError, match="Unexpected preparation reset fields"):
        sim.prepare(history=[{"kind": "reset", "speed_mps": 10, "phase": "trial", "temperature": 400}])
    np.testing.assert_array_equal(sim.data.qpos, initial)
    assert sim.elapsed == 0
