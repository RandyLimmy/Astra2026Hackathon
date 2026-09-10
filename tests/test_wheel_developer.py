"""Builder-only checks; private calibration knowledge is explicit here."""

import ast
import copy
import math
from pathlib import Path

import numpy as np
import pytest

from reference_host import developer_wheel_actuator as developer
from simulator.private import thermal


def test_cold_caps_hold_at_rest_and_respect_pedal_without_mutating_state():
    state = developer.init_state()
    before = copy.deepcopy(state)
    assert developer.compute_brake_torque_limits(state, 1, [0] * 4) == [835, 835, 557, 557]
    assert developer.compute_brake_torque_limits(state, 0.4, [40, -40, 0, 5]) == pytest.approx(
        np.array([835, 835, 557, 557]) * 0.4
    )
    assert developer.compute_brake_torque_limits(state, 0, [40] * 4) == [0] * 4
    assert state == before


def test_dissipation_uses_actual_signed_torque_and_keeps_wheels_independent():
    state = developer.init_state()
    heated = developer.advance_state(state, 1, [50, -50, 0, 50], [-500, 500, -500, 500], 10)
    assert heated["recent_work_j"][0] == pytest.approx(heated["recent_work_j"][1])
    assert heated["recent_work_j"][0] > 200000
    assert heated["recent_work_j"][2:] == [0, 0]
    caps = developer.compute_brake_torque_limits(heated, 1, [50] * 4)
    assert caps[0] < 835
    assert caps[1] < 835
    assert caps[2:] == [557, 557]
    assert state == developer.init_state()


def test_reset_retains_memory_and_180_second_rest_restores_cold_capacity():
    heated = {"recent_work_j": [350000, 350000, 270000, 270000]}
    reset = developer.on_trial_reset(heated)
    assert reset == heated
    assert reset is not heated
    assert reset["recent_work_j"] is not heated["recent_work_j"]
    hot_caps = developer.compute_brake_torque_limits(reset, 1, [0] * 4)
    assert hot_caps == pytest.approx(np.array([835, 835, 557, 557]) * 0.45)
    rested = developer.advance_state(reset, 0, [0] * 4, [0] * 4, 180)
    assert rested["recent_work_j"] == pytest.approx(np.array(heated["recent_work_j"]) * math.exp(-180 / 92))
    assert developer.compute_brake_torque_limits(rested, 1, [0] * 4) == [835, 835, 557, 557]


def test_constant_power_is_independent_of_interval_partition():
    state = developer.init_state()
    once = developer.advance_state(state, 1, [60] * 4, [-500] * 4, 8)
    many = state
    for _ in range(400):
        many = developer.advance_state(many, 1, [60] * 4, [-500] * 4, 0.02)
    assert once["recent_work_j"] == pytest.approx(many["recent_work_j"], rel=1e-12)
    unchanged = developer.advance_state(once, 1, [60] * 4, [-500] * 4, 0)
    assert unchanged == once


def test_developer_curve_approximates_private_calibration_but_is_explicitly_assisted():
    # This is a builder solvability comparison, not blind or held-out evaluation.
    temperatures = np.linspace(20, 650, 251)
    work = (temperatures - thermal.AMBIENT) * thermal.HEAT_CAPACITY
    gains = []
    for value in work:
        state = {"recent_work_j": [float(value)] * 4}
        gains.append(developer.compute_brake_torque_limits(state, 1, [0] * 4)[0] / 835)
    assert np.max(np.abs(np.asarray(gains) - thermal.fade(temperatures))) < 0.021


def test_four_repeated_work_cycles_and_recovery_track_private_calibration():
    state = developer.init_state()
    temperature = np.full(4, thermal.AMBIENT)
    # Controlled dissipated-work fixture independent of any simulator adapter.
    # The direct private comparison is intentionally confined to builder tests.
    observed_max_gain_error = 0.0
    for phase in range(5):
        duration = 4 if phase < 4 else 180
        for _ in range(round(duration / 0.02)):
            powers = np.array([30000, 30000, 20000, 20000]) if phase < 4 else np.zeros(4)
            speeds = np.full(4, 50.0) if phase < 4 else np.zeros(4)
            torques = -powers / 50.0
            state = developer.advance_state(state, int(phase < 4), speeds, torques, 0.02)
            temperature = thermal.advance(temperature, powers, 0.02)
            actual_gain = np.asarray(developer.compute_brake_torque_limits(state, 1, [0] * 4)) / [835, 835, 557, 557]
            observed_max_gain_error = max(observed_max_gain_error, float(np.max(abs(actual_gain - thermal.fade(temperature)))))
        state = developer.on_trial_reset(state)
        if phase == 3:
            assert max(actual_gain) < 0.5
    assert observed_max_gain_error < 0.035
    assert np.all(actual_gain == 1)


@pytest.mark.parametrize("values", [[0] * 3, [0, 0, 0, math.nan], [0, 0, 0, math.inf]])
def test_invalid_wheel_vectors_are_rejected(values):
    with pytest.raises(ValueError):
        developer.compute_brake_torque_limits(developer.init_state(), 1, values)


@pytest.mark.parametrize("brake,dt", [(math.nan, 0.1), (1.1, 0.1), (-0.1, 0.1), (1, -0.1), (1, math.inf)])
def test_invalid_commands_and_intervals_are_rejected(brake, dt):
    with pytest.raises(ValueError):
        developer.advance_state(developer.init_state(), brake, [50] * 4, [-500] * 4, dt)


def test_component_source_has_only_stdlib_math_import_and_no_private_access():
    source = Path(developer.__file__).read_text()
    tree = ast.parse(source)
    imported = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.extend(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            imported.append(node.module)
    assert imported == ["math"]
    assert "DEVELOPER-WRITTEN" in ast.get_docstring(tree)
    assert "calibrated with knowledge" in ast.get_docstring(tree)
