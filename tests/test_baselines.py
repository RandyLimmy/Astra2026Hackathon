import numpy as np
import pytest

from evaluation.baselines import fit_fixed_force, make_fixed_force_source


def constant_force_run(force, speed=20.0, brake=1.0, duration=2.5):
    times = np.linspace(0.0, duration, 101)
    return {
        "config": {"speed_mps": speed, "brake_strength": brake},
        "probe": [
            {"t_s": float(t), "x_m": 0.0,
             "v_mps": float(max(0.0, speed - brake * force / 1200.0 * t))}
            for t in times
        ],
    }


def test_recovers_one_known_force_across_commands_and_stopped_samples():
    result = fit_fixed_force([
        constant_force_run(6250.0, speed=8.0, brake=0.25, duration=8.0),
        constant_force_run(6250.0, speed=20.0, brake=1.0, duration=5.0),
    ])
    assert result["optimizer_success"]
    assert result["fitting_split"] == "development"
    assert result["force_n"] == pytest.approx(6250.0, abs=0.001)
    assert result["speed_rmse_mps"] < 1e-6


def test_one_force_cannot_fit_conflicting_histories_with_identical_commands():
    result = fit_fixed_force([
        constant_force_run(9000.0),
        constant_force_run(3150.0),
    ])
    assert result["optimizer_success"]
    assert result["force_n"] == pytest.approx((9000.0 + 3150.0) / 2, abs=0.001)
    assert result["speed_rmse_mps"] > 3.0


def test_sparse_moving_samples_do_not_get_stuck_on_stopped_prediction():
    run = constant_force_run(3000.0, duration=5.0)
    run["probe"] = [run["probe"][0], run["probe"][-1]]
    result = fit_fixed_force([run])
    assert result["force_n"] == pytest.approx(3000.0, abs=0.001)


def test_emitted_baseline_has_empty_state_and_fitted_force():
    namespace = {}
    exec(make_fixed_force_source(6250.0), namespace)
    state = namespace["init_state"]()
    assert state == {}
    assert namespace["compute_force"](state, 0.4, 20.0) == pytest.approx(2500.0)
    assert namespace["advance_state"](state, 1.0, 20.0, 6250.0, 10.0) == {}


@pytest.mark.parametrize("runs", [[], [constant_force_run(9000.0, brake=0.0)]])
def test_uninformative_fitting_data_is_rejected(runs):
    with pytest.raises(ValueError):
        fit_fixed_force(runs)
