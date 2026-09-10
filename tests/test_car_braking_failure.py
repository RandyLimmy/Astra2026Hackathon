"""Physical F4 failure, neutral history, controls and numerical calibration."""

from dataclasses import replace
import json

import mujoco
import numpy as np
import pytest

from simulator.platforms.car_braking import Config, Simulation
from simulator.runner import Simulator


def finish(sim):
    while not sim.finished:
        sim.step()
    return sim


@pytest.fixture(scope="module")
def hot():
    return finish(Simulation(Config()))


@pytest.fixture(scope="module")
def cold():
    return finish(Simulation(Config(fault="healthy")))


@pytest.fixture(scope="module")
def wall_free():
    return finish(Simulation(Config(probe="wall_free")))


def test_conditioned_car_hits_barrier_and_retains_physical_aftermath(hot):
    result = hot.summary()["public"]
    assert result["outcome"] == "collision"
    assert result["collision"] and result["impact_speed"] > 10
    assert result["collision_time"] == pytest.approx(4.332, abs=1e-8)
    assert result["impact_speed"] == pytest.approx(17.177116922864432, abs=1e-8)
    assert result["aftermath_duration"] == pytest.approx(2.0)
    assert result["brake_start_x"] > 40
    assert result["censored"] and result["stopping_distance"] is None
    assert not result["safe"] and not result["stopped"]
    assert not result["goal_reached"] and not result["task_complete"]
    assert result["repair_status"] == "not_run"
    assert hot.presentation()["status"] == "FAILED - BARRIER COLLISION"
    assert "17.2 m/s" in hot.presentation()["detail"]
    assert "4.33 s" in hot.presentation()["detail"]
    assert hot._plant.actuator is None and isinstance(hot._plant, Simulator)
    assert np.all(hot._plant.data.eq_active[hot._plant.attach])
    np.testing.assert_array_equal(hot._plant.efficiency, np.ones(4))
    assert not any(hot.data.warning.number)
    events = hot.public_events
    onset = next(event for event in events if event["event"] == "brake_onset")
    contact = next(event for event in events if event["event"] == "barrier_contact")
    assert onset["time"] < contact["time"] < events[-1]["time"]
    assert contact["impact_speed"] == result["impact_speed"]
    assert contact["bumper_clearance"] < 0.05
    assert len({event["id"] for event in events}) == len(events)
    snapshot = hot.data.qpos.copy()
    hot.step()
    np.testing.assert_array_equal(snapshot, hot.data.qpos)


def test_cold_control_stops_with_same_trigger_and_clearance(cold, hot):
    result = cold.summary()["public"]
    assert result["outcome"] == "stopped"
    assert result["safe"] and not result["collision"] and not result["censored"]
    assert result["final_speed"] < 0.1
    assert 5 < result["wall_clearance"] < 10
    assert 45 < result["stopping_distance"] < 50
    assert result["provenance"] == "developer_control"
    assert cold.presentation()["status"] == "SUCCESS - STOPPED IN TARGET"
    assert result["goal_reached"] and result["task_complete"]
    assert cold.goal_zone["x_start"] <= result["final_front_x"] <= cold.goal_zone["x_end"]
    assert result["brake_start_time"] == hot.summary()["public"]["brake_start_time"]
    assert cold.config.initial_speed == hot.config.initial_speed
    assert cold._plant.config.thermal == hot._plant.config.thermal


def test_wall_free_control_measures_uncensored_physical_stop(wall_free, hot, cold):
    result = wall_free.summary()["public"]
    assert result["stopped"] and not result["collision"] and not result["censored"]
    assert result["wall_clearance"] is None
    assert result["stopping_distance"] > 2 * cold.summary()["public"]["stopping_distance"]
    assert result["final_front_x"] > hot.config.wall_x + 40
    assert result["provenance"] == "developer_control"
    np.testing.assert_array_equal(wall_free._initial_temperature, hot._initial_temperature)
    assert wall_free.preparation_history == hot.preparation_history


def test_preparation_includes_every_step_reset_and_actual_motion(hot):
    history = hot.preparation_history
    observations = hot.preparation_observations
    diagnostics = hot.preparation_diagnostics
    resets = [row for row in history if row["kind"] == "reset"]
    steps = [row for row in history if row["kind"] == "step"]
    assert len(resets) == hot.config.warmup_cycles + 1
    assert resets[-1] == {"kind": "reset", "speed_mps": 25.0, "phase": "trial"}
    assert len(observations) == sum(row["steps"] for row in steps) + len(resets) - 1
    assert len(diagnostics) == len(observations)
    assert observations[-1]["time"] == pytest.approx(hot.approach_start_time)
    assert sum(row["steps"] * row["dt_s"] for row in steps) == pytest.approx(hot.approach_start_time)
    assert all(b["time"] >= a["time"] for a, b in zip(observations, observations[1:]))
    assert all(row["barrier_enabled"] is False for row in observations)
    for cycle in range(1, 5):
        rows = [row for row in observations if row["phase"] == f"conditioning_{cycle}"]
        assert rows[0]["phase_time"] == 0 and rows[0]["speed"] == 0
        assert max(row["speed"] for row in rows) >= 25
        assert rows[-1]["speed"] < 0.1
        assert any(row["throttle"] > 0 for row in rows)
        assert any(row["brake"] > 0 for row in rows)
    assert min(hot._initial_temperature) > 400
    np.testing.assert_array_equal(diagnostics[-1]["temperature"], hot._initial_temperature)
    encoded = json.dumps({"observation": hot.observe(), "preparation": observations,
                          "events": hot.public_events, "summary": hot.summary()["public"]}, allow_nan=False)
    for private_name in ("temperature", "thermal_history", "brake_torque", "efficiency", "trusted_plant"):
        assert private_name not in encoded


def test_trial_reset_retains_heat_and_full_reset_replays_in_place():
    sim = Simulation(Config(warmup_cycles=1))
    model, data = sim.model, sim.data
    initial_heat = sim._plant.temperature.copy()
    start_time = sim.elapsed
    finish(sim)
    end_time = sim.elapsed
    retained_heat = sim._plant.temperature.copy()
    old_preparation_rows = len(sim.preparation_observations)
    old_trial_rows = len(sim._trial_observations)
    assert not np.array_equal(initial_heat, retained_heat)
    sim.reset_trial()
    assert sim.model is model and sim.data is data
    assert sim.elapsed == end_time and sim.trial_time == 0
    assert sim.conditioning_duration == start_time
    assert len(sim.preparation_observations) == old_preparation_rows + old_trial_rows
    assert sim.preparation_observations[-1]["time"] == end_time
    assert sim.preparation_history[-2]["kind"] == "step"
    np.testing.assert_array_equal(sim._plant.temperature, retained_heat)
    assert not sim.finished and not sim._plant.collision
    assert sim.public_events[-2]["event"] == "trial_reset"
    assert sim.public_events[-2]["preserves_component_state"]
    sim.reset_full()
    assert sim.model is model and sim.data is data
    assert sim.elapsed == start_time
    assert sim.public_events[0]["trial_time"] == 0
    assert sim.presentation()["status"] == "READY - STOP TARGET AHEAD"
    assert np.all(sim.model.geom_rgba[sim._goal_geoms, 3] == 1)
    np.testing.assert_array_equal(sim._plant.temperature, initial_heat)


def test_three_fresh_runs_agree_on_physical_outcome(hot):
    expected = hot.summary()["public"]
    for _ in range(2):
        actual_sim = finish(Simulation(Config()))
        actual = actual_sim.summary()["public"]
        assert actual["outcome"] == expected["outcome"]
        assert abs(actual["collision_time"] - expected["collision_time"]) <= hot.config.timestep
        assert abs(actual["impact_speed"] - expected["impact_speed"]) < 1e-8
        np.testing.assert_allclose(actual_sim.data.qpos, hot.data.qpos, atol=1e-8)


def test_half_timestep_keeps_failure_and_metrics_within_two_percent(hot):
    fine = finish(Simulation(replace(hot.config, timestep=hot.config.timestep / 2)))
    a, b = hot.summary()["public"], fine.summary()["public"]
    assert b["outcome"] == a["outcome"]
    for metric in ("distance_traveled", "collision_time", "impact_speed"):
        assert abs(b[metric] / a[metric] - 1) <= 0.02
    assert b["aftermath_duration"] >= 1


def test_external_bounded_pedals_reach_the_same_physical_plant():
    sim = Simulation(Config(fault="healthy", brake_at=3))
    for _ in range(100):
        sim.step({"throttle": 0, "brake": 0})
    assert sim._plant.front_x > sim.config.brake_at
    assert sim.observe()["command"] == {"throttle": 0, "brake": 0}
    assert sim.observe()["brake"] == 0
    sim.step({"throttle": 0, "brake": 0.3})
    assert sim._plant.last_command == (0, 0.3)
    assert sim.observe()["command"]["brake"] == 0.3
    assert all(sim.model.dof_frictionloss[sim._plant.spin] > 0)
    for command in ({"brake": float("nan")}, {"brake": 2}, {"throttle": -1}, {"steering": 0}):
        before = sim.elapsed
        with pytest.raises(ValueError):
            sim.step(command)
        assert sim.elapsed == before
    assert all(mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_CAMERA, name) >= 0
               for name in ("overview", "chase", "side"))


@pytest.mark.parametrize("values", [
    {"fault": "invented"}, {"probe": "steering"}, {"timestep": float("nan")},
    {"duration": -1}, {"warmup_cycles": True}, {"warmup_cycles": 1.5},
    {"brake_at": 100}, {"initial_speed": float("inf")},
])
def test_configuration_rejects_invalid_values(values):
    with pytest.raises(ValueError):
        Config(**values)


def test_opening_chase_frames_the_car_stop_zone_and_barrier():
    sim = Simulation(Config(fault="healthy"))
    assert sim.goal_zone["x_end"] <= sim.config.wall_x - 2
    assert sim.task_metadata["visible_zone"]["label"] == "STOP TARGET"
    camera = sim.model.camera("chase").id
    rotation = sim.data.cam_xmat[camera].reshape(3, 3)
    tangent = np.tan(np.deg2rad(sim.model.cam_fovy[camera]) / 2)
    points = [sim.data.xpos[sim.focus_body],
              [(sim.goal_zone["x_start"] + sim.goal_zone["x_end"]) / 2, 0, 0.03],
              [sim.config.wall_x, 0, 1.5]]
    for point in points:
        local = rotation.T @ (np.asarray(point) - sim.data.cam_xpos[camera])
        assert local[2] < 0
        x = local[0] / (-local[2] * tangent * 16 / 9)
        y = local[1] / (-local[2] * tangent)
        assert max(abs(x), abs(y)) < 0.95
    assert np.all(sim.model.geom_contype[sim._goal_geoms] == 0)
    assert np.all(sim.model.geom_conaffinity[sim._goal_geoms] == 0)
    assert len(sim._goal_geoms) == 17
    assert all(text.isascii() for text in sim.presentation().values())


def test_goal_geometry_and_cameras_do_not_change_raw_plant_dynamics():
    decorated = Simulation(Config(fault="healthy"))
    reference = Simulator(decorated._plant.config)
    reference.prepare()
    step = 0
    while not decorated.finished:
        decorated.step()
        reference.step(*reference.command())
        step += 1
        if step % 100 == 0 or decorated.finished:
            np.testing.assert_array_equal(decorated.data.qpos, reference.data.qpos)
            np.testing.assert_array_equal(decorated.data.qvel, reference.data.qvel)
            np.testing.assert_array_equal(decorated._plant.temperature, reference.temperature)
    assert decorated._plant.collision == reference.collision
    assert decorated.model.ngeom == reference.model.ngeom
    assert decorated.model.nbody == reference.model.nbody


def test_presentation_distinguishes_timeout_and_open_track():
    timed_out = finish(Simulation(Config(fault="healthy", duration=0.1)))
    assert timed_out.presentation()["status"] == "FAILED - TIME LIMIT"
    assert "incomplete" in timed_out.presentation()["detail"]
    open_track = Simulation(Config(fault="healthy", probe="wall_free"))
    assert open_track.presentation()["status"] == "READY - OPEN TRACK"
    assert open_track.goal_zone is None
    assert open_track._goal_geoms == []


def test_stopping_before_the_zone_is_safe_but_does_not_complete_the_task():
    sim = finish(Simulation(Config(fault="healthy", initial_speed=2, brake_at=3)))
    result = sim.summary()["public"]
    assert result["safe"] and result["stopped"]
    assert result["final_front_x"] < sim.goal_zone["x_start"]
    assert not result["goal_reached"] and not result["task_complete"]
    assert not sim.observe()["goal_reached"]
    assert sim.presentation()["status"] == "FAILED - STOPPED OUTSIDE TARGET"
