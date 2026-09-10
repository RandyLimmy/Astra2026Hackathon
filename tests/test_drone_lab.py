"""Independent drone probe recipes, force frames, and numerical sensitivity checks."""

from dataclasses import replace
import json
import math
from pathlib import Path
import unittest

import mujoco
import numpy as np

from simulator.platforms.drone import Config, FAULTS, Simulation


RECIPES = Path(__file__).resolve().parents[1] / "simulator" / "experiments"


def run_schedule(config, schedule):
    sim = Simulation(config)
    cursor = 0
    states = []
    while not sim.finished:
        while cursor + 1 < len(schedule) and schedule[cursor + 1][0] <= sim.trial_time + 1e-10:
            cursor += 1
        sim.step(schedule[cursor][1])
        states.append(sim.data.qpos.copy())
    return sim, np.array(states)


def recipe(name):
    return json.loads((RECIPES / f"drone-{name}.json").read_text())


class DroneInterventionTests(unittest.TestCase):
    def test_all_recipes_are_ordered_and_healthy_flight_stays_airborne(self):
        for name in ("collective-pulses", "attitude-pulses", "flight-challenge"):
            with self.subTest(recipe=name):
                schedule = recipe(name)
                times = [event[0] for event in schedule]
                self.assertEqual(times[0], 0)
                self.assertTrue(all(a < b for a, b in zip(times, times[1:])))
                sim, states = run_schedule(Config(probe="hover"), schedule)
                self.assertFalse(sim.ground_contact)
                self.assertFalse(sim.data.warning.number.any())
                self.assertTrue(np.isfinite(states).all())
                self.assertTrue(sim.summary()["public"]["safe"])
                self.assertLess(sim.max_tilt, 35)
                self.assertLess(float(np.linalg.norm(sim.data.qpos[:3] - [0, 0, 2])), .1)

    def test_attitude_recipe_preserves_collective_and_excites_both_axes(self):
        schedule = recipe("attitude-pulses")
        rotor_requests = [control["rotor_commands"] for _, control in schedule if "rotor_commands" in control]
        np.testing.assert_allclose(np.sum(rotor_requests, axis=1), np.full(4, 1.962), atol=1e-12)
        sim, states = run_schedule(Config(probe="hover"), schedule)
        self.assertGreater(np.ptp(states[:, 4]), .02)
        self.assertGreater(np.ptp(states[:, 5]), .02)
        self.assertLess(np.max(np.abs(states[:, 6])), .005)
        self.assertTrue(sim.summary()["public"]["safe"])

    def test_collective_recipe_changes_altitude_without_attitude_excursion(self):
        sim, states = run_schedule(Config(probe="hover"), recipe("collective-pulses"))
        self.assertGreater(np.ptp(states[:, 2]), .25)
        self.assertLess(sim.max_tilt, .01)
        self.assertLess(float(np.max(np.linalg.norm(states[:, :2], axis=1))), 1e-6)

    def test_held_out_flight_differs_from_diagnostic_recipes(self):
        schedule = recipe("flight-challenge")
        sim, states = run_schedule(Config(probe="hover"), schedule)
        self.assertGreater(np.max(states[:, 0]), .2)
        self.assertLess(np.min(states[:, 0]), -.2)
        self.assertGreater(np.max(states[:, 1]), .1)
        self.assertLess(np.min(states[:, 1]), -.1)
        self.assertGreater(np.max(states[:, 2]), 2.45)
        diagnostic_times = {time for name in ("attitude-pulses", "collective-pulses")
                            for time, _ in recipe(name) if time}
        self.assertFalse(diagnostic_times.intersection(time for time, _ in schedule if time))
        self.assertTrue(sim.summary()["public"]["safe"])

    def test_rotor_transmissions_use_body_thrust_and_correct_lever_arms(self):
        # Verify generalized force exactly, including a tilted aircraft. This
        # catches accidentally applying thrust in world z or omitting the arm.
        sim = Simulation(Config(probe="hover"))
        for angle in (0., .6):
            sim.data.qpos[3:7] = [math.cos(angle / 2), 0, math.sin(angle / 2), 0]
            for index in range(4):
                with self.subTest(angle=angle, rotor=index):
                    sim.data.ctrl[:] = 0
                    sim.data.ctrl[index] = 1
                    mujoco.mj_forward(sim.model, sim.data)
                    rotation = sim.data.xmat[sim.focus_body].reshape(3, 3)
                    site = sim.model.site(f"rotor_site_{index}").pos
                    reaction = sim.model.actuator_gear[index, 3:]
                    torque = np.cross(site, [0, 0, 1]) + reaction
                    np.testing.assert_allclose(sim.data.qfrc_actuator[:3], rotation @ [0, 0, 1], atol=1e-12)
                    np.testing.assert_allclose(sim.data.qfrc_actuator[3:6], torque, atol=1e-12)

    def test_delay_signature_is_temporal_and_rotor_failure_is_asymmetric(self):
        base = Config(duration=.3, probe="hover", fault_at=0)
        control = {"rotor_commands": [.65] * 4}
        healthy = Simulation(base)
        delayed = Simulation(replace(base, fault="delay", control_delay=.1))
        damaged = Simulation(replace(base, fault="rotor_loss", rotor_effectiveness=.7))
        for sim in (healthy, delayed, damaged):
            for _ in range(25):
                sim.step(control)
        self.assertGreater(healthy.data.qvel[2], delayed.data.qvel[2] + .05)
        self.assertLess(float(np.linalg.norm(delayed.data.qvel[3:6])), 1e-8)
        self.assertGreater(float(np.linalg.norm(damaged.data.qvel[3:6])), .1)
        np.testing.assert_array_equal(delayed.last_command, healthy.last_command)

    def test_each_fault_converges_before_contact_when_timestep_halves(self):
        # Compare a maneuver before impact; post-impact tumble paths are chaotic
        # and cannot reasonably carry the same millimetre-scale tolerance.
        schedule = [[0., {"target_position": [.3, -.2, 2.2]}]]
        for fault in FAULTS:
            with self.subTest(fault=fault):
                config = Config(fault=fault, duration=.7, fault_at=.2, probe="hover",
                                rotor_effectiveness=.6, voltage_ratio=.85, payload_mass=.3,
                                wind_force=1., control_delay=.08)
                coarse, _ = run_schedule(config, schedule)
                fine, _ = run_schedule(replace(config, timestep=config.timestep / 2), schedule)
                self.assertFalse(coarse.ground_contact)
                self.assertFalse(fine.ground_contact)
                self.assertFalse(coarse.data.warning.number.any())
                self.assertFalse(fine.data.warning.number.any())
                np.testing.assert_allclose(coarse.data.qpos[:3], fine.data.qpos[:3], atol=.005, rtol=0)
                self.assertLess(float(np.linalg.norm(coarse.data.qvel[:3] - fine.data.qvel[:3])), .02)

    def test_replayed_probe_has_identical_commands_and_state(self):
        config = Config(fault="wind", duration=8., fault_at=3., wind_force=.8, probe="hover")
        first, states = run_schedule(config, recipe("attitude-pulses"))
        second, repeat = run_schedule(config, recipe("attitude-pulses"))
        np.testing.assert_array_equal(states, repeat)
        self.assertEqual(first.summary(), second.summary())

    def test_public_recipes_and_nested_observations_do_not_disclose_fault_truth(self):
        forbidden = {"fault", "config", "events", "rotor_effectiveness", "voltage_ratio",
                     "control_delay", "wind_force", "mass", "inertia", "center_of_mass",
                     "motor_state", "rotor_thrust", "fault_at"}

        def check(value):
            if isinstance(value, dict):
                self.assertFalse(forbidden.intersection(value))
                for item in value.values():
                    check(item)
            elif isinstance(value, list):
                for item in value:
                    check(item)

        for fault in FAULTS:
            sim = Simulation(Config(fault=fault, fault_at=0, duration=.05))
            sim.step()
            check(sim.observe())
            check(sim.summary()["public"])
            json.dumps(sim.observe(), allow_nan=False)
        for name in ("collective-pulses", "attitude-pulses", "flight-challenge"):
            check(recipe(name))


if __name__ == "__main__":
    unittest.main()
