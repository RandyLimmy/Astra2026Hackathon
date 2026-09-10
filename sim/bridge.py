"""Fixed host mechanics: only force crosses the candidate process boundary."""

from pathlib import Path
import math

import mujoco

MASS_KG = 1200.0
DRIVE_FORCE_N = 4000.0
STOP_SPEED_MPS = 1e-8
SCENE_PATH = Path(__file__).with_name("brake_scene.xml")


class Mechanics:
    def __init__(self, timestep_s=0.01):
        self.model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
        self.model.opt.timestep = timestep_s
        self.data = mujoco.MjData(self.model)

    @property
    def speed(self):
        return float(self.data.qvel[0])

    @property
    def position(self):
        return float(self.data.qpos[0])

    @property
    def time(self):
        return float(self.data.time)

    def advance(self, drive_force_n, braking_force_n, dt_s):
        for value in (drive_force_n, braking_force_n, dt_s):
            if not math.isfinite(value):
                raise ValueError("Non-finite mechanical input")
        if braking_force_n < 0 or not 0 < dt_s <= 0.02:
            raise ValueError("Invalid braking force or timestep")
        v_before, x_before, t_before = self.speed, self.position, self.time
        # Limit impulse to avoid reversing the one-axis cart. RK4 integrates
        # this constant-force interval; the worker state advances only once.
        applied = min(braking_force_n, max(0.0, drive_force_n + MASS_KG * v_before / dt_s))
        self.model.opt.timestep = dt_s
        self.data.qfrc_applied[0] = drive_force_n - applied
        mujoco.mj_step(self.model, self.data)
        if self.speed < STOP_SPEED_MPS:
            self.data.qvel[0] = 0.0
        if not all(math.isfinite(v) for v in (self.speed, self.position)):
            raise RuntimeError("Mechanical rollout produced invalid values")
        return {
            "t_s": t_before, "dt_s": dt_s, "x_m": x_before,
            "v_mps": v_before, "next_v_mps": self.speed,
            "drive_force_n": drive_force_n,
        }, applied
