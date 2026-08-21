"""Bounded in-process MuJoCo bridge; it contains no DDS or real-G1 code."""

from __future__ import annotations

from dataclasses import dataclass
import importlib
from pathlib import Path

import numpy as np

from .adapter import G1_ARM_JOINT_NAMES

G1_BODY_ACTUATORS = (
    "left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee", "left_ankle_pitch", "left_ankle_roll",
    "right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee", "right_ankle_pitch", "right_ankle_roll",
    "waist_yaw", "waist_roll", "waist_pitch",
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw", "left_elbow", "left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw",
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw", "right_elbow", "right_wrist_roll", "right_wrist_pitch", "right_wrist_yaw",
)


@dataclass(frozen=True)
class SimStepMetrics:
    maximum_arm_error_rad: float
    mean_arm_error_rad: float
    maximum_non_arm_drift_rad: float
    finite: bool
    steps: int


class G1MuJoCoBridge:
    """Direct torque-control validation with the floating base pinned in simulation."""

    mode = "SIMULATION_ONLY"

    def __init__(self, model_path: str | Path, *, timestep_s: float = 0.002):
        if not 0 < timestep_s <= 0.01:
            raise ValueError("simulation timestep must be in (0, 0.01] seconds")
        self.mujoco = importlib.import_module("mujoco")
        path = Path(model_path).resolve()
        if not path.is_file():
            raise FileNotFoundError(path)
        self.model = self.mujoco.MjModel.from_xml_path(str(path))
        self.data = self.mujoco.MjData(self.model)
        self.model.opt.timestep = timestep_s
        names = tuple(
            self.mujoco.mj_id2name(self.model, self.mujoco.mjtObj.mjOBJ_ACTUATOR, index)
            for index in range(self.model.nu)
        )
        if names != G1_BODY_ACTUATORS:
            raise RuntimeError(f"unexpected MuJoCo G1 actuator order: {names}")
        if tuple(f"{name}_joint" for name in names[15:]) != G1_ARM_JOINT_NAMES:
            raise RuntimeError("MuJoCo and existing IK arm joint orders differ")

        self.qpos_address = np.empty(29, dtype=int)
        self.dof_address = np.empty(29, dtype=int)
        for index in range(29):
            joint = int(self.model.actuator_trnid[index, 0])
            self.qpos_address[index] = self.model.jnt_qposadr[joint]
            self.dof_address[index] = self.model.jnt_dofadr[joint]
        self.base_qpos = self.data.qpos[:7].copy()
        self.target = self.data.qpos[self.qpos_address].copy()
        self.initial_non_arm = self.target[:15].copy()
        self.kp = np.array([100.0] * 12 + [80.0] * 3 + [40.0] * 14)
        self.kd = np.array([3.0] * 15 + [2.0] * 14)
        self.mujoco.mj_forward(self.model, self.data)

    @property
    def arm_positions(self) -> np.ndarray:
        return self.data.qpos[self.qpos_address[15:]].copy()

    def _pin_base(self) -> None:
        self.data.qpos[:7] = self.base_qpos
        self.data.qvel[:6] = 0.0

    def command(self, arm_joint_positions: np.ndarray, *, steps: int = 250) -> SimStepMetrics:
        target = np.asarray(arm_joint_positions, dtype=float)
        if target.shape != (14,) or not np.isfinite(target).all():
            raise ValueError("sim arm target must be 14 finite joint positions")
        if not 1 <= steps <= 5000:
            raise ValueError("simulation steps must be between 1 and 5000")
        self.target[15:] = target
        errors = []
        finite = True
        for _ in range(steps):
            self._pin_base()
            q = self.data.qpos[self.qpos_address]
            dq = self.data.qvel[self.dof_address]
            torque = self.kp * (self.target - q) - self.kd * dq
            self.data.ctrl[:] = np.clip(torque, self.model.actuator_ctrlrange[:, 0], self.model.actuator_ctrlrange[:, 1])
            self.mujoco.mj_step(self.model, self.data)
            finite = finite and bool(np.isfinite(self.data.qpos).all() and np.isfinite(self.data.qvel).all())
            errors.append(np.abs(self.data.qpos[self.qpos_address[15:]] - target))
            if not finite:
                break
        self._pin_base()
        final_error = np.abs(self.arm_positions - target)
        non_arm = np.abs(self.data.qpos[self.qpos_address[:15]] - self.initial_non_arm)
        return SimStepMetrics(
            float(np.max(final_error)),
            float(np.mean(final_error)),
            float(np.max(non_arm)),
            finite,
            len(errors),
        )
