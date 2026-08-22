"""Bounded in-process MuJoCo bridge; it contains no DDS or real-G1 code."""

from __future__ import annotations

from dataclasses import dataclass
import importlib
from pathlib import Path

import numpy as np

from .adapter import G1_ARM_JOINT_NAMES
from foheart.whole_body.gmr import G1_JOINT_NAMES
from foheart.whole_body.reference import ProcessedG1Reference

G1_BODY_ACTUATORS = (
    "left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee", "left_ankle_pitch", "left_ankle_roll",
    "right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee", "right_ankle_pitch", "right_ankle_roll",
    "waist_yaw", "waist_roll", "waist_pitch",
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw", "left_elbow", "left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw",
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw", "right_elbow", "right_wrist_roll", "right_wrist_pitch", "right_wrist_yaw",
)

# Source: unitree_mujoco/simulate_python/test/g1_stand_hold.py at the pinned
# local unitree_mujoco checkout. These are simulation-only controller gains.
G1_WHOLE_BODY_SIM_KP = np.array(
    [100.0, 100.0, 100.0, 150.0, 40.0, 40.0] * 2
    + [100.0, 40.0, 40.0]
    + [40.0] * 14
)
G1_WHOLE_BODY_SIM_KD = np.array(
    [2.0, 2.0, 2.0, 4.0, 2.0, 2.0] * 2
    + [2.0, 1.0, 1.0]
    + [1.0] * 14
)
G1_WHOLE_BODY_SIM_KP.setflags(write=False)
G1_WHOLE_BODY_SIM_KD.setflags(write=False)


def intersect_joint_limits(
    *bounds: tuple[np.ndarray, np.ndarray],
) -> tuple[np.ndarray, np.ndarray]:
    """Return the conservative intersection of source-proven 29-DoF bounds."""

    if not bounds:
        raise ValueError("at least one G1 joint-limit source is required")
    lowers, uppers = [], []
    for lower, upper in bounds:
        lower = np.asarray(lower, dtype=float)
        upper = np.asarray(upper, dtype=float)
        if lower.shape != (29,) or upper.shape != (29,):
            raise ValueError("G1 joint-limit sources must contain exactly 29 values")
        if not np.isfinite(lower).all() or not np.isfinite(upper).all() or np.any(lower > upper):
            raise ValueError("G1 joint-limit sources must be finite and ordered")
        lowers.append(lower)
        uppers.append(upper)
    lower = np.maximum.reduce(lowers)
    upper = np.minimum.reduce(uppers)
    if np.any(lower > upper):
        raise ValueError("G1 joint-limit sources have an empty intersection")
    return lower.copy(), upper.copy()


@dataclass(frozen=True)
class SimStepMetrics:
    maximum_arm_error_rad: float
    mean_arm_error_rad: float
    maximum_non_arm_drift_rad: float
    finite: bool
    steps: int


@dataclass(frozen=True)
class WholeBodySimMetrics:
    maximum_joint_error_rad: float
    mean_joint_error_rad: float
    finite: bool
    steps: int
    simulation_duration_s: float
    root_position_m: tuple[float, float, float]
    root_quaternion_wxyz: tuple[float, float, float, float]
    base_pinned: bool
    stability_status: str


class G1MuJoCoBridge:
    """Direct torque-control validation with the floating base pinned in simulation."""

    mode = "SIMULATION_ONLY"
    whole_body_mode = "DIRECT_DYNAMIC_SIM"

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
        if (self.model.nq, self.model.nv, self.model.nu) != (36, 35, 29):
            raise RuntimeError("MuJoCo G1 model must expose nq=36, nv=35, nu=29")
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
        joint_ids = np.asarray(self.model.actuator_trnid[:, 0], dtype=int)
        joint_names = tuple(
            self.mujoco.mj_id2name(self.model, self.mujoco.mjtObj.mjOBJ_JOINT, int(index))
            for index in joint_ids
        )
        if joint_names != G1_JOINT_NAMES:
            raise RuntimeError(f"unexpected MuJoCo G1 joint order: {joint_names}")
        limits = np.asarray(self.model.jnt_range[joint_ids], dtype=float)
        if not np.asarray(self.model.jnt_limited[joint_ids], dtype=bool).all():
            raise RuntimeError("every MuJoCo G1 actuator joint must be limited")
        if limits.shape != (29, 2) or not np.isfinite(limits).all() or np.any(limits[:, 0] > limits[:, 1]):
            raise RuntimeError("MuJoCo G1 joint limits are invalid")
        self.joint_lower = limits[:, 0].copy()
        self.joint_upper = limits[:, 1].copy()
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

    def conservative_joint_limits(
        self, source_lower: np.ndarray, source_upper: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        return intersect_joint_limits(
            (source_lower, source_upper), (self.joint_lower, self.joint_upper)
        )

    def command_whole_body(
        self, reference: ProcessedG1Reference, *, steps: int = 8
    ) -> WholeBodySimMetrics:
        """Apply one coherent 29-DoF target through MuJoCo torque dynamics."""

        if not isinstance(reference, ProcessedG1Reference):
            raise TypeError("direct dynamics requires a ProcessedG1Reference")
        if tuple(reference.joint_names) != G1_JOINT_NAMES:
            raise ValueError("whole-body reference joint order is invalid")
        qpos = np.asarray(reference.qpos_wxyz, dtype=float)
        if qpos.shape != (36,) or not np.isfinite(qpos).all():
            raise ValueError("whole-body reference must be a finite length-36 qpos")
        if not np.isclose(np.linalg.norm(qpos[3:7]), 1.0, atol=1e-5):
            raise ValueError("whole-body reference root quaternion must be normalized WXYZ")
        target = qpos[7:]
        if np.any(target < self.joint_lower) or np.any(target > self.joint_upper):
            raise ValueError("whole-body reference violates MuJoCo model joint limits")
        if not 1 <= steps <= 5000:
            raise ValueError("simulation steps must be between 1 and 5000")

        self.target[:] = target
        finite = True
        executed = 0
        for _ in range(steps):
            self._pin_base()
            q = self.data.qpos[self.qpos_address]
            dq = self.data.qvel[self.dof_address]
            torque = G1_WHOLE_BODY_SIM_KP * (self.target - q) - G1_WHOLE_BODY_SIM_KD * dq
            self.data.ctrl[:] = np.clip(
                torque, self.model.actuator_ctrlrange[:, 0], self.model.actuator_ctrlrange[:, 1]
            )
            self.mujoco.mj_step(self.model, self.data)
            executed += 1
            finite = bool(np.isfinite(self.data.qpos).all() and np.isfinite(self.data.qvel).all())
            if not finite:
                break
        self._pin_base()
        self.mujoco.mj_forward(self.model, self.data)
        error = np.abs(self.data.qpos[self.qpos_address] - target)
        root = self.data.qpos[:7]
        return WholeBodySimMetrics(
            float(np.max(error)),
            float(np.mean(error)),
            finite,
            executed,
            float(executed * self.model.opt.timestep),
            tuple(map(float, root[:3])),
            tuple(map(float, root[3:7])),
            True,
            "BASE_PINNED_NOT_ASSESSED",
        )


assert G1_WHOLE_BODY_SIM_KP.shape == G1_WHOLE_BODY_SIM_KD.shape == (29,)
