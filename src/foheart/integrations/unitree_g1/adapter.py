"""Human wrist targets -> existing xr_teleoperate G1 IK, with fail-closed checks."""

from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass, replace
import importlib
import math
import os
from pathlib import Path
import sys
from typing import Any

import numpy as np

from foheart.mocap.frames import (
    BasisTransform,
    homogeneous,
    matrix_to_quaternion,
    quaternion_to_matrix,
    require_rotation_matrix,
    slerp,
)
from foheart.mocap.orientation import quaternion_angular_distance_degrees
from foheart.mocap.skeleton import UpperBodyTargets

G1_ARM_JOINT_NAMES = (
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
)


def _require_pose(pose: np.ndarray, name: str) -> np.ndarray:
    value = np.asarray(pose, dtype=float)
    if value.shape != (4, 4) or not np.isfinite(value).all():
        raise ValueError(f"{name} must be a finite 4x4 transform")
    if not np.allclose(value[3], (0, 0, 0, 1), atol=1e-9):
        raise ValueError(f"{name} must have homogeneous bottom row [0,0,0,1]")
    require_rotation_matrix(value[:3, :3], name=f"{name} rotation")
    return value


@dataclass(frozen=True)
class G1WristTargets:
    left: np.ndarray
    right: np.ndarray
    timestamp_ns: int
    clamped: tuple[str, ...] = ()
    status: str = "CONFIGURED"


class G1FrameAdapter:
    """Neutral-aligned, shoulder-relative reach mapping into the G1 base frame."""

    def __init__(
        self,
        human_neutral: UpperBodyTargets,
        robot_neutral_left: np.ndarray,
        robot_neutral_right: np.ndarray,
        robot_left_shoulder: np.ndarray,
        robot_right_shoulder: np.ndarray,
        *,
        human_reach_m: float,
        robot_reach_m: float,
        max_robot_reach_m: float,
        g1_from_human: BasisTransform | None = None,
    ):
        if min(human_reach_m, robot_reach_m, max_robot_reach_m) <= 0:
            raise ValueError("reach values must be positive")
        self.human_neutral = human_neutral
        self.robot_neutral = {
            "left": _require_pose(robot_neutral_left, "robot neutral left").copy(),
            "right": _require_pose(robot_neutral_right, "robot neutral right").copy(),
        }
        self.robot_shoulders = {
            "left": np.asarray(robot_left_shoulder, dtype=float),
            "right": np.asarray(robot_right_shoulder, dtype=float),
        }
        if any(value.shape != (3,) or not np.isfinite(value).all() for value in self.robot_shoulders.values()):
            raise ValueError("robot shoulder positions must be finite length-3 vectors")
        self.human_reach_m = human_reach_m
        self.robot_reach_m = robot_reach_m
        self.max_robot_reach_m = max_robot_reach_m
        self.basis = g1_from_human or BasisTransform.identity("human_torso", "g1_base")

    def adapt(self, targets: UpperBodyTargets) -> G1WristTargets:
        if not targets.valid:
            raise ValueError(f"invalid upper-body targets: {targets.reason}")
        basis = np.asarray(self.basis.matrix)
        clamped = []
        result = {}
        for side in ("left", "right"):
            wrist = _require_pose(getattr(targets, f"{side}_wrist_pose"), f"human {side} wrist")
            shoulder = _require_pose(getattr(targets, f"{side}_shoulder_pose"), f"human {side} shoulder")
            neutral_wrist = _require_pose(getattr(self.human_neutral, f"{side}_wrist_pose"), f"neutral human {side} wrist")
            neutral_shoulder = _require_pose(getattr(self.human_neutral, f"{side}_shoulder_pose"), f"neutral human {side} shoulder")

            current_reach = (wrist[:3, 3] - shoulder[:3, 3]) / self.human_reach_m
            neutral_reach = (neutral_wrist[:3, 3] - neutral_shoulder[:3, 3]) / self.human_reach_m
            position = self.robot_neutral[side][:3, 3] + self.robot_reach_m * (basis @ (current_reach - neutral_reach))
            shoulder_to_target = position - self.robot_shoulders[side]
            distance = float(np.linalg.norm(shoulder_to_target))
            if distance > self.max_robot_reach_m:
                position = self.robot_shoulders[side] + shoulder_to_target * (self.max_robot_reach_m / distance)
                clamped.append(side)

            human_delta = neutral_wrist[:3, :3].T @ wrist[:3, :3]
            rotation = self.robot_neutral[side][:3, :3] @ basis @ human_delta @ basis.T
            result[side] = homogeneous(rotation, position)
        return G1WristTargets(result["left"], result["right"], targets.timestamp_ns, tuple(clamped))


class UpperBodyTargetFilter:
    def __init__(
        self,
        *,
        position_alpha: float = 0.35,
        orientation_alpha: float = 0.35,
        max_translation_rate_m_s: float = 1.5,
        max_angular_rate_deg_s: float = 360.0,
    ):
        if not 0 < position_alpha <= 1 or not 0 < orientation_alpha <= 1:
            raise ValueError("filter alphas must be in (0, 1]")
        if min(max_translation_rate_m_s, max_angular_rate_deg_s) <= 0:
            raise ValueError("filter rate limits must be positive")
        self.position_alpha = position_alpha
        self.orientation_alpha = orientation_alpha
        self.max_translation_rate_m_s = max_translation_rate_m_s
        self.max_angular_rate_deg_s = max_angular_rate_deg_s
        self.previous: UpperBodyTargets | None = None

    def update(self, targets: UpperBodyTargets) -> UpperBodyTargets:
        if not targets.valid:
            return replace(self.previous, valid=False, reason=targets.reason) if self.previous else targets
        if self.previous is None:
            self.previous = targets
            return targets
        dt = (targets.timestamp_ns - self.previous.timestamp_ns) / 1e9
        if dt <= 0:
            raise ValueError("target timestamps must increase")

        values: dict[str, np.ndarray] = {}
        for side in ("left", "right"):
            current = _require_pose(getattr(targets, f"{side}_wrist_pose"), f"{side} wrist")
            previous = _require_pose(getattr(self.previous, f"{side}_wrist_pose"), f"previous {side} wrist")
            position = previous[:3, 3] + self.position_alpha * (current[:3, 3] - previous[:3, 3])
            delta = position - previous[:3, 3]
            maximum = self.max_translation_rate_m_s * dt
            if np.linalg.norm(delta) > maximum:
                position = previous[:3, 3] + delta * (maximum / np.linalg.norm(delta))

            previous_q = matrix_to_quaternion(previous[:3, :3])
            target_q = matrix_to_quaternion(current[:3, :3])
            filtered_q = slerp(previous_q, target_q, self.orientation_alpha)
            angle = quaternion_angular_distance_degrees(previous_q, filtered_q)
            maximum_angle = self.max_angular_rate_deg_s * dt
            if angle > maximum_angle:
                filtered_q = slerp(previous_q, filtered_q, maximum_angle / angle)
            values[side] = homogeneous(quaternion_to_matrix(filtered_q), position)
        filtered = UpperBodyTargets(
            values["left"],
            values["right"],
            targets.left_shoulder_pose,
            targets.right_shoulder_pose,
            targets.timestamp_ns,
            reference_frame=targets.reference_frame,
            units=targets.units,
        )
        self.previous = filtered
        return filtered


@dataclass(frozen=True)
class IKResult:
    joint_positions: np.ndarray
    feedforward_torque: np.ndarray
    valid: bool
    held_previous: bool
    rate_limited: bool
    reason: str
    position_error_m: float | None = None
    rotation_error_deg: float | None = None


class SafeG1IK:
    def __init__(self, solver: Any, *, max_joint_delta_rad: float = 0.35, workspace_radius_m: float = 1.0):
        if min(max_joint_delta_rad, workspace_radius_m) <= 0:
            raise ValueError("IK safety limits must be positive")
        self.solver = solver
        self.lower = np.asarray(solver.lower_limits, dtype=float)
        self.upper = np.asarray(solver.upper_limits, dtype=float)
        if self.lower.shape != (14,) or self.upper.shape != (14,):
            raise ValueError("G1 IK must expose 14 joint limits")
        self.previous = np.zeros(14)
        self.max_joint_delta_rad = max_joint_delta_rad
        self.workspace_radius_m = workspace_radius_m

    def _hold(self, reason: str) -> IKResult:
        return IKResult(self.previous.copy(), np.zeros(14), False, True, False, reason)

    def solve(self, targets: G1WristTargets) -> IKResult:
        try:
            left = _require_pose(targets.left, "G1 left wrist")
            right = _require_pose(targets.right, "G1 right wrist")
        except ValueError as exc:
            return self._hold(str(exc))
        if max(np.linalg.norm(left[:3, 3]), np.linalg.norm(right[:3, 3])) > self.workspace_radius_m:
            return self._hold("G1 wrist target is outside the configured workspace")
        try:
            q, tau = self.solver.solve_ik(left, right, self.previous.copy(), np.zeros(14))
            q, tau = np.asarray(q, dtype=float), np.asarray(tau, dtype=float)
        except Exception as exc:  # external solver boundary
            return self._hold(f"G1 IK failed: {exc}")
        if q.shape != (14,) or tau.shape != (14,) or not np.isfinite(q).all() or not np.isfinite(tau).all():
            return self._hold("G1 IK returned invalid joint data")
        if np.any(q < self.lower - 1e-6) or np.any(q > self.upper + 1e-6):
            return self._hold("G1 IK returned a joint-limit violation")
        position_error = rotation_error = None
        if hasattr(self.solver, "verify"):
            position_error, rotation_error = self.solver.verify(q, left, right)
            if position_error > 0.05 or rotation_error > 15:
                return self._hold(
                    f"G1 IK residual too large ({position_error:.4f} m, {rotation_error:.2f} deg)"
                )
        delta = q - self.previous
        rate_limited = bool(np.max(np.abs(delta)) > self.max_joint_delta_rad)
        if rate_limited:
            q = self.previous + np.clip(delta, -self.max_joint_delta_rad, self.max_joint_delta_rad)
        self.previous = q
        return IKResult(q.copy(), tau, True, False, rate_limited, "", position_error, rotation_error)


@contextmanager
def _working_directory(path: Path):
    previous = Path.cwd()
    os.chdir(path)
    try:
        yield
    finally:
        os.chdir(previous)


class ExistingG1IK:
    """Read-only adapter for xr_teleoperate's existing ``G1_29_ArmIK``."""

    def __init__(self, xr_root: str | Path):
        self.xr_root = Path(xr_root).resolve()
        teleop_dir = self.xr_root / "teleop"
        cache = teleop_dir / "g1_29_model_cache.pkl"
        if not (teleop_dir / "robot_control" / "robot_arm_ik.py").is_file() or not cache.is_file():
            raise RuntimeError("xr_teleoperate G1 IK or its read-only model cache is missing")
        if str(self.xr_root) not in sys.path:
            sys.path.insert(0, str(self.xr_root))
        with _working_directory(teleop_dir):
            module = importlib.import_module("teleop.robot_control.robot_arm_ik")
            self.ik = module.G1_29_ArmIK(Unit_Test=False, Visualization=False)
        names = tuple(str(name) for name in list(self.ik.reduced_robot.model.names)[1:])
        if names != G1_ARM_JOINT_NAMES:
            raise RuntimeError(f"unexpected G1 IK joint order: {names}")
        self.lower_limits = np.asarray(self.ik.reduced_robot.model.lowerPositionLimit, dtype=float)
        self.upper_limits = np.asarray(self.ik.reduced_robot.model.upperPositionLimit, dtype=float)
        self._populate_neutral_geometry()

    def _populate_neutral_geometry(self) -> None:
        import pinocchio as pin

        q = np.zeros(14)
        model, data = self.ik.reduced_robot.model, self.ik.reduced_robot.data
        pin.framesForwardKinematics(model, data, q)
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        self.neutral_left = np.asarray(data.oMf[self.ik.L_hand_id].homogeneous).copy()
        self.neutral_right = np.asarray(data.oMf[self.ik.R_hand_id].homogeneous).copy()
        self.left_shoulder = np.asarray(data.oMi[model.getJointId("left_shoulder_pitch_joint")].translation).copy()
        self.right_shoulder = np.asarray(data.oMi[model.getJointId("right_shoulder_pitch_joint")].translation).copy()

    def solve_ik(self, left: np.ndarray, right: np.ndarray, q: np.ndarray, dq: np.ndarray):
        return self.ik.solve_ik(left, right, q, dq)

    def verify(self, q: np.ndarray, left: np.ndarray, right: np.ndarray) -> tuple[float, float]:
        import pinocchio as pin

        model, data = self.ik.reduced_robot.model, self.ik.reduced_robot.data
        pin.framesForwardKinematics(model, data, q)
        actual = (data.oMf[self.ik.L_hand_id].homogeneous, data.oMf[self.ik.R_hand_id].homogeneous)
        position = max(float(np.linalg.norm(p[:3, 3] - target[:3, 3])) for p, target in zip(actual, (left, right)))
        rotations = []
        for pose, target in zip(actual, (left, right)):
            cosine = (np.trace(pose[:3, :3] @ target[:3, :3].T) - 1) / 2
            rotations.append(math.degrees(math.acos(float(np.clip(cosine, -1, 1)))))
        return position, max(rotations)
