"""Validated boundary around the pinned GMR Unitree G1 retargeter."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import importlib
from pathlib import Path
from typing import Any

import numpy as np


# Pinned GMR bb1bbe40774794fceb2a7c579a3464a28e68c844 contracts:
# ik_configs/xsens_mvn_to_g1.json and assets/unitree_g1/g1_mocap_29dof.xml.
GMR_SOURCE_HUMAN = "xsens_mvn"
GMR_TARGET_ROBOT = "unitree_g1"
GMR_REQUIRED_BONES = (
    "Pelvis",
    "Chest",
    "Left_UpperLeg",
    "Right_UpperLeg",
    "Left_LowerLeg",
    "Right_LowerLeg",
    "Left_Foot",
    "Right_Foot",
    "Left_UpperArm",
    "Right_UpperArm",
    "Left_Forearm",
    "Right_Forearm",
    "Left_Hand",
    "Right_Hand",
)
G1_JOINT_NAMES = (
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
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
G1_LINK_BODY_NAMES = (
    "pelvis",
    "left_hip_pitch_link",
    "left_hip_roll_link",
    "left_hip_yaw_link",
    "left_knee_link",
    "left_ankle_pitch_link",
    "left_ankle_roll_link",
    "left_toe_link",
    "pelvis_contour_link",
    "right_hip_pitch_link",
    "right_hip_roll_link",
    "right_hip_yaw_link",
    "right_knee_link",
    "right_ankle_pitch_link",
    "right_ankle_roll_link",
    "right_toe_link",
    "waist_yaw_link",
    "waist_roll_link",
    "torso_link",
    "head_link",
    "head_mocap",
    "imu_in_torso",
    "left_shoulder_pitch_link",
    "left_shoulder_roll_link",
    "left_shoulder_yaw_link",
    "left_elbow_link",
    "left_wrist_roll_link",
    "left_wrist_pitch_link",
    "left_wrist_yaw_link",
    "left_rubber_hand",
    "right_shoulder_pitch_link",
    "right_shoulder_roll_link",
    "right_shoulder_yaw_link",
    "right_elbow_link",
    "right_wrist_roll_link",
    "right_wrist_pitch_link",
    "right_wrist_yaw_link",
    "right_rubber_hand",
)


def _readonly(value: Any) -> np.ndarray:
    result = np.asarray(value, dtype=float).copy()
    result.setflags(write=False)
    return result


@dataclass(frozen=True)
class G1KinematicReference:
    """One validated GMR qpos, split for safety and motion recording."""

    qpos_wxyz: np.ndarray
    root_pos: np.ndarray
    root_quat_wxyz: np.ndarray
    dof_pos: np.ndarray
    joint_names: tuple[str, ...]
    joint_lower: np.ndarray | None
    joint_upper: np.ndarray | None


class GMRWholeBodyRetargeter:
    """Call pinned GMR without duplicating its IK or coordinate processing."""

    def __init__(
        self,
        retargeter: Any | None = None,
        *,
        actual_human_height: float | None = None,
        offset_to_ground: bool = False,
    ) -> None:
        if actual_human_height is not None:
            actual_human_height = float(actual_human_height)
            if not np.isfinite(actual_human_height) or actual_human_height <= 0:
                raise ValueError("actual_human_height must be finite and positive")
        if not isinstance(offset_to_ground, (bool, np.bool_)):
            raise TypeError("offset_to_ground must be bool")
        if retargeter is None:
            # Keep optional GMR/MuJoCo/Mink dependencies out of normal imports.
            from general_motion_retargeting import GeneralMotionRetargeting

            retargeter = GeneralMotionRetargeting(
                src_human=GMR_SOURCE_HUMAN,
                tgt_robot=GMR_TARGET_ROBOT,
                actual_human_height=actual_human_height,
                verbose=False,
            )
        elif actual_human_height is not None:
            raise ValueError("actual_human_height is only applied when constructing GMR")
        if not callable(getattr(retargeter, "retarget", None)):
            raise TypeError("GMR retargeter must provide retarget()")

        self.retargeter = retargeter
        self.offset_to_ground = bool(offset_to_ground)
        self._validate_exposed_order()
        limits = self._extract_model_joint_limits()
        self.joint_lower = None if limits is None else limits[0]
        self.joint_upper = None if limits is None else limits[1]

    def retarget(self, body_frame: Mapping[str, Any]) -> G1KinematicReference:
        """Validate/copy GMR human data, call GMR, and validate qpos."""

        human_data = self._copy_required_human_data(body_frame)
        qpos = np.asarray(
            self.retargeter.retarget(human_data, offset_to_ground=self.offset_to_ground),
            dtype=float,
        )
        if qpos.shape != (36,) or not np.isfinite(qpos).all():
            raise ValueError("GMR qpos must be a finite length-36 vector")
        root_quat = qpos[3:7]
        if not np.isclose(np.linalg.norm(root_quat), 1.0, atol=1e-5):
            raise ValueError("GMR root quaternion WXYZ must be normalized")
        dof_pos = qpos[7:]

        return G1KinematicReference(
            qpos_wxyz=_readonly(qpos),
            root_pos=_readonly(qpos[:3]),
            root_quat_wxyz=_readonly(root_quat),
            dof_pos=_readonly(dof_pos),
            joint_names=G1_JOINT_NAMES,
            joint_lower=None if self.joint_lower is None else _readonly(self.joint_lower),
            joint_upper=None if self.joint_upper is None else _readonly(self.joint_upper),
        )

    @staticmethod
    def _copy_required_human_data(body_frame: Mapping[str, Any]) -> dict[str, list[np.ndarray]]:
        if not isinstance(body_frame, Mapping):
            raise TypeError("GMR body_frame must be a mapping")
        missing = [name for name in GMR_REQUIRED_BONES if name not in body_frame]
        if missing:
            raise ValueError("GMR body_frame is missing: " + ",".join(missing))

        result: dict[str, list[np.ndarray]] = {}
        for name in GMR_REQUIRED_BONES:
            pose = body_frame[name]
            if not isinstance(pose, (list, tuple)) or len(pose) != 2:
                raise ValueError(f"GMR bone {name} must contain [XYZ, WXYZ]")
            position = np.asarray(pose[0], dtype=float)
            quaternion = np.asarray(pose[1], dtype=float)
            if position.shape != (3,) or not np.isfinite(position).all():
                raise ValueError(f"GMR bone {name} position must be finite XYZ")
            if quaternion.shape != (4,) or not np.isfinite(quaternion).all():
                raise ValueError(f"GMR bone {name} quaternion must be finite WXYZ")
            if not np.isclose(np.linalg.norm(quaternion), 1.0, atol=1e-5):
                raise ValueError(f"GMR bone {name} quaternion WXYZ must be normalized")
            # Pinned GMR mutates both the mapping and its nested arrays.
            result[name] = [position.copy(), quaternion.copy()]
        return result

    def _validate_exposed_order(self) -> None:
        motor_names = getattr(self.retargeter, "robot_motor_names", None)
        if motor_names is not None:
            ordered = self._ordered_name_map(motor_names, "robot_motor_names")
            if ordered != G1_JOINT_NAMES:
                raise ValueError("GMR motor order does not match the pinned G1 29-DoF order")

        dof_names = getattr(self.retargeter, "robot_dof_names", None)
        if dof_names is not None:
            if not isinstance(dof_names, Mapping):
                raise ValueError("GMR robot_dof_names must be a name-to-index mapping")
            indexed = [(name, index) for name, index in dof_names.items() if name in G1_JOINT_NAMES]
            ordered = self._ordered_pairs(indexed, "robot_dof_names")
            if ordered != G1_JOINT_NAMES:
                raise ValueError("GMR joint order does not match the pinned G1 29-DoF order")

    @classmethod
    def _ordered_name_map(cls, value: Any, label: str) -> tuple[str, ...]:
        if not isinstance(value, Mapping) or set(value) != set(G1_JOINT_NAMES):
            raise ValueError(f"GMR {label} must expose exactly the pinned 29 names")
        return cls._ordered_pairs(list(value.items()), label)

    @staticmethod
    def _ordered_pairs(pairs: list[tuple[Any, Any]], label: str) -> tuple[str, ...]:
        if len(pairs) != len(G1_JOINT_NAMES):
            raise ValueError(f"GMR {label} must expose exactly 29 indexed joints")
        try:
            indexed = [(str(name), int(index)) for name, index in pairs]
        except (TypeError, ValueError) as exc:
            raise ValueError(f"GMR {label} indices must be integers") from exc
        if any(index != raw for (_, raw), (_, index) in zip(pairs, indexed)):
            raise ValueError(f"GMR {label} indices must be integers")
        indices = [index for _, index in indexed]
        if len(set(indices)) != len(indices):
            raise ValueError(f"GMR {label} indices must be unique")
        return tuple(name for name, _ in sorted(indexed, key=lambda item: item[1]))

    def _extract_model_joint_limits(self) -> tuple[np.ndarray, np.ndarray] | None:
        model = getattr(self.retargeter, "model", None)
        if model is None:
            return None
        names = ("nu", "actuator_trnid", "jnt_range", "jnt_limited")
        present = [hasattr(model, name) for name in names]
        if not any(present):
            return None
        if not all(present):
            raise ValueError("GMR model exposes an incomplete joint-limit contract")
        if int(model.nu) != len(G1_JOINT_NAMES):
            raise ValueError("GMR model must expose exactly 29 actuators")

        transmissions = np.asarray(model.actuator_trnid)
        if transmissions.ndim != 2 or transmissions.shape[0] != 29 or transmissions.shape[1] < 1:
            raise ValueError("GMR model actuator_trnid shape is invalid")
        raw_joint_ids = transmissions[:, 0]
        joint_ids = raw_joint_ids.astype(int)
        if not np.array_equal(raw_joint_ids, joint_ids) or np.any(joint_ids < 0):
            raise ValueError("GMR model actuator joint IDs are invalid")
        if len(set(map(int, joint_ids))) != 29:
            raise ValueError("GMR model actuator joint IDs must be unique")

        ranges = np.asarray(model.jnt_range, dtype=float)
        limited = np.asarray(model.jnt_limited)
        if ranges.ndim != 2 or ranges.shape[1] != 2 or limited.ndim != 1:
            raise ValueError("GMR model joint-limit arrays are invalid")
        if np.any(joint_ids >= ranges.shape[0]) or np.any(joint_ids >= limited.shape[0]):
            raise ValueError("GMR model actuator joint ID is out of range")
        selected = ranges[joint_ids]
        if not np.asarray(limited[joint_ids], dtype=bool).all():
            raise ValueError("every pinned G1 actuator joint must be limited")
        if not np.isfinite(selected).all() or np.any(selected[:, 0] > selected[:, 1]):
            raise ValueError("GMR model joint limits must be finite and ordered")
        return _readonly(selected[:, 0]), _readonly(selected[:, 1])


class G1ReferenceMuJoCo:
    """Apply GMR qpos kinematically and compute TWIST2-compatible body FK."""

    mode = "KINEMATIC_REFERENCE"

    def __init__(self, model_path: str | Path, *, viewer: bool = False) -> None:
        self.mujoco = importlib.import_module("mujoco")
        path = Path(model_path).resolve()
        if not path.is_file():
            raise FileNotFoundError(path)
        self.model_path = path
        self.model = self.mujoco.MjModel.from_xml_path(str(path))
        self.data = self.mujoco.MjData(self.model)
        self.fk_data = self.mujoco.MjData(self.model)
        if (self.model.nq, self.model.nv, self.model.nu) != (36, 35, 29):
            raise ValueError("G1 reference model must expose nq=36, nv=35, nu=29")

        joints = tuple(
            self.mujoco.mj_id2name(self.model, self.mujoco.mjtObj.mjOBJ_JOINT, index)
            for index in range(1, self.model.njnt)
        )
        motors = tuple(
            self.mujoco.mj_id2name(self.model, self.mujoco.mjtObj.mjOBJ_ACTUATOR, index)
            for index in range(self.model.nu)
        )
        bodies = tuple(
            self.mujoco.mj_id2name(self.model, self.mujoco.mjtObj.mjOBJ_BODY, index)
            for index in range(1, self.model.nbody)
        )
        if joints != G1_JOINT_NAMES or motors != G1_JOINT_NAMES:
            raise ValueError("G1 reference model joint order does not match pinned GMR")
        if bodies != G1_LINK_BODY_NAMES:
            raise ValueError("G1 reference model body order does not match pinned TWIST2 motions")

        joint_ids = np.asarray(self.model.actuator_trnid[:, 0], dtype=int)
        self.joint_lower = np.asarray(self.model.jnt_range[joint_ids, 0], dtype=float)
        self.joint_upper = np.asarray(self.model.jnt_range[joint_ids, 1], dtype=float)
        self.viewer = None
        if viewer:
            viewer_module = importlib.import_module("mujoco.viewer")
            self.viewer = viewer_module.launch_passive(self.model, self.data)

    def _qpos(self, reference: G1KinematicReference | Any) -> np.ndarray:
        if isinstance(reference, G1KinematicReference):
            if reference.joint_names != G1_JOINT_NAMES:
                raise ValueError("G1 reference joint order is invalid")
            value = np.asarray(reference.qpos_wxyz, dtype=float)
        else:
            value = np.asarray(reference, dtype=float)
        if value.shape != (36,) or not np.isfinite(value).all():
            raise ValueError("G1 reference qpos must be a finite length-36 vector")
        if not np.isclose(np.linalg.norm(value[3:7]), 1.0, atol=1e-5):
            raise ValueError("G1 reference root quaternion must be normalized WXYZ")
        if np.any(value[7:] < self.joint_lower) or np.any(value[7:] > self.joint_upper):
            raise ValueError("G1 reference violates model joint limits")
        return value

    def apply(self, reference: G1KinematicReference | Any) -> np.ndarray:
        qpos = self._qpos(reference)
        self.data.qpos[:] = qpos
        self.data.qvel[:] = 0.0
        self.mujoco.mj_forward(self.model, self.data)
        if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.xpos).all():
            raise ValueError("G1 reference produced non-finite MuJoCo state")
        if self.viewer is not None:
            self.viewer.sync()
        return self.data.qpos.copy()

    def local_body_positions(self, reference: G1KinematicReference | Any) -> np.ndarray:
        qpos = self._qpos(reference)
        self.fk_data.qpos[:] = qpos
        self.fk_data.qpos[:3] = 0.0
        self.fk_data.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
        self.fk_data.qvel[:] = 0.0
        self.mujoco.mj_forward(self.model, self.fk_data)
        result = self.fk_data.xpos[1:].copy()
        if result.shape != (len(G1_LINK_BODY_NAMES), 3) or not np.isfinite(result).all():
            raise ValueError("G1 forward kinematics produced invalid body positions")
        return result

    def close(self) -> None:
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None

    @property
    def is_running(self) -> bool:
        return self.viewer is None or bool(self.viewer.is_running())


assert len(GMR_REQUIRED_BONES) == 14
assert len(G1_JOINT_NAMES) == 29
assert len(G1_LINK_BODY_NAMES) == 38
