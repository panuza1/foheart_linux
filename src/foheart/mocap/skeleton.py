"""Robot-independent upper- and full-body orientation-driven kinematics."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Mapping

import numpy as np

from .frames import homogeneous, matrix_to_quaternion, quaternion_to_matrix, slerp
from .sensor import Quaternion
from .suit import FULL_BODY_ROLES, UPPER_BODY_ROLES


@dataclass(frozen=True)
class BodyDimensions:
    shoulder_width_m: float = 0.38
    left_upper_arm_m: float = 0.30
    left_forearm_m: float = 0.26
    left_hand_m: float = 0.10
    right_upper_arm_m: float = 0.30
    right_forearm_m: float = 0.26
    right_hand_m: float = 0.10
    status: str = "CONFIGURED"

    def __post_init__(self) -> None:
        lengths = self.__dict__.copy()
        lengths.pop("status")
        if any(not np.isfinite(value) or value <= 0 for value in lengths.values()):
            raise ValueError("all body dimensions must be positive finite meters")


@dataclass(frozen=True)
class UpperBodyPose:
    timestamp_ns: int
    poses: Mapping[str, np.ndarray]
    reference_frame: str = "human_torso"
    units: str = "meters"
    status: str = "CONFIGURED"

    @property
    def left_wrist_pose(self) -> np.ndarray:
        return self.poses["left_wrist"]

    @property
    def right_wrist_pose(self) -> np.ndarray:
        return self.poses["right_wrist"]


@dataclass(frozen=True)
class UpperBodyTargets:
    left_wrist_pose: np.ndarray
    right_wrist_pose: np.ndarray
    left_shoulder_pose: np.ndarray
    right_shoulder_pose: np.ndarray
    timestamp_ns: int
    valid: bool = True
    reason: str = ""
    reference_frame: str = "human_torso"
    units: str = "meters"


JOINT_NAMES = (
    "torso",
    "left_shoulder",
    "left_elbow",
    "left_wrist",
    "left_hand_end",
    "right_shoulder",
    "right_elbow",
    "right_wrist",
    "right_hand_end",
)


@dataclass(frozen=True)
class JointFrame:
    timestamp_ns: int
    joints: Mapping[str, np.ndarray]
    valid: bool = True
    reason: str = ""
    reference_frame: str = "human_torso"
    units: str = "meters"

    def __post_init__(self) -> None:
        if self.valid and set(self.joints) != set(JOINT_NAMES):
            raise ValueError("valid joint frame must contain the complete upper body")
        for name, value in self.joints.items():
            point = np.asarray(value, dtype=float)
            if name not in JOINT_NAMES or point.shape != (3,) or not np.isfinite(point).all():
                raise ValueError(f"invalid joint coordinate: {name}")

    def segment_lengths(self) -> dict[str, float]:
        lengths = {}
        for side in ("left", "right"):
            for segment, start, end in (
                ("upper_arm", "shoulder", "elbow"),
                ("forearm", "elbow", "wrist"),
                ("hand", "wrist", "hand_end"),
            ):
                left = self.joints.get(f"{side}_{start}")
                right = self.joints.get(f"{side}_{end}")
                if left is not None and right is not None:
                    lengths[f"{side}_{segment}"] = float(
                        np.linalg.norm(np.asarray(right) - np.asarray(left))
                    )
        return lengths


class UpperBodyKinematics:
    """Absolute segment orientations -> shoulder/elbow/wrist SE(3) poses."""

    BONE_AXIS = np.array((0.0, 0.0, -1.0))

    def __init__(self, dimensions: BodyDimensions = BodyDimensions()):
        self.dimensions = dimensions

    def solve(
        self, orientations: Mapping[str, Quaternion], timestamp_ns: int
    ) -> UpperBodyPose:
        missing = set(UPPER_BODY_ROLES) - set(orientations)
        if missing:
            raise ValueError(f"upper-body orientations missing: {', '.join(sorted(missing))}")
        torso = quaternion_to_matrix(orientations["torso"])
        relative = {
            role: torso.T @ quaternion_to_matrix(orientations[role])
            for role in UPPER_BODY_ROLES
        }
        poses: dict[str, np.ndarray] = {"torso": homogeneous(np.eye(3), np.zeros(3))}
        for side, sign in (("left", 1.0), ("right", -1.0)):
            shoulder = np.array((0.0, sign * self.dimensions.shoulder_width_m / 2, 0.0))
            upper_rotation = relative[f"{side}_upper_arm"]
            elbow = shoulder + upper_rotation @ self.BONE_AXIS * getattr(self.dimensions, f"{side}_upper_arm_m")
            forearm_rotation = relative[f"{side}_forearm"]
            wrist = elbow + forearm_rotation @ self.BONE_AXIS * getattr(self.dimensions, f"{side}_forearm_m")
            hand_rotation = relative[f"{side}_hand"]
            poses[f"{side}_shoulder"] = homogeneous(upper_rotation, shoulder)
            poses[f"{side}_elbow"] = homogeneous(forearm_rotation, elbow)
            poses[f"{side}_wrist"] = homogeneous(hand_rotation, wrist)
            hand_end = wrist + hand_rotation @ self.BONE_AXIS * getattr(self.dimensions, f"{side}_hand_m")
            poses[f"{side}_hand_end"] = homogeneous(hand_rotation, hand_end)
        return UpperBodyPose(timestamp_ns, poses)

    def targets(self, pose: UpperBodyPose) -> UpperBodyTargets:
        return UpperBodyTargets(
            pose.left_wrist_pose.copy(),
            pose.right_wrist_pose.copy(),
            pose.poses["left_shoulder"].copy(),
            pose.poses["right_shoulder"].copy(),
            pose.timestamp_ns,
            reference_frame=pose.reference_frame,
            units=pose.units,
        )

    def joints(self, pose: UpperBodyPose) -> JointFrame:
        return JointFrame(
            pose.timestamp_ns,
            {name: pose.poses[name][:3, 3].copy() for name in JOINT_NAMES},
            reference_frame=pose.reference_frame,
            units=pose.units,
        )

    def diagnose(self, frame: JointFrame, *, tolerance_m: float = 1e-7) -> tuple[str, ...]:
        if tolerance_m <= 0:
            raise ValueError("bone-length tolerance must be positive")
        expected = {
            f"{side}_{segment}": getattr(self.dimensions, f"{side}_{segment}_m")
            for side in ("left", "right")
            for segment in ("upper_arm", "forearm", "hand")
        }
        return tuple(
            f"{name} changed to {actual:.9f} m (expected {expected[name]:.9f} m)"
            for name, actual in frame.segment_lengths().items()
            if abs(actual - expected[name]) > tolerance_m
        )


@dataclass(frozen=True)
class FullBodyDimensions(BodyDimensions):
    """Configured diagnostic anthropometry; defaults are not user measurements."""

    torso_length_m: float = 0.50
    neck_length_m: float = 0.10
    head_length_m: float = 0.18
    hip_width_m: float = 0.30
    left_thigh_m: float = 0.42
    left_lower_leg_m: float = 0.43
    left_foot_m: float = 0.25
    right_thigh_m: float = 0.42
    right_lower_leg_m: float = 0.43
    right_foot_m: float = 0.25


FULL_BODY_JOINT_NAMES = (
    "pelvis",
    "lower_spine",
    "mid_spine",
    "torso",
    "neck",
    "head",
    "left_shoulder",
    "left_elbow",
    "left_wrist",
    "left_hand_end",
    "right_shoulder",
    "right_elbow",
    "right_wrist",
    "right_hand_end",
    "left_hip",
    "left_knee",
    "left_ankle",
    "left_foot_end",
    "right_hip",
    "right_knee",
    "right_ankle",
    "right_foot_end",
)

# A semantic compatibility vocabulary, not a reproduction of MotionVenus FK.
FULL_BODY_23_SEGMENTS = (
    "pelvis",
    "lower_spine",
    "mid_spine",
    "upper_spine",
    "chest",
    "neck",
    "head",
    "left_shoulder",
    "left_upper_arm",
    "left_forearm",
    "left_hand",
    "right_shoulder",
    "right_upper_arm",
    "right_forearm",
    "right_hand",
    "left_thigh",
    "left_lower_leg",
    "left_foot",
    "left_toe",
    "right_thigh",
    "right_lower_leg",
    "right_foot",
    "right_toe",
)

MEASUREMENT_STATUSES = frozenset(("MEASURED", "DERIVED", "CONFIGURED_OFFSET"))


@dataclass(frozen=True)
class FullBodyJointFrame:
    timestamp_ns: int
    joints: Mapping[str, np.ndarray]
    valid: bool = True
    reason: str = ""
    segment_orientations: Mapping[str, np.ndarray] = field(default_factory=dict)
    joint_status: Mapping[str, str] = field(default_factory=dict)
    segment_status: Mapping[str, str] = field(default_factory=dict)
    reference_frame: str = "human_pelvis"
    units: str = "meters"
    root_translation: str = "NOT_TRACKED_FIXED_ORIGIN"

    def __post_init__(self) -> None:
        object.__setattr__(
            self, "segment_orientations", dict(self.segment_orientations or {})
        )
        object.__setattr__(self, "joint_status", dict(self.joint_status or {}))
        object.__setattr__(self, "segment_status", dict(self.segment_status or {}))
        if self.valid and set(self.joints) != set(FULL_BODY_JOINT_NAMES):
            raise ValueError("valid full-body joint frame must contain every joint")
        if self.valid and set(self.segment_orientations) != set(FULL_BODY_ROLES):
            raise ValueError("valid full-body joint frame must contain 17 orientations")
        for name, value in self.joints.items():
            point = np.asarray(value, dtype=float)
            if (
                name not in FULL_BODY_JOINT_NAMES
                or point.shape != (3,)
                or not np.isfinite(point).all()
            ):
                raise ValueError(f"invalid full-body joint coordinate: {name}")
        if set(self.joint_status) - set(FULL_BODY_JOINT_NAMES):
            raise ValueError("full-body joint status contains an unknown joint")
        if set(self.segment_status) - set(FULL_BODY_23_SEGMENTS):
            raise ValueError("full-body segment status contains an unknown segment")
        if any(
            status not in MEASUREMENT_STATUSES
            for status in (*self.joint_status.values(), *self.segment_status.values())
        ):
            raise ValueError("unsupported full-body measurement status")
        for role, rotation in self.segment_orientations.items():
            value = np.asarray(rotation, dtype=float)
            if (
                role not in FULL_BODY_ROLES
                or value.shape != (3, 3)
                or not np.isfinite(value).all()
                or not np.allclose(value.T @ value, np.eye(3), atol=1e-7)
                or not np.isclose(np.linalg.det(value), 1.0, atol=1e-7)
            ):
                raise ValueError(f"invalid full-body segment orientation: {role}")

    def segment_lengths(self) -> dict[str, float]:
        pairs = {
            "lower_spine": ("pelvis", "lower_spine"),
            "mid_spine": ("lower_spine", "mid_spine"),
            "upper_spine": ("mid_spine", "torso"),
            "neck": ("torso", "neck"),
            "head": ("neck", "head"),
            "left_shoulder_offset": ("torso", "left_shoulder"),
            "right_shoulder_offset": ("torso", "right_shoulder"),
            "left_hip_offset": ("pelvis", "left_hip"),
            "right_hip_offset": ("pelvis", "right_hip"),
        }
        for side in ("left", "right"):
            pairs.update(
                {
                    f"{side}_upper_arm": (f"{side}_shoulder", f"{side}_elbow"),
                    f"{side}_forearm": (f"{side}_elbow", f"{side}_wrist"),
                    f"{side}_hand": (f"{side}_wrist", f"{side}_hand_end"),
                    f"{side}_thigh": (f"{side}_hip", f"{side}_knee"),
                    f"{side}_lower_leg": (f"{side}_knee", f"{side}_ankle"),
                    f"{side}_foot": (f"{side}_ankle", f"{side}_foot_end"),
                }
            )
        return {
            name: float(np.linalg.norm(self.joints[end] - self.joints[start]))
            for name, (start, end) in pairs.items()
            if start in self.joints and end in self.joints
        }


class FullBodyKinematics:
    """Seventeen calibrated orientations -> fixed-root diagnostic skeleton."""

    DOWN = np.array((0.0, 0.0, -1.0))
    UP = np.array((0.0, 0.0, 1.0))
    FORWARD = np.array((1.0, 0.0, 0.0))

    def __init__(self, dimensions: FullBodyDimensions = FullBodyDimensions()):
        self.dimensions = dimensions

    def solve(
        self, orientations: Mapping[str, Quaternion], timestamp_ns: int
    ) -> FullBodyJointFrame:
        missing = set(FULL_BODY_ROLES) - set(orientations)
        if missing:
            raise ValueError(
                "full-body orientations missing: " + ", ".join(sorted(missing))
            )
        pelvis_world = quaternion_to_matrix(orientations["pelvis"])
        relative = {
            role: pelvis_world.T @ quaternion_to_matrix(orientations[role])
            for role in FULL_BODY_ROLES
        }
        identity = Quaternion((1.0, 0.0, 0.0, 0.0), "wxyz")
        torso_q = matrix_to_quaternion(relative["torso"])
        spine_rotations = tuple(
            quaternion_to_matrix(slerp(identity, torso_q, amount))
            for amount in (1 / 3, 2 / 3, 1.0)
        )
        points: dict[str, np.ndarray] = {"pelvis": np.zeros(3)}
        spine_step = self.dimensions.torso_length_m / 3
        points["lower_spine"] = points["pelvis"] + spine_rotations[0] @ self.UP * spine_step
        points["mid_spine"] = points["lower_spine"] + spine_rotations[1] @ self.UP * spine_step
        points["torso"] = points["mid_spine"] + spine_rotations[2] @ self.UP * spine_step
        points["neck"] = (
            points["torso"]
            + relative["torso"] @ self.UP * self.dimensions.neck_length_m
        )
        points["head"] = (
            points["neck"]
            + relative["head"] @ self.UP * self.dimensions.head_length_m
        )

        for side, sign in (("left", 1.0), ("right", -1.0)):
            shoulder = points["torso"] + relative[f"{side}_shoulder"] @ np.array(
                (0.0, sign * self.dimensions.shoulder_width_m / 2, 0.0)
            )
            elbow = (
                shoulder
                + relative[f"{side}_upper_arm"]
                @ self.DOWN
                * getattr(self.dimensions, f"{side}_upper_arm_m")
            )
            wrist = (
                elbow
                + relative[f"{side}_forearm"]
                @ self.DOWN
                * getattr(self.dimensions, f"{side}_forearm_m")
            )
            hand_end = (
                wrist
                + relative[f"{side}_hand"]
                @ self.DOWN
                * getattr(self.dimensions, f"{side}_hand_m")
            )
            hip = points["pelvis"] + np.array(
                (0.0, sign * self.dimensions.hip_width_m / 2, 0.0)
            )
            knee = (
                hip
                + relative[f"{side}_thigh"]
                @ self.DOWN
                * getattr(self.dimensions, f"{side}_thigh_m")
            )
            ankle = (
                knee
                + relative[f"{side}_lower_leg"]
                @ self.DOWN
                * getattr(self.dimensions, f"{side}_lower_leg_m")
            )
            foot_end = (
                ankle
                + relative[f"{side}_foot"]
                @ self.FORWARD
                * getattr(self.dimensions, f"{side}_foot_m")
            )
            points.update(
                {
                    f"{side}_shoulder": shoulder,
                    f"{side}_elbow": elbow,
                    f"{side}_wrist": wrist,
                    f"{side}_hand_end": hand_end,
                    f"{side}_hip": hip,
                    f"{side}_knee": knee,
                    f"{side}_ankle": ankle,
                    f"{side}_foot_end": foot_end,
                }
            )

        joint_status = {
            name: (
                "CONFIGURED_OFFSET"
                if name
                in {
                    "pelvis",
                    "left_shoulder",
                    "right_shoulder",
                    "left_hip",
                    "right_hip",
                }
                else "DERIVED"
            )
            for name in FULL_BODY_JOINT_NAMES
        }
        measured_segments = {
            "head",
            "left_shoulder",
            "right_shoulder",
            "chest",
            "pelvis",
            "left_upper_arm",
            "right_upper_arm",
            "left_forearm",
            "right_forearm",
            "left_hand",
            "right_hand",
            "left_thigh",
            "right_thigh",
            "left_lower_leg",
            "right_lower_leg",
            "left_foot",
            "right_foot",
        }
        segment_status = {
            name: "MEASURED" if name in measured_segments else "DERIVED"
            for name in FULL_BODY_23_SEGMENTS
        }
        return FullBodyJointFrame(
            timestamp_ns,
            points,
            segment_orientations=relative,
            joint_status=joint_status,
            segment_status=segment_status,
        )

    def diagnose(
        self, frame: FullBodyJointFrame, *, tolerance_m: float = 1e-7
    ) -> tuple[str, ...]:
        if tolerance_m <= 0:
            raise ValueError("bone-length tolerance must be positive")
        d = self.dimensions
        expected = {
            "lower_spine": d.torso_length_m / 3,
            "mid_spine": d.torso_length_m / 3,
            "upper_spine": d.torso_length_m / 3,
            "neck": d.neck_length_m,
            "head": d.head_length_m,
            "left_shoulder_offset": d.shoulder_width_m / 2,
            "right_shoulder_offset": d.shoulder_width_m / 2,
            "left_hip_offset": d.hip_width_m / 2,
            "right_hip_offset": d.hip_width_m / 2,
        }
        expected.update(
            {
                f"{side}_{segment}": getattr(d, f"{side}_{segment}_m")
                for side in ("left", "right")
                for segment in (
                    "upper_arm",
                    "forearm",
                    "hand",
                    "thigh",
                    "lower_leg",
                    "foot",
                )
            }
        )
        return tuple(
            f"{name} changed to {actual:.9f} m (expected {expected[name]:.9f} m)"
            for name, actual in frame.segment_lengths().items()
            if abs(actual - expected[name]) > tolerance_m
        )
