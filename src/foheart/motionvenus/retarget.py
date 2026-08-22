"""Solved MotionVenus skeleton -> shared G1 wrist-target pipeline."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Iterable, Mapping

import numpy as np
import yaml

from foheart.integrations.unitree_g1.adapter import G1FrameAdapter, G1WristTargets, UpperBodyTargetFilter
from foheart.mocap.frames import BasisTransform, homogeneous, quaternion_to_matrix
from foheart.mocap.sensor import Quaternion
from foheart.mocap.skeleton import UpperBodyTargets

from .skeleton import HumanSkeletonFrame


NEUTRAL_BONES = ("Pelvis", "T8", "LeftShoulder", "RightShoulder", "LeftHand", "RightHand")
PROFILE_STATUSES = ("SOFTWARE_CONFIGURED", "LIVE_VISUALLY_VALIDATED")


def _normal_xyzw(value, name: str) -> tuple[float, float, float, float]:
    array = np.asarray(value, dtype=float)
    if array.shape != (4,) or not np.isfinite(array).all() or np.linalg.norm(array) < 1e-8:
        raise ValueError(f"{name} must be a finite non-zero XYZW quaternion")
    array /= np.linalg.norm(array)
    return tuple(map(float, array))


def _position(value, name: str) -> tuple[float, float, float]:
    array = np.asarray(value, dtype=float)
    if array.shape != (3,) or not np.isfinite(array).all():
        raise ValueError(f"{name} must be a finite position in metres")
    return tuple(map(float, array))


def _rotation(value: tuple[float, float, float, float]) -> np.ndarray:
    x, y, z, w = value
    return quaternion_to_matrix(Quaternion((w, x, y, z), "wxyz"))


@dataclass(frozen=True)
class NeutralBone:
    position_m: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]

    def __post_init__(self) -> None:
        object.__setattr__(self, "position_m", _position(self.position_m, "neutral position"))
        object.__setattr__(
            self,
            "quaternion_xyzw",
            _normal_xyzw(self.quaternion_xyzw, "neutral quaternion"),
        )

    @property
    def pose(self) -> np.ndarray:
        return homogeneous(_rotation(self.quaternion_xyzw), np.asarray(self.position_m))


@dataclass(frozen=True)
class RetargetProfile:
    version: int
    status: str
    source_frame: str
    reference_bone: str
    motionvenus_to_project: BasisTransform
    project_to_g1: BasisTransform
    neutral: Mapping[str, NeutralBone]
    position_scale: float
    max_robot_reach_m: float
    workspace_radius_m: float
    position_alpha: float
    orientation_alpha: float
    max_translation_rate_m_s: float
    max_angular_rate_deg_s: float

    def __post_init__(self) -> None:
        if self.version != 1:
            raise ValueError("retarget profile version must be 1")
        if self.status not in PROFILE_STATUSES:
            raise ValueError(f"retarget profile status must be one of {PROFILE_STATUSES}")
        if self.source_frame != "motionvenus_global":
            raise ValueError("retarget source_frame must be motionvenus_global")
        missing = set(NEUTRAL_BONES) - set(self.neutral)
        if missing:
            raise ValueError("retarget neutral is missing: " + ", ".join(sorted(missing)))
        if self.reference_bone not in self.neutral:
            raise ValueError("retarget reference_bone must have a neutral transform")
        numeric = (
            self.position_scale,
            self.max_robot_reach_m,
            self.workspace_radius_m,
            self.position_alpha,
            self.orientation_alpha,
            self.max_translation_rate_m_s,
            self.max_angular_rate_deg_s,
        )
        if not np.isfinite(numeric).all():
            raise ValueError("retarget numeric settings must be finite")
        if min(self.position_scale, self.max_robot_reach_m, self.workspace_radius_m) <= 0:
            raise ValueError("retarget scale and workspaces must be positive")
        if not 0 < self.position_alpha <= 1 or not 0 < self.orientation_alpha <= 1:
            raise ValueError("retarget filter alphas must be in (0, 1]")
        if min(self.max_translation_rate_m_s, self.max_angular_rate_deg_s) <= 0:
            raise ValueError("retarget filter rates must be positive")
        object.__setattr__(self, "neutral", MappingProxyType(dict(self.neutral)))

    @classmethod
    def load(cls, path: str | Path) -> "RetargetProfile":
        try:
            data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        except (OSError, yaml.YAMLError) as exc:
            raise ValueError(f"could not load retarget profile {path}: {exc}") from exc
        if not isinstance(data, dict):
            raise ValueError("retarget profile root must be a mapping")
        frames = data.get("frames") or {}
        neutral_data = data.get("neutral") or {}
        filtering = data.get("filter") or {}
        safety = data.get("safety") or {}
        neutral = {
            name: NeutralBone(
                _position(value.get("position_m"), f"neutral.{name}.position_m"),
                _normal_xyzw(value.get("quaternion_xyzw"), f"neutral.{name}.quaternion_xyzw"),
            )
            for name, value in neutral_data.items()
            if isinstance(value, dict)
        }
        mv_basis = BasisTransform(
            tuple(tuple(map(float, row)) for row in frames.get("motionvenus_to_project", ())),
            "motionvenus_global",
            "project_human",
            "CONFIGURED",
        )
        g1_basis = BasisTransform(
            tuple(tuple(map(float, row)) for row in frames.get("project_to_g1", ())),
            "project_human",
            "g1_base",
            "CONFIGURED",
        )
        return cls(
            int(data.get("version", 0)),
            str(data.get("status", "")),
            str(data.get("source_frame", "")),
            str(data.get("reference_bone", "T8")),
            mv_basis,
            g1_basis,
            neutral,
            float(data.get("position_scale", 0)),
            float(safety.get("max_robot_reach_m", 0)),
            float(safety.get("workspace_radius_m", 0)),
            float(filtering.get("position_alpha", 0)),
            float(filtering.get("orientation_alpha", 0)),
            float(filtering.get("max_translation_rate_m_s", 0)),
            float(filtering.get("max_angular_rate_deg_s", 0)),
        )

    def save(self, path: str | Path) -> None:
        data = {
            "version": self.version,
            "status": self.status,
            "source_frame": self.source_frame,
            "reference_bone": self.reference_bone,
            "frames": {
                "motionvenus_to_project": [list(row) for row in self.motionvenus_to_project.matrix],
                "project_to_g1": [list(row) for row in self.project_to_g1.matrix],
            },
            "neutral": {
                name: {
                    "position_m": list(value.position_m),
                    "quaternion_xyzw": list(value.quaternion_xyzw),
                }
                for name, value in self.neutral.items()
            },
            "position_scale": self.position_scale,
            "filter": {
                "position_alpha": self.position_alpha,
                "orientation_alpha": self.orientation_alpha,
                "max_translation_rate_m_s": self.max_translation_rate_m_s,
                "max_angular_rate_deg_s": self.max_angular_rate_deg_s,
            },
            "safety": {
                "max_robot_reach_m": self.max_robot_reach_m,
                "workspace_radius_m": self.workspace_radius_m,
            },
        }
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        with destination.open("x", encoding="utf-8") as stream:
            yaml.safe_dump(data, stream, sort_keys=False)

    @classmethod
    def capture(
        cls,
        frames: Iterable[HumanSkeletonFrame],
        *,
        motionvenus_to_project: BasisTransform,
        project_to_g1: BasisTransform,
        position_scale: float = 0.65,
        max_robot_reach_m: float = 0.43,
        workspace_radius_m: float = 1.0,
    ) -> "RetargetProfile":
        samples = list(frames)
        if not samples:
            raise ValueError("retarget calibration requires at least one frame")
        neutral: dict[str, NeutralBone] = {}
        for name in NEUTRAL_BONES:
            positions, quaternions = [], []
            for frame in samples:
                if not frame.valid or frame.stale:
                    raise ValueError("retarget calibration frames must be valid and live")
                bone = frame.bone(name)
                if bone.position_global_m is None or bone.rotation_global_xyzw is None:
                    raise ValueError("retarget calibration requires global position+quaternion forwarding")
                positions.append(bone.position_global_m)
                quaternions.append(bone.rotation_global_xyzw)
            neutral[name] = NeutralBone(
                tuple(map(float, np.mean(np.asarray(positions), axis=0))),
                average_quaternions_xyzw(quaternions),
            )
        return cls(
            1, "SOFTWARE_CONFIGURED", "motionvenus_global", "T8",
            motionvenus_to_project, project_to_g1, neutral, position_scale,
            max_robot_reach_m, workspace_radius_m, 0.2, 0.2, 0.8, 180.0,
        )


def average_quaternions_xyzw(values: Iterable[tuple[float, float, float, float]]) -> tuple[float, float, float, float]:
    quaternions = np.asarray([_normal_xyzw(value, "calibration quaternion") for value in values])
    if not len(quaternions):
        raise ValueError("cannot average zero quaternions")
    reference = quaternions[0]
    quaternions[quaternions @ reference < 0] *= -1
    matrix = quaternions.T @ quaternions
    result = np.linalg.eigh(matrix)[1][:, -1]
    if result @ reference < 0:
        result *= -1
    return _normal_xyzw(result, "average quaternion")


def _relative_pose(pose: np.ndarray, reference: np.ndarray, basis: np.ndarray) -> np.ndarray:
    rotation = reference[:3, :3].T @ pose[:3, :3]
    position = reference[:3, :3].T @ (pose[:3, 3] - reference[:3, 3])
    return homogeneous(basis @ rotation @ basis.T, basis @ position)


def _targets_from_poses(
    poses: Mapping[str, np.ndarray],
    *,
    timestamp_ns: int,
    reference_bone: str,
    basis: BasisTransform,
) -> UpperBodyTargets:
    reference = poses[reference_bone]
    matrix = np.asarray(basis.matrix)
    converted = {
        name: _relative_pose(poses[name], reference, matrix)
        for name in ("LeftShoulder", "RightShoulder", "LeftHand", "RightHand")
    }
    return UpperBodyTargets(
        converted["LeftHand"], converted["RightHand"],
        converted["LeftShoulder"], converted["RightShoulder"],
        timestamp_ns, reference_frame="project_human_torso", units="meters",
    )


class MotionVenusG1Retargeter:
    """Task-space arm retargeting; no human Euler-to-motor-angle mapping."""

    def __init__(
        self,
        profile: RetargetProfile,
        *,
        robot_neutral_left: np.ndarray,
        robot_neutral_right: np.ndarray,
        robot_left_shoulder: np.ndarray,
        robot_right_shoulder: np.ndarray,
    ):
        self.profile = profile
        neutral_poses = {name: value.pose for name, value in profile.neutral.items()}
        human_neutral = _targets_from_poses(
            neutral_poses,
            timestamp_ns=0,
            reference_bone=profile.reference_bone,
            basis=profile.motionvenus_to_project,
        )
        self.filter = UpperBodyTargetFilter(
            position_alpha=profile.position_alpha,
            orientation_alpha=profile.orientation_alpha,
            max_translation_rate_m_s=profile.max_translation_rate_m_s,
            max_angular_rate_deg_s=profile.max_angular_rate_deg_s,
        )
        self.adapter = G1FrameAdapter(
            human_neutral,
            robot_neutral_left,
            robot_neutral_right,
            robot_left_shoulder,
            robot_right_shoulder,
            human_reach_m=1.0,
            robot_reach_m=profile.position_scale,
            max_robot_reach_m=profile.max_robot_reach_m,
            g1_from_human=profile.project_to_g1,
        )

    def extract_human_targets(self, frame: HumanSkeletonFrame) -> UpperBodyTargets:
        if not frame.valid or frame.stale or frame.status != "LIVE":
            raise ValueError(frame.reason or f"MotionVenus frame is not LIVE ({frame.status})")
        required = (
            self.profile.reference_bone,
            "LeftShoulder", "LeftUpperArm", "LeftForeArm", "LeftHand",
            "RightShoulder", "RightUpperArm", "RightForeArm", "RightHand",
        )
        poses: dict[str, np.ndarray] = {}
        for name in required:
            pose = frame.bone(name).pose_global
            if pose is None:
                raise ValueError("retargeting requires global position+quaternion forwarding")
            poses[name] = pose
        return _targets_from_poses(
            poses,
            timestamp_ns=frame.timestamp_ns,
            reference_bone=self.profile.reference_bone,
            basis=self.profile.motionvenus_to_project,
        )

    def retarget(self, frame: HumanSkeletonFrame) -> G1WristTargets:
        return self.adapter.adapt(self.filter.update(self.extract_human_targets(frame)))
