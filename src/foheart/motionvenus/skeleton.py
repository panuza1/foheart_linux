"""Robot-independent solved human skeleton model."""

from __future__ import annotations

from dataclasses import dataclass
from types import MappingProxyType
from typing import Mapping

import numpy as np

from foheart.mocap.frames import homogeneous, quaternion_to_matrix
from foheart.mocap.sensor import Quaternion

from .protocol import BODY_BONE_NAMES, MotionVenusBone, MotionVenusFrame


BODY_BONE_EDGES = (
    ("Pelvis", "L5"), ("L5", "L3"), ("L3", "T12"), ("T12", "T8"),
    ("T8", "Neck"), ("Neck", "Head"),
    ("T8", "RightShoulder"), ("RightShoulder", "RightUpperArm"),
    ("RightUpperArm", "RightForeArm"), ("RightForeArm", "RightHand"),
    ("T8", "LeftShoulder"), ("LeftShoulder", "LeftUpperArm"),
    ("LeftUpperArm", "LeftForeArm"), ("LeftForeArm", "LeftHand"),
    ("Pelvis", "RightUpperLeg"), ("RightUpperLeg", "RightLowerLeg"),
    ("RightLowerLeg", "RightFoot"), ("RightFoot", "RightToe"),
    ("Pelvis", "LeftUpperLeg"), ("LeftUpperLeg", "LeftLowerLeg"),
    ("LeftLowerLeg", "LeftFoot"), ("LeftFoot", "LeftToe"),
)


def _rotation_xyzw(value: tuple[float, float, float, float]) -> np.ndarray:
    x, y, z, w = value
    return quaternion_to_matrix(Quaternion((w, x, y, z), "wxyz"))


@dataclass(frozen=True)
class HumanBone:
    index: int
    name: str
    position_global_m: tuple[float, float, float] | None
    rotation_global_xyzw: tuple[float, float, float, float] | None
    rotation_local_xyzw: tuple[float, float, float, float] | None
    euler_global_deg: tuple[float, float, float] | None
    euler_local_deg: tuple[float, float, float] | None

    @classmethod
    def from_motionvenus(cls, bone: MotionVenusBone) -> "HumanBone":
        return cls(
            bone.index,
            bone.name,
            bone.position_global_m,
            bone.rotation_global_xyzw,
            bone.rotation_local_xyzw,
            bone.euler_global_deg,
            bone.euler_local_deg,
        )

    @property
    def pose_global(self) -> np.ndarray | None:
        if self.position_global_m is None or self.rotation_global_xyzw is None:
            return None
        return homogeneous(_rotation_xyzw(self.rotation_global_xyzw), np.asarray(self.position_global_m))

    @property
    def rotation_local(self) -> np.ndarray | None:
        return _rotation_xyzw(self.rotation_local_xyzw) if self.rotation_local_xyzw else None


@dataclass(frozen=True)
class HumanSkeletonFrame:
    timestamp_ns: int
    motionvenus_frame_number: int
    suit_number: int
    avatar: str
    bones: Mapping[str, HumanBone]
    valid: bool
    stale: bool
    status: str
    reason: str
    sender: tuple[str, int]
    source_format: str
    source_coordinate: str

    def __post_init__(self) -> None:
        object.__setattr__(self, "bones", MappingProxyType(dict(self.bones)))

    @classmethod
    def from_motionvenus(
        cls,
        frame: MotionVenusFrame,
        *,
        stale: bool = False,
        status: str = "LIVE",
    ) -> "HumanSkeletonFrame":
        source = {bone.name: HumanBone.from_motionvenus(bone) for bone in frame.bones}
        missing = tuple(name for name in BODY_BONE_NAMES if name not in source)
        positions_missing = tuple(
            name for name in BODY_BONE_NAMES if name in source and source[name].position_global_m is None
        )
        valid = not stale and not missing and not positions_missing
        reasons = []
        if missing:
            reasons.append("missing bones: " + ", ".join(missing))
        if positions_missing:
            reasons.append("missing positions: " + ", ".join(positions_missing))
        if stale:
            reasons.append("MotionVenus input is stale")
        return cls(
            frame.received_ns,
            frame.header.frame_number,
            frame.header.suit_number,
            frame.header.avatar_name,
            {name: source[name] for name in BODY_BONE_NAMES if name in source},
            valid,
            stale,
            status,
            "; ".join(reasons),
            frame.sender,
            frame.header.stream_format,
            frame.header.skeleton_coordinate,
        )

    def bone(self, name: str) -> HumanBone:
        try:
            return self.bones[name]
        except KeyError as exc:
            raise KeyError(f"human skeleton has no bone {name!r}") from exc

