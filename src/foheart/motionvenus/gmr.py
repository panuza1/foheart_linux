"""Direct MotionVenus solved-skeleton input for pinned GMR ``xsens_mvn``."""

from __future__ import annotations

import numpy as np

from foheart.mocap.frames import (
    BasisTransform,
    axis_rotation,
    matrix_to_quaternion,
    quaternion_to_matrix,
)
from foheart.mocap.sensor import Quaternion, Vector3
from foheart.whole_body.gmr import GMR_REQUIRED_BONES

from .skeleton import HumanSkeletonFrame


# MotionVenus SDK V4003 names -> pinned GMR xsens_mvn_to_g1.json names.
# T8 -> Chest and the underscore spellings also match GMR's own Xsens adapter.
MOTIONVENUS_TO_GMR_BONES = (
    ("Pelvis", "Pelvis"),
    ("T8", "Chest"),
    ("LeftUpperLeg", "Left_UpperLeg"),
    ("RightUpperLeg", "Right_UpperLeg"),
    ("LeftLowerLeg", "Left_LowerLeg"),
    ("RightLowerLeg", "Right_LowerLeg"),
    ("LeftFoot", "Left_Foot"),
    ("RightFoot", "Right_Foot"),
    ("LeftUpperArm", "Left_UpperArm"),
    ("RightUpperArm", "Right_UpperArm"),
    ("LeftForeArm", "Left_Forearm"),
    ("RightForeArm", "Right_Forearm"),
    ("LeftHand", "Left_Hand"),
    ("RightHand", "Right_Hand"),
)

# Authorized for software/synthetic validation only; live axes remain unvalidated.
MOTIONVENUS_TO_GMR_BASIS = BasisTransform(
    ((0.0, 1.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    "motionvenus_global",
    "gmr_xsens_mvn_global",
    "CONFIGURED",
)


class HeadingCalibration:
    """Remove only the first valid pelvis world-Z yaw from later frames."""

    def __init__(self, *, enabled: bool = True) -> None:
        if not isinstance(enabled, (bool, np.bool_)):
            raise TypeError("enabled must be bool")
        self.enabled = bool(enabled)
        self.initial_yaw_rad: float | None = None
        self._inverse_yaw: np.ndarray | None = None

    @property
    def calibrated(self) -> bool:
        return self._inverse_yaw is not None

    @property
    def status(self) -> str:
        return "SOFTWARE_CONFIGURED" if self.enabled else "DISABLED"

    def reset(self) -> None:
        self.initial_yaw_rad = None
        self._inverse_yaw = None

    def apply(self, human_data: dict[str, list[np.ndarray]]) -> dict[str, list[np.ndarray]]:
        if not self.enabled:
            return {
                name: [np.asarray(position, dtype=float).copy(), np.asarray(quaternion, dtype=float).copy()]
                for name, (position, quaternion) in human_data.items()
            }

        try:
            pelvis_quaternion = np.asarray(human_data["Pelvis"][1], dtype=float)
        except (KeyError, TypeError, ValueError, IndexError) as exc:
            raise ValueError("heading calibration requires a valid Pelvis pose") from exc
        if pelvis_quaternion.shape != (4,) or not np.isfinite(pelvis_quaternion).all():
            raise ValueError("heading calibration pelvis quaternion must be finite WXYZ")
        norm = float(np.linalg.norm(pelvis_quaternion))
        if norm < 1e-8:
            raise ValueError("heading calibration pelvis quaternion is near zero")
        pelvis_rotation = quaternion_to_matrix(
            Quaternion(tuple(map(float, pelvis_quaternion / norm)), "wxyz")
        )
        if self._inverse_yaw is None:
            self.initial_yaw_rad = float(np.arctan2(pelvis_rotation[1, 0], pelvis_rotation[0, 0]))
            self._inverse_yaw = axis_rotation("z", -float(np.degrees(self.initial_yaw_rad)))

        inverse_yaw = self._inverse_yaw
        normalized: dict[str, list[np.ndarray]] = {}
        for name, (position, quaternion) in human_data.items():
            rotation = quaternion_to_matrix(
                Quaternion(tuple(map(float, np.asarray(quaternion, dtype=float))), "wxyz")
            )
            normalized[name] = [
                inverse_yaw @ np.asarray(position, dtype=float),
                np.asarray(matrix_to_quaternion(inverse_yaw @ rotation).values),
            ]
        return normalized


class MotionVenusGMRAdapter:
    """Validate and convert one global 23-bone frame into GMR human data."""

    status = "SOFTWARE_CONFIGURED"

    def __init__(
        self,
        basis: BasisTransform = MOTIONVENUS_TO_GMR_BASIS,
        *,
        normalize_heading: bool = True,
    ) -> None:
        if not isinstance(basis, BasisTransform):
            raise TypeError("basis must be a BasisTransform")
        self.basis = basis
        self.heading_calibration = HeadingCalibration(enabled=normalize_heading)

    def reset_heading(self) -> None:
        self.heading_calibration.reset()

    def adapt(self, frame: HumanSkeletonFrame) -> dict[str, list[np.ndarray]]:
        if not isinstance(frame, HumanSkeletonFrame):
            raise TypeError("frame must be a HumanSkeletonFrame")
        if frame.stale:
            raise ValueError("stale MotionVenus frame")
        if not frame.valid:
            raise ValueError(f"invalid MotionVenus frame: {frame.reason or 'unspecified'}")
        if frame.source_coordinate != "global":
            raise ValueError("MotionVenus frame must contain global poses")

        missing = [source for source, _ in MOTIONVENUS_TO_GMR_BONES if source not in frame.bones]
        if missing:
            raise ValueError("MotionVenus frame is missing GMR bones: " + ",".join(missing))

        human_data: dict[str, list[np.ndarray]] = {}
        for source_name, target_name in MOTIONVENUS_TO_GMR_BONES:
            bone = frame.bones[source_name]
            position = np.asarray(bone.position_global_m, dtype=float)
            xyzw = np.asarray(bone.rotation_global_xyzw, dtype=float)
            if position.shape != (3,) or not np.isfinite(position).all():
                raise ValueError(f"MotionVenus {source_name} position must be finite XYZ metres")
            if xyzw.shape != (4,) or not np.isfinite(xyzw).all():
                raise ValueError(f"MotionVenus {source_name} quaternion must be finite XYZW")
            norm = float(np.linalg.norm(xyzw))
            if norm < 1e-8:
                raise ValueError(f"MotionVenus {source_name} quaternion is near zero")
            xyzw /= norm

            converted_position = self.basis.vector(Vector3(*map(float, position)))
            converted_quaternion = self.basis.orientation(
                Quaternion((float(xyzw[3]), float(xyzw[0]), float(xyzw[1]), float(xyzw[2])), "wxyz")
            )
            human_data[target_name] = [
                np.asarray((converted_position.x, converted_position.y, converted_position.z)),
                np.asarray(converted_quaternion.values),
            ]
        return self.heading_calibration.apply(human_data)


assert tuple(target for _, target in MOTIONVENUS_TO_GMR_BONES) == GMR_REQUIRED_BONES
