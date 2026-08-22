"""Thin MotionVenus solved-skeleton reader for HumDex/GMR."""

from __future__ import annotations

from typing import Any

import numpy as np

from foheart.mocap.frames import BasisTransform
from foheart.mocap.sensor import Quaternion, Vector3
from foheart.motionvenus.protocol import MotionVenusFrame
from foheart.motionvenus.skeleton import HumanSkeletonFrame


# Exact pinned HumDex XsensBodyReader.to_gmr_human_frame spelling and order.
MOTIONVENUS_TO_XSENS_MVN_BONES = (
    ("Pelvis", "Pelvis"),
    ("L5", "Spine"),
    ("L3", "Spine1"),
    ("T12", "Spine2"),
    ("T8", "Chest"),
    ("Neck", "Neck"),
    ("Head", "Head"),
    ("RightShoulder", "Right_Shoulder"),
    ("RightUpperArm", "Right_UpperArm"),
    ("RightForeArm", "Right_Forearm"),
    ("RightHand", "Right_Hand"),
    ("LeftShoulder", "Left_Shoulder"),
    ("LeftUpperArm", "Left_UpperArm"),
    ("LeftForeArm", "Left_Forearm"),
    ("LeftHand", "Left_Hand"),
    ("RightUpperLeg", "Right_UpperLeg"),
    ("RightLowerLeg", "Right_LowerLeg"),
    ("RightFoot", "Right_Foot"),
    ("RightToe", "Right_Toe"),
    ("LeftUpperLeg", "Left_UpperLeg"),
    ("LeftLowerLeg", "Left_LowerLeg"),
    ("LeftFoot", "Left_Foot"),
    ("LeftToe", "Left_Toe"),
)

# Authorized for offline implementation; physical axes remain unvalidated.
MOTIONVENUS_TO_GMR_XSENS_BASIS = BasisTransform(
    ((0.0, 1.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    "motionvenus_global",
    "gmr_xsens_mvn_global",
    "CONFIGURED",
)


class MotionVenusHumDexAdapter:
    """Expose an existing MotionVenus source through HumDex's body-reader API."""

    def __init__(self, source: Any):
        self.source = source
        self._initialized = False
        self._last_frame_index: int | None = None
        self._last_timestamp_ns: int | None = None
        self._last_sender: tuple[str, int] | None = None

    def initialize(self) -> None:
        for name in ("start", "receive", "close"):
            if not callable(getattr(self.source, name, None)):
                raise TypeError(f"MotionVenus source must provide {name}()")
        self.source.start()
        self._last_frame_index = self._last_timestamp_ns = None
        self._last_sender = None
        self._initialized = True

    def read_frame(self) -> dict[str, Any]:
        if not self._initialized:
            return {"ok": False, "reason": "not_initialized"}
        try:
            value = self.source.receive()
        except Exception as exc:
            return {"ok": False, "reason": f"source_error:{type(exc).__name__}:{exc}"}
        if value is None:
            return {"ok": False, "reason": "no_update"}
        if isinstance(value, MotionVenusFrame):
            frame = HumanSkeletonFrame.from_motionvenus(value, status="LIVE")
        elif isinstance(value, HumanSkeletonFrame):
            frame = value
        else:
            return {"ok": False, "reason": f"unsupported_frame:{type(value).__name__}"}
        return self._adapt(frame)

    def _adapt(self, frame: HumanSkeletonFrame) -> dict[str, Any]:
        metadata = self._metadata(frame)
        missing = [source for source, _ in MOTIONVENUS_TO_XSENS_MVN_BONES if source not in frame.bones]
        if missing:
            return self._failure("missing_bones:" + ",".join(missing), metadata)
        if frame.stale:
            return self._failure("stale_source", metadata)
        if frame.status != "LIVE":
            return self._failure(f"source_status:{frame.status}", metadata)
        if not frame.valid:
            return self._failure("invalid_source:" + (frame.reason or "unspecified"), metadata)
        if frame.source_coordinate != "global":
            return self._failure(f"source_coordinate:{frame.source_coordinate}", metadata)

        frame_index = frame.motionvenus_frame_number
        if not isinstance(frame_index, int) or not 0 <= frame_index <= 0xFFFFFFFF:
            return self._failure("invalid_frame_index", metadata)
        if not isinstance(frame.timestamp_ns, int) or frame.timestamp_ns < 0:
            return self._failure("invalid_timestamp", metadata)
        if self._last_sender == frame.sender and self._last_frame_index is not None:
            delta = (frame_index - self._last_frame_index) & 0xFFFFFFFF
            if delta == 0:
                return self._failure("duplicate_frame", metadata)
            if delta >= 0x80000000:
                return self._failure("out_of_order_frame", metadata)
        if self._last_timestamp_ns is not None and frame.timestamp_ns <= self._last_timestamp_ns:
            return self._failure("non_monotonic_timestamp", metadata)

        body_frame: dict[str, list[np.ndarray]] = {}
        try:
            for source_name, target_name in MOTIONVENUS_TO_XSENS_MVN_BONES:
                bone = frame.bones[source_name]
                if bone.position_global_m is None or bone.rotation_global_xyzw is None:
                    raise ValueError(f"missing_global_pose:{source_name}")
                position = np.asarray(bone.position_global_m, dtype=float)
                xyzw = np.asarray(bone.rotation_global_xyzw, dtype=float)
                if position.shape != (3,) or not np.isfinite(position).all():
                    raise ValueError(f"invalid_position:{source_name}")
                if xyzw.shape != (4,) or not np.isfinite(xyzw).all() or np.linalg.norm(xyzw) < 1e-8:
                    raise ValueError(f"invalid_quaternion:{source_name}")
                wxyz = Quaternion((float(xyzw[3]), float(xyzw[0]), float(xyzw[1]), float(xyzw[2])), "wxyz")
                converted_position = MOTIONVENUS_TO_GMR_XSENS_BASIS.vector(Vector3(*map(float, position)))
                converted_quaternion = MOTIONVENUS_TO_GMR_XSENS_BASIS.orientation(wxyz)
                body_frame[target_name] = [
                    np.asarray((converted_position.x, converted_position.y, converted_position.z), dtype=np.float32),
                    np.asarray(converted_quaternion.values, dtype=np.float32),
                ]
        except ValueError as exc:
            return self._failure(str(exc), metadata)

        self._last_frame_index = frame_index
        self._last_timestamp_ns = frame.timestamp_ns
        self._last_sender = frame.sender
        return {
            "ok": True,
            "frame_index": frame_index,
            "body_frame": body_frame,
            "source_metadata": metadata,
        }

    @staticmethod
    def _metadata(frame: HumanSkeletonFrame) -> dict[str, Any]:
        return {
            "motionvenus_frame_number": frame.motionvenus_frame_number,
            "host_timestamp_ns": frame.timestamp_ns,
            "suit_number": frame.suit_number,
            "avatar": frame.avatar,
            "valid": frame.valid,
            "stale": frame.stale,
            "status": frame.status,
            "reason": frame.reason,
            "sender": frame.sender,
            "source_format": frame.source_format,
            "source_coordinate": frame.source_coordinate,
            "source_bone_names": tuple(frame.bones),
            "bone_mapping": MOTIONVENUS_TO_XSENS_MVN_BONES,
            "coordinate_mapping_status": "SOFTWARE_CONFIGURED",
        }

    @staticmethod
    def _failure(reason: str, metadata: dict[str, Any]) -> dict[str, Any]:
        return {"ok": False, "reason": reason, "source_metadata": metadata}

    def close(self) -> None:
        try:
            if self._initialized:
                self.source.close()
        finally:
            self._initialized = False
            self._last_frame_index = self._last_timestamp_ns = None
            self._last_sender = None


assert len(MOTIONVENUS_TO_XSENS_MVN_BONES) == 23
