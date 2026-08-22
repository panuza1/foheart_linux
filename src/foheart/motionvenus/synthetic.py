"""Deterministic solved-skeleton poses at the MotionVenus frame boundary."""

from __future__ import annotations

import math
import time

import numpy as np

from .protocol import BODY_BONE_NAMES, MotionVenusBone, MotionVenusFrame, MotionVenusHeader


SYNTHETIC_POSES = (
    "neutral",
    "arms_forward",
    "t_pose",
    "left_elbow_flex",
    "right_elbow_flex",
    "left_arm_raise",
    "right_arm_raise",
    "symmetric_reach",
    "wrist_rotations",
    "torso_yaw",
)

GMR_SYNTHETIC_POSES = (
    "neutral",
    "t_pose",
    "arms_forward",
    "left_arm_raise",
    "right_arm_raise",
    "torso_yaw",
    "squat",
    "left_leg_raise",
    "right_leg_raise",
    "weight_shift",
)


def _axis_angle_xyzw(axis: tuple[float, float, float], degrees: float) -> tuple[float, float, float, float]:
    vector = np.asarray(axis, dtype=float)
    vector /= np.linalg.norm(vector)
    half = math.radians(degrees) / 2
    xyz = vector * math.sin(half)
    return float(xyz[0]), float(xyz[1]), float(xyz[2]), math.cos(half)


def _base_positions() -> dict[str, np.ndarray]:
    return {
        "Pelvis": np.array((0.0, 0.0, 1.00)),
        "L5": np.array((0.0, 0.0, 1.10)),
        "L3": np.array((0.0, 0.0, 1.20)),
        "T12": np.array((0.0, 0.0, 1.32)),
        "T8": np.array((0.0, 0.0, 1.45)),
        "Neck": np.array((0.0, 0.0, 1.58)),
        "Head": np.array((0.0, 0.0, 1.73)),
        "RightShoulder": np.array((0.12, 0.0, 1.48)),
        "RightUpperArm": np.array((0.22, 0.0, 1.45)),
        "RightForeArm": np.array((0.30, 0.0, 1.20)),
        "RightHand": np.array((0.32, 0.0, 0.95)),
        "LeftShoulder": np.array((-0.12, 0.0, 1.48)),
        "LeftUpperArm": np.array((-0.22, 0.0, 1.45)),
        "LeftForeArm": np.array((-0.30, 0.0, 1.20)),
        "LeftHand": np.array((-0.32, 0.0, 0.95)),
        "RightUpperLeg": np.array((0.10, 0.0, 0.93)),
        "RightLowerLeg": np.array((0.10, 0.0, 0.52)),
        "RightFoot": np.array((0.10, 0.04, 0.10)),
        "RightToe": np.array((0.10, 0.22, 0.06)),
        "LeftUpperLeg": np.array((-0.10, 0.0, 0.93)),
        "LeftLowerLeg": np.array((-0.10, 0.0, 0.52)),
        "LeftFoot": np.array((-0.10, 0.04, 0.10)),
        "LeftToe": np.array((-0.10, 0.22, 0.06)),
    }


def _set_arm(positions: dict[str, np.ndarray], side: str, upper, forearm, hand) -> None:
    positions[f"{side}UpperArm"] = np.asarray(upper, dtype=float)
    positions[f"{side}ForeArm"] = np.asarray(forearm, dtype=float)
    positions[f"{side}Hand"] = np.asarray(hand, dtype=float)


def synthetic_frame(
    pose: str = "neutral",
    *,
    frame_number: int = 0,
    timestamp_ns: int | None = None,
    avatar: str = "SyntheticActor",
) -> MotionVenusFrame:
    if pose not in set(SYNTHETIC_POSES) | set(GMR_SYNTHETIC_POSES):
        raise ValueError(f"unknown synthetic MotionVenus pose {pose!r}")
    positions = _base_positions()
    rotations = {name: (0.0, 0.0, 0.0, 1.0) for name in BODY_BONE_NAMES}
    if pose == "arms_forward":
        _set_arm(positions, "Right", (0.14, 0.12, 1.47), (0.14, 0.37, 1.43), (0.14, 0.62, 1.40))
        _set_arm(positions, "Left", (-0.14, 0.12, 1.47), (-0.14, 0.37, 1.43), (-0.14, 0.62, 1.40))
    elif pose == "t_pose":
        _set_arm(positions, "Right", (0.22, 0, 1.48), (0.48, 0, 1.48), (0.73, 0, 1.48))
        _set_arm(positions, "Left", (-0.22, 0, 1.48), (-0.48, 0, 1.48), (-0.73, 0, 1.48))
    elif pose == "left_elbow_flex":
        _set_arm(positions, "Left", (-0.22, 0, 1.48), (-0.47, 0, 1.48), (-0.47, 0.25, 1.48))
    elif pose == "right_elbow_flex":
        _set_arm(positions, "Right", (0.22, 0, 1.48), (0.47, 0, 1.48), (0.47, 0.25, 1.48))
    elif pose == "left_arm_raise":
        _set_arm(positions, "Left", (-0.18, 0, 1.58), (-0.14, 0, 1.84), (-0.10, 0, 2.08))
    elif pose == "right_arm_raise":
        _set_arm(positions, "Right", (0.18, 0, 1.58), (0.14, 0, 1.84), (0.10, 0, 2.08))
    elif pose == "symmetric_reach":
        _set_arm(positions, "Right", (0.17, 0.10, 1.46), (0.22, 0.31, 1.38), (0.24, 0.49, 1.32))
        _set_arm(positions, "Left", (-0.17, 0.10, 1.46), (-0.22, 0.31, 1.38), (-0.24, 0.49, 1.32))
    elif pose == "wrist_rotations":
        rotations["LeftHand"] = _axis_angle_xyzw((0, 1, 0), 35)
        rotations["RightHand"] = _axis_angle_xyzw((0, 1, 0), -35)
    elif pose == "torso_yaw":
        angle = math.radians(25)
        rotation = np.array(((math.cos(angle), -math.sin(angle), 0), (math.sin(angle), math.cos(angle), 0), (0, 0, 1)))
        pivot = positions["Pelvis"].copy()
        for name in positions:
            positions[name] = pivot + rotation @ (positions[name] - pivot)
            rotations[name] = _axis_angle_xyzw((0, 0, 1), 25)
    elif pose == "squat":
        fixed = {"LeftFoot", "LeftToe", "RightFoot", "RightToe"}
        for name in positions:
            if name not in fixed:
                positions[name] -= (0.0, 0.0, 0.25)
        for side, x in (("Left", -0.10), ("Right", 0.10)):
            positions[f"{side}UpperLeg"] = (x, 0.08, 0.70)
            positions[f"{side}LowerLeg"] = (x, 0.20, 0.40)
    elif pose in ("left_leg_raise", "right_leg_raise"):
        side = "Left" if pose.startswith("left") else "Right"
        x = -0.10 if side == "Left" else 0.10
        positions[f"{side}UpperLeg"] = (x, 0.18, 1.05)
        positions[f"{side}LowerLeg"] = (x, 0.35, 0.78)
        positions[f"{side}Foot"] = (x, 0.48, 0.48)
        positions[f"{side}Toe"] = (x, 0.62, 0.45)
    elif pose == "weight_shift":
        fixed = {"LeftFoot", "LeftToe", "RightFoot", "RightToe"}
        for name in positions:
            if name not in fixed:
                positions[name] += (-0.08, 0.0, 0.0)
    timestamp = time.time_ns() if timestamp_ns is None else timestamp_ns
    avatar_raw = avatar.encode("utf-8")
    header = MotionVenusHeader(
        4003, avatar, avatar_raw, 0, 0, frame_number & 0xFFFFFFFF,
        23, 0, 0, "binary", "meter", "quaternion", "global", "XYZ",
        hip_height_m=1.0,
    )
    bones = tuple(
        MotionVenusBone(index, name, tuple(map(float, positions[name])), rotations[name])
        for index, name in enumerate(BODY_BONE_NAMES)
    )
    return MotionVenusFrame(header, bones, timestamp, ("127.0.0.1", 5001), 0, timestamp)


class SyntheticMotionVenusSource:
    def __init__(self, *, fps: float = 60.0, poses: tuple[str, ...] = SYNTHETIC_POSES):
        if fps <= 0 or not poses:
            raise ValueError("synthetic source requires positive fps and at least one pose")
        self.fps, self.poses = fps, poses
        self.frame_number = 0
        self.started_ns: int | None = None

    def start(self) -> None:
        self.frame_number = 0
        self.started_ns = time.time_ns()

    def receive(self) -> MotionVenusFrame:
        if self.started_ns is None:
            raise RuntimeError("synthetic source is not started")
        timestamp = self.started_ns + round(self.frame_number * 1e9 / self.fps)
        frame = synthetic_frame(
            self.poses[self.frame_number % len(self.poses)],
            frame_number=self.frame_number,
            timestamp_ns=timestamp,
        )
        self.frame_number += 1
        return frame

    def close(self) -> None:
        self.started_ns = None
