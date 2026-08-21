"""Deterministic sensor orientations for upper- and full-body validation."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .frames import axis_rotation, matrix_to_quaternion
from .sensor import SensorSample
from .suit import (
    BodyProfile,
    BodySensorMap,
    FULL_BODY_ROLES,
    SuitFrame,
    TimedSensorSample,
    UPPER_BODY_ROLES,
    body_profile,
)

SYNTHETIC_BODY_MAP = BodySensorMap(
    {role: f"slot_{index}" for index, role in enumerate(UPPER_BODY_ROLES)}
)
SYNTHETIC_FULL_BODY_MAP = BodySensorMap(
    {role: f"slot_{index}" for index, role in enumerate(FULL_BODY_ROLES)},
    profile=BodyProfile.FULL,
)

SYNTHETIC_LIVE_MOTIONS = (
    "neutral",
    "arms_forward",
    "t_pose",
    "left_elbow_flex",
    "right_elbow_flex",
    "left_arm_raise_lower",
    "right_arm_raise_lower",
    "torso_yaw",
    "symmetric_reach",
    "slow_continuous_arm_wave",
)

SYNTHETIC_FULL_BODY_MOTIONS = (
    "neutral_standing",
    "t_pose",
    "left_arm_raise",
    "right_arm_raise",
    "left_elbow_flex",
    "right_elbow_flex",
    "head_yaw",
    "torso_yaw",
    "pelvis_yaw",
    "left_hip_flex",
    "right_hip_flex",
    "left_knee_bend",
    "right_knee_bend",
    "left_foot_pitch",
    "right_foot_pitch",
    "left_leg_side_lift",
    "right_leg_side_lift",
    "squat",
    "slow_whole_body_movement",
)


@dataclass(frozen=True)
class SyntheticPose:
    name: str
    frame: SuitFrame


def _arm_rotations(name: str, step: int) -> dict[str, np.ndarray]:
    identity = np.eye(3)
    result = {role: identity for role in UPPER_BODY_ROLES}
    if name == "arms_forward":
        for role in UPPER_BODY_ROLES[1:]:
            result[role] = axis_rotation("y", -65)
    elif name == "t_pose":
        for side, angle in (("left", 90), ("right", -90)):
            for segment in ("upper_arm", "forearm", "hand"):
                result[f"{side}_{segment}"] = axis_rotation("x", angle)
    elif name == "left_elbow_bend":
        for segment in ("upper_arm",):
            result[f"left_{segment}"] = axis_rotation("x", 90)
        result["left_forearm"] = identity
        result["left_hand"] = identity
    elif name == "right_elbow_bend":
        result["right_upper_arm"] = axis_rotation("x", -90)
        result["right_forearm"] = identity
        result["right_hand"] = identity
    elif name == "symmetric_reach":
        for role in UPPER_BODY_ROLES[1:]:
            result[role] = axis_rotation("y", -45)
    elif name == "wrist_circle":
        phase = step * 15.0
        left = axis_rotation("z", phase) @ axis_rotation("y", -45)
        right = axis_rotation("z", -phase) @ axis_rotation("y", -45)
        for segment in ("upper_arm", "forearm", "hand"):
            result[f"left_{segment}"] = left
            result[f"right_{segment}"] = right
    return result


def synthetic_upper_body_sequence(start_timestamp_ns: int = 1_000_000_000) -> tuple[SyntheticPose, ...]:
    names = (
        "neutral",
        "arms_forward",
        "t_pose",
        "left_elbow_bend",
        "right_elbow_bend",
        "symmetric_reach",
        "wrist_circle",
    )
    poses = []
    for step, name in enumerate(names):
        timestamp = start_timestamp_ns + step * 100_000_000
        rotations = _arm_rotations(name, step)
        samples = {}
        for sensor_id, role in enumerate(UPPER_BODY_ROLES):
            slot = f"slot_{sensor_id}"
            raw = SensorSample(
                sensor_id,
                quaternion=matrix_to_quaternion(rotations[role]),
                slot=slot,
                coordinate_frame="synthetic_human_world",
                validation_status="CONFIGURED",
                field_status=(("quaternion", "CONFIGURED"),),
            )
            samples[slot] = TimedSensorSample(
                timestamp, slot, raw, "synthetic_human_world", "CONFIGURED"
            )
        poses.append(SyntheticPose(name, SuitFrame(timestamp, samples)))
    return tuple(poses)


def synthetic_live_rotations(
    frame_number: int,
    fps: float = 30.0,
    profile: BodyProfile | str = BodyProfile.UPPER,
) -> tuple[str, dict[str, np.ndarray]]:
    """Return one deterministic frame from the repeating ten-motion live demo."""
    if frame_number < 0 or fps <= 0:
        raise ValueError("synthetic frame number must be non-negative and fps positive")
    if body_profile(profile) is BodyProfile.FULL:
        return synthetic_full_body_rotations(frame_number, fps)
    seconds = frame_number / fps
    motion_seconds = 1.5
    index = int(seconds / motion_seconds) % len(SYNTHETIC_LIVE_MOTIONS)
    phase = (seconds % motion_seconds) / motion_seconds
    motion = SYNTHETIC_LIVE_MOTIONS[index]
    rotations = {role: np.eye(3) for role in UPPER_BODY_ROLES}
    transition = min(1.0, phase * 4.0)
    wave = np.sin(2 * np.pi * phase)

    if motion == "arms_forward":
        for role in UPPER_BODY_ROLES[1:]:
            rotations[role] = axis_rotation("y", -65 * transition)
    elif motion == "t_pose":
        for side, angle in (("left", 90), ("right", -90)):
            for segment in ("upper_arm", "forearm", "hand"):
                rotations[f"{side}_{segment}"] = axis_rotation(
                    "x", angle * transition
                )
    elif motion in {"left_elbow_flex", "right_elbow_flex"}:
        side = motion.split("_", 1)[0]
        sign = 1 if side == "left" else -1
        rotations[f"{side}_upper_arm"] = axis_rotation("x", 45 * sign)
        flex = axis_rotation("y", -100 * transition)
        rotations[f"{side}_forearm"] = flex
        rotations[f"{side}_hand"] = flex
    elif motion in {"left_arm_raise_lower", "right_arm_raise_lower"}:
        side = motion.split("_", 1)[0]
        sign = 1 if side == "left" else -1
        rotation = axis_rotation("x", 80 * sign * wave)
        for segment in ("upper_arm", "forearm", "hand"):
            rotations[f"{side}_{segment}"] = rotation
    elif motion == "torso_yaw":
        rotation = axis_rotation("z", 45 * wave)
        rotations = {role: rotation for role in UPPER_BODY_ROLES}
    elif motion == "symmetric_reach":
        rotation = axis_rotation("y", -65 * transition)
        for role in UPPER_BODY_ROLES[1:]:
            rotations[role] = rotation
    elif motion == "slow_continuous_arm_wave":
        for side, sign in (("left", 1), ("right", -1)):
            rotations[f"{side}_upper_arm"] = axis_rotation(
                "x", sign * (45 + 25 * wave)
            )
            rotations[f"{side}_forearm"] = axis_rotation(
                "y", -55 + 30 * wave
            )
            rotations[f"{side}_hand"] = axis_rotation("z", sign * 35 * wave)
    return motion, rotations


def synthetic_full_body_rotations(
    frame_number: int, fps: float = 30.0
) -> tuple[str, dict[str, np.ndarray]]:
    """Return one frame from the deterministic 17-role diagnostic motion set."""
    if frame_number < 0 or fps <= 0:
        raise ValueError("synthetic frame number must be non-negative and fps positive")
    seconds = frame_number / fps
    motion_seconds = 1.5
    index = int(seconds / motion_seconds) % len(SYNTHETIC_FULL_BODY_MOTIONS)
    phase = (seconds % motion_seconds) / motion_seconds
    motion = SYNTHETIC_FULL_BODY_MOTIONS[index]
    rotations = {role: np.eye(3) for role in FULL_BODY_ROLES}
    transition = min(1.0, phase * 4.0)
    wave = np.sin(2 * np.pi * phase)

    def set_chain(side: str, segments: tuple[str, ...], rotation: np.ndarray) -> None:
        for segment in segments:
            rotations[f"{side}_{segment}"] = rotation

    if motion == "t_pose":
        for side, sign in (("left", 1), ("right", -1)):
            set_chain(
                side,
                ("upper_arm", "forearm", "hand"),
                axis_rotation("x", sign * 90 * transition),
            )
    elif motion in {"left_arm_raise", "right_arm_raise"}:
        side = motion.split("_", 1)[0]
        sign = 1 if side == "left" else -1
        set_chain(
            side,
            ("upper_arm", "forearm", "hand"),
            axis_rotation("x", sign * 80 * transition),
        )
    elif motion in {"left_elbow_flex", "right_elbow_flex"}:
        side = motion.split("_", 1)[0]
        sign = 1 if side == "left" else -1
        rotations[f"{side}_upper_arm"] = axis_rotation("x", sign * 35)
        flex = axis_rotation("y", -100 * transition)
        rotations[f"{side}_forearm"] = flex
        rotations[f"{side}_hand"] = flex
    elif motion == "head_yaw":
        rotations["head"] = axis_rotation("z", 60 * wave)
    elif motion == "torso_yaw":
        rotation = axis_rotation("z", 45 * wave)
        for role in (
            "head",
            "left_shoulder",
            "right_shoulder",
            "torso",
            "left_upper_arm",
            "right_upper_arm",
            "left_forearm",
            "right_forearm",
            "left_hand",
            "right_hand",
        ):
            rotations[role] = rotation
    elif motion == "pelvis_yaw":
        rotations["pelvis"] = axis_rotation("z", 35 * wave)
    elif motion in {"left_hip_flex", "right_hip_flex"}:
        side = motion.split("_", 1)[0]
        set_chain(
            side,
            ("thigh", "lower_leg", "foot"),
            axis_rotation("y", -65 * transition),
        )
    elif motion in {"left_knee_bend", "right_knee_bend"}:
        side = motion.split("_", 1)[0]
        rotation = axis_rotation("y", 95 * transition)
        rotations[f"{side}_lower_leg"] = rotation
        rotations[f"{side}_foot"] = rotation
    elif motion in {"left_foot_pitch", "right_foot_pitch"}:
        side = motion.split("_", 1)[0]
        rotations[f"{side}_foot"] = axis_rotation("y", 55 * wave)
    elif motion in {"left_leg_side_lift", "right_leg_side_lift"}:
        side = motion.split("_", 1)[0]
        sign = 1 if side == "left" else -1
        set_chain(
            side,
            ("thigh", "lower_leg", "foot"),
            axis_rotation("x", sign * 55 * transition),
        )
    elif motion == "squat":
        for side in ("left", "right"):
            rotations[f"{side}_thigh"] = axis_rotation("y", -45 * transition)
            rotations[f"{side}_lower_leg"] = axis_rotation("y", 45 * transition)
    elif motion == "slow_whole_body_movement":
        rotations["head"] = axis_rotation("z", 25 * wave)
        rotations["torso"] = axis_rotation("z", 18 * wave)
        rotations["pelvis"] = axis_rotation("z", -12 * wave)
        for side, sign in (("left", 1), ("right", -1)):
            rotations[f"{side}_shoulder"] = axis_rotation("z", 12 * sign * wave)
            rotations[f"{side}_upper_arm"] = axis_rotation(
                "x", sign * (35 + 20 * wave)
            )
            rotations[f"{side}_forearm"] = axis_rotation("y", -40 + 20 * wave)
            rotations[f"{side}_hand"] = axis_rotation("z", 20 * sign * wave)
            rotations[f"{side}_thigh"] = axis_rotation("y", -25 * sign * wave)
            rotations[f"{side}_lower_leg"] = axis_rotation("y", 20 * sign * wave)
            rotations[f"{side}_foot"] = axis_rotation("y", 12 * sign * wave)
    return motion, rotations
