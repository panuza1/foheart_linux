import math

import numpy as np
import pytest

from foheart.integrations.unitree_g1.adapter import (
    G1FrameAdapter,
    G1WristTargets,
    SafeG1IK,
    UpperBodyTargetFilter,
)
from foheart.integrations.unitree_g1.sim_bridge import G1_BODY_ACTUATORS
from foheart.mocap.calibration import CalibrationProfile
from foheart.mocap.frames import (
    BasisTransform,
    axis_rotation,
    matrix_to_quaternion,
    quaternion_to_matrix,
)
from foheart.mocap.sensor import Quaternion
from foheart.mocap.skeleton import BodyDimensions, UpperBodyKinematics
from foheart.mocap.suit import LatestSuitBuffer, SuitFrame
from foheart.mocap.synthetic import SYNTHETIC_BODY_MAP, synthetic_upper_body_sequence
from foheart.tools.calibrate import main as calibrate_main


def _calibrated(sequence, index=0):
    neutral = SYNTHETIC_BODY_MAP.assign(sequence[0].frame)
    profile = CalibrationProfile.capture(
        {role: sample.raw.quaternion for role, sample in neutral.items()}
    )
    assigned = SYNTHETIC_BODY_MAP.assign(sequence[index].frame)
    return profile, {
        role: profile.apply(role, sample.raw.quaternion)
        for role, sample in assigned.items()
    }


def test_basis_rotation_round_trip_and_axis_validation():
    rotation = axis_rotation("y", 90)
    quaternion = matrix_to_quaternion(rotation)
    assert quaternion_to_matrix(quaternion) == pytest.approx(rotation)
    transform = BasisTransform.from_axis_map(
        ("y", "z", "x"), (1, 1, 1), "sensor", "body"
    )
    assert np.linalg.det(transform.matrix) == pytest.approx(1.0)
    converted = transform.orientation(quaternion)
    assert np.linalg.det(quaternion_to_matrix(converted)) == pytest.approx(1.0)
    with pytest.raises(ValueError, match="proper right-handed"):
        BasisTransform.from_axis_map(("x", "y", "z"), (-1, 1, 1), "a", "b")


def test_neutral_calibration_convention_and_yaml_round_trip(tmp_path):
    neutral = matrix_to_quaternion(axis_rotation("z", 30))
    current = matrix_to_quaternion(axis_rotation("z", 80))
    profile = CalibrationProfile.capture({"torso": neutral})
    calibrated = profile.apply("torso", current)
    assert quaternion_to_matrix(calibrated) == pytest.approx(axis_rotation("z", 50))
    path = tmp_path / "calibration.yaml"
    profile.save(path)
    assert CalibrationProfile.load(path).apply("torso", neutral).values == pytest.approx(
        (1, 0, 0, 0)
    )


def test_body_mapping_and_stale_grouping_fail_closed():
    sequence = synthetic_upper_body_sequence()
    mapped = SYNTHETIC_BODY_MAP.assign(sequence[0].frame)
    assert set(mapped) == set(SYNTHETIC_BODY_MAP.role_to_slot)
    buffer = LatestSuitBuffer(max_age_ns=50)
    first = buffer.update(sequence[0].frame, tuple(SYNTHETIC_BODY_MAP.role_to_slot.values()))
    assert not first.stale_slots and not first.missing_slots
    empty = SuitFrame(sequence[0].frame.timestamp_ns + 51, {})
    stale = buffer.update(empty)
    assert set(stale.stale_slots) == set(SYNTHETIC_BODY_MAP.role_to_slot.values())
    with pytest.raises(ValueError, match="stale mapped roles"):
        SYNTHETIC_BODY_MAP.assign(stale)


def test_upper_body_fk_known_poses_and_symmetry():
    sequence = synthetic_upper_body_sequence()
    kinematics = UpperBodyKinematics(BodyDimensions())
    _, neutral = _calibrated(sequence)
    neutral_pose = kinematics.solve(neutral, 1)
    assert neutral_pose.left_wrist_pose[:3, 3] == pytest.approx((0, 0.19, -0.56))
    assert neutral_pose.right_wrist_pose[:3, 3] == pytest.approx((0, -0.19, -0.56))

    _, t_pose = _calibrated(sequence, 2)
    pose = kinematics.solve(t_pose, 2)
    assert pose.left_wrist_pose[:3, 3] == pytest.approx((0, 0.75, 0), abs=1e-9)
    assert pose.right_wrist_pose[:3, 3] == pytest.approx((0, -0.75, 0), abs=1e-9)

    _, left_bend = _calibrated(sequence, 3)
    pose = kinematics.solve(left_bend, 3)
    assert pose.poses["left_elbow"][:3, 3] == pytest.approx((0, 0.49, 0), abs=1e-9)
    assert pose.left_wrist_pose[:3, 3] == pytest.approx((0, 0.49, -0.26), abs=1e-9)


def _robot_geometry():
    left = np.eye(4)
    right = np.eye(4)
    left[:3, 3] = (0.25, 0.15, 0.1)
    right[:3, 3] = (0.25, -0.15, 0.1)
    return left, right, np.array((0, 0.1, 0.3)), np.array((0, -0.1, 0.3))


def test_g1_neutral_alignment_reach_clamp_and_filter_limits():
    sequence = synthetic_upper_body_sequence()
    kinematics = UpperBodyKinematics()
    _, neutral = _calibrated(sequence)
    neutral_targets = kinematics.targets(kinematics.solve(neutral, 1))
    robot_left, robot_right, shoulder_left, shoulder_right = _robot_geometry()
    adapter = G1FrameAdapter(
        neutral_targets,
        robot_left,
        robot_right,
        shoulder_left,
        shoulder_right,
        human_reach_m=0.56,
        robot_reach_m=0.32,
        max_robot_reach_m=0.40,
    )
    adapted = adapter.adapt(neutral_targets)
    assert adapted.left == pytest.approx(robot_left)
    assert adapted.right == pytest.approx(robot_right)

    _, t_pose = _calibrated(sequence, 2)
    target = kinematics.targets(kinematics.solve(t_pose, 100_000_001))
    assert set(adapter.adapt(target).clamped) <= {"left", "right"}

    filter_ = UpperBodyTargetFilter(
        position_alpha=1,
        orientation_alpha=1,
        max_translation_rate_m_s=0.1,
        max_angular_rate_deg_s=10,
    )
    filter_.update(neutral_targets)
    filtered = filter_.update(target)
    assert np.linalg.norm(filtered.left_wrist_pose[:3, 3] - neutral_targets.left_wrist_pose[:3, 3]) <= 0.0100001
    left_angle = math.degrees(
        math.acos(np.clip((np.trace(filtered.left_wrist_pose[:3, :3]) - 1) / 2, -1, 1))
    )
    assert left_angle <= 1.0001


class FakeIK:
    lower_limits = np.full(14, -1.0)
    upper_limits = np.full(14, 1.0)

    def solve_ik(self, left, right, q, dq):
        value = np.clip((left[2, 3] - right[2, 3]) * np.ones(14), -0.5, 0.5)
        return value, np.zeros(14)

    def verify(self, q, left, right):
        return 0.001, 0.1


def test_safe_ik_holds_invalid_and_rate_limits_valid_output():
    safe = SafeG1IK(FakeIK(), max_joint_delta_rad=0.1)
    left, right, *_ = _robot_geometry()
    left[2, 3] = 0.5
    right[2, 3] = 0.0
    result = safe.solve(G1WristTargets(left, right, 1))
    assert result.valid and result.rate_limited
    assert np.max(np.abs(result.joint_positions)) == pytest.approx(0.1)
    invalid = left.copy()
    invalid[0, 0] = np.nan
    held = safe.solve(G1WristTargets(invalid, right, 2))
    assert not held.valid and held.held_previous
    assert held.joint_positions == pytest.approx(result.joint_positions)


def test_synthetic_pipeline_reuses_same_mapping_calibration_fk_adapter_and_ik():
    sequence = synthetic_upper_body_sequence()
    profile, neutral = _calibrated(sequence)
    kinematics = UpperBodyKinematics()
    neutral_targets = kinematics.targets(kinematics.solve(neutral, sequence[0].frame.timestamp_ns))
    robot_left, robot_right, shoulder_left, shoulder_right = _robot_geometry()
    adapter = G1FrameAdapter(
        neutral_targets,
        robot_left,
        robot_right,
        shoulder_left,
        shoulder_right,
        human_reach_m=0.56,
        robot_reach_m=0.32,
        max_robot_reach_m=0.50,
    )
    safe = SafeG1IK(FakeIK())
    results = []
    for item in sequence:
        assigned = SYNTHETIC_BODY_MAP.assign(item.frame)
        orientations = {
            role: profile.apply(role, sample.raw.quaternion)
            for role, sample in assigned.items()
        }
        targets = kinematics.targets(kinematics.solve(orientations, item.frame.timestamp_ns))
        results.append(safe.solve(adapter.adapt(targets)))
    assert len(results) == 7
    assert all(result.valid for result in results)


def test_existing_ik_and_mujoco_joint_order_contract_is_exact():
    assert tuple(f"{name}_joint" for name in G1_BODY_ACTUATORS[15:]) == (
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


def test_offline_calibration_cli_creates_profile_and_refuses_overwrite(tmp_path):
    output = tmp_path / "neutral.yaml"
    assert calibrate_main(["--synthetic-upper-body", "--output", str(output)]) == 0
    assert set(CalibrationProfile.load(output).neutral_wxyz) == set(
        SYNTHETIC_BODY_MAP.role_to_slot
    )
    assert calibrate_main(["--synthetic-upper-body", "--output", str(output)]) == 2
