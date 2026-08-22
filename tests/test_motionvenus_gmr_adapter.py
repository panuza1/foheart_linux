from dataclasses import replace

import numpy as np
import pytest

from foheart.mocap.frames import (
    BasisTransform,
    axis_rotation,
    matrix_to_quaternion,
    quaternion_to_matrix,
)
from foheart.mocap.sensor import Quaternion
from foheart.motionvenus.gmr import (
    MOTIONVENUS_TO_GMR_BASIS,
    MOTIONVENUS_TO_GMR_BONES,
    MotionVenusGMRAdapter,
)
from foheart.motionvenus.skeleton import HumanSkeletonFrame
from foheart.motionvenus.synthetic import GMR_SYNTHETIC_POSES, synthetic_frame
from foheart.whole_body.gmr import GMR_REQUIRED_BONES


def skeleton(pose="neutral"):
    return HumanSkeletonFrame.from_motionvenus(
        synthetic_frame(pose, frame_number=1, timestamp_ns=1), status="LIVE"
    )


def facing(yaw_deg, *, pitch_deg=0.0, roll_deg=0.0, sign=1.0):
    frame = skeleton()
    rotation = (
        axis_rotation("z", yaw_deg)
        @ axis_rotation("y", pitch_deg)
        @ axis_rotation("x", roll_deg)
    )
    w, x, y, z = matrix_to_quaternion(rotation).values
    xyzw = tuple(sign * value for value in (x, y, z, w))
    bones = {
        name: replace(
            bone,
            position_global_m=tuple(rotation @ np.asarray(bone.position_global_m)),
            rotation_global_xyzw=xyzw,
        )
        for name, bone in frame.bones.items()
    }
    return replace(frame, bones=bones)


def heading_deg(quaternion_wxyz):
    rotation = quaternion_to_matrix(Quaternion(tuple(quaternion_wxyz), "wxyz"))
    return np.degrees(np.arctan2(rotation[1, 0], rotation[0, 0]))


@pytest.mark.parametrize("pose", GMR_SYNTHETIC_POSES)
def test_ten_requested_poses_produce_complete_finite_gmr_human_data(pose):
    data = MotionVenusGMRAdapter().adapt(skeleton(pose))
    assert tuple(data) == GMR_REQUIRED_BONES
    assert all(position.shape == (3,) and np.isfinite(position).all() for position, _ in data.values())
    assert all(quaternion.shape == (4,) for _, quaternion in data.values())
    assert all(np.linalg.norm(quaternion) == pytest.approx(1.0) for _, quaternion in data.values())


def test_mapping_wxyz_left_right_metres_and_proper_basis_are_explicit():
    adapter = MotionVenusGMRAdapter()
    frame = skeleton("wrist_rotations")
    data = adapter.adapt(frame)
    basis = np.asarray(MOTIONVENUS_TO_GMR_BASIS.matrix)

    assert tuple(target for _, target in MOTIONVENUS_TO_GMR_BONES) == GMR_REQUIRED_BONES
    assert basis.T @ basis == pytest.approx(np.eye(3))
    assert np.linalg.det(basis) == pytest.approx(1.0)
    assert data["Left_Hand"][0][1] > 0 > data["Right_Hand"][0][1]
    assert np.linalg.norm(data["Left_Hand"][0]) == pytest.approx(
        np.linalg.norm(frame.bone("LeftHand").position_global_m)
    )

    source_xyzw = frame.bone("LeftHand").rotation_global_xyzw
    source_wxyz = Quaternion(
        (source_xyzw[3], source_xyzw[0], source_xyzw[1], source_xyzw[2]), "wxyz"
    )
    expected = basis @ quaternion_to_matrix(source_wxyz) @ basis.T
    actual = quaternion_to_matrix(Quaternion(tuple(data["Left_Hand"][1]), "wxyz"))
    assert actual == pytest.approx(expected)

    with pytest.raises(ValueError, match="right-handed"):
        BasisTransform(((-1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)), "a", "b")


def test_stale_nan_missing_and_non_global_frames_are_rejected():
    adapter = MotionVenusGMRAdapter()
    frame = skeleton()
    with pytest.raises(ValueError, match="stale"):
        adapter.adapt(replace(frame, stale=True, valid=False, reason="stale"))
    assert not adapter.heading_calibration.calibrated

    bones = dict(frame.bones)
    bones["Pelvis"] = replace(bones["Pelvis"], position_global_m=(np.nan, 0.0, 1.0))
    with pytest.raises(ValueError, match="finite XYZ"):
        adapter.adapt(replace(frame, bones=bones))
    assert not adapter.heading_calibration.calibrated

    bones = dict(frame.bones)
    del bones["T8"]
    with pytest.raises(ValueError, match="missing GMR bones: T8"):
        adapter.adapt(replace(frame, bones=bones))

    with pytest.raises(ValueError, match="global poses"):
        adapter.adapt(replace(frame, source_coordinate="local"))


@pytest.mark.parametrize("initial_yaw", (0.0, 90.0, -90.0))
def test_first_valid_pelvis_heading_is_normalized_once(initial_yaw):
    adapter = MotionVenusGMRAdapter()
    output = adapter.adapt(facing(initial_yaw, pitch_deg=15.0, roll_deg=-10.0))
    raw = MotionVenusGMRAdapter(normalize_heading=False).adapt(
        facing(initial_yaw, pitch_deg=15.0, roll_deg=-10.0)
    )
    raw_pelvis = quaternion_to_matrix(Quaternion(tuple(raw["Pelvis"][1]), "wxyz"))
    yaw = np.arctan2(raw_pelvis[1, 0], raw_pelvis[0, 0])

    assert adapter.heading_calibration.calibrated
    assert adapter.heading_calibration.status == "SOFTWARE_CONFIGURED"
    assert heading_deg(output["Pelvis"][1]) == pytest.approx(0.0, abs=1e-10)
    assert quaternion_to_matrix(Quaternion(tuple(output["Pelvis"][1]), "wxyz")) == pytest.approx(
        axis_rotation("z", -np.degrees(yaw)) @ raw_pelvis
    )
    assert output["Left_Hand"][0] == pytest.approx(
        axis_rotation("z", -np.degrees(yaw)) @ raw["Left_Hand"][0]
    )


@pytest.mark.parametrize(
    ("initial_yaw", "later_yaw", "expected_relative"),
    ((90.0, 120.0, 30.0), (-90.0, -135.0, -45.0)),
)
def test_later_turning_is_relative_to_initial_heading(initial_yaw, later_yaw, expected_relative):
    adapter = MotionVenusGMRAdapter()
    adapter.adapt(facing(initial_yaw))
    later = adapter.adapt(facing(later_yaw))
    assert heading_deg(later["Pelvis"][1]) == pytest.approx(expected_relative)


def test_heading_reset_disable_and_quaternion_sign_equivalence():
    adapter = MotionVenusGMRAdapter()
    adapter.adapt(facing(90.0))
    positive = adapter.adapt(facing(120.0))
    negative = adapter.adapt(facing(120.0, sign=-1.0))
    assert positive["Pelvis"][1] == pytest.approx(negative["Pelvis"][1])

    adapter.reset_heading()
    assert not adapter.heading_calibration.calibrated
    assert heading_deg(adapter.adapt(facing(120.0))["Pelvis"][1]) == pytest.approx(0.0)

    disabled = MotionVenusGMRAdapter(normalize_heading=False)
    assert heading_deg(disabled.adapt(facing(90.0))["Pelvis"][1]) == pytest.approx(90.0)
    assert disabled.heading_calibration.status == "DISABLED"
    assert not disabled.heading_calibration.calibrated


def test_invalid_pelvis_quaternion_cannot_calibrate():
    adapter = MotionVenusGMRAdapter()
    frame = skeleton()
    bones = dict(frame.bones)
    bones["Pelvis"] = replace(bones["Pelvis"], rotation_global_xyzw=(0.0, 0.0, 0.0, 0.0))
    with pytest.raises(ValueError, match="near zero"):
        adapter.adapt(replace(frame, bones=bones))
    assert not adapter.heading_calibration.calibrated
