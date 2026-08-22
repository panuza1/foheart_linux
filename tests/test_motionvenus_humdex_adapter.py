from collections import deque
from dataclasses import replace

import numpy as np
import pytest

from foheart.integrations.humdex import (
    MOTIONVENUS_TO_GMR_XSENS_BASIS,
    MOTIONVENUS_TO_XSENS_MVN_BONES,
    MotionVenusHumDexAdapter,
)
from foheart.mocap.frames import quaternion_to_matrix
from foheart.mocap.sensor import Quaternion
from foheart.motionvenus.skeleton import HumanSkeletonFrame
from foheart.motionvenus.synthetic import synthetic_frame


class FakeSource:
    def __init__(self, *frames):
        self.frames = deque(frames)
        self.started = False
        self.closed = False

    def start(self):
        self.started = True

    def receive(self):
        return self.frames.popleft() if self.frames else None

    def close(self):
        self.closed = True


def skeleton(pose="neutral", frame_number=1, timestamp_ns=None, **changes):
    raw = synthetic_frame(
        pose,
        frame_number=frame_number,
        timestamp_ns=timestamp_ns if timestamp_ns is not None else frame_number * 1_000_000,
    )
    return replace(HumanSkeletonFrame.from_motionvenus(raw, status="LIVE"), **changes)


def read_one(frame):
    adapter = MotionVenusHumDexAdapter(FakeSource(frame))
    adapter.initialize()
    return adapter.read_frame()


def alter_positions(frame, changes):
    bones = {
        name: replace(bone, position_global_m=changes.get(name, bone.position_global_m))
        for name, bone in frame.bones.items()
    }
    return replace(frame, bones=bones)


def test_exact_humdex_lifecycle_contract_and_metadata():
    source = FakeSource(synthetic_frame("neutral", frame_number=7, timestamp_ns=123))
    adapter = MotionVenusHumDexAdapter(source)
    assert adapter.read_frame() == {"ok": False, "reason": "not_initialized"}
    adapter.initialize()
    result = adapter.read_frame()
    assert source.started and result["ok"] and result["frame_index"] == 7
    assert result["source_metadata"]["host_timestamp_ns"] == 123
    assert result["source_metadata"]["valid"] is True
    assert result["source_metadata"]["stale"] is False
    assert result["source_metadata"]["source_bone_names"][0] == "Pelvis"
    assert result["source_metadata"]["bone_mapping"] == MOTIONVENUS_TO_XSENS_MVN_BONES
    assert adapter.read_frame() == {"ok": False, "reason": "no_update"}
    adapter.close()
    assert source.closed and adapter.read_frame() == {"ok": False, "reason": "not_initialized"}


def test_exact_xsens_names_left_right_basis_and_normalized_wxyz():
    result = read_one(skeleton())
    body = result["body_frame"]
    assert tuple(body) == tuple(target for _, target in MOTIONVENUS_TO_XSENS_MVN_BONES)
    assert body["Right_Hand"][0] == pytest.approx((0.0, -0.32, 0.95))
    assert body["Left_Hand"][0] == pytest.approx((0.0, 0.32, 0.95))
    assert body["Chest"][0] == pytest.approx((0.0, 0.0, 1.45))
    assert all(np.linalg.norm(value[1]) == pytest.approx(1.0) for value in body.values())
    assert np.linalg.det(np.asarray(MOTIONVENUS_TO_GMR_XSENS_BASIS.matrix)) == pytest.approx(1.0)


def test_global_quaternion_is_changed_by_the_documented_proper_basis():
    frame = skeleton("wrist_rotations")
    result = read_one(frame)
    source_xyzw = frame.bones["LeftHand"].rotation_global_xyzw
    source_rotation = quaternion_to_matrix(
        Quaternion((source_xyzw[3], source_xyzw[0], source_xyzw[1], source_xyzw[2]), "wxyz")
    )
    basis = np.asarray(MOTIONVENUS_TO_GMR_XSENS_BASIS.matrix)
    actual = quaternion_to_matrix(Quaternion(tuple(result["body_frame"]["Left_Hand"][1]), "wxyz"))
    assert actual == pytest.approx(basis @ source_rotation @ basis.T)


@pytest.mark.parametrize(
    "pose",
    ("neutral", "arms_forward", "t_pose", "left_arm_raise", "right_arm_raise", "torso_yaw", "symmetric_reach"),
)
def test_representative_arm_torso_and_symmetric_poses_are_finite(pose):
    result = read_one(skeleton(pose))
    assert result["ok"]
    assert all(np.isfinite(np.concatenate(value)).all() for value in result["body_frame"].values())


@pytest.mark.parametrize("kind", ("squat", "left_leg_raise", "right_leg_raise"))
def test_representative_lower_body_poses_are_finite(kind):
    frame = skeleton()
    if kind == "squat":
        changes = {
            name: tuple(np.asarray(bone.position_global_m) - (0.0, 0.0, 0.25))
            for name, bone in frame.bones.items()
            if name not in ("LeftFoot", "LeftToe", "RightFoot", "RightToe")
        }
    else:
        side = "Left" if kind.startswith("left") else "Right"
        changes = {
            f"{side}UpperLeg": ((-0.10 if side == "Left" else 0.10), 0.18, 1.05),
            f"{side}LowerLeg": ((-0.10 if side == "Left" else 0.10), 0.35, 0.78),
            f"{side}Foot": ((-0.10 if side == "Left" else 0.10), 0.48, 0.48),
            f"{side}Toe": ((-0.10 if side == "Left" else 0.10), 0.62, 0.45),
        }
    result = read_one(alter_positions(frame, changes))
    assert result["ok"]
    assert all(np.isfinite(np.concatenate(value)).all() for value in result["body_frame"].values())


@pytest.mark.parametrize("bone_name", ("Pelvis", "T8", "LeftHand", "RightFoot"))
def test_missing_required_bones_fail_closed(bone_name):
    frame = skeleton()
    bones = dict(frame.bones)
    del bones[bone_name]
    result = read_one(replace(frame, bones=bones))
    assert not result["ok"] and result["reason"].startswith("missing_bones:")


def test_stale_invalid_local_and_missing_pose_fail_closed():
    assert read_one(skeleton(stale=True, valid=False, reason="MotionVenus input is stale"))["reason"] == "stale_source"
    assert read_one(skeleton(status="MALFORMED"))["reason"] == "source_status:MALFORMED"
    assert read_one(skeleton(valid=False, reason="watchdog fault"))["reason"] == "invalid_source:watchdog fault"
    assert read_one(skeleton(source_coordinate="local"))["reason"] == "source_coordinate:local"
    frame = skeleton()
    bones = dict(frame.bones)
    bones["LeftHand"] = replace(bones["LeftHand"], rotation_global_xyzw=None)
    assert read_one(replace(frame, bones=bones))["reason"] == "missing_global_pose:LeftHand"


@pytest.mark.parametrize(
    ("field", "value", "reason"),
    (
        ("position_global_m", (np.nan, 0.0, 0.0), "invalid_position:Pelvis"),
        ("rotation_global_xyzw", (0.0, 0.0, 0.0, 0.0), "invalid_quaternion:Pelvis"),
        ("rotation_global_xyzw", (0.0, np.nan, 0.0, 1.0), "invalid_quaternion:Pelvis"),
    ),
)
def test_nan_and_invalid_quaternion_fail_closed(field, value, reason):
    frame = skeleton()
    bones = dict(frame.bones)
    bones["Pelvis"] = replace(bones["Pelvis"], **{field: value})
    assert read_one(replace(frame, bones=bones))["reason"] == reason


def test_duplicate_out_of_order_and_non_monotonic_timestamp_fail_closed():
    source = FakeSource(
        skeleton(frame_number=10, timestamp_ns=100),
        skeleton(frame_number=10, timestamp_ns=200),
        skeleton(frame_number=9, timestamp_ns=300),
        skeleton(frame_number=11, timestamp_ns=100),
        skeleton(frame_number=11, timestamp_ns=400),
    )
    adapter = MotionVenusHumDexAdapter(source)
    adapter.initialize()
    assert adapter.read_frame()["ok"]
    assert adapter.read_frame()["reason"] == "duplicate_frame"
    assert adapter.read_frame()["reason"] == "out_of_order_frame"
    assert adapter.read_frame()["reason"] == "non_monotonic_timestamp"
    assert adapter.read_frame()["ok"]


def test_uint32_frame_rollover_is_monotonic():
    adapter = MotionVenusHumDexAdapter(
        FakeSource(
            skeleton(frame_number=0xFFFFFFFF, timestamp_ns=1),
            skeleton(frame_number=0, timestamp_ns=2),
        )
    )
    adapter.initialize()
    assert adapter.read_frame()["ok"] and adapter.read_frame()["ok"]
