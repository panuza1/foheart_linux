from copy import deepcopy

import numpy as np
import pytest

from foheart.integrations.humdex import MotionVenusHumDexAdapter
from foheart.motionvenus.synthetic import synthetic_frame
from foheart.whole_body.gmr import (
    G1_JOINT_NAMES,
    GMR_REQUIRED_BONES,
    GMRWholeBodyRetargeter,
)
from foheart.whole_body.reference import G1ReferenceProcessor


class OneFrameSource:
    def __init__(self, frame):
        self.frame = frame

    def start(self):
        pass

    def receive(self):
        frame, self.frame = self.frame, None
        return frame

    def close(self):
        pass


class FakeModel:
    nu = 29

    def __init__(self):
        self.actuator_trnid = np.column_stack((np.arange(1, 30), np.full(29, -1)))
        self.jnt_range = np.vstack((np.zeros((1, 2)), np.tile((-2.0, 2.0), (29, 1))))
        self.jnt_limited = np.concatenate(([False], np.ones(29, dtype=bool)))


class FakeGMR:
    def __init__(self, qpos=None, *, use_input_root=True):
        self.robot_motor_names = dict(zip(G1_JOINT_NAMES, range(29)))
        self.robot_dof_names = {"pelvis": 5, **dict(zip(G1_JOINT_NAMES, range(6, 35)))}
        self.model = FakeModel()
        self.qpos = np.zeros(36) if qpos is None else np.asarray(qpos, dtype=float)
        if qpos is None:
            self.qpos[2] = 0.8
            self.qpos[3] = 1.0
        self.use_input_root = use_input_root
        self.calls = []

    def retarget(self, human_data, *, offset_to_ground=False):
        snapshot = {name: [pose[0].copy(), pose[1].copy()] for name, pose in human_data.items()}
        self.calls.append((snapshot, offset_to_ground))
        result = self.qpos.copy()
        if self.use_input_root:
            result[:3] = snapshot["Pelvis"][0]
            result[3:7] = snapshot["Pelvis"][1]
        human_data["Pelvis"][0][:] = 999.0  # Pinned GMR also mutates nested arrays.
        return result


def humdex_body_frame(pose="neutral"):
    source = OneFrameSource(synthetic_frame(pose, frame_number=1, timestamp_ns=1))
    adapter = MotionVenusHumDexAdapter(source)
    adapter.initialize()
    result = adapter.read_frame()
    adapter.close()
    assert result["ok"]
    return result["body_frame"]


@pytest.mark.parametrize("pose", ("neutral", "arms_forward", "t_pose", "torso_yaw", "left_arm_raise"))
def test_humdex_to_fake_gmr_representative_frames(pose):
    fake = FakeGMR()
    reference = GMRWholeBodyRetargeter(fake).retarget(humdex_body_frame(pose))

    assert tuple(fake.calls[0][0]) == GMR_REQUIRED_BONES
    assert fake.calls[0][1] is False
    assert reference.qpos_wxyz.shape == (36,)
    assert reference.root_pos.shape == (3,)
    assert reference.root_quat_wxyz.shape == (4,)
    assert reference.dof_pos.shape == (29,)
    assert reference.joint_names == G1_JOINT_NAMES
    assert np.isfinite(reference.qpos_wxyz).all()
    assert np.linalg.norm(reference.root_quat_wxyz) == pytest.approx(1.0)
    assert reference.joint_lower == pytest.approx(np.full(29, -2.0))
    assert reference.joint_upper == pytest.approx(np.full(29, 2.0))


def test_wrapper_copies_mutated_gmr_input_and_is_repeatable():
    body_frame = humdex_body_frame()
    before = deepcopy(body_frame)
    wrapper = GMRWholeBodyRetargeter(FakeGMR(), offset_to_ground=True)

    first = wrapper.retarget(body_frame)
    second = wrapper.retarget(body_frame)

    assert np.array_equal(first.qpos_wxyz, second.qpos_wxyz)
    assert all(
        np.array_equal(body_frame[name][part], before[name][part])
        for name in body_frame
        for part in (0, 1)
    )
    assert wrapper.retargeter.calls[0][1] is True
    assert not first.qpos_wxyz.flags.writeable


@pytest.mark.parametrize("failure", ("missing", "position", "quaternion"))
def test_invalid_human_contract_fails_before_gmr(failure):
    body_frame = humdex_body_frame()
    fake = FakeGMR()
    if failure == "missing":
        del body_frame["Chest"]
    elif failure == "position":
        body_frame["Pelvis"][0][0] = np.nan
    else:
        body_frame["Pelvis"][1] = np.asarray((2.0, 0.0, 0.0, 0.0))

    with pytest.raises(ValueError):
        GMRWholeBodyRetargeter(fake).retarget(body_frame)
    assert fake.calls == []


@pytest.mark.parametrize("failure", ("shape", "finite", "root_quaternion"))
def test_invalid_gmr_output_fails_closed(failure):
    qpos = np.zeros(36)
    qpos[3] = 1.0
    if failure == "shape":
        qpos = qpos[:-1]
    elif failure == "finite":
        qpos[7] = np.nan
    else:
        qpos[3] = 0.0

    with pytest.raises(ValueError):
        GMRWholeBodyRetargeter(FakeGMR(qpos, use_input_root=failure != "root_quaternion")).retarget(
            humdex_body_frame()
        )


def test_exposed_motor_and_joint_order_must_match_pinned_order():
    wrong_motor = FakeGMR()
    wrong_motor.robot_motor_names = dict(zip(reversed(G1_JOINT_NAMES), range(29)))
    with pytest.raises(ValueError, match="motor order"):
        GMRWholeBodyRetargeter(wrong_motor)

    wrong_joint = FakeGMR()
    wrong_joint.robot_dof_names = {"pelvis": 5, **dict(zip(reversed(G1_JOINT_NAMES), range(6, 35)))}
    with pytest.raises(ValueError, match="joint order"):
        GMRWholeBodyRetargeter(wrong_joint)


def test_model_limits_are_extracted_for_processor_enforcement():
    outside = FakeGMR()
    outside.qpos[7] = np.nextafter(-2.0, -np.inf)
    wrapper = GMRWholeBodyRetargeter(outside)
    reference = wrapper.retarget(humdex_body_frame())
    processed = G1ReferenceProcessor(
        wrapper.joint_lower, wrapper.joint_upper, stale_after_s=0.1
    ).process(reference, source_timestamp_s=0.0)
    assert processed is not None
    assert processed.dof_pos[0] == -2.0
    assert processed.hard_clamped_joints == (G1_JOINT_NAMES[0],)

    broken = FakeGMR()
    broken.model.jnt_range[1] = (1.0, -1.0)
    with pytest.raises(ValueError, match="finite and ordered"):
        GMRWholeBodyRetargeter(broken)


def test_constructor_rejects_ambiguous_options():
    with pytest.raises(ValueError, match="actual_human_height"):
        GMRWholeBodyRetargeter(FakeGMR(), actual_human_height=1.8)
    with pytest.raises(TypeError, match="offset_to_ground"):
        GMRWholeBodyRetargeter(FakeGMR(), offset_to_ground="false")
