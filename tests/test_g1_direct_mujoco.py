from pathlib import Path
from dataclasses import replace

import numpy as np
import pytest

from foheart.integrations.unitree_g1.sim_bridge import (
    G1MuJoCoBridge,
    intersect_joint_limits,
)
from foheart.whole_body.gmr import G1_JOINT_NAMES, G1KinematicReference, G1ReferenceMuJoCo
from foheart.whole_body.reference import G1ReferenceProcessor, ProcessedG1Reference


PROJECT = Path(__file__).resolve().parents[3]
DYNAMIC_MODEL = PROJECT / "unitree_mujoco/unitree_robots/g1/scene_29dof.xml"
GMR_MODEL = PROJECT / "third_party/HumDex/GMR/assets/unitree_g1/g1_mocap_29dof.xml"


def test_conservative_joint_limit_intersection_and_invalid_sources():
    lower, upper = intersect_joint_limits(
        (np.full(29, -2.0), np.full(29, 2.0)),
        (np.full(29, -1.0), np.full(29, 1.0)),
    )
    assert lower == pytest.approx(np.full(29, -1.0))
    assert upper == pytest.approx(np.full(29, 1.0))
    with pytest.raises(ValueError, match="exactly 29"):
        intersect_joint_limits((np.zeros(28), np.ones(28)))
    with pytest.raises(ValueError, match="finite and ordered"):
        intersect_joint_limits((np.zeros(29), np.full(29, np.nan)))
    with pytest.raises(ValueError, match="empty intersection"):
        intersect_joint_limits((np.ones(29), np.full(29, 2.0)), (np.full(29, -2.0), np.zeros(29)))


@pytest.fixture()
def models():
    pytest.importorskip("mujoco")
    dynamic = G1MuJoCoBridge(DYNAMIC_MODEL)
    reference = G1ReferenceMuJoCo(GMR_MODEL)
    try:
        yield dynamic, reference
    finally:
        reference.close()


def make_reference(dynamic, gmr, joints):
    qpos = dynamic.model.qpos0.copy()
    qpos[7:] = joints
    lower, upper = dynamic.conservative_joint_limits(gmr.joint_lower, gmr.joint_upper)
    return G1KinematicReference(
        qpos,
        qpos[:3],
        qpos[3:7],
        qpos[7:],
        G1_JOINT_NAMES,
        lower,
        upper,
    )


def test_local_models_have_exact_order_and_gmr_is_the_conservative_intersection(models):
    dynamic, gmr = models
    lower, upper = dynamic.conservative_joint_limits(gmr.joint_lower, gmr.joint_upper)
    assert lower == pytest.approx(gmr.joint_lower)
    assert upper == pytest.approx(gmr.joint_upper)
    differing = (dynamic.joint_lower != gmr.joint_lower) | (dynamic.joint_upper != gmr.joint_upper)
    assert tuple(np.asarray(G1_JOINT_NAMES)[differing]) == (
        "left_hip_pitch_joint",
        "left_hip_roll_joint",
        "left_hip_yaw_joint",
        "right_hip_pitch_joint",
        "right_hip_roll_joint",
        "right_hip_yaw_joint",
        "waist_yaw_joint",
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_joint",
    )


def test_direct_dynamic_ladder_applies_every_29_dof_target_coherently(models):
    dynamic, gmr = models
    lower, upper = dynamic.conservative_joint_limits(gmr.joint_lower, gmr.joint_upper)
    processor = G1ReferenceProcessor(lower, upper, stale_after_s=0.5)
    stand = np.zeros(29)
    stand[[0, 6]] = -0.2
    stand[[3, 9]] = 0.42
    stand[[4, 10]] = -0.23
    targets = []
    for changes in (
        {},
        {15: 0.08, 22: 0.08},
        {14: 0.04},
        {12: 0.04},
        {0: -0.25, 3: 0.5, 4: -0.25, 6: -0.25, 9: 0.5, 10: -0.25},
        {1: 0.03, 7: 0.03},
        {0: -0.12, 3: 0.35},
        {0: -0.15, 3: 0.38, 6: -0.08, 9: 0.3},
    ):
        target = stand.copy()
        for index, value in changes.items():
            target[index] = value
        targets.append(target)

    for number, target in enumerate(targets):
        processed = processor.process(
            make_reference(dynamic, gmr, target),
            source_timestamp_s=number * 0.02,
            now_s=number * 0.02,
            source_frame_number=number,
        )
        assert isinstance(processed, ProcessedG1Reference) and processed.status == "FOLLOW"
        assert processed.clamp_count == processed.rate_limit_count == 0
        metrics = dynamic.command_whole_body(processed, steps=8)
        assert metrics.finite and metrics.steps == 8
        assert metrics.simulation_duration_s == pytest.approx(0.016)
        assert metrics.root_position_m == pytest.approx(dynamic.base_qpos[:3])
        assert metrics.root_quaternion_wxyz == pytest.approx(dynamic.base_qpos[3:7])
        assert metrics.base_pinned
        assert metrics.stability_status == "BASE_PINNED_NOT_ASSESSED"
        assert dynamic.target == pytest.approx(target)

    held = processor.check_stale(1.0)
    assert held is not None and held.status == "HOLD"
    metrics = dynamic.command_whole_body(held, steps=8)
    assert metrics.finite and dynamic.target == pytest.approx(targets[-1])


def test_direct_dynamic_rejects_bad_order_shape_nan_quaternion_limits_and_steps(models):
    dynamic, gmr = models
    lower, upper = dynamic.conservative_joint_limits(gmr.joint_lower, gmr.joint_upper)
    good = G1ReferenceProcessor(lower, upper, stale_after_s=0.5).process(
        make_reference(dynamic, gmr, np.zeros(29)), source_timestamp_s=0.0
    )
    assert good is not None
    wrong_order = replace(good, joint_names=tuple(reversed(G1_JOINT_NAMES)))
    with pytest.raises(ValueError, match="joint order"):
        dynamic.command_whole_body(wrong_order)
    with pytest.raises(TypeError, match="ProcessedG1Reference"):
        dynamic.command_whole_body(make_reference(dynamic, gmr, np.zeros(29)))
    with pytest.raises(ValueError, match="length-36"):
        dynamic.command_whole_body(replace(good, qpos_wxyz=np.zeros(35)))
    invalid = good.qpos_wxyz.copy()
    invalid[0] = np.nan
    with pytest.raises(ValueError, match="finite"):
        dynamic.command_whole_body(replace(good, qpos_wxyz=invalid))
    invalid = good.qpos_wxyz.copy()
    invalid[3:7] = 0.0
    with pytest.raises(ValueError, match="quaternion"):
        dynamic.command_whole_body(replace(good, qpos_wxyz=invalid))
    invalid = good.qpos_wxyz.copy()
    invalid[7] = dynamic.joint_upper[0] + 0.01
    with pytest.raises(ValueError, match="joint limits"):
        dynamic.command_whole_body(replace(good, qpos_wxyz=invalid))
    with pytest.raises(ValueError, match="steps"):
        dynamic.command_whole_body(good, steps=0)
