import numpy as np
import pytest

from foheart.whole_body.gmr import G1_JOINT_NAMES, G1KinematicReference
from foheart.whole_body.reference import G1ReferenceProcessor


def reference(joints=None, *, qpos_size=36, joint_names=G1_JOINT_NAMES):
    qpos = np.zeros(qpos_size)
    qpos[3] = 1.0
    if qpos_size == 36:
        qpos[7:] = np.zeros(29) if joints is None else joints
    return G1KinematicReference(
        qpos,
        qpos[:3],
        qpos[3:7],
        qpos[7:],
        joint_names,
        np.full(29, -2.0),
        np.full(29, 2.0),
    )


def processor(**changes):
    options = {"stale_after_s": 0.1}
    options.update(changes)
    return G1ReferenceProcessor(np.full(29, -1.0), np.full(29, 1.0), **options)


def test_hard_limits_inside_boundary_outside_and_multiple_joint_diagnostics():
    target = np.zeros(29)
    target[0], target[1], target[2] = 0.5, 1.0, -1.0
    output = processor().process(reference(target), source_timestamp_s=0.0)
    assert output is not None
    assert output.clamp_count == 0
    assert output.dof_pos[:3] == pytest.approx((0.5, 1.0, -1.0))

    outside = target.copy()
    outside[0], outside[1], outside[2] = 1.5, -3.0, 1.0
    output = processor().process(reference(outside), source_timestamp_s=0.0)
    assert output is not None
    assert output.dof_pos[:3] == pytest.approx((1.0, -1.0, 1.0))
    assert output.clamped_joints == G1_JOINT_NAMES[:2]
    assert output.clamp_count == 2


@pytest.mark.parametrize("failure", ("qpos_length", "joint_length", "nan", "order"))
def test_invalid_reference_holds_last_safe_complete_frame(failure):
    safe = processor()
    first = safe.process(reference(), source_timestamp_s=0.0)
    broken = reference()
    if failure == "qpos_length":
        broken = reference(qpos_size=35)
    elif failure == "joint_length":
        broken = G1KinematicReference(
            broken.qpos_wxyz,
            broken.root_pos,
            broken.root_quat_wxyz,
            np.zeros(28),
            broken.joint_names,
            broken.joint_lower,
            broken.joint_upper,
        )
    elif failure == "nan":
        broken.qpos_wxyz[7] = np.nan
        broken.dof_pos[0] = np.nan
    else:
        broken = reference(joint_names=tuple(reversed(G1_JOINT_NAMES)))

    held = safe.process(broken, source_timestamp_s=0.01)
    assert first is not None and held is not None and held.status == "HOLD"
    assert np.array_equal(held.qpos_wxyz, first.qpos_wxyz)


def test_soft_limits_are_disabled_at_zero_and_configurable_without_crossing_hard_limits():
    target = np.full(29, 0.9)
    disabled = processor().process(reference(target), source_timestamp_s=0.0)
    assert disabled is not None and disabled.dof_pos == pytest.approx(target)
    assert disabled.soft_clamped_joints == ()

    enabled = processor(soft_limit_margin=0.2)
    inside = enabled.process(reference(np.full(29, 0.7)), source_timestamp_s=0.0)
    crossed = enabled.process(reference(target), source_timestamp_s=0.01)
    assert inside is not None and inside.clamp_count == 0
    assert crossed is not None and crossed.dof_pos == pytest.approx(np.full(29, 0.8))
    assert crossed.soft_clamped_joints == G1_JOINT_NAMES
    assert np.all(crossed.dof_pos <= 1.0)

    hard = processor(soft_limit_margin=0.2).process(
        reference(np.full(29, 3.0)), source_timestamp_s=0.0
    )
    assert hard is not None and hard.dof_pos == pytest.approx(np.full(29, 0.8))
    assert hard.hard_clamped_joints == G1_JOINT_NAMES


@pytest.mark.parametrize(
    ("alpha", "expected"),
    ((None, 1.0), (0.0, 0.0), (0.25, 0.25), (1.0, 1.0)),
)
def test_ema_disabled_and_alpha_edges_apply_to_all_29_joints(alpha, expected):
    filtered = processor(ema_alpha=alpha)
    first = filtered.process(reference(np.zeros(29)), source_timestamp_s=0.0)
    second = filtered.process(reference(np.ones(29)), source_timestamp_s=0.01)
    assert first is not None and second is not None
    assert second.dof_pos == pytest.approx(np.full(29, expected))
    assert second.smoothing_applied is (alpha is not None)

    constant = filtered.process(reference(np.full(29, expected)), source_timestamp_s=0.02)
    assert constant is not None and constant.dof_pos == pytest.approx(np.full(29, expected))


def test_joint_rate_limit_uses_rate_times_dt_for_tiny_and_large_steps():
    limited = processor(max_joint_rate=2.0)
    limited.process(reference(np.zeros(29)), source_timestamp_s=0.0)
    small = limited.process(reference(np.ones(29)), source_timestamp_s=0.1)
    assert small is not None and small.dof_pos == pytest.approx(np.full(29, 0.2))
    assert small.rate_limited_joints == G1_JOINT_NAMES

    tiny = limited.process(reference(np.ones(29)), source_timestamp_s=0.100000001)
    assert tiny is not None
    assert tiny.dof_pos - small.dof_pos == pytest.approx(np.full(29, 2e-9), abs=1e-12)

    unlimited_by_dt = processor(max_joint_rate=2.0)
    unlimited_by_dt.process(reference(np.zeros(29)), source_timestamp_s=0.0)
    large = unlimited_by_dt.process(reference(np.ones(29)), source_timestamp_s=10.0)
    assert large is not None and large.dof_pos == pytest.approx(np.ones(29))
    assert large.rate_limit_count == 0


def test_invalid_duplicate_and_out_of_order_dt_hold_then_recover():
    safe = processor(max_joint_rate=1.0)
    first = safe.process(reference(), source_timestamp_s=1.0, source_frame_number=10)
    duplicate_time = safe.process(reference(), source_timestamp_s=1.0, source_frame_number=11)
    duplicate_frame = safe.process(reference(), source_timestamp_s=1.1, source_frame_number=10)
    out_of_order = safe.process(reference(), source_timestamp_s=1.2, source_frame_number=9)
    invalid = safe.process(reference(), source_timestamp_s=np.nan, source_frame_number=11)
    recovered = safe.process(reference(), source_timestamp_s=1.3, source_frame_number=11)

    assert first is not None
    assert all(
        item is not None and item.status == "HOLD" and np.array_equal(item.dof_pos, first.dof_pos)
        for item in (duplicate_time, duplicate_frame, out_of_order, invalid)
    )
    assert recovered is not None and recovered.status == "FOLLOW"


def test_source_frame_number_wrap_remains_in_order():
    safe = processor()
    first = safe.process(
        reference(), source_timestamp_s=0.0, source_frame_number=0xFFFFFFFF
    )
    wrapped = safe.process(reference(), source_timestamp_s=0.01, source_frame_number=0)
    assert first is not None and wrapped is not None and wrapped.status == "FOLLOW"


def test_follow_packet_timeout_invalid_frame_gmr_failure_nan_and_recovery():
    safe = processor()
    followed = safe.process(reference(), source_timestamp_s=0.0, source_frame_number=1)
    assert followed is not None and followed.status == "FOLLOW"
    assert safe.check_stale(0.1).status == "FOLLOW"

    timed_out = safe.check_stale(0.100001)
    timeout_events = safe.hold_events
    timed_out_again = safe.check_stale(0.2)
    timeout_events_after_poll = safe.hold_events
    invalid = safe.process(
        reference(), source_timestamp_s=0.01, source_frame_number=2, source_valid=False
    )
    failed = safe.process(
        None, source_timestamp_s=0.02, source_frame_number=2, error="GMR solve failed"
    )
    nan = reference()
    nan.qpos_wxyz[7] = nan.dof_pos[0] = np.nan
    rejected_nan = safe.process(nan, source_timestamp_s=0.03, source_frame_number=2)
    recovered = safe.process(reference(), source_timestamp_s=0.04, source_frame_number=2)

    assert timed_out is not None and "packet timeout" in timed_out.reason
    assert timed_out_again is not None and timed_out_again.status == "HOLD"
    assert timeout_events_after_poll == timeout_events == 1
    assert invalid is not None and invalid.status == "HOLD"
    assert failed is not None and failed.reason == "GMR solve failed"
    assert rejected_nan is not None and rejected_nan.status == "HOLD"
    assert recovered is not None and recovered.status == "FOLLOW"

    retained = processor()
    retained.process(reference(), source_timestamp_s=0.0)
    retained.process(reference(), source_timestamp_s=0.01, source_valid=False)
    hold_events = retained.hold_events
    assert retained.check_stale(0.05).status == "HOLD"
    assert retained.hold_events == hold_events


def test_output_is_synchronized_read_only_and_quaternion_sign_continuous():
    safe = processor()
    first = safe.process(reference(np.arange(29) / 100), source_timestamp_s=0.0)
    signed = reference(np.arange(29) / 100)
    signed.qpos_wxyz[3:7] *= -1
    second = safe.process(signed, source_timestamp_s=0.01)

    assert first is not None and second is not None
    assert second.qpos_wxyz.shape == (36,) and second.dof_pos.shape == (29,)
    assert np.array_equal(second.qpos_wxyz[:3], second.root_pos)
    assert np.array_equal(second.qpos_wxyz[3:7], second.root_quat_wxyz)
    assert np.array_equal(second.qpos_wxyz[7:], second.dof_pos)
    assert second.root_quat_wxyz @ first.root_quat_wxyz > 0
    assert not second.qpos_wxyz.flags.writeable and not second.dof_pos.flags.writeable


@pytest.mark.parametrize(
    ("option", "value"),
    (
        ("stale_after_s", 0.0),
        ("soft_limit_margin", -0.1),
        ("soft_limit_margin", 1.1),
        ("ema_alpha", -0.1),
        ("ema_alpha", 1.1),
        ("max_joint_rate", 0.0),
    ),
)
def test_processor_configuration_rejects_undocumented_or_invalid_values(option, value):
    with pytest.raises(ValueError):
        processor(**{option: value})


def test_hard_limits_require_exact_29_value_vectors():
    with pytest.raises(ValueError, match="exactly 29"):
        G1ReferenceProcessor(np.full(28, -1.0), np.ones(29), stale_after_s=0.1)
    with pytest.raises(ValueError, match="exactly 29"):
        G1ReferenceProcessor(-1.0, np.ones(29), stale_after_s=0.1)
