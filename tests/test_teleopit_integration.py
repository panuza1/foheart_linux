from argparse import Namespace
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("teleopit")

from foheart.config import load_config
from foheart.integrations.teleopit import (
    FoheartTeleopitAdapter,
    FoheartTeleopitPolicySimulator,
)
from foheart.motionvenus.protocol import encode_binary_frame
from foheart.motionvenus.synthetic import GMR_SYNTHETIC_POSES, synthetic_frame
from foheart.motionvenus.transport import MotionVenusCaptureWriter, MotionVenusDatagram
from foheart.tools.motionvenus_g1_reference import (
    DEFAULT_GMR_MODEL,
    DEFAULT_TELEOPIT_POLICY,
    DEFAULT_TELEOPIT_ROOT,
    run,
)
from foheart.whole_body.gmr import G1_JOINT_NAMES
from foheart.whole_body.reference import ProcessedG1Reference


def processed(
    timestamp_s=10.0,
    *,
    joints=None,
    status="FOLLOW",
    frame_number=1,
):
    qpos = np.zeros(36)
    qpos[:3] = (1.0, 2.0, 0.8)
    qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
    qpos[7:] = np.arange(29) / 100 if joints is None else joints
    return ProcessedG1Reference(
        qpos,
        qpos[:3],
        qpos[3:7],
        qpos[7:],
        G1_JOINT_NAMES,
        status,
        timestamp_s,
        frame_number,
        "test reference",
    )


def test_adapter_maps_by_name_preserves_wxyz_and_converts_timestamps():
    adapter = FoheartTeleopitAdapter(max_reference_age_s=0.25)
    first = adapter.adapt(processed(), now_s=10.0)
    second = adapter.adapt(processed(10.02, frame_number=2), now_s=10.02)

    assert adapter.target_joint_names == G1_JOINT_NAMES
    assert adapter.joint_permutation == tuple(range(29))
    assert first.qpos_wxyz[:7] == pytest.approx((1.0, 2.0, 0.8, 1.0, 0.0, 0.0, 0.0))
    assert first.qpos_wxyz[7:] == pytest.approx(np.arange(29) / 100)
    assert first.timestamp_s == 0.0 and second.timestamp_s == pytest.approx(0.02)
    assert not first.qpos_wxyz.flags.writeable


def test_adapter_rejects_nan_and_holds_stale_or_out_of_order_reference():
    adapter = FoheartTeleopitAdapter(max_reference_age_s=0.1)
    first = adapter.adapt(processed(), now_s=10.0)
    stale = adapter.adapt(processed(10.01, frame_number=2), now_s=10.2)
    out_of_order = adapter.adapt(processed(9.0, frame_number=3), now_s=9.0)
    missing = adapter.adapt(None, now_s=10.3)

    assert all(item.held and np.array_equal(item.qpos_wxyz, first.qpos_wxyz) for item in (stale, out_of_order, missing))

    invalid = processed(10.02, frame_number=4)
    invalid.qpos_wxyz[7] = invalid.dof_pos[0] = np.nan
    with pytest.raises(ValueError, match="finite"):
        adapter.adapt(invalid, now_s=10.02)


def test_policy_simulator_uses_167d_observation_29d_action_and_free_base():
    if not DEFAULT_TELEOPIT_POLICY.is_file():
        pytest.skip("TeleopIt policy assets are not installed")
    simulator = FoheartTeleopitPolicySimulator(DEFAULT_TELEOPIT_ROOT, DEFAULT_TELEOPIT_POLICY)
    try:
        reference = processed(joints=np.asarray(simulator.robot.default_dof_pos), timestamp_s=0.0)
        metrics = simulator.command_whole_body(reference, steps=5)
    finally:
        simulator.close()

    assert metrics.finite and metrics.observation_finite and metrics.action_finite
    assert metrics.observation_shape == (167,) and metrics.action_shape == (29,)
    assert metrics.base_pinned is False and simulator.robot.model.nq == 36
    assert metrics.fall_status == "NOT_SOURCE_DEFINED"


def integration_args(source, *, replay=None):
    return Namespace(
        mode="policy-sim",
        source=source,
        bind="127.0.0.1",
        port=5001,
        format="binary",
        timeout=0.1,
        replay=replay,
        max_frames=None,
        duration=None,
        fps=50.0,
        human_height=None,
        model=DEFAULT_GMR_MODEL,
        dynamic_model=Path("unused.xml"),
        sim_steps=1,
        policy_steps_per_reference=25,
        teleopit_root=DEFAULT_TELEOPIT_ROOT,
        policy=DEFAULT_TELEOPIT_POLICY,
        soft_limit_margin=0.0,
        ema_alpha=None,
        max_joint_rate=None,
        disable_heading_normalization=False,
        viewer=False,
        record=None,
    )


@pytest.mark.parametrize("source_name", ("synthetic", "replay"))
def test_actual_gmr_synthetic_and_mvudp_reach_free_base_teleopit_policy(tmp_path, source_name):
    pytest.importorskip("general_motion_retargeting")
    if not DEFAULT_TELEOPIT_POLICY.is_file():
        pytest.skip("TeleopIt policy assets are not installed")
    capture = None
    if source_name == "replay":
        capture = tmp_path / "policy.mvudp"
        with MotionVenusCaptureWriter(capture) as writer:
            for number, pose in enumerate(GMR_SYNTHETIC_POSES):
                frame = synthetic_frame(
                    pose,
                    frame_number=number,
                    timestamp_ns=1_000_000_000 + number * 20_000_000,
                )
                writer.write(
                    MotionVenusDatagram(
                        encode_binary_frame(frame), frame.received_ns, number, frame.sender
                    )
                )

    status, rows = run(integration_args(source_name, replay=capture), load_config())
    assert status == 0 and len(rows) == len(GMR_SYNTHETIC_POSES)
    assert all(row["policy_sim_status"] == "TELEOPIT_POLICY_SIM_VALIDATED" for row in rows)
    assert all(row["policy_observation_shape"] == (167,) for row in rows)
    assert all(row["policy_action_shape"] == (29,) for row in rows)
    assert all(row["sim_base_pinned"] is False and row["sim_finite"] for row in rows)
