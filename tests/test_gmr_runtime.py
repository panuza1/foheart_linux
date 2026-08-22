from argparse import Namespace
from pathlib import Path

import numpy as np
import pytest

gmr_package = pytest.importorskip("general_motion_retargeting")
pytest.importorskip("mujoco")

from foheart.config import load_config
from foheart.motionvenus.gmr import MotionVenusGMRAdapter
from foheart.motionvenus.protocol import encode_binary_frame
from foheart.motionvenus.skeleton import HumanSkeletonFrame
from foheart.motionvenus.synthetic import GMR_SYNTHETIC_POSES, synthetic_frame
from foheart.motionvenus.transport import MotionVenusCaptureWriter, MotionVenusDatagram
from foheart.tools.motionvenus_g1_reference import DEFAULT_GMR_MODEL, run
from foheart.whole_body.gmr import (
    G1_JOINT_NAMES,
    G1ReferenceMuJoCo,
    GMRWholeBodyRetargeter,
)
from foheart.whole_body.reference import G1ReferenceProcessor


def human_data(pose="neutral"):
    skeleton = HumanSkeletonFrame.from_motionvenus(
        synthetic_frame(pose, frame_number=1, timestamp_ns=1), status="SYNTHETIC"
    )
    return MotionVenusGMRAdapter().adapt(skeleton)


def retargeter():
    return GMRWholeBodyRetargeter(
        gmr_package.GeneralMotionRetargeting(
            src_human="xsens_mvn", tgt_robot="unitree_g1", verbose=False
        )
    )


def cli_args(tmp_path, **changes):
    values = {
        "mode": "direct-sim", "sim_steps": 2,
        "source": "synthetic", "bind": "127.0.0.1", "port": 5001,
        "format": "binary", "timeout": 0.1, "replay": None,
        "max_frames": None, "duration": None, "fps": 50.0,
        "human_height": None, "model": DEFAULT_GMR_MODEL, "viewer": False,
        "record": tmp_path / "gmr.pkl",
    }
    values.update(changes)
    return Namespace(**values)


def test_gmr_import_model_qpos_contract_and_fresh_solver_determinism():
    first_solver = retargeter()
    second_solver = retargeter()
    first = first_solver.retarget(human_data()).qpos_wxyz
    second = second_solver.retarget(human_data()).qpos_wxyz
    assert first_solver.retargeter.model.nq == 36
    assert first_solver.retargeter.model.nu == 29
    assert tuple(first_solver.retargeter.robot_motor_names) == G1_JOINT_NAMES
    assert first.shape == (36,) and np.isfinite(first).all()
    assert np.linalg.norm(first[3:7]) == pytest.approx(1.0)
    assert np.allclose(first, second, rtol=0.0, atol=1e-10)


def test_all_ten_direct_poses_retarget_and_apply_as_kinematic_references():
    gmr = retargeter()
    model = G1ReferenceMuJoCo(DEFAULT_GMR_MODEL)
    processor = G1ReferenceProcessor(
        gmr.joint_lower, gmr.joint_upper, stale_after_s=0.1
    )
    previous = None
    try:
        for number, pose in enumerate(GMR_SYNTHETIC_POSES):
            reference = gmr.retarget(human_data(pose))
            processed = processor.process(reference, source_timestamp_s=number / 50)
            assert processed is not None and not processed.held
            applied = model.apply(processed.qpos_wxyz)
            assert applied.shape == (36,) and np.isfinite(applied).all()
            assert np.all(applied[7:] >= model.joint_lower)
            assert np.all(applied[7:] <= model.joint_upper)
            if previous is not None:
                assert np.isfinite(np.max(np.abs(applied[7:] - previous[7:])))
            previous = applied
    finally:
        model.close()


def test_actual_gmr_mujoco_synthetic_and_mvudp_replay_share_cli_path(tmp_path):
    status, synthetic_rows = run(cli_args(tmp_path), load_config())
    assert status == 0 and len(synthetic_rows) == 10

    capture = tmp_path / "synthetic.mvudp"
    with MotionVenusCaptureWriter(capture) as writer:
        for number, pose in enumerate(GMR_SYNTHETIC_POSES):
            frame = synthetic_frame(
                pose, frame_number=number, timestamp_ns=1_000_000_000 + number * 20_000_000
            )
            writer.write(
                MotionVenusDatagram(
                    encode_binary_frame(frame), frame.received_ns, number, frame.sender
                )
            )
    status, replay_rows = run(
        cli_args(tmp_path, source="replay", replay=capture, record=None), load_config()
    )
    assert status == 0
    assert [row["frame"] for row in replay_rows] == list(range(10))
