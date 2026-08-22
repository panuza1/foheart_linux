from argparse import Namespace
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from foheart.config import load_config
from foheart.integrations.twist2 import load_motion, load_pinned_motionlib
from foheart.motionvenus.protocol import encode_binary_frame
from foheart.motionvenus.synthetic import GMR_SYNTHETIC_POSES, synthetic_frame
from foheart.motionvenus.transport import MotionVenusCaptureWriter, MotionVenusDatagram
from foheart.tools.motionvenus_g1_reference import DEFAULT_DYNAMIC_MODEL, DEFAULT_GMR_MODEL, run
from foheart.whole_body.gmr import G1_JOINT_NAMES, G1_LINK_BODY_NAMES, G1KinematicReference


TWIST2_ROOT = Path(__file__).resolve().parents[3] / "third_party/TWIST2"


class FakeRetargeter:
    def __init__(self):
        self.frames = 0

    def retarget(self, human_data):
        qpos = np.zeros(36)
        qpos[:3] = human_data["Pelvis"][0]
        qpos[3:7] = human_data["Pelvis"][1]
        qpos[7:] = self.frames * 0.01
        self.frames += 1
        return G1KinematicReference(
            qpos, qpos[:3], qpos[3:7], qpos[7:], G1_JOINT_NAMES,
            np.full(29, -10.0), np.full(29, 10.0),
        )


class FakeReferenceModel:
    is_running = True

    def __init__(self):
        self.frames = []
        self.closed = False

    def apply(self, reference):
        value = np.asarray(getattr(reference, "qpos_wxyz", reference)).copy()
        self.frames.append(value)
        return value

    def local_body_positions(self, _reference):
        return np.zeros((len(G1_LINK_BODY_NAMES), 3))

    def close(self):
        self.closed = True


class FakeDynamicModel:
    joint_lower = np.full(29, -1.5)
    joint_upper = np.full(29, 1.5)

    def __init__(self):
        self.targets = []

    def command_whole_body(self, reference, *, steps):
        target = np.asarray(reference.qpos_wxyz).copy()
        self.targets.append(target)
        return SimpleNamespace(
            maximum_joint_error_rad=0.02,
            mean_joint_error_rad=0.01,
            finite=True,
            steps=steps,
            simulation_duration_s=steps * 0.002,
            root_position_m=(0.0, 0.0, 0.8),
            root_quaternion_wxyz=(1.0, 0.0, 0.0, 0.0),
            base_pinned=True,
            stability_status="BASE_PINNED_NOT_ASSESSED",
        )


def args(tmp_path, **changes):
    values = {
        "source": "synthetic", "bind": "127.0.0.1", "port": 5001,
        "format": "binary", "timeout": 0.1, "replay": None,
        "max_frames": None, "duration": None, "fps": 50.0,
        "human_height": None, "model": Path("unused.xml"), "viewer": False,
        "record": tmp_path / "motion.pkl",
    }
    values.update(changes)
    return Namespace(**values)


def test_synthetic_cli_uses_all_ten_direct_poses_and_records(tmp_path):
    model = FakeReferenceModel()
    status, rows = run(args(tmp_path), load_config(), retargeter=FakeRetargeter(), reference_model=model)
    motion = load_motion(tmp_path / "motion.pkl")
    assert status == 0 and [row["pose"] for row in rows] == list(GMR_SYNTHETIC_POSES)
    assert [row["frame"] for row in rows] == list(range(10))
    assert motion["root_pos"].shape == (10, 3)
    assert motion["dof_pos"].shape == (10, 29)
    assert motion["local_body_pos"].shape == (10, 38, 3)
    assert motion["link_body_list"] == list(G1_LINK_BODY_NAMES)
    assert model.closed


def test_mvudp_replay_uses_same_adapter_processor_and_dynamic_path(tmp_path):
    capture = tmp_path / "synthetic.mvudp"
    with MotionVenusCaptureWriter(capture) as writer:
        for number, pose in enumerate(GMR_SYNTHETIC_POSES):
            frame = synthetic_frame(pose, frame_number=number, timestamp_ns=1_000_000_000 + number)
            writer.write(
                MotionVenusDatagram(
                    encode_binary_frame(frame), frame.received_ns, number, frame.sender
                )
            )
    model, dynamic = FakeReferenceModel(), FakeDynamicModel()
    status, rows = run(
        args(
            tmp_path,
            mode="direct-sim",
            source="replay",
            replay=capture,
            record=None,
            sim_steps=2,
        ),
        load_config(),
        retargeter=FakeRetargeter(),
        reference_model=model,
        dynamic_model=dynamic,
    )
    assert status == 0
    assert [row["frame"] for row in rows] == list(range(10))
    assert all(row["processor_status"] == "FOLLOW" for row in rows)
    assert len(dynamic.targets) == 10 and model.frames == [] and model.closed


def test_direct_sim_records_processed_reference_before_servo_tracking(tmp_path):
    model, dynamic = FakeReferenceModel(), FakeDynamicModel()
    status, rows = run(
        args(tmp_path, mode="direct-sim", sim_steps=4),
        load_config(),
        retargeter=FakeRetargeter(),
        reference_model=model,
        dynamic_model=dynamic,
    )
    motion = load_motion(tmp_path / "motion.pkl")
    targets = np.stack(dynamic.targets)

    assert status == 0 and len(rows) == len(dynamic.targets) == len(GMR_SYNTHETIC_POSES)
    assert all(row["processor_status"] == "FOLLOW" for row in rows)
    assert all(row["direct_dynamic_status"] == "DIRECT_DYNAMIC_SIM_VALIDATED" for row in rows)
    assert np.array_equal(motion["root_pos"], targets[:, :3])
    assert np.array_equal(motion["dof_pos"], targets[:, 7:])
    assert motion["root_rot"] == pytest.approx(targets[:, [4, 5, 6, 3]])
    assert motion["local_body_pos"].shape == (10, 38, 3)
    assert motion["link_body_list"] == list(G1_LINK_BODY_NAMES)
    assert model.frames == [] and model.closed


@pytest.mark.parametrize("source_name", ("synthetic", "replay"))
def test_synthetic_and_mvudp_processor_actual_dynamic_mujoco_and_recording(
    tmp_path, source_name
):
    pytest.importorskip("mujoco")
    pytest.importorskip("torch")
    capture = None
    if source_name == "replay":
        capture = tmp_path / "actual.mvudp"
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

    status, rows = run(
        args(
            tmp_path,
            mode="direct-sim",
            source=source_name,
            replay=capture,
            model=DEFAULT_GMR_MODEL,
            dynamic_model=DEFAULT_DYNAMIC_MODEL,
            sim_steps=2,
        ),
        load_config(),
        retargeter=FakeRetargeter(),
    )
    motion = load_motion(tmp_path / "motion.pkl")
    pinned = load_pinned_motionlib(tmp_path / "motion.pkl", TWIST2_ROOT, motion_smooth=False)
    assert status == 0 and len(rows) == motion["dof_pos"].shape[0] == 10
    assert all(row["direct_dynamic_status"] == "DIRECT_DYNAMIC_SIM_VALIDATED" for row in rows)
    assert np.isfinite(motion["root_pos"]).all() and motion["local_body_pos"].shape == (10, 38, 3)
    assert tuple(pinned._motion_dof_pos.shape) == (10, 29)
