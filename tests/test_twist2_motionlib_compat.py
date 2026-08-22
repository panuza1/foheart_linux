from pathlib import Path

import pytest

pytest.importorskip("torch")
pytest.importorskip("mujoco")

from foheart.integrations.twist2 import (
    MotionRecorder,
    create_dataset,
    load_pinned_motionlib,
)
from foheart.whole_body.gmr import G1_LINK_BODY_NAMES, G1ReferenceMuJoCo


PROJECT = Path(__file__).resolve().parents[3]
MODEL = PROJECT / "third_party/HumDex/GMR/assets/unitree_g1/g1_mocap_29dof.xml"
TWIST2 = PROJECT / "third_party/TWIST2"


def write_motion(path, *, frames=10):
    model = G1ReferenceMuJoCo(MODEL)
    try:
        recorder = MotionRecorder(fps=50)
        for frame in range(frames):
            qpos = model.model.qpos0.copy()
            qpos[0] = frame * 0.001
            recorder.append(
                qpos,
                model.local_body_positions(qpos),
                timestamp_ns=1_000_000_000 + frame * 20_000_000,
                source_frame_number=frame,
            )
        recorder.save(path)
    finally:
        model.close()


def test_generated_pickle_loads_with_actual_pinned_motionlib(tmp_path):
    motion = tmp_path / "motion.pkl"
    write_motion(motion)
    loaded = load_pinned_motionlib(motion, TWIST2, motion_smooth=False)
    assert loaded.num_motions() == 1
    assert tuple(loaded._motion_dof_pos.shape) == (10, 29)
    assert tuple(loaded._body_link_list) == G1_LINK_BODY_NAMES


def test_generated_dataset_yaml_loads_with_actual_pinned_motionlib(tmp_path):
    root = tmp_path / "motions"
    root.mkdir()
    first, second = root / "first.pkl", root / "second.pkl"
    write_motion(first)
    write_motion(second)
    dataset = create_dataset(tmp_path / "dataset.yaml", root, [first, second], weights=[1, 0.5])
    loaded = load_pinned_motionlib(dataset, TWIST2, motion_smooth=False)
    assert loaded.num_motions() == 2
    assert tuple(loaded._motion_dof_pos.shape) == (20, 29)
