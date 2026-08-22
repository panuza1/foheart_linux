import numpy as np
import pytest

import foheart.integrations.twist2.motion as motion_module
from foheart.integrations.twist2.motion import MotionRecorder, dof_velocity, load_motion
from foheart.whole_body.gmr import G1_LINK_BODY_NAMES


LINKS = G1_LINK_BODY_NAMES


def _qpos(frame: int) -> np.ndarray:
    value = np.zeros(36)
    value[:3] = (frame * 0.01, 0.0, 0.8)
    value[3:7] = (2.0, 0.0, 0.0, 0.0)  # WXYZ; recorder normalizes it.
    value[7:] = frame * 0.001
    return value


@pytest.mark.parametrize("frames", (10, 100))
def test_motion_record_save_load_and_velocity(tmp_path, frames):
    recorder = MotionRecorder(fps=50.0, link_body_list=LINKS)
    for frame in range(frames):
        recorder.append(
            _qpos(frame),
            np.full((len(LINKS), 3), frame * 0.01),
            timestamp_ns=1_000_000_000 + frame * 20_000_000,
            source_frame_number=frame,
        )
    destination = tmp_path / f"motion_{frames}.pkl"
    recorder.save(destination)
    loaded = load_motion(destination)
    assert set(loaded) == {"fps", "root_pos", "root_rot", "dof_pos", "local_body_pos", "link_body_list"}
    assert loaded["root_pos"].shape == (frames, 3)
    assert loaded["dof_pos"].shape == (frames, 29)
    assert loaded["local_body_pos"].shape == (frames, len(LINKS), 3)
    assert np.allclose(np.linalg.norm(loaded["root_rot"], axis=1), 1.0)
    assert np.allclose(dof_velocity(loaded["dof_pos"], loaded["fps"])[1:-1], 0.05)
    with pytest.raises(FileExistsError):
        recorder.save(destination)
    assert load_motion(destination)["root_pos"].shape == (frames, 3)


def test_motion_recorder_rejects_empty_bad_order_and_nonfinite(tmp_path):
    recorder = MotionRecorder(fps=50, link_body_list=LINKS)
    with pytest.raises(ValueError, match="empty"):
        recorder.save(tmp_path / "empty.pkl")
    assert not (tmp_path / "empty.pkl").exists()
    recorder.append(_qpos(1), np.zeros((len(LINKS), 3)), timestamp_ns=10, source_frame_number=1)
    with pytest.raises(ValueError, match="timestamps"):
        recorder.append(_qpos(2), np.zeros((len(LINKS), 3)), timestamp_ns=10, source_frame_number=2)
    with pytest.raises(ValueError, match="frame numbers"):
        recorder.append(_qpos(1), np.zeros((len(LINKS), 3)), timestamp_ns=11, source_frame_number=1)
    broken = _qpos(2)
    broken[7] = np.nan
    with pytest.raises(ValueError, match="finite"):
        recorder.append(broken, np.zeros((len(LINKS), 3)), timestamp_ns=12, source_frame_number=2)
    with pytest.raises(ValueError, match="at least two"):
        recorder.save(tmp_path / "one_frame.pkl")


def test_interrupted_save_removes_partial_file(tmp_path, monkeypatch):
    recorder = MotionRecorder(fps=50, link_body_list=LINKS)
    for frame in range(2):
        recorder.append(
            _qpos(frame),
            np.zeros((len(LINKS), 3)),
            timestamp_ns=frame + 1,
            source_frame_number=frame,
        )
    destination = tmp_path / "interrupted.pkl"

    def interrupt(_value, stream, protocol):
        stream.write(b"partial")
        raise KeyboardInterrupt

    monkeypatch.setattr(motion_module.pickle, "dump", interrupt)
    with pytest.raises(KeyboardInterrupt):
        recorder.save(destination)
    assert not destination.exists()


def test_single_frame_velocity_is_zero():
    assert np.array_equal(dof_velocity(np.ones((1, 29)), 50), np.zeros((1, 29)))


def test_recorder_requires_exact_pinned_g1_body_order():
    with pytest.raises(ValueError, match="pinned GMR/TWIST2"):
        MotionRecorder(fps=50, link_body_list=tuple(reversed(LINKS)))
