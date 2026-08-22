from pathlib import Path
import pickle

import numpy as np
import pytest

pytest.importorskip("mujoco")

from foheart.whole_body.gmr import (
    G1_JOINT_NAMES,
    G1_LINK_BODY_NAMES,
    G1KinematicReference,
    G1ReferenceMuJoCo,
)


PROJECT = Path(__file__).resolve().parents[3]
MODEL = PROJECT / "third_party/HumDex/GMR/assets/unitree_g1/g1_mocap_29dof.xml"
EXAMPLE = PROJECT / "third_party/TWIST2/assets/example_motions/0807_yanjie_walk_001.pkl"


def reference(qpos, model):
    return G1KinematicReference(
        qpos,
        qpos[:3],
        qpos[3:7],
        qpos[7:],
        G1_JOINT_NAMES,
        model.joint_lower,
        model.joint_upper,
    )


def test_pinned_gmr_model_accepts_qpos_and_exposes_exact_twist2_fk_order():
    model = G1ReferenceMuJoCo(MODEL)
    try:
        qpos = model.model.qpos0.copy()
        applied = model.apply(reference(qpos, model))
        local = model.local_body_positions(qpos)
        assert applied.shape == (36,) and np.isfinite(applied).all()
        assert local.shape == (38, 3) and np.isfinite(local).all()
        assert tuple(G1_LINK_BODY_NAMES) == tuple(
            model.mujoco.mj_id2name(model.model, model.mujoco.mjtObj.mjOBJ_BODY, index)
            for index in range(1, model.model.nbody)
        )
    finally:
        model.close()


def test_mujoco_fk_matches_pinned_twist2_example_convention():
    if not EXAMPLE.is_file():
        pytest.skip("pinned TWIST2 example motion is absent")
    with EXAMPLE.open("rb") as stream:
        motion = pickle.load(stream)
    model = G1ReferenceMuJoCo(MODEL)
    try:
        qpos = np.concatenate(
            (motion["root_pos"][0], motion["root_rot"][0][[3, 0, 1, 2]], motion["dof_pos"][0])
        )
        assert motion["link_body_list"] == list(G1_LINK_BODY_NAMES)
        assert model.local_body_positions(qpos) == pytest.approx(
            motion["local_body_pos"][0], abs=2e-6
        )
    finally:
        model.close()


def test_reference_model_rejects_bad_quaternion_and_joint_limit():
    model = G1ReferenceMuJoCo(MODEL)
    try:
        qpos = model.model.qpos0.copy()
        qpos[3:7] = 0.0
        with pytest.raises(ValueError, match="quaternion"):
            model.apply(qpos)
        qpos = model.model.qpos0.copy()
        qpos[7] = model.joint_upper[0] + 0.01
        with pytest.raises(ValueError, match="joint limits"):
            model.apply(qpos)
    finally:
        model.close()
