import importlib
import importlib.util
from pathlib import Path

import numpy as np
import pytest

from foheart.integrations.twist2.reference import TWIST2ReferenceAdapter
from foheart.whole_body.gmr import G1_JOINT_NAMES, G1KinematicReference
from foheart.whole_body.safety import SAFE_IDLE_BODY_35


def reference(*, root_x=0.0, joints=None, names=G1_JOINT_NAMES, limits=True):
    qpos = np.zeros(36)
    qpos[:3] = (root_x, 0.0, 0.8)
    qpos[3] = 1.0
    qpos[7:] = np.zeros(29) if joints is None else joints
    bound = np.full(29, 2.0)
    return G1KinematicReference(
        qpos_wxyz=qpos,
        root_pos=qpos[:3],
        root_quat_wxyz=qpos[3:7],
        dof_pos=qpos[7:],
        joint_names=names,
        joint_lower=-bound if limits else None,
        joint_upper=bound if limits else None,
    )


def test_first_frame_is_exact_humdex_safe_idle_then_calls_injected_helper():
    calls = []

    def extractor(qpos, last_qpos, *, dt):
        calls.append((qpos.copy(), last_qpos.copy(), dt))
        return np.concatenate((np.arange(6, dtype=float), qpos[7:]))

    adapter = TWIST2ReferenceAdapter(extractor)
    first = reference(root_x=1.0)
    second = reference(root_x=1.1, joints=np.linspace(-0.1, 0.1, 29))

    assert np.array_equal(adapter.adapt(first, dt_s=0.02), SAFE_IDLE_BODY_35)
    output = adapter.adapt(second, dt_s=0.02)

    assert len(calls) == 1
    assert np.array_equal(calls[0][0], second.qpos_wxyz)
    assert np.array_equal(calls[0][1], first.qpos_wxyz)
    assert calls[0][2] == 0.02
    assert np.array_equal(output[:6], np.arange(6, dtype=float))
    assert np.array_equal(output[6:], second.dof_pos)


@pytest.mark.parametrize("failure", ("dt", "order", "limits", "shape", "finite", "swapped"))
def test_invalid_contract_fails_closed(failure):
    def extractor(qpos, last_qpos, *, dt):
        output = np.concatenate((np.zeros(6), qpos[7:]))
        if failure == "shape":
            return output[:-1]
        if failure == "finite":
            output[0] = np.nan
        if failure == "swapped":
            output[6:8] = output[7:5:-1]
        return output

    adapter = TWIST2ReferenceAdapter(extractor)
    adapter.adapt(reference(), dt_s=0.02)
    joints = np.linspace(-0.2, 0.2, 29)
    current = reference(joints=joints)
    dt = 0.0 if failure == "dt" else 0.02
    if failure == "order":
        current = reference(joints=joints, names=tuple(reversed(G1_JOINT_NAMES)))
    elif failure == "limits":
        joints[0] = 3.0
        current = reference(joints=joints)

    with pytest.raises((TypeError, ValueError)):
        adapter.adapt(current, dt_s=dt)


def test_limits_are_mandatory_and_safe_idle_must_fit_them():
    with pytest.raises(ValueError, match="source-verified"):
        TWIST2ReferenceAdapter(lambda *_args, **_kwargs: np.zeros(35)).adapt(
            reference(limits=False), dt_s=0.02
        )

    too_tight = reference()
    too_tight.joint_lower[:] = -0.1
    too_tight.joint_upper[:] = 0.1
    with pytest.raises(ValueError, match="safe-idle"):
        TWIST2ReferenceAdapter(lambda *_args, **_kwargs: np.zeros(35)).adapt(
            too_tight, dt_s=0.02
        )


def test_matches_actual_pinned_humdex_helper_when_optional_deps_exist(monkeypatch):
    if importlib.util.find_spec("scipy") is None or importlib.util.find_spec("torch") is None:
        pytest.skip("pinned HumDex helper dependencies scipy/torch are absent from project .venv")
    deploy_real = Path(
        "/home/panu/Documents/fibo/project_humanoid/third_party/HumDex/deploy_real"
    )
    if not deploy_real.is_dir():
        pytest.skip("authorized pinned HumDex checkout is absent")
    monkeypatch.syspath_prepend(str(deploy_real))
    helper = importlib.import_module("common.teleop_compat").extract_mimic_obs_whole_body

    previous = reference(root_x=0.0)
    current = reference(root_x=0.01, joints=np.linspace(-0.1, 0.1, 29))
    adapter = TWIST2ReferenceAdapter(helper)
    adapter.adapt(previous, dt_s=0.02)
    actual = adapter.adapt(current, dt_s=0.02)
    expected = helper(current.qpos_wxyz, previous.qpos_wxyz, dt=0.02)

    assert np.allclose(actual, expected)
