"""Stateful wrapper for HumDex's pinned 36-qpos to 35D TWIST2 helper."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

import numpy as np

from foheart.whole_body.gmr import G1_JOINT_NAMES, G1KinematicReference
from foheart.whole_body.safety import SAFE_IDLE_BODY_35, TWIST2_REFERENCE_SIZE


class TWIST2ReferenceAdapter:
    """Produce ``[vxy_local, z, roll, pitch, wz_local, joints29]``.

    ``extractor`` must be HumDex's pinned
    ``deploy_real.common.teleop_compat.extract_mimic_obs_whole_body``.
    It is injected so this package does not guess an upstream checkout path.
    """

    def __init__(self, extractor: Callable[..., Any]) -> None:
        if not callable(extractor):
            raise TypeError("TWIST2 extractor must be callable")
        self._extractor = extractor
        self._last_qpos: np.ndarray | None = None

    def adapt(self, reference: G1KinematicReference, *, dt_s: float) -> np.ndarray:
        dt = float(dt_s)
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError("dt_s must be finite and positive")
        qpos, lower, upper = self._validate_reference(reference)

        if self._last_qpos is None:
            output = SAFE_IDLE_BODY_35.copy()
        else:
            output = np.asarray(
                self._extractor(qpos.copy(), self._last_qpos.copy(), dt=dt),
                dtype=float,
            )
        self._validate_output(output, qpos, lower, upper, first=self._last_qpos is None)
        self._last_qpos = qpos.copy()
        return output.copy()

    @staticmethod
    def _validate_reference(
        reference: G1KinematicReference,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        if not isinstance(reference, G1KinematicReference):
            raise TypeError("reference must be a G1KinematicReference")
        if tuple(reference.joint_names) != G1_JOINT_NAMES:
            raise ValueError("G1 reference joint order does not match pinned TWIST2 order")
        qpos = np.asarray(reference.qpos_wxyz, dtype=float)
        root_pos = np.asarray(reference.root_pos, dtype=float)
        root_quat = np.asarray(reference.root_quat_wxyz, dtype=float)
        joints = np.asarray(reference.dof_pos, dtype=float)
        if qpos.shape != (36,) or not np.isfinite(qpos).all():
            raise ValueError("G1 qpos must be a finite length-36 WXYZ vector")
        if root_pos.shape != (3,) or root_quat.shape != (4,) or joints.shape != (29,):
            raise ValueError("G1 reference component shapes are invalid")
        if not np.array_equal(qpos[:3], root_pos) or not np.array_equal(qpos[3:7], root_quat):
            raise ValueError("G1 reference root components do not match qpos")
        if not np.array_equal(qpos[7:], joints):
            raise ValueError("G1 reference joint values do not match qpos order")
        if not np.isclose(np.linalg.norm(root_quat), 1.0, atol=1e-5):
            raise ValueError("G1 root quaternion WXYZ must be normalized")

        if reference.joint_lower is None or reference.joint_upper is None:
            raise ValueError("G1 reference must provide source-verified 29-DoF limits")
        lower = np.asarray(reference.joint_lower, dtype=float)
        upper = np.asarray(reference.joint_upper, dtype=float)
        if lower.shape != (29,) or upper.shape != (29,):
            raise ValueError("G1 joint limits must each contain exactly 29 values")
        if not np.isfinite(lower).all() or not np.isfinite(upper).all() or np.any(lower > upper):
            raise ValueError("G1 joint limits must be finite and ordered")
        if np.any(joints < lower) or np.any(joints > upper):
            raise ValueError("G1 reference violates source-verified joint limits")
        if np.any(SAFE_IDLE_BODY_35[6:] < lower) or np.any(SAFE_IDLE_BODY_35[6:] > upper):
            raise ValueError("HumDex safe-idle preset 0 violates source-verified joint limits")
        return qpos.copy(), lower, upper

    @staticmethod
    def _validate_output(
        output: np.ndarray,
        qpos: np.ndarray,
        lower: np.ndarray,
        upper: np.ndarray,
        *,
        first: bool,
    ) -> None:
        if output.shape != (TWIST2_REFERENCE_SIZE,) or not np.isfinite(output).all():
            raise ValueError("HumDex extractor must return a finite length-35 TWIST2 reference")
        if not first and not np.allclose(output[6:], qpos[7:], rtol=0.0, atol=1e-6):
            raise ValueError("HumDex extractor changed the pinned TWIST2 joint order or values")
        if np.any(output[6:] < lower) or np.any(output[6:] > upper):
            raise ValueError("TWIST2 reference violates source-verified joint limits")
