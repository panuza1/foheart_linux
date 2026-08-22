"""Synchronized safety processing for one complete G1 29-DoF reference."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Sequence

import numpy as np

from .gmr import G1_JOINT_NAMES, G1KinematicReference


G1_QPOS_SIZE = 36
G1_DOF = len(G1_JOINT_NAMES)


def _readonly(value: Any) -> np.ndarray:
    result = np.asarray(value, dtype=float).copy()
    result.setflags(write=False)
    return result


def _joint_vector(
    value: Any,
    name: str,
    *,
    nonnegative: bool,
    allow_scalar: bool = False,
) -> np.ndarray:
    try:
        vector = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must contain finite numbers") from exc
    if vector.shape == () and allow_scalar:
        vector = np.full(G1_DOF, float(vector))
    if vector.shape != (G1_DOF,) or not np.isfinite(vector).all():
        size = "one or 29 values" if allow_scalar else "exactly 29 values"
        raise ValueError(f"{name} must be finite and contain {size}")
    if nonnegative and np.any(vector < 0):
        raise ValueError(f"{name} must be non-negative")
    return vector.copy()


@dataclass(frozen=True)
class ProcessedG1Reference:
    """One coherent 36-qpos output plus per-frame processing diagnostics."""

    qpos_wxyz: np.ndarray
    root_pos: np.ndarray
    root_quat_wxyz: np.ndarray
    dof_pos: np.ndarray
    joint_names: tuple[str, ...]
    status: str
    source_timestamp_s: float
    source_frame_number: int | None
    reason: str
    hard_clamped_joints: tuple[str, ...] = ()
    soft_clamped_joints: tuple[str, ...] = ()
    rate_limited_joints: tuple[str, ...] = ()
    smoothing_applied: bool = False

    @property
    def clamped_joints(self) -> tuple[str, ...]:
        clamped = set(self.hard_clamped_joints) | set(self.soft_clamped_joints)
        return tuple(name for name in self.joint_names if name in clamped)

    @property
    def clamp_count(self) -> int:
        return len(self.clamped_joints)

    @property
    def rate_limit_count(self) -> int:
        return len(self.rate_limited_joints)

    @property
    def held(self) -> bool:
        return self.status == "HOLD"


class G1ReferenceProcessor:
    """Clamp, filter, rate-limit, and HOLD one synchronized 29-joint target."""

    def __init__(
        self,
        joint_lower: Sequence[float] | np.ndarray,
        joint_upper: Sequence[float] | np.ndarray,
        *,
        stale_after_s: float,
        soft_limit_margin: float | Sequence[float] | np.ndarray = 0.0,
        ema_alpha: float | None = None,
        max_joint_rate: float | Sequence[float] | np.ndarray | None = None,
    ) -> None:
        self.joint_lower = _joint_vector(joint_lower, "joint_lower", nonnegative=False)
        self.joint_upper = _joint_vector(joint_upper, "joint_upper", nonnegative=False)
        if np.any(self.joint_lower > self.joint_upper):
            raise ValueError("joint limits must be ordered")

        margin = _joint_vector(
            soft_limit_margin,
            "soft_limit_margin",
            nonnegative=True,
            allow_scalar=True,
        )
        self.soft_lower = self.joint_lower + margin
        self.soft_upper = self.joint_upper - margin
        if np.any(self.soft_lower > self.soft_upper):
            raise ValueError("soft_limit_margin leaves an empty joint range")
        self.soft_limit_margin = margin

        if ema_alpha is not None:
            ema_alpha = float(ema_alpha)
            if not np.isfinite(ema_alpha) or not 0.0 <= ema_alpha <= 1.0:
                raise ValueError("ema_alpha must be finite and in [0, 1]")
        self.ema_alpha = ema_alpha

        self.max_joint_rate = (
            None
            if max_joint_rate is None
            else _joint_vector(
                max_joint_rate,
                "max_joint_rate",
                nonnegative=False,
                allow_scalar=True,
            )
        )
        if self.max_joint_rate is not None and np.any(self.max_joint_rate <= 0):
            raise ValueError("max_joint_rate must be positive")

        self.stale_after_s = float(stale_after_s)
        if not np.isfinite(self.stale_after_s) or self.stale_after_s <= 0:
            raise ValueError("stale_after_s must be finite and positive")

        self.status = "HOLD"
        self.reason = "waiting for first valid G1 reference"
        self.accepted_frames = 0
        self.hold_events = 0
        self.clamp_events = 0
        self.rate_limit_events = 0
        self._last_safe: ProcessedG1Reference | None = None
        self._last_source_timestamp_s: float | None = None
        self._last_source_frame_number: int | None = None

    def process(
        self,
        reference: G1KinematicReference | None,
        *,
        source_timestamp_s: float,
        now_s: float | None = None,
        source_frame_number: int | None = None,
        source_valid: bool = True,
        error: str | None = None,
    ) -> ProcessedG1Reference | None:
        """Accept one source frame or return the last safe output in HOLD."""

        if not isinstance(source_valid, (bool, np.bool_)):
            raise TypeError("source_valid must be bool")
        if error is not None:
            return self.hold(error)
        if not source_valid:
            return self.hold("source frame is invalid")
        if reference is None:
            return self.hold("GMR failed")

        try:
            timestamp = float(source_timestamp_s)
            now = timestamp if now_s is None else float(now_s)
        except (TypeError, ValueError) as exc:
            return self.hold(f"invalid source timing: {exc}")
        if not np.isfinite(timestamp) or not np.isfinite(now):
            return self.hold("source timing must be finite")
        age = now - timestamp
        if age < 0:
            return self.hold("source timestamp is in the future")
        if age > self.stale_after_s:
            return self.hold(
                f"source is stale ({age:.6f}s > {self.stale_after_s:.6f}s)"
            )
        if self._last_source_timestamp_s is not None and timestamp <= self._last_source_timestamp_s:
            kind = "duplicate" if timestamp == self._last_source_timestamp_s else "out of order"
            return self.hold(f"source timestamp is {kind}")

        try:
            frame_number = self._validate_frame_number(source_frame_number)
        except ValueError as exc:
            return self.hold(str(exc))
        if frame_number is not None and self._last_source_frame_number is not None:
            delta = (frame_number - self._last_source_frame_number) & 0xFFFFFFFF
            if delta == 0:
                return self.hold("source frame number is duplicate")
            if delta >= 0x80000000:
                return self.hold("source frame number is out of order")

        try:
            qpos, joints = self._validate_reference(reference)
        except (TypeError, ValueError) as exc:
            return self.hold(str(exc))

        hard_mask = (joints < self.joint_lower) | (joints > self.joint_upper)
        limited = np.clip(joints, self.joint_lower, self.joint_upper)
        soft_mask = (limited < self.soft_lower) | (limited > self.soft_upper)
        limited = np.clip(limited, self.soft_lower, self.soft_upper)
        hard_clamped = self._joint_names(hard_mask)
        soft_clamped = self._joint_names(soft_mask)

        previous = None if self._last_safe is None else self._last_safe.dof_pos
        smoothing_applied = previous is not None and self.ema_alpha is not None
        if smoothing_applied:
            limited = self.ema_alpha * limited + (1.0 - self.ema_alpha) * previous

        rate_mask = np.zeros(G1_DOF, dtype=bool)
        if previous is not None and self.max_joint_rate is not None:
            dt = timestamp - self._last_source_timestamp_s
            if not np.isfinite(dt) or dt <= 0:
                return self.hold("source dt must be finite and positive")
            maximum_delta = self.max_joint_rate * dt
            delta = limited - previous
            rate_mask = np.abs(delta) > maximum_delta
            limited = previous + np.clip(delta, -maximum_delta, maximum_delta)

        qpos[7:] = limited
        if self._last_safe is not None and qpos[3:7] @ self._last_safe.root_quat_wxyz < 0:
            qpos[3:7] *= -1
        rate_limited = self._joint_names(rate_mask)
        output = ProcessedG1Reference(
            _readonly(qpos),
            _readonly(qpos[:3]),
            _readonly(qpos[3:7]),
            _readonly(qpos[7:]),
            G1_JOINT_NAMES,
            "FOLLOW",
            timestamp,
            frame_number,
            "accepted synchronized whole-body reference",
            hard_clamped,
            soft_clamped,
            rate_limited,
            smoothing_applied,
        )
        self.status, self.reason = output.status, output.reason
        self._last_safe = output
        self._last_source_timestamp_s = timestamp
        if frame_number is not None:
            self._last_source_frame_number = frame_number
        self.accepted_frames += 1
        self.clamp_events += output.clamp_count
        self.rate_limit_events += output.rate_limit_count
        return output

    def check_stale(self, now_s: float) -> ProcessedG1Reference | None:
        """Enter HOLD once no accepted source frame has arrived before the deadline."""

        try:
            now = float(now_s)
        except (TypeError, ValueError) as exc:
            return self.hold(f"invalid source timing: {exc}")
        if not np.isfinite(now):
            return self.hold("source timing must be finite")
        if self._last_source_timestamp_s is None:
            return None
        age = now - self._last_source_timestamp_s
        if age < 0:
            return self.hold("source clock moved backwards")
        if age > self.stale_after_s:
            return self.hold(
                f"source packet timeout ({age:.6f}s > {self.stale_after_s:.6f}s)"
            )
        return self._hold_output() if self.status == "HOLD" else self._last_safe

    def hold(self, reason: str) -> ProcessedG1Reference | None:
        """Retain, but never mutate, the last safe synchronized output."""

        entering_hold = self.status != "HOLD"
        self.status = "HOLD"
        self.reason = str(reason) or "source reference is invalid"
        self.hold_events += int(entering_hold)
        return self._hold_output()

    def _hold_output(self) -> ProcessedG1Reference | None:
        if self._last_safe is None:
            return None
        previous = self._last_safe
        return ProcessedG1Reference(
            previous.qpos_wxyz,
            previous.root_pos,
            previous.root_quat_wxyz,
            previous.dof_pos,
            previous.joint_names,
            "HOLD",
            previous.source_timestamp_s,
            previous.source_frame_number,
            self.reason,
        )

    def _validate_reference(self, reference: G1KinematicReference) -> tuple[np.ndarray, np.ndarray]:
        if not isinstance(reference, G1KinematicReference):
            raise TypeError("reference must be a G1KinematicReference")
        if tuple(reference.joint_names) != G1_JOINT_NAMES:
            raise ValueError("G1 reference joint order must match the pinned 29-DoF order")

        qpos = np.asarray(reference.qpos_wxyz, dtype=float)
        root_pos = np.asarray(reference.root_pos, dtype=float)
        root_quat = np.asarray(reference.root_quat_wxyz, dtype=float)
        joints = np.asarray(reference.dof_pos, dtype=float)
        if qpos.shape != (G1_QPOS_SIZE,) or not np.isfinite(qpos).all():
            raise ValueError("G1 qpos must be a finite length-36 vector")
        if root_pos.shape != (3,) or root_quat.shape != (4,) or joints.shape != (G1_DOF,):
            raise ValueError("G1 reference components must contain root XYZ, WXYZ, and 29 joints")
        if not np.isfinite(root_pos).all() or not np.isfinite(root_quat).all() or not np.isfinite(joints).all():
            raise ValueError("G1 reference components must be finite")
        if not np.array_equal(qpos[:3], root_pos) or not np.array_equal(qpos[3:7], root_quat):
            raise ValueError("G1 reference root components do not match qpos")
        if not np.array_equal(qpos[7:], joints):
            raise ValueError("G1 reference joint components do not match qpos")
        if not np.isclose(np.linalg.norm(root_quat), 1.0, atol=1e-5):
            raise ValueError("G1 root quaternion must be normalized WXYZ")
        self._validate_reference_limits(reference)
        return qpos.copy(), joints.copy()

    def _validate_reference_limits(self, reference: G1KinematicReference) -> None:
        if (reference.joint_lower is None) != (reference.joint_upper is None):
            raise ValueError("G1 reference must provide both model limit arrays or neither")
        if reference.joint_lower is None:
            return
        lower = _joint_vector(reference.joint_lower, "reference joint_lower", nonnegative=False)
        upper = _joint_vector(reference.joint_upper, "reference joint_upper", nonnegative=False)
        if np.any(lower > upper):
            raise ValueError("G1 reference model limits must be ordered")
        if np.any(self.joint_lower < lower) or np.any(self.joint_upper > upper):
            raise ValueError("configured hard limits must stay inside the GMR model limits")

    @staticmethod
    def _validate_frame_number(value: int | None) -> int | None:
        if value is None:
            return None
        if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, np.integer)):
            raise ValueError("source frame number must be an integer")
        result = int(value)
        if not 0 <= result <= 0xFFFFFFFF:
            raise ValueError("source frame number must be in uint32 range")
        return result

    @staticmethod
    def _joint_names(mask: np.ndarray) -> tuple[str, ...]:
        return tuple(name for name, changed in zip(G1_JOINT_NAMES, mask) if changed)


assert G1_DOF == 29
