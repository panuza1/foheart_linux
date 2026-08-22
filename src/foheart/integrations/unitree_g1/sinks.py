"""The explicit post-IK split between MuJoCo and guarded real hardware."""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any, Callable

import numpy as np

from .adapter import G1_ARM_JOINT_NAMES, IKResult
from .sim_bridge import G1MuJoCoBridge, SimStepMetrics


REAL_CONTROLLER_SOURCE = "xr_teleoperate/teleop/robot_control/robot_arm.py:G1_29_ArmController"
REAL_COMMAND_BACKEND = "BLOCKED_NEEDS_FEEDBACK_TIMESTAMP_AND_CONTROLLED_CLOSE"


class SimG1Sink:
    """In-process MuJoCo only: no DDS imports, publishers, or robot APIs."""

    mode = "SIMULATION_ONLY"

    def __init__(self, bridge: G1MuJoCoBridge, *, steps_per_update: int = 8):
        if not 1 <= steps_per_update <= 5000:
            raise ValueError("steps_per_update must be in 1..5000")
        self.bridge, self.steps_per_update = bridge, steps_per_update
        self.last_metrics: SimStepMetrics | None = None

    def update(self, result: IKResult) -> SimStepMetrics:
        if not result.valid:
            raise ValueError(f"invalid IK result cannot enter simulation sink: {result.reason}")
        self.last_metrics = self.bridge.command(result.joint_positions, steps=self.steps_per_update)
        if not self.last_metrics.finite:
            raise RuntimeError("MuJoCo produced a non-finite robot state")
        return self.last_metrics

    def close(self) -> None:
        pass


@dataclass(frozen=True)
class RealSafetyState:
    source_live: bool
    profile_valid: bool
    arms_present: bool
    input_fresh: bool
    transforms_valid: bool
    workspace_valid: bool
    ik_valid: bool
    joint_limits_valid: bool
    joint_rate_valid: bool
    joint_delta_valid: bool
    feedback_fresh: bool
    controller_ready: bool

    @property
    def all_pass(self) -> bool:
        return all(vars(self).values())

    @property
    def failed(self) -> tuple[str, ...]:
        return tuple(name for name, value in vars(self).items() if not value)


class RealBackendBlocked(RuntimeError):
    pass


def production_real_backend_factory() -> Any:
    """Fail closed until the existing controller gains safe lifecycle hooks."""

    raise RealBackendBlocked(
        f"{REAL_COMMAND_BACKEND}: {REAL_CONTROLLER_SOURCE} starts publishing in its constructor, "
        "does not timestamp feedback, and has no controlled publisher stop method"
    )


class RealG1Sink:
    """Arm-only interlock usable with an injected backend; production remains blocked."""

    mode = "REAL_ARM_ONLY"

    def __init__(
        self,
        backend_factory: Callable[[], Any] = production_real_backend_factory,
        *,
        ramp_time_s: float = 3.0,
        max_joint_delta_rad: float = 0.1,
    ):
        if min(ramp_time_s, max_joint_delta_rad) <= 0:
            raise ValueError("real ramp time and joint delta must be positive")
        self.backend_factory = backend_factory
        self.ramp_time_s = ramp_time_s
        self.max_joint_delta_rad = max_joint_delta_rad
        self.state = "WAITING_FOR_OPERATOR"
        self.backend: Any | None = None
        self.initial_q: np.ndarray | None = None
        self.last_safe_q: np.ndarray | None = None
        self.enabled_ns: int | None = None

    def mark_ready(self, safety: RealSafetyState) -> None:
        if not safety.all_pass:
            raise ValueError("real safety gates failed: " + ", ".join(safety.failed))
        if self.state != "WAITING_FOR_OPERATOR":
            raise RuntimeError(f"cannot mark ready from state {self.state}")
        self.state = "READY"

    def enable(self, confirmation: str, safety: RealSafetyState) -> None:
        if confirmation != "ENABLE":
            raise ValueError("real mode requires the exact operator confirmation ENABLE")
        if self.state != "READY" or not safety.all_pass:
            raise RuntimeError("real sink is not ready or a safety gate failed")
        backend = self.backend_factory()
        current = np.asarray(backend.current_arm_positions(), dtype=float)
        if current.shape != (14,) or not np.isfinite(current).all():
            backend.close()
            raise RuntimeError("real backend returned invalid G1 arm feedback")
        self.backend = backend
        self.initial_q = self.last_safe_q = current.copy()
        self.enabled_ns = time.monotonic_ns()
        self.state = "ENABLED"

    def update(self, result: IKResult, safety: RealSafetyState, *, monotonic_ns: int | None = None) -> np.ndarray:
        if self.state not in ("ENABLED", "HOLDING") or self.backend is None:
            raise RuntimeError("real sink is not enabled")
        if not safety.all_pass or not result.valid:
            self.state = "HOLDING"
            if self.last_safe_q is None:
                raise RuntimeError("no safe real target exists to hold")
            return self.last_safe_q.copy()  # Existing controller retains its last target.
        target = np.asarray(result.joint_positions, dtype=float)
        torque = np.asarray(result.feedforward_torque, dtype=float)
        if (
            target.shape != (14,)
            or torque.shape != (14,)
            or not np.isfinite(target).all()
            or not np.isfinite(torque).all()
        ):
            self.state = "HOLDING"
            return self.last_safe_q.copy()
        now = time.monotonic_ns() if monotonic_ns is None else monotonic_ns
        amount = min(1.0, max(0.0, (now - self.enabled_ns) / 1e9 / self.ramp_time_s))
        blended = self.initial_q + amount * (target - self.initial_q)
        command = self.last_safe_q + np.clip(
            blended - self.last_safe_q,
            -self.max_joint_delta_rad,
            self.max_joint_delta_rad,
        )
        self.backend.send_arm_positions(command, torque)
        self.last_safe_q = command.copy()
        self.state = "ENABLED"
        return command

    def close(self) -> None:
        if self.backend is not None:
            self.backend.close()
            self.backend = None
        self.state = "CLOSED"


assert len(G1_ARM_JOINT_NAMES) == 14
