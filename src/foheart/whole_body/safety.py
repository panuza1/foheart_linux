"""Fail-closed safety gate for simulated TWIST2 reference transport.

``DAMP`` means a cosine ramp to HumDex's safe-idle reference in simulation.
It is not a Unitree damping command and this module imports no robot backend.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import logging
import math
from typing import Sequence

import numpy as np


LOGGER = logging.getLogger(__name__)
TWIST2_REFERENCE_SIZE = 35
G1_DOF = 29
DAMP_RAMP_SECONDS = 3.0

# HumDex pinned commit 5bcdc8b32db435bd1b48a265fc554cc467a3ad92,
# deploy_real/common/teleop_compat.py:SAFE_IDLE_BODY_35_PRESETS[0].
SAFE_IDLE_BODY_35 = np.asarray(
    [
        0.0, 0.0, 0.79, 0.004581602464116093, 0.054385222258041876,
        -0.01047197449952364, -0.1705406904220581, -0.011608824133872986,
        -0.08608310669660568, 0.2819371521472931, -0.13509835302829742,
        0.028368590399622917, -0.15945219993591309, -0.011438383720815182,
        0.09397093206644058, 0.2500985264778137, -0.12299267947673798,
        0.033810943365097046, 0.01984678953886032, 0.04372693970799446,
        0.04439987987279892, -0.052922338247299194, 0.3638530671596527,
        0.018935075029730797, 1.2066316604614258, 0.0026964505668729544,
        -0.0038426220417022705, -0.05543806776404381, 0.016382435336709023,
        -0.3776109516620636, -0.07517704367637634, 1.2037315368652344,
        -0.03580886498093605, -0.07851681113243103, -0.011213400401175022,
    ],
    dtype=float,
)
SAFE_IDLE_BODY_35.setflags(write=False)


class SafetyState(str, Enum):
    FOLLOW = "FOLLOW"
    HOLD = "HOLD"
    DAMP = "DAMP"
    FAULT = "FAULT"


@dataclass(frozen=True)
class SafetyInput:
    source_live: bool
    source_timestamp_s: float
    required_bones_complete: bool
    gmr_valid: bool
    reference_body_35: Sequence[float] | np.ndarray | None
    controller_alive: bool


@dataclass(frozen=True)
class SafetyDecision:
    state: SafetyState
    reference_body_35: np.ndarray | None
    reason: str
    should_terminate: bool = False


@dataclass(frozen=True)
class SafetyTransition:
    previous: SafetyState
    current: SafetyState
    timestamp_s: float
    reason: str


class SafetyGate:
    """Validate references and apply simulation-only loss-of-tracking behavior."""

    def __init__(
        self,
        *,
        follow_timeout_s: float,
        hold_to_damp_s: float,
        joint_lower: Sequence[float] | np.ndarray,
        joint_upper: Sequence[float] | np.ndarray,
    ) -> None:
        if not math.isfinite(follow_timeout_s) or follow_timeout_s <= 0:
            raise ValueError("follow_timeout_s must be finite and positive")
        if not math.isfinite(hold_to_damp_s) or hold_to_damp_s < 0:
            raise ValueError("hold_to_damp_s must be finite and non-negative")
        lower = np.asarray(joint_lower, dtype=float)
        upper = np.asarray(joint_upper, dtype=float)
        if lower.shape != (G1_DOF,) or upper.shape != (G1_DOF,):
            raise ValueError("G1 joint limits must each contain exactly 29 values")
        if not np.isfinite(lower).all() or not np.isfinite(upper).all() or np.any(lower > upper):
            raise ValueError("G1 joint limits must be finite and ordered")
        if np.any(SAFE_IDLE_BODY_35[6:] < lower) or np.any(SAFE_IDLE_BODY_35[6:] > upper):
            raise ValueError("HumDex safe-idle preset 0 violates the supplied G1 joint limits")

        self.follow_timeout_s = float(follow_timeout_s)
        self.hold_to_damp_s = float(hold_to_damp_s)
        self.joint_lower = lower.copy()
        self.joint_upper = upper.copy()
        self.state = SafetyState.HOLD
        self._last_reference: np.ndarray | None = None
        self._last_source_timestamp_s: float | None = None
        self._last_now_s: float | None = None
        self._hold_started_s: float | None = None
        self._damp_started_s: float | None = None
        self._damp_from: np.ndarray | None = None
        self._recovery_requested = False
        self._reason = "waiting for first valid reference"
        self._transitions: list[SafetyTransition] = []

    @property
    def transitions(self) -> tuple[SafetyTransition, ...]:
        return tuple(self._transitions)

    def request_recovery(self) -> None:
        """Arm one recovery from latched DAMP or FAULT after inputs are valid."""

        if self.state not in (SafetyState.DAMP, SafetyState.FAULT):
            raise RuntimeError("operator recovery is only valid from DAMP or FAULT")
        self._recovery_requested = True
        LOGGER.info("safety recovery requested from %s", self.state.value)

    def trip_controller_exception(self, error: BaseException, *, now_s: float) -> SafetyDecision:
        """Latch terminal FAULT after a caught controller exception."""

        now = self._validated_now(now_s)
        return self._fault(f"controller exception: {type(error).__name__}: {error}", now)

    def update(self, sample: SafetyInput, *, now_s: float) -> SafetyDecision:
        now = self._validated_now(now_s)

        if not sample.controller_alive:
            return self._fault("controller is not alive", now)
        if not sample.gmr_valid:
            return self._fault("GMR result is invalid", now)

        reference, reference_error = self._validate_reference(sample.reference_body_35)
        tracking_error = self._tracking_error(sample, now)
        if reference_error is not None and not (
            sample.reference_body_35 is None and tracking_error is not None
        ):
            return self._fault(reference_error, now)

        if self.state is SafetyState.FAULT and not self._recovery_requested:
            return self._decision(None, self._reason, terminate=True)

        if tracking_error is not None:
            return self._tracking_lost(tracking_error, now)
        assert reference is not None

        self._last_source_timestamp_s = float(sample.source_timestamp_s)
        if self.state in (SafetyState.DAMP, SafetyState.FAULT) and not self._recovery_requested:
            if self.state is SafetyState.FAULT:
                return self._decision(None, self._reason, terminate=True)
            return self._damp_decision(now)

        self._recovery_requested = False
        self._last_reference = reference.copy()
        self._hold_started_s = None
        self._damp_started_s = None
        self._damp_from = None
        self._transition(SafetyState.FOLLOW, "validated reference", now)
        return self._decision(reference, "validated reference")

    def _validated_now(self, now_s: float) -> float:
        now = float(now_s)
        if not math.isfinite(now):
            raise ValueError("now_s must be finite")
        if self._last_now_s is not None and now < self._last_now_s:
            # A non-monotonic local clock makes all timeout decisions untrustworthy.
            return_now = self._last_now_s
            self._fault("safety clock moved backwards", return_now)
            return return_now
        self._last_now_s = now
        return now

    def _validate_reference(
        self, value: Sequence[float] | np.ndarray | None
    ) -> tuple[np.ndarray | None, str | None]:
        if value is None:
            return None, "reference is missing"
        try:
            reference = np.asarray(value, dtype=float)
        except (TypeError, ValueError):
            return None, "reference cannot be converted to finite numbers"
        if reference.shape != (TWIST2_REFERENCE_SIZE,):
            return None, "TWIST2 reference must contain exactly 35 values"
        if not np.isfinite(reference).all():
            return None, "TWIST2 reference contains a non-finite value"
        joints = reference[6:]
        if np.any(joints < self.joint_lower) or np.any(joints > self.joint_upper):
            return None, "TWIST2 reference violates G1 joint limits"
        return reference.copy(), None

    def _tracking_error(self, sample: SafetyInput, now: float) -> str | None:
        if not sample.source_live:
            return "MotionVenus source is not LIVE"
        if not sample.required_bones_complete:
            return "required MotionVenus bones are incomplete"
        timestamp = float(sample.source_timestamp_s)
        if not math.isfinite(timestamp):
            return "source timestamp is not finite"
        age = now - timestamp
        if age < 0:
            return "source timestamp is in the future"
        if age > self.follow_timeout_s:
            return f"packet age {age:.6f}s exceeds follow timeout {self.follow_timeout_s:.6f}s"
        if self._last_source_timestamp_s is not None and timestamp <= self._last_source_timestamp_s:
            return "source timestamp is duplicate or out of order"
        return None

    def _tracking_lost(self, reason: str, now: float) -> SafetyDecision:
        if self.state is SafetyState.FAULT:
            return self._decision(None, self._reason, terminate=True)
        if self._last_reference is None:
            return self._fault(f"{reason}; no validated reference exists to HOLD", now)
        if self.state is SafetyState.FOLLOW:
            self._hold_started_s = now
            self._transition(SafetyState.HOLD, reason, now)
        if self.state is SafetyState.HOLD:
            assert self._hold_started_s is not None
            if now - self._hold_started_s < self.hold_to_damp_s:
                return self._decision(self._last_reference, reason)
            self._damp_started_s = self._hold_started_s + self.hold_to_damp_s
            self._damp_from = self._last_reference.copy()
            self._transition(SafetyState.DAMP, f"HOLD timeout: {reason}", now)
        return self._damp_decision(now)

    def _damp_decision(self, now: float) -> SafetyDecision:
        assert self._damp_started_s is not None and self._damp_from is not None
        alpha = min(1.0, max(0.0, (now - self._damp_started_s) / DAMP_RAMP_SECONDS))
        weight = 0.5 - 0.5 * math.cos(math.pi * alpha)
        output = (1.0 - weight) * self._damp_from + weight * SAFE_IDLE_BODY_35
        return self._decision(output, "simulation-only safe-idle cosine ramp")

    def _fault(self, reason: str, now: float) -> SafetyDecision:
        self._recovery_requested = False
        self._transition(SafetyState.FAULT, reason, now)
        return self._decision(None, reason, terminate=True)

    def _transition(self, state: SafetyState, reason: str, now: float) -> None:
        self._reason = reason
        if state is self.state:
            return
        transition = SafetyTransition(self.state, state, now, reason)
        self._transitions.append(transition)
        LOGGER.warning(
            "whole-body safety transition %s -> %s at %.6f: %s",
            transition.previous.value,
            transition.current.value,
            transition.timestamp_s,
            transition.reason,
        )
        self.state = state

    def _decision(
        self,
        reference: np.ndarray | None,
        reason: str,
        *,
        terminate: bool = False,
    ) -> SafetyDecision:
        return SafetyDecision(
            self.state,
            None if reference is None else reference.copy(),
            reason,
            terminate,
        )


assert SAFE_IDLE_BODY_35.shape == (TWIST2_REFERENCE_SIZE,)
assert np.isfinite(SAFE_IDLE_BODY_35).all()
