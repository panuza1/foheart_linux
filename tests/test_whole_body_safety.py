import logging

import numpy as np
import pytest

from foheart.whole_body.safety import (
    DAMP_RAMP_SECONDS,
    SAFE_IDLE_BODY_35,
    SafetyGate,
    SafetyInput,
    SafetyState,
)


DEFAULT_REFERENCE = object()


def gate(*, follow_timeout_s=0.25, hold_to_damp_s=1.0):
    return SafetyGate(
        follow_timeout_s=follow_timeout_s,
        hold_to_damp_s=hold_to_damp_s,
        joint_lower=np.full(29, -2.0),
        joint_upper=np.full(29, 2.0),
    )


def sample(
    timestamp,
    reference=DEFAULT_REFERENCE,
    *,
    source_live=True,
    bones=True,
    gmr=True,
    controller=True,
):
    return SafetyInput(
        source_live=source_live,
        source_timestamp_s=timestamp,
        required_bones_complete=bones,
        gmr_valid=gmr,
        reference_body_35=np.zeros(35) if reference is DEFAULT_REFERENCE else reference,
        controller_alive=controller,
    )


def start_following(safety_gate):
    decision = safety_gate.update(sample(0.0), now_s=0.0)
    assert decision.state is SafetyState.FOLLOW
    return decision


def test_follow_hold_follow_and_transition_logging(caplog):
    safety_gate = gate()
    with caplog.at_level(logging.WARNING):
        start_following(safety_gate)
        held = safety_gate.update(
            sample(0.0, reference=None, source_live=False),
            now_s=0.1,
        )
        recovered = safety_gate.update(sample(0.2), now_s=0.2)

    assert held.state is SafetyState.HOLD
    assert np.array_equal(held.reference_body_35, np.zeros(35))
    assert recovered.state is SafetyState.FOLLOW
    assert [item.current for item in safety_gate.transitions] == [
        SafetyState.FOLLOW,
        SafetyState.HOLD,
        SafetyState.FOLLOW,
    ]
    assert len([record for record in caplog.records if "safety transition" in record.message]) == 3


def test_hold_to_damp_uses_exact_three_second_cosine_ramp_and_never_holds_forever():
    safety_gate = gate(hold_to_damp_s=1.0)
    start_following(safety_gate)
    safety_gate.update(sample(0.0, source_live=False), now_s=0.1)

    ramp_start = safety_gate.update(sample(0.0, source_live=False), now_s=1.1)
    ramp_middle = safety_gate.update(
        sample(0.0, source_live=False),
        now_s=1.1 + DAMP_RAMP_SECONDS / 2,
    )
    ramp_end = safety_gate.update(
        sample(0.0, source_live=False),
        now_s=1.1 + DAMP_RAMP_SECONDS,
    )
    much_later = safety_gate.update(sample(0.0, source_live=False), now_s=100.0)

    assert ramp_start.state is SafetyState.DAMP
    assert np.allclose(ramp_start.reference_body_35, 0.0)
    assert np.allclose(ramp_middle.reference_body_35, SAFE_IDLE_BODY_35 * 0.5)
    assert np.allclose(ramp_end.reference_body_35, SAFE_IDLE_BODY_35)
    assert np.allclose(much_later.reference_body_35, SAFE_IDLE_BODY_35)
    assert "simulation-only" in ramp_end.reason
    assert not ramp_end.should_terminate


def test_damp_recovery_requires_an_explicit_operator_request():
    safety_gate = gate(hold_to_damp_s=0.0)
    start_following(safety_gate)
    safety_gate.update(sample(0.0, source_live=False), now_s=0.1)

    still_damped = safety_gate.update(sample(0.2), now_s=0.2)
    assert still_damped.state is SafetyState.DAMP

    safety_gate.request_recovery()
    recovered = safety_gate.update(sample(0.3), now_s=0.3)
    assert recovered.state is SafetyState.FOLLOW


@pytest.mark.parametrize(
    ("changed", "expected"),
    [
        ({"source_live": False}, "not LIVE"),
        ({"bones": False}, "bones are incomplete"),
    ],
)
def test_tracking_loss_enters_hold(changed, expected):
    safety_gate = gate()
    start_following(safety_gate)
    decision = safety_gate.update(sample(0.1, **changed), now_s=0.1)
    assert decision.state is SafetyState.HOLD
    assert expected in decision.reason


@pytest.mark.parametrize(
    ("timestamp", "now", "expected"),
    [
        (0.0, 0.3, "packet age"),
        (0.0, 0.1, "duplicate"),
        (-0.1, 0.1, "out of order"),
        (0.2, 0.1, "in the future"),
    ],
)
def test_packet_age_and_timestamp_failures_enter_hold(timestamp, now, expected):
    safety_gate = gate()
    start_following(safety_gate)
    decision = safety_gate.update(sample(timestamp), now_s=now)
    assert decision.state is SafetyState.HOLD
    assert expected in decision.reason


@pytest.mark.parametrize("failure", ["shape", "nan", "limits", "gmr", "controller"])
def test_fatal_input_failure_returns_no_publishable_reference(failure):
    safety_gate = gate()
    start_following(safety_gate)
    reference = np.zeros(35)
    kwargs = {}
    if failure == "shape":
        reference = np.zeros(34)
    elif failure == "nan":
        reference[0] = np.nan
    elif failure == "limits":
        reference[6] = 3.0
    elif failure == "gmr":
        kwargs["gmr"] = False
    else:
        kwargs["controller"] = False

    decision = safety_gate.update(sample(0.1, reference, **kwargs), now_s=0.1)
    assert decision.state is SafetyState.FAULT
    assert decision.reference_body_35 is None
    assert decision.should_terminate


def test_fault_is_latched_until_explicit_recovery():
    safety_gate = gate()
    start_following(safety_gate)
    invalid = np.zeros(35)
    invalid[0] = np.nan
    safety_gate.update(sample(0.1, invalid), now_s=0.1)

    still_faulted = safety_gate.update(sample(0.2), now_s=0.2)
    assert still_faulted.state is SafetyState.FAULT
    assert still_faulted.reference_body_35 is None

    safety_gate.request_recovery()
    recovered = safety_gate.update(sample(0.3), now_s=0.3)
    assert recovered.state is SafetyState.FOLLOW
    assert not recovered.should_terminate


def test_controller_exception_is_a_terminal_fault():
    safety_gate = gate()
    start_following(safety_gate)
    decision = safety_gate.trip_controller_exception(RuntimeError("policy failed"), now_s=0.1)
    assert decision.state is SafetyState.FAULT
    assert decision.reference_body_35 is None
    assert decision.should_terminate
    assert "policy failed" in decision.reason


def test_incomplete_initial_frame_fails_closed_instead_of_inventing_a_hold_pose():
    safety_gate = gate()
    decision = safety_gate.update(
        sample(0.0, reference=None, bones=False),
        now_s=0.0,
    )
    assert decision.state is SafetyState.FAULT
    assert decision.reference_body_35 is None
    assert decision.should_terminate
    assert "no validated reference" in decision.reason


def test_constructor_rejects_missing_or_invalid_limits_and_timeouts():
    with pytest.raises(TypeError):
        SafetyGate()  # type: ignore[call-arg]
    with pytest.raises(ValueError, match="follow_timeout"):
        gate(follow_timeout_s=0.0)
    with pytest.raises(ValueError, match="29"):
        SafetyGate(
            follow_timeout_s=0.1,
            hold_to_damp_s=1.0,
            joint_lower=np.full(28, -2.0),
            joint_upper=np.full(29, 2.0),
        )
    with pytest.raises(ValueError, match="safe-idle"):
        SafetyGate(
            follow_timeout_s=0.1,
            hold_to_damp_s=1.0,
            joint_lower=np.full(29, -0.1),
            joint_upper=np.full(29, 0.1),
        )


def test_decision_reference_is_not_an_alias_of_gate_state():
    safety_gate = gate()
    decision = start_following(safety_gate)
    decision.reference_body_35[:] = 1.0
    held = safety_gate.update(sample(0.0, source_live=False), now_s=0.1)
    assert np.array_equal(held.reference_body_35, np.zeros(35))
