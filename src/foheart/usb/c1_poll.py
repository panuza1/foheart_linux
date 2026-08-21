from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Callable

import usb.core

from foheart.constants import FOHEART_VID
from foheart.protocol.frame import PollCaptureRecord
from foheart.protocol.poll import C1_HID_POLL
from foheart.usb.c1_device import C1Device

C1_HID_PID = 0x5851
C1_HID_INTERFACE = 0
C1_HID_IN_ENDPOINT = 0x81
C1_HID_OUT_ENDPOINT = 0x01
C1_HID_REPORT_SIZE = 64
C1_HID_POLL_TIMEOUT_MS = 100
C1_HID_CAPTURE_MAX_POLLS = 200
C1_HID_CAPTURE_MAX_RUNTIME_S = 30.0


class C1PollSafetyError(RuntimeError):
    pass


class C1PollShortWriteError(C1PollSafetyError):
    def __init__(self, transferred: int):
        self.transferred = transferred
        super().__init__(
            f"short C1 poll OUT transfer: {transferred} bytes, expected 64"
        )


class C1PollReadError(RuntimeError):
    def __init__(self, message: str, *, out_transferred: int, elapsed_ns: int):
        self.out_transferred = out_transferred
        self.elapsed_ns = elapsed_ns
        super().__init__(message)


@dataclass(frozen=True)
class C1PollResult:
    poll_timestamp_ns: int
    out_transferred: int
    in_timestamp_ns: int | None
    payload: bytes | None
    timed_out: bool
    elapsed_ns: int


@dataclass(frozen=True)
class C1PollCaptureResult:
    records: tuple[PollCaptureRecord, ...]
    elapsed_ns: int
    stop_reason: str
    hard_error: bool


def _require_authorized_payload(payload: bytes) -> None:
    if not isinstance(payload, bytes) or payload != C1_HID_POLL:
        raise C1PollSafetyError(
            "refusing USB OUT: only the exact 64-byte 0x70 C1 HID poll is authorized"
        )


def _validate_device(device: C1Device) -> None:
    selection = device.selection
    actual = (
        device.vid,
        device.pid,
        selection.configuration_value if selection else None,
        selection.transfer_type if selection else None,
        selection.interface_number if selection else None,
        selection.alternate_setting if selection else None,
        selection.in_endpoint if selection else None,
        selection.out_endpoint if selection else None,
        selection.in_max_packet_size if selection else None,
        selection.out_max_packet_size if selection else None,
    )
    expected = (
        FOHEART_VID,
        C1_HID_PID,
        1,
        "INTERRUPT",
        C1_HID_INTERFACE,
        0,
        C1_HID_IN_ENDPOINT,
        C1_HID_OUT_ENDPOINT,
        C1_HID_REPORT_SIZE,
        C1_HID_REPORT_SIZE,
    )
    if actual != expected:
        raise C1PollSafetyError(
            "refusing C1 poll: active USB descriptor does not match the validated "
            f"1483:5851 config/interface/endpoints/report sizes (actual={actual!r})"
        )


def _poll_open_device_once(
    device: C1Device,
    *,
    payload: bytes = C1_HID_POLL,
    timeout_ms: int = C1_HID_POLL_TIMEOUT_MS,
) -> C1PollResult:
    if timeout_ms < 1:
        raise ValueError("timeout_ms must be positive")
    _validate_device(device)
    _require_authorized_payload(payload)

    poll_timestamp_ns = time.time_ns()
    started_ns = time.monotonic_ns()
    transferred = device.write(payload, timeout_ms=timeout_ms)
    if transferred != C1_HID_REPORT_SIZE:
        raise C1PollShortWriteError(transferred)
    try:
        response = device.read(size=C1_HID_REPORT_SIZE, timeout_ms=timeout_ms)
    except usb.core.USBTimeoutError:
        return C1PollResult(
            poll_timestamp_ns,
            transferred,
            None,
            None,
            True,
            time.monotonic_ns() - started_ns,
        )
    except usb.core.USBError as exc:
        elapsed_ns = time.monotonic_ns() - started_ns
        raise C1PollReadError(
            f"C1 interrupt IN failed after the poll: {exc}",
            out_transferred=transferred,
            elapsed_ns=elapsed_ns,
        ) from exc
    return C1PollResult(
        poll_timestamp_ns,
        transferred,
        time.time_ns(),
        response,
        False,
        time.monotonic_ns() - started_ns,
    )


def poll_once(
    *,
    payload: bytes = C1_HID_POLL,
    timeout_ms: int = C1_HID_POLL_TIMEOUT_MS,
    opener: Callable[..., Any] | None = None,
) -> C1PollResult:
    """Open the real HID router, issue one authorized poll, then read at most once."""
    _require_authorized_payload(payload)
    if timeout_ms < 1:
        raise ValueError("timeout_ms must be positive")
    with _open_poll_device(opener) as device:
        return _poll_open_device_once(
            device, payload=payload, timeout_ms=timeout_ms
        )


def _open_poll_device(opener: Callable[..., Any] | None = None) -> Any:
    open_device = C1Device.open_first if opener is None else opener
    return open_device(
        vid=FOHEART_VID,
        pid=C1_HID_PID,
        usb_mode="hid",
        interface=C1_HID_INTERFACE,
        in_endpoint=C1_HID_IN_ENDPOINT,
        out_endpoint=C1_HID_OUT_ENDPOINT,
        read_size=C1_HID_REPORT_SIZE,
    )


def capture_polls(
    *,
    max_polls: int = C1_HID_CAPTURE_MAX_POLLS,
    timeout_ms: int = C1_HID_POLL_TIMEOUT_MS,
    max_runtime_s: float = C1_HID_CAPTURE_MAX_RUNTIME_S,
    opener: Callable[..., Any] | None = None,
) -> C1PollCaptureResult:
    """Run a finite OUT-then-IN loop using only the canonical C1 HID poll."""
    if not 1 <= max_polls <= C1_HID_CAPTURE_MAX_POLLS:
        raise ValueError(f"max_polls must be between 1 and {C1_HID_CAPTURE_MAX_POLLS}")
    if not 1 <= timeout_ms <= C1_HID_POLL_TIMEOUT_MS:
        raise ValueError(f"timeout_ms must be between 1 and {C1_HID_POLL_TIMEOUT_MS}")
    if not 0 < max_runtime_s <= C1_HID_CAPTURE_MAX_RUNTIME_S:
        raise ValueError(
            f"max_runtime_s must be positive and at most {C1_HID_CAPTURE_MAX_RUNTIME_S:g}"
        )

    started_ns = time.monotonic_ns()
    deadline_ns = started_ns + int(max_runtime_s * 1_000_000_000)
    records: list[PollCaptureRecord] = []
    stop_reason = "poll limit reached"
    hard_error = False
    with _open_poll_device(opener) as device:
        for sequence in range(1, max_polls + 1):
            # Reserve one timeout for OUT and one for IN; never begin an attempt
            # that can exceed the configured total-runtime bound.
            if time.monotonic_ns() + 2 * timeout_ms * 1_000_000 > deadline_ns:
                stop_reason = "runtime limit reached"
                break
            fallback_timestamp_ns = time.time_ns()
            fallback_started_ns = time.monotonic_ns()
            try:
                result = _poll_open_device_once(device, timeout_ms=timeout_ms)
            except C1PollShortWriteError as exc:
                records.append(
                    PollCaptureRecord(
                        sequence,
                        fallback_timestamp_ns,
                        exc.transferred,
                        None,
                        C1_HID_IN_ENDPOINT,
                        b"",
                        False,
                        str(exc),
                        time.monotonic_ns() - fallback_started_ns,
                    )
                )
                stop_reason = str(exc)
                hard_error = True
                break
            except C1PollReadError as exc:
                records.append(
                    PollCaptureRecord(
                        sequence,
                        fallback_timestamp_ns,
                        exc.out_transferred,
                        None,
                        C1_HID_IN_ENDPOINT,
                        b"",
                        False,
                        str(exc),
                        exc.elapsed_ns,
                    )
                )
                stop_reason = str(exc)
                hard_error = True
                break
            except (C1PollSafetyError, usb.core.USBError) as exc:
                records.append(
                    PollCaptureRecord(
                        sequence,
                        fallback_timestamp_ns,
                        0,
                        None,
                        C1_HID_IN_ENDPOINT,
                        b"",
                        False,
                        str(exc),
                        time.monotonic_ns() - fallback_started_ns,
                    )
                )
                stop_reason = str(exc)
                hard_error = True
                break
            records.append(
                PollCaptureRecord(
                    sequence,
                    result.poll_timestamp_ns,
                    result.out_transferred,
                    result.in_timestamp_ns,
                    C1_HID_IN_ENDPOINT,
                    result.payload or b"",
                    result.timed_out,
                    None,
                    result.elapsed_ns,
                )
            )
    return C1PollCaptureResult(
        tuple(records), time.monotonic_ns() - started_ns, stop_reason, hard_error
    )
