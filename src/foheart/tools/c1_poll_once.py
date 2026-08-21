from __future__ import annotations

import usb.core

from foheart.protocol.poll import C1_HID_POLL
from foheart.usb.c1_device import C1NotFoundError, C1OpenError
from foheart.usb.c1_poll import (
    C1PollReadError,
    C1PollSafetyError,
    C1PollShortWriteError,
    poll_once,
)


def main() -> int:
    print("=" * 64)
    print("EXPERIMENTAL FOHEART C1 HARDWARE POLL")
    print("Authorized packet: 0x70 + 63 zero bytes only")
    print("Maximum writes this run: 1")
    print("No retry, 0x73, RTTRANS, control transfer, or feature report")
    print("=" * 64)
    print(f"OUT payload ({len(C1_HID_POLL)} bytes): {C1_HID_POLL.hex(' ')}")
    try:
        result = poll_once()
    except C1NotFoundError as exc:
        print(f"ERROR: {exc}")
        return 1
    except (C1OpenError, C1PollSafetyError) as exc:
        if isinstance(exc, C1PollShortWriteError):
            print("OUT success: false")
            print(f"OUT bytes transferred: {exc.transferred}")
        print(f"ERROR: {exc}")
        return 2
    except C1PollReadError as exc:
        print("OUT success: true")
        print(f"OUT bytes transferred: {exc.out_transferred}")
        print(f"elapsed_ms: {exc.elapsed_ns / 1_000_000:.3f}")
        print(f"ERROR: {exc}")
        return 3
    except usb.core.USBError as exc:
        print("OUT success: false or unknown")
        print(f"ERROR: C1 interrupt OUT failed: {exc}")
        return 3

    print("USB descriptor validation: 1483:5851 config=1 interface=0 alt=0")
    print("USB endpoints: OUT=0x01 INTERRUPT/64 IN=0x81 INTERRUPT/64")
    print("OUT success: true")
    print(f"OUT bytes transferred: {result.out_transferred}")
    print(f"elapsed_ms: {result.elapsed_ns / 1_000_000:.3f}")
    if result.timed_out:
        print("IN success: false")
        print("IN timeout: true (100 ms)")
        return 4
    assert result.payload is not None
    print("IN success: true")
    print(f"IN length: {len(result.payload)}")
    print(f"IN first byte: 0x{result.payload[0]:02x}" if result.payload else "IN first byte: NONE")
    print(f"IN hex: {result.payload.hex(' ')}")
    print("IN timeout: false")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
