"""The one C1 HID host payload authorized for experimental use."""

# REAL_CAPTURE_VALIDATED on 1483:5851; this remains the only permitted payload.
C1_HID_POLL = bytes((0x70,)) + bytes(63)


def build_c1_hid_poll() -> bytes:
    """Return the immutable, fixed 64-byte C1 HID poll."""
    return C1_HID_POLL
