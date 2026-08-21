import usb.core
import usb.util
import pytest

from foheart.protocol.poll import C1_HID_POLL, build_c1_hid_poll
from foheart.usb.c1_device import C1Device
from foheart.usb.c1_poll import (
    C1PollReadError,
    C1PollSafetyError,
    C1PollShortWriteError,
    capture_polls,
    poll_once,
)


class FakeEndpoint:
    bmAttributes = 0x03
    bInterval = 1

    def __init__(self, address, max_packet_size=64):
        self.bEndpointAddress = address
        self.wMaxPacketSize = max_packet_size


class FakeInterface:
    bInterfaceNumber = 0
    bAlternateSetting = 0
    bInterfaceClass = 3
    bInterfaceSubClass = 0
    bInterfaceProtocol = 0

    def __init__(self, in_size=64, out_size=64):
        self.endpoints = (
            FakeEndpoint(0x81, in_size),
            FakeEndpoint(0x01, out_size),
        )

    def __iter__(self):
        return iter(self.endpoints)


class FakeConfiguration:
    def __init__(self, value=1, in_size=64, out_size=64):
        self.bConfigurationValue = value
        self.interface = FakeInterface(in_size, out_size)

    def __iter__(self):
        return iter((self.interface,))


class FakeUSBDevice:
    idVendor = 0x1483
    idProduct = 0x5851

    def __init__(
        self,
        *,
        response=bytes(range(64)),
        read_error=None,
        write_length=64,
        configuration=1,
        in_size=64,
        out_size=64,
    ):
        self.configuration = FakeConfiguration(configuration, in_size, out_size)
        self.response = response
        self.read_error = read_error
        self.write_length = write_length
        self.kernel_active = True
        self.events = []

    def get_active_configuration(self):
        return self.configuration

    def is_kernel_driver_active(self, interface):
        self.events.append(("kernel_active", interface))
        return self.kernel_active

    def detach_kernel_driver(self, interface):
        self.events.append(("detach", interface))
        self.kernel_active = False

    def attach_kernel_driver(self, interface):
        self.events.append(("attach", interface))
        self.kernel_active = True

    def write(self, endpoint, payload, timeout=None):
        self.events.append(("write", endpoint, bytes(payload), timeout))
        return self.write_length

    def read(self, endpoint, size, timeout=None):
        self.events.append(("read", endpoint, size, timeout))
        if self.read_error is not None:
            raise self.read_error
        return self.response


@pytest.fixture(autouse=True)
def fake_usb_lifecycle(monkeypatch):
    monkeypatch.setattr(
        usb.util,
        "claim_interface",
        lambda device, interface: device.events.append(("claim", interface)),
    )
    monkeypatch.setattr(
        usb.util,
        "release_interface",
        lambda device, interface: device.events.append(("release", interface)),
    )
    monkeypatch.setattr(
        usb.util,
        "dispose_resources",
        lambda device: device.events.append(("dispose",)),
    )


def opener_for(raw_device):
    def opener(**kwargs):
        assert kwargs == {
            "vid": 0x1483,
            "pid": 0x5851,
            "usb_mode": "hid",
            "interface": 0,
            "in_endpoint": 0x81,
            "out_endpoint": 0x01,
            "read_size": 64,
        }
        device = C1Device(
            raw_device,
            usb_mode="hid",
            interface=0,
            in_endpoint=0x81,
            out_endpoint=0x01,
            read_size=64,
        )
        device.open()
        return device

    return opener


def io_events(raw_device):
    return [event for event in raw_device.events if event[0] in ("write", "read")]


def test_poll_packet_and_one_shot_order_are_exact():
    assert isinstance(C1_HID_POLL, bytes)
    assert build_c1_hid_poll() is C1_HID_POLL
    assert len(C1_HID_POLL) == 64
    assert C1_HID_POLL[0] == 0x70
    assert C1_HID_POLL[1:] == bytes(63)

    raw_device = FakeUSBDevice()
    result = poll_once(opener=opener_for(raw_device))
    assert io_events(raw_device) == [
        ("write", 0x01, C1_HID_POLL, 100),
        ("read", 0x81, 64, 100),
    ]
    assert result.out_transferred == 64
    assert result.payload == bytes(range(64))
    assert result.timed_out is False


@pytest.mark.parametrize(
    "forbidden",
    [
        bytes((0x73,)) + bytes(63),
        bytes((0x21,)) + bytes(63),
        bytes(64),
        C1_HID_POLL + b"\x00",
        bytearray(C1_HID_POLL),
    ],
)
def test_every_noncanonical_payload_is_rejected_before_open(forbidden):
    def must_not_open(**_):
        raise AssertionError("device was opened for a forbidden payload")

    with pytest.raises(C1PollSafetyError):
        poll_once(payload=forbidden, opener=must_not_open)


def test_short_out_stops_before_in_and_closes():
    raw_device = FakeUSBDevice(write_length=63)
    with pytest.raises(C1PollShortWriteError) as caught:
        poll_once(opener=opener_for(raw_device))
    assert caught.value.transferred == 63
    assert [event[0] for event in io_events(raw_device)] == ["write"]
    assert ("release", 0) in raw_device.events
    assert ("attach", 0) in raw_device.events
    assert raw_device.kernel_active is True


def test_timeout_reads_once_and_restores_interface():
    raw_device = FakeUSBDevice(read_error=usb.core.USBTimeoutError("timed out"))
    result = poll_once(opener=opener_for(raw_device))
    assert result.timed_out is True
    assert result.payload is None
    assert [event[0] for event in io_events(raw_device)] == ["write", "read"]
    assert raw_device.events.count(("release", 0)) == 1
    assert raw_device.events.count(("attach", 0)) == 1
    assert raw_device.kernel_active is True
    assert raw_device.events[-1] == ("dispose",)


def test_usb_read_exception_closes_and_restores_interface():
    raw_device = FakeUSBDevice(read_error=usb.core.USBError("read failed"))
    with pytest.raises(C1PollReadError) as caught:
        poll_once(opener=opener_for(raw_device))
    assert caught.value.out_transferred == 64
    assert [event[0] for event in io_events(raw_device)] == ["write", "read"]
    assert ("release", 0) in raw_device.events
    assert ("attach", 0) in raw_device.events
    assert raw_device.kernel_active is True


@pytest.mark.parametrize(
    "device",
    [
        FakeUSBDevice(configuration=2),
        FakeUSBDevice(in_size=32),
        FakeUSBDevice(out_size=32),
    ],
)
def test_descriptor_mismatch_is_rejected_without_out(device):
    with pytest.raises(C1PollSafetyError):
        poll_once(opener=opener_for(device))
    assert io_events(device) == []
    assert ("release", 0) in device.events
    assert ("attach", 0) in device.events


def test_nonpositive_timeout_is_rejected_before_open():
    def must_not_open(**_):
        raise AssertionError("device was opened with an unbounded timeout")

    with pytest.raises(ValueError):
        poll_once(timeout_ms=0, opener=must_not_open)


def test_bounded_capture_preserves_one_out_then_one_in_per_attempt():
    raw_device = FakeUSBDevice(response=bytes((0x15,)) + bytes(63))
    capture = capture_polls(
        max_polls=3, max_runtime_s=1, opener=opener_for(raw_device)
    )
    assert [event[0] for event in io_events(raw_device)] == [
        "write",
        "read",
        "write",
        "read",
        "write",
        "read",
    ]
    assert [record.sequence for record in capture.records] == [1, 2, 3]
    assert all(record.out_transferred == 64 for record in capture.records)
    assert all(record.payload == bytes((0x15,)) + bytes(63) for record in capture.records)
    assert capture.hard_error is False
    assert capture.stop_reason == "poll limit reached"


def test_bounded_capture_continues_normal_timeouts_but_never_retries_an_out():
    raw_device = FakeUSBDevice(read_error=usb.core.USBTimeoutError("timed out"))
    capture = capture_polls(
        max_polls=2, max_runtime_s=1, opener=opener_for(raw_device)
    )
    assert [event[0] for event in io_events(raw_device)] == [
        "write",
        "read",
        "write",
        "read",
    ]
    assert len(capture.records) == 2
    assert all(record.timed_out for record in capture.records)
    assert all(record.error is None for record in capture.records)


def test_bounded_capture_hard_stops_after_short_out():
    raw_device = FakeUSBDevice(write_length=63)
    capture = capture_polls(
        max_polls=3, max_runtime_s=1, opener=opener_for(raw_device)
    )
    assert [event[0] for event in io_events(raw_device)] == ["write"]
    assert len(capture.records) == 1
    assert capture.records[0].out_transferred == 63
    assert capture.records[0].error is not None
    assert capture.hard_error is True


@pytest.mark.parametrize(
    "limits",
    [
        {"max_polls": 0},
        {"max_polls": 201},
        {"timeout_ms": 0},
        {"timeout_ms": 101},
        {"max_runtime_s": 0},
        {"max_runtime_s": 31},
    ],
)
def test_bounded_capture_limits_cannot_be_disabled(limits):
    with pytest.raises(ValueError):
        capture_polls(**limits)
