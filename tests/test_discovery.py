import usb.core

from foheart.constants import FOHEART_VID, is_foheart_product
from foheart.usb import discovery
from foheart.usb.descriptors import (
    ConfigurationDescriptor,
    DescriptorSelectionError,
    EndpointDescriptor,
    InterfaceDescriptor,
    describe_configuration,
    select_endpoints,
)


def endpoint(address: int, direction: str, transfer_type: str) -> EndpointDescriptor:
    return EndpointDescriptor(address, direction, transfer_type, 64)


def config(*endpoints: EndpointDescriptor) -> ConfigurationDescriptor:
    return ConfigurationDescriptor(1, (InterfaceDescriptor(0, 0, 0, 0, 0, endpoints),))


def test_known_vid_pids():
    assert is_foheart_product(FOHEART_VID, 0x5751)
    assert is_foheart_product(FOHEART_VID, 0x5851)
    assert not is_foheart_product(FOHEART_VID, 0x9999)


def test_raw_descriptor_classification():
    class Descriptor:
        bEndpointAddress = 0x81
        bmAttributes = 0x03
        wMaxPacketSize = 64
        bInterval = 1

    class Interface:
        bInterfaceNumber = 2
        bAlternateSetting = 0
        bInterfaceClass = 3
        bInterfaceSubClass = 0
        bInterfaceProtocol = 0

        def __iter__(self):
            return iter((Descriptor(),))

    class Configuration:
        bConfigurationValue = 1

        def __iter__(self):
            return iter((Interface(),))

    described = describe_configuration(Configuration())
    actual = described.interfaces[0].endpoints[0]
    assert (actual.direction, actual.transfer_type, actual.address) == (
        "IN",
        "INTERRUPT",
        0x81,
    )


def test_bulk_endpoint_selection():
    selected = select_endpoints(
        (config(endpoint(0x81, "IN", "BULK"), endpoint(0x01, "OUT", "BULK")),)
    )
    assert (selected.transfer_type, selected.in_endpoint, selected.out_endpoint) == (
        "BULK",
        0x81,
        0x01,
    )


def test_interrupt_endpoint_selection_and_classification():
    selected = select_endpoints((config(endpoint(0x83, "IN", "INTERRUPT")),))
    assert selected.transfer_type == "INTERRUPT"
    assert selected.out_endpoint is None


def test_descriptor_overrides_are_validated_and_applied():
    configurations = (
        config(endpoint(0x81, "IN", "BULK"), endpoint(0x01, "OUT", "BULK")),
    )
    selected = select_endpoints(
        configurations,
        transfer_type="BULK",
        interface_number=0,
        in_endpoint=0x81,
        out_endpoint=0x01,
        read_size=0x1400,
    )
    assert selected.read_size == 0x1400


def test_descriptor_override_mismatch_is_rejected():
    configurations = (
        config(endpoint(0x81, "IN", "BULK"), endpoint(0x01, "OUT", "BULK")),
    )
    try:
        select_endpoints(configurations, in_endpoint=0x82)
    except DescriptorSelectionError as exc:
        assert "IN=0x82" in str(exc)
    else:
        raise AssertionError("invalid endpoint override was accepted")


def test_ambiguous_endpoints_are_rejected():
    configs = (
        config(endpoint(0x81, "IN", "BULK"), endpoint(0x01, "OUT", "BULK")),
        config(endpoint(0x82, "IN", "INTERRUPT")),
    )
    try:
        select_endpoints(configs)
    except DescriptorSelectionError as exc:
        assert "found 2" in str(exc)
    else:
        raise AssertionError("ambiguous descriptors were accepted")


def test_no_device_behavior(monkeypatch):
    monkeypatch.setattr(discovery.usb.core, "find", lambda **_: [])
    assert discovery.discover_foheart_devices().devices == []


def test_permission_error_behavior(monkeypatch):
    denied = usb.core.USBError("denied", errno=13)

    def fail(**_):
        raise denied

    monkeypatch.setattr(discovery.usb.core, "find", fail)
    result = discovery.discover_foheart_devices()
    assert result.devices == []
    assert "permission denied" in result.errors[0].lower()
