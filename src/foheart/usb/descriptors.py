from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import usb.util

TRANSFER_TYPES = {
    usb.util.ENDPOINT_TYPE_CTRL: "CONTROL",
    usb.util.ENDPOINT_TYPE_ISO: "ISOCHRONOUS",
    usb.util.ENDPOINT_TYPE_BULK: "BULK",
    usb.util.ENDPOINT_TYPE_INTR: "INTERRUPT",
}


@dataclass(frozen=True)
class EndpointDescriptor:
    address: int
    direction: str
    transfer_type: str
    max_packet_size: int
    interval: int | None = None


@dataclass(frozen=True)
class InterfaceDescriptor:
    number: int
    alternate_setting: int
    interface_class: int
    interface_subclass: int
    interface_protocol: int
    endpoints: tuple[EndpointDescriptor, ...]


@dataclass(frozen=True)
class ConfigurationDescriptor:
    value: int
    interfaces: tuple[InterfaceDescriptor, ...]


@dataclass(frozen=True)
class EndpointSelection:
    configuration_value: int
    transfer_type: str
    interface_number: int
    alternate_setting: int
    in_endpoint: int
    out_endpoint: int | None
    read_size: int
    in_max_packet_size: int
    out_max_packet_size: int | None


class DescriptorSelectionError(RuntimeError):
    pass


def describe_configuration(configuration: object) -> ConfigurationDescriptor:
    interfaces = []
    for interface in configuration:
        endpoints = tuple(
            EndpointDescriptor(
                address=int(endpoint.bEndpointAddress),
                direction=(
                    "IN"
                    if usb.util.endpoint_direction(endpoint.bEndpointAddress)
                    == usb.util.ENDPOINT_IN
                    else "OUT"
                ),
                transfer_type=TRANSFER_TYPES.get(
                    usb.util.endpoint_type(endpoint.bmAttributes), "UNKNOWN"
                ),
                max_packet_size=int(endpoint.wMaxPacketSize),
                interval=getattr(endpoint, "bInterval", None),
            )
            for endpoint in interface
        )
        interfaces.append(
            InterfaceDescriptor(
                number=int(interface.bInterfaceNumber),
                alternate_setting=int(interface.bAlternateSetting),
                interface_class=int(interface.bInterfaceClass),
                interface_subclass=int(interface.bInterfaceSubClass),
                interface_protocol=int(interface.bInterfaceProtocol),
                endpoints=endpoints,
            )
        )
    return ConfigurationDescriptor(int(configuration.bConfigurationValue), tuple(interfaces))


def select_endpoints(
    configurations: Iterable[ConfigurationDescriptor],
    *,
    transfer_type: str | None = None,
    interface_number: int | None = None,
    in_endpoint: int | None = None,
    out_endpoint: int | None = None,
    read_size: int | None = None,
) -> EndpointSelection:
    if transfer_type not in (None, "BULK", "INTERRUPT"):
        raise ValueError("transfer_type must be BULK, INTERRUPT, or None")
    if read_size is not None and read_size < 1:
        raise ValueError("read_size must be positive")

    candidates: list[EndpointSelection] = []
    diagnostics: list[str] = []
    available: list[str] = []
    for configuration in configurations:
        for interface in configuration.interfaces:
            available.append(
                f"interface={interface.number} alt={interface.alternate_setting} "
                + ",".join(
                    f"0x{endpoint.address:02x}/{endpoint.direction}/{endpoint.transfer_type}"
                    for endpoint in interface.endpoints
                )
            )
            if interface_number is not None and interface.number != interface_number:
                continue
            types = (transfer_type,) if transfer_type else ("BULK", "INTERRUPT")
            for candidate_type in types:
                endpoints = [
                    endpoint
                    for endpoint in interface.endpoints
                    if endpoint.transfer_type == candidate_type
                ]
                ins = [endpoint for endpoint in endpoints if endpoint.direction == "IN"]
                outs = [endpoint for endpoint in endpoints if endpoint.direction == "OUT"]
                if in_endpoint is not None:
                    ins = [endpoint for endpoint in ins if endpoint.address == in_endpoint]
                if out_endpoint is not None:
                    outs = [endpoint for endpoint in outs if endpoint.address == out_endpoint]
                if len(ins) > 1 or len(outs) > 1:
                    diagnostics.append(
                        f"config {configuration.value}, interface {interface.number}, "
                        f"alt {interface.alternate_setting}: {len(ins)} {candidate_type} IN and "
                        f"{len(outs)} {candidate_type} OUT endpoints after overrides"
                    )
                    continue
                if len(ins) == 1 and (
                    (candidate_type == "INTERRUPT" and len(outs) <= 1)
                    or (candidate_type == "BULK" and len(outs) == 1)
                ):
                    candidates.append(
                        EndpointSelection(
                            configuration_value=configuration.value,
                            transfer_type=candidate_type,
                            interface_number=interface.number,
                            alternate_setting=interface.alternate_setting,
                            in_endpoint=ins[0].address,
                            out_endpoint=outs[0].address if outs else None,
                            read_size=read_size or ins[0].max_packet_size,
                            in_max_packet_size=ins[0].max_packet_size,
                            out_max_packet_size=(
                                outs[0].max_packet_size if outs else None
                            ),
                        )
                    )
    if len(candidates) != 1:
        found = ", ".join(
            f"{item.transfer_type} interface={item.interface_number} alt={item.alternate_setting} "
            f"IN=0x{item.in_endpoint:02x} OUT="
            f"{f'0x{item.out_endpoint:02x}' if item.out_endpoint is not None else 'none'}"
            for item in candidates
        )
        detail = "; ".join(diagnostics)
        requested = (
            f"type={transfer_type or 'auto'} interface={interface_number if interface_number is not None else 'auto'} "
            f"IN={f'0x{in_endpoint:02x}' if in_endpoint is not None else 'auto'} "
            f"OUT={f'0x{out_endpoint:02x}' if out_endpoint is not None else 'auto'}"
        )
        raise DescriptorSelectionError(
            "Expected exactly one usable Bulk IN/OUT or Interrupt IN endpoint set; "
            f"found {len(candidates)}. Candidates: {found or 'none'}. "
            f"Requested: {requested}. Available: {'; '.join(available) or 'none'}. "
            f"Descriptor diagnostics: {detail or 'none'}"
        )
    return candidates[0]
