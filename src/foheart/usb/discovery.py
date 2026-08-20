from __future__ import annotations

import errno
from dataclasses import dataclass, field
from typing import Any

import usb.core
import usb.util

from foheart.constants import FOHEART_VID, PRODUCTS
from foheart.usb.descriptors import ConfigurationDescriptor, describe_configuration


@dataclass
class DiscoveredDevice:
    vid: int
    pid: int
    bus: int | None
    address: int | None
    port_path: tuple[int, ...] | None
    manufacturer: str | None
    product: str | None
    serial: str | None
    configurations: tuple[ConfigurationDescriptor, ...]
    access_errors: tuple[str, ...] = ()
    usb_device: Any = field(default=None, repr=False, compare=False)

    @property
    def known_product_name(self) -> str:
        return PRODUCTS[self.pid]


@dataclass
class DiscoveryResult:
    devices: list[DiscoveredDevice]
    errors: list[str]


def _permission_message(exc: BaseException) -> str:
    if getattr(exc, "errno", None) in (errno.EACCES, errno.EPERM, 13):
        return f"USB permission denied: {exc}"
    return f"USB access error: {exc}"


def _read_string(device: Any, index: int, errors: list[str]) -> str | None:
    if not index:
        return None
    try:
        return usb.util.get_string(device, index)
    except (usb.core.USBError, ValueError) as exc:
        errors.append(_permission_message(exc))
        return None


def _port_path(device: Any) -> tuple[int, ...] | None:
    try:
        ports = getattr(device, "port_numbers", None)
        if ports:
            return tuple(int(port) for port in ports)
        port = getattr(device, "port_number", None)
        return (int(port),) if port is not None else None
    except (usb.core.USBError, ValueError, TypeError):
        return None


def discover_foheart_devices() -> DiscoveryResult:
    try:
        raw_devices = list(usb.core.find(find_all=True, idVendor=FOHEART_VID) or [])
    except usb.core.USBError as exc:
        return DiscoveryResult([], [_permission_message(exc)])

    devices: list[DiscoveredDevice] = []
    errors: list[str] = []
    for device in raw_devices:
        if int(device.idProduct) not in PRODUCTS:
            continue
        device_errors: list[str] = []
        configurations: tuple[ConfigurationDescriptor, ...]
        try:
            configurations = tuple(describe_configuration(config) for config in device)
        except (usb.core.USBError, ValueError) as exc:
            device_errors.append(_permission_message(exc))
            configurations = ()
        devices.append(
            DiscoveredDevice(
                vid=int(device.idVendor),
                pid=int(device.idProduct),
                bus=getattr(device, "bus", None),
                address=getattr(device, "address", None),
                port_path=_port_path(device),
                manufacturer=_read_string(device, getattr(device, "iManufacturer", 0), device_errors),
                product=_read_string(device, getattr(device, "iProduct", 0), device_errors),
                serial=_read_string(device, getattr(device, "iSerialNumber", 0), device_errors),
                configurations=configurations,
                access_errors=tuple(dict.fromkeys(device_errors)),
                usb_device=device,
            )
        )
    return DiscoveryResult(devices, errors)

