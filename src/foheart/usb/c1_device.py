from __future__ import annotations

import errno
from typing import Any

import usb.core
import usb.util

from foheart.constants import FOHEART_VID, ROUTER_PIDS
from foheart.protocol.definitions import UnsupportedProtocolError
from foheart.usb.descriptors import (
    DescriptorSelectionError,
    EndpointSelection,
    describe_configuration,
    select_endpoints,
)
from foheart.usb.transport import BulkTransport, C1Transport, InterruptTransport


class C1NotFoundError(RuntimeError):
    pass


class C1OpenError(RuntimeError):
    pass


class C1Device:
    def __init__(
        self,
        usb_device: Any,
        *,
        usb_mode: str = "auto",
        interface: int | None = None,
        in_endpoint: int | None = None,
        out_endpoint: int | None = None,
        read_size: int | None = None,
    ):
        self.usb_device = usb_device
        self.usb_mode = usb_mode
        self.interface = interface
        self.in_endpoint = in_endpoint
        self.out_endpoint = out_endpoint
        self.read_size = read_size
        self.selection: EndpointSelection | None = None
        self.transport: C1Transport | None = None
        self._claimed_interface: int | None = None
        self._detached_interface: int | None = None

    @property
    def vid(self) -> int:
        return int(self.usb_device.idVendor)

    @property
    def pid(self) -> int:
        return int(self.usb_device.idProduct)

    @classmethod
    def open_first(
        cls,
        vid: int = FOHEART_VID,
        pid: int | None = None,
        *,
        usb_mode: str = "auto",
        interface: int | None = None,
        in_endpoint: int | None = None,
        out_endpoint: int | None = None,
        read_size: int | None = None,
    ) -> "C1Device":
        if usb_mode not in ("auto", "bulk", "hid"):
            raise C1OpenError(f"Unsupported USB mode: {usb_mode}")
        if vid == FOHEART_VID and pid is not None and pid not in ROUTER_PIDS:
            raise C1OpenError(
                f"PID 0x{pid:04x} is not a supported C1 router; ChargePlate and wired "
                "sensor access are out of scope"
            )
        try:
            devices = list(usb.core.find(find_all=True, idVendor=vid) or [])
        except usb.core.USBError as exc:
            raise C1OpenError(f"USB enumeration failed: {exc}") from exc
        routers = [
            device
            for device in devices
            if int(device.idProduct) in ROUTER_PIDS
        ]
        matches = (
            [device for device in routers if int(device.idProduct) == pid]
            if pid is not None
            else routers
        )
        preferred_pid = {"bulk": 0x5751, "hid": 0x5851}.get(usb_mode)
        if pid is None and preferred_pid is not None:
            preferred = [
                device for device in matches if int(device.idProduct) == preferred_pid
            ]
            if preferred:
                matches = preferred
        if not matches:
            expected = f"{vid:04x}:{pid:04x}" if pid is not None else "1483:5751 or 1483:5851"
            raise C1NotFoundError(f"No FOHEART C1 device found (expected {expected})")
        if len(matches) != 1:
            found = ", ".join(
                f"{int(device.idVendor):04x}:{int(device.idProduct):04x}"
                for device in matches
            )
            raise C1OpenError(
                f"Multiple matching C1 routers found ({found}); select one with --pid"
            )
        instance = cls(
            matches[0],
            usb_mode=usb_mode,
            interface=interface,
            in_endpoint=in_endpoint,
            out_endpoint=out_endpoint,
            read_size=read_size,
        )
        instance.open()
        return instance

    def open(self) -> None:
        if self.transport is not None:
            return
        try:
            active = self.usb_device.get_active_configuration()
            self.selection = select_endpoints(
                (describe_configuration(active),),
                transfer_type={"bulk": "BULK", "hid": "INTERRUPT"}.get(
                    self.usb_mode
                ),
                interface_number=self.interface,
                in_endpoint=self.in_endpoint,
                out_endpoint=self.out_endpoint,
                read_size=self.read_size,
            )
            try:
                kernel_active = self.usb_device.is_kernel_driver_active(
                    self.selection.interface_number
                )
            except NotImplementedError:
                kernel_active = False
            if kernel_active:
                self.usb_device.detach_kernel_driver(self.selection.interface_number)
                self._detached_interface = self.selection.interface_number
            usb.util.claim_interface(self.usb_device, self.selection.interface_number)
            self._claimed_interface = self.selection.interface_number
            if self.selection.alternate_setting:
                self.usb_device.set_interface_altsetting(
                    interface=self.selection.interface_number,
                    alternate_setting=self.selection.alternate_setting,
                )
            transport_class = (
                BulkTransport
                if self.selection.transfer_type == "BULK"
                else InterruptTransport
            )
            self.transport = transport_class(self.usb_device, self.selection)
            self.transport.open()
        except DescriptorSelectionError as exc:
            self.close()
            raise C1OpenError(f"USB descriptor selection failed: {exc}") from exc
        except usb.core.USBError as exc:
            self.close()
            hint = (
                " Check udev permissions."
                if getattr(exc, "errno", None) in (errno.EACCES, errno.EPERM, 13)
                else " The interface may be busy or unavailable."
            )
            raise C1OpenError(f"Could not open C1: {exc}.{hint}") from exc

    def read(self, size: int | None = None, timeout_ms: int | None = None) -> bytes:
        if self.transport is None:
            raise C1OpenError("C1 device is not open")
        return self.transport.read(size=size, timeout_ms=timeout_ms)

    def write(self, payload: bytes, timeout_ms: int | None = None) -> int:
        if self.transport is None:
            raise C1OpenError("C1 device is not open")
        return self.transport.write(payload, timeout_ms=timeout_ms)

    def start_stream(self) -> None:
        raise UnsupportedProtocolError("C1 stream-start command not yet recovered.")

    def close(self) -> None:
        if self.transport is not None:
            self.transport.close()
            self.transport = None
        if self._claimed_interface is not None:
            try:
                usb.util.release_interface(self.usb_device, self._claimed_interface)
            except usb.core.USBError:
                pass
            self._claimed_interface = None
        if self._detached_interface is not None:
            try:
                self.usb_device.attach_kernel_driver(self._detached_interface)
            except (usb.core.USBError, NotImplementedError):
                pass
            self._detached_interface = None
        usb.util.dispose_resources(self.usb_device)

    def __enter__(self) -> "C1Device":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
