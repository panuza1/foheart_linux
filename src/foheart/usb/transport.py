from __future__ import annotations

from abc import ABC
from typing import Any

from foheart.usb.descriptors import EndpointSelection


class TransportError(RuntimeError):
    pass


class C1Transport(ABC):
    mode = "UNKNOWN"

    def __init__(self, usb_device: Any, endpoints: EndpointSelection):
        self.usb_device = usb_device
        self.endpoints = endpoints
        self.is_open = False

    def open(self) -> None:
        self.is_open = True

    def close(self) -> None:
        self.is_open = False

    def read(self, size: int | None = None, timeout_ms: int | None = None) -> bytes:
        if not self.is_open:
            raise TransportError("Transport is not open")
        return bytes(
            self.usb_device.read(
                self.endpoints.in_endpoint,
                size or self.endpoints.read_size,
                timeout=timeout_ms,
            )
        )

    def write(self, payload: bytes, timeout_ms: int | None = None) -> int:
        if not self.is_open:
            raise TransportError("Transport is not open")
        if self.endpoints.out_endpoint is None:
            raise TransportError(f"{self.mode} interface has no OUT endpoint")
        return int(
            self.usb_device.write(
                self.endpoints.out_endpoint, payload, timeout=timeout_ms
            )
        )


class BulkTransport(C1Transport):
    mode = "BULK"


class InterruptTransport(C1Transport):
    mode = "INTERRUPT"

