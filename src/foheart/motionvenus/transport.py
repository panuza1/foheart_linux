"""UDP transport, diagnostics, capture, and replay for MotionVenus datagrams."""

from __future__ import annotations

from dataclasses import dataclass
import ipaddress
from pathlib import Path
import socket
import struct
import time
from typing import BinaryIO, Iterator

from .protocol import MotionVenusFrame


CAPTURE_MAGIC = b"MVUDP\x00\x01\n"
MAX_UDP_PAYLOAD = 65535
_CAPTURE_RECORD = struct.Struct("!Q4sHI")
LIVE_STATES = ("NO_PACKETS", "LIVE", "STALE", "MALFORMED", "PROTOCOL_MISMATCH")


@dataclass(frozen=True)
class MotionVenusDatagram:
    payload: bytes
    received_ns: int
    monotonic_ns: int
    sender: tuple[str, int]


@dataclass(frozen=True)
class ReceiverStats:
    packets: int
    bytes: int
    malformed_packets: int
    minimum_size: int
    maximum_size: int
    average_size: float
    rate_hz: float
    timeouts: int
    backlog_drops: int


class MotionVenusReceiver:
    """A blocking UDP receiver; construction has no socket side effects."""

    def __init__(self, bind: str = "0.0.0.0", port: int = 5001, *, timeout_s: float = 0.25):
        if not isinstance(bind, str) or not bind:
            raise ValueError("bind address cannot be empty")
        if not 1024 <= port <= 65535:
            raise ValueError("port must be in 1024..65535")
        if timeout_s <= 0:
            raise ValueError("timeout_s must be positive")
        self.bind, self.port, self.timeout_s = bind, port, timeout_s
        self.socket: socket.socket | None = None
        self._packets = self._bytes = self._malformed = self._timeouts = self._backlog_drops = 0
        self._minimum_size = 0
        self._maximum_size = 0
        self._first_monotonic_ns: int | None = None
        self._last_monotonic_ns: int | None = None

    def start(self) -> None:
        if self.socket is not None:
            return
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(self.timeout_s)
        try:
            sock.bind((self.bind, self.port))
        except Exception:
            sock.close()
            raise
        self.socket = sock

    def receive(self) -> MotionVenusDatagram | None:
        if self.socket is None:
            raise RuntimeError("receiver.start() must be called before receive()")
        try:
            payload, sender = self.socket.recvfrom(65535)
        except socket.timeout:
            self._timeouts += 1
            return None
        return self._record(payload, sender)

    def receive_latest(self) -> MotionVenusDatagram | None:
        """Block for one datagram, then discard any queued backlog except the newest."""

        latest = self.receive()
        if latest is None:
            return None
        assert self.socket is not None
        timeout = self.socket.gettimeout()
        self.socket.setblocking(False)
        try:
            while True:
                try:
                    payload, sender = self.socket.recvfrom(65535)
                except BlockingIOError:
                    return latest
                self._backlog_drops += 1
                latest = self._record(payload, sender)
        finally:
            self.socket.settimeout(timeout)

    def _record(self, payload: bytes, sender: tuple[str, int]) -> MotionVenusDatagram:
        monotonic_ns, received_ns = time.monotonic_ns(), time.time_ns()
        size = len(payload)
        self._packets += 1
        self._bytes += size
        self._minimum_size = size if self._minimum_size == 0 else min(self._minimum_size, size)
        self._maximum_size = max(self._maximum_size, size)
        self._first_monotonic_ns = self._first_monotonic_ns or monotonic_ns
        self._last_monotonic_ns = monotonic_ns
        return MotionVenusDatagram(payload, received_ns, monotonic_ns, (sender[0], sender[1]))

    def record_malformed(self) -> None:
        self._malformed += 1

    @property
    def stats(self) -> ReceiverStats:
        elapsed = (
            (self._last_monotonic_ns - self._first_monotonic_ns) / 1e9
            if self._packets > 1 and self._first_monotonic_ns is not None and self._last_monotonic_ns is not None
            else 0.0
        )
        return ReceiverStats(
            self._packets,
            self._bytes,
            self._malformed,
            self._minimum_size,
            self._maximum_size,
            self._bytes / self._packets if self._packets else 0.0,
            (self._packets - 1) / elapsed if elapsed > 0 else 0.0,
            self._timeouts,
            self._backlog_drops,
        )

    def close(self) -> None:
        if self.socket is not None:
            self.socket.close()
            self.socket = None

    def __enter__(self) -> "MotionVenusReceiver":
        self.start()
        return self

    def __exit__(self, *_args) -> None:
        self.close()


@dataclass(frozen=True)
class WatchdogObservation:
    accepted: bool
    event: str
    status: str


@dataclass(frozen=True)
class WatchdogDiagnostics:
    status: str
    accepted_frames: int
    duplicate_frames: int
    out_of_order_frames: int
    estimated_lost_frames: int
    malformed_packets: int
    protocol_mismatches: int
    sender_changes: int
    last_sender: tuple[str, int] | None
    last_frame_number: int | None
    age_ms: float | None
    last_error: str


class MotionVenusWatchdog:
    def __init__(self, *, stale_after_s: float = 0.1):
        if stale_after_s <= 0:
            raise ValueError("stale_after_s must be positive")
        self.stale_after_ns = int(stale_after_s * 1e9)
        self.accepted_frames = self.duplicate_frames = self.out_of_order_frames = 0
        self.estimated_lost_frames = self.malformed_packets = self.protocol_mismatches = 0
        self.sender_changes = 0
        self.last_sender: tuple[str, int] | None = None
        self.last_frame_number: int | None = None
        self.last_valid_monotonic_ns: int | None = None
        self.last_error = ""
        self._event_status = "NO_PACKETS"

    def observe(self, frame: MotionVenusFrame, *, monotonic_ns: int | None = None) -> WatchdogObservation:
        now = time.monotonic_ns() if monotonic_ns is None else monotonic_ns
        if self.last_sender is not None and frame.sender != self.last_sender:
            self.sender_changes += 1
            self.last_frame_number = None
        self.last_sender = frame.sender
        current = frame.header.frame_number
        event, accepted = "first", True
        if self.last_frame_number is not None:
            delta = (current - self.last_frame_number) & 0xFFFFFFFF
            if delta == 0:
                self.duplicate_frames += 1
                event, accepted = "duplicate", False
            elif delta < 0x80000000:
                if delta > 1:
                    self.estimated_lost_frames += delta - 1
                    event = "gap"
            else:
                self.out_of_order_frames += 1
                event, accepted = "out_of_order", False
        if accepted:
            self.last_frame_number = current
            self.last_valid_monotonic_ns = now
            self.accepted_frames += 1
            self.last_error = ""
            self._event_status = "LIVE"
        return WatchdogObservation(accepted, event, self.status(now))

    def mark_error(self, error: Exception, *, protocol_mismatch: bool = False) -> None:
        if protocol_mismatch:
            self.protocol_mismatches += 1
            self._event_status = "PROTOCOL_MISMATCH"
        else:
            self.malformed_packets += 1
            self._event_status = "MALFORMED"
        self.last_error = str(error)

    def status(self, monotonic_ns: int | None = None) -> str:
        now = time.monotonic_ns() if monotonic_ns is None else monotonic_ns
        if self.last_valid_monotonic_ns is not None and now - self.last_valid_monotonic_ns > self.stale_after_ns:
            return "STALE"
        return self._event_status

    def diagnostics(self, monotonic_ns: int | None = None) -> WatchdogDiagnostics:
        now = time.monotonic_ns() if monotonic_ns is None else monotonic_ns
        age = None if self.last_valid_monotonic_ns is None else (now - self.last_valid_monotonic_ns) / 1e6
        return WatchdogDiagnostics(
            self.status(now), self.accepted_frames, self.duplicate_frames,
            self.out_of_order_frames, self.estimated_lost_frames,
            self.malformed_packets, self.protocol_mismatches, self.sender_changes,
            self.last_sender, self.last_frame_number, age, self.last_error,
        )


class MotionVenusCaptureWriter:
    def __init__(self, path: str | Path):
        self.path = Path(path)
        self.file: BinaryIO | None = None
        self.records = 0

    def open(self) -> None:
        if self.file is not None:
            return
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.file = self.path.open("xb")
        self.file.write(CAPTURE_MAGIC)

    def write(self, datagram: MotionVenusDatagram) -> None:
        if self.file is None:
            raise RuntimeError("capture writer is not open")
        try:
            address = ipaddress.IPv4Address(datagram.sender[0]).packed
        except ipaddress.AddressValueError as exc:
            raise ValueError("capture format supports IPv4 senders") from exc
        if len(datagram.payload) > MAX_UDP_PAYLOAD or not 0 <= datagram.sender[1] <= 65535:
            raise ValueError("datagram metadata is out of range")
        self.file.write(_CAPTURE_RECORD.pack(datagram.received_ns, address, datagram.sender[1], len(datagram.payload)))
        self.file.write(datagram.payload)
        self.records += 1

    def close(self) -> None:
        if self.file is not None:
            self.file.close()
            self.file = None

    def __enter__(self) -> "MotionVenusCaptureWriter":
        self.open()
        return self

    def __exit__(self, *_args) -> None:
        self.close()


def read_capture(path: str | Path) -> Iterator[MotionVenusDatagram]:
    with Path(path).open("rb") as stream:
        if stream.read(len(CAPTURE_MAGIC)) != CAPTURE_MAGIC:
            raise ValueError("not a MotionVenus UDP capture or unsupported capture version")
        while True:
            raw_header = stream.read(_CAPTURE_RECORD.size)
            if not raw_header:
                return
            if len(raw_header) != _CAPTURE_RECORD.size:
                raise ValueError("truncated MotionVenus capture record header")
            timestamp, address, port, length = _CAPTURE_RECORD.unpack(raw_header)
            if length > MAX_UDP_PAYLOAD:
                raise ValueError(f"impossible captured UDP payload length {length}")
            payload = stream.read(length)
            if len(payload) != length:
                raise ValueError("truncated MotionVenus capture payload")
            yield MotionVenusDatagram(
                payload,
                timestamp,
                time.monotonic_ns(),
                (str(ipaddress.IPv4Address(address)), port),
            )


class MotionVenusReplaySource:
    def __init__(self, path: str | Path, *, realtime: bool = False, speed: float = 1.0):
        if speed <= 0:
            raise ValueError("replay speed must be positive")
        self.path = Path(path)
        self.realtime, self.speed = realtime, speed
        self._records: Iterator[MotionVenusDatagram] | None = None
        self._previous_timestamp: int | None = None
        self.eof = False

    def start(self) -> None:
        self._records = iter(read_capture(self.path))
        self._previous_timestamp = None
        self.eof = False

    def receive(self) -> MotionVenusDatagram | None:
        if self._records is None:
            raise RuntimeError("replay.start() must be called before receive()")
        try:
            record = next(self._records)
        except StopIteration:
            self.eof = True
            return None
        if self.realtime and self._previous_timestamp is not None:
            delay = max(0.0, (record.received_ns - self._previous_timestamp) / 1e9 / self.speed)
            time.sleep(min(delay, 1.0))
        self._previous_timestamp = record.received_ns
        return MotionVenusDatagram(record.payload, record.received_ns, time.monotonic_ns(), record.sender)

    def close(self) -> None:
        self._records = None
