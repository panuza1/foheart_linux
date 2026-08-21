from __future__ import annotations

import struct
from dataclasses import dataclass
from pathlib import Path
from typing import BinaryIO, Iterator

MAGIC = b"FHC1RAW\x01"
RECORD_HEADER = struct.Struct("<QBI")
POLL_MAGIC = b"FHC1POL\x01"
POLL_RECORD_HEADER = struct.Struct("<IQHQBQBIH")
MAX_PAYLOAD_SIZE = 16 * 1024 * 1024


@dataclass(frozen=True)
class RawTransfer:
    timestamp_ns: int
    endpoint: int
    payload: bytes


@dataclass(frozen=True)
class PollCaptureRecord:
    sequence: int
    poll_timestamp_ns: int
    out_transferred: int
    in_timestamp_ns: int | None
    in_endpoint: int
    payload: bytes
    timed_out: bool
    error: str | None
    round_trip_ns: int


class RecordingFormatError(ValueError):
    pass


class RawRecorder:
    def __init__(self, stream: BinaryIO):
        self.stream = stream
        self.stream.write(MAGIC)

    def write(self, transfer: RawTransfer) -> None:
        if not 0 <= transfer.endpoint <= 0xFF:
            raise ValueError("endpoint must fit in one byte")
        if len(transfer.payload) > MAX_PAYLOAD_SIZE:
            raise ValueError("payload is too large")
        self.stream.write(
            RECORD_HEADER.pack(
                transfer.timestamp_ns, transfer.endpoint, len(transfer.payload)
            )
        )
        self.stream.write(transfer.payload)


class PollRecorder:
    """Versioned, boundary-preserving records for exact-poll capture attempts."""

    def __init__(self, stream: BinaryIO):
        self.stream = stream
        self.stream.write(POLL_MAGIC)

    def write(self, record: PollCaptureRecord) -> None:
        if record.sequence < 1:
            raise ValueError("poll sequence must be positive")
        if not 0 <= record.out_transferred <= 0xFFFF:
            raise ValueError("OUT transfer length must fit in uint16")
        if not 0 <= record.in_endpoint <= 0xFF:
            raise ValueError("endpoint must fit in one byte")
        if len(record.payload) > MAX_PAYLOAD_SIZE:
            raise ValueError("payload is too large")
        if record.timed_out and record.error is not None:
            raise ValueError("a poll record cannot be both timeout and error")
        status = 2 if record.error is not None else 1 if record.timed_out else 0
        error = record.error.encode("utf-8") if record.error is not None else b""
        if len(error) > 0xFFFF:
            raise ValueError("error text is too large")
        self.stream.write(
            POLL_RECORD_HEADER.pack(
                record.sequence,
                record.poll_timestamp_ns,
                record.out_transferred,
                record.in_timestamp_ns or 0,
                record.in_endpoint,
                record.round_trip_ns,
                status,
                len(record.payload),
                len(error),
            )
        )
        self.stream.write(record.payload)
        self.stream.write(error)


def iter_recording(path: str | Path) -> Iterator[RawTransfer]:
    with Path(path).open("rb") as stream:
        if stream.read(len(MAGIC)) != MAGIC:
            raise RecordingFormatError("Not a foheart raw recording or unsupported version")
        while header := stream.read(RECORD_HEADER.size):
            if len(header) != RECORD_HEADER.size:
                raise RecordingFormatError("Truncated record header")
            timestamp_ns, endpoint, payload_length = RECORD_HEADER.unpack(header)
            if payload_length > MAX_PAYLOAD_SIZE:
                raise RecordingFormatError(f"Invalid payload length: {payload_length}")
            payload = stream.read(payload_length)
            if len(payload) != payload_length:
                raise RecordingFormatError("Truncated record payload")
            yield RawTransfer(timestamp_ns, endpoint, payload)


def iter_poll_recording(path: str | Path) -> Iterator[PollCaptureRecord]:
    with Path(path).open("rb") as stream:
        if stream.read(len(POLL_MAGIC)) != POLL_MAGIC:
            raise RecordingFormatError(
                "Not a foheart poll recording or unsupported version"
            )
        while header := stream.read(POLL_RECORD_HEADER.size):
            if len(header) != POLL_RECORD_HEADER.size:
                raise RecordingFormatError("Truncated poll record header")
            (
                sequence,
                poll_timestamp_ns,
                out_transferred,
                in_timestamp_ns,
                in_endpoint,
                round_trip_ns,
                status,
                payload_length,
                error_length,
            ) = POLL_RECORD_HEADER.unpack(header)
            if status not in (0, 1, 2):
                raise RecordingFormatError(f"Invalid poll status: {status}")
            if payload_length > MAX_PAYLOAD_SIZE:
                raise RecordingFormatError(f"Invalid payload length: {payload_length}")
            payload = stream.read(payload_length)
            error_bytes = stream.read(error_length)
            if len(payload) != payload_length or len(error_bytes) != error_length:
                raise RecordingFormatError("Truncated poll record data")
            try:
                error = error_bytes.decode("utf-8") if status == 2 else None
            except UnicodeDecodeError as exc:
                raise RecordingFormatError("Invalid UTF-8 poll error text") from exc
            yield PollCaptureRecord(
                sequence,
                poll_timestamp_ns,
                out_transferred,
                in_timestamp_ns or None,
                in_endpoint,
                payload,
                status == 1,
                error,
                round_trip_ns,
            )
