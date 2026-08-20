from __future__ import annotations

import struct
from dataclasses import dataclass
from pathlib import Path
from typing import BinaryIO, Iterator

MAGIC = b"FHC1RAW\x01"
RECORD_HEADER = struct.Struct("<QBI")
MAX_PAYLOAD_SIZE = 16 * 1024 * 1024


@dataclass(frozen=True)
class RawTransfer:
    timestamp_ns: int
    endpoint: int
    payload: bytes


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

