from __future__ import annotations

import argparse
from pathlib import Path

from foheart.protocol.definitions import ProtocolNotDecodedError
from foheart.protocol.frame import (
    POLL_MAGIC,
    PollCaptureRecord,
    RawTransfer,
    iter_poll_recording,
    iter_recording,
)
from foheart.protocol.parser import C1ProtocolParser


def replay_transfers(path: str | Path) -> list[RawTransfer]:
    return list(iter_recording(path))


def replay_poll_attempts(path: str | Path) -> list[PollCaptureRecord]:
    return list(iter_poll_recording(path))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Replay a foheart raw recording")
    parser.add_argument("capture", type=Path)
    args = parser.parse_args(argv)
    is_poll_capture = args.capture.read_bytes()[: len(POLL_MAGIC)] == POLL_MAGIC
    protocol = C1ProtocolParser()
    decoder_notice_printed = False
    count = 0
    if is_poll_capture:
        in_count = 0
        decoded_count = 0
        for count, record in enumerate(iter_poll_recording(args.capture), 1):
            print(
                f"poll={record.sequence} out_bytes={record.out_transferred} "
                f"in_bytes={len(record.payload)} timeout={record.timed_out} "
                f"error={record.error or 'NONE'}"
            )
            if not record.payload:
                continue
            in_count += 1
            try:
                frames = protocol.feed(
                    record.payload, timestamp_ns=record.in_timestamp_ns
                )
            except ProtocolNotDecodedError as exc:
                if not decoder_notice_printed:
                    print(f"Protocol decoder: NOT READY ({exc})")
                    decoder_notice_printed = True
            else:
                decoded_count += len(frames)
                if not decoder_notice_printed:
                    print("Protocol decoder: HID 0x15 REAL_CAPTURE_VALIDATED")
                    decoder_notice_printed = True
        print(
            f"Replayed {count} poll attempts, {in_count} IN transfers, "
            f"and {decoded_count} decoded frames"
        )
        return 0
    for count, transfer in enumerate(iter_recording(args.capture), 1):
        print(
            f"timestamp_ns={transfer.timestamp_ns} transfer={count} "
            f"bytes={len(transfer.payload)} endpoint=0x{transfer.endpoint:02x}"
        )
        try:
            protocol.feed(transfer.payload, timestamp_ns=transfer.timestamp_ns)
        except ProtocolNotDecodedError as exc:
            if not decoder_notice_printed:
                print(f"Protocol decoder: NOT READY ({exc})")
                decoder_notice_printed = True
    print(f"Replayed {count} raw transfers")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
