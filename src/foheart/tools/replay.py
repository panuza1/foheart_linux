from __future__ import annotations

import argparse
from pathlib import Path

from foheart.protocol.definitions import ProtocolNotDecodedError
from foheart.protocol.frame import RawTransfer, iter_recording
from foheart.protocol.parser import C1ProtocolParser


def replay_transfers(path: str | Path) -> list[RawTransfer]:
    return list(iter_recording(path))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Replay a foheart raw recording")
    parser.add_argument("capture", type=Path)
    args = parser.parse_args(argv)
    protocol = C1ProtocolParser()
    decoder_notice_printed = False
    count = 0
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
