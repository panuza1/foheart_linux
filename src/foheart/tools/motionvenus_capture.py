"""Versioned raw UDP capture with packet boundaries and sender metadata."""

from __future__ import annotations

import argparse
from pathlib import Path
import time

from foheart.config import load_config
from foheart.motionvenus.transport import MotionVenusCaptureWriter, MotionVenusReceiver


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Capture MotionVenus UDP datagrams; no parser and no G1 action")
    parser.add_argument("--config", type=Path)
    parser.add_argument("--bind")
    parser.add_argument("--port", type=int)
    parser.add_argument("--timeout", type=float)
    parser.add_argument("--duration", type=float)
    parser.add_argument("--max-packets", type=int)
    parser.add_argument("--output", type=Path, required=True)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    config = load_config(args.config).motionvenus
    receiver = MotionVenusReceiver(
        args.bind or config.bind,
        args.port or config.port,
        timeout_s=args.timeout or config.receive_timeout_ms / 1000,
    )
    deadline = None if args.duration is None else time.monotonic() + args.duration
    count = 0
    try:
        with receiver, MotionVenusCaptureWriter(args.output) as capture:
            print(f"Capturing {receiver.bind}:{receiver.port} -> {args.output} (G1 action: NONE)")
            while (deadline is None or time.monotonic() < deadline) and (
                args.max_packets is None or count < args.max_packets
            ):
                datagram = receiver.receive()
                if datagram is not None:
                    capture.write(datagram)
                    count += 1
    except KeyboardInterrupt:
        print("Stopped by operator.")
    print(f"Captured {count} packet(s) with boundaries preserved.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
