"""Read-only MotionVenus UDP protocol and health monitor."""

from __future__ import annotations

import argparse
from pathlib import Path
import time

from foheart.config import load_config
from foheart.motionvenus.protocol import MotionVenusProtocolError, MotionVenusStreamDecoder
from foheart.motionvenus.transport import MotionVenusCaptureWriter, MotionVenusReceiver, MotionVenusWatchdog


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Monitor solved MotionVenus UDP frames; no G1 action")
    parser.add_argument("--config", type=Path)
    parser.add_argument("--bind")
    parser.add_argument("--port", type=int)
    parser.add_argument("--format", choices=("auto", "binary", "json"))
    parser.add_argument("--timeout", type=float, help="socket timeout in seconds")
    parser.add_argument("--duration", type=float)
    parser.add_argument("--debug", action="store_true")
    parser.add_argument(
        "--dump-packets",
        type=Path,
        nargs="?",
        const=Path("samples/motionvenus_monitor.bin"),
        help="capture raw packet boundaries, optionally to PATH",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    config = load_config(args.config)
    settings = config.motionvenus
    bind, port = args.bind or settings.bind, args.port or settings.port
    timeout = args.timeout or settings.receive_timeout_ms / 1000
    decoder = MotionVenusStreamDecoder(
        expected_body_bones=settings.expected_body_bones,
        packet_format=args.format or settings.format,
    )
    receiver = MotionVenusReceiver(bind, port, timeout_s=timeout)
    watchdog = MotionVenusWatchdog(stale_after_s=settings.stale_after_ms / 1000)
    capture = MotionVenusCaptureWriter(args.dump_packets) if args.dump_packets else None
    deadline = None if args.duration is None else time.monotonic() + args.duration
    latest = None
    last_print = 0.0
    print(f"MotionVenus UDP\n\nBind: {bind}:{port}\nG1 action: NONE")
    try:
        receiver.start()
        if capture:
            capture.open()
        while deadline is None or time.monotonic() < deadline:
            datagram = receiver.receive()
            if datagram is not None:
                if capture:
                    capture.write(datagram)
                try:
                    latest = decoder.decode(
                        datagram.payload,
                        received_ns=datagram.received_ns,
                        sender=datagram.sender,
                    )
                    watchdog.observe(latest, monotonic_ns=datagram.monotonic_ns)
                except MotionVenusProtocolError as exc:
                    receiver.record_malformed()
                    watchdog.mark_error(exc, protocol_mismatch=exc.kind == "protocol_mismatch")
                    if args.debug:
                        print(f"Rejected {len(datagram.payload)} bytes from {datagram.sender}: {exc}")
            now = time.monotonic()
            if now - last_print >= 1.0:
                diagnostics, stats = watchdog.diagnostics(), receiver.stats
                sender = f"{latest.sender[0]}:{latest.sender[1]}" if latest else "-"
                frame_number = latest.header.frame_number if latest else "-"
                suit = latest.header.suit_number if latest else "-"
                bones = latest.header.body_skeleton_count if latest else "-"
                protocol = latest.header.protocol_version if latest else "-"
                print(
                    f"Status: {diagnostics.status:<18} Sender: {sender:<22} "
                    f"Packets: {stats.packets:<7} Rate: {stats.rate_hz:5.1f} Hz "
                    f"Frame: {frame_number} Suit: {suit} Bones: {bones} Protocol: {protocol} "
                    f"Lost~: {diagnostics.estimated_lost_frames} Dup: {diagnostics.duplicate_frames} "
                    f"OOO: {diagnostics.out_of_order_frames} Bad: {stats.malformed_packets}"
                )
                last_print = now
    except KeyboardInterrupt:
        print("Stopped by operator.")
    finally:
        receiver.close()
        if capture:
            capture.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

