from __future__ import annotations

import argparse
import statistics
from collections import Counter
from pathlib import Path

from foheart.protocol.frame import PollRecorder
from foheart.protocol.poll import C1_HID_POLL
from foheart.usb.c1_device import C1NotFoundError, C1OpenError
from foheart.usb.c1_poll import (
    C1_HID_CAPTURE_MAX_POLLS,
    C1_HID_CAPTURE_MAX_RUNTIME_S,
    C1_HID_POLL_TIMEOUT_MS,
    capture_polls,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Bounded capture using only the exact 64-byte C1 HID poll"
    )
    parser.add_argument("--polls", type=int, default=C1_HID_CAPTURE_MAX_POLLS)
    parser.add_argument("--timeout-ms", type=int, default=C1_HID_POLL_TIMEOUT_MS)
    parser.add_argument(
        "--max-runtime-s", type=float, default=C1_HID_CAPTURE_MAX_RUNTIME_S
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--hex", action="store_true", dest="show_hex")
    return parser


def _format_counter(counter: Counter[int], *, hexadecimal: bool = False) -> str:
    if not counter:
        return "NONE"
    return ", ".join(
        f"{'0x' + key.to_bytes(1, 'big').hex() if hexadecimal else key}: {value}"
        for key, value in sorted(counter.items())
    )


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    print("=" * 64)
    print("EXPERIMENTAL FOHEART C1 BOUNDED POLL CAPTURE")
    print("Authorized packet: 0x70 + 63 zero bytes only")
    print(f"Maximum writes this run: {args.polls} (hard cap 200)")
    print(f"IN timeout: {args.timeout_ms} ms; total runtime cap: {args.max_runtime_s:g} s")
    print("No retry after USB error or short OUT; no other command is available")
    print("=" * 64)
    print(f"OUT payload ({len(C1_HID_POLL)} bytes): {C1_HID_POLL.hex(' ')}")
    if args.output.exists():
        print(f"ERROR: refusing to overwrite capture: {args.output}")
        return 2
    try:
        capture = capture_polls(
            max_polls=args.polls,
            timeout_ms=args.timeout_ms,
            max_runtime_s=args.max_runtime_s,
        )
    except (ValueError, C1NotFoundError, C1OpenError) as exc:
        print(f"ERROR: {exc}")
        return 2

    with args.output.open("xb") as stream:
        recorder = PollRecorder(stream)
        for record in capture.records:
            recorder.write(record)

    for record in capture.records:
        detail = (
            f" error={record.error}"
            if record.error is not None
            else " timeout=true"
            if record.timed_out
            else f" in_bytes={len(record.payload)}"
        )
        preview = f" hex={record.payload.hex(' ')}" if args.show_hex and record.payload else ""
        print(
            f"poll={record.sequence} out_bytes={record.out_transferred}{detail} "
            f"round_trip_ms={record.round_trip_ns / 1_000_000:.3f}{preview}"
        )

    payloads = [record.payload for record in capture.records if record.payload]
    lengths = Counter(map(len, payloads))
    first_bytes = Counter(payload[0] for payload in payloads)
    prefixes = Counter(payload[:4].hex(" ") for payload in payloads)
    successful_rtt = [
        record.round_trip_ns / 1_000_000
        for record in capture.records
        if record.payload
    ]
    elapsed_s = capture.elapsed_ns / 1_000_000_000
    print("Summary:")
    print(f"  polls attempted: {len(capture.records)}")
    print(f"  OUT successes: {sum(record.out_transferred == 64 for record in capture.records)}")
    print(f"  IN reports: {len(payloads)}")
    print(f"  timeouts: {sum(record.timed_out for record in capture.records)}")
    print(f"  errors: {sum(record.error is not None for record in capture.records)}")
    print(f"  IN length distribution: {_format_counter(lengths)}")
    print(f"  first-byte distribution: {_format_counter(first_bytes, hexadecimal=True)}")
    print(
        "  common 4-byte prefixes: "
        + (", ".join(f"{key}: {value}" for key, value in prefixes.most_common(10)) or "NONE")
    )
    print(f"  approximate report rate: {len(payloads) / elapsed_s:.3f} reports/s")
    if successful_rtt:
        print(
            "  successful round-trip ms min/mean/max: "
            f"{min(successful_rtt):.3f}/{statistics.mean(successful_rtt):.3f}/{max(successful_rtt):.3f}"
        )
    else:
        print("  successful round-trip ms min/mean/max: NONE")
    print(f"  stop reason: {capture.stop_reason}")
    print(f"  capture: {args.output}")
    return 3 if capture.hard_error else 0


if __name__ == "__main__":
    raise SystemExit(main())
