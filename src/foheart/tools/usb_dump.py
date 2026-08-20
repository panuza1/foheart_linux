from __future__ import annotations

import argparse
import time
from contextlib import nullcontext
from pathlib import Path

import usb.core

from foheart.config import (
    ConfigError,
    add_config_arguments,
    load_config_from_args,
    require_read_only,
)
from foheart.constants import FOHEART_VID
from foheart.protocol.frame import RawRecorder, RawTransfer
from foheart.protocol.parser import resolve_outer_frame
from foheart.usb.c1_device import C1Device, C1NotFoundError, C1OpenError


def auto_int(value: str) -> int:
    return int(value, 0)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Read and optionally record raw C1 USB transfers")
    parser.add_argument("--vid", type=auto_int, default=FOHEART_VID)
    parser.add_argument("--count", type=int, default=100)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--hex", action="store_true", dest="show_hex")
    add_config_arguments(parser)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.count < 1:
        raise SystemExit("--count must be positive")
    try:
        config = load_config_from_args(args)
        require_read_only(config)
    except ConfigError as exc:
        print(f"Configuration error: {exc}")
        return 2
    try:
        device_context = C1Device.open_first(
            args.vid,
            config.usb.pid,
            usb_mode=config.usb.mode,
            interface=config.usb.interface,
            in_endpoint=config.usb.in_endpoint,
            out_endpoint=config.usb.out_endpoint,
            read_size=config.usb.read_size,
        )
    except C1NotFoundError:
        print("NO FOHEART C1 DEVICE CONNECTED")
        return 1
    except C1OpenError as exc:
        print(exc)
        return 2

    output_context = args.output.open("wb") if args.output else nullcontext(None)
    with device_context as device, output_context as output:
        assert device.selection is not None
        recorder = RawRecorder(output) if output else None
        for transfer_number in range(1, args.count + 1):
            try:
                payload = device.read(timeout_ms=config.usb.timeout_ms)
            except usb.core.USBTimeoutError:
                print(
                    f"transfer={transfer_number} timeout_ms={config.usb.timeout_ms}"
                )
                continue
            except usb.core.USBError as exc:
                print(f"USB read failed on transfer {transfer_number}: {exc}")
                return 3
            timestamp_ns = time.time_ns()
            transfer = RawTransfer(timestamp_ns, device.selection.in_endpoint, payload)
            if recorder:
                recorder.write(transfer)
            preview = f" hex={payload[:32].hex(' ')}" if args.show_hex else ""
            resolved_outer = resolve_outer_frame(
                payload, config.protocol.outer_frame
            )
            fallback = (
                " fallback=true"
                if resolved_outer == "raw"
                and config.protocol.outer_frame != "raw"
                else ""
            )
            print(
                f"timestamp_ns={timestamp_ns} transfer={transfer_number} "
                f"bytes={len(payload)} endpoint=0x{device.selection.in_endpoint:02x} "
                f"outer_frame={resolved_outer}{fallback}{preview}"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
