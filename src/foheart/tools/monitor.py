from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import numpy as np
import usb.core

from foheart.config import (
    ConfigError,
    MonitorConfig,
    RuntimeConfig,
    add_config_arguments,
    load_config_from_args,
    require_read_only,
)
from foheart.constants import FOHEART_VID
from foheart.mocap.sensor import Quaternion, SensorFrame, SensorSample, Vector3
from foheart.protocol.definitions import ProtocolError
from foheart.protocol.frame import iter_poll_recording
from foheart.protocol.parser import (
    C1ProtocolParser,
    resolve_outer_frame,
    resolve_sensor_id_mode,
)
from foheart.usb.c1_device import C1Device, C1NotFoundError, C1OpenError


def generate_mock_frame(
    frame_number: int, sensor_count: int = 4, timestamp_ns: int | None = None
) -> SensorFrame:
    angle = frame_number * 0.02
    sensors = []
    for sensor_id in range(sensor_count):
        phase = angle + sensor_id * 0.1
        sensors.append(
            SensorSample(
                sensor_id=sensor_id,
                online=True,
                quaternion=Quaternion(
                    (float(np.cos(phase / 2)), 0.0, 0.0, float(np.sin(phase / 2))),
                    component_order="wxyz (mock only)",
                ),
                accel=Vector3(float(math.sin(phase)), 0.0, 9.81),
            )
        )
    return SensorFrame(timestamp_ns or time.time_ns(), frame_number, sensors)


def _auto(value: int | None, *, hexadecimal: bool = False) -> str:
    if value is None:
        return "auto"
    return f"0x{value:02x}" if hexadecimal else str(value)


def _print_runtime_config(
    config: RuntimeConfig,
    device: C1Device | None = None,
) -> None:
    selection = device.selection if device else None
    resolved_sensor_id = resolve_sensor_id_mode(config.protocol.sensor_id_mode)
    initial_parser = {
        "raw": "raw",
        "fixed_0x13": "fixed_0x13 (payload validation pending)",
        "auto": "pending first transfer",
    }[config.protocol.outer_frame]
    print("FOHEART runtime configuration\n")
    print("Device:")
    print(f"  VID: {FOHEART_VID:04x}")
    print(
        f"  PID: {device.pid:04x}"
        if device
        else f"  PID: {_auto(config.usb.pid, hexadecimal=True)}"
    )
    print(
        f"  USB mode: {selection.transfer_type}"
        if selection
        else f"  USB mode: {config.usb.mode.upper()}"
    )
    print("\nTransport:")
    print(
        f"  interface: {selection.interface_number}"
        if selection
        else f"  interface: {_auto(config.usb.interface)}"
    )
    print(
        f"  IN: 0x{selection.in_endpoint:02x}"
        if selection
        else f"  IN: {_auto(config.usb.in_endpoint, hexadecimal=True)}"
    )
    print(
        f"  OUT: {f'0x{selection.out_endpoint:02x}' if selection.out_endpoint is not None else 'none'}"
        if selection
        else f"  OUT: {_auto(config.usb.out_endpoint, hexadecimal=True)}"
    )
    print(f"  timeout: {config.usb.timeout_ms} ms")
    if config.usb.read_size is not None:
        print(f"  read size: {config.usb.read_size}")
    elif selection:
        print(f"  read size: {selection.read_size} (descriptor/default)")
    else:
        print("  read size: descriptor/default")
    print("\nProtocol:")
    print(f"  outer frame: {config.protocol.outer_frame}")
    print(f"  resolved parser: {initial_parser}")
    print(f"  sensor ID mode: {resolved_sensor_id}")
    if resolved_sensor_id == "loop_index":
        print("  identity label: provisional loop index")
    elif resolved_sensor_id == "decoded_index":
        print("  identity label: decoded packed index (PARTIAL)")
    else:
        print("  identity label: slot only; hardware identity UNKNOWN")
    print("\nStream:")
    print("  READ ONLY")
    print("\nUSB transport: CONFIRMED STATICALLY / HARDWARE VALIDATION PENDING")
    print("Outer frame: PARTIAL")
    print("Sensor ID: PARTIAL")
    print("Start stream: UNKNOWN")


def _print_frame(
    frame: SensorFrame,
    monitor: MonitorConfig,
    sensor_id_mode: str,
    *,
    mock: bool = False,
) -> None:
    print(f"Frame {frame.frame_number if frame.frame_number is not None else '-'}")
    identity = (
        "Mock ID"
        if mock
        else {
            "loop_index": "Loop index",
            "decoded_index": "Decoded index",
            "unknown": "Slot",
        }[sensor_id_mode]
    )
    for sensor in frame.sensors:
        fields = [
            f"{identity} {sensor.sensor_id}",
            f"online={'YES' if sensor.online else 'UNKNOWN' if sensor.online is None else 'NO'}",
        ]
        if monitor.show_quaternion and sensor.quaternion:
            fields.append(f"quat={tuple(round(value, 4) for value in sensor.quaternion.values)}")
            fields.append(
                "quat_norm="
                f"{math.sqrt(sum(value * value for value in sensor.quaternion.values)):.6f}"
            )
        if monitor.show_euler and sensor.euler:
            fields.append(f"euler=({sensor.euler.x:.4f}, {sensor.euler.y:.4f}, {sensor.euler.z:.4f})")
        if monitor.show_imu:
            for name, vector in (
                ("accel", sensor.accel),
                ("gyro", sensor.gyro),
                ("mag", sensor.magnetometer),
            ):
                if vector:
                    fields.append(f"{name}=({vector.x:.4f}, {vector.y:.4f}, {vector.z:.4f})")
        if sensor.field_status:
            fields.append(
                "status="
                + ",".join(f"{name}:{status}" for name, status in sensor.field_status)
            )
        print("  " + "  ".join(fields))


def _print_raw(payload: bytes, endpoint: int, show_raw: bool) -> None:
    preview = f" hex={payload[:32].hex(' ')}" if show_raw else ""
    print(f"Raw transfer: bytes={len(payload)} endpoint=0x{endpoint:02x}{preview}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="FOHEART terminal sensor monitor")
    parser.add_argument("--mock", action="store_true")
    parser.add_argument(
        "--capture",
        type=Path,
        help="offline boundary-preserving poll capture; performs no USB operations",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=0,
        help="mock frames or live read attempts; live 0 prints status only",
    )
    parser.add_argument("--sensors", type=int, default=4)
    add_config_arguments(parser)
    for name in ("raw", "euler", "quaternion", "imu"):
        parser.add_argument(
            f"--show-{name}",
            action=argparse.BooleanOptionalAction,
            default=argparse.SUPPRESS,
        )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.mock and args.capture:
        raise SystemExit("--mock and --capture are mutually exclusive")
    if args.count < 0 or args.sensors < 1:
        raise SystemExit("--count must be non-negative and --sensors must be positive")
    try:
        config = load_config_from_args(args)
        require_read_only(config)
    except ConfigError as exc:
        print(f"Configuration error: {exc}")
        return 2

    sensor_id_mode = resolve_sensor_id_mode(config.protocol.sensor_id_mode)
    if args.capture:
        protocol = C1ProtocolParser("unknown")
        decoded = 0
        print(f"FOHEART OFFLINE CAPTURE MONITOR: {args.capture}")
        print("NO USB OPERATIONS")
        try:
            records = iter_poll_recording(args.capture)
            for record in records:
                if not record.payload:
                    continue
                try:
                    frames = protocol.feed(
                        record.payload, timestamp_ns=record.in_timestamp_ns
                    )
                except ProtocolError as exc:
                    print(f"poll={record.sequence} raw/undecoded: {exc}")
                    continue
                for frame in frames:
                    _print_frame(frame, config.monitor, "unknown", mock=False)
                    decoded += 1
                    if args.count and decoded >= args.count:
                        print(f"Decoded {decoded} frames")
                        return 0
        except OSError as exc:
            print(f"Could not read capture: {exc}")
            return 2
        print(f"Decoded {decoded} frames")
        return 0
    if args.mock:
        _print_runtime_config(config)
        print("\n*** MOCK DATA ***")
        frame_number = 0
        try:
            while args.count == 0 or frame_number < args.count:
                _print_frame(
                    generate_mock_frame(frame_number, args.sensors),
                    config.monitor,
                    sensor_id_mode,
                    mock=True,
                )
                frame_number += 1
                if args.count == 0 or frame_number < args.count:
                    time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        return 0

    try:
        with C1Device.open_first(
            pid=config.usb.pid,
            usb_mode=config.usb.mode,
            interface=config.usb.interface,
            in_endpoint=config.usb.in_endpoint,
            out_endpoint=config.usb.out_endpoint,
            read_size=config.usb.read_size,
        ) as device:
            assert device.selection is not None
            _print_runtime_config(config, device)
            if args.count == 0:
                return 0

            protocol = C1ProtocolParser(sensor_id_mode)
            for transfer_number in range(1, args.count + 1):
                try:
                    payload = device.read(timeout_ms=config.usb.timeout_ms)
                except usb.core.USBTimeoutError:
                    print(
                        f"Read {transfer_number}: timeout after {config.usb.timeout_ms} ms "
                        "(read-only; no start command sent)"
                    )
                    continue
                resolved_outer = resolve_outer_frame(
                    payload, config.protocol.outer_frame
                )
                if resolved_outer == "fixed_0x13":
                    try:
                        frames = protocol.feed(payload)
                    except ProtocolError as exc:
                        print(f"Protocol fallback: raw ({exc})")
                        _print_raw(
                            payload,
                            device.selection.in_endpoint,
                            config.monitor.show_raw,
                        )
                        continue
                    print("Protocol selection: fixed_0x13 (PARTIAL)")
                    if config.monitor.show_raw:
                        _print_raw(payload, device.selection.in_endpoint, True)
                    for frame in frames:
                        _print_frame(
                            frame, config.monitor, sensor_id_mode, mock=False
                        )
                else:
                    if config.protocol.outer_frame != "raw":
                        print("Protocol fallback: raw (transfer does not match fixed_0x13)")
                    _print_raw(
                        payload,
                        device.selection.in_endpoint,
                        config.monitor.show_raw,
                    )
    except C1NotFoundError:
        _print_runtime_config(config)
        print("\nNO FOHEART C1 DEVICE CONNECTED")
        return 1
    except C1OpenError as exc:
        print(exc)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
