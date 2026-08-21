"""Logical-slot monitor for synthetic, replay, and future live C1 sources."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import sys
import time

from foheart.config import load_config
from foheart.mocap.stream import (
    LogicalSlotRegistry,
    SensorSourceError,
    create_sensor_source,
)
from foheart.mocap.suit import BodyProfile, roles_for_profile


@dataclass(frozen=True)
class MonitorRow:
    slot: str
    age_ms: float
    packet_rate_hz: float
    quaternion_norm: float | None
    gyro_norm: float | None
    status: str
    transport_key: str


def monitor_rows(
    registry: LogicalSlotRegistry, timestamp_ns: int, stale_after_ms: float
) -> tuple[MonitorRow, ...]:
    rows = []
    for logical in registry.sensors.values():
        quaternion = logical.last_sample.quaternion
        gyro = logical.last_sample.gyro
        age_ms = max(0.0, (timestamp_ns - logical.last_seen_ns) / 1_000_000)
        rows.append(
            MonitorRow(
                logical.slot,
                age_ms,
                logical.packet_rate_hz,
                None
                if quaternion is None
                else math.sqrt(sum(value * value for value in quaternion.values)),
                None
                if gyro is None
                else math.sqrt(gyro.x**2 + gyro.y**2 + gyro.z**2),
                "STALE" if age_ms > stale_after_ms else "OK",
                logical.transport_key.debug_label,
            )
        )
    return tuple(rows)


def _print_snapshot(source, registry, now_ns, stale_after_ms, debug_transport, profile):
    print(f"FOHEART C1: {source.c1_status}")
    print(
        f"Source: {source.source_name}  Profile: {profile.upper()}  "
        f"Sensors detected: {len(registry.sensors)}"
    )
    expected = len(roles_for_profile(profile))
    print(f"Expected stable slots for profile: {expected}")
    print("Slot       Age ms       Hz   Quat norm   Gyro norm  Status")
    for row in monitor_rows(registry, now_ns, stale_after_ms):
        quat = "       -" if row.quaternion_norm is None else f"{row.quaternion_norm:9.5f}"
        gyro = "        -" if row.gyro_norm is None else f"{row.gyro_norm:10.3f}"
        print(
            f"{row.slot:<9} {row.age_ms:8.1f} {row.packet_rate_hz:8.1f} "
            f"{quat} {gyro}  {row.status}"
        )
        if debug_transport:
            print(f"  TRANSPORT_KEY: {row.transport_key}")
    for diagnostic in registry.diagnostics:
        print(f"REGISTRY WARNING: {diagnostic}")
    if len(registry.sensors) < expected:
        print(
            "STATUS: INCOMPLETE — SENSOR MISSING OR CANDIDATE TRANSPORT-KEY "
            "COLLISION; DO NOT MAP"
        )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="FOHEART logical live sensor monitor")
    parser.add_argument("--config", type=Path)
    parser.add_argument("--mode", choices=tuple(profile.value for profile in BodyProfile))
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--synthetic", action="store_true")
    source.add_argument("--replay", type=Path)
    parser.add_argument("--duration", type=float)
    parser.add_argument("--samples", type=int)
    parser.add_argument("--refresh", type=float, default=0.25)
    parser.add_argument("--debug-transport", action="store_true")
    parser.add_argument("--no-clear", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.duration is not None and args.duration <= 0:
        raise SystemExit("--duration must be positive")
    if args.samples is not None and args.samples < 1:
        raise SystemExit("--samples must be positive")
    if args.refresh <= 0:
        raise SystemExit("--refresh must be positive")
    try:
        config = load_config(args.config)
        profile = args.mode or config.viewer.mode
        source = create_sensor_source(
            synthetic=args.synthetic,
            replay=args.replay,
            fps=config.viewer.fps,
            profile=profile,
        )
        source.start()
    except (OSError, RuntimeError, ValueError, SensorSourceError) as exc:
        print(f"Cannot start sensor monitor: {exc}")
        return 2

    registry = LogicalSlotRegistry()
    started = time.monotonic()
    next_refresh = started
    count = 0
    now_ns = time.time_ns()
    try:
        while True:
            event = source.next_sample()
            if event is not None:
                registry.observe(event)
                count += 1
                now_ns = event.timestamp_ns
            elif source.eof:
                break
            else:
                now_ns = time.time_ns()
            now = time.monotonic()
            if now >= next_refresh:
                if not args.no_clear and sys.stdout.isatty():
                    print("\033[2J\033[H", end="")
                _print_snapshot(
                    source,
                    registry,
                    now_ns,
                    config.stream.stale_after_ms,
                    args.debug_transport,
                    profile,
                )
                next_refresh = now + args.refresh
            if args.samples is not None and count >= args.samples:
                break
            if args.duration is not None and now - started >= args.duration:
                break
    except KeyboardInterrupt:
        print("\nSensor monitor stopped.")
    except (OSError, RuntimeError, ValueError, SensorSourceError) as exc:
        print(f"Sensor monitor stopped: {exc}")
        return 2
    finally:
        source.close()
    _print_snapshot(
        source,
        registry,
        now_ns,
        config.stream.stale_after_ms,
        args.debug_transport,
        profile,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
