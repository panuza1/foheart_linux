"""Interactive logical-slot mapping for upper- or full-body profiles."""

from __future__ import annotations

import argparse
from collections import defaultdict
from pathlib import Path
import time

from foheart.config import load_config
from foheart.mocap.stream import (
    LogicalSlotRegistry,
    SensorSourceError,
    create_sensor_source,
    propose_moving_slot,
)
from foheart.mocap.suit import BodyProfile, BodySensorMap, roles_for_profile


def confirm_assignment(answer: str) -> bool:
    return answer.strip().lower() in {"y", "yes"}


def _collect_slots(source, registry, *, expected: int, seconds: float) -> None:
    deadline = time.monotonic() + seconds
    maximum_events = max(
        100, round(getattr(source, "fps", 30.0) * seconds * expected)
    )
    for _ in range(maximum_events):
        event = source.next_sample()
        if event is not None:
            registry.observe(event)
            if len(registry.sensors) >= expected:
                return
        if source.eof or time.monotonic() >= deadline:
            return


def _motion_window(source, registry, seconds, expected):
    samples = defaultdict(list)
    deadline = time.monotonic() + seconds
    maximum_events = max(
        expected * 2, round(getattr(source, "fps", 30.0) * seconds * expected)
    )
    for _ in range(maximum_events):
        event = source.next_sample()
        if event is not None:
            logical = registry.observe(event)
            samples[logical.slot].append(logical.last_sample)
        if source.eof or time.monotonic() >= deadline:
            break
    return samples


def _manual_slot(role: str, available: tuple[str, ...], used: set[str]) -> str:
    while True:
        selected = input(f"Select {role.replace('_', ' ').upper()}:\n> ").strip()
        if selected not in available:
            print(f"Unknown slot. Choose one of: {', '.join(available)}")
        elif selected in used:
            print(f"{selected} is already assigned.")
        else:
            return selected


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Map FOHEART logical slots to body roles")
    parser.add_argument("--config", type=Path)
    parser.add_argument("--mode", choices=tuple(profile.value for profile in BodyProfile))
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--synthetic", action="store_true")
    source.add_argument("--replay", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--motion-assisted", action="store_true")
    parser.add_argument("--detect-seconds", type=float, default=10.0)
    parser.add_argument("--motion-window", type=float, default=2.0)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if min(args.detect_seconds, args.motion_window) <= 0:
        raise SystemExit("detection windows must be positive")
    if args.output.exists():
        print(f"Refusing to overwrite body mapping: {args.output}")
        return 2
    try:
        config = load_config(args.config)
        profile = args.mode or config.viewer.mode
        roles = roles_for_profile(profile)
        source = create_sensor_source(
            synthetic=args.synthetic,
            replay=args.replay,
            fps=config.viewer.fps,
            profile=profile,
        )
        registry = LogicalSlotRegistry()
        source.start()
        _collect_slots(
            source, registry, expected=len(roles), seconds=args.detect_seconds
        )
        available = tuple(registry.sensors)
        if len(available) < len(roles):
            print(f"Cannot map {profile} body: insufficient stable logical slots.")
            print(f"Required: {len(roles)}  Detected: {len(available)}")
            print("Detected: " + (", ".join(available) or "NONE"))
            return 2
        print("Detected slots:")
        for slot in available:
            print(slot)
        assignments = {}
        used = set()
        for role in roles:
            proposed = None
            if args.motion_assisted:
                print(f"\nMove only {role.replace('_', ' ').upper()} now.")
                input("Press ENTER to begin detection. ")
                proposed = propose_moving_slot(
                    _motion_window(source, registry, args.motion_window, len(roles))
                )
                if proposed is None:
                    print("No unique moving slot detected; select manually.")
                else:
                    slot, score = proposed
                    print(f"Strongest moving slot: {slot} (energy {score:.3f})")
                    if slot not in used and confirm_assignment(
                        input(f"Assign {role} -> {slot} ? [y/N] ")
                    ):
                        assignments[role] = slot
                        used.add(slot)
                        continue
            slot = _manual_slot(role, available, used)
            assignments[role] = slot
            used.add(slot)
        transport_keys = {
            slot: registry.sensors[slot].transport_key for slot in assignments.values()
        }
        mapping = BodySensorMap(
            assignments,
            slot_transport_keys=transport_keys,
            profile=profile,
        )
        mapping.require_complete()
        mapping.save(args.output)
    except KeyboardInterrupt:
        print("\nBody mapping cancelled; no file saved.")
        return 130
    except (OSError, RuntimeError, ValueError, SensorSourceError) as exc:
        print(f"Cannot create body mapping: {exc}")
        return 2
    finally:
        if "source" in locals():
            source.close()
    print(f"Saved CONFIGURED body mapping: {args.output}")
    print("Physical sensor identity remains UNKNOWN; every assignment was operator-confirmed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
