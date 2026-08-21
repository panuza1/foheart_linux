"""Guided upper/full neutral calibration over the shared SensorSource path."""

from __future__ import annotations

import argparse
from collections import defaultdict
from pathlib import Path
import time

from foheart.config import load_config
from foheart.mocap.calibration import CalibrationObservation, CalibrationProfile
from foheart.mocap.frames import BasisTransform
from foheart.mocap.stream import (
    LogicalSlotRegistry,
    SensorSourceError,
    create_sensor_source,
)
from foheart.mocap.suit import BodyProfile, BodySensorMap, roles_for_profile
from foheart.mocap.synthetic import SYNTHETIC_BODY_MAP, SYNTHETIC_FULL_BODY_MAP


NEUTRAL_PROMPT = """NEUTRAL CALIBRATION

Stand upright.
Face forward.
Torso straight.
Arms relaxed straight down at the sides, palms facing inward.
Hold completely still.
"""

FULL_NEUTRAL_PROMPT = """NEUTRAL CALIBRATION — FULL BODY

Stand upright and face forward.
Keep head, torso, and pelvis neutral.
Relax shoulders; hold arms straight down, palms inward.
Keep legs straight and feet parallel, pointing forward.
Hold completely still.
"""


def _mapping(args, config, profile: str) -> BodySensorMap:
    if args.body_mapping:
        mapping = BodySensorMap.load(args.body_mapping)
        mapping.require_profile(profile)
        return mapping
    if args.synthetic:
        return (
            SYNTHETIC_FULL_BODY_MAP
            if profile == BodyProfile.FULL.value
            else SYNTHETIC_BODY_MAP
        )
    mapping = BodySensorMap(config.body_mapping.role_to_slot, profile=profile)
    mapping.require_complete()
    return mapping


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="FOHEART upper/full neutral calibration")
    parser.add_argument("--config", type=Path)
    parser.add_argument("--mode", choices=tuple(profile.value for profile in BodyProfile))
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--synthetic", action="store_true")
    source.add_argument("--replay", type=Path)
    parser.add_argument("--body-mapping", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=float)
    parser.add_argument("--samples-per-role", type=int)
    parser.add_argument("--maximum-angle-deg", type=float)
    parser.add_argument("--maximum-gyro", type=float)
    parser.add_argument("--no-prompt", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.output.exists():
        print(f"Refusing to overwrite calibration: {args.output}")
        return 2
    try:
        config = load_config(args.config)
        selected_profile = args.mode or config.viewer.mode
        roles = roles_for_profile(selected_profile)
        mapping = _mapping(args, config, selected_profile)
        mapping.require_complete()
        duration = args.duration if args.duration is not None else config.calibration.duration_s
        minimum_samples = (
            args.samples_per_role
            if args.samples_per_role is not None
            else config.calibration.minimum_samples
        )
        maximum_angle = (
            args.maximum_angle_deg
            if args.maximum_angle_deg is not None
            else config.calibration.maximum_angular_deviation_deg
        )
        maximum_gyro = (
            args.maximum_gyro
            if args.maximum_gyro is not None
            else config.calibration.maximum_gyro_magnitude
        )
        if duration <= 0 or minimum_samples < 2 or min(maximum_angle, maximum_gyro) <= 0:
            raise ValueError("calibration duration, sample count, and limits must be positive")
        basis = BasisTransform(
            config.frames.sensor_to_body_matrix,
            "foheart_sensor_unknown",
            "configured_body_sensor",
            config.frames.status,
        )
        source = create_sensor_source(
            synthetic=args.synthetic,
            replay=args.replay,
            fps=config.viewer.fps,
            profile=selected_profile,
        )
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"Cannot prepare neutral calibration: {exc}")
        return 2

    print(
        FULL_NEUTRAL_PROMPT
        if selected_profile == BodyProfile.FULL.value
        else NEUTRAL_PROMPT
    )
    if not args.no_prompt and not args.synthetic:
        try:
            input("Press ENTER when ready. ")
        except KeyboardInterrupt:
            print("\nCalibration cancelled; no file saved.")
            return 130

    observations = defaultdict(list)
    slot_to_role = {slot: role for role, slot in mapping.role_to_slot.items()}
    registry = LogicalSlotRegistry(mapping.registry_bindings)
    deadline = time.monotonic() + duration
    maximum_events = max(minimum_samples * len(roles) * 2, 100)
    try:
        source.start()
        for _ in range(maximum_events):
            event = source.next_sample()
            if event is not None:
                logical = registry.observe(event)
                role = slot_to_role.get(logical.slot)
                sample = logical.last_sample
                if role and sample.quaternion is not None:
                    observations[role].append(
                        CalibrationObservation(
                            logical.last_seen_ns,
                            basis.orientation(sample.quaternion),
                            None if sample.gyro is None else basis.vector(sample.gyro),
                        )
                    )
                if all(
                    len(observations[role]) >= minimum_samples
                    for role in roles
                ):
                    break
            if source.eof or time.monotonic() >= deadline:
                break
        counts = {role: len(observations[role]) for role in roles}
        if any(count < minimum_samples for count in counts.values()):
            missing = ", ".join(
                f"{role} {count}/{minimum_samples}"
                for role, count in counts.items()
                if count < minimum_samples
            )
            raise ValueError(f"insufficient calibration samples: {missing}")
        profile = CalibrationProfile.capture_window(
            observations,
            minimum_samples=minimum_samples,
            maximum_angular_deviation_degrees=maximum_angle,
            maximum_gyro_magnitude=maximum_gyro,
        )
        profile.require_roles(roles)
        profile.save(args.output)
    except KeyboardInterrupt:
        print("\nCalibration cancelled; no file saved.")
        return 130
    except (OSError, RuntimeError, ValueError, SensorSourceError) as exc:
        print(f"Neutral calibration failed: {exc}")
        return 2
    finally:
        source.close()

    for role in roles:
        quality = profile.quality[role]
        print(
            f"{role}: samples={quality.sample_count} "
            f"norm={quality.quaternion_norm_min:.6f}..{quality.quaternion_norm_max:.6f} "
            f"spread={quality.orientation_spread_degrees:.3f} deg "
            f"max={quality.maximum_angular_deviation_degrees:.3f} deg "
            f"gyro_max={quality.gyro_magnitude_max:.3f}"
        )
    print(f"Saved SOFTWARE_TESTED neutral calibration: {args.output}")
    print("LIVE_VALIDATED = false")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
