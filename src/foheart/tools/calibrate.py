"""Create a neutral-pose calibration file from explicit offline WXYZ values."""

from __future__ import annotations

import argparse
from pathlib import Path

import yaml

from foheart.mocap.calibration import CalibrationProfile
from foheart.mocap.sensor import Quaternion
from foheart.mocap.suit import UPPER_BODY_ROLES


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="OFFLINE FOHEART neutral calibration")
    parser.add_argument("--input", type=Path, help="YAML mapping body role -> WXYZ")
    parser.add_argument("--synthetic-upper-body", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    if bool(args.input) == bool(args.synthetic_upper_body):
        parser.error("choose exactly one of --input or --synthetic-upper-body")
    if args.output.exists():
        print(f"Refusing to overwrite calibration: {args.output}")
        return 2
    if args.synthetic_upper_body:
        values = {role: [1.0, 0.0, 0.0, 0.0] for role in UPPER_BODY_ROLES}
    else:
        try:
            values = yaml.safe_load(args.input.read_text(encoding="utf-8"))
        except (OSError, yaml.YAMLError) as exc:
            print(f"Could not read neutral orientations: {exc}")
            return 2
        if not isinstance(values, dict):
            print("Neutral orientation input must be a role -> WXYZ mapping")
            return 2
    try:
        profile = CalibrationProfile.capture(
            {str(role): Quaternion(tuple(map(float, quaternion)), "wxyz") for role, quaternion in values.items()}
        )
        profile.save(args.output)
    except (TypeError, ValueError) as exc:
        print(f"Invalid neutral calibration: {exc}")
        return 2
    print("OFFLINE CALIBRATION ONLY — no USB or robot operations")
    print(f"Saved CONFIGURED neutral profile: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
