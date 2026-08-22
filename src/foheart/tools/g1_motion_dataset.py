"""Create a TWIST2 MotionLib dataset YAML from recorded G1 motion pickles."""

from __future__ import annotations

import argparse
from pathlib import Path

from foheart.integrations.twist2 import create_dataset


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Create a TWIST2-compatible motion dataset YAML")
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--weight", type=float, action="append")
    parser.add_argument("motions", type=Path, nargs="+")
    args = parser.parse_args(argv)
    if args.weight is not None and len(args.weight) != len(args.motions):
        parser.error("provide exactly one --weight per motion, or omit all weights")
    try:
        create_dataset(args.output, args.root, args.motions, weights=args.weight)
    except (FileExistsError, FileNotFoundError, ValueError) as exc:
        print(f"Dataset not created: {exc}")
        return 2
    print(f"Dataset: {args.output} ({len(args.motions)} motion(s))")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
