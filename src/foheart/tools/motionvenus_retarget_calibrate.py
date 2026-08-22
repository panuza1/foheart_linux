"""Guided Linux human-to-G1 neutral capture (not FOHEART sensor calibration)."""

from __future__ import annotations

import argparse
from dataclasses import replace
from pathlib import Path

from foheart.config import load_config
from foheart.motionvenus.protocol import MotionVenusProtocolError
from foheart.motionvenus.retarget import RetargetProfile
from foheart.motionvenus.skeleton import HumanSkeletonFrame
from foheart.motionvenus.transport import MotionVenusWatchdog
from foheart.tools._motionvenus import MotionVenusFrameSource


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Capture a MotionVenus-to-G1 neutral; no robot connection")
    parser.add_argument("--config", type=Path)
    parser.add_argument("--source", choices=("live", "replay", "synthetic"), default="live")
    parser.add_argument("--replay", type=Path)
    parser.add_argument("--bind")
    parser.add_argument("--port", type=int)
    parser.add_argument("--format", choices=("auto", "binary", "json"))
    parser.add_argument("--samples", type=int, default=120)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--yes", action="store_true", help="skip the pose confirmation prompt")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.samples < 2:
        raise SystemExit("--samples must be at least 2")
    runtime = load_config(args.config)
    config = runtime.motionvenus
    profile_path = args.output
    base = RetargetProfile.load(config.retarget_profile)
    source = MotionVenusFrameSource(
        args.source,
        bind=args.bind or config.bind,
        port=args.port or config.port,
        packet_format=args.format or config.format,
        timeout_s=config.receive_timeout_ms / 1000,
        expected_body_bones=config.expected_body_bones,
        replay=args.replay,
        synthetic_poses=("neutral",),
    )
    watchdog = MotionVenusWatchdog(stale_after_s=config.stale_after_ms / 1000)
    print(
        "Linux retarget neutral (separate from MotionVenus suit calibration)\n"
        "Stand upright, face MotionVenus forward, keep both arms relaxed at your sides, and remain still.\n"
        "No G1 connection or command is created."
    )
    if args.source == "live" and not args.yes:
        input("Press ENTER when the pose is steady: ")
    samples: list[HumanSkeletonFrame] = []
    try:
        source.start()
        while len(samples) < args.samples:
            try:
                packet = source.receive()
            except MotionVenusProtocolError as exc:
                watchdog.mark_error(exc, protocol_mismatch=exc.kind == "protocol_mismatch")
                continue
            if packet is None:
                if source.eof:
                    break
                continue
            observation = watchdog.observe(packet)
            if observation.accepted:
                samples.append(HumanSkeletonFrame.from_motionvenus(packet, status="LIVE"))
    finally:
        source.close()
    if len(samples) < args.samples:
        raise SystemExit(f"only {len(samples)} valid frames were available; requested {args.samples}")
    captured = RetargetProfile.capture(
        samples,
        motionvenus_to_project=base.motionvenus_to_project,
        project_to_g1=base.project_to_g1,
        position_scale=base.position_scale,
        max_robot_reach_m=base.max_robot_reach_m,
        workspace_radius_m=base.workspace_radius_m,
    )
    captured = replace(
        captured,
        position_alpha=base.position_alpha,
        orientation_alpha=base.orientation_alpha,
        max_translation_rate_m_s=base.max_translation_rate_m_s,
        max_angular_rate_deg_s=base.max_angular_rate_deg_s,
    )
    captured.save(profile_path)
    print(f"Saved {len(samples)}-frame quaternion-safe neutral to {profile_path}")
    print("Status remains SOFTWARE_CONFIGURED until Windows/Linux skeletons are visually compared.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
