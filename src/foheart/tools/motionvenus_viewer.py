"""Optional diagnostic viewer for the solved MotionVenus skeleton."""

from __future__ import annotations

import argparse
from pathlib import Path
import time

import numpy as np

from foheart.config import load_config
from foheart.mocap.frames import quaternion_to_matrix
from foheart.mocap.sensor import Quaternion
from foheart.motionvenus.protocol import MotionVenusProtocolError
from foheart.motionvenus.skeleton import BODY_BONE_EDGES, HumanSkeletonFrame
from foheart.motionvenus.transport import MotionVenusWatchdog
from foheart.tools._motionvenus import MotionVenusFrameSource


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="View MotionVenus's solved skeleton directly; no G1 action")
    parser.add_argument("--config", type=Path)
    parser.add_argument("--source", choices=("live", "replay", "synthetic"), default="live")
    parser.add_argument("--replay", type=Path)
    parser.add_argument("--bind")
    parser.add_argument("--port", type=int)
    parser.add_argument("--format", choices=("auto", "binary", "json"))
    parser.add_argument("--timeout", type=float)
    parser.add_argument("--duration", type=float)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--show-bone-axes", action="store_true")
    parser.add_argument("--show-bone-names", action="store_true")
    return parser


def _draw(ax, frame: HumanSkeletonFrame, *, fps: float, show_axes: bool, show_names: bool) -> None:
    ax.clear()
    for parent, child in BODY_BONE_EDGES:
        if parent not in frame.bones or child not in frame.bones:
            continue
        a, b = frame.bone(parent).position_global_m, frame.bone(child).position_global_m
        if a is not None and b is not None:
            points = np.asarray((a, b))
            ax.plot(points[:, 0], points[:, 1], points[:, 2], color="#34495e", linewidth=2.5)
    for bone in frame.bones.values():
        if bone.position_global_m is None:
            continue
        origin = np.asarray(bone.position_global_m)
        ax.scatter(*origin, color="#f39c12", s=20)
        if show_names:
            ax.text(*origin, bone.name, fontsize=6)
        if show_axes and bone.rotation_global_xyzw is not None:
            x, y, z, w = bone.rotation_global_xyzw
            rotation = quaternion_to_matrix(Quaternion((w, x, y, z), "wxyz"))
            for index, color in enumerate(("#e74c3c", "#2ecc71", "#3498db")):
                end = origin + rotation[:, index] * 0.08
                ax.plot((origin[0], end[0]), (origin[1], end[1]), (origin[2], end[2]), color=color)
    ax.set(xlim=(-1.1, 1.1), ylim=(-1.1, 1.1), zlim=(0, 2.2))
    ax.set_box_aspect((1, 1, 1))
    ax.set_xlabel("MotionVenus X / horizontal axis (m)")
    ax.set_ylabel("MotionVenus Y / depth axis (m)")
    ax.set_zlabel("MotionVenus Z / vertical axis (m)")
    ax.set_title(
        f"MotionVenus solved skeleton — {frame.status} — frame {frame.motionvenus_frame_number} — {fps:.1f} Hz",
        color="#c0392b" if frame.stale or not frame.valid else "#1f2937",
    )
    ax.text2D(
        0.01,
        0.98,
        f"Sender: {frame.sender[0]}:{frame.sender[1]}\nAvatar: {frame.avatar}\nBones: {len(frame.bones)}\nG1 action: NONE",
        transform=ax.transAxes,
        va="top",
        family="monospace",
    )


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.fps <= 0:
        raise SystemExit("--fps must be positive")
    config = load_config(args.config).motionvenus
    source = MotionVenusFrameSource(
        args.source,
        bind=args.bind or config.bind,
        port=args.port or config.port,
        packet_format=args.format or config.format,
        timeout_s=args.timeout or config.receive_timeout_ms / 1000,
        expected_body_bones=config.expected_body_bones,
        replay=args.replay,
        synthetic_fps=args.fps,
    )
    watchdog = MotionVenusWatchdog(stale_after_s=config.stale_after_ms / 1000)
    import matplotlib.pyplot as plt

    figure = plt.figure(figsize=(11, 8))
    axis = figure.add_subplot(111, projection="3d")
    deadline = None if args.duration is None else time.monotonic() + args.duration
    latest: HumanSkeletonFrame | None = None
    count, started = 0, time.monotonic()
    try:
        source.start()
        while plt.fignum_exists(figure.number) and (deadline is None or time.monotonic() < deadline):
            try:
                packet = source.receive_latest()
            except MotionVenusProtocolError as exc:
                watchdog.mark_error(exc, protocol_mismatch=exc.kind == "protocol_mismatch")
                packet = None
            if packet is not None:
                observation = watchdog.observe(packet)
                if observation.accepted:
                    latest = HumanSkeletonFrame.from_motionvenus(packet, status="LIVE")
                    count += 1
            status = watchdog.status()
            if latest is not None:
                displayed = latest
                if status != "LIVE":
                    displayed = HumanSkeletonFrame(
                        latest.timestamp_ns, latest.motionvenus_frame_number, latest.suit_number,
                        latest.avatar, latest.bones, False, status == "STALE", status,
                        watchdog.diagnostics().last_error or "input is not live",
                        latest.sender, latest.source_format, latest.source_coordinate,
                    )
                _draw(axis, displayed, fps=count / max(time.monotonic() - started, 1e-9), show_axes=args.show_bone_axes, show_names=args.show_bone_names)
            plt.pause(1 / args.fps)
            if source.eof:
                break
    except KeyboardInterrupt:
        pass
    finally:
        source.close()
        plt.close(figure)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
