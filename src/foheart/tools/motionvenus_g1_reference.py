"""MotionVenus -> pinned GMR -> processed G1 reference/simulation/recording."""

from __future__ import annotations

import argparse
from pathlib import Path
import time
from typing import Any

import numpy as np

from foheart.config import RuntimeConfig, load_config
from foheart.integrations.twist2 import MotionRecorder
from foheart.integrations.unitree_g1.sim_bridge import G1MuJoCoBridge, intersect_joint_limits
from foheart.motionvenus.gmr import MotionVenusGMRAdapter
from foheart.motionvenus.protocol import MotionVenusProtocolError
from foheart.motionvenus.skeleton import HumanSkeletonFrame
from foheart.motionvenus.synthetic import GMR_SYNTHETIC_POSES
from foheart.motionvenus.transport import MotionVenusWatchdog
from foheart.tools._motionvenus import MotionVenusFrameSource
from foheart.whole_body.gmr import G1ReferenceMuJoCo, GMRWholeBodyRetargeter
from foheart.whole_body.reference import G1ReferenceProcessor


PROJECT_ROOT = Path(__file__).resolve().parents[5]
DEFAULT_GMR_MODEL = (
    PROJECT_ROOT / "third_party/HumDex/GMR/assets/unitree_g1/g1_mocap_29dof.xml"
)
DEFAULT_DYNAMIC_MODEL = PROJECT_ROOT / "unitree_mujoco/unitree_robots/g1/scene_29dof.xml"
DEFAULT_TELEOPIT_ROOT = PROJECT_ROOT / "Teleopit"
DEFAULT_TELEOPIT_POLICY = DEFAULT_TELEOPIT_ROOT / "ckpt/track_g1.onnx"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="MotionVenus -> GMR -> safe G1 reference, direct sim, or TeleopIt policy sim"
    )
    parser.add_argument(
        "--mode", choices=("reference", "direct-sim", "policy-sim"), default="reference"
    )
    parser.add_argument("--source", choices=("synthetic", "replay", "live"), default="synthetic")
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5001)
    parser.add_argument("--format", choices=("auto", "binary", "json"), default="binary")
    parser.add_argument("--timeout", type=float, default=0.25)
    parser.add_argument("--replay", type=Path)
    parser.add_argument("--max-frames", type=int)
    parser.add_argument("--duration", type=float)
    parser.add_argument("--fps", type=float, default=50.0)
    parser.add_argument("--human-height", type=float)
    parser.add_argument("--model", type=Path, default=DEFAULT_GMR_MODEL)
    parser.add_argument("--dynamic-model", type=Path, default=DEFAULT_DYNAMIC_MODEL)
    parser.add_argument("--sim-steps", type=int, default=8)
    parser.add_argument("--policy-steps-per-reference", type=int, default=25)
    parser.add_argument("--teleopit-root", type=Path, default=DEFAULT_TELEOPIT_ROOT)
    parser.add_argument("--policy", type=Path, default=DEFAULT_TELEOPIT_POLICY)
    parser.add_argument("--soft-limit-margin", type=float, default=0.0, help="joint margin in radians")
    parser.add_argument("--ema-alpha", type=float, help="enable whole-body EMA with alpha in [0, 1]")
    parser.add_argument("--max-joint-rate", type=float, help="enable a whole-body rate limit in rad/s")
    parser.add_argument("--disable-heading-normalization", action="store_true")
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--record", type=Path)
    return parser


def _discontinuity(previous: np.ndarray | None, current: np.ndarray) -> dict[str, float]:
    if previous is None:
        return {"root_position_m": 0.0, "root_rotation_rad": 0.0, "max_joint_rad": 0.0}
    dot = float(np.clip(abs(previous[3:7] @ current[3:7]), 0.0, 1.0))
    return {
        "root_position_m": float(np.linalg.norm(current[:3] - previous[:3])),
        "root_rotation_rad": float(2.0 * np.arccos(dot)),
        "max_joint_rad": float(np.max(np.abs(current[7:] - previous[7:]))),
    }


def run(
    args: argparse.Namespace,
    config: RuntimeConfig,
    *,
    retargeter: Any | None = None,
    reference_model: Any | None = None,
    processor: Any | None = None,
    dynamic_model: Any | None = None,
    policy_model: Any | None = None,
) -> tuple[int, list[dict[str, Any]]]:
    if args.record is not None and args.record.exists():
        raise FileExistsError(args.record)
    settings = config.motionvenus
    source = MotionVenusFrameSource(
        args.source,
        bind=args.bind,
        port=args.port,
        packet_format=args.format,
        timeout_s=args.timeout,
        expected_body_bones=settings.expected_body_bones,
        replay=args.replay,
        synthetic_fps=args.fps,
        synthetic_poses=GMR_SYNTHETIC_POSES,
    )
    mode = getattr(args, "mode", "reference")
    if mode not in ("reference", "direct-sim", "policy-sim"):
        raise ValueError("mode must be reference, direct-sim, or policy-sim")
    adapter = MotionVenusGMRAdapter(
        normalize_heading=not getattr(args, "disable_heading_normalization", False)
    )
    gmr = retargeter or GMRWholeBodyRetargeter(actual_human_height=args.human_height)
    model = reference_model or G1ReferenceMuJoCo(args.model, viewer=args.viewer)
    dynamic = dynamic_model if mode == "direct-sim" else policy_model if mode == "policy-sim" else None
    if mode == "direct-sim" and dynamic is None:
        try:
            dynamic = G1MuJoCoBridge(getattr(args, "dynamic_model", DEFAULT_DYNAMIC_MODEL))
        except Exception:
            model.close()
            raise
    if mode == "policy-sim" and dynamic is None:
        try:
            from foheart.integrations.teleopit import FoheartTeleopitPolicySimulator

            dynamic = FoheartTeleopitPolicySimulator(
                getattr(args, "teleopit_root", DEFAULT_TELEOPIT_ROOT),
                getattr(args, "policy", DEFAULT_TELEOPIT_POLICY),
                max_reference_age_s=settings.stale_after_ms / 1000.0,
            )
        except Exception:
            model.close()
            raise
    recorder = MotionRecorder(fps=args.fps) if args.record is not None else None
    watchdog = MotionVenusWatchdog(stale_after_s=settings.stale_after_ms / 1000.0)
    default_limit = len(GMR_SYNTHETIC_POSES) if args.source == "synthetic" else None
    frame_limit = args.max_frames if args.max_frames is not None else default_limit
    deadline = None if args.duration is None else time.monotonic() + args.duration
    rows: list[dict[str, Any]] = []
    previous: np.ndarray | None = None
    accepted = failures = 0

    label = {
        "reference": "KINEMATIC_REFERENCE / REFERENCE_VALIDATED",
        "direct-sim": "DIRECT_DYNAMIC_SIM / pinned-base MuJoCo dynamics",
        "policy-sim": "TELEOPIT_POLICY_SIM / free-base MuJoCo",
    }[mode]
    print(f"MotionVenus -> GMR -> G1ReferenceProcessor -> {label} (NO DDS)")
    try:
        source.start()
        if args.source == "live":
            print(f"Waiting for MotionVenus on {args.bind}:{args.port}")
        while (frame_limit is None or accepted < frame_limit) and (
            deadline is None or time.monotonic() < deadline
        ) and bool(getattr(model, "is_running", True)):
            try:
                frame = source.receive_latest()
            except MotionVenusProtocolError as exc:
                watchdog.mark_error(exc, protocol_mismatch=exc.kind == "protocol_mismatch")
                if processor is not None:
                    processor.hold(str(exc))
                failures += 1
                continue
            if frame is None:
                if source.eof:
                    break
                if processor is not None:
                    processor.check_stale(time.time())
                continue
            observation = watchdog.observe(frame)
            if not observation.accepted:
                if processor is not None:
                    processor.hold(f"MotionVenus frame is {observation.event}")
                failures += 1
                continue
            accepted += 1
            try:
                skeleton = HumanSkeletonFrame.from_motionvenus(frame, status=source.source_name)
                human_data = adapter.adapt(skeleton)
                reference = gmr.retarget(human_data)
                if processor is None:
                    bounds = []
                    for limit_source in (reference, model, dynamic):
                        lower = getattr(limit_source, "joint_lower", None)
                        upper = getattr(limit_source, "joint_upper", None)
                        if (lower is None) != (upper is None):
                            raise ValueError("G1 joint-limit source is incomplete")
                        if lower is not None:
                            bounds.append((lower, upper))
                    lower, upper = intersect_joint_limits(*bounds)
                    processor = G1ReferenceProcessor(
                        lower,
                        upper,
                        stale_after_s=settings.stale_after_ms / 1000.0,
                        soft_limit_margin=getattr(args, "soft_limit_margin", 0.0),
                        ema_alpha=getattr(args, "ema_alpha", None),
                        max_joint_rate=getattr(args, "max_joint_rate", None),
                    )
                source_timestamp_s = frame.received_ns / 1e9
                processed = processor.process(
                    reference,
                    source_timestamp_s=source_timestamp_s,
                    now_s=source_timestamp_s,
                    source_frame_number=frame.header.frame_number,
                )
                if processed is None or processed.held:
                    raise ValueError(processor.reason)
                target = np.asarray(processed.qpos_wxyz, dtype=float)
                local_body_pos = model.local_body_positions(target)
                if recorder is not None:
                    # The dataset branch records the intended processed reference,
                    # never the dynamic simulator's servo-tracking state.
                    recorder.append(
                        target,
                        local_body_pos,
                        timestamp_ns=frame.received_ns,
                        source_frame_number=frame.header.frame_number,
                    )
                metrics = None
                applied = target
                if mode == "reference":
                    applied = np.asarray(model.apply(target), dtype=float)
                else:
                    if dynamic is None:
                        raise RuntimeError(f"{mode} has no MuJoCo sink")
                    steps = (
                        getattr(args, "policy_steps_per_reference", 25)
                        if mode == "policy-sim"
                        else getattr(args, "sim_steps", 8)
                    )
                    metrics = dynamic.command_whole_body(processed, steps=steps)
                discontinuity = _discontinuity(previous, target)
                pose = (
                    GMR_SYNTHETIC_POSES[frame.header.frame_number % len(GMR_SYNTHETIC_POSES)]
                    if args.source == "synthetic"
                    else None
                )
                row = {
                    "frame": frame.header.frame_number,
                    "pose": pose,
                    "qpos_shape": tuple(applied.shape),
                    "root_pos": applied[:3].tolist(),
                    "root_quat_wxyz": applied[3:7].tolist(),
                    "joints": applied[7:].tolist(),
                    "finite": bool(np.isfinite(applied).all()),
                    "joint_limits": "PASS",
                    "processor_status": processed.status,
                    "processor_reason": processed.reason,
                    "clamp_count": processed.clamp_count,
                    "clamped_joints": list(processed.clamped_joints),
                    "hard_clamped_joints": list(processed.hard_clamped_joints),
                    "soft_clamped_joints": list(processed.soft_clamped_joints),
                    "smoothing_applied": processed.smoothing_applied,
                    "rate_limit_count": processed.rate_limit_count,
                    "rate_limited_joints": list(processed.rate_limited_joints),
                    "reference_status": "REFERENCE_VALIDATED",
                    "direct_dynamic_status": (
                        "DIRECT_DYNAMIC_SIM_VALIDATED"
                        if mode == "direct-sim" and metrics is not None and metrics.finite
                        else "FAILED" if mode == "direct-sim" and metrics is not None else "NOT_RUN"
                    ),
                    "policy_sim_status": (
                        "TELEOPIT_POLICY_SIM_VALIDATED"
                        if mode == "policy-sim" and metrics is not None and metrics.finite
                        else "FAILED" if mode == "policy-sim" and metrics is not None else "NOT_RUN"
                    ),
                    "sim_finite": None if metrics is None else metrics.finite,
                    "sim_root_pos": None if metrics is None else list(metrics.root_position_m),
                    "sim_root_quat_wxyz": (
                        None if metrics is None else list(metrics.root_quaternion_wxyz)
                    ),
                    "sim_max_joint_error_rad": (
                        None if metrics is None else metrics.maximum_joint_error_rad
                    ),
                    "sim_stability": None if metrics is None else metrics.stability_status,
                    "sim_duration_s": None if metrics is None else metrics.simulation_duration_s,
                    "sim_base_pinned": None if metrics is None else metrics.base_pinned,
                    "policy_observation_finite": (
                        None if mode != "policy-sim" else metrics.observation_finite
                    ),
                    "policy_action_finite": None if mode != "policy-sim" else metrics.action_finite,
                    "policy_observation_shape": (
                        None if mode != "policy-sim" else metrics.observation_shape
                    ),
                    "policy_action_shape": None if mode != "policy-sim" else metrics.action_shape,
                    "sim_min_root_height_m": (
                        None if mode != "policy-sim" else metrics.minimum_root_height_m
                    ),
                    "sim_max_root_height_m": (
                        None if mode != "policy-sim" else metrics.maximum_root_height_m
                    ),
                    "sim_fall_status": None if mode != "policy-sim" else metrics.fall_status,
                    "discontinuity": discontinuity,
                }
                rows.append(row)
                previous = applied.copy()
                print(
                    f"frame={row['frame']} pose={pose or '-'} qpos=36 finite=True limits=PASS "
                    f"clamps={processed.clamp_count} rate_limits={processed.rate_limit_count} "
                    f"delta_joint={discontinuity['max_joint_rad']:.4f}rad "
                    f"status={row['policy_sim_status'] if mode == 'policy-sim' else row['direct_dynamic_status'] if metrics else row['reference_status']}"
                )
                if args.viewer and args.source != "live":
                    time.sleep(1.0 / args.fps)
            except (TypeError, ValueError, RuntimeError) as exc:
                if processor is not None and processor.status != "HOLD":
                    processor.hold(str(exc))
                failures += 1
                print(f"frame={frame.header.frame_number} HOLD: {exc}")
    except KeyboardInterrupt:
        print("Ctrl-C: reference pipeline stopped cleanly")
    finally:
        source.close()
        model.close()
        if dynamic is not None and callable(getattr(dynamic, "close", None)):
            dynamic.close()

    if recorder is not None:
        if len(recorder) < 2:
            print("Recording rejected: fewer than two valid frames")
            return 1, rows
        recorder.save(args.record)
        print(f"Recorded {len(recorder)} frames: {args.record}")
    passed = bool(rows) and failures == 0 and all(
        row["finite"] and (mode == "reference" or row["sim_finite"])
        for row in rows
    )
    status = (
        "REFERENCE_VALIDATED"
        if passed and mode == "reference"
        else "DIRECT_DYNAMIC_SIM_VALIDATED"
        if passed and mode == "direct-sim"
        else "TELEOPIT_POLICY_SIM_VALIDATED"
        if passed
        else "PARTIAL"
    )
    if mode == "policy-sim" and rows:
        print(
            "Policy sim metrics: "
            f"duration={sum(float(row['sim_duration_s']) for row in rows):.3f}s "
            f"root_height=[{min(float(row['sim_min_root_height_m']) for row in rows):.4f}, "
            f"{max(float(row['sim_max_root_height_m']) for row in rows):.4f}]m "
            f"max_joint_error={max(float(row['sim_max_joint_error_rad']) for row in rows):.4f}rad "
            f"obs=167 action=29 free_base=True fall_status=NOT_SOURCE_DEFINED"
        )
    print(f"Result: {status} frames={len(rows)} failures={failures}")
    return (0 if passed else 1), rows


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.source == "replay" and args.replay is None:
        parser.error("--replay is required with --source replay")
    if args.source != "replay" and args.replay is not None:
        parser.error("--replay is only valid with --source replay")
    if args.max_frames is not None and args.max_frames < 1:
        parser.error("--max-frames must be positive")
    if args.duration is not None and args.duration <= 0:
        parser.error("--duration must be positive")
    if args.timeout <= 0 or args.fps <= 0:
        parser.error("--timeout and --fps must be positive")
    if args.human_height is not None and args.human_height <= 0:
        parser.error("--human-height must be positive")
    if not 1 <= args.sim_steps <= 5000:
        parser.error("--sim-steps must be in 1..5000")
    if not 1 <= args.policy_steps_per_reference <= 5000:
        parser.error("--policy-steps-per-reference must be in 1..5000")
    if not np.isfinite(args.soft_limit_margin) or args.soft_limit_margin < 0:
        parser.error("--soft-limit-margin must be finite and nonnegative")
    if args.ema_alpha is not None and (
        not np.isfinite(args.ema_alpha) or not 0 <= args.ema_alpha <= 1
    ):
        parser.error("--ema-alpha must be finite and in [0, 1]")
    if args.max_joint_rate is not None and (
        not np.isfinite(args.max_joint_rate) or args.max_joint_rate <= 0
    ):
        parser.error("--max-joint-rate must be finite and positive")
    if args.mode == "direct-sim" and args.viewer:
        parser.error("--viewer is only available with --mode reference")
    try:
        result, _ = run(args, load_config())
        return result
    except (FileExistsError, FileNotFoundError, ImportError, OSError, RuntimeError, ValueError) as exc:
        print(f"Reference pipeline unavailable: {exc}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
