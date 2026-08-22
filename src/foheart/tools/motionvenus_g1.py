"""MotionVenus -> shared retarget/IK -> explicit SIMULATION or guarded REAL branch."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

from foheart.config import RuntimeConfig, load_config
from foheart.integrations.unitree_g1.adapter import ExistingG1IK, SafeG1IK
from foheart.integrations.unitree_g1.sim_bridge import G1MuJoCoBridge
from foheart.integrations.unitree_g1.sinks import (
    REAL_COMMAND_BACKEND,
    RealBackendBlocked,
    RealG1Sink,
    SimG1Sink,
)
from foheart.motionvenus.protocol import MotionVenusProtocolError
from foheart.motionvenus.retarget import MotionVenusG1Retargeter, RetargetProfile
from foheart.motionvenus.skeleton import HumanSkeletonFrame
from foheart.motionvenus.synthetic import SYNTHETIC_POSES
from foheart.motionvenus.transport import MotionVenusWatchdog
from foheart.tools._motionvenus import MotionVenusFrameSource


PROJECT_ROOT = Path(__file__).resolve().parents[5]
DEFAULT_XR_ROOT = PROJECT_ROOT / "xr_teleoperate"
DEFAULT_MUJOCO_MODEL = PROJECT_ROOT / "unitree_mujoco/unitree_robots/g1/scene_29dof.xml"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="MotionVenus solved skeleton -> Unitree G1")
    parser.add_argument("--config", type=Path)
    parser.add_argument("--mode", choices=("sim", "real"), help="must be explicit for real; default is always sim")
    parser.add_argument("--source", choices=("live", "replay", "synthetic"), default="synthetic")
    parser.add_argument("--replay", type=Path)
    parser.add_argument("--bind")
    parser.add_argument("--port", type=int)
    parser.add_argument("--format", choices=("auto", "binary", "json"))
    parser.add_argument("--timeout", type=float)
    parser.add_argument("--duration", type=float)
    parser.add_argument("--max-frames", type=int)
    parser.add_argument("--retarget", type=Path)
    parser.add_argument("--xr-root", type=Path)
    parser.add_argument("--mujoco-model", type=Path)
    parser.add_argument("--sim-steps", type=int, default=8)
    parser.add_argument("--record", type=Path, help="optional JSONL processing metrics")
    parser.add_argument("--real-monitor-only", action="store_true", help="run through IK without creating any robot API")
    parser.add_argument("--i-understand-this-controls-real-hardware", action="store_true")
    return parser


def _paths(config: RuntimeConfig, args) -> tuple[Path, Path]:
    xr = args.xr_root or (Path(config.g1.xr_root) if config.g1.xr_root else DEFAULT_XR_ROOT)
    model = args.mujoco_model or (
        Path(config.g1.mujoco_model) if config.g1.mujoco_model else DEFAULT_MUJOCO_MODEL
    )
    return xr, model


def _pipeline(config: RuntimeConfig, profile_path: Path, xr_root: Path):
    solver = ExistingG1IK(xr_root)
    profile = RetargetProfile.load(profile_path)
    retargeter = MotionVenusG1Retargeter(
        profile,
        robot_neutral_left=solver.neutral_left,
        robot_neutral_right=solver.neutral_right,
        robot_left_shoulder=solver.left_shoulder,
        robot_right_shoulder=solver.right_shoulder,
    )
    safe_ik = SafeG1IK(
        solver,
        max_joint_delta_rad=config.g1.max_joint_delta_rad,
        workspace_radius_m=profile.workspace_radius_m,
    )
    return profile, retargeter, safe_ik


def _row(
    frame, human, targets, ik, sim, timings, timestamps, source_name,
    watchdog, transport_stats, accepted_status,
):
    diagnostics = watchdog.diagnostics()
    backlog_drops = 0 if transport_stats is None else transport_stats.backlog_drops
    return {
        "source": source_name,
        "frame": frame.header.frame_number,
        "sender": list(frame.sender),
        "source_status": accepted_status,
        "source_status_after_sink": watchdog.status(),
        "network": {
            "estimated_lost_frames": diagnostics.estimated_lost_frames,
            "estimated_network_loss": max(0, diagnostics.estimated_lost_frames - backlog_drops),
            "duplicate_frames": diagnostics.duplicate_frames,
            "out_of_order_frames": diagnostics.out_of_order_frames,
            "sender_changes": diagnostics.sender_changes,
            "received_packets": None if transport_stats is None else transport_stats.packets,
            "packet_rate_hz": None if transport_stats is None else transport_stats.rate_hz,
            "backlog_drops": None if transport_stats is None else backlog_drops,
        },
        "human_valid": human.valid,
        "left_wrist_xyz_m": targets.left[:3, 3].tolist(),
        "right_wrist_xyz_m": targets.right[:3, 3].tolist(),
        "workspace_clamped": list(targets.clamped),
        "ik_valid": ik.valid,
        "ik_reason": ik.reason,
        "ik_rate_limited": ik.rate_limited,
        "ik_position_error_m": ik.position_error_m,
        "ik_rotation_error_deg": ik.rotation_error_deg,
        "joint_targets": ik.joint_positions.tolist(),
        "sim_finite": None if sim is None else sim.finite,
        "sim_max_arm_error_rad": None if sim is None else sim.maximum_arm_error_rad,
        "timing_ms": timings,
        "timestamps_ns": timestamps,
    }


def run(args, config: RuntimeConfig) -> tuple[int, list[dict[str, object]]]:
    mode = args.mode or "sim"  # A config file alone can never opt into real hardware.
    settings = config.motionvenus
    if mode == "real" and not args.i_understand_this_controls_real_hardware:
        raise ValueError("real mode requires --i-understand-this-controls-real-hardware")
    if mode == "real" and not args.real_monitor_only:
        confirmation = input("WAITING_FOR_OPERATOR — type ENABLE to continue: ").strip()
        if confirmation != "ENABLE":
            raise ValueError("operator did not type ENABLE; no robot interface was created")
        # The production factory fails before importing or constructing a DDS controller.
        sink = RealG1Sink()
        try:
            sink.backend_factory()
        except RealBackendBlocked as exc:
            print(f"REAL_COMMAND_BACKEND={REAL_COMMAND_BACKEND}\n{exc}")
            return 2, []

    profile_path = args.retarget or Path(settings.retarget_profile)
    xr_root, model_path = _paths(config, args)
    profile, retargeter, safe_ik = _pipeline(config, profile_path, xr_root)
    sim_sink = None
    if mode == "sim":
        sim_sink = SimG1Sink(G1MuJoCoBridge(model_path), steps_per_update=args.sim_steps)
    source = MotionVenusFrameSource(
        args.source,
        bind=args.bind or settings.bind,
        port=args.port or settings.port,
        packet_format=args.format or settings.format,
        timeout_s=args.timeout or settings.receive_timeout_ms / 1000,
        expected_body_bones=settings.expected_body_bones,
        replay=args.replay,
    )
    watchdog = MotionVenusWatchdog(stale_after_s=settings.stale_after_ms / 1000)
    deadline = None if args.duration is None else time.monotonic() + args.duration
    default_limit = len(SYNTHETIC_POSES) if args.source == "synthetic" and args.duration is None else None
    frame_limit = args.max_frames if args.max_frames is not None else default_limit
    rows: list[dict[str, object]] = []
    record = None
    if args.record:
        args.record.parent.mkdir(parents=True, exist_ok=True)
        record = args.record.open("x", encoding="utf-8")
    print(
        f"MotionVenus -> G1 — {'SIMULATION_ONLY / MuJoCo / NO DDS' if mode == 'sim' else 'REAL MONITOR-ONLY / NO ROBOT API'}\n"
        f"Source: {args.source.upper()}  Profile: {profile_path} ({profile.status})"
    )
    last_print = 0.0
    run_started = time.monotonic()
    try:
        source.start()
        while (deadline is None or time.monotonic() < deadline) and (
            frame_limit is None or len(rows) < frame_limit
        ):
            try:
                frame = source.receive_latest()
            except MotionVenusProtocolError as exc:
                watchdog.mark_error(exc, protocol_mismatch=exc.kind == "protocol_mismatch")
                continue
            if frame is None:
                if source.eof:
                    break
                now = time.monotonic()
                if args.source == "live" and now - last_print >= 1.0:
                    stats = source.stats
                    print(
                        f"Source {watchdog.status()}: packets={stats.packets if stats else 0} "
                        f"network={stats.rate_hz if stats else 0.0:.1f}Hz "
                        f"backlog_drops={stats.backlog_drops if stats else 0} fresh_target=HOLD"
                    )
                    last_print = now
                continue
            observation = watchdog.observe(frame)
            if not observation.accepted:
                continue
            parse_done = time.perf_counter_ns()
            human = HumanSkeletonFrame.from_motionvenus(frame, status="LIVE")
            retarget_started = time.perf_counter_ns()
            try:
                targets = retargeter.retarget(human)
            except ValueError as exc:
                print(f"Frame {frame.header.frame_number}: retarget HOLD — {exc}")
                continue
            retarget_done = time.perf_counter_ns()
            retarget_wall_ns = time.time_ns()
            ik = safe_ik.solve(targets)
            ik_done = time.perf_counter_ns()
            ik_wall_ns = time.time_ns()
            sim = sim_sink.update(ik) if sim_sink is not None and ik.valid else None
            sink_done = time.perf_counter_ns()
            sink_wall_ns = time.time_ns()
            timings = {
                "receive_to_parser": max(0.0, (frame.parsed_ns - frame.received_ns) / 1e6),
                "parser_to_retarget_start": (retarget_started - parse_done) / 1e6,
                "retarget": (retarget_done - retarget_started) / 1e6,
                "ik": (ik_done - retarget_done) / 1e6,
                "sink": (sink_done - ik_done) / 1e6,
                "processing_total": (sink_done - parse_done) / 1e6,
                "packet_age": max(0.0, (sink_wall_ns - frame.received_ns) / 1e6),
            }
            timestamps = {
                "udp_receive": frame.received_ns,
                "parser_complete": frame.parsed_ns,
                "retarget_complete": retarget_wall_ns,
                "ik_complete": ik_wall_ns,
                "sink_update": sink_wall_ns,
            }
            transport_stats = source.stats
            row = _row(
                frame, human, targets, ik, sim, timings, timestamps,
                source.source_name, watchdog, transport_stats, observation.status,
            )
            rows.append(row)
            if record:
                record.write(json.dumps(row) + "\n")
            now = time.monotonic()
            if args.source != "live" or now - last_print >= 1.0:
                sim_text = "monitor-only" if sim is None else f"finite={sim.finite} error={sim.maximum_arm_error_rad:.4f}rad"
                diagnostics = watchdog.diagnostics()
                source_fps = len(rows) / max(now - run_started, 1e-9)
                network_rate = f"{transport_stats.rate_hz:.1f}Hz" if transport_stats is not None else "n/a"
                backlog_drops = transport_stats.backlog_drops if transport_stats is not None else 0
                network_loss = max(0, diagnostics.estimated_lost_frames - backlog_drops)
                print(
                    f"Frame {frame.header.frame_number}: source={observation.status} "
                    f"after_sink={watchdog.status()} network={network_rate} "
                    f"processed={source_fps:.1f}Hz backlog_drops={backlog_drops} "
                    f"gaps~={diagnostics.estimated_lost_frames} network_loss~={network_loss} "
                    f"dup={diagnostics.duplicate_frames} "
                    f"ooo={diagnostics.out_of_order_frames} human={'OK' if human.valid else 'INVALID'} "
                    f"IK={'OK' if ik.valid else 'HOLD'} residual={ik.position_error_m}m "
                    f"clamps={','.join(targets.clamped) or '-'} rate_limit={ik.rate_limited} "
                    f"sink={sim_text} latency={timings['processing_total']:.2f}ms"
                )
                last_print = now
    except KeyboardInterrupt:
        print("Ctrl-C: receiver and simulation loop stopped; no physical robot interface existed.")
    finally:
        source.close()
        if sim_sink:
            sim_sink.close()
        if record:
            record.close()
    passed = bool(rows) and all(
        bool(row["ik_valid"]) and (mode != "sim" or bool(row["sim_finite"]))
        for row in rows
    )
    status = "SIM_VALIDATED" if mode == "sim" and passed else "SOFTWARE_VALIDATED" if passed else "PARTIAL"
    print(f"Result: {status} ({len(rows)} accepted frame(s)); physical G1 activity: NONE")
    return (0 if passed else 1), rows


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.duration is not None and args.duration <= 0:
        parser.error("--duration must be positive")
    if args.max_frames is not None and args.max_frames < 1:
        parser.error("--max-frames must be positive")
    if not 1 <= args.sim_steps <= 5000:
        parser.error("--sim-steps must be in 1..5000")
    try:
        config = load_config(args.config)
        result, _ = run(args, config)
        return result
    except (ImportError, OSError, RuntimeError, ValueError) as exc:
        print(f"Pipeline unavailable: {exc}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
