"""Bounded synthetic/replay evidence -> existing G1 IK -> in-process MuJoCo."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from foheart.config import RuntimeConfig, load_config
from foheart.integrations.unitree_g1.adapter import (
    ExistingG1IK,
    G1FrameAdapter,
    SafeG1IK,
    UpperBodyTargetFilter,
)
from foheart.integrations.unitree_g1.sim_bridge import G1MuJoCoBridge
from foheart.mocap.calibration import CalibrationProfile
from foheart.mocap.frames import BasisTransform
from foheart.mocap.motion import load_motion_capture
from foheart.mocap.skeleton import BodyDimensions, UpperBodyKinematics
from foheart.mocap.synthetic import SYNTHETIC_BODY_MAP, synthetic_upper_body_sequence

PROJECT_ROOT = Path(__file__).resolve().parents[5]
DEFAULT_XR_ROOT = PROJECT_ROOT / "xr_teleoperate"
DEFAULT_MUJOCO_MODEL = PROJECT_ROOT / "unitree_mujoco/unitree_robots/g1/scene_29dof.xml"


def _orientations(item, mapping, calibration, sensor_basis):
    return {
        role: calibration.apply(
            role, sensor_basis.orientation(sample.raw.quaternion)
        )
        for role, sample in mapping.assign(item.frame).items()
    }


def run_pipeline(
    xr_root: Path,
    model_path: Path,
    *,
    steps_per_pose: int,
    capture_check: Path | None = None,
    config: RuntimeConfig | None = None,
) -> dict[str, object]:
    config = config or load_config()
    sensor_basis = BasisTransform(
        config.frames.sensor_to_body_matrix,
        "foheart_sensor_unknown",
        "configured_body_sensor",
        config.frames.status,
    )
    capture = None
    if capture_check is not None:
        checked = load_motion_capture(capture_check)
        capture = {
            "path": str(capture_check),
            "records": checked.records,
            "decoded_frames": len(checked.samples),
            "message_ids": checked.message_ids,
            "decode_errors": list(checked.decode_errors),
            "status": "REAL_CAPTURE_VALIDATED" if checked.samples and not checked.decode_errors else "PARTIAL",
            "body_pipeline": "PENDING: one slot cannot satisfy seven configured body roles",
        }
        if checked.samples:
            raw = checked.samples[0].raw_quaternion
            capture["first_raw_wxyz"] = list(raw.values)
            capture["first_configured_wxyz"] = list(sensor_basis.orientation(raw).values)
            capture["frame_conversion_status"] = config.frames.status

    solver = ExistingG1IK(xr_root)
    safe_ik = SafeG1IK(
        solver,
        max_joint_delta_rad=config.g1.max_joint_delta_rad,
        workspace_radius_m=config.retarget.workspace_radius_m,
    )
    bridge = G1MuJoCoBridge(model_path)
    sequence = synthetic_upper_body_sequence()
    neutral_samples = SYNTHETIC_BODY_MAP.assign(sequence[0].frame)
    calibration = CalibrationProfile.capture(
        {
            role: sensor_basis.orientation(sample.raw.quaternion)
            for role, sample in neutral_samples.items()
        }
    )
    kinematics = UpperBodyKinematics(BodyDimensions(**config.skeleton.__dict__))
    neutral_pose = kinematics.solve(
        _orientations(sequence[0], SYNTHETIC_BODY_MAP, calibration, sensor_basis),
        sequence[0].frame.timestamp_ns,
    )
    neutral_targets = kinematics.targets(neutral_pose)
    measured_robot_reach = float(
        np.mean(
            (
                np.linalg.norm(solver.neutral_left[:3, 3] - solver.left_shoulder),
                np.linalg.norm(solver.neutral_right[:3, 3] - solver.right_shoulder),
            )
        )
    )
    adapter = G1FrameAdapter(
        neutral_targets,
        solver.neutral_left,
        solver.neutral_right,
        solver.left_shoulder,
        solver.right_shoulder,
        human_reach_m=config.retarget.human_reach_m,
        robot_reach_m=config.retarget.g1_reach_m,
        max_robot_reach_m=config.retarget.max_robot_reach_m,
    )
    filter_ = UpperBodyTargetFilter(
        **config.filter.__dict__,
    )

    rows = []
    for item in sequence:
        pose = kinematics.solve(
            _orientations(item, SYNTHETIC_BODY_MAP, calibration, sensor_basis),
            item.frame.timestamp_ns,
        )
        human_targets = filter_.update(kinematics.targets(pose))
        robot_targets = adapter.adapt(human_targets)
        ik = safe_ik.solve(robot_targets)
        sim = bridge.command(ik.joint_positions, steps=steps_per_pose)
        rows.append(
            {
                "pose": item.name,
                "left_wrist_xyz_m": human_targets.left_wrist_pose[:3, 3].tolist(),
                "right_wrist_xyz_m": human_targets.right_wrist_pose[:3, 3].tolist(),
                "workspace_clamped": list(robot_targets.clamped),
                "ik_valid": ik.valid,
                "ik_reason": ik.reason,
                "ik_rate_limited": ik.rate_limited,
                "ik_position_error_m": ik.position_error_m,
                "ik_rotation_error_deg": ik.rotation_error_deg,
                "sim_max_arm_error_rad": sim.maximum_arm_error_rad,
                "sim_mean_arm_error_rad": sim.mean_arm_error_rad,
                "sim_max_non_arm_drift_rad": sim.maximum_non_arm_drift_rad,
                "sim_finite": sim.finite,
            }
        )
    sim_validated = all(
        row["ik_valid"]
        and row["sim_finite"]
        and row["sim_max_arm_error_rad"] < 0.05
        and row["sim_max_non_arm_drift_rad"] < 0.01
        for row in rows
    )
    return {
        "mode": "SIMULATION_ONLY",
        "usb_activity": "NONE",
        "real_g1_activity": "NONE",
        "source": "CONFIGURED deterministic seven-role synthetic upper body",
        "real_capture_check": capture,
        "calibration": "CONFIGURED synthetic neutral; software tested, not live body validated",
        "body_mapping": dict(SYNTHETIC_BODY_MAP.role_to_slot),
        "existing_ik": str(xr_root / "teleop/robot_control/robot_arm_ik.py") + ":G1_29_ArmIK",
        "ik_joint_order": list(solver.ik.reduced_robot.model.names[1:]),
        "mujoco_model": str(model_path),
        "floating_base": "pinned in-process for deterministic upper-body validation",
        "measured_neutral_g1_reach_m": measured_robot_reach,
        "configured_g1_reach_m": config.retarget.g1_reach_m,
        "steps_per_pose": steps_per_pose,
        "poses": rows,
        "ik_successes": sum(bool(row["ik_valid"]) for row in rows),
        "sim_validated": sim_validated,
        "status": "SIM_VALIDATED" if sim_validated else "PARTIAL",
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="SIM-ONLY FOHEART upper-body -> existing G1 IK -> MuJoCo validation")
    parser.add_argument("--config", type=Path)
    parser.add_argument("--xr-root", type=Path)
    parser.add_argument("--mujoco-model", type=Path)
    parser.add_argument("--steps-per-pose", type=int)
    parser.add_argument("--capture-check", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    try:
        config = load_config(args.config)
    except ValueError as exc:
        print(f"Configuration error: {exc}")
        return 2
    xr_root = args.xr_root or (Path(config.g1.xr_root) if config.g1.xr_root else DEFAULT_XR_ROOT)
    model = args.mujoco_model or (Path(config.g1.mujoco_model) if config.g1.mujoco_model else DEFAULT_MUJOCO_MODEL)
    steps = args.steps_per_pose or config.g1.steps_per_pose
    if not 50 <= steps <= 1000:
        parser.error("--steps-per-pose must be between 50 and 1000")
    if args.output and args.output.exists():
        print(f"Refusing to overwrite validation output: {args.output}")
        return 2
    print("FOHEART -> G1 MUJOCO — SIMULATION ONLY")
    print("No FOHEART USB operation. No DDS. No physical G1 path.")
    try:
        summary = run_pipeline(
            xr_root,
            model,
            steps_per_pose=steps,
            capture_check=args.capture_check,
            config=config,
        )
    except (ImportError, OSError, RuntimeError, ValueError) as exc:
        print(f"Simulation pipeline unavailable: {exc}")
        return 2
    for row in summary["poses"]:
        print(
            f"{row['pose']}: ik={'OK' if row['ik_valid'] else 'HOLD'} "
            f"sim_error={row['sim_max_arm_error_rad']:.6f} rad "
            f"non_arm_drift={row['sim_max_non_arm_drift_rad']:.3g} rad "
            f"finite={row['sim_finite']}"
        )
    print(
        f"Result: {summary['status']} "
        f"({summary['ik_successes']}/{len(summary['poses'])} IK targets)"
    )
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        print(f"Metrics: {args.output}")
    return 0 if summary["sim_validated"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
