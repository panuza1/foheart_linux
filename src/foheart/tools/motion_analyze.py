from __future__ import annotations

import argparse
import json
from pathlib import Path

from foheart.mocap.motion import analyze_motion_capture


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Offline FOHEART controlled-motion analysis")
    parser.add_argument("capture", type=Path)
    parser.add_argument("--baseline", type=Path, help="stationary capture used only to derive segmentation thresholds")
    parser.add_argument("--json", type=Path, dest="json_path")
    return parser


def print_analysis(analysis: dict[str, object]) -> None:
    qnorm = analysis["quaternion_norm"]
    step = analysis["consecutive_angular_change_degrees"]
    gyro = analysis["gyro"]
    accel = analysis["accel"]
    segment = analysis["segmentation"]
    print(f"capture: {analysis['capture']}")
    print(f"samples: {analysis['sample_count']}")
    print(f"duration_s: {analysis['duration_seconds']:.6f}")
    print(f"report_rate_hz: {analysis['report_rate_hz']:.3f}")
    print(f"message_ids: {analysis['message_ids']}")
    print(f"flags: {analysis['flags']}")
    print(f"quaternion_norm min/mean/max: {qnorm['min']:.9f}/{qnorm['mean']:.9f}/{qnorm['max']:.9f}")
    print(f"raw_initial_quaternion WXYZ: {analysis['raw_initial_quaternion']}")
    print(f"raw_final_quaternion WXYZ: {analysis['raw_final_quaternion']}")
    print(f"analysis_initial_quaternion WXYZ: {analysis['analysis_initial_quaternion']}")
    print(f"analysis_final_quaternion WXYZ: {analysis['analysis_final_quaternion']}")
    print(f"initial_to_final_angle_deg: {analysis['initial_to_final_angle_degrees']:.6f}")
    print(f"consecutive_angle_deg mean/max: {step['mean']:.6f}/{step['max']:.6f}")
    print(f"relative_quaternion WXYZ: {analysis['relative_quaternion']}")
    print(f"relative_axis QX/QY/QZ: {analysis['relative_axis_qx_qy_qz']}")
    print(f"relative_angle_deg: {analysis['relative_angle_degrees']:.6f}")
    print(f"dominant_quaternion: {analysis['dominant_quaternion']}")
    print(f"gyro_mean GX/GY/GZ: {gyro['mean_vector']}")
    print(f"gyro_max_abs GX/GY/GZ: {gyro['maximum_absolute_vector']}")
    print(f"gyro_peak GX/GY/GZ: {gyro['peak_vector']} magnitude={gyro['peak_magnitude']:.6f}")
    print(f"gyro_motion_integral GX/GY/GZ: {gyro['motion_integral']}")
    print(f"dominant_gyro: {gyro['dominant_motion']}")
    print(f"accel_mean AX/AY/AZ: {accel['mean_vector']}")
    print(f"accel_norm min/mean/max: {accel['norm']['min']:.6f}/{accel['norm']['mean']:.6f}/{accel['norm']['max']:.6f}")
    print(f"segmentation: {segment}")
    print(f"quaternion_gyro_axis_agreement: {analysis['quaternion_gyro_axis_agreement']}")
    print(f"quaternion_gyro_sign_agreement: {analysis['quaternion_gyro_sign_agreement']}")
    print(f"confidence: {analysis['confidence']}")


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        baseline = analyze_motion_capture(args.baseline) if args.baseline else None
        analysis = analyze_motion_capture(args.capture, baseline=baseline)
    except (OSError, ValueError) as exc:
        print(f"ERROR: {exc}")
        return 2
    print("FOHEART OFFLINE MOTION ANALYSIS — NO USB OPERATIONS")
    print_analysis(analysis)
    if args.json_path:
        if args.json_path.exists():
            print(f"ERROR: refusing to overwrite analysis: {args.json_path}")
            return 2
        args.json_path.write_text(json.dumps(analysis, indent=2) + "\n", encoding="utf-8")
        print(f"analysis_json: {args.json_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
