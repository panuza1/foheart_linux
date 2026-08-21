from __future__ import annotations

import math
import statistics
from collections import Counter
from dataclasses import dataclass
from pathlib import Path

from foheart.protocol.definitions import ProtocolError
from foheart.protocol.frame import iter_poll_recording
from foheart.protocol.parser import decode_hid_0x15_report

from .orientation import (
    continuity_adjusted,
    quaternion_angular_distance_degrees,
    quaternion_norm,
    quaternion_to_axis_angle,
    relative_quaternion,
)
from .sensor import Quaternion, Vector3

AXIS_NAMES = ("QX", "QY", "QZ")
GYRO_AXIS_NAMES = ("GX", "GY", "GZ")


@dataclass(frozen=True)
class MotionSample:
    timestamp_ns: int
    raw_quaternion: Quaternion
    analysis_quaternion: Quaternion
    accel: Vector3
    gyro: Vector3
    magnetometer: Vector3
    flags: int


@dataclass(frozen=True)
class MotionCapture:
    path: Path
    records: int
    samples: tuple[MotionSample, ...]
    message_ids: dict[str, int]
    flags: dict[str, int]
    decode_errors: tuple[str, ...]
    sign_flips: int


def _vector(vector: Vector3) -> tuple[float, float, float]:
    return vector.x, vector.y, vector.z


def _magnitude(values: tuple[float, ...]) -> float:
    return math.sqrt(sum(value * value for value in values))


def _stats(values: list[float]) -> dict[str, float]:
    return {
        "min": min(values),
        "mean": statistics.fmean(values),
        "max": max(values),
    }


def _mean_vector(vectors: list[tuple[float, float, float]]) -> list[float]:
    return [statistics.fmean(values) for values in zip(*vectors)]


def _dominant(values: tuple[float, float, float], names: tuple[str, str, str]) -> dict[str, object]:
    ranked = sorted(range(3), key=lambda index: abs(values[index]), reverse=True)
    index = ranked[0]
    largest = abs(values[index])
    return {
        "axis": names[index] if largest else "UNKNOWN",
        "sign": "+" if values[index] > 0 else "-" if values[index] < 0 else "UNKNOWN",
        "dominance_ratio": (
            largest / abs(values[ranked[1]])
            if values[ranked[1]]
            else math.inf if largest else 0.0
        ),
    }


def _mean_orientation(quaternions: tuple[Quaternion, ...]) -> Quaternion:
    values = tuple(statistics.fmean(items) for items in zip(*(q.values for q in quaternions)))
    norm = math.sqrt(sum(value * value for value in values))
    if norm == 0:
        raise ValueError("mean quaternion is zero")
    return Quaternion(tuple(value / norm for value in values), "wxyz")


def load_motion_capture(path: str | Path) -> MotionCapture:
    records = list(iter_poll_recording(path))
    message_ids: Counter[int] = Counter()
    flags: Counter[int] = Counter()
    errors: list[str] = []
    samples: list[MotionSample] = []
    previous: Quaternion | None = None
    sign_flips = 0
    for record in records:
        if not record.payload:
            continue
        message_ids[record.payload[0]] += 1
        if record.payload[0] != 0x15:
            continue
        try:
            report = decode_hid_0x15_report(record.payload)
        except ProtocolError as exc:
            errors.append(f"poll {record.sequence}: {exc}")
            continue
        sample = report.sample
        if not all((sample.quaternion, sample.accel, sample.gyro, sample.magnetometer)):
            errors.append(f"poll {record.sequence}: validated 0x15 fields are incomplete")
            continue
        raw = sample.quaternion
        analysis = raw if previous is None else continuity_adjusted(previous, raw)
        sign_flips += previous is not None and analysis.values != raw.values
        previous = analysis
        flags[report.flags] += 1
        samples.append(
            MotionSample(
                record.in_timestamp_ns or record.poll_timestamp_ns,
                raw,
                analysis,
                sample.accel,
                sample.gyro,
                sample.magnetometer,
                report.flags,
            )
        )
    return MotionCapture(
        Path(path),
        len(records),
        tuple(samples),
        {f"0x{key:02x}": value for key, value in sorted(message_ids.items())},
        {f"0x{key:08x}": value for key, value in sorted(flags.items())},
        tuple(errors),
        sign_flips,
    )


def _noise_threshold(values: list[float]) -> float:
    return max(max(values) * 1.5, statistics.fmean(values) + 6 * statistics.pstdev(values))


def _active_runs(active: list[bool], minimum: int = 3) -> list[tuple[int, int]]:
    runs: list[tuple[int, int]] = []
    start: int | None = None
    for index, value in enumerate(active + [False]):
        if value and start is None:
            start = index
        elif not value and start is not None:
            if index - start >= minimum:
                runs.append((start, index - 1))
            start = None
    return runs


def _segment_motion(
    samples: tuple[MotionSample, ...],
    consecutive_angles: list[float],
    thresholds: dict[str, float],
) -> dict[str, object]:
    active = [False]
    for index in range(1, len(samples)):
        active.append(
            _magnitude(_vector(samples[index].gyro)) > thresholds["gyro_magnitude"]
            or consecutive_angles[index - 1] > thresholds["quaternion_step_degrees"]
        )
    runs = _active_runs(active)
    if not runs:
        return {
            "status": "UNKNOWN",
            "thresholds": thresholds,
            "initial_stationary": False,
            "motion": False,
            "final_stationary": False,
            "initial_samples": 0,
            "motion_samples": 0,
            "final_samples": 0,
            "motion_start_index": None,
            "motion_end_index": None,
        }
    start, end = runs[0][0], runs[-1][1]
    initial_count = start
    final_count = len(samples) - end - 1
    initial_stationary = initial_count >= 10
    final_stationary = final_count >= 10
    return {
        "status": "VALIDATED" if initial_stationary and final_stationary else "PARTIAL",
        "thresholds": thresholds,
        "initial_stationary": initial_stationary,
        "motion": True,
        "final_stationary": final_stationary,
        "initial_samples": initial_count,
        "motion_samples": end - start + 1,
        "final_samples": final_count,
        "motion_start_index": start,
        "motion_end_index": end,
    }


def analyze_motion_capture(
    path: str | Path, *, baseline: dict[str, object] | None = None
) -> dict[str, object]:
    capture = load_motion_capture(path)
    samples = capture.samples
    if not samples:
        raise ValueError(f"no validated HID 0x15 samples in {path}")

    timestamps = [sample.timestamp_ns for sample in samples]
    duration = max(0.0, (timestamps[-1] - timestamps[0]) / 1_000_000_000)
    raw_quaternions = tuple(sample.raw_quaternion for sample in samples)
    analysis_quaternions = tuple(sample.analysis_quaternion for sample in samples)
    norms = [quaternion_norm(quaternion) for quaternion in raw_quaternions]
    consecutive_angles = [
        quaternion_angular_distance_degrees(left, right)
        for left, right in zip(analysis_quaternions, analysis_quaternions[1:])
    ]
    gyro_vectors = [_vector(sample.gyro) for sample in samples]
    gyro_magnitudes = [_magnitude(vector) for vector in gyro_vectors]
    accel_vectors = [_vector(sample.accel) for sample in samples]
    accel_norms = [_magnitude(vector) for vector in accel_vectors]

    thresholds = (
        baseline["segmentation_thresholds"]
        if baseline is not None
        else {
            "gyro_magnitude": _noise_threshold(gyro_magnitudes),
            "quaternion_step_degrees": _noise_threshold(consecutive_angles or [0.0]),
        }
    )
    segmentation = (
        _segment_motion(samples, consecutive_angles, thresholds)
        if baseline is not None
        else {
            "status": "NOT_APPLICABLE",
            "thresholds": thresholds,
            "initial_stationary": True,
            "motion": False,
            "final_stationary": True,
            "initial_samples": len(samples),
            "motion_samples": 0,
            "final_samples": len(samples),
            "motion_start_index": None,
            "motion_end_index": None,
        }
    )

    start = segmentation["motion_start_index"]
    end = segmentation["motion_end_index"]
    window = min(20, max(1, len(samples) // 10))
    if start is None or end is None:
        initial_region = analysis_quaternions[:window]
        final_region = analysis_quaternions[-window:]
        motion_slice = slice(0, len(samples))
    else:
        initial_region = analysis_quaternions[max(0, start - window) : start] or analysis_quaternions[:1]
        final_region = analysis_quaternions[end + 1 : end + 1 + window] or analysis_quaternions[-1:]
        motion_slice = slice(start, end + 1)
    initial = _mean_orientation(initial_region)
    final = _mean_orientation(final_region)
    relative = relative_quaternion(initial, final)
    relative_axis, relative_angle = quaternion_to_axis_angle(relative)
    quaternion_dominant = _dominant(_vector(relative_axis), AXIS_NAMES)

    motion_samples = samples[motion_slice]
    peak_sample = max(motion_samples, key=lambda sample: _magnitude(_vector(sample.gyro)))
    peak_vector = _vector(peak_sample.gyro)
    integrated = [0.0, 0.0, 0.0]
    for previous, current in zip(motion_samples, motion_samples[1:]):
        dt = max(0.0, (current.timestamp_ns - previous.timestamp_ns) / 1_000_000_000)
        for index, value in enumerate(_vector(current.gyro)):
            integrated[index] += value * dt
    gyro_direction = _dominant(tuple(integrated), GYRO_AXIS_NAMES)
    axis_agreement = quaternion_dominant["axis"][1:] == gyro_direction["axis"][1:] if (
        quaternion_dominant["axis"] != "UNKNOWN" and gyro_direction["axis"] != "UNKNOWN"
    ) else False
    sign_agreement = quaternion_dominant["sign"] == gyro_direction["sign"] if axis_agreement else False
    confidence = "UNKNOWN"
    if segmentation["motion"]:
        confidence = "PARTIAL"
        if relative_angle >= 10 and quaternion_dominant["dominance_ratio"] >= 1.5:
            confidence = "MEDIUM"
        if (
            segmentation["status"] == "VALIDATED"
            and relative_angle >= 30
            and quaternion_dominant["dominance_ratio"] >= 2
            and gyro_direction["dominance_ratio"] >= 2
            and axis_agreement
        ):
            confidence = "HIGH"

    parser_validated = (
        capture.message_ids == {"0x15": len(samples)} and not capture.decode_errors
    )
    return {
        "capture": str(Path(path)),
        "poll_records": capture.records,
        "decoded_frames": len(samples),
        "sample_count": len(samples),
        "duration_seconds": duration,
        "report_rate_hz": (len(samples) - 1) / duration if duration > 0 else 0.0,
        "message_ids": capture.message_ids,
        "flags": capture.flags,
        "decode_errors": list(capture.decode_errors),
        "parser_regression": "REAL_CAPTURE_VALIDATED" if parser_validated and not capture.decode_errors else "PARTIAL",
        "euler": "STATIC_ONLY" if not any(sample.flags & 0x20 for sample in samples) else "UNKNOWN",
        "sensor_identity": "slot_0",
        "raw_quaternion_sign_flips_for_continuity": capture.sign_flips,
        "raw_initial_quaternion": list(raw_quaternions[0].values),
        "raw_final_quaternion": list(raw_quaternions[-1].values),
        "analysis_initial_quaternion": list(initial.values),
        "analysis_final_quaternion": list(final.values),
        "quaternion_norm": _stats(norms),
        "initial_to_final_angle_degrees": quaternion_angular_distance_degrees(initial, final),
        "consecutive_angular_change_degrees": _stats(consecutive_angles or [0.0]),
        "relative_quaternion": list(relative.values),
        "relative_axis_qx_qy_qz": list(_vector(relative_axis)),
        "relative_angle_degrees": relative_angle,
        "dominant_quaternion": quaternion_dominant,
        "gyro": {
            "mean_vector": _mean_vector(gyro_vectors),
            "maximum_absolute_vector": [max(abs(vector[i]) for vector in gyro_vectors) for i in range(3)],
            "magnitude": _stats(gyro_magnitudes),
            "peak_vector": list(peak_vector),
            "peak_magnitude": _magnitude(peak_vector),
            "motion_integral": integrated,
            "dominant_motion": gyro_direction,
        },
        "accel": {
            "mean_vector": _mean_vector(accel_vectors),
            "norm": _stats(accel_norms),
        },
        "segmentation_thresholds": thresholds,
        "segmentation": segmentation,
        "quaternion_gyro_axis_agreement": axis_agreement,
        "quaternion_gyro_sign_agreement": sign_agreement,
        "confidence": confidence,
    }


def infer_axis_mapping(motions: dict[str, dict[str, object]]) -> dict[str, object]:
    physical = {
        "table_yaw_cw": ("UP", -1),
        "forward_tilt": ("RIGHT", -1),
        "right_roll": ("FRONT", 1),
    }
    if set(motions) != set(physical) or any(
        motion["confidence"] != "HIGH" or not motion["quaternion_gyro_axis_agreement"]
        for motion in motions.values()
    ):
        return {"status": "UNKNOWN", "mapping": {}, "sign_mapping": {}, "handedness": "UNKNOWN", "confidence": "UNKNOWN"}

    mapping: dict[str, str] = {}
    signs: dict[str, str] = {}
    axis_indices: dict[str, int] = {}
    sign_values: dict[str, int] = {}
    for key, (physical_axis, motion_sign) in physical.items():
        dominant = motions[key]["dominant_quaternion"]
        axis = dominant["axis"]
        observed_sign = 1 if dominant["sign"] == "+" else -1
        physical_positive_sign = observed_sign * motion_sign
        mapping[physical_axis] = axis
        signs[physical_axis] = "+" if physical_positive_sign > 0 else "-"
        axis_indices[physical_axis] = AXIS_NAMES.index(axis)
        sign_values[physical_axis] = physical_positive_sign
    if len(set(axis_indices.values())) != 3:
        return {"status": "PARTIAL", "mapping": mapping, "sign_mapping": signs, "handedness": "UNKNOWN", "confidence": "PARTIAL"}

    order = [axis_indices[name] for name in ("RIGHT", "FRONT", "UP")]
    inversions = sum(order[i] > order[j] for i in range(3) for j in range(i + 1, 3))
    determinant = (-1 if inversions % 2 else 1) * math.prod(
        sign_values[name] for name in ("RIGHT", "FRONT", "UP")
    )
    return {
        "status": "CONTROLLED_MOTION_VALIDATED",
        "mapping": mapping,
        "sign_mapping": signs,
        "handedness": "right-handed" if determinant > 0 else "left-handed",
        "confidence": "HIGH",
    }
