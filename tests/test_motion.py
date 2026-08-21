import math
import struct
import json

import pytest

from foheart.mocap.motion import analyze_motion_capture, infer_axis_mapping
from foheart.mocap.orientation import (
    continuity_adjusted,
    quaternion_angular_distance_degrees,
    quaternion_conjugate,
    quaternion_inverse,
    quaternion_multiply,
    quaternion_norm,
    quaternion_to_axis_angle,
    relative_quaternion,
    shortest_quaternion_distance,
)
from foheart.mocap.sensor import Quaternion, Vector3
from foheart.protocol.frame import PollCaptureRecord, PollRecorder
from foheart.tools import guided_motion_capture
from foheart.usb.c1_poll import C1PollCaptureResult


IDENTITY = Quaternion((1.0, 0.0, 0.0, 0.0), "wxyz")


def rotation_90(axis):
    half = math.sqrt(0.5)
    return Quaternion((half, *(half * value for value in axis)), "wxyz")


def test_quaternion_identity_inverse_and_multiply():
    rotation = rotation_90((0, 1, 0))
    assert quaternion_norm(Quaternion((2.0, 0.0, 0.0, 0.0), "wxyz")) == 2.0
    assert quaternion_conjugate(rotation).values == pytest.approx(
        (rotation.values[0], 0.0, -rotation.values[2], 0.0)
    )
    assert quaternion_multiply(rotation, quaternion_inverse(rotation)).values == pytest.approx(
        IDENTITY.values
    )
    assert relative_quaternion(IDENTITY, rotation).values == pytest.approx(rotation.values)


@pytest.mark.parametrize(
    ("axis", "expected"),
    [
        ((1, 0, 0), Vector3(1.0, 0.0, 0.0)),
        ((0, 1, 0), Vector3(0.0, 1.0, 0.0)),
        ((0, 0, 1), Vector3(0.0, 0.0, 1.0)),
    ],
)
def test_90_degree_axis_angle_and_angular_distance(axis, expected):
    rotation = rotation_90(axis)
    measured_axis, angle = quaternion_to_axis_angle(rotation)
    assert measured_axis == pytest.approx(expected)
    assert angle == pytest.approx(90.0)
    assert quaternion_angular_distance_degrees(IDENTITY, rotation) == pytest.approx(90.0)


def test_q_and_negative_q_are_equivalent_but_raw_value_is_preserved():
    raw = rotation_90((0, 0, 1))
    negative = Quaternion(tuple(-value for value in raw.values), "wxyz")
    adjusted = continuity_adjusted(raw, negative)
    assert negative.values[0] < 0
    assert adjusted.values == pytest.approx(raw.values)
    assert shortest_quaternion_distance(raw, negative) == pytest.approx(0.0)
    assert quaternion_angular_distance_degrees(raw, negative) == pytest.approx(0.0)


def test_zero_quaternion_rejected_for_orientation_operations():
    zero = Quaternion((0.0, 0.0, 0.0, 0.0), "wxyz")
    with pytest.raises(ValueError):
        quaternion_inverse(zero)
    with pytest.raises(ValueError):
        quaternion_to_axis_angle(zero)


def hid_report(quaternion, gyro=(0.0, 0.0, 0.0), counter=0):
    payload = bytearray(64)
    payload[0:5] = bytes.fromhex("15 00 dd 03 14")
    payload[5:7] = counter.to_bytes(2, "little")
    payload[7:11] = (0x1D).to_bytes(4, "little")
    struct.pack_into("<4h", payload, 11, *(round(value * 16384) for value in quaternion))
    struct.pack_into("<3h", payload, 19, 0, 0, 2048)
    struct.pack_into("<3h", payload, 25, *(round(value * 16.4) for value in gyro))
    struct.pack_into("<3h", payload, 31, 1000, 2000, 3000)
    return bytes(payload)


def write_capture(path, quaternions, gyros):
    with path.open("wb") as stream:
        recorder = PollRecorder(stream)
        for index, (quaternion, gyro) in enumerate(zip(quaternions, gyros), 1):
            timestamp = index * 10_000_000
            recorder.write(
                PollCaptureRecord(
                    index,
                    timestamp - 1_000_000,
                    64,
                    timestamp,
                    0x81,
                    hid_report(quaternion, gyro, index),
                    False,
                    None,
                    1_000_000,
                )
            )


def test_offline_motion_analysis_uses_baseline_noise_and_finds_90_degree_motion(tmp_path):
    baseline_path = tmp_path / "baseline.bin"
    motion_path = tmp_path / "motion.bin"
    identity = IDENTITY.values
    write_capture(baseline_path, [identity] * 60, [(0.0, 0.0, 0.0)] * 60)

    quaternions = []
    gyros = []
    for index in range(60):
        angle = 0.0 if index < 20 else 90.0 if index >= 40 else (index - 19) * 4.5
        half = math.radians(angle) / 2
        quaternions.append((math.cos(half), 0.0, 0.0, math.sin(half)))
        gyros.append((0.0, 0.0, 450.0) if 20 <= index < 40 else (0.0, 0.0, 0.0))
    write_capture(motion_path, quaternions, gyros)

    baseline = analyze_motion_capture(baseline_path)
    motion = analyze_motion_capture(motion_path, baseline=baseline)
    assert baseline["segmentation_thresholds"] == {
        "gyro_magnitude": 0.0,
        "quaternion_step_degrees": 0.0,
    }
    assert motion["relative_angle_degrees"] == pytest.approx(90.0, abs=0.02)
    assert motion["dominant_quaternion"]["axis"] == "QZ"
    assert motion["gyro"]["dominant_motion"]["axis"] == "GZ"
    assert motion["gyro"]["peak_vector"] == pytest.approx([0.0, 0.0, 450.0])
    assert motion["segmentation"]["initial_stationary"] is True
    assert motion["segmentation"]["final_stationary"] is True
    assert motion["confidence"] == "HIGH"
    assert motion["parser_regression"] == "REAL_CAPTURE_VALIDATED"


def test_axis_mapping_and_handedness_require_three_clear_independent_motions():
    def clear(axis, sign):
        return {
            "confidence": "HIGH",
            "quaternion_gyro_axis_agreement": True,
            "dominant_quaternion": {"axis": axis, "sign": sign},
        }

    inferred = infer_axis_mapping(
        {
            "table_yaw_cw": clear("QZ", "-"),
            "forward_tilt": clear("QX", "-"),
            "right_roll": clear("QY", "+"),
        }
    )
    assert inferred["mapping"] == {"UP": "QZ", "RIGHT": "QX", "FRONT": "QY"}
    assert inferred["sign_mapping"] == {"UP": "+", "RIGHT": "+", "FRONT": "+"}
    assert inferred["handedness"] == "right-handed"

    incomplete = infer_axis_mapping({"table_yaw_cw": clear("QZ", "-")})
    assert incomplete["status"] == "UNKNOWN"


def test_guided_capture_reuses_fixed_bounded_poller_and_preserves_four_files(
    tmp_path, monkeypatch
):
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr("builtins.input", lambda _: "")
    calls = []

    def fake_capture_polls(**kwargs):
        calls.append(kwargs)
        records = tuple(
            PollCaptureRecord(
                index,
                index * 10_000_000 - 1_000_000,
                64,
                index * 10_000_000,
                0x81,
                hid_report(IDENTITY.values, counter=index),
                False,
                None,
                1_000_000,
            )
            for index in range(1, 201)
        )
        return C1PollCaptureResult(records, 2_000_000_000, "poll limit reached", False)

    monkeypatch.setattr(guided_motion_capture, "capture_polls", fake_capture_polls)
    assert guided_motion_capture.main() == 0
    assert calls == [
        {"max_polls": 200, "timeout_ms": 100, "max_runtime_s": 30.0}
    ] * 4
    for _, path, _ in guided_motion_capture.PHASES:
        assert path.exists()
    summary = json.loads(guided_motion_capture.SUMMARY_PATH.read_text())
    assert summary["usb_safety"]["payload"] == "70 + 63 zero bytes"
    assert summary["usb_safety"]["any_other_payload"] is False
    assert summary["usb_safety"]["successful_64_byte_out_transfers"] == 800


def test_guided_capture_refuses_overwrite_before_hardware(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    (tmp_path / "samples").mkdir()
    (tmp_path / "samples/motion_baseline.bin").write_bytes(b"existing")
    monkeypatch.setattr(
        guided_motion_capture,
        "capture_polls",
        lambda **_: pytest.fail("hardware path opened despite existing capture"),
    )
    assert guided_motion_capture.main() == 2
