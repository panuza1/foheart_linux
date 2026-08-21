import struct

import pytest

from foheart.mocap.sensor import Quaternion, SensorFrame, SensorSample, Vector3
from foheart.protocol.definitions import MalformedPayloadError, ProtocolNotDecodedError
from foheart.protocol.parser import (
    BULK_HS_FIXED_HEADER_SIZE,
    BULK_HS_FIXED_MESSAGE_SIZE,
    FIXED_SENSOR_RECORD_SIZE,
    C1ProtocolParser,
    decode_fixed_sensor_record,
    decode_hid_0x15_report,
)


def test_sensor_frame_dataclasses_do_not_assume_quaternion_order():
    quaternion = Quaternion((1.0, 0.0, 0.0, 0.0))
    frame = SensorFrame(123, None, [SensorSample(7, True, quaternion)])
    assert frame.sensors[0].quaternion.component_order is None


def test_parser_rejects_malformed_input():
    with pytest.raises(MalformedPayloadError):
        C1ProtocolParser().feed(b"")


def test_parser_refuses_to_invent_real_layout():
    with pytest.raises(ProtocolNotDecodedError):
        C1ProtocolParser().feed(b"synthetic raw bytes")


def _synthetic_fixed_record(quaternion=(16384, -8192, 4096, -16384)):
    # Synthetic only: values exercise the statically proven offsets and scales.
    return struct.pack(
        "<17h",
        0x1234,
        -2048,
        1024,
        0,
        164,
        -82,
        0,
        12000,
        -12000,
        6000,
        *quaternion,
        128,
        -256,
        64,
    )


def test_decode_statically_recovered_fixed_sensor_record():
    sample = decode_fixed_sensor_record(_synthetic_fixed_record(), sensor_id=3)
    assert sample.sensor_id == 3
    assert sample.online is None
    assert sample.accel == Vector3(-1.0, 0.5, 0.0)
    assert sample.gyro == Vector3(10.0, -5.0, 0.0)
    assert sample.magnetometer == Vector3(1.0, -1.0, 0.5)
    assert sample.euler == Vector3(1.0, -2.0, 0.5)
    assert sample.quaternion.component_order == "wxyz"
    assert sample.quaternion.values == (1.0, -0.5, 0.25, -1.0)
    assert dict(sample.field_status)["quaternion"] == "STATIC_ONLY"


@pytest.mark.parametrize("size", [FIXED_SENSOR_RECORD_SIZE - 1, FIXED_SENSOR_RECORD_SIZE + 1])
def test_fixed_sensor_record_rejects_wrong_length(size):
    with pytest.raises(MalformedPayloadError):
        decode_fixed_sensor_record(bytes(size), sensor_id=1)


def test_parser_decodes_synthetic_bulk_hs_fixed_message():
    payload = bytearray(BULK_HS_FIXED_MESSAGE_SIZE)
    payload[0] = 0x13
    payload[5] = 0
    for sensor_id in range(1, 5):
        start = BULK_HS_FIXED_HEADER_SIZE + sensor_id * FIXED_SENSOR_RECORD_SIZE
        payload[start : start + FIXED_SENSOR_RECORD_SIZE] = _synthetic_fixed_record(
            quaternion=(sensor_id * 100, 0, 0, 0)
        )

    frame = C1ProtocolParser().feed(bytes(payload), timestamp_ns=123)[0]
    assert frame.timestamp_ns == 123
    assert frame.frame_number is None
    assert [sensor.sensor_id for sensor in frame.sensors] == [1, 2, 3, 4]
    assert frame.sensors[3].quaternion.values[0] == 400 / 16384.0


def test_bulk_hs_fixed_message_rejects_truncation():
    payload = bytes([0x13]) + bytes(BULK_HS_FIXED_MESSAGE_SIZE - 2)
    with pytest.raises(MalformedPayloadError):
        C1ProtocolParser().feed(payload)


def test_decoded_sensor_index_mode_uses_recovered_packed_index():
    payload = bytearray(BULK_HS_FIXED_MESSAGE_SIZE)
    payload[0] = 0x13
    payload[1:5] = (0x20).to_bytes(4, "little")
    payload[5] = 0
    frame = C1ProtocolParser("decoded_index").feed(bytes(payload))[0]
    assert [sensor.sensor_id for sensor in frame.sensors] == [0x21, 0x22, 0x23, 0x24]


REAL_HID_0X15_REPORT = bytes.fromhex(
    "15 00 dd 03 14 20 20 1d 8c 00 00 5f 1e d4 00 f4 ff ad c7 d6 ff "
    "21 00 ce 07 03 00 02 00 00 00 b3 e8 09 f0 5c f9 00 00 94 8f "
    "00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00"
)


def test_decode_real_hid_0x15_report():
    report = decode_hid_0x15_report(REAL_HID_0X15_REPORT)
    assert report.identity_raw == bytes.fromhex("00 dd 03 14")
    assert report.counter_raw == 0x2020
    assert report.flags == 0x8C1D
    assert report.identity_status == "UNKNOWN"
    assert report.counter_status == "UNKNOWN"
    assert report.sample.sensor_id == 0
    assert report.sample.quaternion.values == pytest.approx(
        (0.47454833984375, 0.012939453125, -0.000732421875, -0.88006591796875)
    )
    assert sum(value * value for value in report.sample.quaternion.values) ** 0.5 == pytest.approx(
        1.0, abs=0.0001
    )
    assert report.sample.accel == Vector3(-42 / 2048, 33 / 2048, 1998 / 2048)
    assert report.sample.gyro == Vector3(3 / 16.4, 2 / 16.4, 0.0)
    assert report.sample.magnetometer == Vector3(-5965 / 12000, -4087 / 12000, -1700 / 12000)
    assert report.sample.euler is None
    assert dict(report.sample.field_status) == {
        "sensor_id": "UNKNOWN",
        "quaternion": "REAL_CAPTURE_VALIDATED",
        "accel": "REAL_CAPTURE_VALIDATED",
        "gyro": "REAL_CAPTURE_VALIDATED",
        "magnetometer": "REAL_CAPTURE_VALIDATED",
    }


def test_protocol_parser_emits_one_capture_local_slot_for_real_hid_report():
    frame = C1ProtocolParser().feed(REAL_HID_0X15_REPORT, timestamp_ns=123)[0]
    assert frame.timestamp_ns == 123
    assert frame.frame_number is None
    assert [sample.sensor_id for sample in frame.sensors] == [0]


def test_hid_0x15_rejects_unvalidated_matrix_variant():
    payload = bytearray(REAL_HID_0X15_REPORT)
    payload[7] |= 0x02
    with pytest.raises(ProtocolNotDecodedError):
        decode_hid_0x15_report(bytes(payload))
