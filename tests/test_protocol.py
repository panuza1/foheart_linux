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
