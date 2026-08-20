from __future__ import annotations

import struct
import time

from foheart.mocap.sensor import Quaternion, SensorFrame, SensorSample, Vector3
from foheart.protocol.definitions import MalformedPayloadError, ProtocolNotDecodedError

# fhusb.dll ce9049...: USBBulkReadWorkerHs @ 0x10020916..0x10020b6c.
BULK_HS_FIXED_MESSAGE_ID = 0x13
BULK_HS_FIXED_MESSAGE_SIZE = 0x88E
BULK_HS_FIXED_HEADER_SIZE = 0x0E
FIXED_SENSOR_RECORD_SIZE = 0x22
_FIXED_RECORD = struct.Struct("<17h")


def resolve_outer_frame(payload: bytes, configured_mode: str = "auto") -> str:
    if configured_mode not in ("auto", "fixed_0x13", "raw"):
        raise ValueError(f"unsupported outer frame mode: {configured_mode}")
    if configured_mode == "raw":
        return "raw"
    matches_fixed = (
        len(payload) == BULK_HS_FIXED_MESSAGE_SIZE
        and payload[0] == BULK_HS_FIXED_MESSAGE_ID
        and payload[5] == 0
    )
    return "fixed_0x13" if matches_fixed else "raw"


def resolve_sensor_id_mode(configured_mode: str = "auto") -> str:
    if configured_mode not in ("auto", "loop_index", "decoded_index", "unknown"):
        raise ValueError(f"unsupported sensor ID mode: {configured_mode}")
    # Loop index is the strongest safe default: it is passed separately by the
    # recovered worker, while the identity-prefix low bits remain unvalidated.
    return "loop_index" if configured_mode == "auto" else configured_mode


def decode_fixed_sensor_record(record: bytes, sensor_id: int) -> SensorSample:
    """Decode the statically recovered 0x22-byte record variant.

    The sensor ID is external to the record on the recovered bulk path. Evidence:
    fhusb.dll parser 0x10006d30 and caller 0x1002098e..0x100209b2.
    """
    if not isinstance(record, bytes) or len(record) != FIXED_SENSOR_RECORD_SIZE:
        raise MalformedPayloadError("fixed sensor record must be exactly 0x22 bytes")
    if not 0 <= sensor_id <= 0x3F:
        raise ValueError("sensor_id must fit the decoded six-bit index")

    (
        _unknown,
        ax,
        ay,
        az,
        gx,
        gy,
        gz,
        mx,
        my,
        mz,
        qw,
        qx,
        qy,
        qz,
        ex,
        ey,
        ez,
    ) = _FIXED_RECORD.unpack(record)
    # Scales are literal float constants loaded by fhusb.dll parser 0x10006d30:
    # accel 0x10029730, gyro 0x10029720, mag 0x10029740,
    # quaternion 0x10029744, and Euler 0x10029724.
    # Raw quaternion order is preserved by that parser. MotionVenus.exe
    # 0x00e0c89f..0x00e0c916 maps returned [0,1,2,3] to osg::Quat(w,x,y,z).
    return SensorSample(
        sensor_id=sensor_id,
        online=None,
        quaternion=Quaternion(
            tuple(value / 16384.0 for value in (qw, qx, qy, qz)),
            component_order="wxyz",
        ),
        euler=Vector3(ex / 128.0, ey / 128.0, ez / 128.0),
        accel=Vector3(ax / 2048.0, ay / 2048.0, az / 2048.0),
        gyro=Vector3(gx / 16.4, gy / 16.4, gz / 16.4),
        magnetometer=Vector3(mx / 12000.0, my / 12000.0, mz / 12000.0),
    )


class C1ProtocolParser:
    def __init__(self, sensor_id_mode: str = "auto"):
        self.sensor_id_mode = resolve_sensor_id_mode(sensor_id_mode)

    def feed(
        self, payload: bytes, *, timestamp_ns: int | None = None
    ) -> list[SensorFrame]:
        if not isinstance(payload, bytes) or not payload:
            raise MalformedPayloadError("C1 payload must be non-empty bytes")
        if payload[0] != BULK_HS_FIXED_MESSAGE_ID:
            raise ProtocolNotDecodedError(
                f"C1 message 0x{payload[0]:02x} is not decoded"
            )
        if len(payload) != BULK_HS_FIXED_MESSAGE_SIZE:
            raise MalformedPayloadError(
                "bulk HS 0x13 message must be exactly 0x88e bytes; "
                "USB aggregation is not yet validated"
            )
        if payload[5] != 0:
            raise ProtocolNotDecodedError(
                f"bulk HS 0x13 record format {payload[5]} is not decoded"
            )

        # The DLL loop is hard-coded to indices 1..4; it does not read a count.
        sensors = []
        identity_prefix = int.from_bytes(payload[1:5], "little")
        for slot in range(1, 5):
            start = BULK_HS_FIXED_HEADER_SIZE + slot * FIXED_SENSOR_RECORD_SIZE
            sensor_id = (
                (identity_prefix | slot) & 0x3F
                if self.sensor_id_mode == "decoded_index"
                else slot
            )
            sensors.append(
                decode_fixed_sensor_record(
                    payload[start : start + FIXED_SENSOR_RECORD_SIZE], sensor_id
                )
            )
        return [
            SensorFrame(
                timestamp_ns=time.time_ns() if timestamp_ns is None else timestamp_ns,
                frame_number=None,
                sensors=sensors,
            )
        ]
