"""MotionVenus SDK 3.2.5 custom-forwarding protocol (V4003).

Binary layout and JSON keys follow the local SDK sources exactly.  The binary
header is little-endian and padded to 128 bytes.  Skeleton positions are metres;
quaternions are XYZW on the wire.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import struct
import time
from typing import Any, Iterable


PROTOCOL_VERSION = 4003
BINARY_HEADER_SIZE = 128

# MotionVenusSDK v1.3 is the unambiguous canonical order for a body-only suit.
BODY_BONE_NAMES = (
    "Pelvis", "L5", "L3", "T12", "T8", "Neck", "Head",
    "RightShoulder", "RightUpperArm", "RightForeArm", "RightHand",
    "LeftShoulder", "LeftUpperArm", "LeftForeArm", "LeftHand",
    "RightUpperLeg", "RightLowerLeg", "RightFoot", "RightToe",
    "LeftUpperLeg", "LeftLowerLeg", "LeftFoot", "LeftToe",
)

# SDK 3.2.5's 53-slot array.  Its published string array places LeftHandMiddle3
# after the ring slots; retain that source-defined order instead of guessing.
FULL_BONE_NAMES = (
    "Pelvis", "L5", "L3", "T12", "T8", "Neck", "Head",
    "RightShoulder", "RightUpperArm", "RightForeArm", "RightHand",
    "RightHandThumb1", "RightHandThumb2", "RightHandThumb3",
    "RightHandIndex1", "RightHandIndex2", "RightHandIndex3",
    "RightHandMiddle1", "RightHandMiddle2", "RightHandMiddle3",
    "RightHandRing1", "RightHandRing2", "RightHandRing3",
    "RightHandPinky1", "RightHandPinky2", "RightHandPinky3",
    "LeftShoulder", "LeftUpperArm", "LeftForeArm", "LeftHand",
    "LeftHandThumb1", "LeftHandThumb2", "LeftHandThumb3",
    "LeftHandIndex1", "LeftHandIndex2", "LeftHandIndex3",
    "LeftHandMiddle1", "LeftHandMiddle2", "LeftHandRing1",
    "LeftHandRing2", "LeftHandRing3", "LeftHandMiddle3",
    "LeftHandPinky1", "LeftHandPinky2", "LeftHandPinky3",
    "RightUpperLeg", "RightLowerLeg", "RightFoot", "RightToe",
    "LeftUpperLeg", "LeftLowerLeg", "LeftFoot", "LeftToe",
)

FULL_BONE_SHORT_NAMES = (
    "PELV", "L5", "L3", "T12", "T8", "NECK", "HEAD",
    "rSHOU", "ruARM", "rfARM", "rHAND",
    "RHT1", "RHT2", "RHT3", "RHI1", "RHI2", "RHI3",
    "RHM1", "RHM2", "RHM3", "RHR1", "RHR2", "RHR3",
    "RHP1", "RHP2", "RHP3",
    "lSHOU", "luARM", "lfARM", "lHAND",
    "LHT1", "LHT2", "LHT3", "LHI1", "LHI2", "LHI3",
    "LHM1", "LHM2", "LHR1", "LHR2", "LHR3", "LHM3",
    "LHP1", "LHP2", "LHP3",
    "ruLEG", "rLEG", "rFOOT", "rTOE", "luLEG", "lLEG", "lFOOT", "lTOE",
)

_CHANNEL_ORDERS = ("XYZ", "XZY", "YXZ", "YZX", "ZXY", "ZYX")
_POSITION_FORMATS = ("none", "meter")
_ATTITUDE_FORMATS = ("none", "euler", "quaternion")
_COORDINATES = ("local", "global")
_SENSOR_ATTITUDES = ("none", "euler", "quaternion")


class MotionVenusProtocolError(ValueError):
    """A datagram cannot be decoded safely."""

    def __init__(self, message: str, *, kind: str = "malformed"):
        super().__init__(message)
        self.kind = kind


def _finite_tuple(values: Iterable[Any], length: int, name: str) -> tuple[float, ...]:
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError) as exc:
        raise MotionVenusProtocolError(f"{name} must contain finite numbers") from exc
    if len(result) != length or not all(math.isfinite(value) for value in result):
        raise MotionVenusProtocolError(f"{name} must contain {length} finite numbers")
    return result


def _normalize_xyzw(values: Iterable[Any], name: str) -> tuple[float, float, float, float]:
    value = _finite_tuple(values, 4, name)
    norm = math.sqrt(sum(component * component for component in value))
    if norm < 1e-8:
        raise MotionVenusProtocolError(f"{name} is near zero")
    return tuple(component / norm for component in value)  # type: ignore[return-value]


@dataclass(frozen=True)
class MotionVenusHeader:
    protocol_version: int
    avatar_name: str
    avatar_name_raw: bytes
    suit_number: int
    suit_type: int
    frame_number: int
    body_skeleton_count: int
    left_finger_skeleton_count: int
    right_finger_skeleton_count: int
    stream_format: str
    position_format: str
    attitude_format: str
    skeleton_coordinate: str
    channel_order: str
    sensor_attitude_format: str = "none"
    sensor_accel_unit: str = "none"
    sensor_linear_accel_unit: str = "none"
    sensor_gyro_unit: str = "none"
    sensor_magnetometer_unit: str = "none"
    hip_height_m: float | None = None

    @property
    def total_skeleton_count(self) -> int:
        return self.body_skeleton_count + self.left_finger_skeleton_count + self.right_finger_skeleton_count


@dataclass(frozen=True)
class MotionVenusBone:
    index: int
    name: str
    position_global_m: tuple[float, float, float] | None = None
    rotation_global_xyzw: tuple[float, float, float, float] | None = None
    rotation_local_xyzw: tuple[float, float, float, float] | None = None
    euler_global_deg: tuple[float, float, float] | None = None
    euler_local_deg: tuple[float, float, float] | None = None
    sensor_quaternion_xyzw: tuple[float, float, float, float] | None = None
    sensor_euler_deg: tuple[float, float, float] | None = None
    sensor_accel: tuple[float, float, float] | None = None
    sensor_linear_accel: tuple[float, float, float] | None = None
    sensor_gyro: tuple[float, float, float] | None = None
    sensor_magnetometer: tuple[float, float, float] | None = None


@dataclass(frozen=True)
class MotionVenusFrame:
    header: MotionVenusHeader
    bones: tuple[MotionVenusBone, ...]
    received_ns: int
    sender: tuple[str, int]
    packet_size: int
    parsed_ns: int

    def bone(self, name: str) -> MotionVenusBone:
        for bone in self.bones:
            if bone.name == name:
                return bone
        raise KeyError(name)


class _Cursor:
    def __init__(self, payload: bytes):
        self.payload = payload
        self.offset = 0

    def read(self, fmt: str, name: str):
        size = struct.calcsize(fmt)
        if self.offset + size > len(self.payload):
            raise MotionVenusProtocolError(f"packet truncated while reading {name}")
        values = struct.unpack_from(fmt, self.payload, self.offset)
        self.offset += size
        return values[0] if len(values) == 1 else values

    def bytes(self, length: int, name: str) -> bytes:
        if length < 0 or self.offset + length > len(self.payload):
            raise MotionVenusProtocolError(f"packet truncated while reading {name}")
        value = self.payload[self.offset : self.offset + length]
        self.offset += length
        return value


class MotionVenusStreamDecoder:
    """Decode SDK custom-forwarding binary or JSON without binding a socket."""

    def __init__(self, *, expected_body_bones: int = 23, packet_format: str = "auto"):
        if expected_body_bones != len(BODY_BONE_NAMES):
            raise ValueError(f"this solved-body model requires {len(BODY_BONE_NAMES)} bones")
        if packet_format not in ("auto", "binary", "json"):
            raise ValueError("packet_format must be auto, binary, or json")
        self.expected_body_bones = expected_body_bones
        self.packet_format = packet_format

    def decode(
        self,
        payload: bytes,
        *,
        received_ns: int | None = None,
        sender: tuple[str, int] = ("0.0.0.0", 0),
    ) -> MotionVenusFrame:
        if not isinstance(payload, bytes) or not payload:
            raise MotionVenusProtocolError("empty MotionVenus datagram")
        detected = "json" if payload.lstrip().startswith(b"{") else "binary"
        if self.packet_format != "auto" and detected != self.packet_format:
            raise MotionVenusProtocolError(
                f"received {detected}, configured for {self.packet_format}", kind="protocol_mismatch"
            )
        received = time.time_ns() if received_ns is None else received_ns
        header, bones = self._decode_json(payload) if detected == "json" else self._decode_binary(payload)
        return MotionVenusFrame(header, tuple(bones), received, sender, len(payload), time.time_ns())

    def _decode_binary(self, payload: bytes) -> tuple[MotionVenusHeader, list[MotionVenusBone]]:
        if len(payload) < BINARY_HEADER_SIZE:
            raise MotionVenusProtocolError(
                f"binary packet is {len(payload)} bytes; minimum is {BINARY_HEADER_SIZE}"
            )
        cursor = _Cursor(payload)
        version = cursor.read("<H", "protocolVersion")
        if version != PROTOCOL_VERSION:
            raise MotionVenusProtocolError(
                f"unsupported MotionVenus protocol {version}; expected {PROTOCOL_VERSION}",
                kind="protocol_mismatch",
            )
        name_length = cursor.read("<B", "AvatarNameLength")
        if name_length >= 64:
            raise MotionVenusProtocolError(f"AvatarNameLength {name_length} must be below the SDK limit 64")
        avatar_raw = cursor.bytes(name_length, "AvatarName")
        suit_number = cursor.read("<I", "suitNumber")
        suit_type = cursor.read("<B", "suitType")
        if suit_type not in (0, 1, 255):
            raise MotionVenusProtocolError(f"unsupported suitType value {suit_type}")
        frame_number = cursor.read("<I", "frameNumber")
        body_count, left_count, right_count = cursor.read("<BBB", "skeleton counts")
        flags = cursor.read("<10B", "stream flags")
        (
            stream_format, position_format, attitude_format, coordinate, channel_order,
            sensor_attitude, sensor_accel, sensor_laccel, sensor_gyro, sensor_mag,
        ) = flags
        if stream_format != 0:
            raise MotionVenusProtocolError("binary payload header does not declare binary format")
        self._validate_flags(
            position_format, attitude_format, coordinate, channel_order,
            sensor_attitude, sensor_accel, sensor_laccel, sensor_gyro, sensor_mag,
        )
        hip_height_raw = cursor.read("<I", "hipHeight")
        cursor.bytes(5, "engine templates")
        cursor.bytes(5, "other engine templates")
        cursor.bytes(3, "idColorRGB")
        cursor.read("<B", "isContainSkeletonEulerBias")
        cursor.bytes(6, "thumb0AddEuler")
        cursor.bytes(6, "armAddEuler")
        if cursor.offset > BINARY_HEADER_SIZE:
            raise MotionVenusProtocolError("variable header exceeds the SDK 128-byte boundary")
        cursor.offset = BINARY_HEADER_SIZE

        total_count = body_count + left_count + right_count
        names = self._validate_counts(body_count, left_count, right_count)
        record_size = self._record_size(
            position_format, attitude_format, sensor_attitude,
            sensor_accel, sensor_laccel, sensor_gyro, sensor_mag,
        )
        expected_length = BINARY_HEADER_SIZE + total_count * record_size
        if len(payload) != expected_length:
            relation = "truncated" if len(payload) < expected_length else "has trailing data"
            raise MotionVenusProtocolError(
                f"binary packet {relation}: got {len(payload)} bytes, expected {expected_length}"
            )
        bones = [
            self._decode_binary_bone(
                cursor, index, name, position_format, attitude_format, coordinate,
                sensor_attitude, sensor_accel, sensor_laccel, sensor_gyro, sensor_mag,
            )
            for index, name in enumerate(names)
        ]
        header = MotionVenusHeader(
            version,
            avatar_raw.decode("utf-8", errors="replace"),
            avatar_raw,
            suit_number,
            suit_type,
            frame_number,
            body_count,
            left_count,
            right_count,
            "binary",
            _POSITION_FORMATS[position_format],
            _ATTITUDE_FORMATS[attitude_format],
            _COORDINATES[coordinate],
            _CHANNEL_ORDERS[channel_order],
            _SENSOR_ATTITUDES[sensor_attitude],
            "g" if sensor_accel == 1 else "none",
            "g" if sensor_laccel == 1 else "none",
            "degree_per_second" if sensor_gyro == 1 else "none",
            "mGauss" if sensor_mag == 1 else "none",
            hip_height_raw / 65536.0,
        )
        return header, bones

    def _validate_flags(
        self,
        position: int,
        attitude: int,
        coordinate: int,
        channel: int,
        sensor_attitude: int,
        sensor_accel: int,
        sensor_laccel: int,
        sensor_gyro: int,
        sensor_mag: int,
    ) -> None:
        checks = (
            (position, range(2), "skeletonPosition"),
            (attitude, range(3), "skeletonAttitude"),
            (coordinate, range(2), "skeletonCoordinate"),
            (channel, range(6), "channelOrder"),
            (sensor_attitude, range(3), "sensorAttitude"),
            (sensor_accel, range(2), "sensorAccel"),
            (sensor_laccel, range(2), "sensorLAccel"),
            (sensor_gyro, range(2), "sensorGyro"),
            (sensor_mag, range(2), "sensorMag"),
        )
        for value, allowed, name in checks:
            if value not in allowed:
                raise MotionVenusProtocolError(f"unsupported {name} value {value}")

    def _validate_counts(self, body: int, left: int, right: int) -> tuple[str, ...]:
        if body != self.expected_body_bones:
            raise MotionVenusProtocolError(
                f"body bone count {body} does not match expected {self.expected_body_bones}",
                kind="protocol_mismatch",
            )
        if left == right == 0:
            return BODY_BONE_NAMES
        if left == right == 15:
            return FULL_BONE_NAMES
        raise MotionVenusProtocolError(
            f"unsupported skeleton counts {body}+{left}+{right}; SDK layouts are 23 body or 53 body+hands",
            kind="protocol_mismatch",
        )

    @staticmethod
    def _record_size(position: int, attitude: int, sensor_attitude: int, accel: int, laccel: int, gyro: int, mag: int) -> int:
        return (
            (12 if position == 1 else 0)
            + (6 if attitude == 1 else 8 if attitude == 2 else 0)
            + (6 if sensor_attitude == 1 else 8 if sensor_attitude == 2 else 0)
            + (6 if accel == 1 else 0)
            + (6 if laccel == 1 else 0)
            + (12 if gyro == 1 else 0)  # SDK 3.2.5 reads three int32 values.
            + (6 if mag == 1 else 0)
        )

    def _decode_binary_bone(
        self,
        cursor: _Cursor,
        index: int,
        name: str,
        position_format: int,
        attitude_format: int,
        coordinate: int,
        sensor_attitude: int,
        sensor_accel: int,
        sensor_laccel: int,
        sensor_gyro: int,
        sensor_mag: int,
    ) -> MotionVenusBone:
        position = tuple(value / 65536.0 for value in cursor.read("<3i", f"{name} position")) if position_format else None
        rotation_global = rotation_local = euler_global = euler_local = None
        if attitude_format == 1:
            euler = tuple(value / 128.0 for value in cursor.read("<3h", f"{name} Euler"))
            if coordinate:
                euler_global = euler
            else:
                euler_local = euler
        elif attitude_format == 2:
            quat = _normalize_xyzw(
                (value / 8192.0 for value in cursor.read("<4h", f"{name} quaternion")),
                f"{name} quaternion",
            )
            if coordinate:
                rotation_global = quat
            else:
                rotation_local = quat
        sensor_quat = sensor_euler = accel = laccel = gyro = mag = None
        if sensor_attitude == 1:
            sensor_euler = tuple(value / 128.0 for value in cursor.read("<3h", f"{name} sensor Euler"))
        elif sensor_attitude == 2:
            sensor_quat = _normalize_xyzw(
                (value / 8192.0 for value in cursor.read("<4h", f"{name} sensor quaternion")),
                f"{name} sensor quaternion",
            )
        if sensor_accel:
            accel = tuple(value / 1024.0 for value in cursor.read("<3h", f"{name} acceleration"))
        if sensor_laccel:
            laccel = tuple(value / 1024.0 for value in cursor.read("<3h", f"{name} linear acceleration"))
        if sensor_gyro:
            gyro = tuple(value / 1024.0 for value in cursor.read("<3i", f"{name} gyro"))
        if sensor_mag:
            mag = tuple(value / 1024.0 for value in cursor.read("<3h", f"{name} magnetometer"))
        return MotionVenusBone(
            index, name, position, rotation_global, rotation_local, euler_global, euler_local,
            sensor_quat, sensor_euler, accel, laccel, gyro, mag,
        )

    def _decode_json(self, payload: bytes) -> tuple[MotionVenusHeader, list[MotionVenusBone]]:
        try:
            document = json.loads(
                payload.decode("utf-8"),
                parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)),
            )
        except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as exc:
            raise MotionVenusProtocolError(f"invalid MotionVenus JSON: {exc}") from exc
        if not isinstance(document, dict):
            raise MotionVenusProtocolError("MotionVenus JSON root must be an object")
        version_text = document.get("streamVer")
        if version_text != "V4003":
            raise MotionVenusProtocolError(
                f"unsupported MotionVenus JSON streamVer {version_text!r}", kind="protocol_mismatch"
            )
        avatar = document.get("ActorName")
        if not isinstance(avatar, str) or len(avatar.encode("utf-8")) >= 64:
            raise MotionVenusProtocolError("ActorName must be a string below 64 bytes")
        sn = document.get("SN")
        if not isinstance(sn, str) or not sn:
            raise MotionVenusProtocolError("SN must be a hexadecimal string")
        try:
            suit_number = int(sn, 16)
        except ValueError as exc:
            raise MotionVenusProtocolError("SN must be a hexadecimal string") from exc
        if not 0 <= suit_number <= 0xFFFFFFFF:
            raise MotionVenusProtocolError("SN must fit uint32")
        frame_number = document.get("FrameNumber")
        skeleton_count = document.get("SkeletonCount")
        if isinstance(frame_number, bool) or not isinstance(frame_number, int) or not 0 <= frame_number <= 0xFFFFFFFF:
            raise MotionVenusProtocolError("FrameNumber must be a uint32")
        if skeleton_count != 53:
            raise MotionVenusProtocolError(
                "SDK 3.2.5 JSON V4003 defines SkeletonCount 53", kind="protocol_mismatch"
            )
        if self.expected_body_bones != 23:
            raise MotionVenusProtocolError("JSON V4003 body count is 23", kind="protocol_mismatch")
        position_enabled = self._json_bool(document, "SkeletonPosition")
        quat_enabled = self._json_bool(document, "SkeletonQuat")
        euler_enabled = self._json_bool(document, "SkeletonEuler")
        if quat_enabled == euler_enabled:
            raise MotionVenusProtocolError("exactly one of SkeletonQuat and SkeletonEuler must be true")
        global_coord = self._json_bool(document, "GlobalCoord")
        local_coord = self._json_bool(document, "LocalCoord")
        if global_coord == local_coord:
            raise MotionVenusProtocolError("exactly one of GlobalCoord and LocalCoord must be true")
        sensor_quat_enabled = self._json_bool(document, "SensorQuat", default=False)
        sensor_euler_enabled = self._json_bool(document, "SensorEuler", default=False)
        if sensor_quat_enabled and sensor_euler_enabled:
            raise MotionVenusProtocolError("SensorQuat and SensorEuler cannot both be true")
        sensor_accel = self._json_bool(document, "SensorAccel", default=False)
        sensor_laccel = self._json_bool(document, "SensorLAccel", default=False)
        sensor_gyro = self._json_bool(document, "SensorGyro", default=False)
        sensor_mag = self._json_bool(document, "SensorMag", default=False)
        order = document.get("RotOrder")
        if order not in _CHANNEL_ORDERS:
            raise MotionVenusProtocolError(f"unsupported RotOrder {order!r}")
        bones = [
            self._decode_json_bone(
                document, index, name, short, position_enabled, quat_enabled,
                global_coord, sensor_quat_enabled, sensor_euler_enabled,
                sensor_accel, sensor_laccel, sensor_gyro, sensor_mag,
            )
            for index, (name, short) in enumerate(zip(FULL_BONE_NAMES, FULL_BONE_SHORT_NAMES))
        ]
        avatar_raw = avatar.encode("utf-8")
        header = MotionVenusHeader(
            PROTOCOL_VERSION, avatar, avatar_raw, suit_number, 255, frame_number,
            23, 15, 15, "json", "meter" if position_enabled else "none",
            "quaternion" if quat_enabled else "euler",
            "global" if global_coord else "local", order,
            "quaternion" if sensor_quat_enabled else "euler" if sensor_euler_enabled else "none",
            "g" if sensor_accel else "none", "g" if sensor_laccel else "none",
            "degree_per_second" if sensor_gyro else "none",
            "mGauss" if sensor_mag else "none", None,
        )
        return header, bones

    @staticmethod
    def _json_bool(document: dict[str, Any], key: str, *, default: bool | None = None) -> bool:
        if key not in document and default is not None:
            return default
        value = document.get(key)
        if not isinstance(value, bool):
            raise MotionVenusProtocolError(f"{key} must be boolean")
        return value

    def _decode_json_bone(
        self,
        document: dict[str, Any],
        index: int,
        name: str,
        short: str,
        position_enabled: bool,
        quat_enabled: bool,
        global_coord: bool,
        sensor_quat_enabled: bool,
        sensor_euler_enabled: bool,
        sensor_accel_enabled: bool,
        sensor_laccel_enabled: bool,
        sensor_gyro_enabled: bool,
        sensor_mag_enabled: bool,
    ) -> MotionVenusBone:
        value = document.get(short)
        if not isinstance(value, dict):
            raise MotionVenusProtocolError(f"missing JSON bone object {short}")
        position = _finite_tuple(value.get("LOC", ()), 3, f"{short}.LOC") if position_enabled else None
        quat = _normalize_xyzw(value.get("KQ", ()), f"{short}.KQ") if quat_enabled else None
        euler = _finite_tuple(value.get("KE", ()), 3, f"{short}.KE") if not quat_enabled else None
        sensor_quat = _normalize_xyzw(value.get("SQ", ()), f"{short}.SQ") if sensor_quat_enabled else None
        sensor_euler = _finite_tuple(value.get("SE", ()), 3, f"{short}.SE") if sensor_euler_enabled else None
        accel = _finite_tuple(value.get("A", ()), 3, f"{short}.A") if sensor_accel_enabled else None
        laccel = _finite_tuple(value.get("LA", ()), 3, f"{short}.LA") if sensor_laccel_enabled else None
        gyro = _finite_tuple(value.get("G", ()), 3, f"{short}.G") if sensor_gyro_enabled else None
        mag = _finite_tuple(value.get("M", ()), 3, f"{short}.M") if sensor_mag_enabled else None
        return MotionVenusBone(
            index, name, position,
            quat if global_coord else None, quat if not global_coord else None,
            euler if global_coord else None, euler if not global_coord else None,
            sensor_quat, sensor_euler, accel, laccel, gyro, mag,
        )


def encode_binary_frame(frame: MotionVenusFrame) -> bytes:
    """Encode the SDK's recommended body-only position+global-quaternion profile.

    This exists for deterministic synthetic/loopback validation, not as a second
    parser or a robot command format.
    """

    header = frame.header
    if header.protocol_version != PROTOCOL_VERSION or len(frame.bones) != 23:
        raise ValueError("encoder supports V4003 body-only 23-bone frames")
    if header.body_skeleton_count != 23 or header.left_finger_skeleton_count or header.right_finger_skeleton_count:
        raise ValueError("encoder supports body-only frames")
    avatar = header.avatar_name_raw
    if len(avatar) >= 64:
        raise ValueError("avatar name must be below 64 bytes")
    prefix = bytearray()
    prefix.extend(struct.pack("<HB", PROTOCOL_VERSION, len(avatar)))
    prefix.extend(avatar)
    prefix.extend(struct.pack("<IBIBBB", header.suit_number, header.suit_type, header.frame_number, 23, 0, 0))
    prefix.extend(struct.pack("<10B", 0, 1, 2, 1, 0, 0, 0, 0, 0, 0))
    prefix.extend(struct.pack("<I", round((header.hip_height_m or 0.0) * 65536)))
    prefix.extend(bytes(5 + 5 + 3 + 1 + 6 + 6))
    if len(prefix) > BINARY_HEADER_SIZE:
        raise ValueError("encoded MotionVenus header exceeds 128 bytes")
    prefix.extend(bytes(BINARY_HEADER_SIZE - len(prefix)))
    for expected_name, bone in zip(BODY_BONE_NAMES, frame.bones):
        if bone.name != expected_name or bone.position_global_m is None or bone.rotation_global_xyzw is None:
            raise ValueError("encoder requires canonical global position+quaternion bones")
        position = _finite_tuple(bone.position_global_m, 3, f"{bone.name} position")
        quaternion = _normalize_xyzw(bone.rotation_global_xyzw, f"{bone.name} quaternion")
        raw_position = tuple(round(value * 65536) for value in position)
        raw_quaternion = tuple(max(-32768, min(32767, round(value * 8192))) for value in quaternion)
        prefix.extend(struct.pack("<3i4h", *raw_position, *raw_quaternion))
    return bytes(prefix)
