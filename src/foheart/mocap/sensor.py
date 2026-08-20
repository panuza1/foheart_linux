from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Quaternion:
    values: tuple[float, float, float, float]
    component_order: str | None = None


@dataclass(frozen=True)
class Vector3:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class SensorSample:
    sensor_id: int
    online: bool | None = None
    quaternion: Quaternion | None = None
    euler: Vector3 | None = None
    accel: Vector3 | None = None
    gyro: Vector3 | None = None
    magnetometer: Vector3 | None = None


@dataclass(frozen=True)
class SensorFrame:
    timestamp_ns: int
    frame_number: int | None
    sensors: list[SensorSample]

