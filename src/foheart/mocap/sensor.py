from __future__ import annotations

from dataclasses import dataclass, field


EVIDENCE_STATES = frozenset(
    {
        "REAL_CAPTURE_VALIDATED",
        "CONTROLLED_MOTION_VALIDATED",
        "STATIC_ONLY",
        "CONFIGURED",
        "SIM_VALIDATED",
        "SOFTWARE_TESTED",
        "MANUAL_DERIVED",
        "PARTIAL",
        "UNKNOWN",
    }
)


@dataclass(frozen=True)
class TransportKey:
    """Opaque transport identity; it is never a body role or physical label."""

    kind: str
    value: str
    evidence_status: str = field(default="UNKNOWN", compare=False)

    def __post_init__(self) -> None:
        if not self.kind or not self.value:
            raise ValueError("transport key kind and value cannot be empty")
        if self.evidence_status not in EVIDENCE_STATES:
            raise ValueError(f"unsupported evidence status: {self.evidence_status}")

    @property
    def debug_label(self) -> str:
        return f"{self.kind}:{self.value} ({self.evidence_status})"


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
    field_status: tuple[tuple[str, str], ...] = ()
    slot: str | None = None
    coordinate_frame: str = "foheart_sensor_unknown"
    validation_status: str = "UNKNOWN"


@dataclass(frozen=True)
class SensorFrame:
    timestamp_ns: int
    frame_number: int | None
    sensors: list[SensorSample]
