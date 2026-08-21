"""Shared synthetic, replay, and future-live sensor stream path."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field, replace
import math
from pathlib import Path
import re
import time
from typing import Any, Iterator, Mapping, Sequence

from foheart.protocol.definitions import ProtocolError
from foheart.protocol.frame import PollCaptureRecord, iter_poll_recording
from foheart.protocol.parser import decode_hid_0x15_report
from foheart.usb.c1_poll import _open_poll_device, _poll_open_device_once

from .calibration import CalibrationProfile
from .frames import BasisTransform, matrix_to_quaternion
from .orientation import quaternion_angular_distance_degrees
from .sensor import Quaternion, SensorSample, TransportKey, Vector3
from .skeleton import (
    BodyDimensions,
    FullBodyDimensions,
    FullBodyJointFrame,
    FullBodyKinematics,
    JointFrame,
    UpperBodyKinematics,
)
from .suit import (
    BodyProfile,
    BodySensorMap,
    LatestSuitBuffer,
    SuitFrame,
    TimedSensorSample,
    body_profile,
    build_calibrated_suit_frame,
)
from .synthetic import synthetic_live_rotations


class SensorSourceError(RuntimeError):
    pass


class TransportKeyCollisionError(SensorSourceError):
    pass


class UnexpectedTransportKeyError(SensorSourceError):
    pass


@dataclass(frozen=True)
class SourceSample:
    timestamp_ns: int
    transport_key: TransportKey
    sample: SensorSample
    debug: Mapping[str, object] = field(default_factory=dict)
    physical_sensor_label: str | None = None


class SensorSource(ABC):
    source_name = "UNKNOWN"
    c1_status = "NOT USED"

    def __init__(self) -> None:
        self.started = False
        self.eof = False

    def start(self) -> None:
        self.started = True

    @abstractmethod
    def next_sample(self) -> SourceSample | None:
        raise NotImplementedError

    def close(self) -> None:
        self.started = False

    def __enter__(self) -> "SensorSource":
        self.start()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()


def _event_from_hid_record(record: PollCaptureRecord) -> SourceSample:
    report = decode_hid_0x15_report(record.payload)
    timestamp_ns = record.in_timestamp_ns or record.poll_timestamp_ns
    return SourceSample(
        timestamp_ns,
        TransportKey(
            "hid_0x15_header_bytes_1_4", report.identity_raw.hex(), "UNKNOWN"
        ),
        report.sample,
        {
            "transport_key_status": "UNKNOWN",
            "identity_raw_hex": report.identity_raw.hex(" "),
            "counter_raw": report.counter_raw,
            "flags_hex": f"0x{report.flags:08x}",
        },
    )


class ReplaySensorSource(SensorSource):
    source_name = "REPLAY"

    def __init__(self, path: str | Path):
        super().__init__()
        self.path = Path(path)
        self._records: Iterator[PollCaptureRecord] | None = None
        self.decode_errors: list[str] = []

    def start(self) -> None:
        if not self.path.is_file():
            raise SensorSourceError(f"replay capture does not exist: {self.path}")
        self._records = iter(iter_poll_recording(self.path))
        self.decode_errors.clear()
        self.eof = False
        super().start()

    def next_sample(self) -> SourceSample | None:
        if not self.started or self._records is None:
            raise SensorSourceError("replay source is not started")
        for record in self._records:
            if not record.payload:
                continue
            try:
                return _event_from_hid_record(record)
            except ProtocolError as exc:
                self.decode_errors.append(f"poll {record.sequence}: {exc}")
        self.eof = True
        return None


class LiveC1SensorSource(SensorSource):
    """Future live source; construction and import perform no USB operation."""

    source_name = "LIVE_C1"
    c1_status = "NOT CONNECTED"

    def __init__(self, *, timeout_ms: int = 100, opener: Any = None):
        super().__init__()
        if not 1 <= timeout_ms <= 100:
            raise ValueError("live C1 timeout must be between 1 and 100 ms")
        self.timeout_ms = timeout_ms
        self.opener = opener
        self._device = None

    def start(self) -> None:
        if self.started:
            return
        self._device = _open_poll_device(self.opener)
        self.c1_status = "CONNECTED"
        super().start()

    def next_sample(self) -> SourceSample | None:
        if not self.started or self._device is None:
            raise SensorSourceError("live C1 source is not started")
        result = _poll_open_device_once(self._device, timeout_ms=self.timeout_ms)
        if result.timed_out or result.payload is None:
            return None
        record = PollCaptureRecord(
            1,
            result.poll_timestamp_ns,
            result.out_transferred,
            result.in_timestamp_ns,
            0x81,
            result.payload,
            False,
            None,
            result.elapsed_ns,
        )
        return _event_from_hid_record(record)

    def close(self) -> None:
        if self._device is not None:
            self._device.close()
            self._device = None
        self.c1_status = "NOT CONNECTED"
        super().close()


class SyntheticSensorSource(SensorSource):
    source_name = "SYNTHETIC"

    def __init__(
        self,
        *,
        fps: float = 30.0,
        start_timestamp_ns: int = 1_000_000_000,
        profile: BodyProfile | str = BodyProfile.UPPER,
    ):
        super().__init__()
        if fps <= 0:
            raise ValueError("synthetic fps must be positive")
        self.fps = float(fps)
        self.profile = body_profile(profile)
        self.start_timestamp_ns = start_timestamp_ns
        self.frame_number = 0
        self.sensor_index = 0
        self.current_motion = "neutral"
        self._previous: dict[int, Quaternion] = {}

    def start(self) -> None:
        self.frame_number = 0
        self.sensor_index = 0
        self.current_motion = "neutral"
        self._previous.clear()
        self.eof = False
        super().start()

    def next_sample(self) -> SourceSample:
        if not self.started:
            raise SensorSourceError("synthetic source is not started")
        motion, rotations = synthetic_live_rotations(
            self.frame_number, self.fps, self.profile
        )
        self.current_motion = motion
        sensor_id = self.sensor_index
        role = tuple(rotations)[sensor_id]
        quaternion = matrix_to_quaternion(rotations[role])
        previous = self._previous.get(sensor_id)
        angular_speed = (
            0.0
            if previous is None
            else quaternion_angular_distance_degrees(previous, quaternion) * self.fps
        )
        self._previous[sensor_id] = quaternion
        timestamp_ns = self.start_timestamp_ns + round(
            self.frame_number * 1_000_000_000 / self.fps
        )
        sample = SensorSample(
            sensor_id,
            online=True,
            quaternion=quaternion,
            accel=Vector3(0.0, 0.0, 1.0),
            gyro=Vector3(angular_speed, 0.0, 0.0),
            magnetometer=Vector3(1.0, 0.0, 0.0),
            field_status=(
                ("quaternion", "SOFTWARE_TESTED"),
                ("accel", "SOFTWARE_TESTED"),
                ("gyro", "SOFTWARE_TESTED"),
                ("magnetometer", "SOFTWARE_TESTED"),
            ),
            coordinate_frame="synthetic_human_world",
            validation_status="SOFTWARE_TESTED",
        )
        event = SourceSample(
            timestamp_ns,
            TransportKey("synthetic_sensor", str(sensor_id), "SOFTWARE_TESTED"),
            sample,
            {"motion": motion, "synthetic_role_hint": role},
            f"synthetic_sensor_{sensor_id}",
        )
        self.sensor_index += 1
        if self.sensor_index == len(rotations):
            self.sensor_index = 0
            self.frame_number += 1
        return event


@dataclass(frozen=True)
class LogicalSensor:
    slot: str
    transport_key: TransportKey
    last_sample: SensorSample
    first_seen_ns: int
    last_seen_ns: int
    packet_count: int
    debug: Mapping[str, object] = field(default_factory=dict)
    physical_sensor_label: str | None = None

    @property
    def packet_rate_hz(self) -> float:
        duration = (self.last_seen_ns - self.first_seen_ns) / 1_000_000_000
        return (self.packet_count - 1) / duration if duration > 0 else 0.0


class LogicalSlotRegistry:
    """Stable session slots keyed only by an explicit opaque transport key."""

    def __init__(self, bindings: Mapping[TransportKey, str] | None = None):
        bindings = dict(bindings or {})
        if len(set(bindings.values())) != len(bindings):
            raise ValueError("logical slot bindings must be unique")
        if any(not re.fullmatch(r"slot_\d+", slot) for slot in bindings.values()):
            raise ValueError("logical slots must use slot_N names")
        self._key_to_slot = bindings
        self._sensors: dict[str, LogicalSensor] = {}
        self._reserved_slots = set(bindings.values())
        self._bound_slots = set(bindings.values())
        self._running = False
        self._frozen = False
        self._diagnostics: list[str] = []

    @property
    def sensors(self) -> Mapping[str, LogicalSensor]:
        return dict(sorted(self._sensors.items(), key=lambda item: int(item[0][5:])))

    @property
    def diagnostics(self) -> tuple[str, ...]:
        return tuple(self._diagnostics)

    @property
    def missing_bound_slots(self) -> tuple[str, ...]:
        return tuple(sorted(self._bound_slots - set(self._sensors)))

    def stale_slots(self, timestamp_ns: int, stale_after_ns: int) -> tuple[str, ...]:
        if stale_after_ns < 1:
            raise ValueError("stale_after_ns must be positive")
        return tuple(
            slot
            for slot, sensor in self.sensors.items()
            if timestamp_ns - sensor.last_seen_ns > stale_after_ns
        )

    def mark_running(self) -> None:
        self._running = True

    def freeze(self) -> None:
        """Reject any key first seen after this point."""
        self._running = True
        self._frozen = True

    def _allocate(self) -> str:
        index = 0
        while f"slot_{index}" in self._reserved_slots:
            index += 1
        slot = f"slot_{index}"
        self._reserved_slots.add(slot)
        return slot

    def observe(self, event: SourceSample) -> LogicalSensor:
        slot = self._key_to_slot.get(event.transport_key)
        if slot is None:
            if self._frozen:
                message = (
                    "new transport key observed after slot registry was frozen: "
                    + event.transport_key.debug_label
                )
                self._diagnostics.append(message)
                raise UnexpectedTransportKeyError(message)
            slot = self._allocate()
            self._key_to_slot[event.transport_key] = slot
            if self._running:
                self._diagnostics.append(
                    f"new slot during running session: {slot} "
                    f"({event.transport_key.debug_label})"
                )
        sample = replace(event.sample, slot=slot)
        previous = self._sensors.get(slot)
        if previous is not None and previous.transport_key != event.transport_key:
            message = (
                f"slot reassignment refused for {slot}: "
                f"{previous.transport_key.debug_label} -> {event.transport_key.debug_label}"
            )
            self._diagnostics.append(message)
            raise TransportKeyCollisionError(message)
        if previous is not None and (
            previous.last_sample.sensor_id != event.sample.sensor_id
            or (
                previous.physical_sensor_label is not None
                and event.physical_sensor_label is not None
                and previous.physical_sensor_label != event.physical_sensor_label
            )
        ):
            message = (
                f"duplicate candidate transport key collision for {slot}: "
                f"{event.transport_key.debug_label}"
            )
            self._diagnostics.append(message)
            raise TransportKeyCollisionError(message)
        if previous is None:
            logical = LogicalSensor(
                slot,
                event.transport_key,
                sample,
                event.timestamp_ns,
                event.timestamp_ns,
                1,
                event.debug,
                event.physical_sensor_label,
            )
        else:
            newest_sample = sample if event.timestamp_ns >= previous.last_seen_ns else previous.last_sample
            logical = LogicalSensor(
                slot,
                event.transport_key,
                newest_sample,
                min(previous.first_seen_ns, event.timestamp_ns),
                max(previous.last_seen_ns, event.timestamp_ns),
                previous.packet_count + 1,
                event.debug if event.timestamp_ns >= previous.last_seen_ns else previous.debug,
                event.physical_sensor_label or previous.physical_sensor_label,
            )
        self._sensors[slot] = logical
        return logical


@dataclass(frozen=True)
class PipelineFrame:
    suit: SuitFrame
    joints: JointFrame | FullBodyJointFrame
    diagnostics: tuple[str, ...] = ()


class _BodyStreamProcessor:
    profile: BodyProfile

    def __init__(
        self,
        mapping: BodySensorMap,
        sensor_basis: BasisTransform,
        calibration: CalibrationProfile | None,
        *,
        stale_after_ms: float = 100.0,
        registry: LogicalSlotRegistry | None = None,
    ):
        if stale_after_ms <= 0:
            raise ValueError("stale_after_ms must be positive")
        mapping.require_profile(self.profile)
        self.mapping = mapping
        self.sensor_basis = sensor_basis
        self.calibration = calibration
        self.registry = registry or LogicalSlotRegistry(mapping.registry_bindings)
        self.buffer = LatestSuitBuffer(round(stale_after_ms * 1_000_000))
        self.expected_slots = tuple(mapping.role_to_slot.values())

    def _finish(self, grouped: SuitFrame) -> PipelineFrame:
        suit = build_calibrated_suit_frame(
            grouped, self.mapping, self.sensor_basis, self.calibration
        )
        if not suit.valid:
            return PipelineFrame(suit, self._invalid_joints(suit))
        joints = self._solve(suit)
        return PipelineFrame(suit, joints, self.kinematics.diagnose(joints))

    def _invalid_joints(self, suit: SuitFrame) -> JointFrame | FullBodyJointFrame:
        raise NotImplementedError

    def _solve(self, suit: SuitFrame) -> JointFrame | FullBodyJointFrame:
        raise NotImplementedError

    def process(self, event: SourceSample) -> PipelineFrame:
        logical = self.registry.observe(event)
        timed = TimedSensorSample(
            logical.last_seen_ns,
            logical.slot,
            logical.last_sample,
            logical.last_sample.coordinate_frame,
            logical.last_sample.validation_status,
        )
        grouped = self.buffer.update(
            SuitFrame(
                event.timestamp_ns,
                {logical.slot: timed},
                profile=self.mapping.profile.value,
            ),
            self.expected_slots,
        )
        return self._finish(grouped)

    def tick(self, timestamp_ns: int | None = None) -> PipelineFrame:
        grouped = self.buffer.update(
            SuitFrame(
                timestamp_ns or time.time_ns(),
                {},
                profile=self.mapping.profile.value,
            ),
            self.expected_slots,
        )
        return self._finish(grouped)


class UpperBodyStreamProcessor(_BodyStreamProcessor):
    profile = BodyProfile.UPPER

    def __init__(
        self,
        mapping: BodySensorMap,
        sensor_basis: BasisTransform,
        calibration: CalibrationProfile | None,
        *,
        stale_after_ms: float = 100.0,
        dimensions: BodyDimensions = BodyDimensions(),
        registry: LogicalSlotRegistry | None = None,
    ):
        super().__init__(
            mapping,
            sensor_basis,
            calibration,
            stale_after_ms=stale_after_ms,
            registry=registry,
        )
        self.kinematics = UpperBodyKinematics(dimensions)

    def _invalid_joints(self, suit: SuitFrame) -> JointFrame:
        return JointFrame(suit.timestamp_ns, {}, False, suit.reason)

    def _solve(self, suit: SuitFrame) -> JointFrame:
        pose = self.kinematics.solve(suit.orientations, suit.timestamp_ns)
        return self.kinematics.joints(pose)


class FullBodyStreamProcessor(_BodyStreamProcessor):
    profile = BodyProfile.FULL

    def __init__(
        self,
        mapping: BodySensorMap,
        sensor_basis: BasisTransform,
        calibration: CalibrationProfile | None,
        *,
        stale_after_ms: float = 100.0,
        dimensions: FullBodyDimensions = FullBodyDimensions(),
        registry: LogicalSlotRegistry | None = None,
    ):
        super().__init__(
            mapping,
            sensor_basis,
            calibration,
            stale_after_ms=stale_after_ms,
            registry=registry,
        )
        self.kinematics = FullBodyKinematics(dimensions)

    def _invalid_joints(self, suit: SuitFrame) -> FullBodyJointFrame:
        return FullBodyJointFrame(suit.timestamp_ns, {}, False, suit.reason)

    def _solve(self, suit: SuitFrame) -> FullBodyJointFrame:
        return self.kinematics.solve(suit.orientations, suit.timestamp_ns)


def create_sensor_source(
    *,
    synthetic: bool = False,
    replay: str | Path | None = None,
    fps: float = 30.0,
    profile: BodyProfile | str = BodyProfile.UPPER,
) -> SensorSource:
    if synthetic and replay is not None:
        raise ValueError("synthetic and replay sources are mutually exclusive")
    if synthetic:
        return SyntheticSensorSource(fps=fps, profile=profile)
    if replay is not None:
        return ReplaySensorSource(replay)
    return LiveC1SensorSource()


def motion_energy(samples: Sequence[SensorSample]) -> float:
    quaternions = [sample.quaternion for sample in samples if sample.quaternion]
    orientation = sum(
        quaternion_angular_distance_degrees(left, right)
        for left, right in zip(quaternions, quaternions[1:])
    )
    gyros = [
        math.sqrt(sample.gyro.x**2 + sample.gyro.y**2 + sample.gyro.z**2)
        for sample in samples
        if sample.gyro is not None
    ]
    return orientation + (sum(gyros) / len(gyros) if gyros else 0.0) * 0.01


def propose_moving_slot(
    samples_by_slot: Mapping[str, Sequence[SensorSample]],
) -> tuple[str, float] | None:
    scores = sorted(
        ((motion_energy(samples), slot) for slot, samples in samples_by_slot.items()),
        reverse=True,
    )
    if not scores or scores[0][0] <= 0:
        return None
    if len(scores) > 1 and math.isclose(scores[0][0], scores[1][0], rel_tol=0.05):
        return None
    return scores[0][1], scores[0][0]
