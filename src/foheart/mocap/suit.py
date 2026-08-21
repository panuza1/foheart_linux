"""Configured body mapping and timestamp-aware sensor grouping."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Mapping

import yaml

from .calibration import CalibrationProfile
from .frames import BasisTransform
from .sensor import Quaternion, SensorFrame, SensorSample, TransportKey

UPPER_BODY_ROLES = (
    "torso",
    "left_upper_arm",
    "left_forearm",
    "left_hand",
    "right_upper_arm",
    "right_forearm",
    "right_hand",
)

FULL_BODY_ROLES = (
    "head",
    "left_shoulder",
    "right_shoulder",
    "torso",
    "pelvis",
    "left_upper_arm",
    "right_upper_arm",
    "left_forearm",
    "right_forearm",
    "left_hand",
    "right_hand",
    "left_thigh",
    "right_thigh",
    "left_lower_leg",
    "right_lower_leg",
    "left_foot",
    "right_foot",
)


class BodyProfile(str, Enum):
    UPPER = "upper"
    FULL = "full"

    @property
    def roles(self) -> tuple[str, ...]:
        return UPPER_BODY_ROLES if self is BodyProfile.UPPER else FULL_BODY_ROLES


def body_profile(value: BodyProfile | str) -> BodyProfile:
    try:
        return value if isinstance(value, BodyProfile) else BodyProfile(value)
    except ValueError as exc:
        raise ValueError("body profile must be upper or full") from exc


def roles_for_profile(profile: BodyProfile | str) -> tuple[str, ...]:
    return body_profile(profile).roles


class _UniqueSafeLoader(yaml.SafeLoader):
    pass


def _construct_unique_mapping(loader, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise yaml.constructor.ConstructorError(
                "while constructing a mapping",
                node.start_mark,
                f"duplicate key: {key}",
                key_node.start_mark,
            )
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_UniqueSafeLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_unique_mapping
)


@dataclass(frozen=True)
class TimedSensorSample:
    timestamp_ns: int
    slot: str
    raw: SensorSample
    coordinate_frame: str = "foheart_sensor_unknown"
    status: str = "UNKNOWN"


@dataclass(frozen=True)
class SuitFrame:
    timestamp_ns: int
    samples: Mapping[str, TimedSensorSample]
    stale_slots: tuple[str, ...] = ()
    missing_slots: tuple[str, ...] = ()
    orientations: Mapping[str, Quaternion] = field(default_factory=dict)
    raw_orientations: Mapping[str, Quaternion] = field(default_factory=dict)
    converted_orientations: Mapping[str, Quaternion] = field(default_factory=dict)
    sample_ages_ms: Mapping[str, float] = field(default_factory=dict)
    valid: bool = True
    missing_roles: tuple[str, ...] = ()
    stale_roles: tuple[str, ...] = ()
    reason: str = ""
    profile: str = BodyProfile.UPPER.value


@dataclass(frozen=True)
class BodySensorMap:
    role_to_slot: Mapping[str, str]
    status: str = "CONFIGURED"
    slot_transport_keys: Mapping[str, TransportKey] = field(default_factory=dict)
    physical_labels: Mapping[str, str] = field(default_factory=dict)
    profile: BodyProfile | str = BodyProfile.UPPER

    def __post_init__(self) -> None:
        selected = body_profile(self.profile)
        object.__setattr__(self, "profile", selected)
        unknown = set(self.role_to_slot) - set(selected.roles)
        if unknown:
            raise ValueError(f"unknown body role: {sorted(unknown)[0]}")
        slots = list(self.role_to_slot.values())
        if any(not isinstance(slot, str) or not slot for slot in slots):
            raise ValueError("body mapping slots must be non-empty strings")
        if len(set(slots)) != len(slots):
            raise ValueError("a sensor slot cannot be assigned to multiple body roles")
        if self.status != "CONFIGURED":
            raise ValueError("body mapping status must be CONFIGURED")
        configured_slots = set(slots)
        if set(self.slot_transport_keys) - configured_slots:
            raise ValueError("transport keys must belong to mapped logical slots")
        if len(set(self.slot_transport_keys.values())) != len(self.slot_transport_keys):
            raise ValueError("one transport key cannot bind multiple logical slots")
        if set(self.physical_labels) - configured_slots:
            raise ValueError("physical labels must belong to mapped logical slots")
        if any(not isinstance(label, str) or not label for label in self.physical_labels.values()):
            raise ValueError("physical sensor labels must be non-empty strings")

    @property
    def registry_bindings(self) -> dict[TransportKey, str]:
        return {key: slot for slot, key in self.slot_transport_keys.items()}

    @property
    def required_roles(self) -> tuple[str, ...]:
        return self.profile.roles

    def require_complete(self) -> None:
        missing = set(self.required_roles) - set(self.role_to_slot)
        if missing:
            raise ValueError(
                f"{self.profile.value} body mapping is missing roles: "
                + ", ".join(sorted(missing))
            )

    def require_profile(self, profile: BodyProfile | str) -> None:
        expected = body_profile(profile)
        if self.profile is not expected:
            raise ValueError(
                f"body mapping profile is {self.profile.value}, expected {expected.value}"
            )
        self.require_complete()

    def require_upper_body(self) -> None:
        self.require_profile(BodyProfile.UPPER)

    def require_full_body(self) -> None:
        self.require_profile(BodyProfile.FULL)

    def assign(self, frame: SuitFrame) -> dict[str, TimedSensorSample]:
        self.require_complete()
        if frame.profile != self.profile.value:
            raise ValueError(
                f"suit frame profile is {frame.profile}, expected {self.profile.value}"
            )
        missing = [role for role, slot in self.role_to_slot.items() if slot not in frame.samples]
        if missing:
            raise ValueError(f"suit frame is missing mapped roles: {', '.join(sorted(missing))}")
        stale = [role for role, slot in self.role_to_slot.items() if slot in frame.stale_slots]
        if stale:
            raise ValueError(f"suit frame has stale mapped roles: {', '.join(sorted(stale))}")
        return {role: frame.samples[slot] for role, slot in self.role_to_slot.items()}

    def save(self, path: str | Path) -> None:
        logical_slots = {}
        for slot in sorted(set(self.role_to_slot.values())):
            entry: dict[str, object] = {}
            key = self.slot_transport_keys.get(slot)
            if key is not None:
                entry["transport_key"] = {
                    "kind": key.kind,
                    "value": key.value,
                    "evidence_status": key.evidence_status,
                }
            if slot in self.physical_labels:
                entry["physical_label"] = self.physical_labels[slot]
            if entry:
                logical_slots[slot] = entry
        data: dict[str, object] = {
            "version": 1,
            "profile": self.profile.value,
            "status": self.status,
            "body_mapping": {
                role: self.role_to_slot[role]
                for role in self.required_roles
                if role in self.role_to_slot
            },
        }
        if logical_slots:
            data["logical_slots"] = logical_slots
        destination = Path(path)
        with destination.open("x", encoding="utf-8") as stream:
            yaml.safe_dump(data, stream, sort_keys=False)

    @classmethod
    def load(cls, path: str | Path) -> "BodySensorMap":
        try:
            data = yaml.load(
                Path(path).read_text(encoding="utf-8"), Loader=_UniqueSafeLoader
            )
            if not isinstance(data, dict) or set(data) - {
                "version",
                "profile",
                "status",
                "body_mapping",
                "logical_slots",
            }:
                raise ValueError("unknown body-mapping fields")
            if data.get("version") != 1:
                raise ValueError("body-mapping version must be 1")
            profile = body_profile(str(data.get("profile", BodyProfile.UPPER.value)))
            mapping = data.get("body_mapping")
            if not isinstance(mapping, dict):
                raise ValueError("body_mapping must be a mapping")
            logical_slots = data.get("logical_slots", {})
            if not isinstance(logical_slots, dict):
                raise ValueError("logical_slots must be a mapping")
            transport_keys = {}
            physical_labels = {}
            for slot, entry in logical_slots.items():
                if not isinstance(slot, str) or not isinstance(entry, dict) or set(entry) - {
                    "transport_key",
                    "physical_label",
                }:
                    raise ValueError(f"invalid logical slot entry: {slot}")
                if "transport_key" in entry:
                    key = entry["transport_key"]
                    if not isinstance(key, dict) or set(key) != {
                        "kind",
                        "value",
                        "evidence_status",
                    }:
                        raise ValueError(f"invalid transport key for {slot}")
                    transport_keys[slot] = TransportKey(
                        str(key["kind"]),
                        str(key["value"]),
                        str(key["evidence_status"]),
                    )
                if "physical_label" in entry:
                    physical_labels[slot] = entry["physical_label"]
            return cls(
                {str(role): slot for role, slot in mapping.items()},
                str(data.get("status", "CONFIGURED")),
                transport_keys,
                physical_labels,
                profile,
            )
        except (OSError, TypeError, yaml.YAMLError) as exc:
            raise ValueError(f"could not load body mapping {path}: {exc}") from exc


def decoded_frame_to_suit(frame: SensorFrame) -> SuitFrame:
    samples = {}
    for sensor in frame.sensors:
        slot = sensor.slot or f"slot_{sensor.sensor_id}"
        if slot in samples:
            raise ValueError(f"duplicate sensor slot in frame: {slot}")
        samples[slot] = TimedSensorSample(
            frame.timestamp_ns,
            slot,
            sensor,
            sensor.coordinate_frame,
            sensor.validation_status,
        )
    return SuitFrame(frame.timestamp_ns, samples)


def build_calibrated_suit_frame(
    frame: SuitFrame,
    mapping: BodySensorMap,
    sensor_basis: BasisTransform,
    calibration: CalibrationProfile | None,
) -> SuitFrame:
    raw_orientations = {}
    converted_orientations = {}
    orientations = {}
    ages = {}
    required_roles = mapping.required_roles
    missing = set(required_roles) - set(mapping.role_to_slot)
    stale = set()
    for role, slot in mapping.role_to_slot.items():
        timed = frame.samples.get(slot)
        if timed is None or timed.raw.quaternion is None:
            missing.add(role)
            continue
        ages[role] = max(0.0, (frame.timestamp_ns - timed.timestamp_ns) / 1_000_000)
        raw_orientations[role] = timed.raw.quaternion
        try:
            converted = sensor_basis.orientation(timed.raw.quaternion)
        except ValueError:
            missing.add(role)
            continue
        converted_orientations[role] = converted
        if slot in frame.stale_slots:
            stale.add(role)
            continue
        if calibration is None or role not in calibration.neutral_wxyz:
            missing.add(role)
            continue
        orientations[role] = calibration.apply(role, converted)
    valid = not missing and not stale and set(orientations) == set(required_roles)
    reasons = []
    if calibration is None:
        reasons.append("calibration missing")
    if missing:
        reasons.append("missing roles: " + ", ".join(sorted(missing)))
    if stale:
        reasons.append("stale roles: " + ", ".join(sorted(stale)))
    return SuitFrame(
        frame.timestamp_ns,
        frame.samples,
        frame.stale_slots,
        frame.missing_slots,
        orientations,
        raw_orientations,
        converted_orientations,
        ages,
        valid,
        tuple(sorted(missing)),
        tuple(sorted(stale)),
        "; ".join(reasons),
        mapping.profile.value,
    )


class LatestSuitBuffer:
    """Single-slot-per-sensor grouping; missing data is never fabricated."""

    def __init__(self, max_age_ns: int):
        if max_age_ns < 1:
            raise ValueError("max_age_ns must be positive")
        self.max_age_ns = max_age_ns
        self._latest: dict[str, TimedSensorSample] = {}

    def update(self, frame: SuitFrame, expected_slots: tuple[str, ...] = ()) -> SuitFrame:
        for slot, sample in frame.samples.items():
            previous = self._latest.get(slot)
            if previous is None or sample.timestamp_ns >= previous.timestamp_ns:
                self._latest[slot] = sample
        now = max(
            (frame.timestamp_ns, *(sample.timestamp_ns for sample in self._latest.values()))
        )
        stale = tuple(
            sorted(
                slot
                for slot, sample in self._latest.items()
                if now - sample.timestamp_ns > self.max_age_ns
            )
        )
        missing = tuple(sorted(set(expected_slots) - set(self._latest)))
        reason = "; ".join(
            item
            for item in (
                "missing slots: " + ", ".join(missing) if missing else "",
                "stale slots: " + ", ".join(stale) if stale else "",
            )
            if item
        )
        return SuitFrame(
            now,
            dict(self._latest),
            stale,
            missing,
            valid=not stale and not missing,
            reason=reason,
            profile=frame.profile,
        )


# Semantic alias: full mode uses the same non-fabricating frame implementation.
FullBodySuitFrame = SuitFrame
