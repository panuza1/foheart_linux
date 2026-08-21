"""Neutral-pose orientation calibration, independent of USB and robot code."""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
import math
from pathlib import Path
import statistics
from typing import Mapping, Sequence

import yaml

from .frames import normalize_quaternion
from .orientation import (
    continuity_adjusted,
    quaternion_angular_distance_degrees,
    quaternion_inverse,
    quaternion_multiply,
    quaternion_norm,
)
from .sensor import Quaternion, Vector3

CALIBRATION_ALGORITHM = "temporal_sign_continuity_normalized_wxyz_mean"


@dataclass(frozen=True)
class CalibrationObservation:
    timestamp_ns: int
    quaternion: Quaternion
    gyro: Vector3 | None = None


@dataclass(frozen=True)
class CalibrationQuality:
    role: str
    sample_count: int
    quaternion_norm_min: float
    quaternion_norm_max: float
    orientation_spread_degrees: float
    maximum_angular_deviation_degrees: float
    gyro_magnitude_mean: float
    gyro_magnitude_max: float
    timestamp_ns: int
    acceptable: bool

    def __post_init__(self) -> None:
        values = (
            self.quaternion_norm_min,
            self.quaternion_norm_max,
            self.orientation_spread_degrees,
            self.maximum_angular_deviation_degrees,
            self.gyro_magnitude_mean,
            self.gyro_magnitude_max,
        )
        if not self.role or self.sample_count < 1 or self.timestamp_ns < 0:
            raise ValueError("invalid calibration quality identity/count/timestamp")
        if not all(math.isfinite(value) and value >= 0 for value in values):
            raise ValueError("calibration quality values must be finite and non-negative")
        if self.quaternion_norm_min <= 0 or self.quaternion_norm_max < self.quaternion_norm_min:
            raise ValueError("invalid calibration quaternion norm range")
        if self.gyro_magnitude_max < self.gyro_magnitude_mean:
            raise ValueError("invalid calibration gyro magnitude range")
        if not isinstance(self.acceptable, bool):
            raise ValueError("calibration quality acceptable must be boolean")


def estimate_neutral_quaternion(
    observations: Sequence[CalibrationObservation],
) -> tuple[Quaternion, tuple[float, ...]]:
    """Average WXYZ samples after normalization and temporal q/-q continuity."""
    if not observations:
        raise ValueError("cannot estimate a quaternion from no samples")
    adjusted: list[Quaternion] = []
    for observation in observations:
        current = normalize_quaternion(observation.quaternion)
        if adjusted:
            current = continuity_adjusted(adjusted[-1], current)
        adjusted.append(current)
    values = tuple(
        statistics.fmean(items) for items in zip(*(item.values for item in adjusted))
    )
    mean = normalize_quaternion(Quaternion(values, "wxyz"))
    deviations = tuple(
        quaternion_angular_distance_degrees(mean, item) for item in adjusted
    )
    return mean, deviations


def _gyro_magnitude(gyro: Vector3 | None) -> float:
    return 0.0 if gyro is None else math.sqrt(gyro.x**2 + gyro.y**2 + gyro.z**2)


@dataclass(frozen=True)
class CalibrationProfile:
    neutral_wxyz: Mapping[str, tuple[float, float, float, float]]
    quality: Mapping[str, CalibrationQuality] = field(default_factory=dict)
    version: int = 1
    status: str = "SOFTWARE_TESTED"
    live_validated: bool = False
    algorithm: str = CALIBRATION_ALGORITHM
    created_at_utc: str = field(
        default_factory=lambda: datetime.now(timezone.utc).isoformat()
    )

    def __post_init__(self) -> None:
        if self.version != 1 or not self.neutral_wxyz:
            raise ValueError("calibration version must be 1 and contain sensors")
        if self.algorithm != CALIBRATION_ALGORITHM:
            raise ValueError(f"unsupported calibration algorithm: {self.algorithm}")
        if self.status not in {"CONFIGURED", "SOFTWARE_TESTED"}:
            raise ValueError("calibration status must be CONFIGURED or SOFTWARE_TESTED")
        if not isinstance(self.live_validated, bool):
            raise ValueError("live_validated must be boolean")
        for role, values in self.neutral_wxyz.items():
            if not role:
                raise ValueError("calibration role names cannot be empty")
            normalize_quaternion(Quaternion(tuple(values), "wxyz"))
        if set(self.quality) - set(self.neutral_wxyz):
            raise ValueError("calibration quality contains an unknown role")

    @classmethod
    def capture(cls, orientations: Mapping[str, Quaternion]) -> "CalibrationProfile":
        if not orientations:
            raise ValueError("cannot calibrate an empty orientation set")
        return cls(
            {
                role: normalize_quaternion(quaternion).values
                for role, quaternion in orientations.items()
            }
        )

    @classmethod
    def capture_window(
        cls,
        observations: Mapping[str, Sequence[CalibrationObservation]],
        *,
        minimum_samples: int = 20,
        maximum_angular_deviation_degrees: float = 3.0,
        maximum_gyro_magnitude: float = 5.0,
        reject_motion: bool = True,
    ) -> "CalibrationProfile":
        if minimum_samples < 2:
            raise ValueError("minimum calibration samples must be at least 2")
        if min(maximum_angular_deviation_degrees, maximum_gyro_magnitude) <= 0:
            raise ValueError("calibration motion limits must be positive")
        neutral: dict[str, tuple[float, float, float, float]] = {}
        quality: dict[str, CalibrationQuality] = {}
        rejected: list[str] = []
        for role, samples in observations.items():
            if len(samples) < minimum_samples:
                rejected.append(f"{role}: {len(samples)}/{minimum_samples} samples")
                continue
            mean, deviations = estimate_neutral_quaternion(samples)
            norms = [quaternion_norm(item.quaternion) for item in samples]
            gyros = [_gyro_magnitude(item.gyro) for item in samples]
            maximum_deviation = max(deviations)
            maximum_gyro = max(gyros)
            acceptable = (
                maximum_deviation <= maximum_angular_deviation_degrees
                and maximum_gyro <= maximum_gyro_magnitude
            )
            neutral[role] = mean.values
            quality[role] = CalibrationQuality(
                role,
                len(samples),
                min(norms),
                max(norms),
                math.sqrt(statistics.fmean(value * value for value in deviations)),
                maximum_deviation,
                statistics.fmean(gyros),
                maximum_gyro,
                max(item.timestamp_ns for item in samples),
                acceptable,
            )
            if not acceptable:
                rejected.append(
                    f"{role}: angular deviation {maximum_deviation:.2f} deg, "
                    f"gyro {maximum_gyro:.2f}"
                )
        if rejected and reject_motion:
            raise ValueError("neutral calibration rejected: " + "; ".join(rejected))
        if not neutral:
            raise ValueError("neutral calibration has no usable roles")
        return cls(neutral, quality)

    def require_roles(self, roles: Sequence[str], *, exact: bool = True) -> None:
        required = set(roles)
        actual = set(self.neutral_wxyz)
        missing = required - actual
        extra = actual - required if exact else set()
        if missing or extra:
            details = []
            if missing:
                details.append("missing " + ", ".join(sorted(missing)))
            if extra:
                details.append("unknown " + ", ".join(sorted(extra)))
            raise ValueError("calibration role mismatch: " + "; ".join(details))

    def apply(self, role: str, current: Quaternion) -> Quaternion:
        """Return neutral-to-current WXYZ rotation; raw input remains unchanged."""
        try:
            neutral = Quaternion(tuple(self.neutral_wxyz[role]), "wxyz")
        except KeyError as exc:
            raise KeyError(f"no calibration for body role {role!r}") from exc
        relative = quaternion_multiply(quaternion_inverse(neutral), current)
        return normalize_quaternion(relative)

    def save(self, path: str | Path) -> None:
        sensors = {}
        for role, values in sorted(self.neutral_wxyz.items()):
            entry: dict[str, object] = {
                "role": role,
                "neutral_quaternion_wxyz": list(values),
            }
            if role in self.quality:
                quality = self.quality[role]
                entry.update(
                    {
                        "sample_count": quality.sample_count,
                        "quaternion_norm_range": [
                            quality.quaternion_norm_min,
                            quality.quaternion_norm_max,
                        ],
                        "orientation_spread_degrees": quality.orientation_spread_degrees,
                        "maximum_angular_deviation_degrees": quality.maximum_angular_deviation_degrees,
                        "gyro_magnitude": {
                            "mean": quality.gyro_magnitude_mean,
                            "maximum": quality.gyro_magnitude_max,
                        },
                        "timestamp_ns": quality.timestamp_ns,
                        "acceptable": quality.acceptable,
                    }
                )
            sensors[role] = entry
        data = {
            "version": self.version,
            "status": self.status,
            "live_validated": self.live_validated,
            "algorithm": self.algorithm,
            "created_at_utc": self.created_at_utc,
            "sensors": sensors,
        }
        Path(path).write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")

    @classmethod
    def load(cls, path: str | Path) -> "CalibrationProfile":
        try:
            data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
            allowed_root = {
                "version",
                "status",
                "live_validated",
                "algorithm",
                "created_at_utc",
                "sensors",
            }
            if not isinstance(data, dict) or set(data) - allowed_root:
                raise ValueError("unknown or missing calibration fields")
            sensors = data["sensors"]
            if not isinstance(sensors, dict):
                raise ValueError("calibration sensors must be a mapping")
            neutral = {}
            quality = {}
            allowed_entry = {
                "role",
                "neutral_quaternion_wxyz",
                "sample_count",
                "quaternion_norm_range",
                "orientation_spread_degrees",
                "maximum_angular_deviation_degrees",
                "gyro_magnitude",
                "timestamp_ns",
                "acceptable",
            }
            for role, entry in sensors.items():
                if not isinstance(entry, dict) or set(entry) - allowed_entry:
                    raise ValueError(f"invalid calibration entry for {role}")
                if entry.get("role", role) != role:
                    raise ValueError(f"calibration role label mismatch for {role}")
                values = entry.get("neutral_quaternion_wxyz")
                if not isinstance(values, list) or len(values) != 4:
                    raise ValueError(f"invalid neutral quaternion for {role}")
                neutral[str(role)] = tuple(float(value) for value in values)
                if "sample_count" in entry:
                    norm_range = entry["quaternion_norm_range"]
                    gyro = entry["gyro_magnitude"]
                    acceptable = entry["acceptable"]
                    if (
                        not isinstance(norm_range, list)
                        or len(norm_range) != 2
                        or not isinstance(gyro, dict)
                        or set(gyro) != {"mean", "maximum"}
                        or not isinstance(acceptable, bool)
                    ):
                        raise ValueError(f"invalid calibration quality for {role}")
                    quality[str(role)] = CalibrationQuality(
                        str(role),
                        int(entry["sample_count"]),
                        float(norm_range[0]),
                        float(norm_range[1]),
                        float(entry["orientation_spread_degrees"]),
                        float(entry["maximum_angular_deviation_degrees"]),
                        float(gyro["mean"]),
                        float(gyro["maximum"]),
                        int(entry["timestamp_ns"]),
                        acceptable,
                    )
            live_validated = data.get("live_validated", False)
            if not isinstance(live_validated, bool):
                raise ValueError("live_validated must be boolean")
            return cls(
                neutral,
                quality,
                version=int(data["version"]),
                status=str(data.get("status", "CONFIGURED")),
                live_validated=live_validated,
                algorithm=str(data.get("algorithm", CALIBRATION_ALGORITHM)),
                created_at_utc=str(data.get("created_at_utc", "UNKNOWN")),
            )
        except (OSError, KeyError, TypeError, yaml.YAMLError) as exc:
            raise ValueError(f"could not load calibration {path}: {exc}") from exc
