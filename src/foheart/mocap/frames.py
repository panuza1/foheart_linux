"""Small, explicit rotation helpers for WXYZ sensor and body frames."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .sensor import EVIDENCE_STATES, Quaternion, Vector3


def normalize_quaternion(quaternion: Quaternion) -> Quaternion:
    """Return a normalized derived value without modifying the raw sample."""
    values = np.asarray(quaternion.values, dtype=float)
    if values.shape != (4,) or not np.isfinite(values).all():
        raise ValueError("quaternion must contain four finite WXYZ values")
    norm = float(np.linalg.norm(values))
    if norm < 1e-12:
        raise ValueError("zero quaternion has no orientation")
    return Quaternion(tuple(map(float, values / norm)), "wxyz")


def quaternion_to_matrix(quaternion: Quaternion) -> np.ndarray:
    w, x, y, z = normalize_quaternion(quaternion).values
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def require_rotation_matrix(matrix: np.ndarray, *, name: str = "rotation") -> np.ndarray:
    rotation = np.asarray(matrix, dtype=float)
    if rotation.shape != (3, 3) or not np.isfinite(rotation).all():
        raise ValueError(f"{name} must be a finite 3x3 matrix")
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-7):
        raise ValueError(f"{name} must be orthonormal")
    if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1e-7):
        raise ValueError(f"{name} must be a proper right-handed rotation")
    return rotation


def matrix_to_quaternion(matrix: np.ndarray) -> Quaternion:
    rotation = require_rotation_matrix(matrix)
    r = rotation
    trace = float(np.trace(r))
    if trace > 0:
        scale = math.sqrt(trace + 1.0) * 2
        quaternion = np.array(
            (0.25 * scale, (r[2, 1] - r[1, 2]) / scale, (r[0, 2] - r[2, 0]) / scale, (r[1, 0] - r[0, 1]) / scale)
        )
    else:
        index = int(np.argmax(np.diag(r)))
        if index == 0:
            scale = math.sqrt(1 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
            quaternion = np.array(((r[2, 1] - r[1, 2]) / scale, 0.25 * scale, (r[0, 1] + r[1, 0]) / scale, (r[0, 2] + r[2, 0]) / scale))
        elif index == 1:
            scale = math.sqrt(1 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
            quaternion = np.array(((r[0, 2] - r[2, 0]) / scale, (r[0, 1] + r[1, 0]) / scale, 0.25 * scale, (r[1, 2] + r[2, 1]) / scale))
        else:
            scale = math.sqrt(1 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
            quaternion = np.array(((r[1, 0] - r[0, 1]) / scale, (r[0, 2] + r[2, 0]) / scale, (r[1, 2] + r[2, 1]) / scale, 0.25 * scale))
    quaternion /= np.linalg.norm(quaternion)
    if quaternion[0] < 0:
        quaternion *= -1
    return Quaternion(tuple(map(float, quaternion)), "wxyz")


def axis_rotation(axis: str, degrees: float) -> np.ndarray:
    angle = math.radians(degrees)
    c, s = math.cos(angle), math.sin(angle)
    matrices = {
        "x": np.array(((1, 0, 0), (0, c, -s), (0, s, c)), dtype=float),
        "y": np.array(((c, 0, s), (0, 1, 0), (-s, 0, c)), dtype=float),
        "z": np.array(((c, -s, 0), (s, c, 0), (0, 0, 1)), dtype=float),
    }
    try:
        return matrices[axis.lower()]
    except KeyError as exc:
        raise ValueError("axis must be x, y, or z") from exc


@dataclass(frozen=True)
class BasisTransform:
    """Coordinates in target = ``matrix @ coordinates in source``."""

    matrix: tuple[tuple[float, float, float], ...]
    source_frame: str
    target_frame: str
    status: str = "CONFIGURED"

    def __post_init__(self) -> None:
        require_rotation_matrix(np.asarray(self.matrix), name="basis transform")
        if self.status not in EVIDENCE_STATES:
            raise ValueError(f"unsupported evidence status: {self.status}")

    @classmethod
    def identity(cls, source_frame: str, target_frame: str) -> "BasisTransform":
        return cls(((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)), source_frame, target_frame)

    @classmethod
    def from_axis_map(
        cls,
        order: tuple[str, str, str],
        signs: tuple[int, int, int],
        source_frame: str,
        target_frame: str,
        *,
        status: str = "CONFIGURED",
    ) -> "BasisTransform":
        if sorted(axis.lower() for axis in order) != ["x", "y", "z"]:
            raise ValueError("axis order must be a permutation of x, y, z")
        if any(sign not in (-1, 1) for sign in signs):
            raise ValueError("axis signs must each be -1 or +1")
        rows = []
        for axis, sign in zip(order, signs):
            row = [0.0, 0.0, 0.0]
            row["xyz".index(axis.lower())] = float(sign)
            rows.append(tuple(row))
        return cls(tuple(rows), source_frame, target_frame, status)

    def vector(self, value: Vector3) -> Vector3:
        transformed = np.asarray(self.matrix) @ np.array((value.x, value.y, value.z))
        return Vector3(*map(float, transformed))

    def orientation(self, quaternion: Quaternion) -> Quaternion:
        basis = np.asarray(self.matrix)
        return matrix_to_quaternion(basis @ quaternion_to_matrix(quaternion) @ basis.T)


def homogeneous(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = require_rotation_matrix(rotation)
    position = np.asarray(translation, dtype=float)
    if position.shape != (3,) or not np.isfinite(position).all():
        raise ValueError("translation must be a finite length-3 vector")
    result[:3, 3] = position
    return result


def slerp(left: Quaternion, right: Quaternion, amount: float) -> Quaternion:
    if not 0 <= amount <= 1:
        raise ValueError("SLERP amount must be between 0 and 1")
    a = np.asarray(normalize_quaternion(left).values)
    b = np.asarray(normalize_quaternion(right).values)
    dot = float(a @ b)
    if dot < 0:
        b, dot = -b, -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        value = a + amount * (b - a)
        value /= np.linalg.norm(value)
    else:
        angle = math.acos(dot)
        value = (math.sin((1 - amount) * angle) * a + math.sin(amount * angle) * b) / math.sin(angle)
    return Quaternion(tuple(map(float, value)), "wxyz")
