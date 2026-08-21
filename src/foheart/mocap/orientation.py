from __future__ import annotations

import math

from .sensor import Quaternion, Vector3


def _values(quaternion: Quaternion) -> tuple[float, float, float, float]:
    values = quaternion.values
    if len(values) != 4 or not all(math.isfinite(value) for value in values):
        raise ValueError("quaternion must contain four finite WXYZ values")
    return values


def quaternion_norm(quaternion: Quaternion) -> float:
    return math.sqrt(sum(value * value for value in _values(quaternion)))


def quaternion_conjugate(quaternion: Quaternion) -> Quaternion:
    w, x, y, z = _values(quaternion)
    return Quaternion((w, -x, -y, -z), component_order="wxyz")


def quaternion_inverse(quaternion: Quaternion) -> Quaternion:
    norm_squared = sum(value * value for value in _values(quaternion))
    if norm_squared == 0:
        raise ValueError("zero quaternion has no inverse")
    return Quaternion(
        tuple(value / norm_squared for value in quaternion_conjugate(quaternion).values),
        component_order="wxyz",
    )


def quaternion_multiply(left: Quaternion, right: Quaternion) -> Quaternion:
    aw, ax, ay, az = _values(left)
    bw, bx, by, bz = _values(right)
    return Quaternion(
        (
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ),
        component_order="wxyz",
    )


def relative_quaternion(start: Quaternion, end: Quaternion) -> Quaternion:
    """Return the algebraic WXYZ delta ``inverse(start) * end``."""
    return quaternion_multiply(quaternion_inverse(start), end)


def continuity_adjusted(previous: Quaternion, current: Quaternion) -> Quaternion:
    """Choose current or -current without changing the captured quaternion."""
    if sum(a * b for a, b in zip(_values(previous), _values(current))) >= 0:
        return Quaternion(tuple(current.values), component_order="wxyz")
    return Quaternion(tuple(-value for value in current.values), component_order="wxyz")


def shortest_quaternion_distance(left: Quaternion, right: Quaternion) -> float:
    """Euclidean component distance with the q/-q equivalence respected."""
    a = _values(left)
    b = _values(right)
    return min(
        math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b))),
        math.sqrt(sum((x + y) ** 2 for x, y in zip(a, b))),
    )


def quaternion_angular_distance_degrees(left: Quaternion, right: Quaternion) -> float:
    """Shortest orientation angle; inputs are scaled for math but not mutated."""
    left_norm = quaternion_norm(left)
    right_norm = quaternion_norm(right)
    if left_norm == 0 or right_norm == 0:
        raise ValueError("zero quaternion has no orientation")
    dot = abs(sum(a * b for a, b in zip(_values(left), _values(right))))
    return math.degrees(2 * math.acos(max(-1.0, min(1.0, dot / (left_norm * right_norm)))))


def quaternion_to_axis_angle(quaternion: Quaternion) -> tuple[Vector3, float]:
    """Return the shortest QX/QY/QZ axis and angle in degrees."""
    norm = quaternion_norm(quaternion)
    if norm == 0:
        raise ValueError("zero quaternion has no orientation")
    w, x, y, z = (value / norm for value in _values(quaternion))
    if w < 0:
        w, x, y, z = -w, -x, -y, -z
    angle = 2 * math.acos(max(-1.0, min(1.0, w)))
    vector_norm = math.sqrt(x * x + y * y + z * z)
    if vector_norm < 1e-12:
        return Vector3(1.0, 0.0, 0.0), 0.0
    return Vector3(x / vector_norm, y / vector_norm, z / vector_norm), math.degrees(angle)
