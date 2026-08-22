"""TWIST2 MotionLib pickle recording without changing its six-key schema."""

from __future__ import annotations

from pathlib import Path
import pickle
from typing import Any, Mapping, Sequence

import numpy as np

from foheart.whole_body.gmr import G1_LINK_BODY_NAMES


MOTION_KEYS = frozenset(
    {"fps", "root_pos", "root_rot", "dof_pos", "local_body_pos", "link_body_list"}
)


def validate_motion(data: Mapping[str, Any]) -> dict[str, Any]:
    """Validate and copy the exact schema read by pinned TWIST2 MotionLib."""

    if set(data) != MOTION_KEYS:
        missing = sorted(MOTION_KEYS - set(data))
        extra = sorted(set(data) - MOTION_KEYS)
        raise ValueError(f"motion keys mismatch; missing={missing}, extra={extra}")

    fps = float(data["fps"])
    if not np.isfinite(fps) or fps <= 0:
        raise ValueError("fps must be finite and positive")

    arrays = {
        name: np.asarray(data[name], dtype=float)
        for name in ("root_pos", "root_rot", "dof_pos", "local_body_pos")
    }
    root_pos, root_rot = arrays["root_pos"], arrays["root_rot"]
    dof_pos, local_body_pos = arrays["dof_pos"], arrays["local_body_pos"]
    if root_pos.ndim != 2 or root_pos.shape[1] != 3:
        raise ValueError("root_pos must have shape (T, 3)")
    frames = root_pos.shape[0]
    if frames < 2:
        raise ValueError("motion needs at least two frames for MotionLib gradients")
    if root_rot.shape != (frames, 4):
        raise ValueError("root_rot must have shape (T, 4)")
    if dof_pos.shape != (frames, 29):
        raise ValueError("dof_pos must have shape (T, 29)")
    if local_body_pos.ndim != 3 or local_body_pos.shape[0] != frames or local_body_pos.shape[2] != 3:
        raise ValueError("local_body_pos must have shape (T, B, 3)")
    if not all(np.isfinite(value).all() for value in arrays.values()):
        raise ValueError("motion arrays must be finite")

    quat_norm = np.linalg.norm(root_rot, axis=1)
    if np.any(quat_norm < 1e-8) or not np.allclose(quat_norm, 1.0, atol=1e-5):
        raise ValueError("root_rot XYZW quaternions must be normalized")

    links = list(data["link_body_list"])
    if len(links) != local_body_pos.shape[1] or not links:
        raise ValueError("link_body_list must match local_body_pos body count")
    if any(not isinstance(name, str) or not name for name in links) or len(set(links)) != len(links):
        raise ValueError("link_body_list names must be non-empty and unique")

    return {"fps": fps, **arrays, "link_body_list": links}


def load_motion(path: str | Path) -> dict[str, Any]:
    """Load a trusted upstream-compatible pickle and validate it."""

    with Path(path).open("rb") as stream:
        value = pickle.load(stream)
    if not isinstance(value, Mapping):
        raise ValueError("motion pickle must contain a mapping")
    return validate_motion(value)


def save_motion(path: str | Path, data: Mapping[str, Any]) -> Path:
    """Save once; remove a partial file if serialization fails."""

    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    checked = validate_motion(data)
    stream = destination.open("xb")
    try:
        with stream:
            pickle.dump(checked, stream, protocol=pickle.HIGHEST_PROTOCOL)
    except BaseException:
        destination.unlink(missing_ok=True)
        raise
    return destination


def dof_velocity(dof_pos: Any, fps: float) -> np.ndarray:
    """Match MotionLib's time-gradient convention without storing an extra key."""

    values = np.asarray(dof_pos, dtype=float)
    rate = float(fps)
    if values.ndim != 2 or values.shape[1] != 29 or values.shape[0] == 0:
        raise ValueError("dof_pos must have non-empty shape (T, 29)")
    if not np.isfinite(values).all() or not np.isfinite(rate) or rate <= 0:
        raise ValueError("dof_pos and fps must be finite; fps must be positive")
    if values.shape[0] == 1:
        return np.zeros_like(values)
    return np.gradient(values, 1.0 / rate, axis=0)


class MotionRecorder:
    """Collect GMR qpos plus source-verified robot-local forward kinematics."""

    def __init__(
        self,
        *,
        fps: float,
        link_body_list: Sequence[str] = G1_LINK_BODY_NAMES,
    ):
        self.fps = float(fps)
        self.links = list(link_body_list)
        if not np.isfinite(self.fps) or self.fps <= 0:
            raise ValueError("fps must be finite and positive")
        if not self.links or any(not isinstance(name, str) or not name for name in self.links):
            raise ValueError("link_body_list must contain non-empty names")
        if len(set(self.links)) != len(self.links):
            raise ValueError("link_body_list contains duplicates")
        if tuple(self.links) != G1_LINK_BODY_NAMES:
            raise ValueError("link_body_list must match the pinned GMR/TWIST2 G1 body order")
        self._qpos: list[np.ndarray] = []
        self._local_body_pos: list[np.ndarray] = []
        self._timestamps_ns: list[int] = []
        self._source_frames: list[int] = []

    def append(
        self,
        qpos_wxyz: Any,
        local_body_pos: Any,
        *,
        timestamp_ns: int,
        source_frame_number: int,
    ) -> None:
        qpos = np.asarray(qpos_wxyz, dtype=float)
        body_pos = np.asarray(local_body_pos, dtype=float)
        if qpos.shape != (36,) or not np.isfinite(qpos).all():
            raise ValueError("GMR qpos must be a finite length-36 vector")
        if body_pos.shape != (len(self.links), 3) or not np.isfinite(body_pos).all():
            raise ValueError("local_body_pos shape must match link_body_list")
        timestamp, frame = int(timestamp_ns), int(source_frame_number)
        if timestamp < 0 or not 0 <= frame <= 0xFFFFFFFF:
            raise ValueError("timestamp/frame number is out of range")
        if self._timestamps_ns and timestamp <= self._timestamps_ns[-1]:
            raise ValueError("source timestamps must increase")
        if self._source_frames:
            delta = (frame - self._source_frames[-1]) & 0xFFFFFFFF
            if delta == 0 or delta >= 0x80000000:
                raise ValueError("source frame numbers must increase modulo uint32")
        quat = qpos[3:7]
        norm = float(np.linalg.norm(quat))
        if norm < 1e-8:
            raise ValueError("GMR root quaternion is invalid")
        qpos = qpos.copy()
        qpos[3:7] = quat / norm
        self._qpos.append(qpos)
        self._local_body_pos.append(body_pos.copy())
        self._timestamps_ns.append(timestamp)
        self._source_frames.append(frame)

    def build(self) -> dict[str, Any]:
        if not self._qpos:
            raise ValueError("cannot build an empty recording")
        qpos = np.stack(self._qpos)
        root_wxyz = qpos[:, 3:7]
        root_xyzw = root_wxyz[:, [1, 2, 3, 0]]
        return validate_motion(
            {
                "fps": self.fps,
                "root_pos": qpos[:, :3],
                "root_rot": root_xyzw,
                "dof_pos": qpos[:, 7:36],
                "local_body_pos": np.stack(self._local_body_pos),
                "link_body_list": self.links,
            }
        )

    def save(self, path: str | Path) -> Path:
        return save_motion(path, self.build())

    def __len__(self) -> int:
        return len(self._qpos)
