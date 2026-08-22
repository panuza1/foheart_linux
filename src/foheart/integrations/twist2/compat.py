"""Isolated loader for the pinned TWIST2 MotionLib NumPy packaging bug."""

from __future__ import annotations

from contextlib import contextmanager
from pathlib import Path
import sys
from types import ModuleType
from typing import Any


_BROKEN_NUMPY_PATCHES = (
    "sys.modules['numpy._core'] = FakeModule('numpy._core', np.core if hasattr(np, 'core') else np)",
    "sys.modules['numpy._core.multiarray'] = FakeModule('numpy._core.multiarray', getattr(np.core, 'multiarray', None))",
)


@contextmanager
def _python_path(path: Path):
    value = str(path)
    sys.path.insert(0, value)
    try:
        yield
    finally:
        sys.path.remove(value)


def load_pinned_motionlib(
    motion_path: str | Path,
    twist2_root: str | Path,
    *,
    device: str = "cpu",
    **options: Any,
):
    """Construct the actual pinned class while suppressing only two unsafe aliases."""

    root = Path(twist2_root).resolve()
    source_path = root / "pose/pose/utils/motion_lib_pkl.py"
    if not source_path.is_file():
        raise FileNotFoundError(source_path)
    source = source_path.read_text(encoding="utf-8")
    for line in _BROKEN_NUMPY_PATCHES:
        if source.count(line) != 1:
            raise RuntimeError("pinned MotionLib NumPy compatibility lines changed")
        source = source.replace(line, f"# foheart isolated compatibility: {line}")

    module = ModuleType("_foheart_pinned_motion_lib_pkl")
    module.__file__ = str(source_path)
    with _python_path(root / "pose"):
        exec(compile(source, str(source_path), "exec"), module.__dict__)
        return module.MotionLib(str(Path(motion_path).resolve()), device=device, **options)
