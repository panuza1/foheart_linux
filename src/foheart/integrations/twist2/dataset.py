"""Thin helpers for the YAML schema consumed by pinned TWIST2 MotionLib."""

from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
import tempfile
from typing import Any, Mapping, Sequence

import yaml

from .motion import load_motion


@dataclass(frozen=True)
class DatasetEntry:
    file: str
    weight: float
    description: str | None = None


def load_dataset(path: str | Path, *, validate_files: bool = True) -> tuple[Path, list[DatasetEntry]]:
    source = Path(path)
    try:
        data = yaml.safe_load(source.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f"cannot load TWIST2 dataset YAML: {exc}") from exc
    if not isinstance(data, Mapping) or set(data) != {"root_path", "motions"}:
        raise ValueError("dataset YAML must contain exactly root_path and motions")
    root_value = data["root_path"]
    if not isinstance(root_value, str) or not root_value:
        raise ValueError("dataset root_path must be a non-empty string")
    root = Path(root_value)
    motions = data["motions"]
    if not isinstance(motions, list) or not motions:
        raise ValueError("dataset motions must be a non-empty list")

    entries: list[DatasetEntry] = []
    resolved: set[Path] = set()
    for value in motions:
        if not isinstance(value, Mapping) or "file" not in value or "weight" not in value:
            raise ValueError("each motion requires file and weight")
        unknown = set(value) - {"file", "weight", "description"}
        if unknown:
            raise ValueError(f"unsupported motion fields: {sorted(unknown)}")
        file = value["file"]
        if not isinstance(file, str) or not file:
            raise ValueError("motion file must be a non-empty string")
        weight = float(value["weight"])
        if not weight >= 0 or not weight < float("inf"):
            raise ValueError("motion weight must be finite and nonnegative")
        description = value.get("description")
        if description is not None and not isinstance(description, str):
            raise ValueError("motion description must be a string")
        full_path = Path(os.path.join(str(root), file)).resolve()
        if full_path in resolved:
            raise ValueError(f"duplicate motion entry: {file}")
        resolved.add(full_path)
        if validate_files:
            load_motion(full_path)
        entries.append(DatasetEntry(file, weight, description))
    if not any(entry.weight > 0 for entry in entries):
        raise ValueError("dataset motion weights must have a positive total")
    return root, entries


def add_motion(
    dataset_path: str | Path,
    motion_path: str | Path,
    *,
    weight: float,
    description: str | None = None,
) -> DatasetEntry:
    """Append one validated entry atomically, preserving the upstream schema."""

    yaml_path = Path(dataset_path)
    root, entries = load_dataset(yaml_path, validate_files=False)
    motion = Path(motion_path).resolve()
    load_motion(motion)
    try:
        relative = motion.relative_to(root.resolve()).as_posix()
    except ValueError as exc:
        raise ValueError("motion file must be inside dataset root_path") from exc
    value = DatasetEntry(relative, float(weight), description)
    if not value.weight >= 0 or not value.weight < float("inf"):
        raise ValueError("motion weight must be finite and nonnegative")
    if description is not None and not isinstance(description, str):
        raise ValueError("motion description must be a string")
    if any(Path(os.path.join(str(root), entry.file)).resolve() == motion for entry in entries):
        raise ValueError(f"motion already exists in dataset: {relative}")

    output: dict[str, Any] = {
        "root_path": str(root),
        "motions": [
            {"file": entry.file, "weight": entry.weight, **({"description": entry.description} if entry.description is not None else {})}
            for entry in [*entries, value]
        ],
    }
    yaml_path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{yaml_path.name}.", dir=yaml_path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            yaml.safe_dump(output, stream, sort_keys=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, yaml_path)
    except Exception:
        try:
            os.close(descriptor)
        except OSError:
            pass
        Path(temporary).unlink(missing_ok=True)
        raise
    return value


def create_dataset(
    dataset_path: str | Path,
    root_path: str | Path,
    motions: Sequence[str | Path],
    *,
    weights: Sequence[float] | None = None,
) -> Path:
    """Create one upstream-compatible catalogue without absolute developer paths."""

    destination = Path(dataset_path)
    root = Path(root_path).resolve()
    files = [Path(path).resolve() for path in motions]
    values = [1.0] * len(files) if weights is None else [float(value) for value in weights]
    if not files or len(values) != len(files):
        raise ValueError("motions and weights must be non-empty and have equal length")

    entries = []
    seen: set[Path] = set()
    for motion, weight in zip(files, values):
        if motion in seen:
            raise ValueError(f"duplicate motion entry: {motion}")
        seen.add(motion)
        try:
            relative = motion.relative_to(root).as_posix()
        except ValueError as exc:
            raise ValueError("motion file must be inside dataset root_path") from exc
        if not weight >= 0 or not weight < float("inf"):
            raise ValueError("motion weight must be finite and nonnegative")
        load_motion(motion)
        entries.append({"file": relative, "weight": weight})
    if not any(entry["weight"] > 0 for entry in entries):
        raise ValueError("dataset motion weights must have a positive total")

    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("x", encoding="utf-8") as stream:
        yaml.safe_dump({"root_path": str(root), "motions": entries}, stream, sort_keys=False)
    return destination
