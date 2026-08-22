from pathlib import Path

import numpy as np
import pytest
import yaml

from foheart.integrations.twist2.dataset import add_motion, create_dataset, load_dataset
from foheart.integrations.twist2.motion import save_motion
from foheart.tools.g1_motion_dataset import main as dataset_main


def _motion(path: Path) -> None:
    save_motion(
        path,
        {
            "fps": 50,
            "root_pos": np.zeros((2, 3)),
            "root_rot": np.tile((0.0, 0.0, 0.0, 1.0), (2, 1)),
            "dof_pos": np.zeros((2, 29)),
            "local_body_pos": np.zeros((2, 2, 3)),
            "link_body_list": ["pelvis", "torso"],
        },
    )


def test_dataset_add_validate_description_and_duplicate(tmp_path):
    root = tmp_path / "motions"
    root.mkdir()
    first, second = root / "first.pkl", root / "second.pkl"
    _motion(first)
    _motion(second)
    catalogue = tmp_path / "dataset.yaml"
    catalogue.write_text(
        yaml.safe_dump(
            {"root_path": str(root), "motions": [{"file": "first.pkl", "weight": 1.0, "description": "first motion"}]},
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    added = add_motion(catalogue, second, weight=0.5, description="suit replay")
    assert added.file == "second.pkl"
    parsed_root, entries = load_dataset(catalogue)
    assert parsed_root == root
    assert [(entry.file, entry.weight, entry.description) for entry in entries] == [
        ("first.pkl", 1.0, "first motion"),
        ("second.pkl", 0.5, "suit replay"),
    ]
    with pytest.raises(ValueError, match="already exists"):
        add_motion(catalogue, second, weight=1.0)


def test_dataset_rejects_missing_file_negative_weight_and_outside_root(tmp_path):
    root = tmp_path / "motions"
    root.mkdir()
    catalogue = tmp_path / "dataset.yaml"
    catalogue.write_text(
        yaml.safe_dump({"root_path": str(root), "motions": [{"file": "missing.pkl", "weight": 1.0}]}),
        encoding="utf-8",
    )
    with pytest.raises(FileNotFoundError):
        load_dataset(catalogue)

    outside = tmp_path / "outside.pkl"
    _motion(outside)
    with pytest.raises(ValueError, match="inside"):
        add_motion(catalogue, outside, weight=1.0)

    catalogue.write_text(
        yaml.safe_dump({"root_path": str(root), "motions": [{"file": "missing.pkl", "weight": -1.0}]}),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="nonnegative"):
        load_dataset(catalogue, validate_files=False)


def test_dataset_rejects_zero_total_weight(tmp_path):
    root = tmp_path / "motions"
    root.mkdir()
    first = root / "first.pkl"
    _motion(first)
    catalogue = tmp_path / "dataset.yaml"
    catalogue.write_text(
        yaml.safe_dump({"root_path": str(root), "motions": [{"file": "first.pkl", "weight": 0.0}]}),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="positive total"):
        load_dataset(catalogue)


def test_create_dataset_supports_one_multiple_and_rejects_duplicates(tmp_path):
    root = tmp_path / "motions"
    root.mkdir()
    first, second = root / "first.pkl", root / "second.pkl"
    _motion(first)
    _motion(second)

    one = create_dataset(tmp_path / "one.yaml", root, [first])
    assert [entry.file for entry in load_dataset(one)[1]] == ["first.pkl"]
    multiple = create_dataset(tmp_path / "multiple.yaml", root, [first, second], weights=[1, 0.5])
    assert [entry.weight for entry in load_dataset(multiple)[1]] == [1, 0.5]
    with pytest.raises(ValueError, match="duplicate"):
        create_dataset(tmp_path / "duplicate.yaml", root, [first, first])
    with pytest.raises(ValueError, match="nonnegative"):
        create_dataset(tmp_path / "weight.yaml", root, [first], weights=[-1])
    with pytest.raises(FileNotFoundError):
        create_dataset(tmp_path / "missing.yaml", root, [root / "missing.pkl"])
    with pytest.raises(FileExistsError):
        create_dataset(one, root, [first])


def test_dataset_cli_uses_configurable_root(tmp_path):
    root = tmp_path / "motions"
    root.mkdir()
    motion = root / "motion.pkl"
    _motion(motion)
    output = tmp_path / "dataset.yaml"
    assert dataset_main([
        "--root", str(root), "--output", str(output), "--weight", "0.75", str(motion)
    ]) == 0
    parsed_root, entries = load_dataset(output)
    assert parsed_root == root.resolve()
    assert [(entry.file, entry.weight) for entry in entries] == [("motion.pkl", 0.75)]
