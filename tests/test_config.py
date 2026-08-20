import argparse

import pytest

from foheart.config import (
    DEFAULT_CONFIG_PATH,
    ConfigError,
    add_config_arguments,
    load_config,
    load_config_from_args,
    require_read_only,
)
from foheart.protocol.parser import (
    BULK_HS_FIXED_MESSAGE_SIZE,
    resolve_outer_frame,
)


def test_default_config_file_matches_runtime_defaults():
    assert load_config(DEFAULT_CONFIG_PATH) == load_config()


def test_yaml_loading(tmp_path):
    path = tmp_path / "config.yaml"
    path.write_text(
        "usb:\n  mode: bulk\n  timeout_ms: 250\n"
        "protocol:\n  outer_frame: raw\n",
        encoding="utf-8",
    )
    config = load_config(path)
    assert config.usb.mode == "bulk"
    assert config.usb.timeout_ms == 250
    assert config.protocol.outer_frame == "raw"


def test_cli_overrides_config_file_and_can_restore_auto(tmp_path):
    path = tmp_path / "config.yaml"
    path.write_text(
        "usb:\n  mode: bulk\n  pid: 0x5751\n"
        "protocol:\n  outer_frame: fixed_0x13\n",
        encoding="utf-8",
    )
    parser = argparse.ArgumentParser()
    add_config_arguments(parser)
    args = parser.parse_args(
        [
            "--config",
            str(path),
            "--usb-mode",
            "hid",
            "--pid",
            "auto",
            "--outer-frame",
            "raw",
        ]
    )
    config = load_config_from_args(args)
    assert config.usb.mode == "hid"
    assert config.usb.pid is None
    assert config.protocol.outer_frame == "raw"


def test_hex_pid_and_endpoint_cli_values():
    parser = argparse.ArgumentParser()
    add_config_arguments(parser)
    config = load_config_from_args(
        parser.parse_args(
            [
                "--pid",
                "0x5751",
                "--interface",
                "0x0",
                "--in-endpoint",
                "0x81",
                "--out-endpoint",
                "0x01",
                "--read-size",
                "0x1400",
            ]
        )
    )
    assert config.usb.pid == 0x5751
    assert config.usb.interface == 0
    assert config.usb.in_endpoint == 0x81
    assert config.usb.out_endpoint == 0x01
    assert config.usb.read_size == 0x1400


@pytest.mark.parametrize(
    ("section", "key", "value"),
    [
        ("usb", "mode", "serial"),
        ("protocol", "outer_frame", "invented"),
        ("protocol", "sensor_id_mode", "physical_id"),
    ],
)
def test_invalid_modes_are_rejected(tmp_path, section, key, value):
    path = tmp_path / "invalid.yaml"
    path.write_text(f"{section}:\n  {key}: {value}\n", encoding="utf-8")
    with pytest.raises(ConfigError):
        load_config(path)


def test_unrelated_foheart_pid_is_rejected():
    with pytest.raises(ConfigError, match="not a supported C1 router"):
        load_config(overrides={"usb": {"pid": 0x5752}})


def test_outer_frame_auto_and_safe_raw_fallback():
    fixed = bytearray(BULK_HS_FIXED_MESSAGE_SIZE)
    fixed[0] = 0x13
    fixed[5] = 0
    assert resolve_outer_frame(bytes(fixed), "auto") == "fixed_0x13"
    fixed[5] = 1
    assert resolve_outer_frame(bytes(fixed), "auto") == "raw"
    assert resolve_outer_frame(b"unknown", "auto") == "raw"
    assert resolve_outer_frame(b"unknown", "fixed_0x13") == "raw"
    assert resolve_outer_frame(bytes(fixed), "raw") == "raw"


def test_experimental_stream_mode_is_refused():
    config = load_config(overrides={"stream": {"mode": "experimental"}})
    with pytest.raises(ConfigError, match="start-stream command is not validated"):
        require_read_only(config)
