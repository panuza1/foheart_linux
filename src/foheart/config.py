from __future__ import annotations

import argparse
from copy import deepcopy
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping

import yaml

from foheart.constants import ROUTER_PIDS

USB_MODES = ("auto", "bulk", "hid")
OUTER_FRAME_MODES = ("auto", "fixed_0x13", "raw")
SENSOR_ID_MODES = ("auto", "loop_index", "decoded_index", "unknown")
STREAM_MODES = ("read_only", "experimental")
DEFAULT_CONFIG_PATH = Path(__file__).resolve().parents[2] / "config" / "default.yaml"

_DEFAULTS: dict[str, dict[str, Any]] = {
    "usb": {
        "mode": "auto",
        "pid": None,
        "interface": None,
        "in_endpoint": None,
        "out_endpoint": None,
        "timeout_ms": 1000,
        "read_size": None,
    },
    "protocol": {"outer_frame": "auto", "sensor_id_mode": "auto"},
    "stream": {"mode": "read_only"},
    "monitor": {
        "show_raw": False,
        "show_euler": True,
        "show_quaternion": True,
        "show_imu": True,
    },
}


class ConfigError(ValueError):
    pass


@dataclass(frozen=True)
class USBConfig:
    mode: str = "auto"
    pid: int | None = None
    interface: int | None = None
    in_endpoint: int | None = None
    out_endpoint: int | None = None
    timeout_ms: int = 1000
    read_size: int | None = None


@dataclass(frozen=True)
class ProtocolConfig:
    outer_frame: str = "auto"
    sensor_id_mode: str = "auto"


@dataclass(frozen=True)
class StreamConfig:
    mode: str = "read_only"


@dataclass(frozen=True)
class MonitorConfig:
    show_raw: bool = False
    show_euler: bool = True
    show_quaternion: bool = True
    show_imu: bool = True


@dataclass(frozen=True)
class RuntimeConfig:
    usb: USBConfig = field(default_factory=USBConfig)
    protocol: ProtocolConfig = field(default_factory=ProtocolConfig)
    stream: StreamConfig = field(default_factory=StreamConfig)
    monitor: MonitorConfig = field(default_factory=MonitorConfig)


def _choice(value: Any, field_name: str, choices: tuple[str, ...]) -> str:
    if not isinstance(value, str) or value not in choices:
        raise ConfigError(f"{field_name} must be one of: {', '.join(choices)}")
    return value


def _auto_int(value: Any, field_name: str, maximum: int | None = None) -> int | None:
    if value is None or (isinstance(value, str) and value.lower() == "auto"):
        return None
    if isinstance(value, bool):
        raise ConfigError(f"{field_name} must be an integer or auto")
    try:
        parsed = value if isinstance(value, int) else int(value, 0)
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"{field_name} must be an integer or auto") from exc
    if parsed < 0 or (maximum is not None and parsed > maximum):
        limit = f"0..0x{maximum:x}" if maximum is not None else "non-negative"
        raise ConfigError(f"{field_name} must be {limit}")
    return parsed


def _positive_int(value: Any, field_name: str) -> int:
    parsed = _auto_int(value, field_name)
    if parsed is None or parsed < 1:
        raise ConfigError(f"{field_name} must be positive")
    return parsed


def _boolean(value: Any, field_name: str) -> bool:
    if not isinstance(value, bool):
        raise ConfigError(f"{field_name} must be true or false")
    return value


def _merge(target: dict[str, Any], update: Mapping[str, Any]) -> None:
    unknown_sections = set(update) - set(_DEFAULTS)
    if unknown_sections:
        raise ConfigError(f"unknown configuration section: {sorted(unknown_sections)[0]}")
    for section, values in update.items():
        if not isinstance(values, Mapping):
            raise ConfigError(f"{section} must be a mapping")
        unknown_keys = set(values) - set(_DEFAULTS[section])
        if unknown_keys:
            raise ConfigError(f"unknown configuration key: {section}.{sorted(unknown_keys)[0]}")
        target[section].update(values)


def load_config(
    path: str | Path | None = None,
    overrides: Mapping[str, Mapping[str, Any]] | None = None,
) -> RuntimeConfig:
    data = deepcopy(_DEFAULTS)
    if path is not None:
        try:
            loaded = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
        except (OSError, yaml.YAMLError) as exc:
            raise ConfigError(f"could not load config {path}: {exc}") from exc
        if not isinstance(loaded, Mapping):
            raise ConfigError("configuration root must be a mapping")
        _merge(data, loaded)
    if overrides:
        _merge(data, overrides)

    usb = data["usb"]
    pid = _auto_int(usb["pid"], "usb.pid", 0xFFFF)
    if pid is not None and pid not in ROUTER_PIDS:
        raise ConfigError(
            f"usb.pid 0x{pid:04x} is not a supported C1 router; "
            "only 0x5751 and 0x5851 are accepted"
        )
    return RuntimeConfig(
        usb=USBConfig(
            mode=_choice(usb["mode"], "usb.mode", USB_MODES),
            pid=pid,
            interface=_auto_int(usb["interface"], "usb.interface", 0xFF),
            in_endpoint=_auto_int(usb["in_endpoint"], "usb.in_endpoint", 0xFF),
            out_endpoint=_auto_int(usb["out_endpoint"], "usb.out_endpoint", 0xFF),
            timeout_ms=_positive_int(usb["timeout_ms"], "usb.timeout_ms"),
            read_size=(
                None
                if usb["read_size"] is None
                or (isinstance(usb["read_size"], str) and usb["read_size"].lower() == "auto")
                else _positive_int(usb["read_size"], "usb.read_size")
            ),
        ),
        protocol=ProtocolConfig(
            outer_frame=_choice(
                data["protocol"]["outer_frame"],
                "protocol.outer_frame",
                OUTER_FRAME_MODES,
            ),
            sensor_id_mode=_choice(
                data["protocol"]["sensor_id_mode"],
                "protocol.sensor_id_mode",
                SENSOR_ID_MODES,
            ),
        ),
        stream=StreamConfig(
            mode=_choice(data["stream"]["mode"], "stream.mode", STREAM_MODES)
        ),
        monitor=MonitorConfig(
            show_raw=_boolean(data["monitor"]["show_raw"], "monitor.show_raw"),
            show_euler=_boolean(
                data["monitor"]["show_euler"], "monitor.show_euler"
            ),
            show_quaternion=_boolean(
                data["monitor"]["show_quaternion"], "monitor.show_quaternion"
            ),
            show_imu=_boolean(data["monitor"]["show_imu"], "monitor.show_imu"),
        ),
    )


def _arg_auto_int(value: str) -> int | None:
    try:
        return _auto_int(value, "value")
    except ConfigError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc


def _arg_positive_int(value: str) -> int:
    try:
        return _positive_int(value, "value")
    except ConfigError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc


def add_config_arguments(parser: argparse.ArgumentParser) -> None:
    suppressed = argparse.SUPPRESS
    parser.add_argument("--config", type=Path)
    parser.add_argument("--usb-mode", choices=USB_MODES, default=suppressed)
    parser.add_argument("--pid", type=_arg_auto_int, default=suppressed)
    parser.add_argument("--interface", type=_arg_auto_int, default=suppressed)
    parser.add_argument("--in-endpoint", type=_arg_auto_int, default=suppressed)
    parser.add_argument("--out-endpoint", type=_arg_auto_int, default=suppressed)
    parser.add_argument("--timeout-ms", type=_arg_positive_int, default=suppressed)
    parser.add_argument("--read-size", type=_arg_auto_int, default=suppressed)
    parser.add_argument("--outer-frame", choices=OUTER_FRAME_MODES, default=suppressed)
    parser.add_argument("--sensor-id-mode", choices=SENSOR_ID_MODES, default=suppressed)
    parser.add_argument("--stream-mode", choices=STREAM_MODES, default=suppressed)


def load_config_from_args(args: argparse.Namespace) -> RuntimeConfig:
    fields = {
        "usb": (
            ("usb_mode", "mode"),
            ("pid", "pid"),
            ("interface", "interface"),
            ("in_endpoint", "in_endpoint"),
            ("out_endpoint", "out_endpoint"),
            ("timeout_ms", "timeout_ms"),
            ("read_size", "read_size"),
        ),
        "protocol": (
            ("outer_frame", "outer_frame"),
            ("sensor_id_mode", "sensor_id_mode"),
        ),
        "stream": (("stream_mode", "mode"),),
        "monitor": (
            ("show_raw", "show_raw"),
            ("show_euler", "show_euler"),
            ("show_quaternion", "show_quaternion"),
            ("show_imu", "show_imu"),
        ),
    }
    overrides: dict[str, dict[str, Any]] = {}
    for section, names in fields.items():
        values = {
            config_name: getattr(args, cli_name)
            for cli_name, config_name in names
            if hasattr(args, cli_name)
        }
        if values:
            overrides[section] = values
    return load_config(getattr(args, "config", None), overrides)


def require_read_only(config: RuntimeConfig) -> None:
    if config.stream.mode != "read_only":
        raise ConfigError(
            "Experimental USB write path is disabled because\n"
            "the C1 start-stream command is not validated."
        )
