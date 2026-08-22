from __future__ import annotations

import argparse
from copy import deepcopy
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping

import numpy as np
import yaml

from foheart.constants import ROUTER_PIDS
from foheart.mocap.suit import FULL_BODY_ROLES, BodyProfile, roles_for_profile

USB_MODES = ("auto", "bulk", "hid")
OUTER_FRAME_MODES = ("auto", "fixed_0x13", "raw")
SENSOR_ID_MODES = ("auto", "loop_index", "decoded_index", "unknown")
STREAM_MODES = ("read_only", "experimental")
FRAME_STATUSES = ("CONFIGURED", "MANUAL_DERIVED", "PARTIAL")
VIEWER_CAMERAS = ("perspective", "front", "side")
BODY_PROFILES = tuple(profile.value for profile in BodyProfile)
MOTIONVENUS_FORMATS = ("auto", "binary", "json")
MOTIONVENUS_MODES = ("sim", "real")
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
    "stream": {"mode": "read_only", "stale_after_ms": 100.0},
    "monitor": {
        "show_raw": False,
        "show_euler": True,
        "show_quaternion": True,
        "show_imu": True,
    },
    "frames": {
        "version": 1,
        "sensor_to_body_matrix": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        "status": "CONFIGURED",
    },
    "body_mapping": {role: None for role in FULL_BODY_ROLES},
    "calibration": {
        "file": None,
        "duration_s": 2.0,
        "minimum_samples": 20,
        "maximum_angular_deviation_deg": 3.0,
        "maximum_gyro_magnitude": 5.0,
    },
    "skeleton": {
        "shoulder_width_m": 0.38,
        "left_upper_arm_m": 0.30,
        "left_forearm_m": 0.26,
        "left_hand_m": 0.10,
        "right_upper_arm_m": 0.30,
        "right_forearm_m": 0.26,
        "right_hand_m": 0.10,
    },
    "full_body": {
        "shoulder_width_m": 0.38,
        "left_upper_arm_m": 0.30,
        "left_forearm_m": 0.26,
        "left_hand_m": 0.10,
        "right_upper_arm_m": 0.30,
        "right_forearm_m": 0.26,
        "right_hand_m": 0.10,
        "torso_length_m": 0.50,
        "neck_length_m": 0.10,
        "head_length_m": 0.18,
        "hip_width_m": 0.30,
        "left_thigh_m": 0.42,
        "left_lower_leg_m": 0.43,
        "left_foot_m": 0.25,
        "right_thigh_m": 0.42,
        "right_lower_leg_m": 0.43,
        "right_foot_m": 0.25,
    },
    "retarget": {
        "human_reach_m": 0.56,
        "g1_reach_m": 0.321,
        "max_robot_reach_m": 0.43,
        "workspace_radius_m": 1.0,
    },
    "filter": {
        "position_alpha": 0.2,
        "orientation_alpha": 0.2,
        "max_translation_rate_m_s": 0.8,
        "max_angular_rate_deg_s": 180.0,
    },
    "g1": {
        "mode": "mujoco",
        "xr_root": None,
        "mujoco_model": None,
        "max_joint_delta_rad": 0.35,
        "steps_per_pose": 250,
    },
    "viewer": {
        "fps": 30.0,
        "camera": "perspective",
        "show_segment_axes": False,
        "show_sensors": False,
        "mode": "upper",
    },
    "motionvenus": {
        "bind": "0.0.0.0",
        "port": 5001,
        "format": "binary",
        "receive_timeout_ms": 250.0,
        "stale_after_ms": 100.0,
        "expected_body_bones": 23,
        "retarget_profile": "config/motionvenus_g1_retarget.yaml",
        "mode": "sim",
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
    stale_after_ms: float = 100.0


@dataclass(frozen=True)
class MonitorConfig:
    show_raw: bool = False
    show_euler: bool = True
    show_quaternion: bool = True
    show_imu: bool = True


@dataclass(frozen=True)
class FramesConfig:
    sensor_to_body_matrix: tuple[tuple[float, float, float], ...]
    status: str = "CONFIGURED"
    version: int = 1


@dataclass(frozen=True)
class BodyMappingConfig:
    role_to_slot: Mapping[str, str]
    profile: str = BodyProfile.UPPER.value


@dataclass(frozen=True)
class CalibrationConfig:
    file: str | None = None
    duration_s: float = 2.0
    minimum_samples: int = 20
    maximum_angular_deviation_deg: float = 3.0
    maximum_gyro_magnitude: float = 5.0


@dataclass(frozen=True)
class SkeletonConfig:
    shoulder_width_m: float
    left_upper_arm_m: float
    left_forearm_m: float
    left_hand_m: float
    right_upper_arm_m: float
    right_forearm_m: float
    right_hand_m: float


@dataclass(frozen=True)
class FullBodyConfig:
    shoulder_width_m: float
    left_upper_arm_m: float
    left_forearm_m: float
    left_hand_m: float
    right_upper_arm_m: float
    right_forearm_m: float
    right_hand_m: float
    torso_length_m: float
    neck_length_m: float
    head_length_m: float
    hip_width_m: float
    left_thigh_m: float
    left_lower_leg_m: float
    left_foot_m: float
    right_thigh_m: float
    right_lower_leg_m: float
    right_foot_m: float


@dataclass(frozen=True)
class RetargetConfig:
    human_reach_m: float
    g1_reach_m: float
    max_robot_reach_m: float
    workspace_radius_m: float


@dataclass(frozen=True)
class FilterConfig:
    position_alpha: float
    orientation_alpha: float
    max_translation_rate_m_s: float
    max_angular_rate_deg_s: float


@dataclass(frozen=True)
class G1Config:
    mode: str
    xr_root: str | None
    mujoco_model: str | None
    max_joint_delta_rad: float
    steps_per_pose: int


@dataclass(frozen=True)
class ViewerConfig:
    fps: float
    camera: str
    show_segment_axes: bool
    show_sensors: bool
    mode: str


@dataclass(frozen=True)
class MotionVenusConfig:
    bind: str
    port: int
    format: str
    receive_timeout_ms: float
    stale_after_ms: float
    expected_body_bones: int
    retarget_profile: str
    mode: str


@dataclass(frozen=True)
class RuntimeConfig:
    usb: USBConfig = field(default_factory=USBConfig)
    protocol: ProtocolConfig = field(default_factory=ProtocolConfig)
    stream: StreamConfig = field(default_factory=StreamConfig)
    monitor: MonitorConfig = field(default_factory=MonitorConfig)
    frames: FramesConfig = field(
        default_factory=lambda: FramesConfig(((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)))
    )
    body_mapping: BodyMappingConfig = field(default_factory=lambda: BodyMappingConfig({}))
    calibration: CalibrationConfig = field(default_factory=CalibrationConfig)
    skeleton: SkeletonConfig = field(default_factory=lambda: SkeletonConfig(0.38, 0.30, 0.26, 0.10, 0.30, 0.26, 0.10))
    full_body: FullBodyConfig = field(
        default_factory=lambda: FullBodyConfig(
            0.38, 0.30, 0.26, 0.10, 0.30, 0.26, 0.10,
            0.50, 0.10, 0.18, 0.30, 0.42, 0.43, 0.25, 0.42, 0.43, 0.25,
        )
    )
    retarget: RetargetConfig = field(default_factory=lambda: RetargetConfig(0.56, 0.321, 0.43, 1.0))
    filter: FilterConfig = field(default_factory=lambda: FilterConfig(0.2, 0.2, 0.8, 180.0))
    g1: G1Config = field(default_factory=lambda: G1Config("mujoco", None, None, 0.35, 250))
    viewer: ViewerConfig = field(default_factory=lambda: ViewerConfig(30.0, "perspective", False, False, "upper"))
    motionvenus: MotionVenusConfig = field(
        default_factory=lambda: MotionVenusConfig(
            "0.0.0.0", 5001, "binary", 250.0, 100.0, 23,
            "config/motionvenus_g1_retarget.yaml", "sim",
        )
    )


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


def _udp_port(value: Any, field_name: str) -> int:
    parsed = _positive_int(value, field_name)
    if not 1024 <= parsed <= 65535:
        raise ConfigError(f"{field_name} must be in 1024..65535")
    return parsed


def _boolean(value: Any, field_name: str) -> bool:
    if not isinstance(value, bool):
        raise ConfigError(f"{field_name} must be true or false")
    return value


def _positive_float(value: Any, field_name: str) -> float:
    if isinstance(value, bool):
        raise ConfigError(f"{field_name} must be a positive number")
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"{field_name} must be a positive number") from exc
    if not np.isfinite(parsed) or parsed <= 0:
        raise ConfigError(f"{field_name} must be a positive finite number")
    return parsed


def _alpha(value: Any, field_name: str) -> float:
    parsed = _positive_float(value, field_name)
    if parsed > 1:
        raise ConfigError(f"{field_name} must be at most 1")
    return parsed


def _optional_string(value: Any, field_name: str) -> str | None:
    if value is None or value == "auto":
        return None
    if not isinstance(value, str) or not value:
        raise ConfigError(f"{field_name} must be a path string or auto")
    return value


def _nonempty_string(value: Any, field_name: str) -> str:
    if not isinstance(value, str) or not value:
        raise ConfigError(f"{field_name} must be a non-empty string")
    return value


def _proper_matrix(value: Any, field_name: str) -> tuple[tuple[float, float, float], ...]:
    try:
        matrix = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"{field_name} must be a numeric 3x3 matrix") from exc
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ConfigError(f"{field_name} must be a finite 3x3 matrix")
    if not np.allclose(matrix.T @ matrix, np.eye(3), atol=1e-7) or not np.isclose(np.linalg.det(matrix), 1.0, atol=1e-7):
        raise ConfigError(f"{field_name} must be a proper right-handed rotation")
    return tuple(tuple(map(float, row)) for row in matrix)


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
    mapping = {
        role: slot
        for role, slot in data["body_mapping"].items()
        if slot is not None and slot != "UNKNOWN"
    }
    if any(not isinstance(slot, str) or not slot for slot in mapping.values()):
        raise ConfigError("body_mapping values must be non-empty slot names or null")
    if len(set(mapping.values())) != len(mapping):
        raise ConfigError("body_mapping cannot assign one slot to multiple roles")
    skeleton = data["skeleton"]
    full_body = data["full_body"]
    retarget = data["retarget"]
    filtering = data["filter"]
    g1 = data["g1"]
    if g1["mode"] != "mujoco":
        raise ConfigError("g1.mode must be mujoco; real G1 modes are forbidden")
    if data["frames"]["version"] != 1:
        raise ConfigError("frames.version must be 1")
    frame_status = _choice(data["frames"]["status"], "frames.status", FRAME_STATUSES)
    calibration = data["calibration"]
    viewer = data["viewer"]
    motionvenus = data["motionvenus"]
    expected_body_bones = _positive_int(
        motionvenus["expected_body_bones"], "motionvenus.expected_body_bones"
    )
    if expected_body_bones != 23:
        raise ConfigError("motionvenus.expected_body_bones must be 23 for the solved-body model")
    profile = _choice(viewer["mode"], "viewer.mode", BODY_PROFILES)
    unknown_mapping_roles = set(mapping) - set(roles_for_profile(profile))
    if unknown_mapping_roles:
        raise ConfigError(
            f"body_mapping role {sorted(unknown_mapping_roles)[0]} is not valid for {profile} mode"
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
            mode=_choice(data["stream"]["mode"], "stream.mode", STREAM_MODES),
            stale_after_ms=_positive_float(
                data["stream"]["stale_after_ms"], "stream.stale_after_ms"
            ),
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
        frames=FramesConfig(
            _proper_matrix(data["frames"]["sensor_to_body_matrix"], "frames.sensor_to_body_matrix"),
            frame_status,
            1,
        ),
        body_mapping=BodyMappingConfig(mapping, profile),
        calibration=CalibrationConfig(
            _optional_string(calibration["file"], "calibration.file"),
            _positive_float(calibration["duration_s"], "calibration.duration_s"),
            _positive_int(calibration["minimum_samples"], "calibration.minimum_samples"),
            _positive_float(
                calibration["maximum_angular_deviation_deg"],
                "calibration.maximum_angular_deviation_deg",
            ),
            _positive_float(
                calibration["maximum_gyro_magnitude"],
                "calibration.maximum_gyro_magnitude",
            ),
        ),
        skeleton=SkeletonConfig(
            *(_positive_float(skeleton[name], f"skeleton.{name}") for name in SkeletonConfig.__dataclass_fields__)
        ),
        full_body=FullBodyConfig(
            *(
                _positive_float(full_body[name], f"full_body.{name}")
                for name in FullBodyConfig.__dataclass_fields__
            )
        ),
        retarget=RetargetConfig(
            *(_positive_float(retarget[name], f"retarget.{name}") for name in RetargetConfig.__dataclass_fields__)
        ),
        filter=FilterConfig(
            _alpha(filtering["position_alpha"], "filter.position_alpha"),
            _alpha(filtering["orientation_alpha"], "filter.orientation_alpha"),
            _positive_float(filtering["max_translation_rate_m_s"], "filter.max_translation_rate_m_s"),
            _positive_float(filtering["max_angular_rate_deg_s"], "filter.max_angular_rate_deg_s"),
        ),
        g1=G1Config(
            "mujoco",
            _optional_string(g1["xr_root"], "g1.xr_root"),
            _optional_string(g1["mujoco_model"], "g1.mujoco_model"),
            _positive_float(g1["max_joint_delta_rad"], "g1.max_joint_delta_rad"),
            _positive_int(g1["steps_per_pose"], "g1.steps_per_pose"),
        ),
        viewer=ViewerConfig(
            _positive_float(viewer["fps"], "viewer.fps"),
            _choice(viewer["camera"], "viewer.camera", VIEWER_CAMERAS),
            _boolean(viewer["show_segment_axes"], "viewer.show_segment_axes"),
            _boolean(viewer["show_sensors"], "viewer.show_sensors"),
            profile,
        ),
        motionvenus=MotionVenusConfig(
            _nonempty_string(motionvenus["bind"], "motionvenus.bind"),
            _udp_port(motionvenus["port"], "motionvenus.port"),
            _choice(motionvenus["format"], "motionvenus.format", MOTIONVENUS_FORMATS),
            _positive_float(motionvenus["receive_timeout_ms"], "motionvenus.receive_timeout_ms"),
            _positive_float(motionvenus["stale_after_ms"], "motionvenus.stale_after_ms"),
            expected_body_bones,
            _nonempty_string(motionvenus["retarget_profile"], "motionvenus.retarget_profile"),
            _choice(motionvenus["mode"], "motionvenus.mode", MOTIONVENUS_MODES),
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
