"""Linux 3D upper/full-body joint viewer over the shared sensor pipeline."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import time

import numpy as np

from foheart.config import RuntimeConfig, load_config
from foheart.mocap.calibration import CalibrationProfile
from foheart.mocap.frames import BasisTransform, quaternion_to_matrix
from foheart.mocap.sensor import Quaternion
from foheart.mocap.skeleton import (
    BodyDimensions,
    FullBodyDimensions,
    FullBodyJointFrame,
    JointFrame,
)
from foheart.mocap.stream import (
    FullBodyStreamProcessor,
    LogicalSlotRegistry,
    PipelineFrame,
    SensorSource,
    SensorSourceError,
    UpperBodyStreamProcessor,
    create_sensor_source,
)
from foheart.mocap.suit import BodyProfile, BodySensorMap, body_profile
from foheart.mocap.synthetic import SYNTHETIC_BODY_MAP, SYNTHETIC_FULL_BODY_MAP

UPPER_BONES = (
    ("torso", "left_shoulder"),
    ("torso", "right_shoulder"),
    ("left_shoulder", "left_elbow"),
    ("left_elbow", "left_wrist"),
    ("left_wrist", "left_hand_end"),
    ("right_shoulder", "right_elbow"),
    ("right_elbow", "right_wrist"),
    ("right_wrist", "right_hand_end"),
)
FULL_BONES = (
    ("pelvis", "lower_spine"),
    ("lower_spine", "mid_spine"),
    ("mid_spine", "torso"),
    ("torso", "neck"),
    ("neck", "head"),
    ("torso", "left_shoulder"),
    ("torso", "right_shoulder"),
    ("left_shoulder", "left_elbow"),
    ("left_elbow", "left_wrist"),
    ("left_wrist", "left_hand_end"),
    ("right_shoulder", "right_elbow"),
    ("right_elbow", "right_wrist"),
    ("right_wrist", "right_hand_end"),
    ("pelvis", "left_hip"),
    ("left_hip", "left_knee"),
    ("left_knee", "left_ankle"),
    ("left_ankle", "left_foot_end"),
    ("pelvis", "right_hip"),
    ("right_hip", "right_knee"),
    ("right_knee", "right_ankle"),
    ("right_ankle", "right_foot_end"),
)
UPPER_JOINT_TABLE = (
    ("L Shoulder", "left_shoulder"),
    ("L Elbow", "left_elbow"),
    ("L Wrist", "left_wrist"),
    ("L Hand", "left_hand_end"),
    ("R Shoulder", "right_shoulder"),
    ("R Elbow", "right_elbow"),
    ("R Wrist", "right_wrist"),
    ("R Hand", "right_hand_end"),
)
FULL_JOINT_TABLE = (
    ("Pelvis", "pelvis"),
    ("Head", "head"),
    ("L Shoulder", "left_shoulder"),
    ("L Elbow", "left_elbow"),
    ("L Wrist", "left_wrist"),
    ("R Shoulder", "right_shoulder"),
    ("R Elbow", "right_elbow"),
    ("R Wrist", "right_wrist"),
    ("L Hip", "left_hip"),
    ("L Knee", "left_knee"),
    ("L Ankle", "left_ankle"),
    ("R Hip", "right_hip"),
    ("R Knee", "right_knee"),
    ("R Ankle", "right_ankle"),
)
UPPER_SEGMENT_ANCHORS = {
    "torso": "torso",
    "left_upper_arm": "left_shoulder",
    "left_forearm": "left_elbow",
    "left_hand": "left_wrist",
    "right_upper_arm": "right_shoulder",
    "right_forearm": "right_elbow",
    "right_hand": "right_wrist",
}
FULL_SEGMENT_ANCHORS = {
    "head": "neck",
    "left_shoulder": "torso",
    "right_shoulder": "torso",
    "torso": "mid_spine",
    "pelvis": "pelvis",
    "left_upper_arm": "left_shoulder",
    "right_upper_arm": "right_shoulder",
    "left_forearm": "left_elbow",
    "right_forearm": "right_elbow",
    "left_hand": "left_wrist",
    "right_hand": "right_wrist",
    "left_thigh": "left_hip",
    "right_thigh": "right_hip",
    "left_lower_leg": "left_knee",
    "right_lower_leg": "right_knee",
    "left_foot": "left_ankle",
    "right_foot": "right_ankle",
}


@dataclass
class ViewerState:
    source_name: str
    c1_status: str
    mapping: BodySensorMap
    calibration_loaded: bool
    sensor_basis_status: str
    registry: LogicalSlotRegistry
    latest: PipelineFrame | None = None
    last_valid: PipelineFrame | None = None
    measured_fps: float = 0.0
    message: str = ""

    def accept(self, frame: PipelineFrame) -> None:
        self.latest = frame
        if frame.joints.valid:
            self.last_valid = frame

    @property
    def displayed_joints(self) -> JointFrame | FullBodyJointFrame | None:
        if self.latest and self.latest.joints.valid:
            return self.latest.joints
        return self.last_valid.joints if self.last_valid else None

    @property
    def mapped_count(self) -> int:
        detected = set(self.registry.sensors)
        return sum(slot in detected for slot in self.mapping.role_to_slot.values())

    @property
    def roles(self) -> tuple[str, ...]:
        return self.mapping.required_roles

    @property
    def frame_status(self) -> str:
        if self.latest is None:
            return "MISSING"
        if self.latest.suit.valid:
            return "VALID"
        return "STALE" if self.latest.suit.stale_roles else "MISSING"

    def status_panel(self) -> str:
        lines = [
            f"Source: {self.source_name}",
            f"Profile: {self.mapping.profile.value.upper()}",
            f"C1: {self.c1_status}",
            f"Sensors detected: {len(self.registry.sensors)}",
            f"Mapped: {self.mapped_count} / {len(self.roles)}",
            f"Calibration: {'LOADED' if self.calibration_loaded else 'MISSING'}",
            f"Sensor basis: {self.sensor_basis_status}",
            f"FPS: {self.measured_fps:.1f}",
            f"Frame: {self.frame_status}",
        ]
        if self.mapping.profile is BodyProfile.FULL:
            lines.extend(
                (
                    "Segments: 17 MEASURED + 6 DERIVED",
                    "Root translation: FIXED / NOT TRACKED",
                )
            )
        ages = self.latest.suit.sample_ages_ms if self.latest else {}
        lines.extend(
            f"{role}: {ages[role]:.1f} ms" if role in ages else f"{role}: -"
            for role in self.roles
        )
        if self.latest and self.latest.suit.reason:
            lines.extend(("", self.latest.suit.reason))
        if self.registry.diagnostics:
            lines.extend(("", *self.registry.diagnostics[-3:]))
        if self.registry.missing_bound_slots:
            lines.extend(
                (
                    "",
                    "Missing saved transport slots: "
                    + ", ".join(self.registry.missing_bound_slots),
                )
            )
        if self.message:
            lines.extend(("", self.message))
        return "\n".join(lines)

    def joint_table(self) -> str:
        lines = ["Joint              X        Y        Z  (m)"]
        joints = self.displayed_joints
        table = (
            FULL_JOINT_TABLE
            if self.mapping.profile is BodyProfile.FULL
            else UPPER_JOINT_TABLE
        )
        for label, name in table:
            if joints is None or name not in joints.joints:
                lines.append(f"{label:<12}       -        -        -")
            else:
                x, y, z = joints.joints[name]
                lines.append(f"{label:<12} {x:+8.3f} {y:+8.3f} {z:+8.3f}")
        return "\n".join(lines)


def _load_runtime(
    config: RuntimeConfig,
    *,
    mode: BodyProfile | str,
    synthetic: bool,
    replay: Path | None,
    body_mapping: Path | None,
    calibration_path: Path | None,
) -> tuple[BodySensorMap, BasisTransform, CalibrationProfile | None]:
    profile = body_profile(mode)
    if body_mapping:
        mapping = BodySensorMap.load(body_mapping)
    elif synthetic or replay is not None:
        mapping = (
            SYNTHETIC_FULL_BODY_MAP
            if profile is BodyProfile.FULL
            else SYNTHETIC_BODY_MAP
        )
    else:
        mapping = BodySensorMap(config.body_mapping.role_to_slot, profile=profile)
    mapping.require_profile(profile)
    basis = BasisTransform(
        config.frames.sensor_to_body_matrix,
        "foheart_sensor_unknown",
        "configured_body_sensor",
        config.frames.status,
    )
    selected_calibration = calibration_path or (
        Path(config.calibration.file) if config.calibration.file else None
    )
    if selected_calibration:
        calibration = CalibrationProfile.load(selected_calibration)
    elif synthetic:
        calibration = CalibrationProfile.capture(
            {
                role: Quaternion((1.0, 0.0, 0.0, 0.0), "wxyz")
                for role in profile.roles
            }
        )
    else:
        calibration = None
    if not synthetic and replay is None:
        if calibration is None:
            raise ValueError(
                f"Cannot start {profile.value}-body viewer.\n\nCalibration is missing. "
                "Run foheart.tools.calibrate_live first."
            )
        calibration.require_roles(profile.roles)
    elif calibration is not None:
        calibration.require_roles(profile.roles)
    return mapping, basis, calibration


def _pump(source: SensorSource, processor, state: ViewerState) -> bool:
    event_count = len(state.roles) if source.source_name == "SYNTHETIC" else 1
    received = False
    for _ in range(event_count):
        event = source.next_sample()
        if event is None:
            if source.eof:
                break
            state.accept(processor.tick())
            continue
        state.accept(processor.process(event))
        received = True
    state.c1_status = source.c1_status
    return received


def _prime_live(
    source: SensorSource,
    processor,
    state: ViewerState,
    timeout_s: float,
) -> None:
    deadline = time.monotonic() + timeout_s
    required_slots = set(state.mapping.role_to_slot.values())
    while time.monotonic() < deadline and not required_slots <= set(state.registry.sensors):
        _pump(source, processor, state)
    missing_roles = [
        role
        for role, slot in state.mapping.role_to_slot.items()
        if slot not in state.registry.sensors
    ]
    if missing_roles:
        raise ValueError(
            f"Cannot start {state.mapping.profile.value}-body viewer.\n\n"
            f"Required roles: {len(state.roles)}\n"
            f"Available mapped roles: {state.mapped_count}\n\nMissing:\n  "
            + "\n  ".join(missing_roles)
        )
    if state.latest is None or not state.latest.suit.valid:
        raise ValueError(
            f"Cannot start {state.mapping.profile.value}-body viewer: "
            "mapped sensors are stale or incomplete"
        )
    state.registry.mark_running()


def _draw_orientation_axes(ax, origin, quaternion, *, scale, alpha, linestyle):
    rotation = quaternion_to_matrix(quaternion)
    for index, color in enumerate(("#e74c3c", "#2ecc71", "#3498db")):
        vector = rotation[:, index] * scale
        ax.plot(
            (origin[0], origin[0] + vector[0]),
            (origin[1], origin[1] + vector[1]),
            (origin[2], origin[2] + vector[2]),
            color=color,
            alpha=alpha,
            linestyle=linestyle,
            linewidth=1.2,
        )


def render_viewer(
    ax,
    state: ViewerState,
    *,
    camera: str,
    show_segment_axes: bool,
    show_sensors: bool,
) -> None:
    ax.clear()
    joints = state.displayed_joints
    full = state.mapping.profile is BodyProfile.FULL
    bones = FULL_BONES if full else UPPER_BONES
    anchors = FULL_SEGMENT_ANCHORS if full else UPPER_SEGMENT_ANCHORS
    if joints is not None:
        for start, end in bones:
            points = np.vstack((joints.joints[start], joints.joints[end]))
            ax.plot(points[:, 0], points[:, 1], points[:, 2], color="#34495e", linewidth=3)
        points = np.vstack(tuple(joints.joints.values()))
        ax.scatter(points[:, 0], points[:, 1], points[:, 2], color="#f39c12", s=28)
        orientation_frame = state.latest if state.latest and state.latest.suit.valid else state.last_valid
        if orientation_frame is not None:
            suit = orientation_frame.suit
            if show_segment_axes:
                if full:
                    for role, rotation in joints.segment_orientations.items():
                        origin = joints.joints[anchors[role]]
                        for index, color in enumerate(
                            ("#e74c3c", "#2ecc71", "#3498db")
                        ):
                            vector = rotation[:, index] * 0.09
                            ax.plot(
                                (origin[0], origin[0] + vector[0]),
                                (origin[1], origin[1] + vector[1]),
                                (origin[2], origin[2] + vector[2]),
                                color=color,
                                alpha=0.9,
                                linewidth=1.2,
                            )
                else:
                    for role, quaternion in suit.orientations.items():
                        _draw_orientation_axes(
                            ax,
                            joints.joints[anchors[role]],
                            quaternion,
                            scale=0.09,
                            alpha=0.9,
                            linestyle="-",
                        )
            if show_sensors:
                groups = (
                    ("RAW SENSOR", suit.raw_orientations, -0.025, ":", 0.45),
                    ("FRAME CONVERTED", suit.converted_orientations, 0.0, "--", 0.65),
                    ("CALIBRATED SEGMENT", suit.orientations, 0.025, "-", 0.9),
                )
                for _, orientations, offset, linestyle, alpha in groups:
                    for role, quaternion in orientations.items():
                        origin = joints.joints[anchors[role]] + np.array((offset, 0, 0))
                        _draw_orientation_axes(
                            ax,
                            origin,
                            quaternion,
                            scale=0.065,
                            alpha=alpha,
                            linestyle=linestyle,
                        )
    limit = 1.15 if full else 0.85
    ax.set(xlim=(-limit, limit), ylim=(-limit, limit), zlim=(-limit, limit))
    ax.set_box_aspect((1, 1, 1))
    ax.set_xlabel("+X forward (m)")
    ax.set_ylabel("+Y left (m)")
    ax.set_zlabel("+Z up (m)")
    views = {
        "front": (0, 0),
        "side": (0, -90),
        "perspective": (20, -55),
    }
    ax.view_init(*views[camera])
    invalid = state.latest is not None and not state.latest.suit.valid
    title = f"FOHEART {'full' if full else 'upper'}-body joints"
    if invalid:
        title += " — INVALID, LAST VALID SKELETON FROZEN"
    ax.set_title(title, color="#c0392b" if invalid else "#1f2937")
    font_size = 7 if full else 9
    ax.text2D(
        -0.30,
        0.98,
        state.joint_table(),
        transform=ax.transAxes,
        family="monospace",
        va="top",
        fontsize=font_size,
    )
    ax.text2D(
        1.02,
        0.98,
        state.status_panel(),
        transform=ax.transAxes,
        family="monospace",
        va="top",
        fontsize=font_size,
    )
    if show_sensors:
        ax.text2D(
            0.0,
            -0.08,
            "Sensor axes: RAW dotted | FRAME CONVERTED dashed | CALIBRATED solid; X red, Y green, Z blue",
            transform=ax.transAxes,
            fontsize=8,
        )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="FOHEART Linux 3D joint viewer")
    parser.add_argument("--config", type=Path)
    parser.add_argument("--mode", choices=tuple(profile.value for profile in BodyProfile))
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--synthetic", action="store_true")
    source.add_argument("--replay", type=Path)
    parser.add_argument("--body-mapping", type=Path)
    parser.add_argument("--calibration", type=Path)
    parser.add_argument("--duration", type=float)
    parser.add_argument("--fps", type=float)
    parser.add_argument("--camera", choices=("perspective", "front", "side"))
    parser.add_argument("--show-segment-axes", action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument("--show-sensors", action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--startup-timeout", type=float, default=10.0)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.duration is not None and args.duration <= 0:
        raise SystemExit("--duration must be positive")
    if args.fps is not None and args.fps <= 0:
        raise SystemExit("--fps must be positive")
    if args.startup_timeout <= 0:
        raise SystemExit("--startup-timeout must be positive")
    try:
        config = load_config(args.config)
        mode = args.mode or config.viewer.mode
        fps = args.fps or config.viewer.fps
        camera = args.camera or config.viewer.camera
        show_segment_axes = (
            config.viewer.show_segment_axes
            if args.show_segment_axes is None
            else args.show_segment_axes
        )
        show_sensors = (
            config.viewer.show_sensors if args.show_sensors is None else args.show_sensors
        )
        mapping, basis, calibration = _load_runtime(
            config,
            mode=mode,
            synthetic=args.synthetic,
            replay=args.replay,
            body_mapping=args.body_mapping,
            calibration_path=args.calibration,
        )
        source = create_sensor_source(
            synthetic=args.synthetic,
            replay=args.replay,
            fps=fps,
            profile=mode,
        )
        registry = LogicalSlotRegistry(mapping.registry_bindings)
        if mapping.profile is BodyProfile.FULL:
            processor = FullBodyStreamProcessor(
                mapping,
                basis,
                calibration,
                stale_after_ms=config.stream.stale_after_ms,
                dimensions=FullBodyDimensions(**config.full_body.__dict__),
                registry=registry,
            )
        else:
            processor = UpperBodyStreamProcessor(
                mapping,
                basis,
                calibration,
                stale_after_ms=config.stream.stale_after_ms,
                dimensions=BodyDimensions(**config.skeleton.__dict__),
                registry=registry,
            )
        state = ViewerState(
            source.source_name,
            source.c1_status,
            mapping,
            calibration is not None,
            basis.status,
            registry,
        )
        source.start()
        state.c1_status = source.c1_status
        if source.source_name == "LIVE_C1":
            _prime_live(source, processor, state, args.startup_timeout)
    except (OSError, RuntimeError, ValueError, SensorSourceError) as exc:
        if "source" in locals():
            source.close()
        print(str(exc))
        return 2

    if args.headless:
        import matplotlib

        matplotlib.use("Agg")
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        source.close()
        print("The joint viewer requires matplotlib. Install the project dependencies.")
        return 2

    duration = args.duration
    if duration is None and args.headless and source.source_name != "REPLAY":
        duration = 1.0
    started = time.monotonic()
    previous_frame_time = started
    frames = 0
    fig = plt.figure(
        figsize=(14, 9) if mapping.profile is BodyProfile.FULL else (12, 8)
    )
    ax = fig.add_subplot(111, projection="3d")
    try:
        while True:
            frame_started = time.monotonic()
            _pump(source, processor, state)
            frames += 1
            elapsed = frame_started - previous_frame_time
            if elapsed > 0:
                state.measured_fps = 1.0 / elapsed
            previous_frame_time = frame_started
            if source.eof and source.source_name == "REPLAY":
                if len(state.registry.sensors) < len(state.roles):
                    state.message = (
                        "INSUFFICIENT REAL SENSOR ROLES\n"
                        f"detected: {len(state.registry.sensors)}  "
                        f"required: {len(state.roles)}"
                    )
                render_viewer(
                    ax,
                    state,
                    camera=camera,
                    show_segment_axes=show_segment_axes,
                    show_sensors=show_sensors,
                )
                break
            if duration is not None and (
                (args.headless and frames >= max(1, math.ceil(duration * fps)))
                or (not args.headless and time.monotonic() - started >= duration)
            ):
                if source.source_name == "REPLAY" and len(state.registry.sensors) < len(state.roles):
                    state.message = (
                        "INSUFFICIENT REAL SENSOR ROLES\n"
                        f"detected: {len(state.registry.sensors)}  "
                        f"required: {len(state.roles)}"
                    )
                render_viewer(
                    ax,
                    state,
                    camera=camera,
                    show_segment_axes=show_segment_axes,
                    show_sensors=show_sensors,
                )
                break
            if not args.headless:
                render_viewer(
                    ax,
                    state,
                    camera=camera,
                    show_segment_axes=show_segment_axes,
                    show_sensors=show_sensors,
                )
                fig.canvas.draw_idle()
                plt.pause(max(0.001, 1 / fps - (time.monotonic() - frame_started)))
                if not plt.fignum_exists(fig.number):
                    break
        fig.canvas.draw()
    except KeyboardInterrupt:
        print("\nJoint viewer stopped.")
    except (OSError, RuntimeError, ValueError, SensorSourceError) as exc:
        print(f"Joint viewer stopped: {exc}")
        return 2
    finally:
        source.close()
        plt.close(fig)
    print(state.status_panel())
    print(state.joint_table())
    if state.latest and state.latest.diagnostics:
        print("Bone-length warning: " + "; ".join(state.latest.diagnostics))
        return 1
    print(f"Viewer frames generated: {frames}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
