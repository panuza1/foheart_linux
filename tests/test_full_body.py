import numpy as np
import pytest

from foheart.config import ConfigError, load_config
from foheart.mocap.calibration import CalibrationProfile
from foheart.mocap.frames import BasisTransform, matrix_to_quaternion
from foheart.mocap.sensor import Quaternion, SensorSample, TransportKey
from foheart.mocap.skeleton import (
    FULL_BODY_23_SEGMENTS,
    FULL_BODY_JOINT_NAMES,
    FullBodyDimensions,
    FullBodyJointFrame,
    FullBodyKinematics,
)
from foheart.mocap.stream import (
    FullBodyStreamProcessor,
    LogicalSlotRegistry,
    SourceSample,
    SyntheticSensorSource,
    TransportKeyCollisionError,
    UnexpectedTransportKeyError,
)
from foheart.mocap.suit import (
    FULL_BODY_ROLES,
    UPPER_BODY_ROLES,
    BodyProfile,
    BodySensorMap,
    FullBodySuitFrame,
)
from foheart.mocap.synthetic import (
    SYNTHETIC_FULL_BODY_MAP,
    SYNTHETIC_FULL_BODY_MOTIONS,
    synthetic_full_body_rotations,
)
from foheart.protocol.frame import PollCaptureRecord, PollRecorder
from foheart.tools.calibrate_live import main as calibrate_live_main
from foheart.tools.live_joint_viewer import (
    ViewerState,
    main as viewer_main,
    render_viewer,
)
from foheart.tools.map_body_sensors import main as map_body_main

IDENTITY = Quaternion((1.0, 0.0, 0.0, 0.0), "wxyz")
REAL_REPORT = bytes.fromhex(
    "15 00 dd 03 14 20 20 1d 8c 00 00 5f 1e d4 00 f4 ff ad c7 d6 ff "
    "21 00 ce 07 03 00 02 00 00 00 b3 e8 09 f0 5c f9 00 00 94 8f "
    "00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00"
)


def _calibration():
    return CalibrationProfile.capture({role: IDENTITY for role in FULL_BODY_ROLES})


def _motion(name):
    index = SYNTHETIC_FULL_BODY_MOTIONS.index(name)
    actual, rotations = synthetic_full_body_rotations(index * 45 + 20, 30)
    assert actual == name
    return FullBodyKinematics().solve(
        {role: matrix_to_quaternion(rotation) for role, rotation in rotations.items()},
        index,
    )


def test_body_profiles_are_exact_and_old_upper_yaml_remains_loadable(tmp_path):
    assert BodyProfile.UPPER.roles == UPPER_BODY_ROLES
    assert BodyProfile.FULL.roles == FULL_BODY_ROLES
    assert len(FULL_BODY_ROLES) == 17 == len(set(FULL_BODY_ROLES))
    assert FULL_BODY_ROLES == (
        "head", "left_shoulder", "right_shoulder", "torso", "pelvis",
        "left_upper_arm", "right_upper_arm", "left_forearm", "right_forearm",
        "left_hand", "right_hand", "left_thigh", "right_thigh",
        "left_lower_leg", "right_lower_leg", "left_foot", "right_foot",
    )
    old = tmp_path / "old-upper.yaml"
    old.write_text(
        "version: 1\nstatus: CONFIGURED\nbody_mapping:\n"
        + "".join(f"  {role}: slot_{index}\n" for index, role in enumerate(UPPER_BODY_ROLES)),
        encoding="utf-8",
    )
    loaded = BodySensorMap.load(old)
    loaded.require_upper_body()
    assert loaded.profile is BodyProfile.UPPER


def test_full_mapping_yaml_round_trip_and_validation(tmp_path):
    keys = {
        slot: TransportKey("synthetic_sensor", str(index), "SOFTWARE_TESTED")
        for index, slot in enumerate(SYNTHETIC_FULL_BODY_MAP.role_to_slot.values())
    }
    mapping = BodySensorMap(
        SYNTHETIC_FULL_BODY_MAP.role_to_slot,
        slot_transport_keys=keys,
        physical_labels={"slot_0": "head sticker"},
        profile=BodyProfile.FULL,
    )
    path = tmp_path / "full-map.yaml"
    mapping.save(path)
    loaded = BodySensorMap.load(path)
    loaded.require_full_body()
    assert loaded.role_to_slot == mapping.role_to_slot
    assert loaded.physical_labels == {"slot_0": "head sticker"}
    assert "profile: full" in path.read_text(encoding="utf-8")
    with pytest.raises(ValueError, match="multiple body roles"):
        BodySensorMap(
            {"head": "slot_0", "pelvis": "slot_0"}, profile=BodyProfile.FULL
        )
    with pytest.raises(ValueError, match="unknown body role"):
        BodySensorMap({"tail": "slot_0"}, profile=BodyProfile.FULL)


def test_seventeen_slots_are_stable_and_registry_reports_session_changes():
    source = SyntheticSensorSource(profile=BodyProfile.FULL)
    registry = LogicalSlotRegistry()
    source.start()
    for _ in FULL_BODY_ROLES:
        registry.observe(source.next_sample())
    assert tuple(registry.sensors) == tuple(f"slot_{index}" for index in range(17))
    first_keys = {slot: sensor.transport_key for slot, sensor in registry.sensors.items()}
    for _ in FULL_BODY_ROLES:
        registry.observe(source.next_sample())
    assert {slot: sensor.transport_key for slot, sensor in registry.sensors.items()} == first_keys
    assert all(sensor.packet_count == 2 for sensor in registry.sensors.values())

    registry.mark_running()
    registry.observe(
        SourceSample(
            2_000_000_000,
            TransportKey("test", "new", "SOFTWARE_TESTED"),
            SensorSample(99, quaternion=IDENTITY),
        )
    )
    assert any("new slot during running" in item for item in registry.diagnostics)
    registry.freeze()
    with pytest.raises(UnexpectedTransportKeyError):
        registry.observe(
            SourceSample(
                2_000_000_001,
                TransportKey("test", "later", "SOFTWARE_TESTED"),
                SensorSample(100, quaternion=IDENTITY),
            )
        )


def test_transport_key_collision_and_missing_or_disappeared_binding_fail_closed():
    key = TransportKey("candidate", "same", "UNKNOWN")
    registry = LogicalSlotRegistry({key: "slot_16"})
    assert registry.missing_bound_slots == ("slot_16",)
    registry.observe(SourceSample(10, key, SensorSample(1, quaternion=IDENTITY)))
    assert not registry.missing_bound_slots
    assert registry.stale_slots(111, 100) == ("slot_16",)
    with pytest.raises(TransportKeyCollisionError, match="collision"):
        registry.observe(SourceSample(12, key, SensorSample(2, quaternion=IDENTITY)))


def test_full_mapping_and_calibration_clis_use_all_seventeen_roles(tmp_path, monkeypatch):
    mapping_path = tmp_path / "full-map.yaml"
    answers = iter(f"slot_{index}" for index in range(17))
    monkeypatch.setattr("builtins.input", lambda _prompt: next(answers))
    assert map_body_main(
        ["--mode", "full", "--synthetic", "--output", str(mapping_path)]
    ) == 0
    mapping = BodySensorMap.load(mapping_path)
    mapping.require_full_body()
    assert len(mapping.slot_transport_keys) == 17

    calibration_path = tmp_path / "full-neutral.yaml"
    assert calibrate_live_main(
        [
            "--mode", "full", "--synthetic", "--no-prompt",
            "--samples-per-role", "5", "--output", str(calibration_path),
        ]
    ) == 0
    calibration = CalibrationProfile.load(calibration_path)
    calibration.require_roles(FULL_BODY_ROLES)
    assert all(quality.sample_count == 5 for quality in calibration.quality.values())
    assert calibration.live_validated is False


def test_full_suit_frame_missing_stale_and_complete_states():
    source = SyntheticSensorSource(profile=BodyProfile.FULL)
    source.start()
    processor = FullBodyStreamProcessor(
        SYNTHETIC_FULL_BODY_MAP,
        BasisTransform.identity("sensor", "body"),
        _calibration(),
        stale_after_ms=100,
    )
    first = processor.process(source.next_sample())
    assert isinstance(first.suit, FullBodySuitFrame)
    assert not first.suit.valid and len(first.suit.missing_roles) == 16
    for _ in range(16):
        complete = processor.process(source.next_sample())
    assert complete.suit.valid and complete.joints.valid
    assert set(complete.suit.orientations) == set(FULL_BODY_ROLES)
    stale = processor.tick(complete.suit.timestamp_ns + 100_000_001)
    assert not stale.suit.valid and not stale.joints.valid
    assert set(stale.suit.stale_roles) == set(FULL_BODY_ROLES)


def test_full_body_fk_neutral_geometry_statuses_and_invariant_lengths():
    frame = _motion("neutral_standing")
    assert isinstance(frame, FullBodyJointFrame)
    assert set(frame.joints) == set(FULL_BODY_JOINT_NAMES)
    assert frame.reference_frame == "human_pelvis"
    assert frame.units == "meters"
    assert frame.root_translation == "NOT_TRACKED_FIXED_ORIGIN"
    assert frame.joints["pelvis"] == pytest.approx((0, 0, 0))
    assert frame.joints["head"][2] > frame.joints["torso"][2] > 0
    assert frame.joints["left_hip"][1] > 0 > frame.joints["right_hip"][1]
    assert len(FULL_BODY_23_SEGMENTS) == 23
    assert list(frame.segment_status.values()).count("MEASURED") == 17
    assert list(frame.segment_status.values()).count("DERIVED") == 6
    expected = FullBodyDimensions()
    lengths = frame.segment_lengths()
    assert lengths["left_thigh"] == pytest.approx(expected.left_thigh_m)
    assert lengths["right_lower_leg"] == pytest.approx(expected.right_lower_leg_m)
    assert lengths["left_foot"] == pytest.approx(expected.left_foot_m)
    assert lengths["upper_spine"] == pytest.approx(expected.torso_length_m / 3)
    assert not FullBodyKinematics(expected).diagnose(frame)


def test_full_body_motion_chains_are_distinct_and_orientation_driven():
    neutral = _motion("neutral_standing")
    left_arm = _motion("left_arm_raise")
    right_arm = _motion("right_arm_raise")
    assert np.linalg.norm(left_arm.joints["left_wrist"] - neutral.joints["left_wrist"]) > 0.2
    assert left_arm.joints["right_wrist"] == pytest.approx(neutral.joints["right_wrist"])
    assert np.linalg.norm(right_arm.joints["right_wrist"] - neutral.joints["right_wrist"]) > 0.2
    assert right_arm.joints["left_wrist"] == pytest.approx(neutral.joints["left_wrist"])

    for side in ("left", "right"):
        elbow = _motion(f"{side}_elbow_flex")
        neutral_forearm = neutral.joints[f"{side}_wrist"] - neutral.joints[f"{side}_elbow"]
        flexed_forearm = elbow.joints[f"{side}_wrist"] - elbow.joints[f"{side}_elbow"]
        assert not np.allclose(flexed_forearm, neutral_forearm)
        other = "right" if side == "left" else "left"
        assert elbow.joints[f"{other}_wrist"] == pytest.approx(
            neutral.joints[f"{other}_wrist"]
        )

    for side in ("left", "right"):
        knee = _motion(f"{side}_knee_bend")
        assert knee.joints[f"{side}_knee"] == pytest.approx(neutral.joints[f"{side}_knee"])
        assert np.linalg.norm(knee.joints[f"{side}_ankle"] - neutral.joints[f"{side}_ankle"]) > 0.2
        hip = _motion(f"{side}_hip_flex")
        assert hip.joints[f"{side}_knee"][0] > neutral.joints[f"{side}_knee"][0] + 0.2
        foot = _motion(f"{side}_foot_pitch")
        assert foot.joints[f"{side}_ankle"] == pytest.approx(neutral.joints[f"{side}_ankle"])
        assert not np.allclose(foot.joints[f"{side}_foot_end"], neutral.joints[f"{side}_foot_end"])

    head = _motion("head_yaw")
    assert not np.allclose(
        head.segment_orientations["head"], neutral.segment_orientations["head"]
    )
    torso = _motion("torso_yaw")
    pelvis = _motion("pelvis_yaw")
    assert not np.allclose(torso.joints["left_wrist"], neutral.joints["left_wrist"])
    assert not np.allclose(pelvis.joints["right_wrist"], neutral.joints["right_wrist"])


def test_every_full_motion_is_finite_and_preserves_all_bone_lengths():
    kinematics = FullBodyKinematics()
    observed = []
    for index, expected_name in enumerate(SYNTHETIC_FULL_BODY_MOTIONS):
        name, rotations = synthetic_full_body_rotations(index * 45 + 20, 30)
        observed.append(name)
        frame = kinematics.solve(
            {role: matrix_to_quaternion(value) for role, value in rotations.items()},
            index,
        )
        assert name == expected_name
        assert all(np.isfinite(point).all() for point in frame.joints.values())
        assert not kinematics.diagnose(frame)
    assert tuple(observed) == SYNTHETIC_FULL_BODY_MOTIONS


def test_full_viewer_model_agg_render_and_bounded_cli(tmp_path, monkeypatch):
    source = SyntheticSensorSource(profile=BodyProfile.FULL)
    source.start()
    registry = LogicalSlotRegistry()
    processor = FullBodyStreamProcessor(
        SYNTHETIC_FULL_BODY_MAP,
        BasisTransform.identity("sensor", "body"),
        _calibration(),
        registry=registry,
    )
    state = ViewerState(
        "SYNTHETIC", "NOT USED", SYNTHETIC_FULL_BODY_MAP, True,
        "CONFIGURED", registry,
    )
    for _ in FULL_BODY_ROLES:
        state.accept(processor.process(source.next_sample()))
    assert "Profile: FULL" in state.status_panel()
    assert "Mapped: 17 / 17" in state.status_panel()
    assert "L Knee" in state.joint_table()

    monkeypatch.setenv("MPLBACKEND", "Agg")
    monkeypatch.setenv("MPLCONFIGDIR", str(tmp_path / "mpl"))
    import matplotlib.pyplot as plt

    figure = plt.figure()
    axes = figure.add_subplot(111, projection="3d")
    render_viewer(
        axes,
        state,
        camera="perspective",
        show_segment_axes=True,
        show_sensors=True,
    )
    figure.canvas.draw()
    plt.close(figure)
    assert viewer_main(
        ["--mode", "full", "--synthetic", "--headless", "--duration", "0.05"]
    ) == 0


def test_one_real_replay_is_insufficient_for_full_mode(tmp_path, monkeypatch, capsys):
    capture = tmp_path / "one-real-sensor.bin"
    with capture.open("wb") as stream:
        recorder = PollRecorder(stream)
        recorder.write(
            PollCaptureRecord(1, 10, 64, 11, 0x81, REAL_REPORT, False, None, 1_000)
        )
    monkeypatch.setenv("MPLBACKEND", "Agg")
    monkeypatch.setenv("MPLCONFIGDIR", str(tmp_path / "mpl-replay"))
    assert viewer_main(
        ["--mode", "full", "--replay", str(capture), "--headless"]
    ) == 0
    output = capsys.readouterr().out
    assert "INSUFFICIENT REAL SENSOR ROLES" in output
    assert "detected: 1" in output and "required: 17" in output


def test_full_configuration_and_live_preflight_fail_before_usb(monkeypatch, capsys):
    config = load_config(overrides={"viewer": {"mode": "full"}})
    assert config.viewer.mode == "full"
    assert config.full_body.left_thigh_m == pytest.approx(0.42)
    with pytest.raises(ConfigError, match="not valid for upper"):
        load_config(overrides={"body_mapping": {"head": "slot_0"}})
    monkeypatch.setattr(
        "foheart.tools.live_joint_viewer.create_sensor_source",
        lambda **_: pytest.fail("USB source created before full mapping/calibration validation"),
    )
    assert viewer_main(["--mode", "full"]) == 2
    assert "full body mapping is missing roles" in capsys.readouterr().out
