import math

import numpy as np
import pytest

from foheart.config import ConfigError, load_config
from foheart.mocap.calibration import (
    CalibrationObservation,
    CalibrationProfile,
    estimate_neutral_quaternion,
)
from foheart.mocap.frames import BasisTransform, axis_rotation, matrix_to_quaternion
from foheart.mocap.sensor import Quaternion, SensorSample, TransportKey, Vector3
from foheart.mocap.skeleton import BodyDimensions, JointFrame, UpperBodyKinematics
from foheart.mocap.stream import (
    LiveC1SensorSource,
    LogicalSlotRegistry,
    ReplaySensorSource,
    SensorSource,
    SourceSample,
    SyntheticSensorSource,
    UpperBodyStreamProcessor,
    motion_energy,
    propose_moving_slot,
)
from foheart.mocap.suit import BodySensorMap, UPPER_BODY_ROLES
from foheart.mocap.synthetic import (
    SYNTHETIC_BODY_MAP,
    SYNTHETIC_LIVE_MOTIONS,
    synthetic_live_rotations,
)
from foheart.protocol.frame import PollCaptureRecord, PollRecorder
from foheart.tools.calibrate_live import main as calibrate_live_main
from foheart.tools.live_joint_viewer import ViewerState, main as viewer_main
from foheart.tools.live_sensor_monitor import main as sensor_monitor_main
from foheart.tools.map_body_sensors import confirm_assignment, main as map_body_main
from foheart.usb.c1_poll import C1PollResult

IDENTITY = Quaternion((1.0, 0.0, 0.0, 0.0), "wxyz")
REAL_REPORT = bytes.fromhex(
    "15 00 dd 03 14 20 20 1d 8c 00 00 5f 1e d4 00 f4 ff ad c7 d6 ff "
    "21 00 ce 07 03 00 02 00 00 00 b3 e8 09 f0 5c f9 00 00 94 8f "
    "00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00"
)


def _identity_calibration():
    return CalibrationProfile.capture({role: IDENTITY for role in UPPER_BODY_ROLES})


def _write_replay(path, reports=(REAL_REPORT,)):
    with path.open("wb") as stream:
        recorder = PollRecorder(stream)
        for index, report in enumerate(reports, 1):
            recorder.write(
                PollCaptureRecord(
                    index,
                    index * 1_000_000,
                    64,
                    index * 1_000_000 + 1,
                    0x81,
                    report,
                    False,
                    None,
                    1_000,
                )
            )


def test_sources_share_one_abstraction_and_live_construction_does_not_open():
    opened = False

    def opener(**_):
        nonlocal opened
        opened = True

    assert isinstance(SyntheticSensorSource(), SensorSource)
    assert isinstance(ReplaySensorSource("unused"), SensorSource)
    source = LiveC1SensorSource(opener=opener)
    assert isinstance(source, SensorSource)
    assert source.c1_status == "NOT CONNECTED"
    assert not opened


def test_live_source_wraps_validated_poll_layer_with_fake_device(monkeypatch):
    class Device:
        closed = False

        def close(self):
            self.closed = True

    device = Device()
    monkeypatch.setattr("foheart.mocap.stream._open_poll_device", lambda _: device)
    monkeypatch.setattr(
        "foheart.mocap.stream._poll_open_device_once",
        lambda *_args, **_kwargs: C1PollResult(10, 64, 11, REAL_REPORT, False, 1_000),
    )
    source = LiveC1SensorSource()
    source.start()
    event = source.next_sample()
    assert event.transport_key.kind == "hid_0x15_header_bytes_1_4"
    assert event.transport_key.evidence_status == "UNKNOWN"
    assert event.sample.quaternion is not None
    source.close()
    assert device.closed


def test_logical_slots_are_stable_by_transport_key_not_packet_order():
    registry = LogicalSlotRegistry()
    key_a = TransportKey("test", "A", "SOFTWARE_TESTED")
    key_b = TransportKey("test", "B", "SOFTWARE_TESTED")
    sample = SensorSample(99, quaternion=IDENTITY)
    first = registry.observe(SourceSample(10, key_a, sample, physical_sensor_label="label-A"))
    registry.observe(SourceSample(11, key_b, sample))
    again = registry.observe(SourceSample(12, key_a, sample))
    assert first.slot == again.slot == "slot_0"
    assert registry.sensors["slot_1"].transport_key == key_b
    assert again.packet_count == 2
    assert again.physical_sensor_label == "label-A"


def test_replay_source_exposes_candidate_key_and_one_real_slot(tmp_path):
    path = tmp_path / "one_sensor.bin"
    _write_replay(path, (REAL_REPORT, REAL_REPORT))
    source = ReplaySensorSource(path)
    registry = LogicalSlotRegistry()
    with source:
        while not source.eof:
            event = source.next_sample()
            if event:
                registry.observe(event)
    assert tuple(registry.sensors) == ("slot_0",)
    assert registry.sensors["slot_0"].packet_count == 2
    assert registry.sensors["slot_0"].transport_key.evidence_status == "UNKNOWN"


def test_body_mapping_yaml_round_trip_metadata_and_duplicate_rejection(tmp_path):
    keys = {
        slot: TransportKey("test", slot, "SOFTWARE_TESTED")
        for slot in SYNTHETIC_BODY_MAP.role_to_slot.values()
    }
    mapping = BodySensorMap(
        SYNTHETIC_BODY_MAP.role_to_slot,
        slot_transport_keys=keys,
        physical_labels={"slot_0": "chest sticker"},
    )
    path = tmp_path / "body.yaml"
    mapping.save(path)
    loaded = BodySensorMap.load(path)
    loaded.require_upper_body()
    assert loaded.role_to_slot == mapping.role_to_slot
    assert loaded.slot_transport_keys == keys
    assert loaded.physical_labels == {"slot_0": "chest sticker"}
    with pytest.raises(FileExistsError):
        mapping.save(path)

    duplicate_role = tmp_path / "duplicate_role.yaml"
    duplicate_role.write_text(
        "version: 1\nstatus: CONFIGURED\nbody_mapping:\n"
        "  torso: slot_0\n  torso: slot_1\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="duplicate key"):
        BodySensorMap.load(duplicate_role)
    with pytest.raises(ValueError, match="multiple body roles"):
        BodySensorMap({"torso": "slot_0", "left_upper_arm": "slot_0"})


def test_interactive_synthetic_mapping_requires_manual_choices(tmp_path, monkeypatch):
    output = tmp_path / "mapping.yaml"
    answers = iter(f"slot_{index}" for index in range(7))
    monkeypatch.setattr("builtins.input", lambda _prompt: next(answers))
    assert map_body_main(["--synthetic", "--output", str(output)]) == 0
    loaded = BodySensorMap.load(output)
    loaded.require_upper_body()
    assert len(loaded.slot_transport_keys) == 7
    assert map_body_main(["--synthetic", "--output", str(output)]) == 2


def test_motion_proposal_and_confirmation_are_separate():
    moved = matrix_to_quaternion(axis_rotation("x", 45))
    still = [SensorSample(0, quaternion=IDENTITY, gyro=Vector3(0, 0, 0))] * 2
    moving = [
        SensorSample(1, quaternion=IDENTITY, gyro=Vector3(0, 0, 0)),
        SensorSample(1, quaternion=moved, gyro=Vector3(20, 0, 0)),
    ]
    assert motion_energy(moving) > motion_energy(still)
    assert propose_moving_slot({"slot_0": still, "slot_1": moving})[0] == "slot_1"
    assert confirm_assignment("yes")
    assert not confirm_assignment("")
    assert not confirm_assignment("no")


def test_quaternion_estimator_handles_sign_and_quality_rejects_motion():
    observations = [
        CalibrationObservation(1, IDENTITY, Vector3(0, 0, 0)),
        CalibrationObservation(2, Quaternion((-1, 0, 0, 0), "wxyz"), Vector3(0, 0, 0)),
        CalibrationObservation(3, IDENTITY, Vector3(0, 0, 0)),
    ]
    mean, deviations = estimate_neutral_quaternion(observations)
    assert mean.values == pytest.approx(IDENTITY.values)
    assert max(deviations) == pytest.approx(0)
    profile = CalibrationProfile.capture_window(
        {"torso": observations}, minimum_samples=3
    )
    assert profile.quality["torso"].sample_count == 3
    moving = [
        CalibrationObservation(1, IDENTITY, Vector3(0, 0, 0)),
        CalibrationObservation(
            2, matrix_to_quaternion(axis_rotation("x", 20)), Vector3(20, 0, 0)
        ),
    ]
    with pytest.raises(ValueError, match="rejected"):
        CalibrationProfile.capture_window(
            {"torso": moving},
            minimum_samples=2,
            maximum_angular_deviation_degrees=1,
            maximum_gyro_magnitude=1,
        )


def test_synthetic_live_calibration_file_has_quality_and_validation_boundary(tmp_path):
    output = tmp_path / "neutral.yaml"
    assert (
        calibrate_live_main(
            [
                "--synthetic",
                "--no-prompt",
                "--samples-per-role",
                "5",
                "--output",
                str(output),
            ]
        )
        == 0
    )
    profile = CalibrationProfile.load(output)
    profile.require_roles(UPPER_BODY_ROLES)
    assert profile.status == "SOFTWARE_TESTED"
    assert profile.live_validated is False
    assert all(quality.sample_count == 5 for quality in profile.quality.values())


def test_suitframe_stale_missing_and_jointframe_fail_closed():
    source = SyntheticSensorSource()
    source.start()
    processor = UpperBodyStreamProcessor(
        SYNTHETIC_BODY_MAP,
        BasisTransform.identity("sensor", "body"),
        _identity_calibration(),
        stale_after_ms=100,
    )
    first = processor.process(source.next_sample())
    assert not first.suit.valid
    assert not first.joints.valid
    assert len(first.suit.missing_roles) == 6
    latest = first
    for _ in range(6):
        latest = processor.process(source.next_sample())
    assert latest.suit.valid and latest.joints.valid
    assert set(latest.suit.sample_ages_ms) == set(UPPER_BODY_ROLES)
    stale = processor.tick(latest.suit.timestamp_ns + 100_000_001)
    assert not stale.suit.valid and not stale.joints.valid
    assert set(stale.suit.stale_roles) == set(UPPER_BODY_ROLES)


def test_synthetic_end_to_end_joint_motion_and_bone_invariance():
    source = SyntheticSensorSource(fps=30)
    source.start()
    processor = UpperBodyStreamProcessor(
        SYNTHETIC_BODY_MAP,
        BasisTransform.identity("sensor", "body"),
        _identity_calibration(),
        dimensions=BodyDimensions(),
    )
    neutral = forward = None
    for frame_number in range(60):
        for _ in UPPER_BODY_ROLES:
            result = processor.process(source.next_sample())
        assert result.joints.valid
        assert not result.diagnostics
        assert all(np.isfinite(point).all() for point in result.joints.joints.values())
        lengths = result.joints.segment_lengths()
        assert lengths["left_upper_arm"] == pytest.approx(0.30)
        assert lengths["right_forearm"] == pytest.approx(0.26)
        if frame_number == 0:
            neutral = result.joints
        if frame_number == 59:
            forward = result.joints
    assert neutral.joints["left_shoulder"][1] > neutral.joints["right_shoulder"][1]
    assert forward.joints["left_wrist"][0] > neutral.joints["left_wrist"][0] + 0.4


def test_all_required_synthetic_live_motions_exist():
    names = [synthetic_live_rotations(index * 45, 30)[0] for index in range(10)]
    assert tuple(names) == SYNTHETIC_LIVE_MOTIONS
    for index in range(10):
        _, rotations = synthetic_live_rotations(index * 45 + 20, 30)
        assert set(rotations) == set(UPPER_BODY_ROLES)
        assert all(np.isfinite(rotation).all() for rotation in rotations.values())


def test_joint_diagnostics_detect_impossible_numeric_length_change():
    source = SyntheticSensorSource()
    source.start()
    processor = UpperBodyStreamProcessor(
        SYNTHETIC_BODY_MAP,
        BasisTransform.identity("sensor", "body"),
        _identity_calibration(),
    )
    for _ in UPPER_BODY_ROLES:
        result = processor.process(source.next_sample())
    changed = dict(result.joints.joints)
    changed["left_wrist"] = changed["left_wrist"] + np.array((0.1, 0, 0))
    invalid_length = JointFrame(result.joints.timestamp_ns, changed)
    assert UpperBodyKinematics().diagnose(invalid_length)


def test_viewer_state_model_status_and_replay_insufficiency(tmp_path, monkeypatch, capsys):
    source = SyntheticSensorSource()
    source.start()
    registry = LogicalSlotRegistry()
    processor = UpperBodyStreamProcessor(
        SYNTHETIC_BODY_MAP,
        BasisTransform.identity("sensor", "body"),
        _identity_calibration(),
        registry=registry,
    )
    state = ViewerState("SYNTHETIC", "NOT USED", SYNTHETIC_BODY_MAP, True, "CONFIGURED", registry)
    for _ in UPPER_BODY_ROLES:
        state.accept(processor.process(source.next_sample()))
    assert "Mapped: 7 / 7" in state.status_panel()
    assert "L Shoulder" in state.joint_table()

    replay = tmp_path / "one.bin"
    _write_replay(replay)
    monkeypatch.setenv("MPLBACKEND", "Agg")
    monkeypatch.setenv("MPLCONFIGDIR", str(tmp_path / "mpl"))
    assert viewer_main(["--replay", str(replay), "--headless", "--duration", "0.05"]) == 0
    assert "INSUFFICIENT REAL SENSOR ROLES" in capsys.readouterr().out


def test_live_sensor_monitor_synthetic_mode_is_bounded_and_offline(capsys):
    assert sensor_monitor_main(["--synthetic", "--samples", "7", "--no-clear"]) == 0
    output = capsys.readouterr().out
    assert "FOHEART C1: NOT USED" in output
    assert "Sensors detected: 7" in output
    assert "slot_6" in output


def test_live_viewer_preflight_fails_before_any_usb_open(monkeypatch, capsys):
    monkeypatch.setattr(
        "foheart.tools.live_joint_viewer.create_sensor_source",
        lambda **_: pytest.fail("source created despite missing live mapping/calibration"),
    )
    assert viewer_main([]) == 2
    assert "body mapping is missing roles" in capsys.readouterr().out


def test_live_configuration_defaults_and_manual_derived_profile():
    config = load_config()
    assert config.stream.stale_after_ms == 100
    assert config.viewer.fps == 30
    assert config.frames.version == 1
    assert (
        load_config(overrides={"frames": {"status": "MANUAL_DERIVED"}}).frames.status
        == "MANUAL_DERIVED"
    )
    with pytest.raises(ConfigError, match="frames.version"):
        load_config(overrides={"frames": {"version": 2}})
