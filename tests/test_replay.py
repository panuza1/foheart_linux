from pathlib import Path

from foheart.protocol.frame import (
    MAGIC,
    POLL_MAGIC,
    PollCaptureRecord,
    PollRecorder,
    RawRecorder,
    RawTransfer,
    iter_poll_recording,
    iter_recording,
)
from foheart.protocol.parser import C1ProtocolParser
from foheart.tools.monitor import generate_mock_frame, main as monitor_main
from foheart.tools.replay import main as replay_main
from foheart.tools.replay import replay_poll_attempts, replay_transfers


def test_recording_format_and_replay(tmp_path):
    path = tmp_path / "capture.bin"
    with path.open("wb") as stream:
        recorder = RawRecorder(stream)
        recorder.write(RawTransfer(10, 0x81, b"abc"))
        recorder.write(RawTransfer(20, 0x82, b"defg"))
    assert path.read_bytes().startswith(MAGIC)
    expected = [RawTransfer(10, 0x81, b"abc"), RawTransfer(20, 0x82, b"defg")]
    assert list(iter_recording(path)) == expected
    assert replay_transfers(path) == expected


def test_mock_sensor_generation_is_explicitly_ordered():
    frame = generate_mock_frame(2, sensor_count=2, timestamp_ns=123)
    assert frame.timestamp_ns == 123
    assert len(frame.sensors) == 2
    assert frame.sensors[0].quaternion.component_order == "wxyz (mock only)"


def test_poll_capture_round_trip_preserves_attempt_boundaries(tmp_path):
    path = tmp_path / "poll_capture.bin"
    expected = [
        PollCaptureRecord(1, 10, 64, 11, 0x81, b"\x15abc", False, None, 1_000),
        PollCaptureRecord(2, 20, 64, None, 0x81, b"", True, None, 100_000_000),
        PollCaptureRecord(3, 30, 63, None, 0x81, b"", False, "short OUT", 2_000),
    ]
    with path.open("wb") as stream:
        recorder = PollRecorder(stream)
        for record in expected:
            recorder.write(record)
    assert path.read_bytes().startswith(POLL_MAGIC)
    assert list(iter_poll_recording(path)) == expected
    assert replay_poll_attempts(path) == expected
    assert replay_poll_attempts(path) == expected


def test_poll_capture_replay_output_is_deterministic(tmp_path, capsys):
    path = tmp_path / "poll_capture.bin"
    with path.open("wb") as stream:
        recorder = PollRecorder(stream)
        recorder.write(
            PollCaptureRecord(
                1,
                10,
                64,
                11,
                0x81,
                bytes((0x15,)) + bytes(63),
                False,
                None,
                1_000,
            )
        )
    assert replay_main([str(path)]) == 0
    first = capsys.readouterr().out
    assert replay_main([str(path)]) == 0
    second = capsys.readouterr().out
    assert first == second
    assert "field profile is not real-capture validated" in first


def _real_fixture_bytes():
    fixture = Path(__file__).parents[1] / "samples" / "c1_real_poll_fixture.hex"
    return bytes.fromhex(
        "".join(
            line.strip()
            for line in fixture.read_text(encoding="ascii").splitlines()
            if line and not line.startswith("#")
        )
    )


def test_sanitized_real_fixture_replays_and_decodes_boundaries(tmp_path):
    path = tmp_path / "real_fixture.fhc1poll"
    path.write_bytes(_real_fixture_bytes())
    records = replay_poll_attempts(path)
    assert [record.sequence for record in records] == [1, 2, 3]
    assert [record.out_transferred for record in records] == [64, 64, 64]
    assert [len(record.payload) for record in records] == [64, 64, 64]
    assert [record.payload[0] for record in records] == [0x15, 0x15, 0x15]
    frames = [
        C1ProtocolParser().feed(record.payload, timestamp_ns=record.in_timestamp_ns)[0]
        for record in records
    ]
    assert [len(frame.sensors) for frame in frames] == [1, 1, 1]
    norms = [
        sum(value * value for value in frame.sensors[0].quaternion.values) ** 0.5
        for frame in frames
    ]
    assert min(norms) > 0.9999
    assert max(norms) < 1.0001


def test_offline_monitor_prints_validated_real_fields_without_usb(tmp_path, capsys):
    path = tmp_path / "real_fixture.fhc1poll"
    path.write_bytes(_real_fixture_bytes())
    assert monitor_main(["--capture", str(path), "--count", "1"]) == 0
    output = capsys.readouterr().out
    assert "NO USB OPERATIONS" in output
    assert "quat_norm=" in output
    assert "quaternion:REAL_CAPTURE_VALIDATED" in output
