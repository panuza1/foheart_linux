from foheart.protocol.frame import MAGIC, RawRecorder, RawTransfer, iter_recording
from foheart.tools.monitor import generate_mock_frame
from foheart.tools.replay import replay_transfers


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
