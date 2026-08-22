import json
from dataclasses import replace
from pathlib import Path
import socket
import struct
import time

import numpy as np
import pytest

from foheart.config import ConfigError, load_config
from foheart.integrations.unitree_g1.adapter import IKResult, SafeG1IK
from foheart.integrations.unitree_g1.sim_bridge import SimStepMetrics
from foheart.integrations.unitree_g1.sinks import (
    RealBackendBlocked,
    RealG1Sink,
    RealSafetyState,
    SimG1Sink,
    production_real_backend_factory,
)
from foheart.mocap.frames import BasisTransform
from foheart.motionvenus.protocol import (
    FULL_BONE_SHORT_NAMES,
    MotionVenusProtocolError,
    MotionVenusStreamDecoder,
    encode_binary_frame,
)
from foheart.motionvenus.retarget import (
    MotionVenusG1Retargeter,
    RetargetProfile,
    average_quaternions_xyzw,
)
from foheart.motionvenus.skeleton import HumanSkeletonFrame
from foheart.motionvenus.synthetic import SYNTHETIC_POSES, synthetic_frame
from foheart.motionvenus.transport import (
    CAPTURE_MAGIC,
    MotionVenusCaptureWriter,
    MotionVenusDatagram,
    MotionVenusReceiver,
    MotionVenusReplaySource,
    MotionVenusWatchdog,
    read_capture,
)
from foheart.tools._motionvenus import MotionVenusFrameSource


ROOT = Path(__file__).resolve().parents[1]


def binary_packet(pose="neutral", frame_number=7):
    return encode_binary_frame(synthetic_frame(pose, frame_number=frame_number, timestamp_ns=1_000_000_000))


def decoded(pose="neutral", frame_number=7, sender=("192.0.2.10", 5001)):
    return MotionVenusStreamDecoder().decode(
        binary_packet(pose, frame_number), received_ns=1_000_000_000, sender=sender
    )


def make_retargeter():
    profile = RetargetProfile.load(ROOT / "config/motionvenus_g1_retarget.yaml")
    left = np.eye(4)
    right = np.eye(4)
    left[:3, 3] = (0.30, 0.20, 0.20)
    right[:3, 3] = (0.30, -0.20, 0.20)
    return MotionVenusG1Retargeter(
        profile,
        robot_neutral_left=left,
        robot_neutral_right=right,
        robot_left_shoulder=np.array((0.0, 0.20, 0.40)),
        robot_right_shoulder=np.array((0.0, -0.20, 0.40)),
    )


def human(pose="neutral", frame_number=7, **kwargs):
    return HumanSkeletonFrame.from_motionvenus(decoded(pose, frame_number), status="LIVE", **kwargs)


def safety(**changes):
    values = {name: True for name in RealSafetyState.__dataclass_fields__}
    values.update(changes)
    return RealSafetyState(**values)


def test_sdk_binary_header_and_23_bone_semantics():
    frame = decoded()
    assert frame.header.protocol_version == 4003
    assert frame.header.avatar_name == "SyntheticActor"
    assert frame.header.frame_number == 7
    assert frame.header.body_skeleton_count == 23
    assert frame.header.attitude_format == "quaternion"
    assert frame.header.skeleton_coordinate == "global"
    assert frame.bone("LeftHand").rotation_global_xyzw == pytest.approx((0, 0, 0, 1))
    assert frame.bone("LeftToe").position_global_m is not None


def test_sdk_binary_53_slot_and_local_rotation_labels():
    packet = bytearray(binary_packet())
    count_offset = 12 + len(b"SyntheticActor")
    packet[count_offset + 1] = packet[count_offset + 2] = 15
    record = struct.pack("<3i4h", 0, 0, 0, 0, 0, 0, 8192)
    full = MotionVenusStreamDecoder().decode(bytes(packet[:128]) + record * 53)
    assert len(full.bones) == 53
    assert full.bones[11].name == "RightHandThumb1"
    assert full.bones[52].name == "LeftToe"

    packet = bytearray(binary_packet())
    packet[18 + len(b"SyntheticActor")] = 0
    local = MotionVenusStreamDecoder().decode(bytes(packet))
    assert local.bone("LeftHand").rotation_global_xyzw is None
    assert local.bone("LeftHand").rotation_local_xyzw == pytest.approx((0, 0, 0, 1))


@pytest.mark.parametrize("change", ("truncate", "trailing", "wrong_count"))
def test_binary_length_and_bone_count_rejection(change):
    packet = bytearray(binary_packet())
    if change == "truncate":
        packet.pop()
    elif change == "trailing":
        packet.append(0)
    else:
        packet[12 + len(b"SyntheticActor")] = 22
    with pytest.raises(MotionVenusProtocolError):
        MotionVenusStreamDecoder().decode(bytes(packet))


def test_unknown_protocol_version_is_distinct_mismatch():
    packet = bytearray(binary_packet())
    struct.pack_into("<H", packet, 0, 9999)
    with pytest.raises(MotionVenusProtocolError) as error:
        MotionVenusStreamDecoder().decode(bytes(packet))
    assert error.value.kind == "protocol_mismatch"


def test_decoder_rejects_unsupported_model_and_suit_type():
    with pytest.raises(ValueError, match="requires 23 bones"):
        MotionVenusStreamDecoder(expected_body_bones=22)
    packet = bytearray(binary_packet())
    packet[3 + len(b"SyntheticActor") + 4] = 2
    with pytest.raises(MotionVenusProtocolError, match="suitType"):
        MotionVenusStreamDecoder().decode(bytes(packet))
    packet = bytearray(binary_packet())
    packet[13 + len(b"SyntheticActor")] = 15
    with pytest.raises(MotionVenusProtocolError, match="23 body or 53"):
        MotionVenusStreamDecoder().decode(bytes(packet))
    packet = bytearray(binary_packet())
    packet[2] = 64
    with pytest.raises(MotionVenusProtocolError, match="below the SDK limit"):
        MotionVenusStreamDecoder().decode(bytes(packet))
    with pytest.raises(ConfigError, match="must be 23"):
        load_config(overrides={"motionvenus": {"expected_body_bones": 22}})


def test_malformed_wire_quaternion_is_rejected():
    packet = bytearray(binary_packet())
    struct.pack_into("<4h", packet, 128 + 12, 0, 0, 0, 0)
    with pytest.raises(MotionVenusProtocolError, match="near zero"):
        MotionVenusStreamDecoder().decode(bytes(packet))


def test_sdk_json_schema_is_supported_exactly():
    document = {
        "streamVer": "V4003",
        "ActorName": "JSONActor",
        "SN": "1A",
        "FrameNumber": 12,
        "SkeletonCount": 53,
        "SkeletonPosition": True,
        "SkeletonQuat": True,
        "SkeletonEuler": False,
        "GlobalCoord": True,
        "LocalCoord": False,
        "RotOrder": "XYZ",
    }
    document.update({name: {"LOC": [0, 0, 1], "KQ": [0, 0, 0, 1]} for name in FULL_BONE_SHORT_NAMES})
    frame = MotionVenusStreamDecoder(packet_format="json").decode(json.dumps(document).encode())
    assert frame.header.suit_number == 0x1A
    assert frame.header.body_skeleton_count == 23
    assert len(frame.bones) == 53
    document["SN"] = 26
    with pytest.raises(MotionVenusProtocolError, match="hexadecimal string"):
        MotionVenusStreamDecoder(packet_format="json").decode(json.dumps(document).encode())


def test_human_skeleton_is_immutable_and_uses_global_se3():
    frame = human()
    assert frame.valid and len(frame.bones) == 23
    assert frame.bone("T8").pose_global.shape == (4, 4)
    with pytest.raises(TypeError):
        frame.bones["extra"] = frame.bone("T8")
    with pytest.raises(KeyError, match="no bone"):
        frame.bone("NotABone")


def test_watchdog_tracks_duplicate_loss_order_sender_and_stale():
    watchdog = MotionVenusWatchdog(stale_after_s=0.1)
    assert watchdog.status(0) == "NO_PACKETS"
    assert watchdog.observe(decoded(frame_number=10), monotonic_ns=1_000_000_000).accepted
    assert not watchdog.observe(decoded(frame_number=10), monotonic_ns=1_010_000_000).accepted
    assert watchdog.observe(decoded(frame_number=13), monotonic_ns=1_020_000_000).event == "gap"
    assert not watchdog.observe(decoded(frame_number=12), monotonic_ns=1_030_000_000).accepted
    assert watchdog.observe(decoded(frame_number=1, sender=("192.0.2.11", 5001)), monotonic_ns=1_040_000_000).accepted
    diagnostics = watchdog.diagnostics(1_200_000_000)
    assert diagnostics.status == "STALE"
    assert diagnostics.duplicate_frames == 1
    assert diagnostics.estimated_lost_frames == 2
    assert diagnostics.out_of_order_frames == 1
    assert diagnostics.sender_changes == 1


def test_capture_replay_preserves_boundaries_timestamps_and_sender(tmp_path):
    path = tmp_path / "capture.bin"
    records = (
        MotionVenusDatagram(binary_packet(frame_number=1), 100, 10, ("192.0.2.20", 5001)),
        MotionVenusDatagram(binary_packet(frame_number=2), 200, 20, ("192.0.2.20", 5001)),
    )
    with MotionVenusCaptureWriter(path) as writer:
        for record in records:
            writer.write(record)
    loaded = list(read_capture(path))
    assert [item.payload for item in loaded] == [item.payload for item in records]
    assert [item.received_ns for item in loaded] == [100, 200]
    assert loaded[0].sender == ("192.0.2.20", 5001)
    replay = MotionVenusReplaySource(path)
    replay.start()
    assert replay.receive().payload == records[0].payload
    assert replay.receive().payload == records[1].payload
    assert replay.receive() is None and replay.eof
    with pytest.raises(FileExistsError):
        MotionVenusCaptureWriter(path).open()
    impossible = tmp_path / "impossible.bin"
    impossible.write_bytes(CAPTURE_MAGIC + struct.pack("!Q4sHI", 1, b"\x7f\x00\x00\x01", 5001, 65536))
    with pytest.raises(ValueError, match="impossible captured UDP payload"):
        list(read_capture(impossible))


def test_udp_loopback_uses_the_same_parser_and_skeleton():
    try:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    except PermissionError:
        pytest.skip("sandbox forbids loopback sockets; bounded loopback is run in final validation")
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()
    receiver = MotionVenusReceiver("127.0.0.1", port, timeout_s=0.5)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        receiver.start()
        for frame_number in (20, 21, 22):
            sender.sendto(binary_packet("arms_forward", frame_number), ("127.0.0.1", port))
        datagram = receiver.receive_latest()
        assert datagram is not None
        frame = MotionVenusStreamDecoder().decode(
            datagram.payload, received_ns=datagram.received_ns, sender=datagram.sender
        )
        skeleton = HumanSkeletonFrame.from_motionvenus(frame, status="LIVE")
        assert skeleton.valid and skeleton.motionvenus_frame_number == 22
        assert datagram.sender[1] == sender.getsockname()[1]
        assert receiver.stats.packets == 3
        assert receiver.stats.backlog_drops == 2
    finally:
        sender.close()
        receiver.close()


def test_retarget_neutral_scaling_left_right_and_orientation():
    neutral_retargeter = make_retargeter()
    neutral = neutral_retargeter.retarget(human("neutral", 1))
    forward_retargeter = make_retargeter()
    forward = forward_retargeter.retarget(human("arms_forward", 2))
    assert not np.allclose(neutral.left[:3, 3], forward.left[:3, 3])
    assert forward.left[1, 3] > forward.right[1, 3]
    wrist_retargeter = make_retargeter()
    rotated = wrist_retargeter.retarget(human("wrist_rotations", 3))
    assert not np.allclose(rotated.left[:3, :3], rotated.right[:3, :3])


def test_retarget_rejects_stale_and_requires_proper_basis(tmp_path):
    retargeter = make_retargeter()
    with pytest.raises(ValueError, match="stale"):
        retargeter.retarget(human(stale=True))
    data = (ROOT / "config/motionvenus_g1_retarget.yaml").read_text()
    path = tmp_path / "reflection.yaml"
    path.write_text(data.replace("- [-1.0, 0.0, 0.0]", "- [1.0, 0.0, 0.0]"))
    with pytest.raises(ValueError, match="right-handed"):
        RetargetProfile.load(path)


def test_quaternion_safe_neutral_capture_handles_sign_equivalence(tmp_path):
    result = average_quaternions_xyzw(((0, 0, 0, 1), (0, 0, 0, -1)))
    assert abs(result[3]) == pytest.approx(1)
    frames = [human("neutral", number) for number in (1, 2, 3)]
    profile = RetargetProfile.capture(
        frames,
        motionvenus_to_project=BasisTransform.from_axis_map(
            ("y", "x", "z"), (1, -1, 1), "motionvenus", "project"
        ),
        project_to_g1=BasisTransform.identity("project", "g1"),
    )
    assert profile.status == "SOFTWARE_CONFIGURED"
    assert profile.neutral["LeftHand"].position_m == pytest.approx((-0.32, 0, 0.95), abs=1e-5)
    output = tmp_path / "retarget.yaml"
    profile.save(output)
    assert RetargetProfile.load(output).neutral["LeftHand"] == profile.neutral["LeftHand"]
    with pytest.raises(FileExistsError):
        profile.save(output)


def test_retarget_requires_complete_global_arm_chains():
    frame = decoded()
    bones = tuple(
        replace(bone, rotation_global_xyzw=None) if bone.name == "LeftForeArm" else bone
        for bone in frame.bones
    )
    incomplete = HumanSkeletonFrame.from_motionvenus(replace(frame, bones=bones), status="LIVE")
    with pytest.raises(ValueError, match=r"global position\+quaternion"):
        make_retargeter().retarget(incomplete)


def test_all_requested_synthetic_motionvenus_poses_are_finite():
    assert set(SYNTHETIC_POSES) >= {
        "neutral", "arms_forward", "t_pose", "left_elbow_flex", "right_elbow_flex",
        "left_arm_raise", "right_arm_raise", "symmetric_reach", "wrist_rotations", "torso_yaw",
    }
    for number, pose in enumerate(SYNTHETIC_POSES):
        skeleton = HumanSkeletonFrame.from_motionvenus(decoded(pose, number), status="LIVE")
        assert skeleton.valid
        assert all(np.isfinite(bone.position_global_m).all() for bone in skeleton.bones.values())


def test_synthetic_neutral_capture_source_does_not_average_movement_poses():
    source = MotionVenusFrameSource(
        "synthetic", bind="127.0.0.1", port=5001, packet_format="binary",
        timeout_s=0.1, expected_body_bones=23, synthetic_poses=("neutral",),
    )
    source.start()
    try:
        hands = [source.receive().bone("LeftHand").position_global_m for _ in range(12)]
    finally:
        source.close()
    assert hands == [(-0.32, 0.0, 0.95)] * 12


class FakeSolver:
    lower_limits = np.full(14, -2.0)
    upper_limits = np.full(14, 2.0)

    def solve_ik(self, left, right, q, dq):
        value = np.zeros(14)
        value[0], value[7] = left[0, 3], right[0, 3]
        return value, np.zeros(14)


class FakeBridge:
    def __init__(self):
        self.commands = []

    def command(self, q, *, steps):
        self.commands.append(np.asarray(q).copy())
        return SimStepMetrics(0.0, 0.0, 0.0, True, steps)


def test_motionvenus_to_safe_ik_and_sim_sink_is_finite():
    targets = make_retargeter().retarget(human("symmetric_reach", 2))
    result = SafeG1IK(FakeSolver()).solve(targets)
    bridge = FakeBridge()
    sink = SimG1Sink(bridge, steps_per_update=8)
    metrics = sink.update(result)
    assert result.valid and metrics.finite and len(bridge.commands) == 1
    assert sink.mode == "SIMULATION_ONLY"


class FakeRealBackend:
    def __init__(self):
        self.sent = []
        self.closed = False

    def current_arm_positions(self):
        return np.zeros(14)

    def send_arm_positions(self, q, tau):
        self.sent.append((q.copy(), tau.copy()))

    def close(self):
        self.closed = True


def test_real_sink_confirmation_ramp_hold_and_close_use_fake_only():
    backend = FakeRealBackend()
    sink = RealG1Sink(lambda: backend, ramp_time_s=1.0, max_joint_delta_rad=0.1)
    sink.mark_ready(safety())
    with pytest.raises(ValueError, match="ENABLE"):
        sink.enable("yes", safety())
    sink.enable("ENABLE", safety())
    result = IKResult(np.full(14, 0.05), np.zeros(14), True, False, False, "")
    command = sink.update(result, safety(), monotonic_ns=sink.enabled_ns + 500_000_000)
    assert command == pytest.approx(np.full(14, 0.025))
    assert len(backend.sent) == 1
    held = sink.update(result, safety(input_fresh=False))
    assert held == pytest.approx(command)
    assert len(backend.sent) == 1 and sink.state == "HOLDING"
    sink.close()
    assert backend.closed and sink.state == "CLOSED"


def test_real_sink_rate_limits_a_far_startup_target_instead_of_deadlocking():
    backend = FakeRealBackend()
    sink = RealG1Sink(lambda: backend, ramp_time_s=1.0, max_joint_delta_rad=0.1)
    sink.mark_ready(safety())
    sink.enable("ENABLE", safety())
    result = IKResult(np.full(14, 0.8), np.zeros(14), True, False, False, "")
    command = sink.update(result, safety(), monotonic_ns=sink.enabled_ns + 500_000_000)
    assert command == pytest.approx(np.full(14, 0.1))
    assert len(backend.sent) == 1 and sink.state == "ENABLED"


def test_production_real_backend_fails_before_robot_construction():
    with pytest.raises(RealBackendBlocked, match="starts publishing"):
        production_real_backend_factory()
