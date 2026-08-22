"""Small shared source wrapper for MotionVenus command-line tools."""

from __future__ import annotations

from pathlib import Path

from foheart.motionvenus.protocol import MotionVenusFrame, MotionVenusStreamDecoder
from foheart.motionvenus.synthetic import SyntheticMotionVenusSource
from foheart.motionvenus.transport import (
    MotionVenusDatagram,
    MotionVenusReceiver,
    MotionVenusReplaySource,
)


class MotionVenusFrameSource:
    def __init__(
        self,
        source: str,
        *,
        bind: str,
        port: int,
        packet_format: str,
        timeout_s: float,
        expected_body_bones: int,
        replay: Path | None = None,
        synthetic_fps: float = 60.0,
        synthetic_poses: tuple[str, ...] | None = None,
    ):
        self.source_name = source.upper()
        self.decoder = MotionVenusStreamDecoder(
            expected_body_bones=expected_body_bones,
            packet_format=packet_format,
        )
        if source == "live":
            self.source = MotionVenusReceiver(bind, port, timeout_s=timeout_s)
        elif source == "replay":
            if replay is None:
                raise ValueError("--replay is required with --source replay")
            self.source = MotionVenusReplaySource(replay)
        elif source == "synthetic":
            self.source = (
                SyntheticMotionVenusSource(fps=synthetic_fps)
                if synthetic_poses is None
                else SyntheticMotionVenusSource(fps=synthetic_fps, poses=synthetic_poses)
            )
        else:
            raise ValueError("source must be live, replay, or synthetic")
        self.last_datagram: MotionVenusDatagram | None = None

    def start(self) -> None:
        self.source.start()

    @property
    def eof(self) -> bool:
        return bool(getattr(self.source, "eof", False))

    @property
    def stats(self):
        return getattr(self.source, "stats", None)

    def receive(self) -> MotionVenusFrame | None:
        value = self.source.receive()
        return self._decode(value)

    def receive_latest(self) -> MotionVenusFrame | None:
        receive = getattr(self.source, "receive_latest", self.source.receive)
        return self._decode(receive())

    def _decode(self, value) -> MotionVenusFrame | None:
        if value is None:
            return None
        if isinstance(value, MotionVenusFrame):
            self.last_datagram = None
            return value
        self.last_datagram = value
        return self.decoder.decode(value.payload, received_ns=value.received_ns, sender=value.sender)

    def close(self) -> None:
        self.source.close()
