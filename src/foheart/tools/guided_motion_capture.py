from __future__ import annotations

import json
from pathlib import Path

from foheart.mocap.motion import analyze_motion_capture, infer_axis_mapping
from foheart.protocol.frame import PollRecorder
from foheart.protocol.poll import C1_HID_POLL
from foheart.tools.motion_analyze import print_analysis
from foheart.usb.c1_device import C1NotFoundError, C1OpenError
from foheart.usb.c1_poll import (
    C1_HID_CAPTURE_MAX_POLLS,
    C1_HID_CAPTURE_MAX_RUNTIME_S,
    C1_HID_POLL_TIMEOUT_MS,
    capture_polls,
)

PHASES = (
    (
        "baseline",
        Path("samples/motion_baseline.bin"),
        """PHASE 0 — BASELINE

Place sensor flat on table.
TOP face upward.
FRONT edge away from you.

Do not touch sensor.""",
    ),
    (
        "table_yaw_cw",
        Path("samples/motion_table_yaw_cw.bin"),
        """PHASE A — TABLE YAW

Start with:
TOP upward
FRONT away from you

During capture:

hold still briefly
→ rotate sensor approximately 90 degrees CLOCKWISE
  while keeping it flat on the table
→ hold still at final orientation""",
    ),
    (
        "forward_tilt",
        Path("samples/motion_forward_tilt.bin"),
        """PHASE B — FORWARD TILT

Start with:
TOP upward
FRONT away from you

During capture:

hold still briefly
→ rotate the FRONT edge downward / tilt sensor forward
  approximately 60-90 degrees
→ hold still""",
    ),
    (
        "right_roll",
        Path("samples/motion_right_roll.bin"),
        """PHASE C — RIGHT ROLL

Start with:
TOP upward
FRONT away from you

During capture:

hold still briefly
→ lower the RIGHT side / rotate sensor toward the right
  approximately 60-90 degrees
→ hold still""",
    ),
)
SUMMARY_PATH = Path("samples/motion_validation_summary.json")


def _save_capture(path: Path):
    with path.open("xb") as stream:
        recorder = PollRecorder(stream)
        capture = capture_polls(
            max_polls=C1_HID_CAPTURE_MAX_POLLS,
            timeout_ms=C1_HID_POLL_TIMEOUT_MS,
            max_runtime_s=C1_HID_CAPTURE_MAX_RUNTIME_S,
        )
        for record in capture.records:
            recorder.write(record)
    return capture


def main() -> int:
    outputs = [path for _, path, _ in PHASES] + [SUMMARY_PATH]
    existing = [path for path in outputs if path.exists()]
    if existing:
        print("ERROR: refusing to overwrite existing output:")
        for path in existing:
            print(f"  {path}")
        return 2
    SUMMARY_PATH.parent.mkdir(parents=True, exist_ok=True)

    print("=" * 72)
    print("EXPERIMENTAL FOHEART C1 GUIDED CONTROLLED-MOTION CAPTURE")
    print("PHYSICAL TEST FRAME")
    print()
    print("Choose a physical TOP face of the sensor.")
    print("Choose a physical FRONT edge of the sensor.")
    print("Keep these definitions unchanged for the complete test.")
    print()
    print("Place the sensor flat on a table:")
    print("TOP = facing upward")
    print("FRONT = pointing away from you")
    print("RIGHT = the edge on your right")
    print()
    print("Authorized USB payload: 0x70 + 63 zero bytes only")
    print(f"Exact payload: {C1_HID_POLL.hex(' ')}")
    print("Each phase: exactly 200 bounded poll attempts; no retry or other command")
    print("The previous hardware rate implies each phase may last only about 1-2 seconds.")
    print("=" * 72)

    analyses: dict[str, dict[str, object]] = {}
    baseline: dict[str, object] | None = None
    total_out = 0
    try:
        for name, path, prompt in PHASES:
            print(f"\n{prompt}")
            input("\nPress ENTER when ready; capture begins immediately. ")
            capture = _save_capture(path)
            total_out += sum(record.out_transferred == 64 for record in capture.records)
            print(f"Saved {len(capture.records)} bounded poll attempts to {path}")
            if capture.hard_error:
                print(f"HARD STOP: {capture.stop_reason}")
                return 3
            if len(capture.records) != C1_HID_CAPTURE_MAX_POLLS:
                print(f"STOP: incomplete bounded phase ({capture.stop_reason})")
                return 3
            analysis = analyze_motion_capture(path, baseline=baseline)
            analyses[name] = analysis
            print_analysis(analysis)
            if name == "baseline":
                baseline = analysis
    except KeyboardInterrupt:
        print("\nSTOP: guided capture interrupted; no automatic retry")
        return 130
    except (C1NotFoundError, C1OpenError, OSError, ValueError) as exc:
        print(f"HARD STOP: {exc}")
        return 3

    motions = {name: analyses[name] for name in ("table_yaw_cw", "forward_tilt", "right_roll")}
    summary = {
        "schema_version": 1,
        "physical_reference": {
            "name": "PHYSICAL TEST FRAME",
            "TOP": "chosen face pointing upward in baseline",
            "FRONT": "chosen edge pointing away from operator in baseline",
            "RIGHT": "edge on operator's right in baseline",
        },
        "usb_safety": {
            "payload": "70 + 63 zero bytes",
            "out_endpoint": "0x01",
            "in_endpoint": "0x81",
            "polls_per_phase": 200,
            "successful_64_byte_out_transfers": total_out,
            "any_other_payload": False,
        },
        "baseline": analyses["baseline"],
        "motions": motions,
        "axis_mapping": infer_axis_mapping(motions),
    }
    SUMMARY_PATH.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(f"\nMachine-readable summary: {SUMMARY_PATH}")
    print(f"Axis mapping result: {summary['axis_mapping']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
