# CODEX implementation report

> This is the phase-one foundation report. Static protocol findings and parser
> status were superseded on 2026-08-20 by
> `CODEX_PROTOCOL_RECOVERY_REPORT.md`; the original results below are retained
> as historical implementation evidence.
>
> Runtime configuration and selection behavior added later is documented in
> `CODEX_CONFIG_REPORT.md`.

## What was implemented

- Installable Python 3.10+ package using PyUSB, libusb, NumPy, and pytest
- Known FOHEART product discovery with safe string/configuration inspection
- Actual descriptor printing and descriptor-driven bulk/interrupt selection
- `C1Device` context manager, reversible kernel-driver detach/reattach,
  interface claim/release, and raw read transport
- Versioned raw transfer recording and deterministic replay
- Protocol-independent sensor dataclasses and an explicit undecoded parser boundary
- Terminal discovery, raw dump, replay, real readiness, and labelled mock monitor tools
- Tests for recognition, endpoint selection, ambiguity, absent/denied devices,
  recording/replay, models, malformed parsing, and mock generation
- Static-analysis evidence for transfer wrappers and a partial record layout

No guessed command, control transfer, sensor data, quaternion order, checksum, or
outer packet framing was implemented.

## Files created/modified

All files are new beneath `foheart_linux/`: package sources in `src/foheart/`,
three test modules, three protocol/architecture documents, README, pyproject,
sample documentation, udev template, and this report.

## Commands to install

```bash
cd foheart_linux
sudo apt install libusb-1.0-0 libusb-1.0-0-dev
python -m venv .venv
source .venv/bin/activate
pip install -e '.[dev]'
```

## Commands to test

```bash
pytest
```

Actual isolated result on 2026-08-20: `12 passed`. The host exports a
ROS `PYTHONPATH`, so verification used `env -u PYTHONPATH .venv/bin/pytest -q`;
this project does not depend on or load ROS.

## Commands to run

```bash
python -m foheart.tools.discover
python -m foheart.tools.usb_dump --pid 0x5751 --count 100 --timeout-ms 1000 --output samples/c1_capture.bin --hex
python -m foheart.tools.replay samples/c1_capture.bin
python -m foheart.tools.monitor
python -m foheart.tools.monitor --mock --count 10
```

## Static reverse engineering discovered

For `fhusb.dll` SHA-256
`ce9049b82f5e8d06f7faedb42605379a8e47dab775a2f9e9888b79f81a15defa`:

- Interface number: `0`, wrapper `0x10023780`, HIGH for this DLL build
- Bulk IN: `0x81`, call site `0x100237c7`, HIGH for this DLL build
- Bulk OUT: `0x01`, call site `0x10023881`, HIGH for this DLL build
- Interrupt IN/OUT: `0x81` / `0x01`, sites `0x10023944` / `0x1002399e`
- HS bulk read length: `0x1400`, site `0x100237b5`; other calls use `0x40`
- Timeouts are call-specific: `0`, `100`, and `500` ms; no single timeout recovered
- Decoded C++ `SensorFrame` ABI size: `0x381` (897), copy at `0x1000a7b3`;
  this is not established as an on-wire frame size
- One fixed raw sensor record variant is `0x22` (34) bytes, caller
  `0x1002099d`; quaternion is four signed 16-bit values at record offset `0x14`,
  parsed by `0x10006d30`; component order remains unknown

Full evidence snippets and confidence are in `docs/protocol_status.md` and
`docs/reverse_engineering_notes.md`.

## Protocol status and blockers

- Initialization/start stream: UNKNOWN. A candidate bulk write starts with
  `0x70` in a `0x200`-byte buffer, but its complete semantics are unproven and it
  is not sent.
- SensorFrame outer packet layout: UNKNOWN.
- Quaternion layout: raw record offset recovered for one variant; order UNKNOWN.
- Checksum, frame counter, sensor count, and universal sensor ID offset: UNKNOWN.
- Hardware was absent, so opening and raw reads are not validated claims.

## Exact next step with C1 hardware

Run discovery, save `lsusb -v -d 1483:5751` (or `1483:5851`), then run a
read-only `usb_dump`. If no transfers arrive without output, capture the official
Windows initialization exchange with usbmon/Wireshark before implementing any
write. Record simultaneous controlled motion of one labelled sensor to validate
the 34-byte record framing, ID mapping, scaling, and quaternion order.

=== INFORMATION FOR CHATGPT ===

USB discovery implementation: IMPLEMENTED, hardware not validated
Transport implementation: IMPLEMENTED, descriptor-driven bulk/interrupt
1483:5751 support: IMPLEMENTED, hardware not validated
1483:5851 support: IMPLEMENTED, hardware not validated

Interface: 0 (STATIC EVIDENCE; dynamically selected at runtime)
Bulk IN endpoint: 0x81 (STATIC EVIDENCE; dynamically selected at runtime)
Bulk OUT endpoint: 0x01 (STATIC EVIDENCE; dynamically selected at runtime)
Interrupt endpoint: IN 0x81, OUT 0x01 (STATIC EVIDENCE; dynamically selected)
USB read size: 0x1400 for one HS bulk wrapper; 0x40 elsewhere; runtime default is descriptor wMaxPacketSize
Timeout: UNKNOWN universal value; static call sites use 0/100/500 ms; CLI is configurable

Start-stream command: UNKNOWN; candidate pre-read buffer starts 0x70, not sent
SensorFrame size: 0x381 decoded in-memory ABI; on-wire frame size UNKNOWN
Sensor record size: 0x22 for one fixed raw record variant
Sensor ID offset: UNKNOWN on wire; decoded packed index at 0x34d bits 0..5
Quaternion offset: 0x14 in one 0x22-byte raw record variant
Quaternion order: UNKNOWN
Frame counter: UNKNOWN
Checksum: UNKNOWN

Can open C1 on Linux: NOT VALIDATED (hardware absent)
Can read raw transfers: IMPLEMENTED, NOT VALIDATED (hardware absent/start command unknown)
Can decode one sensor quaternion: NO
Can decode all sensors: NO

Current blocker: No real C1 descriptors/capture and unresolved outer framing/start sequence
Most useful file/function to inspect next: fhusb.dll worker 0x100204e0 and parser dispatch 0x100206c0..0x10021004, then validate against a real capture
