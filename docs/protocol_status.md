# Protocol status

Static source binaries:

- `fhusb.dll` SHA-256 `ce9049b82f5e8d06f7faedb42605379a8e47dab775a2f9e9888b79f81a15defa`
- `MotionVenus_3.2.0_setup.exe` SHA-256 `473692ab1ea10dbcda8cb1c7ef996b7b0cbf609099ba57c5bab89f02553b7e0e`

Addresses are virtual addresses in the named 32-bit PE image. Static findings
describe this software build. The separate real-hardware section records the
only facts validated against the connected C1 on 2026-08-21.

## REAL HARDWARE VALIDATED — 2026-08-21

The connected router remained `1483:5851`, configuration 1, interface 0,
alternate 0, interrupt OUT `0x01`, interrupt IN `0x81`, 64-byte maximum packets.
After both hardware runs it remained enumerated and the interface was rebound to
`usbhid`.

The only host payload sent was exactly 64 bytes:

```text
70 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
```

One one-shot poll returned one 64-byte report beginning `0x15` in 0.851 ms.
The gated bounded run then completed 200 polls and 200 reads with no timeout,
short transfer, reset, disconnect, or other error. All 200 IN reports were 64
bytes and began `0x15`; neither `74 01`, `0x13`, nor `0x22` was observed. The
boundary-preserving capture is `samples/c1_real_poll_capture.bin`, SHA-256
`837804311fe3996adb9176c5a8ec8014c9fdcbb46240b3b887ce5d072d8e4392`.

### Real HID `0x15` compact report

`USBHidReadWorkerHs` reads 64 bytes directly into its report buffer at
`0x1002237e..0x100223ac`, compares byte 0 with `0x15` at
`0x10022986..0x10022997`, and passes the same buffer (no offset/reassembly) to
decoder `0x100070b0` in mode 3 at `0x10022a70..0x10022a8c`. Byte 0 is therefore
a FOHEART message code, not an HID report ID, on this path.

Mode 3 uses an 11-byte header and then consumes fields in flag order:

| Captured offset | Size | Field | Conversion | Static instruction/data flow | Real evidence | Status |
|---:|---:|---|---|---|---|---|
| `0x00` | 1 | message code `0x15` | none | dispatch `0x10022994` | 200/200 reports | REAL HARDWARE VALIDATED |
| `0x01..0x04` | 4 | identity-like raw value | UNKNOWN | copied to decoded `+0x34d` at `0x10007618..0x1000761e` | constant `00 dd 03 14` | UNKNOWN semantics |
| `0x05..0x06` | 2 | counter-like raw value | UNKNOWN | copied to decoded `+0x35a` at `0x100075ff..0x1000760c` | increments by 1 or 2 | UNKNOWN semantics |
| `0x07..0x0a` | 4 | field/status flags | little-endian | copied to decoded `+0xd6` at `0x10007624..0x10007630` | core low bits always `0x1d` | PARTIAL |
| `0x0b..0x12` | 8 | quaternion W/X/Y/Z | signed int16 / `16384` | mode-3 bit-0 path `0x1000764c..0x100076b6` | norm `0.999935..0.999994` | REAL_CAPTURE_VALIDATED |
| `0x13..0x18` | 6 | accel X/Y/Z | signed int16 / `2048` | common bit-2 path `0x100079c9..0x10007a4a` | stationary norm `0.9686..0.9867` | REAL_CAPTURE_VALIDATED |
| `0x19..0x1e` | 6 | gyro X/Y/Z | signed int16 / float32 `16.4` | common bit-3 path `0x10007d5e..0x10007ddf` | near zero while stationary | REAL_CAPTURE_VALIDATED |
| `0x1f..0x24` | 6 | magnetometer X/Y/Z | signed int16 / `12000` | common bit-4 path `0x10007f11..0x10007f92` | stable norm `0.6061..0.6284` | REAL_CAPTURE_VALIDATED |
| `0x25..0x3f` | 27 | optional fields and zero padding | UNKNOWN | higher flag paths remain only partly traced | variable through byte 45; bytes 46..63 zero | UNKNOWN |

The captured flags were `0x8c1d` (193), `0x8d1d` (3), `0xac1d` (3), and
`0xec1d` (1). Bit 1 (matrix) and bit 5 (Euler) were absent in every report.
Consequently Euler remains STATIC_ONLY and is deliberately not decoded by the
HID parser. High flag bits and trailing bytes remain UNKNOWN.

The real parser exposes one capture-local `slot_0` per report. It does not map
bytes `01..04` to a physical sensor identity. Across the stationary-only run,
the mean/max consecutive quaternion change was about `0.00251/0.02179` degrees
and first-to-last change was `0.02211` degrees. No controlled movement occurred,
so motion correlation and axis conventions remain UNKNOWN.

The first-to-last IN wire span was 1.073789 s (186.26 reports/s overall).
Reports 107..200 followed a backlog-like fast prefix and measured about 92.19
reports/s; this is an observation, not a configured FPS claim.

### CONTROLLED MOTION VALIDATION — PARTIAL

A later guided session superseded the earlier PyUSB backend failure. Four raw
captures each contain 200 poll records, 200 successful 64-byte IN reports, no
timeout/error, and only message `0x15`. This documentation review performed no
USB activity.

The physical test used TOP upward, the circular sensor's visible groove pointing
away as FRONT, and the operator's right as RIGHT. The baseline is stationary:
quaternion norms are `0.999939..0.999997`, consecutive angular change is
`0.002846` degrees mean / `0.009890` maximum, and accel mean is
`(-0.047400, -0.005029, +1.024961)`. Thus +AZ is the dominant decoded gravity
component for TOP-up in this pose (CONTROLLED_MOTION_VALIDATED for this limited
gravity observation).

The requested three-region motion was not captured cleanly:

- TABLE_YAW_CW is stationary within its file (`0.2121` degrees first-to-last),
  but its mean pose is `91.6906` degrees from baseline about a nearly pure
  `-QZ` axis. The target yaw pose is supported; the yaw gyro response is not.
- FORWARD_TILT contains motion throughout (`199/200` samples active), no
  stationary bookends, and only `2.6860` degrees within-file quaternion change.
  Its accel mean `(-0.220920, -0.793210, +0.583564)` confirms a distinct tilted
  pose, but quaternion/gyro axis correlation is not clean.
- RIGHT_ROLL likewise contains motion throughout, no stationary bookends, and
  only `1.4619` degrees within-file quaternion change. Its accel mean
  `(-0.869495, +0.003984, +0.525356)` strongly differentiates right-side-down;
  the small captured quaternion segment is QY-positive and the peak gyro is
  GY-positive, but the full requested rotation was missed.

Status: real pose/motion evidence is PARTIAL. Physical UP rotation ↔ QZ is a
candidate, and physical FRONT rotation ↔ QY/GY is a weaker candidate. Physical
RIGHT rotation, complete sign mapping, quaternion/gyro agreement, and
handedness remain UNKNOWN. No physical mapping can be promoted as a validated
default. The downstream transform and neutral-calibration software therefore
uses explicit `CONFIGURED` values and preserves this uncertainty.

Offline WXYZ analysis preserves raw quaternions separately from q/-q
continuity-adjusted values; no raw capture was changed.

### Downstream software boundary

The protocol decoder now emits raw `SensorSample` values with capture-local
`slot_0` and field evidence. Frame conversion, neutral calibration,
seven-role body mapping, upper-body FK, and G1 retargeting consume derived
values in separate modules; none alters these wire-level facts. The configured
seven-role synthetic path through the existing G1 IK and real local MuJoCo G1
model is `SIM_VALIDATED`. Real multi-sensor body mapping remains pending.

## CONFIRMED STATICALLY

### USB calls used by `fhusb.dll`

| Value | Function/address and instruction evidence | Confidence |
|---|---|---|
| interface `0` | `fhusb.dll` wrapper `0x10023780`: pushes `0` before `libusb_claim_interface` at `0x1002378c` | HIGH |
| bulk IN `0x81`, capacity `0x1400`, timeout `0` | wrapper `0x10023780`, argument setup `0x100237af..0x100237c7` | HIGH |
| bulk OUT `0x01`, length `0x200`, timeout `0` | wrapper `0x100238a0`, argument setup `0x100238cc..0x100238e1` | HIGH |
| interrupt IN/OUT `0x81`/`0x01`, length `0x40`, timeout `100` | wrappers `0x10023920` and `0x10023980`, calls at `0x10023944` and `0x1002399e` | HIGH |
| alternate bulk transfers use `0x40`, timeout `500` | wrappers at `0x10023800` and `0x10023850` | HIGH |

These are observations, not Linux endpoint defaults. Runtime descriptor selection remains
authoritative.

### Fixed `0x22` sensor record

`fhusb.dll` parser `0x10006d30` uses signed loads (`movsx WORD`) for all 16-bit
fields. Its caller computes a `0x22` stride at `0x1002099d`. Constants are
float32 values in `.rdata`.

| Offset | Size | Raw type | Meaning | Scaling | Function/address and data-flow evidence | Confidence |
|---:|---:|---|---|---|---|---|
| `0x00` | 2 | opaque | UNKNOWN; ignored by this parser | UNKNOWN | `0x10006d30..0x1000701b` never reads raw offsets `0x00..0x01` | HIGH that it is unused here; LOW meaning |
| `0x02` | 2 | int16 | accel X | raw / `2048.0` | load `0x10006e0c`, divide by `[0x10029730]` at `0x10006e15`, store decoded `+0x50` | HIGH |
| `0x04` | 2 | int16 | accel Y | raw / `2048.0` | load `0x10006e39`, same constant, store decoded `+0x54` at `0x10006e55` | HIGH |
| `0x06` | 2 | int16 | accel Z | raw / `2048.0` | load `0x10006e65`, same constant, store decoded `+0x58` at `0x10006e80` | HIGH |
| `0x08` | 2 | int16 | gyro X | raw / float32 `16.4` | load `0x10006e91`, divide by `[0x10029720]` at `0x10006e9a`, store decoded `+0x70` | HIGH |
| `0x0a` | 2 | int16 | gyro Y | raw / float32 `16.4` | load `0x10006ebe`, same constant, store decoded `+0x74` at `0x10006eda` | HIGH |
| `0x0c` | 2 | int16 | gyro Z | raw / float32 `16.4` | load `0x10006eea`, same constant, store decoded `+0x78` at `0x10006f05` | HIGH |
| `0x0e` | 2 | int16 | magnetometer X | raw / `12000.0` | load `0x10006f16`, divide by `[0x10029740]` at `0x10006f1f`, store decoded `+0x7c` | HIGH |
| `0x10` | 2 | int16 | magnetometer Y | raw / `12000.0` | load `0x10006f43`, same constant, store decoded `+0x80` at `0x10006f5f` | HIGH |
| `0x12` | 2 | int16 | magnetometer Z | raw / `12000.0` | load `0x10006f6f`, same constant, store decoded `+0x84` at `0x10006f8a` | HIGH |
| `0x14` | 2 | int16 | quaternion W | raw / `16384.0` | load `0x10006d5a`, divide by `[0x10029744]`, sequential decoded component 0; MotionVenus order proof below | HIGH |
| `0x16` | 2 | int16 | quaternion X | raw / `16384.0` | load `0x10006d87`, same constant, sequential decoded component 1 | HIGH |
| `0x18` | 2 | int16 | quaternion Y | raw / `16384.0` | load `0x10006db3`, same constant, sequential decoded component 2 | HIGH |
| `0x1a` | 2 | int16 | quaternion Z | raw / `16384.0` | load `0x10006ddf`, same constant, sequential decoded component 3 | HIGH |
| `0x1c` | 2 | int16 | Euler X | raw / `128.0` | load `0x10006f9b`, divide by `[0x10029724]`, store decoded `+0x38` | HIGH |
| `0x1e` | 2 | int16 | Euler Y | raw / `128.0` | load `0x10006fc8`, same constant, store decoded `+0x3c` at `0x10006fe4` | HIGH |
| `0x20` | 2 | int16 | Euler Z | raw / `128.0` | load `0x10006ff4`, same constant, store decoded `+0x40` at `0x1000700f` | HIGH |

The gyro divisor stored at `0x10029720` is the float32 representation
`16.399999618530273`; `16.4` is the corresponding source-level literal.

### Quaternion order and normalization

The raw parser preserves component order from raw offsets `0x14..0x1a` into
decoded components 0..3 (`fhusb.dll` `0x10006d4f..0x10006dfb`). The getter
`getLastQuatByIndexInSuit` at `0x1000b010` returns those four components in
sequence without arithmetic.

At `MotionVenus_3.2.0_setup.exe` `0x00e0c89f..0x00e0c916`, MotionVenus obtains
the returned vector and supplies vector indices `1,2,3,0` as the logical
`x,y,z,w` parameters of imported `osg::Quat(double x,double y,double z,double
w)`. The x86 pushes appear in index order `0,3,2,1` because arguments are
pushed right-to-left. A second equivalent path exists at
`0x00e0f201..0x00e0f299`. Therefore the returned and raw component order is
`w,x,y,z` (HIGH).

No magnitude normalization occurs in the fixed raw parser, the observed getter,
or the observed MotionVenus handoff; those paths only divide each component by
`16384.0`. The real HID capture produced norms within `0.000066` of 1 without
host renormalization (REAL_CAPTURE_VALIDATED for this run). Whether firmware
guarantees normalization in all modes remains UNKNOWN.

### Bulk-HS fixed message `0x13`

The branch below is one message variant, not a universal C1 frame definition.

| Offset/size | Meaning | Function/address and instruction evidence | Confidence |
|---|---|---|---|
| `+0x00`, 1 byte | message code `0x13` | dispatch comparison at `0x10020916` | HIGH |
| `+0x01`, 4 bytes | identity prefix passed to each decoded record | load dword at `0x10020996`; parser ORs it with loop index at `0x10006d3f..0x10006d49` | HIGH for data flow; exact bit semantics PARTIAL |
| `+0x05`, 1 byte | record-format selector: `0` = `0x22`, `1` = `0x1b` | tests at `0x10020967..0x10020983`; calls `0x10006d30` or `0x10007020` | HIGH |
| `+0x06..+0x0d`, 8 bytes | UNKNOWN | record base derived as block `+0x0e` at `0x1002099d..0x100209ab` | HIGH boundary; LOW meaning |
| `+0x0e` | record-slot area | pointer expression at `0x1002099d..0x100209ab` | HIGH |
| total copied bytes `0x88e` | fixed block copied before parsing | `push 0x88e; memcpy` at `0x1002091f..0x10020937` | HIGH |

`0x88e == 0x0e + 64 * 0x22`, so format 0 has storage for 64 record slots.
This equality is corroborated by the actual header base, stride, and copy size;
it does not prove that 64 sensors are present. The worker loop is hard-coded to
indices 1 through 4 (`0x1002093f..0x10020961`) and skips slot 0. It does not read
a record-count field.

The bulk read accepts up to `0x1400` bytes and stores an actual length, but this
branch does not compare that length with `0x88e` before copying. It dispatches
once on byte 0 and contains no sync scan, remainder loop, `memmove`, or
accumulation on this path (`0x100206c0..0x10021004`). Statically, it treats one
successful transfer as one message. USB transfer atomicity and aggregation still
require hardware validation.

### Sensor index mapping

The fixed parser does not obtain the sensor index from the 0x22 bytes. The worker
passes loop index 1..4 separately at `0x1002098e..0x100209b2`; parser
`0x10006d3f..0x10006d49` ORs it with the outer dword and stores the packed value
at decoded offset `0x34d`.

- `fhusb.dll` helper `0x10017bb0` returns `[frame+0x34d] & 0x3f`: six-bit sensor index (HIGH).
- helper `0x10017c10` returns bits 6..7; their meaning is UNKNOWN.
- `MotionCaptureSuit` routing at `0x10010737..0x1001074d` compares
  `packed >> 8` with `getSuitNumber` and uses `packed & 0x3f` as the sensor index
  (HIGH). Thus bits 8..31 act as a suit-number field in that path.

The ignored raw word at record `+0x00` could be redundant metadata, but there is
no evidence identifying it as a sensor ID.

### Function/message values

These are inner MC1507 request function bytes, not outer USB message codes.

| Symbol | Value | Direction | Meaning | Function/address and instruction evidence | Confidence |
|---|---:|---|---|---|---|
| `MB_MC1507_FUNC_REQFW_USB_PC2ND` | `0x01` | PC -> device | request firmware | `mc1507_reqFw` `0x1000d510`, store at `0x1000d563`; matching timeout string | HIGH |
| `MB_FUNC_ACCELINFO_PC2ND` | `0x03` | PC -> device | request accel info | `mc1507_getAccelInfo` `0x1000d680`, store at `0x1000d6d4`; matching string | HIGH |
| `MB_FUNC_GYROINFO_PC2ND` | `0x05` | PC -> device | request gyro info | `mc1507_getGyroInfo` `0x1000dc10`, store at `0x1000dc64`; matching string | HIGH |
| `MB_FUNC_MAGINFO_PC2ND` | `0x07` | PC -> device | request mag info | `mc1507_getMagInfo` `0x1000e1a0`, store at `0x1000e1f4`; matching string | HIGH |
| `MB_FUNC_SETZERO_PC2ND` | `0x0b` | PC -> device | set/reset zero | `mc1507_setZero/resetZero` `0x1000f040/0x1000f120`, stores at `0x1000f093/0x1000f173` | HIGH |
| `MB_FUNC_RAWDATA_PC2ND` | `0x0d` | PC -> device | set/reset raw-data mode | `mc1507_setRawData/resetRawData` `0x1000f200/0x1000f2e0`, stores at `0x1000f253/0x1000f333` | HIGH |
| `MB_FUNC_MC1487CONFIG_USB_PC2ND` | `0x11` | PC -> device | MC1487 configuration | `mc1507_setMC1487Config` `0x1000f5b0`, store at `0x1000f603` | HIGH |
| `MB_FUNC_RTTRANS_USB_PC2ND` | `0x21` | PC -> device | realtime-transfer configuration | `mc1507_setUARTRtTransConfig` `0x1000f4a0`, store at `0x1000f4f3`; also `fhusb::reqRtTrans` `0x1000d0f2` | HIGH |

Related higher-level `fhusb` inner request values include `rawData = 0x37`
(`0x1000d032`), `reqAccelConfig = 0x3d`, `reqGyroConfig = 0x3f`, and
`reqMagConfig = 0x41`. These belong to a different encapsulation layer and are
not substituted for the MC1507 values above.

### Default `0x70` poll and acknowledgement

In every high-speed bulk worker iteration, `USBBulkReadWorkerHs` zeros a
`0x200`-byte output buffer and writes `0x70` to byte 0
(`0x1002059d..0x100205c4`). A pending command replaces the first `0x86` bytes
(`0x100205cf..0x100205f4`). The worker sends all `0x200` bytes through bulk OUT
before reading bulk IN (`0x10020617..0x100206c0`). Therefore `0x70` is the outer
code of the repeating default bulk poll/request packet, not the RTTRANS function
and not independently proven to start streaming (HIGH).

A response beginning `74 01` clears the pending-command flag at
`0x10021065..0x100210a8`. This is a transport-level acknowledgement condition;
it does not prove a semantic RTTRANS response.

### Decoded in-memory objects

`SensorFrame` constructor `0x10009170` initializes through decoded offset
`0x380`. `MotionCaptureSuit::distributeSF` at `0x1000a780` copies `0xe0` dwords
plus one byte (`0x1000a7b3..0x1000a7c3`), proving an ABI size of `0x381` = 897
bytes (HIGH). It is one decoded per-sensor state object, not a wire frame and
not `N * 0x22` records.

Relevant decoded offsets written by parser `0x10006d30` are quaternion `+0x04`,
Euler `+0x38`, accel `+0x50`, gyro `+0x70`, mag `+0x7c`, and packed identity
`+0x34d`. The constructor also initializes two 256-byte regions at `+0x13a` and
`+0x23a` and a 20-byte tail at `+0x36d`; their meanings are not recovered.

`MC1507USBSensorFrame` is separate: its distribution path at `0x1000c240`
copies `0x70` dwords, one word, and one byte (`0x1000c267..0x1000c276`), an ABI
size of `0x1c3` = 451 bytes (HIGH). Neither ABI size is an on-wire size.

## PARTIALLY RECOVERED

### RTTRANS command construction

`fhusb::reqRtTrans` at `0x1000d0d0` constructs a 70-byte inner message:

| Inner offset | Value/meaning | Evidence | Confidence |
|---:|---|---|---|
| `0x00` | `0x40` inner header | constructor `0x10014a00` | HIGH |
| `0x01` | function `0x21` (RTTRANS) | store at `0x1000d0f2` | HIGH |
| `0x02..0x05` | target/address dword | stores `0x1000d0f6..0x1000d0f9` | HIGH data flow; target semantics UNKNOWN |
| `0x06..0x45` | 64-byte payload | constructor zeroing at `0x10014a00`; 11-byte request copied at `0x1000d0fc..0x1000d106` | HIGH boundary; most fields UNKNOWN |

For bulk mode, sender `0x10012c70` wraps it in an outer message constructed at
`0x10014a30`: outer byte 0 is `0x73`, bytes 1..4 are zero, byte 5 is inner
length `0x46`, and the 70-byte inner message begins at byte 6. The worker queue
copies `0x86` bytes, then sends a zero-based `0x200`-byte USB transfer. Bytes
`0x4c..0x85` of the queued outer object are not initialized by this constructor,
so no deterministic full 512-byte command can be reported.

MotionVenus methods correlated through their Qt method-name table set the
11-byte RTTRANS request as follows:

- `onWriteAllRtTrans50Hz`, `100Hz`, `onWriteRtTrans10Hz`, and `20Hz` zero all
  11 bytes, set request byte `+5 = 1`, and set little-endian uint16 `+6` to
  `50`, `100`, `10`, or `20`; they call `reqRtTrans` five times with 20 ms
  delay (`0x00f14980`, `0x00f14a70`, `0x00f14fa0`, `0x00f15190`).
- `onReqRtTrans` zeroes 11 bytes, sets byte `+5 = 2`, repeats five times with
  300 ms delay, and queries the result (`0x00f15380`).
- `onWriteAllRtTrans1Hz` sets `+5 = 1` and `+6 = 1` but does not initialize the
  rest of its stack request in the observed function (`0x00f148a0`), so its
  complete bytes are not statically deterministic.

The method names and literal rates establish byte `+5` as a set/query selector
and `+6..+7` as rate in these paths (HIGH). Other stream-selection flags,
sensor mask semantics, target address, required ordering, and whether this
configuration starts streaming remain UNKNOWN. No RTTRANS, `0x73`, or other
command was sent by the Linux implementation.

### Other receive messages

The bulk-HS worker dispatches byte-0 codes `0x01`, `0x11`, `0x13`, `0x14`,
`0x15`, and `0x64`; codes `0x3e`, `0x40`, `0x42`, `0x22`, `0x3c`, and `0x24`
are forwarded as 64 raw bytes (`0x100206e8..0x10020efd`). The HID `0x15` mode-3
layout is now partially real-validated above. Other layouts remain unproven.

The HID-HS worker beginning `0x100220b0` reads 64-byte reports. Its apparent
`0x88e` fixed-frame block at `0x100225ef..0x10022868` is unreachable: an earlier
comparison handles byte 0 equal to `0x11` and jumps away, while the fixed block
requires the same byte to equal `0x11` again. It is not evidence of HID
reassembly.

### Checksum search

No checksum construction or validation was found on the analyzed live fixed
receive path (`0x10020916..0x10020b6c`) or bulk command path
(`0x10014a00`, `0x10014a30`, `0x10012c70`). Searches found no live-path use of
CRC constants `0x1021`, `0xa001`, or `0xedb88320`; the routine at `0x10023f00`
is a hex logger, not a checksum. This does not prove that no other protocol
variant has integrity checking. Checksum type and location remain UNKNOWN.

## UNKNOWN

- Meaning of raw record bytes `0x00..0x01`; no ID, online, battery,
  calibration, magnetic-disturbance, or status field is proven inside `0x22`.
- Meaning of fixed-message header bytes `0x06..0x0d`.
- Exact meaning of HID `0x15` bytes `1..6`, high flag bits, optional trailing
  fields, logical payload length, and padding boundary.
- Frame counter, sensor-count field, terminator, checksum, and universal outer framing.
- Why the fixed worker consumes only record slots 1..4 and what other slots mean.
- Complete `0x1b` compact record layout and other outer message layouts.
- Whether firmware normalizes quaternions and the physical coordinate/handedness conventions.
- Exact RTTRANS target/address, sensor mask, data-selection flags, initialization
  ordering, and semantic response.
- Whether RTTRANS configuration is mandatory to begin live streaming.
- Whether `0x70` starts any internal producer or only clocks one already-queued
  report. It is proven to elicit one report per successful poll in this setup.

## REQUIRES FURTHER HARDWARE VALIDATION

- Whether a HID transfer can ever be short, fragmented, or aggregate a logical
  message other than the observed direct 64-byte `0x15` path.
- Presence, frequency, header bytes, and record-slot population of message `0x13`.
- Controlled known-axis motion to validate correlation, axes, handedness, and
  Euler output; the completed capture was stationary-only.
- Identity-prefix and sensor-index mapping across multiple physical sensors/suits.
- RTTRANS set/query behavior and acknowledgement, only after captures of the
  official software or a separately approved experimental-write procedure.
- Checksum/integrity behavior under intentionally truncated replay data; do not
  induce malformed writes on hardware.
