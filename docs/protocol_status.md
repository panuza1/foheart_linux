# Protocol status

Static source binaries:

- `fhusb.dll` SHA-256 `ce9049b82f5e8d06f7faedb42605379a8e47dab775a2f9e9888b79f81a15defa`
- `MotionVenus_3.2.0_setup.exe` SHA-256 `473692ab1ea10dbcda8cb1c7ef996b7b0cbf609099ba57c5bab89f02553b7e0e`

Addresses are virtual addresses in the named 32-bit PE image. Static findings describe
this software build; none have yet been validated against a connected C1.

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
`16384.0`. Whether sensor firmware normalizes the raw quaternion is UNKNOWN.

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
configuration starts streaming remain UNKNOWN. No Linux USB write was added.

### Other receive messages

The bulk-HS worker dispatches byte-0 codes `0x01`, `0x11`, `0x13`, `0x14`,
`0x15`, and `0x64`; codes `0x3e`, `0x40`, `0x42`, `0x22`, `0x3c`, and `0x24`
are forwarded as 64 raw bytes (`0x100206e8..0x10020efd`). Except for `0x13`,
their outer layouts and semantic symbol mappings are not proven.

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
- Frame counter, payload-length field, sensor-count field, terminator, checksum,
  and universal outer framing.
- Why the fixed worker consumes only record slots 1..4 and what other slots mean.
- Complete `0x1b` compact record layout and other outer message layouts.
- Whether firmware normalizes quaternions and the physical coordinate/handedness conventions.
- Exact RTTRANS target/address, sensor mask, data-selection flags, initialization
  ordering, and semantic response.
- Whether RTTRANS configuration is mandatory to begin live streaming.
- Whether the repeating `0x70` poll alone triggers or merely clocks reads.

## REQUIRES HARDWARE VALIDATION

- Actual VID/PID, product strings, active configuration, interface, alternate
  setting, endpoint descriptors, and kernel-driver state.
- Whether the C1 emits anything before the first write and the effect of a
  read-only timeout.
- Actual `0x70` poll response, only after explicit user-authorized write testing.
- USB transfer lengths and whether a transfer ever contains a partial or
  multiple logical message.
- Presence, frequency, header bytes, and record-slot population of message `0x13`.
- Raw/scaled values against known stationary poses to validate sensor axes,
  quaternion order, scale, and firmware normalization.
- Identity-prefix and sensor-index mapping across multiple physical sensors/suits.
- RTTRANS set/query behavior and acknowledgement, only after captures of the
  official software or a separately approved experimental-write procedure.
- Checksum/integrity behavior under intentionally truncated replay data; do not
  induce malformed writes on hardware.
