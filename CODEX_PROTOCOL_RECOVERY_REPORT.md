# FOHEART C1 protocol recovery report

This report covers focused static analysis of the FOHEART C1 live USB path.
Hardware was not connected, no USB writes were performed, and no fabricated
capture data was used.

Primary evidence binary:

```text
/home/panu/Downloads/foheart_extract/app/fhusb.dll
SHA-256 ce9049b82f5e8d06f7faedb42605379a8e47dab775a2f9e9888b79f81a15defa
```

Quaternion/order and caller evidence binary:

```text
/home/panu/Downloads/foheart_extract/app/MotionVenus_3.2.0_setup.exe
SHA-256 473692ab1ea10dbcda8cb1c7ef996b7b0cbf609099ba57c5bab89f02553b7e0e
```

## 1. Functions analyzed

| Binary | Function/address | Result |
|---|---:|---|
| `fhusb.dll` | `0x100204e0` | high-speed bulk worker and complete byte-0 dispatch traced |
| `fhusb.dll` | `0x100220b0` | high-speed HID worker traced; apparent fixed-frame branch shown unreachable |
| `fhusb.dll` | `0x10023780`, `0x100238a0` | bulk IN/OUT libusb arguments confirmed |
| `fhusb.dll` | `0x10023920`, `0x10023980` | interrupt IN/OUT libusb arguments confirmed |
| `fhusb.dll` | `0x10006d30` | complete fixed `0x22` raw sensor-record decoder recovered |
| `fhusb.dll` | `0x10007020` | alternate `0x1b` record decoder identified, not fully recovered |
| `fhusb.dll` | `0x10009170` | decoded `SensorFrame` constructor extent traced |
| `fhusb.dll` | `0x1000a780` | `MotionCaptureSuit::distributeSF`, ABI copy size traced |
| `fhusb.dll` | `0x1000b010`, `0x1000b5d0` | quaternion and Euler getters inspected |
| `fhusb.dll` | `0x10010737..0x1001074d` | suit-number and sensor-index routing traced |
| `fhusb.dll` | `0x10017bb0`, `0x10017c10` | packed identity bit extraction traced |
| `fhusb.dll` | `0x1000d0d0` | `fhusb::reqRtTrans` inner message traced |
| `fhusb.dll` | `0x10012c70` | transport-specific command wrapping traced |
| `fhusb.dll` | `0x10014a00`, `0x10014a30` | inner and bulk outer constructors traced |
| `fhusb.dll` | `0x100210e0` | bulk worker pending-command queue traced |
| `fhusb.dll` | `0x1000d510..0x1000f603` | named MC1507 command functions correlated with numeric IDs |
| MotionVenus | `0x00e0c89f..0x00e0c916` | returned quaternion mapped into `osg::Quat` |
| MotionVenus | `0x00e0f201..0x00e0f299` | independent repeat of quaternion mapping |
| MotionVenus | `0x00f148a0..0x00f1556c` | named RTTRANS set/query rate methods traced |

Analysis used locally installed command-line tools (`sha256sum`, `strings`, and
GNU `objdump`). No large reverse-engineering tool was installed.

## 2. Important call graph

```text
Receive:
USBBulkReadWorkerHs @ 0x100204e0
  -> bulk OUT @ 0x100238a0 (default poll or queued command)
  -> bulk IN @ 0x10023780 (0x1400 capacity)
  -> byte-0 dispatch @ 0x100206e8..0x10021004
     -> code 0x13 @ 0x10020916
        -> copy 0x88e
        -> format byte +0x05
        -> hard-coded slots 1..4
           -> 0x22 decoder @ 0x10006d30, or 0x1b decoder @ 0x10007020
        -> decoded SensorFrame callback/distribution
           -> MotionCaptureSuit::distributeSF @ 0x1000a780
           -> getLastQuatByIndexInSuit @ 0x1000b010

RTTRANS request:
MotionVenus rate method @ 0x00f148a0..0x00f15380
  -> fhusb::reqRtTrans @ 0x1000d0d0
     -> inner command constructor @ 0x10014a00
     -> sender @ 0x10012c70
        -> bulk envelope constructor @ 0x10014a30
        -> pending queue @ 0x100210e0
           -> worker bulk OUT @ 0x100238a0
```

## 3. `0x22` record layout

`fhusb.dll` decoder `0x10006d30` uses `movsx` word loads. Caller
`0x1002099d` supplies the `0x22` stride.

| Offset | Size | Raw type | Meaning | Scaling | Evidence | Confidence |
|---:|---:|---|---|---|---|---|
| `0x00` | 2 | opaque | UNKNOWN; not read | UNKNOWN | no load from `+0x00` in `0x10006d30..0x1000701b` | HIGH unused here; LOW meaning |
| `0x02` | 2 | int16 | accel X | / `2048.0` | load `0x10006e0c`, constant `0x10029730` | HIGH |
| `0x04` | 2 | int16 | accel Y | / `2048.0` | load `0x10006e39`, constant `0x10029730` | HIGH |
| `0x06` | 2 | int16 | accel Z | / `2048.0` | load `0x10006e65`, constant `0x10029730` | HIGH |
| `0x08` | 2 | int16 | gyro X | / float32 `16.4` | load `0x10006e91`, constant `0x10029720` | HIGH |
| `0x0a` | 2 | int16 | gyro Y | / float32 `16.4` | load `0x10006ebe`, constant `0x10029720` | HIGH |
| `0x0c` | 2 | int16 | gyro Z | / float32 `16.4` | load `0x10006eea`, constant `0x10029720` | HIGH |
| `0x0e` | 2 | int16 | magnetometer X | / `12000.0` | load `0x10006f16`, constant `0x10029740` | HIGH |
| `0x10` | 2 | int16 | magnetometer Y | / `12000.0` | load `0x10006f43`, constant `0x10029740` | HIGH |
| `0x12` | 2 | int16 | magnetometer Z | / `12000.0` | load `0x10006f6f`, constant `0x10029740` | HIGH |
| `0x14` | 2 | int16 | quaternion W | / `16384.0` | load `0x10006d5a`, constant `0x10029744`, order trace in section 6 | HIGH |
| `0x16` | 2 | int16 | quaternion X | / `16384.0` | load `0x10006d87`, same constant/order trace | HIGH |
| `0x18` | 2 | int16 | quaternion Y | / `16384.0` | load `0x10006db3`, same constant/order trace | HIGH |
| `0x1a` | 2 | int16 | quaternion Z | / `16384.0` | load `0x10006ddf`, same constant/order trace | HIGH |
| `0x1c` | 2 | int16 | Euler X | / `128.0` | load `0x10006f9b`, constant `0x10029724` | HIGH |
| `0x1e` | 2 | int16 | Euler Y | / `128.0` | load `0x10006fc8`, constant `0x10029724` | HIGH |
| `0x20` | 2 | int16 | Euler Z | / `128.0` | load `0x10006ff4`, constant `0x10029724` | HIGH |

The record ends at `0x22`. There is no remaining byte range in this variant for
additional battery/status values. That does not rule out such values in an outer
header, another record format, or library-maintained state.

## 4. Quaternion raw type

Raw quaternion type is exactly four consecutive signed 16-bit integers at raw
offsets `0x14`, `0x16`, `0x18`, and `0x1a` (HIGH). Evidence is the four
`movsx WORD` loads at `fhusb.dll` `0x10006d4f..0x10006de4` followed by float
conversion.

## 5. Quaternion scaling

Each raw signed integer is converted to float and divided by the float32 value
`16384.0` loaded from `fhusb.dll` address `0x10029744`. Representative sequence:

```asm
10006d5a  movsx    edx, WORD PTR [record+0x14]
10006d5f  cvtsi2ss xmm0, edx
10006d63  divss    xmm0, DWORD PTR ds:0x10029744
```

Exact conversion: `decoded = raw / 16384.0` (HIGH).

No normalization is performed by `0x10006d30`, getter `0x1000b010`, or the
observed MotionVenus handoff. Whether the firmware has already normalized the
raw quaternion is UNKNOWN.

## 6. Quaternion component order

Order is `w,x,y,z` (HIGH).

Evidence chain:

1. Raw parser `0x10006d4f..0x10006dfb` preserves raw word order into decoded
   components 0..3.
2. Getter `0x1000b010` returns those four components in sequence.
3. MotionVenus path `0x00e0c89f..0x00e0c916` passes returned indices 1, 2, 3,
   0 as logical `x,y,z,w` to imported `osg::Quat(double x,double y,double z,
   double w)`. The machine push order is 0,3,2,1 because x86 arguments are
   pushed right-to-left.
4. The same mapping appears independently at `0x00e0f201..0x00e0f299`.

This conclusion does not rely on assuming a common convention.

## 7. Sensor ID/index mapping

On the recovered format-0 path, the worker loop is fixed at 1..4
(`0x1002093f..0x10020961`). It computes the record pointer from that loop index
and passes the index separately into decoder `0x10006d30`. Decoder
`0x10006d3f..0x10006d49` ORs it with the outer dword at message `+0x01` and
stores the result at decoded `SensorFrame +0x34d`.

Accessor `0x10017bb0` extracts low bits 0..5 as the sensor index. Routing at
`0x10010737..0x1001074d` uses bits 8..31 as the suit number and the same low six
bits as the destination index. Bits 6..7 have an accessor at `0x10017c10`, but
their meaning is UNKNOWN.

Therefore:

- sensor ID used by this path is external/synthesized from loop index, not read
  from a proven location in the 0x22 bytes;
- decoded indices are 1,2,3,4 for this worker branch;
- copied storage can hold 64 fixed slots, but the actual sensor-record count is
  not 64 and no count field is consulted;
- raw `+0x00` remains UNKNOWN rather than being labelled as an ID.

## 8. Outer frame layout

Recovered only for the bulk-HS byte-0 `0x13` branch:

```text
+0x00       uint8       0x13 message code
+0x01       uint32 LE   identity prefix passed to every record
+0x05       uint8       record format: 0 => stride 0x22, 1 => stride 0x1b
+0x06..0x0d 8 bytes     UNKNOWN
+0x0e       record-slot storage
total       0x88e       bytes copied by this branch
```

Evidence: message comparison `0x10020916`, `memcpy` size at
`0x1002091f..0x10020937`, format tests `0x10020967..0x10020983`, and record
pointer/stride calculations `0x1002098e..0x10020a9b`.

The size identity `0x88e = 0x0e + 64 * 0x22` is validated against the actual
base, stride, and copy size. It establishes capacity, not active count. The
worker only decodes slots 1..4.

No payload-size field, frame counter, sensor-count field, terminator, or checksum
has been identified. The bulk read capacity is `0x1400`, but the branch copies
`0x88e` without checking the actual transfer length. There is no fragment
accumulator or multiple-message loop in the analyzed worker. It treats one
successful transfer as one message; real USB boundaries remain unvalidated.

The Linux parser intentionally requires exactly `0x88e` bytes. It does not copy
the DLL's unchecked behavior.

## 9. Function/message IDs

| Symbol | Numeric value | Direction | Meaning | Evidence | Confidence |
|---|---:|---|---|---|---|
| `MB_MC1507_FUNC_REQFW_USB_PC2ND` | `0x01` | PC -> device | firmware request | `mc1507_reqFw` `0x1000d510`, store `0x1000d563`, matching diagnostic string | HIGH |
| `MB_FUNC_ACCELINFO_PC2ND` | `0x03` | PC -> device | accel-info request | `mc1507_getAccelInfo` `0x1000d680`, store `0x1000d6d4` | HIGH |
| `MB_FUNC_GYROINFO_PC2ND` | `0x05` | PC -> device | gyro-info request | `mc1507_getGyroInfo` `0x1000dc10`, store `0x1000dc64` | HIGH |
| `MB_FUNC_MAGINFO_PC2ND` | `0x07` | PC -> device | mag-info request | `mc1507_getMagInfo` `0x1000e1a0`, store `0x1000e1f4` | HIGH |
| `MB_FUNC_SETZERO_PC2ND` | `0x0b` | PC -> device | set/reset zero | `0x1000f040/0x1000f120`, stores `0x1000f093/0x1000f173` | HIGH |
| `MB_FUNC_RAWDATA_PC2ND` | `0x0d` | PC -> device | set/reset raw data | `0x1000f200/0x1000f2e0`, stores `0x1000f253/0x1000f333` | HIGH |
| `MB_FUNC_MC1487CONFIG_USB_PC2ND` | `0x11` | PC -> device | MC1487 config | `0x1000f5b0`, store `0x1000f603` | HIGH |
| `MB_FUNC_RTTRANS_USB_PC2ND` | `0x21` | PC -> device | realtime-transfer config | `0x1000f4a0`, store `0x1000f4f3`; `fhusb::reqRtTrans` store `0x1000d0f2` | HIGH |

Known outer/envelope codes are separate:

| Value | Direction | Statically observed role | Confidence |
|---:|---|---|---|
| `0x13` | device -> PC | fixed decoded sensor-block branch | HIGH role; formal symbol name UNKNOWN |
| `0x70` | PC -> device | repeating default bulk poll/request packet | HIGH |
| `0x73` | PC -> device | queued bulk command envelope | HIGH |
| `0x74 0x01` | device -> PC | clears queued-command pending flag | HIGH transport behavior; semantic response UNKNOWN |

## 10. Stream-start command

A complete stream-start command was not recovered.

What is proven about `fhusb::reqRtTrans`:

```text
inner +0x00: 0x40
inner +0x01: 0x21
inner +0x02: target/address uint32 LE (semantics UNKNOWN)
inner +0x06: 64-byte payload; caller copies an 11-byte request here

bulk outer +0x00: 0x73
bulk outer +0x01..+0x04: zero
bulk outer +0x05: 0x46 (70-byte inner length)
bulk outer +0x06: inner message
```

The worker copies a `0x86`-byte queued object into a zeroed `0x200`-byte USB OUT
buffer. Only the first `0x4c` bytes are meaningful for this 70-byte inner
message; the constructor does not deterministically initialize queued bytes
`0x4c..0x85`. Consequently there is no safe complete literal packet to send.

MotionVenus rate setters for 10, 20, 50, and 100 Hz create this deterministic
11-byte request payload:

```text
00 00 00 00 00 01 RR RR 00 00 00
```

`RR RR` is the little-endian uint16 rate. The methods send it five times with a
20 ms delay. Query changes byte 5 to `02`, sends five times with 300 ms delays,
and retrieves a result. The 1 Hz method fails to initialize all 11 local bytes
in its observed body, so its complete payload is not declared.

Unknowns that prevent implementation are target semantics, sensor mask,
quaternion/Euler/raw selection flags, initialization ordering, whether the rate
request starts streaming at all, and the semantic reply. `start_stream()`
therefore remains unsupported and no production write path was added.

## 11. Meaning of `0x70`

`0x70` is the outer code for the high-speed bulk worker's default 512-byte
poll/request packet (HIGH). At `0x1002059d..0x100205c4` the worker zeroes
`0x200` bytes and stores `0x70` in the first byte. It sends that buffer before
each read. If a command is pending, `0x100205cf..0x100205f4` overwrites the first
`0x86` bytes with the queued `0x73` command.

It is not the RTTRANS ID (`0x21`) and is not independently proven to start the
stream. Whether the device requires this poll to produce each response requires
hardware validation.

## 12. Checksum/CRC

No checksum construction or validation was discovered in the analyzed live
fixed-frame branch (`0x10020916..0x10020b6c`) or RTTRANS command constructors and
sender (`0x10014a00`, `0x10014a30`, `0x10012c70`). Searches for CRC strings and
common constants `0x1021`, `0xa001`, and `0xedb88320` did not identify a live-path
routine. Function `0x10023f00` is a hex logger, not a checksum.

Checksum type and location remain UNKNOWN. The precise conclusion is “no
checksum discovered on these analyzed paths,” not “the protocol has no
checksum.”

## 13. Meaning of decoded `SensorFrame` size `0x381`

`SensorFrame` constructor `0x10009170` initializes through offset `0x380`.
`MotionCaptureSuit::distributeSF` at `0x1000a780` copies `0xe0` dwords plus one
byte at `0x1000a7b3..0x1000a7c3`, proving 897 (`0x381`) ABI bytes.

It is one decoded per-sensor state object, not an on-wire USB frame and not an
array of raw `0x22` records. Recovered decoded offsets include:

```text
+0x04 quaternion float32[4]
+0x38 Euler float32[3]
+0x50 accel float32[3]
+0x70 gyro float32[3]
+0x7c magnetometer float32[3]
+0x34d packed suit/index/status value
```

The constructor also covers 256-byte regions at `+0x13a` and `+0x23a` and a
20-byte tail at `+0x36d`; meanings are UNKNOWN. A separate
`MC1507USBSensorFrame` copy path at `0x1000c240` establishes an ABI size of
`0x1c3` (451), also not a wire size.

## 14. Parser changes made

- Added a narrow `decode_fixed_sensor_record()` using stdlib `struct.Struct("<17h")`.
- Implemented only proven accel, gyro, magnetometer, quaternion, and Euler
  offsets/scales; raw bytes `0x00..0x01` are deliberately ignored.
- Marked quaternion component order as `wxyz` from the cross-binary data-flow proof.
- Added fail-closed support for exact-length `0x88e`, code-`0x13`, format-0
  logical blocks and the DLL's hard-coded indices 1..4.
- Added an explicit raw-capture `--read-size` override so bulk validation can
  request the proven `0x1400` DLL buffer capacity without making it a default.
- Preserved recording timestamps when replay passes a transfer into the parser.
- Left every other message/format as `ProtocolNotDecodedError` and every
  malformed fixed length as `MalformedPayloadError`.
- Did not add `start_stream()`, command bytes, checksum guesses, or any USB write.

## 15. Tests added

Synthetic-only tests cover:

- signed int16 extraction;
- all five statically proven scale groups;
- statically proven `wxyz` quaternion order;
- exact `0x22` record length and rejection of both short and long input;
- exact synthetic `0x88e` block parsing with hard-coded slots 1..4;
- timestamp propagation;
- truncated fixed-message rejection.

No test vector is described as captured C1 data.

## 16. Actual pytest result

Command run:

```bash
env -u PYTHONPATH .venv/bin/pytest -q
```

Actual result:

```text
.................                                                        [100%]
17 passed in 0.07s
```

## 17. Unknowns remaining

- raw record `+0x00..+0x01` meaning;
- live meaning of fixed header `+0x06..+0x0d`;
- universal outer frame, payload length, frame counter, sensor-count field,
  checksum, and record population rules;
- format-1 `0x1b` record and other receive-message layouts;
- bits 6..7 of packed identity and physical sensor mapping;
- firmware quaternion normalization, axes, handedness, and Euler units/convention;
- RTTRANS target/address, sensor mask, content flags, ordering, mandatory status,
  and semantic response;
- whether `0x70` actively clocks a response or merely polls already-running data;
- real endpoint/configuration/transfer behavior because hardware was absent.

## 18. Exact hardware validation procedure

Begin read-only and retain every raw transfer boundary:

1. Plug in the C1 and record `lsusb` output. Do not install a udev rule until the
   reported VID/PID has been checked.
2. From the project virtualenv, run:

   ```bash
   python -m foheart.tools.discover
   ```

   Save the full configuration/interface/alternate-setting/endpoint output.
3. For a `1483:5751` device, attempt a read-only capture without sending an
   initialization command:

   ```bash
   python -m foheart.tools.usb_dump \
     --pid 0x5751 \
     --count 100 \
     --timeout-ms 250 \
     --read-size 0x1400 \
     --hex \
     --output samples/c1_read_only.bin
   ```

4. If transfers arrive, preserve the unmodified recording and replay it:

   ```bash
   python -m foheart.tools.replay samples/c1_read_only.bin
   ```

   Compare each USB transfer length and byte 0. For any `0x13` transfer, verify
   exact length, bytes `+0x01..+0x0d`, format byte, and which of slots 0..63
   change. Do not discard zero/unchanged slots.
5. With sensors stationary, record at least 10 seconds. Then move exactly one
   labelled sensor around one axis and record again. Use these paired captures
   to validate slot identity, signed values, scale, quaternion order, norm, axes,
   Euler convention, counters, and changing header bits.
6. If read-only capture times out, stop. Do not send `0x70`, `0x73`, RTTRANS, or
   guessed control transfers. The next write experiment must be separately
   reviewed and explicitly enabled, starting with the proven zeroed 512-byte
   `0x70` poll and usbmon capture; RTTRANS must wait until target and payload
   semantics are resolved.

This procedure can validate the parser without claiming that the current code
can start a silent C1.

=== INFORMATION FOR CHATGPT ===

0x22 sensor record:
  offset 0x00: UNKNOWN
  offset 0x02: int16 accel X / 2048.0
  offset 0x04: int16 accel Y / 2048.0
  offset 0x06: int16 accel Z / 2048.0
  offset 0x08: int16 gyro X / float32 16.4
  offset 0x0A: int16 gyro Y / float32 16.4
  offset 0x0C: int16 gyro Z / float32 16.4
  offset 0x0E: int16 magnetometer X / 12000.0
  offset 0x10: int16 magnetometer Y / 12000.0
  offset 0x12: int16 magnetometer Z / 12000.0
  offset 0x14: int16 quaternion W / 16384.0
  offset 0x16: int16 quaternion X / 16384.0
  offset 0x18: int16 quaternion Y / 16384.0
  offset 0x1A: int16 quaternion Z / 16384.0
  offset 0x1C: int16 Euler X / 128.0
  offset 0x1E: int16 Euler Y / 128.0
  offset 0x20: int16 Euler Z / 128.0

Quaternion raw type: signed int16[4]
Quaternion scale: raw / 16384.0
Quaternion order: w,x,y,z
Quaternion normalized by firmware/library: firmware UNKNOWN; observed library path NO

Sensor ID location: external loop index; decoded +0x34D bits 0..5; raw record location UNKNOWN
Sensor record count: worker decodes 4 (indices 1..4); actual populated count UNKNOWN
Sensor record stride: 0x22 bytes

Outer frame header: fixed 0x13 variant is 0x0E bytes; +0 code, +1 identity dword, +5 format, +6..+0D UNKNOWN
Outer frame size: 0x88E for fixed 0x13 block; universal size UNKNOWN
Payload size field: UNKNOWN
Frame counter: UNKNOWN
Sensor count field: UNKNOWN; none consulted by recovered worker branch
Fragmentation/reassembly: none in analyzed bulk worker; hardware behavior UNKNOWN

Realtime message ID: 0x13 fixed sensor-block branch; formal RTTRANS mapping UNKNOWN
RTTRANS numeric value: 0x21
Start-stream command bytes: UNKNOWN; partial RTTRANS rate envelope recovered
Start-stream command length: UNKNOWN
Expected response: UNKNOWN; transport acknowledgement 74 01 observed

0x70 meaning: outer code of repeating zero-filled 0x200-byte bulk poll/request; not proven stream start

Checksum type: UNKNOWN; none discovered on analyzed fixed receive/RTTRANS paths
Checksum location: UNKNOWN

0x381 SensorFrame meaning: 897-byte decoded per-sensor C++ ABI object; not USB wire size

Parser implementation status: PARTIAL; exact 0x88E bulk-HS 0x13 format-0 blocks and 0x22 records only
Can decode synthetic sensor record: YES
Can decode captured C1 frame: NO

Tests: 17 passed in 0.07s with env -u PYTHONPATH .venv/bin/pytest -q

Current blocker: no real C1 capture; startup target/content/order remain UNKNOWN
Next exact function/address to inspect: MotionVenus 0x00e86320 RTTRANS caller and following 17-byte mask construction
Hardware test needed: descriptor dump, read-only boundary-preserving capture, then stationary/single-sensor-motion validation
