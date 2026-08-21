# Reverse-engineering notes

Primary binary: `fhusb.dll`, SHA-256
`ce9049b82f5e8d06f7faedb42605379a8e47dab775a2f9e9888b79f81a15defa`.
It is 32-bit PE/i386 with image base `0x10000000`; `.text` starts at VA
`0x10001000`, file offset `0x400`.

Quaternion-use correlation binary: `MotionVenus_3.2.0_setup.exe`, SHA-256
`473692ab1ea10dbcda8cb1c7ef996b7b0cbf609099ba57c5bab89f02553b7e0e`.

Analysis used `sha256sum`, `strings`, and GNU `objdump -p/-s/-d -M intel`.
Neither binary was executed or modified. The original analysis was static; the
2026-08-21 validation later sent only the exact authorized 64-byte `0x70` poll
to the connected C1.

## Focused function inventory

| Binary | Function/address | Reason inspected |
|---|---:|---|
| `fhusb.dll` | `0x100204e0` | `USBBulkReadWorkerHs`, confirmed by its start-log string xref |
| `fhusb.dll` | `0x100220b0` | `USBHidReadWorkerHs`, confirmed by its start-log string xref |
| `fhusb.dll` | `0x10023780`, `0x100238a0`, `0x10023920`, `0x10023980` | libusb read/write wrappers |
| `fhusb.dll` | `0x10006d30` | fixed `0x22` raw-record decoder |
| `fhusb.dll` | `0x100070b0` | flag-driven sensor decoder; mode 3 used by real HID `0x15` reports |
| `fhusb.dll` | `0x10007020` | alternate `0x1b` decoder dispatch target (not fully decoded) |
| `fhusb.dll` | `0x10009170` | decoded `SensorFrame` constructor |
| `fhusb.dll` | `0x1000a780`, `0x1000b010` | `MotionCaptureSuit::distributeSF`, quaternion getter |
| `fhusb.dll` | `0x10010737`, `0x10017bb0`, `0x10017c10` | suit/index routing and packed-ID accessors |
| `fhusb.dll` | `0x1000d0d0`, `0x10012c70` | `fhusb::reqRtTrans` and transport sender |
| `fhusb.dll` | `0x10014a00`, `0x10014a30`, `0x100210e0` | inner/outer command constructors and bulk-worker queue |
| `fhusb.dll` | `0x1000d510..0x1000f603` | named MC1507 request functions and numeric function bytes |
| MotionVenus | `0x00e0c89f..0x00e0c916`, `0x00e0f201..0x00e0f299` | fhusb-vector to `osg::Quat` mapping |
| MotionVenus | `0x00f148a0..0x00f1556c` | RTTRANS set/query methods correlated with Qt method names |

## Important call graph

Receive path recovered for the fixed bulk-HS variant:

```text
USBBulkReadWorkerHs @ 0x100204e0
  -> bulk OUT wrapper @ 0x100238a0 (0x200-byte poll/command)
  -> bulk IN wrapper @ 0x10023780 (capacity 0x1400)
  -> dispatch on USB buffer[0] @ 0x100206e8..
       -> code 0x13 branch @ 0x10020916
          -> copy 0x88e bytes
          -> format byte at +0x05
          -> slots 1..4
             -> format 0: record decoder @ 0x10006d30, stride 0x22
             -> format 1: record decoder @ 0x10007020, stride 0x1b
          -> decoded-frame callback path
             -> MotionCaptureSuit::distributeSF @ 0x1000a780
             -> getLastQuatByIndexInSuit @ 0x1000b010
```

Real-validated HID receive path:

```text
USBHidReadWorkerHs @ 0x100220b0
  -> clear/send 64-byte 0x70 poll @ 0x1002229a..0x100222f5
  -> interrupt IN 0x81, 64 bytes @ 0x1002237e..0x100223ac
  -> byte-0 dispatch
       -> code 0x15 comparison @ 0x10022986..0x10022997
       -> mode-3 decoder @ 0x100070b0, call @ 0x10022a70..0x10022a8c
          -> 11-byte header
          -> flag bit 0: quaternion
          -> flag bit 2: accel
          -> flag bit 3: gyro
          -> flag bit 4: magnetometer
       -> decoded-frame callback path
```

RTTRANS construction path:

```text
MotionVenus rate method @ 0x00f148a0..0x00f15380
  -> fhusb::reqRtTrans @ 0x1000d0d0
     -> inner constructor @ 0x10014a00: 0x40, function 0x21, target, payload
     -> sender @ 0x10012c70
        -> bulk outer constructor @ 0x10014a30: 0x73, length 0x46
        -> worker queue @ 0x100210e0: pending + copy 0x86
           -> next USBBulkReadWorkerHs iteration
              -> bulk OUT endpoint 0x01, length 0x200
```

## Fixed record decoder

`0x10006d30` is straight-line code. Representative quaternion instructions are:

```asm
10006d5a  movsx    edx, WORD PTR [ecx+eax+0x14]
10006d5f  cvtsi2ss xmm0, edx
10006d63  divss    xmm0, DWORD PTR ds:0x10029744
10006d76  movss    DWORD PTR [edx+ecx+0x04], xmm0
```

The same sequence is repeated four times. `.rdata[0x10029744]` is float32
`16384.0`. The remaining literal divisors are `2048.0` at `0x10029730`,
float32 `16.4` at `0x10029720`, `12000.0` at `0x10029740`, and `128.0` at
`0x10029724`.

The complete recovered record is:

| Raw bytes | Decoded field | Conversion |
|---|---|---|
| `00..01` | UNKNOWN; not read by this parser | none observed |
| `02..07` | accel X/Y/Z, signed int16 | divide each by `2048.0` |
| `08..0d` | gyro X/Y/Z, signed int16 | divide each by float32 `16.4` |
| `0e..13` | magnetometer X/Y/Z, signed int16 | divide each by `12000.0` |
| `14..1b` | quaternion W/X/Y/Z, signed int16 | divide each by `16384.0` |
| `1c..21` | Euler X/Y/Z, signed int16 | divide each by `128.0` |

No online, battery, calibration, magnetic-disturbance, or sensor-ID field is
read from this record by the recovered function.

## Quaternion order proof

The fixed decoder writes raw quaternion words sequentially to decoded float
components 0..3. `getLastQuatByIndexInSuit` returns all four in that order and
does no reordering or normalization.

MotionVenus imports `getLastQuatByIndexInSuit` at IAT `0x00fc5b3c` and
`osg::Quat(double x,double y,double z,double w)` at IAT `0x00fc6708`. At
`0x00e0c89f..0x00e0c916`, vector indices are fetched/pushed in machine order
`0,3,2,1` before that constructor call. Because x86 pushes the logical
arguments right-to-left, the constructor receives:

```text
x = returned[1]
y = returned[2]
z = returned[3]
w = returned[0]
```

Therefore returned order, decoded order, and the raw words at `0x14..0x1a` are
`w,x,y,z`. The same data flow repeats at `0x00e0f201..0x00e0f299`. This is not
an inference from a conventional quaternion API; it is a constructor-argument
trace.

The observed library/application path only scales components. It does not
compute a norm or renormalize. Firmware-side normalization remains unknown.

## Sensor identity and record loop

The `0x13` worker branch copies the incoming block, initializes its loop to 1,
and exits after 4:

```asm
1002093f  mov      [loop], 1
1002095a  cmp      [loop], 4
10020961  jg       done
1002099d  imul     eax, [loop], 0x22
100209b2  call     0x10006d30
```

For each record it passes three arguments: pointer `block + 0x0e + index*stride`,
the dword at block `+0x01`, and the byte-sized loop index. Parser
`0x10006d3f..0x10006d49` ORs the last two values into decoded offset `0x34d`.
The index is therefore synthesized outside the record on this path.

Accessor `0x10017bb0` masks that packed value with `0x3f`; accessor `0x10017c10`
extracts bits 6..7. Suit routing at `0x10010737..0x1001074d` compares the value
shifted right by 8 with the suit number and uses the low six bits as the sensor
index. Bits 6..7 remain unidentified.

## Outer fixed block and transfer boundaries

At `0x10020916`, byte 0 is compared with `0x13`. The branch copies exactly
`0x88e` bytes (`0x1002091f..0x10020937`). Format is selected by byte 5.
Record storage begins at byte `0x0e`; format 0 uses stride `0x22`, format 1 uses
`0x1b`. The arithmetic identity `0x88e = 0x0e + 64*0x22` shows that the copied
format-0 block can hold 64 slots, while the actual loop consumes only 1..4.

The bulk wrapper capacity is `0x1400`; the actual transferred length is only
checked for nonzero before dispatch. The `0x13` branch does not ensure it is at
least `0x88e`. It also contains no scan for a sync marker, no remaining-length
loop, and no fragment accumulator. Static behavior is one dispatch per libusb
transfer, but this unsafe assumption must be checked on real captures rather
than copied into a permissive parser. The Linux parser therefore accepts only
an exact `0x88e` logical payload and rejects everything else.

The HID-HS code cannot establish reassembly. At `0x100224f4` it handles report
byte 0 equal to `0x11` and jumps to the loop end. An immediately following block
at `0x100225f7` requires the same byte to equal `0x11` before copying `0x88e`;
that block is unreachable in ordinary control flow.

## Real HID `0x15` capture and decoder

The boundary-preserving real capture contains 200 poll attempts and 200 exact
64-byte IN reports, all with byte 0 equal to `0x15`. Its SHA-256 is
`837804311fe3996adb9176c5a8ec8014c9fdcbb46240b3b887ce5d072d8e4392`.
No `0x74 0x01`, `0x13`, or `0x22` report occurred.

Mode 3 of `0x100070b0` copies 11 header bytes. In the captured low-bit profile
`flags & 0x3f == 0x1d`, the field sequence is:

```text
00        message ID 0x15
01..04    identity-like raw dword; semantics UNKNOWN
05..06    counter-like raw uint16; semantics UNKNOWN
07..0a    flags uint32 little-endian
0b..12    quaternion W/X/Y/Z, four int16 / 16384
13..18    accel X/Y/Z, three int16 / 2048
19..1e    gyro X/Y/Z, three int16 / float32 16.4
1f..24    mag X/Y/Z, three int16 / 12000
25..3f    optional fields / padding; UNKNOWN
```

The decoder does not receive a shifted pointer and no accumulator appears on
this branch. For the observed message, one USB HID report is one compact
logical sensor frame. Byte 0 is FOHEART framing, not a separate HID report ID.
The meaning and length of the optional tail remain unknown.

Across 200 unrenormalized decoded quaternions, norms ranged from
`0.99993498` to `0.99999377`. Accel magnitude was `0.9686..0.9867`, gyro
magnitude `0..0.4312`, and magnetometer magnitude `0.6061..0.6284`. The
orientation changed only `0.02211` degrees first-to-last, consistent with the
uncoordinated sensor remaining stationary. This validates the captured field
boundaries/scales but not physical axes or motion correlation.

The Euler flag (bit 5) was absent in all 200 reports. The implementation leaves
Euler undecoded and marks it STATIC_ONLY. Bytes `01..04` are retained as raw
identity-like data, but the exposed sensor name is only capture-local `slot_0`.

## Controlled-motion offline review

Offline analysis now implements WXYZ norm, conjugate, inverse, Hamilton product,
`inverse(start) * end` relative rotation, q/-q continuity, shortest distance,
angular distance, and shortest axis-angle conversion. Raw capture values are
never sign-flipped or normalized in place; continuity-adjusted quaternions are
separate derived values, and raw norms remain the sensor-quality measurement.

Segmentation thresholds are derived from a stationary baseline as the greater
of `1.5 * observed maximum` and `mean + 6 * population standard deviation` for
gyro magnitude and consecutive quaternion angle. A motion run must contain at
least three consecutive active samples; ten samples are required on each side
to label the stationary regions present. These rules were verified with
synthetic rotations but have not been validated on the real sensor.

The first guided attempt did stop with `No backend available`, but it is now
historical: a later session produced four valid 200-record captures. Each has
200 successful 64-byte `0x15` reports, zero timeout/error, and no Euler bit.

The stationary baseline gives quaternion norm `0.999939..0.999997`, mean/max
step `0.002846/0.009890` degrees, gyro magnitude mean/max
`0.136946/0.413557`, and accel mean
`(-0.047400, -0.005029, +1.024961)`. +AZ is therefore the dominant decoded
gravity component for the defined TOP-up pose.

Motion timing limits the result. The yaw file contains the post-yaw pose but no
motion; its mean is `91.6906` degrees from baseline around axis
`(QX,QY,QZ)=(+0.03958,-0.00173,-0.99922)`. Tilt and roll are active almost
from their first sample through their last, so neither contains the requested
stationary-before/after regions. Their within-file changes are only `2.6860`
and `1.4619` degrees. The roll fragment is QY-positive and has a GY-positive
peak, but integrated gyro direction differs; the tilt axes are mixed.

Consequently controlled motion is PARTIAL, not CONTROLLED_MOTION_VALIDATED as
a complete coordinate mapping. QZ is a candidate for physical UP-axis rotation;
QY/GY is a weaker candidate for physical FRONT-axis rotation. Physical RIGHT
mapping, sign convention, quaternion/gyro agreement, and handedness remain
UNKNOWN.

Downstream software does not promote those candidates. It applies an explicit
`CONFIGURED` proper rotation, retains raw WXYZ separately, and can continue
through neutral calibration, configured body mapping, torso-local FK, the
existing G1 IK, and MuJoCo. The full synthetic seven-role path is
`SIM_VALIDATED`; real multi-sensor mapping is still pending.

## Message codes

Directly correlated inner MC1507 request values are:

```text
0x01 request firmware
0x03 accel info
0x05 gyro info
0x07 mag info
0x0b set/reset zero
0x0d set/reset raw-data mode
0x11 MC1487 configuration
0x21 RTTRANS configuration
```

Each value is written next to the corresponding exported function and diagnostic
string in `0x1000d510..0x1000f603`; exact addresses are tabulated in
`protocol_status.md`. They are PC-to-device function values. They must not be
confused with outer receive code `0x13`, poll code `0x70`, command envelope
code `0x73`, or acknowledgement code `0x74`.

## Poll and RTTRANS command

The high-speed bulk worker clears 512 bytes, puts `0x70` in byte 0, and writes
all 512 bytes before every read. If a command is pending, it replaces the first
`0x86` bytes. This proves `0x70` is a default bulk poll envelope code; it does
not prove that the poll is a stream-start command.

`fhusb::reqRtTrans` constructs this inner message:

```text
+0x00  0x40
+0x01  0x21 (RTTRANS)
+0x02  target/address uint32 LE
+0x06  64-byte payload; first 11 bytes supplied by caller
```

The bulk outer message starts `73 00 00 00 00 46`, followed by the 70-byte inner
message. The worker copies a 134-byte queue object over its 512-byte zeroed poll
buffer. Bytes beyond the meaningful 76-byte outer prefix are not fully
initialized by the constructor, so a complete literal write packet cannot be
recovered safely from this path alone.

Named MotionVenus methods establish an 11-byte RTTRANS rate request for 10, 20,
50, and 100 Hz: bytes 0..4 zero, byte 5 = 1, bytes 6..7 = little-endian rate,
bytes 8..10 zero. The set methods repeat five times with 20 ms delays. Query sets
byte 5 = 2, repeats five times with 300 ms delays, and then retrieves a result.
The 1 Hz method does not initialize all 11 bytes in its observed function.

These are rate configuration messages, but static code does not establish that
they are necessary or sufficient to start live streaming. Stream content flags,
sensor masks, target semantics, initialization ordering, and semantic response
remain unresolved. No RTTRANS or `0x73` write was implemented or sent. The only
implemented hardware write is guarded byte-for-byte equality with the exact
64-byte `0x70` HID poll.

## Checksum search

The fixed record decoder and `0x13` dispatch branch have no checksum call or
validation loop. The inner/outer command constructors and bulk sender similarly
do not append a checksum. Searches for `CRC`, `checksum`, `0x1021`, `0xa001`,
and `0xedb88320` did not identify an integrity routine on these paths.
`0x10023f00` only formats a byte buffer as hexadecimal for logging.

Result: no checksum was discovered on the analyzed live fixed-frame or RTTRANS
command path. This is intentionally narrower than claiming the whole DLL has
none.

## Decoded object sizes are not wire sizes

`SensorFrame` constructor `0x10009170` initializes an object through byte
`0x380`. `MotionCaptureSuit::distributeSF` copies `0xe0` dwords and one byte,
which proves ABI size `0x381` (897). It contains a single sensor's quaternion,
Euler, accel, gyro, mag, packed identity, large state regions, and tail state.
It is not divisible into a meaningful array of `0x22` wire records.

The separate `MC1507USBSensorFrame` distribution path at `0x1000c240` copies
`0x1c3` bytes. Keeping raw USB messages, `MC1507USBSensorFrame`, decoded
`SensorFrame`, and `MotionCaptureSuit` state distinct avoids the earlier size
ambiguity.
