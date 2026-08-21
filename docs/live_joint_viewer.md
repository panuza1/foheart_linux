# Linux joint viewer

The matplotlib viewer supports the preserved seven-role upper body and the new
17-role full body through one source/slot/mapping/calibration/SuitFrame path.

```text
SOFTWARE_READY_FULL_BODY = YES
SYNTHETIC_17_SENSOR_VALIDATED = YES
REAL_17_SENSOR_VALIDATED = NO
```

The C1 and sensors were disconnected during full-body implementation. No USB
operation was performed.

## Run now

Full-body interactive animation:

```bash
python -m foheart.tools.live_joint_viewer --mode full --synthetic
```

Bounded Agg render:

```bash
MPLCONFIGDIR=/tmp/foheart-mpl \
python -m foheart.tools.live_joint_viewer \
  --mode full --synthetic --headless --duration 5
```

Upper mode remains available and is still the default:

```bash
python -m foheart.tools.live_joint_viewer --mode upper --synthetic
python -m foheart.tools.live_joint_viewer --synthetic
```

Ctrl-C/window close exits cleanly. Headless mode defaults to a bounded run when
no duration is supplied.

## Shared pipeline

```text
SyntheticSensorSource / ReplaySensorSource / LiveC1SensorSource
 -> SourceSample(existing SensorSample + opaque key)
 -> LogicalSlotRegistry
 -> BodySensorMap(profile)
 -> BasisTransform
 -> CalibrationProfile
 -> SuitFrame
 -> UpperBodyKinematics | FullBodyKinematics
 -> JointFrame | FullBodyJointFrame
 -> viewer state and renderer
```

Synthetic mode emits seven or 17 stable sensor keys and uses the same upstream
data model as future hardware. Live source construction/import does not open
USB.

## Full-body display

Full mode draws points and lines for:

```text
head and neck
derived spine/torso
shoulders, arms, wrists, hand ends
pelvis and hips
thighs, knees, lower legs, ankles, foot ends
```

XYZ scaling is equal. Coordinates are meters in `human_pelvis`, never G1
coordinates. Pelvis translation is fixed at origin; absolute walking motion is
not tracked.

Camera choices:

```bash
python -m foheart.tools.live_joint_viewer --mode full --synthetic --camera front
python -m foheart.tools.live_joint_viewer --mode full --synthetic --camera side
python -m foheart.tools.live_joint_viewer --mode full --synthetic --camera perspective
```

## Status panel and joint table

Full status shows:

```text
Source: SYNTHETIC / REPLAY / LIVE_C1
Profile: FULL
C1: CONNECTED / NOT USED / NOT CONNECTED
Sensors detected: N
Mapped: N / 17
Calibration: LOADED / MISSING
Sensor basis: CONFIGURED / MANUAL_DERIVED / PARTIAL
FPS
Frame: VALID / MISSING / STALE
Segments: 17 MEASURED + 6 DERIVED
Root translation: FIXED / NOT TRACKED
age for every one of the 17 roles
```

Missing/stale reason, recent slot-registry diagnostics, and missing saved key
bindings are shown. The joint table contains pelvis/head, bilateral shoulders,
elbows, wrists, hips, knees, and ankles in meters. It updates in the viewer and
prints once at bounded exit; it does not flood the terminal per frame.

## Orientation diagnostics

```bash
python -m foheart.tools.live_joint_viewer \
  --mode full --synthetic \
  --show-segment-axes --show-sensors
```

`--show-segment-axes` draws pelvis-relative calibrated axes at head, shoulders,
torso, pelvis, arms, forearms, hands, thighs, lower legs, and feet.

`--show-sensors` distinguishes the three shared stages:

```text
RAW SENSOR          dotted
FRAME CONVERTED     dashed
CALIBRATED SEGMENT  solid
```

X/Y/Z are red/green/blue. This supports isolation of parser, key/slot, body-map,
basis, calibration, and FK problems—especially thigh/foot sign errors.

## Invalid/stale behavior

Default full mode requires all 17 mapped/calibrated roles. Missing or stale
data prevents FK. After a valid frame has been drawn, later invalidity displays
`INVALID, LAST VALID SKELETON FROZEN`; no indefinite extrapolation occurs. Bone
diagnostics cover upper/lower limbs, feet, shoulder/hip offsets, and spine/head
lengths.

## Replay

```bash
MPLCONFIGDIR=/tmp/foheart-mpl \
python -m foheart.tools.live_joint_viewer \
  --mode full --replay samples/motion_baseline.bin --headless
```

Existing real captures contain one candidate transport key. Expected output:

```text
INSUFFICIENT REAL SENSOR ROLES
detected: 1
required: 17
```

Replay validates boundaries, decoding, raw orientation, and configured basis.
It does not synthesize the other 16 sensors or generate a full skeleton.

## Future real commands

After 17 stable slots, mapping, and calibration:

```bash
python -m foheart.tools.live_sensor_monitor --mode full --debug-transport

python -m foheart.tools.map_body_sensors \
  --mode full \
  --output config/my_full_body_mapping.yaml

python -m foheart.tools.calibrate_live \
  --mode full \
  --body-mapping config/my_full_body_mapping.yaml \
  --output config/my_full_body_neutral.yaml

python -m foheart.tools.live_joint_viewer \
  --mode full \
  --body-mapping config/my_full_body_mapping.yaml \
  --calibration config/my_full_body_neutral.yaml
```

Startup rejects missing C1, incomplete/duplicate/profile-mismatched mapping,
calibration mismatch, invalid basis, absent saved keys, or stale required
sensors. Candidate key bytes retain `UNKNOWN` physical semantics.

## Human-supervised physical validation

Observe one action at a time:

```text
A. stand neutral
B. turn head left/right
C. raise left arm
D. flex left elbow
E. rotate/move left hand
F. repeat right arm
G. rotate torso left/right
H. rotate pelvis slightly
I. raise left thigh forward
J. bend left knee
K. move left foot
L. repeat right leg
M. perform a shallow squat
N. return neutral
O. perform slow combined upper/lower movement
```

A real PASS requires human evidence that 17 sensors remain distinguishable with
no swapping; correct head/left/right chains respond; elbow, knee, hand, and foot
distal relationships are sensible; torso/pelvis behavior is sensible; bone
lengths remain constant; no role is persistently stale; and return-to-neutral is
near neutral. Software does not award that physical PASS.
