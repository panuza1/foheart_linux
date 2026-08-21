# FOHEART Linux user manual

## Safety boundary

The only implemented C1 hardware write is this immutable 64-byte HID poll:

```text
70 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
```

It is restricted to VID:PID `1483:5851`, configuration 1, interface 0,
alternate 0, interrupt OUT `0x01`; each write is followed by at most one
64-byte interrupt IN read from `0x81`. The safety guard rejects every other
payload, including `0x73` and RTTRANS `0x21`.

The live joint-viewer implementation did not execute this path during its
software-completion session because the C1 and sensors were disconnected.

## Upper-body viewer quick start — no hardware

```bash
python -m foheart.tools.live_joint_viewer --synthetic
```

For a bounded noninteractive check:

```bash
MPLCONFIGDIR=/tmp/foheart-mpl \
python -m foheart.tools.live_joint_viewer \
  --synthetic --headless --duration 5
```

Terminal slot diagnostics are also available offline:

```bash
python -m foheart.tools.live_sensor_monitor \
  --synthetic --samples 70 --debug-transport
```

The synthetic source supplies the same `SensorSample` boundary used by replay
and future C1 input. It is not a direct `JointFrame` animation shortcut.

## Discovery

Discovery performs no application USB write:

```bash
python -m foheart.tools.discover
```

The validated router reports product `FOHEART X Router in HS Mode` and uses
64-byte interrupt endpoints `OUT 0x01` and `IN 0x81`.

## One-shot experimental poll

This command performs one real USB OUT and at most one IN. It has no retry:

```bash
python -m foheart.tools.c1_poll_once
```

Stop after any short OUT, non-timeout USB error, reset, disconnect, or endpoint
change. A normal single-read timeout is reported without retry.

## Bounded capture

Use bounded capture only after a successful one-shot response:

```bash
python -m foheart.tools.c1_capture \
  --polls 200 \
  --timeout-ms 100 \
  --output samples/c1_real_poll_capture.bin \
  --hex
```

The hard limits cannot be disabled: at most 200 polls, at most 100 ms per USB
operation, and at most 30 seconds total. The output path must not already
exist. Every record preserves poll sequence/timestamp, OUT length, IN
timestamp/endpoint/payload boundary, timeout/error state, and round-trip time.

## Offline replay and monitoring

These commands perform no USB operations:

```bash
python -m foheart.tools.replay samples/c1_real_poll_capture.bin
python -m foheart.tools.monitor \
  --capture samples/c1_real_poll_capture.bin \
  --count 10
```

The offline monitor labels the single decoded sensor `Slot 0`; that is a
capture-local record slot, not a recovered physical identity or body placement.

The upper-body replay viewer is:

```bash
MPLCONFIGDIR=/tmp/foheart-mpl \
python -m foheart.tools.live_joint_viewer \
  --replay samples/motion_baseline.bin \
  --headless
```

Current real files contain one role only. The expected result is
`INSUFFICIENT REAL SENSOR ROLES`, not a skeleton.

## Current decoding status

For the real HID `0x15` profile, quaternion W/X/Y/Z, accel, gyro, and
magnetometer are `REAL_CAPTURE_VALIDATED`. Euler is `STATIC_ONLY` because its
flag was absent in every real report. Physical sensor identity, header bytes
`1..6`, high flag bits, and optional trailing bytes remain `UNKNOWN`.
Controlled motion has validated TOP-up gravity on +AZ and produced partial
QZ/QY axis candidates, but the complete physical frame, signs, and handedness
remain `UNKNOWN`.

The bulk `0x13`/`0x88e` and `0x22` record parser is independent and remains
`STATIC_ONLY`; do not apply it to a 64-byte HID report.

## Offline motion analysis

This command performs no USB operations:

```bash
python -m foheart.tools.motion_analyze CAPTURE
```

For motion segmentation, provide a separately recorded stationary baseline:

```bash
python -m foheart.tools.motion_analyze \
  samples/motion_table_yaw_cw.bin \
  --baseline samples/motion_baseline.bin
```

The analysis retains raw WXYZ values and separately derives q/-q
continuity-adjusted values. It does not rewrite a capture or renormalize raw
quaternions before reporting their norms.

## Guided controlled-motion capture

```bash
python -m foheart.tools.guided_motion_capture
```

The tool has no payload, endpoint, timeout, or poll-count options. It reuses the
validated bounded poller for four separate 200-poll phases and refuses to
overwrite any raw capture or summary. Each phase still sends only one exact
`0x70` OUT followed by at most one `0x81` IN per iteration.

The first guided attempt on 2026-08-21 stopped before device open with
`No backend available`. The later session superseded it with four valid
200-record captures and `samples/motion_validation_summary.json`. Despite its
name, `motion_baseline_failed_backend.bin` itself contains 200 valid reports; it
is an older incomplete-session capture, not an eight-byte backend-failure marker,
and is excluded by mtime/session completeness.

Offline review classifies that session as PARTIAL: baseline is valid, but yaw
motion occurred before its capture and tilt/roll lack stationary bookends. Do
not use the current summary as a calibration or handedness transform. Use
`motion_analyze` to inspect the immutable raw files; repeat controlled motion in
a future separately authorized task only if complete axis mapping is required.

## Offline neutral calibration

Create a versioned profile from an explicit body-role to WXYZ mapping:

```bash
python -m foheart.tools.calibrate \
  --input neutral_wxyz.yaml \
  --output calibration.yaml
```

For deterministic software testing only:

```bash
python -m foheart.tools.calibrate \
  --synthetic-upper-body \
  --output /tmp/foheart-neutral.yaml
```

Both modes are offline and refuse to overwrite output. The implementation uses
`inverse(neutral) * current`; raw quaternions remain separate. A live
seven-sensor neutral calibration has not been validated.

## Body mapping and skeleton

The upper-body model requires configured slots for torso, left/right upper arm,
left/right forearm, and left/right hand. The default mapping is empty and fails
closed. Physical sensor IDs and body placement are not inferred.

The FK output is a torso-local, configured right-handed frame with `+X`
forward, `+Y` left, and `+Z` up. Left/right wrist outputs are homogeneous
`4x4` poses in meters. See `body_mapping.md`, `calibration.md`, and
`skeleton.md`.

## Future seven-sensor setup

Recommended minimum physical placements:

```text
torso
left upper arm
left forearm
left hand
right upper arm
right forearm
right hand
```

Hardware identity does not select these roles. Use this exact sequence after
the C1 and sensors are connected:

```bash
# 1. Inspect stable session slots.
python -m foheart.tools.live_sensor_monitor

# 2. Assign all seven body roles; add --motion-assisted if wanted.
python -m foheart.tools.map_body_sensors \
  --output config/my_body_mapping.yaml

# 3. Capture the neutral pose.
python -m foheart.tools.calibrate_live \
  --body-mapping config/my_body_mapping.yaml \
  --output config/my_neutral.yaml

# 4. Start the real joint viewer.
python -m foheart.tools.live_joint_viewer \
  --body-mapping config/my_body_mapping.yaml \
  --calibration config/my_neutral.yaml
```

The mapping tool saves candidate transport keys separately from roles and
requires manual confirmation for motion-assisted proposals. It refuses to
overwrite output. If fewer than seven stable keys are visible, stop and retain
`NOT ATTEMPTED`; do not number packets or invent sensor IDs.

## Neutral pose for the future run

Feet and legs do not matter for the current upper-body test. Stand with torso
upright, face forward, shoulders neutral, arms straight down at the sides, and
palms facing inward. Hold still until all seven role sample counts and quality
checks pass. This arms-down pose is the only neutral convention for this
viewer.

## Future real joint validation

With live mapping and calibration loaded, observe these actions one at a time:

```text
A. stand neutral
B. raise left upper arm
C. flex left elbow
D. rotate/move left hand
E. return neutral

F. raise right upper arm
G. flex right elbow
H. rotate/move right hand
I. return neutral

J. rotate torso slowly left/right
```

PASS requires human observation that left motion affects the left chain, right
motion affects the right chain, elbow motion changes forearm/wrist relation,
the hand sensor changes distal orientation, torso-relative behavior is correct,
configured bone lengths remain constant, slots do not swap, and no prolonged
stale state appears. The software does not automatically promote these physical
checks to PASS.

For basis/sign diagnosis, repeat with `--show-segment-axes --show-sensors`.
Raw sensor axes are dotted, frame-converted axes dashed, and calibrated segment
axes solid.

## G1 MuJoCo validation

The reproducible end-to-end software demo is:

```bash
cd /home/panu/Documents/fibo/project_humanoid/fh/foheart_linux
env PYTHONPATH="$PWD/src" MPLCONFIGDIR=/tmp/foheart-mpl \
  /home/panu/miniconda3/envs/tv/bin/python \
  -m foheart.tools.g1_sim_replay \
  --config config/default.yaml \
  --capture-check samples/motion_baseline.bin
```

It loads the existing local `G1_29_ArmIK` and actual G1
`scene_29dof.xml` directly, then runs seven bounded synthetic upper-body
poses. It performs no USB operation, starts no DDS bridge, and has no physical
G1 mode. The saved run is `SIM_VALIDATED`; see `mujoco.md`.

The real capture check validates one-sensor decoding and configured frame
conversion only. A complete real body-to-simulation run remains disabled until
seven actual slots are explicitly mapped and calibrated.

## Full-body viewer quick start — no hardware

Full mode extends the same source, logical slots, mapping, basis, calibration,
and stale-watchdog path to 17 roles. Upper mode remains the default and does
not require the additional sensors.

```bash
python -m foheart.tools.live_joint_viewer --mode full --synthetic

MPLCONFIGDIR=/tmp/foheart-mpl \
python -m foheart.tools.live_joint_viewer \
  --mode full --synthetic --headless --duration 5
```

The deterministic source emits 17 distinct `SensorSample` streams and drives
the complete full-body pipeline. Pelvis translation remains fixed at the
origin; the viewer does not invent walking displacement from orientation-only
IMU input.

For sensor/segment diagnosis:

```bash
python -m foheart.tools.live_joint_viewer \
  --mode full --synthetic \
  --show-segment-axes --show-sensors
```

The status panel reports `FULL`, detected/mapped counts, calibration and basis
status, frame state, and the age of all 17 roles. The joint table includes
pelvis/head and bilateral shoulder, elbow, wrist, hip, knee, and ankle
coordinates in meters.

## Full-body roles and placement

Full mode requires exactly these measured roles:

```text
head
left_shoulder right_shoulder
torso pelvis
left_upper_arm right_upper_arm
left_forearm right_forearm
left_hand right_hand
left_thigh right_thigh
left_lower_leg right_lower_leg
left_foot right_foot
```

The approximate wearing guidance is `MANUAL_DERIVED`: head mount; corresponding
shoulder locations; back/upper-torso sensor; waist/crotch/pelvis sensor; sensors
on the outside of upper arms and forearms; back of hands; outside middle
thighs; lower legs below the knee where muscle influence is minimized; and
feet/shoes following the FOHEART guide-column direction. This project has not
physically validated those positions or the complete signed sensor-axis map.

## Full-body standing neutral

For calibration, face the head forward, keep torso upright and pelvis neutral,
relax and level the shoulders, hold straight arms down at the sides with palms
inward, keep legs straight, and point parallel feet forward. Hold completely
still. This is the sole calibration neutral convention. Synthetic `t_pose` is
a motion test, not an alternative neutral.

The estimator normalizes samples, maintains quaternion sign continuity,
normalizes the mean, and checks angular spread plus gyro motion for every role.
Generated files remain `live_validated: false` until a later physical session.

## Future full-suit workflow

Do not proceed past monitoring unless 17 unique, stable candidate transport
keys are visible with no collision diagnostic.

```bash
# 1. Inspect all streams after connecting C1 and powering 17 sensors.
python -m foheart.tools.live_sensor_monitor --mode full --debug-transport

# 2. Place sensors and assign all roles with manual confirmation.
python -m foheart.tools.map_body_sensors \
  --mode full \
  --output config/my_full_body_mapping.yaml

# 3. Hold the documented standing neutral pose.
python -m foheart.tools.calibrate_live \
  --mode full \
  --body-mapping config/my_full_body_mapping.yaml \
  --output config/my_full_body_neutral.yaml

# 4. Start the full-body viewer.
python -m foheart.tools.live_joint_viewer \
  --mode full \
  --body-mapping config/my_full_body_mapping.yaml \
  --calibration config/my_full_body_neutral.yaml
```

Mapping records anatomy as `CONFIGURED`, never auto-discovered. Motion-assisted
mapping only proposes the strongest moving slot and always requires explicit
confirmation. Candidate header bytes remain opaque transport keys with
`UNKNOWN` physical identity semantics.

## Future real full-body validation

With 17 roles mapped and calibrated, observe these actions one at a time:

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

A real PASS requires human observation of 17 continuously distinguishable
streams without slot swapping; correct unswapped head/arm/leg chains; sensible
elbow, knee, hand, foot, torso, and pelvis responses; fixed configured bone
lengths; no persistent stale role; and a return near neutral. Until then, real
17-slot discovery, mapping, calibration, and live full-body viewing remain
`NOT VALIDATED`.

See `full_body.md` for the FK/reference model, derived spine/neck status, and
23-segment semantic compatibility boundary.
