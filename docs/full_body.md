# FOHEART X full-body profile

The `full` profile is a robot-independent, orientation-driven diagnostic
skeleton for 17 FOHEART sensor roles. It does not reproduce MotionVenus's
proprietary biomechanics solver and it does not estimate absolute human
translation.

```text
17 SENSOR INGESTION                 SOFTWARE_TESTED
17 ROLE BODY MAPPING                SOFTWARE_TESTED
17 ROLE CALIBRATION                 SOFTWARE_TESTED
FULL-BODY FK                        SOFTWARE_VALIDATED
DERIVED SPINE/NECK SEGMENTS         SOFTWARE_TESTED / DERIVED
23-SEGMENT SEMANTIC COMPATIBILITY   SOFTWARE_TESTED / DERIVED
MOTIONVENUS SOLVER EQUIVALENCE      NOT CLAIMED
REAL 17-SENSOR OPERATION            NOT VALIDATED
```

## Required measured roles

Full mode requires all of these roles by default:

```text
head
left_shoulder             right_shoulder
torso
pelvis
left_upper_arm            right_upper_arm
left_forearm              right_forearm
left_hand                 right_hand
left_thigh                right_thigh
left_lower_leg            right_lower_leg
left_foot                 right_foot
```

Transport key, logical slot, optional physical label, body role, skeleton
segment, and joint are separate concepts. Candidate C1 header bytes identify a
stream only for the current session; their physical meaning remains `UNKNOWN`.

## Wearing positions

The following placement guidance is `MANUAL_DERIVED`, not physically validated
by this project:

| Role | Approximate placement |
|---|---|
| `head` | head-mounted sensor |
| `left_shoulder`, `right_shoulder` | corresponding shoulder locations |
| `torso` | back / upper torso |
| `pelvis` | crotch / waist / pelvis location |
| upper arms | outside upper arm |
| forearms | outside forearm, near but not on the wrist joint |
| hands | back of each hand |
| thighs | outside middle thigh |
| lower legs | below the knee where muscle influence is minimized |
| feet | on shoes/feet according to the wearing layout |

Foot sensor orientation and guide-column direction matter. Use
`--show-sensors --show-segment-axes` during physical validation; do not encode
an unverified signed-axis permutation as hardware truth.

## Neutral capture convention

Use one standing neutral convention:

```text
head facing forward
torso upright
pelvis neutral
shoulders relaxed and level
arms straight down at the sides, palms inward
legs straight
feet parallel and facing forward
```

This is the sensor neutral capture and software skeleton reference used here.
The synthetic `t_pose` is a motion-test pose, not a second calibration
convention.

## Dimensions and coordinate frame

Defaults in `config/default.yaml` are safe synthetic examples, not the user's
measurements. Configure shoulder width, torso/neck/head offsets, bilateral arm,
hand, hip, thigh, lower-leg, and foot dimensions in meters.

The FK convention is right-handed:

```text
+X forward
+Y left
+Z up
origin: pelvis
units: meters
ROOT_TRANSLATION: NOT TRACKED / FIXED AT ORIGIN
```

Pelvis orientation drives the root-relative pose. Arms start at configured
shoulder offsets, legs start at configured hip offsets, and every measured
segment orientation advances a configured fixed-length bone. Walking
translation is deliberately not fabricated from orientation-only IMU data.

## Measured, derived, and configured data

The 17 role orientations are `MEASURED` inputs after mapping, basis conversion,
and neutral calibration. The pelvis origin and shoulder/hip offsets are
`CONFIGURED_OFFSET`. Lower/mid/upper spine, chest, and neck behavior is
`DERIVED` conservatively between pelvis, torso, shoulder, and head inputs.

The exported 23-segment semantic vocabulary is:

```text
pelvis
lower_spine mid_spine upper_spine chest neck head
left/right shoulder
left/right upper_arm
left/right forearm
left/right hand
left/right thigh
left/right lower_leg
left/right foot
left/right toe
```

It is a compatibility vocabulary with explicit status metadata. It is not a
claim of proprietary MotionVenus solver equivalence.

## Run now without hardware

```bash
python -m foheart.tools.live_joint_viewer --mode full --synthetic

MPLCONFIGDIR=/tmp/foheart-mpl \
python -m foheart.tools.live_joint_viewer \
  --mode full --synthetic --headless --duration 5
```

Synthetic input still travels through `SensorSample`, 17 transport keys,
`LogicalSlotRegistry`, `BodySensorMap`, calibration, `SuitFrame`, full-body FK,
and the renderer. It does not inject joints directly.

## Future 17-sensor procedure

Only when the hardware is connected:

1. Connect the C1 and power all 17 sensors.
2. Run `python -m foheart.tools.live_sensor_monitor --mode full --debug-transport`.
3. Verify 17 stable slots and no key collision; stop if fewer appear.
4. Put the sensors on the documented body locations.
5. Run `python -m foheart.tools.map_body_sensors --mode full --output config/my_full_body_mapping.yaml`.
6. Assign every role and manually confirm any motion-assisted proposal.
7. Run `python -m foheart.tools.calibrate_live --mode full --body-mapping config/my_full_body_mapping.yaml --output config/my_full_body_neutral.yaml`.
8. Hold the documented standing neutral pose and require acceptable quality for all roles.
9. Run `python -m foheart.tools.live_joint_viewer --mode full --body-mapping config/my_full_body_mapping.yaml --calibration config/my_full_body_neutral.yaml`.

The slot registry fails closed for observable key conflicts, saved-key
mismatches, slot reassignment, and unexpected new slots after a session is
frozen. If two physical devices emit indistinguishable candidate keys, they
cannot be safely separated: detected count remains below 17 and full mode must
not start.

## Future human-supervised validation

Observe each action independently:

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

A real PASS requires human evidence of 17 distinguishable stable streams,
correct unswapped chains, sensible head/torso/pelvis behavior, responsive
elbows/knees/hands/feet, invariant configured bone lengths, no persistent
staleness, and return near neutral. Software tests cannot grant that PASS.
