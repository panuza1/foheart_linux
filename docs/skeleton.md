# Human skeleton and forward kinematics

Both kinematics models are robot-independent and use configured right-handed
human axes:

```text
+X forward
+Y left
+Z up
units: meters
```

This is a software convention, not a validated FOHEART physical axis claim.
Every required orientation arrives through the shared basis/calibration and
stale-safe `SuitFrame` path.

## Upper mode

The preserved `UpperBodyKinematics` consumes torso plus bilateral upper arm,
forearm, and hand orientations. Its reference is `human_torso`; its local bone
axis is `-Z`.

For each side:

```text
shoulder = (0, +/- shoulder_width/2, 0)
elbow    = shoulder + R_upper_arm * (0,0,-upper_arm_length)
wrist    = elbow    + R_forearm   * (0,0,-forearm_length)
hand end = wrist    + R_hand      * (0,0,-hand_length)
```

Existing `UpperBodyTargets` and the separate G1 simulation consumer are
unchanged.

## Full mode

`FullBodyKinematics` requires all 17 measured roles and produces
`FullBodyJointFrame` in `human_pelvis` coordinates. Pelvis position is fixed at
the origin:

```text
ROOT_TRANSLATION = NOT_TRACKED_FIXED_ORIGIN
```

The pelvis sensor supplies root orientation; segment orientations are expressed
relative to it. Absolute human translation is not estimated from orientation
alone, and walking displacement is never synthesized.

Minimum output plus derived spine points:

```text
pelvis
lower_spine -> mid_spine -> torso -> neck -> head

torso -> left/right shoulder -> elbow -> wrist -> hand_end
pelvis -> left/right hip -> knee -> ankle -> foot_end
```

### Spine and head

The configured torso length is divided into three fixed-length pieces. Their
orientations use conservative quaternion interpolation from pelvis/root toward
the measured torso orientation. Neck position continues from torso; the head
sensor controls the final configured head vector. Lower/mid/upper spine and
neck are explicitly `DERIVED`.

This is a diagnostic model, not the proprietary MotionVenus biomechanics
solver.

### Shoulders and arms

Each measured shoulder orientation rotates a configured half-shoulder-width
offset from the torso point. Measured upper-arm, forearm, and hand orientations
then drive fixed `-Z` chain lengths. Left and right chains remain independent.

### Hips and legs

Left/right hips are configured half-hip-width offsets from pelvis. Measured
thigh and lower-leg orientations drive fixed `-Z` lengths. A foot orientation
drives a fixed `+X` ankle-to-foot-end vector, matching the configured human
forward axis.

## FullBodyJointFrame

A valid frame contains finite XYZ for:

```text
pelvis, lower_spine, mid_spine, torso, neck, head
left/right shoulder, elbow, wrist, hand_end
left/right hip, knee, ankle, foot_end
```

It also carries:

- all 17 calibrated segment rotation matrices relative to pelvis;
- per-joint and semantic-segment measurement status;
- `reference_frame: human_pelvis`;
- `units: meters`;
- `root_translation: NOT_TRACKED_FIXED_ORIGIN`;
- validity and reason.

Statuses are `MEASURED`, `DERIVED`, or `CONFIGURED_OFFSET`. Sensor orientation
inputs are measured; joint coordinates are derived FK; shoulder/hip/root
locations include configured offsets.

## 23-segment semantic compatibility

`FULL_BODY_23_SEGMENTS` defines:

```text
pelvis
lower_spine, mid_spine, upper_spine, chest, neck, head
left/right shoulder, upper_arm, forearm, hand
left/right thigh, lower_leg, foot, toe
```

Seventeen semantics correspond to measured orientation roles. Six
(lower/mid/upper spine, neck, and bilateral toe semantics) are derived. The
vocabulary is software-tested and allows a future adapter, but
`MotionVenus solver equivalence = NOT CLAIMED`.

## Bone-length invariants

Every full frame diagnoses:

- three torso/spine pieces, neck, and head offset;
- bilateral shoulder and hip offsets;
- bilateral upper arm, forearm, hand;
- bilateral thigh, lower leg, foot.

Distances must equal configured dimensions within numerical tolerance. FK uses
direct rotation of fixed vectors, so no integration drift changes bone length.
The viewer reports a warning and nonzero exit if a diagnostic fails.

## SuitFrame and stale behavior

The generic SuitFrame records raw, frame-converted, and calibrated
orientations; per-role age; missing/stale lists; profile; validity; and reason.
Full mode requires 17/17. A missing quaternion, missing calibration, stale
sample, or profile mismatch invalidates the frame. The viewer freezes only the
last valid skeleton and does not extrapolate.

## Synthetic validation

Full motions cover neutral standing, T-pose, independent arm raise and elbow
flex, head/torso/pelvis yaw, independent hip flex/knee bend/foot pitch/side leg
lift, squat, and coordinated whole-body movement. Tests verify chain isolation,
finite joints, root convention, head orientation, torso/pelvis response, foot
response, and every configured length across the motion set.

Real 17-sensor FK and physical placement/axis correctness remain `NOT
ATTEMPTED`.
