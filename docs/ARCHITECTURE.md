# Architecture

## System flow

```text
FOHEART suit
  |
  v
MotionVenus on Windows
  |
  | UDP V4003: global position + quaternion
  v
MotionVenusReceiver
  |
  v
MotionVenusStreamDecoder
  |
  v
HumanSkeletonFrame
  |
  v
HeadingCalibration
  |
  v
MotionVenusGMRAdapter
  |
  v
GeneralMotionRetargeting (pinned GMR)
  |
  v
G1ReferenceProcessor
  |
  v
ProcessedG1Reference
  |------------------------------------------|
  v                                          v
FoheartTeleopitAdapter                 MotionRecorder
  |                                          |
  v                                          v
TeleopIt                                    PKL
  |                                          |
  v                                          v
Tracking policy                         Dataset builder
  |                                          |
  v                                          v
Free-base MuJoCo                       General-Tracking-G1
  |                                          |
  v                                          v
Future G1 backend                      New ONNX policy
```

The left branch executes the released policy. The right branch records the
processed intended motion for later training. The direct MuJoCo branch remains
separate for reference and safety debugging:

```text
G1ReferenceProcessor -> reference FK or bounded direct MuJoCo
```

## Ownership

`foheart_linux` owns:

- MotionVenus UDP reception, capture, replay, and strict V4003 decoding;
- the immutable solved human skeleton;
- heading normalization and the MotionVenus-to-GMR semantic/basis mapping;
- the pinned GMR call and its 36D result validation;
- hard limits, optional soft margins, optional EMA/rate limiting, and
  FOLLOW/HOLD behavior in `G1ReferenceProcessor`;
- recording the accepted intended G1 reference.

TeleopIt owns:

- reference-derived policy features and velocities;
- the 167D observation and 10-frame observation history;
- released and future tracking policies;
- the 29D policy action, default pose, action scaling, and clipping;
- 50 Hz policy inference and 200 Hz PD/control;
- free-base MuJoCo;
- `General-Tracking-G1` dataset/training/export tooling;
- the future real-G1 backend.

The project does not duplicate TeleopIt's observation builder, history buffer,
policy runner, PD controller, trainer, or robot bridge.

## MotionVenus contract

The supported source is MotionVenus SDK/custom protocol V4003:

- little-endian binary with a 128-byte header;
- 23 canonical body bones without optional finger groups;
- global positions in metres (`int32 / 2^16` on the wire);
- quaternions transmitted as XYZW (`int16 / 2^13` on the wire);
- position and quaternion fields both required;
- global rotation required for the current GMR path.

The receiver binds only when started. MVUDP capture preserves each datagram,
host receive time, and sender, and replay returns those boundaries to the same
decoder. The watchdog tracks duplicates, gaps, out-of-order frames, sender
changes, malformed input, and staleness.

`HumanSkeletonFrame` is robot-independent. It contains the solved 23-bone body
and never treats human Euler or bone angles as G1 motor angles.

## GMR boundary

Pinned `GeneralMotionRetargeting` uses:

```text
src_human = xsens_mvn
tgt_robot = unitree_g1
commit = bb1bbe40774794fceb2a7c579a3464a28e68c844
```

`MotionVenusGMRAdapter` maps the source names needed by GMR, converts global
XYZW quaternions to WXYZ, applies the configured proper basis rotation, and
normalizes initial pelvis heading. It does not implement IK; GMR remains the
only whole-body retargeter.

The result is a MuJoCo-style 36D qpos:

```text
qpos[0:3]   root position XYZ in world metres
qpos[3:7]   normalized root quaternion WXYZ
qpos[7:36]  29 G1 joints in radians
```

The pinned 29-joint order is:

```text
left_hip_pitch_joint, left_hip_roll_joint, left_hip_yaw_joint,
left_knee_joint, left_ankle_pitch_joint, left_ankle_roll_joint,
right_hip_pitch_joint, right_hip_roll_joint, right_hip_yaw_joint,
right_knee_joint, right_ankle_pitch_joint, right_ankle_roll_joint,
waist_yaw_joint, waist_roll_joint, waist_pitch_joint,
left_shoulder_pitch_joint, left_shoulder_roll_joint,
left_shoulder_yaw_joint, left_elbow_joint, left_wrist_roll_joint,
left_wrist_pitch_joint, left_wrist_yaw_joint,
right_shoulder_pitch_joint, right_shoulder_roll_joint,
right_shoulder_yaw_joint, right_elbow_joint, right_wrist_roll_joint,
right_wrist_pitch_joint, right_wrist_yaw_joint
```

## Processed reference and safety

`G1ReferenceProcessor` is the trust boundary after GMR. It validates shape,
finiteness, root quaternion, source ordering, and the exact joint order. It
applies the conservative intersection of available source/model limits, then
optional soft-limit margins, EMA smoothing, and time-based joint-rate limits.

Accepted output is `ProcessedG1Reference`. Invalid or stale input stops
FOLLOW and retains the last accepted safe target as HOLD. The policy adapter
does not add a second joint-limit policy.

## TeleopIt interface

The only policy integration boundary is:

```text
ProcessedG1Reference -> FoheartTeleopitAdapter -> TeleopIt
```

`FoheartTeleopitAdapter` validates the 36D contract, derives its mapping from
joint names, converts absolute source time to a zero-based reference timeline,
returns immutable arrays, and holds the previous valid result for a
missing/stale/out-of-order reference. It performs no GMR, IK, policy inference,
PD control, joint limiting, DDS, or physical robot control.

The foheart and TeleopIt 29-joint sequences are currently identity by name.
Both runtime boundaries use root XYZ plus WXYZ, so the adapter performs no root
representation conversion.

The validated TeleopIt pair is:

```text
TeleopIt commit  f9263865c581802ad531854b8e547e2403a945f3
model            assets/robots/unitree_g1/g1_29dof.xml
policy           ckpt/track_g1.onnx
model dimensions nq=36, nv=35, nu=29; free root
policy inputs    obs [1,167], obs_history [1,10,167]
policy output    actions [1,29]
policy rate      50 Hz
PD rate          200 Hz (decimation 4, dt 0.005 s)
```

TeleopIt's default `reference_steps=[0]` consumes the current 36D reference.
Its existing runtime derives joint and torso-anchor velocities at policy rate
and owns the observation history. No hidden padding, trimming, or observation
normalizer is applied at the project boundary.

The GMR, direct-simulation, and TeleopIt XML files share dimensions and joint
order but differ in body meshes and some limits. The models are not replaced or
treated as identical; the processed-reference boundary isolates the differences.

## Recording and training data

`MotionRecorder` branches after processing and before dynamic servo tracking.
It writes exactly:

```text
fps
root_pos        (T,3), metres
root_rot        (T,4), normalized XYZW
dof_pos         (T,29)
local_body_pos  (T,38,3), pinned-model root-neutral FK
link_body_list  38 names in pinned-model order
```

Runtime qpos uses WXYZ; the PKL intentionally uses XYZW. TeleopIt's current
`type: pkl` loader performs XYZW-to-WXYZ conversion, derives joint velocities,
and recomputes its required body FK and velocities. The recorded 38-body
metadata is a valid superset, so no converter is required.

The future training branch is:

```text
PKL -> TeleopIt minimal dataset -> precomputed dataset
  -> General-Tracking-G1 -> PT checkpoint -> ONNX -> same runtime
```

The isolated inference environment does not contain the training stack or
training datasets. No training smoke test, checkpoint, or new ONNX policy
exists yet.

## Simulation and evidence boundaries

| Path | Purpose | Current evidence |
|---|---|---|
| Reference MuJoCo | geometry/FK inspection | READY |
| Direct MuJoCo | retarget and safety debugging | READY; not balance evidence |
| TeleopIt free-base MuJoCo | policy tracking and bounded balance | synthetic and MVUDP PASS |
| Live MotionVenus to policy sim | operator/suit validation | software ready; manual run pending |
| Physical G1 | later deployment | not validated |

The synthetic and MVUDP policy tests each ran ten actual-GMR references for
five seconds with finite 167D observations, 29D actions, and free-base state.
That proves bounded upright pose tracking only. It does not prove walking,
general balance robustness, or physical-robot safety.

## Future real G1 boundary

The intended real path changes only the backend after the common TeleopIt
observation/controller path:

```text
same frontend -> FoheartTeleopitAdapter -> observation builder -> policy
  -> TeleopIt G1 backend -> physical G1
```

TeleopIt's real backend was source-audited for state acquisition, mode gates,
fresh-reference checks, stale hold, joint clipping, startup gain ramp, damping,
and controlled shutdown. It was not initialized. No DDS, robot mode change,
motor enable, or physical motion occurred. A separate safety review and
authorized staged hardware validation are required before this branch is used.

## Preserved legacy boundary

The repository still includes experimental direct C1 USB polling, packet
decoding, sensor-role mapping, calibration, and synthetic orientation-driven
FK. Those modules are isolated from the preferred MotionVenus solved-skeleton
pipeline. Preserved sample captures are evidence/fixtures, not live runtime
inputs and not proof of a complete physical sensor-axis or 17-sensor mapping.
