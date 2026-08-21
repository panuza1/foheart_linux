# Validation matrix

Statuses distinguish saved real evidence, offline software tests, synthetic
validation, and work that requires the disconnected suit.

## C1 transport and decoding

| Capability | Status | Evidence |
|---|---|---|
| C1 descriptors | `REAL_CAPTURE_VALIDATED` | `1483:5851`, interface 0, interrupt OUT `0x01`, IN `0x81`, 64 bytes |
| Immutable `0x70 + 63 zero` polling | `REAL_CAPTURE_VALIDATED` | earlier bounded 200 OUT/IN capture; no other application payload validated |
| HID `0x15` framing | `REAL_CAPTURE_VALIDATED` | earlier real reports |
| Quaternion WXYZ `/16384` | `REAL_CAPTURE_VALIDATED` | real report decoding and norm evidence |
| Accel `/2048`, gyro `/16.4`, mag `/12000` | `REAL_CAPTURE_VALIDATED` | real report decoding |
| Real one-sensor orientation replay | `REAL_CAPTURE_VALIDATED` + `CONFIGURED` | decoder, candidate key, configured basis |
| Complete signed sensor-axis mapping | `UNKNOWN` | controlled physical evidence remains partial |
| Candidate transport-key semantics | `PARTIAL` | header bytes 1..4 exposed opaquely; physical identity unknown |
| Live C1 source | `SOFTWARE_TESTED`, `NOT EXECUTED` | fake-device boundary tests; no USB in this session |

## Shared sensor pipeline

| Capability | Status | Evidence |
|---|---|---|
| Upper/full body profile system | `SOFTWARE_TESTED` | exact 7/17 requirements and legacy upper YAML compatibility |
| Source abstraction | `SOFTWARE_TESTED` | synthetic, replay, and guarded live adapters |
| Synthetic upper source | `SOFTWARE_VALIDATED` | deterministic seven-role motion path |
| Synthetic full source | `SOFTWARE_VALIDATED` | 17 stable keys and complete deterministic motion sequence |
| Replay source | `SOFTWARE_TESTED` | real `FHC1POL` fixture/captures; one distinct key |
| Logical slot registry | `SOFTWARE_TESTED` | arbitrary allocation, saved bindings, `slot_0` through `slot_16` exercised |
| Collision/reassignment/new-slot diagnostics | `SOFTWARE_TESTED` | conflicting metadata, frozen session, saved-key mismatch, disappearance |
| Real 17 simultaneous slots | `NOT ATTEMPTED` | hardware intentionally disconnected |
| Real key uniqueness/stability | `NOT VALIDATED` | requires 17 powered physical sensors |
| Live sensor monitor | `SOFTWARE_TESTED` | upper/full synthetic and replay paths; live path not executed |
| Sensor-basis validation | `SOFTWARE_TESTED` | finite orthonormal determinant +1; reflections rejected |
| Default basis | `CONFIGURED` | identity safe example, not physical FOHEART truth |
| Manual-derived basis support | `SOFTWARE_TESTED` | evidence label supported; no definitive physical mapping claimed |

## Mapping and calibration

| Capability | Status | Evidence |
|---|---|---|
| Seven-role upper mapping | `SOFTWARE_TESTED` | complete/duplicate/unknown/profile checks |
| Seventeen-role full mapping | `SOFTWARE_TESTED` | versioned YAML round trip and exact-role validation |
| Motion-assisted mapping | `SOFTWARE_TESTED` | generic energy ranking with mandatory manual confirmation |
| Real full-body role mapping | `NOT ATTEMPTED` | anatomy cannot be inferred from opaque keys |
| Quaternion neutral estimator | `SOFTWARE_TESTED` | normalization, q/-q continuity, normalized mean, angular spread |
| Upper calibration | `SOFTWARE_TESTED` | shared synthetic capture and quality metadata |
| Full 17-role calibration | `SOFTWARE_TESTED` | all-role capture and per-role norm/spread/gyro quality |
| Real full-body calibration | `NOT ATTEMPTED` | generated files retain `live_validated: false` |

## Suit frames and skeletons

| Capability | Status | Evidence |
|---|---|---|
| Canonical profile-aware `SuitFrame` | `SOFTWARE_TESTED` | upper/full role validation and calibrated orientations |
| Missing/stale watchdog | `SOFTWARE_TESTED` | all required roles checked; no fabrication or extrapolation |
| Upper-body FK / `JointFrame` | `SOFTWARE_VALIDATED` | preserved tests, finite torso-frame positions and invariants |
| Full-body dimensions | `SOFTWARE_TESTED` | configurable synthetic defaults and bilateral lengths |
| `FullBodyKinematics` | `SOFTWARE_VALIDATED` | head, spine, arms, pelvis, hips, legs, and feet |
| `FullBodyJointFrame` | `SOFTWARE_VALIDATED` | finite pelvis-frame meter coordinates and semantic status |
| Bone-length invariance | `SOFTWARE_VALIDATED` | arms/hands/thighs/lower legs/feet/spine offsets across motions |
| Root translation | `NOT TRACKED / FIXED` | explicit orientation-only diagnostic limitation |
| Derived spine/neck model | `SOFTWARE_TESTED / DERIVED` | conservative interpolation with status metadata |
| 23-segment semantic model | `SOFTWARE_TESTED / DERIVED` | 23 named segments with measured/derived distinction |
| MotionVenus solver equivalence | `NOT CLAIMED` | proprietary biomechanics behavior is unknown |

## Viewer and replay

| Capability | Status | Evidence |
|---|---|---|
| Upper matplotlib viewer | `SOFTWARE_VALIDATED` | shared renderer and bounded Agg regression |
| Full matplotlib viewer | `SOFTWARE_VALIDATED` | full skeleton, equal scale, status, table, bounded Agg render |
| Segment axes | `SOFTWARE_TESTED` | upper/full calibrated axes |
| Raw/converted/calibrated sensor axes | `SOFTWARE_TESTED` | distinct styles and XYZ colors for all profile roles |
| Synthetic seven-role end to end | `PASS` | samples -> slots -> mapping -> calibration -> SuitFrame -> FK -> renderer |
| Synthetic 17-role end to end | `PASS` | 17 samples/slots/roles through full-body renderer |
| One-real-sensor full replay | `PASS (FAIL-CLOSED)` | reports `INSUFFICIENT REAL SENSOR ROLES`, detected 1 / required 17 |
| Real 17-sensor full viewer | `NOT ATTEMPTED` | requires physical mapping and calibration |
| Human-observed full-body motion | `NOT ATTEMPTED` | requires later A–O procedure |

## G1 regression boundary

| Capability | Status | Evidence |
|---|---|---|
| Human-to-G1 upper-arm adapter | `CONFIGURED` (offline-tested) | existing frame, reach clamp, and filter tests |
| Existing `G1_29_ArmIK` | `SIM_VALIDATED` | actual local solver and bounded synthetic targets |
| G1 joint ordering / MuJoCo model | `SIM_VALIDATED` | actual 29-DOF scene and ordering checks |
| Synthetic upper body -> G1 MuJoCo | `SIM_VALIDATED` | existing bounded regression retained |
| Full-body G1 control | `NOT IMPLEMENTED` | only robot-independent full-body output added |
| Physical Unitree G1 | `NOT ATTEMPTED` | outside this task; no physical command path used |

```text
SOFTWARE_READY_FULL_BODY = YES
SYNTHETIC_17_SENSOR_VALIDATED = YES
REAL_17_SENSOR_VALIDATED = NO
REAL_17_SLOT_VALIDATED = NO
REAL_FULL_BODY_MAPPING_VALIDATED = NO
REAL_FULL_BODY_CALIBRATION_VALIDATED = NO
REAL_FULL_BODY_VIEWER_VALIDATED = NO
```
