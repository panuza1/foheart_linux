# Architecture

Protocol, human kinematics, visualization, robot retargeting, and simulation are
separate. No USB parser imports matplotlib, G1, or MuJoCo code.

```text
Synthetic source / FHC1POL replay / future live C1
  -> SourceSample(existing SensorSample + opaque TransportKey)
  -> LogicalSlotRegistry
       stable session-local slot_N
       collision/reassignment/new/disappearance diagnostics
  -> LatestSuitBuffer
  -> CONFIGURED BodySensorMap(profile=upper|full)
  -> configured proper BasisTransform
  -> shared CalibrationProfile
  -> shared SuitFrame(profile, ages, missing/stale, validity)
       |
       +-> upper (7 roles) -> UpperBodyKinematics -> JointFrame
       |                                      |-> matplotlib viewer
       |                                      `-> existing UpperBodyTargets/G1 branch
       |
       `-> full (17 roles) -> FullBodyKinematics -> FullBodyJointFrame
                                              `-> same matplotlib viewer
```

Synthetic data enters as `SensorSample`; it does not bypass slots, mapping,
calibration, SuitFrame, FK, or rendering. Replay preserves the real `0x15`
decoder boundary. Importing or constructing `LiveC1SensorSource` does not open
USB; only `start()` invokes the already guarded C1 poll layer.

## Body profiles

`BodyProfile.UPPER` preserves the original seven required roles:

```text
torso
left/right upper_arm
left/right forearm
left/right hand
```

`BodyProfile.FULL` requires exactly 17 measured orientation roles:

```text
head
left_shoulder, right_shoulder
torso, pelvis
left_upper_arm, right_upper_arm
left_forearm, right_forearm
left_hand, right_hand
left_thigh, right_thigh
left_lower_leg, right_lower_leg
left_foot, right_foot
```

Transport/header identity, logical slot, body role, optional physical sticker,
skeleton segment, and joint are distinct concepts.

## Shared layer contracts

| Layer | Contract | Evidence boundary |
|---|---|---|
| USB poll/capture | exact C1 `1483:5851`, 64-byte `0x70 + 63 zero` request | `REAL_CAPTURE_VALIDATED`; no other output permitted |
| HID decoder | one real 64-byte `0x15` -> raw WXYZ/accel/gyro/mag | `REAL_CAPTURE_VALIDATED` |
| Source | one explicit `SourceSample` at a time | synthetic/replay tested; live fake-device tested, not executed this session |
| Registry | new opaque key -> first free `slot_N`, retained for session | arbitrary count; tested through `slot_16` |
| Mapping | unique logical slot per required role | `CONFIGURED`; never anatomical auto-discovery |
| Basis | proper 3x3 rotation only | identity default `CONFIGURED`; physical mapping incomplete |
| Calibration | normalized sign-continuous quaternion mean plus motion quality | shared 7/17-role software tests; `live_validated: false` |
| SuitFrame | exact profile roles, ages, raw/converted/calibrated stages | missing/stale role invalidates frame; no fabrication |
| Upper FK | torso-local arms | existing software validated, G1 consumer preserved |
| Full FK | pelvis-rooted head/spine/arms/legs/feet | synthetic 17-role software validated |
| Viewer | joints/bones/status/tables/debug axes | upper/full Agg rendering tested |
| G1 | upper wrist targets -> existing IK -> MuJoCo | `SIM_VALIDATED`; unchanged, no full-body or physical control |

## Slot identity and collisions

Real report bytes 1..4 are exposed as key kind
`hid_0x15_header_bytes_1_4` with evidence `UNKNOWN`. They are not called a
physical sensor ID.

The registry:

- keeps a key-to-slot binding for its lifetime;
- honors unique saved key bindings from mapping YAML;
- rejects a key that presents conflicting source identity/physical-label data;
- rejects any actual slot reassignment;
- can freeze and reject a new key after startup;
- records a warning for a new key after the session is marked running;
- reports missing saved bindings and timestamp-based disappearance/staleness.

If two real physical sensors emit indistinguishable candidate keys, the current
wire evidence cannot separate them. They cannot satisfy 17 unique mapped slots,
so monitor/mapping/calibration/viewer startup fails closed. No proprietary ID or
packet-order slot is invented.

## Coordinate frames

- Raw FOHEART: `foheart_sensor_unknown`; axes/handedness remain incomplete.
- Configured sensor/body: a versioned proper `BasisTransform`.
- Upper FK: `human_torso`, right-handed `+X` forward, `+Y` left, `+Z` up.
- Full FK: `human_pelvis`, same right-handed axes, pelvis at `(0,0,0)`.
- G1 IK: separate Pinocchio root/base frame; never used by the human viewer.

Full-body coordinates are meters. Pelvis translation is fixed:

```text
ROOT_TRANSLATION = NOT_TRACKED_FIXED_ORIGIN
```

Only orientations and configured bone dimensions drive pose. Absolute walking
translation is not estimated from IMU orientation data.

## Full-body derivation boundary

All 17 role orientations are tagged `MEASURED` at the software input boundary.
The diagnostic skeleton derives lower/mid/upper spine and neck orientations by
conservative interpolation between pelvis/torso/head information. Joint
locations are orientation-driven FK from `CONFIGURED_OFFSET` dimensions.

`FULL_BODY_23_SEGMENTS` supplies a 23-name semantic vocabulary: 17 measured
semantic segments plus six derived spine/neck/toe semantics. This supports a
future compatibility adapter; it is not a reproduction of the proprietary
MotionVenus biomechanics solver.

## Fail-closed behavior

- Profile/mapping mismatch, missing/duplicate role or slot, invalid saved key,
  calibration role mismatch, and improper basis are startup errors.
- Default full mode requires 17/17 mapped, calibrated, fresh orientations.
- Runtime missing/stale roles invalidate SuitFrame; the viewer freezes the last
  valid skeleton and never extrapolates.
- Real one-sensor replay produces `INSUFFICIENT REAL SENSOR ROLES`, not a fake
  body.
- G1 safety remains isolated: invalid transforms, solver residuals, limits, and
  excessive deltas hold the last safe simulated target.
- Physical whole-body G1 control does not exist in this project.
