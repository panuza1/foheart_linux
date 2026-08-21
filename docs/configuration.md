# Configuration

`config/default.yaml` mirrors the built-in validated model. Precedence is:

```text
explicit CLI option > selected YAML > built-in safe default
```

Unknown sections/keys and invalid values are rejected. Configuration cannot
enable unvalidated USB output or a physical G1.

## USB and protocol

### `usb`

| Key | Default | Accepted values |
|---|---|---|
| `mode` | `auto` | `auto`, `bulk`, `hid` |
| `pid` | `auto` | `auto`, `0x5751`, `0x5851` |
| `interface` | `auto` | `auto` or integer |
| `in_endpoint` | `auto` | `auto` or integer |
| `out_endpoint` | `auto` | `auto` or integer |
| `timeout_ms` | `1000` | positive integer |
| `read_size` | `auto` | `auto` or positive integer |

Discovery is descriptor-driven and refuses ambiguity. The real C1 HID profile
is `1483:5851`, configuration 1, interface 0, interrupt OUT `0x01`, interrupt
IN `0x81`, 64-byte packets.

### `protocol`

| Key | Default | Accepted values |
|---|---|---|
| `outer_frame` | `auto` | `auto`, `fixed_0x13`, `raw` |
| `sensor_id_mode` | `auto` | `auto`, `loop_index`, `decoded_index`, `unknown` |

The real HID `0x15` decoder exposes bytes 1..4 only as an opaque candidate key.
The bulk `0x13` path remains `STATIC_ONLY`.

### `stream`

```yaml
stream:
  mode: read_only
  stale_after_ms: 100.0
```

`stale_after_ms` must be positive and applies to every role in the selected
profile. It is an invalidation threshold, not an extrapolation horizon.
`experimental` is recognized only to produce a refusal in legacy read tools.
Dedicated poll code can send only the immutable `0x70 + 63 zero` packet.

### `monitor`

`show_raw`, `show_euler`, `show_quaternion`, and `show_imu` are booleans. Real
Euler remains absent/`STATIC_ONLY`; display does not invent it.

## Sensor basis

```yaml
frames:
  version: 1
  sensor_to_body_matrix:
    - [1.0, 0.0, 0.0]
    - [0.0, 1.0, 0.0]
    - [0.0, 0.0, 1.0]
  status: CONFIGURED
```

The matrix must be finite, orthonormal, and determinant `+1`. Reflections are
rejected. Status accepts `CONFIGURED`, `MANUAL_DERIVED`, or `PARTIAL`. Identity
is a safe software default, not validated FOHEART physical axes.

## Profile and inline mapping

Viewer profile lives at:

```yaml
viewer:
  mode: upper
```

Accepted values are `upper` and `full`; `--mode` overrides it. Upper requires
seven roles and full requires all 17. `body_mapping` contains nullable keys for
the full superset so either mode can be configured inline. Non-null slots must
be unique. A non-null full-only role is rejected while mode is `upper`.

For repeatable hardware sessions, prefer a versioned mapping file selected by
`--body-mapping`; it embeds `profile: upper|full`, status `CONFIGURED`, and
optional saved transport keys. See `body_mapping.md`.

## Neutral calibration

```yaml
calibration:
  file: null
  duration_s: 2.0
  minimum_samples: 20
  maximum_angular_deviation_deg: 3.0
  maximum_gyro_magnitude: 5.0
```

All limits are positive. `calibrate_live --mode full` applies them independently
to 17 roles. The viewer requires an exact calibration role set for its profile.
Generated files remain `live_validated: false` until physical observation.

## Upper-body dimensions

```yaml
skeleton:
  shoulder_width_m: 0.38
  left_upper_arm_m: 0.30
  left_forearm_m: 0.26
  left_hand_m: 0.10
  right_upper_arm_m: 0.30
  right_forearm_m: 0.26
  right_hand_m: 0.10
```

All values are positive meters. Existing upper mode and G1 simulation consume
this section unchanged.

## Full-body dimensions

```yaml
full_body:
  shoulder_width_m: 0.38
  left_upper_arm_m: 0.30
  left_forearm_m: 0.26
  left_hand_m: 0.10
  right_upper_arm_m: 0.30
  right_forearm_m: 0.26
  right_hand_m: 0.10
  torso_length_m: 0.50
  neck_length_m: 0.10
  head_length_m: 0.18
  hip_width_m: 0.30
  left_thigh_m: 0.42
  left_lower_leg_m: 0.43
  left_foot_m: 0.25
  right_thigh_m: 0.42
  right_lower_leg_m: 0.43
  right_foot_m: 0.25
```

These are deterministic synthetic defaults, not the user's measurements.
Measure the wearer before physical validation. Left and right arms/legs may be
configured independently. Every value must be positive and finite.

## Viewer

```yaml
viewer:
  mode: upper
  fps: 30.0
  camera: perspective
  show_segment_axes: false
  show_sensors: false
```

`camera` accepts `perspective`, `front`, or `side`. CLI overrides are `--mode`,
`--fps`, `--camera`, `--show-segment-axes`, and `--show-sensors` (with their
`--no-*` forms). `--duration` and `--headless` provide bounded Agg validation.

Examples:

```bash
python -m foheart.tools.live_joint_viewer \
  --config config/default.yaml --mode full --synthetic

python -m foheart.tools.live_sensor_monitor \
  --config config/default.yaml --mode full --synthetic --samples 34
```

## Retargeting, filtering, and G1 simulation

The existing sections remain upper-body-only:

```yaml
retarget:
  human_reach_m: 0.56
  g1_reach_m: 0.321
  max_robot_reach_m: 0.43
  workspace_radius_m: 1.0

filter:
  position_alpha: 0.2
  orientation_alpha: 0.2
  max_translation_rate_m_s: 0.8
  max_angular_rate_deg_s: 180.0

g1:
  mode: mujoco
  xr_root: null
  mujoco_model: null
  max_joint_delta_rad: 0.35
  steps_per_pose: 250
```

Alphas are `(0,1]`; rates, limits, and steps are positive. `g1.mode` accepts
only `mujoco`; values containing real/physical intent are rejected. Full-body
configuration is exposed only as robot-independent joints and does not enable
whole-body G1 control.

## Evidence boundaries

| Configuration area | Status |
|---|---|
| C1 descriptor and immutable poll | `REAL_CAPTURE_VALIDATED` |
| Physical signed basis | default `CONFIGURED`; complete mapping `UNKNOWN` |
| Profile/mapping/calibration validation | `SOFTWARE_TESTED` for 7 and 17 roles |
| Full dimensions | `CONFIGURED` examples |
| Full-body FK/viewer | `SOFTWARE_VALIDATED` with synthetic sensors |
| Real 17-role configuration | `NOT ATTEMPTED` |
| G1 IK/MuJoCo | existing `SIM_VALIDATED` upper-body path |
