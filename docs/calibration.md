# Neutral-pose calibration

Upper and full modes reuse the same `CalibrationProfile`. Calibration never
performs robot I/O and never overwrites raw sensor quaternions.

For WXYZ values:

```text
q_calibrated = inverse(q_neutral) * q_current
```

The captured neutral orientation therefore becomes identity. Frame conversion
is applied before estimation; raw, converted, and calibrated values remain
separate.

## One neutral convention

The software uses a standing, arms-down sensor neutral pose for both profiles:

```text
head: forward
torso: upright
pelvis: neutral and level
shoulders: relaxed/neutral
arms: straight down at sides
palms: inward
legs: straight
feet: parallel and pointing forward
```

Feet/legs are ignored only in upper mode. The synthetic viewer's T-pose is a
motion test, not a second calibration convention. Do not capture a T-pose
profile and call it the standing neutral profile.

## Full 17-role workflow

Software-only:

```bash
python -m foheart.tools.calibrate_live \
  --mode full --synthetic --no-prompt \
  --output /tmp/foheart-full-neutral.yaml
```

Future real sensors:

```bash
python -m foheart.tools.calibrate_live \
  --mode full \
  --body-mapping config/my_full_body_mapping.yaml \
  --output config/my_full_body_neutral.yaml
```

The mapping profile must be `full`, all 17 unique mapped slots must produce
samples, and the output must not exist. Current real one-sensor replay fails
with per-role sample shortages; it cannot create a full profile.

Upper mode remains available:

```bash
python -m foheart.tools.calibrate_live \
  --mode upper \
  --body-mapping config/my_upper_mapping.yaml \
  --output config/my_upper_neutral.yaml
```

## Quaternion estimator

Raw component averaging is not used. Per role, the implementation:

1. validates and normalizes each derived WXYZ sample;
2. applies temporal q/-q sign continuity;
3. takes a mean within that continuous quaternion hemisphere;
4. normalizes the mean;
5. measures shortest angular deviation from the mean.

The serialized algorithm name is:

```text
temporal_sign_continuity_normalized_wxyz_mean
```

This remains the same estimator for 7 or 17 roles.

## Quality and rejection

For every role, the file stores:

```text
sample_count
raw quaternion norm minimum/maximum
RMS orientation spread in degrees
maximum angular deviation in degrees
mean and maximum gyro magnitude
last timestamp_ns
acceptable
```

Defaults require at least 20 samples, maximum 3° angular deviation, and maximum
gyro magnitude 5. A sample shortage or excessive movement rejects the entire
guided calibration; partial role profiles are not silently saved.

Every generated workflow file begins with the evidence boundary:

```yaml
version: 1
status: SOFTWARE_TESTED
live_validated: false
algorithm: temporal_sign_continuity_normalized_wxyz_mean
```

The `sensors` mapping then contains one neutral WXYZ and quality record for each
required role. Full viewer startup calls exact role matching, so a seven-role
profile cannot start full mode and a 17-role profile cannot be mistaken for
upper mode.

## Offline explicit-value utility

`foheart.tools.calibrate` still supports explicit offline role-to-WXYZ input
and `--synthetic-upper-body`. The profile-aware future hardware workflow uses
`calibrate_live --mode ...`; no second calibration class exists.

## Evidence boundary

Quaternion math, q/-q continuity, quality rejection, versioned serialization,
role matching, upper sampling, and full 17-role synthetic sampling are
`SOFTWARE_TESTED`. Real full-body neutral capture, placement, sensor basis, and
visual return-to-neutral behavior are `NOT VALIDATED`. A created YAML file does
not promote physical evidence.
