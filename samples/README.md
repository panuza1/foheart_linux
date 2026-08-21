# Samples

Store raw captures here. Do not commit captures containing fabricated values as
if they came from hardware. Version 1 files begin with `FHC1RAW\x01`; each record
contains timestamp, endpoint, payload length, and the exact transfer payload.

`c1_real_poll_fixture.hex` is a sanitized, hex-encoded `FHC1POL\x01` recording
containing three real 64-byte IN boundaries from the 2026-08-21 C1 capture.
Host timestamps were replaced with deterministic values; report bytes were not
changed. The original generated `.bin` remains ignored and unmodified.

Controlled-motion history and latest artifacts:

- `motion_baseline_failed_backend.bin` predates the authoritative set. Despite
  its name, it contains 200 valid `0x15` reports, not an eight-byte failure
  marker. It belongs to an older incomplete session and is excluded by mtime and
  session completeness.
- `motion_baseline.bin`, `motion_table_yaw_cw.bin`,
  `motion_forward_tilt.bin`, and `motion_right_roll.bin` are the newest real
  captures. Each is 20,408 bytes with 200 successful 64-byte `0x15` reports,
  zero timeout, and zero error.
- `motion_validation_summary.json` contains the raw analyzer results plus the
  latest conservative offline review. The experiment status is PARTIAL because
  yaw motion was missed and tilt/roll lack stationary bookends.

The raw `.bin` files are canonical and must not be edited. No physical sensor
identity or body placement is encoded by the filenames; decoded identity remains
capture-local `slot_0`.

`g1_sim_validation.json` is a derived, reproducible simulation result rather
than a hardware capture. It records the one-sensor baseline decode boundary and
the complete configured seven-role synthetic path through calibration, FK, the
existing G1 IK, and the actual local MuJoCo model. Its status is
`SIM_VALIDATED`; it does not claim real multi-sensor or physical G1 activity.
