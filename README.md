# foheart-linux

Linux-native FOHEART C1 capture, HID `0x15` decoding, and a matplotlib
diagnostic skeleton viewer. Synthetic, replay, and live C1 sources share one
path:

```text
SensorSample -> logical slot -> body mapping -> basis ->
neutral calibration -> SuitFrame -> FK -> JointFrame
```

`--mode upper` (default) uses 7 torso/arm roles. `--mode full` uses 17 measured
roles (head, shoulders, torso, pelvis, arms, hands, thighs, lower legs, feet)
plus derived spine/neck segments. The compact HID `0x15` quaternion/IMU decoder
is real-capture validated. Simultaneous 17-sensor hardware behavior is not.

The full-body root stays at the origin. FOHEART orientation data does not
provide validated absolute human translation, so walking displacement is not
invented.

| Area | Status |
|---|---|
| C1 HID `0x70` polling and `0x15` decoding | `REAL_CAPTURE_VALIDATED` |
| Physical signed sensor axes/handedness | `PARTIAL` / `UNKNOWN` |
| Seven-role upper-body software/viewer | `SOFTWARE_VALIDATED` |
| Seventeen-role mapping and calibration workflows | `SOFTWARE_TESTED` |
| Full-body SuitFrame, FK, JointFrame, and viewer | `SOFTWARE_VALIDATED` |
| Synthetic 17-sensor end to end | `PASS` |
| Real one-sensor replay | decoder/orientation path only |
| Real 17 sensors, mapping, calibration, viewer | `NOT ATTEMPTED` |
| Existing upper-body G1 IK + MuJoCo branch | `SIM_VALIDATED` |
| Full-body G1 control / physical G1 | not implemented / not attempted |

## Ubuntu setup

```bash
sudo apt install libusb-1.0-0 libusb-1.0-0-dev
python -m venv .venv
source .venv/bin/activate
pip install -e '.[dev]'
pytest
```

The viewer needs a display, or `MPLCONFIGDIR` plus `--headless` for bounded
Agg checks.

## Run now — no hardware

Full body:

```bash
python -m foheart.tools.live_joint_viewer --mode full --synthetic
```

Upper body (default):

```bash
python -m foheart.tools.live_joint_viewer --mode upper --synthetic
python -m foheart.tools.live_joint_viewer --synthetic
```

Bounded headless check:

```bash
MPLCONFIGDIR=/tmp/foheart-mpl \
python -m foheart.tools.live_joint_viewer \
  --mode full --synthetic --headless --duration 5
```

Other offline 17-role exercises:

```bash
python -m foheart.tools.live_sensor_monitor \
  --mode full --synthetic --samples 34 --debug-transport

python -m foheart.tools.map_body_sensors \
  --mode full --synthetic --output /tmp/foheart-full-map.yaml

python -m foheart.tools.calibrate_live \
  --mode full --synthetic --no-prompt \
  --output /tmp/foheart-full-neutral.yaml
```

Mapping and calibration outputs use exclusive creation and refuse overwrite.
Motion-assisted mapping proposes a slot but still requires explicit confirmation.

## Tools

| Command | Purpose |
|---|---|
| `foheart.tools.discover` | USB discovery only; no application write |
| `foheart.tools.c1_poll_once` | one guarded `0x70` OUT and at most one IN |
| `foheart.tools.c1_capture` | bounded real poll capture (max 200 / 100 ms / 30 s) |
| `foheart.tools.guided_motion_capture` | four fixed 200-poll controlled-motion phases |
| `foheart.tools.replay` | replay a saved capture; no USB |
| `foheart.tools.monitor` | decode/monitor a capture or live source |
| `foheart.tools.motion_analyze` | offline quaternion/IMU motion review |
| `foheart.tools.live_sensor_monitor` | slot/transport diagnostics |
| `foheart.tools.map_body_sensors` | assign logical slots to body roles |
| `foheart.tools.calibrate` | offline explicit/synthetic neutral profile |
| `foheart.tools.calibrate_live` | capture standing-neutral calibration |
| `foheart.tools.live_joint_viewer` | 7- or 17-role matplotlib skeleton |
| `foheart.tools.g1_sim_replay` | upper-body G1 IK + MuJoCo regression |

Configuration precedence is `CLI > selected YAML > built-in safe defaults`.
Full-body lengths in `config/default.yaml` are synthetic examples, not measured
wearer dimensions.

## Viewer diagnostics

```bash
python -m foheart.tools.live_joint_viewer \
  --mode full --synthetic \
  --show-segment-axes --show-sensors --camera perspective
```

`--show-sensors` distinguishes raw sensor axes (dotted), frame-converted axes
(dashed), and calibrated segment axes (solid). X/Y/Z use red/green/blue.
Cameras are `front`, `side`, and `perspective`. The status panel reports all 17
ages, mapping/calibration/basis state, stale/missing roles, and fixed root
translation. The table shows pelvis/head plus bilateral arm and leg joints in
meters.

Missing or stale required roles freeze the last valid skeleton. The viewer
never extrapolates or fabricates missing sensors.

## Replay

```bash
MPLCONFIGDIR=/tmp/foheart-mpl \
python -m foheart.tools.live_joint_viewer \
  --mode full --replay samples/motion_baseline.bin --headless
```

Existing real captures contain one candidate key. Full mode therefore reports
`INSUFFICIENT REAL SENSOR ROLES`, `detected: 1`, `required: 17`, and generates
no full skeleton.

```bash
python -m foheart.tools.replay samples/c1_real_poll_capture.bin
python -m foheart.tools.monitor \
  --capture samples/c1_real_poll_capture.bin --count 10
python -m foheart.tools.motion_analyze \
  samples/motion_table_yaw_cw.bin \
  --baseline samples/motion_baseline.bin
```

## Future 17-sensor hardware workflow

Run this only after the C1 and all 17 sensors are connected. Stop if fewer than
17 stable slots appear, any collision/reassignment warning appears, or
calibration rejects motion.

```bash
# 1. Verify exactly 17 stable candidate transport keys / logical slots.
python -m foheart.tools.live_sensor_monitor --mode full --debug-transport

# 2. Assign all 17 physical placements manually.
python -m foheart.tools.map_body_sensors \
  --mode full \
  --output config/my_full_body_mapping.yaml

# 3. Capture the documented standing neutral pose.
python -m foheart.tools.calibrate_live \
  --mode full \
  --body-mapping config/my_full_body_mapping.yaml \
  --output config/my_full_body_neutral.yaml

# 4. Start the real full-body viewer.
python -m foheart.tools.live_joint_viewer \
  --mode full \
  --body-mapping config/my_full_body_mapping.yaml \
  --calibration config/my_full_body_neutral.yaml
```

Neutral pose: head forward, torso upright, pelvis neutral, shoulders relaxed,
arms straight down with palms inward, legs straight, feet parallel and forward.
Synthetic T-pose is a motion test, not a second calibration convention.

Header bytes 1..4 are only an opaque candidate `TRANSPORT_KEY`; they are not a
proven FOHEART physical ID.

Approximate wearing locations (head; both shoulders; upper back/torso;
pelvis/waist; outside upper arms; outside forearms near but not on the wrist;
backs of hands; outside mid-thighs; lower legs below the knee; both feet/shoes)
are `MANUAL_DERIVED`, not physically validated here. See
[docs/full_body.md](docs/full_body.md).

The seven-role upper-body sequence is the same without `--mode full` and with
torso plus left/right upper arm, forearm, and hand.

## USB safety

The only implemented C1 application output is the immutable 64-byte payload
`0x70 + 63 zero bytes`, restricted to real descriptor profile `1483:5851`,
interface 0, interrupt OUT `0x01`, interrupt IN `0x81`, 64-byte packets. No
configuration can enable `0x73`, RTTRANS, feature/control reports, firmware,
reboot, reset, or arbitrary HID output.

Hardware-facing commands (`discover`, `c1_poll_once`, `c1_capture`,
`guided_motion_capture`, live source mode) must be used only with connected
hardware. They were not run while implementing the 17-role software.

```bash
python -m foheart.tools.discover
python -m foheart.tools.c1_poll_once
python -m foheart.tools.c1_capture \
  --polls 200 --timeout-ms 100 \
  --output samples/c1_real_poll_capture.bin --hex
```

The default sensor basis is identity with status `CONFIGURED`; it is not a
physical FOHEART truth. Controlled motion proves TOP-up gravity on +AZ and only
partial QZ/QY candidates. Every configured basis must be finite, orthonormal,
right-handed, and determinant +1.

## G1 simulation regression

The pre-existing robot branch remains upper-body-only and simulation-only. It
reuses local `xr_teleoperate.G1_29_ArmIK` and the G1 MuJoCo model, starts no
DDS, and contains no physical robot mode.

```bash
env PYTHONPATH="$PWD/src" MPLCONFIGDIR=/tmp/foheart-mpl \
  python -m foheart.tools.g1_sim_replay \
  --config config/default.yaml \
  --capture-check samples/motion_baseline.bin
```

This command needs the extra Pinocchio/MuJoCo environment used for that
regression, not only the package `.venv`. Full-body retargeting/control was not
implemented.

## References

- [architecture](docs/architecture.md)
- [configuration](docs/configuration.md)
- [body mapping](docs/body_mapping.md)
- [calibration](docs/calibration.md)
- [skeleton/FK](docs/skeleton.md)
- [full-body model and physical procedure](docs/full_body.md)
- [viewer](docs/live_joint_viewer.md)
- [user manual](docs/USER_MANUAL.md)
- [validation matrix](docs/validation_matrix.md)
- [protocol evidence](docs/protocol_status.md)
- [G1 integration](docs/g1_integration.md)
- [G1 MuJoCo validation](docs/mujoco.md)

No software-only test may promote real 17-sensor behavior to PASS. The later
hardware session requires human observation of correct head, left/right arm,
torso/pelvis, hip, knee, and foot chains with stable slots and invariant bone
lengths.
