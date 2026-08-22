# Usage

All commands in this guide are simulation-only unless explicitly described as
monitoring or recording. They do not initialize TeleopIt's real-G1 bridge or
send DDS. Physical G1 operation has not been validated.

Run commands from:

```bash
cd /home/panu/Documents/fibo/project_humanoid/fh/foheart_linux
```

## Environments

The project uses separate interpreters because the pinned GMR solver and
TeleopIt inference stack have different dependency sets:

| Purpose | Interpreter/environment |
|---|---|
| Project unit tests | `.venv/bin/python` |
| GMR reference and direct MuJoCo | `/home/panu/miniconda3/envs/gmr/bin/python` |
| TeleopIt policy and free-base MuJoCo | `/home/panu/miniconda3/envs/teleopit/bin/python` |

The pinned GMR source is:

```text
/home/panu/Documents/fibo/project_humanoid/third_party/HumDex/GMR
commit bb1bbe40774794fceb2a7c579a3464a28e68c844
```

The TeleopIt checkout is:

```text
/home/panu/Documents/fibo/project_humanoid/Teleopit
commit f9263865c581802ad531854b8e547e2403a945f3
version 0.5.0
```

Do not merge these environments. The host exports a ROS `PYTHONPATH`, which can
load incompatible system packages. Use the following prefix for isolated GMR
or TeleopIt commands:

```bash
env -u PYTHONPATH PYTHONNOUSERSITE=1 \
  PYTHONPATH="$PWD/src:/home/panu/Documents/fibo/project_humanoid/third_party/HumDex/GMR"
```

The project test suite needs only:

```bash
env -u PYTHONPATH PYTHONNOUSERSITE=1 .venv/bin/python -m pytest -q
```

## Configure MotionVenus on Windows

1. Connect and power the FOHEART suit.
2. Open MotionVenus and select the correct actor/suit.
3. Enter the wearer dimensions required by MotionVenus.
4. Calibrate while holding the pose requested by MotionVenus.
5. Verify the Windows avatar before sending anything to Linux: left/right,
   arms, torso, hips, knees, feet, heading, and return to neutral.
6. Configure SDK/custom UDP forwarding as Binary protocol V4003.
7. Enable Position.
8. Enable Quaternion.
9. Select Global coordinates.
10. Set the destination to the Linux machine's IP and UDP port 5001.

Do not continue if the Windows avatar is wrong. Local rotations, omitted
positions, or a different protocol version do not satisfy the Linux contract.

## Monitor MotionVenus

The monitor is read-only and constructs no G1 object:

```bash
env -u PYTHONPATH PYTHONNOUSERSITE=1 \
  PYTHONPATH="$PWD/src" \
  /home/panu/miniconda3/envs/gmr/bin/python -s \
  -m foheart.tools.motionvenus_monitor \
  --bind 0.0.0.0 --port 5001 --format binary --duration 20
```

Expect advancing frame numbers, protocol 4003, 23 body bones, one sender, and
no persistent `STALE`, malformed, duplicate, or out-of-order state. Use
`--debug` for rejected-packet details.

To preserve packet boundaries for replay:

```bash
env -u PYTHONPATH PYTHONNOUSERSITE=1 \
  PYTHONPATH="$PWD/src" \
  /home/panu/miniconda3/envs/gmr/bin/python -s \
  -m foheart.tools.motionvenus_capture \
  --bind 0.0.0.0 --port 5001 --duration 20 \
  --output /absolute/path/session.mvudp
```

The output path must be new. MVUDP stores each datagram with its receive time
and sender; replay uses the same decoder as live traffic.

## Direct debug simulation

The direct path is useful for checking retargeting, joint limits, and the
processed reference without involving the tracking policy:

```bash
env -u PYTHONPATH PYTHONNOUSERSITE=1 \
  PYTHONPATH="$PWD/src:/home/panu/Documents/fibo/project_humanoid/third_party/HumDex/GMR" \
  /home/panu/miniconda3/envs/gmr/bin/python -s \
  -m foheart.tools.motionvenus_g1_reference \
  --source synthetic --mode direct-sim
```

Use `--source live --bind 0.0.0.0 --port 5001` for live input or
`--source replay --replay /absolute/path/session.mvudp` for replay. This path
uses bounded direct MuJoCo dynamics and is not the preferred balance/policy
test.

For passive kinematic inspection, use reference mode instead:

```bash
env -u PYTHONPATH PYTHONNOUSERSITE=1 \
  PYTHONPATH="$PWD/src:/home/panu/Documents/fibo/project_humanoid/third_party/HumDex/GMR" \
  /home/panu/miniconda3/envs/gmr/bin/python -s \
  -m foheart.tools.motionvenus_g1_reference \
  --source synthetic --mode reference --viewer
```

Reference mode applies qpos and forward kinematics; it does not prove dynamic
tracking or balance.

## Policy simulation: main mode

First verify the complete offline path:

```bash
env -u PYTHONPATH PYTHONNOUSERSITE=1 \
  PYTHONPATH="$PWD/src:/home/panu/Documents/fibo/project_humanoid/third_party/HumDex/GMR" \
  /home/panu/miniconda3/envs/teleopit/bin/python -s \
  -m foheart.tools.motionvenus_g1_reference \
  --source synthetic --mode policy-sim --policy-steps-per-reference 25
```

Then run the live software path:

```bash
env -u PYTHONPATH PYTHONNOUSERSITE=1 \
  PYTHONPATH="$PWD/src:/home/panu/Documents/fibo/project_humanoid/third_party/HumDex/GMR" \
  /home/panu/miniconda3/envs/teleopit/bin/python -s \
  -m foheart.tools.motionvenus_g1_reference \
  --source live --mode policy-sim --bind 0.0.0.0 --port 5001
```

The flow is MotionVenus to actual pinned GMR, `G1ReferenceProcessor`,
`FoheartTeleopitAdapter`, TeleopIt's `VelCmdObservationBuilder`, released
`track_g1.onnx`, and TeleopIt's free-base MuJoCo runtime.

Replay uses the same path:

```bash
env -u PYTHONPATH PYTHONNOUSERSITE=1 \
  PYTHONPATH="$PWD/src:/home/panu/Documents/fibo/project_humanoid/third_party/HumDex/GMR" \
  /home/panu/miniconda3/envs/teleopit/bin/python -s \
  -m foheart.tools.motionvenus_g1_reference \
  --source replay --replay /absolute/path/session.mvudp --mode policy-sim
```

Stop with Ctrl-C. The receiver and simulation objects close with the process;
this command creates no physical-robot backend.

## First live test

Keep both feet planted and progress slowly:

1. standing;
2. small left-arm motion;
3. small right-arm motion;
4. both arms;
5. torso yaw;
6. waist motion;
7. shallow squat;
8. weight shift;
9. small left-leg raise;
10. small right-leg raise.

Do not begin with walking. Stop if the Windows avatar is wrong, the Linux
stream becomes stale, joints move in the wrong direction, the simulator falls,
or any value becomes non-finite. Current automated evidence covers bounded
upright poses, not robust walking.

## Record processed G1 motion

Record only after the Windows avatar and Linux direction/heading comparison is
acceptable. Recording branches immediately after `G1ReferenceProcessor`, so
it stores the intended safe reference before policy or servo tracking error:

```bash
env -u PYTHONPATH PYTHONNOUSERSITE=1 \
  PYTHONPATH="$PWD/src:/home/panu/Documents/fibo/project_humanoid/third_party/HumDex/GMR" \
  /home/panu/miniconda3/envs/gmr/bin/python -s \
  -m foheart.tools.motionvenus_g1_reference \
  --source live --mode reference --bind 0.0.0.0 --port 5001 \
  --fps 50 --viewer --record /absolute/path/to/motions/suit_001.pkl
```

The path must not already exist. Ctrl-C saves when at least two accepted frames
were recorded. The source cadence should match `--fps`; the recorder does not
invent interpolated frames.

The canonical PKL contains exactly:

| Key | Contract |
|---|---|
| `fps` | positive source cadence |
| `root_pos` | `(T,3)` metres |
| `root_rot` | `(T,4)` normalized XYZW |
| `dof_pos` | `(T,29)` in the G1 joint order |
| `local_body_pos` | `(T,38,3)` pinned-model root-neutral FK |
| `link_body_list` | exact 38-body pinned-model order |

Runtime references use WXYZ. The recorder intentionally stores root rotation
as XYZW because that is the dataset convention.

## TeleopIt dataset path

No project-specific converter is required. TeleopIt's current dataset builder
accepts these PKL files directly with a source entry of `type: pkl`. It owns
the XYZW-to-WXYZ conversion, joint-velocity derivation, and model FK needed by
training.

A future dataset spec follows TeleopIt's upstream schema:

```yaml
name: foheart
target_fps: 50
preprocess:
  normalize_root_xy: true
  ground_align: first_frame_foot
sources:
  - name: foheart_recordings
    type: pkl
    input: /absolute/path/to/motions
```

TeleopIt's `train_mimic/scripts/data/build_dataset.py` creates the minimal HDF5
dataset; `precompute_dataset.py` creates the training-ready dataset. These are
future entry points, not current quick-start commands: training dependencies
and datasets are not installed or validated in the inference environment.

## Training status

The selected upstream task is `General-Tracking-G1`. The intended future path
is:

```text
recorded PKL -> build dataset -> precompute -> General-Tracking-G1
  -> PT checkpoint -> TeleopIt ONNX export -> same policy runtime
```

Training has not run, no new checkpoint exists, and no new ONNX policy has
been evaluated. Install and validate TeleopIt's training stack in a separate,
explicitly authorized phase before using the upstream training instructions.

## Real G1 boundary

The future architecture reuses the same frontend, adapter, observation builder,
policy, default pose, action scaling, and 50 Hz inference path, then selects
TeleopIt's G1 backend instead of MuJoCo. That backend was source-audited only.

Physical G1 is **not validated**. Do not treat policy-simulation success as
authorization to initialize the bridge, send DDS, enable motors, change robot
mode, or perform motion. There is intentionally no physical-G1 quick-start
command here.

## Troubleshooting

### No UDP packets

- Confirm MotionVenus forwarding is enabled and targets the Linux IP, not
  `127.0.0.1` on Windows.
- Confirm both programs use port 5001.
- Check the Windows firewall, Linux firewall, subnet, and Wi-Fi client
  isolation.
- Check whether another process owns the port with `ss -lunp`.

### Packets are rejected

- Select Binary V4003, Position ON, Quaternion ON, and Global coordinates.
- Recalibrate MotionVenus and verify the Windows avatar.
- Expect 23 body bones. Preserve an MVUDP capture before changing parser code.

### Import or pytest loads ROS packages

Use `env -u PYTHONPATH PYTHONNOUSERSITE=1`. Add back only `$PWD/src` and the
pinned GMR source path as shown above.

### GMR or TeleopIt is missing

Check the interpreter first. GMR commands use the `gmr` environment; policy
commands use `teleopit`. Do not install either stack into the project `.venv`.

### TeleopIt assets are missing

The TeleopIt checkout must contain:

```text
assets/robots/unitree_g1/g1_29dof.xml
ckpt/track_g1.onnx
ckpt/track_g1.pt
data/sample_bvh/aiming1_subject1.bvh
teleopit/retargeting/gmr/assets/
```

Use TeleopIt's upstream asset setup in its isolated environment. Do not
substitute an unmatched model or policy.

### Clean shutdown

Press Ctrl-C once and allow the process to close the receiver and simulator.
If a recording has fewer than two valid frames it is rejected rather than
written as a misleading motion clip.
