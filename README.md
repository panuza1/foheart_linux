# FOHEART MotionVenus to Unitree G1

This project turns a solved FOHEART full-body motion stream into a safe Unitree
G1 reference. It supports two related goals:

1. Run live or recorded FOHEART motion through the released TeleopIt tracking
   policy in free-base MuJoCo.
2. Record processed G1 references as PKL motion clips for a future TeleopIt
   `General-Tracking-G1` training run.

Training, a newly trained policy, live-suit validation, and physical G1
validation are not complete. No physical G1 command is part of the quick start.

## Architecture

```text
FOHEART suit
  -> MotionVenus on Windows
  -> UDP V4003
  -> MotionVenusReceiver / MotionVenusStreamDecoder
  -> HumanSkeletonFrame
  -> HeadingCalibration / MotionVenusGMRAdapter
  -> pinned GeneralMotionRetargeting
  -> G1ReferenceProcessor
  -> ProcessedG1Reference (root XYZ + root WXYZ + 29 joints)
       |-> FoheartTeleopitAdapter -> TeleopIt policy -> free-base MuJoCo
       `-> MotionRecorder -> PKL -> future TeleopIt training
```

The existing direct MuJoCo path remains available for retargeting and safety
debugging. TeleopIt is the main policy-tracking and balance-simulation path.
See [Architecture](docs/ARCHITECTURE.md) for ownership and interface details.

## Repository structure

```text
config/                    Runtime defaults and retarget configuration
docs/                      Usage and architecture documentation
samples/                   Preserved capture fixtures and validation artifacts
src/foheart/motionvenus/   V4003 transport, parser, skeleton, replay, and GMR map
src/foheart/whole_body/    Pinned-GMR wrapper and processed-reference safety
src/foheart/integrations/  TeleopIt, MuJoCo, recorder, and compatibility boundaries
src/foheart/tools/         Monitor, capture, simulation, recording, and dataset CLIs
tests/                     Offline, GMR, recorder, and TeleopIt integration tests
```

The older Linux-native C1 USB/parser code remains under `src/foheart/usb`,
`protocol`, and `mocap` for diagnostics. It is not the preferred whole-body
runtime.

## Prerequisites

- Linux with Python 3.10 and the project `.venv` for ordinary tests.
- FOHEART plus MotionVenus on Windows for live input.
- Pinned GMR checkout at
  `/home/panu/Documents/fibo/project_humanoid/third_party/HumDex/GMR`, commit
  `bb1bbe40774794fceb2a7c579a3464a28e68c844`.
- Dedicated GMR environment at `/home/panu/miniconda3/envs/gmr`.
- TeleopIt checkout at `/home/panu/Documents/fibo/project_humanoid/Teleopit`,
  commit `f9263865c581802ad531854b8e547e2403a945f3`.
- Dedicated TeleopIt environment at `/home/panu/miniconda3/envs/teleopit`, with
  the released G1 model, `track_g1.onnx`, GMR assets, and sample BVH installed.

GMR and TeleopIt intentionally use separate environments. The host exports a
ROS `PYTHONPATH`; isolated commands remove it before adding only the project
and pinned GMR source paths.

## Quick start

From the project root, verify the existing project environment:

```bash
env -u PYTHONPATH PYTHONNOUSERSITE=1 .venv/bin/python -m pytest -q
```

Run the complete synthetic reference path through the released policy and
free-base MuJoCo:

```bash
env -u PYTHONPATH PYTHONNOUSERSITE=1 \
  PYTHONPATH="$PWD/src:/home/panu/Documents/fibo/project_humanoid/third_party/HumDex/GMR" \
  /home/panu/miniconda3/envs/teleopit/bin/python -s \
  -m foheart.tools.motionvenus_g1_reference \
  --source synthetic --mode policy-sim --policy-steps-per-reference 25
```

For live input, first configure MotionVenus to send Binary V4003, position and
quaternion enabled, global coordinates, to the Linux host on UDP port 5001.
Then run:

```bash
env -u PYTHONPATH PYTHONNOUSERSITE=1 \
  PYTHONPATH="$PWD/src:/home/panu/Documents/fibo/project_humanoid/third_party/HumDex/GMR" \
  /home/panu/miniconda3/envs/teleopit/bin/python -s \
  -m foheart.tools.motionvenus_g1_reference \
  --source live --mode policy-sim --bind 0.0.0.0 --port 5001
```

Start with standing and small isolated motions, not walking. The exact Windows
setup, monitor, replay, direct debug simulation, recording, and troubleshooting
procedures are in [Usage](docs/USAGE.md).

## Current validation status

| Capability | Status |
|---|---|
| Actual pinned GMR and ten-pose reference path | READY / PASS |
| G1ReferenceProcessor | READY |
| Direct/reference MuJoCo | READY |
| MVUDP capture and replay | READY |
| MotionRecorder and six-key PKL | READY |
| TeleopIt isolated inference environment/assets | READY |
| TeleopIt upstream free-base sample | PASS |
| FoheartTeleopitAdapter | READY |
| Synthetic actual-GMR to TeleopIt policy simulation | PASS |
| MVUDP actual-GMR to TeleopIt policy simulation | PASS |
| Live FOHEART policy simulation | SOFTWARE READY; manual suit test pending |
| TeleopIt PKL dataset compatibility | READY; no converter required |
| Training / new policy | NOT RUN / NOT TRAINED |
| Physical G1 | NOT VALIDATED |

The simulation evidence is bounded upright pose tracking. It is not a claim of
robust walking or physical-robot safety.

## Documentation

- [Usage](docs/USAGE.md): setup, commands, recording, datasets, and troubleshooting.
- [Architecture](docs/ARCHITECTURE.md): component ownership, data contracts, rates,
  and safety boundaries.
