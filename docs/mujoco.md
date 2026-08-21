# G1 MuJoCo validation

The local model is:

```text
/home/panu/Documents/fibo/project_humanoid/unitree_mujoco/
  unitree_robots/g1/scene_29dof.xml
```

`G1MuJoCoBridge` loads that actual model directly in-process. It does not
start the repository's DDS bridge, does not import the Unitree SDK controller,
and exposes no real-robot mode. The 29 actuator names are checked exactly; the
14 arm actuators must occupy indices 15..28 in the same order as the existing
IK.

For deterministic upper-body validation, the floating base is pinned and a
bounded PD loop drives only the 14 arm targets while the other 15 joints retain
their initial targets. This validates arm target integration, not standing,
locomotion, contacts, or a physical controller.

## Reproduce

The dependencies already exist in the local `tv` conda environment:

```bash
cd /home/panu/Documents/fibo/project_humanoid/fh/foheart_linux
env PYTHONPATH="$PWD/src" MPLCONFIGDIR=/tmp/foheart-mpl \
  /home/panu/miniconda3/envs/tv/bin/python \
  -m foheart.tools.g1_sim_replay \
  --config config/default.yaml \
  --capture-check samples/motion_baseline.bin
```

The single command loads the existing IK and MuJoCo model; no separate DDS
process is used. Add an unused output path to retain metrics:

```text
--output samples/new_g1_sim_validation.json
```

Existing output files are never overwritten.

The joint viewer is deliberately separate and never starts this command or
imports MuJoCo. Its `JointFrame` remains in the human torso frame.

## Recorded result

`samples/g1_sim_validation.json` records:

- seven synthetic poses and 7/7 valid IK results;
- finite MuJoCo state for every pose;
- maximum final arm error `0.0038147537 rad`;
- maximum non-arm drift `0.0019654616 rad`;
- no workspace clamp or joint-rate limit in this run;
- status `SIM_VALIDATED`.

The real baseline capture check decoded 200/200 `0x15` frames, then stopped
at the correct evidence boundary: one slot cannot populate seven body roles.
Accordingly, synthetic seven-role motion validates the complete software/sim
path; real multi-sensor body retargeting remains pending.

The command was rerun after the source/slot/viewer implementation and again
reported `SIM_VALIDATED (7/7 IK targets)`, maximum final arm error
`0.003815 rad`, maximum non-arm drift `0.00197 rad`, and finite state for all
seven poses. This was an offline regression only.
