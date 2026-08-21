# Unitree G1 integration

This integration is simulation-only. It contains no network address, Unitree
DDS initialization, or physical robot command path.

The Linux joint viewer stops at `JointFrame` in `human_torso`, meters. It does
not convert display coordinates to G1 coordinates and never launches MuJoCo.
The optional downstream branch still begins from the existing
`UpperBodyTargets`; viewer work did not alter the adapter or IK.

## Existing stack reused

The discovered local implementation is:

```text
/home/panu/Documents/fibo/project_humanoid/xr_teleoperate/
  teleop/robot_control/robot_arm_ik.py:G1_29_ArmIK
```

The adapter loads its existing read-only model cache and calls:

```python
solve_ik(left_wrist_4x4, right_wrist_4x4, current_q14, current_dq14)
```

Inputs are NumPy homogeneous `4x4` transforms in meters in the Pinocchio
model's root/base frame, targeting `L_ee` and `R_ee` (each 0.05 m beyond
the wrist-yaw joint). Output is 14 arm positions plus 14 feed-forward torque
values.

The exact checked joint order is:

```text
left_shoulder_pitch, left_shoulder_roll, left_shoulder_yaw,
left_elbow, left_wrist_roll, left_wrist_pitch, left_wrist_yaw,
right_shoulder_pitch, right_shoulder_roll, right_shoulder_yaw,
right_elbow, right_wrist_roll, right_wrist_pitch, right_wrist_yaw
```

All names carry the upstream `_joint` suffix. The bridge refuses a different
IK or MuJoCo order.

## Human-to-G1 adapter

`G1FrameAdapter` records neutral human and robot wrist poses, then maps the
change in shoulder-relative human reach into the G1 base frame:

```text
p_g1 = p_g1_neutral
     + configured_g1_reach * B * (human_reach - neutral_human_reach)
```

`B` is an explicit proper rotation. Position may be radially clamped about
the G1 shoulder; orientation uses the neutral-relative rotation and is never
scaled.

The software measured neutral model reach as `0.3215094551 m`; the configured
example is `0.321 m`.

## Filtering and safety

- position EMA plus translation-rate limit;
- quaternion SLERP plus angular-rate limit;
- finite/SE(3)/workspace checks;
- upstream joint-limit checks;
- FK residual limits: 0.05 m and 15°;
- maximum per-frame joint change;
- invalid input or IK output holds the last safe target.

The saved simulation run solved 7/7 targets. Maximum observed IK residuals were
`0.0388698 m` and `11.7530°`, within the explicit wrapper limits.

After the live-viewer additions, the same simulation-only regression was rerun:
7/7 IK targets remained valid and the maximum final MuJoCo arm error remained
`0.003815 rad`. No USB, DDS, or physical G1 operation occurred.
