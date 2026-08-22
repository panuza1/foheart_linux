"""Thin processed-reference boundary into TeleopIt's existing policy runtime."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from foheart.whole_body.gmr import G1_JOINT_NAMES
from foheart.whole_body.reference import ProcessedG1Reference


def _readonly(value: Any) -> np.ndarray:
    result = np.asarray(value, dtype=float).copy()
    result.setflags(write=False)
    return result


@dataclass(frozen=True)
class TeleopitReference:
    """Immutable 36D TeleopIt qpos plus boundary timing diagnostics."""

    qpos_wxyz: np.ndarray
    source_timestamp_s: float
    timestamp_s: float
    source_frame_number: int | None
    status: str
    reason: str

    @property
    def held(self) -> bool:
        return self.status == "HOLD"


class FoheartTeleopitAdapter:
    """Map a processed FOHEART reference to TeleopIt's current 36D contract."""

    def __init__(self, *, max_reference_age_s: float = 0.25) -> None:
        from teleopit.constants import G1_JOINT_NAMES as TELEOPIT_G1_JOINT_NAMES

        age = float(max_reference_age_s)
        if not np.isfinite(age) or age <= 0.0:
            raise ValueError("max_reference_age_s must be finite and positive")
        if set(TELEOPIT_G1_JOINT_NAMES) != set(G1_JOINT_NAMES):
            raise ValueError("FOHEART and TeleopIt G1 joint-name sets differ")
        self.target_joint_names = tuple(TELEOPIT_G1_JOINT_NAMES)
        self.joint_permutation = tuple(G1_JOINT_NAMES.index(name) for name in self.target_joint_names)
        self.max_reference_age_s = age
        self._source_epoch_s: float | None = None
        self._last_follow: TeleopitReference | None = None

    def reset(self) -> None:
        self._source_epoch_s = None
        self._last_follow = None

    def adapt(
        self,
        reference: ProcessedG1Reference | None,
        *,
        now_s: float | None = None,
    ) -> TeleopitReference:
        if reference is None:
            return self._hold("reference is missing")
        if not isinstance(reference, ProcessedG1Reference):
            raise TypeError("reference must be a ProcessedG1Reference")
        if tuple(reference.joint_names) != G1_JOINT_NAMES:
            raise ValueError("FOHEART G1 joint order is invalid")

        source_timestamp_s = float(reference.source_timestamp_s)
        now = source_timestamp_s if now_s is None else float(now_s)
        if not np.isfinite(source_timestamp_s) or not np.isfinite(now):
            raise ValueError("reference timing must be finite")
        age = now - source_timestamp_s
        if age < 0.0:
            return self._hold("reference timestamp is in the future")
        if age > self.max_reference_age_s:
            return self._hold(
                f"reference is stale ({age:.6f}s > {self.max_reference_age_s:.6f}s)"
            )
        if (
            self._last_follow is not None
            and not reference.held
            and source_timestamp_s <= self._last_follow.source_timestamp_s
        ):
            return self._hold("reference timestamp is duplicate or out of order")

        root_pos = np.asarray(reference.root_pos, dtype=float)
        root_quat = np.asarray(reference.root_quat_wxyz, dtype=float)
        joints = np.asarray(reference.dof_pos, dtype=float)
        qpos = np.asarray(reference.qpos_wxyz, dtype=float)
        if root_pos.shape != (3,) or root_quat.shape != (4,) or joints.shape != (29,):
            raise ValueError("reference must contain root XYZ, root WXYZ, and 29 joints")
        if qpos.shape != (36,) or not all(
            np.isfinite(value).all() for value in (qpos, root_pos, root_quat, joints)
        ):
            raise ValueError("reference must be a finite length-36 qpos")
        if not np.array_equal(qpos[:3], root_pos) or not np.array_equal(qpos[3:7], root_quat):
            raise ValueError("reference root components do not match qpos")
        if not np.array_equal(qpos[7:], joints):
            raise ValueError("reference joint components do not match qpos")
        if not np.isclose(np.linalg.norm(root_quat), 1.0, atol=1e-5):
            raise ValueError("reference root quaternion must be normalized WXYZ")

        if self._source_epoch_s is None:
            self._source_epoch_s = source_timestamp_s
        output_qpos = np.concatenate(
            (root_pos, root_quat, joints[np.asarray(self.joint_permutation, dtype=int)])
        )
        output = TeleopitReference(
            qpos_wxyz=_readonly(output_qpos),
            source_timestamp_s=source_timestamp_s,
            timestamp_s=source_timestamp_s - self._source_epoch_s,
            source_frame_number=reference.source_frame_number,
            status="HOLD" if reference.held else "FOLLOW",
            reason=reference.reason,
        )
        if output.held:
            return self._hold(output.reason) if self._last_follow is not None else output
        self._last_follow = output
        return output

    def _hold(self, reason: str) -> TeleopitReference:
        if self._last_follow is None:
            raise RuntimeError(f"cannot HOLD before the first valid reference: {reason}")
        previous = self._last_follow
        return TeleopitReference(
            previous.qpos_wxyz,
            previous.source_timestamp_s,
            previous.timestamp_s,
            previous.source_frame_number,
            "HOLD",
            reason,
        )


@dataclass(frozen=True)
class PolicySimMetrics:
    maximum_joint_error_rad: float
    mean_joint_error_rad: float
    finite: bool
    steps: int
    simulation_duration_s: float
    root_position_m: tuple[float, float, float]
    root_quaternion_wxyz: tuple[float, float, float, float]
    base_pinned: bool
    stability_status: str
    observation_finite: bool
    action_finite: bool
    observation_shape: tuple[int, ...]
    action_shape: tuple[int, ...]
    minimum_root_height_m: float
    maximum_root_height_m: float
    fall_status: str


class FoheartTeleopitPolicySimulator:
    """Free-base MuJoCo sink composed from TeleopIt's standard inference pieces."""

    mode = "SIMULATION_ONLY"
    whole_body_mode = "TELEOPIT_POLICY_SIM"

    def __init__(
        self,
        teleopit_root: str | Path,
        policy_path: str | Path,
        *,
        max_reference_age_s: float = 0.25,
    ) -> None:
        from omegaconf import OmegaConf
        import mujoco
        from teleopit.controllers.observation import VelCmdObservationBuilder
        from teleopit.controllers.rl_policy import RLPolicyController
        from teleopit.robots.mujoco_robot import MuJoCoRobot
        from teleopit.sim.runtime_components import PolicyStepRunner

        root = Path(teleopit_root).expanduser().resolve()
        robot_cfg = OmegaConf.load(root / "teleopit/configs/robot/g1.yaml")
        controller_cfg = OmegaConf.load(root / "teleopit/configs/controller/rl_policy.yaml")
        robot_cfg.xml_path = str((root / str(robot_cfg.xml_path)).resolve())
        controller_cfg.policy_path = str(Path(policy_path).expanduser().resolve())
        controller_cfg.default_dof_pos = list(robot_cfg.default_angles)
        controller_cfg.action_scale = list(robot_cfg.action_scale)

        self.robot = MuJoCoRobot(robot_cfg)
        joint_ids = np.asarray(self.robot.model.actuator_trnid[:, 0], dtype=int)
        model_joint_names = tuple(self.robot.model.joint(int(index)).name for index in joint_ids)
        from teleopit.constants import G1_JOINT_NAMES as TELEOPIT_G1_JOINT_NAMES

        if model_joint_names != tuple(TELEOPIT_G1_JOINT_NAMES):
            raise RuntimeError(f"unexpected TeleopIt model actuator order: {model_joint_names}")
        limits = np.asarray(self.robot.model.jnt_range[joint_ids], dtype=float)
        self.joint_lower = limits[:, 0].copy()
        self.joint_upper = limits[:, 1].copy()

        self.controller = RLPolicyController(controller_cfg)
        self.obs_builder = VelCmdObservationBuilder(
            {
                "num_actions": int(robot_cfg.num_actions),
                "default_dof_pos": list(robot_cfg.default_angles),
                "xml_path": str(robot_cfg.xml_path),
                "anchor_body_name": "torso_link",
            }
        )
        self.policy_hz = 50.0
        self.pd_hz = 200.0
        self.decimation = 4
        self.adapter = FoheartTeleopitAdapter(max_reference_age_s=max_reference_age_s)
        self._runner = PolicyStepRunner(
            robot=self.robot,
            controller=self.controller,
            obs_builder=self.obs_builder,
            policy_hz=self.policy_hz,
            decimation=self.decimation,
            num_actions=int(robot_cfg.num_actions),
            kps=np.asarray(robot_cfg.kps, dtype=np.float32),
            kds=np.asarray(robot_cfg.kds, dtype=np.float32),
            torque_limits=np.asarray(robot_cfg.torque_limits, dtype=np.float32),
            default_dof_pos=np.asarray(robot_cfg.default_angles, dtype=np.float32),
        )
        if self.obs_builder.total_obs_size != 167 or self.controller._expected_obs_dim != 167:
            raise RuntimeError("TeleopIt policy and observation builder must both use 167D observations")
        if not self.controller._multi_input:
            raise RuntimeError("TeleopIt policy must expose obs and obs_history inputs")
        if self.robot.model.nq != 36 or self.robot.model.nu != 29:
            raise RuntimeError("TeleopIt policy simulator requires a free-base nq=36, nu=29 model")
        if int(self.robot.model.jnt_type[0]) != int(mujoco.mjtJoint.mjJNT_FREE):
            raise RuntimeError("TeleopIt policy simulator model root must be a free joint")

    def command_whole_body(
        self,
        reference: ProcessedG1Reference,
        *,
        steps: int = 1,
    ) -> PolicySimMetrics:
        if not 1 <= int(steps) <= 5000:
            raise ValueError("policy simulation steps must be between 1 and 5000")
        converted = self.adapter.adapt(reference, now_s=reference.source_timestamp_s)
        qpos = np.asarray(converted.qpos_wxyz, dtype=np.float64)
        observations: list[np.ndarray] = []
        actions: list[np.ndarray] = []
        errors: list[np.ndarray] = []
        heights: list[float] = []

        for _ in range(int(steps)):
            state = self.robot.get_state()
            prepared = self._runner.prepare_motion_command(qpos, state)
            observation = self._runner.build_observation(
                state,
                prepared,
                self._runner.last_action,
            )
            observation = self._runner.validate_observation_for_policy(observation)
            action = np.asarray(self.controller.compute_action(observation), dtype=np.float32).reshape(-1)
            if action.shape != (29,):
                raise ValueError(f"TeleopIt policy returned {action.shape}, expected (29,)")
            target = self._runner.compute_target_dof_pos(action)
            _, final_state = self._runner.apply_control(target)
            self._runner.finish_step(action, prepared.qpos)
            observations.append(observation.copy())
            actions.append(action.copy())
            errors.append(np.abs(np.asarray(final_state.qpos, dtype=float) - qpos[7:]))
            heights.append(float(np.asarray(final_state.base_pos, dtype=float)[2]))

        state = self.robot.get_state()
        error = np.concatenate(errors)
        observation_finite = bool(np.isfinite(np.stack(observations)).all())
        action_finite = bool(np.isfinite(np.stack(actions)).all())
        state_finite = bool(
            all(
                np.isfinite(np.asarray(value)).all()
                for value in (state.qpos, state.qvel, state.quat, state.ang_vel, state.base_pos)
            )
        )
        root_pos = tuple(map(float, np.asarray(state.base_pos, dtype=float)))
        root_quat = tuple(map(float, np.asarray(state.quat, dtype=float)))
        return PolicySimMetrics(
            maximum_joint_error_rad=float(np.max(error)),
            mean_joint_error_rad=float(np.mean(error)),
            finite=observation_finite and action_finite and state_finite,
            steps=int(steps),
            simulation_duration_s=float(steps / self.policy_hz),
            root_position_m=root_pos,
            root_quaternion_wxyz=root_quat,
            base_pinned=False,
            stability_status="FREE_BASE_FINITE" if state_finite else "NONFINITE_STATE",
            observation_finite=observation_finite,
            action_finite=action_finite,
            observation_shape=tuple(observations[-1].shape),
            action_shape=tuple(actions[-1].shape),
            minimum_root_height_m=float(min(heights)),
            maximum_root_height_m=float(max(heights)),
            fall_status="NOT_SOURCE_DEFINED",
        )

    def close(self) -> None:
        return None
