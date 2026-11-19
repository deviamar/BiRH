"""A Gymnasium-style environment over the scene/task/controller stack.

For gradient-based RL (PPO, SAC) when a task is too hard for ES -- in-hand reorientation,
say -- or when you want to plug a standard library in.  ``gymnasium`` is optional: the
class exposes the ``reset``/``step`` protocol with or without it, and registers ``Box``
spaces when it is installed so ``stable-baselines3`` can consume it directly.

The wrist follows the task's scripted stages exactly as the rollout engine would; the
policy controls only the actuators.  That keeps the learning problem about *grasping*
rather than about re-learning the approach, and keeps results comparable with the
scripted controllers, which also do not command the wrist.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

from ..morphology.spec import HandSpec
from .controller import ACT_DIM, OBS_DIM, PolicyController, observation
from .policy import MLPPolicy

try:  # pragma: no cover - depends on the environment
    import gymnasium as _gym
    _Base = _gym.Env
except ImportError:  # pragma: no cover
    _gym = None

    class _Base:  # type: ignore[no-redef]
        metadata: dict[str, Any] = {}


__all__ = ["GraspEnv", "make_env"]


@dataclass
class _StageCursor:
    index: int = 0
    t_in_stage: float = 0.0
    base_wrist: Any = None


class GraspEnv(_Base):
    """``obs`` is the same fixed-layout vector the ES policy sees; ``action`` is a
    per-actuator target *rate* in ``[-1, 1]`` (see :class:`PolicyController`)."""

    metadata = {"render_modes": ["rgb_array"], "render_fps": 30}

    def __init__(
        self,
        hand: HandSpec,
        obj: str = "sphere_medium",
        task: str = "lift",
        *,
        frame_skip: int = 10,
        residual: bool = True,
        rate: float = 1.5,
        scene_options: dict[str, Any] | None = None,
        render_size: tuple[int, int] = (480, 360),
    ) -> None:
        from ..objects import get_object
        from ..sim import Rollout, build_scene
        from ..sim.scene import SceneOptions
        from ..tasks import get_task

        self.hand, self.obj, self.task = hand, get_object(obj), get_task(task)
        self.frame_skip, self.residual, self.rate = int(frame_skip), residual, rate
        self.scene = build_scene(hand, self.obj, options=SceneOptions(**scene_options) if scene_options else None,
                                 approach=self.task.approach, standoff=self.task.standoff)
        # A zero policy: the controller machinery is reused for target bookkeeping only.
        self._ctl = PolicyController(MLPPolicy(OBS_DIM, ACT_DIM, ()), rate=rate, residual=residual)
        self.rollout = Rollout(self.scene, self.task, self._ctl)
        self.model, self.data, self.ctx = self.rollout.model, self.rollout.data, self.rollout.ctx
        self._cursor = _StageCursor()
        self._reference = None
        self._start_z = 0.0
        self._prev_lift = 0.0
        self._steps = 0
        self._renderer = None
        self._render_size = render_size
        self.max_steps = int(sum(s.duration for s in self.task.stages) / (self.ctx.timestep * self.frame_skip)) + 1
        if _gym is not None:
            self.observation_space = _gym.spaces.Box(-np.inf, np.inf, shape=(OBS_DIM,), dtype=np.float32)
            self.action_space = _gym.spaces.Box(-1.0, 1.0, shape=(ACT_DIM,), dtype=np.float32)

    # -- gym protocol -----------------------------------------------------------------

    def reset(self, *, seed: int | None = None, options: dict[str, Any] | None = None):
        import mujoco

        from ..core.rng import RandomContext
        from ..experiments.runner import _apply_poses

        mujoco.mj_resetData(self.model, self.data)
        if seed is not None and (self.task.randomize.active or seed > 0):
            rng = RandomContext(["env", self.hand.morphology_id, self.obj.name, seed])
            object_pose, wrist_pose = self.task.initial_poses(self.hand, self.obj, rng.stream("init_pose"))
            _apply_poses(self.rollout, object_pose, wrist_pose)
        self.rollout._apply_initial_state()
        self._ctl.reset(self.ctx, self.data)
        self.rollout._apply_preshape_posture()
        mujoco.mj_forward(self.model, self.data)
        self._cursor = _StageCursor(0, 0.0, self.rollout._current_wrist())
        self._reference = self.rollout._palm_frame_pose()
        self._start_z = float(self.data.xipos[self.ctx.object_body_id][2])
        self._prev_lift, self._steps = 0.0, 0
        return self._obs(), {}

    def _stage(self):
        return self.task.stages[min(self._cursor.index, len(self.task.stages) - 1)]

    def _obs(self) -> np.ndarray:
        stage = self._stage()
        progress = min(self._cursor.t_in_stage / max(stage.duration, 1e-6), 1.0)
        return observation(self.ctx, self.data, stage.grip, progress, self._ctl._targets).astype(np.float32)

    def step(self, action: np.ndarray):
        import mujoco

        a = np.clip(np.asarray(action, float).ravel()[: ACT_DIM], -1.0, 1.0)
        dt = self.ctx.timestep
        done_task = False
        for _ in range(self.frame_skip):
            stage = self._stage()
            pose = stage.wrist.pose_at(self._cursor.t_in_stage, self._cursor.base_wrist, stage.duration)
            self.rollout._set_wrist(pose)
            # Same update rule as PolicyController.act, but the action comes from outside.
            delta = a[: self._ctl._targets.size] * self.rate * dt
            if self.residual:
                goal = {"open": 0.0, "release": 0.0, "close": 1.0, "hold": 1.0}.get(stage.grip)
                if goal is not None:
                    base = np.clip(goal - self._ctl._targets, -self._ctl.baseline_rate * dt, self._ctl.baseline_rate * dt)
                    base[self._ctl._is_abduction] = 0.0
                    delta = delta + base
            self._ctl._targets = np.clip(self._ctl._targets + delta, 0.0, 1.0)
            self.data.ctrl[:] = self._ctl._lo + self._ctl._targets * (self._ctl._hi - self._ctl._lo)
            mujoco.mj_step(self.model, self.data)
            self._cursor.t_in_stage += dt
            if self._cursor.t_in_stage >= stage.duration:
                self._cursor.index += 1
                self._cursor.t_in_stage = 0.0
                self._cursor.base_wrist = self.rollout._current_wrist()
                if self._cursor.index >= len(self.task.stages):
                    done_task = True
                    break
        self._steps += 1

        obs = self._obs()
        reward, info = self._reward()
        terminated = bool(self.rollout._check_divergence()) or self.rollout._object_escaped(self._reference)
        if terminated:
            reward -= 1.0
            info["escaped"] = True
        truncated = done_task or self._steps >= self.max_steps
        if truncated and not terminated:
            info["success"] = bool(info["lift"] >= 0.8 * self._target_lift() and info["contact_digits"] >= 2)
            reward += 2.0 if info["success"] else 0.0
        return obs, float(reward), bool(terminated), bool(truncated), info

    def _target_lift(self) -> float:
        return float(getattr(self.task.criteria, "min_lift_height", 0.05))

    def _reward(self) -> tuple[float, dict[str, Any]]:
        contacts = self.rollout._contacts()
        slip, rot = self.rollout._palm_frame_error(self._reference)
        lift = float(self.data.xipos[self.ctx.object_body_id][2]) - self._start_z
        n_digits = max(len(self.ctx.digits), 1)
        r = 0.02 * contacts.n_digits / n_digits
        r += 5.0 * (lift - self._prev_lift)
        r -= 0.02 * min(slip / 0.05, 1.0)
        self._prev_lift = lift
        return r, {"lift": lift, "slip": slip, "rotation": rot, "contact_digits": contacts.n_digits,
                   "phase": str(self._stage().phase)}

    def render(self):
        from ..viz.render import Renderer

        if self._renderer is None:
            self._renderer = Renderer(self.model, *self._render_size).__enter__()
        return self._renderer.frame(self.data, camera="front")

    def close(self) -> None:
        if self._renderer is not None:
            self._renderer.__exit__(None, None, None)
            self._renderer = None


def make_env(hand: HandSpec | str, obj: str = "sphere_medium", task: str = "lift", **kwargs: Any) -> GraspEnv:
    """``hand`` may be a preset name, a morphology JSON path or a ``HandSpec``."""
    if isinstance(hand, str):
        from ..experiments.runner import _resolve_morphology

        hand = _resolve_morphology(hand, None)
    return GraspEnv(hand, obj, task, **kwargs)
