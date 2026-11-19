"""A learned policy as a :class:`~morpholab.control.base.Controller`.

The rollout engine, the tasks, the metrics and the failure taxonomy do not know or care
whether the controller is a script or a network: they call ``act`` and read the trace.
That is what lets a policy be trained on the same machinery that scores hand-written
controllers, and lets a learned controller's results sit in the same result rows.

**Observation layout** is fixed-size and padded (``MAX_DIGITS x MAX_JOINTS``), so one
policy architecture serves every hand in a parameter space.  Co-design then inherits
policy weights from one hand to the next instead of restarting from zero.

**Residual mode** (the default) adds the policy's output to a scripted closure ramp.  At
initialization the policy is near zero, so the hand closes like the ``position``
controller and the search begins from a sensible grasp instead of from random flailing.
Set ``residual=False`` for a pure learned controller.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

from ..control.base import ControlContext, DigitHandles
from .policy import MLPPolicy

__all__ = ["PolicyController", "observation", "OBS_DIM", "ACT_DIM", "MAX_DIGITS", "MAX_JOINTS", "MAX_ACTUATORS"]

MAX_DIGITS = 5
MAX_JOINTS = 4
MAX_ACTUATORS = 12
_GRIPS = ("open", "close", "hold", "release")
OBS_DIM = MAX_DIGITS * MAX_JOINTS * 2 + MAX_DIGITS + 3 + 3 + len(_GRIPS) + 1 + MAX_ACTUATORS
ACT_DIM = MAX_ACTUATORS


def _digit_force(ctx: ControlContext, data: Any, digit: DigitHandles) -> float:
    import mujoco

    geoms = set(digit.pad_geom_ids) | set(digit.link_geom_ids)
    total, buf = 0.0, np.zeros(6)
    for i in range(data.ncon):
        c = data.contact[i]
        g1, g2 = int(c.geom1), int(c.geom2)
        if (g1 in geoms and g2 in ctx.object_geom_ids) or (g2 in geoms and g1 in ctx.object_geom_ids):
            mujoco.mj_contactForce(ctx.model, data, i, buf)
            total += abs(float(buf[0]))
    return total


def _palm_frame(ctx: ControlContext, data: Any) -> tuple[np.ndarray, np.ndarray]:
    """Object position and linear velocity in the palm frame."""
    palm_pos = np.asarray(data.xpos[ctx.palm_body_id], float)
    R = np.asarray(data.xmat[ctx.palm_body_id], float).reshape(3, 3)
    obj_pos = np.asarray(data.xipos[ctx.object_body_id], float)
    obj_vel = np.asarray(data.cvel[ctx.object_body_id, 3:], float)
    return R.T @ (obj_pos - palm_pos), R.T @ obj_vel


def observation(ctx: ControlContext, data: Any, grip: str, progress: float, targets: np.ndarray) -> np.ndarray:
    """The fixed-layout observation vector (see module docstring)."""
    model = ctx.model
    q = np.zeros((MAX_DIGITS, MAX_JOINTS))
    dq = np.zeros((MAX_DIGITS, MAX_JOINTS))
    forces = np.zeros(MAX_DIGITS)
    for i, d in enumerate(ctx.digits[:MAX_DIGITS]):
        for k, (jid, qadr, dadr) in enumerate(zip(d.joint_ids[:MAX_JOINTS], d.qpos_adr, d.dof_adr)):
            lo, hi = model.jnt_range[jid]
            span = hi - lo if hi > lo else 1.0
            q[i, k] = (float(data.qpos[qadr]) - lo) / span
            dq[i, k] = float(np.clip(data.qvel[dadr] / 10.0, -1.0, 1.0))
        forces[i] = min(_digit_force(ctx, data, d) / 10.0, 1.0)
    rel_pos, rel_vel = _palm_frame(ctx, data)
    grip_onehot = np.array([1.0 if grip == g else 0.0 for g in _GRIPS])
    tgt = np.zeros(MAX_ACTUATORS)
    tgt[: min(targets.size, MAX_ACTUATORS)] = targets[:MAX_ACTUATORS]
    return np.concatenate([
        q.ravel(), dq.ravel(), forces, np.clip(rel_pos / 0.2, -1, 1), np.clip(rel_vel / 1.0, -1, 1),
        grip_onehot, [progress], tgt,
    ])


@dataclass
class PolicyController:
    """Drive the actuators from a policy over the observation above."""

    policy: MLPPolicy
    rate: float = 1.5
    """Maximum change in target fraction per second at full action."""

    residual: bool = True
    baseline_rate: float = 0.75
    """Ramp rate of the scripted baseline in residual mode (fraction of range / s)."""

    stage_duration: float = 1.0
    name: str = "policy"
    _targets: np.ndarray = field(default=None, init=False, repr=False)  # type: ignore[assignment]
    _lo: np.ndarray = field(default=None, init=False, repr=False)  # type: ignore[assignment]
    _hi: np.ndarray = field(default=None, init=False, repr=False)  # type: ignore[assignment]
    _is_abduction: np.ndarray = field(default=None, init=False, repr=False)  # type: ignore[assignment]
    _phase_t0: float = field(default=0.0, init=False, repr=False)
    _last_grip: str = field(default="", init=False, repr=False)

    def reset(self, ctx: ControlContext, data: Any) -> None:
        nu = ctx.model.nu
        self._lo, self._hi = ctx.ctrl_open.copy(), ctx.ctrl_closed.copy()
        self._is_abduction = np.zeros(nu, bool)
        for a in ctx.abduction_actuator_ids:
            self._is_abduction[a] = True
            self._lo[a], self._hi[a] = ctx.model.actuator_ctrlrange[a]
        preshape = float(ctx.meta.get("preshape", 0.0))
        self._targets = np.where(self._is_abduction, 0.5, preshape).astype(float)
        self._phase_t0, self._last_grip = 0.0, ""

    def act(self, ctx: ControlContext, data: Any, t: float, grip: str) -> np.ndarray:
        if grip != self._last_grip:
            self._phase_t0, self._last_grip = t, grip
        progress = min((t - self._phase_t0) / max(self.stage_duration, 1e-6), 1.0)
        obs = observation(ctx, data, grip, progress, self._targets)
        a = self.policy(obs)[: self._targets.size]
        dt = ctx.timestep
        delta = a * self.rate * dt
        if self.residual:
            goal = {"open": 0.0, "release": 0.0, "close": 1.0, "hold": 1.0}.get(grip, None)
            if goal is not None:
                base = np.clip(goal - self._targets, -self.baseline_rate * dt, self.baseline_rate * dt)
                base[self._is_abduction] = 0.0
                delta = delta + base
        self._targets = np.clip(self._targets + delta, 0.0, 1.0)
        return self._lo + self._targets * (self._hi - self._lo)

    def state(self) -> dict[str, Any]:
        return {"targets": None if self._targets is None else self._targets.tolist()}
