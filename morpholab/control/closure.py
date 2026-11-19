"""Closure controllers.

The original project's `grasp_step` had the right idea -- advance a digit until its pad
touches, then move on -- but fused it to the evaluation loop and to specific joint names
(``docs/00-REVIEW.md`` §3.7).  Here it is a controller among several, so a morphology can
be evaluated under a *matched* control policy, which is the only way a morphology
comparison means anything.

Three policies, in increasing order of what they assume:

``position``
    Open-loop: drive every actuator to a commanded fraction.  The honest baseline, and
    the one to use when comparing morphologies without letting the controller adapt.
``ramp``
    Close at a fixed rate and freeze each digit when its pads make contact.  Works for
    per-joint, tendon-coupled and synergy hands alike, and is the platform default.
``sequential``
    Advance joints proximal-to-distal, freezing each on contact.  Per-joint hands only;
    it reproduces in software what tendon coupling does in hardware, which makes it the
    control condition for "is underactuation worth it".
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

from ..core.logging import get_logger
from .base import ControlContext, DigitHandles, controllers

log = get_logger(__name__)

__all__ = ["PositionController", "RampClosureController", "SequentialClosureController"]

_CONTACT_FORCE_EPS = 0.05
"""Normal force (N) above which a contact counts as real rather than as grazing."""


def _digit_contact_force(ctx: ControlContext, data: Any, digit: DigitHandles) -> float:
    """Total normal force between this digit's geoms and the object."""
    import mujoco

    total = 0.0
    geoms = set(digit.pad_geom_ids) | set(digit.link_geom_ids)
    buf = np.zeros(6)
    for i in range(data.ncon):
        c = data.contact[i]
        g1, g2 = int(c.geom1), int(c.geom2)
        hit = (g1 in geoms and g2 in ctx.object_geom_ids) or (
            g2 in geoms and g1 in ctx.object_geom_ids
        )
        if hit:
            mujoco.mj_contactForce(ctx.model, data, i, buf)
            total += abs(float(buf[0]))
    return total


def _geom_contact_force(ctx: ControlContext, data: Any, geom_ids: set[int]) -> float:
    import mujoco

    total = 0.0
    buf = np.zeros(6)
    for i in range(data.ncon):
        c = data.contact[i]
        g1, g2 = int(c.geom1), int(c.geom2)
        if (g1 in geom_ids and g2 in ctx.object_geom_ids) or (
            g2 in geom_ids and g1 in ctx.object_geom_ids
        ):
            mujoco.mj_contactForce(ctx.model, data, i, buf)
            total += abs(float(buf[0]))
    return total


# ---------------------------------------------------------------------------------------

@dataclass
class PositionController:
    """Open-loop position control: every actuator goes to a fixed fraction of its range.

    Deliberately blind.  When two morphologies are compared under this controller, the
    difference between them is morphology and nothing else -- no adaptive behaviour is
    quietly compensating for a bad design.
    """

    close_fraction: float = 1.0
    open_fraction: float = 0.0
    rate: float = 2.5
    """Fraction of range traversed per second, so the command is a ramp rather than a
    step; stepping a position actuator through a compliant weld launches the object."""

    name: str = "position"
    _target: float = field(default=0.0, init=False, repr=False)

    def reset(self, ctx: ControlContext, data: Any) -> None:
        self._target = max(self.open_fraction, float(ctx.meta.get("preshape", 0.0)))

    def act(self, ctx: ControlContext, data: Any, t: float, grip: str) -> np.ndarray:
        goal = {
            "open": self.open_fraction,
            "close": self.close_fraction,
            "release": self.open_fraction,
        }.get(grip, self._target)
        step = self.rate * ctx.timestep
        self._target += float(np.clip(goal - self._target, -step, step))
        return ctx.ctrl_open + self._target * (ctx.ctrl_closed - ctx.ctrl_open)

    def state(self) -> dict[str, Any]:
        return {"target": self._target}


@dataclass
class RampClosureController:
    """Close at a fixed rate; freeze each digit once it is loaded against the object.

    Freezing on *force* rather than on contact existence is what makes this usable: a
    grazing touch during approach would otherwise stop a digit before it has any grip, and
    the resulting "grasp" would be a statistic about the controller, not the hand.

    The frozen command is held slightly *past* the contact point (``squeeze``) so the digit
    keeps loading the object instead of resting exactly at the threshold, which is what a
    real position-controlled finger does against its force limit.
    """

    rate: float = 0.75
    """Fraction of the actuator range per second."""

    grip_force: float = 3.0
    """Target normal force per digit, in newtons.

    Closing to a *force* rather than to a contact event is what makes this controller
    comparable across morphologies: every hand is asked for the same grip, so the
    difference in outcome is the hand's ability to deliver and distribute it, not an
    accident of how much extra travel a fixed 'squeeze' happened to produce in its
    particular actuator units.
    """

    force_threshold: float = 0.2
    """Force above which a digit counts as having *made* contact (for event logging and
    for the sequential controller's advance test)."""

    squeeze: float = 0.02
    """Extra fraction of range held past the force target, so the grip settles into its
    limit rather than oscillating around it."""

    release_rate: float = 1.5
    freeze_digits: bool = True
    name: str = "ramp"

    _cmd: np.ndarray = field(default=None, init=False, repr=False)  # type: ignore[assignment]
    _frozen: dict[str, float] = field(default_factory=dict, init=False, repr=False)
    _contact_time: dict[str, float] = field(default_factory=dict, init=False, repr=False)

    def reset(self, ctx: ControlContext, data: Any) -> None:
        # The approach planner may pre-close the digits so their closing arc clears the
        # support surface (``morpholab.sim.approach``). Starting from that fraction is
        # what makes a long-fingered hand able to pick a marker off a table at all.
        self._cmd = np.full(ctx.n_actuators, float(ctx.meta.get("preshape", 0.0)))
        for a in ctx.abduction_actuator_ids:
            self._cmd[a] = 0.0
        self._frozen = {}
        self._contact_time = {}

    def act(self, ctx: ControlContext, data: Any, t: float, grip: str) -> np.ndarray:
        if self._cmd is None:
            self.reset(ctx, data)

        if grip in ("open", "release"):
            floor = float(ctx.meta.get("preshape", 0.0))
            self._cmd -= self.release_rate * ctx.timestep
            np.clip(self._cmd, floor, 1.0, out=self._cmd)
            self._frozen.clear()
        elif grip == "close":
            step = self.rate * ctx.timestep
            for digit in ctx.digits:
                if digit.name in self._frozen:
                    continue
                if self.freeze_digits and self._loaded(ctx, data, digit):
                    self._frozen[digit.name] = float(
                        min(1.0, max(self._cmd[list(digit.actuator_ids)].max(), 0.0) + self.squeeze)
                    )
                    self._contact_time.setdefault(digit.name, t)
                    for a in digit.actuator_ids:
                        self._cmd[a] = self._frozen[digit.name]
                    continue
                for a in digit.actuator_ids:
                    self._cmd[a] = min(1.0, self._cmd[a] + step)
        # "hold": leave the command where it is.

        return ctx.ctrl_open + self._cmd * (ctx.ctrl_closed - ctx.ctrl_open)

    def _loaded(self, ctx: ControlContext, data: Any, digit: DigitHandles) -> bool:
        force = _digit_contact_force(ctx, data, digit)
        if force >= self.force_threshold and digit.name not in self._contact_time:
            self._contact_time[digit.name] = -1.0  # marks first touch; overwritten on freeze
        return force >= self.grip_force

    def state(self) -> dict[str, Any]:
        return {
            "frozen_digits": sorted(self._frozen),
            "contact_times": dict(self._contact_time),
            "command": None if self._cmd is None else self._cmd.tolist(),
        }


@dataclass
class SequentialClosureController:
    """Advance a digit's joints proximal-to-distal, freezing each as it makes contact.

    This is the corrected, generalized form of the original ``grasp_step``: it is a
    controller rather than a fused control-and-evaluation loop, it reads forces rather
    than mere contact existence, it works for any digit count and phalanx count, and it
    cannot advance one digit while forgetting to check the others.

    Per-joint actuation only -- a tendon-coupled hand produces this sequencing mechanically
    and has no independent joint commands to schedule.  That contrast is the point: this
    controller is the software control condition against which underactuation is measured.
    """

    rate: float = 1.4
    """Radians per second of joint travel."""

    force_threshold: float = 0.3
    squeeze: float = 0.05
    name: str = "sequential"

    _cmd: np.ndarray = field(default=None, init=False, repr=False)  # type: ignore[assignment]
    _active: dict[str, int] = field(default_factory=dict, init=False, repr=False)

    def reset(self, ctx: ControlContext, data: Any) -> None:
        preshape = float(ctx.meta.get("preshape", 0.0))
        self._cmd = np.full(ctx.n_actuators, preshape)
        for a in ctx.abduction_actuator_ids:
            self._cmd[a] = 0.0
        self._active = {d.name: 0 for d in ctx.digits}
        for digit in ctx.digits:
            if len(digit.actuator_ids) < digit.n_joints:
                log.debug(
                    "sequential closure on digit %s has %d actuators for %d joints; "
                    "it will behave like a ramp controller",
                    digit.name, len(digit.actuator_ids), digit.n_joints,
                )

    def act(self, ctx: ControlContext, data: Any, t: float, grip: str) -> np.ndarray:
        if self._cmd is None:
            self.reset(ctx, data)

        if grip in ("open", "release"):
            self._cmd -= 2.0 * ctx.timestep
            np.clip(self._cmd, 0.0, 1.0, out=self._cmd)
            self._active = {d.name: 0 for d in ctx.digits}
        elif grip == "close":
            for digit in ctx.digits:
                self._advance(ctx, data, digit)

        return ctx.ctrl_open + self._cmd * (ctx.ctrl_closed - ctx.ctrl_open)

    def _advance(self, ctx: ControlContext, data: Any, digit: DigitHandles) -> None:
        idx = self._active.get(digit.name, 0)
        n = len(digit.actuator_ids)
        if idx >= n:
            return

        actuator = digit.actuator_ids[idx]
        span = float(
            ctx.model.actuator_ctrlrange[actuator, 1] - ctx.model.actuator_ctrlrange[actuator, 0]
        )
        step = (self.rate * ctx.timestep / span) if span > 1e-9 else 1.0
        self._cmd[actuator] = min(1.0, self._cmd[actuator] + step)

        # Advance when this phalanx's own pad is loaded, or when the joint has run out of
        # travel -- otherwise a digit that misses the object stalls forever.
        pad_id = digit.pad_geom_ids[idx] if idx < len(digit.pad_geom_ids) else None
        loaded = (
            pad_id is not None
            and _geom_contact_force(ctx, data, {pad_id}) >= self.force_threshold
        )
        if loaded or self._cmd[actuator] >= 1.0 - 1e-9:
            self._cmd[actuator] = min(1.0, self._cmd[actuator] + self.squeeze)
            self._active[digit.name] = idx + 1

    def state(self) -> dict[str, Any]:
        return {"active_joint": dict(self._active),
                "command": None if self._cmd is None else self._cmd.tolist()}


# ---------------------------------------------------------------------------------------
# registry
# ---------------------------------------------------------------------------------------

@controllers.register("position", doc="Open-loop position control at a fixed rate", tags=["scripted"])
def _position(**kw: Any) -> PositionController:
    return PositionController(**kw)


@controllers.register("ramp", doc="Close at a fixed rate, freeze each digit on contact force",
                      tags=["scripted", "default"])
def _ramp(**kw: Any) -> RampClosureController:
    return RampClosureController(**kw)


@controllers.register("sequential", doc="Proximal-to-distal joint closure (per-joint hands)",
                      tags=["scripted"])
def _sequential(**kw: Any) -> SequentialClosureController:
    return SequentialClosureController(**kw)


@controllers.register("firm", doc="Ramp closure with a high force threshold: a tighter grip",
                      tags=["scripted"])
def _firm(**kw: Any) -> RampClosureController:
    """Squeezes harder before freezing.  Useful as the second level of a controller factor
    in ablations, where holding the controller fixed across morphologies is the point."""
    return RampClosureController(grip_force=8.0, **kw)


@controllers.register("gentle", doc="Ramp closure that stops at first touch: precision grasps",
                      tags=["scripted"])
def _gentle(**kw: Any) -> RampClosureController:
    return RampClosureController(rate=0.45, grip_force=1.0, force_threshold=0.08, **kw)
