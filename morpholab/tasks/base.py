"""Task interface: what a trial does, and what counts as success.

A task is a **declarative script**: a list of stages, each naming a duration, what the
grip should do, and how the wrist should move.  The rollout engine executes stages without
knowing what task it is running, which is what lets a new task be added without touching
the engine (``docs/01-ARCHITECTURE.md`` §13).

Success criteria live here too, because "did it work?" is a property of the task -- a
lift succeeds if the object rises and stays held, a reorientation succeeds if it turns --
and burying that in the engine would make every new task an engine change.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import Any, Sequence

import numpy as np

from ..core.registry import Registry
from ..core.types import Phase, Pose, Quat, Vec3
from ..mjcf.transforms import quat_from_axis_angle, quat_multiply
from ..objects.spec import ObjectSpec
from ..morphology.spec import HandSpec

__all__ = [
    "WristMotion", "Static", "Translate", "Orbit", "Shake", "Sequence_",
    "Stage", "SuccessCriteria", "Task", "tasks", "get_task",
]

tasks: Registry["Task"] = Registry("task")


def get_task(name: str, **kwargs) -> "Task":
    return tasks.create(name, **kwargs)


# ---------------------------------------------------------------------------------------
# wrist motions
# ---------------------------------------------------------------------------------------

@dataclass(frozen=True)
class WristMotion:
    """How the mocap wrist target moves during a stage.

    ``pose_at`` receives the stage-local time and the pose the wrist held when the stage
    began, and returns the commanded pose.  Motions are therefore composable and
    stage-relative, so a lift followed by a shake needs no absolute bookkeeping.
    """

    def pose_at(self, t: float, base: Pose, duration: float) -> Pose:  # pragma: no cover - interface
        raise NotImplementedError

    def to_dict(self) -> dict[str, Any]:
        return {"kind": type(self).__name__, **{k: _jsonable(v) for k, v in vars(self).items()}}


def _jsonable(v: Any) -> Any:
    if isinstance(v, WristMotion):
        return v.to_dict()
    if isinstance(v, (list, tuple)):
        return [_jsonable(x) for x in v]
    return v


@dataclass(frozen=True)
class Static(WristMotion):
    """Hold position. The default for settling, closing and holding."""

    def pose_at(self, t: float, base: Pose, duration: float) -> Pose:
        return base


@dataclass(frozen=True)
class Translate(WristMotion):
    """Move by ``delta`` over the stage, with a smooth velocity profile.

    The profile matters: a step command through a compliant weld produces an impulse that
    can shake an object loose on its own, which would be measured as a grasp failure
    rather than as the artifact it is.  A smoothstep ramp has zero velocity at both ends.
    """

    delta: Vec3 = (0.0, 0.0, 0.1)
    profile: str = "smooth"  # smooth | linear

    def pose_at(self, t: float, base: Pose, duration: float) -> Pose:
        u = 0.0 if duration <= 0 else min(max(t / duration, 0.0), 1.0)
        s = u * u * (3.0 - 2.0 * u) if self.profile == "smooth" else u
        return Pose(
            pos=tuple(base.pos[i] + s * self.delta[i] for i in range(3)),  # type: ignore[arg-type]
            quat=base.quat,
        )


@dataclass(frozen=True)
class Orbit(WristMotion):
    """Rotate the wrist about a body-fixed axis by ``angle`` over the stage.

    Used for the reorientation task and for the "does the grasp survive being turned over"
    check that motivated the original three-finger, force-closure-under-rotation argument
    (``docs/00-REVIEW.md`` §2.2).
    """

    axis: Vec3 = (1.0, 0.0, 0.0)
    angle: float = math.pi
    profile: str = "smooth"

    def pose_at(self, t: float, base: Pose, duration: float) -> Pose:
        u = 0.0 if duration <= 0 else min(max(t / duration, 0.0), 1.0)
        s = u * u * (3.0 - 2.0 * u) if self.profile == "smooth" else u
        return Pose(pos=base.pos, quat=quat_multiply(base.quat, quat_from_axis_angle(self.axis, s * self.angle)))


@dataclass(frozen=True)
class Shake(WristMotion):
    """Sinusoidal translation, optionally with a growing amplitude.

    A shake test is the most physically interpretable robustness probe there is: a hand
    that survives 4 g of shaking is holding the object, whatever the analytic metric says.
    """

    axis: Vec3 = (1.0, 0.0, 0.0)
    amplitude: float = 0.02
    frequency: float = 4.0
    ramp: bool = True

    def pose_at(self, t: float, base: Pose, duration: float) -> Pose:
        scale = (t / duration if (self.ramp and duration > 0) else 1.0)
        offset = self.amplitude * scale * math.sin(2.0 * math.pi * self.frequency * t)
        return Pose(
            pos=tuple(base.pos[i] + offset * self.axis[i] for i in range(3)),  # type: ignore[arg-type]
            quat=base.quat,
        )

    def peak_acceleration(self) -> float:
        """Peak acceleration in g, the number worth reporting."""
        return self.amplitude * (2 * math.pi * self.frequency) ** 2 / 9.81


@dataclass(frozen=True)
class Sequence_(WristMotion):
    """Two motions applied one after the other within a single stage."""

    first: WristMotion
    second: WristMotion
    split: float = 0.5

    def pose_at(self, t: float, base: Pose, duration: float) -> Pose:
        cut = duration * self.split
        if t <= cut:
            return self.first.pose_at(t, base, cut)
        mid = self.first.pose_at(cut, base, cut)
        return self.second.pose_at(t - cut, mid, duration - cut)


STATIC = Static()


# ---------------------------------------------------------------------------------------
# stages
# ---------------------------------------------------------------------------------------

@dataclass(frozen=True)
class Stage:
    """One phase of a trial."""

    phase: Phase
    duration: float
    grip: str = "hold"
    """``open`` | ``close`` | ``hold`` | ``release``. The controller interprets this; the
    engine only passes it along, so a learned policy can ignore it entirely."""

    wrist: WristMotion = STATIC
    snapshot: bool = False
    """Capture full simulation state on exit. The ``grasp`` snapshot is what the analytic
    metrics and the perturbation battery both re-enter the sim from
    (``docs/01-ARCHITECTURE.md`` §4.3)."""

    abort_if_lost: bool = False
    """End the trial early if the object leaves the hand. Most bad morphologies fail in
    the first second, so this is where the bulk of a large sweep's compute is saved."""

    label: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "phase": str(self.phase), "duration": self.duration, "grip": self.grip,
            "wrist": self.wrist.to_dict(), "snapshot": self.snapshot,
            "abort_if_lost": self.abort_if_lost, "label": self.label,
        }


# ---------------------------------------------------------------------------------------
# success
# ---------------------------------------------------------------------------------------

@dataclass(frozen=True)
class SuccessCriteria:
    """What the trial must achieve.

    Thresholds are absolute rather than relative to object size, because "the object moved
    2 cm in the palm frame" means the same thing for a pen and for a bottle -- it means the
    grasp is not holding it.
    """

    min_lift_height: float = 0.05
    """Rise of the object above its start, at the end of the lift stage."""

    max_slip_distance: float = 0.03
    """Translation of the object *in the palm frame* over the holding stages.
    Measuring in the palm frame is essential: a palm-relative measure of distance alone
    cannot see an object sliding tangentially around the palm at constant radius
    (``docs/00-REVIEW.md`` §3.3)."""

    max_slip_rotation: float = math.radians(35.0)
    min_contacts: int = 2
    min_contact_digits: int = 2
    """Distinct digits touching. Three contacts from one finger is not a three-finger
    grasp, and counting contacts alone would call it one."""

    require_final_contact: bool = True
    max_object_speed: float = 0.25
    """Object speed at the end; a fumbled object can momentarily satisfy every position
    criterion while already flying out of the hand."""

    def to_dict(self) -> dict[str, Any]:
        return dict(vars(self))


# ---------------------------------------------------------------------------------------
# task
# ---------------------------------------------------------------------------------------

@dataclass(frozen=True)
class Task:
    """A complete trial script."""

    name: str
    stages: tuple[Stage, ...]
    criteria: SuccessCriteria = field(default_factory=SuccessCriteria)
    approach: str = "top"
    standoff: float = 0.0
    """Extra clearance *beyond* the planned approach pose. The planner already positions
    the palm so the digits converge on the object (``morpholab.sim.approach``); this is
    only for tasks that deliberately want a shallower grasp."""

    grasp_type: str = "power"
    """AHAP-style label, so results connect to the published benchmark."""

    object_filter: tuple[str, ...] = ()
    """Only run on objects whose ``affords`` intersects this. Empty means any object."""

    randomize: "Randomization" = None  # type: ignore[assignment]
    doc: str = ""

    def __post_init__(self) -> None:
        if not self.stages:
            raise ValueError(f"task {self.name!r} has no stages")
        if self.randomize is None:
            object.__setattr__(self, "randomize", Randomization())

    @property
    def duration(self) -> float:
        return sum(s.duration for s in self.stages)

    def stage(self, phase: Phase) -> Stage | None:
        return next((s for s in self.stages if s.phase == phase), None)

    def applies_to(self, obj: ObjectSpec) -> bool:
        """Whether this task is meaningful for ``obj``."""
        if not self.object_filter:
            return True
        return bool(set(self.object_filter) & set(obj.affords))

    def initial_poses(
        self, hand: HandSpec, obj: ObjectSpec, rng: np.random.Generator | None = None
    ) -> tuple[Pose, Pose]:
        """Object and wrist poses at the start of a trial, with randomization applied."""
        from ..sim.scene import approach_pose

        object_pose = Pose(pos=(0.0, 0.0, obj.rest_height), quat=obj.spawn_quat)
        wrist = approach_pose(obj, hand, direction=self.approach, standoff=self.standoff)
        if rng is not None:
            object_pose, wrist = self.randomize.apply(object_pose, wrist, rng)
        return object_pose, wrist

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name, "stages": [s.to_dict() for s in self.stages],
            "criteria": self.criteria.to_dict(), "approach": self.approach,
            "standoff": self.standoff, "grasp_type": self.grasp_type,
            "object_filter": list(self.object_filter),
            "randomize": self.randomize.to_dict(),
        }

    def describe(self) -> str:
        lines = [f"{self.name}  ({self.grasp_type} grasp, {self.approach} approach, "
                 f"{self.duration:.1f} s)"]
        for s in self.stages:
            motion = type(s.wrist).__name__.rstrip("_")
            lines.append(
                f"  {str(s.phase):<13} {s.duration:>4.1f}s  grip={s.grip:<7} wrist={motion}"
                + ("  [snapshot]" if s.snapshot else "")
            )
        return "\n".join(lines)


@dataclass(frozen=True)
class Randomization:
    """Trial-to-trial variation of the initial condition.

    Kept separate from domain randomization of *physics* (friction, mass), which the
    perturbation module owns: pose variation tests whether a grasp generalizes over
    approach geometry, parameter variation tests whether it generalizes over the world
    model.  Conflating them makes the result uninterpretable.
    """

    object_xy: float = 0.0
    """Std. dev. of horizontal object placement noise, metres."""

    object_yaw: float = 0.0
    """Std. dev. of object yaw noise, radians."""

    object_tilt: float = 0.0
    wrist_xy: float = 0.0
    wrist_yaw: float = 0.0
    wrist_height: float = 0.0

    def apply(
        self, object_pose: Pose, wrist_pose: Pose, rng: np.random.Generator
    ) -> tuple[Pose, Pose]:
        if self.object_xy:
            dx, dy = rng.normal(0.0, self.object_xy, size=2)
            object_pose = Pose(
                pos=(object_pose.pos[0] + dx, object_pose.pos[1] + dy, object_pose.pos[2]),
                quat=object_pose.quat,
            )
        if self.object_yaw:
            object_pose = Pose(
                pos=object_pose.pos,
                quat=quat_multiply(
                    quat_from_axis_angle((0, 0, 1), rng.normal(0.0, self.object_yaw)),
                    object_pose.quat,
                ),
            )
        if self.object_tilt:
            axis = rng.normal(size=3)
            axis[2] = 0.0
            n = float(np.linalg.norm(axis))
            if n > 1e-9:
                object_pose = Pose(
                    pos=object_pose.pos,
                    quat=quat_multiply(
                        quat_from_axis_angle(tuple(axis / n), rng.normal(0.0, self.object_tilt)),
                        object_pose.quat,
                    ),
                )
        if self.wrist_xy or self.wrist_height:
            dx, dy = rng.normal(0.0, self.wrist_xy, size=2) if self.wrist_xy else (0.0, 0.0)
            dz = rng.normal(0.0, self.wrist_height) if self.wrist_height else 0.0
            wrist_pose = Pose(
                pos=(wrist_pose.pos[0] + dx, wrist_pose.pos[1] + dy, wrist_pose.pos[2] + dz),
                quat=wrist_pose.quat,
            )
        if self.wrist_yaw:
            wrist_pose = Pose(
                pos=wrist_pose.pos,
                quat=quat_multiply(
                    quat_from_axis_angle((0, 0, 1), rng.normal(0.0, self.wrist_yaw)),
                    wrist_pose.quat,
                ),
            )
        return object_pose, wrist_pose

    @property
    def active(self) -> bool:
        return any(v > 0 for v in vars(self).values())

    def to_dict(self) -> dict[str, Any]:
        return dict(vars(self))

    @staticmethod
    def moderate() -> "Randomization":
        """The default for robustness studies: 1 cm placement, 15 degrees yaw."""
        return Randomization(object_xy=0.010, object_yaw=math.radians(15.0),
                             wrist_xy=0.005, wrist_yaw=math.radians(8.0))

    @staticmethod
    def aggressive() -> "Randomization":
        return Randomization(object_xy=0.020, object_yaw=math.radians(45.0),
                             object_tilt=math.radians(10.0), wrist_xy=0.012,
                             wrist_yaw=math.radians(20.0), wrist_height=0.008)
