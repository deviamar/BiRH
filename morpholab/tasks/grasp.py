"""The task library.

Tasks are ordered by what they can tell you.  ``lift`` answers "can this hand pick the
object up at all"; ``hold_disturbance`` answers "would it still be holding it in a
vehicle"; ``reorient`` answers the question that motivated the original three-finger
design -- does the grasp survive being turned over (``docs/00-REVIEW.md`` §2.2).

Durations are chosen so a trial costs ~2,500-4,500 physics steps at a 2 ms timestep, which
is a few tens of milliseconds of wall clock.  That budget is what makes a 20,000-trial
factorial affordable, and it is why stages that only need to settle are short.
"""

from __future__ import annotations

import math

from ..core.types import Phase
from .base import Orbit, Randomization, Shake, Stage, SuccessCriteria, Task, Translate, tasks

DEG = math.pi / 180.0


def _approach_close_lift(
    *,
    close_time: float = 1.6,
    lift_height: float = 0.12,
    lift_time: float = 1.0,
    hold_time: float = 0.6,
) -> tuple[Stage, ...]:
    """The common opening: settle, close until grasped, snapshot, lift, hold.

    The snapshot on exit from ``settle_grasp`` is the single most important artifact of a
    trial -- every analytic grasp metric and the whole perturbation battery re-enter the
    simulation from it, so evaluation is separable from the rollout and new metrics can be
    computed later without re-simulating (``docs/01-ARCHITECTURE.md`` §4.3).
    """
    return (
        Stage(Phase.SETTLE, 0.35, grip="open", label="object settles on the support"),
        Stage(Phase.CLOSE, close_time, grip="close", abort_if_lost=False, label="digits close"),
        Stage(Phase.SETTLE_GRASP, 0.3, grip="hold", snapshot=True, label="grasp settles"),
        Stage(Phase.LIFT, lift_time, grip="hold", wrist=Translate((0.0, 0.0, lift_height)),
              abort_if_lost=True, label="lift clear of the support"),
        Stage(Phase.HOLD, hold_time, grip="hold", abort_if_lost=True, label="hold"),
    )


# ---------------------------------------------------------------------------------------
# core tasks
# ---------------------------------------------------------------------------------------

@tasks.register("lift", doc="Close, lift 12 cm, hold: the baseline grasp test",
                tags=["core", "power"])
def _lift() -> Task:
    return Task(
        name="lift",
        stages=_approach_close_lift(),
        criteria=SuccessCriteria(min_lift_height=0.05),
        grasp_type="power",
        doc="The default trial. Everything else adds a probe on top of this.",
    )


@tasks.register("lift_randomized", doc="Lift with randomized object and wrist placement",
                tags=["core", "power", "robustness"])
def _lift_randomized() -> Task:
    return Task(
        name="lift_randomized",
        stages=_approach_close_lift(),
        criteria=SuccessCriteria(min_lift_height=0.05),
        grasp_type="power",
        randomize=Randomization.moderate(),
        doc="Tests whether a grasp generalizes over approach geometry rather than "
            "depending on a single lucky alignment.",
    )


@tasks.register("hold_disturbance", doc="Lift, then survive a growing shake",
                tags=["core", "robustness"])
def _hold_disturbance() -> Task:
    """A ramped shake, ending at roughly 3 g of peak acceleration.

    Reported alongside the analytic epsilon metric, because the two disagree often enough
    that the disagreement is itself informative (``docs/00-REVIEW.md`` §8).
    """
    return Task(
        name="hold_disturbance",
        stages=(
            *_approach_close_lift(hold_time=0.3),
            Stage(Phase.PERTURB, 1.6, grip="hold",
                  wrist=Shake(axis=(1.0, 0.0, 0.0), amplitude=0.02, frequency=6.0, ramp=True),
                  abort_if_lost=True, label="lateral shake, ramping to ~3 g"),
            Stage(Phase.HOLD, 0.4, grip="hold", abort_if_lost=True, label="settle after shaking"),
        ),
        criteria=SuccessCriteria(min_lift_height=0.05, max_slip_distance=0.035),
        grasp_type="power",
    )


@tasks.register("reorient", doc="Lift, then rotate the wrist 180 degrees",
                tags=["core", "robustness", "rotation"])
def _reorient() -> Task:
    """Turning the hand over is the direct test of the project's founding argument -- that
    three symmetric fingers keep force closure as the hand rotates about the palm axis."""
    return Task(
        name="reorient",
        stages=(
            *_approach_close_lift(lift_height=0.15, hold_time=0.3),
            Stage(Phase.PERTURB, 1.6, grip="hold", wrist=Orbit(axis=(1.0, 0.0, 0.0), angle=math.pi),
                  abort_if_lost=True, label="roll 180 degrees"),
            Stage(Phase.HOLD, 0.5, grip="hold", abort_if_lost=True, label="hold inverted"),
        ),
        criteria=SuccessCriteria(min_lift_height=0.05, max_slip_rotation=45.0 * DEG),
        grasp_type="power",
    )


@tasks.register("rotate_about_palm", doc="Lift, then spin about the palm axis",
                tags=["robustness", "rotation"])
def _rotate_about_palm() -> Task:
    """Rotation about the palm normal, where a C3-symmetric layout should be least
    sensitive and a two-finger gripper most."""
    return Task(
        name="rotate_about_palm",
        stages=(
            *_approach_close_lift(hold_time=0.3),
            Stage(Phase.PERTURB, 1.4, grip="hold", wrist=Orbit(axis=(0.0, 0.0, 1.0), angle=2 * math.pi),
                  abort_if_lost=True, label="full turn about the palm axis"),
        ),
        criteria=SuccessCriteria(min_lift_height=0.05),
        grasp_type="power",
    )


@tasks.register("transfer", doc="Lift, translate sideways, set down",
                tags=["manipulation"])
def _transfer() -> Task:
    return Task(
        name="transfer",
        stages=(
            *_approach_close_lift(lift_height=0.14, hold_time=0.2),
            Stage(Phase.PERTURB, 1.2, grip="hold", wrist=Translate((0.18, 0.0, 0.0)),
                  abort_if_lost=True, label="translate 18 cm"),
            Stage(Phase.HOLD, 0.4, grip="hold", abort_if_lost=True, label="hold at the target"),
        ),
        criteria=SuccessCriteria(min_lift_height=0.05, max_slip_distance=0.04),
        grasp_type="power",
    )


# ---------------------------------------------------------------------------------------
# grasp-type tasks (AHAP-inspired)
# ---------------------------------------------------------------------------------------

@tasks.register("pinch", doc="Fingertip pinch of a small object, lifted gently",
                tags=["ahap", "precision"])
def _pinch() -> Task:
    """A precision grasp: shorter lift, tighter slip tolerance, and only run on objects
    that afford precision -- pinching a bottle is not a meaningful failure."""
    return Task(
        name="pinch",
        stages=_approach_close_lift(close_time=1.8, lift_height=0.08, lift_time=1.0, hold_time=0.6),
        criteria=SuccessCriteria(
            min_lift_height=0.035, max_slip_distance=0.015,
            max_slip_rotation=25.0 * DEG, min_contacts=2, min_contact_digits=2,
        ),
        approach="top",
        standoff=0.0,
        grasp_type="precision",
        object_filter=("precision", "tripod", "lateral"),
    )


@tasks.register("tripod", doc="Three-digit tripod pinch of a small object",
                tags=["ahap", "precision"])
def _tripod() -> Task:
    return Task(
        name="tripod",
        stages=_approach_close_lift(close_time=1.8, lift_height=0.08, hold_time=0.6),
        criteria=SuccessCriteria(
            min_lift_height=0.035, max_slip_distance=0.02, min_contacts=3, min_contact_digits=3
        ),
        standoff=0.0,
        grasp_type="tripod",
        object_filter=("tripod", "precision"),
    )


@tasks.register("lateral_pinch", doc="Side approach to a flat object -- needs abduction",
                tags=["ahap", "precision", "lateral"])
def _lateral() -> Task:
    """The grasp category the v2 hand testing found impossible without thumb
    abduction/adduction (``docs/00-REVIEW.md`` §2.5).  This task is how that claim becomes
    a measurement instead of a table entry."""
    return Task(
        name="lateral_pinch",
        stages=_approach_close_lift(close_time=1.8, lift_height=0.07, hold_time=0.6),
        criteria=SuccessCriteria(
            min_lift_height=0.03, max_slip_distance=0.02, min_contacts=2, min_contact_digits=2
        ),
        approach="side",
        standoff=0.005,
        grasp_type="lateral",
        object_filter=("lateral",),
    )


@tasks.register("cylindrical", doc="Side approach to a cylinder or bottle: power wrap",
                tags=["ahap", "power"])
def _cylindrical() -> Task:
    return Task(
        name="cylindrical",
        stages=_approach_close_lift(close_time=1.7, lift_height=0.12, hold_time=0.6),
        criteria=SuccessCriteria(min_lift_height=0.05),
        approach="side",
        standoff=0.0,
        grasp_type="cylindrical",
        object_filter=("cylindrical", "power"),
    )


@tasks.register("hook", doc="Hook grasp: engage a handle or opening without opposition",
                tags=["ahap", "hook"])
def _hook() -> Task:
    """No thumb opposition; the digits act as a hook.  Success is carrying load, so the
    lift is longer and slip tolerance is loose."""
    return Task(
        name="hook",
        stages=(
            Stage(Phase.SETTLE, 0.35, grip="open"),
            Stage(Phase.CLOSE, 1.6, grip="close"),
            Stage(Phase.SETTLE_GRASP, 0.3, grip="hold", snapshot=True),
            Stage(Phase.LIFT, 1.2, grip="hold", wrist=Translate((0.0, 0.0, 0.14)),
                  abort_if_lost=True),
            Stage(Phase.HOLD, 0.8, grip="hold", abort_if_lost=True),
        ),
        criteria=SuccessCriteria(
            min_lift_height=0.05, max_slip_distance=0.05, max_slip_rotation=60.0 * DEG,
            min_contacts=2, min_contact_digits=1,
        ),
        approach="side",
        standoff=0.01,
        grasp_type="hook",
        object_filter=("hook",),
    )


# ---------------------------------------------------------------------------------------
# diagnostics
# ---------------------------------------------------------------------------------------

@tasks.register("close_only", doc="Close on the object without lifting: cheap and fast",
                tags=["diagnostic"])
def _close_only() -> Task:
    """For pilots, screening sweeps and metric development: it produces the grasp snapshot
    that the analytic metrics need at a third of the cost of a full lift."""
    return Task(
        name="close_only",
        stages=(
            Stage(Phase.SETTLE, 0.3, grip="open"),
            Stage(Phase.CLOSE, 1.5, grip="close"),
            Stage(Phase.SETTLE_GRASP, 0.4, grip="hold", snapshot=True),
        ),
        criteria=SuccessCriteria(min_lift_height=0.0, require_final_contact=True),
        grasp_type="power",
        doc="No lift, so 'success' means only that a multi-digit contact was achieved.",
    )


@tasks.register("full_battery", doc="Lift, shake, rotate and translate in one trial",
                tags=["robustness", "expensive"])
def _full_battery() -> Task:
    """Everything in sequence.  More expensive than the individual tasks but produces a
    single trial whose failure point locates the weakest capability."""
    return Task(
        name="full_battery",
        stages=(
            *_approach_close_lift(lift_height=0.15, hold_time=0.3),
            Stage(Phase.PERTURB, 1.2, grip="hold",
                  wrist=Shake((1.0, 0.0, 0.0), amplitude=0.018, frequency=6.0),
                  abort_if_lost=True, label="shake"),
            Stage(Phase.PERTURB, 1.4, grip="hold", wrist=Orbit((1.0, 0.0, 0.0), math.pi),
                  abort_if_lost=True, label="invert"),
            Stage(Phase.PERTURB, 1.0, grip="hold", wrist=Translate((0.15, 0.0, 0.0)),
                  abort_if_lost=True, label="translate"),
            Stage(Phase.HOLD, 0.4, grip="hold", abort_if_lost=True),
        ),
        criteria=SuccessCriteria(min_lift_height=0.05, max_slip_distance=0.045,
                                 max_slip_rotation=60.0 * DEG),
        grasp_type="power",
    )


# ---------------------------------------------------------------------------------------
# suites
# ---------------------------------------------------------------------------------------

AHAP_SUITE: tuple[str, ...] = (
    "lift", "cylindrical", "pinch", "tripod", "lateral_pinch", "hook",
)
"""Grasp types drawn from the Anthropomorphic Hand Assessment Protocol.

Each task declares an ``object_filter``, so running the suite over the object library
automatically pairs each grasp type with the objects that afford it, rather than scoring a
hand on pinching a bottle.
"""

CORE_SUITE: tuple[str, ...] = ("lift", "hold_disturbance", "reorient")
QUICK_SUITE: tuple[str, ...] = ("close_only",)
