"""The perturbation battery: how much disturbance a grasp actually survives.

This is the empirical counterpart of the analytic epsilon metric, and the one the review
argues should be trusted more (``docs/00-REVIEW.md`` §8).  It replaces the original
project's ramp-until-slip loop, which was wrong in three ways (§3.3): the force ramped
while the object accelerated, so the answer depended on the ramp rate and the timestep;
there was no upper bound, so a good grasp looped forever; and the restore snapshot could
be stale.

The corrected procedure (``docs/02-MATH.md`` §5.1):

* apply a **constant** wrench for a **fixed** window, from a **restored snapshot**;
* judge survival by the object's pose error **in the palm frame**;
* find the threshold by geometric bracketing then bisection, so cost is logarithmic and
  bounded.

Every probe restarts from the same snapshot, so the reported tolerance is a property of
the grasp rather than of the integration path taken to measure it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Callable, Sequence

import numpy as np

from ..core.logging import get_logger
from ..core.registry import Registry
from ..core.rng import fibonacci_sphere, spherical_directions
from ..core.types import Array
from ..mjcf.transforms import geodesic_angle, matrix_from_quat, quat_conjugate, quat_multiply
from .trace import Snapshot

if TYPE_CHECKING:  # pragma: no cover
    import mujoco

log = get_logger(__name__)

__all__ = ["DisturbanceResult", "PerturbationOptions", "disturbance_tolerance", "perturbations"]

perturbations: Registry[Any] = Registry("perturbation")


@dataclass(frozen=True)
class PerturbationOptions:
    """Settings for the disturbance search."""

    window: float = 0.25
    """Seconds the constant wrench is applied. Long enough for the object to move if the
    grasp cannot hold it, short enough that 12 probes per direction stay cheap."""

    max_slip: float = 0.02
    max_rotation: float = math.radians(30.0)
    directions: int = 26
    """Force directions on the sphere, from a Fibonacci lattice. The worst-case estimate is
    biased upward by sampling, so this number is reported with the result."""

    include_torque: bool = False
    f_start: float = 0.25
    f_max: float = 60.0
    bisection_steps: int = 6
    settle_steps: int = 10
    """Steps run after restoring, before the wrench is applied, so the restore transient
    does not count as disturbance-induced motion."""

    def to_dict(self) -> dict[str, Any]:
        return dict(vars(self))


@dataclass
class DisturbanceResult:
    """Outcome of a disturbance sweep over directions."""

    directions: Array = field(default_factory=lambda: np.zeros((0, 3)))
    thresholds: Array = field(default_factory=lambda: np.zeros(0))
    gravity_multiplier: float = 0.0
    monotonicity_violations: int = 0
    options: dict[str, Any] = field(default_factory=dict)

    @property
    def worst(self) -> float:
        """Minimum force survived over sampled directions -- the number a grasp fails at."""
        return float(self.thresholds.min()) if self.thresholds.size else 0.0

    @property
    def mean(self) -> float:
        return float(self.thresholds.mean()) if self.thresholds.size else 0.0

    @property
    def anisotropy(self) -> float:
        """``1 - worst/mean``: zero for an isotropically strong grasp, near one when the
        grasp has a single fragile direction."""
        m = self.mean
        return float(1.0 - self.worst / m) if m > 1e-9 else 0.0

    @property
    def polytope_volume(self) -> float:
        """Volume of the convex hull of ``threshold * direction``, normalized by the ball
        of radius ``mean`` -- the empirical analogue of the epsilon/volume pair."""
        if self.thresholds.size < 4:
            return 0.0
        from scipy.spatial import ConvexHull, QhullError

        pts = self.directions * self.thresholds[:, None]
        try:
            vol = float(ConvexHull(pts).volume)
        except (QhullError, ValueError):
            return 0.0
        ball = 4.0 / 3.0 * math.pi * max(self.mean, 1e-9) ** 3
        return vol / ball

    def to_dict(self) -> dict[str, Any]:
        return {
            "disturbance_worst": self.worst,
            "disturbance_mean": self.mean,
            "disturbance_anisotropy": self.anisotropy,
            "disturbance_polytope": self.polytope_volume,
            "gravity_multiplier": self.gravity_multiplier,
            "disturbance_directions": int(self.thresholds.size),
            "disturbance_monotonicity_violations": self.monotonicity_violations,
        }


def _palm_frame_pose(model: Any, data: Any, palm_body: int, object_body: int) -> tuple[Array, Array]:
    palm_pos = np.asarray(data.xpos[palm_body], float)
    palm_quat = np.asarray(data.xquat[palm_body], float)
    obj_pos = np.asarray(data.xipos[object_body], float)
    obj_quat = np.asarray(data.xquat[object_body], float)
    R = matrix_from_quat(palm_quat)
    return R.T @ (obj_pos - palm_pos), np.asarray(quat_multiply(quat_conjugate(palm_quat), obj_quat), float)


def _survives(
    model: Any,
    data: Any,
    snapshot: Snapshot,
    wrench: Array,
    *,
    palm_body: int,
    object_body: int,
    options: PerturbationOptions,
) -> bool:
    """Apply a constant wrench for the window and report whether the grasp held."""
    import mujoco

    snapshot.restore(model, data)
    for _ in range(options.settle_steps):
        mujoco.mj_step(model, data)
    ref_pos, ref_quat = _palm_frame_pose(model, data, palm_body, object_body)

    steps = max(1, int(round(options.window / model.opt.timestep)))
    data.xfrc_applied[object_body, :] = wrench
    try:
        for _ in range(steps):
            mujoco.mj_step(model, data)
            if not np.all(np.isfinite(data.qpos)):
                return False
    finally:
        data.xfrc_applied[object_body, :] = 0.0

    pos, quat = _palm_frame_pose(model, data, palm_body, object_body)
    return bool(
        np.linalg.norm(pos - ref_pos) < options.max_slip
        and geodesic_angle(ref_quat, quat) < options.max_rotation
    )


def _threshold(
    model: Any,
    data: Any,
    snapshot: Snapshot,
    direction: Array,
    *,
    palm_body: int,
    object_body: int,
    options: PerturbationOptions,
) -> tuple[float, bool]:
    """Largest force in ``direction`` the grasp survives, by bracketing then bisection.

    Returns ``(threshold, monotone)``.  Monotonicity of survival in magnitude is assumed
    by the bisection and is *checked* rather than taken on faith: a grasp can occasionally
    be stabilized by a larger force seating the object, and a platform that silently
    assumes otherwise would report a confident wrong number (``docs/02-MATH.md`` §5.1).
    """
    def probe(f: float) -> bool:
        wrench = np.zeros(6)
        wrench[: direction.size] = f * direction
        return _survives(model, data, snapshot, wrench, palm_body=palm_body,
                         object_body=object_body, options=options)

    if not probe(options.f_start):
        return 0.0, True

    lo, hi = options.f_start, options.f_start * 2.0
    while hi <= options.f_max and probe(hi):
        lo, hi = hi, hi * 2.0
    if hi > options.f_max:
        return options.f_max, True

    for _ in range(options.bisection_steps):
        mid = 0.5 * (lo + hi)
        if probe(mid):
            lo = mid
        else:
            hi = mid

    # Monotonicity spot check: a force well above the threshold must also fail.
    monotone = not probe(min(hi * 1.6, options.f_max))
    return lo, monotone


def disturbance_tolerance(
    model: Any,
    data: Any,
    snapshot: Snapshot,
    *,
    palm_body: int,
    object_body: int,
    options: PerturbationOptions | None = None,
) -> DisturbanceResult:
    """Sweep force directions and find the survivable magnitude in each."""
    options = options or PerturbationOptions()
    dirs = fibonacci_sphere(options.directions)
    thresholds = np.zeros(len(dirs))
    violations = 0

    for i, direction in enumerate(dirs):
        thresholds[i], monotone = _threshold(
            model, data, snapshot, direction,
            palm_body=palm_body, object_body=object_body, options=options,
        )
        violations += int(not monotone)

    result = DisturbanceResult(
        directions=dirs, thresholds=thresholds,
        gravity_multiplier=_gravity_tolerance(
            model, data, snapshot, object_body=object_body, palm_body=palm_body, options=options
        ),
        monotonicity_violations=violations,
        options=options.to_dict(),
    )
    if violations:
        log.debug("%d/%d directions violated survival monotonicity", violations, len(dirs))
    return result


def _gravity_tolerance(
    model: Any,
    data: Any,
    snapshot: Snapshot,
    *,
    object_body: int,
    palm_body: int,
    options: PerturbationOptions,
    max_multiplier: float = 12.0,
) -> float:
    """Largest gravity multiplier the grasp holds under.

    The most physically intuitive single robustness number, and it needs no direction
    sampling: it is simply "how many g can this hand carry this object through".
    """
    mass = float(model.body_mass[object_body])
    if mass <= 0:
        return 0.0
    weight = mass * 9.81
    lo, hi = 0.0, 1.0
    while hi <= max_multiplier:
        wrench = np.zeros(6)
        wrench[2] = -weight * hi
        if not _survives(model, data, snapshot, wrench, palm_body=palm_body,
                         object_body=object_body, options=options):
            break
        lo, hi = hi, hi * 2.0
    else:
        return max_multiplier
    for _ in range(4):
        mid = 0.5 * (lo + hi)
        wrench = np.zeros(6)
        wrench[2] = -weight * mid
        if _survives(model, data, snapshot, wrench, palm_body=palm_body,
                     object_body=object_body, options=options):
            lo = mid
        else:
            hi = mid
    return float(lo)


# ---------------------------------------------------------------------------------------
# domain randomization (docs/02-MATH.md §5.3)
# ---------------------------------------------------------------------------------------

@dataclass(frozen=True)
class DomainRandomization:
    """Physics-parameter variation, applied to a compiled model before a trial.

    Kept distinct from the task's pose randomization: pose variation asks whether a grasp
    generalizes over approach geometry, parameter variation asks whether it generalizes
    over the world model.  Conflating them makes the result uninterpretable.
    """

    friction_scale: float = 0.0
    """Log-normal spread of sliding friction, as a fraction. 0.2 means +/- ~20 %."""

    mass_scale: float = 0.0
    solref_scale: float = 0.0
    actuator_gain_scale: float = 0.0
    actuator_noise: float = 0.0
    """Std. dev. of multiplicative control noise applied each step."""

    @property
    def active(self) -> bool:
        return any(v > 0 for v in vars(self).values())

    def apply(self, model: Any, rng: np.random.Generator) -> dict[str, float]:
        """Perturb ``model`` in place; returns the realized factors for the result row."""
        realized: dict[str, float] = {}
        if self.friction_scale > 0:
            factor = float(np.exp(rng.normal(0.0, self.friction_scale)))
            model.geom_friction[:, 0] *= factor
            realized["dr_friction"] = factor
        if self.mass_scale > 0:
            factor = float(np.exp(rng.normal(0.0, self.mass_scale)))
            model.body_mass[:] *= factor
            model.body_inertia[:] *= factor
            realized["dr_mass"] = factor
        if self.solref_scale > 0:
            factor = float(np.exp(rng.normal(0.0, self.solref_scale)))
            model.geom_solref[:, 0] *= factor
            realized["dr_solref"] = factor
        if self.actuator_gain_scale > 0:
            factor = float(np.exp(rng.normal(0.0, self.actuator_gain_scale)))
            model.actuator_gainprm[:, 0] *= factor
            realized["dr_gain"] = factor
        return realized

    def to_dict(self) -> dict[str, Any]:
        return dict(vars(self))

    @staticmethod
    def moderate() -> "DomainRandomization":
        return DomainRandomization(friction_scale=0.20, mass_scale=0.15,
                                   solref_scale=0.20, actuator_gain_scale=0.10)

    @staticmethod
    def aggressive() -> "DomainRandomization":
        return DomainRandomization(friction_scale=0.35, mass_scale=0.30, solref_scale=0.35,
                                   actuator_gain_scale=0.25, actuator_noise=0.03)


@perturbations.register("wrench_sweep", doc="Constant-wrench bisection over sampled directions")
def _wrench_sweep(**kw: Any) -> PerturbationOptions:
    return PerturbationOptions(**kw)


@perturbations.register("wrench_sweep_fast", doc="Six axis directions only: for large sweeps")
def _wrench_fast(**kw: Any) -> PerturbationOptions:
    return PerturbationOptions(directions=6, bisection_steps=4, window=0.2, **kw)


@perturbations.register("wrench_sweep_fine", doc="66 directions, 8 bisection steps: for reports")
def _wrench_fine(**kw: Any) -> PerturbationOptions:
    return PerturbationOptions(directions=66, bisection_steps=8, window=0.35, **kw)
