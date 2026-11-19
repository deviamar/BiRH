"""Shared type aliases and small value objects.

Kept deliberately thin: only things used across at least two layers live here.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Sequence

import numpy as np

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]  # MuJoCo order: (w, x, y, z)
Array = np.ndarray

__all__ = [
    "Vec3", "Quat", "Array", "Phase", "Outcome", "FailureMode", "Pose",
    "MaterialSpec", "SolverSpec",
]


class _ValueStr(str, Enum):
    """A string enum whose ``str()`` is its value.

    Python 3.11 changed ``str()`` on mixin enums to return ``Class.MEMBER``, which would
    put ``"Phase.LIFT"`` into result rows and filenames. Overriding it keeps the on-disk
    representation stable across Python versions.
    """

    def __str__(self) -> str:
        return str(self.value)


class Phase(_ValueStr):
    """Stages of a rollout (``docs/01-ARCHITECTURE.md`` §6.2)."""

    SETTLE = "settle"
    APPROACH = "approach"
    CLOSE = "close"
    SETTLE_GRASP = "settle_grasp"
    LIFT = "lift"
    HOLD = "hold"
    PERTURB = "perturb"
    DONE = "done"


class Outcome(_ValueStr):
    """Top-level trial result."""

    SUCCESS = "success"
    FAILURE = "failure"
    INADMISSIBLE = "inadmissible"  # object cannot fit the hand: excluded from success stats
    ERROR = "error"                # infrastructure fault, not a scientific observation


class FailureMode(_ValueStr):
    """Automatic failure taxonomy (``docs/02-MATH.md`` §5.4).

    Mutually exclusive and assigned in priority order, so failure fractions sum to
    ``1 - success_rate`` and can be stacked in a single bar per morphology.
    """

    NONE = "none"
    NO_CONTACT = "no_contact"
    INSUFFICIENT_DIGITS = "insufficient_digits"
    SLIP_OUT = "slip_out"
    ROTATE_OUT = "rotate_out"
    EJECT_ON_LIFT = "eject_on_lift"
    DISTURBANCE_FAILURE = "disturbance_failure"
    ACTUATOR_SATURATION = "actuator_saturation"
    SELF_COLLISION = "self_collision"
    UNREACHABLE = "unreachable"
    SIM_DIVERGENCE = "sim_divergence"

    @classmethod
    def priority(cls) -> list["FailureMode"]:
        """Classification order; the first matching mode wins."""
        return [
            cls.SIM_DIVERGENCE,
            cls.UNREACHABLE,
            cls.NO_CONTACT,
            cls.INSUFFICIENT_DIGITS,
            cls.SELF_COLLISION,
            cls.ACTUATOR_SATURATION,
            cls.EJECT_ON_LIFT,
            cls.ROTATE_OUT,
            cls.SLIP_OUT,
            cls.DISTURBANCE_FAILURE,
        ]


@dataclass(frozen=True)
class Pose:
    """A rigid-body pose: position and unit quaternion in MuJoCo ``(w, x, y, z)`` order."""

    pos: Vec3 = (0.0, 0.0, 0.0)
    quat: Quat = (1.0, 0.0, 0.0, 0.0)

    def as_arrays(self) -> tuple[Array, Array]:
        return np.asarray(self.pos, float), np.asarray(self.quat, float)

    @staticmethod
    def from_arrays(pos: Sequence[float], quat: Sequence[float]) -> "Pose":
        return Pose(tuple(float(v) for v in pos), tuple(float(v) for v in quat))  # type: ignore[arg-type]

    def to_dict(self) -> dict[str, list[float]]:
        return {"pos": list(self.pos), "quat": list(self.quat)}


@dataclass(frozen=True)
class SolverSpec:
    """MuJoCo soft-contact parameters (``docs/02-MATH.md`` §3).

    ``solref = (time_const, damping_ratio)`` is a *target impedance*, not a spring
    constant: the constraint is driven toward zero violation as a second-order system with
    time constant ``time_const``.  Stability requires ``time_const >= 2 * timestep``; the
    model builder enforces this at compile time rather than letting it silently corrupt
    contact forces (``docs/00-REVIEW.md`` §3.6).
    """

    time_const: float = 0.02
    damping_ratio: float = 1.0
    dmin: float = 0.9
    dmax: float = 0.95
    width: float = 0.001
    midpoint: float = 0.5
    power: float = 2.0

    @property
    def solref(self) -> str:
        return f"{self.time_const:g} {self.damping_ratio:g}"

    @property
    def solimp(self) -> str:
        return (
            f"{self.dmin:g} {self.dmax:g} {self.width:g} "
            f"{self.midpoint:g} {self.power:g}"
        )

    def check_stable(self, timestep: float) -> tuple[bool, str]:
        """Whether these parameters are safe at ``timestep``; message explains failures."""
        if self.time_const < 2.0 * timestep:
            return False, (
                f"solref time constant {self.time_const:g}s is below 2*timestep "
                f"({2 * timestep:g}s); contact forces will be unreliable"
            )
        if not 0.0 < self.dmin <= self.dmax < 1.0:
            return False, f"solimp requires 0 < dmin <= dmax < 1, got {self.dmin}, {self.dmax}"
        return True, ""

    @staticmethod
    def rigid() -> "SolverSpec":
        """Stiff default for structural geometry."""
        return SolverSpec(time_const=0.02, damping_ratio=1.0, dmin=0.9, dmax=0.95)

    @staticmethod
    def compliant() -> "SolverSpec":
        """Softer, slower-recovering surface: the 'compliant palm/pad' research condition."""
        return SolverSpec(time_const=0.05, damping_ratio=1.2, dmin=0.5, dmax=0.9, width=0.005)


@dataclass(frozen=True)
class MaterialSpec:
    """Surface properties of a contacting body.

    ``friction`` is MuJoCo's ``(sliding, torsional, rolling)`` triple.  Torsional friction
    is what lets a fingertip resist twisting about its contact normal; it is small but not
    negligible for pads, and setting it to zero (as hand-authored models often do) makes
    every pinch grasp spin.
    """

    name: str = "default"
    friction: tuple[float, float, float] = (1.0, 0.005, 0.0001)
    solver: SolverSpec = field(default_factory=SolverSpec.rigid)
    rgba: tuple[float, float, float, float] = (0.7, 0.7, 0.72, 1.0)
    density: float = 1000.0

    @property
    def friction_str(self) -> str:
        return " ".join(f"{v:g}" for v in self.friction)

    @property
    def rgba_str(self) -> str:
        return " ".join(f"{v:g}" for v in self.rgba)

    def with_friction(self, sliding: float) -> "MaterialSpec":
        """Copy with a new sliding-friction coefficient (the main swept surface parameter)."""
        return MaterialSpec(
            name=self.name,
            friction=(float(sliding), self.friction[1], self.friction[2]),
            solver=self.solver,
            rgba=self.rgba,
            density=self.density,
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "friction": list(self.friction),
            "solver": vars(self.solver),
            "rgba": list(self.rgba),
            "density": self.density,
        }


# Standard materials used across hands and objects. Values are ordinary engineering
# estimates for dry contact, not measurements; Study 1 calibrates them against the
# physical prototype.
MATERIALS: dict[str, MaterialSpec] = {
    "structure": MaterialSpec("structure", (0.6, 0.005, 0.0001), SolverSpec.rigid(), (0.72, 0.74, 0.78, 1.0)),
    "pad_rubber": MaterialSpec("pad_rubber", (1.2, 0.01, 0.0005), SolverSpec.compliant(), (0.18, 0.18, 0.20, 1.0)),
    "pad_hard": MaterialSpec("pad_hard", (0.6, 0.005, 0.0001), SolverSpec.rigid(), (0.30, 0.30, 0.34, 1.0)),
    "plastic": MaterialSpec("plastic", (0.5, 0.005, 0.0001), SolverSpec.rigid(), (0.85, 0.55, 0.25, 1.0), 950.0),
    "glass": MaterialSpec("glass", (0.35, 0.002, 0.0001), SolverSpec.rigid(), (0.6, 0.8, 0.9, 0.55), 2500.0),
    "metal": MaterialSpec("metal", (0.45, 0.003, 0.0001), SolverSpec.rigid(), (0.6, 0.62, 0.66, 1.0), 7800.0),
    "wood": MaterialSpec("wood", (0.6, 0.006, 0.0002), SolverSpec.rigid(), (0.72, 0.55, 0.33, 1.0), 650.0),
    "organic": MaterialSpec("organic", (0.8, 0.008, 0.0003), SolverSpec.compliant(), (0.85, 0.25, 0.22, 1.0), 900.0),
    "ground": MaterialSpec("ground", (1.0, 0.005, 0.0001), SolverSpec.rigid(), (0.35, 0.38, 0.42, 1.0)),
}
