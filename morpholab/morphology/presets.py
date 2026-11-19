"""Named reference morphologies.

Presets serve three purposes: they are control conditions for experiments, they are the
starting points optimizers seed from, and they are the platform's own regression fixtures.

The two BiRH presets reproduce the project's physical prototypes as faithfully as the
recorded CAD notes allow, so simulated results can be checked against the hand-testing
table in ``design/v2-improved-design/improvements.md`` (``docs/00-REVIEW.md`` §7).
Neither is privileged: v2 is *not* labelled "the improvement", because by the platform's
own complexity metric it is 33 % more complex than v1 (``docs/02-MATH.md`` §6.3).
"""

from __future__ import annotations

import math
from typing import Callable

from ..core.registry import Registry
from .spec import ActuationMode, ActuationSpec, DigitSpec, HandSpec, JointSpec, MountSpec, PalmSpec

__all__ = ["presets", "get", "names", "register"]

presets: Registry[HandSpec] = Registry("morphology preset")
register = presets.register

DEG = math.pi / 180.0


def get(name: str, **kwargs) -> HandSpec:
    """Instantiate a preset by name."""
    return presets.create(name, **kwargs)


def names() -> list[str]:
    return presets.names()


def _finger(
    name: str,
    azimuth_deg: float,
    *,
    length: float,
    ratios: tuple[float, ...],
    flexion_deg: tuple[float, float] = (0.0, 100.0),
    base_radius: float = 0.011,
    tip_radius: float = 0.008,
    damping: float = 0.06,
    abduction_deg: float | None = None,
    abduction_damping: float = 0.12,
    elevation_deg: float = 0.0,
    roll_deg: float = 0.0,
    radius_frac: float = 1.0,
    is_thumb: bool = False,
    pad_material: str = "pad_rubber",
) -> DigitSpec:
    """Digit constructor in degrees, so preset definitions read like design drawings."""
    abduction = (
        JointSpec(
            axis=(0.0, 0.0, 1.0),
            range=(-abduction_deg * DEG, abduction_deg * DEG),
            damping=abduction_damping,
            armature=0.003,
            kind="abduction",
        )
        if abduction_deg
        else None
    )
    return DigitSpec(
        name=name,
        mount=MountSpec(
            azimuth=azimuth_deg * DEG,
            radius_frac=radius_frac,
            elevation=elevation_deg * DEG,
            roll=roll_deg * DEG,
        ),
        total_length=length,
        length_ratios=ratios,
        base_radius=base_radius,
        tip_radius=tip_radius,
        flexion_range=(flexion_deg[0] * DEG, flexion_deg[1] * DEG),
        damping=damping,
        abduction=abduction,
        is_thumb=is_thumb,
        pad_material=pad_material,
    )


# ---------------------------------------------------------------------------------------
# BiRH prototypes
# ---------------------------------------------------------------------------------------

@register("birh_v1", doc="Prototype 1: 3 symmetric fingers, 3 phalanges, tendon-driven",
          tags=["birh", "symmetric", "underactuated"])
def _birh_v1() -> HandSpec:
    """The original CAD hand: three identical fingers at 120 degrees, one motor each.

    Underactuation is restored here.  It is the defining feature of the physical hand and
    was dropped from the simulation when the spatial-tendon model failed to move
    (``docs/00-REVIEW.md`` §4.4), which silently changed the object of study.
    """
    ratios = (0.44, 0.33, 0.23)
    digits = tuple(
        _finger(f"finger_{i + 1}", 90.0 + 120.0 * i, length=0.115, ratios=ratios)
        for i in range(3)
    )
    return HandSpec(
        name="birh_v1",
        palm=PalmSpec(radius=0.05, thickness=0.022),
        digits=digits,
        actuation=ActuationSpec(
            mode=ActuationMode.TENDON_COUPLED,
            kp=6.0,
            force_limit=1.2,
            tendon_ratio=0.7,
            extension_stiffness=0.25,
        ),
        meta={"source": "design/v1-initial-concept", "phalanges": 3},
    )


@register("birh_v1_direct", doc="Prototype 1 geometry with independent joint actuators",
          tags=["birh", "symmetric", "ablation"])
def _birh_v1_direct() -> HandSpec:
    """v1 geometry, per-joint actuation: the control condition that isolates the effect of
    underactuation from the effect of layout."""
    base = _birh_v1()
    return HandSpec(
        name="birh_v1_direct",
        palm=base.palm,
        digits=base.digits,
        actuation=ActuationSpec(mode=ActuationMode.PER_JOINT, kp=6.0, force_limit=1.2),
        meta={"source": "design/v1-initial-concept", "ablation_of": "birh_v1"},
    )


@register("birh_v1_2ph", doc="Prototype 1 layout with 2 phalanges (the simulated variant)",
          tags=["birh", "symmetric"])
def _birh_v1_2ph() -> HandSpec:
    """Matches ``simulation/python-scripts/hand_design.py``: palm radius 5 cm, phalanges
    7.5 cm and 5 cm.  Kept so results from the original scripts remain comparable."""
    digits = tuple(
        _finger(f"finger_{i + 1}", 90.0 + 120.0 * i, length=0.125, ratios=(0.6, 0.4))
        for i in range(3)
    )
    return HandSpec(
        name="birh_v1_2ph",
        palm=PalmSpec(radius=0.05, thickness=0.025),
        digits=digits,
        actuation=ActuationSpec(mode=ActuationMode.PER_JOINT, kp=6.0, force_limit=1.2),
        meta={"source": "simulation/python-scripts/hand_design.py", "phalanges": 2},
    )


@register("birh_v2", doc="Prototype 2: 2 fingers opposing a thumb with abduction",
          tags=["birh", "thumb", "opposed"])
def _birh_v2() -> HandSpec:
    """Reproduces ``simulation/mujoco-models/hand.xml``: digit azimuths 90 deg, 135 deg and
    315 deg, two phalanges each, thumb with an abduction DOF."""
    return HandSpec(
        name="birh_v2",
        palm=PalmSpec(radius=0.05, thickness=0.025),
        digits=(
            _finger("finger_1", 90.0, length=0.11, ratios=(0.6, 0.4)),
            _finger("finger_2", 135.0, length=0.11, ratios=(0.6, 0.4)),
            _finger("thumb", 315.0, length=0.10, ratios=(0.6, 0.4),
                    abduction_deg=40.0, is_thumb=True),
        ),
        actuation=ActuationSpec(mode=ActuationMode.PER_JOINT, kp=6.0, force_limit=1.2),
        meta={"source": "simulation/mujoco-models/hand.xml", "phalanges": 2},
    )


@register("birh_v2_no_abduction", doc="Prototype 2 with the thumb abduction DOF removed",
          tags=["birh", "thumb", "ablation"])
def _birh_v2_no_abd() -> HandSpec:
    """The paired control for the project's central claim -- that removing abduction
    eliminates whole grasp categories (``docs/00-REVIEW.md`` A6)."""
    base = _birh_v2()
    digits = tuple(
        DigitSpec(**{**{k: getattr(d, k) for k in
                       ("name", "mount", "total_length", "length_ratios", "base_radius",
                        "tip_radius", "joints", "flexion_range", "stiffness", "damping",
                        "pad_material", "density", "is_thumb")},
                     "abduction": None})
        for d in base.digits
    )
    return HandSpec(
        name="birh_v2_no_abduction", palm=base.palm, digits=digits,
        actuation=base.actuation, meta={"ablation_of": "birh_v2", "removed": "thumb_abduction"},
    )


# ---------------------------------------------------------------------------------------
# baselines and bounds
# ---------------------------------------------------------------------------------------

@register("two_finger", doc="Two opposed fingers: the minimal-complexity baseline",
          tags=["baseline", "opposed"])
def _two_finger() -> HandSpec:
    """A two-digit opposed gripper. Without a baseline, 'effective' has no scale
    (``docs/00-REVIEW.md`` §4.6)."""
    return HandSpec(
        name="two_finger",
        palm=PalmSpec(radius=0.035, thickness=0.02),
        digits=(
            _finger("finger_1", 90.0, length=0.09, ratios=(0.55, 0.45)),
            _finger("finger_2", 270.0, length=0.09, ratios=(0.55, 0.45)),
        ),
        actuation=ActuationSpec(mode=ActuationMode.PER_JOINT, kp=6.0, force_limit=1.5),
        meta={"role": "baseline"},
    )


@register("synergy_three", doc="Three fingers driven by a single actuator",
          tags=["baseline", "underactuated", "symmetric"])
def _synergy_three() -> HandSpec:
    """The cheapest plausible hand: one motor, three adaptive fingers. The lower bound of
    the complexity axis."""
    base = _birh_v1()
    return HandSpec(
        name="synergy_three", palm=base.palm, digits=base.digits,
        actuation=ActuationSpec(
            mode=ActuationMode.SYNERGY, kp=6.0, force_limit=2.2,
            tendon_ratio=0.7, extension_stiffness=0.25,
        ),
        meta={"role": "complexity_lower_bound"},
    )


@register("four_finger", doc="Four symmetric fingers, for the digit-count sweep",
          tags=["symmetric", "sweep"])
def _four_finger() -> HandSpec:
    """Tests where the marginal return of an extra finger goes (``docs/00-REVIEW.md`` A1)."""
    ratios = (0.44, 0.33, 0.23)
    digits = tuple(
        _finger(f"finger_{i + 1}", 90.0 + 90.0 * i, length=0.11, ratios=ratios)
        for i in range(4)
    )
    return HandSpec(
        name="four_finger", palm=PalmSpec(radius=0.05, thickness=0.022), digits=digits,
        actuation=ActuationSpec(mode=ActuationMode.TENDON_COUPLED, kp=6.0, force_limit=1.2),
        meta={"role": "digit_count_sweep"},
    )


@register("anthropomorphic_ref", doc="Human-proportioned 2 fingers + opposable thumb",
          tags=["reference", "thumb"])
def _anthro() -> HandSpec:
    """Human phalanx proportions and a thumb with abduction plus base roll, as a reference
    point for 'did evolution rediscover the human hand?' (``docs/00-REVIEW.md`` §8)."""
    human = (0.47, 0.30, 0.23)
    return HandSpec(
        name="anthropomorphic_ref",
        palm=PalmSpec(radius=0.045, thickness=0.02, curvature=0.25),
        digits=(
            _finger("index", 74.0, length=0.10, ratios=human, elevation_deg=6.0, base_radius=0.010,
                    abduction_deg=15.0),
            _finger("middle", 112.0, length=0.105, ratios=human, elevation_deg=6.0, base_radius=0.010),
            _finger("thumb", 320.0, length=0.085, ratios=(0.55, 0.45), roll_deg=-55.0,
                    elevation_deg=20.0, abduction_deg=50.0, is_thumb=True),
        ),
        actuation=ActuationSpec(mode=ActuationMode.PER_JOINT, kp=6.0, force_limit=1.2),
        meta={"role": "biological_reference"},
    )
