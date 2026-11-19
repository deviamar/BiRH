"""Mechanical complexity: the axis the project's premise depends on.

"Simpler but still effective" is the whole thesis, yet simplicity was never quantified
(``docs/00-REVIEW.md`` §4.7).  Without a number, the v1 -> v2 redesign could be -- and was
-- presented as a refinement while actually increasing complexity by a third.

The cost model (``docs/02-MATH.md`` §6.3) is deliberately explicit, editable and
*reported with every result*, because a Pareto front is only meaningful relative to a
stated cost model.  The weights below are engineering judgement, not measurements: an
actuator is the dominant unit cost because a motor plus driver plus encoder plus wiring
dominates a 3D-printed link, and a study that disagrees should change the weights and say
so rather than argue about the conclusion.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from ..morphology.spec import ActuationMode, HandSpec

__all__ = ["ComplexityWeights", "complexity", "complexity_breakdown", "DEFAULT_WEIGHTS"]


@dataclass(frozen=True)
class ComplexityWeights:
    """Unit costs. Editable configuration, never buried in code."""

    actuator: float = 1.0
    joint: float = 0.6
    digit: float = 0.4
    phalanx: float = 0.15
    abduction: float = 0.3
    """Extra cost of an abduction mechanism beyond its joint and actuator: it needs an
    orthogonal axis at the base, which is the hardest part of the assembly to make stiff."""

    tendon_route: float = 0.5
    """Per digit, for tendon routing: pulleys, sheaths, tensioning and maintenance."""

    compliant_element: float = 0.2
    """Per compliant surface (compliant palm or pad): a real but modest manufacturing cost."""

    def to_dict(self) -> dict[str, float]:
        return dict(vars(self))


DEFAULT_WEIGHTS = ComplexityWeights()


def complexity_breakdown(hand: HandSpec, weights: ComplexityWeights | None = None) -> dict[str, float]:
    """Per-term contributions, so a total can always be explained."""
    w = weights or DEFAULT_WEIGHTS
    n_tendon_routes = hand.n_digits if hand.actuation.is_tendon else 0
    compliant = sum(
        1
        for name in ({hand.palm.pad_material, hand.palm.material} | {d.pad_material for d in hand.digits})
        if _is_compliant(hand, name)
    )
    return {
        "actuators": w.actuator * hand.n_actuators,
        "joints": w.joint * hand.n_joints,
        "digits": w.digit * hand.n_digits,
        "phalanges": w.phalanx * hand.n_phalanges,
        "abduction": w.abduction * hand.n_abduction,
        "tendon_routes": w.tendon_route * n_tendon_routes,
        "compliant_elements": w.compliant_element * compliant,
    }


def _is_compliant(hand: HandSpec, material_name: str) -> bool:
    try:
        mat = hand.material(material_name)
    except KeyError:
        return False
    return mat.solver.time_const > 0.03 or mat.solver.dmin < 0.7


def complexity(hand: HandSpec, weights: ComplexityWeights | None = None) -> float:
    """Total mechanical complexity of a hand."""
    return float(sum(complexity_breakdown(hand, weights).values()))


def compare(*hands: HandSpec, weights: ComplexityWeights | None = None) -> str:
    """A side-by-side table, used by ``morpholab morphology show`` and by reports."""
    rows = []
    keys = list(complexity_breakdown(hands[0], weights))
    width = max(len(h.name) for h in hands)
    header = f"{'hand':<{width}}  " + "  ".join(f"{k[:9]:>9}" for k in keys) + f"  {'TOTAL':>7}"
    rows.append(header)
    rows.append("-" * len(header))
    for hand in hands:
        parts = complexity_breakdown(hand, weights)
        rows.append(
            f"{hand.name:<{width}}  "
            + "  ".join(f"{parts[k]:>9.2f}" for k in keys)
            + f"  {sum(parts.values()):>7.2f}"
        )
    return "\n".join(rows)
