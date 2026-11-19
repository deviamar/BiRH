"""The registered metric set: contact structure, effort, complexity, grasp quality, robustness.

Grouped in one module because each metric is a handful of lines and the value is in seeing
them side by side -- what is free, what costs simulation, and what each one is for.
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np

from ..core.types import Phase
from ..sim.contacts import ContactSet
from .base import MetricContext, register_metric
from .complexity import complexity, complexity_breakdown

__all__ = ["contact_metrics", "effort_metrics", "complexity_metrics"]


# ---------------------------------------------------------------------------------------
# free metrics: arithmetic on the spec or the trace
# ---------------------------------------------------------------------------------------

@register_metric(
    "outcome",
    keys=("lift_height", "slip_distance", "slip_rotation", "final_object_speed"),
    doc="Task outcome quantities read off the trace",
)
def outcome_metrics(ctx: MetricContext) -> dict[str, Any]:
    trace = ctx.trace
    return {
        "lift_height": trace.final("lift_height"),
        "peak_lift_height": trace.peak("lift_height"),
        "slip_distance": trace.final("slip_distance"),
        "max_slip_distance": trace.peak("slip_distance"),
        "slip_rotation": trace.final("slip_rotation"),
        "max_slip_rotation": trace.peak("slip_rotation"),
        "final_object_speed": trace.final("object_speed"),
    }


@register_metric(
    "contact",
    keys=("contact_count", "contact_digits", "contact_spread", "opposition", "palm_force_fraction"),
    doc="Contact count, distribution and opposition at the moment of grasp",
)
def contact_metrics(ctx: MetricContext) -> dict[str, Any]:
    """Contact-distribution measures (``docs/02-MATH.md`` §4.3).

    ``contact_spread`` is the resultant-length statistic from directional statistics: zero
    when every contact is on one side of the object (a push, not a grasp) and approaching
    one for well-distributed enclosure.  It is rotation-invariant and needs no reference
    axis, which is why it is used in preference to any per-axis measure.
    """
    contacts: ContactSet | None = ctx.grasp_contacts or ctx.trace.contact_sets.get("final")
    if contacts is None or len(contacts) == 0:
        return {
            "contact_count": 0, "contact_digits": 0, "contact_spread": 0.0,
            "opposition": 0.0, "palm_force_fraction": 0.0, "min_slip_margin": 0.0,
            "grasp_normal_force": 0.0,
        }

    positions = contacts.positions()
    com = np.asarray(contacts.object_com, float)
    arms = positions - com[None, :]
    norms = np.linalg.norm(arms, axis=1, keepdims=True)
    unit = arms / np.maximum(norms, 1e-9)
    spread = float(1.0 - np.linalg.norm(unit.mean(axis=0)))

    normals = contacts.normals()
    if len(normals) >= 2:
        gram = normals @ normals.T
        np.fill_diagonal(gram, 1.0)
        opposition = float(-gram.min())
    else:
        opposition = 0.0

    return {
        "contact_count": contacts.count,
        "contact_digits": contacts.n_digits,
        "contact_spread": spread,
        "opposition": opposition,
        "palm_force_fraction": contacts.palm_force_fraction,
        "min_slip_margin": contacts.min_slip_margin,
        "grasp_normal_force": contacts.total_normal_force,
    }


@register_metric(
    "effort",
    keys=("energy", "peak_torque", "mean_joint_utilization", "peak_normal_force"),
    doc="Mechanical work, peak torque and joint utilization",
)
def effort_metrics(ctx: MetricContext) -> dict[str, Any]:
    """Effort measures (``docs/02-MATH.md`` §6.1).

    ``mean_joint_utilization`` is the cheapest available evidence for a *removable* DOF:
    an actuator that is never worked hard is a candidate for deletion, which is the
    project's central question restated as an arithmetic one.
    """
    trace = ctx.trace
    t = trace.t
    power = trace.series.get("mech_power")
    energy = float(np.trapezoid(power, t)) if (power is not None and t.size > 1) else 0.0
    limit = max(ctx.hand.actuation.force_limit, 1e-9)
    torque = trace.series.get("joint_torque_max", np.zeros(0))
    return {
        "energy": energy,
        "peak_torque": trace.peak("joint_torque_max"),
        "mean_torque": float(np.mean(np.abs(torque))) if torque.size else 0.0,
        "mean_joint_utilization": float(np.mean(np.abs(torque)) / limit) if torque.size else 0.0,
        "peak_actuator_force": trace.peak("actuator_force_max"),
        "peak_normal_force": trace.peak("normal_force"),
        "median_normal_force": float(np.median(trace.series["normal_force"]))
        if "normal_force" in trace.series and trace.series["normal_force"].size else 0.0,
        "peak_self_collision_force": trace.peak("self_collision_force"),
    }


@register_metric(
    "complexity",
    needs=("spec",),
    keys=("complexity", "n_actuators", "n_joints", "n_digits"),
    doc="Mechanical complexity cost model, from the spec alone",
)
def complexity_metrics(ctx: MetricContext) -> dict[str, Any]:
    hand = ctx.hand
    row = {
        "complexity": complexity(hand),
        "n_actuators": hand.n_actuators,
        "n_joints": hand.n_joints,
        "n_digits": hand.n_digits,
        "n_phalanges": hand.n_phalanges,
        "n_abduction": hand.n_abduction,
        "actuation_mode": hand.actuation.mode,
        "hand_mass": hand.mass,
        "palm_radius": hand.palm.radius,
        "digit_length": hand.digits[0].total_length,
        "max_aperture": hand.max_aperture,
    }
    row.update({f"complexity_{k}": v for k, v in complexity_breakdown(hand).items()})
    return row


@register_metric(
    "force_plausibility",
    keys=("force_plausibility",),
    doc="Whether observed contact forces are consistent with the actuator limits",
)
def force_plausibility(ctx: MetricContext) -> dict[str, Any]:
    """A self-check on the simulation, not a property of the hand.

    Contact forces far above what the actuators could generate mean the object is being
    reacted against something else -- pinned to the table, wedged inside a hole, or
    penetrating.  Recording the ratio turns a class of silent modelling error into a
    filterable column instead of a mysterious outlier (``docs/02-MATH.md`` §11).
    """
    hand = ctx.hand
    # Crude upper bound: every digit applies its proximal-joint torque at the fingertip.
    lever = max(min(d.total_length for d in hand.digits), 1e-3)
    ceiling = hand.n_digits * hand.actuation.force_limit / lever
    peak = ctx.trace.peak("normal_force")
    ratio = float(peak / max(ceiling, 1e-9))
    return {
        "force_ceiling": float(ceiling),
        "force_plausibility": ratio,
        "force_implausible": bool(ratio > 3.0),
    }


# ---------------------------------------------------------------------------------------
# analytic grasp quality (free: computed from the stored contact set)
# ---------------------------------------------------------------------------------------

@register_metric(
    "grasp_quality",
    keys=("grasp_epsilon", "grasp_volume", "force_closure", "grasp_isotropy"),
    doc="Ferrari-Canny epsilon, grasp wrench space volume, force closure, isotropy",
)
def grasp_quality_metrics(ctx: MetricContext) -> dict[str, Any]:
    from .grasp import analyze_grasp

    contacts = ctx.grasp_contacts or ctx.trace.contact_sets.get("final")
    if contacts is None or len(contacts) == 0:
        from .grasp import GraspQuality

        return GraspQuality().to_dict()
    return analyze_grasp(contacts, edges=int(ctx.options.get("cone_edges", 8))).to_dict()


# ---------------------------------------------------------------------------------------
# simulation-based metrics
# ---------------------------------------------------------------------------------------

@register_metric(
    "robustness",
    needs=("trace", "simulator"),
    keys=("disturbance_worst", "disturbance_mean", "gravity_multiplier"),
    cost="sim",
    doc="Disturbance tolerance by constant-wrench bisection from the grasp snapshot",
)
def robustness_metrics(ctx: MetricContext) -> dict[str, Any]:
    """The empirical counterpart of epsilon (``docs/02-MATH.md`` §5.1).

    Costs roughly ``directions x 12`` short simulations, all restarted from the same
    snapshot, so it is bounded and parallel-safe -- unlike the unbounded ramp it replaces.
    """
    from ..sim.perturb import PerturbationOptions, disturbance_tolerance

    snapshot = ctx.grasp_snapshot
    rollout = ctx.rollout
    if snapshot is None or rollout is None:
        return {}
    if not ctx.trace.success and ctx.trace.final("contact_digits") < 2:
        return {}  # nothing to disturb; a zero here would be indistinguishable from a real zero

    options = ctx.options.get("perturbation") or PerturbationOptions()
    result = disturbance_tolerance(
        rollout.model, rollout.data, snapshot,
        palm_body=rollout.ctx.palm_body_id,
        object_body=rollout.ctx.object_body_id,
        options=options,
    )
    return result.to_dict()
