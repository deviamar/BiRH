"""Approach planning: where to put the palm, and how far to pre-close the digits.

A fixed standoff cannot work across a design space.  A hand with 15 cm fingers and one
with 7 cm fingers converge at completely different depths below the palm, and placing both
at "3 cm above the object" would compare a grasp against a near-miss.  Worse, it would
attribute the difference to morphology, which is exactly the error the platform exists to
avoid.

So the approach is *computed* from the morphology, using closed-form forward kinematics of
the digit (``docs/02-MATH.md`` §2.2):

1. Sweep the digits' flexion fraction ``f`` from 0 to 1 and trace the fingertip through the
   ``(radial, depth)`` plane -- ``radial`` from the palm axis, ``depth`` below the palm's
   grasping surface.
2. Find ``f*``, the first fraction at which the fingertip radius reaches the object's
   half-width: that is where the digits would close *on* the object.
3. Place the palm so the object's centre sits at ``depth(f*)``.
4. **Pre-shape.**  The arc's deepest point usually lies at a middling flexion, so a digit
   closing from fully open would sweep *below* the object and hit the support surface
   first -- which is what makes long-fingered hands appear to fail at small objects on a
   table when the real problem is the approach.  So the digits start partly closed, at the
   largest fraction from which the remaining arc stays clear of the support.

This is a deliberately simple planner.  It is not the object of study -- it exists so that
every morphology is given a fair, comparable attempt, which is a precondition for the
comparison meaning anything.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ..core.logging import get_logger
from ..core.types import Pose, Quat, Vec3
from ..mjcf.transforms import quat_from_axis_angle, quat_from_matrix, quat_multiply
from ..morphology.spec import DigitSpec, HandSpec
from ..objects.spec import ObjectSpec

log = get_logger(__name__)

__all__ = ["ApproachPlan", "digit_arc", "plan_approach"]

_SAMPLES = 121


@dataclass(frozen=True)
class ApproachPlan:
    """Where the palm starts and how closed the digits start."""

    pose: Pose
    preshape: float = 0.0
    """Flexion fraction the digits hold before closing, in ``[0, 1)``."""

    grasp_depth: float = 0.0
    """Depth below the palm surface at which the digits converge on the object."""

    reachable: bool = True
    """False when no flexion brings the fingertips within the object's half-width -- the
    hand cannot close on this object, which is a morphological fact worth recording rather
    than a reason to move the palm somewhere arbitrary."""

    note: str = ""

    def to_dict(self) -> dict:
        return {
            "pose": self.pose.to_dict(), "preshape": self.preshape,
            "grasp_depth": self.grasp_depth, "reachable": self.reachable, "note": self.note,
        }


def digit_arc(digit: DigitSpec, palm_radius: float, samples: int = _SAMPLES) -> tuple[np.ndarray, ...]:
    """Trace a digit's fingertip through flexion.

    Returns ``(f, radial, depth)``: the flexion fraction, the fingertip's distance from the
    palm axis, and its depth below the palm surface.  All joints move together as a
    fraction of their range, which is the posture family both the ramp controller and the
    tendon coupling actually produce.

    In the mount frame the digit extends along ``+x`` and curls toward ``+z``, so after
    cumulative flexion ``Theta_j`` the ``j``-th phalanx points along
    ``(cos Theta_j, sin Theta_j)`` in the ``(radial, depth)`` plane.
    """
    f = np.linspace(0.0, 1.0, samples)
    lengths = np.asarray(digit.phalanx_lengths, float)
    joints = digit.flexion_joints
    lo = np.array([j.range[0] for j in joints], float)
    hi = np.array([j.range[1] for j in joints], float)

    angles = lo[None, :] + f[:, None] * (hi - lo)[None, :]
    theta = np.cumsum(angles, axis=1)
    # Tip in the digit's own flexion plane: ``a`` along the digit axis, ``b`` toward the
    # palm normal.
    a = (lengths[None, :] * np.cos(theta)).sum(axis=1)
    b = (lengths[None, :] * np.sin(theta)).sum(axis=1)

    # Rotate that plane out through the mount's roll and elevation
    # (``docs/02-MATH.md`` §2.1). A rolled thumb flexes *sideways*, so its fingertip does
    # not approach the palm axis at all; treating every digit as if it curled straight
    # inward mis-places the palm for any opposed hand.
    psi, phi = digit.mount.roll, digit.mount.elevation
    x = a * math.cos(phi) - b * math.cos(psi) * math.sin(phi)
    y = -b * math.sin(psi)
    depth = a * math.sin(phi) + b * math.cos(psi) * math.cos(phi)

    mount_r = palm_radius * digit.mount.radius_frac
    radial = np.hypot(mount_r + x, y)
    return f, radial, depth


def _hand_arc(hand: HandSpec, samples: int = _SAMPLES) -> tuple[np.ndarray, ...]:
    """The arc of the digits that actually have to meet.

    A grasp needs at least two opposing digits, so the limiting radius at each flexion
    fraction is the **second smallest** across digits -- not the smallest (which one lucky
    finger could satisfy alone) and not the largest (which a rolled thumb, whose fingertip
    never approaches the palm axis, would dominate).  Depth is the shallowest, because the
    palm must be placed where the *shallowest* digit can still reach the object.
    """
    arcs = [digit_arc(d, hand.palm.radius, samples) for d in hand.digits]
    f = arcs[0][0]
    # The digit's *inner surface* touches the object, not its centreline, so the contact
    # radius is the fingertip radius less the link's own thickness. Ignoring this makes
    # every thin object look unreachable by roughly one finger diameter.
    per_digit = np.vstack([a[1] - d.tip_radius for a, d in zip(arcs, hand.digits)])
    k = min(1, per_digit.shape[0] - 1)
    radial = np.sort(per_digit, axis=0)[k]
    depth = np.min([a[2] for a in arcs], axis=0)
    return f, np.maximum(radial, 0.0), depth


def plan_approach(
    hand: HandSpec,
    obj: ObjectSpec,
    *,
    direction: str = "top",
    object_pose: Pose | None = None,
    support_z: float = 0.0,
    clearance: float = 0.006,
    azimuth: float = 0.0,
    standoff: float = 0.0,
) -> ApproachPlan:
    """Compute the palm pose and pre-shape for one (hand, object, direction)."""
    object_pose = object_pose or Pose(pos=(0.0, 0.0, obj.rest_height), quat=obj.spawn_quat)
    lo, hi = obj.aabb
    half_width = max(obj.width / 2.0, 1e-3)

    f, radial, depth = _hand_arc(hand)
    # Where the fingertips first come within the object's half-width.
    inside = np.flatnonzero(radial <= half_width)
    if inside.size:
        k = int(inside[0])
        reachable, note = True, ""
    else:
        k = int(np.argmin(radial))
        reachable = False
        note = (
            f"fingertips close to {radial[k] * 100:.1f} cm from the palm axis but the object "
            f"half-width is {half_width * 100:.1f} cm: the digits cannot converge on it"
        )
    grasp_depth = float(max(depth[k], 0.005))

    if direction == "top":
        object_centre_z = float(object_pose.pos[2] + (lo[2] + hi[2]) / 2.0)
        object_top_z = float(object_pose.pos[2] + hi[2])
        # The palm must clear the object. Without this clamp a tall object (a bottle) puts
        # the palm *inside* it: the hand then presses the object into the table through the
        # wrist weld, producing contact forces of hundreds of newtons that no actuator
        # generated. When the clamp binds, the digits grasp the object's upper region --
        # which is what a top-down grasp of a bottle actually does.
        palm_surface_z = max(
            object_centre_z + grasp_depth, object_top_z + clearance
        ) + standoff

        # Pre-shape: start at the largest flexion fraction from which the remaining arc
        # (up to f*) never dips below the support.
        allowed_depth = palm_surface_z - support_z - clearance
        preshape = _preshape_for(f, depth, k, allowed_depth)

        # Palm normal (+z local) must point down at the object.
        quat: Quat = quat_multiply(
            quat_from_axis_angle((0.0, 0.0, 1.0), azimuth),
            quat_from_axis_angle((1.0, 0.0, 0.0), math.pi),
        )
        # The palm body origin sits half a thickness *behind* its grasping surface.
        palm_z = palm_surface_z + hand.palm.thickness / 2.0
        return ApproachPlan(
            pose=Pose(pos=(float(object_pose.pos[0]), float(object_pose.pos[1]), palm_z), quat=quat),
            preshape=preshape, grasp_depth=grasp_depth, reachable=reachable, note=note,
        )

    if direction in ("side", "front"):
        yaw = azimuth if direction == "side" else azimuth + math.pi / 2
        object_centre_z = float(object_pose.pos[2] + (lo[2] + hi[2]) / 2.0)
        # Grasp the object at a height the digits can actually enclose: for a tall object,
        # its mid-height; for a short one, whatever is above the support.
        grasp_z = max(object_centre_z, support_z + half_width + clearance)

        z_axis = np.array([-math.cos(yaw), -math.sin(yaw), 0.0])   # palm normal, inward
        x_axis = np.array([0.0, 0.0, 1.0])                          # digits extend upward
        y_axis = np.cross(z_axis, x_axis)
        quat = quat_from_matrix(np.column_stack([x_axis, y_axis, z_axis]))

        radius = grasp_depth + standoff + hand.palm.thickness / 2.0

        # A side approach sweeps in a *vertical* plane, so the digits mounted on the lower
        # half of the palm dive toward the support exactly as a top approach's digits do.
        # The clearance available is the palm's height above the support, less the palm's
        # own radius (the mount points of the lowest digits).
        allowed_depth = grasp_z - support_z - hand.palm.radius - clearance
        preshape = _preshape_for(f, depth, k, allowed_depth)
        return ApproachPlan(
            pose=Pose(
                pos=(
                    float(object_pose.pos[0] + radius * math.cos(yaw)),
                    float(object_pose.pos[1] + radius * math.sin(yaw)),
                    grasp_z,
                ),
                quat=quat,
            ),
            preshape=preshape, grasp_depth=grasp_depth, reachable=reachable, note=note,
        )

    raise ValueError(f"unknown approach direction {direction!r}; use top, side or front")


def _preshape_for(
    f: np.ndarray, depth: np.ndarray, target: int, allowed_depth: float
) -> float:
    """Largest starting fraction whose remaining arc stays above the support.

    Scans backwards from ``f*``: the answer is just past the last violation, so the digits
    start as open as they can while still clearing the table on their way in.
    """
    if allowed_depth <= 0:
        return float(f[target])
    for i in range(target, -1, -1):
        if depth[i] > allowed_depth:
            return float(min(f[min(i + 1, target)], 0.95))
    return 0.0


def describe_plan(hand: HandSpec, obj: ObjectSpec, plan: ApproachPlan) -> str:
    return (
        f"{hand.name} -> {obj.name}: palm at z={plan.pose.pos[2] * 100:.1f} cm, "
        f"grasp depth {plan.grasp_depth * 100:.1f} cm, preshape {plan.preshape:.2f}"
        + ("" if plan.reachable else f"  [UNREACHABLE: {plan.note}]")
    )
