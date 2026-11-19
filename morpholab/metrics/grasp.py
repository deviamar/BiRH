"""Analytic grasp quality: grasp matrix, friction cones, grasp wrench space, epsilon.

Implements ``docs/02-MATH.md`` §4.  Three details that are routinely omitted from reported
epsilon values, and are therefore recorded explicitly here:

* **The torque normalization length** ``lambda``.  Force and torque components of a wrench
  are not commensurable without it, and epsilon values computed with different ``lambda``
  are not comparable.  We use the standard ``lambda = max_i ||p_i - c||`` and return it.
* **The number of friction-cone edges** ``m``.  The pyramid is *inscribed* in the cone, so
  it never claims friction the cone does not provide, but a finer pyramid gives a larger
  epsilon.  Returned alongside.
* **Degeneracy.**  Fewer than 7 affinely independent primitive wrenches cannot enclose the
  origin in R^6; that is reported as ``epsilon = 0`` with a reason, not as an exception.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Sequence

import numpy as np

from ..core.logging import get_logger
from ..core.types import Array
from ..mjcf.transforms import orthonormal_basis
from ..sim.contacts import ContactSet

log = get_logger(__name__)

__all__ = ["GraspQuality", "grasp_matrix", "friction_cone_edges", "grasp_wrench_space", "analyze_grasp"]

DEFAULT_CONE_EDGES = 8


@dataclass
class GraspQuality:
    """Analytic quality of one grasp configuration."""

    epsilon: float = 0.0
    """Ferrari-Canny metric: the smallest external wrench magnitude the grasp cannot
    resist, in its weakest direction. Zero means no force closure."""

    volume: float = 0.0
    """Volume of the grasp wrench space: average-case rather than worst-case resistance.
    Reported beside epsilon because they disagree, and the disagreement identifies grasps
    that are strong on average but fragile in one direction."""

    force_closure: bool = False
    rank: int = 0
    isotropy: float = 0.0
    """``sigma_min / sigma_max`` of the grasp matrix: how uniformly the grasp can resist
    wrenches in different directions."""

    sigma_min: float = 0.0
    n_contacts: int = 0
    n_digits: int = 0
    torque_scale: float = 1.0
    cone_edges: int = DEFAULT_CONE_EDGES
    degenerate_reason: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "grasp_epsilon": self.epsilon, "grasp_volume": self.volume,
            "force_closure": bool(self.force_closure), "grasp_rank": self.rank,
            "grasp_isotropy": self.isotropy, "grasp_sigma_min": self.sigma_min,
            "grasp_torque_scale": self.torque_scale, "grasp_cone_edges": self.cone_edges,
        }


def friction_cone_edges(
    normal: Array, mu: float, edges: int = DEFAULT_CONE_EDGES, torsion: float = 0.0
) -> Array:
    """Generators of the linearized friction cone at one contact (``docs/02-MATH.md`` §3.1).

    The pyramid is inscribed in the true cone, so every generator is an admissible force.
    Circumscribing would be optimistic, and an optimistic grasp metric is worse than none.
    """
    t1, t2, n = orthonormal_basis(normal)
    angles = 2.0 * np.pi * np.arange(edges) / edges
    return n[None, :] + mu * (np.cos(angles)[:, None] * t1[None, :] + np.sin(angles)[:, None] * t2[None, :])


def grasp_matrix(contacts: ContactSet, com: Array | None = None, torque_scale: float | None = None) -> tuple[Array, float]:
    """Grasp matrix ``G`` in ``R^{6 x 3n}`` and the torque scale used.

    Column block ``i`` maps a contact force at ``p_i`` to the wrench it applies about the
    object's centre of mass.
    """
    if len(contacts) == 0:
        return np.zeros((6, 0)), 1.0
    c = np.asarray(com if com is not None else contacts.object_com, float)
    positions = contacts.positions()
    arms = positions - c[None, :]
    scale = float(torque_scale if torque_scale is not None else max(np.linalg.norm(arms, axis=1).max(), 1e-6))

    blocks = []
    for r in arms:
        skew = np.array([[0.0, -r[2], r[1]], [r[2], 0.0, -r[0]], [-r[1], r[0], 0.0]])
        blocks.append(np.vstack([np.eye(3), skew / scale]))
    return np.hstack(blocks), scale


def grasp_wrench_space(
    contacts: ContactSet,
    *,
    edges: int = DEFAULT_CONE_EDGES,
    com: Array | None = None,
    torsion: bool = False,
) -> tuple[Array, float]:
    """Primitive wrenches spanning the grasp wrench space, and the torque scale.

    Uses the unit-total-normal-force convention, so the hull is the set of wrenches the
    grasp can apply while its contacts share one unit of normal force.
    """
    if len(contacts) == 0:
        return np.zeros((0, 6)), 1.0
    c = np.asarray(com if com is not None else contacts.object_com, float)
    positions = contacts.positions()
    arms = positions - c[None, :]
    scale = float(max(np.linalg.norm(arms, axis=1).max(), 1e-6))

    wrenches: list[Array] = []
    for contact, arm in zip(contacts, arms):
        for f in friction_cone_edges(contact.normal, contact.friction, edges):
            wrenches.append(np.concatenate([f, np.cross(arm, f) / scale]))
        if torsion and contact.friction > 0:
            # Soft-finger contact: torsional friction about the normal.
            mu_t = max(contact.friction * 0.1, 1e-4)
            for sign in (1.0, -1.0):
                wrenches.append(np.concatenate([np.zeros(3), sign * mu_t * contact.normal / scale]))
    return np.asarray(wrenches, float), scale


def analyze_grasp(
    contacts: ContactSet,
    *,
    edges: int = DEFAULT_CONE_EDGES,
    com: Array | None = None,
) -> GraspQuality:
    """Compute every analytic grasp quality measure from a contact set."""
    n = len(contacts)
    result = GraspQuality(n_contacts=n, n_digits=contacts.n_digits, cone_edges=edges)
    if n == 0:
        result.degenerate_reason = "no contacts"
        return result

    G, scale = grasp_matrix(contacts, com=com)
    result.torque_scale = scale
    singular = np.linalg.svd(G, compute_uv=False)
    result.rank = int(np.sum(singular > max(singular[0], 1e-12) * 1e-9))
    result.sigma_min = float(singular[-1]) if singular.size >= 6 else 0.0
    result.isotropy = float(singular[-1] / singular[0]) if (singular.size >= 6 and singular[0] > 0) else 0.0

    if result.rank < 6:
        result.degenerate_reason = (
            f"grasp matrix has rank {result.rank} < 6: some wrench direction is "
            "uncontrollable regardless of friction"
        )
        return result

    wrenches, _ = grasp_wrench_space(contacts, edges=edges, com=com)
    eps, vol, closure, reason = _hull_metrics(wrenches)
    result.epsilon, result.volume, result.force_closure = eps, vol, closure
    result.degenerate_reason = reason
    return result


def _hull_metrics(wrenches: Array) -> tuple[float, float, bool, str]:
    """Epsilon, volume and force closure from the convex hull of primitive wrenches."""
    from scipy.spatial import ConvexHull, QhullError

    if wrenches.shape[0] < 7:
        return 0.0, 0.0, False, f"only {wrenches.shape[0]} primitive wrenches; 7 are needed in R^6"

    points = np.vstack([wrenches, np.zeros((1, 6))])
    try:
        hull = ConvexHull(points, qhull_options="QJ")
    except (QhullError, ValueError) as exc:
        return 0.0, 0.0, False, f"degenerate wrench set: {str(exc).splitlines()[0][:80]}"

    # Facet form a.w <= -b (qhull stores [a | b] with a.x + b <= 0 inside).
    a = hull.equations[:, :6]
    b = hull.equations[:, 6]
    norms = np.linalg.norm(a, axis=1)
    good = norms > 1e-12
    if not np.any(good):
        return 0.0, 0.0, False, "degenerate hull"
    distances = -b[good] / norms[good]

    closure = bool(np.all(distances > 1e-9))
    epsilon = float(distances.min()) if closure else 0.0
    return epsilon, float(hull.volume), closure, "" if closure else "origin is outside the grasp wrench space"


# ---------------------------------------------------------------------------------------
# reference cases, used by tests and by calibration
# ---------------------------------------------------------------------------------------

def antipodal_reference(mu: float = 1.0, radius: float = 0.05, edges: int = DEFAULT_CONE_EDGES) -> float:
    """Epsilon for an ideal two-contact antipodal grasp on a sphere.

    Two opposed point contacts cannot resist torque about the line joining them, so the
    grasp matrix is rank 5 and epsilon is exactly zero.  A metric implementation that
    returns anything else here is wrong, which makes this the sharpest available check.
    """
    from ..sim.contacts import Contact, ContactSet

    contacts = ContactSet(
        contacts=(
            Contact(pos=np.array([radius, 0, 0]), normal=np.array([-1.0, 0, 0]),
                    tangent1=np.array([0, 1.0, 0]), tangent2=np.array([0, 0, 1.0]),
                    force=np.zeros(3), normal_force=1.0, tangential_force=0.0,
                    friction=mu, distance=0.0, hand_geom=0, object_geom=1, digit="a"),
            Contact(pos=np.array([-radius, 0, 0]), normal=np.array([1.0, 0, 0]),
                    tangent1=np.array([0, 1.0, 0]), tangent2=np.array([0, 0, 1.0]),
                    force=np.zeros(3), normal_force=1.0, tangential_force=0.0,
                    friction=mu, distance=0.0, hand_geom=2, object_geom=1, digit="b"),
        ),
        object_com=np.zeros(3),
    )
    return analyze_grasp(contacts, edges=edges).epsilon


def symmetric_three_reference(mu: float = 1.0, radius: float = 0.05, edges: int = DEFAULT_CONE_EDGES) -> GraspQuality:
    """Three contacts at 120 degrees on a sphere's equator -- the BiRH v1 configuration.

    Planar and therefore *not* force closed on its own: nothing resists a wrench pushing
    the object out of the contact plane... except friction, which with mu >= ~0.6 does.
    A useful, non-trivial reference case.
    """
    from ..sim.contacts import Contact, ContactSet

    items = []
    for k in range(3):
        theta = 2 * np.pi * k / 3
        p = np.array([radius * np.cos(theta), radius * np.sin(theta), 0.0])
        items.append(
            Contact(pos=p, normal=-p / radius, tangent1=np.array([0, 0, 1.0]),
                    tangent2=np.cross(-p / radius, np.array([0, 0, 1.0])),
                    force=np.zeros(3), normal_force=1.0, tangential_force=0.0,
                    friction=mu, distance=0.0, hand_geom=k, object_geom=99, digit=f"d{k}")
        )
    return analyze_grasp(ContactSet(tuple(items), object_com=np.zeros(3)), edges=edges)
