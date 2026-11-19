"""Contact extraction.

MuJoCo's contact array is transient, index-based and re-ordered every step.  Everything
downstream -- grasp matrices, force closure, contact-distribution metrics, overlays --
needs a stable, named, physically complete description instead, which is what this module
produces.

The contact *frame* convention matters and is easy to get wrong: ``contact.frame`` is a
row-major 3x3 whose **first row is the normal**, pointing from ``geom1`` toward ``geom2``.
:func:`extract_contacts` re-orients every contact so its normal points *into the object*,
so callers never have to reason about which geom happened to be listed first.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Iterable, Sequence

import numpy as np

from ..core.types import Array

if TYPE_CHECKING:  # pragma: no cover
    import mujoco

__all__ = ["Contact", "ContactSet", "extract_contacts"]


@dataclass(frozen=True)
class Contact:
    """One hand-object contact, in world coordinates."""

    pos: Array
    normal: Array
    """Unit normal pointing *into the object* (i.e. the direction the hand pushes)."""

    tangent1: Array
    tangent2: Array
    force: Array
    """Contact force in world coordinates, applied to the object."""

    normal_force: float
    tangential_force: float
    friction: float
    """Sliding friction coefficient at this contact."""

    distance: float
    hand_geom: int
    object_geom: int
    digit: str
    """Digit name, ``"palm"``, or ``""`` if the contact is not hand-object."""

    phalanx: int = -1
    is_pad: bool = False

    @property
    def slip_margin(self) -> float:
        """How far inside the friction cone this contact is, in ``[0, 1]``.

        ``1`` means purely normal loading; ``0`` means on the verge of sliding.  This is
        the per-contact quantity that predicts *where* a grasp will fail, which a scalar
        grasp metric cannot tell you.
        """
        limit = self.friction * self.normal_force
        if limit <= 1e-9:
            return 0.0
        return float(max(0.0, 1.0 - self.tangential_force / limit))


@dataclass
class ContactSet:
    """All hand-object contacts at one instant, with aggregate queries."""

    contacts: tuple[Contact, ...] = ()
    object_com: Array = None  # type: ignore[assignment]
    palm_pos: Array = None  # type: ignore[assignment]
    time: float = 0.0

    def __len__(self) -> int:
        return len(self.contacts)

    def __iter__(self):
        return iter(self.contacts)

    @property
    def count(self) -> int:
        return len(self.contacts)

    @property
    def digits(self) -> tuple[str, ...]:
        """Distinct digits in contact, excluding the palm.

        Three contacts from one finger is not a three-finger grasp; counting contacts
        alone cannot tell the difference (``docs/02-MATH.md`` §4.3).
        """
        return tuple(sorted({c.digit for c in self.contacts if c.digit and c.digit != "palm"}))

    @property
    def n_digits(self) -> int:
        return len(self.digits)

    @property
    def total_normal_force(self) -> float:
        return float(sum(c.normal_force for c in self.contacts))

    @property
    def palm_force_fraction(self) -> float:
        """Share of normal force carried by the palm: separates power from fingertip
        grasps quantitatively, which is what AHAP grasp-type classification needs."""
        total = self.total_normal_force
        if total <= 1e-9:
            return 0.0
        palm = sum(c.normal_force for c in self.contacts if c.digit == "palm")
        return float(palm / total)

    @property
    def min_slip_margin(self) -> float:
        return float(min((c.slip_margin for c in self.contacts), default=0.0))

    def positions(self) -> Array:
        return np.array([c.pos for c in self.contacts]) if self.contacts else np.zeros((0, 3))

    def normals(self) -> Array:
        return np.array([c.normal for c in self.contacts]) if self.contacts else np.zeros((0, 3))

    def frictions(self) -> Array:
        return np.array([c.friction for c in self.contacts]) if self.contacts else np.zeros(0)

    def normal_forces(self) -> Array:
        return np.array([c.normal_force for c in self.contacts]) if self.contacts else np.zeros(0)

    def by_digit(self) -> dict[str, list[Contact]]:
        out: dict[str, list[Contact]] = {}
        for c in self.contacts:
            out.setdefault(c.digit, []).append(c)
        return out

    def summary(self) -> dict[str, Any]:
        return {
            "contact_count": self.count,
            "contact_digits": self.n_digits,
            "total_normal_force": self.total_normal_force,
            "palm_force_fraction": self.palm_force_fraction,
            "min_slip_margin": self.min_slip_margin,
        }


def extract_contacts(
    model: "mujoco.MjModel",
    data: "mujoco.MjData",
    *,
    object_geoms: Iterable[int],
    hand_geoms: Iterable[int],
    digit_of_geom: dict[int, tuple[str, int, bool]] | None = None,
    object_body: int | None = None,
    palm_body: int | None = None,
    min_force: float = 1e-6,
) -> ContactSet:
    """Collect hand-object contacts into a :class:`ContactSet`.

    Parameters
    ----------
    digit_of_geom:
        ``geom id -> (digit name, phalanx index, is_pad)``.  Built once by the engine from
        the scene's name index, so contacts are attributable without string parsing.
    min_force:
        Contacts weaker than this are discarded.  MuJoCo reports contacts within the
        collision margin that carry no force; counting them would inflate every
        contact-count metric.
    """
    import mujoco

    object_geoms = frozenset(int(g) for g in object_geoms)
    hand_geoms = frozenset(int(g) for g in hand_geoms)
    digit_of_geom = digit_of_geom or {}

    com = (
        np.asarray(data.xipos[object_body], float).copy()
        if object_body is not None and object_body >= 0
        else np.zeros(3)
    )
    palm = (
        np.asarray(data.xpos[palm_body], float).copy()
        if palm_body is not None and palm_body >= 0
        else np.zeros(3)
    )

    out: list[Contact] = []
    buf = np.zeros(6)
    for i in range(data.ncon):
        c = data.contact[i]
        g1, g2 = int(c.geom1), int(c.geom2)

        if g1 in hand_geoms and g2 in object_geoms:
            hand_geom, obj_geom, flip = g1, g2, False
        elif g2 in hand_geoms and g1 in object_geoms:
            hand_geom, obj_geom, flip = g2, g1, True
        else:
            continue

        mujoco.mj_contactForce(model, data, i, buf)
        fn = float(buf[0])
        if abs(fn) < min_force:
            continue

        frame = np.asarray(c.frame, float).reshape(3, 3)
        normal, t1, t2 = frame[0].copy(), frame[1].copy(), frame[2].copy()
        # frame[0] points from geom1 to geom2; we want it pointing into the object.
        if flip:
            normal, t1 = -normal, -t1

        # Contact force in the contact frame is (normal, tangent1, tangent2); rotate it
        # into world coordinates, with the sign convention matching the flipped normal.
        f_local = np.array([buf[0], buf[1], buf[2]], float)
        f_world = f_local[0] * normal + f_local[1] * t1 + f_local[2] * t2
        ft = float(np.hypot(f_local[1], f_local[2]))

        digit, phalanx, is_pad = digit_of_geom.get(hand_geom, ("", -1, False))
        out.append(
            Contact(
                pos=np.asarray(c.pos, float).copy(),
                normal=normal, tangent1=t1, tangent2=t2,
                force=f_world, normal_force=abs(fn), tangential_force=ft,
                friction=float(c.friction[0]), distance=float(c.dist),
                hand_geom=hand_geom, object_geom=obj_geom,
                digit=digit, phalanx=phalanx, is_pad=is_pad,
            )
        )

    return ContactSet(tuple(out), object_com=com, palm_pos=palm, time=float(data.time))


def build_geom_lookup(index: Any, hand_spec: Any, model: "mujoco.MjModel") -> dict[int, tuple[str, int, bool]]:
    """``geom id -> (digit, phalanx, is_pad)`` for every hand geom in a scene."""
    import mujoco

    lookup: dict[int, tuple[str, int, bool]] = {}

    def gid(name: str) -> int:
        return int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name))

    for digit in hand_spec.digits:
        pads = index.group(f"hand.{digit.name}.pads") or index.group(f"{digit.name}.pads")
        links = index.group(f"hand.{digit.name}.links") or index.group(f"{digit.name}.links")
        for i, name in enumerate(pads):
            lookup[gid(name)] = (digit.name, i, True)
        for i, name in enumerate(links):
            j = gid(name)
            if j not in lookup:
                lookup[j] = (digit.name, i, False)

    for name in index.group("hand.palm_geoms") or index.group("palm_geoms"):
        lookup[gid(name)] = ("palm", -1, "pad" in name)
    return lookup


def self_collision_force(
    model: "mujoco.MjModel", data: "mujoco.MjData", hand_geoms: Iterable[int]
) -> float:
    """Total normal force between hand geoms.

    A hand whose digits press harder on each other than on the object has a morphology
    problem, and it is a distinct failure mode from simply dropping the object
    (``docs/02-MATH.md`` §5.4).
    """
    import mujoco

    hand_geoms = frozenset(int(g) for g in hand_geoms)
    total = 0.0
    buf = np.zeros(6)
    for i in range(data.ncon):
        c = data.contact[i]
        if int(c.geom1) in hand_geoms and int(c.geom2) in hand_geoms:
            mujoco.mj_contactForce(model, data, i, buf)
            total += abs(float(buf[0]))
    return total
