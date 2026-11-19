"""Object descriptions.

An object is a small tree of convex parts plus metadata.  Two decisions matter:

**Composites over meshes.**  MuJoCo collides meshes by their *convex hull*, so a mug's
handle, a pair of scissors and a hammer's claw simply do not exist as far as contact is
concerned.  Since the whole point of a grasping study is where the fingers can get
purchase, the core library is built from convex primitives assembled into non-convex
shapes.  Meshes remain available (:func:`mesh_object`) and are honestly labelled with
``convex_hull_collision=True`` so results computed on them can be filtered.

**Metadata is part of the object.**  Category, size class, and the grasp types an object
affords are recorded here rather than in an experiment config, so per-category reporting
(``docs/00-REVIEW.md`` A10) needs no bookkeeping at the call site.

All lengths are metres, masses kilograms.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import Any, Iterable, Sequence

import numpy as np

from ..core.ids import content_hash
from ..core.types import MATERIALS, MaterialSpec, Quat, Vec3

__all__ = ["Part", "ObjectSpec", "SizeClass", "size_class_of"]

#: Bounding-box half-extents, per primitive, given MuJoCo's ``size`` convention.
_HALF_EXTENT: dict[str, Any] = {
    "sphere": lambda s: (s[0], s[0], s[0]),
    "box": lambda s: (s[0], s[1], s[2]),
    "cylinder": lambda s: (s[0], s[0], s[1]),
    "capsule": lambda s: (s[0], s[0], s[1] + s[0]),
    "ellipsoid": lambda s: (s[0], s[1], s[2]),
}

#: Volume, per primitive -- used to distribute a target mass across parts by density.
_VOLUME: dict[str, Any] = {
    "sphere": lambda s: 4.0 / 3.0 * math.pi * s[0] ** 3,
    "box": lambda s: 8.0 * s[0] * s[1] * s[2],
    "cylinder": lambda s: 2.0 * math.pi * s[0] ** 2 * s[1],
    "capsule": lambda s: 2.0 * math.pi * s[0] ** 2 * s[1] + 4.0 / 3.0 * math.pi * s[0] ** 3,
    "ellipsoid": lambda s: 4.0 / 3.0 * math.pi * s[0] * s[1] * s[2],
}


class SizeClass(str):
    """Object size relative to a reference 5 cm palm radius."""

    TINY = "tiny"        # < 2.5 cm: pen, key -- fingertip grasps only
    SMALL = "small"      # 2.5-5 cm: marker, tape roll
    MEDIUM = "medium"    # 5-9 cm: apple, mug, can
    LARGE = "large"      # 9-14 cm: bottle, box
    HUGE = "huge"        # > 14 cm: beyond most 3-finger hands
    ALL = (TINY, SMALL, MEDIUM, LARGE, HUGE)


def size_class_of(width: float) -> str:
    """Classify by the *graspable* width (the smallest cross-section fingers must span)."""
    if width < 0.025:
        return SizeClass.TINY
    if width < 0.05:
        return SizeClass.SMALL
    if width < 0.09:
        return SizeClass.MEDIUM
    if width < 0.14:
        return SizeClass.LARGE
    return SizeClass.HUGE


@dataclass(frozen=True)
class Part:
    """One convex primitive (or mesh) making up an object."""

    type: str = "box"
    size: tuple[float, ...] = (0.02, 0.02, 0.02)
    pos: Vec3 = (0.0, 0.0, 0.0)
    quat: Quat = (1.0, 0.0, 0.0, 0.0)
    mesh: str | None = None
    material: str | None = None
    """Overrides the object's material; used for e.g. a metal blade on a plastic handle."""

    def __post_init__(self) -> None:
        if self.type == "mesh":
            if not self.mesh:
                raise ValueError("a mesh part needs a mesh name")
        elif self.type not in _HALF_EXTENT:
            raise ValueError(
                f"unknown part type {self.type!r}; expected mesh or one of {sorted(_HALF_EXTENT)}"
            )

    @property
    def half_extent(self) -> tuple[float, float, float]:
        """Axis-aligned half extents in the part's own frame."""
        if self.type == "mesh":
            return (self.size[0] if self.size else 0.05,) * 3
        return _HALF_EXTENT[self.type](self.size)

    @property
    def volume(self) -> float:
        if self.type == "mesh":
            s = self.size[0] if self.size else 0.05
            return (2 * s) ** 3 * 0.5  # rough: half the bounding cube
        return _VOLUME[self.type](self.size)

    def to_dict(self) -> dict[str, Any]:
        return {
            "type": self.type, "size": list(self.size), "pos": list(self.pos),
            "quat": list(self.quat), "mesh": self.mesh, "material": self.material,
        }

    @staticmethod
    def from_dict(d: dict[str, Any]) -> "Part":
        return Part(
            type=str(d.get("type", "box")), size=tuple(d.get("size", (0.02,) * 3)),  # type: ignore[arg-type]
            pos=tuple(d.get("pos", (0.0, 0.0, 0.0))),  # type: ignore[arg-type]
            quat=tuple(d.get("quat", (1.0, 0.0, 0.0, 0.0))),  # type: ignore[arg-type]
            mesh=d.get("mesh"), material=d.get("material"),
        )


@dataclass(frozen=True)
class ObjectSpec:
    """A graspable object."""

    name: str
    parts: tuple[Part, ...]
    mass: float = 0.1
    """Total mass. Part densities are derived so the parts sum to exactly this, which
    keeps mass an independent, sweepable variable rather than an accident of geometry."""

    material: str = "plastic"
    category: str = "primitive"
    """``primitive``, ``food``, ``tool``, ``container``, ``deformable``, ``flat``,
    ``elongated``, ``household`` -- the grouping for per-category reporting."""

    affords: tuple[str, ...] = ()
    """AHAP-style grasp types this object is normally grasped with, e.g.
    ``("power", "cylindrical")``. Used to select tasks and to check whether a hand
    succeeds on the grasps the object actually calls for."""

    spawn_quat: Quat = (1.0, 0.0, 0.0, 0.0)
    spawn_clearance: float = 0.002
    """Gap left under the object when it is placed on a support, so it settles rather than
    starting in penetration."""

    graspable_width: float | None = None
    """The dimension the digits must span. Defaults to the second-smallest bounding
    extent -- for an elongated object that is its diameter, not its length, which is the
    dimension that actually determines whether a hand can close around it."""

    tags: tuple[str, ...] = ()
    convex_hull_collision: bool = False
    """True for mesh objects, whose contact geometry is the convex hull rather than the
    visible shape. Results on such objects are flagged in reports."""

    meta: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.parts:
            raise ValueError(f"object {self.name!r} has no parts")
        if self.mass <= 0:
            raise ValueError(f"object {self.name!r} must have positive mass")

    # -- derived geometry --------------------------------------------------------------

    @property
    def aabb(self) -> tuple[np.ndarray, np.ndarray]:
        """Axis-aligned bounding box ``(lower, upper)`` in the object frame.

        Rotated parts are bounded by rotating their half-extent vector's absolute value
        through the rotation matrix, which is exact for boxes and conservative for the
        rest -- the right direction to err for an admissibility check.
        """
        from ..mjcf.transforms import matrix_from_quat

        lo = np.full(3, np.inf)
        hi = np.full(3, -np.inf)
        for part in self.parts:
            R = np.abs(matrix_from_quat(part.quat))
            extent = R @ np.asarray(part.half_extent, float)
            centre = np.asarray(part.pos, float)
            lo = np.minimum(lo, centre - extent)
            hi = np.maximum(hi, centre + extent)
        return lo, hi

    @property
    def extents(self) -> np.ndarray:
        """Full bounding-box dimensions, sorted ascending."""
        lo, hi = self.aabb
        return np.sort(hi - lo)

    @property
    def max_extent(self) -> float:
        return float(self.extents[-1])

    @property
    def width(self) -> float:
        """The span the fingers must close over."""
        if self.graspable_width is not None:
            return float(self.graspable_width)
        return float(self.extents[1])

    @property
    def size_class(self) -> str:
        return size_class_of(self.width)

    @property
    def rest_height(self) -> float:
        """Height of the object frame above a support surface when resting on it."""
        lo, _ = self.aabb
        return float(-lo[2]) + self.spawn_clearance

    @property
    def volume(self) -> float:
        return sum(p.volume for p in self.parts)

    @property
    def density(self) -> float:
        """Uniform density that makes the parts sum to :attr:`mass`."""
        return self.mass / max(self.volume, 1e-12)

    def resolve_material(self, name: str | None) -> MaterialSpec:
        key = name or self.material
        try:
            return MATERIALS[key]
        except KeyError:
            raise KeyError(f"unknown material {key!r}; available: {sorted(MATERIALS)}") from None

    # -- variation ---------------------------------------------------------------------

    def scaled(self, factor: float, *, keep_density: bool = True) -> "ObjectSpec":
        """A geometrically scaled copy.

        With ``keep_density`` the mass scales as ``factor**3``, which is what physical
        scaling means; setting it False scales size while holding mass fixed, which is the
        right control when isolating a size effect from a weight effect.
        """
        if factor <= 0:
            raise ValueError("scale factor must be positive")
        parts = tuple(
            replace(p, size=tuple(s * factor for s in p.size),
                    pos=tuple(v * factor for v in p.pos))
            for p in self.parts
        )
        mass = self.mass * factor**3 if keep_density else self.mass
        return replace(
            self, name=f"{self.name}_x{factor:g}", parts=parts, mass=mass,
            graspable_width=None if self.graspable_width is None else self.graspable_width * factor,
        )

    def with_mass(self, mass: float) -> "ObjectSpec":
        return replace(self, mass=float(mass))

    def with_material(self, material: str) -> "ObjectSpec":
        return replace(self, material=material)

    # -- identity ----------------------------------------------------------------------

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name, "parts": [p.to_dict() for p in self.parts], "mass": self.mass,
            "material": self.material, "category": self.category, "affords": list(self.affords),
            "spawn_quat": list(self.spawn_quat), "spawn_clearance": self.spawn_clearance,
            "graspable_width": self.graspable_width, "tags": list(self.tags),
            "convex_hull_collision": self.convex_hull_collision, "meta": dict(self.meta),
        }

    @staticmethod
    def from_dict(d: dict[str, Any]) -> "ObjectSpec":
        return ObjectSpec(
            name=str(d["name"]), parts=tuple(Part.from_dict(p) for p in d["parts"]),
            mass=float(d.get("mass", 0.1)), material=str(d.get("material", "plastic")),
            category=str(d.get("category", "primitive")),
            affords=tuple(d.get("affords", ())), spawn_quat=tuple(d.get("spawn_quat", (1, 0, 0, 0))),  # type: ignore[arg-type]
            spawn_clearance=float(d.get("spawn_clearance", 0.002)),
            graspable_width=d.get("graspable_width"), tags=tuple(d.get("tags", ())),
            convex_hull_collision=bool(d.get("convex_hull_collision", False)),
            meta=dict(d.get("meta", {})),
        )

    @property
    def object_id(self) -> str:
        return content_hash({k: v for k, v in self.to_dict().items() if k != "meta"})

    def describe(self) -> str:
        lo, hi = self.aabb
        dims = " x ".join(f"{v * 100:.1f}" for v in (hi - lo))
        affords = f", affords {'/'.join(self.affords)}" if self.affords else ""
        return (
            f"{self.name:<18} {self.category:<10} {self.size_class:<7} "
            f"{dims} cm  {self.mass * 1000:.0f} g  width {self.width * 100:.1f} cm"
            f"  [{len(self.parts)} parts]{affords}"
        )

    def __repr__(self) -> str:
        return f"ObjectSpec({self.name!r}, {self.size_class}, {self.mass * 1000:.0f}g)"
