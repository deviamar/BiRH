"""The hand genome: a serializable, hashable description of one morphology.

A hand is *data* (``docs/01-ARCHITECTURE.md`` §1).  Nothing downstream may hardcode a
digit count, a joint name or a geometry -- everything is read from this tree.

The spec is deliberately declarative and free of MuJoCo concepts: it says "this digit is
0.11 m long with ratios (0.45, 0.33, 0.22)", not "this body has a capsule geom".  The
translation to MJCF happens in :mod:`morpholab.morphology.builder`, which is the only
module allowed to know about XML.

All lengths are metres, angles radians, masses kilograms.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import Any, Iterator, Sequence

from ..core.ids import content_hash
from ..core.types import MATERIALS, MaterialSpec

__all__ = [
    "JointSpec", "PhalanxSpec", "MountSpec", "DigitSpec", "PalmSpec", "ActuationSpec",
    "HandSpec", "ActuationMode",
]


class ActuationMode(str):
    """Actuation strategies. A ``str`` subclass so it serializes transparently."""

    PER_JOINT = "per_joint"
    """One position actuator per joint: maximum control authority, maximum cost."""

    TENDON_COUPLED = "tendon_coupled"
    """One actuator per digit driving a fixed tendon across its joints.

    Reproduces the underactuation of the physical BiRH prototype: joints close in
    proportion to ``c_i / k_i`` until one contacts the object, after which the remaining
    joints continue -- the proximal-first sequencing the v1 design notes describe achieving
    with differential screw friction (``docs/02-MATH.md`` §6.2).
    """

    SYNERGY = "synergy"
    """One actuator for the whole hand, driving all digits through a shared tendon.

    The cheapest possible hand, and the natural lower bound in a complexity study.
    """

    ALL = ("per_joint", "tendon_coupled", "synergy")


@dataclass(frozen=True)
class JointSpec:
    """One rotational degree of freedom.

    ``stiffness`` and ``springref`` define the passive extension spring that opposes the
    tendon in an underactuated digit; with ``stiffness = 0`` the joint is free and only the
    actuator moves it.
    """

    axis: tuple[float, float, float] = (0.0, -1.0, 0.0)
    """Axis in the *parent body's local frame*. The mount frame convention
    (``docs/02-MATH.md`` §2.1) makes ``(0,-1,0)`` flexion and ``(0,0,1)`` abduction for
    every digit regardless of where it sits on the palm -- which is precisely why the
    frame convention exists."""

    range: tuple[float, float] = (0.0, math.radians(100.0))
    stiffness: float = 0.0
    damping: float = 0.05
    armature: float = 0.002
    """Rotor inertia reflected through the gearbox. A small non-zero value is essential:
    it conditions the mass matrix and is the difference between a stable contact-rich
    simulation and one that needs a 10x smaller timestep."""

    frictionloss: float = 0.0
    springref: float = 0.0
    kind: str = "flexion"
    """``flexion`` or ``abduction``; used for grouping in the name index and for the
    complexity cost model."""

    @property
    def span(self) -> float:
        return float(self.range[1] - self.range[0])

    def to_dict(self) -> dict[str, Any]:
        return {
            "axis": list(self.axis), "range": list(self.range), "stiffness": self.stiffness,
            "damping": self.damping, "armature": self.armature,
            "frictionloss": self.frictionloss, "springref": self.springref, "kind": self.kind,
        }

    @staticmethod
    def from_dict(d: dict[str, Any]) -> "JointSpec":
        return JointSpec(
            axis=tuple(d.get("axis", (0.0, -1.0, 0.0))),  # type: ignore[arg-type]
            range=tuple(d.get("range", (0.0, math.radians(100.0)))),  # type: ignore[arg-type]
            stiffness=float(d.get("stiffness", 0.0)),
            damping=float(d.get("damping", 0.05)),
            armature=float(d.get("armature", 0.002)),
            frictionloss=float(d.get("frictionloss", 0.0)),
            springref=float(d.get("springref", 0.0)),
            kind=str(d.get("kind", "flexion")),
        )


@dataclass(frozen=True)
class PhalanxSpec:
    """One link of a digit, with the contact pad on its palm-facing side."""

    length: float
    radius: float
    pad_coverage: float = 0.85
    """Fraction of the phalanx length covered by the pad. Pads are where grasping actually
    happens, so their extent is a real design parameter, not decoration."""

    pad_thickness: float = 0.0025
    pad_material: str = "pad_rubber"
    density: float = 700.0
    """Effective density of a 3D-printed link with infill -- lighter than solid plastic."""

    @property
    def mass(self) -> float:
        """Capsule mass from geometry and density (cylinder + two hemispherical caps)."""
        r, L = self.radius, self.length
        return self.density * (math.pi * r * r * L + (4.0 / 3.0) * math.pi * r**3)

    def to_dict(self) -> dict[str, Any]:
        return {
            "length": self.length, "radius": self.radius, "pad_coverage": self.pad_coverage,
            "pad_thickness": self.pad_thickness, "pad_material": self.pad_material,
            "density": self.density,
        }

    @staticmethod
    def from_dict(d: dict[str, Any]) -> "PhalanxSpec":
        return PhalanxSpec(
            length=float(d["length"]), radius=float(d["radius"]),
            pad_coverage=float(d.get("pad_coverage", 0.85)),
            pad_thickness=float(d.get("pad_thickness", 0.0025)),
            pad_material=str(d.get("pad_material", "pad_rubber")),
            density=float(d.get("density", 700.0)),
        )


@dataclass(frozen=True)
class MountSpec:
    """Where and how a digit attaches to the palm (``docs/02-MATH.md`` §2.1)."""

    azimuth: float
    """Angle around the palm axis, radians. The parameter the original project varied."""

    radius_frac: float = 1.0
    """Mount distance from the palm axis, as a fraction of the palm radius."""

    elevation: float = 0.0
    """Tilt out of the palm plane, toward the palm normal. Positive pre-converges the
    fingertips, which matters for precision grasps."""

    roll: float = 0.0
    """Rotation of the flexion plane about the digit's own axis. Non-zero roll is what
    lets a digit oppose *sideways* -- the mechanism behind the lateral pinch that the v2
    testing found impossible without abduction."""

    def to_dict(self) -> dict[str, Any]:
        return {"azimuth": self.azimuth, "radius_frac": self.radius_frac,
                "elevation": self.elevation, "roll": self.roll}

    @staticmethod
    def from_dict(d: dict[str, Any]) -> "MountSpec":
        return MountSpec(
            azimuth=float(d["azimuth"]), radius_frac=float(d.get("radius_frac", 1.0)),
            elevation=float(d.get("elevation", 0.0)), roll=float(d.get("roll", 0.0)),
        )


@dataclass(frozen=True)
class DigitSpec:
    """A finger or thumb: a serial chain of phalanges on a mount frame."""

    name: str
    mount: MountSpec
    total_length: float
    length_ratios: tuple[float, ...] = (0.45, 0.33, 0.22)
    """Proximal-to-distal length fractions; must be positive and sum to 1
    (``docs/02-MATH.md`` §1.1). The default is close to human proportions -- the
    assumption the platform exists to test, not to assume (``docs/00-REVIEW.md`` A3/A4)."""

    base_radius: float = 0.011
    tip_radius: float = 0.008
    """Linear taper from base to tip; a real feature of both human and printed digits."""

    joints: tuple[JointSpec, ...] = ()
    """One per phalanx. Empty means "derive from ``flexion_range`` and the joint defaults"
    -- which is what the parameter space does, so that a search never has to produce a
    consistent list of per-joint specs."""

    flexion_range: tuple[float, float] = (0.0, math.radians(100.0))
    stiffness: float = 0.0
    damping: float = 0.05
    abduction: JointSpec | None = None
    """The disputed DOF (``docs/00-REVIEW.md`` A6). ``None`` means no abduction."""

    pad_material: str = "pad_rubber"
    density: float = 700.0
    is_thumb: bool = False
    """Marks the digit as opposing rather than symmetric. Thumbs are excluded from the
    reflection step of layout canonicalization, because a reflected thumb is a different
    hand."""

    def __post_init__(self) -> None:
        if not self.length_ratios:
            raise ValueError(f"digit {self.name!r}: length_ratios must be non-empty")
        if any(r <= 0 for r in self.length_ratios):
            raise ValueError(f"digit {self.name!r}: length_ratios must be positive, got {self.length_ratios}")
        total = sum(self.length_ratios)
        if abs(total - 1.0) > 1e-6:
            object.__setattr__(self, "length_ratios", tuple(r / total for r in self.length_ratios))
        if self.total_length <= 0:
            raise ValueError(f"digit {self.name!r}: total_length must be positive")

    # -- derived geometry ------------------------------------------------------------

    @property
    def n_phalanges(self) -> int:
        return len(self.length_ratios)

    @property
    def phalanx_lengths(self) -> tuple[float, ...]:
        return tuple(self.total_length * r for r in self.length_ratios)

    def phalanx_radius(self, index: int) -> float:
        """Radius at phalanx ``index``, linearly tapered base -> tip."""
        if self.n_phalanges == 1:
            return self.base_radius
        t = index / (self.n_phalanges - 1)
        return self.base_radius + t * (self.tip_radius - self.base_radius)

    @property
    def phalanges(self) -> tuple[PhalanxSpec, ...]:
        return tuple(
            PhalanxSpec(
                length=L, radius=self.phalanx_radius(i), pad_material=self.pad_material,
                density=self.density,
            )
            for i, L in enumerate(self.phalanx_lengths)
        )

    @property
    def flexion_joints(self) -> tuple[JointSpec, ...]:
        """Per-phalanx flexion joints, derived if not explicitly given."""
        if self.joints:
            if len(self.joints) != self.n_phalanges:
                raise ValueError(
                    f"digit {self.name!r}: {len(self.joints)} joints for "
                    f"{self.n_phalanges} phalanges"
                )
            return self.joints
        return tuple(
            JointSpec(
                axis=(0.0, -1.0, 0.0), range=self.flexion_range,
                stiffness=self.stiffness, damping=self.damping, kind="flexion",
            )
            for _ in range(self.n_phalanges)
        )

    @property
    def n_joints(self) -> int:
        return self.n_phalanges + (1 if self.abduction is not None else 0)

    @property
    def mass(self) -> float:
        return sum(p.mass for p in self.phalanges)

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name, "mount": self.mount.to_dict(), "total_length": self.total_length,
            "length_ratios": list(self.length_ratios), "base_radius": self.base_radius,
            "tip_radius": self.tip_radius,
            "joints": [j.to_dict() for j in self.joints] if self.joints else [],
            "flexion_range": list(self.flexion_range), "stiffness": self.stiffness,
            "damping": self.damping,
            "abduction": self.abduction.to_dict() if self.abduction else None,
            "pad_material": self.pad_material, "density": self.density, "is_thumb": self.is_thumb,
        }

    @staticmethod
    def from_dict(d: dict[str, Any]) -> "DigitSpec":
        abd = d.get("abduction")
        return DigitSpec(
            name=str(d["name"]), mount=MountSpec.from_dict(d["mount"]),
            total_length=float(d["total_length"]),
            length_ratios=tuple(d.get("length_ratios", (0.45, 0.33, 0.22))),  # type: ignore[arg-type]
            base_radius=float(d.get("base_radius", 0.011)),
            tip_radius=float(d.get("tip_radius", 0.008)),
            joints=tuple(JointSpec.from_dict(j) for j in d.get("joints") or ()),
            flexion_range=tuple(d.get("flexion_range", (0.0, math.radians(100.0)))),  # type: ignore[arg-type]
            stiffness=float(d.get("stiffness", 0.0)), damping=float(d.get("damping", 0.05)),
            abduction=JointSpec.from_dict(abd) if abd else None,
            pad_material=str(d.get("pad_material", "pad_rubber")),
            density=float(d.get("density", 700.0)), is_thumb=bool(d.get("is_thumb", False)),
        )


@dataclass(frozen=True)
class PalmSpec:
    """The base the digits mount on."""

    radius: float = 0.05
    thickness: float = 0.022
    curvature: float = 0.0
    """Concavity of the palm surface, as a fraction of the radius. ``0`` is flat; positive
    cups the palm toward the fingers, increasing enclosure contact. Realized as a spherical
    cap approximated by the palm pad's offset and a rim geom."""

    material: str = "structure"
    pad_material: str = "pad_rubber"
    pad_thickness: float = 0.003
    density: float = 700.0
    has_pad: bool = True

    @property
    def mass(self) -> float:
        return self.density * math.pi * self.radius**2 * self.thickness

    def to_dict(self) -> dict[str, Any]:
        return {
            "radius": self.radius, "thickness": self.thickness, "curvature": self.curvature,
            "material": self.material, "pad_material": self.pad_material,
            "pad_thickness": self.pad_thickness, "density": self.density, "has_pad": self.has_pad,
        }

    @staticmethod
    def from_dict(d: dict[str, Any]) -> "PalmSpec":
        return PalmSpec(
            radius=float(d.get("radius", 0.05)), thickness=float(d.get("thickness", 0.022)),
            curvature=float(d.get("curvature", 0.0)), material=str(d.get("material", "structure")),
            pad_material=str(d.get("pad_material", "pad_rubber")),
            pad_thickness=float(d.get("pad_thickness", 0.003)),
            density=float(d.get("density", 700.0)), has_pad=bool(d.get("has_pad", True)),
        )


@dataclass(frozen=True)
class ActuationSpec:
    """How the joints are driven (``docs/02-MATH.md`` §6.2)."""

    mode: str = ActuationMode.PER_JOINT
    kp: float = 6.0
    """Position gain. Units are N*m/rad for a joint actuator, N/m for a tendon actuator."""

    kv: float = 0.3
    force_limit: float = 1.2
    """Joint torque limit, N*m -- and for tendon drives, the equivalent tendon force
    ``force_limit / c_0``.

    The number matters and it is easy to set carelessly. A hand-scale serial servo
    (Dynamixel XL330 class) delivers a few tenths of a N*m; a small tendon drive perhaps
    1-2 N*m at the proximal pulley. An unlimited -- or merely generous -- actuator produces
    grasps no motor could achieve, which is the classic way to optimize a hand that cannot
    be built.
    """

    tendon_coefficients: tuple[float, ...] = ()
    """Moment arms ``c_i`` (pulley radii, metres) per joint, proximal to distal. Empty
    means a geometric default ``c_i = c_0 * ratio^i``, which produces proximal-first
    closure -- the behaviour the v1 prototype approximated with differential screw
    friction."""

    tendon_ratio: float = 0.75
    """Ratio between successive default moment arms; < 1 closes proximal joints first."""

    base_moment_arm: float = 0.012
    extension_stiffness: float = 0.25
    """Passive extension spring per joint, opposing the tendon (N*m/rad).

    Must comfortably exceed the gravitational torque on the joint (order
    ``m_finger * g * L/2`` ~= 0.03 N*m for a 50 g, 11 cm digit), or the resting posture is
    set by gravity rather than by the spring and the closure sequence inverts.
    """

    def coefficients_for(self, n_joints: int) -> tuple[float, ...]:
        """Moment arms for a digit with ``n_joints`` driven joints."""
        if self.tendon_coefficients:
            if len(self.tendon_coefficients) < n_joints:
                raise ValueError(
                    f"tendon_coefficients has {len(self.tendon_coefficients)} entries "
                    f"but the digit has {n_joints} joints"
                )
            return tuple(self.tendon_coefficients[:n_joints])
        return tuple(self.base_moment_arm * self.tendon_ratio**i for i in range(n_joints))

    @property
    def is_tendon(self) -> bool:
        return self.mode in (ActuationMode.TENDON_COUPLED, ActuationMode.SYNERGY)

    def to_dict(self) -> dict[str, Any]:
        return {
            "mode": self.mode, "kp": self.kp, "kv": self.kv, "force_limit": self.force_limit,
            "tendon_coefficients": list(self.tendon_coefficients),
            "tendon_ratio": self.tendon_ratio, "base_moment_arm": self.base_moment_arm,
            "extension_stiffness": self.extension_stiffness,
        }

    @staticmethod
    def from_dict(d: dict[str, Any]) -> "ActuationSpec":
        return ActuationSpec(
            mode=str(d.get("mode", ActuationMode.PER_JOINT)), kp=float(d.get("kp", 6.0)),
            kv=float(d.get("kv", 0.3)), force_limit=float(d.get("force_limit", 8.0)),
            tendon_coefficients=tuple(d.get("tendon_coefficients") or ()),
            tendon_ratio=float(d.get("tendon_ratio", 0.75)),
            base_moment_arm=float(d.get("base_moment_arm", 0.012)),
            extension_stiffness=float(d.get("extension_stiffness", 0.06)),
        )


@dataclass(frozen=True)
class HandSpec:
    """A complete hand morphology -- one point in the design space.

    Examples
    --------
    >>> from morpholab.morphology import presets
    >>> spec = presets.get("birh_v1")
    >>> spec.n_digits, spec.n_actuators
    (3, 3)
    >>> spec.morphology_id == presets.get("birh_v1").morphology_id
    True
    """

    name: str
    palm: PalmSpec = field(default_factory=PalmSpec)
    digits: tuple[DigitSpec, ...] = ()
    actuation: ActuationSpec = field(default_factory=ActuationSpec)
    materials: dict[str, MaterialSpec] = field(default_factory=dict)
    """Per-hand material overrides, keyed by the name digits and the palm refer to.

    This is what makes surface properties *searchable*: a parameter space can vary pad
    friction or pad compliance continuously by supplying an override here, instead of
    being limited to the handful of names in the shared material library
    (``docs/00-REVIEW.md`` A8, §3.6).
    """

    meta: dict[str, Any] = field(default_factory=dict)
    """Provenance: parent genome id, generation, optimizer, preset lineage. Excluded from
    the morphology id so that the same hand found twice is recognized as the same hand."""

    def __post_init__(self) -> None:
        if not self.digits:
            raise ValueError(f"hand {self.name!r} has no digits")
        names = [d.name for d in self.digits]
        if len(set(names)) != len(names):
            raise ValueError(f"hand {self.name!r} has duplicate digit names: {names}")

    # -- structure -------------------------------------------------------------------

    @property
    def n_digits(self) -> int:
        return len(self.digits)

    @property
    def n_thumbs(self) -> int:
        return sum(1 for d in self.digits if d.is_thumb)

    @property
    def n_joints(self) -> int:
        """Total actuated-or-passive rotational DOF, excluding the wrist."""
        return sum(d.n_joints for d in self.digits)

    @property
    def n_phalanges(self) -> int:
        return sum(d.n_phalanges for d in self.digits)

    @property
    def n_abduction(self) -> int:
        return sum(1 for d in self.digits if d.abduction is not None)

    @property
    def n_actuators(self) -> int:
        """Motor count -- the dominant term in the complexity cost model.

        Abduction always gets its own actuator even in tendon modes: coupling it to
        flexion would make it useless, and pretending otherwise would understate the cost
        of the feature whose value the project is trying to measure.
        """
        if self.actuation.mode == ActuationMode.PER_JOINT:
            return self.n_joints
        if self.actuation.mode == ActuationMode.TENDON_COUPLED:
            return self.n_digits + self.n_abduction
        return 1 + self.n_abduction  # SYNERGY

    @property
    def mass(self) -> float:
        return self.palm.mass + sum(d.mass for d in self.digits)

    @property
    def max_aperture(self) -> float:
        """Largest object dimension the hand could conceivably span, fully extended."""
        reach = max(
            self.palm.radius * d.mount.radius_frac + d.total_length for d in self.digits
        )
        return 2.0 * reach

    @property
    def workspace_radius(self) -> float:
        """Distance from the palm axis to the farthest fingertip when fully extended."""
        return max(self.palm.radius * d.mount.radius_frac + d.total_length for d in self.digits)

    def digit(self, name: str) -> DigitSpec:
        for d in self.digits:
            if d.name == name:
                return d
        raise KeyError(f"hand {self.name!r} has no digit named {name!r}; have {[d.name for d in self.digits]}")

    def material(self, name: str) -> MaterialSpec:
        """Resolve a material name: per-hand overrides first, then the shared library."""
        if name in self.materials:
            return self.materials[name]
        try:
            return MATERIALS[name]
        except KeyError:
            raise KeyError(
                f"unknown material {name!r}; hand defines {sorted(self.materials)}, "
                f"library has {sorted(MATERIALS)}"
            ) from None

    def with_material(self, name: str, material: MaterialSpec) -> "HandSpec":
        """Copy with one material overridden."""
        return replace(self, materials={**self.materials, name: material})

    def __iter__(self) -> Iterator[DigitSpec]:
        return iter(self.digits)

    # -- identity and serialization ---------------------------------------------------

    def to_dict(self, *, include_meta: bool = True) -> dict[str, Any]:
        d: dict[str, Any] = {
            "name": self.name, "palm": self.palm.to_dict(),
            "digits": [x.to_dict() for x in self.digits],
            "actuation": self.actuation.to_dict(),
            "materials": {k: v.to_dict() for k, v in sorted(self.materials.items())},
        }
        if include_meta:
            d["meta"] = dict(self.meta)
        return d

    @staticmethod
    def from_dict(d: dict[str, Any]) -> "HandSpec":
        from ..core.types import SolverSpec  # local import: avoids a cycle at module load

        materials = {}
        for key, m in (d.get("materials") or {}).items():
            solver = m.get("solver") or {}
            materials[key] = MaterialSpec(
                name=str(m.get("name", key)),
                friction=tuple(m.get("friction", (1.0, 0.005, 0.0001))),  # type: ignore[arg-type]
                solver=SolverSpec(**solver) if solver else SolverSpec(),
                rgba=tuple(m.get("rgba", (0.7, 0.7, 0.72, 1.0))),  # type: ignore[arg-type]
                density=float(m.get("density", 1000.0)),
            )
        return HandSpec(
            name=str(d["name"]),
            palm=PalmSpec.from_dict(d.get("palm", {})),
            digits=tuple(DigitSpec.from_dict(x) for x in d["digits"]),
            actuation=ActuationSpec.from_dict(d.get("actuation", {})),
            materials=materials,
            meta=dict(d.get("meta", {})),
        )

    @property
    def morphology_id(self) -> str:
        """Content id over the *physical* description only.

        Excludes ``name`` and ``meta``: two searches that converge on the same geometry
        should be recognized as having found the same hand, whatever they called it.
        """
        return content_hash(self.to_dict(include_meta=False) | {"name": ""})

    def with_meta(self, **meta: Any) -> "HandSpec":
        return replace(self, meta={**self.meta, **meta})

    def renamed(self, name: str) -> "HandSpec":
        return replace(self, name=name)

    def with_digits(self, digits: Sequence[DigitSpec]) -> "HandSpec":
        return replace(self, digits=tuple(digits))

    def describe(self) -> str:
        """Multi-line human summary, used by ``morpholab morphology show``."""
        lines = [
            f"{self.name}  ({self.morphology_id})",
            f"  palm      radius {self.palm.radius * 100:.1f} cm, thickness "
            f"{self.palm.thickness * 100:.1f} cm, curvature {self.palm.curvature:.2f}",
            f"  actuation {self.actuation.mode}, kp={self.actuation.kp:g}, "
            f"force limit {self.actuation.force_limit:g} N",
            f"  totals    {self.n_digits} digits, {self.n_joints} joints, "
            f"{self.n_actuators} actuators, {self.mass * 1000:.0f} g",
            "  digits:",
        ]
        for d in self.digits:
            ratios = "/".join(f"{r:.2f}" for r in d.length_ratios)
            abd = f", abduction ±{math.degrees(max(abs(v) for v in d.abduction.range)):.0f}°" if d.abduction else ""
            lines.append(
                f"    {d.name:<10} azimuth {math.degrees(d.mount.azimuth):>6.1f}°, "
                f"length {d.total_length * 100:.1f} cm, ratios {ratios}, "
                f"flexion 0-{math.degrees(d.flexion_range[1]):.0f}°{abd}"
                + ("  [thumb]" if d.is_thumb else "")
            )
        return "\n".join(lines)

    def __repr__(self) -> str:
        return (
            f"HandSpec({self.name!r}, digits={self.n_digits}, joints={self.n_joints}, "
            f"actuators={self.n_actuators}, id={self.morphology_id[:8]})"
        )
