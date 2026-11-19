"""Parameter spaces: the bridge between a ``HandSpec`` tree and the flat vectors
optimizers work on (``docs/02-MATH.md`` §1).

An optimizer never sees a hand.  It proposes ``x`` in the unit cube ``[0,1]^n``; the space
decodes it into a ``HandSpec``, and encodes an existing hand back into ``x`` so a search can
be warm-started from a preset.

Three things make this more than a scaling utility:

* **Stick-breaking on the simplex** (§1.1) means every proposed set of phalanx length
  ratios is valid by construction, so no optimizer budget is spent on rejected points.
* **Canonicalization** (§1.2) quotients out the ``D_n`` symmetry of digit placement.
  Without it a 3-finger layout appears 6 times in the search space and 5 of every 6
  evaluations rediscover a hand already evaluated.
* **Log scaling** for stiffness, damping and friction, which vary over decades; searching
  them linearly wastes almost the whole range on values that behave identically.

The initial space is deliberately small -- three fingers, hinge joints, ~11 parameters --
per the project constraint to start narrow and expand only once the framework is validated
(``docs/01-ARCHITECTURE.md`` §4.2).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import Any, Callable, Iterable, Sequence

import numpy as np

from ..core.registry import Registry
from ..core.types import MaterialSpec, SolverSpec
from ..mjcf.transforms import wrap_2pi
from .spec import ActuationMode, ActuationSpec, DigitSpec, HandSpec, JointSpec, MountSpec, PalmSpec
from .validate import ValidationReport, validate_hand

__all__ = ["Param", "ParamSpace", "spaces", "canonicalize_layout", "stick_breaking", "inverse_stick_breaking"]

DEG = math.pi / 180.0
_EPS = 1e-9

spaces: Registry["ParamSpace"] = Registry("parameter space")


# ---------------------------------------------------------------------------------------
# parameters
# ---------------------------------------------------------------------------------------

@dataclass(frozen=True)
class Param:
    """One searchable parameter, with its bounds and encoding."""

    name: str
    low: float = 0.0
    high: float = 1.0
    kind: str = "continuous"  # continuous | integer | boolean | categorical
    log: bool = False
    choices: tuple[Any, ...] = ()
    unit: str = ""
    doc: str = ""

    def __post_init__(self) -> None:
        if self.kind == "categorical":
            if len(self.choices) < 2:
                raise ValueError(f"param {self.name!r}: categorical needs >= 2 choices")
        elif self.kind in ("continuous", "integer"):
            if not self.high > self.low:
                raise ValueError(f"param {self.name!r}: high must exceed low")
            if self.log and self.low <= 0:
                raise ValueError(f"param {self.name!r}: log scaling needs a positive low bound")
        elif self.kind != "boolean":
            raise ValueError(f"param {self.name!r}: unknown kind {self.kind!r}")

    @property
    def n_levels(self) -> int:
        """Number of distinct values, or 0 for continuous."""
        if self.kind == "boolean":
            return 2
        if self.kind == "categorical":
            return len(self.choices)
        if self.kind == "integer":
            return int(round(self.high - self.low)) + 1
        return 0

    def decode(self, x: float) -> Any:
        """Unit-cube coordinate -> parameter value."""
        u = float(min(max(x, 0.0), 1.0))
        if self.kind == "boolean":
            return u >= 0.5
        if self.kind == "categorical":
            return self.choices[_bin(u, len(self.choices))]
        if self.kind == "integer":
            return int(self.low) + _bin(u, self.n_levels)
        if self.log:
            return float(math.exp(math.log(self.low) + u * (math.log(self.high) - math.log(self.low))))
        return float(self.low + u * (self.high - self.low))

    def encode(self, value: Any) -> float:
        """Parameter value -> unit-cube coordinate (the inverse of :meth:`decode`)."""
        if self.kind == "boolean":
            return 0.75 if bool(value) else 0.25
        if self.kind == "categorical":
            try:
                i = self.choices.index(value)
            except ValueError:
                raise ValueError(
                    f"param {self.name!r}: {value!r} is not among {self.choices}"
                ) from None
            return _bin_center(i, len(self.choices))
        if self.kind == "integer":
            i = int(round(float(value))) - int(self.low)
            return _bin_center(min(max(i, 0), self.n_levels - 1), self.n_levels)
        v = float(min(max(float(value), self.low), self.high))
        if self.log:
            return (math.log(v) - math.log(self.low)) / (math.log(self.high) - math.log(self.low))
        return (v - self.low) / (self.high - self.low)

    def describe(self) -> str:
        if self.kind == "categorical":
            rng = "{" + ", ".join(str(c) for c in self.choices) + "}"
        elif self.kind == "boolean":
            rng = "{false, true}"
        else:
            scale = " (log)" if self.log else ""
            rng = f"[{self.low:g}, {self.high:g}]{scale}"
        unit = f" {self.unit}" if self.unit else ""
        return f"{self.name:<22} {rng}{unit}"


def _bin(u: float, k: int) -> int:
    """Map ``u`` in [0,1] to one of ``k`` equal cells.

    Binning rather than rounding keeps the optimizer's view of the space uniform: every
    level occupies exactly ``1/k`` of the range, whereas rounding gives the end levels half
    the width and biases search away from them.
    """
    return min(int(u * k), k - 1)


def _bin_center(i: int, k: int) -> float:
    return (i + 0.5) / k


# ---------------------------------------------------------------------------------------
# simplex transform (docs/02-MATH.md §1.1)
# ---------------------------------------------------------------------------------------

def stick_breaking(u: Sequence[float]) -> tuple[float, ...]:
    """Map ``u`` in ``(0,1)^{k-1}`` to a point on the open ``k``-simplex.

    >>> r = stick_breaking([0.5, 0.5])
    >>> abs(sum(r) - 1.0) < 1e-12, len(r)
    (True, 3)
    """
    remaining = 1.0
    out: list[float] = []
    for ui in u:
        share = remaining * float(min(max(ui, _EPS), 1.0 - _EPS))
        out.append(share)
        remaining -= share
    out.append(max(remaining, _EPS))
    total = sum(out)
    return tuple(v / total for v in out)


def inverse_stick_breaking(ratios: Sequence[float]) -> tuple[float, ...]:
    """Inverse of :func:`stick_breaking`, for encoding an existing hand."""
    remaining = 1.0
    out: list[float] = []
    for r in list(ratios)[:-1]:
        out.append(float(min(max(r / max(remaining, _EPS), _EPS), 1.0 - _EPS)))
        remaining -= r
    return tuple(out)


# ---------------------------------------------------------------------------------------
# layout canonicalization (docs/02-MATH.md §1.2)
# ---------------------------------------------------------------------------------------

def canonicalize_layout(spec: HandSpec) -> HandSpec:
    """Map a hand to the canonical representative of its symmetry class.

    Rotates so the reference digit sits at 90 degrees (pointing along ``+y``), sorts the
    others by azimuth, and picks between the layout and its mirror image by lexicographic
    order of the gap sequence.  Hands containing a thumb skip the mirror step, because a
    reflected thumb is a genuinely different hand.

    Returns the spec unchanged when it is already canonical, so this is safe to apply
    everywhere.
    """
    if spec.n_digits < 2:
        return spec

    ref_index = next((i for i, d in enumerate(spec.digits) if d.is_thumb), 0)
    offset = spec.digits[ref_index].mount.azimuth - math.pi / 2.0

    def rotated(digits: Sequence[DigitSpec], flip: bool) -> list[DigitSpec]:
        out = []
        for d in digits:
            a = wrap_2pi(d.mount.azimuth - offset)
            if flip:
                a = wrap_2pi(math.pi - a)  # mirror about the +y axis
            mount = replace(d.mount, azimuth=a, roll=-d.mount.roll if flip else d.mount.roll)
            out.append(replace(d, mount=mount))
        return sorted(out, key=lambda d: (not d.is_thumb, wrap_2pi(d.mount.azimuth - math.pi / 2)))

    candidates = [rotated(spec.digits, False)]
    if spec.n_thumbs == 0:
        candidates.append(rotated(spec.digits, True))

    def gap_key(digits: list[DigitSpec]) -> tuple[float, ...]:
        angles = [wrap_2pi(d.mount.azimuth - math.pi / 2) for d in digits]
        return tuple(round(a, 9) for a in angles)

    best = min(candidates, key=gap_key)
    return replace(spec, digits=tuple(best))


# ---------------------------------------------------------------------------------------
# the space
# ---------------------------------------------------------------------------------------

@dataclass
class ParamSpace:
    """A named, invertible mapping between ``[0,1]^n`` and ``HandSpec``."""

    name: str
    params: tuple[Param, ...]
    build: Callable[[dict[str, Any]], HandSpec]
    """``{param name: value}`` -> HandSpec."""

    extract: Callable[[HandSpec], dict[str, Any]] | None = None
    """HandSpec -> ``{param name: value}``, for warm starts. Optional but strongly
    recommended: without it, a search cannot be seeded from a preset."""

    canonicalize: bool = True
    doc: str = ""

    def __post_init__(self) -> None:
        names = [p.name for p in self.params]
        if len(set(names)) != len(names):
            raise ValueError(f"space {self.name!r} has duplicate parameter names")
        self._by_name = {p.name: p for p in self.params}

    # -- shape ------------------------------------------------------------------------

    @property
    def dim(self) -> int:
        return len(self.params)

    def names(self) -> list[str]:
        return [p.name for p in self.params]

    def param(self, name: str) -> Param:
        try:
            return self._by_name[name]
        except KeyError:
            raise KeyError(f"space {self.name!r} has no parameter {name!r}") from None

    def bounds(self) -> tuple[np.ndarray, np.ndarray]:
        """Always the unit cube -- optimizers need know nothing about physical units."""
        return np.zeros(self.dim), np.ones(self.dim)

    # -- coding -----------------------------------------------------------------------

    def values(self, x: Sequence[float]) -> dict[str, Any]:
        """Unit-cube vector -> named parameter values."""
        x = np.asarray(x, float).ravel()
        if x.size != self.dim:
            raise ValueError(f"space {self.name!r} expects {self.dim} values, got {x.size}")
        return {p.name: p.decode(float(x[i])) for i, p in enumerate(self.params)}

    def vector(self, values: dict[str, Any]) -> np.ndarray:
        """Named parameter values -> unit-cube vector."""
        missing = [p.name for p in self.params if p.name not in values]
        if missing:
            raise KeyError(f"space {self.name!r}: missing parameter values for {missing}")
        return np.array([p.encode(values[p.name]) for p in self.params], float)

    def decode(self, x: Sequence[float], *, name: str | None = None) -> HandSpec:
        """Unit-cube vector -> ``HandSpec``."""
        values = self.values(x)
        spec = self.build(values)
        if self.canonicalize:
            spec = canonicalize_layout(spec)
        if name:
            spec = spec.renamed(name)
        return spec.with_meta(space=self.name, params=values)

    def encode(self, spec: HandSpec) -> np.ndarray:
        """``HandSpec`` -> unit-cube vector. Requires an ``extract`` function."""
        if self.extract is None:
            raise NotImplementedError(
                f"space {self.name!r} cannot encode hands (no extract function); "
                "searches in this space must start from random or default points"
            )
        return self.vector(self.extract(spec))

    # -- sampling ---------------------------------------------------------------------

    def sample_vector(self, rng: np.random.Generator, n: int = 1) -> np.ndarray:
        return rng.random((n, self.dim))

    def sample(
        self,
        rng: np.random.Generator,
        *,
        feasible_only: bool = True,
        max_tries: int = 200,
    ) -> HandSpec:
        """One random hand. With ``feasible_only`` it rejects invalid designs."""
        for _ in range(max_tries):
            spec = self.decode(rng.random(self.dim))
            if not feasible_only or validate_hand(spec).ok:
                return spec
        raise RuntimeError(
            f"space {self.name!r}: no feasible hand in {max_tries} draws; the bounds and "
            "the feasibility constraints are probably inconsistent"
        )

    def sample_many(
        self, rng: np.random.Generator, n: int, *, feasible_only: bool = True
    ) -> list[HandSpec]:
        return [self.sample(rng, feasible_only=feasible_only) for _ in range(n)]

    def validate_vector(self, x: Sequence[float]) -> ValidationReport:
        return validate_hand(self.decode(x))

    def describe(self) -> str:
        head = f"{self.name} ({self.dim} parameters)"
        body = "\n".join(f"  {p.describe()}" + (f"   -- {p.doc}" if p.doc else "") for p in self.params)
        return f"{head}\n{body}"

    def __repr__(self) -> str:
        return f"ParamSpace({self.name!r}, dim={self.dim})"


# ---------------------------------------------------------------------------------------
# the initial (v1) space: three fingers, hinge joints
# ---------------------------------------------------------------------------------------

_V1_PARAMS: tuple[Param, ...] = (
    Param("gap_1", 40.0, 170.0, unit="deg",
          doc="azimuth from digit 1 to digit 2"),
    Param("gap_2", 40.0, 170.0, unit="deg",
          doc="azimuth from digit 2 to digit 3"),
    Param("palm_radius", 0.030, 0.075, unit="m",
          doc="sets opposition geometry and maximum aperture"),
    Param("finger_length", 0.070, 0.150, unit="m",
          doc="total digit length, matched against object size"),
    Param("ratio_u1", 0.15, 0.80,
          doc="stick-breaking coordinate 1 for phalanx length ratios"),
    Param("ratio_u2", 0.15, 0.85,
          doc="stick-breaking coordinate 2 for phalanx length ratios"),
    Param("flexion_max", 60.0, 140.0, unit="deg",
          doc="per-joint flexion range; total enclosure is 3x this"),
    Param("joint_stiffness", 0.02, 2.0, log=True, unit="N.m/rad",
          doc="passive extension spring"),
    Param("joint_damping", 0.01, 0.5, log=True, unit="N.m.s/rad",
          doc="joint damping"),
    Param("pad_friction", 0.4, 1.6,
          doc="sliding friction of the contact pads"),
    Param("thumb_abduction", kind="boolean",
          doc="give digit 3 an abduction DOF (the v2 claim)"),
    Param("abduction_range", 10.0, 60.0, unit="deg",
          doc="abduction limit; ignored when thumb_abduction is false"),
)


def _build_v1(v: dict[str, Any]) -> HandSpec:
    """Construct a three-finger hand from v1 parameter values."""
    ratios = stick_breaking([v["ratio_u1"], v["ratio_u2"]])
    flexion = (0.0, float(v["flexion_max"]) * DEG)
    azimuths = [
        90.0 * DEG,
        (90.0 + float(v["gap_1"])) * DEG,
        (90.0 + float(v["gap_1"]) + float(v["gap_2"])) * DEG,
    ]
    has_thumb = bool(v["thumb_abduction"])
    abd_range = float(v["abduction_range"]) * DEG

    digits = []
    for i, az in enumerate(azimuths):
        is_thumb = has_thumb and i == 2
        abduction = (
            JointSpec(
                axis=(0.0, 0.0, 1.0), range=(-abd_range, abd_range),
                damping=float(v["joint_damping"]) * 2.0, armature=0.003, kind="abduction",
            )
            if is_thumb
            else None
        )
        digits.append(
            DigitSpec(
                name=f"digit_{i + 1}",
                mount=MountSpec(azimuth=wrap_2pi(az)),
                total_length=float(v["finger_length"]),
                length_ratios=ratios,
                # Link thickness scales with length. Holding radius fixed while length
                # varies would make short fingers stubby capsules whose joints barely
                # articulate, so the search would be penalised for reasons of the
                # parameterization rather than of the design.
                base_radius=0.095 * float(v["finger_length"]),
                tip_radius=0.070 * float(v["finger_length"]),
                flexion_range=flexion,
                stiffness=float(v["joint_stiffness"]),
                damping=float(v["joint_damping"]),
                abduction=abduction,
                is_thumb=is_thumb,
                pad_material="pad_tuned",
            )
        )

    # Friction is realized as a per-hand material override, so it is a genuine continuous
    # search dimension rather than a choice among library names.
    pad = MaterialSpec(
        name="pad_tuned",
        friction=(float(v["pad_friction"]), 0.01, 0.0005),
        solver=SolverSpec.compliant(),
        rgba=(0.18, 0.18, 0.20, 1.0),
    )
    return HandSpec(
        name="candidate",
        palm=PalmSpec(
            radius=float(v["palm_radius"]),
            thickness=max(0.016, float(v["palm_radius"]) * 0.42),
            pad_material="pad_tuned",
        ),
        digits=tuple(digits),
        actuation=ActuationSpec(
            mode=ActuationMode.TENDON_COUPLED,
            kp=6.0,
            force_limit=1.2,
            extension_stiffness=float(v["joint_stiffness"]),
        ),
        materials={"pad_tuned": pad},
    )


def _extract_v1(spec: HandSpec) -> dict[str, Any]:
    """Recover v1 parameter values from a hand, for warm starts from presets."""
    if spec.n_digits != 3:
        raise ValueError(
            f"space 'three_finger_v1' encodes 3-digit hands; {spec.name!r} has {spec.n_digits}"
        )
    canon = canonicalize_layout(spec)
    az = [wrap_2pi(d.mount.azimuth) for d in canon.digits]
    gap_1 = math.degrees(wrap_2pi(az[1] - az[0]))
    gap_2 = math.degrees(wrap_2pi(az[2] - az[1]))

    d0 = canon.digits[0]
    ratios = d0.length_ratios
    if len(ratios) != 3:
        # Re-express on three phalanges so any hand can seed the search; the search space
        # is defined for 3-phalanx digits and re-encoding is better than refusing.
        ratios = stick_breaking([ratios[0], 0.5])
    u = inverse_stick_breaking(ratios)

    thumb = next((d for d in canon.digits if d.abduction is not None), None)
    pad = canon.material(d0.pad_material)

    return {
        "gap_1": gap_1,
        "gap_2": gap_2,
        "palm_radius": canon.palm.radius,
        "finger_length": d0.total_length,
        "ratio_u1": u[0],
        "ratio_u2": u[1],
        "flexion_max": math.degrees(d0.flexion_range[1]),
        "joint_stiffness": max(d0.stiffness, canon.actuation.extension_stiffness, 0.02),
        "joint_damping": max(d0.damping, 0.01),
        "pad_friction": pad.friction[0],
        "thumb_abduction": thumb is not None,
        "abduction_range": math.degrees(abs(thumb.abduction.range[1])) if thumb else 30.0,
    }


@spaces.register(
    "three_finger_v1",
    doc="Three hinge-jointed fingers: layout, size, proportions, compliance, optional thumb",
    tags=["v1", "three_finger"],
)
def _space_v1() -> ParamSpace:
    return ParamSpace(
        name="three_finger_v1",
        params=_V1_PARAMS,
        build=_build_v1,
        extract=_extract_v1,
        doc=(
            "The initial search space. Deliberately narrow: three fingers, hinge joints, "
            "tendon-coupled actuation, twelve parameters. Tendon routing, compliant palms, "
            "digit count and joint type are held fixed until this space is validated."
        ),
    )


@spaces.register(
    "three_finger_layout",
    doc="Layout only: the two placement angles, everything else fixed at birh_v1",
    tags=["v1", "three_finger", "minimal"],
)
def _space_layout() -> ParamSpace:
    """A two-dimensional space for the question the project originally asked -- 'what finger
    placement is best?' -- which is small enough to grid-search exhaustively and therefore
    to serve as ground truth when checking that the optimizers work."""
    params = (_V1_PARAMS[0], _V1_PARAMS[1])

    from . import presets as _presets

    base_values = _extract_v1(_presets.get("birh_v1"))

    def build(v: dict[str, Any]) -> HandSpec:
        return _build_v1({**base_values, **v})

    def extract(spec: HandSpec) -> dict[str, Any]:
        full = _extract_v1(spec)
        return {"gap_1": full["gap_1"], "gap_2": full["gap_2"]}

    return ParamSpace(name="three_finger_layout", params=params, build=build, extract=extract)


def get_space(name: str) -> ParamSpace:
    """Look up a registered parameter space."""
    return spaces.create(name)
