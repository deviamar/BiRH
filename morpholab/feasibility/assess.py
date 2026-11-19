"""Manufacturing and actuation feasibility of a ``HandSpec`` (``docs/05-DESIGN-SPACE.md`` §5).

The simulator will happily evaluate a finger 4 mm thick carrying 1.2 N.m, or nine servos
inside a 5 cm palm.  These checks ask whether the hand could be *built* from a named
material with a named actuator, and they are deliberately first-order: beam bending,
volume packing, strain in a flexure, torque through a tendon.  Each check reports the
number it computed, the limit it compared against and the assumption it used, so a
failing check is an engineering conversation rather than a verdict.

Assumptions stated once:

* ``ActuationSpec.force_limit`` is read as the **joint torque** (N.m) the model can apply.
* A finger is loaded worst-case as a cantilever: fully extended, the full joint torque
  reacted at the fingertip, so the proximal phalanx root sees a moment equal to it.
* Rigid printed links are solid; tendon channels and pin holes reduce the section.
* Soft materials turn every hinge into a flexure: the simulated joint stiffness must be
  achievable as a living hinge, and the flexure must survive the full flexion range.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Literal

from ..morphology.spec import ActuationMode, HandSpec
from .catalog import Actuator, Material, get_actuator, get_material

__all__ = ["Check", "FeasibilityReport", "assess", "feasibility_matrix"]

Status = Literal["pass", "warn", "fail"]

G = 9.81
SAFETY_FACTOR = 2.5
PIN_DIAMETER = 2.0e-3
TENDON_DIAMETER = 0.8e-3
PACKING_FRACTION = 0.55
"""Fraction of the palm volume usable for actuators (walls, bearings, wiring take the rest)."""

FLEXURE_LENGTH = 4.0e-3
"""Hinge length used to map a joint stiffness to a flexure thickness."""


@dataclass(frozen=True)
class Check:
    name: str
    status: Status
    value: float
    limit: float
    unit: str
    message: str
    scope: str = "hand"

    @property
    def ratio(self) -> float:
        return self.value / self.limit if self.limit else float("inf")

    def to_dict(self) -> dict[str, Any]:
        return dict(vars(self))


@dataclass
class FeasibilityReport:
    hand: str
    morphology_id: str
    material: str
    actuator: str
    pad: str
    checks: list[Check] = field(default_factory=list)
    estimates: dict[str, float] = field(default_factory=dict)

    @property
    def n_fail(self) -> int:
        return sum(c.status == "fail" for c in self.checks)

    @property
    def n_warn(self) -> int:
        return sum(c.status == "warn" for c in self.checks)

    @property
    def feasible(self) -> bool:
        return self.n_fail == 0

    @property
    def score(self) -> float:
        """1.0 for all-pass, decreasing with warnings and failures. Usable as a penalty."""
        if not self.checks:
            return 1.0
        return max(0.0, 1.0 - (0.5 * self.n_fail + 0.15 * self.n_warn) / len(self.checks))

    def to_dict(self) -> dict[str, Any]:
        return {"hand": self.hand, "morphology_id": self.morphology_id, "material": self.material,
                "actuator": self.actuator, "pad": self.pad, "feasible": self.feasible, "score": self.score,
                "n_fail": self.n_fail, "n_warn": self.n_warn, "estimates": self.estimates,
                "checks": [c.to_dict() for c in self.checks]}

    def describe(self) -> str:
        head = (f"{self.hand} ({self.morphology_id[:8]})  material={self.material}  actuator={self.actuator}  "
                f"pad={self.pad}\n  {'FEASIBLE' if self.feasible else 'NOT FEASIBLE'}: score {self.score:.2f}, "
                f"{self.n_fail} fail, {self.n_warn} warn\n")
        rows = []
        for c in self.checks:
            mark = {"pass": " ok ", "warn": "WARN", "fail": "FAIL"}[c.status]
            rows.append(f"  [{mark}] {c.name:<26} {c.value:>9.3g} / {c.limit:<9.3g} {c.unit:<6} {c.message}")
        est = "  " + ", ".join(f"{k}={v:.3g}" for k, v in self.estimates.items())
        return head + "\n".join(rows) + "\n" + est


def _status(value: float, limit: float, *, warn_at: float = 0.8, higher_is_worse: bool = True) -> Status:
    r = value / limit if limit else float("inf")
    if not higher_is_worse:
        r = limit / value if value else float("inf")
    if r > 1.0:
        return "fail"
    if r > warn_at:
        return "warn"
    return "pass"


def _joint_torque(spec: HandSpec) -> float:
    return float(spec.actuation.force_limit)


def assess(spec: HandSpec, *, material: str = "pla", actuator: str = "xl330_m288", pad: str = "silicone_dragonskin10") -> FeasibilityReport:
    """Run every check for one material / actuator / pad choice."""
    mat, act, pad_mat = get_material(material), get_actuator(actuator), get_material(pad)
    rep = FeasibilityReport(spec.name, spec.morphology_id, material, actuator, pad)
    tau = _joint_torque(spec)
    mode = spec.actuation.mode
    tendon_driven = mode in (ActuationMode.TENDON_COUPLED, ActuationMode.SYNERGY) or act.kind in ("spool", "linear")

    # -- 1. actuator can deliver the simulated torque ---------------------------------
    if act.kind == "rotary":
        deliverable = act.torque
        how = "servo at the joint"
    elif act.kind in ("spool", "linear"):
        deliverable = act.tendon_tension() * spec.actuation.base_moment_arm
        how = f"tension {act.tendon_tension():.0f} N x moment arm {spec.actuation.base_moment_arm * 1e3:.0f} mm"
    else:
        deliverable = act.torque
        how = "one chamber per joint"
    rep.checks.append(Check(
        "actuator_torque", _status(tau, deliverable, warn_at=0.9), tau, deliverable, "N.m",
        f"simulated joint torque vs {how}", "actuation"))
    if deliverable > 3.0 * tau:
        rep.checks.append(Check(
            "actuator_oversized", "warn", deliverable, 3.0 * tau, "N.m",
            "actuator delivers >3x the simulated torque: the sim under-claims, or a smaller actuator would do",
            "actuation"))
    if act.requires_soft and mat.kind != "soft":
        rep.checks.append(Check("pneumatic_needs_soft", "fail", 1, 0, "", f"{act.name} needs a soft body material, not {mat.name}", "actuation"))

    # -- 2. phalanx strength and stiffness (rigid) / flexure (soft) --------------------
    worst_stress_ratio, worst_deflection = 0.0, 0.0
    for d in spec.digits:
        L = d.total_length
        r = d.base_radius
        F_tip = tau / L
        M = tau  # moment at the proximal root when the tip carries the full joint torque
        hole = (TENDON_DIAMETER if tendon_driven else 0.0) + PIN_DIAMETER
        r_in = max(0.0, min(hole / 2, r - mat.min_wall))
        I = math.pi * (r ** 4 - r_in ** 4) / 4
        if mat.kind == "rigid":
            sigma = M * r / I if I > 0 else float("inf")
            allow = mat.yield_strength / SAFETY_FACTOR
            worst_stress_ratio = max(worst_stress_ratio, sigma / allow)
            defl = F_tip * L ** 3 / (3 * mat.youngs_modulus * I) if I > 0 else float("inf")
            worst_deflection = max(worst_deflection, defl / L)
        else:
            # Soft body: the hinge is a flexure. Required thickness for the simulated stiffness.
            k = d.stiffness if d.stiffness > 0 else spec.actuation.extension_stiffness
            w = 2 * r
            t = (12 * k * FLEXURE_LENGTH / (mat.youngs_modulus * w)) ** (1 / 3) if k > 0 else mat.min_wall
            theta = d.flexion_range[1] - d.flexion_range[0]
            strain = t * theta / (2 * FLEXURE_LENGTH)
            t_max = 0.8 * 2 * r
            if t < mat.min_wall:
                st, lim, msg = "fail", mat.min_wall, (f"required flexure {t * 1e3:.2f} mm is below the {mat.min_wall * 1e3:.1f} mm "
                                                     f"{mat.process} can make: joint stiffness too low for {mat.name}")
            elif t > t_max:
                st, lim, msg = "fail", t_max, (f"required flexure {t * 1e3:.2f} mm exceeds 80% of the link diameter: "
                                              f"joint stiffness {k:.2g} N.m/rad is unreachable in {mat.name}")
            else:
                st = "warn" if (t < 1.25 * mat.min_wall or t > 0.8 * t_max) else "pass"
                lim, msg = t_max, f"living hinge {t * 1e3:.2f} mm for k={k:.2g} N.m/rad (min {mat.min_wall * 1e3:.1f} mm)"
            rep.checks.append(Check(f"flexure_thickness[{d.name}]", st, t, lim, "m", msg, "structure"))
            rep.checks.append(Check(
                f"flexure_strain[{d.name}]", _status(strain, mat.max_strain), strain, mat.max_strain, "",
                f"strain at full flexion ({math.degrees(theta):.0f} deg) vs usable strain of {mat.name}", "structure"))
    if mat.kind == "rigid":
        rep.checks.append(Check("bending_stress", _status(worst_stress_ratio, 1.0), worst_stress_ratio, 1.0, "x allow",
                                f"proximal root stress / (yield / {SAFETY_FACTOR}) at full joint torque, worst digit", "structure"))
        rep.checks.append(Check("tip_deflection", _status(worst_deflection, 0.03), worst_deflection, 0.03, "L",
                                "elastic tip deflection as a fraction of digit length at full load", "structure"))
        needs_spring = any(d.stiffness > 0 for d in spec.digits) or tendon_driven
        if needs_spring:
            rep.checks.append(Check("return_spring", "warn", 1, 1, "",
                                    "rigid links with joint stiffness need a torsion spring or elastic band per joint (packaging, not modelled)",
                                    "structure"))

    # -- 3. minimum features: pins, channels, pads ------------------------------------
    r_needed = PIN_DIAMETER / 2 + mat.min_wall + (TENDON_DIAMETER + mat.min_wall if tendon_driven else 0.0)
    tip_min = min(d.tip_radius for d in spec.digits)
    rep.checks.append(Check("tip_radius", _status(r_needed, tip_min), tip_min, r_needed, "m",
                            f"distal link must house a {PIN_DIAMETER * 1e3:.0f} mm pin" + (", a tendon channel" if tendon_driven else "") + f" and {mat.min_wall * 1e3:.1f} mm walls",
                            "structure"))
    pad_t = min([spec.palm.pad_thickness] + [p.pad_thickness for d in spec.digits for p in d.phalanges])
    rep.checks.append(Check("pad_thickness", _status(pad_mat.min_wall, pad_t), pad_t, pad_mat.min_wall, "m",
                            f"thinnest pad vs the minimum {pad_mat.name} can be made", "surface"))
    if pad_mat.kind != "soft":
        rep.checks.append(Check("pad_material", "warn", 0, 0, "", f"{pad_mat.name} is rigid: pads will not conform; the simulated pad compliance is optimistic", "surface"))

    # -- 4. packaging -------------------------------------------------------------------
    palm_volume = math.pi * spec.palm.radius ** 2 * spec.palm.thickness
    n_act = spec.n_actuators
    in_palm = n_act
    if act.kind == "rotary" and mode == ActuationMode.PER_JOINT:
        # Rotary actuators may live in the phalanges if they fit; otherwise they go in the palm with tendons.
        fits = 0
        for d in spec.digits:
            for p in d.phalanges[:-1]:
                if p.length >= act.dims[0] and 2 * p.radius >= act.dims[2]:
                    fits += 1
        in_palm = max(0, n_act - fits)
        if in_palm > 0 and fits < n_act - spec.n_abduction:
            rep.checks.append(Check("actuators_in_phalanges", "warn", fits, n_act, "",
                                    f"only {fits}/{n_act} joints can hold a {act.name} in the link; the rest need tendons from the palm (adds routing complexity)",
                                    "packaging"))
    elif act.kind == "pneumatic":
        in_palm = 0
        r_min = 3e-3 + 2 * mat.min_wall
        rep.checks.append(Check("chamber_radius", _status(r_min, min(d.base_radius for d in spec.digits)),
                                min(d.base_radius for d in spec.digits), r_min, "m",
                                "finger must house a pressure chamber plus walls", "packaging"))
    required = in_palm * act.volume
    rep.checks.append(Check("palm_volume", _status(required, PACKING_FRACTION * palm_volume), required, PACKING_FRACTION * palm_volume, "m^3",
                            f"{in_palm} x {act.name} in the palm vs {PACKING_FRACTION:.0%} of palm volume", "packaging"))
    if tendon_driven or in_palm > 0:
        exits = in_palm
        circumference = 2 * math.pi * spec.palm.radius
        rep.checks.append(Check("tendon_exits", _status(exits * 4e-3, 0.5 * circumference), exits * 4e-3, 0.5 * circumference, "m",
                                f"{exits} tendon exits at 4 mm pitch on half the palm rim", "packaging"))

    # -- 5. mass and grip-to-weight -----------------------------------------------------
    structure_mass = spec.mass * (mat.density / 700.0)  # spec density default is 700 kg/m^3
    actuator_mass = n_act * act.mass
    total = structure_mass + actuator_mass
    grip = sum(tau / d.total_length for d in spec.digits)
    rep.estimates.update(mass_kg=total, structure_kg=structure_mass, actuators_kg=actuator_mass,
                         fingertip_force_total_N=grip, grip_to_weight=grip / (total * G) if total > 0 else 0.0,
                         palm_volume_cm3=palm_volume * 1e6, joint_torque_Nm=tau)
    rep.checks.append(Check("grip_to_weight", _status(1.0, grip / (total * G) if total > 0 else 0.0), grip / (total * G) if total > 0 else 0.0, 1.0, "",
                            "total fingertip force / hand weight; below 1 the hand cannot lift its own mass", "actuation"))
    rep.checks.append(Check("hand_mass", _status(total, 0.6), total, 0.6, "kg",
                            "estimated mass vs a 600 g budget for a wrist-mounted hand", "packaging"))
    return rep


def feasibility_matrix(spec: HandSpec, materials: list[str] | None = None, actuators: list[str] | None = None,
                       pad: str = "silicone_dragonskin10") -> list[dict[str, Any]]:
    """Score every material x actuator pairing, so a design can be asked 'what could this
    be built from?' rather than 'is it buildable from PLA and an XL330?'."""
    from .catalog import ACTUATORS, MATERIALS

    rows = []
    for m in materials or list(MATERIALS):
        for a in actuators or list(ACTUATORS):
            r = assess(spec, material=m, actuator=a, pad=pad)
            rows.append({"material": m, "actuator": a, "feasible": r.feasible, "score": r.score,
                         "n_fail": r.n_fail, "n_warn": r.n_warn, "mass_kg": r.estimates["mass_kg"],
                         "fails": [c.name for c in r.checks if c.status == "fail"]})
    rows.sort(key=lambda r: (-r["score"], r["mass_kg"]))
    return rows
