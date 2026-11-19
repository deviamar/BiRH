"""Feasibility checks on a morphology, applied before it costs a simulation.

Three classes of problem, kept distinct because they call for different responses
(``docs/02-MATH.md`` §1.3):

``error``
    The hand is geometrically impossible or would not compile.  Reject.
``warning``
    The hand is buildable but suspicious (a digit that cannot reach the palm axis, a
    spring that cannot hold the digit against gravity).  Run it, but record the flag --
    an optimizer that converges on warned designs is telling you something.
``note``
    Informational, e.g. a hand whose aperture excludes large objects.

An optimizer receives errors as a penalty rather than a crash, so the search learns the
constraint boundary instead of dying at it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

from ..mjcf.transforms import wrap_2pi
from .spec import ActuationMode, HandSpec

__all__ = ["Issue", "ValidationReport", "validate_hand", "is_feasible"]

GRAVITY = 9.81


@dataclass(frozen=True)
class Issue:
    severity: str  # "error" | "warning" | "note"
    code: str
    message: str

    def __str__(self) -> str:
        return f"[{self.severity}] {self.code}: {self.message}"


@dataclass(frozen=True)
class ValidationReport:
    issues: tuple[Issue, ...] = ()

    @property
    def errors(self) -> tuple[Issue, ...]:
        return tuple(i for i in self.issues if i.severity == "error")

    @property
    def warnings(self) -> tuple[Issue, ...]:
        return tuple(i for i in self.issues if i.severity == "warning")

    @property
    def ok(self) -> bool:
        return not self.errors

    @property
    def codes(self) -> tuple[str, ...]:
        return tuple(i.code for i in self.issues)

    def describe(self) -> str:
        return "\n".join(str(i) for i in self.issues) or "no issues"

    def __bool__(self) -> bool:
        return self.ok


def is_feasible(spec: HandSpec) -> bool:
    """Shorthand for ``validate_hand(spec).ok``."""
    return validate_hand(spec).ok


def validate_hand(spec: HandSpec, *, min_clearance: float = 0.002) -> ValidationReport:
    """Check ``spec`` for geometric and physical feasibility."""
    issues: list[Issue] = []
    issues += _check_digit_clearance(spec, min_clearance)
    issues += _check_opposability(spec)
    issues += _check_proportions(spec)
    issues += _check_actuation(spec)
    issues += _check_materials(spec)
    return ValidationReport(tuple(issues))


def _check_digit_clearance(spec: HandSpec, min_clearance: float) -> list[Issue]:
    """Adjacent digit bases must not interpenetrate (``docs/02-MATH.md`` §1.3)."""
    issues: list[Issue] = []
    entries = sorted(
        ((wrap_2pi(d.mount.azimuth), d) for d in spec.digits), key=lambda kv: kv[0]
    )
    n = len(entries)
    if n < 2:
        return issues
    for i in range(n):
        theta_a, a = entries[i]
        theta_b, b = entries[(i + 1) % n]
        gap = wrap_2pi(theta_b - theta_a)
        if n == 2 and i == 1:
            break  # the two gaps of a two-digit hand are the same pair, checked once
        r_a = spec.palm.radius * a.mount.radius_frac
        r_b = spec.palm.radius * b.mount.radius_frac
        # Chord between mount points, compared against the two proximal radii.
        chord = math.dist(
            (r_a * math.cos(theta_a), r_a * math.sin(theta_a)),
            (r_b * math.cos(theta_b), r_b * math.sin(theta_b)),
        )
        needed = a.base_radius + b.base_radius + min_clearance
        if chord < needed:
            issues.append(
                Issue(
                    "error",
                    "digit_overlap",
                    f"digits {a.name!r} and {b.name!r} are {math.degrees(gap):.1f}° apart "
                    f"({chord * 1000:.1f} mm between mounts) but need "
                    f"{needed * 1000:.1f} mm to avoid interpenetration",
                )
            )
    return issues


def _check_opposability(spec: HandSpec) -> list[Issue]:
    """A digit that cannot reach the palm axis can never oppose the others."""
    issues: list[Issue] = []
    for d in spec.digits:
        mount_r = spec.palm.radius * d.mount.radius_frac
        if d.total_length < mount_r:
            issues.append(
                Issue(
                    "error",
                    "unopposable_digit",
                    f"digit {d.name!r} is {d.total_length * 100:.1f} cm long but mounts "
                    f"{mount_r * 100:.1f} cm from the palm axis, so it can never contact "
                    "anything the other digits contact",
                )
            )
        elif d.total_length < mount_r * 1.25:
            issues.append(
                Issue(
                    "warning",
                    "short_digit",
                    f"digit {d.name!r} barely reaches the palm axis "
                    f"(length/mount radius = {d.total_length / max(mount_r, 1e-9):.2f})",
                )
            )

    # Enclosure: the summed flexion must be able to curl a fingertip inward past the axis.
    for d in spec.digits:
        total_flexion = sum(j.span for j in d.flexion_joints)
        if total_flexion < math.radians(90.0):
            issues.append(
                Issue(
                    "warning",
                    "limited_flexion",
                    f"digit {d.name!r} has only {math.degrees(total_flexion):.0f}° of total "
                    "flexion; enclosure grasps need roughly 180°",
                )
            )
    return issues


def _check_proportions(spec: HandSpec) -> list[Issue]:
    issues: list[Issue] = []
    for d in spec.digits:
        for i, ph in enumerate(d.phalanges):
            # A capsule of length L and radius r spans L + 2r, so it stops being a
            # distinguishable link only as L approaches zero. L < r is an error; L < 2r is
            # merely stubby, and stubby distal phalanges are a real design choice.
            if ph.length < ph.radius:
                issues.append(
                    Issue(
                        "error",
                        "degenerate_phalanx",
                        f"{d.name!r} phalanx {i} is {ph.length * 1000:.1f} mm long but "
                        f"{ph.radius * 1000:.1f} mm in radius: the capsule degenerates to "
                        "a sphere and the joint does nothing",
                    )
                )
            elif ph.length < 2.0 * ph.radius:
                issues.append(
                    Issue(
                        "note",
                        "stubby_phalanx",
                        f"{d.name!r} phalanx {i} is shorter than its diameter "
                        f"({ph.length * 1000:.1f} mm vs {ph.radius * 2000:.1f} mm)",
                    )
                )
        if d.total_length > 4.0 * spec.palm.radius:
            issues.append(
                Issue(
                    "note",
                    "long_digits",
                    f"digit {d.name!r} is {d.total_length / spec.palm.radius:.1f}x the palm "
                    "radius; expect fingertip rather than enclosing grasps",
                )
            )
    if spec.palm.thickness > spec.palm.radius:
        issues.append(
            Issue("warning", "thick_palm",
                  f"palm is {spec.palm.thickness * 100:.1f} cm thick but only "
                  f"{spec.palm.radius * 100:.1f} cm in radius")
        )
    return issues


def _check_actuation(spec: HandSpec) -> list[Issue]:
    """Actuator authority, and springs strong enough to matter."""
    issues: list[Issue] = []
    act = spec.actuation
    if act.mode not in ActuationMode.ALL:
        issues.append(Issue("error", "bad_actuation_mode", f"unknown actuation mode {act.mode!r}"))
        return issues
    if act.force_limit <= 0:
        issues.append(Issue("error", "no_actuator_force", "force_limit must be positive"))

    if act.is_tendon:
        for d in spec.digits:
            # Gravitational torque on the proximal joint with the digit held horizontal.
            grav_torque = d.mass * GRAVITY * d.total_length * 0.5
            spring = max(act.extension_stiffness, max(j.stiffness for j in d.flexion_joints))
            spring_torque = spring * d.flexion_joints[0].span
            if spring_torque < grav_torque:
                issues.append(
                    Issue(
                        "warning",
                        "weak_extension_spring",
                        f"digit {d.name!r}: extension spring provides {spring_torque:.3f} N·m "
                        f"over its range but gravity applies up to {grav_torque:.3f} N·m, so "
                        "resting posture is set by gravity rather than by design",
                    )
                )
            arms = act.coefficients_for(d.n_phalanges)
            if min(arms) <= 0:
                issues.append(
                    Issue("error", "bad_moment_arm",
                          f"digit {d.name!r} has a non-positive tendon moment arm")
                )
            elif max(arms) / min(arms) > 20.0:
                issues.append(
                    Issue(
                        "warning",
                        "extreme_tendon_ratio",
                        f"digit {d.name!r} moment arms span {max(arms) / min(arms):.0f}x; "
                        "distal joints will barely move",
                    )
                )
    return issues


def _check_materials(spec: HandSpec) -> list[Issue]:
    issues: list[Issue] = []
    names = {spec.palm.material, spec.palm.pad_material} | {d.pad_material for d in spec.digits}
    for name in sorted(names):
        try:
            mat = spec.material(name)
        except KeyError as exc:
            issues.append(Issue("error", "unknown_material", str(exc)))
            continue
        if mat.friction[0] <= 0:
            issues.append(
                Issue("error", "zero_friction",
                      f"material {name!r} has zero sliding friction; no grasp is possible")
            )
        ok, msg = mat.solver.check_stable(0.002)
        if not ok:
            issues.append(Issue("warning", "contact_stability", f"material {name!r}: {msg}"))
    return issues


def summarize(reports: Iterable[ValidationReport]) -> dict[str, int]:
    """Count issue codes across many hands -- used to report why a sweep rejected designs."""
    counts: dict[str, int] = {}
    for report in reports:
        for issue in report.issues:
            counts[issue.code] = counts.get(issue.code, 0) + 1
    return dict(sorted(counts.items(), key=lambda kv: -kv[1]))
