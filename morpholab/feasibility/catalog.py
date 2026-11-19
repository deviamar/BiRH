"""Material and actuator catalogs for the feasibility checks.

Values are typical datasheet or handbook numbers for the *as-manufactured* part, not the
bulk material: a printed PLA link is weaker than a moulded one, and the numbers here
already carry that discount.  Every entry is editable and every check reports which entry
it used, so a different printer, resin or motor is a one-line change, not a code change.

Sources (representative, not exhaustive): Prusa / Bambu filament datasheets for FDM
polymers; Smooth-On datasheets for silicones; Dynamixel, TowerPro and Pololu datasheets
for actuators.  Shore-to-modulus conversion for elastomers follows Gent (1958).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

__all__ = ["Material", "Actuator", "MATERIALS", "ACTUATORS", "get_material", "get_actuator"]


@dataclass(frozen=True)
class Material:
    name: str
    kind: Literal["rigid", "soft"]
    process: str
    youngs_modulus: float
    """Pa, effective for the manufactured part."""

    yield_strength: float
    """Pa. For elastomers this is the tensile strength; the strain limit governs instead."""

    density: float
    """kg/m^3."""

    min_wall: float
    """m. Thinnest reliable wall / flexure for the process."""

    min_feature: float
    """m. Smallest feature (pin hole, tendon channel) the process resolves."""

    max_strain: float
    """Usable engineering strain for flexures (fatigue-limited, not ultimate)."""

    doc: str = ""


@dataclass(frozen=True)
class Actuator:
    name: str
    kind: Literal["rotary", "spool", "linear", "pneumatic"]
    """``rotary``: a servo in the joint. ``spool``: a motor winding a tendon.
    ``linear``: a linear actuator pulling a tendon. ``pneumatic``: an embedded chamber."""

    torque: float
    """N.m at the output shaft (rotary / spool). For ``linear`` this is the pull force in N;
    for ``pneumatic`` the bending torque one chamber develops at its rated pressure."""

    mass: float
    """kg per actuator (pneumatic: chamber material only, compressor excluded)."""

    dims: tuple[float, float, float]
    """m: the bounding box, longest side first."""

    spool_radius: float = 0.0
    """m, for ``spool`` actuators: tendon tension = torque / spool_radius."""

    requires_soft: bool = False
    doc: str = ""

    @property
    def volume(self) -> float:
        a, b, c = self.dims
        return a * b * c

    def tendon_tension(self) -> float:
        """Maximum tendon tension this actuator can develop, in N."""
        if self.kind == "spool":
            return self.torque / self.spool_radius if self.spool_radius > 0 else 0.0
        if self.kind == "linear":
            return self.torque
        return 0.0


# --------------------------------------------------------------------------------------
# materials
# --------------------------------------------------------------------------------------

MATERIALS: dict[str, Material] = {
    m.name: m for m in (
        Material("pla", "rigid", "FDM", 3.0e9, 35e6, 1240, 1.2e-3, 1.0e-3, 0.02,
                 "Easy and stiff; brittle, creeps above 50 C. Fine for prototypes, poor for flexures."),
        Material("petg", "rigid", "FDM", 1.8e9, 30e6, 1270, 1.2e-3, 1.0e-3, 0.03,
                 "Tougher than PLA, slightly less stiff."),
        Material("abs", "rigid", "FDM", 1.9e9, 30e6, 1040, 1.2e-3, 1.0e-3, 0.03,
                 "Tough, heat tolerant; warps when printed."),
        Material("nylon_pa12", "rigid", "SLS", 1.6e9, 42e6, 1010, 0.8e-3, 0.6e-3, 0.06,
                 "SLS nylon: strong, good for living hinges, isotropic. Needs a service bureau or SLS printer."),
        Material("resin_tough", "rigid", "SLA", 2.5e9, 45e6, 1150, 0.8e-3, 0.4e-3, 0.03,
                 "Tough SLA resin: fine features, moderate toughness."),
        Material("tpu_95a", "soft", "FDM", 26e6, 25e6, 1210, 0.8e-3, 0.8e-3, 0.5,
                 "Printable elastomer, Shore 95A. Flexure joints and pads; too stiff for very soft fingers."),
        Material("tpu_85a", "soft", "FDM", 12e6, 15e6, 1180, 1.0e-3, 1.0e-3, 0.6,
                 "Softer TPU; harder to print, better pads."),
        Material("silicone_dragonskin10", "soft", "cast", 0.25e6, 3.3e6, 1070, 1.5e-3, 1.5e-3, 1.0,
                 "Shore 10A cast silicone: pads, soft fingers, pneumatic chambers."),
        Material("silicone_ecoflex30", "soft", "cast", 0.07e6, 1.4e6, 1070, 2.0e-3, 2.0e-3, 1.5,
                 "Shore 00-30: very soft; fingers made of it need embedded reinforcement."),
        Material("aluminum_6061", "rigid", "machined", 69e9, 240e6, 2700, 1.0e-3, 0.5e-3, 0.005,
                 "Machined aluminium: the rigid reference; heavy for fingers."),
    )
}


# --------------------------------------------------------------------------------------
# actuators
# --------------------------------------------------------------------------------------

ACTUATORS: dict[str, Actuator] = {
    a.name: a for a in (
        Actuator("sg90", "rotary", 0.18, 0.009, (0.029, 0.023, 0.012),
                 doc="9 g micro servo. Cheap, weak, plastic gears."),
        Actuator("mg90s", "rotary", 0.22, 0.0135, (0.029, 0.023, 0.012),
                 doc="Metal-gear micro servo."),
        Actuator("mg996r", "rotary", 1.1, 0.055, (0.043, 0.040, 0.020),
                 doc="Standard servo: the torque of birh_v1's model, at 55 g each."),
        Actuator("xl330_m288", "rotary", 0.52, 0.018, (0.034, 0.026, 0.020),
                 doc="Dynamixel XL330-M288: serial bus, position/current control, 18 g."),
        Actuator("xc330_m288", "rotary", 0.93, 0.023, (0.034, 0.026, 0.020),
                 doc="Dynamixel XC330-M288: metal gears, 23 g."),
        Actuator("n20_spool", "spool", 0.15, 0.010, (0.034, 0.012, 0.010), spool_radius=0.004,
                 doc="N20 gearmotor (100:1) winding a tendon on a 4 mm spool."),
        Actuator("xl330_spool", "spool", 0.52, 0.018, (0.034, 0.026, 0.020), spool_radius=0.006,
                 doc="XL330 with a 6 mm tendon spool: one tendon per finger, motors in the palm."),
        Actuator("xc330_spool", "spool", 0.93, 0.023, (0.034, 0.026, 0.020), spool_radius=0.006,
                 doc="XC330 with a 6 mm tendon spool: enough for the 1.2 N.m the reference hand simulates."),
        Actuator("pq12_linear", "linear", 18.0, 0.015, (0.045, 0.015, 0.012),
                 doc="Actuonix PQ12 linear actuator: 18 N pull, 20 mm stroke, tendon in line."),
        Actuator("pneumatic_chamber", "pneumatic", 0.06, 0.006, (0.0, 0.0, 0.0), requires_soft=True,
                 doc="Cast bellows chamber at 100 kPa in a ~20 mm finger; compressor and valves external."),
    )
}


def get_material(name: str) -> Material:
    try:
        return MATERIALS[name]
    except KeyError:
        raise KeyError(f"unknown material {name!r}; known: {sorted(MATERIALS)}") from None


def get_actuator(name: str) -> Actuator:
    try:
        return ACTUATORS[name]
    except KeyError:
        raise KeyError(f"unknown actuator {name!r}; known: {sorted(ACTUATORS)}") from None
