"""Material and actuator feasibility of a hand design (``docs/05-DESIGN-SPACE.md`` §5).

    from morpholab.feasibility import assess
    print(assess(get_preset("birh_v1"), material="pla", actuator="xl330_spool").describe())

Command line: ``python -m morpholab.feasibility birh_v1 --matrix``.
"""

from __future__ import annotations

from .assess import Check, FeasibilityReport, assess, feasibility_matrix
from .catalog import ACTUATORS, MATERIALS, Actuator, Material, get_actuator, get_material

__all__ = ["assess", "feasibility_matrix", "FeasibilityReport", "Check",
           "MATERIALS", "ACTUATORS", "Material", "Actuator", "get_material", "get_actuator"]
