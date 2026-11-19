"""Morphology Lab — a computational research platform for robotic hand morphology optimization.

The platform is organized in strict dependency layers (see ``docs/01-ARCHITECTURE.md``)::

    core -> mjcf -> {morphology, objects} -> {control, tasks} -> sim -> metrics
         -> experiments -> {optimize, analysis} -> {viz, report} -> cli

Nothing imports upward. Nothing imports a renderer at module scope; headless is the default.

Typical entry points:

>>> from morpholab.morphology import presets, build_hand
>>> spec = presets.get("birh_v1")
>>> model = build_hand(spec)
"""

from __future__ import annotations

__version__ = "0.1.0"

__all__ = ["__version__"]
