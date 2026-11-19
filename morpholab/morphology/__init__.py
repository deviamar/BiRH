"""Parameterized hand morphology: spec, generation, search space, validation.

``morpholab.morphology.presets`` stays importable as a module (it holds the preset
registry); the convenience wrappers below are re-exported so callers rarely need it::

    >>> from morpholab.morphology import get_preset, build_hand
    >>> build = build_hand(get_preset("birh_v1"))
"""

from __future__ import annotations

from .builder import HandBuild, build_hand, hand_body_names
from .presets import get as get_preset
from .presets import names as preset_names
from .presets import presets as preset_registry
from .space import Param, ParamSpace, canonicalize_layout, get_space, spaces, stick_breaking
from .spec import (
    ActuationMode,
    ActuationSpec,
    DigitSpec,
    HandSpec,
    JointSpec,
    MountSpec,
    PalmSpec,
    PhalanxSpec,
)
from .validate import Issue, ValidationReport, is_feasible, validate_hand

__all__ = [
    "HandSpec", "PalmSpec", "DigitSpec", "MountSpec", "JointSpec", "PhalanxSpec",
    "ActuationSpec", "ActuationMode",
    "build_hand", "HandBuild", "hand_body_names",
    "get_preset", "preset_names", "preset_registry",
    "Param", "ParamSpace", "spaces", "get_space", "canonicalize_layout", "stick_breaking",
    "validate_hand", "is_feasible", "ValidationReport", "Issue",
]
