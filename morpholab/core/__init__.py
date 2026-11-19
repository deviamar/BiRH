"""Foundational services: registries, seeding, ids, config, paths, logging, shared types.

This layer imports nothing else from the platform and never imports MuJoCo at module
scope, so it is usable from analysis-only and test contexts with no simulator present.
"""

from __future__ import annotations

from .config import Config, ConfigError, config_from_dict, deep_merge, load_config, parse_overrides
from .ids import canonical_json, content_hash, named_id, short_id
from .logging import get_logger, setup_logging
from .paths import RunDirs, asset_root, collect_provenance, default_runs_root, project_root
from .registry import Registry, RegistryError
from .rng import RandomContext, fibonacci_sphere, seed_from, spherical_directions
from .types import (
    MATERIALS,
    Array,
    FailureMode,
    MaterialSpec,
    Outcome,
    Phase,
    Pose,
    Quat,
    SolverSpec,
    Vec3,
)

__all__ = [
    "Config", "ConfigError", "load_config", "config_from_dict", "deep_merge", "parse_overrides",
    "content_hash", "short_id", "named_id", "canonical_json",
    "RandomContext", "seed_from", "fibonacci_sphere", "spherical_directions",
    "Registry", "RegistryError",
    "RunDirs", "project_root", "default_runs_root", "asset_root", "collect_provenance",
    "setup_logging", "get_logger",
    "Phase", "Outcome", "FailureMode", "Pose", "MaterialSpec", "SolverSpec", "MATERIALS",
    "Vec3", "Quat", "Array",
]
