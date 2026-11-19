"""Metrics: analytic grasp quality, contact structure, robustness, effort, complexity.

Importing this package registers every metric, so ``evaluate_metrics`` sees a complete set.
"""

from __future__ import annotations

from .base import MetricContext, MetricSpec, base_row, evaluate_metrics, metrics
from .complexity import ComplexityWeights, complexity, complexity_breakdown
from .grasp import GraspQuality, analyze_grasp, grasp_matrix, grasp_wrench_space
from . import registry as _registry  # noqa: F401  -- import for its registration side effects

__all__ = [
    "metrics", "MetricContext", "MetricSpec", "evaluate_metrics", "base_row",
    "GraspQuality", "analyze_grasp", "grasp_matrix", "grasp_wrench_space",
    "complexity", "complexity_breakdown", "ComplexityWeights",
]
