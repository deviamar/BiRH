"""Analysis: confidence intervals, significance testing, ablations, Pareto fronts."""

from __future__ import annotations

from .stats import (
    Comparison, Interval, bootstrap_ci, cliffs_delta, cohens_d, compare_groups,
    holm_correction, paired_compare, required_n, wilson_interval,
)
from .summary import (
    ablation, compare_all, failure_breakdown, load_results, pareto_front, summarize_by, to_markdown,
)

__all__ = [
    "wilson_interval", "bootstrap_ci", "compare_groups", "paired_compare", "holm_correction",
    "cohens_d", "cliffs_delta", "required_n", "Interval", "Comparison",
    "load_results", "summarize_by", "failure_breakdown", "pareto_front", "ablation",
    "compare_all", "to_markdown",
]
