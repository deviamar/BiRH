"""Morphology optimization: optimizers over a ``ParamSpace`` and the objective that turns
a point into simulated grasps (``docs/06-OPTIMIZATION-AND-RL.md``)."""

from __future__ import annotations

from .algorithms import CMAES, NSGA2, GeneticAlgorithm, RandomSearch, SimpleES, pareto_mask
from .base import Evaluation, Optimizer, get_optimizer, optimizers
from .loop import OptimizationResult, optimize
from .objective import GraspObjective, ObjectiveWeights, aggregate_rows

__all__ = [
    "optimizers", "get_optimizer", "Optimizer", "Evaluation",
    "RandomSearch", "SimpleES", "CMAES", "GeneticAlgorithm", "NSGA2", "pareto_mask",
    "GraspObjective", "ObjectiveWeights", "aggregate_rows",
    "optimize", "OptimizationResult",
]
