"""Optimizer protocol and registry.

An optimizer never sees a hand.  It proposes points in the unit cube, is told a loss for
each, and proposes again.  Every optimizer here **minimizes**; a fitness becomes a loss
by negation in the objective layer, so optimizers need no knowledge of what "good" means.

Multi-objective optimizers receive a loss *vector* per point and maintain a Pareto set;
single-objective ones receive a scalar.  ``n_objectives`` says which an optimizer expects,
so the loop can refuse a mismatched objective before spending any simulation.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Protocol, runtime_checkable

import numpy as np

from ..core.registry import Registry

__all__ = ["Optimizer", "optimizers", "get_optimizer", "Evaluation"]

optimizers: Registry["Optimizer"] = Registry("optimizer")


def get_optimizer(name: str, **kwargs: Any) -> "Optimizer":
    return optimizers.create(name, **kwargs)


@dataclass
class Evaluation:
    """One evaluated point: where it was, what it cost, and what it turned out to be."""

    x: np.ndarray
    loss: np.ndarray
    """Shape ``(n_objectives,)``. Scalar problems use a length-1 vector."""

    info: dict[str, Any] = field(default_factory=dict)

    @property
    def scalar(self) -> float:
        return float(self.loss[0])


@runtime_checkable
class Optimizer(Protocol):
    """Ask/tell interface. ``ask`` returns ``(n, dim)`` points in ``[0,1]^dim``."""

    name: str
    dim: int

    @property
    def n_objectives(self) -> int: ...

    def ask(self) -> np.ndarray: ...

    def tell(self, X: np.ndarray, F: np.ndarray) -> None: ...

    def best(self) -> tuple[np.ndarray, np.ndarray] | None:
        """``(x, loss)`` of the best point seen, or ``None`` before any ``tell``."""
        ...

    def state(self) -> dict[str, Any]: ...

    def load_state(self, state: dict[str, Any]) -> None: ...
