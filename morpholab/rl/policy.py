"""A small numpy policy network.

Numpy rather than a deep-learning framework because the training method that fits this
platform is evolution strategies (``docs/06-OPTIMIZATION-AND-RL.md`` §3): the parameter
vector is what gets perturbed, evaluated in parallel across the existing process pool,
and updated.  No gradients through the simulator, no framework dependency, and a policy
is a flat float array that serializes into a result row.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence

import numpy as np

__all__ = ["MLPPolicy"]


@dataclass
class MLPPolicy:
    """Tanh MLP: ``obs -> action in [-1, 1]^act_dim``."""

    obs_dim: int
    act_dim: int
    hidden: Sequence[int] = (32,)
    _shapes: list[tuple[int, int]] = field(default_factory=list, init=False, repr=False)
    _params: np.ndarray = field(default=None, init=False, repr=False)  # type: ignore[assignment]

    def __post_init__(self) -> None:
        sizes = [self.obs_dim, *self.hidden, self.act_dim]
        self._shapes = [(sizes[i], sizes[i + 1]) for i in range(len(sizes) - 1)]
        self._params = np.zeros(self.n_params)

    @property
    def n_params(self) -> int:
        return sum(i * o + o for i, o in self._shapes)

    def init_random(self, rng: np.random.Generator, scale: float = 0.1) -> "MLPPolicy":
        self._params = rng.standard_normal(self.n_params) * scale
        return self

    def get_flat(self) -> np.ndarray:
        return self._params.copy()

    def set_flat(self, params: np.ndarray) -> "MLPPolicy":
        params = np.asarray(params, float).ravel()
        if params.size != self.n_params:
            raise ValueError(f"policy expects {self.n_params} parameters, got {params.size}")
        self._params = params.copy()
        return self

    def __call__(self, obs: np.ndarray) -> np.ndarray:
        h = np.asarray(obs, float).ravel()
        if h.size != self.obs_dim:
            raise ValueError(f"policy expects obs of size {self.obs_dim}, got {h.size}")
        p, off = self._params, 0
        for k, (i, o) in enumerate(self._shapes):
            W = p[off : off + i * o].reshape(i, o)
            off += i * o
            b = p[off : off + o]
            off += o
            h = h @ W + b
            h = np.tanh(h)  # tanh on the output too: actions are bounded by construction
        return h

    def to_dict(self) -> dict:
        return {"obs_dim": self.obs_dim, "act_dim": self.act_dim, "hidden": list(self.hidden),
                "params": self._params.tolist()}

    @staticmethod
    def from_dict(d: dict) -> "MLPPolicy":
        pol = MLPPolicy(int(d["obs_dim"]), int(d["act_dim"]), tuple(int(h) for h in d["hidden"]))
        return pol.set_flat(np.array(d["params"], float))
