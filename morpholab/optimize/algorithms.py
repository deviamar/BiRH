"""Optimizers over the unit cube (``docs/06-OPTIMIZATION-AND-RL.md`` §2).

Four algorithms, chosen for what each one *tells you*:

``random``
    The control condition.  An optimizer that does not beat random search at matched
    budget has not earned its compute, so random search runs first and is reported beside
    every other result.
``cmaes``
    Covariance Matrix Adaptation.  The default for continuous spaces of 5-50 dimensions;
    uses the ``cma`` package, with a self-contained ``es`` fallback when it is missing.
``ga``
    Real-coded genetic algorithm (SBX crossover, polynomial mutation, tournament
    selection).  Handles the boolean/categorical parameters the space encodes as bins
    better than CMA-ES does, and its population is easy to inspect.
``nsga2``
    Multi-objective: performance *and* complexity at once, returning a Pareto front
    rather than a single winner.  The front is the project's actual deliverable -- the
    question is not "the best hand" but "how much capability each unit of complexity buys".

All state is plain arrays, so a killed run resumes from its last generation.
"""

from __future__ import annotations

import base64
import pickle
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from ..core.logging import get_logger
from .base import optimizers

log = get_logger(__name__)

__all__ = ["RandomSearch", "SimpleES", "CMAES", "GeneticAlgorithm", "NSGA2",
           "non_dominated_sort", "crowding_distance", "pareto_mask"]


def _clip(X: np.ndarray) -> np.ndarray:
    return np.clip(np.asarray(X, float), 0.0, 1.0)


def _as_matrix(F: np.ndarray) -> np.ndarray:
    F = np.asarray(F, float)
    return F.reshape(len(F), -1) if F.ndim == 1 else F


# ---------------------------------------------------------------------------------------
# variation operators (shared by GA and NSGA-II)
# ---------------------------------------------------------------------------------------

def sbx_crossover(rng: np.random.Generator, a: np.ndarray, b: np.ndarray, eta: float, pc: float) -> tuple[np.ndarray, np.ndarray]:
    """Simulated binary crossover (Deb & Agrawal 1995) on the unit cube."""
    if rng.random() > pc:
        return a.copy(), b.copy()
    u = rng.random(a.size)
    beta = np.where(u <= 0.5, (2 * u) ** (1 / (eta + 1)), (1 / (2 * (1 - u))) ** (1 / (eta + 1)))
    c1 = 0.5 * ((1 + beta) * a + (1 - beta) * b)
    c2 = 0.5 * ((1 - beta) * a + (1 + beta) * b)
    return _clip(c1), _clip(c2)


def polynomial_mutation(rng: np.random.Generator, x: np.ndarray, eta: float, pm: float) -> np.ndarray:
    """Polynomial mutation (Deb 2001) with bounds [0,1]."""
    y = x.copy()
    for i in range(y.size):
        if rng.random() > pm:
            continue
        u = rng.random()
        d1, d2 = y[i], 1.0 - y[i]
        if u < 0.5:
            dq = (2 * u + (1 - 2 * u) * (1 - d1) ** (eta + 1)) ** (1 / (eta + 1)) - 1
        else:
            dq = 1 - (2 * (1 - u) + 2 * (u - 0.5) * (1 - d2) ** (eta + 1)) ** (1 / (eta + 1))
        y[i] = y[i] + dq
    return _clip(y)


def non_dominated_sort(F: np.ndarray) -> list[list[int]]:
    """Fast non-dominated sort (Deb et al. 2002). Returns fronts as index lists."""
    F = _as_matrix(F)
    n = len(F)
    dominated_by: list[list[int]] = [[] for _ in range(n)]
    dom_count = np.zeros(n, int)
    for i in range(n):
        for j in range(n):
            if i == j:
                continue
            if np.all(F[i] <= F[j]) and np.any(F[i] < F[j]):
                dominated_by[i].append(j)
            elif np.all(F[j] <= F[i]) and np.any(F[j] < F[i]):
                dom_count[i] += 1
    fronts: list[list[int]] = [[i for i in range(n) if dom_count[i] == 0]]
    k = 0
    while fronts[k]:
        nxt: list[int] = []
        for i in fronts[k]:
            for j in dominated_by[i]:
                dom_count[j] -= 1
                if dom_count[j] == 0:
                    nxt.append(j)
        k += 1
        fronts.append(nxt)
    fronts.pop()
    return fronts


def crowding_distance(F: np.ndarray, idx: list[int]) -> np.ndarray:
    """Crowding distance of the points ``idx`` within one front."""
    F = _as_matrix(F)
    m = len(idx)
    dist = np.zeros(m)
    if m <= 2:
        dist[:] = np.inf
        return dist
    sub = F[idx]
    for k in range(sub.shape[1]):
        order = np.argsort(sub[:, k])
        lo, hi = sub[order[0], k], sub[order[-1], k]
        dist[order[0]] = dist[order[-1]] = np.inf
        span = hi - lo
        if span <= 0:
            continue
        for r in range(1, m - 1):
            dist[order[r]] += (sub[order[r + 1], k] - sub[order[r - 1], k]) / span
    return dist


def pareto_mask(F: np.ndarray) -> np.ndarray:
    """Boolean mask of non-dominated rows."""
    F = _as_matrix(F)
    mask = np.ones(len(F), bool)
    for i in range(len(F)):
        if not mask[i]:
            continue
        dominated = np.all(F <= F[i], axis=1) & np.any(F < F[i], axis=1)
        if dominated.any():
            mask[i] = False
    return mask


# ---------------------------------------------------------------------------------------
# single-objective
# ---------------------------------------------------------------------------------------

@dataclass
class _Tracker:
    """Best-so-far bookkeeping shared by every optimizer."""

    best_x: np.ndarray | None = None
    best_f: np.ndarray | None = None
    n_evaluated: int = 0

    def update(self, X: np.ndarray, F: np.ndarray) -> None:
        F = _as_matrix(F)
        self.n_evaluated += len(X)
        i = int(np.argmin(F[:, 0]))
        if self.best_f is None or F[i, 0] < self.best_f[0]:
            self.best_x, self.best_f = np.array(X[i], float), np.array(F[i], float)

    def state(self) -> dict[str, Any]:
        return {
            "best_x": None if self.best_x is None else self.best_x.tolist(),
            "best_f": None if self.best_f is None else self.best_f.tolist(),
            "n_evaluated": self.n_evaluated,
        }

    def load(self, s: dict[str, Any]) -> None:
        self.best_x = None if s.get("best_x") is None else np.array(s["best_x"], float)
        self.best_f = None if s.get("best_f") is None else np.array(s["best_f"], float)
        self.n_evaluated = int(s.get("n_evaluated", 0))


@optimizers.register("random", doc="Uniform random search: the control condition every optimizer must beat",
                     tags=["single", "baseline"])
@dataclass
class RandomSearch:
    dim: int
    popsize: int = 16
    seed: int = 0
    name: str = "random"
    _rng: np.random.Generator = field(init=False, repr=False)
    _track: _Tracker = field(default_factory=_Tracker, init=False, repr=False)

    def __post_init__(self) -> None:
        self._rng = np.random.default_rng(self.seed)

    @property
    def n_objectives(self) -> int:
        return 1

    def ask(self) -> np.ndarray:
        return self._rng.random((self.popsize, self.dim))

    def tell(self, X: np.ndarray, F: np.ndarray) -> None:
        self._track.update(np.asarray(X, float), F)

    def best(self):
        return None if self._track.best_x is None else (self._track.best_x, self._track.best_f)

    def state(self) -> dict[str, Any]:
        return {"rng": self._rng.bit_generator.state, **self._track.state()}

    def load_state(self, s: dict[str, Any]) -> None:
        self._rng.bit_generator.state = s["rng"]
        self._track.load(s)


@optimizers.register("es", doc="(mu, lambda) evolution strategy with 1/5-rule step adaptation; no dependencies",
                     tags=["single"])
@dataclass
class SimpleES:
    """A dependency-free evolution strategy: Gaussian mutation of the elite mean with a
    step size adapted by the 1/5 success rule.  Weaker than CMA-ES, but always available."""

    dim: int
    popsize: int = 16
    sigma: float = 0.25
    seed: int = 0
    x0: np.ndarray | None = None
    name: str = "es"
    _rng: np.random.Generator = field(init=False, repr=False)
    _mean: np.ndarray = field(init=False, repr=False)
    _track: _Tracker = field(default_factory=_Tracker, init=False, repr=False)
    _last_best: float = field(default=np.inf, init=False, repr=False)

    def __post_init__(self) -> None:
        self._rng = np.random.default_rng(self.seed)
        self._mean = np.full(self.dim, 0.5) if self.x0 is None else _clip(np.asarray(self.x0, float))

    @property
    def n_objectives(self) -> int:
        return 1

    def ask(self) -> np.ndarray:
        return _clip(self._mean + self.sigma * self._rng.standard_normal((self.popsize, self.dim)))

    def tell(self, X: np.ndarray, F: np.ndarray) -> None:
        X, F = np.asarray(X, float), _as_matrix(F)[:, 0]
        mu = max(1, self.popsize // 4)
        elite = np.argsort(F)[:mu]
        self._mean = X[elite].mean(axis=0)
        improved = float(np.mean(F < self._last_best))
        self._last_best = min(self._last_best, float(F.min()))
        # 1/5 rule: expand when more than a fifth of offspring improve, otherwise contract.
        self.sigma *= 1.2 if improved > 0.2 else 0.85
        self.sigma = float(np.clip(self.sigma, 0.01, 0.5))
        self._track.update(X, F)

    def best(self):
        return None if self._track.best_x is None else (self._track.best_x, self._track.best_f)

    def state(self) -> dict[str, Any]:
        return {"rng": self._rng.bit_generator.state, "mean": self._mean.tolist(), "sigma": self.sigma,
                "last_best": self._last_best, **self._track.state()}

    def load_state(self, s: dict[str, Any]) -> None:
        self._rng.bit_generator.state = s["rng"]
        self._mean = np.array(s["mean"], float)
        self.sigma = float(s["sigma"])
        self._last_best = float(s["last_best"])
        self._track.load(s)


class _Randn:
    """A picklable Gaussian sampler with its own generator, so CMA-ES state round-trips
    through a checkpoint. ``cma`` otherwise draws from numpy's global RNG, which a
    pickle of the strategy does not capture."""

    def __init__(self, seed: int) -> None:
        self.rng = np.random.default_rng(seed)

    def __call__(self, *shape: int) -> np.ndarray:
        return self.rng.standard_normal(shape if len(shape) != 1 else shape[0])


@optimizers.register("cmaes", doc="CMA-ES (Hansen) via the `cma` package; the default for continuous spaces",
                     tags=["single", "default"])
@dataclass
class CMAES:
    dim: int
    popsize: int | None = None
    sigma: float = 0.3
    seed: int = 0
    x0: np.ndarray | None = None
    name: str = "cmaes"
    _es: Any = field(default=None, init=False, repr=False)
    _track: _Tracker = field(default_factory=_Tracker, init=False, repr=False)

    def __post_init__(self) -> None:
        try:
            import cma
        except ImportError as exc:  # pragma: no cover - depends on the environment
            raise ImportError("CMA-ES needs the 'cma' package: pip install cma  (or use --algorithm es)") from exc
        x0 = np.full(self.dim, 0.5) if self.x0 is None else _clip(np.asarray(self.x0, float))
        opts = {"bounds": [0.0, 1.0], "seed": self.seed + 1, "verbose": -9, "randn": _Randn(self.seed)}
        if self.popsize:
            opts["popsize"] = int(self.popsize)
        self._es = cma.CMAEvolutionStrategy(x0, self.sigma, opts)
        self.popsize = int(self._es.popsize)

    @property
    def n_objectives(self) -> int:
        return 1

    def ask(self) -> np.ndarray:
        return _clip(np.array(self._es.ask(), float))

    def tell(self, X: np.ndarray, F: np.ndarray) -> None:
        X = np.asarray(X, float)
        f = _as_matrix(F)[:, 0]
        self._es.tell([np.array(x) for x in X], [float(v) for v in f])
        self._track.update(X, f)

    def best(self):
        return None if self._track.best_x is None else (self._track.best_x, self._track.best_f)

    @property
    def mean(self) -> np.ndarray:
        return _clip(np.array(self._es.mean, float))

    def state(self) -> dict[str, Any]:
        blob = base64.b64encode(pickle.dumps(self._es)).decode("ascii")
        return {"cma_pickle": blob, **self._track.state()}

    def load_state(self, s: dict[str, Any]) -> None:
        self._es = pickle.loads(base64.b64decode(s["cma_pickle"]))
        self.popsize = int(self._es.popsize)
        self._track.load(s)


@optimizers.register("ga", doc="Real-coded genetic algorithm: SBX crossover, polynomial mutation, tournament selection",
                     tags=["single"])
@dataclass
class GeneticAlgorithm:
    dim: int
    popsize: int = 24
    seed: int = 0
    pc: float = 0.9
    pm: float | None = None
    eta_c: float = 15.0
    eta_m: float = 20.0
    elitism: int = 2
    x0: np.ndarray | None = None
    name: str = "ga"
    _rng: np.random.Generator = field(init=False, repr=False)
    _pop: np.ndarray | None = field(default=None, init=False, repr=False)
    _fit: np.ndarray | None = field(default=None, init=False, repr=False)
    _track: _Tracker = field(default_factory=_Tracker, init=False, repr=False)

    def __post_init__(self) -> None:
        self._rng = np.random.default_rng(self.seed)
        if self.pm is None:
            self.pm = 1.0 / self.dim

    @property
    def n_objectives(self) -> int:
        return 1

    def _tournament(self, k: int = 2) -> np.ndarray:
        idx = self._rng.integers(0, len(self._pop), k)
        return self._pop[idx[np.argmin(self._fit[idx])]]

    def ask(self) -> np.ndarray:
        if self._pop is None:
            X = self._rng.random((self.popsize, self.dim))
            if self.x0 is not None:
                X[0] = _clip(np.asarray(self.x0, float))
            return X
        children = []
        while len(children) < self.popsize:
            a, b = self._tournament(), self._tournament()
            c1, c2 = sbx_crossover(self._rng, a, b, self.eta_c, self.pc)
            children.append(polynomial_mutation(self._rng, c1, self.eta_m, self.pm))
            if len(children) < self.popsize:
                children.append(polynomial_mutation(self._rng, c2, self.eta_m, self.pm))
        return np.array(children)

    def tell(self, X: np.ndarray, F: np.ndarray) -> None:
        X, f = np.asarray(X, float), _as_matrix(F)[:, 0]
        self._track.update(X, f)
        if self._pop is None:
            self._pop, self._fit = X.copy(), f.copy()
            return
        # (mu + lambda) survival with elitism: the best of parents and children survive.
        allX, allF = np.vstack([self._pop, X]), np.concatenate([self._fit, f])
        order = np.argsort(allF)[: self.popsize]
        self._pop, self._fit = allX[order], allF[order]

    def best(self):
        return None if self._track.best_x is None else (self._track.best_x, self._track.best_f)

    def state(self) -> dict[str, Any]:
        return {"rng": self._rng.bit_generator.state,
                "pop": None if self._pop is None else self._pop.tolist(),
                "fit": None if self._fit is None else self._fit.tolist(), **self._track.state()}

    def load_state(self, s: dict[str, Any]) -> None:
        self._rng.bit_generator.state = s["rng"]
        self._pop = None if s["pop"] is None else np.array(s["pop"], float)
        self._fit = None if s["fit"] is None else np.array(s["fit"], float)
        self._track.load(s)


# ---------------------------------------------------------------------------------------
# multi-objective
# ---------------------------------------------------------------------------------------

@optimizers.register("nsga2", doc="NSGA-II: a Pareto front over (performance, complexity) or any objective pair",
                     tags=["multi"])
@dataclass
class NSGA2:
    dim: int
    popsize: int = 32
    n_obj: int = 2
    seed: int = 0
    pc: float = 0.9
    pm: float | None = None
    eta_c: float = 15.0
    eta_m: float = 20.0
    x0: np.ndarray | None = None
    name: str = "nsga2"
    _rng: np.random.Generator = field(init=False, repr=False)
    _pop: np.ndarray | None = field(default=None, init=False, repr=False)
    _F: np.ndarray | None = field(default=None, init=False, repr=False)
    _rank: np.ndarray | None = field(default=None, init=False, repr=False)
    _crowd: np.ndarray | None = field(default=None, init=False, repr=False)
    _archive_X: list = field(default_factory=list, init=False, repr=False)
    _archive_F: list = field(default_factory=list, init=False, repr=False)
    _track: _Tracker = field(default_factory=_Tracker, init=False, repr=False)

    def __post_init__(self) -> None:
        self._rng = np.random.default_rng(self.seed)
        if self.pm is None:
            self.pm = 1.0 / self.dim

    @property
    def n_objectives(self) -> int:
        return self.n_obj

    def _tournament(self) -> np.ndarray:
        i, j = self._rng.integers(0, len(self._pop), 2)
        if self._rank[i] != self._rank[j]:
            return self._pop[i] if self._rank[i] < self._rank[j] else self._pop[j]
        return self._pop[i] if self._crowd[i] >= self._crowd[j] else self._pop[j]

    def ask(self) -> np.ndarray:
        if self._pop is None:
            X = self._rng.random((self.popsize, self.dim))
            if self.x0 is not None:
                X[0] = _clip(np.asarray(self.x0, float))
            return X
        children = []
        while len(children) < self.popsize:
            c1, c2 = sbx_crossover(self._rng, self._tournament(), self._tournament(), self.eta_c, self.pc)
            children.append(polynomial_mutation(self._rng, c1, self.eta_m, self.pm))
            if len(children) < self.popsize:
                children.append(polynomial_mutation(self._rng, c2, self.eta_m, self.pm))
        return np.array(children)

    def _select(self, X: np.ndarray, F: np.ndarray) -> None:
        fronts = non_dominated_sort(F)
        keep: list[int] = []
        rank = np.zeros(len(X), int)
        crowd = np.zeros(len(X))
        for r, front in enumerate(fronts):
            cd = crowding_distance(F, front)
            for k, i in enumerate(front):
                rank[i], crowd[i] = r, cd[k]
            if len(keep) + len(front) <= self.popsize:
                keep += front
            else:
                order = np.argsort(-cd)
                keep += [front[k] for k in order[: self.popsize - len(keep)]]
                break
        keep_arr = np.array(keep, int)
        self._pop, self._F = X[keep_arr], F[keep_arr]
        self._rank, self._crowd = rank[keep_arr], crowd[keep_arr]

    def tell(self, X: np.ndarray, F: np.ndarray) -> None:
        X, F = np.asarray(X, float), _as_matrix(F)
        if F.shape[1] != self.n_obj:
            raise ValueError(f"nsga2 expects {self.n_obj} objectives, got {F.shape[1]}")
        self._track.update(X, F)
        self._archive_X += [x.tolist() for x in X]
        self._archive_F += [f.tolist() for f in F]
        if self._pop is None:
            self._select(X, F)
        else:
            self._select(np.vstack([self._pop, X]), np.vstack([self._F, F]))

    def best(self):
        return None if self._track.best_x is None else (self._track.best_x, self._track.best_f)

    def front(self) -> tuple[np.ndarray, np.ndarray]:
        """Non-dominated points among everything ever evaluated: ``(X, F)``."""
        if not self._archive_X:
            return np.zeros((0, self.dim)), np.zeros((0, self.n_obj))
        X, F = np.array(self._archive_X, float), np.array(self._archive_F, float)
        m = pareto_mask(F)
        order = np.argsort(F[m][:, 0])
        return X[m][order], F[m][order]

    def hypervolume(self, ref: np.ndarray | None = None) -> float:
        """2-D hypervolume of the archive front against ``ref`` (defaults to the worst
        observed corner). Monotone non-decreasing over generations when the algorithm is
        working, which is the acceptance test the roadmap specifies."""
        _, F = self.front()
        if len(F) == 0 or F.shape[1] != 2:
            return 0.0
        allF = np.array(self._archive_F, float)
        ref = allF.max(axis=0) + 1e-9 if ref is None else np.asarray(ref, float)
        hv, prev_y = 0.0, ref[1]
        for x, y in F:  # sorted by first objective ascending; second descends along the front
            if x >= ref[0] or y >= prev_y:
                continue
            hv += (ref[0] - x) * (prev_y - y)
            prev_y = y
        return float(hv)

    def state(self) -> dict[str, Any]:
        return {"rng": self._rng.bit_generator.state,
                "pop": None if self._pop is None else self._pop.tolist(),
                "F": None if self._F is None else self._F.tolist(),
                "rank": None if self._rank is None else self._rank.tolist(),
                "crowd": None if self._crowd is None else self._crowd.tolist(),
                "archive_X": self._archive_X, "archive_F": self._archive_F, **self._track.state()}

    def load_state(self, s: dict[str, Any]) -> None:
        self._rng.bit_generator.state = s["rng"]
        self._pop = None if s["pop"] is None else np.array(s["pop"], float)
        self._F = None if s["F"] is None else np.array(s["F"], float)
        self._rank = None if s["rank"] is None else np.array(s["rank"], int)
        self._crowd = None if s["crowd"] is None else np.array(s["crowd"], float)
        self._archive_X, self._archive_F = list(s["archive_X"]), list(s["archive_F"])
        self._track.load(s)
