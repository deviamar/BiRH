"""Deterministic random number derivation.

Determinism here is *structural*, not aspirational (``docs/01-ARCHITECTURE.md`` §6.3): a
trial's randomness is a pure function of its content, so results never depend on execution
order, worker count, or wall-clock time.

The scheme is a two-level derivation::

    root entropy  ──►  SeedSequence(entropy=hash(job content))
                            │
                            ├── stream("init_pose")
                            ├── stream("domain")
                            ├── stream("perturb")
                            └── stream("actuator_noise")

Streams are derived by *name*, not by draw order, which is the property that matters in
practice: adding a new randomized element to the platform next month does not shift the
numbers consumed by existing streams, so an experiment run today remains reproducible
after the framework grows.

Never call ``numpy.random`` directly anywhere in the platform; always go through a
:class:`RandomContext` (``docs/03-ROADMAP.md``, cross-cutting requirements).
"""

from __future__ import annotations

import hashlib
from dataclasses import dataclass, field
from typing import Any, Iterable, Sequence

import numpy as np

from .ids import canonical_json

__all__ = ["RandomContext", "seed_from", "fibonacci_sphere", "spherical_directions"]


def seed_from(*parts: Any) -> int:
    """Derive a 128-bit integer seed from arbitrary content.

    ``seed_from("exp1", morphology_id, "sphere", 7)`` is stable across processes, Python
    versions and machines -- unlike :func:`hash`, which is randomized per process.
    """
    payload = canonical_json(list(parts)).encode("utf-8")
    return int.from_bytes(hashlib.sha256(payload).digest()[:16], "big")


@dataclass
class RandomContext:
    """A named tree of independent RNG streams derived from job content.

    Parameters
    ----------
    parts:
        The content the seed derives from -- typically
        ``(experiment_id, morphology_id, object, task, controller, trial_index)``.

    Examples
    --------
    >>> ctx = RandomContext(["demo", 3])
    >>> a = ctx.stream("init_pose").normal(size=3)
    >>> b = RandomContext(["demo", 3]).stream("init_pose").normal(size=3)
    >>> bool((a == b).all())
    True
    """

    parts: Sequence[Any] = ()
    _seq: np.random.SeedSequence = field(init=False, repr=False)
    _streams: dict[str, np.random.Generator] = field(default_factory=dict, init=False, repr=False)

    def __post_init__(self) -> None:
        self._seq = np.random.SeedSequence(seed_from(*self.parts))

    @property
    def entropy(self) -> int:
        """The root entropy, recorded in provenance so a run can be reconstructed."""
        return int(self._seq.entropy)

    def stream(self, name: str) -> np.random.Generator:
        """Return the named generator, creating it on first use.

        The same name always yields the same generator object within a context, so callers
        may hold on to it; different names yield statistically independent streams.
        """
        gen = self._streams.get(name)
        if gen is None:
            # Spawn by name rather than by order: derive a child SeedSequence whose entropy
            # mixes the root entropy with the stream name.
            child = np.random.SeedSequence(seed_from(self.entropy, name))
            gen = np.random.default_rng(child)
            self._streams[name] = gen
        return gen

    def child(self, *parts: Any) -> RandomContext:
        """A sub-context, e.g. per-trial contexts derived from a per-morphology one."""
        return RandomContext(list(self.parts) + list(parts))

    def state(self) -> dict[str, Any]:
        """Serializable description, for provenance records."""
        return {"entropy": self.entropy, "parts": list(self.parts), "streams": sorted(self._streams)}


# ---------------------------------------------------------------------------------------
# Low-discrepancy direction sampling (docs/02-MATH.md §5.2)
# ---------------------------------------------------------------------------------------


def fibonacci_sphere(n: int) -> np.ndarray:
    """``n`` near-uniformly distributed unit vectors on S^2, deterministically.

    The spherical Fibonacci lattice has markedly lower discrepancy than i.i.d. uniform
    sampling, which matters because worst-case disturbance tolerance is a *minimum* over
    sampled directions: clustered samples leave gaps, and a gap is an overestimate of
    robustness.

    Returns
    -------
    ndarray, shape (n, 3)
    """
    if n < 1:
        raise ValueError(f"n must be >= 1, got {n}")
    if n == 1:
        return np.array([[0.0, 0.0, -1.0]])
    i = np.arange(n, dtype=float) + 0.5
    phi = np.arccos(1.0 - 2.0 * i / n)          # polar angle, equal-area in cos(theta)
    theta = np.pi * (1.0 + 5.0**0.5) * i        # golden-angle increment in azimuth
    return np.column_stack(
        [np.cos(theta) * np.sin(phi), np.sin(theta) * np.sin(phi), np.cos(phi)]
    )


def spherical_directions(
    n: int, *, dim: int = 3, rng: np.random.Generator | None = None
) -> np.ndarray:
    """``n`` unit vectors in ``dim`` dimensions.

    Uses the deterministic Fibonacci lattice for ``dim == 3``; for higher dimensions (the
    full 6-D wrench sphere) no comparable low-discrepancy construction is available in
    closed form, so normalized Gaussians are used, which are exactly uniform on the sphere.
    A generator must be supplied in that case so the sampling remains seeded.
    """
    if dim == 3 and rng is None:
        return fibonacci_sphere(n)
    if rng is None:
        raise ValueError(f"a seeded rng is required for dim={dim} direction sampling")
    v = rng.normal(size=(n, dim))
    norms = np.linalg.norm(v, axis=1, keepdims=True)
    # Probability zero, but a zero norm would produce NaNs that propagate into results.
    bad = norms[:, 0] < 1e-12
    if bad.any():
        v[bad] = np.eye(dim)[: bad.sum()] if bad.sum() <= dim else np.eye(dim)[0]
        norms = np.linalg.norm(v, axis=1, keepdims=True)
    return v / norms


def sample_dirichlet(rng: np.random.Generator, k: int, alpha: float = 1.0) -> np.ndarray:
    """Uniform sample from the ``k``-simplex (``docs/02-MATH.md`` §1.1).

    Used for unbiased random-search baselines over phalanx length ratios, where the
    stick-breaking transform used by optimizers would impose a non-uniform prior.
    """
    return rng.dirichlet(np.full(k, float(alpha)))


def as_generator(seed: int | np.random.Generator | None) -> np.random.Generator:
    """Coerce a seed-ish argument into a Generator, for small utility functions."""
    if isinstance(seed, np.random.Generator):
        return seed
    return np.random.default_rng(seed)


def stable_shuffle(items: Iterable[Any], rng: np.random.Generator) -> list[Any]:
    """Shuffle a sequence with an explicit generator (never the global RNG)."""
    out = list(items)
    idx = rng.permutation(len(out))
    return [out[i] for i in idx]
