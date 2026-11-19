"""Statistics.

Implements ``docs/02-MATH.md`` §9.  The choices here are not stylistic:

* **Wilson intervals, never the normal approximation.**  Grasp success rates sit near 0 or
  1 constantly, and at ``n = 30, p = 0.97`` the normal interval's upper bound exceeds 1.
* **Bootstrap resamples at the object level.**  Trials within an object share the object's
  difficulty and are not independent; ignoring the clustering understates the interval by
  roughly the square root of the trials per object.
* **Paired tests when the design is paired.**  An ablation varies one factor with object
  and seed fixed, which is a far more powerful design, and using an unpaired test on it
  throws that power away.
* **Effect sizes beside every p-value.**  With thousands of cheap trials, ``p < 0.001`` is
  easy and uninformative; the effect size is the finding.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Sequence

import numpy as np

__all__ = [
    "Interval", "wilson_interval", "bootstrap_ci", "compare_groups", "paired_compare",
    "holm_correction", "cohens_d", "cliffs_delta", "required_n",
]


@dataclass(frozen=True)
class Interval:
    """A point estimate with a confidence interval."""

    estimate: float
    low: float
    high: float
    n: int
    method: str = ""

    @property
    def half_width(self) -> float:
        return (self.high - self.low) / 2.0

    def __str__(self) -> str:
        return f"{self.estimate:.3f} [{self.low:.3f}, {self.high:.3f}] (n={self.n})"

    def to_dict(self) -> dict[str, Any]:
        return {"estimate": self.estimate, "ci_low": self.low, "ci_high": self.high,
                "n": self.n, "ci_method": self.method}


def wilson_interval(successes: int, n: int, alpha: float = 0.05) -> Interval:
    """Wilson score interval for a proportion.

    >>> ci = wilson_interval(29, 30)
    >>> ci.high <= 1.0
    True
    """
    if n <= 0:
        return Interval(0.0, 0.0, 1.0, 0, "wilson")
    from scipy import stats

    z = float(stats.norm.ppf(1 - alpha / 2))
    p = successes / n
    denom = 1 + z * z / n
    centre = (p + z * z / (2 * n)) / denom
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / denom
    return Interval(p, max(0.0, centre - half), min(1.0, centre + half), n, "wilson")


def bootstrap_ci(
    values: Sequence[float],
    *,
    clusters: Sequence[Any] | None = None,
    alpha: float = 0.05,
    n_boot: int = 10_000,
    statistic=np.mean,
    seed: int = 0,
) -> Interval:
    """Percentile bootstrap CI, resampling clusters rather than observations.

    ``clusters`` is normally the object name.  Passing it is what keeps the interval
    honest when several trials share an object.
    """
    x = np.asarray(values, float)
    x = x[np.isfinite(x)]
    if x.size == 0:
        return Interval(float("nan"), float("nan"), float("nan"), 0, "bootstrap")
    if x.size == 1:
        return Interval(float(x[0]), float(x[0]), float(x[0]), 1, "bootstrap")

    rng = np.random.default_rng(seed)
    point = float(statistic(x))

    if clusters is None:
        idx = rng.integers(0, x.size, size=(n_boot, x.size))
        samples = statistic(x[idx], axis=1)
        method = "bootstrap"
    else:
        keys = np.asarray(list(clusters))[: x.size]
        groups = [x[keys == k] for k in np.unique(keys)]
        g = len(groups)
        samples = np.empty(n_boot)
        for b in range(n_boot):
            pick = rng.integers(0, g, size=g)
            samples[b] = statistic(np.concatenate([groups[i] for i in pick]))
        method = f"bootstrap(cluster,{g})"

    lo, hi = np.percentile(samples, [100 * alpha / 2, 100 * (1 - alpha / 2)])
    return Interval(point, float(lo), float(hi), int(x.size), method)


def cohens_d(a: Sequence[float], b: Sequence[float]) -> float:
    """Standardized mean difference with a pooled standard deviation."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    if a.size < 2 or b.size < 2:
        return float("nan")
    pooled = math.sqrt(((a.size - 1) * a.var(ddof=1) + (b.size - 1) * b.var(ddof=1)) / (a.size + b.size - 2))
    return float((a.mean() - b.mean()) / pooled) if pooled > 0 else 0.0


def cliffs_delta(a: Sequence[float], b: Sequence[float]) -> float:
    """``P(X > Y) - P(X < Y)``: a non-parametric effect size in ``[-1, 1]``.

    The right companion to Mann-Whitney, and the right effect size for the skewed,
    bounded outcomes that grasp metrics produce.
    """
    a, b = np.asarray(a, float), np.asarray(b, float)
    if a.size == 0 or b.size == 0:
        return float("nan")
    diff = np.sign(a[:, None] - b[None, :])
    return float(diff.mean())


@dataclass(frozen=True)
class Comparison:
    """The result of comparing two groups."""

    name_a: str
    name_b: str
    mean_a: float
    mean_b: float
    difference: float
    p_welch: float
    p_mannwhitney: float
    cohens_d: float
    cliffs_delta: float
    n_a: int
    n_b: int
    paired: bool = False
    p_adjusted: float = float("nan")

    @property
    def p(self) -> float:
        """The multiplicity-adjusted p-value when available, else the raw one."""
        return self.p_adjusted if np.isfinite(self.p_adjusted) else self.p_mannwhitney

    @property
    def disagreement(self) -> bool:
        """Welch and Mann-Whitney disagreeing at 0.05 is evidence of a *distributional*
        difference rather than a location shift, and is worth flagging rather than
        resolving by picking the answer you prefer."""
        return (self.p_welch < 0.05) != (self.p_mannwhitney < 0.05)

    def to_dict(self) -> dict[str, Any]:
        return dict(vars(self))

    def __str__(self) -> str:
        star = "*" if self.p < 0.05 else " "
        return (
            f"{self.name_a} vs {self.name_b}: {self.mean_a:.3f} vs {self.mean_b:.3f} "
            f"(diff {self.difference:+.3f}) p={self.p:.4g}{star} d={self.cohens_d:+.2f} "
            f"delta={self.cliffs_delta:+.2f} n={self.n_a}/{self.n_b}"
            + ("  [paired]" if self.paired else "")
            + ("  [tests disagree]" if self.disagreement else "")
        )


def compare_groups(a: Sequence[float], b: Sequence[float], *, name_a: str = "A", name_b: str = "B") -> Comparison:
    """Unpaired comparison: Welch's t and Mann-Whitney U, with both effect sizes."""
    from scipy import stats

    a = np.asarray(a, float)[np.isfinite(a)]
    b = np.asarray(b, float)[np.isfinite(b)]
    if a.size < 2 or b.size < 2:
        return Comparison(name_a, name_b, float(a.mean()) if a.size else float("nan"),
                          float(b.mean()) if b.size else float("nan"), float("nan"),
                          float("nan"), float("nan"), float("nan"), float("nan"), a.size, b.size)

    p_welch = float(stats.ttest_ind(a, b, equal_var=False).pvalue)
    try:
        p_mw = float(stats.mannwhitneyu(a, b, alternative="two-sided").pvalue)
    except ValueError:
        p_mw = 1.0  # identical constant groups
    return Comparison(
        name_a, name_b, float(a.mean()), float(b.mean()), float(a.mean() - b.mean()),
        p_welch, p_mw, cohens_d(a, b), cliffs_delta(a, b), a.size, b.size,
    )


def paired_compare(a: Sequence[float], b: Sequence[float], *, name_a: str = "A", name_b: str = "B") -> Comparison:
    """Paired comparison: Wilcoxon signed-rank plus a paired t-test.

    Use this whenever the design pairs observations -- an ablation with object and seed
    held fixed is paired, and treating it as unpaired discards most of its power.
    """
    from scipy import stats

    a, b = np.asarray(a, float), np.asarray(b, float)
    mask = np.isfinite(a) & np.isfinite(b)
    a, b = a[mask], b[mask]
    if a.size < 2:
        return Comparison(name_a, name_b, float("nan"), float("nan"), float("nan"),
                          float("nan"), float("nan"), float("nan"), float("nan"),
                          a.size, b.size, paired=True)

    p_t = float(stats.ttest_rel(a, b).pvalue)
    diffs = a - b
    if np.allclose(diffs, 0):
        p_w = 1.0
    else:
        p_w = float(stats.wilcoxon(a, b, zero_method="zsplit").pvalue)
    sd = diffs.std(ddof=1)
    return Comparison(
        name_a, name_b, float(a.mean()), float(b.mean()), float(diffs.mean()),
        p_t, p_w, float(diffs.mean() / sd) if sd > 0 else 0.0,
        cliffs_delta(a, b), a.size, b.size, paired=True,
    )


def holm_correction(comparisons: Sequence[Comparison], alpha: float = 0.05) -> list[Comparison]:
    """Holm-Bonferroni across a family of hypotheses.

    Holm rather than Bonferroni: uniformly more powerful, same family-wise error control,
    no extra assumptions -- there is no reason to use plain Bonferroni.
    """
    from dataclasses import replace

    raw = [(i, c.p_mannwhitney if np.isfinite(c.p_mannwhitney) else 1.0) for i, c in enumerate(comparisons)]
    raw.sort(key=lambda kv: kv[1])
    m = len(raw)
    out = list(comparisons)
    running = 0.0
    for rank, (i, p) in enumerate(raw):
        adjusted = min(1.0, max(running, (m - rank) * p))
        running = adjusted
        out[i] = replace(comparisons[i], p_adjusted=adjusted)
    return out


def required_n(delta: float, baseline: float = 0.5, alpha: float = 0.05, power: float = 0.8) -> int:
    """Trials per group needed to detect a success-rate difference ``delta``.

    Run this on pilot data *before* committing a sweep, so trial counts are chosen rather
    than guessed (``docs/02-MATH.md`` §9).
    """
    from scipy import stats

    if delta <= 0:
        raise ValueError("delta must be positive")
    z_a = float(stats.norm.ppf(1 - alpha / 2))
    z_b = float(stats.norm.ppf(power))
    p = float(min(max(baseline, 1e-6), 1 - 1e-6))
    return int(math.ceil(2 * (z_a + z_b) ** 2 * p * (1 - p) / (delta * delta)))
