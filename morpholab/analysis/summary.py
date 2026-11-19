"""Turning a result store into tables, comparisons and a Pareto front."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

import numpy as np

from ..core.logging import get_logger
from .stats import Comparison, Interval, bootstrap_ci, holm_correction, paired_compare, wilson_interval

log = get_logger(__name__)

__all__ = ["load_results", "summarize_by", "failure_breakdown", "pareto_front", "ablation"]


def load_results(path: str | Path):  # noqa: ANN201 - pandas.DataFrame
    """Read a run's ``results.jsonl`` (or a run directory) into a tidy dataframe."""
    import pandas as pd

    p = Path(path)
    if p.is_dir():
        p = p / "results.jsonl"
    if not p.exists():
        raise FileNotFoundError(f"no results at {p}")
    from ..experiments.store import ResultStore

    df = ResultStore(p).to_dataframe()
    if df.empty:
        return df
    # Inadmissible pairings are not observations about the hand's design; keeping them in
    # the frame but flagged lets analysis exclude them by default without losing the record
    # that they were attempted (``docs/02-MATH.md`` §1.3).
    if "admissible" not in df:
        df["admissible"] = True
    return df


def summarize_by(
    df,
    by: str | Sequence[str] = "hand",
    *,
    metrics: Sequence[str] = ("grasp_epsilon", "contact_spread", "lift_height"),
    cluster: str | None = "object",
    admissible_only: bool = True,
    alpha: float = 0.05,
):  # noqa: ANN201 - pandas.DataFrame
    """Per-group success rate with a Wilson interval, plus bootstrapped metric means.

    Success uses Wilson because it is a proportion; continuous metrics use a cluster
    bootstrap over ``cluster`` because trials within an object are not independent.
    """
    import pandas as pd

    if admissible_only and "admissible" in df:
        df = df[df["admissible"].fillna(True)]
    keys = [by] if isinstance(by, str) else list(by)

    rows = []
    for name, group in df.groupby(keys, dropna=False):
        successes = int(group["success"].sum())
        n = int(len(group))
        ci = wilson_interval(successes, n, alpha)
        row: dict[str, Any] = dict(zip(keys, name if isinstance(name, tuple) else (name,)))
        row.update({"n": n, "success": ci.estimate, "success_lo": ci.low, "success_hi": ci.high})
        clusters = group[cluster].to_numpy() if (cluster and cluster in group) else None
        for metric in metrics:
            if metric not in group:
                continue
            values = group[metric].to_numpy(dtype=float)
            m = bootstrap_ci(values, clusters=clusters, alpha=alpha, n_boot=2000)
            row[metric] = m.estimate
            row[f"{metric}_lo"] = m.low
            row[f"{metric}_hi"] = m.high
        for extra in ("complexity", "n_actuators", "n_joints"):
            if extra in group:
                row[extra] = group[extra].iloc[0]
        rows.append(row)

    out = pd.DataFrame(rows)
    return out.sort_values("success", ascending=False).reset_index(drop=True) if not out.empty else out


def failure_breakdown(df, by: str = "hand"):  # noqa: ANN201 - pandas.DataFrame
    """Failure-mode fractions per group.

    Because the taxonomy is mutually exclusive, these sum to ``1 - success rate`` and can be
    plotted as a single stacked bar -- which is what turns "it failed" into "it failed
    *this way*, which points at *this* design change" (``docs/02-MATH.md`` §5.4).
    """
    import pandas as pd

    if "admissible" in df:
        df = df[df["admissible"].fillna(True)]
    table = (
        df.assign(mode=df["failure_mode"].fillna("none"))
        .pivot_table(index=by, columns="mode", values="job_id", aggfunc="count", fill_value=0)
    )
    return table.div(table.sum(axis=1), axis=0).round(4)


def pareto_front(df, *, objective: str = "success", cost: str = "complexity", by: str = "hand"):  # noqa: ANN201
    """Non-dominated (performance, complexity) pairs, plus the knee point.

    The project's actual question is the trade-off curve, not a single winner, so this
    returns the front rather than an argmax (``docs/02-MATH.md`` §7.3).
    """
    import pandas as pd

    grouped = df.groupby(by).agg(**{objective: (objective, "mean"), cost: (cost, "first")}).reset_index()
    points = grouped.sort_values([cost, objective], ascending=[True, False])

    front, best = [], -np.inf
    for _, row in points.iterrows():
        if row[objective] > best:
            front.append(row)
            best = row[objective]
    out = pd.DataFrame(front).reset_index(drop=True)
    out["on_front"] = True

    if len(out) >= 3:
        # Knee = greatest distance from the chord joining the front's extremes, which is
        # the "best value for the complexity" recommendation.
        x = out[cost].to_numpy(float)
        y = out[objective].to_numpy(float)
        xn = (x - x.min()) / max(float(x.max() - x.min()), 1e-9)
        yn = (y - y.min()) / max(float(y.max() - y.min()), 1e-9)
        chord = np.array([xn[-1] - xn[0], yn[-1] - yn[0]])
        chord /= max(np.linalg.norm(chord), 1e-9)
        rel = np.stack([xn - xn[0], yn - yn[0]], axis=1)
        dist = np.abs(rel[:, 0] * chord[1] - rel[:, 1] * chord[0])
        out["knee"] = np.arange(len(out)) == int(dist.argmax())
    else:
        out["knee"] = False
    return out


def ablation(
    df,
    *,
    factor: str,
    baseline: str,
    variant: str,
    metric: str = "success",
    pair_on: Sequence[str] = ("object", "task", "trial"),
) -> Comparison:
    """Paired comparison of two levels of one factor.

    Pairing on ``(object, task, trial)`` is what makes an ablation powerful: the same
    object, the same seed, one morphological feature changed.  This is the analysis that
    Study 2 runs, and the one the original project's confounded comparisons could not
    support (``docs/00-REVIEW.md`` §4.3).
    """
    keys = [k for k in pair_on if k in df]
    left = df[df[factor] == baseline][keys + [metric]]
    right = df[df[factor] == variant][keys + [metric]]
    if not keys:
        from .stats import compare_groups

        return compare_groups(
            left[metric].to_numpy(float), right[metric].to_numpy(float),
            name_a=baseline, name_b=variant,
        )

    # An inner merge on the pairing keys, rather than an index intersection: a duplicated
    # key would silently produce mismatched-length arrays and a wrong paired test.
    merged = left.merge(right, on=keys, suffixes=("_a", "_b"))
    if merged.empty:
        log.warning(
            "no paired cells between %r and %r on %s; falling back to an unpaired test",
            baseline, variant, keys,
        )
        from .stats import compare_groups

        return compare_groups(
            left[metric].to_numpy(float), right[metric].to_numpy(float),
            name_a=baseline, name_b=variant,
        )
    return paired_compare(
        merged[f"{metric}_a"].to_numpy(float), merged[f"{metric}_b"].to_numpy(float),
        name_a=baseline, name_b=variant,
    )


def compare_all(df, *, by: str = "hand", metric: str = "success", pair_on=("object", "task", "trial")) -> list[Comparison]:
    """Every pairwise comparison in a family, Holm-corrected together."""
    names = sorted(df[by].dropna().unique())
    out: list[Comparison] = []
    for i, a in enumerate(names):
        for b in names[i + 1 :]:
            out.append(ablation(df, factor=by, baseline=a, variant=b, metric=metric, pair_on=pair_on))
    return holm_correction(out)


def to_markdown(table, *, floatfmt: str = "{:.3f}", index: bool = True) -> str:
    """Markdown table, for reports and terminal output.

    Written out rather than delegating to ``DataFrame.to_markdown``, which needs the
    optional ``tabulate`` package -- a report generator should not fail on a missing
    formatting dependency.
    """
    import pandas as pd

    if isinstance(table, pd.Series):
        table = table.to_frame()

    def cell(v: Any) -> str:
        if isinstance(v, float):
            return "" if not np.isfinite(v) else floatfmt.format(v)
        if isinstance(v, (bool, np.bool_)):
            return "yes" if v else ""
        return str(v)

    headers = ([str(table.index.name or "")] if index else []) + [str(c) for c in table.columns]
    rows = [
        ([str(i)] if index else []) + [cell(v) for v in row]
        for i, row in zip(table.index, table.to_numpy())
    ]
    widths = [
        max(len(h), *(len(r[j]) for r in rows)) if rows else len(h)
        for j, h in enumerate(headers)
    ]
    out = ["| " + " | ".join(h.ljust(w) for h, w in zip(headers, widths)) + " |",
           "|" + "|".join("-" * (w + 2) for w in widths) + "|"]
    out += ["| " + " | ".join(c.ljust(w) for c, w in zip(r, widths)) + " |" for r in rows]
    return "\n".join(out)
