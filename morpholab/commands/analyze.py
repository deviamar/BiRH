"""``morpholab analyze`` -- statistics and tables from a stored run."""

from __future__ import annotations

import argparse
from pathlib import Path


def add_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("run", help="run directory or results.jsonl")
    parser.add_argument("--by", default="hand", help="grouping column")
    parser.add_argument("--metric", default="success", help="metric for pairwise comparisons")
    parser.add_argument("--pareto", action="store_true", help="show the performance/complexity front")
    parser.add_argument("--compare", action="store_true", help="show Holm-corrected pairwise tests")
    parser.add_argument("--failures", action="store_true", help="show the failure-mode breakdown")
    parser.add_argument("--csv", metavar="PATH", help="export the tidy results to CSV")
    parser.add_argument("--alpha", type=float, default=0.05)


def run(args: argparse.Namespace) -> int:
    from ..analysis import (
        compare_all, failure_breakdown, load_results, pareto_front, summarize_by, to_markdown,
    )

    df = load_results(args.run)
    if df.empty:
        print("no results")
        return 1

    n_err = int((df.get("outcome") == "error").sum()) if "outcome" in df else 0
    print(f"{len(df)} rows, {df[args.by].nunique()} {args.by} groups"
          + (f", {n_err} errors" if n_err else ""))
    print()
    print(to_markdown(summarize_by(df, args.by, alpha=args.alpha).set_index(args.by)))

    if args.failures:
        print("\nfailure modes (fractions; success appears as 'none'):")
        print(to_markdown(failure_breakdown(df, args.by)))

    if args.pareto and "complexity" in df:
        print("\nperformance / complexity Pareto front:")
        front = pareto_front(df, by=args.by)
        print(to_markdown(front.set_index(args.by)))
        knee = front[front["knee"]]
        if not knee.empty:
            print(f"\nknee point: {knee.iloc[0][args.by]} -- the best value for its complexity")

    if args.compare:
        print(f"\npairwise comparisons of {args.metric} (Holm-corrected):")
        for c in compare_all(df, by=args.by, metric=args.metric):
            marker = "  *" if c.p < args.alpha else "   "
            print(f"{marker} {c}")

    if args.csv:
        df.to_csv(args.csv, index=False)
        print(f"\nwrote {args.csv}")
    return 0
