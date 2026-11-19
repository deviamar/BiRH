"""``morpholab sweep`` -- run a parameter sweep from a config file."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def add_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("config", nargs="?", help="sweep config YAML")
    parser.add_argument("--set", dest="overrides", action="append", default=[], metavar="KEY=VALUE")
    parser.add_argument("-j", "--workers", type=int, default=1)
    parser.add_argument("--runs-root", metavar="DIR")
    parser.add_argument("--no-resume", action="store_true", help="ignore existing results")
    parser.add_argument("--dry-run", action="store_true", help="expand and count jobs, run nothing")
    parser.add_argument("--hands", nargs="+", help="override the morphology list")
    parser.add_argument("--objects", nargs="+", help="override the object list")
    parser.add_argument("--tasks", nargs="+", help="override the task list")
    parser.add_argument("--trials", type=int, help="override the trial count")


def run(args: argparse.Namespace) -> int:
    from ..analysis import summarize_by, to_markdown
    from ..core.config import load_config
    from ..core.paths import RunDirs, collect_provenance
    from ..experiments import ResultStore, RunnerOptions, expand, jobs_from_config, run_jobs
    from ..experiments.sweep import SweepSpec

    if args.config:
        config = load_config(args.config, overrides=args.overrides)
        spec, jobs = jobs_from_config(config)
    else:
        spec = SweepSpec(
            experiment="adhoc",
            morphologies=args.hands or ["birh_v1"],
            objects=args.objects or ["core15"],
            tasks=args.tasks or ["lift"],
            trials=args.trials or 1,
        )
        jobs = expand(spec)
        config = None

    if args.hands:
        spec.morphologies = args.hands
    if args.objects:
        spec.objects = args.objects
    if args.tasks:
        spec.tasks = args.tasks
    if args.trials:
        spec.trials = args.trials
    if any((args.hands, args.objects, args.tasks, args.trials)) and args.config:
        jobs = expand(spec)

    if args.dry_run:
        print(f"{len(jobs)} jobs across {len({j.group_key for j in jobs})} scene groups")
        for job in jobs[:10]:
            print(f"  {job}")
        if len(jobs) > 10:
            print(f"  ... and {len(jobs) - 10} more")
        return 0

    config_hash = config.hash[:8] if config else "adhoc"
    dirs = RunDirs.create(spec.experiment, config_hash, runs_root=args.runs_root)
    if config:
        config.save(dirs.config)
    dirs.provenance.write_text(
        json.dumps(collect_provenance({"sweep": spec.to_dict(), "n_jobs": len(jobs)}), indent=2, default=str),
        encoding="utf-8",
    )

    store = ResultStore(dirs.results)
    run_jobs(
        jobs, store=store, resume=not args.no_resume,
        options=RunnerOptions(workers=args.workers, trace_dir=dirs.traces, save_traces=spec.save_traces),
    )

    print(f"\nrun directory: {dirs.root}")
    df = store.to_dataframe()
    if not df.empty:
        print()
        print(to_markdown(summarize_by(df, "hand").set_index("hand")))
    return 0
