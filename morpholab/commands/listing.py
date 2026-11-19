"""``morpholab list`` -- enumerate every registered component."""

from __future__ import annotations

import argparse

REGISTRIES = {
    "presets": ("morpholab.morphology.presets", "presets"),
    "spaces": ("morpholab.morphology.space", "spaces"),
    "objects": ("morpholab.objects.library", "objects"),
    "object-sets": ("morpholab.objects.library", "object_sets"),
    "tasks": ("morpholab.tasks.base", "tasks"),
    "controllers": ("morpholab.control.base", "controllers"),
    "metrics": ("morpholab.metrics.base", "metrics"),
    "perturbations": ("morpholab.sim.perturb", "perturbations"),
    "optimizers": ("morpholab.optimize", "optimizers"),
    "figures": ("morpholab.viz.figures", "figures"),
}


def add_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("kind", nargs="?", choices=sorted(REGISTRIES), help="one registry (default: all)")
    parser.add_argument("--tag", help="only entries carrying this tag")


def run(args: argparse.Namespace) -> int:
    import importlib

    kinds = [args.kind] if args.kind else sorted(REGISTRIES)
    for kind in kinds:
        module_path, attr = REGISTRIES[kind]
        try:
            registry = getattr(importlib.import_module(module_path), attr)
        except Exception as exc:  # noqa: BLE001 - a milestone may not be built yet
            print(f"\n{kind}:\n  (unavailable: {exc})")
            continue
        entries = registry.entries(tag=args.tag) if args.tag else registry.entries()
        print(f"\n{kind} ({len(entries)}):")
        if not entries:
            print("  (none)")
            continue
        width = max(len(e.name) for e in entries)
        for e in entries:
            tags = f"  [{','.join(e.tags)}]" if e.tags else ""
            print(f"  {e.name:<{width}}  {e.summary}{tags}")
    print()
    return 0
