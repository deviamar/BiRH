"""Command line interface.

Subcommands are declared in :data:`COMMANDS` and their handlers imported lazily, so
``morpholab doctor`` does not pay for MuJoCo, matplotlib or pandas import time, and a
missing optional dependency only breaks the command that needs it.
"""

from __future__ import annotations

import argparse
import importlib
import sys
from typing import Any, Callable, NamedTuple

from . import __version__
from .core.logging import setup_logging


class Command(NamedTuple):
    """A lazily-imported subcommand."""

    name: str
    help: str
    module: str          # module implementing add_arguments(parser) and run(args)
    group: str = "core"


COMMANDS: tuple[Command, ...] = (
    Command("doctor", "Check the environment and report platform capabilities", "morpholab.commands.doctor"),
    Command("list", "List registered components (objects, tasks, controllers, ...)", "morpholab.commands.listing"),
    Command("morphology", "Inspect, build, render or sample hand morphologies", "morpholab.commands.morphology", "design"),
    Command("objects", "Inspect the object library", "morpholab.commands.objects", "design"),
    Command("run", "Run a single grasp trial", "morpholab.commands.run", "experiment"),
    Command("sweep", "Run a parameter sweep from a config file", "morpholab.commands.sweep", "experiment"),
    Command("optimize", "Optimize morphology against an objective", "morpholab.commands.optimize", "experiment"),
    Command("analyze", "Compute statistics and tables from a run", "morpholab.commands.analyze", "analysis"),
    Command("figures", "Generate figures from stored results", "morpholab.commands.figures", "analysis"),
    Command("render", "Render a trial to images or video", "morpholab.commands.render", "analysis"),
    Command("report", "Generate an experiment report", "morpholab.commands.report", "analysis"),
)


def _load(cmd: Command) -> Any:
    try:
        return importlib.import_module(cmd.module)
    except ModuleNotFoundError as exc:
        if exc.name and not exc.name.startswith("morpholab"):
            raise SystemExit(
                f"'{cmd.name}' needs the optional dependency '{exc.name}'.\n"
                f"Install it with:  pip install 'morpholab[all]'"
            ) from exc
        raise SystemExit(f"'{cmd.name}' is not available in this build ({exc})") from exc


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="morpholab",
        description="Morphology Lab - robotic hand morphology optimization platform",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=_epilog(),
    )
    parser.add_argument("--version", action="version", version=f"morpholab {__version__}")
    parser.add_argument("-v", "--verbose", action="count", default=0, help="-v for DEBUG logging")
    parser.add_argument("-q", "--quiet", action="store_true", help="suppress console logging")

    subs = parser.add_subparsers(dest="command", metavar="COMMAND")
    for cmd in COMMANDS:
        sub = subs.add_parser(cmd.name, help=cmd.help, description=cmd.help)
        sub.set_defaults(_command=cmd)
        # Arguments are added lazily on dispatch (see main); to keep --help useful per
        # subcommand we import only that subcommand's module here.
        try:
            module = importlib.import_module(cmd.module)
        except Exception:  # noqa: BLE001 - unavailable commands still appear in help
            sub.add_argument("args", nargs=argparse.REMAINDER, help=argparse.SUPPRESS)
            continue
        add_arguments: Callable[[argparse.ArgumentParser], None] | None = getattr(
            module, "add_arguments", None
        )
        if add_arguments is not None:
            add_arguments(sub)
    return parser


def _epilog() -> str:
    groups: dict[str, list[Command]] = {}
    for cmd in COMMANDS:
        groups.setdefault(cmd.group, []).append(cmd)
    lines = ["", "commands by group:"]
    for group, cmds in groups.items():
        lines.append(f"  {group}:")
        width = max(len(c.name) for c in cmds)
        lines += [f"    {c.name:<{width}}  {c.help}" for c in cmds]
    lines += ["", "examples:", "  morpholab doctor",
              "  morpholab morphology show birh_v1 --render hand.png",
              "  morpholab run --hand birh_v1 --object sphere_medium --task lift",
              "  morpholab sweep configs/studies/ablation.yaml -j 20"]
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    if getattr(args, "command", None) is None:
        parser.print_help()
        return 0

    level = "DEBUG" if args.verbose else "INFO"
    setup_logging(level, quiet=args.quiet)

    module = _load(args._command)
    run: Callable[[argparse.Namespace], int | None] | None = getattr(module, "run", None)
    if run is None:
        raise SystemExit(f"command module {args._command.module} defines no run()")
    try:
        return int(run(args) or 0)
    except KeyboardInterrupt:
        print("\ninterrupted", file=sys.stderr)
        return 130


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
