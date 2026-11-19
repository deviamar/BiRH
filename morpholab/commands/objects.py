"""``morpholab objects`` -- inspect the object library."""

from __future__ import annotations

import argparse


def add_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("action", nargs="?", default="list", choices=["list", "sets", "categories"])
    parser.add_argument("--set", dest="set_name", help="only objects in this named set")
    parser.add_argument("--tag", help="only objects carrying this tag")


def run(args: argparse.Namespace) -> int:
    from ..objects import categories, get_object, get_object_set, object_sets, objects

    if args.action == "sets":
        for entry in object_sets.entries():
            members = object_sets.create(entry.name)
            print(f"{entry.name:<12} {len(members):>3} objects  {entry.summary}")
        return 0

    if args.action == "categories":
        for category, names in categories().items():
            print(f"{category:<12} {len(names):>3}  {', '.join(names)}")
        return 0

    names = (
        [o.name for o in get_object_set(args.set_name)] if args.set_name
        else objects.names(tag=args.tag) if args.tag
        else objects.names()
    )
    print(f"{'name':<18} {'category':<10} {'size':<7} {'dimensions':<20} {'mass':<8} width")
    print("-" * 92)
    for name in names:
        try:
            print(get_object(name).describe())
        except Exception as exc:  # noqa: BLE001 - a missing mesh asset must not break listing
            print(f"{name:<18} (unavailable: {exc})")
    print(f"\n{len(names)} objects")
    return 0
