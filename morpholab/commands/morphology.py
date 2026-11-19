"""``morpholab morphology`` -- inspect, build, render, sample and validate hands."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np


def add_arguments(parser: argparse.ArgumentParser) -> None:
    sub = parser.add_subparsers(dest="action", metavar="ACTION", required=True)

    p_show = sub.add_parser("show", help="describe a hand, optionally rendering it")
    p_show.add_argument("name", help="preset name, or a path to a saved morphology JSON")
    p_show.add_argument("--render", metavar="PATH", help="write a PNG (open / half / closed)")
    p_show.add_argument("--camera", default="front", help="camera preset or model camera name")
    p_show.add_argument("--size", default="640x480", help="render size, WIDTHxHEIGHT")
    p_show.add_argument("--json", action="store_true", help="print the spec as JSON")

    p_xml = sub.add_parser("xml", help="print or save the generated MJCF")
    p_xml.add_argument("name")
    p_xml.add_argument("-o", "--output", metavar="PATH", help="write to a file instead of stdout")
    p_xml.add_argument("--standalone", action="store_true", help="include scene scaffolding")

    p_val = sub.add_parser("validate", help="run feasibility checks")
    p_val.add_argument("name")

    p_sample = sub.add_parser("sample", help="draw random hands from a parameter space")
    p_sample.add_argument("--space", default="three_finger_v1")
    p_sample.add_argument("-n", type=int, default=5)
    p_sample.add_argument("--seed", type=int, default=0)
    p_sample.add_argument("--save", metavar="DIR", help="write each sampled hand as JSON")
    p_sample.add_argument("--render", metavar="PATH", help="write a contact sheet PNG")

    p_space = sub.add_parser("space", help="describe a parameter space")
    p_space.add_argument("name", nargs="?", default="three_finger_v1")


def run(args: argparse.Namespace) -> int:
    return {
        "show": _show, "xml": _xml, "validate": _validate,
        "sample": _sample, "space": _space,
    }[args.action](args)


def _load(name: str):
    """Resolve a preset name or a morphology JSON path to a ``HandSpec``."""
    from ..morphology import HandSpec, get_preset

    path = Path(name)
    if path.exists() and path.suffix == ".json":
        return HandSpec.from_dict(json.loads(path.read_text(encoding="utf-8")))
    return get_preset(name)


def _complexity_line(spec) -> str:
    from ..metrics.complexity import complexity  # noqa: PLC0415 - optional at this milestone

    return f"  complexity  {complexity(spec):.2f}"


def _show(args: argparse.Namespace) -> int:
    from ..morphology import build_hand, validate_hand

    spec = _load(args.name)
    if args.json:
        print(json.dumps(spec.to_dict(), indent=2, sort_keys=True))
        return 0

    print(spec.describe())
    try:
        print(_complexity_line(spec))
    except Exception:  # noqa: BLE001 - metrics land in a later milestone
        pass

    build = build_hand(spec, standalone=True)
    print(f"  model       {build.model.summary()}")

    report = validate_hand(spec)
    if report.issues:
        print("  validation:")
        for issue in report.issues:
            print(f"    {issue}")
    else:
        print("  validation  no issues")

    if args.render:
        _render_postures(build, args.render, args.camera, args.size)
        print(f"  rendered    {args.render}")
    return 0


def _render_postures(build, path: str, camera: str, size: str) -> None:
    """Render the hand open, half closed and fully closed, side by side."""
    import mujoco

    from ..viz.render import Renderer, save_png, tile

    width, _, height = size.partition("x")
    model = build.model.compile()
    data = mujoco.MjData(model)
    lo, hi = model.actuator_ctrlrange[:, 0], model.actuator_ctrlrange[:, 1]

    frames = []
    with Renderer(model, int(width), int(height)) as renderer:
        for fraction in (0.0, 0.5, 1.0):
            mujoco.mj_resetData(model, data)
            data.ctrl[:] = lo + fraction * (hi - lo)
            for _ in range(2500):
                mujoco.mj_step(model, data)
            frames.append(renderer.frame(data, camera=camera))
    save_png(tile(frames, columns=3), path)


def _xml(args: argparse.Namespace) -> int:
    from ..morphology import build_hand

    build = build_hand(_load(args.name), standalone=args.standalone)
    xml = build.model.to_xml()
    if args.output:
        Path(args.output).write_text(xml, encoding="utf-8")
        print(f"wrote {args.output} ({len(xml.splitlines())} lines)")
    else:
        print(xml)
    return 0


def _validate(args: argparse.Namespace) -> int:
    from ..morphology import build_hand, validate_hand

    spec = _load(args.name)
    report = validate_hand(spec)
    print(f"{spec.name}: {report.describe()}")
    try:
        build_hand(spec).compile()
        print("compiles: yes")
    except Exception as exc:  # noqa: BLE001 - reporting the failure is the point
        print(f"compiles: NO\n{exc}")
        return 1
    return 0 if report.ok else 1


def _sample(args: argparse.Namespace) -> int:
    from ..morphology import build_hand, get_space, validate_hand

    space = get_space(args.space)
    rng = np.random.default_rng(args.seed)
    specs = space.sample_many(rng, args.n)

    for i, spec in enumerate(specs):
        report = validate_hand(spec)
        flags = "" if not report.issues else "  " + ",".join(sorted(set(report.codes)))
        gaps = [f"{math.degrees(d.mount.azimuth):.0f}" for d in spec.digits]
        print(
            f"[{i}] {spec.morphology_id}  azimuths {'/'.join(gaps)}°  "
            f"palm {spec.palm.radius * 100:.1f}cm  digit {spec.digits[0].total_length * 100:.1f}cm  "
            f"acts {spec.n_actuators}{flags}"
        )
        if args.save:
            out = Path(args.save) / f"{spec.morphology_id}.json"
            out.parent.mkdir(parents=True, exist_ok=True)
            out.write_text(json.dumps(spec.to_dict(), indent=2, sort_keys=True), encoding="utf-8")

    if args.render:
        import mujoco

        from ..viz.render import Renderer, save_png, tile

        frames = []
        for spec in specs:
            model = build_hand(spec, standalone=True).model.compile()
            data = mujoco.MjData(model)
            data.ctrl[:] = model.actuator_ctrlrange[:, 1]
            for _ in range(2000):
                mujoco.mj_step(model, data)
            with Renderer(model, 400, 320) as r:
                frames.append(r.frame(data, camera="front"))
        save_png(tile(frames), args.render)
        print(f"rendered {args.render}")
    return 0


def _space(args: argparse.Namespace) -> int:
    from ..morphology import get_space

    space = get_space(args.name)
    print(space.describe())
    if space.doc:
        print(f"\n{space.doc}")
    return 0
