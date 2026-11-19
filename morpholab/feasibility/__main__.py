"""``python -m morpholab.feasibility HAND [HAND ...]``

HAND is a preset name, a morphology JSON path, or a directory of them.  ``--matrix`` scores
every material x actuator pairing; ``--csv`` writes one row per hand (and pairing).
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

from .assess import assess, feasibility_matrix
from .catalog import ACTUATORS, MATERIALS


def _hands(names: list[str]):
    from ..experiments.runner import _resolve_morphology
    from ..morphology.spec import HandSpec

    for name in names:
        p = Path(name)
        if p.is_dir():
            for f in sorted(p.glob("*.json")):
                yield HandSpec.from_dict(json.loads(f.read_text(encoding="utf-8"))).renamed(f.stem)
        else:
            yield _resolve_morphology(name, None)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m morpholab.feasibility", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("hands", nargs="+", help="preset names, morphology JSON files, or directories of them")
    ap.add_argument("--material", default="pla", choices=sorted(MATERIALS))
    ap.add_argument("--actuator", default="xl330_m288", choices=sorted(ACTUATORS))
    ap.add_argument("--pad", default="silicone_dragonskin10", choices=sorted(MATERIALS))
    ap.add_argument("--matrix", action="store_true", help="score every material x actuator pairing")
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--csv", metavar="PATH")
    ap.add_argument("--list", action="store_true", help="list the catalogs and exit")
    args = ap.parse_args(argv)

    if args.list:
        print("materials:")
        for m in MATERIALS.values():
            print(f"  {m.name:<24} {m.kind:<5} {m.process:<9} E={m.youngs_modulus:.2g} Pa  min wall {m.min_wall * 1e3:.1f} mm  {m.doc}")
        print("actuators:")
        for a in ACTUATORS.values():
            print(f"  {a.name:<18} {a.kind:<9} {a.torque:>5.2f} {'N' if a.kind == 'linear' else 'N.m'}  {a.mass * 1e3:>4.0f} g  {a.doc}")
        return 0

    rows = []
    for spec in _hands(args.hands):
        if args.matrix:
            mat_rows = feasibility_matrix(spec, pad=args.pad)
            if not args.json:
                print(f"\n{spec.name} ({spec.morphology_id[:8]}): material x actuator")
                for r in mat_rows:
                    flag = "ok  " if r["feasible"] else "FAIL"
                    print(f"  [{flag}] {r['material']:<24} {r['actuator']:<18} score {r['score']:.2f}  "
                          f"{r['mass_kg'] * 1e3:.0f} g  {', '.join(r['fails'])}")
            rows += [{"hand": spec.name, "morphology_id": spec.morphology_id, **r} for r in mat_rows]
        else:
            rep = assess(spec, material=args.material, actuator=args.actuator, pad=args.pad)
            if args.json:
                print(json.dumps(rep.to_dict(), indent=1))
            else:
                print(rep.describe())
            rows.append({"hand": spec.name, "morphology_id": spec.morphology_id, "material": args.material,
                         "actuator": args.actuator, "feasible": rep.feasible, "score": rep.score,
                         "n_fail": rep.n_fail, "n_warn": rep.n_warn, **rep.estimates,
                         "fails": ";".join(c.name for c in rep.checks if c.status == "fail")})
    if args.json and args.matrix:
        print(json.dumps(rows, indent=1))
    if args.csv:
        keys = sorted({k for r in rows for k in r})
        with open(args.csv, "w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=keys)
            w.writeheader()
            for r in rows:
                w.writerow({k: (";".join(v) if isinstance(v, list) else v) for k, v in r.items()})
        print(f"wrote {args.csv} ({len(rows)} rows)", file=sys.stderr)
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
