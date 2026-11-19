"""``morpholab optimize`` -- morphology search, policy search, and co-design.

    morpholab optimize morphology --algorithm cmaes --generations 30 -j 20
    morpholab optimize morphology --algorithm nsga2 --multi --generations 40 -j 20
    morpholab optimize policy --hand birh_v1 --objects quick --iterations 50 -j 20
    morpholab optimize codesign --generations 10 -j 20
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def add_arguments(parser: argparse.ArgumentParser) -> None:
    subs = parser.add_subparsers(dest="action", metavar="ACTION", required=True)

    def common(p: argparse.ArgumentParser) -> None:
        p.add_argument("-j", "--workers", type=int, default=1)
        p.add_argument("--seed", type=int, default=0)
        p.add_argument("--runs-root", metavar="DIR")
        p.add_argument("--experiment", help="run directory name (default derives from the action)")
        p.add_argument("--no-resume", action="store_true")
        p.add_argument("--objects", nargs="+", default=["train"])
        p.add_argument("--tasks", nargs="+", default=["lift"])
        p.add_argument("--trials", type=int, default=2)

    m = subs.add_parser("morphology", help="search a parameter space with a fixed controller")
    common(m)
    m.add_argument("--space", default="three_finger_v1")
    m.add_argument("--algorithm", default="cmaes", help="random | es | cmaes | ga | nsga2")
    m.add_argument("--generations", type=int, default=20)
    m.add_argument("--popsize", type=int)
    m.add_argument("--sigma", type=float, default=0.3)
    m.add_argument("--controller", default="ramp")
    m.add_argument("--warm-start", metavar="PRESET", help="start from a preset's encoding")
    m.add_argument("--multi", action="store_true", help="two objectives: performance and complexity (nsga2)")
    m.add_argument("--w-quality", type=float, default=0.3)
    m.add_argument("--w-complexity", type=float, default=0.0)
    m.add_argument("--w-feasibility", type=float, default=0.0)
    m.add_argument("--material", default="pla")
    m.add_argument("--actuator", default="xl330_spool")
    m.add_argument("--test-objects", nargs="+", default=["test"], help="held-out set reported at the end")

    p = subs.add_parser("policy", help="train a controller for one hand by evolution strategies")
    common(p)
    p.add_argument("--hand", default="birh_v1")
    p.add_argument("--iterations", type=int, default=50)
    p.add_argument("--popsize", type=int, default=16)
    p.add_argument("--sigma", type=float, default=0.05)
    p.add_argument("--lr", type=float, default=0.01)
    p.add_argument("--hidden", type=int, nargs="*", default=[32])
    p.add_argument("--no-residual", action="store_true", help="pure learned control, no closure baseline")
    p.add_argument("--init", metavar="POLICY_JSON", help="warm-start from a saved policy")

    c = subs.add_parser("codesign", help="evolve hands, training a policy for each (Study 5)")
    common(c)
    c.add_argument("--space", default="three_finger_v1")
    c.add_argument("--algorithm", default="cmaes")
    c.add_argument("--generations", type=int, default=10)
    c.add_argument("--popsize", type=int, default=8)
    c.add_argument("--sigma", type=float, default=0.3)
    c.add_argument("--inner-iterations", type=int, default=12)
    c.add_argument("--inner-popsize", type=int, default=16)
    c.add_argument("--hidden", type=int, nargs="*", default=[32])
    c.add_argument("--no-inherit", action="store_true", help="train every hand's policy from scratch")
    c.add_argument("--w-complexity", type=float, default=0.0)
    c.set_defaults(objects=["quick"])


def _run_dirs(args: argparse.Namespace, default_name: str, config: dict):
    from ..core.ids import content_hash
    from ..core.paths import RunDirs

    name = args.experiment or default_name
    return RunDirs.create(name, content_hash(config)[:8], runs_root=args.runs_root)


def _make_optimizer(name: str, dim: int, *, popsize, sigma, seed, x0=None, multi=False):
    from ..optimize import get_optimizer

    kwargs = {"dim": dim, "seed": seed}
    if popsize:
        kwargs["popsize"] = popsize
    if name in ("cmaes", "es"):
        kwargs["sigma"] = sigma
    if name in ("cmaes", "es", "ga", "nsga2") and x0 is not None:
        kwargs["x0"] = x0
    if name == "nsga2":
        kwargs["n_obj"] = 2 if multi else 1
    return get_optimizer(name, **kwargs)


def _morphology(args: argparse.Namespace) -> int:
    import numpy as np

    from ..experiments import ResultStore
    from ..morphology import get_preset, get_space
    from ..optimize import GraspObjective, ObjectiveWeights, optimize
    from ..optimize.objective import aggregate_rows

    space = get_space(args.space)
    x0 = None
    if args.warm_start:
        x0 = space.encode(get_preset(args.warm_start))
    config = {"action": "morphology", "space": args.space, "algorithm": args.algorithm, "objects": args.objects,
              "tasks": args.tasks, "trials": args.trials, "controller": args.controller, "multi": args.multi,
              "w": [args.w_quality, args.w_complexity, args.w_feasibility], "seed": args.seed}
    dirs = _run_dirs(args, f"optimize-{args.algorithm}", config)
    store = ResultStore(dirs.results)
    weights = ObjectiveWeights(quality=args.w_quality, complexity=0.0 if args.multi else args.w_complexity,
                               feasibility=args.w_feasibility)
    objective = GraspObjective(
        space=space, store=store, objects=args.objects, tasks=args.tasks, controller=args.controller,
        trials=args.trials, weights=weights, workers=args.workers, experiment=dirs.root.name,
        multi_objective=args.multi,
        feasibility_kwargs={"material": args.material, "actuator": args.actuator} if args.w_feasibility > 0 else None,
    )
    if args.multi and args.algorithm != "nsga2":
        raise SystemExit("--multi needs --algorithm nsga2")
    opt = _make_optimizer(args.algorithm, space.dim, popsize=args.popsize, sigma=args.sigma, seed=args.seed,
                          x0=x0, multi=args.multi)
    result = optimize(space, objective, opt, generations=args.generations, run_dirs=dirs,
                      resume=not args.no_resume, label=f"{args.algorithm}/{args.space}")

    print(f"\nrun directory: {dirs.root}")
    if result.best_x is None:
        print("no hands evaluated")
        return 1
    best = space.decode(result.best_x)
    print(f"best loss {result.best_loss:.4f}  ({result.n_evaluated} hands evaluated)")
    print(f"  performance {result.best_info.get('performance', 0):.3f}  success {result.best_info.get('success', 0):.3f}  "
          f"complexity {result.best_info.get('complexity', 0):.2f}")
    print("  " + "\n  ".join(f"{k:<18} {v}" for k, v in best.meta.get("params", {}).items()))
    print(f"  saved to {dirs.root / 'best_hand.json'}")
    if result.front:
        print(f"Pareto front: {len(result.front)} hands, see {dirs.root / 'pareto.json'}")

    # Held-out evaluation: the number that separates optimization from overfitting.
    if args.test_objects:
        test = GraspObjective(space=space, store=store, objects=args.test_objects, tasks=args.tasks,
                              controller=args.controller, trials=args.trials, weights=weights,
                              workers=args.workers, experiment=dirs.root.name + "-test")
        F, infos = test.evaluate(np.array([result.best_x]))
        print(f"held-out {args.test_objects}: success {infos[0].get('success', 0):.3f}  "
              f"performance {infos[0].get('performance', 0):.3f}  (train success {result.best_info.get('success', 0):.3f})")
        (dirs.root / "held_out.json").write_text(json.dumps(infos[0], indent=1, default=str), encoding="utf-8")
    return 0


def _policy(args: argparse.Namespace) -> int:
    import numpy as np

    from ..experiments.runner import _resolve_morphology
    from ..rl import ESConfig, PolicyEvalConfig, train_policy

    hand = _resolve_morphology(args.hand, None)
    config = {"action": "policy", "hand": hand.morphology_id, "objects": args.objects, "tasks": args.tasks,
              "iterations": args.iterations, "popsize": args.popsize, "hidden": args.hidden, "seed": args.seed}
    dirs = _run_dirs(args, f"policy-{hand.name}", config)
    init = None
    if args.init:
        init = np.array(json.loads(Path(args.init).read_text(encoding="utf-8"))["params"], float)
    cfg = ESConfig(popsize=args.popsize, sigma=args.sigma, lr=args.lr, iterations=args.iterations, seed=args.seed)
    ev = PolicyEvalConfig(hidden=tuple(args.hidden), residual=not args.no_residual, trials=args.trials)
    res = train_policy(hand, args.objects, args.tasks, cfg, ev, workers=args.workers, init_params=init,
                       checkpoint_dir=dirs.checkpoints)
    out = dirs.root / "best_policy.json"
    out.write_text(json.dumps(res.policy_dict), encoding="utf-8")
    print(f"\nrun directory: {dirs.root}\nbest reward {res.best_reward:.3f}\npolicy saved to {out}")
    print("evaluate it with:  morpholab run --hand", args.hand, "--controller policy  (after registering; see docs/06)")
    return 0


def _codesign(args: argparse.Namespace) -> int:
    from ..morphology import get_space
    from ..rl import CodesignConfig, ESConfig, PolicyEvalConfig, codesign

    space = get_space(args.space)
    config = {"action": "codesign", "space": args.space, "objects": args.objects, "tasks": args.tasks,
              "generations": args.generations, "popsize": args.popsize, "inner": args.inner_iterations, "seed": args.seed}
    dirs = _run_dirs(args, f"codesign-{args.algorithm}", config)
    opt = _make_optimizer(args.algorithm, space.dim, popsize=args.popsize, sigma=args.sigma, seed=args.seed)
    cfg = CodesignConfig(
        objects=args.objects, tasks=args.tasks, generations=args.generations,
        inner=ESConfig(iterations=args.inner_iterations, popsize=args.inner_popsize, seed=args.seed),
        eval=PolicyEvalConfig(hidden=tuple(args.hidden), trials=args.trials),
        inherit_policy=not args.no_inherit, complexity_weight=args.w_complexity, workers=args.workers, seed=args.seed,
    )
    summary = codesign(space, opt, cfg, dirs, resume=not args.no_resume)
    print(f"\nrun directory: {dirs.root}")
    print(json.dumps(summary, indent=1))
    return 0


def run(args: argparse.Namespace) -> int:
    return {"morphology": _morphology, "policy": _policy, "codesign": _codesign}[args.action](args)
