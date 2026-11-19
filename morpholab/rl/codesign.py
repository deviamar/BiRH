"""Morphology-control co-design (``docs/00-REVIEW.md`` §7, Study 5).

Outer loop: an optimizer proposes hands.  Inner loop: each hand gets a policy trained by
ES, warm-started from the best policy found so far (*policy inheritance*), and the hand
is scored by what its trained policy achieves.  Beside that score the loop also records
the hand's score under the fixed scripted controller, so the run answers the question
the study exists to ask: **how much of the fixed-controller ranking was an artifact of
the controller?**

Cost is the product of the two loops, so the defaults are small.  With popsize 8, 12
inner iterations, 16 perturbations and 4 objects, one outer generation is about 12,000
rollouts: a few minutes on 20 cores.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Sequence

import numpy as np

from ..core.logging import get_logger
from ..core.paths import RunDirs, collect_provenance
from ..experiments import ResultStore
from ..metrics.complexity import complexity
from ..morphology.space import ParamSpace
from ..morphology.validate import validate_hand
from ..optimize.base import Optimizer
from ..optimize.loop import write_json_atomic
from ..optimize.objective import GraspObjective, ObjectiveWeights
from .es import ESConfig, train_policy
from .evaluate import PolicyEvalConfig

log = get_logger(__name__)

__all__ = ["CodesignConfig", "codesign"]


@dataclass(frozen=True)
class CodesignConfig:
    objects: Sequence[str] = ("quick",)
    tasks: Sequence[str] = ("lift",)
    generations: int = 10
    inner: ESConfig = field(default_factory=lambda: ESConfig(iterations=12, popsize=16))
    eval: PolicyEvalConfig = field(default_factory=PolicyEvalConfig)
    inherit_policy: bool = True
    complexity_weight: float = 0.0
    complexity_ref: float = 12.65
    fixed_controller: str = "ramp"
    workers: int = 1
    seed: int = 0

    def to_dict(self) -> dict[str, Any]:
        return {"objects": list(self.objects), "tasks": list(self.tasks), "generations": self.generations,
                "inner": self.inner.to_dict(), "eval": self.eval.to_dict(), "inherit_policy": self.inherit_policy,
                "complexity_weight": self.complexity_weight, "fixed_controller": self.fixed_controller,
                "workers": self.workers, "seed": self.seed}


def codesign(space: ParamSpace, optimizer: Optimizer, cfg: CodesignConfig, run_dirs: RunDirs, *, resume: bool = True) -> dict[str, Any]:
    """Run the two-loop search. Returns the final summary (also written to ``codesign.json``)."""
    if optimizer.n_objectives != 1:
        raise ValueError("codesign uses a scalar objective; pick a single-objective optimizer")
    store = ResultStore(run_dirs.results)
    fixed = GraspObjective(
        space=space, store=store, objects=cfg.objects, tasks=cfg.tasks, controller=cfg.fixed_controller,
        trials=cfg.eval.trials, weights=ObjectiveWeights(quality=0.0), workers=cfg.workers,
        experiment="codesign-fixed", scene_options=dict(cfg.eval.scene_options),
    )
    ckpt = run_dirs.checkpoints / "codesign.json"
    hist_path = run_dirs.root / "history.jsonl"
    start_gen, lineage, history = 0, None, []
    if resume and ckpt.exists():
        st = json.loads(ckpt.read_text(encoding="utf-8"))
        optimizer.load_state(st["optimizer"])
        start_gen = int(st["generation"])
        lineage = None if st.get("lineage") is None else np.array(st["lineage"], float)
        if hist_path.exists():
            history = [json.loads(l) for l in hist_path.read_text(encoding="utf-8").splitlines() if l.strip()]
        log.info("resuming co-design from generation %d", start_gen)
    else:
        (run_dirs.root / "codesign_config.json").write_text(json.dumps(collect_provenance({
            "space": space.name, "optimizer": optimizer.name, "config": cfg.to_dict()}), indent=2, default=str),
            encoding="utf-8")

    best = max(history, key=lambda r: r["learned_reward"]) if history else None
    started = time.perf_counter()
    for gen in range(start_gen, cfg.generations):
        X = optimizer.ask()
        F_fixed, infos = fixed.evaluate(X)
        losses = []
        for k, (x, info) in enumerate(zip(X, infos)):
            spec = space.decode(x).renamed(f"cand_{info['morphology_id'][:8]}")
            rec: dict[str, Any] = {
                "generation": gen, "index": k, "morphology_id": spec.morphology_id, "x": x.tolist(),
                "complexity": complexity(spec), "valid": info["valid"],
                "fixed_success": info.get("success", 0.0), "fixed_performance": info.get("performance", 0.0),
            }
            if not validate_hand(spec).ok:
                rec.update(learned_reward=-1.0, loss=2.0)
            else:
                inner_dir = run_dirs.checkpoints / f"policy_{spec.morphology_id}"
                res = train_policy(
                    spec, cfg.objects, cfg.tasks, cfg.inner, cfg.eval, workers=cfg.workers,
                    init_params=lineage if cfg.inherit_policy else None, checkpoint_dir=inner_dir,
                )
                rec["learned_reward"] = res.best_reward
                rec["loss"] = -res.best_reward + cfg.complexity_weight * rec["complexity"] / cfg.complexity_ref
                store.save_morphology(spec)
                if best is None or res.best_reward > best["learned_reward"]:
                    best = rec
                    lineage = res.best_params.copy()
                    write_json_atomic(run_dirs.root / "best_hand.json", spec.to_dict())
                    write_json_atomic(run_dirs.root / "best_policy.json", res.policy_dict)
            losses.append([rec["loss"]])
            history.append(rec)
            with hist_path.open("a", encoding="utf-8") as fh:
                fh.write(json.dumps(rec, default=float) + "\n")
        optimizer.tell(X, np.array(losses))
        write_json_atomic(ckpt, {"generation": gen + 1, "optimizer": optimizer.state(),
                                 "lineage": None if lineage is None else lineage.tolist()})
        valid = [r for r in history if r["valid"] and r["generation"] == gen]
        log.info("codesign gen %d/%d  best learned %.3f  gen mean learned %.3f  gen mean fixed %.3f  (%.0fs)",
                 gen + 1, cfg.generations, best["learned_reward"] if best else float("nan"),
                 float(np.mean([r["learned_reward"] for r in valid])) if valid else float("nan"),
                 float(np.mean([r["fixed_performance"] for r in valid])) if valid else float("nan"),
                 time.perf_counter() - started)

    summary = _summarize(history)
    write_json_atomic(run_dirs.root / "codesign.json", summary)
    return summary


def _summarize(history: list[dict[str, Any]]) -> dict[str, Any]:
    """Spearman rank agreement between fixed-controller and learned-controller scores:
    the number Study 5 is for."""
    valid = [r for r in history if r.get("valid")]
    out: dict[str, Any] = {"n_hands": len(valid)}
    if len(valid) >= 3:
        from scipy.stats import spearmanr

        rho, p = spearmanr([r["fixed_performance"] for r in valid], [r["learned_reward"] for r in valid])
        out.update(rank_correlation=float(rho), p_value=float(p))
    if valid:
        best = max(valid, key=lambda r: r["learned_reward"])
        out["best"] = {k: best[k] for k in ("morphology_id", "learned_reward", "fixed_performance", "complexity")}
        best_fixed = max(valid, key=lambda r: r["fixed_performance"])
        out["best_under_fixed_controller"] = {k: best_fixed[k] for k in ("morphology_id", "learned_reward", "fixed_performance")}
    return out
