"""The ask/evaluate/tell loop, with per-generation checkpoints.

Everything an optimization produces lives in one run directory:

    history.jsonl        one line per evaluated hand: generation, x, loss, metrics
    checkpoints/         optimizer state after each generation (atomic write)
    best_hand.json       the best hand so far, loadable by every other command
    pareto.json          the non-dominated set (multi-objective runs)
    morphologies/        every hand the objective simulated
    results.jsonl        every trial, exactly as a sweep would record it

A killed run resumes from its last checkpoint with no repeated simulation, because the
objective's result store already holds the rows for every hand it visited.
"""

from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

import numpy as np

from ..core.logging import get_logger
from ..core.paths import RunDirs, collect_provenance
from .base import Optimizer

log = get_logger(__name__)

__all__ = ["OptimizationResult", "optimize", "write_json_atomic"]


def write_json_atomic(path: Path, payload: Any) -> None:
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, indent=1, default=_jsonable), encoding="utf-8")
    os.replace(tmp, path)


def _jsonable(o: Any) -> Any:
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    if isinstance(o, np.bool_):
        return bool(o)
    return str(o)


@dataclass
class OptimizationResult:
    run_dir: Path
    generations: int
    best_x: np.ndarray | None
    best_loss: float | None
    best_info: dict[str, Any] = field(default_factory=dict)
    history: list[dict[str, Any]] = field(default_factory=list)
    front: list[dict[str, Any]] = field(default_factory=list)

    @property
    def n_evaluated(self) -> int:
        return len(self.history)


def optimize(
    space: Any,
    objective: Any,
    optimizer: Optimizer,
    *,
    generations: int,
    run_dirs: RunDirs,
    resume: bool = True,
    label: str = "",
    on_generation: Callable[[int, dict[str, Any]], None] | None = None,
) -> OptimizationResult:
    """Run ``optimizer`` against ``objective`` for ``generations`` generations."""
    if optimizer.n_objectives != objective.n_objectives:
        raise ValueError(
            f"optimizer {optimizer.name!r} expects {optimizer.n_objectives} objective(s) "
            f"but the objective provides {objective.n_objectives}; pass --multi for nsga2"
        )
    ckpt = run_dirs.checkpoints / "optimizer.json"
    hist_path = run_dirs.root / "history.jsonl"
    start_gen = 0
    history: list[dict[str, Any]] = []

    if resume and ckpt.exists():
        state = json.loads(ckpt.read_text(encoding="utf-8"))
        optimizer.load_state(state["optimizer"])
        start_gen = int(state["generation"])
        if hist_path.exists():
            history = [json.loads(l) for l in hist_path.read_text(encoding="utf-8").splitlines() if l.strip()]
        log.info("resuming %s from generation %d (%d hands evaluated)", label or optimizer.name, start_gen, len(history))
    else:
        (run_dirs.root / "optimization.json").write_text(json.dumps(collect_provenance({
            "label": label, "optimizer": optimizer.name, "space": space.name,
            "generations": generations, "objective": objective.describe(),
        }), indent=2, default=str), encoding="utf-8")

    best_x, best_loss, best_info = _best_from_history(history)
    started = time.perf_counter()

    for gen in range(start_gen, generations):
        t0 = time.perf_counter()
        X = optimizer.ask()
        F, infos = objective.evaluate(X)
        optimizer.tell(X, F)

        with hist_path.open("a", encoding="utf-8") as fh:
            for x, f, info in zip(X, F, infos):
                rec = {"generation": gen, "x": np.asarray(x).tolist(), "loss": float(f[0]),
                       "objectives": np.asarray(f).tolist(), **info}
                history.append(rec)
                fh.write(json.dumps(rec, default=_jsonable) + "\n")
                if best_loss is None or rec["loss"] < best_loss:
                    best_x, best_loss, best_info = np.asarray(x, float), rec["loss"], rec

        write_json_atomic(ckpt, {"generation": gen + 1, "optimizer": optimizer.state()})
        if best_x is not None:
            spec = space.decode(best_x)
            write_json_atomic(run_dirs.root / "best_hand.json", spec.to_dict())
        front: list[dict[str, Any]] = []
        if hasattr(optimizer, "front"):
            fx, ff = optimizer.front()
            front = [{"x": x.tolist(), "objectives": f.tolist()} for x, f in zip(fx, ff)]
            write_json_atomic(run_dirs.root / "pareto.json", {
                "front": front, "hypervolume": optimizer.hypervolume() if hasattr(optimizer, "hypervolume") else None,
            })

        gen_loss = F.reshape(len(F), -1)[:, 0]
        summary = {
            "generation": gen, "n": len(X), "best_loss": best_loss, "gen_best": float(gen_loss.min()),
            "gen_mean": float(gen_loss.mean()), "valid_fraction": float(np.mean([i.get("valid", True) for i in infos])),
            "elapsed": time.perf_counter() - started, "gen_seconds": time.perf_counter() - t0,
        }
        if front:
            summary["front_size"] = len(front)
            summary["hypervolume"] = optimizer.hypervolume() if hasattr(optimizer, "hypervolume") else None
        log.info(
            "gen %d/%d  best %.4f  gen best %.4f  mean %.4f  valid %.0f%%  (%.0fs)",
            gen + 1, generations, best_loss, summary["gen_best"], summary["gen_mean"],
            100 * summary["valid_fraction"], summary["gen_seconds"],
        )
        if on_generation:
            on_generation(gen, summary)

    front_out: list[dict[str, Any]] = []
    if hasattr(optimizer, "front"):
        fx, ff = optimizer.front()
        front_out = [{"x": x.tolist(), "objectives": f.tolist()} for x, f in zip(fx, ff)]
    return OptimizationResult(
        run_dir=run_dirs.root, generations=generations, best_x=best_x, best_loss=best_loss,
        best_info=best_info, history=history, front=front_out,
    )


def _best_from_history(history: list[dict[str, Any]]) -> tuple[np.ndarray | None, float | None, dict[str, Any]]:
    if not history:
        return None, None, {}
    rec = min(history, key=lambda r: r["loss"])
    return np.array(rec["x"], float), float(rec["loss"]), rec
