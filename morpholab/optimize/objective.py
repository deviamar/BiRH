"""The objective: a unit-cube point becomes a hand, the hand becomes trials, the trials
become a loss (``docs/06-OPTIMIZATION-AND-RL.md`` §1).

Every evaluation goes through the experiment runner and its result store, so an
optimization run is *also* a sweep: every hand it ever tried is on disk with its full
result rows, the morphology JSON is saved beside them, and re-evaluating a point the
optimizer already visited costs nothing because the store resumes it.

Performance is aggregated **per object first, then across objects**, so an object set
with many trials of one shape does not dominate.  The held-out ``test`` set is never
passed here by the CLI; reporting it separately is what distinguishes optimization from
overfitting (``docs/00-REVIEW.md`` §7, Study 3).
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Sequence

import numpy as np

from ..core.logging import get_logger
from ..experiments import Job, ResultStore, RunnerOptions, run_jobs
from ..experiments.sweep import _resolve_objects, _resolve_tasks
from ..metrics.complexity import complexity
from ..morphology.space import ParamSpace
from ..morphology.validate import validate_hand

log = get_logger(__name__)

__all__ = ["ObjectiveWeights", "GraspObjective", "aggregate_rows"]


@dataclass(frozen=True)
class ObjectiveWeights:
    """How a result row set becomes one number. Stated, editable, recorded with the run."""

    success: float = 1.0
    quality: float = 0.3
    """Weight on Ferrari-Canny epsilon, clipped at ``eps_ref``: a graded signal that
    distinguishes two grasps that both 'succeeded'."""

    lift: float = 0.0
    """Weight on lift height as a fraction of the task target."""

    complexity: float = 0.0
    """Penalty per unit of ``complexity / complexity_ref``. Zero when the run is
    multi-objective, where complexity is a separate axis."""

    feasibility: float = 0.0
    """Penalty on ``1 - feasibility_score`` from ``morpholab.feasibility``."""

    eps_ref: float = 0.3
    complexity_ref: float = 12.65
    """birh_v1's complexity, so the penalty reads as 'multiples of the reference hand'."""

    infeasible_loss: float = 2.0
    """Loss assigned to a design the validator rejects (never simulated)."""

    def to_dict(self) -> dict[str, float]:
        return dict(vars(self))


def aggregate_rows(rows: Sequence[dict[str, Any]], weights: ObjectiveWeights) -> dict[str, float]:
    """Per-object means, then the mean across objects, then the weighted performance."""
    by_obj: dict[str, list[dict[str, Any]]] = {}
    for r in rows:
        if r.get("outcome") == "error":
            continue
        by_obj.setdefault(str(r.get("object")), []).append(r)
    if not by_obj:
        return {"performance": 0.0, "success": 0.0, "epsilon": 0.0, "lift": 0.0, "n_rows": 0, "n_objects": 0}

    def mean_of(key: str, rs: list[dict[str, Any]]) -> float:
        vals = [float(r.get(key) or 0.0) for r in rs]
        return float(np.mean(vals)) if vals else 0.0

    succ = [mean_of("success", rs) for rs in by_obj.values()]
    eps = [float(np.mean([min(float(r.get("grasp_epsilon") or 0.0) / weights.eps_ref, 1.0) for r in rs]))
           for rs in by_obj.values()]
    lift = [mean_of("lift_height", rs) for rs in by_obj.values()]
    s, e, l = float(np.mean(succ)), float(np.mean(eps)), float(np.mean(lift))
    perf = weights.success * s + weights.quality * e + weights.lift * min(l / 0.12, 1.0)
    return {"performance": perf, "success": s, "epsilon": e, "lift": l,
            "n_rows": sum(len(v) for v in by_obj.values()), "n_objects": len(by_obj)}


@dataclass
class GraspObjective:
    """Evaluate unit-cube points as hands on a grasp sweep."""

    space: ParamSpace
    store: ResultStore
    objects: Sequence[str] = ("train",)
    tasks: Sequence[str] = ("lift",)
    controller: str = "ramp"
    trials: int = 2
    weights: ObjectiveWeights = field(default_factory=ObjectiveWeights)
    workers: int = 1
    experiment: str = "optimize"
    scene_options: dict[str, Any] = field(default_factory=dict)
    controller_kwargs: dict[str, Any] = field(default_factory=dict)
    multi_objective: bool = False
    """When true, ``evaluate`` returns ``[-performance, complexity]`` per point."""

    feasibility_kwargs: dict[str, Any] | None = None
    """``material`` / ``actuator`` / ``pad`` for ``morpholab.feasibility.assess``; the
    feasibility penalty is only applied when this is set."""

    job_timeout: float = 120.0
    _cache: dict[str, dict[str, Any]] = field(default_factory=dict, init=False, repr=False)

    @property
    def n_objectives(self) -> int:
        return 2 if self.multi_objective else 1

    def describe(self) -> dict[str, Any]:
        return {
            "space": self.space.name, "objects": list(self.objects), "tasks": list(self.tasks),
            "controller": self.controller, "trials": self.trials, "weights": self.weights.to_dict(),
            "multi_objective": self.multi_objective, "feasibility": self.feasibility_kwargs,
        }

    # -- evaluation ---------------------------------------------------------------------

    def evaluate(self, X: np.ndarray) -> tuple[np.ndarray, list[dict[str, Any]]]:
        """Losses (``(n,)`` or ``(n, 2)``) and one info dict per point."""
        X = np.asarray(X, float).reshape(-1, self.space.dim)
        started = time.perf_counter()
        specs, infos, jobs = [], [], []
        objects = _resolve_objects(list(self.objects))
        task_names = _resolve_tasks(list(self.tasks))

        for x in X:
            spec = self.space.decode(x)
            spec = spec.renamed(f"cand_{spec.morphology_id[:8]}")
            report = validate_hand(spec)
            info: dict[str, Any] = {
                "morphology_id": spec.morphology_id, "x": x.tolist(),
                "complexity": complexity(spec), "valid": report.ok,
                "issues": list(report.codes), "feasibility": 1.0,
            }
            if self.feasibility_kwargs is not None:
                from ..feasibility import assess

                fr = assess(spec, **self.feasibility_kwargs)
                info["feasibility"] = fr.score
                info["feasibility_fails"] = fr.n_fail
            specs.append(spec)
            infos.append(info)
            if not report.ok:
                continue
            if spec.morphology_id in self._cache:
                continue
            self.store.save_morphology(spec)
            for obj in objects:
                for task in task_names:
                    for trial in range(self.trials):
                        jobs.append(Job(
                            morphology=spec.morphology_id, obj=obj, task=task,
                            controller=self.controller, trial=trial, experiment=self.experiment,
                            controller_kwargs=dict(self.controller_kwargs),
                            scene_options=dict(self.scene_options),
                        ))

        rows_by_id: dict[str, list[dict[str, Any]]] = {}
        if jobs:
            results = run_jobs(
                jobs, store=self.store, resume=True,
                options=RunnerOptions(workers=self.workers, progress=False, job_timeout=self.job_timeout),
            )
            for r in results:
                rows_by_id.setdefault(str(r.row.get("morphology_id")), []).append(r.row)
            # Jobs skipped by resume have rows in the store but not in `results`.
            wanted = {j.morphology for j in jobs} - set(rows_by_id)
            if wanted:
                for row in self.store:
                    mid = str(row.get("morphology_id"))
                    if mid in wanted:
                        rows_by_id.setdefault(mid, []).append(row)

        losses = []
        for spec, info in zip(specs, infos):
            mid = spec.morphology_id
            if not info["valid"]:
                agg = {"performance": 0.0, "success": 0.0, "epsilon": 0.0, "lift": 0.0, "n_rows": 0}
                loss = self.weights.infeasible_loss + 0.1 * len(info["issues"])
            else:
                if mid not in self._cache:
                    self._cache[mid] = aggregate_rows(rows_by_id.get(mid, []), self.weights)
                agg = self._cache[mid]
                loss = -agg["performance"]
                loss += self.weights.complexity * info["complexity"] / self.weights.complexity_ref
                loss += self.weights.feasibility * (1.0 - info["feasibility"])
            info.update(agg)
            info["loss"] = float(loss)
            losses.append([loss, info["complexity"]] if self.multi_objective else [loss])

        log.info("evaluated %d hands (%d trials) in %.1fs", len(X), len(jobs), time.perf_counter() - started)
        return np.array(losses, float), infos
