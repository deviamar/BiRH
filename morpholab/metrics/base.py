"""Metric evaluation: turn a trace (and optionally a live simulator) into a result row.

Metrics declare what they need -- the spec alone, the trace, or a live simulator that can
be re-entered from the grasp snapshot -- so the evaluator knows which ones are free and
which cost simulation, and can skip the expensive family when a sweep does not want it.

Every metric contributes flat scalar keys to one dict.  That dict is the result row, and
the row is the only durable output analysis reads (``docs/01-ARCHITECTURE.md`` §4.4).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable

import numpy as np

from ..core.logging import get_logger
from ..core.registry import Registry
from ..core.types import FailureMode, Outcome
from ..sim.trace import EpisodeTrace

log = get_logger(__name__)

__all__ = ["MetricContext", "metrics", "evaluate_metrics", "MetricSpec"]

metrics: Registry["MetricSpec"] = Registry("metric")


@dataclass
class MetricContext:
    """Everything a metric may read."""

    trace: EpisodeTrace
    hand: Any
    obj: Any
    task: Any
    scene: Any = None
    rollout: Any = None
    """A live :class:`~morpholab.sim.engine.Rollout`, present only when simulation-based
    metrics are enabled. ``None`` means the expensive family is skipped."""

    options: dict[str, Any] = field(default_factory=dict)

    @property
    def has_simulator(self) -> bool:
        return self.rollout is not None

    @property
    def grasp_snapshot(self):
        return self.trace.snapshots.get("grasp")

    @property
    def grasp_contacts(self):
        return self.trace.contact_sets.get("grasp")


@dataclass(frozen=True)
class MetricSpec:
    """A registered metric: a function plus what it needs and what it produces."""

    fn: Callable[[MetricContext], dict[str, Any]]
    needs: tuple[str, ...] = ("trace",)
    """Any of ``spec``, ``trace``, ``simulator``."""

    keys: tuple[str, ...] = ()
    cost: str = "free"
    """``free`` (arithmetic on the spec or trace) or ``sim`` (re-enters the simulator)."""

    def __call__(self, ctx: MetricContext) -> dict[str, Any]:
        return self.fn(ctx)


def register_metric(
    name: str, *, needs: tuple[str, ...] = ("trace",), keys: tuple[str, ...] = (),
    cost: str = "free", doc: str = "", tags: tuple[str, ...] = (),
) -> Callable:
    def decorator(fn: Callable[[MetricContext], dict[str, Any]]) -> Callable:
        metrics.add(
            name, lambda: MetricSpec(fn=fn, needs=needs, keys=keys, cost=cost),
            doc=doc or (fn.__doc__ or "").strip().splitlines()[0] if fn.__doc__ else doc,
            tags=tags + (cost,),
        )
        return fn

    return decorator


def evaluate_metrics(
    ctx: MetricContext,
    *,
    names: list[str] | None = None,
    include_sim: bool = True,
) -> dict[str, Any]:
    """Run the selected metrics and merge their outputs into one flat row.

    A metric that raises is logged and skipped rather than killing the trial: losing one
    number is preferable to losing the trial, and the absence shows up as a null in the
    store rather than as a hole in the sweep.
    """
    selected = names if names is not None else metrics.names()
    row: dict[str, Any] = {}
    for name in selected:
        spec = metrics.create(name)
        if spec.cost == "sim" and (not include_sim or not ctx.has_simulator):
            continue
        try:
            row.update(spec(ctx))
        except Exception as exc:  # noqa: BLE001 - one bad metric must not lose the trial
            log.warning("metric %s failed: %s", name, exc)
            row[f"{name}_error"] = str(exc)[:200]
    return row


def base_row(ctx: MetricContext) -> dict[str, Any]:
    """Identifiers and outcome fields every result row carries."""
    trace = ctx.trace
    return {
        "hand": ctx.hand.name,
        "morphology_id": ctx.hand.morphology_id,
        "object": ctx.obj.name,
        "object_category": ctx.obj.category,
        "object_size_class": ctx.obj.size_class,
        "object_mass": ctx.obj.mass,
        "object_width": ctx.obj.width,
        "object_convex_hull": bool(ctx.obj.convex_hull_collision),
        "task": ctx.task.name,
        "grasp_type": ctx.task.grasp_type,
        "controller": trace.meta.get("controller", ""),
        "outcome": str(trace.outcome),
        "success": bool(trace.outcome == Outcome.SUCCESS),
        "admissible": bool(trace.outcome != Outcome.INADMISSIBLE),
        "failure_mode": str(trace.failure_mode),
        "failure_detail": trace.failure_detail,
        "sim_duration": trace.duration,
        "sim_steps": trace.steps,
        "wall_time": trace.wall_time,
    }
