"""Jobs: one trial, identified by its content.

``job_id = hash(everything that determines the result)``.  Two consequences the platform
depends on (``docs/01-ARCHITECTURE.md`` §8):

* **Resume is a set difference.**  Read the completed ids from the store and skip them.
  A killed 20,000-job sweep restarts with no bookkeeping and no risk of duplicating or
  losing work.
* **Results are independent of execution order and worker count**, because the seed is
  derived from the id rather than from a counter.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Any

from ..core.ids import content_hash
from ..core.rng import RandomContext

__all__ = ["Job", "JobResult"]


@dataclass(frozen=True)
class Job:
    """One (morphology, object, task, controller, trial) evaluation."""

    morphology: str
    """A preset name, or a morphology id resolved through the run's morphology store."""

    obj: str
    task: str
    controller: str = "ramp"
    trial: int = 0
    experiment: str = "default"
    controller_kwargs: dict[str, Any] = field(default_factory=dict)
    scene_options: dict[str, Any] = field(default_factory=dict)
    domain_randomization: dict[str, Any] = field(default_factory=dict)
    metric_options: dict[str, Any] = field(default_factory=dict)
    include_sim_metrics: bool = False
    """Simulation-based metrics (the disturbance battery) cost several times a rollout, so
    they are opt-in: screening sweeps leave them off and reports turn them on."""

    save_trace: bool = False
    tags: dict[str, Any] = field(default_factory=dict)
    """Free-form labels carried into the result row -- the factor levels of an ablation,
    for instance, so analysis does not have to re-derive them."""

    def content(self) -> dict[str, Any]:
        """Everything that determines the result. ``tags`` are excluded: they label a job,
        they do not change what it computes."""
        return {
            "experiment": self.experiment,
            "morphology": self.morphology,
            "object": self.obj,
            "task": self.task,
            "controller": self.controller,
            "controller_kwargs": self.controller_kwargs,
            "scene_options": self.scene_options,
            "domain_randomization": self.domain_randomization,
            "metric_options": self.metric_options,
            "include_sim_metrics": self.include_sim_metrics,
            "trial": self.trial,
        }

    @property
    def job_id(self) -> str:
        return content_hash(self.content())

    @property
    def group_key(self) -> str:
        """Jobs sharing this key share a compiled model.

        Compiling a scene costs about as much as simulating a trial, so the runner sorts by
        this key before dispatch and reuses the rollout -- worth roughly 3x on short trials
        (``docs/01-ARCHITECTURE.md`` §8).
        """
        return content_hash(
            {"m": self.morphology, "o": self.obj, "t": self.task, "s": self.scene_options}
        )

    def rng(self) -> RandomContext:
        """Seeded RNG context for this job, derived from its content."""
        return RandomContext([self.experiment, self.morphology, self.obj, self.task,
                              self.controller, self.trial])

    def with_trial(self, trial: int) -> "Job":
        return replace(self, trial=trial)

    def to_dict(self) -> dict[str, Any]:
        return {**self.content(), "job_id": self.job_id, "tags": dict(self.tags),
                "save_trace": self.save_trace}

    @staticmethod
    def from_dict(d: dict[str, Any]) -> "Job":
        return Job(
            morphology=d["morphology"], obj=d["object"], task=d["task"],
            controller=d.get("controller", "ramp"), trial=int(d.get("trial", 0)),
            experiment=d.get("experiment", "default"),
            controller_kwargs=dict(d.get("controller_kwargs", {})),
            scene_options=dict(d.get("scene_options", {})),
            domain_randomization=dict(d.get("domain_randomization", {})),
            metric_options=dict(d.get("metric_options", {})),
            include_sim_metrics=bool(d.get("include_sim_metrics", False)),
            save_trace=bool(d.get("save_trace", False)),
            tags=dict(d.get("tags", {})),
        )

    def __repr__(self) -> str:
        return (
            f"Job({self.morphology}/{self.obj}/{self.task}/{self.controller}"
            f"#{self.trial} -> {self.job_id[:8]})"
        )


@dataclass
class JobResult:
    """One flat result row, plus the identifiers needed to trace it back."""

    job_id: str
    row: dict[str, Any]
    error: str = ""

    @property
    def ok(self) -> bool:
        return not self.error

    def to_dict(self) -> dict[str, Any]:
        out = {"job_id": self.job_id, **self.row}
        if self.error:
            out["error"] = self.error
        return out
