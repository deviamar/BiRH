"""Sweep expansion: a config becomes a job list.

Three combination modes cover the designs that matter:

``grid``
    Cartesian product of every factor -- the full factorial of Study 2.
``zip``
    Factors advance together, for paired conditions (a hand and its ablated twin).
``list``
    Explicit tuples, for irregular designs.

Object/task pairing respects each task's ``object_filter``, so an AHAP suite automatically
pairs each grasp type with the objects that afford it instead of scoring a hand on
pinching a bottle.
"""

from __future__ import annotations

import itertools
from dataclasses import dataclass, field
from typing import Any, Iterable, Sequence

from ..core.config import Config
from ..core.logging import get_logger
from .job import Job

log = get_logger(__name__)

__all__ = ["SweepSpec", "expand", "jobs_from_config"]


@dataclass
class SweepSpec:
    """A declarative description of a set of trials."""

    experiment: str = "default"
    morphologies: Sequence[str] = ()
    objects: Sequence[str] = ()
    tasks: Sequence[str] = ("lift",)
    controllers: Sequence[str] = ("ramp",)
    trials: int = 1
    mode: str = "grid"
    include_sim_metrics: bool = False
    save_traces: bool = False
    scene_options: dict[str, Any] = field(default_factory=dict)
    domain_randomization: dict[str, Any] = field(default_factory=dict)
    metric_options: dict[str, Any] = field(default_factory=dict)
    controller_kwargs: dict[str, Any] = field(default_factory=dict)
    respect_object_filter: bool = True
    tags: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return dict(vars(self))


def _resolve_objects(names: Sequence[str]) -> list[str]:
    """Expand object-set names (``core15``) into object names, preserving order."""
    from ..objects.library import object_sets, objects

    out: list[str] = []
    for name in names:
        base = name.split("@")[0]
        if base in objects:
            out.append(name)
        elif name in object_sets:
            out += object_sets.create(name)
        else:
            raise KeyError(
                f"{name!r} is neither an object nor an object set; "
                f"objects: {objects.names()[:6]}..., sets: {object_sets.names()}"
            )
    seen: set[str] = set()
    return [o for o in out if not (o in seen or seen.add(o))]


def _resolve_morphologies(names: Sequence[str]) -> list[str]:
    """Expand preset tags (``tag:birh``) into preset names."""
    from ..morphology.presets import presets

    out: list[str] = []
    for name in names:
        if name.startswith("tag:"):
            out += presets.names(tag=name[4:])
        elif name == "all":
            out += presets.names()
        else:
            out.append(name)
    seen: set[str] = set()
    return [m for m in out if not (m in seen or seen.add(m))]


def _resolve_tasks(names: Sequence[str]) -> list[str]:
    from ..tasks.base import tasks
    from ..tasks.grasp import AHAP_SUITE, CORE_SUITE, QUICK_SUITE

    suites = {"ahap": AHAP_SUITE, "core": CORE_SUITE, "quick": QUICK_SUITE}
    out: list[str] = []
    for name in names:
        if name in suites:
            out += list(suites[name])
        elif name == "all":
            out += tasks.names()
        else:
            out.append(name)
    seen: set[str] = set()
    return [t for t in out if not (t in seen or seen.add(t))]


def expand(spec: SweepSpec) -> list[Job]:
    """Turn a :class:`SweepSpec` into the job list it describes."""
    from ..objects import get_object
    from ..tasks import get_task

    morphologies = _resolve_morphologies(spec.morphologies)
    objects_ = _resolve_objects(spec.objects)
    task_names = _resolve_tasks(spec.tasks)
    controllers = list(spec.controllers)

    if not morphologies:
        raise ValueError("sweep specifies no morphologies")
    if not objects_:
        raise ValueError("sweep specifies no objects")

    if spec.mode == "grid":
        combos = list(itertools.product(morphologies, objects_, task_names, controllers))
    elif spec.mode == "zip":
        n = max(len(morphologies), len(objects_), len(task_names), len(controllers))

        def cycle(seq: Sequence[str]) -> list[str]:
            return [seq[i % len(seq)] for i in range(n)]

        combos = list(zip(cycle(morphologies), cycle(objects_), cycle(task_names), cycle(controllers)))
    else:
        raise ValueError(f"unknown sweep mode {spec.mode!r}; use grid or zip")

    skipped = 0
    jobs: list[Job] = []
    for morphology, obj_name, task_name, controller in combos:
        if spec.respect_object_filter:
            task = get_task(task_name)
            if not task.applies_to(get_object(obj_name)):
                skipped += 1
                continue
        for trial in range(spec.trials):
            jobs.append(
                Job(
                    morphology=morphology, obj=obj_name, task=task_name, controller=controller,
                    trial=trial, experiment=spec.experiment,
                    controller_kwargs=dict(spec.controller_kwargs),
                    scene_options=dict(spec.scene_options),
                    domain_randomization=dict(spec.domain_randomization),
                    metric_options=dict(spec.metric_options),
                    include_sim_metrics=spec.include_sim_metrics,
                    save_trace=spec.save_traces,
                    tags=dict(spec.tags),
                )
            )

    if skipped:
        # Never silently truncate coverage: a sweep that quietly dropped a third of its
        # cells reads as "we covered everything" when it did not.
        log.info(
            "skipped %d (object, task) pairs whose grasp type the object does not afford "
            "(set respect_object_filter=false to include them)", skipped,
        )
    log.info(
        "expanded sweep %r: %d morphologies x %d objects x %d tasks x %d controllers "
        "x %d trials = %d jobs",
        spec.experiment, len(morphologies), len(objects_), len(task_names),
        len(controllers), spec.trials, len(jobs),
    )
    return jobs


def jobs_from_config(config: Config) -> tuple[SweepSpec, list[Job]]:
    """Build the sweep described by a config file."""
    sweep = config.section("sweep", {})
    spec = SweepSpec(
        experiment=config.get("experiment", "default"),
        morphologies=list(sweep.get("morphologies", sweep.get("hands", []))),
        objects=list(sweep.get("objects", ["core15"])),
        tasks=list(sweep.get("tasks", ["lift"])),
        controllers=list(sweep.get("controllers", ["ramp"])),
        trials=int(sweep.get("trials", 1)),
        mode=str(sweep.get("mode", "grid")),
        include_sim_metrics=bool(sweep.get("include_sim_metrics", False)),
        save_traces=bool(sweep.get("save_traces", False)),
        scene_options=dict(config.get("sim", {})),
        domain_randomization=dict(config.get("domain_randomization", {})),
        metric_options=dict(config.get("metrics", {})),
        controller_kwargs=dict(sweep.get("controller_kwargs", {})),
        respect_object_filter=bool(sweep.get("respect_object_filter", True)),
        tags=dict(sweep.get("tags", {})),
    )
    return spec, expand(spec)


def factorial(
    experiment: str, factors: dict[str, Sequence[Any]], **base: Any
) -> list[Job]:
    """A full factorial over arbitrary job fields, tagging each job with its cell.

    This is the machinery for Study 2 (``docs/00-REVIEW.md`` §7): each job carries its
    factor levels as tags, so the analysis can fit main effects and interactions without
    re-deriving the design.
    """
    keys = list(factors)
    jobs: list[Job] = []
    for values in itertools.product(*(factors[k] for k in keys)):
        cell = dict(zip(keys, values))
        spec = SweepSpec(experiment=experiment, tags=cell, **{**base, **cell})
        for job in expand(spec):
            jobs.append(job)
    return jobs
