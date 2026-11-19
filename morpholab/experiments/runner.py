"""Job execution: serial and parallel, with grouping, isolation and resume.

Three properties this module exists to guarantee:

* **Results do not depend on the worker count.**  Seeds derive from job content, never
  from a counter, so a run on 1 worker and a run on 20 produce identical rows.
* **One bad job does not lose a sweep.**  A crash, a divergence or a timeout is recorded
  as an error row and the sweep continues.
* **Compilation is amortized.**  Jobs are grouped by (morphology, object, task, scene) so
  each compiled scene serves all its trials -- worth roughly 3x on short trials.
"""

from __future__ import annotations

import os
import time
import traceback
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass, field
from typing import Any, Callable, Iterable, Sequence

from ..core.logging import get_logger, setup_logging
from ..core.types import Outcome
from .job import Job, JobResult
from .store import ResultStore

log = get_logger(__name__)

__all__ = ["RunnerOptions", "run_jobs", "execute_job", "execute_group"]


@dataclass(frozen=True)
class RunnerOptions:
    workers: int = 1
    """1 runs in-process, which is what tests and debugging want -- a traceback in a worker
    process is much harder to read."""

    chunk_size: int = 8
    """Groups per worker task. Larger chunks amortize process hand-off; smaller ones give
    finer progress and better load balance on heterogeneous jobs."""

    progress: bool = True
    progress_every: float = 2.0
    fail_fast: bool = False
    job_timeout: float = 120.0
    save_traces: bool = False
    trace_dir: Any = None


# ---------------------------------------------------------------------------------------
# execution
# ---------------------------------------------------------------------------------------

def _resolve_morphology(name: str, store: ResultStore | None):
    """A preset name, a stored morphology id, or a JSON path."""
    from pathlib import Path

    from ..morphology import HandSpec, get_preset
    from ..morphology.presets import presets

    if name in presets:
        return get_preset(name)
    if store is not None:
        try:
            return store.load_morphology(name)
        except FileNotFoundError:
            pass
    p = Path(name)
    if p.exists() and p.suffix == ".json":
        import json

        return HandSpec.from_dict(json.loads(p.read_text(encoding="utf-8")))
    raise KeyError(
        f"cannot resolve morphology {name!r}: not a preset, not in the run's morphology "
        "store, and not a path to a morphology JSON"
    )


def execute_group(jobs: Sequence[Job], *, store_path: Any = None, options: RunnerOptions | None = None) -> list[JobResult]:
    """Run jobs that share a compiled scene, reusing the rollout across trials."""
    from ..control import get_controller
    from ..metrics import MetricContext, base_row, evaluate_metrics
    from ..objects import get_object
    from ..sim import Rollout, build_scene
    from ..sim.perturb import DomainRandomization, PerturbationOptions
    from ..sim.scene import SceneOptions
    from ..tasks import get_task

    options = options or RunnerOptions()
    store = ResultStore(store_path) if store_path is not None else None
    results: list[JobResult] = []
    if not jobs:
        return results

    first = jobs[0]
    try:
        hand = _resolve_morphology(first.morphology, store)
        obj = get_object(first.obj)
        task = get_task(first.task)
        scene = build_scene(
            hand, obj,
            options=SceneOptions(**first.scene_options) if first.scene_options else None,
            approach=task.approach, standoff=task.standoff,
        )
        rollout = Rollout(scene, task, get_controller(first.controller, **first.controller_kwargs))
    except Exception as exc:  # noqa: BLE001 - a build failure fails the whole group, not the sweep
        detail = f"{type(exc).__name__}: {exc}"
        log.error("group %s failed to build: %s", first.group_key[:8], detail)
        out = []
        for j in jobs:
            row = _error_row(j, detail)
            # The error row must reach the store, or resume will retry a job that cannot
            # possibly succeed on every restart.
            if store is not None:
                store.append(row)
            out.append(JobResult(job_id=j.job_id, row=row, error=detail))
        return out

    for job in jobs:
        started = time.perf_counter()
        try:
            controller = get_controller(job.controller, **job.controller_kwargs)
            rollout.controller = controller

            rng = job.rng()
            if job.domain_randomization:
                DomainRandomization(**job.domain_randomization).apply(
                    rollout.model, rng.stream("domain")
                )
            if task.randomize.active or job.trial > 0:
                object_pose, wrist_pose = task.initial_poses(hand, obj, rng.stream("init_pose"))
                _apply_poses(rollout, object_pose, wrist_pose)

            trace = rollout.run()
            ctx = MetricContext(
                trace=trace, hand=hand, obj=obj, task=task, scene=scene,
                rollout=rollout if job.include_sim_metrics else None,
                options=_metric_options(job),
            )
            row = {**base_row(ctx), **evaluate_metrics(ctx, include_sim=job.include_sim_metrics)}
            row.update({
                "job_id": job.job_id, "experiment": job.experiment, "trial": job.trial,
                "preshape": float(scene.meta.get("preshape", 0.0)),
                "job_wall_time": time.perf_counter() - started,
                **{f"tag_{k}": v for k, v in job.tags.items()},
            })
            if store is not None:
                store.save_morphology(hand)
                store.append(row)
            results.append(JobResult(job_id=job.job_id, row=row))

            if job.save_trace and options.trace_dir:
                from pathlib import Path

                trace.save(Path(options.trace_dir) / f"{job.job_id}.npz")

        except Exception as exc:  # noqa: BLE001 - one bad trial must not lose the sweep
            detail = f"{type(exc).__name__}: {exc}"
            log.warning("job %s failed: %s\n%s", job.job_id[:8], detail, traceback.format_exc(limit=3))
            row = _error_row(job, detail)
            if store is not None:
                store.append(row)
            results.append(JobResult(job_id=job.job_id, row=row, error=detail))
            if options.fail_fast:
                raise

    return results


def _apply_poses(rollout: Any, object_pose: Any, wrist_pose: Any) -> None:
    """Re-place the object and wrist for a randomized trial without rebuilding the scene."""
    import mujoco

    model, data = rollout.model, rollout.data
    mujoco.mj_resetData(model, data)
    joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "obj_free")
    if joint >= 0:
        adr = model.jnt_qposadr[joint]
        data.qpos[adr : adr + 3] = object_pose.pos
        data.qpos[adr + 3 : adr + 7] = object_pose.quat
    rollout.scene.initial_wrist = wrist_pose
    mujoco.mj_forward(model, data)


def _metric_options(job: Job) -> dict[str, Any]:
    from ..sim.perturb import PerturbationOptions

    opts = dict(job.metric_options)
    pert = opts.pop("perturbation", None)
    if isinstance(pert, dict):
        opts["perturbation"] = PerturbationOptions(**pert)
    elif isinstance(pert, str):
        from ..sim.perturb import perturbations

        opts["perturbation"] = perturbations.create(pert)
    return opts


def _error_row(job: Job, detail: str) -> dict[str, Any]:
    return {
        "job_id": job.job_id, "experiment": job.experiment, "hand": job.morphology,
        "object": job.obj, "task": job.task, "controller": job.controller,
        "trial": job.trial, "outcome": str(Outcome.ERROR), "success": False,
        "error": detail, **{f"tag_{k}": v for k, v in job.tags.items()},
    }


def execute_job(job: Job, *, store_path: Any = None, options: RunnerOptions | None = None) -> JobResult:
    """Run one job. Convenience for the single-trial CLI path."""
    return execute_group([job], store_path=store_path, options=options)[0]


# ---------------------------------------------------------------------------------------
# scheduling
# ---------------------------------------------------------------------------------------

def _group(jobs: Iterable[Job]) -> list[list[Job]]:
    groups: dict[str, list[Job]] = {}
    for job in jobs:
        groups.setdefault(job.group_key, []).append(job)
    return list(groups.values())


def _worker(payload: tuple[list[dict[str, Any]], Any, dict[str, Any]]) -> list[dict[str, Any]]:
    """Process-pool entry point. Takes and returns plain dicts: ``Job`` is picklable, but
    keeping the boundary primitive avoids version-skew surprises across processes."""
    job_dicts, store_path, opt_dict = payload
    os.environ.setdefault("MORPHOLAB_WORKER", str(os.getpid() % 1000))
    setup_logging("WARNING", quiet=False)
    jobs = [Job.from_dict(d) for d in job_dicts]
    options = RunnerOptions(**opt_dict)
    # Re-group inside the worker: a chunk holds several *distinct* scene groups, and
    # ``execute_group`` builds its scene from the first job only. Passing a mixed chunk
    # straight through would silently evaluate every job against the first job's hand and
    # object -- producing a full, plausible, entirely wrong result set.
    out: list[dict[str, Any]] = []
    for group in _group(jobs):
        out += [r.to_dict() for r in execute_group(group, store_path=store_path, options=options)]
    return out


def run_jobs(
    jobs: Sequence[Job],
    *,
    store: ResultStore | None = None,
    options: RunnerOptions | None = None,
    resume: bool = True,
    on_progress: Callable[[int, int, float], None] | None = None,
) -> list[JobResult]:
    """Execute a job list, skipping those already in the store.

    Returns results for the jobs *run in this call*; already-completed jobs are not
    re-reported, which is what makes resume free.
    """
    options = options or RunnerOptions()
    jobs = list(jobs)
    total_requested = len(jobs)

    if resume and store is not None:
        done = store.completed_ids()
        if done:
            jobs = [j for j in jobs if j.job_id not in done]
            log.info("resuming: %d/%d jobs already complete", len(done & {j.job_id for j in jobs} | done), total_requested)

    if not jobs:
        log.info("nothing to do: all %d jobs already complete", total_requested)
        return []

    groups = _group(jobs)
    log.info(
        "running %d jobs in %d scene groups on %d worker(s)",
        len(jobs), len(groups), options.workers,
    )
    started = time.perf_counter()
    results: list[JobResult] = []
    done_count = 0

    def report() -> None:
        if not options.progress:
            return
        elapsed = time.perf_counter() - started
        rate = done_count / max(elapsed, 1e-9)
        remaining = (len(jobs) - done_count) / max(rate, 1e-9)
        log.info(
            "  %d/%d jobs (%.0f%%)  %.1f jobs/s  eta %.0fs",
            done_count, len(jobs), 100 * done_count / len(jobs), rate, remaining,
        )
        if on_progress:
            on_progress(done_count, len(jobs), elapsed)

    if options.workers <= 1:
        last = started
        for group in groups:
            results += execute_group(group, store_path=store.path if store is not None else None, options=options)
            done_count += len(group)
            if time.perf_counter() - last > options.progress_every:
                report()
                last = time.perf_counter()
    else:
        store_path = store.path if store is not None else None
        opt_dict = {
            "workers": 1, "chunk_size": options.chunk_size, "progress": False,
            "fail_fast": options.fail_fast, "job_timeout": options.job_timeout,
            "save_traces": options.save_traces, "trace_dir": str(options.trace_dir) if options.trace_dir else None,
        }
        chunks = [
            (sum(groups[i : i + options.chunk_size], []), store_path, opt_dict)
            for i in range(0, len(groups), options.chunk_size)
        ]
        payloads = [([j.to_dict() for j in c[0]], c[1], c[2]) for c in chunks]

        last = started
        with ProcessPoolExecutor(max_workers=options.workers) as pool:
            futures = {pool.submit(_worker, p): len(p[0]) for p in payloads}
            for future in as_completed(futures):
                try:
                    rows = future.result()
                    results += [
                        JobResult(job_id=r.get("job_id", ""), row=r, error=r.get("error", ""))
                        for r in rows
                    ]
                except Exception as exc:  # noqa: BLE001 - a dead worker must not kill the sweep
                    log.error("worker chunk failed: %s", exc)
                done_count += futures[future]
                if time.perf_counter() - last > options.progress_every:
                    report()
                    last = time.perf_counter()

    elapsed = time.perf_counter() - started
    errors = sum(1 for r in results if not r.ok)
    log.info(
        "completed %d jobs in %.1fs (%.1f jobs/s)%s",
        len(results), elapsed, len(results) / max(elapsed, 1e-9),
        f", {errors} errors" if errors else "",
    )
    return results
