"""Experiment management: jobs, sweeps, parallel execution, result storage, resume."""

from __future__ import annotations

from .job import Job, JobResult
from .runner import RunnerOptions, execute_group, execute_job, run_jobs
from .store import ResultStore
from .sweep import SweepSpec, expand, factorial, jobs_from_config

__all__ = [
    "Job", "JobResult", "ResultStore",
    "RunnerOptions", "run_jobs", "execute_job", "execute_group",
    "SweepSpec", "expand", "jobs_from_config", "factorial",
]
