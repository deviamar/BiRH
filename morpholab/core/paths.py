"""Run-directory conventions and provenance capture.

Layout (``docs/01-ARCHITECTURE.md`` §8)::

    runs/<experiment>/<timestamp>-<config_hash>/
        config.resolved.yaml    provenance.json
        results.jsonl           morphologies/<morphology_id>.json
        traces/<job_id>.npz     media/<job_id>.mp4
        figures/                report.md   report.html
        checkpoints/            logs/

The timestamp makes runs sortable; the config hash makes repeated runs of the *same*
experiment recognizable at a glance.  No path in the platform is ever absolute in source
(``docs/00-REVIEW.md`` §3.2) -- roots come from the config or the environment.
"""

from __future__ import annotations

import os
import platform
import shutil
import socket
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

__all__ = ["RunDirs", "project_root", "default_runs_root", "collect_provenance"]

_ENV_RUNS_ROOT = "MORPHOLAB_RUNS"


def project_root() -> Path:
    """Repository root (the directory containing the ``morpholab`` package)."""
    return Path(__file__).resolve().parents[2]


def default_runs_root() -> Path:
    """Where runs are written unless a config says otherwise."""
    env = os.environ.get(_ENV_RUNS_ROOT)
    return Path(env).expanduser().resolve() if env else project_root() / "runs"


def asset_root() -> Path:
    """Directory holding meshes inherited from the original project."""
    return project_root() / "simulation" / "mujoco-models" / "assets"


def utc_stamp() -> str:
    """Filesystem-safe UTC timestamp, sortable lexicographically."""
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


@dataclass(frozen=True)
class RunDirs:
    """The directory tree for one experiment run."""

    root: Path

    @classmethod
    def create(
        cls,
        experiment: str,
        config_hash: str,
        *,
        runs_root: Path | str | None = None,
        reuse: bool = True,
    ) -> RunDirs:
        """Create (or reuse) a run directory for ``experiment``.

        With ``reuse=True`` (the default) an existing directory for the same experiment and
        config hash is reused, which is what makes ``--resume`` work without the caller
        having to remember a path.
        """
        base = Path(runs_root).expanduser().resolve() if runs_root else default_runs_root()
        exp_dir = base / _slug(experiment)
        if reuse and exp_dir.exists():
            matches = sorted(p for p in exp_dir.iterdir() if p.is_dir() and p.name.endswith(config_hash))
            if matches:
                return cls(matches[-1]).ensure()
        return cls(exp_dir / f"{utc_stamp()}-{config_hash}").ensure()

    @classmethod
    def at(cls, path: Path | str) -> RunDirs:
        """Wrap an existing run directory."""
        p = Path(path).expanduser().resolve()
        if not p.exists():
            raise FileNotFoundError(f"run directory does not exist: {p}")
        return cls(p)

    def ensure(self) -> RunDirs:
        """Create all subdirectories."""
        for d in (self.root, self.traces, self.media, self.figures, self.morphologies,
                  self.checkpoints, self.logs):
            d.mkdir(parents=True, exist_ok=True)
        return self

    # -- files and subdirectories ---------------------------------------------------

    @property
    def config(self) -> Path:
        return self.root / "config.resolved.yaml"

    @property
    def provenance(self) -> Path:
        return self.root / "provenance.json"

    @property
    def results(self) -> Path:
        return self.root / "results.jsonl"

    @property
    def traces(self) -> Path:
        return self.root / "traces"

    @property
    def media(self) -> Path:
        return self.root / "media"

    @property
    def figures(self) -> Path:
        return self.root / "figures"

    @property
    def morphologies(self) -> Path:
        return self.root / "morphologies"

    @property
    def checkpoints(self) -> Path:
        return self.root / "checkpoints"

    @property
    def logs(self) -> Path:
        return self.root / "logs"

    def trace(self, job_id: str) -> Path:
        return self.traces / f"{job_id}.npz"

    def video(self, job_id: str, ext: str = "mp4") -> Path:
        return self.media / f"{job_id}.{ext}"

    def figure(self, name: str, ext: str = "png") -> Path:
        return self.figures / f"{name}.{ext}"

    def morphology(self, morphology_id: str) -> Path:
        return self.morphologies / f"{morphology_id}.json"

    def __fspath__(self) -> str:
        return str(self.root)

    def __repr__(self) -> str:
        return f"RunDirs({self.root})"


def _slug(text: str) -> str:
    """Filesystem-safe slug."""
    keep = [c if (c.isalnum() or c in "-_.") else "-" for c in text.strip()]
    slug = "".join(keep).strip("-.") or "run"
    return slug[:80]


def _git(*args: str) -> str | None:
    """Run a git command in the project root, returning ``None`` if unavailable."""
    if shutil.which("git") is None:
        return None
    try:
        out = subprocess.run(
            ["git", *args],
            cwd=project_root(),
            capture_output=True,
            text=True,
            timeout=10,
            check=False,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    return out.stdout.strip() if out.returncode == 0 else None


def collect_provenance(extra: dict[str, Any] | None = None) -> dict[str, Any]:
    """Everything needed to reproduce or explain a run.

    Captures the git SHA *and* whether the tree was dirty, because a result produced from
    uncommitted code is not reproducible and the record should say so rather than imply a
    clean commit.
    """
    sha = _git("rev-parse", "HEAD")
    status = _git("status", "--porcelain")
    versions: dict[str, str] = {}
    for mod in ("mujoco", "numpy", "scipy", "pandas", "matplotlib", "sklearn", "cma", "yaml"):
        try:
            versions[mod] = __import__(mod).__version__
        except Exception:  # noqa: BLE001 - absent or version-less packages are fine
            pass

    prov: dict[str, Any] = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "git": {
            "sha": sha,
            "branch": _git("rev-parse", "--abbrev-ref", "HEAD"),
            "dirty": bool(status),
            "dirty_files": (status or "").splitlines()[:50],
        },
        "python": sys.version,
        "platform": {
            "system": platform.system(),
            "release": platform.release(),
            "machine": platform.machine(),
            "hostname": socket.gethostname(),
            "cpu_count": os.cpu_count(),
        },
        "packages": versions,
        "env": {k: v for k, v in os.environ.items() if k.startswith(("MORPHOLAB_", "MUJOCO_"))},
    }
    if extra:
        prov.update(extra)
    return prov
