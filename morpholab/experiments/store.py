"""Result storage: append-only JSONL, with resume by id set.

JSONL rather than a database because a sweep's writers are separate processes, appends of
a line under the OS page size are atomic enough in practice, the file is readable with
``head``, and a partially written run remains fully usable.  Analysis reads it into a
dataframe once.

Morphologies are stored beside the results as JSON keyed by ``morphology_id``, so a result
row from six months ago can be turned back into a hand.
"""

from __future__ import annotations

import json
import os
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Iterator

from ..core.logging import get_logger

log = get_logger(__name__)

__all__ = ["ResultStore"]


def _jsonable(obj: Any) -> Any:
    import numpy as np

    if isinstance(obj, dict):
        return {str(k): _jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_jsonable(v) for v in obj]
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, (np.floating, np.integer, np.bool_)):
        return obj.item()
    if isinstance(obj, (str, int, float, bool)) or obj is None:
        return obj
    return str(obj)


@dataclass
class ResultStore:
    """Append-only result file with a completed-id index."""

    path: Path
    _lock: threading.Lock = None  # type: ignore[assignment]

    def __post_init__(self) -> None:
        self.path = Path(self.path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()

    # -- writing ------------------------------------------------------------------------

    def append(self, row: dict[str, Any]) -> None:
        line = json.dumps(_jsonable(row), separators=(",", ":"), ensure_ascii=False)
        with self._lock, self.path.open("a", encoding="utf-8") as fh:
            fh.write(line + "\n")
            fh.flush()
            os.fsync(fh.fileno())

    def extend(self, rows: Iterable[dict[str, Any]]) -> int:
        n = 0
        with self._lock, self.path.open("a", encoding="utf-8") as fh:
            for row in rows:
                fh.write(json.dumps(_jsonable(row), separators=(",", ":"), ensure_ascii=False) + "\n")
                n += 1
            fh.flush()
            os.fsync(fh.fileno())
        return n

    # -- reading ------------------------------------------------------------------------

    def __iter__(self) -> Iterator[dict[str, Any]]:
        if not self.path.exists():
            return
        with self.path.open("r", encoding="utf-8") as fh:
            for lineno, line in enumerate(fh, 1):
                line = line.strip()
                if not line:
                    continue
                try:
                    yield json.loads(line)
                except json.JSONDecodeError:
                    # A process killed mid-write leaves a partial final line. Skipping it
                    # is exactly right: the job it belonged to is simply not completed, so
                    # resume will re-run it.
                    log.warning("skipping malformed result at %s:%d", self.path.name, lineno)

    def rows(self) -> list[dict[str, Any]]:
        return list(self)

    def completed_ids(self) -> set[str]:
        """Job ids already in the store -- the entire resume mechanism."""
        return {r["job_id"] for r in self if "job_id" in r}

    def count(self) -> int:
        return sum(1 for _ in self)

    # -- export -------------------------------------------------------------------------

    def to_dataframe(self):  # noqa: ANN201 - pandas.DataFrame
        import pandas as pd

        rows = self.rows()
        return pd.DataFrame(rows) if rows else pd.DataFrame()

    def to_csv(self, path: str | Path) -> Path:
        df = self.to_dataframe()
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        df.to_csv(p, index=False)
        return p

    # -- morphology sidecar --------------------------------------------------------------

    @property
    def morphology_dir(self) -> Path:
        d = self.path.parent / "morphologies"
        d.mkdir(parents=True, exist_ok=True)
        return d

    def save_morphology(self, spec: Any) -> Path:
        """Persist a hand so a result row can be turned back into a design."""
        p = self.morphology_dir / f"{spec.morphology_id}.json"
        if not p.exists():
            p.write_text(json.dumps(spec.to_dict(), indent=2, sort_keys=True), encoding="utf-8")
        return p

    def load_morphology(self, morphology_id: str):  # noqa: ANN201 - HandSpec
        from ..morphology import HandSpec

        p = self.morphology_dir / f"{morphology_id}.json"
        if not p.exists():
            raise FileNotFoundError(f"no stored morphology {morphology_id!r} in {self.morphology_dir}")
        return HandSpec.from_dict(json.loads(p.read_text(encoding="utf-8")))

    def morphology_ids(self) -> list[str]:
        return sorted(p.stem for p in self.morphology_dir.glob("*.json"))

    def __len__(self) -> int:
        return self.count()

    def __bool__(self) -> bool:
        # Without this, an *empty* store is falsy and ``if store:`` silently discards it --
        # which is exactly how a sweep can run to completion and write nothing.
        return True

    def __repr__(self) -> str:
        return f"ResultStore({self.path}, {self.count()} rows)"
