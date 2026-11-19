"""Worker-safe logging.

Requirements that shape this module:

* sweeps run in ``multiprocessing`` workers, so every record carries its worker id;
* a run directory keeps a full log file even when the console is quiet;
* MuJoCo emits warnings through a C callback, which must be routed into Python logging
  rather than to stderr where a parallel run would interleave it into noise.
"""

from __future__ import annotations

import logging
import os
import sys
from pathlib import Path
from typing import Any

_CONFIGURED = False
_ROOT_NAME = "morpholab"

_LEVEL_COLORS = {
    "DEBUG": "\033[2;37m",
    "INFO": "\033[0;36m",
    "WARNING": "\033[0;33m",
    "ERROR": "\033[0;31m",
    "CRITICAL": "\033[1;41m",
}
_RESET = "\033[0m"


class _WorkerFilter(logging.Filter):
    """Attach the process identity to every record."""

    def filter(self, record: logging.LogRecord) -> bool:
        record.worker = os.environ.get("MORPHOLAB_WORKER", "main")
        return True


class _ConsoleFormatter(logging.Formatter):
    """Compact, optionally colored console format."""

    def __init__(self, color: bool) -> None:
        super().__init__("%(asctime)s %(levelname)-7s %(name)s: %(message)s", "%H:%M:%S")
        self.color = color

    def format(self, record: logging.LogRecord) -> str:
        worker = getattr(record, "worker", "main")
        prefix = "" if worker == "main" else f"[w{worker}] "
        record.msg = f"{prefix}{record.msg}"
        text = super().format(record)
        if self.color:
            c = _LEVEL_COLORS.get(record.levelname, "")
            if c:
                text = f"{c}{text}{_RESET}"
        return text


def setup_logging(
    level: int | str = "INFO",
    *,
    log_file: Path | str | None = None,
    color: bool | None = None,
    quiet: bool = False,
) -> logging.Logger:
    """Configure the ``morpholab`` logger hierarchy. Idempotent."""
    global _CONFIGURED
    logger = logging.getLogger(_ROOT_NAME)
    if _CONFIGURED:
        logger.setLevel(_as_level(level))
        return logger

    logger.setLevel(logging.DEBUG)
    logger.propagate = False
    logger.addFilter(_WorkerFilter())

    if not quiet:
        console = logging.StreamHandler(sys.stderr)
        console.setLevel(_as_level(level))
        use_color = sys.stderr.isatty() if color is None else color
        console.setFormatter(_ConsoleFormatter(use_color))
        logger.addHandler(console)

    if log_file is not None:
        path = Path(log_file)
        path.parent.mkdir(parents=True, exist_ok=True)
        fh = logging.FileHandler(path, encoding="utf-8")
        fh.setLevel(logging.DEBUG)
        fh.setFormatter(
            logging.Formatter(
                "%(asctime)s %(levelname)-7s %(worker)s %(name)s: %(message)s",
                "%Y-%m-%d %H:%M:%S",
            )
        )
        logger.addHandler(fh)

    _CONFIGURED = True
    return logger


def _as_level(level: int | str) -> int:
    if isinstance(level, int):
        return level
    resolved = logging.getLevelName(str(level).upper())
    return resolved if isinstance(resolved, int) else logging.INFO


def get_logger(name: str) -> logging.Logger:
    """Module logger, e.g. ``get_logger(__name__)``."""
    short = name.split(".")[-1] if name.startswith(_ROOT_NAME) else name
    return logging.getLogger(f"{_ROOT_NAME}.{short}")


def capture_mujoco_warnings(logger: logging.Logger | None = None) -> list[str]:
    """Route MuJoCo's C-level warnings into Python logging and return the capture list.

    MuJoCo signals numerical trouble (contact explosion, nan, constraint overflow) through
    a warning callback that otherwise prints to stderr.  In a parallel sweep those prints
    are unattributable noise; captured, they become a per-trial ``sim_divergence`` signal
    (``docs/02-MATH.md`` §11).
    """
    import mujoco  # imported lazily: core must not require a simulator

    log = logger or get_logger("mujoco")
    captured: list[str] = []

    def _callback(message: str) -> None:
        captured.append(message)
        log.warning("mujoco: %s", message)

    mujoco.set_mju_user_warning(_callback)
    return captured


def log_dict(logger: logging.Logger, title: str, data: dict[str, Any], level: int = logging.INFO) -> None:
    """Log a small mapping as aligned key/value lines."""
    if not data:
        logger.log(level, "%s: (empty)", title)
        return
    width = max(len(str(k)) for k in data)
    logger.log(level, "%s:", title)
    for key, value in data.items():
        logger.log(level, "  %-*s  %s", width, key, value)
