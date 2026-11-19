"""Content-addressed identifiers.

Every experimental object -- a morphology, a job, a config -- gets an id that is a pure
function of its *content*.  Two consequences the platform depends on
(``docs/01-ARCHITECTURE.md`` §1, §8):

* **Resume is a set difference.**  A killed sweep restarts by skipping ids already present
  in the store, with no bookkeeping and no risk of duplicating or losing work.
* **Results are citable.**  A figure produced last month names the exact hand that produced
  it, and that hand can be rebuilt byte-identically.

Canonicalization rules (so that trivially different encodings hash identically):

* mappings are emitted with sorted keys;
* floats are rounded to ``FLOAT_DIGITS`` significant decimals, and ``-0.0`` becomes ``0.0``;
* numpy scalars/arrays degrade to Python scalars/lists;
* tuples and lists are indistinguishable (both become JSON arrays);
* sets are emitted as sorted arrays;
* ``None`` and absent keys are *not* equivalent -- an explicit null is content.
"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import asdict, is_dataclass
from enum import Enum
from typing import Any

FLOAT_DIGITS = 12
"""Significant decimals retained when hashing floats.

Large enough that meaningfully different parameters never collide, small enough that
round-tripping a value through YAML/JSON does not change its id.
"""

_HASH_LEN = 16


def canonical(obj: Any) -> Any:
    """Recursively convert ``obj`` into a canonical, JSON-serializable form."""
    if obj is None or isinstance(obj, (str, bool)):
        return obj
    if isinstance(obj, Enum):
        return canonical(obj.value)
    if isinstance(obj, int):
        return int(obj)
    if isinstance(obj, float):
        return _canon_float(obj)
    if is_dataclass(obj) and not isinstance(obj, type):
        return canonical(asdict(obj))
    if isinstance(obj, dict):
        return {str(k): canonical(v) for k, v in sorted(obj.items(), key=lambda kv: str(kv[0]))}
    if isinstance(obj, (set, frozenset)):
        return sorted((canonical(v) for v in obj), key=repr)
    if isinstance(obj, (list, tuple)):
        return [canonical(v) for v in obj]
    # numpy without importing numpy at module scope
    if hasattr(obj, "tolist") and hasattr(obj, "shape"):
        return canonical(obj.tolist())
    if hasattr(obj, "item") and hasattr(obj, "dtype"):
        return canonical(obj.item())
    if hasattr(obj, "to_dict"):
        return canonical(obj.to_dict())
    if hasattr(obj, "__dict__"):
        return canonical({k: v for k, v in vars(obj).items() if not k.startswith("_")})
    raise TypeError(f"cannot canonicalize object of type {type(obj).__name__}: {obj!r}")


def _canon_float(x: float) -> Any:
    if math.isnan(x):
        return "NaN"
    if math.isinf(x):
        return "Infinity" if x > 0 else "-Infinity"
    r = round(float(x), FLOAT_DIGITS)
    if r == 0.0:
        return 0.0  # collapse -0.0
    # Integral floats hash as floats, not ints: 1.0 and 1 are different parameter values
    # only in type, never in meaning, so normalize toward float.
    return r


def canonical_json(obj: Any) -> str:
    """Deterministic JSON encoding of ``obj``, suitable for hashing or on-disk storage."""
    return json.dumps(canonical(obj), sort_keys=True, separators=(",", ":"), ensure_ascii=False)


def content_hash(obj: Any, *, length: int = _HASH_LEN) -> str:
    """Stable hex digest of ``obj``'s content.

    Uses SHA-256 truncated to ``length`` hex characters (64 bits at the default 16), which
    keeps ids readable in filenames while making collisions negligible at any experiment
    scale this platform will ever reach.
    """
    digest = hashlib.sha256(canonical_json(obj).encode("utf-8")).hexdigest()
    return digest[:length]


def short_id(obj: Any) -> str:
    """An 8-character id, for display and filenames where collisions are not load-bearing."""
    return content_hash(obj, length=8)


def named_id(prefix: str, obj: Any, *, length: int = 8) -> str:
    """``prefix`` joined to a content hash, e.g. ``"hand-3f0a91c2"``."""
    return f"{prefix}-{content_hash(obj, length=length)}"
