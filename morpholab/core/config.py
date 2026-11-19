"""Configuration loading, merging, and override handling.

An experiment is described by a YAML/JSON file.  Configs support:

* ``extends: base.yaml`` -- inheritance by deep merge, so study configs stay short;
* ``${a.b.c}`` -- interpolation from elsewhere in the same config;
* dotted CLI overrides -- ``--set sim.timestep=0.001 objects=core15``;
* strict access -- reading a missing key raises rather than returning ``None``, because a
  typo in a 10,000-job sweep must fail before the first physics step
  (``docs/01-ARCHITECTURE.md`` §1.7).

The resolved config is content-hashed, so it names its own output directory and two runs
with identical settings are recognizably the same experiment.
"""

from __future__ import annotations

import json
import os
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Iterator, Mapping

import yaml

from .ids import canonical_json, content_hash

__all__ = ["Config", "ConfigError", "load_config", "deep_merge", "parse_overrides"]

_INTERP = re.compile(r"\$\{([A-Za-z0-9_.\-]+)\}")
_MAX_INTERP_PASSES = 10


class _Missing:
    """Sentinel distinguishing 'no default supplied' from an explicit ``None`` default."""

    _instance: _Missing | None = None

    def __new__(cls) -> _Missing:
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __repr__(self) -> str:  # pragma: no cover - debugging aid
        return "<missing>"

    def __bool__(self) -> bool:
        return False


MISSING = _Missing()


class ConfigError(Exception):
    """Raised for malformed configs, missing keys, or unresolvable interpolation."""


def deep_merge(base: Mapping[str, Any], override: Mapping[str, Any]) -> dict[str, Any]:
    """Recursively merge ``override`` into ``base``, returning a new dict.

    Mappings merge key-wise; every other type (including lists) is replaced wholesale.
    Replacing lists rather than concatenating them is deliberate: a study that overrides
    ``objects: [sphere]`` means *only* the sphere, and silent concatenation would produce
    an experiment nobody asked for.
    """
    out = dict(base)
    for key, value in override.items():
        if key in out and isinstance(out[key], Mapping) and isinstance(value, Mapping):
            out[key] = deep_merge(out[key], value)
        else:
            out[key] = value
    return out


def _coerce_scalar(text: str) -> Any:
    """Parse a CLI override value using YAML scalar rules (``3``, ``1e-3``, ``true``, ...)."""
    try:
        return yaml.safe_load(text)
    except yaml.YAMLError:
        return text


def parse_overrides(pairs: Iterable[str]) -> dict[str, Any]:
    """Turn ``["a.b=1", "c=[x,y]"]`` into a nested dict ``{"a": {"b": 1}, "c": ["x","y"]}``."""
    out: dict[str, Any] = {}
    for pair in pairs:
        if "=" not in pair:
            raise ConfigError(f"override must be KEY=VALUE, got {pair!r}")
        key, _, raw = pair.partition("=")
        key = key.strip()
        if not key:
            raise ConfigError(f"override has an empty key: {pair!r}")
        node = out
        parts = key.split(".")
        for part in parts[:-1]:
            node = node.setdefault(part, {})
            if not isinstance(node, dict):
                raise ConfigError(f"override {key!r} conflicts with an earlier scalar override")
        node[parts[-1]] = _coerce_scalar(raw)
    return out


@dataclass(frozen=True)
class Config:
    """An immutable, dotted-access view over a resolved configuration mapping."""

    data: dict[str, Any]
    source: Path | None = None

    # -- access ---------------------------------------------------------------------

    def __getitem__(self, key: str) -> Any:
        return self.get(key)

    def get(self, key: str, default: Any = MISSING) -> Any:
        """Fetch a dotted key. Raises :class:`ConfigError` if absent and no default given."""
        node: Any = self.data
        for i, part in enumerate(key.split(".")):
            if isinstance(node, Mapping) and part in node:
                node = node[part]
            elif isinstance(node, (list, tuple)) and part.lstrip("-").isdigit():
                try:
                    node = node[int(part)]
                except IndexError:
                    return self._missing(key, default, ".".join(key.split(".")[: i + 1]))
            else:
                return self._missing(key, default, ".".join(key.split(".")[: i + 1]))
        return node

    def _missing(self, key: str, default: Any, at: str) -> Any:
        if default is not MISSING:
            return default
        near = self._suggest(at)
        hint = f"; available near {at!r}: {near}" if near else ""
        raise ConfigError(f"missing config key {key!r} (failed at {at!r}){hint}")

    def _suggest(self, at: str) -> str:
        parts = at.split(".")[:-1]
        node: Any = self.data
        for part in parts:
            if isinstance(node, Mapping) and part in node:
                node = node[part]
            else:
                return ""
        if isinstance(node, Mapping):
            return ", ".join(sorted(map(str, node))[:10])
        return ""

    def section(self, key: str, default: Mapping[str, Any] | None = None) -> Config:
        """A sub-config rooted at ``key``."""
        value = self.get(key, default if default is not None else {})
        if not isinstance(value, Mapping):
            raise ConfigError(f"config key {key!r} is a {type(value).__name__}, not a section")
        return Config(dict(value), self.source)

    def with_overrides(self, overrides: Mapping[str, Any]) -> Config:
        """A new config with ``overrides`` deep-merged in and interpolation re-resolved."""
        return Config(_resolve_interpolation(deep_merge(self.data, overrides)), self.source)

    # -- conversion -----------------------------------------------------------------

    def to_dict(self) -> dict[str, Any]:
        """A deep copy of the underlying mapping."""
        return json.loads(json.dumps(self.data, default=str))

    def to_yaml(self) -> str:
        """YAML serialization, as written to ``config.resolved.yaml``."""
        return yaml.safe_dump(self.to_dict(), sort_keys=True, default_flow_style=False)

    def save(self, path: str | os.PathLike[str]) -> Path:
        """Write the resolved config to ``path``."""
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(self.to_yaml(), encoding="utf-8")
        return p

    @property
    def hash(self) -> str:
        """Content hash of the resolved config; names the run directory."""
        return content_hash(self.data)

    def __contains__(self, key: str) -> bool:
        # A distinct sentinel: passing MISSING itself would read as "no default supplied".
        absent = object()
        return self.get(key, absent) is not absent

    def __iter__(self) -> Iterator[str]:
        return iter(self.data)

    def __repr__(self) -> str:
        src = f" from {self.source}" if self.source else ""
        return f"Config({sorted(self.data)}{src}, hash={self.hash[:8]})"


def _resolve_interpolation(data: Any, root: Any = None, _pass: int = 0) -> Any:
    """Expand ``${dotted.key}`` references against the config root."""
    if root is None:
        root = data
    if _pass > _MAX_INTERP_PASSES:
        raise ConfigError("interpolation did not converge; check for a reference cycle")

    def lookup(dotted: str) -> Any:
        node: Any = root
        for part in dotted.split("."):
            if isinstance(node, Mapping) and part in node:
                node = node[part]
            elif isinstance(node, (list, tuple)) and part.isdigit():
                node = node[int(part)]
            else:
                raise ConfigError(f"cannot interpolate ${{{dotted}}}: no such key")
        return node

    def walk(node: Any) -> tuple[Any, bool]:
        changed = False
        if isinstance(node, Mapping):
            out = {}
            for k, v in node.items():
                out[k], c = walk(v)
                changed |= c
            return out, changed
        if isinstance(node, list):
            outs, cs = zip(*(walk(v) for v in node)) if node else ((), ())
            return list(outs), any(cs)
        if isinstance(node, str):
            m = _INTERP.fullmatch(node.strip())
            if m:  # whole-string reference keeps the referent's type
                return lookup(m.group(1)), True
            if _INTERP.search(node):  # embedded reference becomes a string
                return _INTERP.sub(lambda mm: str(lookup(mm.group(1))), node), True
        return node, changed

    resolved, changed = walk(data)
    if changed:
        return _resolve_interpolation(resolved, resolved, _pass + 1)
    return resolved


def _read(path: Path) -> dict[str, Any]:
    if not path.exists():
        raise ConfigError(f"config file not found: {path}")
    text = path.read_text(encoding="utf-8")
    try:
        data = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise ConfigError(f"{path}: invalid YAML: {exc}") from exc
    if data is None:
        data = {}
    if not isinstance(data, dict):
        raise ConfigError(f"{path}: top level must be a mapping, got {type(data).__name__}")
    return data


def load_config(
    path: str | os.PathLike[str] | None = None,
    *,
    overrides: Mapping[str, Any] | Iterable[str] | None = None,
    defaults: Mapping[str, Any] | None = None,
    _seen: tuple[Path, ...] = (),
) -> Config:
    """Load, resolve and validate a config.

    Parameters
    ----------
    path:
        YAML/JSON file.  May contain ``extends: other.yaml`` (relative to itself), which is
        loaded first and deep-merged under this file's contents.  Chains are followed and
        cycles are detected.
    overrides:
        Either a nested mapping or an iterable of ``"a.b=value"`` strings from the CLI.
    defaults:
        Platform defaults merged underneath everything.
    """
    data: dict[str, Any] = dict(defaults or {})
    source: Path | None = None

    if path is not None:
        source = Path(path).expanduser().resolve()
        if source in _seen:
            chain = " -> ".join(str(p) for p in (*_seen, source))
            raise ConfigError(f"circular 'extends' chain: {chain}")
        raw = _read(source)
        parent = raw.pop("extends", None)
        if parent is not None:
            parents = [parent] if isinstance(parent, str) else list(parent)
            for pth in parents:
                base = load_config(
                    (source.parent / pth).resolve(), defaults=None, _seen=(*_seen, source)
                )
                data = deep_merge(data, base.data)
        data = deep_merge(data, raw)

    if overrides:
        ov = parse_overrides(overrides) if not isinstance(overrides, Mapping) else dict(overrides)
        data = deep_merge(data, ov)

    return Config(_resolve_interpolation(data), source)


def config_from_dict(data: Mapping[str, Any]) -> Config:
    """Build a resolved config from an in-memory mapping (tests, notebooks)."""
    return Config(_resolve_interpolation(dict(data)))


def diff_configs(a: Config, b: Config) -> dict[str, tuple[Any, Any]]:
    """Flat dotted diff between two configs, for provenance and report headers."""

    def flatten(node: Any, prefix: str = "") -> dict[str, Any]:
        if isinstance(node, Mapping):
            out: dict[str, Any] = {}
            for k, v in node.items():
                out.update(flatten(v, f"{prefix}{k}."))
            return out
        return {prefix.rstrip("."): node}

    fa, fb = flatten(a.data), flatten(b.data)
    keys = sorted(set(fa) | set(fb))
    return {
        k: (fa.get(k), fb.get(k))
        for k in keys
        if canonical_json(fa.get(k)) != canonical_json(fb.get(k))
    }
