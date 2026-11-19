"""Name -> factory registries.

Every replaceable component in the platform (objects, tasks, controllers, metrics,
perturbations, optimizers, figures) is looked up by string name.  This is what lets a YAML
config name any component and lets a user add one without editing the framework
(``docs/01-ARCHITECTURE.md`` §3, §13).

Usage::

    controllers = Registry[Controller]("controller")

    @controllers.register("pd", doc="Independent-joint PD control")
    def _make_pd(**kw):
        return PDController(**kw)

    ctrl = controllers.create("pd", kp=20.0)
"""

from __future__ import annotations

import difflib
from dataclasses import dataclass, field
from typing import Any, Callable, Generic, Iterator, TypeVar

T = TypeVar("T")


class RegistryError(KeyError):
    """Raised for unknown or duplicate registry names."""


@dataclass(frozen=True)
class Entry(Generic[T]):
    """A single registry record."""

    name: str
    factory: Callable[..., T]
    doc: str = ""
    tags: tuple[str, ...] = ()
    meta: dict[str, Any] = field(default_factory=dict)

    @property
    def summary(self) -> str:
        """First line of the documentation, for listings."""
        if self.doc:
            return self.doc.strip().splitlines()[0]
        raw = self.factory.__doc__ or ""
        return raw.strip().splitlines()[0] if raw.strip() else ""


class Registry(Generic[T]):
    """A typed, introspectable name -> factory table.

    Parameters
    ----------
    kind:
        Human-readable name of what is being registered; used in error messages.
    allow_override:
        If ``False`` (default) re-registering a name raises.  Tests and notebooks that
        reload modules can set this to ``True``.
    """

    def __init__(self, kind: str, *, allow_override: bool = False) -> None:
        self.kind = kind
        self.allow_override = allow_override
        self._entries: dict[str, Entry[T]] = {}

    # -- registration ---------------------------------------------------------------

    def register(
        self,
        name: str,
        *,
        doc: str = "",
        tags: tuple[str, ...] | list[str] = (),
        **meta: Any,
    ) -> Callable[[Callable[..., T]], Callable[..., T]]:
        """Decorator registering a factory under ``name``."""

        def decorator(factory: Callable[..., T]) -> Callable[..., T]:
            self.add(name, factory, doc=doc, tags=tags, **meta)
            return factory

        return decorator

    def add(
        self,
        name: str,
        factory: Callable[..., T],
        *,
        doc: str = "",
        tags: tuple[str, ...] | list[str] = (),
        **meta: Any,
    ) -> None:
        """Register ``factory`` under ``name`` (non-decorator form)."""
        if not name or not isinstance(name, str):
            raise RegistryError(f"{self.kind} name must be a non-empty string, got {name!r}")
        if name in self._entries and not self.allow_override:
            raise RegistryError(
                f"{self.kind} {name!r} is already registered "
                f"(by {self._entries[name].factory!r}); pass allow_override=True to replace"
            )
        self._entries[name] = Entry(name=name, factory=factory, doc=doc, tags=tuple(tags), meta=meta)

    # -- lookup ---------------------------------------------------------------------

    def get(self, name: str) -> Entry[T]:
        """Return the registry entry for ``name``, with a did-you-mean on failure."""
        try:
            return self._entries[name]
        except KeyError:
            raise RegistryError(self._unknown_message(name)) from None

    def create(self, name: str, /, **kwargs: Any) -> T:
        """Instantiate the component registered under ``name``."""
        return self.get(name).factory(**kwargs)

    def factory(self, name: str) -> Callable[..., T]:
        """Return the raw factory registered under ``name``."""
        return self.get(name).factory

    def _unknown_message(self, name: str) -> str:
        close = difflib.get_close_matches(name, self._entries, n=3, cutoff=0.5)
        msg = f"unknown {self.kind}: {name!r}"
        if close:
            msg += f"; did you mean {', '.join(repr(c) for c in close)}?"
        elif self._entries:
            listing = ", ".join(sorted(self._entries)[:12])
            more = "" if len(self._entries) <= 12 else ", ..."
            msg += f"; available: {listing}{more}"
        else:
            msg += f"; no {self.kind}s are registered"
        return msg

    # -- introspection --------------------------------------------------------------

    def names(self, *, tag: str | None = None) -> list[str]:
        """Sorted registered names, optionally filtered by tag."""
        if tag is None:
            return sorted(self._entries)
        return sorted(n for n, e in self._entries.items() if tag in e.tags)

    def entries(self, *, tag: str | None = None) -> list[Entry[T]]:
        """Registry entries in name order, optionally filtered by tag."""
        return [self._entries[n] for n in self.names(tag=tag)]

    def tags(self) -> list[str]:
        """All tags used across the registry."""
        return sorted({t for e in self._entries.values() for t in e.tags})

    def describe(self) -> str:
        """Multi-line human-readable listing, used by ``morpholab list``."""
        if not self._entries:
            return f"(no {self.kind}s registered)"
        width = max(len(n) for n in self._entries)
        lines = []
        for entry in self.entries():
            tagstr = f"  [{','.join(entry.tags)}]" if entry.tags else ""
            lines.append(f"  {entry.name:<{width}}  {entry.summary}{tagstr}")
        return "\n".join(lines)

    def __contains__(self, name: object) -> bool:
        return name in self._entries

    def __iter__(self) -> Iterator[str]:
        return iter(self.names())

    def __len__(self) -> int:
        return len(self._entries)

    def __repr__(self) -> str:
        return f"Registry({self.kind!r}, {len(self._entries)} entries)"
