"""A small declarative MJCF element tree.

The original project generated XML with f-strings, which is how a stray ``quat="..."``
fragment ended up outside a tag and went unnoticed (``docs/00-REVIEW.md`` §3.1).  This
module replaces string formatting with a tree that can be composed, merged, queried and
validated before it is ever handed to the compiler.

Design notes
------------
* Attribute values accept Python types (floats, sequences, bools, enums) and are formatted
  to MJCF conventions on serialization -- so callers never build attribute strings.
* ``merge`` implements MJCF include semantics for the sections that legitimately merge
  (``asset``, ``default``, ``worldbody``, ``actuator``, ...), which is what lets a scene be
  assembled from independent hand and object subtrees.
* Names are checked for collisions at merge time, because MuJoCo's own error for a
  duplicate name does not say which of the two subtrees introduced it.
"""

from __future__ import annotations

import re
from enum import Enum
from typing import Any, Iterable, Iterator, Sequence
from xml.etree import ElementTree as ET

import numpy as np

__all__ = ["Element", "MjcfError", "fmt_value"]

_NAME_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_.\-]*$")

#: MJCF sections whose children combine when two models are merged.
MERGEABLE_SECTIONS = frozenset(
    {
        "asset", "default", "worldbody", "actuator", "sensor", "tendon", "equality",
        "contact", "keyframe", "custom", "extension", "deformable",
    }
)

#: Sections that carry settings rather than lists; the later value wins on merge.
SINGLETON_SECTIONS = frozenset({"compiler", "option", "size", "visual", "statistic"})


class MjcfError(Exception):
    """Raised for malformed MJCF trees, name collisions, or invalid attribute values."""


def fmt_value(value: Any, digits: int = 8) -> str:
    """Format a Python value as an MJCF attribute string.

    Booleans become ``true``/``false`` (MJCF spelling, not Python's), sequences become
    space-separated numbers, and floats use general format so that ``0.1`` stays ``0.1``
    rather than becoming ``0.100000``.
    """
    if isinstance(value, str):
        return value
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, Enum):
        return fmt_value(value.value, digits)
    if isinstance(value, (int, np.integer)):
        return str(int(value))
    if isinstance(value, (float, np.floating)):
        v = float(value)
        if not np.isfinite(v):
            raise MjcfError(f"non-finite attribute value: {value!r}")
        return f"{v:.{digits}g}"
    if isinstance(value, np.ndarray):
        return fmt_value(value.ravel().tolist(), digits)
    if isinstance(value, (list, tuple)):
        return " ".join(fmt_value(v, digits) for v in value)
    raise MjcfError(f"cannot format attribute value of type {type(value).__name__}: {value!r}")


class Element:
    """One MJCF element: a tag, attributes, and children.

    Examples
    --------
    >>> body = Element("body", name="palm", pos=(0, 0, 0.1))
    >>> body.add("geom", type="cylinder", size=(0.05, 0.012))
    Element(geom)
    >>> "cylinder" in body.to_xml()
    True
    """

    __slots__ = ("tag", "attrib", "children")

    def __init__(self, tag: str, _children: Iterable["Element"] = (), **attrib: Any) -> None:
        self.tag = tag
        self.attrib = {}
        self.children = list(_children)
        self.set(**attrib)

    # -- attributes -----------------------------------------------------------------

    def set(self, **attrib: Any) -> "Element":
        """Set attributes, skipping ``None`` values (so optional args need no branching)."""
        for key, value in attrib.items():
            if value is None:
                continue
            self.attrib[key.rstrip("_").replace("__", ":")] = value
        return self

    def get(self, key: str, default: Any = None) -> Any:
        return self.attrib.get(key, default)

    @property
    def name(self) -> str | None:
        n = self.attrib.get("name")
        return str(n) if n is not None else None

    # -- children -------------------------------------------------------------------

    def add(self, tag: str, **attrib: Any) -> "Element":
        """Append a new child and return *the child*, for chaining down a body tree."""
        child = Element(tag, **attrib)
        self.children.append(child)
        return child

    def append(self, *elements: "Element") -> "Element":
        """Append existing elements and return *self*, for chaining across siblings."""
        self.children.extend(elements)
        return self

    def section(self, tag: str) -> "Element":
        """Get or create a direct child with ``tag`` -- the ``<asset>``/``<actuator>`` idiom."""
        for child in self.children:
            if child.tag == tag:
                return child
        return self.add(tag)

    def find_all(self, tag: str) -> list["Element"]:
        """All descendants with ``tag``, in document order."""
        out: list[Element] = []
        stack = list(reversed(self.children))
        while stack:
            node = stack.pop()
            if node.tag == tag:
                out.append(node)
            stack.extend(reversed(node.children))
        return out

    def find_named(self, tag: str, name: str) -> "Element | None":
        for node in self.find_all(tag):
            if node.name == name:
                return node
        return None

    def names(self, tag: str | None = None) -> list[str]:
        """Every ``name`` attribute in the subtree, optionally filtered by tag."""
        out = []
        for node in self.walk():
            if node.name and (tag is None or node.tag == tag):
                out.append(node.name)
        return out

    def walk(self) -> Iterator["Element"]:
        """Depth-first traversal including ``self``."""
        yield self
        for child in self.children:
            yield from child.walk()

    def __iter__(self) -> Iterator["Element"]:
        return iter(self.children)

    def __len__(self) -> int:
        return len(self.children)

    # -- merging --------------------------------------------------------------------

    def merge(self, other: "Element", *, check_names: bool = True) -> "Element":
        """Merge ``other``'s top-level sections into this tree, in place.

        Mergeable sections (``asset``, ``worldbody``, ``actuator``, ...) concatenate their
        children; singleton sections (``option``, ``compiler``, ...) take ``other``'s
        attributes.  This is what lets a scene be assembled as ``world.merge(hand).merge(object)``
        with each subtree built and tested in isolation.
        """
        if check_names:
            collisions = _name_collisions(self, other)
            if collisions:
                shown = ", ".join(sorted(collisions)[:8])
                more = "" if len(collisions) <= 8 else f" (+{len(collisions) - 8} more)"
                raise MjcfError(
                    f"cannot merge: {len(collisions)} duplicate element name(s): {shown}{more}. "
                    "Prefix names per subtree (the hand builder uses the digit name)."
                )
        for section in other.children:
            if section.tag in SINGLETON_SECTIONS:
                self.section(section.tag).set(**section.attrib)
                self.section(section.tag).children.extend(section.children)
            elif section.tag in MERGEABLE_SECTIONS:
                self.section(section.tag).children.extend(section.children)
            else:
                self.children.append(section)
        return self

    def copy(self) -> "Element":
        """Deep copy."""
        clone = Element(self.tag, **dict(self.attrib))
        clone.children = [c.copy() for c in self.children]
        return clone

    # -- serialization --------------------------------------------------------------

    def to_etree(self, digits: int = 8) -> ET.Element:
        node = ET.Element(self.tag, {k: fmt_value(v, digits) for k, v in self.attrib.items()})
        for child in self.children:
            node.append(child.to_etree(digits))
        return node

    def to_xml(self, *, pretty: bool = True, digits: int = 8, header: str | None = None) -> str:
        """Serialize to an MJCF string."""
        node = self.to_etree(digits)
        if pretty:
            _indent(node)
        text = ET.tostring(node, encoding="unicode")
        return f"<!-- {header} -->\n{text}" if header else text

    def validate(self) -> list[str]:
        """Structural problems, as human-readable strings. Empty means the tree looks sane.

        Catches the failure modes that produce confusing MuJoCo errors: duplicate names
        within a tag, invalid identifiers, and bodies with no geometry (which compile fine
        and then behave as massless phantoms).
        """
        problems: list[str] = []
        seen: dict[tuple[str, str], int] = {}
        for node in self.walk():
            if node.name is not None:
                if not _NAME_RE.match(node.name):
                    problems.append(f"invalid name {node.name!r} on <{node.tag}>")
                key = (_name_scope(node.tag), node.name)
                seen[key] = seen.get(key, 0) + 1
            if node.tag == "body" and not any(
                c.tag in ("geom", "body", "site", "camera", "light") for c in node.children
            ):
                problems.append(f"body {node.name or '<unnamed>'} has no geoms or children")
        problems += [f"duplicate {scope} name {name!r} ({n} times)" for (scope, name), n in seen.items() if n > 1]
        return problems

    def __repr__(self) -> str:
        name = f" {self.name}" if self.name else ""
        kids = f", {len(self.children)} children" if self.children else ""
        return f"Element({self.tag}{name}{kids})"


def _name_scope(tag: str) -> str:
    """MuJoCo namespaces names by object type; bodies and geoms may share a name."""
    return {
        "body": "body", "geom": "geom", "joint": "joint", "site": "site", "mesh": "mesh",
        "material": "material", "texture": "texture", "camera": "camera", "light": "light",
        "tendon": "tendon", "sensor": "sensor", "equality": "equality", "key": "key",
    }.get(tag, tag if tag in ("position", "motor", "general", "velocity", "spatial", "fixed") else "other")


def _name_collisions(a: Element, b: Element) -> set[str]:
    def keyed(el: Element) -> set[str]:
        return {
            f"{_name_scope(n.tag)}:{n.name}"
            for n in el.walk()
            if n.name and _name_scope(n.tag) not in ("other", "default")
        }

    # Actuator element names share a namespace across their concrete tags.
    return {k.split(":", 1)[1] for k in keyed(a) & keyed(b)}


def _indent(elem: ET.Element, level: int = 0, step: str = "  ") -> None:
    pad = "\n" + level * step
    if len(elem):
        if not (elem.text or "").strip():
            elem.text = pad + step
        for child in elem:
            _indent(child, level + 1, step)
        if not (elem[-1].tail or "").strip():
            elem[-1].tail = pad
    if level and not (elem.tail or "").strip():
        elem.tail = pad


def element_from_xml(text: str) -> Element:
    """Parse MJCF text back into an :class:`Element` tree.

    Used to ingest the legacy hand-authored models in ``simulation/mujoco-models`` so they
    can be compared against generated ones, and to load vendored assets.
    """
    def convert(node: ET.Element) -> Element:
        el = Element(node.tag, **dict(node.attrib))
        el.children = [convert(c) for c in node]
        return el

    return convert(ET.fromstring(text))
