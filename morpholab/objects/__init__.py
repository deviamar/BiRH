"""The object library: specs, registry, named sets, and MJCF generation."""

from __future__ import annotations

from .builder import ObjectBuild, build_object
from .library import categories, get_object, get_object_set, mesh_object, object_sets, objects
from .spec import ObjectSpec, Part, SizeClass, size_class_of

__all__ = [
    "ObjectSpec", "Part", "SizeClass", "size_class_of",
    "objects", "object_sets", "get_object", "get_object_set", "mesh_object", "categories",
    "build_object", "ObjectBuild",
]
