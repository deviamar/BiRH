"""``ObjectSpec`` -> MJCF."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from ..core.logging import get_logger
from ..core.paths import asset_root
from ..core.types import Pose
from ..mjcf.element import Element
from ..mjcf.model import MjcfModel, NameIndex
from ..mjcf.transforms import quat_to_str
from .spec import ObjectSpec

log = get_logger(__name__)

__all__ = ["build_object", "ObjectBuild"]


@dataclass(frozen=True)
class ObjectBuild:
    model: MjcfModel
    spec: ObjectSpec
    index: NameIndex


def build_object(
    spec: ObjectSpec,
    *,
    pose: Pose | None = None,
    prefix: str = "obj_",
    free: bool = True,
    standalone: bool = False,
) -> ObjectBuild:
    """Generate MJCF for one object.

    Parameters
    ----------
    pose:
        Initial pose.  ``None`` places the object at the origin at its resting height, so
        it sits on a support surface at ``z = 0`` rather than starting in penetration.
    free:
        Attach a free joint.  Fixed objects (a table-mounted handle) set this to False.
    """
    p = prefix
    pose = pose or Pose(pos=(0.0, 0.0, spec.rest_height), quat=spec.spawn_quat)

    root = Element("mujoco", model=f"{p}{spec.name}")
    index = NameIndex()

    if standalone:
        root.add("compiler", angle="radian", autolimits=True)
        root.add("option", timestep=0.002, integrator="implicitfast", cone="elliptic")
        world = root.section("worldbody")
        world.add("light", pos=[0, 0, 1.0], dir=[0, 0, -1], directional=True)
        world.add("geom", name="floor", type="plane", size=[1, 1, 0.05])

    _add_assets(root, spec, prefix=p)
    _add_defaults(root, spec, prefix=p)

    body_name = f"{p}root"
    body = root.section("worldbody").add(
        "body", name=body_name, pos=list(pose.pos), quat=quat_to_str(pose.quat)
    )
    index.bodies["root"] = body_name
    if free:
        body.add("freejoint", name=f"{p}free")
        index.joints["free"] = f"{p}free"

    body.add("site", name=f"{p}center", pos=[0, 0, 0], size=[0.004], group=4)
    index.sites["center"] = f"{p}center"

    density = spec.density
    geom_names: list[str] = []
    for i, part in enumerate(spec.parts):
        name = f"{p}g{i}"
        attrs = {
            "name": name,
            "type": part.type,
            "pos": list(part.pos),
            "quat": quat_to_str(part.quat),
            "class_": f"{p}object",
            "density": density,
            "material": f"mat_{p}{part.material or spec.material}",
        }
        if part.type == "mesh":
            attrs["mesh"] = f"mesh_{p}{part.mesh}"
        else:
            attrs["size"] = list(part.size)
        if part.material and part.material != spec.material:
            mat = spec.resolve_material(part.material)
            attrs["friction"] = list(mat.friction)
        body.add("geom", **attrs)
        geom_names.append(name)
        index.geoms[f"part{i}"] = name

    index.groups["geoms"] = geom_names

    model = MjcfModel(
        root,
        name=f"{p}{spec.name}",
        index=index,
        meta={
            "object_id": spec.object_id,
            "object": spec.name,
            "category": spec.category,
            "size_class": spec.size_class,
            "mass": spec.mass,
            "width": spec.width,
            "spec": spec.to_dict(),
        },
    )
    return ObjectBuild(model=model, spec=spec, index=index)


def _add_defaults(root: Element, spec: ObjectSpec, *, prefix: str) -> None:
    material = spec.resolve_material(None)
    cls = root.section("default").add("default", class_=f"{prefix}object")
    cls.add(
        "geom",
        contype=1,
        conaffinity=1,
        condim=4,  # torsional friction, so a grasped object cannot spin freely on a pad
        group=1,
        friction=list(material.friction),
        solref=material.solver.solref,
        solimp=material.solver.solimp,
    )


def _add_assets(root: Element, spec: ObjectSpec, *, prefix: str) -> None:
    asset = root.section("asset")
    seen: set[str] = set()
    for part in spec.parts:
        name = part.material or spec.material
        if name in seen:
            continue
        seen.add(name)
        mat = spec.resolve_material(name)
        asset.add(
            "material", name=f"mat_{prefix}{name}", rgba=list(mat.rgba),
            specular=0.35, shininess=0.3,
        )

    meshes = {p.mesh for p in spec.parts if p.type == "mesh" and p.mesh}
    if not meshes:
        return
    filename = spec.meta.get("mesh_file")
    scale = float(spec.meta.get("mesh_scale", 1.0))
    for mesh in sorted(meshes):
        path = Path(filename) if filename else Path(f"{mesh}.stl")
        full = path if path.is_absolute() else asset_root() / path
        if not full.exists():
            raise FileNotFoundError(f"mesh file for object {spec.name!r} not found: {full}")
        asset.add("mesh", name=f"mesh_{prefix}{mesh}", file=str(full), scale=[scale] * 3)
