"""The object library.

Objects are stratified over the properties expected to matter -- graspable width, aspect
ratio, mass, surface friction and convexity -- rather than collected ad hoc, so that
"works across various shapes" acquires a defined scope (``docs/00-REVIEW.md`` §4.5).

Named sets (:data:`object_sets`) exist because reporting a single average over a
convenience sample is how object-set overfitting hides.  ``core15`` is the standard
evaluation set; ``train`` and ``test`` are a disjoint split so an optimized morphology can
be scored on objects the optimizer never saw.
"""

from __future__ import annotations

import math
from typing import Callable, Sequence

from ..core.paths import asset_root
from ..core.registry import Registry
from ..mjcf.transforms import quat_from_axis_angle
from .spec import ObjectSpec, Part

__all__ = ["objects", "object_sets", "get_object", "get_object_set", "mesh_object"]

objects: Registry[ObjectSpec] = Registry("object")
object_sets: Registry[list[str]] = Registry("object set")

_X = (1.0, 0.0, 0.0)
_Y = (0.0, 1.0, 0.0)


def get_object(name: str, **kwargs) -> ObjectSpec:
    """Instantiate an object by name.

    Accepts a ``name@scale`` suffix (``"cube_medium@1.5"``) so size sweeps need no extra
    registry entries.
    """
    if "@" in name:
        base, _, factor = name.partition("@")
        return objects.create(base, **kwargs).scaled(float(factor))
    return objects.create(name, **kwargs)


def get_object_set(name: str) -> list[ObjectSpec]:
    """Instantiate a named set of objects."""
    return [get_object(n) for n in object_sets.create(name)]


def _reg(name: str, doc: str, tags: Sequence[str] = ()) -> Callable:
    return objects.register(name, doc=doc, tags=tuple(tags))


# =======================================================================================
# primitives -- the controlled end of the library
# =======================================================================================

@_reg("sphere_small", "2.5 cm sphere: fingertip-only, the hardest precision case", ["primitive", "round"])
def _sphere_small() -> ObjectSpec:
    return ObjectSpec("sphere_small", (Part("sphere", (0.0125,)),), mass=0.02,
                      material="plastic", category="primitive",
                      affords=("precision", "tripod"), tags=("round", "convex"))


@_reg("sphere_medium", "5 cm sphere: the canonical enclosing grasp", ["primitive", "round"])
def _sphere_medium() -> ObjectSpec:
    return ObjectSpec("sphere_medium", (Part("sphere", (0.025,)),), mass=0.08,
                      material="plastic", category="primitive",
                      affords=("power", "spherical"), tags=("round", "convex"))


@_reg("sphere_large", "8 cm sphere: tests aperture limits", ["primitive", "round"])
def _sphere_large() -> ObjectSpec:
    return ObjectSpec("sphere_large", (Part("sphere", (0.04,)),), mass=0.18,
                      material="plastic", category="primitive",
                      affords=("power", "spherical"), tags=("round", "convex"))


@_reg("cube_small", "3 cm cube: flat faces reward pad area over enclosure", ["primitive", "flat"])
def _cube_small() -> ObjectSpec:
    return ObjectSpec("cube_small", (Part("box", (0.015, 0.015, 0.015)),), mass=0.03,
                      material="wood", category="primitive",
                      affords=("precision", "lateral"), tags=("faceted", "convex"))


@_reg("cube_medium", "5 cm cube", ["primitive", "flat"])
def _cube_medium() -> ObjectSpec:
    return ObjectSpec("cube_medium", (Part("box", (0.025, 0.025, 0.025)),), mass=0.09,
                      material="wood", category="primitive",
                      affords=("power", "lateral"), tags=("faceted", "convex"))


@_reg("cylinder_upright", "6 cm tall, 3.5 cm diameter cylinder standing on end",
      ["primitive", "cylindrical"])
def _cylinder_upright() -> ObjectSpec:
    return ObjectSpec("cylinder_upright", (Part("cylinder", (0.0175, 0.03)),), mass=0.07,
                      material="plastic", category="primitive",
                      affords=("cylindrical", "power"), tags=("round", "convex"))


@_reg("cylinder_lying", "A 12 cm cylinder lying down: tests wrap-around alignment",
      ["primitive", "cylindrical", "elongated"])
def _cylinder_lying() -> ObjectSpec:
    return ObjectSpec(
        "cylinder_lying",
        (Part("cylinder", (0.022, 0.06), quat=quat_from_axis_angle(_Y, math.pi / 2)),),
        mass=0.12, material="wood", category="primitive",
        affords=("cylindrical", "power", "hook"), tags=("round", "elongated", "convex"),
    )


@_reg("disc_flat", "9 cm diameter, 1 cm thick disc lying flat: nothing to get under",
      ["primitive", "flat"])
def _disc_flat() -> ObjectSpec:
    return ObjectSpec("disc_flat", (Part("cylinder", (0.045, 0.005)),), mass=0.05,
                      material="plastic", category="flat",
                      affords=("lateral", "platform"), tags=("flat", "convex"),
                      graspable_width=0.01)


# =======================================================================================
# household objects -- composites of convex parts
# =======================================================================================

@_reg("apple", "Apple-sized rounded body with a stalk", ["food", "round"])
def _apple() -> ObjectSpec:
    return ObjectSpec(
        "apple",
        (
            Part("ellipsoid", (0.037, 0.037, 0.033)),
            Part("cylinder", (0.003, 0.010), pos=(0.0, 0.0, 0.040)),
        ),
        mass=0.15, material="organic", category="food",
        affords=("power", "spherical"), tags=("round", "smooth"),
        graspable_width=0.070,
    )


@_reg("banana", "Curved elongated body: three offset capsules", ["food", "elongated"])
def _banana() -> ObjectSpec:
    segments = tuple(
        Part(
            "capsule", (0.018, 0.026),
            pos=(dx, 0.0, dz),
            # Rotating about -y by the tilt lays the capsule along x with a gentle arc.
            quat=quat_from_axis_angle(_Y, math.pi / 2 - tilt),
        )
        for dx, dz, tilt in ((-0.042, 0.008, -0.30), (0.0, 0.0, 0.0), (0.042, 0.008, 0.30))
    )
    return ObjectSpec("banana", segments, mass=0.12, material="organic",
                      category="food", affords=("power", "cylindrical", "hook"),
                      tags=("elongated", "curved"), graspable_width=0.036)


@_reg("bottle", "Water bottle: 18 cm body with a tapered neck and cap",
      ["container", "cylindrical", "elongated"])
def _bottle() -> ObjectSpec:
    return ObjectSpec(
        "bottle",
        (
            Part("cylinder", (0.033, 0.065), pos=(0.0, 0.0, 0.065)),
            Part("cylinder", (0.024, 0.020), pos=(0.0, 0.0, 0.148)),
            Part("cylinder", (0.014, 0.018), pos=(0.0, 0.0, 0.185)),
            Part("cylinder", (0.016, 0.010), pos=(0.0, 0.0, 0.210), material="plastic"),
        ),
        mass=0.45, material="plastic", category="container",
        affords=("power", "cylindrical"), tags=("tall", "heavy"), graspable_width=0.066,
    )


@_reg("mug", "Mug with a genuinely non-convex handle (a hook-grasp target)",
      ["container", "handle"])
def _mug() -> ObjectSpec:
    """The handle is built from four short capsules forming an open loop.

    A convex-hull mesh would fill the handle solid and make the hook grasp -- the exact
    capability being tested -- impossible to observe.
    """
    wall = [
        Part("cylinder", (0.042, 0.048), pos=(0.0, 0.0, 0.048)),
    ]
    handle = []
    for k in range(4):
        theta = -math.pi / 2 + math.pi * (k + 0.5) / 4
        handle.append(
            Part(
                "capsule", (0.006, 0.012),
                pos=(0.042 + 0.024 * math.cos(theta), 0.0, 0.048 + 0.030 * math.sin(theta)),
                quat=quat_from_axis_angle(_Y, -theta),
            )
        )
    return ObjectSpec("mug", tuple(wall + handle), mass=0.28, material="plastic",
                      category="container", affords=("power", "cylindrical", "hook"),
                      tags=("non_convex", "handle"), graspable_width=0.084)


@_reg("tape_roll", "Roll of tape: an annulus built from 8 arc segments",
      ["household", "annular"])
def _tape() -> ObjectSpec:
    """A ring, not a disc -- the hole is the point. One of the objects the original
    project tested against by hand (``docs/00-REVIEW.md`` §4.5)."""
    parts = []
    n = 8
    for k in range(n):
        theta = 2 * math.pi * k / n
        parts.append(
            Part(
                "box", (0.012, 0.019, 0.011),
                pos=(0.035 * math.cos(theta), 0.035 * math.sin(theta), 0.011),
                quat=quat_from_axis_angle((0.0, 0.0, 1.0), theta),
            )
        )
    return ObjectSpec("tape_roll", tuple(parts), mass=0.09, material="plastic",
                      category="household", affords=("lateral", "hook", "precision"),
                      tags=("non_convex", "annular"))


@_reg("hammer", "Hammer: heavy head on a long handle -- a torque-balance test",
      ["tool", "elongated", "heavy"])
def _hammer() -> ObjectSpec:
    return ObjectSpec(
        "hammer",
        (
            Part("cylinder", (0.014, 0.075), pos=(0.0, 0.0, 0.075)),
            Part("cylinder", (0.018, 0.048), pos=(0.0, 0.0, 0.160),
                 quat=quat_from_axis_angle(_Y, math.pi / 2), material="metal"),
            Part("box", (0.012, 0.020, 0.022), pos=(0.036, 0.0, 0.160), material="metal"),
        ),
        mass=0.55, material="wood", category="tool",
        affords=("power", "cylindrical"), tags=("elongated", "eccentric_mass", "heavy"),
        graspable_width=0.028,
    )


@_reg("screwdriver", "Screwdriver: thin shaft, fatter grip", ["tool", "elongated"])
def _screwdriver() -> ObjectSpec:
    return ObjectSpec(
        "screwdriver",
        (
            Part("cylinder", (0.014, 0.045), pos=(0.0, 0.0, 0.045)),
            Part("cylinder", (0.004, 0.050), pos=(0.0, 0.0, 0.140), material="metal"),
        ),
        mass=0.09, material="plastic", category="tool",
        affords=("cylindrical", "precision"), tags=("elongated",), graspable_width=0.028,
    )


@_reg("marker", "Whiteboard marker: the canonical tripod-pinch object", ["household", "elongated"])
def _marker() -> ObjectSpec:
    return ObjectSpec(
        "marker",
        (Part("cylinder", (0.009, 0.062), quat=quat_from_axis_angle(_Y, math.pi / 2)),),
        mass=0.015, material="plastic", category="household",
        affords=("tripod", "precision", "lateral"), tags=("elongated", "small"),
        graspable_width=0.018,
    )


@_reg("pen", "Pen: 7 mm diameter, at the limit of fingertip precision", ["household", "elongated"])
def _pen() -> ObjectSpec:
    return ObjectSpec(
        "pen",
        (Part("capsule", (0.0035, 0.065), quat=quat_from_axis_angle(_Y, math.pi / 2)),),
        mass=0.008, material="plastic", category="household",
        affords=("tripod", "precision"), tags=("elongated", "tiny"), graspable_width=0.007,
    )


@_reg("phone", "Phone: a thin slab -- pure lateral pinch, no enclosure possible",
      ["household", "flat"])
def _phone() -> ObjectSpec:
    """The object class the v2 hand testing found impossible without thumb abduction
    (``docs/00-REVIEW.md`` §2.5)."""
    return ObjectSpec("phone", (Part("box", (0.037, 0.075, 0.005)),), mass=0.19,
                      material="glass", category="flat",
                      affords=("lateral", "precision"), tags=("flat", "slippery"),
                      graspable_width=0.010)


@_reg("remote", "TV remote: a long thin slab", ["household", "flat", "elongated"])
def _remote() -> ObjectSpec:
    return ObjectSpec("remote", (Part("box", (0.024, 0.085, 0.010)),), mass=0.12,
                      material="plastic", category="flat",
                      affords=("power", "lateral"), tags=("flat", "elongated"),
                      graspable_width=0.020)


@_reg("scissors", "Scissors: two crossed blades and two ring handles", ["tool", "flat"])
def _scissors() -> ObjectSpec:
    parts = [
        Part("box", (0.006, 0.045, 0.002), pos=(0.006, 0.045, 0.0),
             quat=quat_from_axis_angle((0.0, 0.0, 1.0), 0.10), material="metal"),
        Part("box", (0.006, 0.045, 0.002), pos=(-0.006, 0.045, 0.0),
             quat=quat_from_axis_angle((0.0, 0.0, 1.0), -0.10), material="metal"),
    ]
    for sign in (1, -1):
        for k in range(6):
            theta = 2 * math.pi * k / 6
            parts.append(
                Part("box", (0.004, 0.007, 0.002),
                     pos=(sign * 0.016 + 0.014 * math.cos(theta), -0.030 + 0.018 * math.sin(theta), 0.0),
                     quat=quat_from_axis_angle((0.0, 0.0, 1.0), theta))
            )
    return ObjectSpec("scissors", tuple(parts), mass=0.07, material="metal",
                      category="tool", affords=("lateral", "precision", "hook"),
                      tags=("non_convex", "flat", "thin"), graspable_width=0.010)


@_reg("can", "Drinks can: 6.6 cm diameter, 12 cm tall", ["container", "cylindrical"])
def _can() -> ObjectSpec:
    return ObjectSpec("can", (Part("cylinder", (0.033, 0.061), pos=(0.0, 0.0, 0.061)),),
                      mass=0.36, material="metal", category="container",
                      affords=("power", "cylindrical"), tags=("round", "heavy", "slippery"),
                      graspable_width=0.066)


@_reg("box_carton", "Cereal-box-shaped carton: wide and thin", ["container", "flat"])
def _carton() -> ObjectSpec:
    return ObjectSpec("box_carton", (Part("box", (0.080, 0.028, 0.105), pos=(0, 0, 0.105)),),
                      mass=0.36, material="wood", category="container",
                      affords=("power", "lateral"), tags=("large", "faceted"),
                      graspable_width=0.056)


@_reg("key", "A key: the smallest thing worth trying to pick up", ["household", "flat"])
def _key() -> ObjectSpec:
    return ObjectSpec(
        "key",
        (Part("box", (0.010, 0.022, 0.0012), material="metal"),
         Part("cylinder", (0.009, 0.0012), pos=(0.0, -0.026, 0.0), material="metal")),
        mass=0.012, material="metal", category="flat",
        affords=("lateral", "precision"), tags=("tiny", "flat", "thin"), graspable_width=0.0024,
    )


@_reg("wire_loop", "A coiled wire: light, thin and floppy in outline", ["household", "thin"])
def _wire() -> ObjectSpec:
    """One of the objects used in the original hand-testing protocol. Modelled as a rigid
    coil, so it tests *thinness*, not compliance -- deformable bodies are a later
    milestone and the distinction is recorded here rather than glossed over."""
    parts = []
    n = 12
    for k in range(n):
        theta = 2 * math.pi * k / n
        parts.append(
            Part("capsule", (0.0018, 0.011),
                 pos=(0.040 * math.cos(theta), 0.040 * math.sin(theta), 0.0),
                 quat=quat_from_axis_angle((0.0, 0.0, 1.0), theta + math.pi / 2),
                 material="metal")
        )
    return ObjectSpec("wire_loop", tuple(parts), mass=0.02, material="metal",
                      category="household", affords=("precision", "hook"),
                      tags=("thin", "non_convex"))


@_reg("plate", "A plate: flat, wide, and nearly impossible to grasp from above",
      ["household", "flat"])
def _plate() -> ObjectSpec:
    return ObjectSpec("plate", (Part("cylinder", (0.075, 0.004)),), mass=0.22,
                      material="glass", category="flat",
                      affords=("lateral", "platform"), tags=("flat", "wide", "slippery"),
                      graspable_width=0.008)


# =======================================================================================
# mesh objects (YCB) -- available, and honestly labelled
# =======================================================================================

def mesh_object(
    name: str,
    filename: str,
    *,
    mass: float,
    category: str = "mesh",
    scale: float = 1.0,
    material: str = "plastic",
    affords: Sequence[str] = (),
    approx_radius: float = 0.05,
) -> ObjectSpec:
    """Wrap a mesh from ``simulation/mujoco-models/assets``.

    Collision uses MuJoCo's convex hull of the mesh, so concavities (a mug handle, a
    pitcher spout) do not exist for contact.  The returned spec is marked
    ``convex_hull_collision=True`` so analysis can separate these results from the
    composite objects, whose contact geometry is faithful.
    """
    path = asset_root() / filename
    if not path.exists():
        raise FileNotFoundError(f"mesh asset not found: {path}")
    return ObjectSpec(
        name=name,
        parts=(Part("mesh", (approx_radius,), mesh=name),),
        mass=mass, material=material, category=category, affords=tuple(affords),
        tags=("mesh",), convex_hull_collision=True,
        meta={"mesh_file": filename, "mesh_scale": scale},
    )


_YCB: dict[str, dict] = {
    "ycb_apple": dict(filename="apple.stl", mass=0.068, category="food",
                      affords=("power", "spherical"), approx_radius=0.037),
    "ycb_pear": dict(filename="pear.stl", mass=0.049, category="food",
                     affords=("power", "spherical"), approx_radius=0.035),
    "ycb_golf_ball": dict(filename="golf_ball.stl", mass=0.046, category="sport",
                          affords=("precision", "tripod"), approx_radius=0.021),
    "ycb_softball": dict(filename="softball.stl", mass=0.191, category="sport",
                         affords=("power", "spherical"), approx_radius=0.048),
    "ycb_tuna_can": dict(filename="tuna_can.stl", mass=0.171, category="container",
                         affords=("lateral", "power"), approx_radius=0.043),
    "ycb_chips_can": dict(filename="chips_can.stl", mass=0.205, category="container",
                          affords=("power", "cylindrical"), approx_radius=0.038),
    "ycb_cup": dict(filename="cup.stl", mass=0.075, category="container",
                    affords=("power", "cylindrical"), approx_radius=0.040),
    "ycb_screwdriver": dict(filename="screwdriver.stl", mass=0.098, category="tool",
                            affords=("cylindrical", "precision"), approx_radius=0.020),
    "ycb_power_drill": dict(filename="power_drill.stl", mass=0.895, category="tool",
                            affords=("power",), approx_radius=0.060),
    "ycb_pudding_box": dict(filename="pudding.stl", mass=0.187, category="container",
                            affords=("lateral", "power"), approx_radius=0.045),
}


def _register_mesh_objects() -> None:
    """Register YCB wrappers, skipping any whose file is absent."""
    root = asset_root()
    for name, cfg in _YCB.items():
        if not (root / cfg["filename"]).exists():
            continue
        objects.add(
            name,
            (lambda n=name, c=cfg: mesh_object(n, **c)),
            doc=f"YCB mesh: {cfg['filename']} (convex-hull collision)",
            tags=("ycb", "mesh", c_cat) if (c_cat := cfg.get("category")) else ("ycb", "mesh"),
        )


_register_mesh_objects()


# =======================================================================================
# named sets
# =======================================================================================

@object_sets.register("core15", doc="Standard 15-object evaluation set, stratified by size and shape")
def _core15() -> list[str]:
    """The default evaluation set: spans tiny to large, round to flat, convex to
    non-convex, light to heavy.  Every result reported against this set is comparable to
    every other."""
    return [
        "sphere_small", "sphere_medium", "sphere_large",
        "cube_small", "cube_medium",
        "cylinder_upright", "cylinder_lying",
        "apple", "banana", "bottle", "mug", "can",
        "marker", "phone", "tape_roll",
    ]


@object_sets.register("train", doc="Training split: an optimizer may see these")
def _train() -> list[str]:
    return [
        "sphere_small", "sphere_large", "cube_medium", "cylinder_upright",
        "apple", "bottle", "marker", "tape_roll", "remote", "can",
    ]


@object_sets.register("test", doc="Held-out split: never shown to an optimizer")
def _test() -> list[str]:
    """Reporting held-out performance is what separates morphology optimization from
    morphology overfitting (``docs/00-REVIEW.md`` §7, Study 3)."""
    return [
        "sphere_medium", "cube_small", "cylinder_lying",
        "banana", "mug", "phone", "hammer", "screwdriver", "box_carton", "pen",
    ]


@object_sets.register("quick", doc="Four objects for smoke tests and pilots")
def _quick() -> list[str]:
    return ["sphere_medium", "cube_medium", "cylinder_upright", "marker"]


@object_sets.register("hard", doc="Objects that defeat simple hands: thin, flat or non-convex")
def _hard() -> list[str]:
    return ["phone", "plate", "key", "wire_loop", "scissors", "tape_roll", "pen", "disc_flat"]


@object_sets.register("primitives", doc="Geometric primitives only: the controlled condition")
def _primitives() -> list[str]:
    return [n for n in objects.names(tag="primitive")]


@object_sets.register("ycb", doc="YCB mesh objects (convex-hull collision)")
def _ycb() -> list[str]:
    return objects.names(tag="ycb")


@object_sets.register("all", doc="Every registered object")
def _all() -> list[str]:
    return objects.names()


def categories() -> dict[str, list[str]]:
    """Object names grouped by category, for per-category reporting."""
    out: dict[str, list[str]] = {}
    for name in objects.names():
        try:
            spec = get_object(name)
        except Exception:  # noqa: BLE001 - a missing mesh asset must not break listing
            continue
        out.setdefault(spec.category, []).append(name)
    return {k: sorted(v) for k, v in sorted(out.items())}
