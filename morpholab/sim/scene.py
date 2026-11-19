"""Scene assembly: world + hand + object + wrist.

The wrist is a **mocap body welded to the palm** (``docs/01-ARCHITECTURE.md`` §6.1).
Commanding the mocap pose moves the hand *through the weld's compliance*, so lift and
shake tests transmit real forces into the grasp -- unlike teleporting the palm, which
would let a hand "carry" an object it is not actually holding.  No arm is needed for
morphology studies, which is what makes a 20,000-trial sweep affordable.

Scene frame: ``+z`` is up, the support surface is the plane ``z = 0``, and the object
starts at the origin.  The hand approaches from a direction the task chooses; for a
top-down grasp the palm is rolled 180 degrees about ``x`` so its normal (local ``+z``)
points down at the object.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Sequence

import numpy as np

from ..core.logging import get_logger
from ..core.types import MATERIALS, Pose, Quat, SolverSpec, Vec3
from ..mjcf.element import Element
from ..mjcf.model import MjcfModel, NameIndex
from ..mjcf.transforms import (
    matrix_from_quat,
    quat_from_axis_angle,
    quat_from_matrix,
    quat_multiply,
    quat_to_str,
)
from ..morphology.builder import build_hand
from ..morphology.spec import HandSpec
from .approach import ApproachPlan, plan_approach
from ..objects.builder import build_object
from ..objects.spec import ObjectSpec

if TYPE_CHECKING:  # pragma: no cover
    import mujoco

log = get_logger(__name__)

__all__ = ["SceneOptions", "SceneBuild", "build_scene", "approach_pose"]

HAND_PREFIX = "hand_"
OBJ_PREFIX = "obj_"


@dataclass(frozen=True)
class SceneOptions:
    """Everything about a scene that is not the hand or the object."""

    timestep: float = 0.002
    integrator: str = "implicitfast"
    """``implicitfast`` integrates damping implicitly and is stable at roughly ten times
    the timestep RK4 needs for a contact-rich model (``docs/00-REVIEW.md`` §3.5)."""

    cone: str = "elliptic"
    """Elliptic friction cones are the true Coulomb cone; MuJoCo's default pyramidal
    approximation over-estimates friction along the pyramid's diagonals by up to 41 %,
    which would bias every grasp-robustness number upward."""

    solver: str = "Newton"
    iterations: int = 100
    ls_iterations: int = 50
    gravity: Vec3 = (0.0, 0.0, -9.81)
    impratio: float = 5.0
    """Ratio of frictional to normal constraint impedance. Raising it above 1 stops
    grasped objects creeping under sustained tangential load, which otherwise shows up as
    spurious slow slip in hold tests."""

    support: str = "plane"
    """``plane`` (an infinite table), ``none`` (free space), or ``pedestal``."""

    support_friction: float = 1.0
    add_lights: bool = True
    add_cameras: bool = True
    weld_solref: tuple[float, float] = (0.02, 1.0)
    """Wrist weld compliance. Stiffer than contact so the hand tracks its command, soft
    enough that grasp forces are transmitted rather than absorbed by an ideal constraint."""

    visual_quality: bool = True

    def to_dict(self) -> dict[str, Any]:
        return {
            "timestep": self.timestep, "integrator": self.integrator, "cone": self.cone,
            "solver": self.solver, "iterations": self.iterations,
            "ls_iterations": self.ls_iterations, "gravity": list(self.gravity),
            "impratio": self.impratio, "support": self.support,
            "support_friction": self.support_friction, "weld_solref": list(self.weld_solref),
        }


@dataclass
class SceneBuild:
    """A compiled-ready scene plus the names needed to drive and read it."""

    model: MjcfModel
    hand: HandSpec
    obj: ObjectSpec
    index: NameIndex
    options: SceneOptions
    initial_wrist: Pose
    plan: ApproachPlan | None = None
    meta: dict[str, Any] = field(default_factory=dict)

    def compile(self):  # noqa: ANN201 - mujoco.MjModel
        return self.model.compile()

    @property
    def hand_index(self) -> NameIndex:
        """Hand roles, un-namespaced (``index`` holds them under ``hand.``)."""
        return self.meta["hand_index"]

    @property
    def object_index(self) -> NameIndex:
        return self.meta["object_index"]


# ---------------------------------------------------------------------------------------
# approach geometry
# ---------------------------------------------------------------------------------------

def approach_pose(
    obj: ObjectSpec,
    hand: HandSpec,
    *,
    direction: str = "top",
    standoff: float = 0.0,
    azimuth: float = 0.0,
    object_pose: Pose | None = None,
    support_z: float = 0.0,
) -> Pose:
    """Palm pose for an approach; a thin wrapper over :func:`~morpholab.sim.approach.plan_approach`.

    Kept because callers that only want a pose should not have to unpack a plan.
    """
    return plan_approach(
        hand, obj, direction=direction, object_pose=object_pose,
        support_z=support_z, azimuth=azimuth, standoff=standoff,
    ).pose


# ---------------------------------------------------------------------------------------
# assembly
# ---------------------------------------------------------------------------------------

def build_scene(
    hand: HandSpec,
    obj: ObjectSpec,
    *,
    options: SceneOptions | None = None,
    wrist_pose: Pose | None = None,
    object_pose: Pose | None = None,
    approach: str = "top",
    standoff: float = 0.0,
) -> SceneBuild:
    """Assemble a hand, an object and a world into one model."""
    options = options or SceneOptions()
    object_pose = object_pose or Pose(pos=(0.0, 0.0, obj.rest_height), quat=obj.spawn_quat)
    plan = plan_approach(
        hand, obj, direction=approach, object_pose=object_pose,
        support_z=0.0 if options.support != "none" else -1e3, standoff=standoff,
    )
    wrist_pose = wrist_pose or plan.pose

    root = Element("mujoco", model=f"scene_{hand.name}_{obj.name}")
    _add_options(root, options)
    _add_world_assets(root, options)

    scene = MjcfModel(root, name=f"scene_{hand.name}_{obj.name}")
    index = NameIndex()

    world = root.section("worldbody")
    if options.add_lights:
        world.add("light", name="key_light", pos=[0.25, -0.35, 0.9], dir=[-0.25, 0.35, -0.9],
                  diffuse=[0.65, 0.65, 0.68], specular=[0.15, 0.15, 0.15], castshadow=True)
        world.add("light", name="fill_light", pos=[-0.4, 0.3, 0.7], dir=[0.4, -0.3, -0.7],
                  diffuse=[0.30, 0.30, 0.34], castshadow=False)
    if options.support == "plane":
        world.add("geom", name="table", type="plane", size=[0.6, 0.6, 0.02],
                  pos=[0, 0, 0], material="mat_table",
                  friction=[options.support_friction, 0.005, 0.0001],
                  contype=1, conaffinity=1, condim=3)
        index.geoms["table"] = "table"
    elif options.support == "pedestal":
        world.add("geom", name="table", type="cylinder", size=[0.08, 0.01], pos=[0, 0, -0.01],
                  material="mat_table", friction=[options.support_friction, 0.005, 0.0001])
        index.geoms["table"] = "table"

    # -- wrist target: a mocap body the controller commands ---------------------------
    mocap = world.add(
        "body", name="wrist_target", mocap=True,
        pos=list(wrist_pose.pos), quat=quat_to_str(wrist_pose.quat),
    )
    mocap.add("site", name="wrist_target_site", size=[0.006], group=4, rgba=[0.9, 0.3, 0.2, 0.5])
    index.bodies["wrist_target"] = "wrist_target"
    index.sites["wrist_target"] = "wrist_target_site"

    # -- hand -------------------------------------------------------------------------
    hand_build = build_hand(
        hand, root_pos=wrist_pose.pos, root_quat=wrist_pose.quat,
        add_freejoint=True, prefix=HAND_PREFIX,
    )
    scene.merge(hand_build.model, prefix="hand.")

    # -- object -----------------------------------------------------------------------
    object_build = build_object(obj, pose=object_pose, prefix=OBJ_PREFIX)
    scene.merge(object_build.model, prefix="object.")

    # -- weld: mocap drives the palm --------------------------------------------------
    root.section("equality").add(
        "weld",
        name="wrist_weld",
        body1="wrist_target",
        body2=f"{HAND_PREFIX}palm",
        solref=list(options.weld_solref),
        solimp=[0.95, 0.99, 0.001, 0.5, 2],
    )
    index.groups["equality"] = ["wrist_weld"]

    scene.index.merge(index)
    scene.index.groups["hand_geoms"] = (
        hand_build.index.group("pads")
        + hand_build.index.group("links")
        + hand_build.index.group("palm_geoms")
    )
    scene.index.groups["object_geoms"] = object_build.index.group("geoms")
    scene.index.groups["pads"] = hand_build.index.group("pads")
    scene.index.bodies["palm"] = f"{HAND_PREFIX}palm"
    scene.index.bodies["object"] = f"{OBJ_PREFIX}root"
    scene.index.sites["palm_center"] = hand_build.index.sites["palm_center"]

    if options.add_cameras:
        _add_cameras(world, obj)

    scene.meta.update(
        {
            "hand": hand.name,
            "morphology_id": hand.morphology_id,
            "object": obj.name,
            "object_id": obj.object_id,
            "options": options.to_dict(),
            "hand_index": hand_build.index,
            "object_index": object_build.index,
            "wrist_pose": wrist_pose.to_dict(),
            "object_pose": object_pose.to_dict(),
            "approach": approach,
            "approach_plan": plan.to_dict(),
            "preshape": plan.preshape,
            "reachable": plan.reachable,
        }
    )

    _check_admissible(hand, obj, scene)

    return SceneBuild(
        model=scene, hand=hand, obj=obj, index=scene.index, options=options,
        initial_wrist=wrist_pose, plan=plan, meta=scene.meta,
    )


def _check_admissible(hand: HandSpec, obj: ObjectSpec, scene: MjcfModel) -> None:
    """Record whether the hand could physically span the object.

    An object wider than the hand's aperture makes the trial *impossible*, not *failed*.
    Keeping the distinction stops hopeless pairs from dragging down a morphology's success
    rate for a reason that has nothing to do with its design (``docs/02-MATH.md`` §1.3).
    """
    aperture = hand.max_aperture
    admissible = obj.width < aperture * 0.9 and bool(scene.meta.get("reachable", True))
    scene.meta["admissible"] = bool(admissible)
    scene.meta["aperture"] = float(aperture)
    scene.meta["object_width"] = float(obj.width)
    if not admissible:
        log.debug(
            "object %s (width %.3f m) exceeds the aperture of hand %s (%.3f m)",
            obj.name, obj.width, hand.name, aperture,
        )


def _add_options(root: Element, options: SceneOptions) -> None:
    root.add("compiler", angle="radian", autolimits=True, balanceinertia=True)
    root.add(
        "option",
        timestep=options.timestep,
        integrator=options.integrator,
        cone=options.cone,
        solver=options.solver,
        iterations=options.iterations,
        ls_iterations=options.ls_iterations,
        gravity=list(options.gravity),
        impratio=options.impratio,
    )
    root.add("size", memory="64M")

    # A soft guard: contact parameters that are unstable at this timestep are the single
    # most common cause of silently wrong contact forces (docs/02-MATH.md §11).
    for name, mat in MATERIALS.items():
        ok, msg = mat.solver.check_stable(options.timestep)
        if not ok:
            log.warning("material %s is unstable at timestep %g: %s", name, options.timestep, msg)


def _add_world_assets(root: Element, options: SceneOptions) -> None:
    asset = root.section("asset")
    asset.add("texture", type="skybox", builtin="gradient", rgb1=[0.20, 0.24, 0.30],
              rgb2=[0.04, 0.05, 0.07], width=512, height=1024)
    asset.add("texture", type="2d", name="tex_table", builtin="checker", mark="edge",
              rgb1=[0.30, 0.31, 0.33], rgb2=[0.26, 0.27, 0.29], markrgb=[0.42, 0.43, 0.45],
              width=300, height=300)
    asset.add("material", name="mat_table", texture="tex_table", texuniform=True,
              texrepeat=[6, 6], reflectance=0.03, specular=0.1)

    if options.visual_quality:
        visual = root.add("visual")
        visual.add("global", azimuth=135, elevation=-22, offwidth=1600, offheight=1200)
        visual.add("quality", shadowsize=4096, offsamples=4)
        visual.add("map", force=0.008, zfar=8)
        visual.add("scale", contactwidth=0.012, contactheight=0.006, forcewidth=0.006,
                   framelength=0.05, framewidth=0.004, com=0.012)
        visual.add("rgba", contactpoint=[1.0, 0.55, 0.1, 0.7], contactforce=[0.95, 0.25, 0.2, 0.8],
                   com=[0.2, 0.8, 0.9, 0.7], force=[0.3, 0.85, 0.4, 0.75])


def _add_cameras(world: Element, obj: ObjectSpec) -> None:
    """Fixed cameras framed on the grasp, so every figure in a study is composed alike."""
    height = float(obj.aabb[1][2]) * 0.5 + 0.05
    world.add("camera", name="grasp_front", pos=[0.0, -0.34, height + 0.16],
              xyaxes=[1, 0, 0, 0, 0.55, 0.84])
    world.add("camera", name="grasp_side", pos=[0.34, 0.0, height + 0.16],
              xyaxes=[0, 1, 0, -0.55, 0, 0.84])
    world.add("camera", name="grasp_top", pos=[0.0, 0.0, height + 0.42],
              xyaxes=[1, 0, 0, 0, 1, 0])
    world.add("camera", name="grasp_close", pos=[0.16, -0.20, height + 0.11],
              xyaxes=[0.78, 0.62, 0, -0.30, 0.38, 0.87])
