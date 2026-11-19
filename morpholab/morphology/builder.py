"""``HandSpec`` -> MJCF.

The only module in the platform that knows how a hand becomes XML.  Its central design
decision is the mount frame (``docs/02-MATH.md`` §2.1): each digit gets **one** rotation,
computed once, after which every phalanx is expressed in trivial local coordinates::

    <body pos="L_prev 0 0">
      <joint axis="0 -1 0" range="..."/>
      <geom type="capsule" fromto="0 0 0  L 0 0" size="r"/>
      <geom type="box" pos="L/2 0 r" .../>          <!-- pad, palm-facing -->
    </body>

There is no per-part trigonometry anywhere below the mount, so the class of error that
made the original generator produce fingers with radial hinge axes
(``docs/00-REVIEW.md`` §3.1) is structurally impossible rather than merely fixed.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from ..core.logging import get_logger
from ..core.types import MATERIALS, MaterialSpec
from ..mjcf.element import Element
from ..mjcf.model import MjcfModel, NameIndex
from ..mjcf.transforms import mount_position, mount_quat, quat_to_str, vec_to_str
from .spec import ActuationMode, DigitSpec, HandSpec

log = get_logger(__name__)

__all__ = ["build_hand", "HandBuild", "hand_body_names"]

PAD_GROUP = 3
"""Render group for pads, so visualization can toggle contact surfaces independently."""

_PALM_SECTORS = 8
"""Facets used to approximate a curved (cupped) palm out of convex pieces."""


@dataclass(frozen=True)
class HandBuild:
    """A generated hand: its MJCF model, name index, and derived quantities."""

    model: MjcfModel
    spec: HandSpec
    index: NameIndex

    @property
    def name(self) -> str:
        return self.spec.name

    def compile(self):  # noqa: ANN201 - returns mujoco.MjModel, imported lazily
        return self.model.compile()


def build_hand(
    spec: HandSpec,
    *,
    root_pos: tuple[float, float, float] = (0.0, 0.0, 0.0),
    root_quat: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0),
    add_freejoint: bool = False,
    standalone: bool = False,
    prefix: str = "",
) -> HandBuild:
    """Generate the MJCF for ``spec``.

    Parameters
    ----------
    root_pos, root_quat:
        Pose of the palm body in its parent frame.  Scenes normally leave this at the
        origin and drive the hand through a mocap weld instead.
    add_freejoint:
        Give the palm a free joint.  Required for the mocap-welded wrist used by grasp
        tasks; omitted when the hand is rigidly mounted (e.g. on an arm).
    standalone:
        Emit a self-contained model with compiler options, a floor and a light, so the
        hand can be viewed or rendered on its own.
    prefix:
        Namespace for every generated name, so two hands can coexist in one scene (used by
        side-by-side morphology comparison renders).

    Returns
    -------
    HandBuild
    """
    p = prefix
    root = Element("mujoco", model=f"{p}{spec.name}")
    index = NameIndex()

    if standalone:
        _add_standalone_preamble(root)

    _add_defaults(root, spec, prefix=p)
    _add_materials(root, spec, prefix=p)

    world = root.section("worldbody")
    palm = world.add(
        "body",
        name=f"{p}palm",
        pos=list(root_pos),
        quat=list(root_quat),
    )
    index.bodies["palm"] = f"{p}palm"
    if add_freejoint:
        palm.add("freejoint", name=f"{p}wrist_free")
        index.joints["wrist_free"] = f"{p}wrist_free"

    _build_palm(palm, spec, index, prefix=p)

    excludes: list[tuple[str, str]] = []
    for digit in spec.digits:
        _build_digit(palm, spec, digit, index, excludes, prefix=p)

    _add_actuation(root, spec, index, prefix=p)

    if excludes:
        contact = root.section("contact")
        for a, b in excludes:
            contact.add("exclude", body1=a, body2=b)

    model = MjcfModel(
        root,
        name=f"{p}{spec.name}",
        index=index,
        meta={
            "morphology_id": spec.morphology_id,
            "spec": spec.to_dict(),
            "n_digits": spec.n_digits,
            "n_joints": spec.n_joints,
            "n_actuators": spec.n_actuators,
        },
    )
    log.debug("built hand %s: %s", spec.name, model.summary())
    return HandBuild(model=model, spec=spec, index=index)


# ---------------------------------------------------------------------------------------
# palm
# ---------------------------------------------------------------------------------------

def _build_palm(palm: Element, spec: HandSpec, index: NameIndex, *, prefix: str) -> None:
    """Palm core, optional pad, and the reference site at the palm centre."""
    p = prefix
    ps = spec.palm
    half_t = ps.thickness / 2.0

    palm.add(
        "geom",
        name=f"{p}palm_core",
        type="cylinder",
        size=[ps.radius, half_t],
        pos=[0.0, 0.0, 0.0],
        class_=f"{p}hand_structure",
        density=ps.density,
    )
    index.geoms["palm_core"] = f"{p}palm_core"
    palm_geoms = [f"{p}palm_core"]

    # The palm frame origin sits at the centre of the palm disc; the grasping surface is
    # at +z. Everything downstream (contact metrics, object pose errors) is expressed
    # relative to this site (docs/02-MATH.md §5.1).
    palm.add("site", name=f"{p}palm_center", pos=[0.0, 0.0, half_t], size=[0.004], group=4)
    index.sites["palm_center"] = f"{p}palm_center"
    palm.add("site", name=f"{p}palm_origin", pos=[0.0, 0.0, 0.0], size=[0.004], group=4)
    index.sites["palm_origin"] = f"{p}palm_origin"

    if ps.has_pad:
        pad_h = ps.pad_thickness / 2.0
        if ps.curvature <= 1e-6:
            palm.add(
                "geom",
                name=f"{p}palm_pad",
                type="cylinder",
                size=[ps.radius * 0.96, pad_h],
                pos=[0.0, 0.0, half_t + pad_h * 0.5],
                class_=f"{p}hand_pad",
                material=f"mat_{p}{ps.pad_material}",
                density=200.0,
            )
            palm_geoms.append(f"{p}palm_pad")
            index.geoms["palm_pad"] = f"{p}palm_pad"
        else:
            palm_geoms += _build_curved_palm_pad(palm, spec, index, prefix=p)

    index.groups["palm_geoms"] = palm_geoms


def _build_curved_palm_pad(
    palm: Element, spec: HandSpec, index: NameIndex, *, prefix: str
) -> list[str]:
    """Approximate a cupped palm with a flat centre disc and tilted rim facets.

    MuJoCo geoms are convex, so a concave surface must be assembled from convex pieces.
    Faceting the rim is the cheapest construction that produces a genuinely concave
    contact surface (rather than a merely thicker flat one), which is what the "does palm
    curvature matter" question needs (``docs/00-REVIEW.md`` A9).
    """
    p = prefix
    ps = spec.palm
    half_t = ps.thickness / 2.0
    pad_h = ps.pad_thickness / 2.0
    tilt = float(min(max(ps.curvature, 0.0), 1.0)) * math.radians(35.0)

    names: list[str] = []
    inner = ps.radius * 0.45
    palm.add(
        "geom",
        name=f"{p}palm_pad_center",
        type="cylinder",
        size=[inner, pad_h],
        pos=[0.0, 0.0, half_t + pad_h * 0.5],
        class_=f"{p}hand_pad",
        material=f"mat_{p}{ps.pad_material}",
        density=200.0,
    )
    names.append(f"{p}palm_pad_center")
    index.geoms["palm_pad"] = f"{p}palm_pad_center"

    ring_w = (ps.radius * 0.96 - inner) / 2.0
    ring_r = inner + ring_w
    for k in range(_PALM_SECTORS):
        theta = 2.0 * math.pi * k / _PALM_SECTORS
        # Facet frame: radial +x, tangential +y, tilted about the tangential axis so the
        # outer edge rises toward the fingers.
        quat = mount_quat(theta, elevation=-tilt, roll=0.0)
        pos = mount_position(theta, ring_r, half_t + pad_h + ring_w * math.sin(tilt) * 0.5)
        half_arc = math.pi / _PALM_SECTORS
        name = f"{p}palm_pad_{k}"
        palm.add(
            "geom",
            name=name,
            type="box",
            size=[ring_w, ring_r * math.tan(half_arc) * 0.95, pad_h],
            pos=list(pos),
            quat=quat_to_str(quat),
            class_=f"{p}hand_pad",
            material=f"mat_{p}{ps.pad_material}",
            density=200.0,
        )
        names.append(name)
    return names


# ---------------------------------------------------------------------------------------
# digits
# ---------------------------------------------------------------------------------------

def _build_digit(
    palm: Element,
    spec: HandSpec,
    digit: DigitSpec,
    index: NameIndex,
    excludes: list[tuple[str, str]],
    *,
    prefix: str,
) -> None:
    """Build one digit as a serial chain inside its mount frame."""
    p = prefix
    d = digit.name
    half_t = spec.palm.thickness / 2.0
    mount_r = spec.palm.radius * digit.mount.radius_frac

    base_name = f"{p}{d}_base"
    base = palm.add(
        "body",
        name=base_name,
        pos=list(mount_position(digit.mount.azimuth, mount_r, half_t * 0.5)),
        quat=quat_to_str(mount_quat(digit.mount.azimuth, digit.mount.elevation, digit.mount.roll)),
    )
    index.bodies[f"{d}.base"] = base_name
    # The palm and the proximal link are rigidly adjacent across the base joint, and a
    # capsule's hemispherical cap necessarily penetrates the palm it emerges from. Only
    # these two pairs are excluded: a *distal* phalanx folding onto the palm is a real
    # contact and is exactly what a power grasp uses.
    excludes.append((f"{p}palm", base_name))
    excludes.append((f"{p}palm", f"{p}{d}_p0"))

    joint_names: list[str] = []
    pad_names: list[str] = []
    link_names: list[str] = []
    body_names: list[str] = [base_name]

    # -- optional abduction, at the base, about the palm normal ---------------------
    if digit.abduction is not None:
        abd = digit.abduction
        name = f"{p}{d}_abd"
        base.add(
            "joint",
            name=name,
            type="hinge",
            axis=[0.0, 0.0, 1.0],
            range=list(abd.range),
            damping=abd.damping,
            armature=abd.armature,
            stiffness=abd.stiffness,
            springref=abd.springref,
            frictionloss=abd.frictionloss or None,
            class_=f"{p}hand_abduction",
        )
        index.joints[f"{d}.abduction"] = name
        index.groups.setdefault("abduction_joints", []).append(name)
        # A base link so the abduction joint has something to move; without a geom here
        # MuJoCo would give the body zero inertia and the joint would be ill-conditioned.
        knuckle = f"{p}{d}_knuckle"
        base.add(
            "geom",
            name=knuckle,
            type="sphere",
            size=[digit.base_radius * 0.9],
            pos=[0.0, 0.0, 0.0],
            class_=f"{p}hand_structure",
            density=digit.density,
        )
        link_names.append(knuckle)

    # -- phalanx chain --------------------------------------------------------------
    parent = base
    phalanges = digit.phalanges
    flex_joints = digit.flexion_joints
    tendon_mode = spec.actuation.is_tendon

    for i, (ph, jnt) in enumerate(zip(phalanges, flex_joints)):
        body_name = f"{p}{d}_p{i}"
        offset = [phalanges[i - 1].length, 0.0, 0.0] if i > 0 else [0.0, 0.0, 0.0]
        body = parent.add("body", name=body_name, pos=offset)
        body_names.append(body_name)

        # Extension spring: in tendon modes every joint needs one, since the tendon can
        # only pull (docs/02-MATH.md §6.2).
        stiffness = jnt.stiffness
        if tendon_mode and stiffness <= 0.0:
            stiffness = spec.actuation.extension_stiffness

        joint_name = f"{p}{d}_p{i}_flex"
        body.add(
            "joint",
            name=joint_name,
            type="hinge",
            axis=list(jnt.axis),
            range=list(jnt.range),
            damping=jnt.damping,
            armature=jnt.armature,
            stiffness=stiffness or None,
            springref=jnt.springref or None,
            frictionloss=jnt.frictionloss or None,
            class_=f"{p}hand_flexion",
        )
        joint_names.append(joint_name)
        index.joints[f"{d}.p{i}.flex"] = joint_name
        index.groups.setdefault("flexion_joints", []).append(joint_name)

        link_name = f"{p}{d}_p{i}_link"
        body.add(
            "geom",
            name=link_name,
            type="capsule",
            fromto=[0.0, 0.0, 0.0, ph.length, 0.0, 0.0],
            size=[ph.radius],
            class_=f"{p}hand_structure",
            density=ph.density,
        )
        link_names.append(link_name)
        index.geoms[f"{d}.p{i}.link"] = link_name

        # Pad on the palm-facing (+z) side: the mount frame guarantees +z is where the
        # digit curls, for every digit, at every azimuth.
        pad_len = ph.length * ph.pad_coverage
        pad_name = f"{p}{d}_p{i}_pad"
        body.add(
            "geom",
            name=pad_name,
            type="box",
            size=[pad_len / 2.0, ph.radius * 0.8, ph.pad_thickness / 2.0],
            pos=[ph.length / 2.0, 0.0, ph.radius * 0.92],
            class_=f"{p}hand_pad",
            material=f"mat_{p}{ph.pad_material}",
            density=200.0,
        )
        pad_names.append(pad_name)
        index.geoms[f"{d}.p{i}.pad"] = pad_name

        if i == len(phalanges) - 1:
            tip = f"{p}{d}_tip"
            body.add("site", name=tip, pos=[ph.length, 0.0, 0.0], size=[0.004], group=4)
            index.sites[f"{d}.tip"] = tip
            index.groups.setdefault("fingertip_sites", []).append(tip)

        parent = body

    # Adjacent links are excluded explicitly rather than relying on MuJoCo's parent-child
    # filter, which does not apply once a body carries a joint chain of its own.
    for a, b in zip(body_names, body_names[1:]):
        excludes.append((a, b))

    index.groups[f"{d}.joints"] = joint_names
    index.groups[f"{d}.pads"] = pad_names
    index.groups[f"{d}.links"] = link_names
    index.groups[f"{d}.bodies"] = body_names
    index.groups.setdefault("pads", []).extend(pad_names)
    index.groups.setdefault("links", []).extend(link_names)
    index.groups.setdefault("digits", []).append(d)


# ---------------------------------------------------------------------------------------
# actuation
# ---------------------------------------------------------------------------------------

def _add_actuation(root: Element, spec: HandSpec, index: NameIndex, *, prefix: str) -> None:
    """Emit actuators (and tendons) for the configured actuation mode."""
    p = prefix
    act = spec.actuation
    actuators = root.section("actuator")
    mode = act.mode

    if mode == ActuationMode.PER_JOINT:
        for digit in spec.digits:
            for i, jnt in enumerate(digit.flexion_joints):
                _position_actuator(
                    actuators, index, f"{p}{digit.name}_p{i}_flex", act, jnt.range,
                    role=f"{digit.name}.p{i}.flex",
                )
    elif mode == ActuationMode.TENDON_COUPLED:
        for digit in spec.digits:
            joints = index.groups[f"{digit.name}.joints"]
            _tendon_for_joints(
                root, index, act,
                name=f"{p}{digit.name}_tendon",
                joint_names=joints,
                joint_specs=list(digit.flexion_joints),
                coeffs=list(act.coefficients_for(len(joints))),
                role=digit.name,
            )
    elif mode == ActuationMode.SYNERGY:
        joint_names: list[str] = []
        joint_specs: list = []
        coeffs: list[float] = []
        for digit in spec.digits:
            joints = index.groups[f"{digit.name}.joints"]
            joint_names += joints
            joint_specs += list(digit.flexion_joints)
            # The moment-arm series restarts per digit. Letting it decay across the whole
            # hand would make the last digit's arms two orders of magnitude smaller than
            # the first's, so a single tendon would close one finger and leave the rest
            # standing -- an artifact of the indexing, not of underactuation.
            coeffs += list(act.coefficients_for(len(joints)))
        _tendon_for_joints(
            root, index, act, name=f"{p}hand_tendon", joint_names=joint_names,
            joint_specs=joint_specs, coeffs=coeffs, role="hand",
        )
    else:
        raise ValueError(f"unknown actuation mode {mode!r}; expected one of {ActuationMode.ALL}")

    # Abduction always gets a dedicated actuator: coupling it into the flexion tendon
    # would make it uncontrollable and would understate the cost of the feature whose
    # value the project is measuring (docs/00-REVIEW.md A6).
    for digit in spec.digits:
        if digit.abduction is not None:
            _position_actuator(
                actuators, index, f"{p}{digit.name}_abd", act, digit.abduction.range,
                role=f"{digit.name}.abduction",
            )


def _position_actuator(
    actuators: Element,
    index: NameIndex,
    joint_name: str,
    act: Any,
    ctrl_range: tuple[float, float],
    *,
    role: str,
) -> None:
    name = f"{joint_name}_act"
    actuators.add(
        "position",
        name=name,
        joint=joint_name,
        kp=act.kp,
        kv=act.kv,
        ctrlrange=list(ctrl_range),
        forcerange=[-act.force_limit, act.force_limit],
    )
    index.actuators[role] = name
    index.groups.setdefault("actuators", []).append(name)


def _tendon_for_joints(
    root: Element,
    index: NameIndex,
    act: Any,
    *,
    name: str,
    joint_names: list[str],
    joint_specs: list,
    coeffs: list[float],
    role: str,
) -> None:
    """A fixed tendon coupling ``joint_names``, driven by one position actuator.

    The tendon defines ``l = sum_i c_i q_i``; servoing ``l`` constrains one dimension of
    an ``n``-joint chain and leaves ``n-1`` free, so the joints distribute themselves
    according to their springs and whatever they are touching.  That is underactuation,
    and it is why one motor can produce an adaptive enclosing grasp
    (``docs/02-MATH.md`` §6.2).
    """
    tendon = root.section("tendon").add("fixed", name=name)
    for joint_name, coef in zip(joint_names, coeffs):
        tendon.add("joint", joint=joint_name, coef=coef)
    index.tendons[role] = name

    # Control range spans fully extended to fully flexed, in tendon-length units.
    lo = sum(c * j.range[0] for c, j in zip(coeffs, joint_specs))
    hi = sum(c * j.range[1] for c, j in zip(coeffs, joint_specs))
    # Gain conversion, not a fudge factor. A tendon actuator's gain is in N/m while a
    # joint's is in N*m/rad; the moment arm relates them by tau = c*F and dl = c*dq, so
    # kp_tendon = kp_joint / c^2. Using an arbitrary multiplier here (as an earlier draft
    # did) makes the hand's grip strength an artifact of the units rather than a property
    # of the design -- and grip strength is one of the things being measured.
    c0 = max(coeffs[0], 1e-4)
    act_name = f"{name}_act"
    root.section("actuator").add(
        "position",
        name=act_name,
        tendon=name,
        kp=act.kp / (c0 * c0),
        kv=act.kv / (c0 * c0),
        ctrlrange=[min(lo, hi), max(lo, hi)],
        forcerange=[-act.force_limit / c0, act.force_limit / c0],
    )
    index.actuators[role] = act_name
    index.groups.setdefault("actuators", []).append(act_name)
    index.groups.setdefault("tendons", []).append(name)


# ---------------------------------------------------------------------------------------
# defaults, materials, standalone scaffolding
# ---------------------------------------------------------------------------------------

def _add_defaults(root: Element, spec: HandSpec, *, prefix: str) -> None:
    """Default classes: contact parameters live in one place, not on nine geoms.

    Repeating ``solref``/``solimp`` per geom is what made those values unexaminable magic
    numbers in the original model (``docs/00-REVIEW.md`` §3.6).  Here they are named
    conditions that a sweep can vary.
    """
    p = prefix
    defaults = root.section("default")

    structure = spec.material(spec.palm.material)
    struct_class = defaults.add("default", class_=f"{p}hand_structure")
    struct_class.add(
        "geom",
        contype=1,
        conaffinity=1,
        condim=3,
        group=2,
        friction=list(structure.friction),
        solref=structure.solver.solref,
        solimp=structure.solver.solimp,
        material=f"mat_{p}structure",
    )

    pad_mat = spec.material(spec.palm.pad_material)
    pad_class = defaults.add("default", class_=f"{p}hand_pad")
    pad_class.add(
        "geom",
        contype=1,
        conaffinity=1,
        condim=4,  # torsional friction: without it every pinch grasp spins
        group=PAD_GROUP,
        friction=list(pad_mat.friction),
        solref=pad_mat.solver.solref,
        solimp=pad_mat.solver.solimp,
        margin=0.0005,
    )

    flex = defaults.add("default", class_=f"{p}hand_flexion")
    flex.add("joint", type="hinge", limited=True)
    abd = defaults.add("default", class_=f"{p}hand_abduction")
    abd.add("joint", type="hinge", limited=True)


def _add_materials(root: Element, spec: HandSpec, *, prefix: str = "") -> None:
    """Visual materials for every material name the hand references."""
    asset = root.section("asset")
    used = {spec.palm.material, spec.palm.pad_material}
    for digit in spec.digits:
        used.add(digit.pad_material)
    used.add("structure")
    for name in sorted(used):
        mat: MaterialSpec = spec.material(name)
        asset.add("material", name=f"mat_{prefix}{name}", rgba=list(mat.rgba), specular=0.3, shininess=0.25)


def _add_standalone_preamble(root: Element) -> None:
    """Compiler settings, a floor and a light, for viewing a hand on its own."""
    root.add("compiler", angle="radian", autolimits=True)
    root.add("option", timestep=0.002, integrator="implicitfast", cone="elliptic")
    visual = root.add("visual")
    visual.add("global", azimuth=140, elevation=-25, offwidth=1280, offheight=960)
    visual.add("rgba", haze=[0.15, 0.18, 0.22, 1.0])
    visual.add("quality", shadowsize=4096)

    asset = root.section("asset")
    asset.add(
        "texture", type="skybox", builtin="gradient",
        rgb1=[0.22, 0.26, 0.32], rgb2=[0.05, 0.06, 0.08], width=512, height=1024,
    )
    asset.add(
        "texture", type="2d", name="tex_ground", builtin="checker", mark="edge",
        rgb1=[0.20, 0.22, 0.25], rgb2=[0.24, 0.26, 0.29], markrgb=[0.4, 0.4, 0.4],
        width=300, height=300,
    )
    asset.add(
        "material", name="mat_ground", texture="tex_ground", texuniform=True,
        texrepeat=[4, 4], reflectance=0.05,
    )
    world = root.section("worldbody")
    world.add("light", pos=[0, 0, 1.2], dir=[0, 0, -1], directional=True, diffuse=[0.7, 0.7, 0.7])
    world.add("light", pos=[0.4, -0.4, 0.8], dir=[-0.4, 0.4, -0.8], diffuse=[0.35, 0.35, 0.4])
    world.add("geom", name="floor", type="plane", size=[1, 1, 0.05], pos=[0, 0, -0.35],
              material="mat_ground", contype=0, conaffinity=0)
    world.add("camera", name="front", pos=[0.0, -0.42, 0.18], xyaxes=[1, 0, 0, 0, 0.45, 0.9])
    world.add("camera", name="side", pos=[0.42, 0.0, 0.18], xyaxes=[0, 1, 0, -0.45, 0, 0.9])
    world.add("camera", name="top", pos=[0.0, 0.0, 0.45], xyaxes=[1, 0, 0, 0, 1, 0])


def hand_body_names(spec: HandSpec, prefix: str = "") -> list[str]:
    """Body names a hand will generate, without building it.

    Used by scene assembly to pre-register collision exclusions and by the name-collision
    check before a merge.
    """
    names = [f"{prefix}palm"]
    for digit in spec.digits:
        names.append(f"{prefix}{digit.name}_base")
        names += [f"{prefix}{digit.name}_p{i}" for i in range(digit.n_phalanges)]
    return names
