"""Tests for the MJCF authoring layer: element trees and transforms.

The transform tests are the important ones.  Every geometric defect in the original
project came from per-part trigonometry with no way to check it (``docs/00-REVIEW.md``
§3.1), so the frame convention is pinned here against hand-computed values.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from morpholab.mjcf.element import Element, MjcfError, element_from_xml, fmt_value
from morpholab.mjcf.model import MjcfModel, NameIndex
from morpholab.mjcf.transforms import (
    geodesic_angle,
    matrix_from_quat,
    mount_frame,
    mount_position,
    mount_quat,
    orthonormal_basis,
    quat_angle,
    quat_from_axis_angle,
    quat_from_matrix,
    quat_multiply,
    quat_rotate,
    rot_x,
    rot_y,
    rot_z,
    wrap_2pi,
    wrap_angle,
)


# --------------------------------------------------------------------------------------
# element tree
# --------------------------------------------------------------------------------------

def test_attribute_formatting_follows_mjcf_conventions():
    assert fmt_value(True) == "true" and fmt_value(False) == "false"
    assert fmt_value(0.1) == "0.1"
    assert fmt_value([1, 2.5, 3]) == "1 2.5 3"
    assert fmt_value(np.array([1.0, 2.0])) == "1 2"
    with pytest.raises(MjcfError, match="non-finite"):
        fmt_value(float("nan"))


def test_none_attributes_are_dropped_so_optional_args_need_no_branching():
    el = Element("geom", name="g", size=0.1, stiffness=None)
    assert "stiffness" not in el.attrib and el.get("name") == "g"


def test_trailing_underscore_escapes_python_keywords():
    el = Element("geom", class_="hand_pad")
    assert el.attrib["class"] == "hand_pad"


def test_add_returns_child_and_append_returns_self():
    body = Element("body", name="b")
    child = body.add("geom", name="g")
    assert child.tag == "geom"
    assert body.append(Element("site", name="s")) is body
    assert len(body) == 2


def test_section_is_get_or_create():
    root = Element("mujoco")
    a = root.section("asset")
    assert root.section("asset") is a and len(root.children) == 1


def test_find_and_walk_traverse_the_whole_subtree():
    root = Element("mujoco")
    world = root.section("worldbody")
    b1 = world.add("body", name="b1")
    b1.add("geom", name="g1")
    b1.add("body", name="b2").add("geom", name="g2")
    assert [g.name for g in root.find_all("geom")] == ["g1", "g2"]
    assert root.find_named("body", "b2") is not None
    assert root.find_named("body", "nope") is None
    assert len(list(root.walk())) == 6


def test_validate_flags_duplicates_and_empty_bodies():
    root = Element("mujoco")
    world = root.section("worldbody")
    world.add("body", name="a").add("geom", name="g")
    world.add("body", name="a").add("geom", name="g")
    world.add("body", name="empty")
    problems = " ".join(root.validate())
    assert "duplicate body name 'a'" in problems
    assert "duplicate geom name 'g'" in problems
    assert "empty has no geoms" in problems


def test_bodies_and_geoms_may_share_a_name_across_namespaces():
    root = Element("mujoco")
    root.section("worldbody").add("body", name="palm").add("geom", name="palm")
    assert root.validate() == []


def test_merge_combines_sections_and_detects_collisions():
    a = Element("mujoco")
    a.section("worldbody").add("body", name="hand").add("geom", name="palm")
    a.section("asset").add("material", name="m1")

    b = Element("mujoco")
    b.section("worldbody").add("body", name="object").add("geom", name="obj")
    b.section("asset").add("material", name="m2")

    a.merge(b)
    assert [n.name for n in a.section("worldbody")] == ["hand", "object"]
    assert [n.name for n in a.section("asset")] == ["m1", "m2"]

    clash = Element("mujoco")
    clash.section("worldbody").add("body", name="hand").add("geom", name="x")
    with pytest.raises(MjcfError, match="duplicate element name"):
        a.merge(clash)


def test_xml_round_trips_through_the_parser():
    root = Element("mujoco", model="m")
    root.section("worldbody").add("body", name="b", pos=(1, 2, 3)).add("geom", type="sphere", size=0.1)
    parsed = element_from_xml(root.to_xml())
    assert parsed.find_named("body", "b").get("pos") == "1 2 3"


# --------------------------------------------------------------------------------------
# transforms: the frame convention
# --------------------------------------------------------------------------------------

def test_elementary_rotations_match_hand_computed_values():
    assert np.allclose(rot_z(math.pi / 2) @ [1, 0, 0], [0, 1, 0], atol=1e-12)
    assert np.allclose(rot_y(math.pi / 2) @ [1, 0, 0], [0, 0, -1], atol=1e-12)
    assert np.allclose(rot_x(math.pi / 2) @ [0, 1, 0], [0, 0, 1], atol=1e-12)


def test_mount_frame_at_zero_azimuth_is_the_identity():
    assert np.allclose(mount_frame(0.0), np.eye(3), atol=1e-12)


@pytest.mark.parametrize("azimuth_deg", [0, 30, 90, 120, 210, 270, 359])
def test_mount_frame_x_axis_points_radially_outward(azimuth_deg):
    """The defining property: local +x is the direction the digit extends."""
    a = math.radians(azimuth_deg)
    R = mount_frame(a)
    assert np.allclose(R[:, 0], [math.cos(a), math.sin(a), 0.0], atol=1e-12)


@pytest.mark.parametrize("azimuth_deg", [0, 45, 120, 250])
def test_mount_frame_z_axis_is_the_palm_normal(azimuth_deg):
    """Flexion must curl toward the palm normal for *every* digit -- the property the
    original generator lost for two of its three fingers."""
    R = mount_frame(math.radians(azimuth_deg))
    assert np.allclose(R[:, 2], [0.0, 0.0, 1.0], atol=1e-12)


def test_mount_frame_is_right_handed_and_orthonormal():
    R = mount_frame(1.1, 0.3, -0.4)
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-12)
    assert np.isclose(np.linalg.det(R), 1.0, atol=1e-12)


def test_positive_flexion_curls_a_digit_toward_the_palm_normal():
    """A rotation of +90 degrees about the local flexion axis (0,-1,0) must map the digit
    direction +x onto the palm normal +z."""
    flexion_axis = np.array([0.0, -1.0, 0.0])
    q = quat_from_axis_angle(flexion_axis, math.pi / 2)
    assert np.allclose(quat_rotate(q, [1.0, 0.0, 0.0]), [0.0, 0.0, 1.0], atol=1e-12)


def test_elevation_tilts_out_of_the_palm_plane_toward_the_normal():
    R = mount_frame(0.0, elevation=math.radians(30.0))
    x_axis = R[:, 0]
    assert x_axis[2] > 0.0  # tilted up, toward the fingers
    assert np.isclose(math.degrees(math.asin(x_axis[2])), 30.0, atol=1e-9)


def test_mount_position_lies_on_the_palm_circle():
    p = mount_position(math.radians(120.0), 0.05, z=0.01)
    assert np.isclose(math.hypot(p[0], p[1]), 0.05)
    assert np.isclose(p[2], 0.01)


# --------------------------------------------------------------------------------------
# transforms: quaternions
# --------------------------------------------------------------------------------------

@pytest.mark.parametrize("seed", range(8))
def test_quaternion_matrix_round_trip(seed):
    rng = np.random.default_rng(seed)
    R = np.linalg.qr(rng.normal(size=(3, 3)))[0]
    if np.linalg.det(R) < 0:
        R[:, 0] *= -1
    assert np.allclose(matrix_from_quat(quat_from_matrix(R)), R, atol=1e-10)


def test_quaternion_from_matrix_is_accurate_at_180_degrees():
    """The naive ``w = sqrt(1 + tr)/2`` formula loses all precision here, and a 180-degree
    rotation is exactly what perfectly opposed digits produce."""
    R = rot_z(math.pi)
    q = quat_from_matrix(R)
    assert np.allclose(matrix_from_quat(q), R, atol=1e-12)
    assert np.isclose(quat_angle(q), math.pi, atol=1e-9)


def test_quaternion_is_returned_in_the_canonical_hemisphere():
    q = quat_from_matrix(rot_x(math.radians(200.0)))
    assert q[0] >= 0.0


def test_quaternion_multiplication_composes_in_the_stated_order():
    a = quat_from_axis_angle([0, 0, 1], math.pi / 2)
    b = quat_from_axis_angle([1, 0, 0], math.pi / 2)
    composed = quat_multiply(a, b)
    assert np.allclose(matrix_from_quat(composed), rot_z(math.pi / 2) @ rot_x(math.pi / 2), atol=1e-12)


def test_geodesic_angle_handles_the_quaternion_double_cover():
    """q and -q are the same orientation; forgetting this reports 360-degree errors for
    grasps that never moved (``docs/02-MATH.md`` §5.1)."""
    q = quat_from_axis_angle([0, 1, 0], 0.4)
    minus_q = tuple(-v for v in q)
    assert geodesic_angle(q, minus_q) < 1e-9
    assert np.isclose(geodesic_angle((1, 0, 0, 0), q), 0.4, atol=1e-9)


def test_orthonormal_basis_never_degenerates():
    for n in ([1, 0, 0], [0, 0, 1], [0, -1, 0], [0.3, -0.5, 0.81]):
        t1, t2, nn = orthonormal_basis(n)
        M = np.column_stack([t1, t2, nn])
        assert np.allclose(M @ M.T, np.eye(3), atol=1e-12)
        assert np.isclose(np.linalg.det(M), 1.0, atol=1e-12)


def test_angle_wrapping():
    assert np.isclose(wrap_angle(3 * math.pi), math.pi)
    assert np.isclose(wrap_angle(-3 * math.pi), math.pi)
    assert np.isclose(wrap_2pi(-math.pi / 2), 1.5 * math.pi)
    assert np.isclose(wrap_2pi(2.5 * math.pi), 0.5 * math.pi)


# --------------------------------------------------------------------------------------
# model wrapper
# --------------------------------------------------------------------------------------

def test_model_requires_a_mujoco_root():
    with pytest.raises(MjcfError, match="must be <mujoco>"):
        MjcfModel(Element("worldbody"))


def test_model_compiles_and_reports_a_summary():
    model = MjcfModel.empty("probe")
    world = model.worldbody
    world.add("light", pos=(0, 0, 2))
    body = world.add("body", name="b", pos=(0, 0, 0.3))
    body.add("freejoint")
    body.add("geom", name="g", type="sphere", size=0.05)
    compiled = model.compile()
    assert compiled.nbody == 2
    assert model.summary()["geoms"] == 1


def test_compile_error_includes_numbered_xml_context():
    model = MjcfModel.empty("bad")
    model.worldbody.add("body", name="b").add("geom", type="sphere", size=-1.0)
    with pytest.raises(Exception) as exc:
        model.compile()
    assert "rejected generated model 'bad'" in str(exc.value)


def test_strict_compile_refuses_a_structurally_invalid_tree():
    model = MjcfModel.empty("dup")
    model.worldbody.add("body", name="a").add("geom", name="g", type="sphere", size=0.1)
    model.worldbody.add("body", name="a").add("geom", name="g2", type="sphere", size=0.1)
    with pytest.raises(Exception, match="failed validation"):
        model.compile()


def test_name_index_merge_namespaces_roles():
    a = NameIndex(bodies={"palm": "palm"}, groups={"pads": ["p1"]})
    b = NameIndex(bodies={"root": "obj"}, groups={"geoms": ["g1"]})
    a.merge(b, prefix="object.")
    assert a.bodies == {"palm": "palm", "object.root": "obj"}
    assert a.group("object.geoms") == ["g1"]
    assert a.group("absent") == []
