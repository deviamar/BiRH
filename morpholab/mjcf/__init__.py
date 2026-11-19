"""Declarative MJCF authoring: element trees, transforms, compilation.

This layer knows nothing about hands, objects or grasping -- it is a general MJCF
construction kit that the morphology and object builders sit on top of.
"""

from __future__ import annotations

from .element import Element, MjcfError, element_from_xml, fmt_value
from .model import CompileError, MjcfModel, NameIndex, joint_dof_addresses, joint_qpos_addresses, resolve_ids
from .transforms import (
    IDENTITY_QUAT,
    angle_between,
    geodesic_angle,
    matrix_from_quat,
    mount_frame,
    mount_position,
    mount_quat,
    normalize,
    orthonormal_basis,
    quat_angle,
    quat_conjugate,
    quat_from_axis_angle,
    quat_from_euler,
    quat_from_matrix,
    quat_multiply,
    quat_rotate,
    quat_to_str,
    rot_x,
    rot_y,
    rot_z,
    vec_to_str,
    wrap_2pi,
    wrap_angle,
)

__all__ = [
    "Element", "MjcfError", "fmt_value", "element_from_xml",
    "MjcfModel", "NameIndex", "CompileError", "resolve_ids",
    "joint_qpos_addresses", "joint_dof_addresses",
    "IDENTITY_QUAT", "mount_frame", "mount_quat", "mount_position",
    "quat_from_matrix", "matrix_from_quat", "quat_from_axis_angle", "quat_from_euler",
    "quat_multiply", "quat_conjugate", "quat_rotate", "quat_angle", "geodesic_angle",
    "rot_x", "rot_y", "rot_z", "normalize", "orthonormal_basis", "angle_between",
    "vec_to_str", "quat_to_str", "wrap_angle", "wrap_2pi",
]
