"""Rotations, quaternions and frames.

Every geometric error in the original project traced back to per-part trigonometry
(``docs/00-REVIEW.md`` §3.1).  The fix is that *all* orientation maths happens here, is
unit-tested against known frames, and is used exactly once per digit -- to build a mount
frame -- after which every phalanx is expressed in trivial local coordinates.

Conventions
-----------
* Quaternions are MuJoCo order ``(w, x, y, z)``, unit norm, right-handed, active rotations.
* Rotation matrices map *local* vectors to *parent* vectors: ``v_parent = R @ v_local``.
* Angles are radians everywhere; degrees appear only in user-facing formatting.

See ``docs/02-MATH.md`` §2.
"""

from __future__ import annotations

import math

import numpy as np

from ..core.types import Array, Quat, Vec3

__all__ = [
    "IDENTITY_QUAT", "quat_from_matrix", "matrix_from_quat", "quat_from_axis_angle",
    "quat_multiply", "quat_conjugate", "quat_rotate", "quat_angle", "quat_from_euler",
    "rot_x", "rot_y", "rot_z", "mount_frame", "look_at_quat", "quat_to_str", "vec_to_str",
    "normalize", "angle_between", "geodesic_angle", "orthonormal_basis",
]

IDENTITY_QUAT: Quat = (1.0, 0.0, 0.0, 0.0)
_EPS = 1e-12


# ---------------------------------------------------------------------------------------
# elementary rotations
# ---------------------------------------------------------------------------------------

def rot_x(a: float) -> Array:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], float)


def rot_y(a: float) -> Array:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], float)


def rot_z(a: float) -> Array:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], float)


# ---------------------------------------------------------------------------------------
# quaternions
# ---------------------------------------------------------------------------------------

def quat_from_matrix(R: Array) -> Quat:
    """Unit quaternion ``(w, x, y, z)`` from a rotation matrix.

    Uses Shepperd's branch selection -- pick the largest of the four candidate
    denominators -- rather than the textbook ``w = sqrt(1 + tr(R))/2``, which loses all
    precision as ``tr(R) -> -1`` (a 180-degree rotation, which occurs for perfectly
    opposed digits and is therefore not a corner case here).
    """
    R = np.asarray(R, float)
    if R.shape != (3, 3):
        raise ValueError(f"expected a 3x3 rotation matrix, got shape {R.shape}")
    m00, m01, m02 = R[0]
    m10, m11, m12 = R[1]
    m20, m21, m22 = R[2]
    trace = m00 + m11 + m22

    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w, x, y, z = 0.25 * s, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w, x, y, z = (m21 - m12) / s, 0.25 * s, (m01 + m10) / s, (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w, x, y, z = (m02 - m20) / s, (m01 + m10) / s, 0.25 * s, (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w, x, y, z = (m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25 * s

    q = np.array([w, x, y, z], float)
    q /= max(np.linalg.norm(q), _EPS)
    if q[0] < 0:  # canonical hemisphere: w >= 0, so equal rotations have equal quaternions
        q = -q
    return tuple(float(v) for v in q)  # type: ignore[return-value]


def matrix_from_quat(q: Quat | Array) -> Array:
    """Rotation matrix from a ``(w, x, y, z)`` quaternion."""
    w, x, y, z = (float(v) for v in q)
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < _EPS:
        raise ValueError("zero-norm quaternion")
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ],
        float,
    )


def quat_from_axis_angle(axis: Vec3 | Array, angle: float) -> Quat:
    """Quaternion for a rotation of ``angle`` about ``axis`` (need not be normalized)."""
    a = np.asarray(axis, float)
    n = np.linalg.norm(a)
    if n < _EPS:
        return IDENTITY_QUAT
    a = a / n
    s = math.sin(angle / 2.0)
    return (math.cos(angle / 2.0), float(a[0] * s), float(a[1] * s), float(a[2] * s))


def quat_from_euler(roll: float, pitch: float, yaw: float) -> Quat:
    """Quaternion from intrinsic X-Y-Z Euler angles, i.e. ``Rz(yaw) Ry(pitch) Rx(roll)``.

    Provided for interoperability with hand-written models; the platform's own geometry
    uses :func:`mount_frame`, which states the axis convention explicitly instead of
    relying on an Euler ordering the reader has to guess (``docs/00-REVIEW.md`` §3.1).
    """
    return quat_from_matrix(rot_z(yaw) @ rot_y(pitch) @ rot_x(roll))


def quat_multiply(a: Quat | Array, b: Quat | Array) -> Quat:
    """Hamilton product ``a * b``: apply ``b`` first, then ``a``."""
    aw, ax, ay, az = (float(v) for v in a)
    bw, bx, by, bz = (float(v) for v in b)
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quat_conjugate(q: Quat | Array) -> Quat:
    w, x, y, z = (float(v) for v in q)
    return (w, -x, -y, -z)


def quat_rotate(q: Quat | Array, v: Vec3 | Array) -> Array:
    """Rotate vector ``v`` by quaternion ``q``."""
    return matrix_from_quat(q) @ np.asarray(v, float)


def quat_angle(q: Quat | Array) -> float:
    """Rotation magnitude of ``q`` in radians, in ``[0, pi]``."""
    w = abs(float(np.clip(np.asarray(q, float)[0] / max(np.linalg.norm(q), _EPS), -1.0, 1.0)))
    return 2.0 * math.acos(w)


def geodesic_angle(a: Quat | Array, b: Quat | Array) -> float:
    """Angle of the relative rotation between two orientations, in ``[0, pi]``.

    This is the rotational slip measure used by the disturbance battery
    (``docs/02-MATH.md`` §5.1).  The absolute value on ``w`` handles the double cover: ``q``
    and ``-q`` are the same orientation, and forgetting it reports 360-degree errors for
    grasps that never moved.
    """
    return quat_angle(quat_multiply(quat_conjugate(a), b))


# ---------------------------------------------------------------------------------------
# frames
# ---------------------------------------------------------------------------------------

def mount_frame(azimuth: float, elevation: float = 0.0, roll: float = 0.0) -> Array:
    """The rotation matrix of a digit mount frame (``docs/02-MATH.md`` §2.1).

    ``R = Rz(azimuth) @ Ry(-elevation) @ Rx(roll)``, whose columns are

    ==========  ==========================================================================
    ``x_hat``   radially outward from the palm axis -- the direction the digit extends
    ``y_hat``   tangential -- the flexion axis (joints use ``axis="0 -1 0"`` so that a
                positive joint angle curls the digit toward the palm normal)
    ``z_hat``   the palm normal at ``elevation = roll = 0`` -- where fingertips converge
    ==========  ==========================================================================

    Positive ``elevation`` tilts the digit *toward* the palm normal (out of the palm
    plane); ``roll`` spins the flexion plane about the digit's own axis.

    Every phalanx below the mount is then expressed with no trigonometry at all, which is
    the whole point: there is exactly one place a sign error can occur, and it is tested.
    """
    return rot_z(azimuth) @ rot_y(-elevation) @ rot_x(roll)


def mount_quat(azimuth: float, elevation: float = 0.0, roll: float = 0.0) -> Quat:
    """:func:`mount_frame` as a quaternion, ready for an MJCF ``quat`` attribute."""
    return quat_from_matrix(mount_frame(azimuth, elevation, roll))


def mount_position(azimuth: float, radius: float, z: float = 0.0) -> Vec3:
    """Mount point on the palm rim at ``azimuth``, ``radius`` from the palm axis."""
    return (radius * math.cos(azimuth), radius * math.sin(azimuth), z)


def orthonormal_basis(normal: Vec3 | Array) -> tuple[Array, Array, Array]:
    """A right-handed basis whose third vector is ``normal``.

    The reference vector is chosen away from ``normal`` (the classic
    "pick the smallest component" trick) so the construction never degenerates -- relevant
    for contact frames, where the normal is arbitrary.
    """
    n = normalize(normal)
    ref = np.array([1.0, 0.0, 0.0]) if abs(n[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    t1 = normalize(np.cross(ref, n))
    t2 = np.cross(n, t1)
    return t1, t2, n


def look_at_quat(direction: Vec3 | Array, up: Vec3 | Array = (0.0, 0.0, 1.0)) -> Quat:
    """Orientation whose local ``+x`` points along ``direction``."""
    x = normalize(direction)
    u = np.asarray(up, float)
    if abs(float(np.dot(x, normalize(u)))) > 0.999:
        u = np.array([0.0, 1.0, 0.0])
    z = normalize(np.cross(x, normalize(np.cross(u, x))))
    y = np.cross(z, x)
    return quat_from_matrix(np.column_stack([x, y, z]))


# ---------------------------------------------------------------------------------------
# small helpers
# ---------------------------------------------------------------------------------------

def normalize(v: Vec3 | Array) -> Array:
    a = np.asarray(v, float)
    n = np.linalg.norm(a)
    if n < _EPS:
        raise ValueError("cannot normalize a zero-length vector")
    return a / n


def angle_between(a: Vec3 | Array, b: Vec3 | Array) -> float:
    """Unsigned angle between two vectors, in ``[0, pi]``."""
    ca = float(np.clip(np.dot(normalize(a), normalize(b)), -1.0, 1.0))
    return math.acos(ca)


def vec_to_str(v: Vec3 | Array, digits: int = 6) -> str:
    """Format a vector as an MJCF attribute value."""
    return " ".join(f"{float(x):.{digits}g}" for x in np.asarray(v, float).ravel())


def quat_to_str(q: Quat | Array, digits: int = 8) -> str:
    """Format a quaternion as an MJCF ``quat`` attribute (``w x y z``)."""
    arr = np.asarray(q, float)
    arr = arr / max(np.linalg.norm(arr), _EPS)
    return " ".join(f"{float(x):.{digits}g}" for x in arr)


def wrap_angle(a: float) -> float:
    """Wrap to ``(-pi, pi]``."""
    a = math.fmod(a + math.pi, 2 * math.pi)
    if a <= 0:
        a += 2 * math.pi
    return a - math.pi


def wrap_2pi(a: float) -> float:
    """Wrap to ``[0, 2pi)`` -- used when canonicalizing digit azimuths."""
    return math.fmod(math.fmod(a, 2 * math.pi) + 2 * math.pi, 2 * math.pi)
