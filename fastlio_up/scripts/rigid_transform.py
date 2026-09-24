import math
from collections.abc import Sequence

Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]


def _finite(values: Sequence[float], size: int, name: str) -> tuple[float, ...]:
    if len(values) != size:
        raise ValueError(f"{name} must contain {size} values")
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must contain finite values")
    return result


def normalize_quaternion(values: Sequence[float]) -> Quaternion:
    x, y, z, w = _finite(values, 4, "quaternion")
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        raise ValueError("quaternion norm is zero")
    return x / norm, y / norm, z / norm, w / norm


def quaternion_multiply(lhs: Sequence[float], rhs: Sequence[float]) -> Quaternion:
    x1, y1, z1, w1 = normalize_quaternion(lhs)
    x2, y2, z2, w2 = normalize_quaternion(rhs)
    return normalize_quaternion(
        (
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        )
    )


def rotate_vector(quaternion: Sequence[float], vector: Sequence[float]) -> Vector3:
    qx, qy, qz, qw = normalize_quaternion(quaternion)
    vx, vy, vz = _finite(vector, 3, "vector")
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def compose_pose(
    parent_position: Sequence[float],
    parent_orientation: Sequence[float],
    child_translation: Sequence[float],
    child_rotation: Sequence[float],
) -> tuple[Vector3, Quaternion]:
    px, py, pz = _finite(parent_position, 3, "parent_position")
    translated = rotate_vector(parent_orientation, child_translation)
    return (
        (px + translated[0], py + translated[1], pz + translated[2]),
        quaternion_multiply(parent_orientation, child_rotation),
    )
