#!/usr/bin/env python3

import math
from typing import Sequence


Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]
Pose = tuple[Vector3, Quaternion]


def _finite_vector(values: Sequence[float], size: int, name: str) -> tuple[float, ...]:
    if len(values) != size:
        raise ValueError(f"{name} must contain {size} values")
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must contain only finite values")
    return result


def normalize_quaternion(values: Sequence[float]) -> Quaternion:
    x, y, z, w = _finite_vector(values, 4, "quaternion")
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        raise ValueError("quaternion norm is zero")
    return (x / norm, y / norm, z / norm, w / norm)


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
    vx, vy, vz = _finite_vector(vector, 3, "vector")

    # q * [v, 0] * q^-1, expanded to avoid temporary quaternion objects.
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def compose_pose(
    world_parent_position: Sequence[float],
    world_parent_orientation: Sequence[float],
    parent_child_translation: Sequence[float],
    parent_child_rotation: Sequence[float],
) -> Pose:
    """Return T_world_child = T_world_parent * T_parent_child using ROS xyzw quaternions."""
    px, py, pz = _finite_vector(world_parent_position, 3, "world_parent_position")
    rotated_translation = rotate_vector(
        world_parent_orientation, parent_child_translation
    )
    orientation = quaternion_multiply(
        world_parent_orientation, parent_child_rotation
    )
    return (
        (
            px + rotated_translation[0],
            py + rotated_translation[1],
            pz + rotated_translation[2],
        ),
        orientation,
    )
