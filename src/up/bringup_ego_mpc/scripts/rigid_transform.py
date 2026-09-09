#!/usr/bin/env python3

"""无 ROS 依赖的刚体变换工具，统一使用 ROS ``xyzw`` 四元数顺序。

本文件专门处理 Point-LIO 的 IMU 原点到无人机机体中心的固定杆臂外参。所有输入
都会先检查维数和有限性，避免无效位姿进入控制和 PX4 视觉里程计链路。
"""

import math
from typing import Sequence


Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]
Pose = tuple[Vector3, Quaternion]


def _finite_vector(values: Sequence[float], size: int, name: str) -> tuple[float, ...]:
    """将输入转为固定长度浮点元组，并拒绝 NaN 和无穷值。"""
    if len(values) != size:
        raise ValueError(f"{name} must contain {size} values")
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must contain only finite values")
    return result


def normalize_quaternion(values: Sequence[float]) -> Quaternion:
    """归一化 ROS ``xyzw`` 四元数；零范数四元数没有有效旋转含义。"""
    x, y, z, w = _finite_vector(values, 4, "quaternion")
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        raise ValueError("quaternion norm is zero")
    return (x / norm, y / norm, z / norm, w / norm)


def quaternion_multiply(lhs: Sequence[float], rhs: Sequence[float]) -> Quaternion:
    """计算 ``lhs * rhs``，对应先应用 rhs、再应用 lhs 的旋转复合。"""
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
    """使用 ROS ``xyzw`` 四元数旋转三维向量。"""
    qx, qy, qz, qw = normalize_quaternion(quaternion)
    vx, vy, vz = _finite_vector(vector, 3, "vector")

    # 展开计算 q * [v, 0] * q^-1，避免构造临时四元数对象。
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
    """计算 ``T_world_child = T_world_parent * T_parent_child``（ROS xyzw）。"""
    px, py, pz = _finite_vector(world_parent_position, 3, "world_parent_position")

    # 杆臂平移定义在父坐标系（此处通常为 MID360 IMU 的 base_link）中。
    # 必须先按父坐标系当前世界姿态把它旋转到 world，再与世界位置相加；若直接
    # 逐轴相加，飞行器滚转或俯仰时会产生姿态相关的位置误差。
    rotated_translation = rotate_vector(world_parent_orientation, parent_child_translation)
    orientation = quaternion_multiply(world_parent_orientation, parent_child_rotation)
    return (
        (
            px + rotated_translation[0],
            py + rotated_translation[1],
            pz + rotated_translation[2],
        ),
        orientation,
    )
