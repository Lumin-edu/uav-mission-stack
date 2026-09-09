#!/usr/bin/env python3
"""用于解析 ``traj_utils/Bspline`` 消息的轻量采样器，不依赖 EGO 运行节点。

尽管 EGO 规划器通常输出均匀 B-spline，消息中仍携带完整的非均匀节点向量。
在本文件内实现 De Boor 求值后，MPC 无需依赖 ``traj_server`` 或其他 bringup 包。

该采样器在 MPC 边界保持规划器轨迹的连续时间语义：收到一条消息后，预先构造
位置、速度和加速度 B-spline；每个控制周期再按 ``now+k*dt`` 获取未来参考点。
求值时间会被限制在有效节点区间内，这是有意的安全行为：一条尚未超时但已经
运行到末端的轨迹会保持终点，而不是向区间外无界外推。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import numpy as np


def _de_boor(u: float, control_points: np.ndarray, degree: int, knots: np.ndarray) -> np.ndarray:
    """在节点坐标 ``u`` 处计算一次向量值 B-spline。

    De Boor 算法相当于 B-spline 版本的 de Casteljau 算法。本实现直接处理向量
    控制点，将请求坐标限制在有效定义域内，并显式处理 EGO 使用的重复末端节点。
    """
    count = int(control_points.shape[0])
    if count == 0 or degree < 0 or knots.size != count + degree + 1:
        raise ValueError("inconsistent B-spline control point/knot dimensions")
    lo = float(knots[degree])
    hi = float(knots[count])
    ub = min(max(float(u), lo), hi)

    # 位于末端节点时选择最后一个有效节点区间，与 EGO 的 C++ 求值行为保持一致。
    span = degree
    while span + 1 < knots.size and knots[span + 1] < ub:
        span += 1
    span = min(span, count - 1)
    work = [control_points[span - degree + i].copy() for i in range(degree + 1)]
    for level in range(1, degree + 1):
        for i in range(degree, level - 1, -1):
            left = knots[i + span - degree]
            right = knots[i + 1 + span - level]
            denominator = right - left
            alpha = 0.0 if abs(denominator) < 1e-12 else (ub - left) / denominator
            work[i] = (1.0 - alpha) * work[i - 1] + alpha * work[i]
    return work[degree]


def _derivative(control_points: np.ndarray, degree: int, knots: np.ndarray):
    """对 B-spline 解析求导，得到次数降低一阶的导数 B-spline。"""
    if degree <= 0:
        return np.zeros((1, control_points.shape[1])), 0, knots[1:-1]
    result = np.empty((control_points.shape[0] - 1, control_points.shape[1]))
    for i in range(result.shape[0]):
        denominator = knots[i + degree + 1] - knots[i + 1]
        if abs(denominator) < 1e-12:
            result[i] = 0.0
        else:
            result[i] = degree * (control_points[i + 1] - control_points[i]) / denominator
    return result, degree - 1, knots[1:-1]


@dataclass(frozen=True)
class ReferencePoint:
    """规划器坐标系中的单个时刻参考：位置、速度、加速度和偏航角。"""

    position: tuple[float, float, float]
    velocity: tuple[float, float, float]
    acceleration: tuple[float, float, float]
    yaw: float


class BsplineReference:
    """一条 EGO B-spline 轨迹的不可变数值表示。

    构造时即检查控制点、节点向量等几何数据，使控制定时器中的 ``evaluate`` 和
    ``sample`` 只负责数值求值，不再经过消息解析分支。导数控制点只预计算一次，
    避免每个 MPC 周期重复求导。
    """

    def __init__(self, order: int, start_time_sec: float, knots: Sequence[float], points: Sequence[Sequence[float]], yaw: Sequence[float], yaw_dt: float):
        self.order = int(order)
        self.start_time_sec = float(start_time_sec)
        self.knots = np.asarray(tuple(float(value) for value in knots), dtype=float)
        self.control_points = np.asarray(tuple(tuple(float(v) for v in point) for point in points), dtype=float)
        if self.control_points.ndim != 2 or self.control_points.shape[1] != 3:
            raise ValueError("pos_pts must be an Nx3 array")
        if self.order < 1 or self.knots.size != self.control_points.shape[0] + self.order + 1:
            raise ValueError("invalid EGO B-spline order or knot vector")
        if not np.all(np.isfinite(self.knots)) or np.any(np.diff(self.knots) < 0.0):
            raise ValueError("B-spline knots must be finite and non-decreasing")
        if not np.all(np.isfinite(self.control_points)):
            raise ValueError("B-spline control points must be finite")
        self.yaw_points = tuple(float(value) for value in yaw)
        self.yaw_dt = float(yaw_dt)
        # 以 EGO 的位置样条为唯一基准，连续解析求导两次得到速度和加速度样条。
        # 这样三者在数学上保持一致，也避免在 MPC 接口处使用噪声较大的有限差分。
        self.velocity_points, self.velocity_order, self.velocity_knots = _derivative(
            self.control_points, self.order, self.knots
        )
        self.acceleration_points, self.acceleration_order, self.acceleration_knots = _derivative(
            self.velocity_points, self.velocity_order, self.velocity_knots
        )
        self.start_u = float(self.knots[self.order])
        self.end_u = float(self.knots[self.control_points.shape[0]])
        self.duration = max(0.0, self.end_u - self.start_u)

    @classmethod
    def from_message(cls, msg, fallback_start_time_sec: float = 0.0) -> "BsplineReference":
        """将 ROS 消息转换为数值轨迹；时间戳为零时使用消息接收时刻。"""
        stamp = getattr(msg, "start_time", None)
        start_sec = 0.0
        if stamp is not None:
            start_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if start_sec <= 1e-6:
            start_sec = float(fallback_start_time_sec)
        points = [(point.x, point.y, point.z) for point in msg.pos_pts]
        return cls(msg.order, start_sec, msg.knots, points, msg.yaw_pts, msg.yaw_dt)

    def _relative_time(self, now_sec: float) -> float:
        # EGO 的 ``traj_server`` 按 t=(now-start_time) 求值，并令 t=0 对应
        # knots[order]。ROS 时间戳为零时，将该轨迹视为收到后立即生效。
        age = float(now_sec) - self.start_time_sec if self.start_time_sec > 1e-6 else 0.0
        return min(max(age, 0.0), self.duration)

    def _yaw(self, relative_time: float) -> float:
        """对 EGO 可选的离散偏航角采样点进行线性插值。"""
        if not self.yaw_points:
            return 0.0
        if len(self.yaw_points) == 1 or self.yaw_dt <= 1e-9:
            return self.yaw_points[0]
        index = min(max(relative_time / self.yaw_dt, 0.0), len(self.yaw_points) - 1.0)
        low = int(math.floor(index))
        high = min(low + 1, len(self.yaw_points) - 1)
        fraction = index - low
        return (1.0 - fraction) * self.yaw_points[low] + fraction * self.yaw_points[high]

    def evaluate(self, relative_time: float) -> ReferencePoint:
        """在一个相对时刻同时计算位置、速度和加速度参考。"""
        t = min(max(float(relative_time), 0.0), self.duration)
        u = self.start_u + t
        position = _de_boor(u, self.control_points, self.order, self.knots)
        velocity = _de_boor(u, self.velocity_points, self.velocity_order, self.velocity_knots)
        acceleration = _de_boor(
            u, self.acceleration_points, self.acceleration_order, self.acceleration_knots
        )
        values = tuple(position) + tuple(velocity) + tuple(acceleration)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("B-spline evaluation returned a non-finite value")
        return ReferencePoint(tuple(position), tuple(velocity), tuple(acceleration), self._yaw(t))

    def sample(self, now_sec: float, horizon: int, dt: float) -> list[ReferencePoint]:
        """返回 MPC 预测时域的未来参考点，不包含当前状态所在的第零拍。"""
        if horizon < 1 or dt <= 0.0:
            raise ValueError("horizon must be positive and dt must be positive")
        age = self._relative_time(now_sec)
        return [self.evaluate(age + index * dt) for index in range(1, horizon + 1)]


__all__ = ["BsplineReference", "ReferencePoint"]
