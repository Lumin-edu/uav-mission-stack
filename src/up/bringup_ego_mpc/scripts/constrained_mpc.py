#!/usr/bin/env python3
"""基于 OSQP 的稀疏线性 MPC，并在整个预测时域内施加硬约束。

控制器在 PX4 NED 坐标系中使用六状态质点模型：

    x = [p_N, p_E, p_D, v_N, v_E, v_D]
    u = [a_N, a_E, a_D]

优化器求解完整的加速度序列 ``u[0:H]``，调用方只发布第一拍 ``u[0]``，并在
下一控制周期重新求解。这就是滚动时域控制：速度、加速度和 jerk 限制必须覆盖
所有预测采样点，不能只对最终发布的第一拍做限幅。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from itertools import product
from typing import Optional, Sequence

import numpy as np
import osqp
from scipy import sparse


class MpcSolveError(RuntimeError):
    """无法得到满足飞行安全约束的 MPC 解时抛出。"""


@dataclass(frozen=True)
class MpcSolveStats:
    """发布到 ROS 诊断话题的求解器数值健康信息。"""

    status: str
    solve_time_ms: float
    iterations: int
    objective: float
    primal_residual: float
    dual_residual: float
    max_constraint_violation: float


def _block_diag(block: np.ndarray, count: int) -> np.ndarray:
    """沿预测时域重复单拍权重块，构造块对角代价矩阵。"""
    return np.kron(np.eye(count), block)


class ConstrainedLinearMpc:
    """带多面体硬约束的凝聚形式双积分 MPC。

    实际物理限制通常采用欧氏范数（L2），而标准 QP 只能直接表达线性不等式。
    因此这里使用保守的内近似：通过全部符号组合构造速度和总加速度的 L1 球，
    用内接正多边形近似水平加速度圆。求解完成后，``_validate_physical_limits``
    还会使用真实 L2 范数独立复核整个预测序列，复核通过后才返回控制量。
    """

    _SIGN_VECTORS = np.asarray(tuple(product((-1.0, 1.0), repeat=3)), dtype=float)

    def __init__(
        self,
        dt: float,
        horizon: int,
        position_weight: float,
        velocity_weight: float,
        acceleration_weight: float,
        jerk_weight: float,
        terminal_weight: float,
        max_velocity: float,
        max_acceleration: float,
        max_horizontal_acceleration: float,
        max_vertical_acceleration: float,
        max_jerk: float,
        control_dt: Optional[float] = None,
        solver_eps_abs: float = 1e-4,
        solver_eps_rel: float = 1e-4,
        solver_max_iter: int = 4000,
        solver_time_limit_ms: float = 10.0,
        solver_constraint_tolerance: float = 5e-4,
        horizontal_polygon_sides: int = 16,
    ) -> None:
        self.dt = float(dt)
        self.control_dt = self.dt if control_dt is None else float(control_dt)
        self.horizon = int(horizon)
        self.max_velocity = float(max_velocity)
        self.max_acceleration = float(max_acceleration)
        self.max_horizontal_acceleration = float(max_horizontal_acceleration)
        self.max_vertical_acceleration = float(max_vertical_acceleration)
        self.max_jerk = float(max_jerk)
        self.solver_time_limit_ms = float(solver_time_limit_ms)
        self.solver_constraint_tolerance = float(solver_constraint_tolerance)
        self.horizontal_polygon_sides = int(horizontal_polygon_sides)

        scalar_values = (
            self.dt,
            self.control_dt,
            self.max_velocity,
            self.max_acceleration,
            self.max_horizontal_acceleration,
            self.max_vertical_acceleration,
            self.max_jerk,
            float(solver_eps_abs),
            float(solver_eps_rel),
            self.solver_time_limit_ms,
            self.solver_constraint_tolerance,
        )
        if self.horizon < 1 or not all(math.isfinite(value) and value > 0.0 for value in scalar_values):
            raise ValueError("MPC timing, limits, and solver tolerances must be positive")
        if int(solver_max_iter) < 1:
            raise ValueError("solver_max_iter must be positive")
        if self.horizontal_polygon_sides < 4 or self.horizontal_polygon_sides % 2:
            raise ValueError("horizontal_polygon_sides must be an even integer of at least four")
        polygon_bound = self.max_horizontal_acceleration * math.cos(
            math.pi / self.horizontal_polygon_sides
        )
        smallest_constraint = min(
            self.max_velocity,
            self.max_acceleration,
            polygon_bound,
            self.max_vertical_acceleration,
            self.max_jerk * min(self.control_dt, self.dt),
        )
        if self.solver_constraint_tolerance >= smallest_constraint:
            raise ValueError("solver_constraint_tolerance leaves no feasible constraint margin")
        weights = (
            position_weight,
            velocity_weight,
            acceleration_weight,
            jerk_weight,
            terminal_weight,
        )
        if not all(math.isfinite(float(value)) and float(value) >= 0.0 for value in weights):
            raise ValueError("MPC cost weights must be finite and non-negative")

        # 当 dt 和预测步数固定时，下列矩阵均为常量，只需在初始化时构造一次。
        # 每次求解仅更新一次项和约束上下界，因此 OSQP 可复用问题结构并进行热启动。
        self.a, self.b = self._system_matrices()
        self.sx, self.su = self._prediction_matrices()
        self.difference = self._difference_matrix()
        self.q_bar, self.r_bar, self.rd_bar, self.hessian = self._cost_matrices(
            float(position_weight),
            float(velocity_weight),
            float(acceleration_weight),
            float(jerk_weight),
            float(terminal_weight),
        )
        self.constraint_matrix, self._base_lower, self._base_upper = self._constraint_matrices()
        self._velocity_row_count = self.horizon * len(self._SIGN_VECTORS)
        self._jerk_row_start = self.constraint_matrix.shape[0] - self.horizon * len(self._SIGN_VECTORS)

        self._solver = osqp.OSQP()
        self._solver.setup(
            P=sparse.triu(sparse.csc_matrix(self.hessian), format="csc"),
            q=np.zeros(3 * self.horizon),
            A=self.constraint_matrix,
            l=self._base_lower,
            u=self._base_upper,
            verbose=False,
            eps_abs=float(solver_eps_abs),
            eps_rel=float(solver_eps_rel),
            max_iter=int(solver_max_iter),
            time_limit=self.solver_time_limit_ms / 1000.0,
            polishing=False,
            warm_starting=True,
            check_termination=10,
        )
        self.last_sequence = np.zeros((self.horizon, 3), dtype=float)
        self.last_predicted_states = np.zeros((self.horizon, 6), dtype=float)
        self.last_stats = MpcSolveStats("not solved", 0.0, 0, math.nan, math.inf, math.inf, math.inf)

    def _system_matrices(self) -> tuple[np.ndarray, np.ndarray]:
        """构造离散常加速度模型 ``x[k+1]=A*x[k]+B*u[k]``。

        位置更新采用零阶保持的精确项 ``0.5*dt^2*a``，速度更新采用 ``dt*a``。
        模型有意不包含姿态和电机动力学，因为这些内环由 PX4 负责；从本 MPC 的
        接口看，飞行器被视为能够跟踪三轴平动加速度指令的执行器。
        """
        a = np.eye(6)
        a[0, 3] = self.dt
        a[1, 4] = self.dt
        a[2, 5] = self.dt
        b = np.zeros((6, 3))
        b[:3, :] = np.eye(3) * (0.5 * self.dt * self.dt)
        b[3:, :] = np.eye(3) * self.dt
        return a, b

    def _prediction_matrices(self) -> tuple[np.ndarray, np.ndarray]:
        """将整个预测时域的动力学凝聚为 ``X = Sx*x0 + Su*U``。

        ``Sx`` 表示未来输入为零时，当前状态 ``x0`` 引起的自由响应；``Su`` 为
        下三角块矩阵，因为第 ``j`` 拍输入只能影响第 ``j`` 拍及其后的状态。
        """
        sx = np.zeros((6 * self.horizon, 6))
        su = np.zeros((6 * self.horizon, 3 * self.horizon))
        power = np.eye(6)
        for row in range(self.horizon):
            power = self.a @ power
            sx[row * 6 : row * 6 + 6] = power
            for column in range(row + 1):
                transition = np.linalg.matrix_power(self.a, row - column) @ self.b
                su[row * 6 : row * 6 + 6, column * 3 : column * 3 + 3] = transition
        return sx, su

    def _difference_matrix(self) -> np.ndarray:
        """构造用于 jerk 代价和约束的一阶差分矩阵。

        第一行对应“本次第一拍加速度减去上次实际发布的加速度”，其右端项稍后在
        ``_bounds`` 中按历史指令平移；其余行直接表示 ``u[k]-u[k-1]``。
        """
        difference = np.zeros((3 * self.horizon, 3 * self.horizon))
        for index in range(self.horizon):
            current = slice(index * 3, index * 3 + 3)
            difference[current, current] = np.eye(3)
            if index:
                previous = slice((index - 1) * 3, (index - 1) * 3 + 3)
                difference[current, previous] = -np.eye(3)
        return difference

    def _cost_matrices(
        self,
        position_weight: float,
        velocity_weight: float,
        acceleration_weight: float,
        jerk_weight: float,
        terminal_weight: float,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """构造 OSQP 使用的二次代价矩阵。

        ``q_bar`` 惩罚预测位置和速度的跟踪误差；``r_bar`` 惩罚加速度跟踪误差，
        并在 ``solve`` 中与参考加速度配对；``rd_bar`` 惩罚相邻加速度指令变化，
        是硬 jerk 约束之外的平滑软代价。
        """
        q = np.diag([position_weight] * 3 + [velocity_weight] * 3)
        q_bar = _block_diag(q, self.horizon)
        q_bar[-6:, -6:] = q * terminal_weight
        r_bar = np.eye(3 * self.horizon) * acceleration_weight
        rd_bar = np.eye(3 * self.horizon) * jerk_weight
        hessian = (
            self.su.T @ q_bar @ self.su
            + r_bar
            + self.difference.T @ rd_bar @ self.difference
        )
        hessian = 0.5 * (hessian + hessian.T)
        hessian += np.eye(hessian.shape[0]) * 1e-8
        return q_bar, r_bar, rd_bar, hessian

    def _constraint_matrices(self) -> tuple[sparse.csc_matrix, np.ndarray, np.ndarray]:
        """为所有未来采样点构造固定的线性约束矩阵。

        约束行顺序不可随意调整：依次为速度、总加速度、水平加速度多边形、
        垂直加速度区间和 jerk。``_bounds`` 会按此布局更新状态相关的速度上界，
        并假定最后一组是 jerk 约束，以便用上一拍指令平移首个差分约束。
        """
        columns = 3 * self.horizon
        rows: list[np.ndarray] = []
        lower: list[float] = []
        upper: list[float] = []

        # 预测总速度：施加 ||v||_1 <= max_velocity。由于 ||v||_2 <= ||v||_1，
        # 该保守约束可确保真实 L2 速度不越界，也不会出现多面体顶点伸出速度球。
        for index in range(self.horizon):
            velocity_su = self.su[index * 6 + 3 : index * 6 + 6]
            for signs in self._SIGN_VECTORS:
                rows.append(signs @ velocity_su)
                lower.append(-math.inf)
                upper.append(self.max_velocity - self.solver_constraint_tolerance)

        # 总加速度：对每个预测控制量施加 ||u||_1 <= max_acceleration。
        for index in range(self.horizon):
            block = slice(index * 3, index * 3 + 3)
            for signs in self._SIGN_VECTORS:
                row = np.zeros(columns)
                row[block] = signs
                rows.append(row)
                lower.append(-math.inf)
                upper.append(self.max_acceleration - self.solver_constraint_tolerance)

        # 各方向半空间的交集形成正多边形，其顶点落在配置的水平加速度圆上；
        # 因而多边形整体位于圆内，是水平 L2 加速度限制的保守内近似。
        polygon_bound = self.max_horizontal_acceleration * math.cos(
            math.pi / self.horizontal_polygon_sides
        ) - self.solver_constraint_tolerance
        for index in range(self.horizon):
            for side in range(self.horizontal_polygon_sides):
                angle = 2.0 * math.pi * side / self.horizontal_polygon_sides
                row = np.zeros(columns)
                row[index * 3] = math.cos(angle)
                row[index * 3 + 1] = math.sin(angle)
                rows.append(row)
                lower.append(-math.inf)
                upper.append(polygon_bound)

        # NED 的 D 轴加速度本身是一维量，可直接写成上下界区间。
        for index in range(self.horizon):
            row = np.zeros(columns)
            row[index * 3 + 2] = 1.0
            rows.append(row)
            lower.append(-self.max_vertical_acceleration + self.solver_constraint_tolerance)
            upper.append(self.max_vertical_acceleration - self.solver_constraint_tolerance)

        # Jerk：||delta_u||_1 <= max_jerk * period。第一拍差分相对于上次已经
        # 发送给 PX4 的加速度计算，因此其边界会在 solve() 中按历史指令平移。
        for index in range(self.horizon):
            difference_block = self.difference[index * 3 : index * 3 + 3]
            period = self.control_dt if index == 0 else self.dt
            for signs in self._SIGN_VECTORS:
                rows.append(signs @ difference_block)
                lower.append(-math.inf)
                upper.append(self.max_jerk * period - self.solver_constraint_tolerance)

        return (
            sparse.csc_matrix(np.asarray(rows, dtype=float)),
            np.asarray(lower, dtype=float),
            np.asarray(upper, dtype=float),
        )

    def _bounds(self, state: np.ndarray, previous: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """在不重建约束矩阵 ``A`` 的情况下更新与当前状态有关的右端项。

        速度约束取决于自由响应 ``Sx*x0``；只有第一拍 jerk 差分依赖已经发送给
        PX4 的上一拍指令，后续差分均直接使用固定的 ``u[k]-u[k-1]`` 矩阵行。
        """
        lower = self._base_lower.copy()
        upper = self._base_upper.copy()

        predicted_without_input = (self.sx @ state).reshape(self.horizon, 6)
        row = 0
        for index in range(self.horizon):
            velocity = predicted_without_input[index, 3:6]
            for signs in self._SIGN_VECTORS:
                upper[row] = (
                    self.max_velocity
                    - self.solver_constraint_tolerance
                    - float(signs @ velocity)
                )
                row += 1

        row = self._jerk_row_start
        for signs in self._SIGN_VECTORS:
            upper[row] = (
                self.max_jerk * self.control_dt
                - self.solver_constraint_tolerance
                + float(signs @ previous)
            )
            row += 1
        return lower, upper

    def _validate_physical_limits(
        self,
        controls: np.ndarray,
        predicted: np.ndarray,
        previous: np.ndarray,
    ) -> None:
        """使用真实物理范数重新检查求解结果。

        OSQP 求解的是保守多面体近似问题。这里有意独立于 QP 约束行再检查一次，
        防止建模或索引错误悄悄放过不安全的速度、加速度或 jerk 指令。
        """
        differences = np.vstack((controls[0] - previous, np.diff(controls, axis=0)))
        periods = np.asarray(
            [self.control_dt] + [self.dt] * (self.horizon - 1), dtype=float
        )
        checks = (
            (np.linalg.norm(predicted[:, 3:6], axis=1), self.max_velocity, "velocity"),
            (np.linalg.norm(controls, axis=1), self.max_acceleration, "acceleration"),
            (
                np.linalg.norm(controls[:, :2], axis=1),
                self.max_horizontal_acceleration,
                "horizontal acceleration",
            ),
            (np.abs(controls[:, 2]), self.max_vertical_acceleration, "vertical acceleration"),
            (np.linalg.norm(differences, axis=1) / periods, self.max_jerk, "jerk"),
        )
        for values, limit, name in checks:
            maximum = float(np.max(values))
            if not math.isfinite(maximum) or maximum > limit + 1e-9:
                raise MpcSolveError(
                    f"OSQP {name} prediction exceeds its physical limit "
                    f"({maximum:.6f} > {limit:.6f})"
                )

    @staticmethod
    def _info_residual(info, preferred: str, legacy: str) -> float:
        return float(getattr(info, preferred, getattr(info, legacy, math.inf)))

    def _stats(self, result, lower: np.ndarray, upper: np.ndarray) -> MpcSolveStats:
        controls = np.asarray(result.x, dtype=float) if result.x is not None else np.zeros(3 * self.horizon)
        values = np.asarray(self.constraint_matrix @ controls).reshape(-1)
        lower_violation = np.maximum(lower - values, 0.0)
        upper_violation = np.maximum(values - upper, 0.0)
        violation = float(max(np.max(lower_violation), np.max(upper_violation)))
        info = result.info
        return MpcSolveStats(
            status=str(info.status),
            solve_time_ms=float(info.run_time) * 1000.0,
            iterations=int(info.iter),
            objective=float(info.obj_val),
            primal_residual=self._info_residual(info, "prim_res", "pri_res"),
            dual_residual=self._info_residual(info, "dual_res", "dua_res"),
            max_constraint_violation=violation,
        )

    def solve(
        self,
        state: Sequence[float],
        references: Sequence[Sequence[float]],
        previous: Optional[Sequence[float]],
    ) -> np.ndarray:
        """求解一次滚动时域优化，并返回第一拍加速度。

        ``references`` 的每个采样点包含九个量：参考位置、速度和加速度。
        这里的参考加速度是代价函数中的前馈跟踪目标，并不是发送给 PX4 的第二条
        控制通道。只有当 OSQP 状态为已求解、耗时/残差/约束检查全部通过，且完整
        预测序列满足物理限制时，结果才会被接受。
        """
        x0 = np.asarray(tuple(float(value) for value in state), dtype=float)
        target = np.asarray(references, dtype=float)
        previous_value = (
            np.zeros(3, dtype=float)
            if previous is None
            else np.asarray(tuple(float(value) for value in previous), dtype=float)
        )
        if x0.shape != (6,) or target.shape != (self.horizon, 9) or previous_value.shape != (3,):
            raise ValueError("MPC state, reference, or previous-input dimensions are invalid")
        if not np.all(np.isfinite(x0)) or not np.all(np.isfinite(target)) or not np.all(np.isfinite(previous_value)):
            raise ValueError("MPC state, reference, or previous input contains a non-finite value")

        # 将逐采样点的 [p_ref, v_ref, a_ref] 展平为预计算凝聚矩阵所需的排列。
        x_ref = target[:, :6].reshape(-1)
        u_ref = target[:, 6:9].reshape(-1)
        previous_stack = np.zeros(3 * self.horizon)
        previous_stack[:3] = previous_value
        gradient = (
            self.su.T @ self.q_bar @ (self.sx @ x0 - x_ref)
            - self.r_bar @ u_ref
            - self.difference.T @ self.rd_bar @ previous_stack
        )
        lower, upper = self._bounds(x0, previous_value)
        self._solver.update(q=gradient, l=lower, u=upper)
        # OSQP 内部使用上次解热启动。即便如此，任何状态异常或超时仍按硬故障处理；
        # 调用方会切换到位置保持，不会发布未经确认的数值。
        result = self._solver.solve(raise_error=False)
        self.last_stats = self._stats(result, lower, upper)

        if int(result.info.status_val) != 1:
            raise MpcSolveError(f"OSQP did not return a solved result ({self.last_stats.status})")
        if self.last_stats.solve_time_ms > self.solver_time_limit_ms:
            raise MpcSolveError(
                f"OSQP solve exceeded {self.solver_time_limit_ms:.3f} ms ({self.last_stats.solve_time_ms:.3f} ms)"
            )
        if not all(
            math.isfinite(value)
            for value in (
                self.last_stats.objective,
                self.last_stats.primal_residual,
                self.last_stats.dual_residual,
                self.last_stats.max_constraint_violation,
            )
        ):
            raise MpcSolveError("OSQP returned non-finite solver statistics")
        if self.last_stats.max_constraint_violation > self.solver_constraint_tolerance:
            raise MpcSolveError(
                "OSQP solution violates hard constraints by "
                f"{self.last_stats.max_constraint_violation:.3e}"
            )

        controls = np.asarray(result.x, dtype=float).reshape(self.horizon, 3)
        # 使用将要保存和检查的同一控制序列重算预测状态，使诊断信息对应的轨迹
        # 与生成 PX4 第一拍指令的轨迹完全一致。
        predicted = (self.sx @ x0 + self.su @ controls.reshape(-1)).reshape(self.horizon, 6)
        if not np.all(np.isfinite(controls)) or not np.all(np.isfinite(predicted)):
            raise MpcSolveError("OSQP returned a non-finite trajectory")
        self._validate_physical_limits(controls, predicted, previous_value)
        self.last_sequence = controls.copy()
        self.last_predicted_states = predicted.copy()
        return controls[0].copy()


__all__ = ["ConstrainedLinearMpc", "MpcSolveError", "MpcSolveStats"]
