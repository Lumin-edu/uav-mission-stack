#!/usr/bin/env python3
"""将 EGO 轨迹转换为 PX4 加速度设定值的 MPC 控制器。

EGO 规划器继续负责生成无碰撞轨迹。本节点在该 bringup 包中独占 PX4 Offboard
设定值发布权，并自行闭合位置/速度外环：

    EGO B-spline + PX4 本地位置/速度 -> 线性 MPC -> 加速度设定值
    -> PX4 姿态环/角速度环/电机环

控制器有意不发布姿态、机体系角速度、推力或执行器指令，这些内环仍由 PX4 负责；
轨迹或定位不健康时，则切换到 PX4 位置模式保持。

控制接口采用互斥约定：正常 MPC 运行期间，``TrajectorySetpoint.position`` 和
``velocity`` 均为 NaN，只有 ``acceleration`` 为有限值。PX4 据此完成加速度到
推力/姿态的转换，并继续负责姿态环、角速度环和电机环。位置设定值只用于起飞、
悬停和安全降级，避免 PX4 的位置/速度反馈叠加到 MPC 外环上。
"""

from __future__ import annotations

import math
from typing import Optional, Sequence

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from constrained_mpc import ConstrainedLinearMpc, MpcSolveError, MpcSolveStats
from reference_sampler import BsplineReference

try:
    from traj_utils.msg import Bspline
except ImportError:  # 允许几何测试在缺少 ROS 消息包时导入数值控制代码。
    Bspline = object


def validate_solver_time_limit(control_rate_hz: float, solver_time_limit_ms: float) -> float:
    """拒绝可能占满整个控制周期的求解时间预算。"""
    rate = float(control_rate_hz)
    limit = float(solver_time_limit_ms)
    if not math.isfinite(rate) or rate <= 0.0 or not math.isfinite(limit) or limit <= 0.0:
        raise ValueError("control rate and solver time limit must be positive and finite")
    control_period_ms = 1000.0 / rate
    if limit >= control_period_ms:
        raise ValueError(
            "solver_time_limit_ms must be shorter than the MPC control period "
            f"({limit:.3f} >= {control_period_ms:.3f} ms)"
        )
    return limit


def rebase_ned_position(
    position: Sequence[float], delta_xy: Sequence[float], delta_z: float
) -> tuple[float, float, float]:
    """按 EKF 本地位置重置增量平移一个已保存的 NED 点。

    PX4 报告的 reset delta 与保存目标使用相同的本地 NED 符号约定。对所有历史
    目标同步换基，可防止下一次 MPC 求解把坐标原点跳变误认为飞行器真实运动。
    """
    point = tuple(float(value) for value in position)
    delta = tuple(float(value) for value in delta_xy)
    if len(point) != 3 or len(delta) != 2 or not math.isfinite(float(delta_z)):
        raise ValueError("NED position reset dimensions or values are invalid")
    if not all(math.isfinite(value) for value in point + delta):
        raise ValueError("NED position reset dimensions or values are invalid")
    return point[0] + delta[0], point[1] + delta[1], point[2] + float(delta_z)


LinearMpc = ConstrainedLinearMpc


class EgoMpcController(Node):
    """带安全门控的 EGO 到 PX4 MPC 桥接节点。

    本节点负责平动外环，并独占该 bringup 中的 PX4 Offboard 设定值话题。定时器
    实现了一个小型安全状态机：先检查定位健康度和坐标对齐，再执行起飞或位置
    保持；只有全部门控条件通过后，约束 MPC 才能发布加速度指令。
    """

    def __init__(self) -> None:
        super().__init__("ego_mpc_controller")

        # 实机输出双重门控：只有开关已启用且确认字符串完全匹配，才允许写入任何
        # PX4 输入话题。launch 默认关闭，便于台架检查时避免误发控制指令。
        self.output_enabled = bool(self.declare_parameter("output_enabled", False).value)
        self.hardware_confirmation = str(self.declare_parameter("hardware_confirmation", "").value)
        self.required_confirmation = str(self.declare_parameter("required_confirmation", "ENABLE_PX4_OUTPUT").value)
        self.output_permitted = self.output_enabled and self.hardware_confirmation == self.required_confirmation
        self.auto_arm = bool(self.declare_parameter("auto_arm", False).value)
        self.auto_offboard = bool(self.declare_parameter("auto_offboard", False).value)

        # 定时和求解器参数来自共享 YAML 配置。第一拍 jerk 使用真实控制周期，
        # 后续预测拍使用 mpc_dt；两者允许不同，不能混用。
        self.control_rate_hz = float(self.declare_parameter("control_rate_hz", 50.0).value)
        self.offboard_prestream_sec = float(self.declare_parameter("offboard_prestream_sec", 2.0).value)
        self.dt = float(self.declare_parameter("mpc_dt", 0.05).value)
        self.horizon = int(self.declare_parameter("mpc_horizon", 20).value)
        self.solver_eps_abs = float(self.declare_parameter("solver_eps_abs", 1e-4).value)
        self.solver_eps_rel = float(self.declare_parameter("solver_eps_rel", 1e-4).value)
        self.solver_max_iter = int(self.declare_parameter("solver_max_iter", 4000).value)
        self.solver_time_limit_ms = validate_solver_time_limit(
            self.control_rate_hz,
            float(self.declare_parameter("solver_time_limit_ms", 18.0).value),
        )
        self.solver_constraint_tolerance = float(
            self.declare_parameter("solver_constraint_tolerance", 5e-4).value
        )
        # 下列超时值是安全门控，不是估计器滤波参数。任一输入过期都会确定性地
        # 切换到位置保持，而不是继续外推旧数据。
        self.command_timeout_sec = float(self.declare_parameter("trajectory_timeout_sec", 2.0).value)
        self.odom_timeout_sec = float(self.declare_parameter("odom_timeout_sec", 0.30).value)
        self.px4_timeout_sec = float(self.declare_parameter("px4_timeout_sec", 0.30).value)
        self.reset_recovery_sec = float(self.declare_parameter("reset_recovery_sec", 0.20).value)
        self.reference_capture_delay_sec = float(self.declare_parameter("reference_capture_delay_sec", 3.0).value)
        self.takeoff_before_ego = bool(self.declare_parameter("takeoff_before_ego", True).value)
        self.takeoff_altitude = float(self.declare_parameter("takeoff_altitude", 0.30).value)
        self.takeoff_vertical_speed = float(self.declare_parameter("takeoff_vertical_speed", 0.20).value)
        self.takeoff_reach_xy_tol = float(self.declare_parameter("takeoff_reach_xy_tol", 0.20).value)
        self.takeoff_reach_z_tol = float(self.declare_parameter("takeoff_reach_z_tol", 0.10).value)
        self.takeoff_speed_xy_tol = float(self.declare_parameter("takeoff_speed_xy_tol", 0.20).value)
        self.takeoff_speed_z_tol = float(self.declare_parameter("takeoff_speed_z_tol", 0.15).value)
        self.takeoff_stable_sec = float(self.declare_parameter("takeoff_stable_sec", 0.8).value)
        self.lock_yaw_to_initial_heading = bool(self.declare_parameter("lock_yaw_to_initial_heading", True).value)
        self.use_initial_heading_frame = bool(self.declare_parameter("use_initial_heading_frame", True).value)
        self.reject_dead_reckoning = bool(self.declare_parameter("reject_dead_reckoning", True).value)
        self.max_eph = float(self.declare_parameter("max_eph", 2.0).value)
        self.max_epv = float(self.declare_parameter("max_epv", 2.0).value)
        self.yaw_sign = float(self.declare_parameter("yaw_sign", 1.0).value)
        self.yaw_offset = float(self.declare_parameter("yaw_offset", 0.0).value)

        self.ego_odom_topic = str(self.declare_parameter("ego_odom_topic", "/ego/odom_base").value)
        self.bspline_topic = str(self.declare_parameter("bspline_topic", "/ego/planning/bspline").value)
        self.px4_position_topic = str(self.declare_parameter("px4_position_topic", "/fmu/out/vehicle_local_position").value)
        self.vehicle_status_topic = str(self.declare_parameter("vehicle_status_topic", "/fmu/out/vehicle_status").value)
        self.takeoff_ready_topic = str(self.declare_parameter("takeoff_ready_topic", "/ego/takeoff_ready").value)
        rotation = self.declare_parameter(
            "world_to_px4_rotation",
            (1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0),
        ).value
        self.world_to_px4_rotation = self._vector_parameter("world_to_px4_rotation", rotation, 9)

        # max_velocity 通过 mpc_params.yaml 与 EGO 规划器共用。其余加速度限制会
        # 作为 MPC 硬约束，必须与 PX4 实际可用推力和最大倾角能力相匹配。
        self.max_velocity = float(self.declare_parameter("max_velocity", 1.0).value)
        if (
            self.control_rate_hz <= 0.0
            or self.offboard_prestream_sec <= 0.0
            or self.dt <= 0.0
            or self.horizon < 1
            or self.max_velocity <= 0.0
            or min(
                self.command_timeout_sec,
                self.odom_timeout_sec,
                self.px4_timeout_sec,
                self.reset_recovery_sec,
            ) <= 0.0
            or self.takeoff_altitude <= 0.0
            or self.takeoff_vertical_speed <= 0.0
            or self.max_eph <= 0.0
            or self.max_epv <= 0.0
        ):
            raise ValueError("MPC timing, takeoff, timeout, velocity, and covariance limits must be positive")
        rotation_matrix = np.asarray(self.world_to_px4_rotation, dtype=float).reshape(3, 3)
        if not np.allclose(rotation_matrix.T @ rotation_matrix, np.eye(3), atol=1e-6) or not math.isclose(
            float(np.linalg.det(rotation_matrix)), 1.0, abs_tol=1e-6
        ):
            raise ValueError("world_to_px4_rotation must be a proper 3-D rotation matrix")

        # 优化器状态为 PX4 本地 NED 下的 [位置, 速度]，输入为三轴加速度向量。
        # 该接口以下的加速度到姿态/推力转换、姿态环、角速度环和电机控制仍由 PX4 负责。
        self.mpc = LinearMpc(
            self.dt,
            self.horizon,
            float(self.declare_parameter("position_weight", 12.0).value),
            float(self.declare_parameter("velocity_weight", 3.0).value),
            float(self.declare_parameter("acceleration_weight", 0.20).value),
            float(self.declare_parameter("jerk_weight", 0.50).value),
            float(self.declare_parameter("terminal_weight", 2.0).value),
            self.max_velocity,
            float(self.declare_parameter("max_acceleration", 2.0).value),
            float(self.declare_parameter("max_horizontal_acceleration", 1.5).value),
            float(self.declare_parameter("max_vertical_acceleration", 1.0).value),
            float(self.declare_parameter("max_jerk", 4.0).value),
            control_dt=1.0 / self.control_rate_hz,
            solver_eps_abs=self.solver_eps_abs,
            solver_eps_rel=self.solver_eps_rel,
            solver_max_iter=self.solver_max_iter,
            solver_time_limit_ms=self.solver_time_limit_ms,
            solver_constraint_tolerance=self.solver_constraint_tolerance,
        )

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        ready_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # Best-effort/transient-local QoS 与 px4_msgs 的 Offboard 输入约定匹配。
        # 独立看门狗还会检查数据流是否新鲜，以及轨迹设定值话题是否只有一个发布者。
        self.mode_pub = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", px4_qos)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", px4_qos)
        self.command_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", px4_qos)
        self.takeoff_ready_pub = self.create_publisher(Bool, self.takeoff_ready_topic, ready_qos)
        self.solver_diagnostics_pub = self.create_publisher(
            DiagnosticArray, "/ego_mpc/solver_diagnostics", 10
        )
        self.create_subscription(Odometry, self.ego_odom_topic, self.odom_callback, sensor_qos)
        self.create_subscription(Bspline, self.bspline_topic, self.bspline_callback, sensor_qos)
        self.create_subscription(VehicleLocalPosition, self.px4_position_topic, self.px4_callback, px4_qos)
        self.create_subscription(VehicleStatus, self.vehicle_status_topic, self.status_callback, px4_qos)

        self.latest_odom: Optional[Odometry] = None
        self.latest_odom_sec: Optional[float] = None
        self.latest_px4: Optional[VehicleLocalPosition] = None
        self.latest_px4_sec: Optional[float] = None
        self.latest_status: Optional[VehicleStatus] = None
        self.latest_status_sec: Optional[float] = None
        self.latest_reference: Optional[BsplineReference] = None
        self.latest_reference_sec: Optional[float] = None
        self.reference_world: Optional[tuple[float, float, float]] = None
        self.reference_px4: Optional[tuple[float, float, float]] = None
        self.reference_heading = 0.0
        self.takeoff_origin: Optional[tuple[float, float, float]] = None
        self.takeoff_target: Optional[tuple[float, float, float]] = None
        self.takeoff_command: Optional[list[float]] = None
        self.takeoff_complete = not self.takeoff_before_ego
        self.takeoff_stable_since: Optional[float] = None
        self.hold_position: Optional[tuple[float, float, float]] = None
        self.hold_yaw = 0.0
        self.previous_acceleration: Optional[np.ndarray] = None
        self.reset_recovery_until_sec = 0.0
        self.last_xy_reset_counter: Optional[int] = None
        self.last_z_reset_counter: Optional[int] = None
        self.last_vxy_reset_counter: Optional[int] = None
        self.last_vz_reset_counter: Optional[int] = None
        self.last_heading_reset_counter: Optional[int] = None
        self.setpoint_cycles = 0
        self.last_arm_request_us = 0
        self.last_offboard_request_us = 0
        # 故意延迟捕获参考原点，等待 Point-LIO/PX4 对齐稳定后再锁定坐标关系。
        self.reference_capture_ready_sec = self.now_sec() + self.reference_capture_delay_sec
        self.last_warn_sec = -math.inf
        self.last_solver_diagnostic_sec = -math.inf
        self.last_takeoff_ready_sec = -math.inf
        self.offboard_prestream_cycles = max(10, int(self.offboard_prestream_sec * self.control_rate_hz))
        self.create_timer(1.0 / self.control_rate_hz, self.timer_callback)

        if not self.output_enabled:
            self.get_logger().warn("MPC PX4 output is disabled; set output_enabled and hardware_confirmation for flight.")
        elif not self.output_permitted:
            self.get_logger().error("MPC PX4 output blocked by hardware_confirmation safety gate.")
        self.get_logger().info(
            f"EGO MPC active: bspline={self.bspline_topic}, state={self.px4_position_topic}, "
            f"dt={self.dt:.3f}s, horizon={self.horizon}, output_acceleration_only=True"
        )

    @staticmethod
    def _vector_parameter(name: str, default: Sequence[float], size: int) -> tuple[float, ...]:
        # 显式转为 float，使 YAML 中以整数形式书写的旋转矩阵同样可用。
        result = tuple(float(value) for value in default)
        if len(result) != size or not all(math.isfinite(value) for value in result):
            raise ValueError(f"{name} must contain {size} finite values")
        return result

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def _age(self, stamp: Optional[float]) -> float:
        return math.inf if stamp is None else max(0.0, self.now_sec() - stamp)

    def _warn(self, message: str) -> None:
        now = self.now_sec()
        if now - self.last_warn_sec > 2.0:
            self.last_warn_sec = now
            self.get_logger().warn(message)

    @staticmethod
    def solver_diagnostic_values(stats: MpcSolveStats) -> list[KeyValue]:
        return [
            KeyValue(key="status", value=stats.status),
            KeyValue(key="solve_time_ms", value=f"{stats.solve_time_ms:.3f}"),
            KeyValue(key="iterations", value=str(stats.iterations)),
            KeyValue(key="objective", value=f"{stats.objective:.6g}"),
            KeyValue(key="primal_residual", value=f"{stats.primal_residual:.6g}"),
            KeyValue(key="dual_residual", value=f"{stats.dual_residual:.6g}"),
            KeyValue(
                key="max_constraint_violation",
                value=f"{stats.max_constraint_violation:.6g}",
            ),
        ]

    def publish_solver_diagnostics(
        self, level: int, message: str, force: bool = False
    ) -> None:
        """限频发布求解器健康信息，不让诊断输出阻塞控制环。"""
        now = self.now_sec()
        if not force and now - self.last_solver_diagnostic_sec < 1.0:
            return
        status = DiagnosticStatus(
            level=int(level),
            name="ego_mpc/solver",
            message=message,
            hardware_id="uav-mission-stack",
            values=self.solver_diagnostic_values(self.mpc.last_stats),
        )
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        diagnostics.status = [status]
        self.solver_diagnostics_pub.publish(diagnostics)
        self.last_solver_diagnostic_sec = now

    def odom_callback(self, msg: Odometry) -> None:
        """只缓存数值有限的 EGO 里程计；消息新鲜度统一由控制定时器检查。"""
        position = msg.pose.pose.position
        if all(math.isfinite(float(value)) for value in (position.x, position.y, position.z)):
            self.latest_odom = msg
            self.latest_odom_sec = self.now_sec()

    def bspline_callback(self, msg: Bspline) -> None:
        """解析规划器消息，成功后一次性替换当前有效轨迹。"""
        try:
            reference = BsplineReference.from_message(msg, self.now_sec())
            if reference.duration <= 0.0:
                raise ValueError("trajectory duration is zero")
            self.latest_reference = reference
            self.latest_reference_sec = self.now_sec()
        except (TypeError, ValueError, IndexError) as error:
            self._warn(f"Ignoring invalid EGO B-spline: {error}")

    def _shift_stored_positions(self, delta_xy: Sequence[float], delta_z: float) -> None:
        if self.reference_px4 is not None:
            self.reference_px4 = rebase_ned_position(self.reference_px4, delta_xy, delta_z)
        if self.takeoff_origin is not None:
            self.takeoff_origin = rebase_ned_position(self.takeoff_origin, delta_xy, delta_z)
        if self.takeoff_target is not None:
            self.takeoff_target = rebase_ned_position(self.takeoff_target, delta_xy, delta_z)
        if self.takeoff_command is not None:
            self.takeoff_command = list(rebase_ned_position(self.takeoff_command, delta_xy, delta_z))
        if self.hold_position is not None:
            self.hold_position = rebase_ned_position(self.hold_position, delta_xy, delta_z)

    def apply_px4_reset(
        self,
        delta_xy: Sequence[float] = (0.0, 0.0),
        delta_z: float = 0.0,
        delta_heading: float = 0.0,
    ) -> None:
        """当 EKF 改变本地坐标系时，对所有已保存的 NED 目标同步换基。"""
        self._shift_stored_positions(delta_xy, delta_z)
        if math.isfinite(float(delta_heading)) and abs(float(delta_heading)) > 1e-9:
            self.reference_heading = self.wrap_angle(self.reference_heading + float(delta_heading))
            self.hold_yaw = self.wrap_angle(self.hold_yaw + float(delta_heading))
        self.previous_acceleration = None
        self.reset_recovery_until_sec = self.now_sec() + self.reset_recovery_sec
        self.get_logger().warn(
            "PX4 local frame reset detected; rebased stored MPC targets "
            f"delta_xy=({float(delta_xy[0]):+.3f},{float(delta_xy[1]):+.3f}) "
            f"delta_z={float(delta_z):+.3f} delta_heading={float(delta_heading):+.3f}."
        )

    def px4_callback(self, msg: VehicleLocalPosition) -> None:
        """缓存 PX4 NED 状态；EKF reset 计数变化时同步换基历史目标。"""
        xy_changed = self.last_xy_reset_counter is not None and int(msg.xy_reset_counter) != self.last_xy_reset_counter
        z_changed = self.last_z_reset_counter is not None and int(msg.z_reset_counter) != self.last_z_reset_counter
        vxy_changed = self.last_vxy_reset_counter is not None and int(msg.vxy_reset_counter) != self.last_vxy_reset_counter
        vz_changed = self.last_vz_reset_counter is not None and int(msg.vz_reset_counter) != self.last_vz_reset_counter
        heading_changed = self.last_heading_reset_counter is not None and int(msg.heading_reset_counter) != self.last_heading_reset_counter
        if xy_changed or z_changed or vxy_changed or vz_changed or heading_changed:
            delta_xy = tuple(float(value) for value in msg.delta_xy) if xy_changed else (0.0, 0.0)
            delta_z = float(msg.delta_z) if z_changed else 0.0
            delta_heading = float(msg.delta_heading) if heading_changed else 0.0
            if not all(math.isfinite(value) for value in delta_xy + (delta_z, delta_heading)):
                self._warn("Ignoring PX4 reset delta because it contains a non-finite value")
            else:
                self.apply_px4_reset(delta_xy, delta_z, delta_heading)
        self.last_xy_reset_counter = int(msg.xy_reset_counter)
        self.last_z_reset_counter = int(msg.z_reset_counter)
        self.last_vxy_reset_counter = int(msg.vxy_reset_counter)
        self.last_vz_reset_counter = int(msg.vz_reset_counter)
        self.last_heading_reset_counter = int(msg.heading_reset_counter)
        self.latest_px4 = msg
        self.latest_px4_sec = self.now_sec()

    def status_callback(self, msg: VehicleStatus) -> None:
        self.latest_status = msg
        self.latest_status_sec = self.now_sec()

    def px4_state_valid(self) -> bool:
        """仅当 PX4 本地状态新鲜、有限且质量检查通过时，才放行 MPC。"""
        if self.latest_px4 is None or self._age(self.latest_px4_sec) > self.px4_timeout_sec:
            return False
        values = (self.latest_px4.x, self.latest_px4.y, self.latest_px4.z, self.latest_px4.vx, self.latest_px4.vy, self.latest_px4.vz)
        heading_valid = not self.use_initial_heading_frame or (
            math.isfinite(float(self.latest_px4.heading)) and self.latest_px4.heading_good_for_control
        )
        accuracy_valid = all(
            math.isfinite(float(value)) and 0.0 <= float(value) <= limit
            for value, limit in ((self.latest_px4.eph, self.max_eph), (self.latest_px4.epv, self.max_epv))
        )
        dead_reckoning_valid = not self.reject_dead_reckoning or not self.latest_px4.dead_reckoning
        return bool(
            all(math.isfinite(float(value)) for value in values)
            and heading_valid
            and accuracy_valid
            and dead_reckoning_valid
            and self.latest_px4.xy_valid
            and self.latest_px4.z_valid
            and self.latest_px4.v_xy_valid
            and self.latest_px4.v_z_valid
        )

    def odom_valid(self) -> bool:
        return self.latest_odom is not None and self._age(self.latest_odom_sec) <= self.odom_timeout_sec

    def try_capture_reference(self) -> None:
        """一次性捕获 EGO 世界坐标与 PX4 NED 的原点及航向对应关系。

        B-spline 位置先相对捕获到的 EGO 原点求位移，PX4 则报告本地 NED 位置。
        同时保存两个原点后，可把规划参考和 PX4 反馈放到同一坐标系中比较，且无需
        修改定位估计器本身。
        """
        if self.reference_world is not None or not self.odom_valid() or not self.px4_state_valid():
            return
        if self.now_sec() < self.reference_capture_ready_sec:
            return
        point = self.latest_odom.pose.pose.position
        self.reference_world = (float(point.x), float(point.y), float(point.z))
        self.reference_px4 = (float(self.latest_px4.x), float(self.latest_px4.y), float(self.latest_px4.z))
        self.reference_heading = self.wrap_angle(float(self.latest_px4.heading)) if self.use_initial_heading_frame else 0.0
        self.takeoff_origin = tuple(self.reference_px4)
        self.takeoff_target = (
            self.reference_px4[0], self.reference_px4[1], self.reference_px4[2] - self.takeoff_altitude
        )
        self.takeoff_command = list(self.takeoff_origin)
        self.hold_position = tuple(self.takeoff_origin)
        self.hold_yaw = self.reference_heading
        self.get_logger().info(
            f"Captured EGO/PX4 reference: world={self.reference_world}, px4_ned={self.reference_px4}, "
            f"heading={self.reference_heading:.3f}"
        )
        if not self.takeoff_before_ego:
            self.publish_takeoff_ready(force=True)

    @staticmethod
    def wrap_angle(value: float) -> float:
        return math.atan2(math.sin(value), math.cos(value))

    def transform_vector(self, vector: Sequence[float]) -> tuple[float, float, float]:
        """将规划器的 ROS/ENU 类向量旋转到配置的 PX4 NED 坐标系。"""
        matrix = self.world_to_px4_rotation
        x, y, z = (float(value) for value in vector)
        mapped = (
            matrix[0] * x + matrix[1] * y + matrix[2] * z,
            matrix[3] * x + matrix[4] * y + matrix[5] * z,
            matrix[6] * x + matrix[7] * y + matrix[8] * z,
        )
        if self.use_initial_heading_frame:
            c, s = math.cos(self.reference_heading), math.sin(self.reference_heading)
            mapped = (c * mapped[0] - s * mapped[1], s * mapped[0] + c * mapped[1], mapped[2])
        return mapped

    def references_in_ned(self, now_sec: float) -> list[tuple[float, ...]]:
        """采样未来 EGO B-spline，并返回 NED 下的 ``[p,v,a]`` 序列。

        位置先减去捕获的 EGO 原点，旋转后再加到 PX4 本地原点；速度和加速度是
        向量，只做旋转，不叠加位置偏移。参考速度限幅用于保护规划器/MPC 共同4
        包线，PX4 实测状态保持原值，不能通过裁剪测量值来掩盖超速。
        """
        if self.latest_reference is None or self.reference_world is None or self.reference_px4 is None:
            raise ValueError("reference trajectory or coordinate alignment is unavailable")
        points = self.latest_reference.sample(now_sec, self.horizon, self.dt)
        result = []
        for point in points:
            world_delta = tuple(point.position[i] - self.reference_world[i] for i in range(3))
            position_delta = self.transform_vector(world_delta)
            position = tuple(self.reference_px4[i] + position_delta[i] for i in range(3))
            velocity = self.transform_vector(point.velocity)
            acceleration = self.transform_vector(point.acceleration)
            # 将期望速度限制在规划器/MPC 的共同包线内；PX4 实际估计值不作修改，
            # 仍以原始值作为 MPC 当前状态。
            speed = math.sqrt(sum(value * value for value in velocity))
            if speed > self.max_velocity:
                velocity = tuple(value * self.max_velocity / speed for value in velocity)
            result.append(position + velocity + acceleration)
        return result

    def current_state(self) -> np.ndarray:
        if not self.px4_state_valid():
            raise ValueError("PX4 local position/velocity is invalid or stale")
        px4 = self.latest_px4
        return np.array([px4.x, px4.y, px4.z, px4.vx, px4.vy, px4.vz], dtype=float)

    def publish_offboard_mode(self, position: bool = False, acceleration: bool = False) -> None:
        """声明本周期唯一启用的 PX4 Offboard 控制接口。"""
        msg = OffboardControlMode()
        msg.timestamp = self.now_us()
        msg.position = bool(position)
        msg.velocity = False
        msg.acceleration = bool(acceleration)
        msg.attitude = False
        msg.body_rate = False
        msg.actuator = False
        self.mode_pub.publish(msg)

    def publish_vehicle_command(self, command: int, param1: float = 0.0, param2: float = 0.0) -> None:
        msg = VehicleCommand()
        msg.timestamp = self.now_us()
        msg.param1, msg.param2 = float(param1), float(param2)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def make_position_setpoint(self, position: Sequence[float], yaw: float) -> TrajectorySetpoint:
        """构造仅位置有效的 PX4 设定值，用于保持、起飞和安全降级。"""
        msg = TrajectorySetpoint()
        msg.timestamp = self.now_us()
        msg.position = [float(value) for value in position]
        msg.velocity = [math.nan, math.nan, math.nan]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.yaw = float(yaw)
        msg.yawspeed = 0.0
        return msg

    def make_mpc_setpoint(self, acceleration: Sequence[float], yaw: float) -> TrajectorySetpoint:
        """构造 MPC 正常模式使用的纯加速度设定值。

        position/velocity 字段有意填写 NaN：PX4 将 NaN 解释为“不控制该状态量”，
        从而避免其位置环和速度环在本节点生成的加速度上再叠加第二套外环反馈。
        """
        msg = TrajectorySetpoint()
        msg.timestamp = self.now_us()
        # 加速度模式下有意不定义位置和速度。PX4 内部继续负责姿态、角速度和电机
        # 控制，本节点只提供平动外环的加速度指令。
        msg.position = [math.nan, math.nan, math.nan]
        msg.velocity = [math.nan, math.nan, math.nan]
        msg.acceleration = [float(value) for value in acceleration]
        msg.yaw = float(yaw)
        msg.yawspeed = 0.0
        return msg

    def publish_takeoff_ready(self, force: bool = False) -> None:
        if not self.takeoff_complete:
            return
        now = self.now_sec()
        if not force and now - self.last_takeoff_ready_sec < 1.0:
            return
        self.takeoff_ready_pub.publish(Bool(data=True))
        self.last_takeoff_ready_sec = now

    def update_takeoff_command(self) -> None:
        if self.takeoff_command is None or self.takeoff_target is None:
            return
        if self.latest_status is None or self.latest_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            return
        step = self.takeoff_vertical_speed / self.control_rate_hz
        delta = self.takeoff_target[2] - self.takeoff_command[2]
        self.takeoff_command[2] = self.takeoff_target[2] if abs(delta) <= step else self.takeoff_command[2] + math.copysign(step, delta)

    def update_takeoff_completion(self) -> None:
        if self.takeoff_complete or self.takeoff_target is None or not self.px4_state_valid():
            return
        px4 = self.latest_px4
        stable = (
            math.hypot(px4.x - self.takeoff_target[0], px4.y - self.takeoff_target[1]) <= self.takeoff_reach_xy_tol
            and abs(px4.z - self.takeoff_target[2]) <= self.takeoff_reach_z_tol
            and math.hypot(px4.vx, px4.vy) <= self.takeoff_speed_xy_tol
            and abs(px4.vz) <= self.takeoff_speed_z_tol
        )
        if not stable:
            self.takeoff_stable_since = None
            return
        if self.takeoff_stable_since is None:
            self.takeoff_stable_since = self.now_sec()
            return
        if self.now_sec() - self.takeoff_stable_since >= self.takeoff_stable_sec:
            self.takeoff_complete = True
            self.hold_position = tuple(self.takeoff_target)
            self.get_logger().info("Takeoff stable; MPC will take over when EGO publishes a valid B-spline.")
            self.publish_takeoff_ready(force=True)

    def make_hold_setpoint(self) -> Optional[TrajectorySetpoint]:
        if self.hold_position is None and self.px4_state_valid():
            px4 = self.latest_px4
            self.hold_position = (float(px4.x), float(px4.y), float(px4.z))
            self.hold_yaw = float(px4.heading) if math.isfinite(float(px4.heading)) else self.reference_heading
        if self.hold_position is None:
            return None
        return self.make_position_setpoint(self.hold_position, self.hold_yaw)

    def publish_hold(self, reason: str) -> None:
        """发生故障后切换到 PX4 位置保持，并清除上一拍加速度历史。"""
        self.previous_acceleration = None
        self.publish_offboard_mode(position=True)
        msg = self.make_hold_setpoint()
        if msg is not None:
            self.setpoint_pub.publish(msg)
            self.setpoint_cycles += 1
        self._warn(reason)

    def trajectory_is_fresh(self) -> bool:
        """结合轨迹声明的起始时刻和持续时间，检查规划结果是否仍然新鲜。"""
        if self.latest_reference is None or self.latest_reference_sec is None:
            return False
        start_time = float(self.latest_reference.start_time_sec)
        duration = float(self.latest_reference.duration)
        if math.isfinite(start_time) and math.isfinite(duration) and start_time > 1e-6 and duration >= 0.0:
            return self.now_sec() <= start_time + duration + self.command_timeout_sec
        return self._age(self.latest_reference_sec) <= self.command_timeout_sec

    def takeoff_is_active(self) -> bool:
        return not self.takeoff_complete and self.takeoff_command is not None

    def publish_takeoff_setpoint(self) -> None:
        """在 MPC 接管前，以有限爬升速率逐步推进位置起飞目标。"""
        if self.takeoff_command is None:
            return
        self.update_takeoff_command()
        self.publish_offboard_mode(position=True)
        self.setpoint_pub.publish(self.make_position_setpoint(self.takeoff_command, self.hold_yaw))
        self.setpoint_cycles += 1

    def manage_px4_state(self) -> None:
        """仅在设定值预发送足够时长后，才按配置请求解锁和进入 Offboard。"""
        if (
            self.latest_status is None
            or self._age(self.latest_status_sec) > self.px4_timeout_sec
            or self.setpoint_cycles < self.offboard_prestream_cycles
        ):
            return
        now = self.now_us()
        armed = self.latest_status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        offboard = self.latest_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        if not armed:
            if self.auto_arm and now - self.last_arm_request_us > 1_000_000:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self.last_arm_request_us = now
            return
        if self.auto_offboard and not offboard and now - self.last_offboard_request_us > 1_000_000:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.last_offboard_request_us = now

    def solve_mpc_or_hold(self) -> Optional[np.ndarray]:
        """执行一次 MPC 求解；任何数值或数据异常都立即改发安全位置保持。"""
        try:
            references = self.references_in_ned(self.now_sec())
            state = self.current_state()
            acceleration = self.mpc.solve(state, references, self.previous_acceleration)
            if not np.all(np.isfinite(acceleration)):
                raise ValueError("MPC acceleration is non-finite")
        except (ValueError, MpcSolveError) as error:
            self.previous_acceleration = None
            reason = f"MPC unavailable ({error})"
            self.publish_hold(reason)
            self.publish_solver_diagnostics(DiagnosticStatus.ERROR, reason, force=True)
            self.manage_px4_state()
            return None
        self.publish_solver_diagnostics(
            DiagnosticStatus.OK,
            "constrained OSQP solution accepted",
        )
        return acceleration

    def timer_callback(self) -> None:
        """按配置的 MPC 频率运行控制状态机。

        检查顺序与安全直接相关：输出门控 -> 定位健康 -> 坐标对齐 -> 起飞 ->
        PX4 状态 -> reset 恢复等待 -> B-spline 新鲜度 -> MPC 求解 ->
        发布纯加速度设定值。任一步失败都会在进入后续阶段前返回。
        """
        if not self.output_permitted:
            return
        self.try_capture_reference()
        if not self.odom_valid():
            self.publish_hold("Waiting for fresh Point-LIO odometry")
            self.manage_px4_state()
            return
        if self.reference_world is None or self.reference_px4 is None:
            self.publish_offboard_mode(position=True)
            self._warn("Waiting for aligned EGO odometry and PX4 local position")
            return
        if self.takeoff_is_active():
            self.previous_acceleration = None
            self.publish_takeoff_setpoint()
            self.manage_px4_state()
            self.update_takeoff_completion()
            return
        self.publish_takeoff_ready()
        if not self.px4_state_valid():
            self.publish_hold("PX4 local position/velocity is stale")
            self.manage_px4_state()
            return
        if self.now_sec() < self.reset_recovery_until_sec:
            self.publish_hold("Waiting for PX4 local-frame reset recovery")
            self.manage_px4_state()
            return
        if not self.trajectory_is_fresh():
            self.publish_hold("Waiting for a fresh EGO B-spline")
            self.manage_px4_state()
            return
        acceleration = self.solve_mpc_or_hold()
        if acceleration is None:
            return
        self.previous_acceleration = acceleration
        yaw = self.reference_heading
        if not self.lock_yaw_to_initial_heading and self.latest_reference is not None:
            point = self.latest_reference.sample(self.now_sec(), 1, self.dt)[0]
            if self.latest_reference.yaw_points:
                yaw = self.wrap_angle(self.yaw_sign * point.yaw + self.yaw_offset + self.reference_heading)
            else:
                velocity = self.transform_vector(point.velocity)
                if math.hypot(velocity[0], velocity[1]) > 0.05:
                    yaw = math.atan2(velocity[1], velocity[0])
        self.publish_offboard_mode(acceleration=True)
        self.setpoint_pub.publish(self.make_mpc_setpoint(acceleration, yaw))
        self.setpoint_cycles += 1
        self.hold_position = None
        self.manage_px4_state()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EgoMpcController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
