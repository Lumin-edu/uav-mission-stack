#!/usr/bin/env python3
"""
EGO 规划器 → PX4 Offboard 桥接节点。

这是 bringup_ego 包的核心节点，负责将 EGO 自主避障规划器的轨迹指令
转换到 PX4 NED 坐标系，并管理完整的 PX4 Offboard 进入/退出流程。

工作流程：
  1. 等待 EGO base 机体中心融合里程计和 PX4 本地位置就绪
  2. 捕获参考原点（EGO 世界系 + PX4 NED 系的对齐点）
  3. 若启用起飞，先飞到 takeoff_altitude 高度并稳定
  4. 起飞完成后，将 EGO 的 PositionCommand 逐步转换为 PX4 TrajectorySetpoint
  5. 如果 EGO 指令短暂丢失，保持最后安全位置（hold）

坐标系说明：
  - EGO 融合里程计使用 ROS 标准系：x=前, y=左, z=上，位置对应 base
  - PX4 NED：x=前(N), y=右(E), z=下(D)
  - 转换矩阵 world_to_px4_rotation = [1,0,0, 0,-1,0, 0,0,-1]
    将 ROS 的 (前,左,上) 映射到 NED 的 (前,右,下)
"""

import math
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from quadrotor_msgs.msg import PositionCommand
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class EgoPx4Bridge(Node):
    """
    EGO → PX4 桥接控制器。

    核心职责：
      - 坐标系转换：ROS标准系(x前,y左,z上) → PX4 NED(x前,y右,z下)
      - Offboard 模式管理：prestream → 解锁 → Offboard → 起飞 → EGO控制
      - 安全保持：EGO指令超时时保持最后安全位置
      - 硬件安全门：双重确认机制防止意外输出
    """

    def __init__(self) -> None:
        super().__init__("ego_px4_bridge")

        # ========== 硬件安全门 ==========
        # 双重确认机制：output_enabled=true AND hardware_confirmation="ENABLE_PX4_OUTPUT"
        # 两者缺一则不会向 PX4 发送任何控制指令
        self.output_enabled = bool(self.declare_parameter("output_enabled", False).value)
        self.hardware_confirmation = str(
            self.declare_parameter("hardware_confirmation", "").value
        )
        self.required_confirmation = str(
            self.declare_parameter("required_confirmation", "ENABLE_PX4_OUTPUT").value
        )
        # ========== 解锁与模式控制 ==========
        self.auto_arm = bool(self.declare_parameter("auto_arm", False).value)          # 是否自动发送解锁指令
        self.auto_offboard = bool(self.declare_parameter("auto_offboard", False).value) # 解锁后是否自动请求Offboard
        self.control_mode = str(
            self.declare_parameter("control_mode", "position").value
        ).strip().lower()                                                              # 控制模式：position 或 velocity
        if self.control_mode == "position_velocity":
            self.get_logger().warn(
                "control_mode='position_velocity' is deprecated; use 'position'. "
                "Position mode already includes EGO velocity and acceleration feed-forward."
            )
            self.control_mode = "position"
        self.control_rate_hz = float(self.declare_parameter("control_rate_hz", 50.0).value)  # 控制频率 (Hz)
        self.offboard_prestream_sec = float(
            self.declare_parameter("offboard_prestream_sec", 2.0).value
        )                                                                                   # Offboard预流时间 (s)
        # ========== 通信超时 ==========
        self.command_timeout_sec = float(
            self.declare_parameter("command_timeout_sec", 0.30).value
        )                                                                                   # EGO指令超时阈值
        self.odom_timeout_sec = float(
            self.declare_parameter("odom_timeout_sec", 0.30).value
        )                                                                                   # EGO里程计超时阈值
        # ========== 速度控制参数 ==========
        self.velocity_position_gain = float(
            self.declare_parameter("velocity_position_gain", 1.0).value
        )                                                                                   # 速度模式下位置误差增益
        self.velocity_limit = float(
            self.declare_parameter("velocity_limit", 1.0).value
        )                                                                                   # 速度模式限幅 (m/s)
        # ========== 起飞参数 ==========
        self.takeoff_before_ego = bool(
            self.declare_parameter("takeoff_before_ego", True).value
        )                                                                                   # EGO控制前是否先起飞
        self.takeoff_altitude = float(
            self.declare_parameter("takeoff_altitude", 0.40).value
        )                                                                                   # 起飞高度 (m)
        self.takeoff_vertical_speed = float(
            self.declare_parameter("takeoff_vertical_speed", 0.20).value
        )                                                                                   # 起飞垂直速度 (m/s)
        self.takeoff_reach_xy_tol = float(
            self.declare_parameter("takeoff_reach_xy_tol", 0.40).value
        )                                                                                   # 起飞水平容差 (m)
        self.takeoff_reach_z_tol = float(
            self.declare_parameter("takeoff_reach_z_tol", 0.10).value
        )                                                                                   # 起飞高度容差 (m)
        self.takeoff_speed_xy_tol = float(
            self.declare_parameter("takeoff_speed_xy_tol", 0.20).value
        )                                                                                   # 起飞水平速度容差 (m/s)
        self.takeoff_speed_z_tol = float(
            self.declare_parameter("takeoff_speed_z_tol", 0.15).value
        )                                                                                   # 起飞垂直速度容差 (m/s)
        self.takeoff_stable_sec = float(
            self.declare_parameter("takeoff_stable_sec", 0.4).value
        )                                                                                   # 起飞稳定时间 (s)
        self.reference_capture_delay_sec = float(
            self.declare_parameter("reference_capture_delay_sec", 3.0).value
        )                                                                                   # 启动后等待捕获参考的时间
        self.takeoff_ready_topic = str(
            self.declare_parameter("takeoff_ready_topic", "/ego/takeoff_ready").value
        )
        self.localization_health_topic = str(
            self.declare_parameter("localization_health_topic", "").value
        )
        if self.control_mode not in {"position", "velocity"}:
            raise ValueError("control_mode must be 'position' or 'velocity'")
        if self.offboard_prestream_sec <= 0.0:
            raise ValueError("offboard_prestream_sec must be positive")
        if self.velocity_position_gain < 0.0:
            raise ValueError("velocity_position_gain must be non-negative")
        if self.velocity_limit <= 0.0:
            raise ValueError("velocity_limit must be positive")
        if self.takeoff_altitude <= 0.0 or self.takeoff_vertical_speed <= 0.0:
            raise ValueError("takeoff_altitude and takeoff_vertical_speed must be positive")
        if min(
            self.takeoff_reach_xy_tol,
            self.takeoff_reach_z_tol,
            self.takeoff_speed_xy_tol,
            self.takeoff_speed_z_tol,
            self.takeoff_stable_sec,
            self.reference_capture_delay_sec,
        ) < 0.0:
            raise ValueError("takeoff tolerances and timing parameters must be non-negative")
        # ========== 坐标系转换参数 ==========
        # 核心：3x3 旋转矩阵（行优先），将 ROS 标准系 (x前,y左,z上) 映射到 PX4 NED (x前,y右,z下)
        # 矩阵含义：
        #   PX4_N(x前) = 1*ROS_x + 0*ROS_y + 0*ROS_z  →  ROS_x(前) → PX4_N(前)
        #   PX4_E(y右) = 0*ROS_x - 1*ROS_y + 0*ROS_z  →  -ROS_y(左) → PX4_E(右)
        #   PX4_D(z下) = 0*ROS_x + 0*ROS_y - 1*ROS_z  →  -ROS_z(上) → PX4_D(下)
        rotation = list(self.declare_parameter("world_to_px4_rotation", [0.0]).value)
        if len(rotation) == 1 and float(rotation[0]) == 0.0:
            rotation = []
        if rotation and len(rotation) != 9:
            raise ValueError("world_to_px4_rotation must contain 9 row-major values")
        self.world_to_px4_rotation = [
            float(value)
            for value in (
                rotation
                if rotation
                else [
                    1.0,  0.0,  0.0,   # PX4_N(前) = ROS_x(前)
                    0.0, -1.0,  0.0,   # PX4_E(右) = -ROS_y(左)
                    0.0,  0.0, -1.0,   # PX4_D(下) = -ROS_z(上)
                ]
            )
        ]
        # ========== 偏航角转换参数 ==========
        # ROS yaw: yaw=0 指向 +x (前方), CCW 为正
        # PX4 heading: yaw=0 指向 N (前), CCW 为正
        # 由于 ROS_x(前) → PX4_N(前) 轴对齐，无需翻转或偏移
        self.yaw_sign = float(self.declare_parameter("yaw_sign", 1.0).value)
        self.yaw_offset = float(
            self.declare_parameter("yaw_offset", 0.0).value
        )
        # ========== 初始航向处理 ==========
        self.use_initial_heading_frame = bool(
            self.declare_parameter("use_initial_heading_frame", False).value
        )                                                                                   # 是否启用初始航向旋转
        self.lock_yaw_to_initial_heading = bool(
            self.declare_parameter("lock_yaw_to_initial_heading", True).value
        )                                                                                   # 是否锁定偏航角到初始航向
        self.align_to_px4_reference = bool(
            self.declare_parameter("align_to_px4_reference", True).value
        )                                                                                   # 是否与PX4参考位置对齐

        self.ego_command_topic = str(
            self.declare_parameter("ego_command_topic", "/ego/position_cmd").value
        )
        self.ego_odom_topic = str(self.declare_parameter("ego_odom_topic", "/odom").value)
        self.px4_position_topic = str(
            self.declare_parameter(
                "px4_position_topic", "/fmu/out/vehicle_local_position"
            ).value
        )
        self.vehicle_status_topic = str(
            self.declare_parameter("vehicle_status_topic", "/fmu/out/vehicle_status").value
        )

        # ========== 硬件安全门检查 ==========
        self.output_permitted = (
            self.output_enabled
            and self.hardware_confirmation == self.required_confirmation
        )
        if self.output_enabled and not self.output_permitted:
            self.get_logger().error(
                "PX4 output blocked: hardware flight requires "
                f"hardware_confirmation={self.required_confirmation!r}."
            )
        elif not self.output_enabled:
            self.get_logger().warn(
                "PX4 output is disabled. The bridge will observe EGO but publish no Offboard messages."
            )
        else:
            self.get_logger().warn(
                "Hardware PX4 output enabled. auto_arm remains disabled unless explicitly requested."
            )

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        command_qos = QoSProfile(depth=10)

        # ========== PX4 Offboard 输出话题 ==========
        self.mode_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", px4_qos
        )
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", px4_qos
        )
        self.command_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", px4_qos)
        ready_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.takeoff_ready_pub = self.create_publisher(
            Bool, self.takeoff_ready_topic, ready_qos
        )
        self.localization_health_required = bool(self.localization_health_topic)
        self.localization_healthy = not self.localization_health_required
        if self.localization_health_required:
            self.create_subscription(
                Bool,
                self.localization_health_topic,
                self.localization_health_callback,
                ready_qos,
            )

        # ========== 订阅话题 ==========
        self.create_subscription(
            PositionCommand, self.ego_command_topic, self.ego_command_callback, command_qos
        )
        self.create_subscription(Odometry, self.ego_odom_topic, self.ego_odom_callback, command_qos)
        self.create_subscription(
            VehicleLocalPosition, self.px4_position_topic, self.px4_position_callback, px4_qos
        )
        self.create_subscription(
            VehicleStatus, self.vehicle_status_topic, self.vehicle_status_callback, px4_qos
        )

        # ========== 状态变量 ==========
        self.latest_command: Optional[PositionCommand] = None     # 最新EGO轨迹指令
        self.latest_command_sec: Optional[float] = None
        self.latest_ego_odom: Optional[Odometry] = None           # 最新 EGO base 融合里程计
        self.latest_ego_odom_sec: Optional[float] = None
        self.latest_px4_position: Optional[VehicleLocalPosition] = None  # 最新PX4本地位置
        self.latest_vehicle_status: Optional[VehicleStatus] = None       # 最新PX4状态
        self.last_px4_z_reset_counter: Optional[int] = None
        # 参考原点（EGO世界系与PX4 NED系的对齐基准）
        self.reference_world: Optional[tuple[float, float, float]] = None
        self.reference_px4: Optional[tuple[float, float, float]] = None
        self.reference_px4_heading: Optional[float] = None
        self.setpoint_cycles = 0                                 # 已发布的setpoint周期数
        self.last_arm_request_us = 0
        self.last_offboard_request_us = 0
        self.last_wait_warning_sec = -math.inf
        self.hold_position_ned: Optional[tuple[float, float, float]] = None  # 保持位置（指令丢失时）
        self.hold_yaw: float = 0.0
        self.reference_capture_ready_sec = self.now_sec() + self.reference_capture_delay_sec
        # 起飞状态变量
        self.takeoff_origin_ned: Optional[tuple[float, float, float]] = None   # 起飞原点(NED)
        self.takeoff_target_ned: Optional[tuple[float, float, float]] = None   # 起飞目标(NED)
        self.takeoff_command_ned: Optional[list[float]] = None                 # 起飞指令位置(NED)
        self.takeoff_complete = not self.takeoff_before_ego                    # 起飞是否完成
        self.takeoff_stable_since_sec: Optional[float] = None
        self.last_takeoff_ready_pub_sec = -math.inf

        if self.control_rate_hz <= 0.0:
            raise ValueError("control_rate_hz must be positive")
        self.offboard_prestream_cycles = max(
            10, int(self.offboard_prestream_sec * self.control_rate_hz)
        )
        self.create_timer(1.0 / self.control_rate_hz, self.timer_callback)
        self.get_logger().info(
            f"PX4 control mode={self.control_mode}, rate={self.control_rate_hz:.1f} Hz, "
            f"Offboard prestream={self.offboard_prestream_sec:.1f} s, "
            "entry_order=arm->Offboard, "
            f"takeoff_before_ego={self.takeoff_before_ego}, "
            f"takeoff_altitude={self.takeoff_altitude:.2f} m, "
            f"lock_yaw_to_initial_heading={self.lock_yaw_to_initial_heading}, "
            f"velocity_position_gain={self.velocity_position_gain:.2f}, "
            f"velocity_limit={self.velocity_limit:.2f} m/s"
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def ego_command_callback(self, msg: PositionCommand) -> None:
        self.latest_command = msg
        self.latest_command_sec = self.now_sec()

    def localization_health_callback(self, msg: Bool) -> None:
        was_healthy = self.localization_healthy
        self.localization_healthy = bool(msg.data)
        if was_healthy and not self.localization_healthy:
            self.get_logger().error(
                "Planner altitude fusion is unhealthy; new EGO trajectory execution is inhibited."
            )

    def ego_odom_callback(self, msg: Odometry) -> None:
        self.latest_ego_odom = msg
        self.latest_ego_odom_sec = self.now_sec()
        self.try_capture_reference()

    def px4_position_callback(self, msg: VehicleLocalPosition) -> None:
        reset_counter = int(msg.z_reset_counter)
        if self.last_px4_z_reset_counter is None:
            self.last_px4_z_reset_counter = reset_counter
        elif reset_counter != self.last_px4_z_reset_counter:
            delta_z = float(msg.delta_z)
            if math.isfinite(delta_z):
                self.apply_px4_z_reset(delta_z)
            self.last_px4_z_reset_counter = reset_counter
        self.latest_px4_position = msg
        self.try_capture_reference()

    def apply_px4_z_reset(self, delta_z: float) -> None:
        """Move stored NED setpoints with a PX4 vertical-origin reset."""
        if self.reference_px4 is not None:
            self.reference_px4 = (
                self.reference_px4[0],
                self.reference_px4[1],
                self.reference_px4[2] + delta_z,
            )
        if self.takeoff_origin_ned is not None:
            self.takeoff_origin_ned = (
                self.takeoff_origin_ned[0],
                self.takeoff_origin_ned[1],
                self.takeoff_origin_ned[2] + delta_z,
            )
        if self.takeoff_target_ned is not None:
            self.takeoff_target_ned = (
                self.takeoff_target_ned[0],
                self.takeoff_target_ned[1],
                self.takeoff_target_ned[2] + delta_z,
            )
        if self.takeoff_command_ned is not None:
            self.takeoff_command_ned[2] += delta_z
        if self.hold_position_ned is not None:
            self.hold_position_ned = (
                self.hold_position_ned[0],
                self.hold_position_ned[1],
                self.hold_position_ned[2] + delta_z,
            )
        self.get_logger().warn(
            f"PX4 z reset detected; shifted stored NED setpoints by {delta_z:+.3f} m."
        )

    def vehicle_status_callback(self, msg: VehicleStatus) -> None:
        self.latest_vehicle_status = msg

    def px4_reference_is_valid(self) -> bool:
        msg = self.latest_px4_position
        if msg is None or not (msg.xy_valid and msg.z_valid):
            return False
        return all(math.isfinite(value) for value in (msg.x, msg.y, msg.z))

    def px4_heading_is_valid(self) -> bool:
        return self.latest_px4_position is not None and math.isfinite(
            float(self.latest_px4_position.heading)
        )

    def try_capture_reference(self) -> None:
        """
        捕获 EGO 世界系与 PX4 NED 系的参考原点。

        当 EGO base 融合里程计和 PX4 本地位置都可用时，记录：
          - reference_world: EGO 世界系下的起始位置
          - reference_px4: PX4 NED 下的起始位置
          - reference_px4_heading: PX4 初始偏航角

        同时初始化起飞原点和目标（target = origin - takeoff_altitude）。
        """
        if self.reference_world is not None or self.latest_ego_odom is None:
            return
        if not self.localization_healthy:
            return
        if self.now_sec() < self.reference_capture_ready_sec:
            return
        if self.align_to_px4_reference and not self.px4_reference_is_valid():
            return
        if self.use_initial_heading_frame and not self.px4_heading_is_valid():
            return

        position = self.latest_ego_odom.pose.pose.position
        self.reference_world = (float(position.x), float(position.y), float(position.z))
        if self.align_to_px4_reference:
            px4 = self.latest_px4_position
            self.reference_px4 = (float(px4.x), float(px4.y), float(px4.z))
        else:
            self.reference_px4 = (0.0, 0.0, 0.0)
        if self.use_initial_heading_frame:
            px4 = self.latest_px4_position
            self.reference_px4_heading = self.wrap_angle(float(px4.heading))
        else:
            self.reference_px4_heading = 0.0
        # 起飞原点 = 当前 PX4 位置，目标 = 原点 - 起飞高度（NED中D轴向下为正）
        self.takeoff_origin_ned = tuple(self.reference_px4)
        self.takeoff_target_ned = (
            self.reference_px4[0],
            self.reference_px4[1],
            self.reference_px4[2] - self.takeoff_altitude,  # NED下：z减小=上升
        )
        self.takeoff_command_ned = list(self.takeoff_origin_ned)
        self.hold_position_ned = tuple(self.takeoff_origin_ned)
        self.hold_yaw = self.reference_px4_heading or 0.0
        self.setpoint_cycles = 0
        self.get_logger().info(
            "Captured EGO-to-PX4 reference: "
            f"world={self.reference_world}, px4_ned={self.reference_px4}, "
            f"initial_heading={self.reference_px4_heading:.3f}"
        )
        if self.takeoff_before_ego:
            self.get_logger().info(
                "Hardware takeoff target captured: "
                f"origin_ned={self.takeoff_origin_ned}, "
                f"target_ned={self.takeoff_target_ned}"
            )
        else:
            self.publish_takeoff_ready(force=True)

    @staticmethod
    def wrap_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def transform_vector(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        """
        将 ROS 标准系向量转换到 PX4 NED 坐标系。

        两步变换：
          1. world_to_px4_rotation 矩阵：轴重映射 (x前,y左,z上) → (x前,y右,z下)
          2. 若 use_initial_heading_frame，绕初始 PX4 偏航角旋转
        """
        matrix = self.world_to_px4_rotation
        # 第一步：矩阵乘法做轴重映射
        mapped = (
            matrix[0] * x + matrix[1] * y + matrix[2] * z,
            matrix[3] * x + matrix[4] * y + matrix[5] * z,
            matrix[6] * x + matrix[7] * y + matrix[8] * z,
        )
        # 第二步：绕初始航向旋转（使坐标系对齐到 PX4 NED 的绝对方向）
        if self.use_initial_heading_frame:
            heading = self.reference_px4_heading or 0.0
            c = math.cos(heading)
            s = math.sin(heading)
            mapped = (
                c * mapped[0] - s * mapped[1],
                s * mapped[0] + c * mapped[1],
                mapped[2],
            )
        return mapped

    def velocity_command_world(self, command: PositionCommand) -> Optional[tuple[float, float, float]]:
        if self.latest_ego_odom is None:
            return None
        current = self.latest_ego_odom.pose.pose.position
        values = (current.x, current.y, current.z)
        if not all(math.isfinite(float(value)) for value in values):
            self.get_logger().error("Dropped velocity command because EGO odometry is non-finite.")
            return None

        error = (
            float(command.position.x) - float(current.x),
            float(command.position.y) - float(current.y),
            float(command.position.z) - float(current.z),
        )
        velocity = (
            float(command.velocity.x) + self.velocity_position_gain * error[0],
            float(command.velocity.y) + self.velocity_position_gain * error[1],
            float(command.velocity.z) + self.velocity_position_gain * error[2],
        )
        norm = math.sqrt(sum(value * value for value in velocity))
        if norm > self.velocity_limit:
            scale = self.velocity_limit / norm
            velocity = tuple(value * scale for value in velocity)
        return velocity

    def make_setpoint(self, command: PositionCommand) -> Optional[TrajectorySetpoint]:
        """
        将 EGO PositionCommand 转换为 PX4 TrajectorySetpoint。

        position 模式下发送完整的 位置+速度+加速度 前馈：
          1. 计算 EGO 指令位置相对于参考原点的增量
          2. 用 transform_vector 转换位置/速度/加速度到 NED
          3. 组装 TrajectorySetpoint 消息

        velocity 模式下无视位置字段，只发送速度指令。
        """
        if self.reference_world is None or self.reference_px4 is None:
            return None
        p = command.position
        v = command.velocity
        a = command.acceleration
        values = (p.x, p.y, p.z, v.x, v.y, v.z, a.x, a.y, a.z, command.yaw, command.yaw_dot)
        if not all(math.isfinite(float(value)) for value in values):
            self.get_logger().error("Dropped EGO command containing non-finite values.")
            return None

        msg = TrajectorySetpoint()
        msg.timestamp = self.now_us()
        if self.control_mode == "velocity":
            velocity_world = self.velocity_command_world(command)
            if velocity_world is None:
                return None
            msg.position = [math.nan, math.nan, math.nan]
            msg.velocity = list(self.transform_vector(*velocity_world))
            msg.acceleration = [math.nan, math.nan, math.nan]
        else:
            # 计算 EGO 世界系下的位置增量（相对于参考原点）
            world_delta = (
                float(p.x) - self.reference_world[0],
                float(p.y) - self.reference_world[1],
                float(p.z) - self.reference_world[2],
            )
            # 通过旋转矩阵转换位置/速度/加速度到 NED
            position_delta = self.transform_vector(*world_delta)
            velocity = self.transform_vector(float(v.x), float(v.y), float(v.z))
            acceleration = self.transform_vector(float(a.x), float(a.y), float(a.z))
            # 最终 NED 位置 = PX4参考原点 + 转换后的增量
            msg.position = [
                self.reference_px4[0] + position_delta[0],
                self.reference_px4[1] + position_delta[1],
                self.reference_px4[2] + position_delta[2],
            ]
            msg.velocity = list(velocity)
            msg.acceleration = list(acceleration)
        # yaw 处理：可锁定到初始航向或跟随 EGO 指令
        initial_heading = self.reference_px4_heading if self.use_initial_heading_frame else 0.0
        if self.lock_yaw_to_initial_heading:
            msg.yaw = self.wrap_angle(float(initial_heading or 0.0))
            msg.yawspeed = 0.0
        else:
            msg.yaw = self.wrap_angle(
                self.yaw_sign * float(command.yaw)
                + self.yaw_offset
                + (initial_heading or 0.0)
            )
            msg.yawspeed = self.yaw_sign * float(command.yaw_dot)
        return msg

    def publish_offboard_mode(self, force_position: bool = False) -> None:
        msg = OffboardControlMode()
        msg.timestamp = self.now_us()
        # PX4 uses the first true field as the requested control level. In
        # position mode, velocity and acceleration remain populated in the
        # TrajectorySetpoint as feed-forward terms; their mode flags stay false.
        msg.position = force_position or self.control_mode == "position"
        msg.velocity = not force_position and self.control_mode == "velocity"
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.actuator = False
        self.mode_pub.publish(msg)

    def publish_vehicle_command(self, command: int, param1: float = 0.0, param2: float = 0.0) -> None:
        msg = VehicleCommand()
        msg.timestamp = self.now_us()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def publish_takeoff_ready(self, force: bool = False) -> None:
        if not self.takeoff_complete:
            return
        now = self.now_sec()
        if not force and now - self.last_takeoff_ready_pub_sec < 1.0:
            return
        msg = Bool()
        msg.data = True
        self.takeoff_ready_pub.publish(msg)
        self.last_takeoff_ready_pub_sec = now

    def make_position_setpoint(
        self, position: tuple[float, float, float] | list[float], yaw: float
    ) -> TrajectorySetpoint:
        msg = TrajectorySetpoint()
        msg.timestamp = self.now_us()
        msg.position = [float(value) for value in position]
        msg.velocity = [math.nan, math.nan, math.nan]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.yaw = float(yaw)
        msg.yawspeed = 0.0
        return msg

    def update_takeoff_command(self) -> None:
        if self.takeoff_command_ned is None or self.takeoff_target_ned is None:
            return
        offboard = bool(
            self.latest_vehicle_status is not None
            and self.latest_vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        )
        if not offboard:
            return
        step = self.takeoff_vertical_speed / self.control_rate_hz
        dz = self.takeoff_target_ned[2] - self.takeoff_command_ned[2]
        if abs(dz) <= step:
            self.takeoff_command_ned[2] = self.takeoff_target_ned[2]
        else:
            self.takeoff_command_ned[2] += math.copysign(step, dz)

    def takeoff_is_stable(self) -> bool:
        if self.latest_px4_position is None or self.takeoff_target_ned is None:
            return False
        px4 = self.latest_px4_position
        values = (px4.x, px4.y, px4.z, px4.vx, px4.vy, px4.vz)
        if not all(math.isfinite(float(value)) for value in values):
            return False
        return (
            math.hypot(px4.x - self.takeoff_target_ned[0], px4.y - self.takeoff_target_ned[1])
            <= self.takeoff_reach_xy_tol
            and abs(px4.z - self.takeoff_target_ned[2]) <= self.takeoff_reach_z_tol
            and math.hypot(px4.vx, px4.vy) <= self.takeoff_speed_xy_tol
            and abs(px4.vz) <= self.takeoff_speed_z_tol
        )

    def update_takeoff_completion(self) -> None:
        if self.takeoff_complete:
            return
        if not self.takeoff_is_stable():
            self.takeoff_stable_since_sec = None
            return
        if self.takeoff_stable_since_sec is None:
            self.takeoff_stable_since_sec = self.now_sec()
            self.get_logger().info(
                f"Takeoff altitude reached; verifying stability for {self.takeoff_stable_sec:.1f} s."
            )
            return
        if self.now_sec() - self.takeoff_stable_since_sec < self.takeoff_stable_sec:
            return
        self.takeoff_complete = True
        self.hold_position_ned = tuple(self.takeoff_target_ned)
        self.get_logger().info("Takeoff complete. EGO goal and trajectory control are now enabled.")
        self.publish_takeoff_ready(force=True)

    def publish_takeoff_setpoint(self) -> None:
        if self.takeoff_command_ned is None:
            return
        self.update_takeoff_command()
        self.publish_offboard_mode(force_position=True)
        self.setpoint_pub.publish(
            self.make_position_setpoint(self.takeoff_command_ned, self.hold_yaw)
        )
        self.setpoint_cycles += 1

    def command_is_fresh(self) -> bool:
        return (
            self.latest_command is not None
            and self.latest_command_sec is not None
            and self.now_sec() - self.latest_command_sec <= self.command_timeout_sec
        )

    def odom_is_fresh(self) -> bool:
        return (
            self.latest_ego_odom is not None
            and self.latest_ego_odom_sec is not None
            and self.now_sec() - self.latest_ego_odom_sec <= self.odom_timeout_sec
        )

    def warn_waiting(self, message: str) -> None:
        now = self.now_sec()
        if now - self.last_wait_warning_sec >= 2.0:
            self.last_wait_warning_sec = now
            self.get_logger().warn(message)

    def make_hold_setpoint(self) -> Optional[TrajectorySetpoint]:
        """
        生成保持位置的 setpoint（EGO 指令丢失时使用）。

        优先级：
          1. 使用已存储的 hold_position_ned
          2. 如果 PX4 位置有效，用当前 PX4 位置
          3. 如果能从最近的 EGO 指令推算，用推算位置
          4. 最后回退到参考原点
        """
        if self.hold_position_ned is None:
            px4 = self.latest_px4_position
            if self.px4_reference_is_valid() and px4 is not None:
                self.hold_position_ned = (float(px4.x), float(px4.y), float(px4.z))
                if self.px4_heading_is_valid():
                    self.hold_yaw = self.wrap_angle(float(px4.heading))
            elif self.latest_command is not None and self.reference_world is not None:
                candidate = self.make_setpoint(self.latest_command)
                if candidate is not None and all(math.isfinite(float(v)) for v in candidate.position):
                    self.hold_position_ned = tuple(float(v) for v in candidate.position)
                    self.hold_yaw = float(candidate.yaw)
            elif self.reference_px4 is not None:
                self.hold_position_ned = tuple(float(v) for v in self.reference_px4)
        if self.hold_position_ned is None:
            return None
        msg = TrajectorySetpoint()
        msg.timestamp = self.now_us()
        msg.position = list(self.hold_position_ned)
        msg.velocity = [math.nan, math.nan, math.nan]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.yaw = self.hold_yaw
        msg.yawspeed = 0.0
        return msg

    def publish_hold(self, reason: str) -> None:
        self.publish_offboard_mode()
        hold = self.make_hold_setpoint()
        if hold is not None:
            self.setpoint_pub.publish(hold)
            self.setpoint_cycles += 1
        self.warn_waiting(f"{reason}; holding the last safe PX4 position.")

    def manage_px4_state(self) -> None:
        """
        管理 PX4 的解锁和 Offboard 模式切换。

        与正方形任务验证过的流程一致：
          1. prestream：先发足够多的 setpoint 心跳
          2. 等待遥控器解锁（或 auto_arm 自动解锁）
          3. 解锁后请求进入 Offboard 模式
        """
        if (
            self.latest_vehicle_status is None
            or self.setpoint_cycles < self.offboard_prestream_cycles
        ):
            return
        now_us = self.now_us()
        armed = self.latest_vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        offboard = (
            self.latest_vehicle_status.nav_state
            == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        )

        # Match the validated square mission. Heartbeats and setpoints prestream
        # while disarmed, then Offboard is requested only after an RC/manual
        # (or explicitly enabled automatic) arm.
        if not armed:
            if self.auto_arm and now_us - self.last_arm_request_us > 1_000_000:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self.last_arm_request_us = now_us
                self.get_logger().warn("Sent PX4 arm request before Offboard entry.")
            return

        if self.auto_offboard and not offboard and now_us - self.last_offboard_request_us > 1_000_000:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.last_offboard_request_us = now_us
            self.get_logger().warn("Sent PX4 Offboard request after arm confirmation.")

    def timer_callback(self) -> None:
        """
        主控制循环（默认 50Hz）。

        状态机流程：
          1. 硬件安全门未通过 → 不输出
          2. 参考原点未捕获 → 等待 + prestream 心跳
          3. 起飞未完成 → 执行起飞流程
          4. EGO 指令/里程计超时 → 保持最后安全位置（hold）
          5. 正常模式 → 将 EGO PositionCommand 转换为 PX4 setpoint 并发布
        """
        # 步骤1：硬件安全门检查
        if not self.output_permitted:
            return
        if not self.localization_healthy:
            if self.reference_world is not None:
                self.publish_hold("Planner altitude fusion is unhealthy")
            else:
                self.warn_waiting("Waiting for healthy planner altitude fusion")
            return
        # 步骤2：等待参考原点
        if self.reference_world is None:
            self.publish_offboard_mode(force_position=True)
            self.warn_waiting("Waiting for EGO base odometry and a valid PX4 takeoff reference")
            return
        # 步骤3：起飞流程
        if not self.takeoff_complete:
            self.publish_takeoff_setpoint()
            self.manage_px4_state()
            self.update_takeoff_completion()
            return
        # 起飞完成后持续发布就绪信号
        self.publish_takeoff_ready()
        # 步骤4：EGO指令超时 → 保持安全位置
        if not self.command_is_fresh():
            self.publish_hold("Waiting for a fresh EGO PositionCommand")
            self.manage_px4_state()
            return
        if not self.odom_is_fresh():
            self.publish_hold("Waiting for fresh EGO odometry before publishing PX4 setpoints")
            self.manage_px4_state()
            return
        if self.reference_world is None:
            self.publish_hold("Waiting for aligned EGO odometry and valid PX4 local position")
            self.manage_px4_state()
            return

        # 步骤5：正常模式 — 转换并发布 EGO 指令
        setpoint = self.make_setpoint(self.latest_command)
        if setpoint is None:
            return
        self.publish_offboard_mode()
        self.setpoint_pub.publish(setpoint)
        self.setpoint_cycles += 1
        self.hold_position_ned = None
        self.manage_px4_state()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EgoPx4Bridge()
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
