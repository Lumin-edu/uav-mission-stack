#!/usr/bin/env python3
"""EGO 到达最终目标点后，向 PX4 请求自动降落。

这个节点只是“到点检测器 + 降落状态协调器”，不是第二个飞行控制器：

1. 订阅 ``/move_base_simple/goal``，记录 EGO 当前最终目标；
2. 订阅杆臂补偿后的 ``/ego/odom_base``，用无人机中心位姿判断是否到点；
3. 位置误差和速度连续满足阈值后，发送一次 ``VEHICLE_CMD_NAV_LAND``；
4. 降落过程中发布 ``/ego/landing_requested=true``，通知桥接节点不要重进 Offboard；
5. 等 PX4 明确报告 ``landed=true`` 后发布 ``/ego/landing_complete=true``；
6. ``ego_px4_bridge`` 收到完成信号后，才停止 Offboard 心跳和轨迹设定值。

因此，本节点不会发布 ``TrajectorySetpoint``，实际下降轨迹和落地检测均由 PX4
的 AUTO_LAND 状态机负责。PX4 的 ``NAV_LAND`` 语义是“在当前位置降落”，
不是 ``NAV_RETURN_TO_LAUNCH``；命令被 PX4 接受时的当前 XY 就是目标点 XY。
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleCommand, VehicleLandDetected, VehicleStatus
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class AutoLandAfterGoal(Node):
    """带硬件安全门的到点检测和 PX4 降落命令节点。"""

    def __init__(self) -> None:
        super().__init__("auto_land_after_goal")

        # ========== 1. PX4 输出安全门 ==========
        # 必须同时满足：
        #   output_enabled == true
        #   hardware_confirmation == "ENABLE_PX4_OUTPUT"
        # 才允许向 /fmu/in/vehicle_command 发送 NAV_LAND。
        # 该安全门与 ego_px4_bridge 完全一致，防止台架调试时残留目标消息误触发降落。
        self.output_enabled = bool(self.declare_parameter("output_enabled", False).value)
        self.hardware_confirmation = str(self.declare_parameter("hardware_confirmation", "").value)
        self.required_confirmation = str(
            self.declare_parameter("required_confirmation", "ENABLE_PX4_OUTPUT").value
        )
        self.output_permitted = (
            self.output_enabled and self.hardware_confirmation == self.required_confirmation
        )

        # ========== 2. 输入、输出话题 ==========
        # require_takeoff_ready=true 时，必须先收到桥接节点发布的起飞完成信号，
        # 这样启动阶段即使当前位置恰好落在目标容差内，也不会在起飞前请求降落。
        self.require_takeoff_ready = bool(self.declare_parameter("require_takeoff_ready", True).value)
        self.goal_topic = str(self.declare_parameter("goal_topic", "/move_base_simple/goal").value)
        self.odom_topic = str(self.declare_parameter("odom_topic", "/ego/odom_base").value)
        self.status_topic = str(
            self.declare_parameter("vehicle_status_topic", "/fmu/out/vehicle_status").value
        )
        self.land_detected_topic = str(
            self.declare_parameter("vehicle_land_detected_topic", "/fmu/out/vehicle_land_detected").value
        )
        self.takeoff_ready_topic = str(
            self.declare_parameter("takeoff_ready_topic", "/ego/takeoff_ready").value
        )
        self.landing_requested_topic = str(
            self.declare_parameter("landing_requested_topic", "/ego/landing_requested").value
        )
        self.landing_complete_topic = str(
            self.declare_parameter("landing_complete_topic", "/ego/landing_complete").value
        )

        # ========== 3. 到点判定参数 ==========
        # 只有“位置接近目标”且“飞机速度已经足够小”才算稳定到点。
        # 单看位置误差可能在飞机高速穿过目标点时误判，因此必须同时检查速度。
        self.goal_xy_tolerance = float(self.declare_parameter("goal_xy_tolerance", 0.25).value)
        self.goal_z_tolerance = float(self.declare_parameter("goal_z_tolerance", 0.15).value)
        self.goal_speed_xy_tolerance = float(
            self.declare_parameter("goal_speed_xy_tolerance", 0.15).value
        )
        self.goal_speed_z_tolerance = float(
            self.declare_parameter("goal_speed_z_tolerance", 0.15).value
        )
        self.goal_stable_sec = float(self.declare_parameter("goal_stable_sec", 1.0).value)
        self.odom_timeout_sec = float(self.declare_parameter("odom_timeout_sec", 0.30).value)
        self.land_command_period_sec = float(
            self.declare_parameter("land_command_period_sec", 1.0).value
        )
        self.publish_complete_period_sec = float(
            self.declare_parameter("publish_complete_period_sec", 0.5).value
        )
        if min(
            self.goal_xy_tolerance,
            self.goal_z_tolerance,
            self.goal_speed_xy_tolerance,
            self.goal_speed_z_tolerance,
            self.goal_stable_sec,
        ) < 0.0:
            raise ValueError("auto-land tolerances and stable time must be non-negative")
        if min(
            self.odom_timeout_sec,
            self.land_command_period_sec,
            self.publish_complete_period_sec,
        ) <= 0.0:
            raise ValueError("auto-land timeout and publish periods must be positive")

        # ========== 4. ROS 2 QoS ==========
        # 高频传感器/状态允许 BEST_EFFORT，优先保证实时性。
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # requested/complete 是低频状态信号，使用 RELIABLE + TRANSIENT_LOCAL：
        # 新启动或短暂重连的桥接节点也能拿到最近一次 true 状态。
        signal_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ========== 5. 发布与订阅关系 ==========
        # 唯一直接送给 PX4 的输出：VEHICLE_CMD_NAV_LAND。
        self.command_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", px4_qos)
        # requested=true：已经开始降落，桥接节点必须禁止正常状态机重进 Offboard。
        self.landing_requested_pub = self.create_publisher(
            Bool, self.landing_requested_topic, signal_qos
        )
        # complete=true：PX4 已确认落地，桥接节点此时才可以停止 Offboard 心跳。
        self.landing_complete_pub = self.create_publisher(
            Bool, self.landing_complete_topic, signal_qos
        )
        # goal 与 odom 必须处于同一个 EGO/odom 世界坐标系。
        # launch passes the Point-LIO odometry topic; compare its position and velocity directly.
        self.create_subscription(PoseStamped, self.goal_topic, self.goal_callback, sensor_qos)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, sensor_qos)
        self.create_subscription(VehicleStatus, self.status_topic, self.status_callback, px4_qos)
        self.create_subscription(
            VehicleLandDetected,
            self.land_detected_topic,
            self.land_detected_callback,
            px4_qos,
        )
        self.create_subscription(
            Bool, self.takeoff_ready_topic, self.takeoff_ready_callback, signal_qos
        )

        # ========== 6. 自动降落状态机变量 ==========
        # 状态 A：landing_requested=false, landing_complete=false，等待并检测目标。
        # 状态 B：landing_requested=true,  landing_complete=false，PX4 正在降落。
        # 状态 C：landing_complete=true，PX4 已落地，持续发布完成信号。
        self.goal: Optional[tuple[float, float, float]] = None
        self.latest_odom: Optional[Odometry] = None
        self.latest_odom_sec: Optional[float] = None
        self.latest_status: Optional[VehicleStatus] = None
        self.takeoff_ready = not self.require_takeoff_ready
        # 注意：PX4 开机停在地面时 landed 本来就是 true，不能直接把它当作本次任务完成。
        self.landed = False
        self.landing_requested = False
        self.landing_complete = False
        self.goal_stable_since_sec: Optional[float] = None
        self.last_land_command_sec = -math.inf
        self.last_complete_publish_sec = -math.inf
        self.last_warning_sec = -math.inf
        self.create_timer(0.1, self.timer_callback)

        if not self.output_enabled:
            self.get_logger().warn("Auto-land output disabled; no PX4 land command will be sent.")
        elif not self.output_permitted:
            self.get_logger().error("Auto-land blocked by hardware_confirmation safety gate.")
        else:
            self.get_logger().info("Auto-land output enabled; waiting for a stable EGO goal.")

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def warn(self, message: str) -> None:
        """将重复警告限制到 2 秒一次，避免 10 Hz 定时器刷屏。"""
        now = self.now_sec()
        if now - self.last_warning_sec >= 2.0:
            self.last_warning_sec = now
            self.get_logger().warn(message)

    def goal_callback(self, msg: PoseStamped) -> None:
        """接收最终目标；新目标会重新开始“连续稳定时间”的计时。"""
        p = msg.pose.position
        values = (p.x, p.y, p.z)
        if not all(math.isfinite(float(value)) for value in values):
            self.warn("Ignoring a non-finite EGO goal.")
            return
        # 已经进入降落阶段后忽略新目标，避免下降途中目标话题更新导致状态机重新开始。
        if self.landing_requested or self.landing_complete:
            return
        self.goal = (float(p.x), float(p.y), float(p.z))
        self.goal_stable_since_sec = None
        self.get_logger().info(
            f"Tracking EGO goal for auto-land: ({self.goal[0]:.2f}, {self.goal[1]:.2f}, {self.goal[2]:.2f})"
        )

    def odom_callback(self, msg: Odometry) -> None:
        """缓存最新 base 机体中心位姿，并记录本机接收时间用于超时检查。"""
        p = msg.pose.pose.position
        values = (p.x, p.y, p.z)
        if all(math.isfinite(float(value)) for value in values):
            self.latest_odom = msg
            self.latest_odom_sec = self.now_sec()

    def status_callback(self, msg: VehicleStatus) -> None:
        """缓存 PX4 解锁状态和当前导航模式。"""
        self.latest_status = msg

    def takeoff_ready_callback(self, msg: Bool) -> None:
        """接收 ego_px4_bridge 的起飞稳定完成信号。"""
        self.takeoff_ready = bool(msg.data)

    def land_detected_callback(self, msg: VehicleLandDetected) -> None:
        """只在本节点已发出 NAV_LAND 后，接受 PX4 的落地确认。"""
        self.landed = bool(msg.landed)
        # 关键保护：PX4 启动时飞机就在地面，通常一开始就会发布 landed=true。
        # 如果缺少 landing_requested 条件，桥接节点会在起飞前被误通知“降落完成”，
        # 从而停止 Offboard 心跳。只有发出本次 NAV_LAND 后的 landed=true 才有效。
        if self.landing_requested and self.landed and not self.landing_complete:
            self.get_logger().info(
                "PX4 landing detector reports landed=true; stopping Offboard heartbeat."
            )
            self.landing_complete = True

    def odom_is_fresh(self) -> bool:
        """确保到点判断使用的是新鲜里程计，而不是断流前的旧位置。"""
        return (
            self.latest_odom is not None
            and self.latest_odom_sec is not None
            and self.now_sec() - self.latest_odom_sec <= self.odom_timeout_sec
        )

    def vehicle_is_armed(self) -> bool:
        """只有 PX4 已解锁时才允许发起本次自动降落。"""
        return bool(
            self.latest_status is not None
            and self.latest_status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        )

    def goal_is_stable(self) -> bool:
        """检查一次采样是否同时满足位置误差和速度阈值。

        判据为：
          sqrt((x-goal_x)^2 + (y-goal_y)^2) <= XY 位置容差
          abs(z-goal_z)                         <= Z 位置容差
          sqrt(vx^2 + vy^2)                    <= 水平速度容差
          abs(vz)                              <= 垂直速度容差
        """
        if self.goal is None or not self.odom_is_fresh() or self.latest_odom is None:
            return False
        p = self.latest_odom.pose.pose.position
        v = self.latest_odom.twist.twist.linear
        if not all(math.isfinite(float(value)) for value in (p.x, p.y, p.z, v.x, v.y, v.z)):
            return False
        xy_error = math.hypot(float(p.x) - self.goal[0], float(p.y) - self.goal[1])
        z_error = abs(float(p.z) - self.goal[2])
        xy_speed = math.hypot(float(v.x), float(v.y))
        return (
            xy_error <= self.goal_xy_tolerance
            and z_error <= self.goal_z_tolerance
            and xy_speed <= self.goal_speed_xy_tolerance
            and abs(float(v.z)) <= self.goal_speed_z_tolerance
        )

    def publish_land_command(self) -> None:
        """请求 PX4 切换到 AUTO_LAND，并通知桥接节点进入降落阶段。"""
        first_request = not self.landing_requested
        msg = VehicleCommand()
        msg.timestamp = self.now_us()
        msg.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        # PX4 的 VEHICLE_CMD_NAV_LAND 会在 Commander 中切换到 AUTO_LAND，
        # Navigator 随后将“当前全局位置”保存为降落点并保持该 XY 下降。
        # 因此这里不填写起飞点、Home 点或返航点；param5/6/7 为 0 不表示回到起飞点。
        # 由于本函数只会在无人机中心里程计已经稳定到目标时调用，命令接受瞬间的
        # 当前 XY 就是目标点 XY，也就是“目标点正下方”降落。
        # 不在此处生成下降位置/速度设定值，避免出现第二个轨迹控制源。
        msg.param1 = 0.0
        msg.param2 = 0.0
        msg.param3 = 0.0
        msg.param4 = 0.0
        msg.param5 = 0.0
        msg.param6 = 0.0
        msg.param7 = 0.0
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)
        # 同一周期通知 ego_px4_bridge：继续维持当前心跳，但停止 Offboard 重入请求。
        self.landing_requested_pub.publish(Bool(data=True))
        self.landing_requested = True
        self.last_land_command_sec = self.now_sec()
        if first_request:
            self.get_logger().warn(
                "Sent PX4 VEHICLE_CMD_NAV_LAND after reaching the EGO goal."
            )

    def publish_complete(self) -> None:
        """周期性发布已落地信号，供桥接节点锁存并关闭 Offboard 输出。"""
        now = self.now_sec()
        if now - self.last_complete_publish_sec < self.publish_complete_period_sec:
            return
        self.landing_complete_pub.publish(Bool(data=True))
        self.last_complete_publish_sec = now

    def timer_callback(self) -> None:
        """10 Hz 自动降落主状态机。

        优先级从高到低：
          已落地 -> 重发完成信号；
          安全门未通过 -> 不输出；
          已请求降落 -> 等待 AUTO_LAND/landed，必要时低频重发命令；
          尚未请求 -> 检查目标、起飞、解锁、到点和连续稳定时间。
        """
        if self.landing_complete:
            # 虽然 TRANSIENT_LOCAL 会保留最后状态，仍以低频重复发布 true，
            # 提高节点重连或 DDS 短暂异常后的状态同步可靠性。
            self.publish_complete()
            return
        if not self.output_permitted:
            return
        if self.landing_requested:
            # NAV_LAND 是离散命令，不应按 10 Hz 高频轰炸。若 PX4 尚未进入
            # AUTO_LAND，则按 land_command_period_sec（默认 1 秒）低频重发。
            auto_land_active = bool(
                self.latest_status is not None
                and self.latest_status.nav_state
                == VehicleStatus.NAVIGATION_STATE_AUTO_LAND
            )
            if (
                not auto_land_active
                and self.now_sec() - self.last_land_command_sec
                >= self.land_command_period_sec
            ):
                self.publish_land_command()
            return
        if self.goal is None:
            return
        if not self.takeoff_ready:
            self.warn("Waiting for takeoff_ready before evaluating the auto-land goal.")
            return
        if not self.vehicle_is_armed():
            self.warn("Waiting for an armed PX4 vehicle before auto-land.")
            return
        if not self.goal_is_stable():
            # 任一位置/速度条件被破坏就清零计时。飞机必须“连续”稳定，
            # 短暂穿过容差圈或测量抖动不会触发降落。
            self.goal_stable_since_sec = None
            return
        if self.goal_stable_since_sec is None:
            self.goal_stable_since_sec = self.now_sec()
            self.get_logger().info(
                f"Goal reached; verifying stability for {self.goal_stable_sec:.1f} s before landing."
            )
            return
        # 默认连续满足 1 秒后才真正发送 NAV_LAND。
        if self.now_sec() - self.goal_stable_since_sec >= self.goal_stable_sec:
            self.publish_land_command()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AutoLandAfterGoal()
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
