#!/usr/bin/env python3
"""
EGO 硬件管线诊断监控节点。

监控 EGO 自主避障完整数据链路的状态：
  1. Point-LIO 里程计 (/odom)
  2. Point-LIO 点云 (/cloud_registered)
  3. EGO 轨迹指令 (/ego/position_cmd)
  4. PX4 视觉里程计输入 (/fmu/in/vehicle_visual_odometry)
  5. PX4 本地位置 (/fmu/out/vehicle_local_position)
  6. PX4 飞控状态 (/fmu/out/vehicle_status)

每 1 秒生成一次 DiagnosticArray 诊断报告，检查数据新鲜度和有效性。
"""

import math
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition, VehicleOdometry, VehicleStatus
from quadrotor_msgs.msg import PositionCommand
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool


class EgoHardwareMonitor(Node):
    """
    EGO 硬件管线监控器。

    检查项（全部通过才报告 "ready for controlled test"）：
      - 各话题数据是否在超时时间内到达
      - PX4 位置估计 XY/Z 是否有效
      - trajectory_setpoint 发布者数量是否正确
    """
    def __init__(self) -> None:
        super().__init__("ego_hw_monitor")
        self.odom_topic = str(self.declare_parameter("odom_topic", "/odom").value)
        self.cloud_topic = str(
            self.declare_parameter("cloud_topic", "/cloud_registered").value
        )
        self.command_topic = str(
            self.declare_parameter("command_topic", "/ego/position_cmd").value
        )
        self.height_fusion_health_topic = str(
            self.declare_parameter(
                "height_fusion_health_topic", "/ego/height_fusion_healthy"
            ).value
        )
        self.require_rangefinder = bool(
            self.declare_parameter("require_rangefinder", True).value
        )
        self.max_odom_age = float(self.declare_parameter("max_odom_age_sec", 0.30).value)
        self.max_cloud_age = float(self.declare_parameter("max_cloud_age_sec", 0.50).value)
        self.max_command_age = float(
            self.declare_parameter("max_command_age_sec", 0.30).value
        )
        self.max_px4_age = float(self.declare_parameter("max_px4_age_sec", 0.50).value)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.last_odom_sec: Optional[float] = None
        self.last_cloud_sec: Optional[float] = None
        self.last_command_sec: Optional[float] = None
        self.last_visual_odom_sec: Optional[float] = None
        self.last_px4_sec: Optional[float] = None
        self.last_status_sec: Optional[float] = None
        self.last_px4: Optional[VehicleLocalPosition] = None
        self.last_status: Optional[VehicleStatus] = None
        self.height_fusion_healthy: Optional[bool] = None
        self.last_level: Optional[int] = None
        self.last_message = ""

        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, sensor_qos)
        self.create_subscription(PointCloud2, self.cloud_topic, self.cloud_callback, sensor_qos)
        self.create_subscription(
            PositionCommand, self.command_topic, self.command_callback, sensor_qos
        )
        self.create_subscription(
            VehicleOdometry,
            "/fmu/in/vehicle_visual_odometry",
            self.visual_odom_callback,
            px4_qos,
        )
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self.px4_callback,
            px4_qos,
        )
        self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status", self.status_callback, px4_qos
        )
        health_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            Bool,
            self.height_fusion_health_topic,
            self.height_fusion_health_callback,
            health_qos,
        )
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, "/ego_hw/diagnostics", 10
        )
        self.create_timer(1.0, self.report)
        self.get_logger().info(
            f"Hardware pipeline monitor: odom={self.odom_topic}, "
            f"cloud={self.cloud_topic}, command={self.command_topic}"
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def mark_now(self) -> float:
        return self.now_sec()

    def odom_callback(self, _msg: Odometry) -> None:
        self.last_odom_sec = self.mark_now()

    def cloud_callback(self, _msg: PointCloud2) -> None:
        self.last_cloud_sec = self.mark_now()

    def command_callback(self, _msg: PositionCommand) -> None:
        self.last_command_sec = self.mark_now()

    def visual_odom_callback(self, _msg: VehicleOdometry) -> None:
        self.last_visual_odom_sec = self.mark_now()

    def px4_callback(self, msg: VehicleLocalPosition) -> None:
        self.last_px4 = msg
        self.last_px4_sec = self.mark_now()

    def status_callback(self, msg: VehicleStatus) -> None:
        self.last_status = msg
        self.last_status_sec = self.mark_now()

    def height_fusion_health_callback(self, msg: Bool) -> None:
        self.height_fusion_healthy = bool(msg.data)

    def age(self, stamp: Optional[float]) -> float:
        return math.inf if stamp is None else max(0.0, self.now_sec() - stamp)

    @staticmethod
    def kv(key: str, value: object) -> KeyValue:
        return KeyValue(key=key, value=str(value))

    def control_publisher_count(self) -> int:
        return len(self.get_publishers_info_by_topic("/fmu/in/trajectory_setpoint"))

    def report(self) -> None:
        """每 1 秒生成诊断报告，检查所有数据链路状态。"""
        odom_age = self.age(self.last_odom_sec)
        cloud_age = self.age(self.last_cloud_sec)
        command_age = self.age(self.last_command_sec)
        visual_odom_age = self.age(self.last_visual_odom_sec)
        px4_age = self.age(self.last_px4_sec)
        status_age = self.age(self.last_status_sec)
        control_publishers = self.control_publisher_count()

        # 9 项检查：6 项数据新鲜度 + 2 项 PX4 有效性 + 1 项发布者数量
        checks = [
            (odom_age <= self.max_odom_age, "Point-LIO odometry is missing or stale"),
            (cloud_age <= self.max_cloud_age, "registered obstacle cloud is missing or stale"),
            (command_age <= self.max_command_age, "EGO trajectory command is missing or stale"),
            (visual_odom_age <= self.max_px4_age, "PX4 visual odometry input is missing or stale"),
            (px4_age <= self.max_px4_age, "PX4 local position is missing or stale"),
            (status_age <= self.max_px4_age, "PX4 vehicle status is missing or stale"),
            (self.last_px4 is not None and self.last_px4.xy_valid, "PX4 XY estimate is invalid"),
            (self.last_px4 is not None and self.last_px4.z_valid, "PX4 Z estimate is invalid"),
            (self.height_fusion_healthy is True, "planner altitude fusion is unhealthy"),
            (
                not self.require_rangefinder
                or (
                    self.last_px4 is not None
                    and self.last_px4.dist_bottom_valid
                    and int(self.last_px4.dist_bottom_sensor_bitfield) & 1
                ),
                "PX4 rangefinder height is invalid",
            ),
            (control_publishers == 1, f"expected one PX4 control publisher, found {control_publishers}"),
        ]
        failures = [message for passed, message in checks if not passed]
        level = DiagnosticStatus.OK if not failures else DiagnosticStatus.ERROR
        message = "ready for controlled test" if not failures else failures[0]
        values = [
            self.kv("odom_age_sec", f"{odom_age:.3f}"),
            self.kv("cloud_age_sec", f"{cloud_age:.3f}"),
            self.kv("command_age_sec", f"{command_age:.3f}"),
            self.kv("visual_odom_age_sec", f"{visual_odom_age:.3f}"),
            self.kv("px4_position_age_sec", f"{px4_age:.3f}"),
            self.kv("px4_status_age_sec", f"{status_age:.3f}"),
            self.kv("px4_xy_valid", getattr(self.last_px4, "xy_valid", False)),
            self.kv("px4_z_valid", getattr(self.last_px4, "z_valid", False)),
            self.kv(
                "px4_dist_bottom_valid",
                getattr(self.last_px4, "dist_bottom_valid", False),
            ),
            self.kv(
                "px4_dist_bottom_sensor_bitfield",
                getattr(self.last_px4, "dist_bottom_sensor_bitfield", 0),
            ),
            self.kv("height_fusion_healthy", self.height_fusion_healthy),
            self.kv("trajectory_setpoint_publishers", control_publishers),
            self.kv("failures", "; ".join(failures)),
        ]
        status = DiagnosticStatus(
            level=level,
            name="ego_hw/pipeline",
            message=message,
            hardware_id="uav-mission-stack",
            values=values,
        )
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        diagnostics.status = [status]
        self.diagnostics_pub.publish(diagnostics)

        if level != self.last_level or message != self.last_message:
            self.last_level = level
            self.last_message = message
            if level == DiagnosticStatus.OK:
                self.get_logger().info(message)
            else:
                self.get_logger().error(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EgoHardwareMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
