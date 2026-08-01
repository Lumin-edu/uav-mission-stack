#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
from quadrotor_msgs.msg import PositionCommand
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from traj_utils.msg import Bspline


class SimHealthMonitor(Node):
    def __init__(self) -> None:
        super().__init__("sim_health_monitor")
        self.mode = str(self.declare_parameter("mode", "headless").value)
        self.odom_topic = str(self.declare_parameter("odom_topic", "/sim/odom").value)
        self.command_topic = str(
            self.declare_parameter("command_topic", "/ego/position_cmd").value
        )
        self.planner_trajectory_topic = str(
            self.declare_parameter("planner_trajectory_topic", "").value
        )
        self.planner_trajectory_timeout_sec = float(
            self.declare_parameter("planner_trajectory_timeout_sec", 2.0).value
        )
        self.cloud_topic = str(
            self.declare_parameter("cloud_topic", "/map_generator/global_cloud").value
        )
        self.px4_position_topic = str(
            self.declare_parameter("px4_position_topic", "/fmu/out/vehicle_local_position").value
        )
        self.vehicle_status_topic = str(
            self.declare_parameter("vehicle_status_topic", "/fmu/out/vehicle_status").value
        )
        self.localization_health_topic = str(
            self.declare_parameter("localization_health_topic", "").value
        )
        self.max_odom_age_sec = float(self.declare_parameter("max_odom_age_sec", 0.25).value)
        self.max_command_age_sec = float(
            self.declare_parameter("max_command_age_sec", 0.50).value
        )

        self.last_odom: Optional[Odometry] = None
        self.last_odom_sec: Optional[float] = None
        self.last_command: Optional[PositionCommand] = None
        self.last_command_sec: Optional[float] = None
        self.last_trajectory_id: Optional[int] = None
        self.trajectory_valid_until_sec: Optional[float] = None
        self.last_cloud: Optional[PointCloud2] = None
        self.last_cloud_sec: Optional[float] = None
        self.last_px4: Optional[VehicleLocalPosition] = None
        self.last_px4_sec: Optional[float] = None
        self.last_status: Optional[VehicleStatus] = None
        self.last_status_sec: Optional[float] = None
        self.localization_healthy: Optional[bool] = None

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.create_subscription(PositionCommand, self.command_topic, self.command_callback, 10)
        self.trajectory_sub = None
        if self.planner_trajectory_topic:
            self.trajectory_sub = self.create_subscription(
                Bspline,
                self.planner_trajectory_topic,
                self.trajectory_callback,
                10,
            )
        self.create_subscription(PointCloud2, self.cloud_topic, self.cloud_callback, 10)
        if self.localization_health_topic:
            health_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self.create_subscription(
                Bool,
                self.localization_health_topic,
                self.localization_health_callback,
                health_qos,
            )
        if self.mode == "px4":
            self.create_subscription(
                VehicleLocalPosition, self.px4_position_topic, self.px4_callback, px4_qos
            )
            self.create_subscription(
                VehicleStatus, self.vehicle_status_topic, self.status_callback, px4_qos
            )

        self.diagnostics_pub = self.create_publisher(DiagnosticArray, "/sim/diagnostics", 10)
        self.create_timer(1.0, self.report)
        self.get_logger().info(
            f"Monitoring mode={self.mode}, odom={self.odom_topic}, command={self.command_topic}, cloud={self.cloud_topic}"
        )

    def now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def odom_callback(self, msg: Odometry) -> None:
        self.last_odom = msg
        self.last_odom_sec = self.now()

    def command_callback(self, msg: PositionCommand) -> None:
        self.last_command = msg
        self.last_command_sec = self.now()

    def trajectory_callback(self, msg: Bspline) -> None:
        order = int(msg.order)
        end_index = len(msg.knots) - 1 - order
        if order < 1 or len(msg.knots) <= 2 * order or end_index <= order:
            return
        duration = float(msg.knots[end_index]) - float(msg.knots[order])
        start_sec = float(msg.start_time.sec) + float(msg.start_time.nanosec) * 1e-9
        if not math.isfinite(duration) or duration <= 0.0 or not math.isfinite(start_sec):
            return
        self.last_trajectory_id = int(msg.traj_id)
        self.trajectory_valid_until_sec = (
            start_sec + duration + self.planner_trajectory_timeout_sec
        )

    def cloud_callback(self, msg: PointCloud2) -> None:
        self.last_cloud = msg
        self.last_cloud_sec = self.now()

    def px4_callback(self, msg: VehicleLocalPosition) -> None:
        self.last_px4 = msg
        self.last_px4_sec = self.now()

    def status_callback(self, msg: VehicleStatus) -> None:
        self.last_status = msg
        self.last_status_sec = self.now()

    def localization_health_callback(self, msg: Bool) -> None:
        self.localization_healthy = bool(msg.data)

    def age(self, timestamp: Optional[float]) -> float:
        return math.inf if timestamp is None else max(0.0, self.now() - timestamp)

    @staticmethod
    def value(key: str, value: object) -> KeyValue:
        return KeyValue(key=key, value=str(value))

    def report(self) -> None:
        odom_age = self.age(self.last_odom_sec)
        command_age = self.age(self.last_command_sec)
        cloud_age = self.age(self.last_cloud_sec)
        trajectory_publisher_count = (
            self.trajectory_sub.get_publisher_count()
            if self.trajectory_sub is not None
            else -1
        )
        trajectory_expired = bool(
            self.trajectory_valid_until_sec is not None
            and self.now() > self.trajectory_valid_until_sec
        )
        level = DiagnosticStatus.OK
        message = "healthy"
        if self.localization_health_topic and self.localization_healthy is False:
            level = DiagnosticStatus.ERROR
            message = "localization guard unhealthy"
        elif odom_age > self.max_odom_age_sec:
            level = DiagnosticStatus.ERROR
            message = "stale odometry"
        elif (
            self.trajectory_sub is not None
            and self.last_trajectory_id is not None
            and trajectory_publisher_count == 0
        ):
            level = DiagnosticStatus.ERROR
            message = "EGO planner publisher missing"
        elif self.trajectory_sub is not None and trajectory_expired:
            level = DiagnosticStatus.WARN
            message = "EGO trajectory expired"
        elif command_age > self.max_command_age_sec:
            level = DiagnosticStatus.WARN
            message = "stale planner command"
        elif self.last_cloud is None:
            level = DiagnosticStatus.WARN
            message = "waiting for obstacle cloud"

        values = [
            self.value("mode", self.mode),
            self.value("odom_age_sec", f"{odom_age:.3f}"),
            self.value("command_age_sec", f"{command_age:.3f}"),
            self.value("cloud_age_sec", f"{cloud_age:.3f}"),
            self.value("cloud_points", 0 if self.last_cloud is None else self.last_cloud.width * self.last_cloud.height),
            self.value("localization_healthy", self.localization_healthy),
            self.value("planner_trajectory_id", self.last_trajectory_id),
            self.value("planner_trajectory_expired", trajectory_expired),
            self.value("planner_trajectory_publishers", trajectory_publisher_count),
        ]
        if self.last_odom is not None:
            position = self.last_odom.pose.pose.position
            values.extend(
                [
                    self.value("odom_x", f"{position.x:.3f}"),
                    self.value("odom_y", f"{position.y:.3f}"),
                    self.value("odom_z", f"{position.z:.3f}"),
                ]
            )
        if self.mode == "px4":
            values.extend(
                [
                    self.value("px4_position_age_sec", f"{self.age(self.last_px4_sec):.3f}"),
                    self.value("px4_status_age_sec", f"{self.age(self.last_status_sec):.3f}"),
                ]
            )
            if self.last_px4 is not None:
                values.extend(
                    [
                        self.value("px4_xy_valid", self.last_px4.xy_valid),
                        self.value("px4_z_valid", self.last_px4.z_valid),
                    ]
                )
            if self.last_status is not None:
                values.extend(
                    [
                        self.value("px4_arming_state", self.last_status.arming_state),
                        self.value("px4_nav_state", self.last_status.nav_state),
                    ]
                )

        status = DiagnosticStatus(
            level=level,
            name="sim_ego/health",
            message=message,
            hardware_id="sim-ego",
            values=values,
        )
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        diagnostics.status = [status]
        self.diagnostics_pub.publish(diagnostics)
        log_message = (
            f"{message}: odom_age={odom_age:.3f}s "
            f"command_age={command_age:.3f}s cloud_age={cloud_age:.3f}s"
        )
        if level == DiagnosticStatus.ERROR:
            self.get_logger().error(log_message)
        elif level == DiagnosticStatus.WARN:
            self.get_logger().warn(log_message)
        else:
            self.get_logger().info(log_message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimHealthMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
