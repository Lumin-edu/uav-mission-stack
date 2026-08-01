#!/usr/bin/env python3
"""
PX4 DDS 话题监控节点。

监控 PX4 飞控的核心输出话题状态，帮助在飞行前确认 uXRCE-DDS 通信链路正常。
监控的话题包括：
  - vehicle_status：飞控状态（包含 nav_state 导航状态）
  - vehicle_local_position：本地位置（包含 xy_valid/z_valid 有效性标志）
  - distance_sensor：距离传感器数据
"""

import rclpy
from px4_msgs.msg import DistanceSensor, VehicleLocalPosition, VehicleStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class Px4DdsMonitor(Node):
    """
    PX4 DDS 监控节点。

    订阅三个核心 PX4 输出话题，每 2 秒打印一次数据到达状态和关键字段值，
    用于验证 uXRCE-DDS 通信链路和传感器数据是否正常到达。
    """
    def __init__(self) -> None:
        super().__init__("px4_dds_monitor")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.status_received = False
        self.local_position_received = False
        self.distance_sensor_received = False
        self.vehicle_status_topic = self.declare_parameter(
            "vehicle_status_topic", "/fmu/out/vehicle_status"
        ).value
        self.vehicle_local_position_topic = self.declare_parameter(
            "vehicle_local_position_topic", "/fmu/out/vehicle_local_position"
        ).value
        self.distance_sensor_topic = self.declare_parameter(
            "distance_sensor_topic", "/fmu/out/distance_sensor"
        ).value

        self.last_nav_state = None
        self.last_xy_valid = None
        self.last_z_valid = None
        self.last_dist_bottom_valid = None
        self.last_dist_bottom = None
        self.last_distance_current = None

        self.create_subscription(
            VehicleStatus,
            self.vehicle_status_topic,
            self.vehicle_status_callback,
            qos,
        )
        self.create_subscription(
            VehicleLocalPosition,
            self.vehicle_local_position_topic,
            self.vehicle_local_position_callback,
            qos,
        )
        self.create_subscription(
            DistanceSensor,
            self.distance_sensor_topic,
            self.distance_sensor_callback,
            qos,
        )

        self.timer = self.create_timer(2.0, self.timer_callback)
        self.get_logger().info(
            "Monitoring PX4 DDS topics: "
            f"status={self.vehicle_status_topic}, "
            f"local_position={self.vehicle_local_position_topic}, "
            f"distance_sensor={self.distance_sensor_topic}"
        )

    def vehicle_status_callback(self, msg: VehicleStatus) -> None:
        self.status_received = True
        self.last_nav_state = int(msg.nav_state)

    def vehicle_local_position_callback(self, msg: VehicleLocalPosition) -> None:
        self.local_position_received = True
        self.last_xy_valid = bool(msg.xy_valid)
        self.last_z_valid = bool(msg.z_valid)
        self.last_dist_bottom_valid = bool(msg.dist_bottom_valid)
        self.last_dist_bottom = float(msg.dist_bottom)

    def distance_sensor_callback(self, msg: DistanceSensor) -> None:
        self.distance_sensor_received = True
        self.last_distance_current = float(msg.current_distance)

    def timer_callback(self) -> None:
        self.get_logger().info(
            "DDS status: "
            f"vehicle_status={self.status_received}, "
            f"local_position={self.local_position_received}, "
            f"distance_sensor={self.distance_sensor_received}, "
            f"nav_state={self.last_nav_state}, "
            f"xy_valid={self.last_xy_valid}, "
            f"z_valid={self.last_z_valid}, "
            f"dist_bottom_valid={self.last_dist_bottom_valid}, "
            f"dist_bottom={self.last_dist_bottom}, "
            f"distance_current={self.last_distance_current}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Px4DdsMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
