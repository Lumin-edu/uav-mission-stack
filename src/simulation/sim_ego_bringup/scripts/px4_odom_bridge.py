#!/usr/bin/env python3

"""Convert PX4 local NED state into the ROS-FLU convention used by EGO."""

import math

import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class Px4OdomBridge(Node):
    def __init__(self) -> None:
        super().__init__("px4_odom_bridge")
        input_topic = str(
            self.declare_parameter(
                "px4_position_topic", "/fmu/out/vehicle_local_position"
            ).value
        )
        output_topic = str(self.declare_parameter("odom_topic", "/sim/odom").value)
        self.frame_id = str(self.declare_parameter("frame_id", "world").value)
        self.child_frame_id = str(
            self.declare_parameter("child_frame_id", "base_link").value
        )
        px4_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(Odometry, output_topic, 10)
        self.create_subscription(VehicleLocalPosition, input_topic, self.callback, px4_qos)
        self.initial_heading = None
        self.get_logger().info(f"PX4 NED -> ROS-FLU odometry: {input_topic} -> {output_topic}")

    def callback(self, msg: VehicleLocalPosition) -> None:
        if not msg.xy_valid or not msg.z_valid:
            return
        values = (msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz)
        if not all(math.isfinite(float(value)) for value in values):
            return

        if self.initial_heading is None:
            self.initial_heading = float(msg.heading) if math.isfinite(float(msg.heading)) else 0.0
            self.get_logger().info(
                f"Captured initial PX4 heading for ROS-FLU fallback: {self.initial_heading:.3f} rad"
            )

        c = math.cos(self.initial_heading)
        s = math.sin(self.initial_heading)
        # PX4 NED -> initial-heading-relative ROS FLU.
        x_forward = c * float(msg.x) + s * float(msg.y)
        y_left = s * float(msg.x) - c * float(msg.y)
        vx_forward = c * float(msg.vx) + s * float(msg.vy)
        vy_left = s * float(msg.vx) - c * float(msg.vy)

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = x_forward
        odom.pose.pose.position.y = y_left
        odom.pose.pose.position.z = -float(msg.z)
        odom.twist.twist.linear.x = vx_forward
        odom.twist.twist.linear.y = vy_left
        odom.twist.twist.linear.z = -float(msg.vz)
        current_heading = float(msg.heading) if math.isfinite(float(msg.heading)) else self.initial_heading
        yaw = -(current_heading - self.initial_heading)
        odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.publisher.publish(odom)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Px4OdomBridge()
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
