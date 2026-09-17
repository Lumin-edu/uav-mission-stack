#!/usr/bin/env python3

import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rigid_transform import normalize_quaternion
from vision_pose_adapter import odometry_to_vision_pose


class PointlioToMavrosVisionPose(Node):
    """Forward Point-LIO FLU/ENU poses to the MAVROS vision pose plugin."""

    def __init__(self) -> None:
        super().__init__("pointlio_to_mavros_vision_pose")
        self.pointlio_odom_topic = str(
            self.declare_parameter("pointlio_odom_topic", "/odom").value
        )
        self.vision_pose_topic = str(
            self.declare_parameter(
                "vision_pose_topic", "/mavros/vision_pose/pose"
            ).value
        )
        self.output_frame_id = str(
            self.declare_parameter("output_frame_id", "odom").value
        )
        self.publish_rate_hz = max(
            1.0, float(self.declare_parameter("publish_rate_hz", 50.0).value)
        )
        translation = self.declare_parameter(
            "base_link_to_base_translation", [-0.011, -0.02329, -0.05588]
        ).value
        rotation = self.declare_parameter(
            "base_link_to_base_rotation_xyzw", [0.0, 0.0, 0.0, 1.0]
        ).value
        if len(translation) != 3:
            raise ValueError("base_link_to_base_translation must contain 3 values")
        self.base_link_to_base_translation = tuple(
            float(value) for value in translation
        )
        self.base_link_to_base_rotation = normalize_quaternion(rotation)
        self.last_publish_time = 0.0

        self.publisher = self.create_publisher(
            PoseStamped, self.vision_pose_topic, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry,
            self.pointlio_odom_topic,
            self.pointlio_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            "Point-LIO FLU/ENU -> MAVROS vision pose: "
            f"{self.pointlio_odom_topic} -> {self.vision_pose_topic}, "
            f"rate={self.publish_rate_hz:.1f} Hz"
        )

    def pointlio_callback(self, msg: Odometry) -> None:
        now = time.monotonic()
        if now - self.last_publish_time < 1.0 / self.publish_rate_hz:
            return
        try:
            output = odometry_to_vision_pose(
                msg,
                base_link_to_base_translation=self.base_link_to_base_translation,
                base_link_to_base_rotation_xyzw=self.base_link_to_base_rotation,
                output_frame_id=self.output_frame_id,
            )
        except ValueError as exc:
            self.get_logger().warning(f"Ignoring invalid Point-LIO pose: {exc}")
            return

        # Point-LIO is already ROS FLU in an ENU world. MAVROS performs the
        # ENU/FLU -> NED/FRD conversion before sending the measurement to PX4.
        self.publisher.publish(output)
        self.last_publish_time = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PointlioToMavrosVisionPose()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
