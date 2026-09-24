#!/usr/bin/env python3

import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rigid_transform import normalize_quaternion
from vision_pose_adapter import odometry_to_vision_pose


# MAVROS' vision_pose plugin uses a reliable subscription. A best-effort
# publisher is incompatible with it, so those samples would never reach PX4.
VISION_POSE_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


class FastlioToMavrosVisionPose(Node):
    """Forward zero-referenced FAST-LIO ENU poses to MAVROS."""

    def __init__(self) -> None:
        super().__init__("fastlio_to_mavros_vision_pose")
        self.fastlio_odom_topic = str(
            self.declare_parameter("fastlio_odom_topic", "/Odometry").value
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
            "base_link_to_base_translation", [0.0, 0.0, 0.0]
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
        self.reference_position = None

        self.publisher = self.create_publisher(
            PoseStamped, self.vision_pose_topic, VISION_POSE_QOS
        )
        self.create_subscription(
            Odometry,
            self.fastlio_odom_topic,
            self.fastlio_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            "FAST-LIO ENU -> MAVROS vision pose: "
            f"{self.fastlio_odom_topic} -> {self.vision_pose_topic}, "
            f"rate={self.publish_rate_hz:.1f} Hz"
        )

    def fastlio_callback(self, msg: Odometry) -> None:
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
            self.get_logger().warning(f"Ignoring invalid FAST-LIO pose: {exc}")
            return

        if self.reference_position is None:
            p = output.pose.position
            self.reference_position = (p.x, p.y, p.z)
            self.get_logger().info("Captured FAST-LIO local reference")
        output.pose.position.x -= self.reference_position[0]
        output.pose.position.y -= self.reference_position[1]
        output.pose.position.z -= self.reference_position[2]
        # FAST-LIO is already ROS ENU in a local world. MAVROS performs the
        # ENU/FLU -> NED/FRD conversion before sending the measurement to PX4.
        self.publisher.publish(output)
        self.last_publish_time = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FastlioToMavrosVisionPose()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
