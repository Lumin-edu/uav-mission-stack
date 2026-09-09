#!/usr/bin/env python3
"""Convert Point-LIO base_link odometry to aircraft-center base odometry."""

from __future__ import annotations

import copy
import math
from typing import Sequence

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from rigid_transform import compose_pose, normalize_quaternion


class RigidOdomTransform(Node):
    def __init__(self) -> None:
        super().__init__("rigid_odom_transform")
        self.source_topic = str(self.declare_parameter("source_topic", "/odom").value)
        self.target_topic = str(self.declare_parameter("target_topic", "/ego/odom_base").value)
        self.target_child_frame = str(self.declare_parameter("target_child_frame", "base").value)
        self.translation = self._vector_parameter("base_link_to_base_translation", [-0.011, -0.02329, -0.05588], 3)
        self.rotation = normalize_quaternion(self._vector_parameter("base_link_to_base_rotation_xyzw", [0.0, 0.0, 0.0, 1.0], 4))
        self.publisher = self.create_publisher(Odometry, self.target_topic, 10)
        self.subscription = self.create_subscription(Odometry, self.source_topic, self.odom_callback, 10)

    @staticmethod
    def _vector_parameter(name: str, values: Sequence[float], size: int) -> tuple[float, ...]:
        result = tuple(float(value) for value in values)
        if len(result) != size or not all(math.isfinite(value) for value in result):
            raise ValueError(f"{name} must contain {size} finite values")
        return result

    def odom_callback(self, msg: Odometry) -> None:
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        try:
            position, orientation = compose_pose((float(p.x), float(p.y), float(p.z)), (float(q.x), float(q.y), float(q.z), float(q.w)), self.translation, self.rotation)
        except ValueError as error:
            self.get_logger().warn(f"Dropping invalid Point-LIO odometry: {error}")
            return
        output = copy.deepcopy(msg)
        output.child_frame_id = self.target_child_frame
        output.pose.pose.position.x, output.pose.pose.position.y, output.pose.pose.position.z = position
        output.pose.pose.orientation.x, output.pose.pose.orientation.y, output.pose.pose.orientation.z, output.pose.pose.orientation.w = orientation
        self.publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RigidOdomTransform()
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
