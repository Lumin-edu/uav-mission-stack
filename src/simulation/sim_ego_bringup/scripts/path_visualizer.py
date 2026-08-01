#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from quadrotor_msgs.msg import PositionCommand
from rclpy.node import Node


class PathVisualizer(Node):
    def __init__(self) -> None:
        super().__init__("path_visualizer")
        odom_topic = str(self.declare_parameter("odom_topic", "/sim/odom").value)
        command_topic = str(
            self.declare_parameter("command_topic", "/ego/position_cmd").value
        )
        odom_path_topic = str(
            self.declare_parameter("odom_path_topic", "/sim/odom_path").value
        )
        command_path_topic = str(
            self.declare_parameter("command_path_topic", "/sim/command_path").value
        )
        max_points = int(self.declare_parameter("max_points", 5000).value)
        self.publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 2.0).value)
        self.sample_distance_m = float(
            self.declare_parameter("sample_distance_m", 0.05).value
        )
        self.sample_interval_sec = float(
            self.declare_parameter("sample_interval_sec", 0.10).value
        )
        self.max_points = max(10, max_points)
        if self.publish_rate_hz <= 0.0 or self.sample_distance_m < 0.0 or self.sample_interval_sec <= 0.0:
            raise ValueError("publish_rate_hz and sample_interval_sec must be positive")
        self.odom_path = Path()
        self.command_path = Path()
        self.last_command: Optional[PositionCommand] = None
        self.last_odom_sample_sec: Optional[float] = None
        self.last_command_sample_sec: Optional[float] = None
        self.last_odom_position: Optional[tuple[float, float, float]] = None
        self.last_command_position: Optional[tuple[float, float, float]] = None
        self.odom_pub = self.create_publisher(Path, odom_path_topic, 10)
        self.command_pub = self.create_publisher(Path, command_path_topic, 10)
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 20)
        self.create_subscription(PositionCommand, command_topic, self.command_callback, 20)
        self.create_timer(1.0 / self.publish_rate_hz, self.publish_paths)
        self.get_logger().info(
            f"Visualizing odometry and command paths: odom={odom_topic}, command={command_topic}, "
            f"publish={self.publish_rate_hz:.1f} Hz, max_points={self.max_points}"
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def should_sample(
        self,
        position: tuple[float, float, float],
        last_position: Optional[tuple[float, float, float]],
        last_sample_sec: Optional[float],
    ) -> bool:
        now = self.now_sec()
        if last_position is None or last_sample_sec is None:
            return True
        distance = math.sqrt(sum((position[i] - last_position[i]) ** 2 for i in range(3)))
        return distance >= self.sample_distance_m or now - last_sample_sec >= self.sample_interval_sec

    def odom_callback(self, msg: Odometry) -> None:
        position = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            float(msg.pose.pose.position.z),
        )
        if not self.should_sample(position, self.last_odom_position, self.last_odom_sample_sec):
            return
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        if not self.odom_path.header.frame_id:
            self.odom_path.header.frame_id = msg.header.frame_id or "world"
        self.odom_path.header.stamp = msg.header.stamp
        self.odom_path.poses.append(pose)
        if len(self.odom_path.poses) > self.max_points:
            self.odom_path.poses = self.odom_path.poses[-self.max_points :]
        self.last_odom_position = position
        self.last_odom_sample_sec = self.now_sec()

    def command_callback(self, msg: PositionCommand) -> None:
        self.last_command = msg
        position = (
            float(msg.position.x),
            float(msg.position.y),
            float(msg.position.z),
        )
        if not self.should_sample(
            position, self.last_command_position, self.last_command_sample_sec
        ):
            return
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = pose.header.frame_id or "world"
        pose.pose.position = msg.position
        pose.pose.orientation.w = 1.0
        if not self.command_path.header.frame_id:
            self.command_path.header.frame_id = pose.header.frame_id
        self.command_path.header.stamp = pose.header.stamp
        self.command_path.poses.append(pose)
        if len(self.command_path.poses) > self.max_points:
            self.command_path.poses = self.command_path.poses[-self.max_points :]
        self.last_command_position = position
        self.last_command_sample_sec = self.now_sec()

    def publish_paths(self) -> None:
        if self.odom_path.poses:
            self.odom_pub.publish(self.odom_path)
        if self.command_path.poses:
            self.command_pub.publish(self.command_path)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathVisualizer()
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
