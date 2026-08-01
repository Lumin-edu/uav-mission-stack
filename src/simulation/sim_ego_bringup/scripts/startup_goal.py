#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool


class StartupGoal(Node):
    """Convert a task-frame launch goal and publish it in Point-LIO ROS axes."""

    def __init__(self) -> None:
        super().__init__("startup_goal")
        self.goal_topic = str(self.declare_parameter("goal_topic", "/move_base_simple/goal").value)
        self.odom_topic = str(self.declare_parameter("odom_topic", "/sim/pointlio/odom").value)
        self.frame_id = str(self.declare_parameter("frame_id", "world").value)
        self.goal_x = float(self.declare_parameter("goal_x", 6.0).value)
        self.goal_y = float(self.declare_parameter("goal_y", 0.0).value)
        self.goal_z = float(self.declare_parameter("goal_z", 1.5).value)
        self.goal_yaw = float(self.declare_parameter("goal_yaw", 0.0).value)
        self.startup_delay_sec = float(self.declare_parameter("startup_delay_sec", 1.0).value)
        self.wait_for_takeoff_ready = bool(
            self.declare_parameter("wait_for_takeoff_ready", True).value
        )
        self.takeoff_ready_topic = str(
            self.declare_parameter("takeoff_ready_topic", "/ego/takeoff_ready").value
        )
        self.wait_for_occupancy = bool(
            self.declare_parameter("wait_for_occupancy", True).value
        )
        self.occupancy_topic = str(
            self.declare_parameter(
                "occupancy_topic", "/ego/grid_map/static_occupancy_inflate"
            ).value
        )
        if self.goal_z <= 0.05 or self.startup_delay_sec < 0.0:
            raise ValueError("goal_z must be positive and startup_delay_sec must be non-negative")

        self.publisher = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        ready_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            Bool, self.takeoff_ready_topic, self.takeoff_ready_callback, ready_qos
        )
        self.create_subscription(
            PointCloud2, self.occupancy_topic, self.occupancy_callback, ready_qos
        )
        self.odom_ready_at = None
        self.takeoff_ready_at = None if self.wait_for_takeoff_ready else time.monotonic()
        self.occupancy_ready_at = None if self.wait_for_occupancy else time.monotonic()
        self.sent = False
        self.create_timer(0.1, self.timer_callback)
        self.get_logger().info(
            "Simulation startup goal: "
            f"x_right={self.goal_x:.2f}, y_forward={self.goal_y:.2f}, "
            f"z_up={self.goal_z:.2f}, frame={self.frame_id}, "
            f"wait_for_takeoff_ready={self.wait_for_takeoff_ready}, "
            f"wait_for_occupancy={self.wait_for_occupancy}"
        )

    @staticmethod
    def task_goal_to_ros(
        x_right: float, y_forward: float, z_up: float, yaw: float
    ) -> tuple[float, float, float, float]:
        """Map launch/task coordinates into Point-LIO's ROS FLU frame."""
        return (y_forward, -x_right, z_up, -yaw)

    def odom_callback(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        if not all(math.isfinite(float(v)) for v in (p.x, p.y, p.z)):
            return
        if self.odom_ready_at is None:
            self.odom_ready_at = time.monotonic()
            self.get_logger().info("Simulation odometry is ready.")

    def takeoff_ready_callback(self, msg: Bool) -> None:
        if not msg.data or self.takeoff_ready_at is not None:
            return
        self.takeoff_ready_at = time.monotonic()
        self.get_logger().info("Takeoff is stable; waiting for the EGO goal subscriber.")

    def occupancy_callback(self, msg: PointCloud2) -> None:
        del msg
        if self.occupancy_ready_at is None:
            self.occupancy_ready_at = time.monotonic()
            self.get_logger().info("EGO static occupancy is ready; startup goal may be validated.")

    def timer_callback(self) -> None:
        if (
            self.sent
            or self.odom_ready_at is None
            or self.takeoff_ready_at is None
            or self.occupancy_ready_at is None
        ):
            return
        if time.monotonic() - self.takeoff_ready_at < self.startup_delay_sec:
            return
        if self.publisher.get_subscription_count() == 0:
            return

        # Launch arguments match the verified aircraft task convention, while
        # EGO and Point-LIO use standard ROS FLU coordinates.
        #
        #   task x right   -> ROS -y (left)
        #   task y forward -> ROS +x (forward)
        #   task z up      -> ROS +z (up)
        ros_x, ros_y, ros_z, ros_yaw = self.task_goal_to_ros(
            self.goal_x, self.goal_y, self.goal_z, self.goal_yaw
        )

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self.frame_id
        goal.pose.position.x = ros_x
        goal.pose.position.y = ros_y
        goal.pose.position.z = ros_z
        goal.pose.orientation.z = math.sin(ros_yaw / 2.0)
        goal.pose.orientation.w = math.cos(ros_yaw / 2.0)
        self.publisher.publish(goal)
        self.sent = True
        self.get_logger().info(
            "Published simulation startup goal to EGO: "
            f"ROS_FLU=({ros_x:+.2f}, {ros_y:+.2f}, {ros_z:+.2f})."
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StartupGoal()
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
