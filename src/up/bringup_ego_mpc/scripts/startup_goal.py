#!/usr/bin/env python3
"""
EGO 启动目标发布节点。

在杆臂补偿后的无人机中心里程计和起飞流程都就绪后，向 EGO 发布初始目标航点。
这个节点只发布一次，然后退出。

发布条件（三者同时满足）：
  1. EGO 已收到有效的无人机中心里程计（/ego/odom_base）
  2. 起飞已完成（takeoff_ready 信号为 True，若 wait_for_takeoff_ready=True）
  3. 等待 startup_delay_sec 秒后
  4. EGO 规划器已订阅目标话题
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class StartupGoal(Node):
    """
    在无人机中心里程计和起飞就绪后发布一次性 EGO 目标。

    用户参数使用任务坐标系：x_right=机头右侧, y_forward=机头前方, z_up=向上。
    内部自动转换为 EGO odom 的 ROS 标准系 (x=前, y=左, z=上) 后发布。
    """

    def __init__(self) -> None:
        super().__init__("startup_goal")
        self.goal_topic = str(
            self.declare_parameter("goal_topic", "/move_base_simple/goal").value
        )
        self.odom_topic = str(
            self.declare_parameter("odom_topic", "/ego/odom_base").value
        )
        self.frame_id = str(self.declare_parameter("frame_id", "odom").value)
        # 用户任务坐标系参数：x=机头右侧, y=机头前方, z=向上
        self.goal_x = float(self.declare_parameter("goal_x", 1.0).value)
        self.goal_y = float(self.declare_parameter("goal_y", 0.0).value)
        self.goal_z = float(self.declare_parameter("goal_z", 0.5).value)
        self.goal_yaw = float(self.declare_parameter("goal_yaw", 0.0).value)
        self.startup_delay_sec = float(
            self.declare_parameter("startup_delay_sec", 1.0).value
        )
        self.wait_for_takeoff_ready = bool(
            self.declare_parameter("wait_for_takeoff_ready", True).value
        )
        self.takeoff_ready_topic = str(
            self.declare_parameter("takeoff_ready_topic", "/ego/takeoff_ready").value
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
        self.odom_ready_at = None
        self.takeoff_ready_at = None if self.wait_for_takeoff_ready else time.monotonic()
        self.sent = False
        self.create_timer(0.1, self.timer_callback)
        self.get_logger().info(
            "Will publish one startup EGO goal after base-center odometry is ready: "
            f"x_right={self.goal_x:.2f}, y_forward={self.goal_y:.2f}, "
            f"z_up={self.goal_z:.2f} in {self.frame_id}, "
            f"wait_for_takeoff_ready={self.wait_for_takeoff_ready}"
        )

    def odom_callback(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        if not all(math.isfinite(value) for value in (position.x, position.y, position.z)):
            return
        if self.odom_ready_at is None:
            self.odom_ready_at = time.monotonic()
            self.get_logger().info("EGO base-center odometry is ready.")

    def takeoff_ready_callback(self, msg: Bool) -> None:
        if not msg.data or self.takeoff_ready_at is not None:
            return
        self.takeoff_ready_at = time.monotonic()
        self.get_logger().info("Takeoff is stable; waiting for the EGO goal subscriber.")

    def timer_callback(self) -> None:
        """
        每 0.1 秒检查一次发布条件。

        等待条件：
          1. 杆臂补偿后的无人机中心里程计已有有效数据（odom_ready_at 不为 None）
          2. 起飞已完成就绪（takeoff_ready_at 不为 None）
          3. 等待 startup_delay_sec 秒
          4. EGO 规划器已订阅目标话题
        """
        if self.sent or self.odom_ready_at is None or self.takeoff_ready_at is None:
            return
        if time.monotonic() - self.takeoff_ready_at < self.startup_delay_sec:
            return
        if self.publisher.get_subscription_count() == 0:
            return

        # 坐标系转换：用户任务系 (x右, y前, z上) → ROS 标准系 (x前, y左, z上)
        #   ROS_x(前) = task_y(前)
        #   ROS_y(左) = -task_x(右)
        #   ROS_z(上) = task_z(上)
        #   ROS_yaw    = -task_yaw    (右手系 → 左手系 y 轴翻转后偏航角取反)
        ros_x = self.goal_y             # y_forward → x (前方)
        ros_y = -self.goal_x            # x_right → y 翻转 (左)
        ros_z = self.goal_z             # z_up 不变
        ros_yaw = -self.goal_yaw        # 偏航角取反

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
        self.get_logger().info("Published startup goal to EGO.")


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
