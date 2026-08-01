#!/usr/bin/env python3
"""
PX4 Offboard 控制话题看门狗节点。

定期检查必需的 Offboard 控制话题（offboard_control_mode、trajectory_setpoint、
vehicle_command）是否有发布者。如果缺少任何发布者，PX4 将无法进入 Offboard 模式，
此时会输出错误日志帮助快速定位问题。
"""

import rclpy
from rclpy.node import Node


class Px4ControlWatchdog(Node):
    """
    PX4 控制看门狗。

    每 2 秒检查一次三个关键 Offboard 话题的发布者状态，
    当所有话题都有发布者时报告就绪，否则报错并列出缺失的话题。
    """
    def __init__(self) -> None:
        super().__init__("px4_control_watchdog")
        self.declare_parameter("control_source", "hover")

        self.control_source = self.get_parameter("control_source").value
        # PX4 Offboard 模式必需的三个控制话题
        self.required_topics = [
            "/fmu/in/offboard_control_mode",
            "/fmu/in/trajectory_setpoint",
            "/fmu/in/vehicle_command",
        ]
        self.reported_ready = False
        self.create_timer(2.0, self.report)
        self.get_logger().info(
            f"PX4 control watchdog active, expected control_source={self.control_source}"
        )

    def report(self) -> None:
        missing = []
        details = []
        for topic in self.required_topics:
            publishers = self.get_publishers_info_by_topic(topic)
            subscriptions = self.get_subscriptions_info_by_topic(topic)
            details.append(f"{topic}: pubs={len(publishers)}, subs={len(subscriptions)}")
            if len(publishers) == 0:
                missing.append(topic)

        if missing:
            self.reported_ready = False
            self.get_logger().error(
                "PX4 will not move because no ROS node is publishing required offboard inputs. "
                f"Missing publishers: {missing}. Details: {', '.join(details)}"
            )
            return

        if not self.reported_ready:
            self.reported_ready = True
            self.get_logger().info(
                "PX4 offboard input publishers are present: " + ", ".join(details)
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Px4ControlWatchdog()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
