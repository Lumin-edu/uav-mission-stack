#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class Px4ControlWatchdog(Node):
    def __init__(self) -> None:
        super().__init__("fastlio_px4_control_watchdog")
        self.control_source = str(self.declare_parameter("control_source", "fixed_point_hover").value)
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
            if not publishers:
                missing.append(topic)
        if missing:
            self.reported_ready = False
            self.get_logger().error(
                "PX4 has missing FAST-LIO hover inputs: "
                f"{missing}. Details: {', '.join(details)}"
            )
        elif not self.reported_ready:
            self.reported_ready = True
            self.get_logger().info("PX4 Offboard input publishers are present: " + ", ".join(details))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Px4ControlWatchdog()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
