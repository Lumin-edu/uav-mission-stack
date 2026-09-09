#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from px4_msgs.msg import VehicleLocalPosition, VehicleOdometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class Px4FastlioPositionCompare(Node):
    """Compare PX4 local position with the bridge output in the same NED frame."""

    def __init__(self) -> None:
        super().__init__("px4_fastlio_position_compare")
        self.px4_topic = str(
            self.declare_parameter("px4_topic", "/fmu/out/vehicle_local_position").value
        )
        self.visual_odom_topic = str(
            self.declare_parameter(
                "visual_odom_topic", "/fmu/in/vehicle_visual_odometry"
            ).value
        )
        self.print_rate = float(self.declare_parameter("print_rate", 2.0).value)
        self.warn_timeout = float(self.declare_parameter("warn_timeout", 1.0).value)
        self.direction_epsilon = float(
            self.declare_parameter("direction_epsilon", 0.15).value
        )
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.latest_px4: Optional[VehicleLocalPosition] = None
        self.latest_visual_odom: Optional[VehicleOdometry] = None
        self.latest_px4_time: Optional[float] = None
        self.latest_visual_odom_time: Optional[float] = None
        self.ref_px4: Optional[tuple[float, float, float]] = None
        self.ref_visual_odom: Optional[tuple[float, float, float]] = None
        self.create_subscription(VehicleLocalPosition, self.px4_topic, self.px4_callback, qos)
        self.create_subscription(VehicleOdometry, self.visual_odom_topic, self.visual_odom_callback, qos)
        self.create_timer(1.0 / self.print_rate if self.print_rate > 0.0 else 0.5, self.timer_callback)
        self.get_logger().info(
            "Comparing PX4 local position and FAST-LIO bridge: "
            f"px4={self.px4_topic}, visual_odom={self.visual_odom_topic}"
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def px4_callback(self, msg: VehicleLocalPosition) -> None:
        self.latest_px4 = msg
        self.latest_px4_time = self.now_sec()

    def visual_odom_callback(self, msg: VehicleOdometry) -> None:
        self.latest_visual_odom = msg
        self.latest_visual_odom_time = self.now_sec()

    def axis_direction(self, value: float) -> str:
        if value > self.direction_epsilon:
            return "+"
        if value < -self.direction_epsilon:
            return "-"
        return "0"

    def build_axis_report(
        self,
        px4_rel: tuple[float, float, float],
        visual_rel: tuple[float, float, float],
    ) -> str:
        parts = []
        mismatches = []
        for label, px4_value, visual_value in zip(("x", "y", "z"), px4_rel, visual_rel):
            px4_direction = self.axis_direction(px4_value)
            visual_direction = self.axis_direction(visual_value)
            parts.append(f"{label}[px4={px4_direction}, bridge={visual_direction}]")
            if (
                px4_direction != "0"
                and visual_direction != "0"
                and px4_direction != visual_direction
            ):
                mismatches.append(label)
        suffix = f" | direction mismatch on {','.join(mismatches)}" if mismatches else " | direction looks consistent"
        return " ".join(parts) + suffix

    def timer_callback(self) -> None:
        if self.latest_px4 is None or self.latest_visual_odom is None:
            self.get_logger().warning(
                "Waiting for comparison data: "
                f"px4_received={self.latest_px4 is not None}, "
                f"bridge_received={self.latest_visual_odom is not None}"
            )
            return
        now = self.now_sec()
        px4_age = now - self.latest_px4_time if self.latest_px4_time is not None else math.inf
        bridge_age = now - self.latest_visual_odom_time if self.latest_visual_odom_time is not None else math.inf
        if px4_age > self.warn_timeout or bridge_age > self.warn_timeout:
            self.get_logger().warning(
                f"Stale comparison input: px4_age={px4_age:.2f}s, bridge_age={bridge_age:.2f}s"
            )
        px4 = self.latest_px4
        bridge = self.latest_visual_odom
        if bridge.pose_frame != VehicleOdometry.POSE_FRAME_NED:
            self.get_logger().warning(
                f"Bridge pose is not NED (pose_frame={bridge.pose_frame}); skipping"
            )
            return
        bridge_position = tuple(float(value) for value in bridge.position)
        px4_position = (float(px4.x), float(px4.y), float(px4.z))
        if not all(math.isfinite(value) for value in bridge_position + px4_position):
            self.get_logger().warning("Non-finite comparison position; skipping")
            return
        if self.ref_px4 is None:
            self.ref_px4 = px4_position
        if self.ref_visual_odom is None:
            self.ref_visual_odom = bridge_position
        px4_rel = tuple(value - reference for value, reference in zip(px4_position, self.ref_px4))
        visual_rel = tuple(value - reference for value, reference in zip(bridge_position, self.ref_visual_odom))
        diff_rel = tuple(visual - px4_value for visual, px4_value in zip(visual_rel, px4_rel))
        diff_norm = math.sqrt(sum(value * value for value in diff_rel))
        self.get_logger().info(
            "NED increment compare | "
            f"PX4=({px4_rel[0]:+.2f}, {px4_rel[1]:+.2f}, {px4_rel[2]:+.2f}) m | "
            f"bridge=({visual_rel[0]:+.2f}, {visual_rel[1]:+.2f}, {visual_rel[2]:+.2f}) m | "
            f"diff=({diff_rel[0]:+.2f}, {diff_rel[1]:+.2f}, {diff_rel[2]:+.2f}) m | "
            f"|diff|={diff_norm:.2f} m | {self.build_axis_report(px4_rel, visual_rel)}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Px4FastlioPositionCompare()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
