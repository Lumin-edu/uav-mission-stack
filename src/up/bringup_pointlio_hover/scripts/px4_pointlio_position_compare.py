#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class Px4PointlioPositionCompare(Node):
    def __init__(self) -> None:
        super().__init__("px4_pointlio_position_compare")
        self.px4_topic = self.declare_parameter(
            "px4_topic", "/fmu/out/vehicle_local_position"
        ).value
        legacy_fastlio_topic = self.declare_parameter("fastlio_topic", "").value
        self.pointlio_topic = self.declare_parameter(
            "pointlio_topic",
            legacy_fastlio_topic if legacy_fastlio_topic else "/drone_0_visual_slam_odom",
        ).value
        self.print_rate = float(self.declare_parameter("print_rate", 2.0).value)
        self.warn_timeout = float(self.declare_parameter("warn_timeout", 1.0).value)
        self.direction_epsilon = float(self.declare_parameter("direction_epsilon", 0.15).value)

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.latest_px4: Optional[VehicleLocalPosition] = None
        self.latest_pointlio: Optional[Odometry] = None
        self.latest_px4_time: Optional[float] = None
        self.latest_pointlio_time: Optional[float] = None

        self.ref_px4: Optional[tuple[float, float, float]] = None
        self.ref_pointlio: Optional[tuple[float, float, float]] = None

        self.create_subscription(
            VehicleLocalPosition, self.px4_topic, self.px4_callback, px4_qos
        )
        self.create_subscription(
            Odometry, self.pointlio_topic, self.pointlio_callback, 10
        )

        period = 1.0 / self.print_rate if self.print_rate > 0.0 else 0.5
        self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            "Comparing PX4 local position and Point-LIO odom: "
            f"px4={self.px4_topic}, pointlio={self.pointlio_topic}, "
            f"print_rate={self.print_rate:.2f} Hz, "
            f"direction_epsilon={self.direction_epsilon:.2f} m"
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def px4_callback(self, msg: VehicleLocalPosition) -> None:
        self.latest_px4 = msg
        self.latest_px4_time = self.now_sec()

    def pointlio_callback(self, msg: Odometry) -> None:
        self.latest_pointlio = msg
        self.latest_pointlio_time = self.now_sec()

    def maybe_init_reference(self) -> bool:
        if self.latest_px4 is None or self.latest_pointlio is None:
            return False
        if self.ref_px4 is None:
            self.ref_px4 = (
                float(self.latest_px4.x),
                float(self.latest_px4.y),
                float(self.latest_px4.z),
            )
        if self.ref_pointlio is None:
            p = self.latest_pointlio.pose.pose.position
            self.ref_pointlio = (float(p.x), float(p.y), float(p.z))
        return True

    def timer_callback(self) -> None:
        if self.latest_px4 is None or self.latest_pointlio is None:
            self.get_logger().warn(
                "Waiting for position data: "
                f"px4_received={self.latest_px4 is not None}, "
                f"pointlio_received={self.latest_pointlio is not None}"
            )
            return

        now = self.now_sec()
        px4_age = now - self.latest_px4_time if self.latest_px4_time is not None else math.inf
        pointlio_age = (
            now - self.latest_pointlio_time
            if self.latest_pointlio_time is not None
            else math.inf
        )
        if px4_age > self.warn_timeout or pointlio_age > self.warn_timeout:
            self.get_logger().warn(
                f"Stale position input: px4_age={px4_age:.2f}s, "
                f"pointlio_age={pointlio_age:.2f}s"
            )

        if not self.maybe_init_reference():
            return

        px4 = self.latest_px4
        pointlio_p = self.latest_pointlio.pose.pose.position
        pointlio_v = self.latest_pointlio.twist.twist.linear

        px4_rel = (
            float(px4.x) - self.ref_px4[0],
            float(px4.y) - self.ref_px4[1],
            float(px4.z) - self.ref_px4[2],
        )
        pointlio_rel = (
            float(pointlio_p.x) - self.ref_pointlio[0],
            float(pointlio_p.y) - self.ref_pointlio[1],
            float(pointlio_p.z) - self.ref_pointlio[2],
        )

        # PX4 local z is usually down-positive. Flip only relative z for a quick
        # z-up comparison with Point-LIO/EGO odom.
        px4_rel_z_up = (px4_rel[0], px4_rel[1], -px4_rel[2])
        diff_rel = (
            pointlio_rel[0] - px4_rel_z_up[0],
            pointlio_rel[1] - px4_rel_z_up[1],
            pointlio_rel[2] - px4_rel_z_up[2],
        )
        diff_norm = math.sqrt(
            diff_rel[0] * diff_rel[0]
            + diff_rel[1] * diff_rel[1]
            + diff_rel[2] * diff_rel[2]
        )

        axis_report = self.build_axis_report(px4_rel_z_up, pointlio_rel)

        self.get_logger().info(
            "increment compare | "
            f"PX4(z-up)=({px4_rel_z_up[0]:+.2f}, {px4_rel_z_up[1]:+.2f}, {px4_rel_z_up[2]:+.2f}) m | "
            f"Point-LIO=({pointlio_rel[0]:+.2f}, {pointlio_rel[1]:+.2f}, {pointlio_rel[2]:+.2f}) m | "
            f"diff=({diff_rel[0]:+.2f}, {diff_rel[1]:+.2f}, {diff_rel[2]:+.2f}) m | "
            f"|diff|={diff_norm:.2f} m | {axis_report}"
        )

        self.get_logger().info(
            "raw state | "
            f"PX4 pos=({px4.x:+.2f}, {px4.y:+.2f}, {px4.z:+.2f}) "
            f"vel=({px4.vx:+.2f}, {px4.vy:+.2f}, {px4.vz:+.2f}) "
            f"valid=({bool(px4.xy_valid)}, {bool(px4.z_valid)}) | "
            f"Point-LIO pos=({pointlio_p.x:+.2f}, {pointlio_p.y:+.2f}, {pointlio_p.z:+.2f}) "
            f"vel=({pointlio_v.x:+.2f}, {pointlio_v.y:+.2f}, {pointlio_v.z:+.2f}) "
            f"frame='{self.latest_pointlio.header.frame_id}'"
        )

    def axis_direction(self, value: float) -> str:
        if value > self.direction_epsilon:
            return "+"
        if value < -self.direction_epsilon:
            return "-"
        return "0"

    def build_axis_report(
        self,
        px4_rel_z_up: tuple[float, float, float],
        pointlio_rel: tuple[float, float, float],
    ) -> str:
        labels = ("x", "y", "z")
        parts = []
        mismatch = []
        for label, px4_value, pointlio_value in zip(labels, px4_rel_z_up, pointlio_rel):
            px4_dir = self.axis_direction(px4_value)
            pointlio_dir = self.axis_direction(pointlio_value)
            parts.append(f"{label}[px4={px4_dir}, pointlio={pointlio_dir}]")
            if px4_dir != "0" and pointlio_dir != "0" and px4_dir != pointlio_dir:
                mismatch.append(label)

        if mismatch:
            return " ".join(parts) + f" | direction mismatch on {','.join(mismatch)}"
        return " ".join(parts) + " | direction looks consistent"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Px4PointlioPositionCompare()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
