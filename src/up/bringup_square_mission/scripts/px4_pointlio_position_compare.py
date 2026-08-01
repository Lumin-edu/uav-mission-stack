#!/usr/bin/env python3
"""
PX4 本地位置与桥接后视觉里程计增量对比调试节点。

用于在飞行前验证实际发送给 PX4 的 base/NED 位姿与 PX4 EKF2 输出是否一致。
"""

import math
from typing import Optional

import rclpy
from px4_msgs.msg import VehicleLocalPosition, VehicleOdometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class Px4PointlioPositionCompare(Node):
    """
    对比 PX4 本地位置和桥接节点输出的视觉里程计增量。

    输出三个视角的对比：
      1. PX4 NED 增量 vs 桥接节点发送的 base/NED 增量
      2. 原始状态（位置、速度、有效性）对比
      3. 任务坐标系（x_right, y_forward, z_up）增量对比
    """
    def __init__(self) -> None:
        super().__init__("px4_pointlio_position_compare")
        self.px4_topic = self.declare_parameter(
            "px4_topic", "/fmu/out/vehicle_local_position"
        ).value
        self.visual_odom_topic = self.declare_parameter(
            "visual_odom_topic", "/fmu/in/vehicle_visual_odometry"
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
        self.latest_visual_odom: Optional[VehicleOdometry] = None
        self.latest_px4_time: Optional[float] = None
        self.latest_visual_odom_time: Optional[float] = None

        self.ref_px4: Optional[tuple[float, float, float]] = None
        self.ref_visual_odom: Optional[tuple[float, float, float]] = None
        self.ref_heading: Optional[float] = None

        self.create_subscription(
            VehicleLocalPosition, self.px4_topic, self.px4_callback, px4_qos
        )
        self.create_subscription(
            VehicleOdometry, self.visual_odom_topic, self.visual_odom_callback, px4_qos
        )

        period = 1.0 / self.print_rate if self.print_rate > 0.0 else 0.5
        self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            "Comparing PX4 local position and bridge visual odometry: "
            f"px4={self.px4_topic}, visual_odom={self.visual_odom_topic}, "
            f"print_rate={self.print_rate:.2f} Hz, "
            f"direction_epsilon={self.direction_epsilon:.2f} m"
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def px4_callback(self, msg: VehicleLocalPosition) -> None:
        self.latest_px4 = msg
        self.latest_px4_time = self.now_sec()

    def visual_odom_callback(self, msg: VehicleOdometry) -> None:
        self.latest_visual_odom = msg
        self.latest_visual_odom_time = self.now_sec()

    def maybe_init_reference(self) -> bool:
        if self.latest_px4 is None or self.latest_visual_odom is None:
            return False
        if self.ref_px4 is None:
            self.ref_px4 = (
                float(self.latest_px4.x),
                float(self.latest_px4.y),
                float(self.latest_px4.z),
            )
            heading = float(self.latest_px4.heading)
            self.ref_heading = self.wrap_angle(heading) if math.isfinite(heading) else 0.0
        if self.ref_visual_odom is None:
            self.ref_visual_odom = tuple(
                float(value) for value in self.latest_visual_odom.position
            )
        return True

    def timer_callback(self) -> None:
        if self.latest_px4 is None or self.latest_visual_odom is None:
            self.get_logger().warn(
                "Waiting for position data: "
                f"px4_received={self.latest_px4 is not None}, "
                f"visual_odom_received={self.latest_visual_odom is not None}"
            )
            return

        now = self.now_sec()
        px4_age = now - self.latest_px4_time if self.latest_px4_time is not None else math.inf
        visual_odom_age = (
            now - self.latest_visual_odom_time
            if self.latest_visual_odom_time is not None
            else math.inf
        )
        if px4_age > self.warn_timeout or visual_odom_age > self.warn_timeout:
            self.get_logger().warn(
                f"Stale position input: px4_age={px4_age:.2f}s, "
                f"visual_odom_age={visual_odom_age:.2f}s"
            )

        visual_odom = self.latest_visual_odom
        px4 = self.latest_px4
        if visual_odom.pose_frame != VehicleOdometry.POSE_FRAME_NED:
            self.get_logger().warn(
                "Bridge visual odometry is not NED; skipping comparison: "
                f"pose_frame={visual_odom.pose_frame}"
            )
            return
        if not all(math.isfinite(float(value)) for value in visual_odom.position):
            self.get_logger().warn("Bridge visual odometry position is non-finite; skipping comparison.")
            return
        if not all(math.isfinite(float(value)) for value in (px4.x, px4.y, px4.z)):
            self.get_logger().warn("PX4 local position is non-finite; skipping comparison.")
            return

        if not self.maybe_init_reference():
            return

        visual_odom_position = tuple(float(value) for value in visual_odom.position)

        px4_rel = (
            float(px4.x) - self.ref_px4[0],
            float(px4.y) - self.ref_px4[1],
            float(px4.z) - self.ref_px4[2],
        )
        visual_odom_rel = (
            visual_odom_position[0] - self.ref_visual_odom[0],
            visual_odom_position[1] - self.ref_visual_odom[1],
            visual_odom_position[2] - self.ref_visual_odom[2],
        )
        px4_task_rel = self.ned_rel_to_task_frame(px4_rel)
        visual_odom_task_rel = self.ned_rel_to_task_frame(visual_odom_rel)

        diff_rel = (
            visual_odom_rel[0] - px4_rel[0],
            visual_odom_rel[1] - px4_rel[1],
            visual_odom_rel[2] - px4_rel[2],
        )
        diff_norm = math.sqrt(
            diff_rel[0] * diff_rel[0]
            + diff_rel[1] * diff_rel[1]
            + diff_rel[2] * diff_rel[2]
        )

        axis_report = self.build_axis_report(px4_rel, visual_odom_rel)

        self.get_logger().info(
            "NED increment compare | "
            f"PX4=({px4_rel[0]:+.2f}, {px4_rel[1]:+.2f}, {px4_rel[2]:+.2f}) m | "
            f"bridge=({visual_odom_rel[0]:+.2f}, "
            f"{visual_odom_rel[1]:+.2f}, {visual_odom_rel[2]:+.2f}) m | "
            f"diff=({diff_rel[0]:+.2f}, {diff_rel[1]:+.2f}, {diff_rel[2]:+.2f}) m | "
            f"|diff|={diff_norm:.2f} m | {axis_report}"
        )

        self.get_logger().info(
            "raw state | "
            f"PX4 pos=({px4.x:+.2f}, {px4.y:+.2f}, {px4.z:+.2f}) "
            f"vel=({px4.vx:+.2f}, {px4.vy:+.2f}, {px4.vz:+.2f}) "
            f"valid=({bool(px4.xy_valid)}, {bool(px4.z_valid)}) | "
            f"bridge NED pos=({visual_odom_position[0]:+.2f}, "
            f"{visual_odom_position[1]:+.2f}, {visual_odom_position[2]:+.2f}) "
            f"pose_frame={visual_odom.pose_frame} velocity_frame={visual_odom.velocity_frame}"
        )

        self.get_logger().info(
            "task-frame relative movement | "
            "axis=(x_right, y_forward, z_up) | "
            f"PX4=({px4_task_rel[0]:+.2f}, {px4_task_rel[1]:+.2f}, {px4_task_rel[2]:+.2f}) m | "
            f"bridge=({visual_odom_task_rel[0]:+.2f}, "
            f"{visual_odom_task_rel[1]:+.2f}, {visual_odom_task_rel[2]:+.2f}) m"
        )

    def wrap_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def ned_rel_to_task_frame(self, rel_ned: tuple[float, float, float]) -> tuple[float, float, float]:
        """
        将 NED 相对增量转换到任务坐标系（x_right, y_forward, z_up）。

        任务坐标系：x=右，y=前（初始机头方向），z=上
        NED 坐标系：N=前，E=右，D=下
        通过绕初始偏航角旋转来实现转换。
        """
        heading = self.ref_heading if self.ref_heading is not None else 0.0
        c = math.cos(heading)
        s = math.sin(heading)
        # 旋转矩阵的逆：将 NED 转回任务坐标系
        x_right = -rel_ned[0] * s + rel_ned[1] * c
        y_forward = rel_ned[0] * c + rel_ned[1] * s
        z_up = -rel_ned[2]
        return (x_right, y_forward, z_up)

    def axis_direction(self, value: float) -> str:
        if value > self.direction_epsilon:
            return "+"
        if value < -self.direction_epsilon:
            return "-"
        return "0"

    def build_axis_report(
        self,
        px4_rel: tuple[float, float, float],
        visual_odom_rel: tuple[float, float, float],
    ) -> str:
        labels = ("N/x", "E/y", "D/z")
        parts = []
        mismatch = []
        for label, px4_value, visual_odom_value in zip(labels, px4_rel, visual_odom_rel):
            px4_dir = self.axis_direction(px4_value)
            visual_odom_dir = self.axis_direction(visual_odom_value)
            parts.append(f"{label}[px4={px4_dir}, bridge={visual_odom_dir}]")
            if px4_dir != "0" and visual_odom_dir != "0" and px4_dir != visual_odom_dir:
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
