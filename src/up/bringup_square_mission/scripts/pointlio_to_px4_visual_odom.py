#!/usr/bin/env python3
"""
Point-LIO -> PX4 EKF2 视觉里程计桥接节点。

将 Point-LIO 的 /Odometry 消息转换到 PX4 NED 坐标系，
并通过 /fmu/in/vehicle_visual_odometry 发送给 PX4 EKF2 作为外部视觉里程计输入。
核心功能：
  1. 坐标系对齐（Point-LIO 世界系 -> PX4 NED 本地系）
  2. 偏航角变换
  3. 位置跳跃保护与合法性检查
  4. 发布速率限制
"""

import math
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import TimesyncStatus, VehicleLocalPosition, VehicleOdometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from rigid_transform import compose_pose, normalize_quaternion


class PointlioToPx4VisualOdom(Node):
    """
    Point-LIO 到 PX4 视觉里程计的转换节点。

    坐标系说明：
      - Point-LIO 坐标系：通常为 ROS 标准坐标系（x 前, y 左, z 上）
      - PX4 NED 坐标系：N（北/前）, E（东/右）, D（下/-z）
      - 转换公式：
          PX4_N = Point-LIO_x
          PX4_E = pointlio_y_to_px4_y_sign * Point-LIO_y   (默认 -1，即 y 左 -> E 右)
          PX4_D = -Point-LIO_z
    """
    def __init__(self) -> None:
        super().__init__("pointlio_to_px4_visual_odom")

        legacy_fastlio_topic = self.declare_parameter("fastlio_topic", "").value
        self.pointlio_topic = self.declare_parameter(
            "pointlio_topic", legacy_fastlio_topic if legacy_fastlio_topic else "/Odometry"
        ).value
        self.px4_local_topic = self.declare_parameter(
            "px4_local_topic", "/fmu/out/vehicle_local_position"
        ).value
        self.timesync_topic = self.declare_parameter(
            "timesync_topic", "/fmu/out/timesync_status"
        ).value
        self.output_topic = self.declare_parameter(
            "output_topic", "/fmu/in/vehicle_visual_odometry"
        ).value
        self.pointlio_pose_frame = str(
            self.declare_parameter("pointlio_pose_frame", "base_link").value
        )
        self.vehicle_frame = str(self.declare_parameter("vehicle_frame", "base").value)

        # Point-LIO 的 base_link 数值对应 MID360 IMU 原点；base 表示机体中心。
        # T_base_link_base 的平移是 base 原点在 base_link 坐标系中的坐标：
        # [-0.011, -0.02329, 0.04412] + [0, 0, -0.10]
        # = [-0.011, -0.02329, -0.05588] m。
        self.base_link_to_base_translation = self._vector_parameter(
            "base_link_to_base_translation", [-0.011, -0.02329, -0.05588], 3
        )
        self.base_link_to_base_rotation_xyzw = normalize_quaternion(
            self._vector_parameter(
                "base_link_to_base_rotation_xyzw", [0.0, 0.0, 0.0, 1.0], 4
            )
        )
        self.publish_rate_limit = float(self.declare_parameter("publish_rate_limit", 50.0).value)
        self.align_when_px4_valid = bool(self.declare_parameter("align_when_px4_valid", True).value)
        self.use_px4_reference = bool(self.declare_parameter("use_px4_reference", False).value)
        self.max_px4_reference_abs_z = float(
            self.declare_parameter("max_px4_reference_abs_z", 20.0).value
        )
        self.max_output_abs_z = float(self.declare_parameter("max_output_abs_z", 20.0).value)
        self.max_position_jump = float(self.declare_parameter("max_position_jump", 3.0).value)
        self.position_variance = float(self.declare_parameter("position_variance", 0.04).value)
        self.orientation_variance = float(self.declare_parameter("orientation_variance", 0.01).value)
        self.velocity_variance = float(self.declare_parameter("velocity_variance", 0.25).value)
        self.publish_orientation = bool(self.declare_parameter("publish_orientation", False).value)
        self.publish_velocity = bool(self.declare_parameter("publish_velocity", False).value)
        self.use_timesync_timestamp = bool(
            self.declare_parameter("use_timesync_timestamp", False).value
        )
        legacy_fastlio_y_to_px4_y_sign = float(
            self.declare_parameter("fastlio_y_to_px4_y_sign", -1.0).value
        )
        self.pointlio_y_to_px4_y_sign = float(
            self.declare_parameter(
                "pointlio_y_to_px4_y_sign", legacy_fastlio_y_to_px4_y_sign
            ).value
        )
        self.yaw_sign = float(self.declare_parameter("yaw_sign", -1.0).value)
        self.yaw_offset = float(self.declare_parameter("yaw_offset", 0.0).value)
        self.print_rate = float(self.declare_parameter("print_rate", 1.0).value)

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.latest_px4: Optional[VehicleLocalPosition] = None
        self.latest_timesync: Optional[TimesyncStatus] = None
        self.aligned = False
        self.ref_pointlio = [0.0, 0.0, 0.0]
        self.ref_px4 = [0.0, 0.0, 0.0]
        self.ref_pointlio_yaw = 0.0
        self.ref_px4_heading = 0.0
        self.yaw_offset_world_to_px4 = 0.0
        self.last_publish_sec: Optional[float] = None
        self.last_print_sec = 0.0
        self.last_pos: Optional[tuple[float, float, float]] = None
        self.last_warn_sec = 0.0

        self.pub = self.create_publisher(VehicleOdometry, self.output_topic, px4_qos)
        self.create_subscription(
            Odometry, self.pointlio_topic, self.pointlio_callback, best_effort_qos
        )
        self.create_subscription(VehicleLocalPosition, self.px4_local_topic, self.px4_callback, px4_qos)
        self.create_subscription(TimesyncStatus, self.timesync_topic, self.timesync_callback, px4_qos)

        self.get_logger().info(
            "Point-LIO -> PX4 visual odometry bridge active: "
            f"pointlio={self.pointlio_topic}, px4_local={self.px4_local_topic}, "
            f"out={self.output_topic}, publish_orientation={self.publish_orientation}, "
            f"publish_velocity={self.publish_velocity}, "
            f"use_px4_reference={self.use_px4_reference}, "
            f"use_timesync_timestamp={self.use_timesync_timestamp}, "
            f"pointlio_y_to_px4_y_sign={self.pointlio_y_to_px4_y_sign:.1f}, "
            f"yaw_sign={self.yaw_sign:.1f}, yaw_offset={self.yaw_offset:.2f}, "
            f"pose_interpretation=T_odom_{self.pointlio_pose_frame}, "
            f"output_body={self.vehicle_frame}, "
            f"t_base_link_base="
            f"{tuple(round(value, 5) for value in self.base_link_to_base_translation)}, "
            f"q_base_link_base_xyzw="
            f"{tuple(round(value, 5) for value in self.base_link_to_base_rotation_xyzw)}"
        )

    def _vector_parameter(self, name: str, default: list[float], size: int) -> tuple[float, ...]:
        values = tuple(float(value) for value in self.declare_parameter(name, default).value)
        if len(values) != size or not all(math.isfinite(value) for value in values):
            raise ValueError(f"{name} must contain {size} finite values")
        return values

    def now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def px4_timestamp_us(self) -> int:
        if self.use_timesync_timestamp and self.latest_timesync is not None:
            return self.now_us() + int(self.latest_timesync.estimated_offset)
        return self.now_us()

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def px4_callback(self, msg: VehicleLocalPosition) -> None:
        self.latest_px4 = msg

    def timesync_callback(self, msg: TimesyncStatus) -> None:
        self.latest_timesync = msg

    def quaternion_to_yaw(self, x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def wrap_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def warn_throttled(self, message: str, period: float = 2.0) -> None:
        now = self.now_sec()
        if now - self.last_warn_sec >= period:
            self.last_warn_sec = now
            self.get_logger().warn(message)

    def px4_reference_usable(self) -> bool:
        if self.latest_px4 is None:
            return False
        if self.align_when_px4_valid and not (self.latest_px4.xy_valid and self.latest_px4.z_valid):
            return False
        if not all(
            math.isfinite(value)
            for value in (
                self.latest_px4.x,
                self.latest_px4.y,
                self.latest_px4.z,
            )
        ):
            return False
        return abs(float(self.latest_px4.z)) <= self.max_px4_reference_abs_z

    def align_if_ready(
        self,
        vehicle_position: tuple[float, float, float],
        vehicle_orientation: tuple[float, float, float, float],
    ) -> bool:
        """
        首次接收数据时进行坐标系对齐。

        记录 Point-LIO 起始位置和偏航角作为参考原点（ref_pointlio），
        若 use_px4_reference=True，则同时记录 PX4 位置作为绝对参考。
        计算 Point-LIO 世界系到 PX4 NED 系的偏航偏移量。
        """
        if self.aligned:
            return True
        if self.use_px4_reference and not self.px4_reference_usable():
            px4_z = float(self.latest_px4.z) if self.latest_px4 is not None else float("nan")
            self.warn_throttled(
                "Waiting for usable PX4 local position before visual odom alignment: "
                f"xy_valid={getattr(self.latest_px4, 'xy_valid', None)}, "
                f"z_valid={getattr(self.latest_px4, 'z_valid', None)}, z={px4_z:.2f}"
            )
            return False

        # 记录补偿后 base 的起始位置和偏航角作为参考原点。
        self.ref_pointlio = [float(value) for value in vehicle_position]
        self.ref_pointlio_yaw = self.quaternion_to_yaw(*vehicle_orientation)
        if self.use_px4_reference:
            # 使用 PX4 本地位置作为绝对参考
            self.ref_px4 = [
                float(self.latest_px4.x),
                float(self.latest_px4.y),
                float(self.latest_px4.z),
            ]
            px4_heading = float(self.latest_px4.heading)
            self.ref_px4_heading = self.wrap_angle(px4_heading) if math.isfinite(px4_heading) else 0.0
            # 计算偏航偏移：PX4偏航 - (yaw_sign * PointLIO偏航) - 额外偏移
            self.yaw_offset_world_to_px4 = self.wrap_angle(
                self.ref_px4_heading
                - self.yaw_sign * self.ref_pointlio_yaw
                - self.yaw_offset
            )
        else:
            # 不以 PX4 为参考时，使用纯相对增量模式
            self.ref_px4 = [0.0, 0.0, 0.0]
            self.ref_px4_heading = 0.0
            self.yaw_offset_world_to_px4 = 0.0
        self.aligned = True
        self.get_logger().info(
            "Initialized Point-LIO visual odom alignment: "
            f"ref_pointlio=({self.ref_pointlio[0]:.2f}, {self.ref_pointlio[1]:.2f}, {self.ref_pointlio[2]:.2f}), "
            f"ref_px4=({self.ref_px4[0]:.2f}, {self.ref_px4[1]:.2f}, {self.ref_px4[2]:.2f}), "
            f"yaw_offset={self.yaw_offset_world_to_px4:.2f}"
        )
        return True

    def pointlio_base_link_pose_to_base(
        self, msg: Odometry
    ) -> Optional[tuple[tuple[float, float, float], tuple[float, float, float, float]]]:
        """把 Point-LIO 的 base_link 位姿补偿成机体中心 base 位姿。"""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        try:
            # T_odom_base = T_odom_base_link * T_base_link_base
            # p_odom_base = p_odom_base_link
            #               + R_odom_base_link * t_base_link_base
            return compose_pose(
                (float(p.x), float(p.y), float(p.z)),
                (float(q.x), float(q.y), float(q.z), float(q.w)),
                self.base_link_to_base_translation,
                self.base_link_to_base_rotation_xyzw,
            )
        except ValueError as error:
            self.warn_throttled(f"Skipping visual odom publish: invalid Point-LIO pose: {error}")
            return None

    def position_is_safe(self, pos: tuple[float, float, float]) -> bool:
        if not all(math.isfinite(value) for value in pos):
            self.warn_throttled("Skipping visual odom publish: non-finite output position.")
            return False
        if abs(pos[2]) > self.max_output_abs_z:
            self.warn_throttled(
                "Skipping visual odom publish: output z is outside sanity limit "
                f"z={pos[2]:.2f}, limit={self.max_output_abs_z:.2f}"
            )
            return False
        if self.last_pos is not None:
            jump = math.sqrt(
                (pos[0] - self.last_pos[0]) ** 2
                + (pos[1] - self.last_pos[1]) ** 2
                + (pos[2] - self.last_pos[2]) ** 2
            )
            if jump > self.max_position_jump:
                self.warn_throttled(
                    "Skipping visual odom publish: position jump is outside sanity limit "
                    f"jump={jump:.2f}, limit={self.max_position_jump:.2f}"
                )
                return False
        return True

    def pointlio_position_to_px4_ned(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        """
        将 Point-LIO 位置转换到 PX4 NED 坐标系。

        步骤：
          1. 计算相对于参考原点的增量
          2. 翻转 y 轴方向（pointlio_y_to_px4_y_sign）
          3. 绕偏航偏移角旋转
          4. 加上 PX4 参考位置，z 取反（NED 下为正）
        """
        # 相对增量
        dx = x - self.ref_pointlio[0]
        dy = self.pointlio_y_to_px4_y_sign * (y - self.ref_pointlio[1])
        dz = z - self.ref_pointlio[2]
        # 绕偏航偏移旋转
        cy = math.cos(self.yaw_offset_world_to_px4)
        sy = math.sin(self.yaw_offset_world_to_px4)
        px4_dx = cy * dx - sy * dy
        px4_dy = sy * dx + cy * dy
        return (
            self.ref_px4[0] + px4_dx,
            self.ref_px4[1] + px4_dy,
            self.ref_px4[2] - dz,  # NED: D轴向下为正
        )

    def pointlio_vector_to_px4_ned(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        px4_y = self.pointlio_y_to_px4_y_sign * y
        cy = math.cos(self.yaw_offset_world_to_px4)
        sy = math.sin(self.yaw_offset_world_to_px4)
        return (
            cy * x - sy * px4_y,
            sy * x + cy * px4_y,
            -z,
        )

    def yaw_to_px4_quaternion(self, pointlio_yaw: float) -> list[float]:
        yaw = self.wrap_angle(
            self.yaw_sign * pointlio_yaw + self.yaw_offset_world_to_px4 + self.yaw_offset
        )
        half = 0.5 * yaw
        return [math.cos(half), 0.0, 0.0, math.sin(half)]

    def should_publish(self) -> bool:
        if self.publish_rate_limit <= 0.0:
            return True
        now = self.now_sec()
        if self.last_publish_sec is None or (now - self.last_publish_sec) >= 1.0 / self.publish_rate_limit:
            self.last_publish_sec = now
            return True
        return False

    def pointlio_callback(self, msg: Odometry) -> None:
        """
        Point-LIO 里程计回调：执行坐标转换并发布到 PX4。

        流程：
          1. 首次调用时进行坐标系对齐
          2. 检查发布速率限制
          3. 转换位置到 PX4 NED
          4. 安全校验（有限性、z 范围、位置跳跃）
          5. 组装 VehicleOdometry 消息并发布
        """
        # 唯一的杆臂补偿点：base_link（IMU 原点）-> base（机体中心）。
        vehicle_pose = self.pointlio_base_link_pose_to_base(msg)
        if vehicle_pose is None:
            return
        vehicle_position, vehicle_orientation = vehicle_pose

        # 首次调用时使用补偿后的 base 进行坐标系对齐。
        if not self.align_if_ready(vehicle_position, vehicle_orientation) or not self.should_publish():
            return

        # 补偿到此结束；base -> PX4 NED 继续调用原有转换逻辑，不重复使用外参。
        pos = self.pointlio_position_to_px4_ned(*vehicle_position)
        # 安全校验
        if not self.position_is_safe(pos):
            return

        out = VehicleOdometry()
        stamp = self.px4_timestamp_us()
        out.timestamp = stamp
        out.timestamp_sample = stamp
        out.pose_frame = VehicleOdometry.POSE_FRAME_NED
        # PX4 收到的是补偿后的 base 机体中心位置。
        out.position = [float(pos[0]), float(pos[1]), float(pos[2])]

        if self.publish_orientation:
            yaw = self.quaternion_to_yaw(*vehicle_orientation)
            out.q = self.yaw_to_px4_quaternion(yaw)
        else:
            out.q = [float("nan"), float("nan"), float("nan"), float("nan")]

        if self.publish_velocity:
            v = msg.twist.twist.linear
            vel = self.pointlio_vector_to_px4_ned(float(v.x), float(v.y), float(v.z))
            out.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
            out.velocity = [float(vel[0]), float(vel[1]), float(vel[2])]
        else:
            out.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
            out.velocity = [float("nan"), float("nan"), float("nan")]

        out.angular_velocity = [float("nan"), float("nan"), float("nan")]
        out.position_variance = [self.position_variance] * 3
        out.orientation_variance = [self.orientation_variance] * 3
        out.velocity_variance = [self.velocity_variance] * 3
        out.reset_counter = 0
        out.quality = 100
        self.pub.publish(out)
        self.last_pos = pos

        now = self.now_sec()
        if self.print_rate > 0.0 and (now - self.last_print_sec) >= 1.0 / self.print_rate:
            self.last_print_sec = now
            yaw_text = ""
            if self.publish_orientation:
                pointlio_yaw = self.quaternion_to_yaw(*vehicle_orientation)
                px4_yaw = self.wrap_angle(
                    self.yaw_sign * pointlio_yaw
                    + self.yaw_offset_world_to_px4
                    + self.yaw_offset
                )
                yaw_text = (
                    f" yaw_pointlio={math.degrees(pointlio_yaw):+.1f}deg"
                    f" yaw_px4={math.degrees(px4_yaw):+.1f}deg"
                )
            self.get_logger().info(
                "visual odom published | "
                f"pos_ned=({pos[0]:+.2f}, {pos[1]:+.2f}, {pos[2]:+.2f}) "
                f"frame_in='{msg.header.frame_id}' child='{msg.child_frame_id}'"
                f" interpreted_child='{self.pointlio_pose_frame}' output_body='{self.vehicle_frame}'"
                f"{yaw_text}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PointlioToPx4VisualOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
