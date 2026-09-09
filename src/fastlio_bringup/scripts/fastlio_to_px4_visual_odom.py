#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import TimesyncStatus, VehicleLocalPosition, VehicleOdometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from rigid_transform import (
    compose_pose,
    normalize_quaternion,
    quaternion_multiply,
    ros_flu_to_px4_frd_quaternion,
)


class FastlioToPx4VisualOdom(Node):
    """Convert FAST-LIO ``/Odometry`` into PX4 NED visual odometry."""

    def __init__(self) -> None:
        super().__init__("fastlio_to_px4_visual_odom")

        self.fastlio_topic = str(
            self.declare_parameter("fastlio_topic", "/Odometry").value
        )
        self.px4_local_topic = str(
            self.declare_parameter(
                "px4_local_topic", "/fmu/out/vehicle_local_position"
            ).value
        )
        self.timesync_topic = str(
            self.declare_parameter("timesync_topic", "/fmu/out/timesync_status").value
        )
        self.output_topic = str(
            self.declare_parameter(
                "output_topic", "/fmu/in/vehicle_visual_odometry"
            ).value
        )
        self.fastlio_pose_frame = str(
            self.declare_parameter("fastlio_pose_frame", "body").value
        )
        self.fastlio_world_frame = str(
            self.declare_parameter("fastlio_world_frame", "camera_init").value
        )
        self.vehicle_frame = str(self.declare_parameter("vehicle_frame", "base").value)

        self.body_to_base_translation = self._vector_parameter(
            "body_to_base_translation", [-0.011, -0.02329, -0.05588], 3
        )
        self.body_to_base_rotation_xyzw = normalize_quaternion(
            self._vector_parameter(
                "body_to_base_rotation_xyzw", [0.0, 0.0, 0.0, 1.0], 4
            )
        )
        self.publish_rate_limit = float(
            self.declare_parameter("publish_rate_limit", 50.0).value
        )
        self.align_when_px4_valid = bool(
            self.declare_parameter("align_when_px4_valid", True).value
        )
        self.use_px4_reference = bool(
            self.declare_parameter("use_px4_reference", False).value
        )
        self.max_px4_reference_abs_z = self._positive_parameter(
            "max_px4_reference_abs_z", 20.0
        )
        self.max_output_abs_z = self._positive_parameter("max_output_abs_z", 20.0)
        self.max_position_jump = self._positive_parameter("max_position_jump", 3.0)
        self.position_variance = float(
            self.declare_parameter("position_variance", 0.04).value
        )
        self.orientation_variance = float(
            self.declare_parameter("orientation_variance", 0.01).value
        )
        self.velocity_variance = float(
            self.declare_parameter("velocity_variance", 0.25).value
        )
        self.publish_orientation = bool(
            self.declare_parameter("publish_orientation", False).value
        )
        self.publish_velocity = bool(
            self.declare_parameter("publish_velocity", False).value
        )
        self.use_timesync_timestamp = bool(
            self.declare_parameter("use_timesync_timestamp", False).value
        )
        self.use_source_timestamp_age = bool(
            self.declare_parameter("use_source_timestamp_age", True).value
        )
        self.max_source_timestamp_age_sec = float(
            self.declare_parameter("max_source_timestamp_age_sec", 0.5).value
        )
        self.fastlio_y_to_px4_y_sign = float(
            self.declare_parameter("fastlio_y_to_px4_y_sign", -1.0).value
        )
        rotation = [
            float(value)
            for value in self.declare_parameter(
                "fastlio_to_px4_rotation", [0.0]
            ).value
        ]
        if len(rotation) == 1 and rotation[0] == 0.0:
            rotation = []
        if rotation and len(rotation) != 9:
            raise ValueError("fastlio_to_px4_rotation must contain 9 values")
        self.fastlio_to_px4_rotation = rotation or [
            1.0,
            0.0,
            0.0,
            0.0,
            self.fastlio_y_to_px4_y_sign,
            0.0,
            0.0,
            0.0,
            -1.0,
        ]
        if not all(math.isfinite(value) for value in self.fastlio_to_px4_rotation):
            raise ValueError("fastlio_to_px4_rotation must contain finite values")
        self.yaw_sign = float(self.declare_parameter("yaw_sign", -1.0).value)
        self.yaw_offset = float(self.declare_parameter("yaw_offset", 0.0).value)
        self.use_px4_heading_reference = bool(
            self.declare_parameter("use_px4_heading_reference", False).value
        )
        self.print_rate = float(self.declare_parameter("print_rate", 1.0).value)

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.latest_px4: Optional[VehicleLocalPosition] = None
        self.latest_timesync: Optional[TimesyncStatus] = None
        self.aligned = False
        self.ref_fastlio = [0.0, 0.0, 0.0]
        self.ref_px4 = [0.0, 0.0, 0.0]
        self.ref_fastlio_yaw = 0.0
        self.ref_px4_heading = 0.0
        self.yaw_offset_world_to_px4 = 0.0
        self.last_publish_sec: Optional[float] = None
        self.last_print_sec = 0.0
        self.last_pos: Optional[tuple[float, float, float]] = None
        self.last_warn_sec = 0.0

        self.pub = self.create_publisher(VehicleOdometry, self.output_topic, px4_qos)
        self.create_subscription(
            Odometry, self.fastlio_topic, self.fastlio_callback, sensor_qos
        )
        self.create_subscription(
            VehicleLocalPosition,
            self.px4_local_topic,
            self.px4_callback,
            px4_qos,
        )
        self.create_subscription(
            TimesyncStatus,
            self.timesync_topic,
            self.timesync_callback,
            px4_qos,
        )

        self.get_logger().info(
            "FAST-LIO -> PX4 visual odometry bridge active: "
            f"fastlio={self.fastlio_topic}, px4_local={self.px4_local_topic}, "
            f"out={self.output_topic}, pose_frame={self.fastlio_pose_frame}, "
            f"world_frame={self.fastlio_world_frame}, "
            f"vehicle_frame={self.vehicle_frame}, "
            f"body_to_base={tuple(round(value, 5) for value in self.body_to_base_translation)}, "
            f"publish_orientation={self.publish_orientation}, "
            f"publish_velocity={self.publish_velocity}"
        )

    def _vector_parameter(
        self, name: str, default: list[float], size: int
    ) -> tuple[float, ...]:
        values = tuple(float(value) for value in self.declare_parameter(name, default).value)
        if len(values) != size or not all(math.isfinite(value) for value in values):
            raise ValueError(f"{name} must contain {size} finite values")
        return values

    def _positive_parameter(self, name: str, default: float) -> float:
        value = float(self.declare_parameter(name, default).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be a finite positive value")
        return value

    def now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def px4_timestamp_us(self) -> int:
        if self.use_timesync_timestamp and self.latest_timesync is not None:
            return self.now_us() + int(self.latest_timesync.estimated_offset)
        return self.now_us()

    @staticmethod
    def message_stamp_sec(msg: Odometry) -> Optional[float]:
        value = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1e9
        return value if value > 0.0 and math.isfinite(value) else None

    def source_timestamp_age_sec(self, msg: Odometry) -> float:
        if not self.use_source_timestamp_age:
            return 0.0
        source_sec = self.message_stamp_sec(msg)
        if source_sec is None:
            return 0.0
        age_sec = self.now_sec() - source_sec
        if -0.02 <= age_sec <= self.max_source_timestamp_age_sec:
            return max(0.0, age_sec)
        self.warn_throttled(
            "FAST-LIO source timestamp is outside the accepted age window; "
            "using arrival time."
        )
        return 0.0

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
        if self.use_px4_reference and self.align_when_px4_valid and not (
            self.latest_px4.xy_valid and self.latest_px4.z_valid
        ):
            return False
        values = []
        if self.use_px4_reference:
            values.extend((self.latest_px4.x, self.latest_px4.y, self.latest_px4.z))
        if self.use_px4_heading_reference:
            values.append(self.latest_px4.heading)
        if not all(math.isfinite(float(value)) for value in values):
            return False
        return not self.use_px4_reference or abs(float(self.latest_px4.z)) <= self.max_px4_reference_abs_z

    def align_if_ready(
        self,
        vehicle_position: tuple[float, float, float],
        vehicle_orientation: tuple[float, float, float, float],
    ) -> bool:
        if self.aligned:
            return True
        if (self.use_px4_reference or self.use_px4_heading_reference) and not self.px4_reference_usable():
            self.warn_throttled(
                "Waiting for usable PX4 local position before FAST-LIO alignment: "
                f"xy_valid={getattr(self.latest_px4, 'xy_valid', None)}, "
                f"z_valid={getattr(self.latest_px4, 'z_valid', None)}"
            )
            return False

        self.ref_fastlio = [float(value) for value in vehicle_position]
        self.ref_fastlio_yaw = self.quaternion_to_yaw(*vehicle_orientation)
        if self.use_px4_reference:
            self.ref_px4 = [
                float(self.latest_px4.x),
                float(self.latest_px4.y),
                float(self.latest_px4.z),
            ]
        else:
            self.ref_px4 = [0.0, 0.0, 0.0]

        if self.use_px4_reference or self.use_px4_heading_reference:
            px4_heading = float(self.latest_px4.heading)
            self.ref_px4_heading = (
                self.wrap_angle(px4_heading) if math.isfinite(px4_heading) else 0.0
            )
            self.yaw_offset_world_to_px4 = self.wrap_angle(
                self.ref_px4_heading
                - self.yaw_sign * self.ref_fastlio_yaw
                - self.yaw_offset
            )
        else:
            self.ref_px4_heading = 0.0
            self.yaw_offset_world_to_px4 = 0.0
        self.aligned = True
        self.get_logger().info(
            "Initialized FAST-LIO visual odometry alignment: "
            f"ref_fastlio=({self.ref_fastlio[0]:.2f}, {self.ref_fastlio[1]:.2f}, "
            f"{self.ref_fastlio[2]:.2f}), "
            f"ref_px4=({self.ref_px4[0]:.2f}, {self.ref_px4[1]:.2f}, "
            f"{self.ref_px4[2]:.2f}), yaw_offset={self.yaw_offset_world_to_px4:.2f}"
        )
        return True

    def fastlio_body_pose_to_base(
        self, msg: Odometry
    ) -> Optional[tuple[tuple[float, float, float], tuple[float, float, float, float]]]:
        if self.fastlio_world_frame and msg.header.frame_id != self.fastlio_world_frame:
            self.warn_throttled(
                "Skipping FAST-LIO odometry with unexpected world frame: "
                f"'{msg.header.frame_id}' (expected '{self.fastlio_world_frame}')."
            )
            return None
        if self.fastlio_pose_frame and msg.child_frame_id != self.fastlio_pose_frame:
            self.warn_throttled(
                "Skipping FAST-LIO odometry with unexpected pose frame: "
                f"'{msg.child_frame_id}' (expected '{self.fastlio_pose_frame}')."
            )
            return None
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        try:
            return compose_pose(
                (float(position.x), float(position.y), float(position.z)),
                (
                    float(orientation.x),
                    float(orientation.y),
                    float(orientation.z),
                    float(orientation.w),
                ),
                self.body_to_base_translation,
                self.body_to_base_rotation_xyzw,
            )
        except ValueError as error:
            self.warn_throttled(f"Skipping FAST-LIO pose with invalid transform: {error}")
            return None

    def position_is_safe(self, position: tuple[float, float, float]) -> bool:
        if not all(math.isfinite(value) for value in position):
            self.warn_throttled("Skipping visual odometry publish: non-finite output position.")
            return False
        if abs(position[2]) > self.max_output_abs_z:
            self.warn_throttled(
                "Skipping visual odometry publish: output z exceeds limit "
                f"({position[2]:.2f} m > {self.max_output_abs_z:.2f} m)."
            )
            return False
        if self.last_pos is not None:
            jump = math.sqrt(
                sum((current - previous) ** 2 for current, previous in zip(position, self.last_pos))
            )
            if jump > self.max_position_jump:
                self.warn_throttled(
                    "Skipping visual odometry publish: position jump exceeds limit "
                    f"({jump:.2f} m > {self.max_position_jump:.2f} m)."
                )
                return False
        return True

    def fastlio_position_to_px4_ned(
        self, x: float, y: float, z: float
    ) -> tuple[float, float, float]:
        dx = x - self.ref_fastlio[0]
        dy = y - self.ref_fastlio[1]
        dz = z - self.ref_fastlio[2]
        matrix = self.fastlio_to_px4_rotation
        mapped = (
            matrix[0] * dx + matrix[1] * dy + matrix[2] * dz,
            matrix[3] * dx + matrix[4] * dy + matrix[5] * dz,
            matrix[6] * dx + matrix[7] * dy + matrix[8] * dz,
        )
        cosine = math.cos(self.yaw_offset_world_to_px4)
        sine = math.sin(self.yaw_offset_world_to_px4)
        return (
            self.ref_px4[0] + cosine * mapped[0] - sine * mapped[1],
            self.ref_px4[1] + sine * mapped[0] + cosine * mapped[1],
            self.ref_px4[2] + mapped[2],
        )

    def fastlio_vector_to_px4_ned(
        self, x: float, y: float, z: float
    ) -> tuple[float, float, float]:
        matrix = self.fastlio_to_px4_rotation
        mapped = (
            matrix[0] * x + matrix[1] * y + matrix[2] * z,
            matrix[3] * x + matrix[4] * y + matrix[5] * z,
            matrix[6] * x + matrix[7] * y + matrix[8] * z,
        )
        cosine = math.cos(self.yaw_offset_world_to_px4)
        sine = math.sin(self.yaw_offset_world_to_px4)
        return (
            cosine * mapped[0] - sine * mapped[1],
            sine * mapped[0] + cosine * mapped[1],
            mapped[2],
        )

    def orientation_to_px4_quaternion(
        self, fastlio_orientation: tuple[float, float, float, float]
    ) -> list[float]:
        """Return PX4's [w, x, y, z] quaternion without dropping tilt."""
        converted = ros_flu_to_px4_frd_quaternion(fastlio_orientation)
        yaw_adjustment = self.yaw_offset_world_to_px4 + self.yaw_offset
        if abs(yaw_adjustment) > 1e-12:
            half = 0.5 * yaw_adjustment
            converted = quaternion_multiply(
                (0.0, 0.0, math.sin(half), math.cos(half)), converted
            )
        x, y, z, w = converted
        return [w, x, y, z]

    def should_publish(self) -> bool:
        if self.publish_rate_limit <= 0.0:
            return True
        now = self.now_sec()
        if self.last_publish_sec is None or now - self.last_publish_sec >= 1.0 / self.publish_rate_limit:
            self.last_publish_sec = now
            return True
        return False

    def fastlio_callback(self, msg: Odometry) -> None:
        vehicle_pose = self.fastlio_body_pose_to_base(msg)
        if vehicle_pose is None:
            return
        vehicle_position, vehicle_orientation = vehicle_pose
        if not self.align_if_ready(vehicle_position, vehicle_orientation) or not self.should_publish():
            return

        position = self.fastlio_position_to_px4_ned(*vehicle_position)
        if not self.position_is_safe(position):
            return

        output = VehicleOdometry()
        timestamp = self.px4_timestamp_us()
        output.timestamp = timestamp
        source_age_sec = self.source_timestamp_age_sec(msg)
        output.timestamp_sample = max(1, timestamp - int(round(source_age_sec * 1e6)))
        output.pose_frame = VehicleOdometry.POSE_FRAME_NED
        output.position = [float(value) for value in position]

        if self.publish_orientation:
            output.q = self.orientation_to_px4_quaternion(vehicle_orientation)
        else:
            output.q = [float("nan")] * 4

        if self.publish_velocity:
            linear = msg.twist.twist.linear
            output.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
            output.velocity = list(
                self.fastlio_vector_to_px4_ned(
                    float(linear.x), float(linear.y), float(linear.z)
                )
            )
        else:
            output.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
            output.velocity = [float("nan")] * 3

        output.angular_velocity = [float("nan")] * 3
        output.position_variance = [self.position_variance] * 3
        output.orientation_variance = [self.orientation_variance] * 3
        output.velocity_variance = [self.velocity_variance] * 3
        output.reset_counter = 0
        output.quality = 100
        self.pub.publish(output)
        self.last_pos = position

        now = self.now_sec()
        if self.print_rate > 0.0 and now - self.last_print_sec >= 1.0 / self.print_rate:
            self.last_print_sec = now
            self.get_logger().info(
                "FAST-LIO visual odometry published | "
                f"pos_ned=({position[0]:+.2f}, {position[1]:+.2f}, {position[2]:+.2f}) "
                f"sample_age={source_age_sec * 1e3:.1f} ms "
                f"frame_in='{msg.header.frame_id}' child='{msg.child_frame_id}' "
                f"output_body='{self.vehicle_frame}'"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FastlioToPx4VisualOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
