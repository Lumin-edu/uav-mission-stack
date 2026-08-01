#!/usr/bin/env python3

import math
import threading
import time
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import TimesyncStatus, VehicleLocalPosition, VehicleOdometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock


class PointlioToPx4VisualOdom(Node):
    """Simulation Point-LIO ROS-FLU to PX4-NED odometry adapter."""

    def __init__(self) -> None:
        super().__init__("pointlio_to_px4_visual_odom")
        self.pointlio_topic = str(self.declare_parameter("pointlio_topic", "/sim/pointlio/odom").value)
        self.px4_local_topic = str(
            self.declare_parameter("px4_local_topic", "/fmu/out/vehicle_local_position").value
        )
        self.timesync_topic = str(
            self.declare_parameter("timesync_topic", "/fmu/out/timesync_status").value
        )
        self.output_topic = str(
            self.declare_parameter("output_topic", "/fmu/in/vehicle_visual_odometry").value
        )
        self.publish_rate_limit = float(self.declare_parameter("publish_rate_limit", 50.0).value)
        self.use_px4_reference = bool(self.declare_parameter("use_px4_reference", False).value)
        self.use_px4_heading_reference = bool(
            self.declare_parameter("use_px4_heading_reference", True).value
        )
        self.align_when_px4_valid = bool(self.declare_parameter("align_when_px4_valid", True).value)
        self.px4_reference_wait_sec = float(
            self.declare_parameter("px4_reference_wait_sec", 3.0).value
        )
        self.max_px4_reference_abs_z = float(
            self.declare_parameter("max_px4_reference_abs_z", 20.0).value
        )
        self.max_output_abs_z = float(self.declare_parameter("max_output_abs_z", 20.0).value)
        self.max_output_abs_xy = float(
            self.declare_parameter("max_output_abs_xy", 20.0).value
        )
        self.max_position_rate_mps = float(
            self.declare_parameter("max_position_rate_mps", 4.0).value
        )
        self.max_position_jump = float(self.declare_parameter("max_position_jump", 3.0).value)
        self.publish_orientation = bool(self.declare_parameter("publish_orientation", True).value)
        self.publish_velocity = bool(self.declare_parameter("publish_velocity", False).value)
        self.use_timesync_timestamp = bool(
            self.declare_parameter("use_timesync_timestamp", False).value
        )
        self.use_source_timestamp_age = bool(
            self.declare_parameter("use_source_timestamp_age", True).value
        )
        self.max_source_timestamp_age_sec = float(
            self.declare_parameter("max_source_timestamp_age_sec", 0.5).value
        )
        self.clock_extrapolation_limit_sec = float(
            self.declare_parameter("clock_extrapolation_limit_sec", 0.25).value
        )
        self.yaw_sign = float(self.declare_parameter("yaw_sign", -1.0).value)
        self.yaw_offset = float(self.declare_parameter("yaw_offset", 0.0).value)
        # Kept as a named parameter for parity with the verified hardware adapter.
        self.pointlio_y_to_px4_y_sign = float(
            self.declare_parameter("pointlio_y_to_px4_y_sign", -1.0).value
        )
        self.position_variance = float(self.declare_parameter("position_variance", 0.04).value)
        self.orientation_variance = float(self.declare_parameter("orientation_variance", 0.01).value)
        self.velocity_variance = float(self.declare_parameter("velocity_variance", 0.25).value)
        self.print_rate = float(self.declare_parameter("print_rate", 1.0).value)
        rotation = list(
            self.declare_parameter(
                "pointlio_to_px4_rotation",
                [1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0],
            ).value
        )
        if len(rotation) != 9:
            raise ValueError("pointlio_to_px4_rotation must contain 9 row-major values")
        self.rotation = [float(v) for v in rotation]
        self.latest_px4: Optional[VehicleLocalPosition] = None
        self.latest_timesync: Optional[TimesyncStatus] = None
        self.aligned = False
        self.ref_pointlio = (0.0, 0.0, 0.0)
        self.ref_px4 = (0.0, 0.0, 0.0)
        self.yaw_offset_world_to_px4 = 0.0
        self.last_publish_sec: Optional[float] = None
        self.last_position: Optional[tuple[float, float, float]] = None
        self.last_position_sample_sec: Optional[float] = None
        self.latest_sample_sec: Optional[float] = None
        self.alignment_wait_start_sec: Optional[float] = None
        self.last_print_sec = 0.0
        self.last_warn_sec = -math.inf
        self.latest_sim_clock_sec: Optional[float] = None
        self.latest_sim_clock_monotonic_sec: Optional[float] = None
        self.sim_clock_rate = 1.0
        self.sim_clock_lock = threading.Lock()
        self.sim_clock_callback_group = MutuallyExclusiveCallbackGroup()

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
        clock_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.publisher = self.create_publisher(VehicleOdometry, self.output_topic, px4_qos)
        self.create_subscription(Odometry, self.pointlio_topic, self.pointlio_callback, sensor_qos)
        self.create_subscription(VehicleLocalPosition, self.px4_local_topic, self.px4_callback, px4_qos)
        self.create_subscription(TimesyncStatus, self.timesync_topic, self.timesync_callback, px4_qos)
        self.create_subscription(
            Clock,
            "/clock",
            self.sim_clock_callback,
            clock_qos,
            callback_group=self.sim_clock_callback_group,
        )
        self.get_logger().info(
            "Simulation Point-LIO -> PX4 visual odom: "
            f"{self.pointlio_topic} -> {self.output_topic}, 50 Hz, ROS-FLU->NED, "
            f"source_timestamp_age={self.use_source_timestamp_age}"
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def now_us(self) -> int:
        # VehicleOdometry timestamps must remain in the PX4/system time domain
        # even though this node uses Gazebo time to measure sensor age.
        return time.time_ns() // 1000

    def px4_timestamp_us(self) -> int:
        if self.use_timesync_timestamp and self.latest_timesync is not None:
            return self.now_us() + int(self.latest_timesync.estimated_offset)
        return self.now_us()

    @staticmethod
    def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    @staticmethod
    def wrap_angle(value: float) -> float:
        return math.atan2(math.sin(value), math.cos(value))

    def warn(self, message: str) -> None:
        now = self.now_sec()
        if now - self.last_warn_sec >= 2.0:
            self.last_warn_sec = now
            self.get_logger().warn(message)

    def px4_callback(self, msg: VehicleLocalPosition) -> None:
        self.latest_px4 = msg

    def timesync_callback(self, msg: TimesyncStatus) -> None:
        self.latest_timesync = msg

    def sim_clock_callback(self, msg: Clock) -> None:
        clock_sec = float(msg.clock.sec) + float(msg.clock.nanosec) / 1e9
        if math.isfinite(clock_sec) and clock_sec >= 0.0:
            monotonic_sec = time.monotonic()
            with self.sim_clock_lock:
                if (
                    self.latest_sim_clock_sec is not None
                    and self.latest_sim_clock_monotonic_sec is not None
                ):
                    sim_delta = clock_sec - self.latest_sim_clock_sec
                    wall_delta = monotonic_sec - self.latest_sim_clock_monotonic_sec
                    if sim_delta > 0.0 and wall_delta > 0.0:
                        measured_rate = sim_delta / wall_delta
                        if 0.1 <= measured_rate <= 2.0:
                            self.sim_clock_rate = 0.8 * self.sim_clock_rate + 0.2 * measured_rate
                self.latest_sim_clock_sec = clock_sec
                self.latest_sim_clock_monotonic_sec = monotonic_sec

    def align_if_ready(self, msg: Odometry) -> bool:
        if self.aligned:
            return True
        px4 = self.latest_px4
        px4_reference_valid = False
        if self.use_px4_reference or self.use_px4_heading_reference:
            if px4 is None:
                self.warn("Waiting for PX4 local position before visual odom alignment.")
                return False
            if self.use_px4_reference:
                px4_reference_valid = bool(px4.xy_valid and px4.z_valid)
                if px4_reference_valid:
                    self.alignment_wait_start_sec = None
                elif self.align_when_px4_valid:
                    now = self.now_sec()
                    if self.alignment_wait_start_sec is None:
                        self.alignment_wait_start_sec = now
                    if now - self.alignment_wait_start_sec < self.px4_reference_wait_sec:
                        self.warn("Waiting for valid PX4 x/y/z before visual odom alignment.")
                        return False
                    self.warn(
                        "PX4 local position stayed invalid during cold start; "
                        "using zero external-vision reference."
                    )
            values = [float(px4.heading)] if self.use_px4_heading_reference else []
            if px4_reference_valid:
                values.extend((float(px4.x), float(px4.y), float(px4.z)))
            if not all(math.isfinite(v) for v in values):
                self.warn("Waiting for finite PX4 reference before visual odom alignment.")
                return False
            if px4_reference_valid and abs(float(px4.z)) > self.max_px4_reference_abs_z:
                self.warn("Ignoring PX4 reference with unreasonable z value.")
                return False

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.ref_pointlio = (float(p.x), float(p.y), float(p.z))
        self.ref_px4 = (
            (float(px4.x), float(px4.y), float(px4.z))
            if px4_reference_valid and px4 is not None
            else (0.0, 0.0, 0.0)
        )
        if self.use_px4_heading_reference and px4 is not None:
            pointlio_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
            self.yaw_offset_world_to_px4 = self.wrap_angle(
                float(px4.heading) - self.yaw_sign * pointlio_yaw - self.yaw_offset
            )
        self.aligned = True
        self.get_logger().info(
            "Captured simulation visual odom reference: "
            f"pointlio={self.ref_pointlio}, px4={self.ref_px4}, "
            f"yaw_offset={self.yaw_offset_world_to_px4:.3f}"
        )
        return True

    def map_vector(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        m = self.rotation
        mapped = (
            m[0] * x + m[1] * y + m[2] * z,
            m[3] * x + m[4] * y + m[5] * z,
            m[6] * x + m[7] * y + m[8] * z,
        )
        c, s = math.cos(self.yaw_offset_world_to_px4), math.sin(self.yaw_offset_world_to_px4)
        return (c * mapped[0] - s * mapped[1], s * mapped[0] + c * mapped[1], mapped[2])

    def map_position(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        dx, dy, dz = (x - self.ref_pointlio[0], y - self.ref_pointlio[1], z - self.ref_pointlio[2])
        mapped = self.map_vector(dx, dy, dz)
        return tuple(self.ref_px4[i] + mapped[i] for i in range(3))

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

        # Read /clock explicitly. With the multithreaded executor this callback
        # can advance while Point-LIO is processing an odometry callback, unlike
        # the node's implicit simulated-time callback in a single-threaded spin.
        with self.sim_clock_lock:
            sim_clock_sec = self.latest_sim_clock_sec
            clock_monotonic_sec = self.latest_sim_clock_monotonic_sec
            clock_rate = self.sim_clock_rate
        if sim_clock_sec is None or clock_monotonic_sec is None:
            self.warn("Waiting for /clock before applying Point-LIO sample age.")
            return 0.0

        # Gazebo publishes /clock at 10 Hz while the LiDAR timestamps scans at
        # arbitrary instants between those updates. Interpolate only inside a
        # short bounded window using the measured simulation/steady-clock rate.
        clock_elapsed_sec = max(0.0, time.monotonic() - clock_monotonic_sec)
        extrapolated_sec = min(clock_elapsed_sec, self.clock_extrapolation_limit_sec)
        sim_now_sec = sim_clock_sec + extrapolated_sec * clock_rate
        age_sec = sim_now_sec - source_sec
        if -0.02 <= age_sec <= self.max_source_timestamp_age_sec:
            return max(0.0, age_sec)
        self.warn(
            "Point-LIO source timestamp is outside the accepted age window; "
            f"raw_age={age_sec * 1e3:.1f} ms source={source_sec:.6f} "
            f"sim_now={sim_now_sec:.6f} clock_rate={clock_rate:.3f}; using arrival time."
        )
        return 0.0

    def safe_position(self, pos: tuple[float, float, float]) -> bool:
        if (
            not all(math.isfinite(v) for v in pos)
            or math.hypot(pos[0], pos[1]) > self.max_output_abs_xy
            or abs(pos[2]) > self.max_output_abs_z
        ):
            self.warn("Skipping simulated visual odom with invalid or unreasonable position.")
            return False
        if self.last_position is not None:
            jump = math.sqrt(sum((pos[i] - self.last_position[i]) ** 2 for i in range(3)))
            if jump > self.max_position_jump:
                self.warn(f"Skipping simulated visual odom jump {jump:.2f} m.")
                return False
            if (
                self.latest_sample_sec is not None
                and self.last_position_sample_sec is not None
                and self.latest_sample_sec > self.last_position_sample_sec
            ):
                rate = jump / (self.latest_sample_sec - self.last_position_sample_sec)
                if rate > self.max_position_rate_mps:
                    self.warn(
                        "Skipping simulated visual odom: position rate "
                        f"{rate:.2f} m/s exceeds {self.max_position_rate_mps:.2f} m/s."
                    )
                    return False
        return True

    def pointlio_callback(self, msg: Odometry) -> None:
        if not self.align_if_ready(msg):
            return
        self.latest_sample_sec = self.message_stamp_sec(msg)
        now = self.now_sec()
        if self.publish_rate_limit > 0.0 and self.last_publish_sec is not None:
            if now - self.last_publish_sec < 1.0 / self.publish_rate_limit:
                return
        p = msg.pose.pose.position
        pos = self.map_position(float(p.x), float(p.y), float(p.z))
        if not self.safe_position(pos):
            return
        self.last_publish_sec = now
        out = VehicleOdometry()
        stamp = self.px4_timestamp_us()
        out.timestamp = stamp
        source_age_sec = self.source_timestamp_age_sec(msg)
        out.timestamp_sample = max(1, stamp - int(round(source_age_sec * 1e6)))
        out.pose_frame = VehicleOdometry.POSE_FRAME_NED
        out.position = list(pos)
        if self.publish_orientation:
            q = msg.pose.pose.orientation
            yaw = self.wrap_angle(
                self.yaw_sign * self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
                + self.yaw_offset_world_to_px4
                + self.yaw_offset
            )
            out.q = [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)]
        else:
            out.q = [math.nan] * 4
        if self.publish_velocity:
            v = msg.twist.twist.linear
            out.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
            out.velocity = list(self.map_vector(float(v.x), float(v.y), float(v.z)))
        else:
            out.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
            out.velocity = [math.nan] * 3
        out.angular_velocity = [math.nan] * 3
        out.position_variance = [self.position_variance] * 3
        out.orientation_variance = [self.orientation_variance] * 3
        out.velocity_variance = [self.velocity_variance] * 3
        out.reset_counter = 0
        out.quality = 100
        self.publisher.publish(out)
        self.last_position = pos
        self.last_position_sample_sec = self.latest_sample_sec
        if self.print_rate > 0.0 and now - self.last_print_sec >= 1.0 / self.print_rate:
            self.last_print_sec = now
            self.get_logger().info(
                f"sim visual odom published: pos_ned=({pos[0]:+.2f}, {pos[1]:+.2f}, "
                f"{pos[2]:+.2f}) sample_age={source_age_sec * 1e3:.1f} ms"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PointlioToPx4VisualOdom()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
