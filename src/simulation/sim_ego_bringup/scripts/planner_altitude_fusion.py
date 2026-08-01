#!/usr/bin/env python3

"""Build the odometry and point clouds consumed by EGO in simulation.

Point-LIO remains the horizontal/map estimator and continues to feed PX4
external vision directly.  EGO receives Point-LIO x/y/orientation with the
independent PX4 vertical estimate substituted for z/vz.  The same z offset is
applied to each registered scan at that scan's timestamp so odometry and
obstacles stay in one frame.  Corrected scans also feed a rolling voxel map;
Point-LIO's historical map is deliberately not shifted by the latest offset.
"""

import copy
from collections import deque
import math
import struct
import time
from typing import Deque, Dict, Optional, Tuple

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float64


class PlannerAltitudeFusion(Node):
    """Fuse Point-LIO horizontal state with PX4's independent vertical state."""

    def __init__(self) -> None:
        super().__init__("planner_altitude_fusion")

        self.raw_odom_topic = str(
            self.declare_parameter("raw_odom_topic", "/sim/pointlio/odom_safe").value
        )
        self.raw_cloud_topic = str(
            self.declare_parameter(
                "raw_cloud_topic", "/sim/pointlio/cloud_registered"
            ).value
        )
        self.px4_topic = str(
            self.declare_parameter(
                "px4_position_topic", "/fmu/out/vehicle_local_position"
            ).value
        )
        self.source_health_topic = str(
            self.declare_parameter(
                "source_health_topic", "/sim/pointlio/odom_healthy"
            ).value
        )
        self.fused_odom_topic = str(
            self.declare_parameter("fused_odom_topic", "/sim/ego/odom_fused").value
        )
        self.fused_cloud_topic = str(
            self.declare_parameter(
                "fused_cloud_topic", "/sim/ego/cloud_registered_fused"
            ).value
        )
        self.fused_map_topic = str(
            self.declare_parameter(
                "fused_map_topic", "/sim/ego/map_cloud_fused"
            ).value
        )
        self.health_topic = str(
            self.declare_parameter(
                "health_topic", "/sim/ego/height_fusion_healthy"
            ).value
        )
        self.correction_topic = str(
            self.declare_parameter(
                "correction_topic", "/sim/ego/pointlio_z_correction"
            ).value
        )
        self.max_px4_age_sec = float(
            self.declare_parameter("max_px4_age_sec", 0.30).value
        )
        self.max_odom_age_sec = float(
            self.declare_parameter("max_odom_age_sec", 0.30).value
        )
        self.max_correction_age_sec = float(
            self.declare_parameter("max_correction_age_sec", 0.30).value
        )
        self.correction_history_sec = float(
            self.declare_parameter("correction_history_sec", 3.0).value
        )
        self.cloud_sync_tolerance_sec = float(
            self.declare_parameter("cloud_sync_tolerance_sec", 0.15).value
        )
        self.max_abs_correction_m = float(
            self.declare_parameter("max_abs_correction_m", 3.0).value
        )
        self.map_voxel_size = float(
            self.declare_parameter("map_voxel_size", 0.20).value
        )
        self.map_radius_xy = float(
            self.declare_parameter("map_radius_xy", 10.0).value
        )
        self.map_min_z = float(self.declare_parameter("map_min_z", -0.5).value)
        self.map_max_z = float(self.declare_parameter("map_max_z", 3.5).value)
        self.map_publish_rate_hz = float(
            self.declare_parameter("map_publish_rate_hz", 1.0).value
        )
        self.map_voxel_ttl_sec = float(
            self.declare_parameter("map_voxel_ttl_sec", 0.0).value
        )
        self.map_max_voxels = int(
            self.declare_parameter("map_max_voxels", 150000).value
        )
        self.map_frame_id = str(
            self.declare_parameter("map_frame_id", "world").value
        )
        self.require_vertical_velocity = bool(
            self.declare_parameter("require_vertical_velocity", True).value
        )
        self.print_rate_hz = float(
            self.declare_parameter("print_rate_hz", 1.0).value
        )
        if min(
            self.max_px4_age_sec,
            self.max_odom_age_sec,
            self.max_correction_age_sec,
            self.correction_history_sec,
            self.cloud_sync_tolerance_sec,
            self.max_abs_correction_m,
            self.map_voxel_size,
            self.map_radius_xy,
            self.map_publish_rate_hz,
        ) <= 0.0:
            raise ValueError("fusion ages, map dimensions, and rates must be positive")
        if self.map_max_z <= self.map_min_z:
            raise ValueError("map_max_z must be greater than map_min_z")
        if self.map_voxel_ttl_sec < 0.0 or self.map_max_voxels <= 0:
            raise ValueError("map_voxel_ttl_sec must be non-negative and map_max_voxels positive")

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        health_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.odom_pub = self.create_publisher(Odometry, self.fused_odom_topic, 10)
        self.cloud_pub = self.create_publisher(PointCloud2, self.fused_cloud_topic, 10)
        self.map_pub = self.create_publisher(PointCloud2, self.fused_map_topic, map_qos)
        self.health_pub = self.create_publisher(Bool, self.health_topic, health_qos)
        self.correction_pub = self.create_publisher(Float64, self.correction_topic, 10)
        self.create_subscription(Odometry, self.raw_odom_topic, self.odom_callback, 10)
        self.create_subscription(PointCloud2, self.raw_cloud_topic, self.cloud_callback, 10)
        self.create_subscription(VehicleLocalPosition, self.px4_topic, self.px4_callback, px4_qos)
        self.create_subscription(Bool, self.source_health_topic, self.health_callback, health_qos)

        self.latest_px4: Optional[VehicleLocalPosition] = None
        self.latest_px4_monotonic: Optional[float] = None
        self.latest_odom_monotonic: Optional[float] = None
        self.source_healthy = False
        self.ref_pointlio_z: Optional[float] = None
        self.ref_px4_z: Optional[float] = None
        self.last_px4_z_reset_counter: Optional[int] = None
        self.latest_z_correction: Optional[float] = None
        self.latest_correction_monotonic: Optional[float] = None
        self.correction_history: Deque[Tuple[float, float]] = deque()
        self.last_correction_stamp_sec: Optional[float] = None
        self.latest_fused_position: Optional[Tuple[float, float, float]] = None
        self.map_voxels: Dict[Tuple[int, int, int], float] = {}
        self.latest_cloud_stamp = None
        self.last_map_point_count = 0
        self.last_health: Optional[bool] = None
        self.last_print_monotonic = -math.inf
        self.last_warn_monotonic = -math.inf
        self.create_timer(0.1, self.health_timer)
        self.create_timer(1.0 / self.map_publish_rate_hz, self.publish_rolling_map)

        self.get_logger().info(
            "Simulation planner altitude fusion: "
            f"Point-LIO x/y + PX4 z, {self.raw_odom_topic} -> {self.fused_odom_topic}, "
            f"{self.raw_cloud_topic} -> {self.fused_cloud_topic}, "
            f"corrected-scan rolling map -> {self.fused_map_topic} "
            f"(voxel={self.map_voxel_size:.2f} m, radius={self.map_radius_xy:.1f} m)"
        )

    @staticmethod
    def finite(*values: float) -> bool:
        return all(math.isfinite(float(value)) for value in values)

    def warn(self, message: str) -> None:
        now = time.monotonic()
        if now - self.last_warn_monotonic >= 2.0:
            self.last_warn_monotonic = now
            self.get_logger().warn(message)

    def health_callback(self, msg: Bool) -> None:
        self.source_healthy = bool(msg.data)

    def px4_message_valid(self, msg: Optional[VehicleLocalPosition]) -> bool:
        if msg is None or not msg.z_valid or not self.finite(msg.z):
            return False
        if self.require_vertical_velocity and (
            not msg.v_z_valid or not self.finite(msg.vz)
        ):
            return False
        return True

    def px4_is_fresh(self) -> bool:
        return bool(
            self.px4_message_valid(self.latest_px4)
            and self.latest_px4_monotonic is not None
            and time.monotonic() - self.latest_px4_monotonic <= self.max_px4_age_sec
        )

    def px4_callback(self, msg: VehicleLocalPosition) -> None:
        now = time.monotonic()
        if not self.px4_message_valid(msg):
            self.latest_px4 = msg
            self.latest_px4_monotonic = now
            return

        reset_counter = int(msg.z_reset_counter)
        if self.last_px4_z_reset_counter is None:
            self.last_px4_z_reset_counter = reset_counter
        elif reset_counter != self.last_px4_z_reset_counter:
            delta_z = float(msg.delta_z)
            if self.ref_px4_z is not None and math.isfinite(delta_z):
                # PX4 reports new_z = old_z + delta_z. Shift the stored origin
                # by the same amount so the ROS-up planner height stays continuous.
                self.ref_px4_z += delta_z
                self.get_logger().warn(
                    "PX4 vertical position reset detected; rebased planner height "
                    f"by delta_z={delta_z:+.3f} m."
                )
            self.last_px4_z_reset_counter = reset_counter

        self.latest_px4 = msg
        self.latest_px4_monotonic = now

    def capture_reference(self, pointlio_z: float) -> bool:
        if self.ref_pointlio_z is not None and self.ref_px4_z is not None:
            return True
        if not self.source_healthy or not self.px4_is_fresh() or self.latest_px4 is None:
            return False
        self.ref_pointlio_z = float(pointlio_z)
        self.ref_px4_z = float(self.latest_px4.z)
        self.last_px4_z_reset_counter = int(self.latest_px4.z_reset_counter)
        self.get_logger().info(
            "Captured planner height reference: "
            f"pointlio_z={self.ref_pointlio_z:+.3f} m, "
            f"px4_ned_z={self.ref_px4_z:+.3f} m."
        )
        return True

    def odom_callback(self, msg: Odometry) -> None:
        self.latest_odom_monotonic = time.monotonic()
        raw = msg.pose.pose.position
        if not self.finite(raw.x, raw.y, raw.z):
            self.warn("Rejecting non-finite Point-LIO odometry in altitude fusion.")
            return
        if not self.capture_reference(float(raw.z)):
            self.warn("Waiting for healthy Point-LIO and valid PX4 vertical state.")
            return
        if not self.source_healthy or not self.px4_is_fresh() or self.latest_px4 is None:
            self.warn("Planner altitude fusion stopped because a source is stale or unhealthy.")
            return

        fused_z = self.ref_pointlio_z - (float(self.latest_px4.z) - self.ref_px4_z)
        correction = fused_z - float(raw.z)
        if not self.finite(fused_z, correction) or abs(correction) > self.max_abs_correction_m:
            self.warn(
                "Planner altitude correction rejected: "
                f"correction={correction:+.3f} m, limit={self.max_abs_correction_m:.3f} m."
            )
            return

        fused = copy.deepcopy(msg)
        fused.pose.pose.position.z = fused_z
        if self.latest_px4.v_z_valid and self.finite(self.latest_px4.vz):
            fused.twist.twist.linear.z = -float(self.latest_px4.vz)
        if self.finite(self.latest_px4.epv) and float(self.latest_px4.epv) >= 0.0:
            fused.pose.covariance[14] = float(self.latest_px4.epv) ** 2
        if self.finite(self.latest_px4.evv) and float(self.latest_px4.evv) >= 0.0:
            fused.twist.covariance[14] = float(self.latest_px4.evv) ** 2

        self.latest_z_correction = correction
        self.latest_correction_monotonic = time.monotonic()
        self.latest_fused_position = (float(raw.x), float(raw.y), fused_z)
        self.remember_correction(self.stamp_to_sec(msg.header.stamp), correction)
        self.odom_pub.publish(fused)
        self.correction_pub.publish(Float64(data=correction))

        now = time.monotonic()
        if self.print_rate_hz > 0.0 and now - self.last_print_monotonic >= 1.0 / self.print_rate_hz:
            self.last_print_monotonic = now
            self.get_logger().info(
                "planner height fused | "
                f"pointlio_z={float(raw.z):+.3f} px4_ned_z={float(self.latest_px4.z):+.3f} "
                f"ego_z={fused_z:+.3f} cloud_dz={correction:+.3f} m"
            )

    @staticmethod
    def shift_cloud_z(msg: PointCloud2, correction: float) -> Optional[PointCloud2]:
        z_fields = [field for field in msg.fields if field.name == "z"]
        if len(z_fields) != 1:
            return None
        field = z_fields[0]
        if field.datatype != PointField.FLOAT32 or field.count != 1:
            return None
        if msg.point_step <= 0 or field.offset + 4 > msg.point_step:
            return None

        data = bytearray(msg.data)
        expected_size = int(msg.row_step) * int(msg.height)
        if expected_size > len(data):
            return None
        fmt = ">f" if msg.is_bigendian else "<f"
        for row in range(int(msg.height)):
            row_base = row * int(msg.row_step)
            for column in range(int(msg.width)):
                offset = row_base + column * int(msg.point_step) + int(field.offset)
                value = struct.unpack_from(fmt, data, offset)[0]
                if math.isfinite(value):
                    struct.pack_into(fmt, data, offset, value + correction)

        shifted = copy.deepcopy(msg)
        shifted.data = bytes(data)
        return shifted

    def cloud_callback(self, msg: PointCloud2) -> None:
        correction = self.correction_for_stamp(self.stamp_to_sec(msg.header.stamp))
        if correction is None:
            self.warn(
                "Registered cloud withheld because no timestamp-matched height "
                "correction is available."
            )
            return
        if (
            not self.source_healthy
            or not self.px4_is_fresh()
            or self.latest_correction_monotonic is None
            or time.monotonic() - self.latest_correction_monotonic
            > self.max_correction_age_sec
        ):
            self.warn("Registered cloud withheld until a fresh fused height is available.")
            return
        shifted = self.shift_cloud_z(msg, correction)
        if shifted is None:
            self.warn("Registered cloud does not contain a supported float32 z field.")
            return
        self.cloud_pub.publish(shifted)
        self.accumulate_scan(msg, correction)

    @staticmethod
    def stamp_to_sec(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def remember_correction(self, stamp_sec: float, correction: float) -> None:
        if (
            self.last_correction_stamp_sec is not None
            and stamp_sec < self.last_correction_stamp_sec - 0.5
        ):
            self.correction_history.clear()
            self.map_voxels.clear()
            self.get_logger().warn(
                "ROS time moved backwards; cleared correction history and rolling map."
            )
        self.last_correction_stamp_sec = stamp_sec
        self.correction_history.append((stamp_sec, correction))
        cutoff = stamp_sec - self.correction_history_sec
        while self.correction_history and self.correction_history[0][0] < cutoff:
            self.correction_history.popleft()

    def correction_for_stamp(self, stamp_sec: float) -> Optional[float]:
        if not self.correction_history:
            return None
        matched_stamp, correction = min(
            self.correction_history, key=lambda sample: abs(sample[0] - stamp_sec)
        )
        if abs(matched_stamp - stamp_sec) > self.cloud_sync_tolerance_sec:
            return None
        return correction

    def accumulate_scan(self, msg: PointCloud2, correction: float) -> None:
        if self.latest_fused_position is None:
            return
        try:
            structured = point_cloud2.read_points(
                msg, field_names=["x", "y", "z"], skip_nans=True
            )
        except (AssertionError, KeyError, TypeError, ValueError) as exc:
            self.warn(f"Cannot decode registered cloud for rolling map: {exc}")
            return
        if len(structured) == 0:
            return

        points = np.column_stack(
            (structured["x"], structured["y"], structured["z"])
        ).astype(np.float64, copy=False)
        points[:, 2] += correction
        center_x, center_y, _ = self.latest_fused_position
        keep = (
            (np.abs(points[:, 0] - center_x) <= self.map_radius_xy)
            & (np.abs(points[:, 1] - center_y) <= self.map_radius_xy)
            & (points[:, 2] >= self.map_min_z)
            & (points[:, 2] <= self.map_max_z)
        )
        points = points[keep]
        if points.size == 0:
            return

        voxel_keys = np.floor(points / self.map_voxel_size).astype(np.int32)
        unique_keys = np.unique(voxel_keys, axis=0)
        seen_at = time.monotonic()
        for key in unique_keys:
            self.map_voxels[(int(key[0]), int(key[1]), int(key[2]))] = seen_at
        self.latest_cloud_stamp = copy.deepcopy(msg.header.stamp)

    def publish_rolling_map(self) -> None:
        if self.latest_fused_position is None or not self.map_voxels:
            return
        now = time.monotonic()
        center_x, center_y, _ = self.latest_fused_position
        retained: Dict[Tuple[int, int, int], float] = {}
        for key, seen_at in self.map_voxels.items():
            x = (key[0] + 0.5) * self.map_voxel_size
            y = (key[1] + 0.5) * self.map_voxel_size
            z = (key[2] + 0.5) * self.map_voxel_size
            if abs(x - center_x) > self.map_radius_xy or abs(y - center_y) > self.map_radius_xy:
                continue
            if z < self.map_min_z or z > self.map_max_z:
                continue
            if self.map_voxel_ttl_sec > 0.0 and now - seen_at > self.map_voxel_ttl_sec:
                continue
            retained[key] = seen_at

        if len(retained) > self.map_max_voxels:
            newest = sorted(retained.items(), key=lambda item: item[1], reverse=True)
            retained = dict(newest[: self.map_max_voxels])
            self.warn(
                f"Rolling map reached {self.map_max_voxels} voxels; dropping oldest voxels."
            )
        self.map_voxels = retained
        if not retained:
            return

        points = np.asarray(
            [
                (
                    (key[0] + 0.5) * self.map_voxel_size,
                    (key[1] + 0.5) * self.map_voxel_size,
                    (key[2] + 0.5) * self.map_voxel_size,
                )
                for key in retained
            ],
            dtype=np.float32,
        )
        header = copy.deepcopy(PointCloud2().header)
        header.stamp = (
            copy.deepcopy(self.latest_cloud_stamp)
            if self.latest_cloud_stamp is not None
            else self.get_clock().now().to_msg()
        )
        header.frame_id = self.map_frame_id
        self.map_pub.publish(point_cloud2.create_cloud_xyz32(header, points))
        if len(points) != self.last_map_point_count:
            self.last_map_point_count = len(points)
            self.get_logger().info(
                f"Corrected rolling map published with {len(points)} occupied voxels."
            )

    def health_timer(self) -> None:
        odom_fresh = bool(
            self.latest_odom_monotonic is not None
            and time.monotonic() - self.latest_odom_monotonic <= self.max_odom_age_sec
        )
        healthy = bool(
            self.source_healthy
            and self.px4_is_fresh()
            and odom_fresh
            and self.latest_z_correction is not None
        )
        if healthy != self.last_health:
            self.last_health = healthy
            self.get_logger().info(
                "Planner altitude fusion healthy."
                if healthy
                else "Planner altitude fusion not ready or unhealthy."
            )
        self.health_pub.publish(Bool(data=healthy))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlannerAltitudeFusion()
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
