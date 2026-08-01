#!/usr/bin/env python3

import argparse
import math
import sys
import time
from typing import Any

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool


TASK_COORDINATE_SYSTEM = "task_right_forward_up"
ROS_COORDINATE_SYSTEM = "ros_flu"


def normalize_coordinate_system(value: Any) -> str:
    normalized = str(value or ROS_COORDINATE_SYSTEM).strip().lower()
    aliases = {
        "task": TASK_COORDINATE_SYSTEM,
        "task_rfu": TASK_COORDINATE_SYSTEM,
        "right_forward_up": TASK_COORDINATE_SYSTEM,
        TASK_COORDINATE_SYSTEM: TASK_COORDINATE_SYSTEM,
        "ros": ROS_COORDINATE_SYSTEM,
        "world": ROS_COORDINATE_SYSTEM,
        ROS_COORDINATE_SYSTEM: ROS_COORDINATE_SYSTEM,
    }
    if normalized not in aliases:
        raise ValueError(
            "coordinate_system must be 'task_right_forward_up' or 'ros_flu'"
        )
    return aliases[normalized]


def waypoint_to_ros(
    item: dict[str, Any], coordinate_system: str
) -> tuple[float, float, float, float]:
    x = float(item["x"])
    y = float(item["y"])
    z = float(item["z"])
    yaw = float(item.get("yaw", 0.0))
    if not all(math.isfinite(value) for value in (x, y, z, yaw)):
        raise ValueError("waypoint coordinates and yaw must be finite")
    if z <= 0.05:
        raise ValueError("waypoint z must be greater than 0.05 m")
    if coordinate_system == TASK_COORDINATE_SYSTEM:
        # Aircraft task frame -> Point-LIO/EGO ROS FLU frame:
        # x right -> -y left, y forward -> +x forward, z up -> +z up.
        return y, -x, z, -yaw
    return x, y, z, yaw


class WaypointRunner(Node):
    def __init__(
        self,
        goal_topic: str,
        odom_topic: str,
        takeoff_ready_topic: str,
        occupancy_topic: str,
    ) -> None:
        super().__init__("send_waypoints")
        self.publisher = self.create_publisher(PoseStamped, goal_topic, 10)
        self.latest_odom = None
        self.latest_odom_received_at = None
        self.takeoff_ready = False
        self.occupancy_ready = False
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 20)
        ready_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            Bool, takeoff_ready_topic, self.takeoff_ready_callback, ready_qos
        )
        self.create_subscription(
            PointCloud2, occupancy_topic, self.occupancy_callback, ready_qos
        )

    def odom_callback(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        velocity = msg.twist.twist.linear
        values = (
            position.x,
            position.y,
            position.z,
            velocity.x,
            velocity.y,
            velocity.z,
        )
        if not all(math.isfinite(float(value)) for value in values):
            return
        self.latest_odom = msg
        self.latest_odom_received_at = time.monotonic()

    def takeoff_ready_callback(self, msg: Bool) -> None:
        if msg.data and not self.takeoff_ready:
            self.takeoff_ready = True
            self.get_logger().info("PX4 takeoff is stable; route execution is enabled.")

    def occupancy_callback(self, msg: PointCloud2) -> None:
        del msg
        if not self.occupancy_ready:
            self.occupancy_ready = True
            self.get_logger().info("EGO static occupancy is ready; route goals may be validated.")


def wait_for_route_readiness(
    node: WaypointRunner,
    wait_takeoff_ready: bool,
    wait_occupancy: bool,
    timeout: float,
) -> None:
    deadline = time.monotonic() + timeout
    last_status_at = -math.inf
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.10)
        odom_ready = node.latest_odom is not None
        planner_ready = node.publisher.get_subscription_count() > 0
        takeoff_ready = node.takeoff_ready or not wait_takeoff_ready
        occupancy_ready = node.occupancy_ready or not wait_occupancy
        if odom_ready and planner_ready and takeoff_ready and occupancy_ready:
            return
        now = time.monotonic()
        if now - last_status_at >= 2.0:
            missing = []
            if not odom_ready:
                missing.append("fused odometry")
            if not planner_ready:
                missing.append("EGO goal subscriber")
            if not takeoff_ready:
                missing.append("stable takeoff")
            if not occupancy_ready:
                missing.append("EGO static occupancy")
            node.get_logger().info("Waiting for " + ", ".join(missing) + ".")
            last_status_at = now
        if now >= deadline:
            raise TimeoutError("route readiness timed out")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Execute a sim-ego waypoint YAML file one stable goal at a time."
    )
    parser.add_argument("path", help="YAML file containing a waypoints list")
    parser.add_argument("--topic", default="/move_base_simple/goal")
    parser.add_argument("--odom-topic", default="/sim/odom")
    parser.add_argument("--takeoff-ready-topic", default="/ego/takeoff_ready")
    parser.add_argument(
        "--occupancy-topic", default="/ego/grid_map/static_occupancy_inflate"
    )
    parser.add_argument(
        "--wait-takeoff-ready",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="wait for the simulation PX4 bridge to report a stable takeoff",
    )
    parser.add_argument(
        "--wait-occupancy",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="wait for EGO's accumulated-map occupancy before sending the route",
    )
    parser.add_argument("--ready-timeout", type=float, default=120.0)
    parser.add_argument("--odom-timeout", type=float, default=0.50)
    parser.add_argument("--reach-xy", type=float, default=0.30)
    parser.add_argument("--reach-z", type=float, default=0.20)
    parser.add_argument("--max-speed-xy", type=float, default=0.20)
    parser.add_argument("--max-speed-z", type=float, default=0.15)
    parser.add_argument("--settle", type=float, default=1.0)
    parser.add_argument("--timeout", type=float, default=90.0)
    # launch_ros appends ROS-specific remap arguments. Remove only those so
    # argparse remains strict for route options and command-line typos.
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])
    if min(
        args.reach_xy,
        args.reach_z,
        args.max_speed_xy,
        args.max_speed_z,
        args.settle,
    ) < 0.0:
        parser.error("reach, speed, and settle limits must be non-negative")
    if args.ready_timeout <= 0.0 or args.odom_timeout <= 0.0 or args.timeout <= 0.0:
        parser.error("ready-timeout, odom-timeout, and timeout must be positive")

    try:
        with open(args.path, "r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
    except (OSError, yaml.YAMLError) as exc:
        parser.error(str(exc))
    if not isinstance(data, dict):
        parser.error("YAML root must be a mapping")
    waypoints = data.get("waypoints", [])
    if not isinstance(waypoints, list) or not waypoints:
        parser.error("YAML file must contain a non-empty waypoints list")
    try:
        coordinate_system = normalize_coordinate_system(
            data.get("coordinate_system", ROS_COORDINATE_SYSTEM)
        )
        ros_waypoints = []
        for index, item in enumerate(waypoints, start=1):
            if not isinstance(item, dict):
                raise ValueError(f"waypoint {index} must be a YAML mapping")
            ros_waypoints.append(waypoint_to_ros(item, coordinate_system))
    except (KeyError, TypeError, ValueError) as exc:
        parser.error(str(exc))

    rclpy.init()
    node = WaypointRunner(
        args.topic,
        args.odom_topic,
        args.takeoff_ready_topic,
        args.occupancy_topic,
    )
    failed = False
    try:
        node.get_logger().info(
            f"Loaded {len(waypoints)} waypoints in '{coordinate_system}' coordinates."
        )
        wait_for_route_readiness(
            node,
            args.wait_takeoff_ready,
            args.wait_occupancy,
            args.ready_timeout,
        )
        node.get_logger().info("Route inputs are ready; starting waypoint 1.")

        for index, (item, ros_point) in enumerate(
            zip(waypoints, ros_waypoints), start=1
        ):
            ros_x, ros_y, ros_z, ros_yaw = ros_point
            message = PoseStamped()
            message.header.frame_id = str(data.get("frame_id", "world"))
            message.pose.position.x = ros_x
            message.pose.position.y = ros_y
            message.pose.position.z = ros_z
            message.pose.orientation.z = math.sin(ros_yaw / 2.0)
            message.pose.orientation.w = math.cos(ros_yaw / 2.0)
            message.header.stamp = node.get_clock().now().to_msg()
            node.publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.10)

            waypoint_timeout = float(item.get("timeout_sec", args.timeout))
            settle_sec = float(item.get("settle_sec", args.settle))
            if (
                not math.isfinite(waypoint_timeout)
                or waypoint_timeout <= 0.0
                or not math.isfinite(settle_sec)
                or settle_sec < 0.0
            ):
                raise ValueError(
                    f"waypoint {index} settle_sec must be non-negative and timeout_sec positive"
                )
            deadline = time.monotonic() + waypoint_timeout
            reached_since = None
            last_progress_at = -math.inf
            task_x = float(item["x"])
            task_y = float(item["y"])
            task_z = float(item["z"])
            node.get_logger().info(
                f"Sent waypoint {index}/{len(waypoints)}: "
                f"input=({task_x:+.2f}, {task_y:+.2f}, {task_z:+.2f}), "
                f"ROS_FLU=({ros_x:+.2f}, {ros_y:+.2f}, {ros_z:+.2f})."
            )

            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.10)
                odom = node.latest_odom
                now = time.monotonic()
                odom_is_fresh = bool(
                    node.latest_odom_received_at is not None
                    and now - node.latest_odom_received_at <= args.odom_timeout
                )
                if odom is not None and odom_is_fresh:
                    position = odom.pose.pose.position
                    velocity = odom.twist.twist.linear
                    xy_error = math.hypot(position.x - ros_x, position.y - ros_y)
                    z_error = abs(position.z - ros_z)
                    speed_xy = math.hypot(velocity.x, velocity.y)
                    speed_z = abs(velocity.z)
                    stable = (
                        xy_error <= args.reach_xy
                        and z_error <= args.reach_z
                        and speed_xy <= args.max_speed_xy
                        and speed_z <= args.max_speed_z
                    )
                    if stable:
                        reached_since = reached_since or now
                        if now - reached_since >= settle_sec:
                            node.get_logger().info(
                                f"Reached waypoint {index}/{len(waypoints)} and stable "
                                f"for {settle_sec:.1f} s."
                            )
                            break
                    else:
                        reached_since = None
                    if now - last_progress_at >= 2.0:
                        node.get_logger().info(
                            f"Waypoint {index}: xy_error={xy_error:.2f} m, "
                            f"z_error={z_error:.2f} m, speed_xy={speed_xy:.2f} m/s, "
                            f"speed_z={speed_z:.2f} m/s."
                        )
                        last_progress_at = now
                else:
                    reached_since = None
                    if now - last_progress_at >= 2.0:
                        node.get_logger().warn(
                            f"Waypoint {index}: fused odometry is stale; holding route progression."
                        )
                        last_progress_at = now
                if now >= deadline:
                    raise TimeoutError(
                        f"timed out before reaching waypoint {index}/{len(waypoints)}"
                    )
        node.get_logger().info(
            "Route complete. No new goal will be sent; PX4 will hold the final position."
        )
    except KeyboardInterrupt:
        node.get_logger().warn("Waypoint route interrupted.")
    except (TimeoutError, ValueError) as exc:
        node.get_logger().error(str(exc))
        failed = True
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if failed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
