#!/usr/bin/env python3

"""Execute a hardware EGO waypoint YAML route one stable goal at a time."""

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
from rclpy.utilities import remove_ros_args
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
    """Convert task x-right/y-forward/z-up into Point-LIO ROS FLU axes."""
    x = float(item["x"])
    y = float(item["y"])
    z = float(item["z"])
    yaw = float(item.get("yaw", 0.0))
    if not all(math.isfinite(value) for value in (x, y, z, yaw)):
        raise ValueError("waypoint coordinates and yaw must be finite")
    if z <= 0.05:
        raise ValueError("waypoint z must be greater than 0.05 m")
    if coordinate_system == TASK_COORDINATE_SYSTEM:
        # task x-right -> ROS -y-left; task y-forward -> ROS +x-forward.
        return y, -x, z, -yaw
    return x, y, z, yaw


class WaypointRunner(Node):
    def __init__(self, goal_topic: str, odom_topic: str, takeoff_ready_topic: str) -> None:
        super().__init__("multi_waypoint_runner")
        self.publisher = self.create_publisher(PoseStamped, goal_topic, 10)
        self.latest_odom: Odometry | None = None
        self.latest_odom_received_at: float | None = None
        self.takeoff_ready = False
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 20)
        self.create_subscription(Bool, takeoff_ready_topic, self.takeoff_ready_callback, 10)

    def odom_callback(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        values = (p.x, p.y, p.z, v.x, v.y, v.z)
        if all(math.isfinite(float(value)) for value in values):
            self.latest_odom = msg
            self.latest_odom_received_at = time.monotonic()

    def takeoff_ready_callback(self, msg: Bool) -> None:
        if msg.data and not self.takeoff_ready:
            self.takeoff_ready = True
            self.get_logger().info("PX4 takeoff is stable; route execution is enabled.")


def wait_for_route_readiness(node: WaypointRunner, timeout: float) -> None:
    deadline = time.monotonic() + timeout
    last_status_at = -math.inf
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.10)
        if (
            node.latest_odom is not None
            and node.takeoff_ready
            and node.publisher.get_subscription_count() > 0
        ):
            return
        now = time.monotonic()
        if now - last_status_at >= 2.0:
            missing = []
            if node.latest_odom is None:
                missing.append("fused odometry")
            if not node.takeoff_ready:
                missing.append("stable takeoff")
            if node.publisher.get_subscription_count() == 0:
                missing.append("EGO goal subscriber")
            node.get_logger().info("Waiting for " + ", ".join(missing) + ".")
            last_status_at = now
        if now >= deadline:
            raise TimeoutError("route readiness timed out")


def load_route(path: str) -> tuple[dict[str, Any], list[dict[str, Any]], str]:
    try:
        with open(path, "r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(str(exc)) from exc
    if not isinstance(data, dict):
        raise ValueError("YAML root must be a mapping")
    waypoints = data.get("waypoints", [])
    if not isinstance(waypoints, list) or not waypoints:
        raise ValueError("YAML file must contain a non-empty waypoints list")
    coordinate_system = normalize_coordinate_system(data.get("coordinate_system"))
    if not all(isinstance(item, dict) for item in waypoints):
        raise ValueError("every waypoint must be a YAML mapping")
    # Validate all points before publishing the first one.
    for item in waypoints:
        waypoint_to_ros(item, coordinate_system)
    return data, waypoints, coordinate_system


def run_route(node: WaypointRunner, args: argparse.Namespace) -> bool:
    data, waypoints, coordinate_system = load_route(args.path)
    ros_waypoints = [waypoint_to_ros(item, coordinate_system) for item in waypoints]
    wait_for_route_readiness(node, args.ready_timeout)
    node.get_logger().info(
        f"Loaded {len(waypoints)} waypoints in '{coordinate_system}' coordinates."
    )

    for index, (item, ros_point) in enumerate(zip(waypoints, ros_waypoints), start=1):
        ros_x, ros_y, ros_z, ros_yaw = ros_point
        message = PoseStamped()
        message.header.frame_id = str(data.get("frame_id", "odom"))
        message.pose.position.x = ros_x
        message.pose.position.y = ros_y
        message.pose.position.z = ros_z
        message.pose.orientation.z = math.sin(ros_yaw / 2.0)
        message.pose.orientation.w = math.cos(ros_yaw / 2.0)
        message.header.stamp = node.get_clock().now().to_msg()
        node.publisher.publish(message)

        waypoint_timeout = float(item.get("timeout_sec", args.timeout))
        settle_sec = float(item.get("settle_sec", args.settle))
        if not math.isfinite(waypoint_timeout) or waypoint_timeout <= 0.0:
            raise ValueError(f"waypoint {index} timeout_sec must be positive")
        if not math.isfinite(settle_sec) or settle_sec < 0.0:
            raise ValueError(f"waypoint {index} settle_sec must be non-negative")

        deadline = time.monotonic() + waypoint_timeout
        reached_since: float | None = None
        last_progress_at = -math.inf
        node.get_logger().info(
            f"Sent waypoint {index}/{len(waypoints)}: "
            f"task=({float(item['x']):+.2f}, {float(item['y']):+.2f}, {float(item['z']):+.2f}), "
            f"ROS_FLU=({ros_x:+.2f}, {ros_y:+.2f}, {ros_z:+.2f})."
        )

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.10)
            now = time.monotonic()
            odom = node.latest_odom
            fresh = bool(
                odom is not None
                and node.latest_odom_received_at is not None
                and now - node.latest_odom_received_at <= args.odom_timeout
            )
            if fresh and odom is not None:
                p = odom.pose.pose.position
                v = odom.twist.twist.linear
                xy_error = math.hypot(p.x - ros_x, p.y - ros_y)
                z_error = abs(p.z - ros_z)
                speed_xy = math.hypot(v.x, v.y)
                speed_z = abs(v.z)
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
                            f"Reached waypoint {index}/{len(waypoints)} and remained stable "
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
                        f"Waypoint {index}: fused odometry is stale; route progression paused."
                    )
                    last_progress_at = now
            if now >= deadline:
                raise TimeoutError(
                    f"timed out before reaching waypoint {index}/{len(waypoints)}"
                )
    node.get_logger().info(
        "Route complete. No new goal will be sent; PX4 will hold the final position."
    )
    return True


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", help="YAML file containing a waypoints list")
    parser.add_argument("--topic", default="/move_base_simple/goal")
    parser.add_argument("--odom-topic", default="/ego/odom_base")
    parser.add_argument("--takeoff-ready-topic", default="/ego/takeoff_ready")
    parser.add_argument("--ready-timeout", type=float, default=120.0)
    parser.add_argument("--odom-timeout", type=float, default=0.50)
    parser.add_argument("--reach-xy", type=float, default=0.30)
    parser.add_argument("--reach-z", type=float, default=0.20)
    parser.add_argument("--max-speed-xy", type=float, default=0.20)
    parser.add_argument("--max-speed-z", type=float, default=0.15)
    parser.add_argument("--settle", type=float, default=1.0)
    parser.add_argument("--timeout", type=float, default=90.0)
    # Kept as an explicit flag in the launch contract: this runner always
    # waits for the verified bridge's stable-takeoff signal.
    parser.add_argument("--wait-takeoff-ready", action="store_true", default=False)
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])
    del args.wait_takeoff_ready
    if min(
        args.ready_timeout,
        args.odom_timeout,
        args.reach_xy,
        args.reach_z,
        args.max_speed_xy,
        args.max_speed_z,
        args.timeout,
    ) <= 0.0 or args.settle < 0.0:
        parser.error("timeouts, reaches, and speed limits must be positive; settle must be non-negative")

    rclpy.init()
    node = WaypointRunner(args.topic, args.odom_topic, args.takeoff_ready_topic)
    failed = False
    try:
        run_route(node, args)
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
