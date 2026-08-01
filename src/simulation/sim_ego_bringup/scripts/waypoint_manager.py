#!/usr/bin/env python3

import os
from pathlib import Path
from typing import List

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray


class WaypointManager(Node):
    """Record RViz 2D Goal Pose clicks and expose them as a visible/savable path."""

    def __init__(self) -> None:
        super().__init__("waypoint_manager")
        self.input_topic = str(
            self.declare_parameter("input_topic", "/move_base_simple/goal").value
        )
        self.marker_topic = str(
            self.declare_parameter("marker_topic", "/sim/waypoints").value
        )
        self.path_topic = str(
            self.declare_parameter("path_topic", "/sim/waypoint_path").value
        )
        self.waypoint_file = Path(
            str(
                self.declare_parameter(
                    "waypoint_file",
                    os.path.join(
                        os.environ.get("SIM_EGO_ROOT", "/home/wu/sim-ego"),
                        "scenarios",
                        "interactive_waypoints.yaml",
                    ),
                ).value
            )
        )
        self.append_mode = bool(self.declare_parameter("append_mode", True).value)
        self.default_altitude = float(self.declare_parameter("default_altitude", 1.5).value)
        self.auto_save = bool(self.declare_parameter("auto_save", True).value)
        self.waypoints: List[PoseStamped] = []
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.path_pub = self.create_publisher(PathMsg, self.path_topic, 10)
        self.create_subscription(PoseStamped, self.input_topic, self.goal_callback, 20)
        self.create_service(Trigger, "~/clear", self.clear_callback)
        self.create_service(Trigger, "~/save", self.save_callback)
        self.create_service(Trigger, "~/load", self.load_callback)
        self.get_logger().info(
            f"RViz waypoints: click '{self.input_topic}', save service '~/{self.get_name()}/save' -> {self.waypoint_file}"
        )
        self.load_waypoints()

    def goal_callback(self, msg: PoseStamped) -> None:
        if not self.append_mode:
            self.waypoints.clear()
        waypoint = PoseStamped()
        waypoint.header = msg.header
        waypoint.header.frame_id = msg.header.frame_id or "world"
        waypoint.pose = msg.pose
        # RViz's 2D Goal Pose is on the XY plane. Keep explicitly supplied 3D
        # goals, but turn a 2D click into a usable flight-level waypoint.
        if waypoint.pose.position.z <= 0.05:
            waypoint.pose.position.z = self.default_altitude
        self.waypoints.append(waypoint)
        self.publish_visualization()
        if self.auto_save:
            try:
                self.save_waypoints()
            except (OSError, yaml.YAMLError) as exc:
                self.get_logger().error(f"Could not save waypoints: {exc}")
        self.get_logger().info(
            f"Added waypoint {len(self.waypoints)}: "
            f"({waypoint.pose.position.x:.2f}, {waypoint.pose.position.y:.2f}, {waypoint.pose.position.z:.2f})"
        )

    def publish_visualization(self) -> None:
        markers = MarkerArray()
        path = PathMsg()
        path.header.frame_id = "world"
        for index, waypoint in enumerate(self.waypoints):
            pose = PoseStamped()
            pose.header = waypoint.header
            pose.header.frame_id = waypoint.header.frame_id or "world"
            pose.pose = waypoint.pose
            path.poses.append(pose)

            marker = Marker()
            marker.header = pose.header
            marker.ns = "sim_ego_waypoints"
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = pose.pose
            marker.scale.x = 0.35
            marker.scale.y = 0.35
            marker.scale.z = 0.35
            marker.color.a = 1.0
            marker.color.r = 0.1
            marker.color.g = 0.85
            marker.color.b = 0.95
            markers.markers.append(marker)

            label = Marker()
            label.header = pose.header
            label.ns = "sim_ego_waypoint_labels"
            label.id = 10000 + index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = pose.pose.position.x
            label.pose.position.y = pose.pose.position.y
            label.pose.position.z = pose.pose.position.z + 0.35
            label.pose.orientation = pose.pose.orientation
            label.scale.z = 0.25
            label.color.a = 1.0
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.text = str(index + 1)
            markers.markers.append(label)

        self.path_pub.publish(path)
        self.marker_pub.publish(markers)

    def clear_callback(self, request, response):
        del request
        self.waypoints.clear()
        self.publish_visualization()
        if self.auto_save:
            try:
                self.save_waypoints()
            except (OSError, yaml.YAMLError) as exc:
                response.success = False
                response.message = str(exc)
                return response
        response.success = True
        response.message = "waypoints cleared"
        return response

    def save_callback(self, request, response):
        del request
        try:
            self.save_waypoints()
            response.success = True
            response.message = f"saved {len(self.waypoints)} waypoints to {self.waypoint_file}"
        except (OSError, yaml.YAMLError) as exc:
            response.success = False
            response.message = str(exc)
        return response

    def load_callback(self, request, response):
        del request
        try:
            count = self.load_waypoints()
            response.success = True
            response.message = f"loaded {count} waypoints from {self.waypoint_file}"
        except (OSError, yaml.YAMLError, ValueError) as exc:
            response.success = False
            response.message = str(exc)
        return response

    def save_waypoints(self) -> None:
        self.waypoint_file.parent.mkdir(parents=True, exist_ok=True)
        data = {
            "frame_id": "world",
            "coordinate_system": "ros_flu",
            "waypoints": [
                {
                    "x": float(point.pose.position.x),
                    "y": float(point.pose.position.y),
                    "z": float(point.pose.position.z),
                    "yaw": 0.0,
                }
                for point in self.waypoints
            ],
        }
        self.waypoint_file.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")

    def load_waypoints(self) -> int:
        if not self.waypoint_file.exists():
            return 0
        data = yaml.safe_load(self.waypoint_file.read_text(encoding="utf-8")) or {}
        loaded = []
        for item in data.get("waypoints", []):
            pose = PoseStamped()
            pose.header.frame_id = str(data.get("frame_id", "world"))
            pose.pose.position.x = float(item["x"])
            pose.pose.position.y = float(item["y"])
            pose.pose.position.z = float(item["z"])
            pose.pose.orientation.w = 1.0
            loaded.append(pose)
        self.waypoints = loaded
        self.publish_visualization()
        return len(loaded)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
