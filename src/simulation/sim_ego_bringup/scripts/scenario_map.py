#!/usr/bin/env python3

"""Publish a repeatable obstacle point cloud from a small YAML scenario file."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Iterable

import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class ScenarioError(ValueError):
    pass


class ScenarioMap(Node):
    def __init__(self) -> None:
        super().__init__("scenario_map")
        self.scenario_file = Path(
            str(self.declare_parameter("scenario_file", "").value)
        )
        self.map_topic = str(
            self.declare_parameter("map_topic", "/map_generator/global_cloud").value
        )
        self.publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 2.0).value)
        if self.publish_rate_hz <= 0.0:
            raise ScenarioError("publish_rate_hz must be greater than zero")

        self.frame_id, self.points = self.load_scenario(self.scenario_file)
        self.publisher = self.create_publisher(PointCloud2, self.map_topic, 10)
        self.create_timer(1.0 / self.publish_rate_hz, self.publish)
        self.get_logger().info(
            f"Loaded {len(self.points)} obstacle points from {self.scenario_file} -> {self.map_topic}"
        )

    @staticmethod
    def _numbers(value: Any, name: str, count: int) -> list[float]:
        if not isinstance(value, (list, tuple)) or len(value) != count:
            raise ScenarioError(f"{name} must contain exactly {count} numeric values")
        try:
            return [float(item) for item in value]
        except (TypeError, ValueError) as exc:
            raise ScenarioError(f"{name} must contain numeric values") from exc

    @staticmethod
    def _samples(start: float, stop: float, resolution: float) -> Iterable[float]:
        count = max(1, int(math.ceil((stop - start) / resolution)))
        for index in range(count + 1):
            yield min(stop, start + index * resolution)

    def load_scenario(self, path: Path) -> tuple[str, list[tuple[float, float, float]]]:
        if not path.is_file():
            raise ScenarioError(f"scenario file does not exist: {path}")
        try:
            data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        except yaml.YAMLError as exc:
            raise ScenarioError(f"invalid YAML in {path}: {exc}") from exc
        if not isinstance(data, dict):
            raise ScenarioError("scenario root must be a YAML mapping")

        frame_id = str(data.get("frame_id", "world"))
        try:
            resolution = float(data.get("resolution", 0.1))
        except (TypeError, ValueError) as exc:
            raise ScenarioError("resolution must be numeric") from exc
        if resolution <= 0.0:
            raise ScenarioError("resolution must be greater than zero")

        obstacles = data.get("obstacles", [])
        if not isinstance(obstacles, list):
            raise ScenarioError("obstacles must be a YAML list")
        points: list[tuple[float, float, float]] = []
        for index, obstacle in enumerate(obstacles, start=1):
            if not isinstance(obstacle, dict):
                raise ScenarioError(f"obstacle {index} must be a mapping")
            obstacle_type = str(obstacle.get("type", "")).lower()
            if obstacle_type == "box":
                points.extend(self.box_points(obstacle, resolution, index))
            elif obstacle_type == "cylinder":
                points.extend(self.cylinder_points(obstacle, resolution, index))
            else:
                raise ScenarioError(
                    f"obstacle {index} has unsupported type {obstacle_type!r}; use box or cylinder"
                )
        return frame_id, points

    def box_points(
        self, obstacle: dict[str, Any], resolution: float, index: int
    ) -> Iterable[tuple[float, float, float]]:
        center = self._numbers(obstacle.get("center"), f"obstacle {index}.center", 3)
        size = self._numbers(obstacle.get("size"), f"obstacle {index}.size", 3)
        if any(value <= 0.0 for value in size):
            raise ScenarioError(f"obstacle {index}.size values must be greater than zero")
        x_range = (center[0] - size[0] / 2.0, center[0] + size[0] / 2.0)
        y_range = (center[1] - size[1] / 2.0, center[1] + size[1] / 2.0)
        z_range = (center[2] - size[2] / 2.0, center[2] + size[2] / 2.0)
        return [
            (x, y, z)
            for x in self._samples(*x_range, resolution)
            for y in self._samples(*y_range, resolution)
            for z in self._samples(*z_range, resolution)
        ]

    def cylinder_points(
        self, obstacle: dict[str, Any], resolution: float, index: int
    ) -> Iterable[tuple[float, float, float]]:
        center = self._numbers(obstacle.get("center"), f"obstacle {index}.center", 3)
        try:
            radius = float(obstacle.get("radius"))
            height = float(obstacle.get("height"))
        except (TypeError, ValueError) as exc:
            raise ScenarioError(f"obstacle {index} radius and height must be numeric") from exc
        if radius <= 0.0 or height <= 0.0:
            raise ScenarioError(f"obstacle {index} radius and height must be greater than zero")
        z_range = (center[2] - height / 2.0, center[2] + height / 2.0)
        points = []
        for x in self._samples(center[0] - radius, center[0] + radius, resolution):
            for y in self._samples(center[1] - radius, center[1] + radius, resolution):
                if math.hypot(x - center[0], y - center[1]) <= radius:
                    points.extend((x, y, z) for z in self._samples(*z_range, resolution))
        return points

    def publish(self) -> None:
        header = Header()
        header.frame_id = self.frame_id
        header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(point_cloud2.create_cloud_xyz32(header, self.points))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ScenarioMap()
        rclpy.spin(node)
    except (KeyboardInterrupt, ScenarioError) as exc:
        if isinstance(exc, ScenarioError):
            print(f"scenario_map: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
