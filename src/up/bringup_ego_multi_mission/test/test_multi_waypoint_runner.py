#!/usr/bin/env python3

import importlib.util
import pathlib
import tempfile
import unittest


SCRIPT = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "multi_waypoint_runner.py"
SPEC = importlib.util.spec_from_file_location("multi_waypoint_runner", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class MultiWaypointGeometryTest(unittest.TestCase):
    def test_task_frame_conversion(self):
        self.assertEqual(
            MODULE.waypoint_to_ros(
                {"x": 2.0, "y": 3.0, "z": 1.0, "yaw": 0.4},
                MODULE.TASK_COORDINATE_SYSTEM,
            ),
            (3.0, -2.0, 1.0, -0.4),
        )

    def test_ros_frame_is_unchanged(self):
        self.assertEqual(
            MODULE.waypoint_to_ros(
                {"x": 1.0, "y": -2.0, "z": 0.8}, MODULE.ROS_COORDINATE_SYSTEM
            ),
            (1.0, -2.0, 0.8, 0.0),
        )

    def test_route_is_validated_before_use(self):
        with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml") as stream:
            stream.write(
                "coordinate_system: task_right_forward_up\n"
                "waypoints:\n"
                "  - {x: 0.0, y: 1.0, z: 0.8}\n"
            )
            stream.flush()
            data, points, system = MODULE.load_route(stream.name)
        self.assertEqual(system, MODULE.TASK_COORDINATE_SYSTEM)
        self.assertEqual(len(points), 1)
        self.assertIn("waypoints", data)

    def test_non_positive_altitude_is_rejected(self):
        with self.assertRaises(ValueError):
            MODULE.waypoint_to_ros(
                {"x": 0.0, "y": 1.0, "z": 0.0}, MODULE.TASK_COORDINATE_SYSTEM
            )


if __name__ == "__main__":
    unittest.main()
