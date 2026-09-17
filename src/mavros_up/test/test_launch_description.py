import importlib.util
import unittest
from pathlib import Path

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node

ROOT = Path(__file__).resolve().parents[1]


class LaunchDescriptionTest(unittest.TestCase):
    def test_default_launch_starts_pointlio_bridge_and_fixed_point_controller(
        self,
    ) -> None:
        launch_path = ROOT / "launch" / "fixed_point_mavros.launch.py"
        spec = importlib.util.spec_from_file_location(
            "fixed_point_mavros_launch", launch_path
        )
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        spec.loader.exec_module(module)

        entities = module.generate_launch_description().entities
        includes = [
            item for item in entities if isinstance(item, IncludeLaunchDescription)
        ]
        nodes = [item for item in entities if isinstance(item, Node)]
        arguments = {
            item.name: item
            for item in entities
            if isinstance(item, DeclareLaunchArgument)
        }

        self.assertEqual(len(includes), 1)
        self.assertEqual(len(nodes), 2)
        self.assertIn("pointlio_config", arguments)
        self.assertIn("pointlio_odom_topic", arguments)
        self.assertIn("vision_pose_topic", arguments)


if __name__ == "__main__":
    unittest.main()
