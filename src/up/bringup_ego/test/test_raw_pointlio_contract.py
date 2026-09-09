import pathlib
import unittest


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
WORKSPACE_SRC = PACKAGE_ROOT.parents[1]


class RawPointlioContractTest(unittest.TestCase):
    def test_ego_launch_uses_center_odom_and_raw_cloud_without_altitude_fusion(self) -> None:
        launch = (PACKAGE_ROOT / "launch" / "ego_avoidance_hw.launch.py").read_text()

        self.assertIn('executable="rigid_odom_transform.py"', launch)
        self.assertIn('"base_link_to_base_translation"', launch)
        self.assertIn('"base_link_to_base_rotation_xyzw"', launch)
        self.assertIn('("odom_world", base_odom_topic)', launch)
        self.assertIn('("grid_map/odom", base_odom_topic)', launch)
        self.assertIn('("grid_map/cloud", raw_cloud_topic)', launch)
        self.assertIn('"odom_topic": base_odom_topic', launch)
        self.assertIn('DeclareLaunchArgument("base_odom_topic", default_value="/ego/odom_base")', launch)
        self.assertNotIn("planner_altitude_fusion", launch)
        self.assertNotIn("/ego/odom_fused", launch)
        self.assertNotIn("/ego/cloud_registered_fused", launch)
        self.assertNotIn("height_fusion", launch)
        self.assertNotIn("require_rangefinder", launch)

    def test_ego_scripts_do_not_gate_control_on_height_fusion(self) -> None:
        for name in ("ego_px4_bridge.py", "ego_hw_monitor.py"):
            source = (PACKAGE_ROOT / "scripts" / name).read_text()
            self.assertNotIn("localization_health", source)
            self.assertNotIn("height_fusion", source)
            self.assertNotIn("require_rangefinder", source)

    def test_all_ego_variants_keep_center_odom_adapter_and_remove_altitude_fusion(self) -> None:
        for package_name, launch_name in (
            ("bringup_ego_mpc", "ego_mpc_hw.launch.py"),
            ("bringup_ego_multi_mission", "ego_multi_mission_hw.launch.py"),
        ):
            package_root = WORKSPACE_SRC / "up" / package_name
            launch = (package_root / "launch" / launch_name).read_text()
            self.assertIn('rigid_odom_transform.py', launch)
            self.assertIn('base_link_to_base_translation', launch)
            self.assertIn('base_link_to_base_rotation_xyzw', launch)
            self.assertIn('("odom_world", base_odom', launch)
            self.assertIn('("grid_map/odom", base_odom', launch)
            self.assertIn('DeclareLaunchArgument("base_odom_topic", default_value="/ego/odom_base")', launch)
            self.assertNotIn("planner_altitude_fusion", launch)
            self.assertNotIn("/ego/odom_fused", launch)
            self.assertNotIn("/ego/cloud_registered_fused", launch)
            self.assertNotIn("height_fusion", launch)
            self.assertNotIn("require_rangefinder", launch)
            self.assertFalse(
                (package_root / "scripts" / "planner_altitude_fusion.py").exists()
            )


if __name__ == "__main__":
    unittest.main()
