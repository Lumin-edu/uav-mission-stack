import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class PackageContractTest(unittest.TestCase):
    def test_package_is_independent_of_px4_dds_and_old_up_tree(self) -> None:
        package_xml = (ROOT / "package.xml").read_text(encoding="utf-8")
        self.assertIn("<name>bringup_ego_mavros</name>", package_xml)
        self.assertIn("mavros_msgs", package_xml)
        self.assertNotIn("px4_msgs", package_xml)
        for path in ROOT.rglob("*"):
            if (
                path.is_file()
                and "__pycache__" not in path.parts
                and path.parent.name != "test"
            ):
                text = path.read_text(encoding="utf-8", errors="ignore")
                self.assertNotIn("src/up/", text, str(path))
                self.assertNotIn("bringup_pointlio_hover", text, str(path))

    def test_package_installs_fixed_point_entrypoints_and_launch(self) -> None:
        cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("scripts/fixed_point_mavros.py", cmake)
        self.assertIn("scripts/pointlio_to_mavros_vision_pose.py", cmake)
        self.assertIn("scripts/vision_pose_adapter.py", cmake)
        self.assertIn("install(DIRECTORY launch config", cmake)
        self.assertTrue((ROOT / "launch/fixed_point_mavros.launch.py").is_file())
        self.assertTrue((ROOT / "README.md").is_file())

    def test_package_declares_pointlio_external_localization_dependencies(self) -> None:
        root = ET.parse(ROOT / "package.xml").getroot()
        dependencies = {element.text for element in root.findall("exec_depend")}
        self.assertTrue(
            {"geometry_msgs", "nav_msgs", "mavros_msgs", "point_lio"}.issubset(
                dependencies
            )
        )

    def test_controller_does_not_manually_convert_mavros_ros_enu_setpoint(self) -> None:
        controller = (ROOT / "scripts/fixed_point_mavros.py").read_text(
            encoding="utf-8"
        )
        self.assertNotIn("enu_to_ned", controller)
        self.assertIn(
            "make_position_target(self.target_enu, self.target_yaw)", controller
        )


if __name__ == "__main__":
    unittest.main()
