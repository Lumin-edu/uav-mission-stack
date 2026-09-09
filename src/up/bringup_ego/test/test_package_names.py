import re
import unittest
from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[4]

PACKAGE_NAMES = {
    "src/fastlio_bringup": "fastlio_bringup",
    "src/up/bringup_pointlio_hover": "bringup_pointlio_hover",
    "src/up/bringup_full_mission": "bringup_full_mission",
    "src/up/bringup_square_mission": "bringup_square_mission",
    "src/up/bringup_ego": "bringup_ego",
    "src/up/bringup_ego_mpc": "bringup_ego_mpc",
    "src/up/bringup_ego_multi_mission": "bringup_ego_multi_mission",
}


class PackageNameTest(unittest.TestCase):
    def test_ros_package_names_drop_hx_prefix_everywhere(self) -> None:
        for relative_path, package_name in PACKAGE_NAMES.items():
            package_root = WORKSPACE_ROOT / relative_path
            old_name = f"hx_{package_name}"

            package_xml = (package_root / "package.xml").read_text(encoding="utf-8")
            cmake = (package_root / "CMakeLists.txt").read_text(encoding="utf-8")

            self.assertRegex(package_xml, rf"<name>{re.escape(package_name)}</name>")
            self.assertRegex(cmake, rf"project\({re.escape(package_name)}\)")

            for path in package_root.rglob("*"):
                if path.is_file() and "__pycache__" not in path.parts:
                    text = path.read_text(encoding="utf-8", errors="ignore")
                    self.assertNotIn(old_name, text, str(path))


if __name__ == "__main__":
    unittest.main()
