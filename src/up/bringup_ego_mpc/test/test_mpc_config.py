import unittest
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = PACKAGE_ROOT / "config" / "mpc_params.yaml"
HIGH_SPEED_CONFIG_PATH = PACKAGE_ROOT / "config" / "mpc_params_5ms.yaml"
LAUNCH_PATH = PACKAGE_ROOT / "launch" / "ego_mpc_hw.launch.py"


class MpcConfigTest(unittest.TestCase):
    def test_shared_motion_limits_are_defined_for_planner_and_mpc(self) -> None:
        config = yaml.safe_load(CONFIG_PATH.read_text())

        mpc = config["ego_mpc_controller"]["ros__parameters"]
        planner = config["ego_planner"]["ros__parameters"]

        self.assertEqual(mpc["max_velocity"], 0.5)
        self.assertEqual(planner["manager/max_vel"], mpc["max_velocity"])
        self.assertEqual(planner["optimization/max_vel"], mpc["max_velocity"])
        self.assertEqual(planner["bspline/limit_vel"], mpc["max_velocity"])
        self.assertLessEqual(planner["manager/max_acc"], mpc["max_acceleration"])
        self.assertEqual(planner["manager/max_acc"], planner["optimization/max_acc"])
        self.assertEqual(planner["manager/max_acc"], planner["bspline/limit_acc"])

        high_speed = yaml.safe_load(HIGH_SPEED_CONFIG_PATH.read_text())
        high_speed_mpc = high_speed["ego_mpc_controller"]["ros__parameters"]
        high_speed_planner = high_speed["ego_planner"]["ros__parameters"]
        self.assertEqual(high_speed_mpc["max_velocity"], 5.0)
        self.assertEqual(high_speed_planner["manager/max_vel"], high_speed_mpc["max_velocity"])
        self.assertLessEqual(high_speed_planner["manager/max_acc"], high_speed_mpc["max_acceleration"])

    def test_launch_accepts_shared_mpc_config(self) -> None:
        launch_text = LAUNCH_PATH.read_text()

        self.assertIn('DeclareLaunchArgument("mpc_config"', launch_text)
        self.assertIn('LaunchConfiguration("mpc_config")', launch_text)
        for legacy_name in (
            "max_velocity",
            "max_acceleration",
            "mpc_rate_hz",
            "mpc_dt",
            "mpc_horizon",
            "mpc_max_acceleration",
            "mpc_max_horizontal_acceleration",
            "mpc_max_vertical_acceleration",
            "mpc_max_jerk",
            "mpc_solver_time_limit_ms",
        ):
            self.assertNotIn(f'LaunchConfiguration("{legacy_name}")', launch_text)


if __name__ == "__main__":
    unittest.main()
