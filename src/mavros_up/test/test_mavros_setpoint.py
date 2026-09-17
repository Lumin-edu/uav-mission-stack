import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from mavros_setpoint_contract import (
    FRAME_LOCAL_NED,
    make_position_target,
    quaternion_to_yaw,
    resolve_target,
    resolve_yaw,
)


class MavrosSetpointTest(unittest.TestCase):
    def test_fixed_point_keeps_ros_enu_coordinates_for_mavros(self) -> None:
        fields = make_position_target((1.0, 2.0, 1.5), 0.3)
        self.assertEqual(fields.coordinate_frame, FRAME_LOCAL_NED)
        self.assertEqual(fields.type_mask, 2552)
        self.assertEqual(fields.position, (1.0, 2.0, 1.5))
        self.assertAlmostEqual(fields.yaw, 0.3)
        self.assertTrue(all(math.isnan(v) for v in fields.velocity))
        self.assertTrue(all(math.isnan(v) for v in fields.acceleration))

    def test_fixed_point_yaw_is_explicit(self) -> None:
        fields = make_position_target((0.0, 0.0, 1.0), -0.8)
        self.assertAlmostEqual(fields.yaw, -0.8)
        self.assertEqual(fields.yaw_rate, 0.0)

    def test_absolute_target_is_available_without_reference_odom(self) -> None:
        self.assertEqual(
            resolve_target(None, (1.0, 2.0, 3.0), use_current_position_reference=False),
            (1.0, 2.0, 3.0),
        )

    def test_relative_enu_target_adds_offset_to_captured_reference(self) -> None:
        self.assertEqual(
            resolve_target(
                (2.0, -1.0, 0.1),
                (0.0, 0.0, 1.0),
                use_current_position_reference=True,
            ),
            (2.0, -1.0, 1.1),
        )

    def test_hold_current_yaw_uses_the_captured_takeoff_heading(self) -> None:
        self.assertAlmostEqual(
            quaternion_to_yaw(
                (0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0))
            ),
            math.pi / 2.0,
        )
        self.assertAlmostEqual(
            resolve_yaw(
                (0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0)),
                configured_yaw=0.7,
                hold_current_yaw=True,
            ),
            math.pi / 2.0,
        )

    def test_hold_current_yaw_waits_for_a_valid_pose(self) -> None:
        self.assertIsNone(resolve_yaw(None, configured_yaw=0.7, hold_current_yaw=True))
        self.assertAlmostEqual(
            resolve_yaw(None, configured_yaw=0.7, hold_current_yaw=False), 0.7
        )


if __name__ == "__main__":
    unittest.main()
