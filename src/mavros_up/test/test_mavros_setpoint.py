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
    task_offset_to_local_enu,
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

    def test_task_target_maps_right_forward_up_at_zero_heading(self) -> None:
        self.assertEqual(
            resolve_target(
                (2.0, -1.0, 0.1),
                (1.0, 2.0, 3.0),
                orientation=(0.0, 0.0, 0.0, 1.0),
            ),
            (4.0, -2.0, 3.1),
        )

    def test_task_target_rotates_with_captured_heading(self) -> None:
        half_angle = math.pi / 4.0
        result = resolve_target(
            (2.0, -1.0, 0.1),
            (1.0, 2.0, 3.0),
            orientation=(0.0, 0.0, math.sin(half_angle), math.cos(half_angle)),
        )
        assert result is not None
        for actual, expected in zip(result, (3.0, 1.0, 3.1)):
            self.assertAlmostEqual(actual, expected)

    def test_task_offset_requires_a_valid_captured_orientation(self) -> None:
        self.assertIsNone(task_offset_to_local_enu((1.0, 2.0, 3.0), None))

    def test_task_frame_uses_heading_only_for_level_right_forward_up_axes(self) -> None:
        half_roll = math.pi / 4.0
        result = task_offset_to_local_enu(
            (0.0, 0.0, 1.0),
            (math.sin(half_roll), 0.0, 0.0, math.cos(half_roll)),
        )
        assert result is not None
        for actual, expected in zip(result, (0.0, 0.0, 1.0)):
            self.assertAlmostEqual(actual, expected)

    def test_relative_task_target_adds_offset_to_captured_reference(self) -> None:
        self.assertEqual(
            resolve_target(
                (2.0, -1.0, 0.1),
                (0.0, 0.0, 1.0),
                orientation=(0.0, 0.0, 0.0, 1.0),
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
