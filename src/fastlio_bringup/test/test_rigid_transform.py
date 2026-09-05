#!/usr/bin/env python3

import math
import sys
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / "scripts"))

from rigid_transform import compose_pose, rotate_vector


BODY_TO_BASE = (-0.011, -0.02329, -0.05588)
IDENTITY = (0.0, 0.0, 0.0, 1.0)
HALF_90 = math.pi / 4.0
YAW_90 = (0.0, 0.0, math.sin(HALF_90), math.cos(HALF_90))
ROLL_90 = (math.sin(HALF_90), 0.0, 0.0, math.cos(HALF_90))
PITCH_90 = (0.0, math.sin(HALF_90), 0.0, math.cos(HALF_90))


class RigidTransformTest(unittest.TestCase):
    def assert_vector_close(self, actual, expected) -> None:
        self.assertEqual(len(actual), len(expected))
        for actual_value, expected_value in zip(actual, expected):
            self.assertAlmostEqual(actual_value, expected_value, delta=1e-9)

    def test_level_pose_applies_body_to_base_offset_with_correct_sign(self) -> None:
        position, orientation = compose_pose(
            (0.0, 0.0, 0.0), IDENTITY, BODY_TO_BASE, IDENTITY
        )

        self.assert_vector_close(position, BODY_TO_BASE)
        self.assert_vector_close(orientation, IDENTITY)

    def test_yaw_rotates_body_lever_arm_into_world(self) -> None:
        position, _ = compose_pose((1.0, 2.0, 3.0), YAW_90, BODY_TO_BASE, IDENTITY)

        self.assert_vector_close(position, (1.02329, 1.989, 2.94412))

    def test_roll_and_pitch_rotate_vertical_offset(self) -> None:
        self.assert_vector_close(
            rotate_vector(ROLL_90, (0.0, 0.0, -0.10)), (0.0, 0.10, 0.0)
        )
        self.assert_vector_close(
            rotate_vector(PITCH_90, (0.0, 0.0, -0.10)), (-0.10, 0.0, 0.0)
        )

    def test_static_rotation_is_composed_after_body_orientation(self) -> None:
        _, orientation = compose_pose(
            (0.0, 0.0, 0.0), YAW_90, (0.0, 0.0, 0.0), YAW_90
        )

        self.assert_vector_close(orientation, (0.0, 0.0, 1.0, 0.0))

    def test_zero_quaternion_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "norm is zero"):
            compose_pose(
                (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0), BODY_TO_BASE, IDENTITY
            )

    def test_bridge_defaults_to_fastlio_odometry_and_stays_package_local(
        self,
    ) -> None:
        source = PACKAGE_ROOT / "scripts" / "fastlio_to_px4_visual_odom.py"
        text = source.read_text(encoding="utf-8")

        self.assertIn('"/Odometry"', text)
        self.assertIn('"body_to_base_translation"', text)
        self.assertNotIn("bringup_" + "point" + "lio_hover", text)
        self.assertNotIn("point" + "lio_to_px4", text)

    def test_hover_controller_keeps_manual_arm_and_position_mode_defaults(self) -> None:
        source = PACKAGE_ROOT / "scripts" / "fixed_point_hover.py"
        text = source.read_text(encoding="utf-8")

        self.assertIn('"auto_arm", False', text)
        self.assertIn("2.0 * self.control_rate_hz", text)
        self.assertIn("msg.position = True", text)
        self.assertIn("msg.xy_valid and msg.z_valid", text)


if __name__ == "__main__":
    unittest.main()
