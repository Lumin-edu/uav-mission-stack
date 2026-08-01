import math
import sys
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from rigid_transform import compose_pose, rotate_vector


BASE_LINK_TO_BASE = (-0.011, -0.02329, -0.05588)
IDENTITY = (0.0, 0.0, 0.0, 1.0)


class RigidTransformTest(unittest.TestCase):
    def assert_vector_close(self, actual, expected) -> None:
        self.assertEqual(len(actual), len(expected))
        for actual_value, expected_value in zip(actual, expected):
            self.assertAlmostEqual(actual_value, expected_value, delta=1e-9)


    def test_level_pose_applies_base_link_to_base_offset_with_correct_sign(self) -> None:
        position, orientation = compose_pose(
            (0.0, 0.0, 0.0), IDENTITY, BASE_LINK_TO_BASE, IDENTITY
        )

        self.assert_vector_close(position, (-0.011, -0.02329, -0.05588))
        self.assert_vector_close(orientation, IDENTITY)


    def test_positive_ninety_degree_yaw_rotates_body_lever_arm_into_world(self) -> None:
        half_angle = math.pi / 4.0
        yaw_90 = (0.0, 0.0, math.sin(half_angle), math.cos(half_angle))

        position, _ = compose_pose((1.0, 2.0, 3.0), yaw_90, BASE_LINK_TO_BASE, IDENTITY)

        self.assert_vector_close(position, (1.02329, 1.989, 2.94412))


    def test_roll_and_pitch_rotate_offset_instead_of_adding_world_constant(self) -> None:
        roll_90 = (math.sin(math.pi / 4.0), 0.0, 0.0, math.cos(math.pi / 4.0))
        pitch_90 = (0.0, math.sin(math.pi / 4.0), 0.0, math.cos(math.pi / 4.0))

        self.assert_vector_close(rotate_vector(roll_90, (0.0, 0.0, -0.10)), (0.0, 0.10, 0.0))
        self.assert_vector_close(rotate_vector(pitch_90, (0.0, 0.0, -0.10)), (-0.10, 0.0, 0.0))


    def test_static_rotation_is_composed_after_pointlio_base_link_orientation(self) -> None:
        half_angle = math.pi / 4.0
        yaw_90 = (0.0, 0.0, math.sin(half_angle), math.cos(half_angle))

        _, orientation = compose_pose((0.0, 0.0, 0.0), yaw_90, (0.0, 0.0, 0.0), yaw_90)

        self.assert_vector_close(orientation, (0.0, 0.0, 1.0, 0.0))


    def test_invalid_zero_quaternion_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "norm is zero"):
            compose_pose(
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0, 0.0),
                BASE_LINK_TO_BASE,
                IDENTITY,
            )


if __name__ == "__main__":
    unittest.main()
