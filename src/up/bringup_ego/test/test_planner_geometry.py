import math
import struct
import sys
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from planner_altitude_fusion import PlannerAltitudeFusion, base_height_from_px4_ned
from rigid_transform import compose_pose
from sensor_msgs.msg import PointCloud2, PointField


BASE_LINK_TO_BASE = (-0.011, -0.02329, -0.05588)
IDENTITY = (0.0, 0.0, 0.0, 1.0)


class PlannerGeometryTest(unittest.TestCase):
    def assert_vector_close(self, actual, expected) -> None:
        self.assertEqual(len(actual), len(expected))
        for actual_value, expected_value in zip(actual, expected):
            self.assertAlmostEqual(actual_value, expected_value, delta=1e-9)

    def test_level_pose_uses_aircraft_center(self) -> None:
        position, orientation = compose_pose(
            (0.0, 0.0, 0.0), IDENTITY, BASE_LINK_TO_BASE, IDENTITY
        )
        self.assert_vector_close(position, BASE_LINK_TO_BASE)
        self.assert_vector_close(orientation, IDENTITY)

    def test_yaw_rotates_lever_arm_before_world_translation(self) -> None:
        half_angle = math.pi / 4.0
        yaw_90 = (0.0, 0.0, math.sin(half_angle), math.cos(half_angle))
        position, _ = compose_pose(
            (1.0, 2.0, 3.0), yaw_90, BASE_LINK_TO_BASE, IDENTITY
        )
        self.assert_vector_close(position, (1.02329, 1.989, 2.94412))

    def test_px4_ned_height_is_converted_to_ros_up(self) -> None:
        reference_base_z = -0.05588
        self.assertAlmostEqual(
            base_height_from_px4_ned(reference_base_z, 0.0, -1.0),
            0.94412,
            delta=1e-9,
        )
        self.assertAlmostEqual(
            base_height_from_px4_ned(reference_base_z, 0.0, 0.5),
            -0.55588,
            delta=1e-9,
        )

    def test_cloud_z_uses_same_height_correction(self) -> None:
        cloud = PointCloud2()
        cloud.height = 1
        cloud.width = 2
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 24
        cloud.data = struct.pack("<ffffff", 1.0, 2.0, 3.0, 4.0, 5.0, -0.5)

        shifted = PlannerAltitudeFusion.shift_cloud_z(cloud, 0.25)

        self.assertIsNotNone(shifted)
        values = struct.unpack("<ffffff", shifted.data)
        self.assert_vector_close(values, (1.0, 2.0, 3.25, 4.0, 5.0, -0.25))
        self.assertEqual(
            bytes(cloud.data),
            struct.pack("<ffffff", 1.0, 2.0, 3.0, 4.0, 5.0, -0.5),
        )

    def test_invalid_quaternion_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "norm is zero"):
            compose_pose(
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0, 0.0),
                BASE_LINK_TO_BASE,
                IDENTITY,
            )


if __name__ == "__main__":
    unittest.main()
