import math
import sys
import unittest
from pathlib import Path

from nav_msgs.msg import Odometry

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from vision_pose_adapter import odometry_to_vision_pose


class VisionPoseAdapterTest(unittest.TestCase):
    def test_pointlio_flu_pose_is_not_converted_to_ned(self) -> None:
        source = Odometry()
        source.header.stamp.sec = 12
        source.header.stamp.nanosec = 345
        source.pose.pose.position.x = 1.0
        source.pose.pose.position.y = 2.0
        source.pose.pose.position.z = 3.0
        source.pose.pose.orientation.w = 1.0

        output = odometry_to_vision_pose(
            source,
            base_link_to_base_translation=(-0.011, -0.02329, -0.05588),
            base_link_to_base_rotation_xyzw=(0.0, 0.0, 0.0, 1.0),
            output_frame_id="odom",
        )

        self.assertEqual(output.header.stamp, source.header.stamp)
        self.assertEqual(output.header.frame_id, "odom")
        self.assertAlmostEqual(output.pose.position.x, 0.989)
        self.assertAlmostEqual(output.pose.position.y, 1.97671)
        self.assertAlmostEqual(output.pose.position.z, 2.94412)
        self.assertAlmostEqual(output.pose.orientation.x, 0.0)
        self.assertAlmostEqual(output.pose.orientation.y, 0.0)
        self.assertAlmostEqual(output.pose.orientation.z, 0.0)
        self.assertAlmostEqual(output.pose.orientation.w, 1.0)

    def test_body_offset_rotates_with_pointlio_flu_yaw(self) -> None:
        source = Odometry()
        source.pose.pose.orientation.z = math.sin(math.pi / 4.0)
        source.pose.pose.orientation.w = math.cos(math.pi / 4.0)

        output = odometry_to_vision_pose(
            source,
            base_link_to_base_translation=(1.0, 0.0, 0.0),
            base_link_to_base_rotation_xyzw=(0.0, 0.0, 0.0, 1.0),
            output_frame_id="odom",
        )

        self.assertAlmostEqual(output.pose.position.x, 0.0, places=7)
        self.assertAlmostEqual(output.pose.position.y, 1.0, places=7)
        self.assertAlmostEqual(output.pose.position.z, 0.0, places=7)
        self.assertAlmostEqual(output.pose.orientation.z, math.sin(math.pi / 4.0))
        self.assertAlmostEqual(output.pose.orientation.w, math.cos(math.pi / 4.0))

    def test_invalid_pointlio_quaternion_is_rejected(self) -> None:
        source = Odometry()
        source.pose.pose.orientation.w = 0.0
        with self.assertRaisesRegex(ValueError, "quaternion norm is zero"):
            odometry_to_vision_pose(
                source,
                base_link_to_base_translation=(0.0, 0.0, 0.0),
                base_link_to_base_rotation_xyzw=(0.0, 0.0, 0.0, 1.0),
                output_frame_id="odom",
            )


if __name__ == "__main__":
    unittest.main()
