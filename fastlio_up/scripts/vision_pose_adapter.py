from collections.abc import Sequence

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rigid_transform import compose_pose


def odometry_to_vision_pose(
    source: Odometry,
    *,
    base_link_to_base_translation: Sequence[float],
    base_link_to_base_rotation_xyzw: Sequence[float],
    output_frame_id: str,
) -> PoseStamped:
    position = source.pose.pose.position
    orientation = source.pose.pose.orientation
    vehicle_position, vehicle_orientation = compose_pose(
        (position.x, position.y, position.z),
        (orientation.x, orientation.y, orientation.z, orientation.w),
        base_link_to_base_translation,
        base_link_to_base_rotation_xyzw,
    )

    output = PoseStamped()
    output.header.stamp = source.header.stamp
    output.header.frame_id = output_frame_id
    output.pose.position.x, output.pose.position.y, output.pose.position.z = (
        vehicle_position
    )
    (
        output.pose.orientation.x,
        output.pose.orientation.y,
        output.pose.orientation.z,
        output.pose.orientation.w,
    ) = vehicle_orientation
    return output
