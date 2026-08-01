from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory("sim_mid360_sensor")
    config = LaunchConfiguration("config")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config", default_value=f"{share_dir}/config/mid360_adapter.yaml"
            ),
            Node(
                package="sim_mid360_sensor",
                executable="mid360_cloud_to_livox",
                name="mid360_cloud_to_livox",
                output="screen",
                parameters=[config],
            ),
        ]
    )
