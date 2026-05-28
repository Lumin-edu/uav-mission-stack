from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    point_lio_dir = get_package_share_directory("point_lio")

    pcd_file = LaunchConfiguration("pcd_file")
    interval = LaunchConfiguration("interval")
    frame_id = LaunchConfiguration("frame_id")
    cloud_topic = LaunchConfiguration("cloud_topic")
    rviz_cfg = LaunchConfiguration("rviz_cfg")

    declare_pcd_file = DeclareLaunchArgument(
        "pcd_file",
        default_value=PathJoinSubstitution(
            [
                EnvironmentVariable("HOME"),
                "venom_ws",
                "src",
                "venom_vnv",
                "lio",
                "Point-LIO",
                "PCD",
                "scans.pcd",
            ]
        ),
        description="Absolute path to the PCD file to visualize",
    )
    declare_interval = DeclareLaunchArgument(
        "interval",
        default_value="0.1",
        description="Publish interval in seconds",
    )
    declare_frame_id = DeclareLaunchArgument(
        "frame_id",
        default_value="map",
        description="Frame id used for the published cloud",
    )
    declare_cloud_topic = DeclareLaunchArgument(
        "cloud_topic",
        default_value="/pcd_map",
        description="Output PointCloud2 topic",
    )
    declare_rviz_cfg = DeclareLaunchArgument(
        "rviz_cfg",
        default_value=PathJoinSubstitution([point_lio_dir, "rviz_cfg", "pcd_map.rviz"]),
        description="RViz config path",
    )

    pcd_pub = Node(
        package="pcl_ros",
        executable="pcd_to_pointcloud",
        name="pcd_to_pointcloud",
        output="screen",
        parameters=[
            {
                "file_name": pcd_file,
                "interval": interval,
                "tf_frame": frame_id,
            }
        ],
        remappings=[("cloud_pcd", cloud_topic)],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz_pcd_map",
        output="screen",
        arguments=["-d", rviz_cfg],
    )

    return LaunchDescription(
        [
            declare_pcd_file,
            declare_interval,
            declare_frame_id,
            declare_cloud_topic,
            declare_rviz_cfg,
            pcd_pub,
            rviz,
        ]
    )
