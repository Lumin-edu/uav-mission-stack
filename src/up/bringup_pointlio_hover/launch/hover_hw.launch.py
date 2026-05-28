import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    use_pointlio = LaunchConfiguration("use_pointlio")
    use_pointlio_px4_visual_odom = LaunchConfiguration("use_pointlio_px4_visual_odom")
    use_hover_control = LaunchConfiguration("use_hover_control")
    use_px4_monitor = LaunchConfiguration("use_px4_monitor")
    use_px4_control_watchdog = LaunchConfiguration("use_px4_control_watchdog")
    use_position_compare = LaunchConfiguration("use_position_compare")

    pointlio_config_file = LaunchConfiguration("pointlio_config_file")
    takeoff_altitude = LaunchConfiguration("takeoff_altitude")
    hover_auto_arm = LaunchConfiguration("hover_auto_arm")
    offboard_rate_hz = LaunchConfiguration("offboard_rate_hz")
    visual_odom_print_rate = LaunchConfiguration("visual_odom_print_rate")
    reference_capture_delay_sec = LaunchConfiguration("reference_capture_delay_sec")

    pointlio_share = get_package_share_directory("point_lio")
    pointlio_launch = os.path.join(pointlio_share, "launch", "mapping_headless.launch.py")
    default_pointlio_config = os.path.join(pointlio_share, "config", "mid360_mapping.yaml")

    pointlio_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pointlio_launch),
        launch_arguments={
            "point_lio_cfg_dir": pointlio_config_file,
            "rviz": "false",
        }.items(),
        condition=IfCondition(use_pointlio),
    )

    pointlio_to_px4_visual_odom = Node(
        package="hx_bringup_pointlio_hover",
        executable="pointlio_to_px4_visual_odom.py",
        output="screen",
        condition=IfCondition(use_pointlio_px4_visual_odom),
        parameters=[
            {
                "pointlio_topic": "/odom",
                "px4_local_topic": "/fmu/out/vehicle_local_position",
                "timesync_topic": "/fmu/out/timesync_status",
                "output_topic": "/fmu/in/vehicle_visual_odometry",
                "publish_rate_limit": 50.0,
                "align_when_px4_valid": True,
                "use_px4_reference": False,
                "max_px4_reference_abs_z": 20.0,
                "max_output_abs_z": 20.0,
                "max_position_jump": 3.0,
                "publish_orientation": True,
                "use_timesync_timestamp": False,
                "pointlio_y_to_px4_y_sign": -1.0,
                "yaw_sign": -1.0,
                "yaw_offset": 0.0,
                "publish_velocity": False,
                "position_variance": 0.04,
                "orientation_variance": 0.01,
                "velocity_variance": 0.25,
                "print_rate": visual_odom_print_rate,
            }
        ],
    )

    fixed_point_hover = Node(
        package="hx_bringup_pointlio_hover",
        executable="fixed_point_hover.py",
        output="screen",
        parameters=[
            {
                "target_x": 0.0,
                "target_y": 0.0,
                "target_z": PythonExpression(["-1.0 * ", takeoff_altitude]),
                "target_yaw": 0.0,
                "auto_arm": hover_auto_arm,
                "use_current_position_reference": True,
                "reference_capture_delay_sec": reference_capture_delay_sec,
                "control_rate_hz": offboard_rate_hz,
                "vehicle_local_position_topic": "/fmu/out/vehicle_local_position",
                "vehicle_status_topic": "/fmu/out/vehicle_status",
            },
        ],
        condition=IfCondition(use_hover_control),
    )

    px4_dds_monitor = Node(
        package="hx_bringup_pointlio_hover",
        executable="px4_dds_monitor.py",
        output="screen",
        condition=IfCondition(use_px4_monitor),
    )

    px4_control_watchdog = Node(
        package="hx_bringup_pointlio_hover",
        executable="px4_control_watchdog.py",
        output="screen",
        condition=IfCondition(use_px4_control_watchdog),
        parameters=[{"control_source": "fixed_point_hover"}],
    )

    px4_pointlio_position_compare = Node(
        package="hx_bringup_pointlio_hover",
        executable="px4_pointlio_position_compare.py",
        output="screen",
        condition=IfCondition(use_position_compare),
        parameters=[
            {
                "px4_topic": "/fmu/out/vehicle_local_position",
                "pointlio_topic": "/odom",
                "print_rate": 2.0,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_pointlio", default_value="true"),
            DeclareLaunchArgument("use_pointlio_px4_visual_odom", default_value="true"),
            DeclareLaunchArgument("use_hover_control", default_value="true"),
            DeclareLaunchArgument("use_px4_monitor", default_value="false"),
            DeclareLaunchArgument("use_px4_control_watchdog", default_value="true"),
            DeclareLaunchArgument("use_position_compare", default_value="false"),
            DeclareLaunchArgument("pointlio_config_file", default_value=default_pointlio_config),
            DeclareLaunchArgument("takeoff_altitude", default_value="0.2"),
            DeclareLaunchArgument("hover_auto_arm", default_value="false"),
            DeclareLaunchArgument("offboard_rate_hz", default_value="50.0"),
            DeclareLaunchArgument("visual_odom_print_rate", default_value="1.0"),
            DeclareLaunchArgument("reference_capture_delay_sec", default_value="3.0"),
            pointlio_mapping,
            pointlio_to_px4_visual_odom,
            px4_dds_monitor,
            px4_control_watchdog,
            px4_pointlio_position_compare,
            fixed_point_hover,
        ]
    )
