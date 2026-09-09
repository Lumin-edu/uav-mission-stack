import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Works both from an installed package and when ros2 launch receives the source path directly.
    package_share = str(Path(__file__).resolve().parents[1])
    default_config = os.path.join(package_share, "config", "mid360_fastlio.yaml")
    default_rviz = os.path.join(package_share, "config", "fastlio.rviz")

    use_fastlio = LaunchConfiguration("use_fastlio")
    use_visual_odom = LaunchConfiguration("use_visual_odom")
    use_hover_control = LaunchConfiguration("use_hover_control")
    use_px4_monitor = LaunchConfiguration("use_px4_monitor")
    use_px4_control_watchdog = LaunchConfiguration("use_px4_control_watchdog")
    use_position_compare = LaunchConfiguration("use_position_compare")
    use_rviz = LaunchConfiguration("use_rviz")
    use_body_to_base_tf = LaunchConfiguration("use_body_to_base_tf")

    fastlio_config_file = LaunchConfiguration("fastlio_config_file")
    fastlio_topic = LaunchConfiguration("fastlio_topic")
    px4_local_topic = LaunchConfiguration("px4_local_topic")
    timesync_topic = LaunchConfiguration("timesync_topic")
    output_topic = LaunchConfiguration("output_topic")
    rviz_cfg = LaunchConfiguration("rviz_cfg")
    takeoff_altitude = LaunchConfiguration("takeoff_altitude")
    hover_auto_arm = LaunchConfiguration("hover_auto_arm")
    offboard_rate_hz = LaunchConfiguration("offboard_rate_hz")
    reference_capture_delay_sec = LaunchConfiguration("reference_capture_delay_sec")
    visual_odom_print_rate = LaunchConfiguration("visual_odom_print_rate")
    max_output_abs_z = LaunchConfiguration("max_output_abs_z")
    max_position_jump = LaunchConfiguration("max_position_jump")

    body_to_base_translation = [-0.011, -0.02329, -0.05588]
    body_to_base_rotation_xyzw = [0.0, 0.0, 0.0, 1.0]

    fastlio_node = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="fastlio_mapping",
        parameters=[fastlio_config_file],
        output="screen",
        condition=IfCondition(use_fastlio),
    )

    fastlio_bridge = Node(
        package="hx_fastlio_bringup",
        executable="fastlio_to_px4_visual_odom.py",
        name="fastlio_to_px4_visual_odom",
        output="screen",
        condition=IfCondition(use_visual_odom),
        parameters=[
            {
                "fastlio_topic": fastlio_topic,
                "px4_local_topic": px4_local_topic,
                "timesync_topic": timesync_topic,
                "output_topic": output_topic,
                "fastlio_pose_frame": "body",
                "fastlio_world_frame": "camera_init",
                "vehicle_frame": "base",
                "body_to_base_translation": body_to_base_translation,
                "body_to_base_rotation_xyzw": body_to_base_rotation_xyzw,
                "publish_rate_limit": 50.0,
                "align_when_px4_valid": True,
                "use_px4_reference": False,
                "use_px4_heading_reference": False,
                "max_output_abs_z": max_output_abs_z,
                "max_position_jump": max_position_jump,
                "publish_orientation": False,
                "publish_velocity": False,
                "use_timesync_timestamp": False,
                "fastlio_y_to_px4_y_sign": -1.0,
                "yaw_sign": -1.0,
                "yaw_offset": 0.0,
                "print_rate": visual_odom_print_rate,
            }
        ],
    )

    body_to_base_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="body_to_base_tf",
        output="screen",
        condition=IfCondition(use_body_to_base_tf),
        arguments=[
            "--x",
            str(body_to_base_translation[0]),
            "--y",
            str(body_to_base_translation[1]),
            "--z",
            str(body_to_base_translation[2]),
            "--qx",
            str(body_to_base_rotation_xyzw[0]),
            "--qy",
            str(body_to_base_rotation_xyzw[1]),
            "--qz",
            str(body_to_base_rotation_xyzw[2]),
            "--qw",
            str(body_to_base_rotation_xyzw[3]),
            "--frame-id",
            "body",
            "--child-frame-id",
            "base",
        ],
    )

    fixed_point_hover = Node(
        package="hx_fastlio_bringup",
        executable="fixed_point_hover.py",
        name="fastlio_fixed_point_hover",
        output="screen",
        condition=IfCondition(use_hover_control),
        parameters=[
            {
                "target_x": 0.0,
                "target_y": 0.0,
                "target_z": PythonExpression(["-1.0 * ", takeoff_altitude]),
                "target_yaw": 0.0,
                "auto_arm": hover_auto_arm,
                "use_current_position_reference": True,
                "reference_capture_delay_sec": reference_capture_delay_sec,
                "max_local_position_age_sec": 0.5,
                "max_vehicle_status_age_sec": 0.5,
                "control_rate_hz": offboard_rate_hz,
                "vehicle_local_position_topic": px4_local_topic,
                "vehicle_status_topic": "/fmu/out/vehicle_status",
            }
        ],
    )

    px4_monitor = Node(
        package="hx_fastlio_bringup",
        executable="px4_dds_monitor.py",
        name="fastlio_px4_dds_monitor",
        output="screen",
        condition=IfCondition(use_px4_monitor),
        parameters=[{"vehicle_local_position_topic": px4_local_topic}],
    )

    control_watchdog = Node(
        package="hx_fastlio_bringup",
        executable="px4_control_watchdog.py",
        name="fastlio_px4_control_watchdog",
        output="screen",
        condition=IfCondition(use_px4_control_watchdog),
        parameters=[{"control_source": "fastlio_fixed_point_hover"}],
    )

    position_compare = Node(
        package="hx_fastlio_bringup",
        executable="px4_fastlio_position_compare.py",
        name="px4_fastlio_position_compare",
        output="screen",
        condition=IfCondition(use_position_compare),
        parameters=[
            {
                "px4_topic": px4_local_topic,
                "visual_odom_topic": output_topic,
                "print_rate": 2.0,
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="fastlio_rviz",
        arguments=["-d", rviz_cfg],
        output="screen",
        condition=IfCondition(use_rviz),
    )

    arguments = [
        DeclareLaunchArgument("use_fastlio", default_value="true"),
        DeclareLaunchArgument("use_visual_odom", default_value="true"),
        DeclareLaunchArgument("use_hover_control", default_value="true"),
        DeclareLaunchArgument("use_px4_monitor", default_value="false"),
        DeclareLaunchArgument("use_px4_control_watchdog", default_value="true"),
        DeclareLaunchArgument("use_position_compare", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("use_body_to_base_tf", default_value="true"),
        DeclareLaunchArgument("fastlio_config_file", default_value=default_config),
        DeclareLaunchArgument("fastlio_topic", default_value="/Odometry"),
        DeclareLaunchArgument(
            "px4_local_topic", default_value="/fmu/out/vehicle_local_position"
        ),
        DeclareLaunchArgument("timesync_topic", default_value="/fmu/out/timesync_status"),
        DeclareLaunchArgument(
            "output_topic", default_value="/fmu/in/vehicle_visual_odometry"
        ),
        DeclareLaunchArgument("rviz_cfg", default_value=default_rviz),
        DeclareLaunchArgument("takeoff_altitude", default_value="0.2"),
        DeclareLaunchArgument("hover_auto_arm", default_value="false"),
        DeclareLaunchArgument("offboard_rate_hz", default_value="50.0"),
        DeclareLaunchArgument("reference_capture_delay_sec", default_value="3.0"),
        DeclareLaunchArgument("visual_odom_print_rate", default_value="1.0"),
        DeclareLaunchArgument("max_output_abs_z", default_value="20.0"),
        DeclareLaunchArgument("max_position_jump", default_value="3.0"),
    ]
    return LaunchDescription(
        arguments
        + [
            fastlio_node,
            body_to_base_tf,
            fastlio_bridge,
            px4_monitor,
            control_watchdog,
            position_compare,
            fixed_point_hover,
            rviz_node,
        ]
    )
