from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _topic(namespace, suffix):
    """Build an absolute topic while keeping the MAVROS namespace configurable."""
    return PythonExpression(["'", namespace, "' + '", suffix, "'"])


def generate_launch_description():
    mavros_namespace = LaunchConfiguration("mavros_namespace")
    config_file = LaunchConfiguration("config_file")
    bridge_config_file = LaunchConfiguration("bridge_config_file")
    use_pointlio = LaunchConfiguration("use_pointlio")
    pointlio_config = LaunchConfiguration("pointlio_config")
    pointlio_odom_topic = LaunchConfiguration("pointlio_odom_topic")
    vision_pose_topic = LaunchConfiguration("vision_pose_topic")

    pointlio_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("point_lio"), "launch", "mapping_headless.launch.py"]
            )
        ),
        launch_arguments={
            "point_lio_cfg_dir": pointlio_config,
            "rviz": "false",
        }.items(),
        condition=IfCondition(use_pointlio),
    )

    vision_pose_bridge = Node(
        package="bringup_ego_mavros",
        executable="pointlio_to_mavros_vision_pose.py",
        name="pointlio_to_mavros_vision_pose",
        output="screen",
        parameters=[
            bridge_config_file,
            {
                "pointlio_odom_topic": pointlio_odom_topic,
                "vision_pose_topic": vision_pose_topic,
            },
        ],
    )
    controller = Node(
        package="bringup_ego_mavros",
        executable="fixed_point_mavros.py",
        name="fixed_point_mavros",
        namespace=mavros_namespace,
        output="screen",
        parameters=[
            config_file,
            {
                "target_x": LaunchConfiguration("target_x"),
                "target_y": LaunchConfiguration("target_y"),
                "target_z": LaunchConfiguration("target_z"),
                "target_yaw": LaunchConfiguration("target_yaw"),
                "hold_current_yaw": LaunchConfiguration("hold_current_yaw"),
                "auto_arm": LaunchConfiguration("auto_arm"),
                "auto_offboard": LaunchConfiguration("auto_offboard"),
                "use_current_position_reference": LaunchConfiguration(
                    "use_current_position_reference"
                ),
                "reference_capture_delay_sec": LaunchConfiguration(
                    "reference_capture_delay_sec"
                ),
                "control_rate_hz": LaunchConfiguration("control_rate_hz"),
                "prestream_sec": LaunchConfiguration("prestream_sec"),
                "state_topic": _topic(mavros_namespace, "/state"),
                "local_odom_topic": _topic(mavros_namespace, "/local_position/odom"),
                "setpoint_topic": _topic(mavros_namespace, "/setpoint_raw/local"),
                "arming_service": _topic(mavros_namespace, "/cmd/arming"),
                "mode_service": _topic(mavros_namespace, "/set_mode"),
            },
        ],
    )

    arguments = [
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("bringup_ego_mavros"),
                    "config",
                    "fixed_point_mavros.yaml",
                ]
            ),
            description="Base parameter file for the fixed-point controller.",
        ),
        DeclareLaunchArgument(
            "bridge_config_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("bringup_ego_mavros"),
                    "config",
                    "pointlio_mavros_bridge.yaml",
                ]
            ),
            description="Point-LIO to MAVROS vision-pose bridge parameters.",
        ),
        DeclareLaunchArgument(
            "use_pointlio",
            default_value="true",
            description="Automatically start Point-LIO.",
        ),
        DeclareLaunchArgument(
            "pointlio_config",
            default_value=PathJoinSubstitution(
                [FindPackageShare("point_lio"), "config", "mid360_mapping.yaml"]
            ),
            description="Point-LIO mapping configuration file.",
        ),
        DeclareLaunchArgument(
            "pointlio_odom_topic",
            default_value="/odom",
            description="Point-LIO FLU/ENU odometry topic.",
        ),
        DeclareLaunchArgument(
            "vision_pose_topic",
            default_value=PythonExpression(
                ["'", mavros_namespace, "' + '/vision_pose/pose'"]
            ),
            description="MAVROS external vision pose input.",
        ),
        DeclareLaunchArgument(
            "mavros_namespace", default_value="/mavros", description="MAVROS namespace."
        ),
        DeclareLaunchArgument(
            "target_x",
            default_value="0.0",
            description="Local ENU x offset from the position captured after the delay.",
        ),
        DeclareLaunchArgument(
            "target_y",
            default_value="0.0",
            description="Local ENU y offset from the position captured after the delay.",
        ),
        DeclareLaunchArgument(
            "target_z",
            default_value="1.0",
            description="Local ENU z offset from the position captured after the delay.",
        ),
        DeclareLaunchArgument(
            "target_yaw", default_value="0.0", description="Target yaw in radians."
        ),
        DeclareLaunchArgument(
            "hold_current_yaw",
            default_value="true",
            description="Hold the yaw captured from MAVROS local odometry at takeoff.",
        ),
        DeclareLaunchArgument(
            "auto_arm",
            default_value="false",
            description="Automatically arm through MAVROS after connection.",
        ),
        DeclareLaunchArgument(
            "auto_offboard",
            default_value="true",
            description="Request OFFBOARD after setpoint pre-stream and arming.",
        ),
        DeclareLaunchArgument(
            "use_current_position_reference",
            default_value="true",
            description="Interpret target values as offsets from the first valid local odometry.",
        ),
        DeclareLaunchArgument(
            "reference_capture_delay_sec",
            default_value="3.0",
            description="Delay before capturing the local odometry reference.",
        ),
        DeclareLaunchArgument(
            "control_rate_hz",
            default_value="50.0",
            description="PositionTarget publication rate in Hz.",
        ),
        DeclareLaunchArgument(
            "prestream_sec",
            default_value="2.0",
            description="Setpoint pre-stream duration before mode/arming requests.",
        ),
    ]
    return LaunchDescription(
        arguments + [pointlio_mapping, vision_pose_bridge, controller]
    )
