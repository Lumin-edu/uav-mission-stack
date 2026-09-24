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
    use_fastlio = LaunchConfiguration("use_fastlio")
    fastlio_config = LaunchConfiguration("fastlio_config")
    fastlio_odom_topic = LaunchConfiguration("fastlio_odom_topic")
    vision_pose_topic = LaunchConfiguration("vision_pose_topic")

    fastlio_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("fast_lio"), "launch", "mapping.launch.py"]
            )
        ),
        launch_arguments={
            "config_path": PathJoinSubstitution([FindPackageShare("fast_lio"), "config"]),
            "config_file": fastlio_config,
            "rviz": "false",
        }.items(),
        condition=IfCondition(use_fastlio),
    )

    vision_pose_bridge = Node(
        package="fastlio_up",
        executable="fastlio_to_mavros_vision_pose.py",
        name="fastlio_to_mavros_vision_pose",
        output="screen",
        parameters=[
            bridge_config_file,
            {
                "fastlio_odom_topic": fastlio_odom_topic,
                "vision_pose_topic": vision_pose_topic,
            },
        ],
    )
    controller = Node(
        package="fastlio_up",
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
                "reference_capture_delay_sec": LaunchConfiguration(
                    "reference_capture_delay_sec"
                ),
                "control_rate_hz": LaunchConfiguration("control_rate_hz"),
                "prestream_sec": LaunchConfiguration("prestream_sec"),
                "max_speed": LaunchConfiguration("max_speed"),
                "state_topic": _topic(mavros_namespace, "/state"),
                "local_odom_topic": _topic(mavros_namespace, "/local_position/odom"),
                "fastlio_odom_topic": fastlio_odom_topic,
                "setpoint_topic": _topic(
                    mavros_namespace, "/setpoint_position/local"
                ),
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
                    FindPackageShare("fastlio_up"),
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
                    FindPackageShare("fastlio_up"),
                    "config",
                    "fastlio_mavros_bridge.yaml",
                ]
            ),
            description="FAST-LIO to MAVROS vision-pose bridge parameters.",
        ),
        DeclareLaunchArgument(
            "use_fastlio",
            default_value="true",
            description="Automatically start FAST-LIO.",
        ),
        DeclareLaunchArgument(
            "fastlio_config", default_value="mid360s.yaml",
            description="FAST-LIO mapping configuration file name.",
        ),
        DeclareLaunchArgument(
            "fastlio_odom_topic", default_value="/Odometry",
            description="FAST-LIO odometry topic.",
        ),
        DeclareLaunchArgument(
            "mavros_namespace", default_value="/mavros", description="MAVROS namespace."
        ),
        DeclareLaunchArgument(
            "vision_pose_topic",
            default_value=PythonExpression(
                ["'", mavros_namespace, "' + '/vision_pose/pose'"]
            ),
            description="MAVROS external vision pose input.",
        ),
        DeclareLaunchArgument(
            "target_x",
            default_value="0.0",
            description="Task x offset: right of the captured heading, in meters.",
        ),
        DeclareLaunchArgument(
            "target_y",
            default_value="0.0",
            description="Task y offset: forward of the captured heading, in meters.",
        ),
        DeclareLaunchArgument(
            "target_z",
            default_value="1.0",
            description="Task z offset: up from the captured position, in meters.",
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
            "reference_capture_delay_sec",
            default_value="3.0",
            description="Delay before capturing the local odometry reference.",
        ),
        DeclareLaunchArgument(
            "control_rate_hz",
            default_value="50.0",
            description="PoseStamped setpoint publication rate in Hz.",
        ),
        DeclareLaunchArgument(
            "prestream_sec",
            default_value="2.0",
            description="Setpoint pre-stream duration before mode/arming requests.",
        ),
        DeclareLaunchArgument(
            "max_speed",
            default_value="0.8",
            description="Maximum setpoint approach speed in meters per second.",
        ),
    ]
    return LaunchDescription(
        arguments + [fastlio_mapping, vision_pose_bridge, controller]
    )
