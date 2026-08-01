from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    share_dir = get_package_share_directory("sim_ego_bringup")
    px4_launch = f"{share_dir}/launch/ego_px4.launch.py"
    root_dir = EnvironmentVariable("SIM_EGO_ROOT", default_value="/home/wu/sim-ego")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "scenario_file",
                default_value=PathJoinSubstitution(
                    [root_dir, "scenarios", "ego_slalom.yaml"]
                ),
            ),
            DeclareLaunchArgument("goal_x", default_value="6.0"),
            DeclareLaunchArgument("goal_y", default_value="0.0"),
            DeclareLaunchArgument("goal_z", default_value="1.5"),
            DeclareLaunchArgument("auto_arm", default_value="false"),
            DeclareLaunchArgument("auto_offboard", default_value="false"),
            DeclareLaunchArgument("control_mode", default_value="position"),
            DeclareLaunchArgument("offboard_prestream_sec", default_value="2.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(px4_launch),
                launch_arguments={
                    "use_pointlio": "false",
                    "use_px4_odom_bridge": "true",
                    "use_scenario_map": "true",
                    "scenario_file": LaunchConfiguration("scenario_file"),
                    "odom_topic": "/sim/odom",
                    "cloud_topic": "/map_generator/global_cloud",
                    "goal_x": LaunchConfiguration("goal_x"),
                    "goal_y": LaunchConfiguration("goal_y"),
                    "goal_z": LaunchConfiguration("goal_z"),
                    "realworld_experiment": "false",
                    "output_enabled": "true",
                    "auto_arm": LaunchConfiguration("auto_arm"),
                    "auto_offboard": LaunchConfiguration("auto_offboard"),
                    "control_mode": LaunchConfiguration("control_mode"),
                    "offboard_prestream_sec": LaunchConfiguration("offboard_prestream_sec"),
                    "use_rviz": LaunchConfiguration("use_rviz"),
                }.items(),
            ),
        ]
    )
