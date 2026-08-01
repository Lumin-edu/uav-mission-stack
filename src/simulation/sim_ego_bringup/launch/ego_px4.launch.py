from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _optional_pointlio(context):
    if LaunchConfiguration("use_pointlio").perform(context).lower() not in {"1", "true", "yes"}:
        return []
    pointlio_share = get_package_share_directory("point_lio")
    pointlio_launch = f"{pointlio_share}/launch/mapping_headless.launch.py"
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pointlio_launch),
            launch_arguments={
                "point_lio_cfg_dir": LaunchConfiguration("pointlio_config"),
                "rviz": "false",
            }.items(),
        )
    ]


def generate_launch_description():
    share_dir = get_package_share_directory("sim_ego_bringup")
    root_dir = EnvironmentVariable("SIM_EGO_ROOT", default_value="/home/wu/sim-ego")
    planner_config = LaunchConfiguration("planner_config")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    use_px4_odom_bridge = LaunchConfiguration("use_px4_odom_bridge")
    use_scenario_map = LaunchConfiguration("use_scenario_map")
    use_health_monitor = LaunchConfiguration("use_health_monitor")
    pointlio_mapping = OpaqueFunction(function=_optional_pointlio)

    planner = Node(
        package="ego_planner",
        executable="ego_planner_node",
        name="ego_planner",
        output="screen",
        parameters=[
            planner_config,
            {
                "fsm/realworld_experiment": LaunchConfiguration("realworld_experiment"),
                "fsm/flight_type": LaunchConfiguration("flight_type"),
                "fsm/waypoint0_x": LaunchConfiguration("goal_x"),
                "fsm/waypoint0_y": LaunchConfiguration("goal_y"),
                "fsm/waypoint0_z": LaunchConfiguration("goal_z"),
                "fsm/manual_goal_z": LaunchConfiguration("goal_z"),
                "fsm/validate_goal_occupancy": LaunchConfiguration(
                    "validate_goal_occupancy"
                ),
                "fsm/require_occupancy_for_goal": LaunchConfiguration(
                    "require_occupancy_for_goal"
                ),
                "grid_map/frame_id": LaunchConfiguration("map_frame"),
                "grid_map/use_static_map": LaunchConfiguration("use_static_map"),
                "grid_map/static_map_inflation": LaunchConfiguration(
                    "static_map_inflation"
                ),
                "manager/max_vel": LaunchConfiguration("max_velocity"),
                "manager/max_acc": LaunchConfiguration("max_acceleration"),
                "optimization/max_vel": LaunchConfiguration("max_velocity"),
                "optimization/max_acc": LaunchConfiguration("max_acceleration"),
                "bspline/limit_vel": LaunchConfiguration("max_velocity"),
                "bspline/limit_acc": LaunchConfiguration("max_acceleration"),
            },
        ],
        remappings=[
            ("odom_world", LaunchConfiguration("odom_topic")),
            ("grid_map/odom", LaunchConfiguration("odom_topic")),
            ("grid_map/cloud", LaunchConfiguration("cloud_topic")),
            ("grid_map/static_cloud", LaunchConfiguration("static_cloud_topic")),
            ("planning/bspline", "/ego/planning/bspline"),
            ("planning/data_display", "/ego/planning/data_display"),
            ("planning/broadcast_bspline_from_planner", "/ego/broadcast_bspline"),
            ("planning/broadcast_bspline_to_planner", "/ego/broadcast_bspline"),
            ("grid_map/occupancy_inflate", "/ego/grid_map/occupancy_inflate"),
            (
                "grid_map/static_occupancy_inflate",
                "/ego/grid_map/static_occupancy_inflate",
            ),
        ],
    )

    px4_odom_bridge = Node(
        package="sim_ego_bringup",
        executable="px4_odom_bridge.py",
        name="px4_odom_bridge",
        output="screen",
        condition=IfCondition(use_px4_odom_bridge),
        parameters=[
            {
                "px4_position_topic": "/fmu/out/vehicle_local_position",
                "odom_topic": LaunchConfiguration("odom_topic"),
                "frame_id": "world",
                "child_frame_id": "base_link",
            }
        ],
    )

    scenario_map = Node(
        package="sim_ego_bringup",
        executable="scenario_map.py",
        name="scenario_map",
        output="screen",
        condition=IfCondition(use_scenario_map),
        parameters=[
            {
                "scenario_file": LaunchConfiguration("scenario_file"),
                "map_topic": LaunchConfiguration("cloud_topic"),
                "publish_rate_hz": 2.0,
            }
        ],
    )

    trajectory_server = Node(
        package="ego_planner",
        executable="traj_server",
        name="ego_trajectory_server",
        output="screen",
        parameters=[{"traj_server/time_forward": 1.0}],
        remappings=[
            ("planning/bspline", "/ego/planning/bspline"),
            ("/position_cmd", "/ego/position_cmd"),
        ],
    )

    px4_bridge = Node(
        package="sim_ego_bringup",
        executable="ego_px4_bridge.py",
        name="ego_px4_bridge",
        output="screen",
        parameters=[
            {
                "ego_command_topic": "/ego/position_cmd",
                "planner_trajectory_topic": LaunchConfiguration(
                    "planner_trajectory_topic"
                ),
                "planner_trajectory_timeout_sec": LaunchConfiguration(
                    "planner_trajectory_timeout_sec"
                ),
                "ego_odom_topic": LaunchConfiguration("odom_topic"),
                "localization_health_topic": LaunchConfiguration(
                    "localization_health_topic"
                ),
                "output_enabled": LaunchConfiguration("output_enabled"),
                "auto_arm": LaunchConfiguration("auto_arm"),
                "auto_offboard": LaunchConfiguration("auto_offboard"),
                "control_mode": LaunchConfiguration("control_mode"),
                "offboard_prestream_sec": LaunchConfiguration("offboard_prestream_sec"),
                "takeoff_before_ego": LaunchConfiguration("takeoff_before_ego"),
                "takeoff_altitude": LaunchConfiguration("takeoff_altitude"),
                "takeoff_vertical_speed": LaunchConfiguration("takeoff_vertical_speed"),
                "takeoff_reach_xy_tol": LaunchConfiguration("takeoff_reach_xy_tol"),
                "takeoff_reach_z_tol": LaunchConfiguration("takeoff_reach_z_tol"),
                "takeoff_speed_xy_tol": LaunchConfiguration("takeoff_speed_xy_tol"),
                "takeoff_speed_z_tol": LaunchConfiguration("takeoff_speed_z_tol"),
                "takeoff_stable_sec": LaunchConfiguration("takeoff_stable_sec"),
                "reference_capture_delay_sec": LaunchConfiguration(
                    "reference_capture_delay_sec"
                ),
                # EGO/Point-LIO use ROS FLU; match the verified hardware bridge.
                # x_forward/y_left/z_up -> PX4 N/E/D.
                "world_to_px4_rotation": [1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0],
                "world_y_to_px4_y_sign": -1.0,
                "world_z_to_px4_z_sign": -1.0,
                "yaw_sign": 1.0,
                "yaw_offset": 0.0,
                # Match square_mission_controller when enabled by the outer
                # launch: task axes are relative to the captured PX4 heading.
                "use_initial_heading_frame": LaunchConfiguration("use_initial_heading_frame"),
                "lock_yaw_to_initial_heading": LaunchConfiguration(
                    "lock_yaw_to_initial_heading"
                ),
                "command_timeout_sec": 0.30,
                "odom_timeout_sec": 0.30,
                "control_rate_hz": 50.0,
            }
        ],
    )

    health_monitor = Node(
        package="sim_ego_bringup",
        executable="sim_health_monitor.py",
        name="px4_health_monitor",
        output="screen",
        condition=IfCondition(use_health_monitor),
        parameters=[
            {
                "mode": "px4",
                "odom_topic": LaunchConfiguration("odom_topic"),
                "command_topic": "/ego/position_cmd",
                "planner_trajectory_topic": LaunchConfiguration(
                    "planner_trajectory_topic"
                ),
                "planner_trajectory_timeout_sec": LaunchConfiguration(
                    "planner_trajectory_timeout_sec"
                ),
                "cloud_topic": LaunchConfiguration("cloud_topic"),
                "px4_position_topic": "/fmu/out/vehicle_local_position",
                "vehicle_status_topic": "/fmu/out/vehicle_status",
                "localization_health_topic": LaunchConfiguration(
                    "localization_health_topic"
                ),
                "max_command_age_sec": 0.30,
                "max_odom_age_sec": 0.25,
            }
        ],
    )

    waypoint_manager = Node(
        package="sim_ego_bringup",
        executable="waypoint_manager.py",
        name="waypoint_manager",
        output="screen",
        parameters=[
            {
                "waypoint_file": LaunchConfiguration("waypoint_file"),
                "input_topic": "/move_base_simple/goal",
                "marker_topic": "/sim/waypoints",
                "path_topic": "/sim/waypoint_path",
                "append_mode": True,
                "default_altitude": LaunchConfiguration("goal_z"),
                "auto_save": True,
            }
        ],
    )

    path_visualizer = Node(
        package="sim_ego_bringup",
        executable="path_visualizer.py",
        name="path_visualizer",
        output="screen",
        parameters=[
            {
                "odom_topic": LaunchConfiguration("odom_topic"),
                "command_topic": "/ego/position_cmd",
                "odom_path_topic": "/sim/odom_path",
                "command_path_topic": "/sim/command_path",
                "publish_rate_hz": 2.0,
                "max_points": 5000,
                "sample_distance_m": 0.05,
                "sample_interval_sec": 0.10,
            }
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("planner_config", default_value=f"{share_dir}/config/ego_planner.yaml"),
            DeclareLaunchArgument("use_pointlio", default_value="false"),
            DeclareLaunchArgument("use_px4_odom_bridge", default_value="false"),
            DeclareLaunchArgument("use_scenario_map", default_value="false"),
            DeclareLaunchArgument("use_health_monitor", default_value="true"),
            DeclareLaunchArgument(
                "scenario_file",
                default_value=PathJoinSubstitution(
                    [root_dir, "scenarios", "ego_slalom.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "pointlio_config",
                default_value=PathJoinSubstitution(
                    [
                        get_package_share_directory("point_lio"),
                        "config",
                        "mid360_mapping.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument("cloud_topic", default_value="/cloud_registered"),
            DeclareLaunchArgument(
                "planner_trajectory_topic", default_value="/ego/planning/bspline"
            ),
            DeclareLaunchArgument(
                "planner_trajectory_timeout_sec", default_value="2.0"
            ),
            DeclareLaunchArgument("validate_goal_occupancy", default_value="false"),
            DeclareLaunchArgument("require_occupancy_for_goal", default_value="false"),
            DeclareLaunchArgument("use_static_map", default_value="false"),
            DeclareLaunchArgument("static_cloud_topic", default_value="/map_cloud"),
            DeclareLaunchArgument("static_map_inflation", default_value="-1.0"),
            DeclareLaunchArgument("localization_health_topic", default_value=""),
            DeclareLaunchArgument("map_frame", default_value="world"),
            DeclareLaunchArgument("goal_x", default_value="5.0"),
            DeclareLaunchArgument("goal_y", default_value="0.0"),
            DeclareLaunchArgument("goal_z", default_value="1.5"),
            DeclareLaunchArgument("flight_type", default_value="2"),
            DeclareLaunchArgument("realworld_experiment", default_value="true"),
            DeclareLaunchArgument("max_velocity", default_value="0.5"),
            DeclareLaunchArgument("max_acceleration", default_value="0.8"),
            DeclareLaunchArgument("output_enabled", default_value="true"),
            DeclareLaunchArgument("auto_arm", default_value="false"),
            DeclareLaunchArgument("auto_offboard", default_value="false"),
            DeclareLaunchArgument(
                "control_mode",
                default_value="position",
                description=(
                    "PX4 input level: 'position' sends EGO position with velocity/acceleration "
                    "feed-forward; 'velocity' is an optional comparison mode"
                ),
            ),
            DeclareLaunchArgument("offboard_prestream_sec", default_value="2.0"),
            DeclareLaunchArgument("takeoff_before_ego", default_value="true"),
            DeclareLaunchArgument("takeoff_altitude", default_value="1.0"),
            DeclareLaunchArgument("takeoff_vertical_speed", default_value="0.30"),
            DeclareLaunchArgument("takeoff_reach_xy_tol", default_value="0.20"),
            DeclareLaunchArgument("takeoff_reach_z_tol", default_value="0.10"),
            DeclareLaunchArgument("takeoff_speed_xy_tol", default_value="0.20"),
            DeclareLaunchArgument("takeoff_speed_z_tol", default_value="0.15"),
            DeclareLaunchArgument("takeoff_stable_sec", default_value="0.8"),
            DeclareLaunchArgument("reference_capture_delay_sec", default_value="3.0"),
            DeclareLaunchArgument("use_initial_heading_frame", default_value="false"),
            DeclareLaunchArgument("lock_yaw_to_initial_heading", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config", default_value=f"{share_dir}/config/sim_ego.rviz"
            ),
            DeclareLaunchArgument(
                "waypoint_file",
                default_value=PathJoinSubstitution(
                    [root_dir, "scenarios", "interactive_waypoints.yaml"]
                ),
            ),
            pointlio_mapping,
            px4_odom_bridge,
            scenario_map,
            planner,
            trajectory_server,
            px4_bridge,
            health_monitor,
            waypoint_manager,
            path_visualizer,
            rviz,
        ]
    )
