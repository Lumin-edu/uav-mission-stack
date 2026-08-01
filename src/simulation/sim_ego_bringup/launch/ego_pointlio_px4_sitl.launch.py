from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _validate_goal_sources(context):
    true_values = {"1", "true", "yes", "on"}
    single_goal = LaunchConfiguration("start_with_goal").perform(context).lower()
    waypoint_route = LaunchConfiguration("start_with_waypoints").perform(context).lower()
    if single_goal in true_values and waypoint_route in true_values:
        raise RuntimeError(
            "start_with_goal and start_with_waypoints cannot both be true; "
            "select one goal source"
        )
    return []


def generate_launch_description():
    sim_share = get_package_share_directory("sim_ego_bringup")
    pointlio_share = get_package_share_directory("point_lio")
    mid360_share = get_package_share_directory("sim_mid360_sensor")
    root_dir = EnvironmentVariable("SIM_EGO_ROOT", default_value="/home/wu/sim-ego")

    pointlio_config = LaunchConfiguration("pointlio_config")
    adapter_config = LaunchConfiguration("adapter_config")
    ego_px4_launch = f"{sim_share}/launch/ego_px4.launch.py"

    mid360_adapter = Node(
        package="sim_mid360_sensor",
        executable="mid360_cloud_to_livox",
        name="mid360_cloud_to_livox",
        output="screen",
        parameters=[adapter_config, {"use_sim_time": True}],
    )

    pointlio = Node(
        package="point_lio",
        executable="pointlio_mapping",
        name="pointlio_mapping",
        output="screen",
        parameters=[pointlio_config],
    )

    odom_guard = Node(
        package="sim_ego_bringup",
        executable="sim_odom_guard.py",
        name="sim_odom_guard",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "input_topic": "/sim/pointlio/odom",
                "output_topic": "/sim/pointlio/odom_safe",
                "health_topic": "/sim/pointlio/odom_healthy",
                # EGO is limited to 1 m/s in this launch. The extra margin
                # tolerates estimator noise while rejecting Point-LIO drift.
                "max_speed_mps": 2.5,
                "max_jump_m": 1.0,
                "max_validation_dt_sec": 0.25,
                "input_timeout_sec": 0.5,
                "max_consecutive_rejections": 3,
                "max_horizontal_distance_m": 12.0,
                "max_vertical_distance_m": 3.0,
            }
        ],
    )

    planner_altitude_fusion = Node(
        package="sim_ego_bringup",
        executable="planner_altitude_fusion.py",
        name="planner_altitude_fusion",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "raw_odom_topic": "/sim/pointlio/odom_safe",
                "raw_cloud_topic": "/sim/pointlio/cloud_registered",
                "px4_position_topic": "/fmu/out/vehicle_local_position",
                "source_health_topic": "/sim/pointlio/odom_healthy",
                "fused_odom_topic": "/sim/ego/odom_fused",
                "fused_cloud_topic": "/sim/ego/cloud_registered_fused",
                "fused_map_topic": "/sim/ego/map_cloud_fused",
                "health_topic": "/sim/ego/height_fusion_healthy",
                "correction_topic": "/sim/ego/pointlio_z_correction",
                "max_px4_age_sec": 0.30,
                "max_odom_age_sec": 0.30,
                "max_correction_age_sec": 0.30,
                "correction_history_sec": 3.0,
                "cloud_sync_tolerance_sec": LaunchConfiguration(
                    "cloud_sync_tolerance_sec"
                ),
                "max_abs_correction_m": 3.0,
                "map_voxel_size": LaunchConfiguration("persistent_map_voxel_size"),
                "map_radius_xy": LaunchConfiguration("persistent_map_radius_xy"),
                "map_min_z": LaunchConfiguration("persistent_map_min_z"),
                "map_max_z": LaunchConfiguration("persistent_map_max_z"),
                "map_publish_rate_hz": LaunchConfiguration(
                    "persistent_map_publish_rate_hz"
                ),
                "map_voxel_ttl_sec": LaunchConfiguration(
                    "persistent_map_voxel_ttl_sec"
                ),
                "map_max_voxels": LaunchConfiguration("persistent_map_max_voxels"),
                "map_frame_id": "world",
                "require_vertical_velocity": True,
                "print_rate_hz": 1.0,
            }
        ],
    )

    pointlio_to_px4 = Node(
        package="sim_ego_bringup",
        executable="pointlio_to_px4_visual_odom.py",
        name="pointlio_to_px4_visual_odom",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "pointlio_topic": "/sim/pointlio/odom_safe",
                "px4_local_topic": "/fmu/out/vehicle_local_position",
                "timesync_topic": "/fmu/out/timesync_status",
                "output_topic": "/fmu/in/vehicle_visual_odometry",
                "publish_rate_limit": 50.0,
                "align_when_px4_valid": True,
                # Keep the Point-LIO origin aligned with the current PX4 local
                # position. This prevents a Point-LIO restart from injecting a
                # false (0, 0, 0) external-vision jump into an already-flying
                # vehicle; on a cold start PX4's valid local origin is zero.
                "use_px4_reference": True,
                "px4_reference_wait_sec": 3.0,
                # PX4's initial heading supplies the ROS-FLU -> NED yaw alignment.
                "use_px4_heading_reference": True,
                "max_px4_reference_abs_z": 20.0,
                "max_output_abs_z": 3.0,
                "max_output_abs_xy": 12.0,
                "max_position_rate_mps": 2.5,
                "max_position_jump": 1.0,
                # Match the verified hardware visual-odometry contract. EGO
                # still holds the captured initial heading at the setpoint layer.
                "publish_orientation": True,
                "publish_velocity": False,
                "use_timesync_timestamp": False,
                # Match the verified hardware adapter exactly:
                # Point-LIO ROS FLU (forward/left/up) -> PX4 NED.
                "pointlio_to_px4_rotation": [1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0],
                "pointlio_y_to_px4_y_sign": -1.0,
                # Point-LIO publishes the installed body attitude.  Keep the
                # same verified body-yaw relation as hardware; this is not the
                # EGO trajectory-yaw convention used by ego_px4_bridge.
                "yaw_sign": -1.0,
                "yaw_offset": 0.0,
                "position_variance": 0.04,
                "orientation_variance": 0.01,
                "velocity_variance": 0.25,
                "print_rate": 1.0,
            }
        ],
    )

    startup_goal = Node(
        package="sim_ego_bringup",
        executable="startup_goal.py",
        name="startup_goal",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_with_goal")),
        parameters=[
            {
                "goal_topic": "/move_base_simple/goal",
                "odom_topic": "/sim/ego/odom_fused",
                "frame_id": "world",
                "goal_x": ParameterValue(LaunchConfiguration("goal_x"), value_type=float),
                "goal_y": ParameterValue(LaunchConfiguration("goal_y"), value_type=float),
                "goal_z": ParameterValue(LaunchConfiguration("goal_z"), value_type=float),
                "goal_yaw": ParameterValue(LaunchConfiguration("goal_yaw"), value_type=float),
                "startup_delay_sec": ParameterValue(
                    LaunchConfiguration("startup_goal_delay_sec"), value_type=float
                ),
                "wait_for_takeoff_ready": LaunchConfiguration("takeoff_before_ego"),
                "takeoff_ready_topic": "/ego/takeoff_ready",
                "wait_for_occupancy": LaunchConfiguration("use_static_map"),
                "occupancy_topic": "/ego/grid_map/static_occupancy_inflate",
            }
        ],
    )

    waypoint_route = Node(
        package="sim_ego_bringup",
        executable="send_waypoints.py",
        name="waypoint_route_runner",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_with_waypoints")),
        arguments=[
            LaunchConfiguration("route_file"),
            "--odom-topic",
            "/sim/ego/odom_fused",
            "--takeoff-ready-topic",
            "/ego/takeoff_ready",
            "--occupancy-topic",
            "/ego/grid_map/static_occupancy_inflate",
            PythonExpression(
                [
                    "'--wait-occupancy' if '",
                    LaunchConfiguration("use_static_map"),
                    "'.lower() in ('1', 'true', 'yes', 'on') ",
                    "else '--no-wait-occupancy'",
                ]
            ),
            "--ready-timeout",
            LaunchConfiguration("route_ready_timeout_sec"),
            "--odom-timeout",
            LaunchConfiguration("route_odom_timeout_sec"),
            "--reach-xy",
            LaunchConfiguration("route_reach_xy"),
            "--reach-z",
            LaunchConfiguration("route_reach_z"),
            "--max-speed-xy",
            LaunchConfiguration("route_max_speed_xy"),
            "--max-speed-z",
            LaunchConfiguration("route_max_speed_z"),
            "--settle",
            LaunchConfiguration("route_settle_sec"),
            "--timeout",
            LaunchConfiguration("route_waypoint_timeout_sec"),
        ],
    )

    ego_px4 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ego_px4_launch),
        launch_arguments={
            "planner_config": LaunchConfiguration("planner_config"),
            "use_pointlio": "false",
            "use_px4_odom_bridge": "false",
            "use_scenario_map": "false",
            "odom_topic": "/sim/ego/odom_fused",
            "localization_health_topic": "/sim/ego/height_fusion_healthy",
            "cloud_topic": "/sim/ego/cloud_registered_fused",
            "validate_goal_occupancy": "true",
            "require_occupancy_for_goal": LaunchConfiguration("use_static_map"),
            "use_static_map": LaunchConfiguration("use_static_map"),
            "static_cloud_topic": "/sim/ego/map_cloud_fused",
            "static_map_inflation": LaunchConfiguration("static_map_inflation"),
            "map_frame": "world",
            "goal_x": LaunchConfiguration("goal_x"),
            "goal_y": LaunchConfiguration("goal_y"),
            "goal_z": LaunchConfiguration("goal_z"),
            # Manual-target mode receives RViz 2D Goal Pose and the YAML
            # waypoint runner directly. Preset-target mode blocks on an RC
            # trigger topic and is not suitable for interactive EGO validation.
            "flight_type": "1",
            "realworld_experiment": "false",
            "max_velocity": LaunchConfiguration("max_velocity"),
            "max_acceleration": LaunchConfiguration("max_acceleration"),
            "output_enabled": "true",
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
            "use_initial_heading_frame": "true",
            "lock_yaw_to_initial_heading": LaunchConfiguration(
                "lock_yaw_to_initial_heading"
            ),
            "planner_trajectory_timeout_sec": LaunchConfiguration(
                "planner_trajectory_timeout_sec"
            ),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "rviz_config": f"{sim_share}/config/sim_ego_pointlio.rviz",
            "waypoint_file": LaunchConfiguration("waypoint_file"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "pointlio_config",
                default_value=f"{sim_share}/config/pointlio_mid360_sitl.yaml",
            ),
            DeclareLaunchArgument(
                "planner_config",
                default_value=f"{sim_share}/config/ego_planner.yaml",
            ),
            DeclareLaunchArgument(
                "adapter_config",
                default_value=f"{mid360_share}/config/mid360_adapter.yaml",
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_static_map", default_value="true"),
            DeclareLaunchArgument("static_map_inflation", default_value="0.40"),
            DeclareLaunchArgument("goal_x", default_value="6.0"),
            DeclareLaunchArgument("goal_y", default_value="0.0"),
            DeclareLaunchArgument("goal_z", default_value="1.5"),
            DeclareLaunchArgument("goal_yaw", default_value="0.0"),
            DeclareLaunchArgument("start_with_goal", default_value="true"),
            DeclareLaunchArgument("start_with_waypoints", default_value="false"),
            DeclareLaunchArgument("startup_goal_delay_sec", default_value="1.0"),
            DeclareLaunchArgument(
                "route_file",
                default_value=PathJoinSubstitution(
                    [root_dir, "scenarios", "pointlio_multi_waypoints.yaml"]
                ),
            ),
            DeclareLaunchArgument("route_ready_timeout_sec", default_value="120.0"),
            DeclareLaunchArgument("route_odom_timeout_sec", default_value="0.50"),
            DeclareLaunchArgument("route_reach_xy", default_value="0.30"),
            DeclareLaunchArgument("route_reach_z", default_value="0.20"),
            DeclareLaunchArgument("route_max_speed_xy", default_value="0.20"),
            DeclareLaunchArgument("route_max_speed_z", default_value="0.15"),
            DeclareLaunchArgument("route_settle_sec", default_value="1.0"),
            DeclareLaunchArgument("route_waypoint_timeout_sec", default_value="90.0"),
            DeclareLaunchArgument("cloud_sync_tolerance_sec", default_value="0.15"),
            DeclareLaunchArgument("persistent_map_voxel_size", default_value="0.20"),
            DeclareLaunchArgument("persistent_map_radius_xy", default_value="10.0"),
            DeclareLaunchArgument("persistent_map_min_z", default_value="-0.5"),
            DeclareLaunchArgument("persistent_map_max_z", default_value="3.5"),
            DeclareLaunchArgument(
                "persistent_map_publish_rate_hz", default_value="1.0"
            ),
            DeclareLaunchArgument("persistent_map_voxel_ttl_sec", default_value="0.0"),
            DeclareLaunchArgument("persistent_map_max_voxels", default_value="150000"),
            DeclareLaunchArgument("planner_trajectory_timeout_sec", default_value="2.0"),
            DeclareLaunchArgument("max_velocity", default_value="0.5"),
            DeclareLaunchArgument("max_acceleration", default_value="0.8"),
            DeclareLaunchArgument("auto_arm", default_value="true"),
            DeclareLaunchArgument("auto_offboard", default_value="true"),
            DeclareLaunchArgument("control_mode", default_value="position"),
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
            DeclareLaunchArgument("lock_yaw_to_initial_heading", default_value="true"),
            DeclareLaunchArgument(
                "waypoint_file",
                default_value=PathJoinSubstitution(
                    [root_dir, "scenarios", "interactive_waypoints.yaml"]
                ),
            ),
            OpaqueFunction(function=_validate_goal_sources),
            mid360_adapter,
            pointlio,
            odom_guard,
            planner_altitude_fusion,
            pointlio_to_px4,
            ego_px4,
            startup_goal,
            waypoint_route,
        ]
    )
