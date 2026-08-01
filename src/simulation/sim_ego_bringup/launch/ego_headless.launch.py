from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory("sim_ego_bringup")
    default_config = f"{share_dir}/config/ego_planner.yaml"
    root_dir = EnvironmentVariable("SIM_EGO_ROOT", default_value="/home/wu/sim-ego")

    init_x = LaunchConfiguration("init_x")
    init_y = LaunchConfiguration("init_y")
    init_z = LaunchConfiguration("init_z")
    goal_x = LaunchConfiguration("goal_x")
    goal_y = LaunchConfiguration("goal_y")
    goal_z = LaunchConfiguration("goal_z")
    map_size_x = LaunchConfiguration("map_size_x")
    map_size_y = LaunchConfiguration("map_size_y")
    map_size_z = LaunchConfiguration("map_size_z")
    obstacle_count = LaunchConfiguration("obstacle_count")
    circle_count = LaunchConfiguration("circle_count")
    max_velocity = LaunchConfiguration("max_velocity")
    max_acceleration = LaunchConfiguration("max_acceleration")
    planner_config = LaunchConfiguration("planner_config")
    flight_type = LaunchConfiguration("flight_type")
    use_rviz = LaunchConfiguration("use_rviz")
    waypoint_file = LaunchConfiguration("waypoint_file")
    use_scenario_map = LaunchConfiguration("use_scenario_map")
    scenario_file = LaunchConfiguration("scenario_file")

    map_generator = Node(
        package="map_generator",
        executable="random_forest",
        name="random_forest",
        output="screen",
        condition=UnlessCondition(use_scenario_map),
        remappings=[("odometry", "/sim/odom")],
        parameters=[
            {
                "map/x_size": map_size_x,
                "map/y_size": map_size_y,
                "map/z_size": map_size_z,
                "map/resolution": 0.1,
                "ObstacleShape/seed": 1,
                "map/obs_num": obstacle_count,
                "ObstacleShape/lower_rad": 0.4,
                "ObstacleShape/upper_rad": 0.8,
                "ObstacleShape/lower_hei": 0.0,
                "ObstacleShape/upper_hei": 3.0,
                "map/circle_num": circle_count,
                "ObstacleShape/radius_l": 0.5,
                "ObstacleShape/radius_h": 0.8,
                "ObstacleShape/z_l": 0.8,
                "ObstacleShape/z_h": 1.2,
                "ObstacleShape/theta": 0.5,
                "pub_rate": 2.0,
                "min_distance": 1.5,
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
                "scenario_file": scenario_file,
                "map_topic": "/map_generator/global_cloud",
                "publish_rate_hz": 2.0,
            }
        ],
    )

    planner = Node(
        package="ego_planner",
        executable="ego_planner_node",
        name="ego_planner",
        output="screen",
        parameters=[
            planner_config,
            {
                "fsm/realworld_experiment": False,
                "fsm/flight_type": flight_type,
                "fsm/waypoint0_x": goal_x,
                "fsm/waypoint0_y": goal_y,
                "fsm/waypoint0_z": goal_z,
                "fsm/manual_goal_z": goal_z,
                "grid_map/map_size_x": map_size_x,
                "grid_map/map_size_y": map_size_y,
                "grid_map/map_size_z": map_size_z,
                "manager/max_vel": max_velocity,
                "manager/max_acc": max_acceleration,
                "optimization/max_vel": max_velocity,
                "optimization/max_acc": max_acceleration,
                "bspline/limit_vel": max_velocity,
                "bspline/limit_acc": max_acceleration,
            },
        ],
        remappings=[
            ("odom_world", "/sim/odom"),
            ("grid_map/odom", "/sim/odom"),
            ("grid_map/cloud", "/map_generator/global_cloud"),
            ("planning/bspline", "/ego/planning/bspline"),
            ("planning/data_display", "/ego/planning/data_display"),
            ("planning/broadcast_bspline_from_planner", "/ego/broadcast_bspline"),
            ("planning/broadcast_bspline_to_planner", "/ego/broadcast_bspline"),
            ("grid_map/occupancy_inflate", "/ego/grid_map/occupancy_inflate"),
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

    kinematic_model = Node(
        package="poscmd_2_odom",
        executable="poscmd_2_odom",
        name="ego_kinematic_uav",
        output="screen",
        parameters=[{"init_x": init_x, "init_y": init_y, "init_z": init_z}],
        remappings=[("command", "/ego/position_cmd"), ("odometry", "/sim/odom")],
    )

    health_monitor = Node(
        package="sim_ego_bringup",
        executable="sim_health_monitor.py",
        name="sim_health_monitor",
        output="screen",
        parameters=[
            {
                "mode": "headless",
                "odom_topic": "/sim/odom",
                "command_topic": "/ego/position_cmd",
                "cloud_topic": "/map_generator/global_cloud",
                "max_command_age_sec": 0.5,
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
                "waypoint_file": waypoint_file,
                "input_topic": "/move_base_simple/goal",
                "marker_topic": "/sim/waypoints",
                "path_topic": "/sim/waypoint_path",
                "append_mode": True,
                "default_altitude": goal_z,
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
                "odom_topic": "/sim/odom",
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
        arguments=["-d", f"{share_dir}/config/sim_ego.rviz"],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("init_x", default_value="0.0"),
            DeclareLaunchArgument("init_y", default_value="0.0"),
            DeclareLaunchArgument("init_z", default_value="1.0"),
            DeclareLaunchArgument("goal_x", default_value="8.0"),
            DeclareLaunchArgument("goal_y", default_value="0.0"),
            DeclareLaunchArgument("goal_z", default_value="1.5"),
            DeclareLaunchArgument("map_size_x", default_value="30.0"),
            DeclareLaunchArgument("map_size_y", default_value="30.0"),
            DeclareLaunchArgument("map_size_z", default_value="5.0"),
            DeclareLaunchArgument("obstacle_count", default_value="35"),
            DeclareLaunchArgument("circle_count", default_value="0"),
            DeclareLaunchArgument("max_velocity", default_value="1.5"),
            DeclareLaunchArgument("max_acceleration", default_value="2.0"),
            DeclareLaunchArgument("planner_config", default_value=default_config),
            DeclareLaunchArgument("flight_type", default_value="1"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_scenario_map", default_value="true"),
            DeclareLaunchArgument(
                "scenario_file",
                default_value=PathJoinSubstitution(
                    [root_dir, "scenarios", "ego_slalom.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "waypoint_file",
                default_value=PathJoinSubstitution(
                    [root_dir, "scenarios", "interactive_waypoints.yaml"]
                ),
            ),
            map_generator,
            scenario_map,
            kinematic_model,
            planner,
            trajectory_server,
            health_monitor,
            waypoint_manager,
            path_visualizer,
            rviz,
        ]
    )
