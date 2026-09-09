#!/usr/bin/env python3
"""Independent hardware EGO + linear-MPC bringup.

The localization and planning path is the verified Point-LIO/EGO path.  Only
the final EGO-to-PX4 outer-loop node differs: ``ego_mpc_controller`` publishes
acceleration setpoints, while PX4 retains attitude, angular-rate, and motor
control.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("bringup_ego_mpc")
    pointlio_share = get_package_share_directory("point_lio")
    livox_share = get_package_share_directory("livox_ros_driver2")

    raw_odom = LaunchConfiguration("odom_topic")
    raw_cloud = LaunchConfiguration("cloud_topic")
    base_odom = LaunchConfiguration("base_odom_topic")

    # Point-LIO base_link is MID360 IMU; base is the aircraft center. The
    # same static transform is used by the center-odometry adapter,
    # visual odometry, and TF.
    base_link_to_base_translation = [-0.011, -0.02329, -0.05588]
    base_link_to_base_rotation_xyzw = [0.0, 0.0, 0.0, 1.0]

    livox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(livox_share, "launch_ROS2", "msg_MID360_launch.py")),
        condition=IfCondition(LaunchConfiguration("use_livox_driver")),
    )
    pointlio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pointlio_share, "launch", "mapping_headless.launch.py")),
        launch_arguments={
            "point_lio_cfg_dir": LaunchConfiguration("pointlio_config"),
            "rviz": "false",
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_pointlio")),
    )

    base_odom_transform = Node(
        package="bringup_ego_mpc",
        executable="rigid_odom_transform.py",
        name="pointlio_base_odom_transform",
        output="screen",
        parameters=[
            {
                "source_topic": raw_odom,
                "target_topic": base_odom,
                "target_child_frame": "base",
                "base_link_to_base_translation": base_link_to_base_translation,
                "base_link_to_base_rotation_xyzw": base_link_to_base_rotation_xyzw,
            }
        ],
    )

    visual_odom = Node(
        package="bringup_ego_mpc",
        executable="pointlio_to_px4_visual_odom.py",
        name="pointlio_to_px4_visual_odom",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_pointlio_px4_visual_odom")),
        parameters=[
            {
                "pointlio_topic": raw_odom,
                "px4_local_topic": "/fmu/out/vehicle_local_position",
                "timesync_topic": "/fmu/out/timesync_status",
                "output_topic": "/fmu/in/vehicle_visual_odometry",
                "pointlio_pose_frame": "base_link",
                "vehicle_frame": "base",
                "base_link_to_base_translation": base_link_to_base_translation,
                "base_link_to_base_rotation_xyzw": base_link_to_base_rotation_xyzw,
                "publish_rate_limit": 50.0,
                "align_when_px4_valid": True,
                "use_px4_reference": False,
                "use_px4_heading_reference": True,
                "publish_orientation": True,
                "publish_velocity": False,
                "use_timesync_timestamp": False,
                "pointlio_y_to_px4_y_sign": -1.0,
                "yaw_sign": -1.0,
                "yaw_offset": 0.0,
                "position_variance": 0.04,
                "orientation_variance": 0.01,
                "velocity_variance": 0.25,
                "print_rate": 1.0,
            }
        ],
    )

    base_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_base_tf",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_base_link_to_base_tf")),
        arguments=[
            "--x", str(base_link_to_base_translation[0]), "--y", str(base_link_to_base_translation[1]), "--z", str(base_link_to_base_translation[2]),
            "--qx", str(base_link_to_base_rotation_xyzw[0]), "--qy", str(base_link_to_base_rotation_xyzw[1]),
            "--qz", str(base_link_to_base_rotation_xyzw[2]), "--qw", str(base_link_to_base_rotation_xyzw[3]),
            "--frame-id", "base_link", "--child-frame-id", "base",
        ],
    )

    planner = Node(
        package="ego_planner",
        executable="ego_planner_node",
        name="ego_planner",
        output="screen",
        parameters=[
            LaunchConfiguration("planner_config"),
            LaunchConfiguration("mpc_config"),
            {
                "fsm/realworld_experiment": True,
                "fsm/flight_type": 1,
                "fsm/manual_goal_z": LaunchConfiguration("goal_z"),
                "grid_map/frame_id": LaunchConfiguration("map_frame"),
            },
        ],
        remappings=[
            ("odom_world", base_odom),
            ("grid_map/odom", base_odom),
            ("grid_map/cloud", raw_cloud),
            ("planning/bspline", "/ego/planning/bspline"),
            ("planning/data_display", "/ego/planning/data_display"),
            ("planning/broadcast_bspline_from_planner", "/ego/broadcast_bspline"),
            ("planning/broadcast_bspline_to_planner", "/ego/broadcast_bspline"),
            ("grid_map/occupancy_inflate", "/ego/grid_map/occupancy_inflate"),
        ],
    )

    # Retained for visualization and compatibility.  MPC consumes the B-spline
    # directly, so this node is not on the PX4 control path.
    trajectory_server = Node(
        package="ego_planner",
        executable="traj_server",
        name="ego_trajectory_server",
        output="screen",
        parameters=[{"traj_server/time_forward": 1.0}],
        remappings=[("planning/bspline", "/ego/planning/bspline"), ("/position_cmd", "/ego/position_cmd")],
    )

    mpc = Node(
        package="bringup_ego_mpc",
        executable="ego_mpc_controller.py",
        name="ego_mpc_controller",
        output="screen",
        parameters=[
            LaunchConfiguration("mpc_config"),
            {
                "ego_odom_topic": base_odom,
                "bspline_topic": "/ego/planning/bspline",
                "output_enabled": LaunchConfiguration("output_enabled"),
                "hardware_confirmation": LaunchConfiguration("hardware_confirmation"),
                "auto_arm": LaunchConfiguration("auto_arm"),
                "auto_offboard": LaunchConfiguration("auto_offboard"),
                "offboard_prestream_sec": LaunchConfiguration("offboard_prestream_sec"),
                "takeoff_before_ego": LaunchConfiguration("takeoff_before_ego"),
                "takeoff_altitude": LaunchConfiguration("takeoff_altitude"),
                "takeoff_vertical_speed": LaunchConfiguration("takeoff_vertical_speed"),
                "takeoff_reach_xy_tol": LaunchConfiguration("takeoff_reach_xy_tol"),
                "takeoff_reach_z_tol": LaunchConfiguration("takeoff_reach_z_tol"),
                "takeoff_speed_xy_tol": LaunchConfiguration("takeoff_speed_xy_tol"),
                "takeoff_speed_z_tol": LaunchConfiguration("takeoff_speed_z_tol"),
                "takeoff_stable_sec": LaunchConfiguration("takeoff_stable_sec"),
                "reference_capture_delay_sec": LaunchConfiguration("reference_capture_delay_sec"),
                "trajectory_timeout_sec": LaunchConfiguration("mpc_trajectory_timeout_sec"),
                "px4_timeout_sec": LaunchConfiguration("mpc_px4_timeout_sec"),
                "reset_recovery_sec": LaunchConfiguration("mpc_reset_recovery_sec"),
                "reject_dead_reckoning": LaunchConfiguration("mpc_reject_dead_reckoning"),
                "max_eph": LaunchConfiguration("mpc_max_eph"),
                "max_epv": LaunchConfiguration("mpc_max_epv"),
                "world_to_px4_rotation": [1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0],
                "use_initial_heading_frame": True,
                "lock_yaw_to_initial_heading": LaunchConfiguration("lock_yaw_to_initial_heading"),
            }
        ],
    )

    startup_goal = Node(
        package="bringup_ego_mpc",
        executable="startup_goal.py",
        name="ego_mpc_startup_goal",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_with_goal")),
        parameters=[
            {
                "goal_topic": "/move_base_simple/goal",
                "odom_topic": base_odom,
                "frame_id": LaunchConfiguration("map_frame"),
                "goal_x": ParameterValue(LaunchConfiguration("goal_x"), value_type=float),
                "goal_y": ParameterValue(LaunchConfiguration("goal_y"), value_type=float),
                "goal_z": ParameterValue(LaunchConfiguration("goal_z"), value_type=float),
                "goal_yaw": ParameterValue(LaunchConfiguration("goal_yaw"), value_type=float),
                "startup_delay_sec": ParameterValue(LaunchConfiguration("startup_goal_delay_sec"), value_type=float),
                "wait_for_takeoff_ready": LaunchConfiguration("takeoff_before_ego"),
                "takeoff_ready_topic": "/ego/takeoff_ready",
            }
        ],
    )

    monitor = Node(
        package="bringup_ego_mpc",
        executable="ego_hw_monitor.py",
        name="ego_mpc_hw_monitor",
        output="screen",
        parameters=[
            {
                "odom_topic": base_odom,
                "cloud_topic": raw_cloud,
                "command_topic": "/ego/position_cmd",
                "bspline_topic": "/ego/planning/bspline",
                "max_odom_age_sec": 0.30,
                "max_cloud_age_sec": 0.50,
                "max_command_age_sec": 0.30,
                "max_bspline_age_sec": LaunchConfiguration("mpc_trajectory_timeout_sec"),
                "max_px4_age_sec": 0.50,
                "max_control_age_sec": 0.30,
            }
        ],
    )

    px4_monitor = Node(
        package="bringup_ego_mpc", executable="px4_dds_monitor.py", name="ego_mpc_px4_dds_monitor",
        output="screen", condition=IfCondition(LaunchConfiguration("use_px4_monitor")),
    )
    watchdog = Node(
        package="bringup_ego_mpc", executable="px4_control_watchdog.py", name="ego_mpc_control_watchdog",
        output="screen", condition=IfCondition(LaunchConfiguration("use_px4_control_watchdog")),
        parameters=[{"control_source": "ego_mpc_controller"}],
    )
    compare = Node(
        package="bringup_ego_mpc", executable="px4_pointlio_position_compare.py", name="ego_mpc_position_compare",
        output="screen", condition=IfCondition(LaunchConfiguration("use_position_compare")),
        parameters=[{"px4_topic": "/fmu/out/vehicle_local_position", "pointlio_topic": raw_odom, "print_rate": 2.0}],
    )
    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="screen",
        arguments=["-d", os.path.join(package_share, "config", "ego_hw.rviz")],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_livox_driver", default_value="false"),
        DeclareLaunchArgument("use_pointlio", default_value="true"),
        DeclareLaunchArgument("use_pointlio_px4_visual_odom", default_value="true"),
        DeclareLaunchArgument("pointlio_config", default_value=os.path.join(pointlio_share, "config", "mid360_mapping.yaml")),
        DeclareLaunchArgument("odom_topic", default_value="/odom"),
        DeclareLaunchArgument("base_odom_topic", default_value="/ego/odom_base"),
        DeclareLaunchArgument("cloud_topic", default_value="/cloud_registered"),
        DeclareLaunchArgument("map_frame", default_value="odom"),
        DeclareLaunchArgument("use_base_link_to_base_tf", default_value="true"),
        DeclareLaunchArgument("planner_config", default_value=os.path.join(package_share, "config", "ego_planner_hw.yaml")),
        DeclareLaunchArgument("mpc_config", default_value=os.path.join(package_share, "config", "mpc_params.yaml")),
        DeclareLaunchArgument("goal_x", default_value="2.0"),
        DeclareLaunchArgument("goal_y", default_value="2.0"),
        DeclareLaunchArgument("goal_z", default_value="0.7"),
        DeclareLaunchArgument("goal_yaw", default_value="0.0"),
        DeclareLaunchArgument("start_with_goal", default_value="false"),
        DeclareLaunchArgument("startup_goal_delay_sec", default_value="3.0"),
        DeclareLaunchArgument("takeoff_before_ego", default_value="true"),
        DeclareLaunchArgument("takeoff_altitude", default_value="0.30"),
        DeclareLaunchArgument("takeoff_vertical_speed", default_value="0.20"),
        DeclareLaunchArgument("takeoff_reach_xy_tol", default_value="0.20"),
        DeclareLaunchArgument("takeoff_reach_z_tol", default_value="0.10"),
        DeclareLaunchArgument("takeoff_speed_xy_tol", default_value="0.20"),
        DeclareLaunchArgument("takeoff_speed_z_tol", default_value="0.15"),
        DeclareLaunchArgument("takeoff_stable_sec", default_value="0.8"),
        DeclareLaunchArgument("reference_capture_delay_sec", default_value="3.0"),
        DeclareLaunchArgument("mpc_trajectory_timeout_sec", default_value="2.0"),
        DeclareLaunchArgument("mpc_px4_timeout_sec", default_value="0.20"),
        DeclareLaunchArgument("mpc_reset_recovery_sec", default_value="0.20"),
        DeclareLaunchArgument("mpc_reject_dead_reckoning", default_value="true"),
        DeclareLaunchArgument("mpc_max_eph", default_value="2.0"),
        DeclareLaunchArgument("mpc_max_epv", default_value="2.0"),
        DeclareLaunchArgument("lock_yaw_to_initial_heading", default_value="true"),
        DeclareLaunchArgument("offboard_prestream_sec", default_value="2.0"),
        DeclareLaunchArgument("output_enabled", default_value="false"),
        DeclareLaunchArgument("hardware_confirmation", default_value=""),
        DeclareLaunchArgument("auto_arm", default_value="false"),
        DeclareLaunchArgument("auto_offboard", default_value="false"),
        DeclareLaunchArgument("use_px4_monitor", default_value="false"),
        DeclareLaunchArgument("use_px4_control_watchdog", default_value="true"),
        DeclareLaunchArgument("use_position_compare", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        livox, pointlio, base_odom_transform, visual_odom, base_tf, planner,
        trajectory_server, mpc, startup_goal, monitor, px4_monitor, watchdog,
        compare, rviz,
    ])
