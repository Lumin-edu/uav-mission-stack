"""
EGO 自主避障硬件启动文件。

启动的完整管线：
  1. Livox MID-360 激光雷达驱动
  2. Point-LIO 建图（里程计 + 点云）
  3. Point-LIO → PX4 视觉里程计桥接
  4. EGO 规划器 + 轨迹服务器
  5. ego_px4_bridge：EGO → PX4 Offboard 控制桥接
  6. startup_goal：启动时发布初始目标
  7. ego_hw_monitor：硬件管线诊断
  8. PX4 DDS 监控 + 控制看门狗 + 位置对比

完整数据流：
  Livox /livox/lidar + /livox/imu
    → Point-LIO /odom + /cloud_registered
    → (visual odom bridge) → PX4 EKF2
    → (EGO planner) → /ego/position_cmd
    → (ego_px4_bridge) → PX4 TrajectorySetpoint

坐标系约定：
  - Point-LIO/EGO 使用 ROS 标准系：x=前, y=左, z=上
  - PX4 NED：x=前(N), y=右(E), z=下(D)
  - 转换矩阵：world_to_px4_rotation = [1,0,0, 0,-1,0, 0,0,-1]
  - startup_goal 的用户参数仍为任务系 (x右,y前,z上)，内部自动转 ROS 系
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


def generate_launch_description():
    ego_hw_share = get_package_share_directory("hx_bringup_ego")
    pointlio_share = get_package_share_directory("point_lio")
    livox_share = get_package_share_directory("livox_ros_driver2")

    odom_topic = LaunchConfiguration("odom_topic")
    cloud_topic = LaunchConfiguration("cloud_topic")

    # ========== Node 1: Livox MID-360 激光雷达驱动 ==========
    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_share, "launch_ROS2", "msg_MID360_launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("use_livox_driver")),
    )

    # ========== Node 2: Point-LIO 建图（无头模式） ==========
    pointlio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pointlio_share, "launch", "mapping_headless.launch.py")
        ),
        launch_arguments={
            "point_lio_cfg_dir": LaunchConfiguration("pointlio_config"),
            "rviz": "false",
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_pointlio")),
    )

    # ========== Node 3: Point-LIO → PX4 视觉里程计桥接 ==========
    pointlio_to_px4 = Node(
        package="hx_bringup_pointlio_hover",
        executable="pointlio_to_px4_visual_odom.py",
        name="pointlio_to_px4_visual_odom",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_pointlio_px4_visual_odom")),
        parameters=[
            {
                "pointlio_topic": odom_topic,
                "px4_local_topic": "/fmu/out/vehicle_local_position",
                "timesync_topic": "/fmu/out/timesync_status",
                "output_topic": "/fmu/in/vehicle_visual_odometry",
                "publish_rate_limit": 50.0,
                "align_when_px4_valid": True,
                "use_px4_reference": False,
                "use_px4_heading_reference": True,
                "max_px4_reference_abs_z": 20.0,
                "max_output_abs_z": 20.0,
                "max_position_jump": 3.0,
                "publish_orientation": True,
                "publish_velocity": False,
                "use_timesync_timestamp": False,
                # Point-LIO /odom 使用 ROS 标准系：x前, y左, z上
                # 通过 pointlio_y_to_px4_y_sign=-1 将 y左 → y右(NED E轴)
                # 通过 z 方向取反将 z上 → z下(NED D轴)
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

    # ========== Node 4: EGO 规划器 ==========
    # 使用 EGO-Planner 进行局部轨迹规划与避障
    # 关键 remap：odom_world → Point-LIO /odom, grid_map/cloud → /cloud_registered
    planner = Node(
        package="ego_planner",
        executable="ego_planner_node",
        name="ego_planner",
        output="screen",
        parameters=[
            LaunchConfiguration("planner_config"),
            {
                "fsm/realworld_experiment": True,
                "fsm/flight_type": 1,
                # flight_type=1 时目标通过 /move_base_simple/goal 发布，不使用 waypoint0
                "fsm/manual_goal_z": LaunchConfiguration("goal_z"),
                "grid_map/frame_id": LaunchConfiguration("map_frame"),
                "manager/max_vel": LaunchConfiguration("max_velocity"),
                "manager/max_acc": LaunchConfiguration("max_acceleration"),
                "optimization/max_vel": LaunchConfiguration("max_velocity"),
                "optimization/max_acc": LaunchConfiguration("max_acceleration"),
                "bspline/limit_vel": LaunchConfiguration("max_velocity"),
                "bspline/limit_acc": LaunchConfiguration("max_acceleration"),
            },
        ],
        remappings=[
            ("odom_world", odom_topic),
            ("grid_map/odom", odom_topic),
            ("grid_map/cloud", cloud_topic),
            ("planning/bspline", "/ego/planning/bspline"),
            ("planning/data_display", "/ego/planning/data_display"),
            ("planning/broadcast_bspline_from_planner", "/ego/broadcast_bspline"),
            ("planning/broadcast_bspline_to_planner", "/ego/broadcast_bspline"),
            ("grid_map/occupancy_inflate", "/ego/grid_map/occupancy_inflate"),
        ],
    )

    # ========== Node 5: EGO 轨迹服务器 ==========
    # 将规划器输出的 B-spline 轨迹采样为 100Hz PositionCommand
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

    # ========== Node 6: EGO → PX4 桥接（核心） ==========
    # 将 EGO 的 PositionCommand 转换为 PX4 NED 的 TrajectorySetpoint
    # 管理完整的起飞流程：prestream → 解锁 → Offboard → 起飞 → EGO 控制
    px4_bridge = Node(
        package="hx_bringup_ego",
        executable="ego_px4_bridge.py",
        name="ego_px4_bridge",
        output="screen",
        parameters=[
            {
                "ego_command_topic": "/ego/position_cmd",
                "ego_odom_topic": odom_topic,
                "output_enabled": LaunchConfiguration("output_enabled"),
                "hardware_confirmation": LaunchConfiguration("hardware_confirmation"),
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
                "world_to_px4_rotation": [
                    1.0,  0.0,  0.0,   # PX4_N(前) = ROS_x(前)
                    0.0, -1.0,  0.0,   # PX4_E(右) = -ROS_y(左)
                    0.0,  0.0, -1.0,   # PX4_D(下) = -ROS_z(上)
                ],
                "yaw_sign": 1.0,
                "yaw_offset": 0.0,
                "use_initial_heading_frame": True,
                "lock_yaw_to_initial_heading": LaunchConfiguration(
                    "lock_yaw_to_initial_heading"
                ),
                "command_timeout_sec": 0.30,
                "odom_timeout_sec": 0.30,
                "control_rate_hz": 50.0,
                "velocity_position_gain": 1.0,
                "velocity_limit": LaunchConfiguration("max_velocity"),
            }
        ],
    )

    # ========== Node 7: 启动目标发布 ==========
    # 起飞完成后向 EGO 规划器发布初始目标航点，触发轨迹规划
    startup_goal = Node(
        package="hx_bringup_ego",
        executable="startup_goal.py",
        name="ego_hw_startup_goal",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_with_goal")),
        parameters=[
            {
                "goal_topic": "/move_base_simple/goal",
                "odom_topic": odom_topic,
                "frame_id": LaunchConfiguration("map_frame"),
                "goal_x": ParameterValue(LaunchConfiguration("goal_x"), value_type=float),
                "goal_y": ParameterValue(LaunchConfiguration("goal_y"), value_type=float),
                "goal_z": ParameterValue(LaunchConfiguration("goal_z"), value_type=float),
                "goal_yaw": ParameterValue(LaunchConfiguration("goal_yaw"), value_type=float),
                "startup_delay_sec": ParameterValue(
                    LaunchConfiguration("startup_goal_delay_sec"), value_type=float
                ),
                "wait_for_takeoff_ready": LaunchConfiguration("takeoff_before_ego"),
                "takeoff_ready_topic": "/ego/takeoff_ready",
            }
        ],
    )

    hardware_monitor = Node(
        package="hx_bringup_ego",
        executable="ego_hw_monitor.py",
        name="ego_hw_monitor",
        output="screen",
        parameters=[
            {
                "odom_topic": odom_topic,
                "cloud_topic": cloud_topic,
                "command_topic": "/ego/position_cmd",
                "max_odom_age_sec": 0.30,
                "max_cloud_age_sec": 0.50,
                "max_command_age_sec": 0.30,
                "max_px4_age_sec": 0.50,
            }
        ],
    )

    px4_dds_monitor = Node(
        package="hx_bringup_pointlio_hover",
        executable="px4_dds_monitor.py",
        name="ego_px4_dds_monitor",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_px4_monitor")),
    )

    px4_control_watchdog = Node(
        package="hx_bringup_pointlio_hover",
        executable="px4_control_watchdog.py",
        name="ego_px4_control_watchdog",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_px4_control_watchdog")),
        parameters=[{"control_source": "ego_px4_bridge"}],
    )

    px4_pointlio_position_compare = Node(
        package="hx_bringup_pointlio_hover",
        executable="px4_pointlio_position_compare.py",
        name="ego_px4_pointlio_position_compare",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_position_compare")),
        parameters=[
            {
                "px4_topic": "/fmu/out/vehicle_local_position",
                "pointlio_topic": odom_topic,
                "print_rate": 2.0,
            }
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(ego_hw_share, "config", "ego_hw.rviz")],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_livox_driver", default_value="true"),
            DeclareLaunchArgument("use_pointlio", default_value="true"),
            DeclareLaunchArgument("use_pointlio_px4_visual_odom", default_value="true"),
            DeclareLaunchArgument(
                "pointlio_config",
                default_value=os.path.join(pointlio_share, "config", "mid360_mapping.yaml"),
            ),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument("cloud_topic", default_value="/cloud_registered"),
            DeclareLaunchArgument("map_frame", default_value="odom"),
            DeclareLaunchArgument(
                "planner_config",
                default_value=os.path.join(ego_hw_share, "config", "ego_planner_hw.yaml"),
            ),
            DeclareLaunchArgument("goal_x", default_value="1.0"),
            DeclareLaunchArgument("goal_y", default_value="0.0"),
            DeclareLaunchArgument("goal_z", default_value="0.5"),
            DeclareLaunchArgument("goal_yaw", default_value="0.0"),
            DeclareLaunchArgument("start_with_goal", default_value="true"),
            DeclareLaunchArgument("startup_goal_delay_sec", default_value="3.0"),
            DeclareLaunchArgument("takeoff_before_ego", default_value="true"),
            DeclareLaunchArgument("takeoff_altitude", default_value="0.40"),
            DeclareLaunchArgument("takeoff_vertical_speed", default_value="0.20"),
            DeclareLaunchArgument("takeoff_reach_xy_tol", default_value="0.20"),
            DeclareLaunchArgument("takeoff_reach_z_tol", default_value="0.10"),
            DeclareLaunchArgument("takeoff_speed_xy_tol", default_value="0.20"),
            DeclareLaunchArgument("takeoff_speed_z_tol", default_value="0.15"),
            DeclareLaunchArgument("takeoff_stable_sec", default_value="0.8"),
            DeclareLaunchArgument("reference_capture_delay_sec", default_value="3.0"),
            DeclareLaunchArgument("lock_yaw_to_initial_heading", default_value="true"),
            DeclareLaunchArgument("max_velocity", default_value="0.3"),
            DeclareLaunchArgument("max_acceleration", default_value="0.5"),
            DeclareLaunchArgument(
                "control_mode",
                default_value="position",
                description=(
                    "Use 'position' for EGO position plus velocity/acceleration feed-forward; "
                    "'velocity' is retained only for controlled comparison tests"
                ),
            ),
            DeclareLaunchArgument("offboard_prestream_sec", default_value="2.0"),
            DeclareLaunchArgument("output_enabled", default_value="false"),
            DeclareLaunchArgument("hardware_confirmation", default_value=""),
            DeclareLaunchArgument("auto_arm", default_value="false"),
            DeclareLaunchArgument("auto_offboard", default_value="true"),
            DeclareLaunchArgument("use_px4_monitor", default_value="false"),
            DeclareLaunchArgument("use_px4_control_watchdog", default_value="true"),
            DeclareLaunchArgument("use_position_compare", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            livox_driver,
            pointlio,
            pointlio_to_px4,
            planner,
            trajectory_server,
            px4_bridge,
            startup_goal,
            hardware_monitor,
            px4_dds_monitor,
            px4_control_watchdog,
            px4_pointlio_position_compare,
            rviz,
        ]
    )
