"""
正方形航点任务硬件启动文件。

启动的节点和功能：
  1. Point-LIO 建图（可选，use_pointlio）
  2. Point-LIO -> PX4 EKF2 视觉里程计桥接（可选，use_pointlio_px4_visual_odom）
  3. 正方形航点任务控制器（核心）
  4. PX4 DDS 链路监控（可选，use_px4_monitor）
  5. PX4 Offboard 控制看门狗（可选，use_px4_control_watchdog）
  6. PX4/Point-LIO 位置对比调试（可选，use_position_compare）

数据流：
  Point-LIO /odom
    -> pointlio_to_px4_visual_odom
    -> /fmu/in/vehicle_visual_odometry
    -> PX4 EKF2
    -> /fmu/out/vehicle_local_position
    -> square_mission_controller
    -> /fmu/in/offboard_control_mode + trajectory_setpoint + vehicle_command
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ========== 功能开关 ==========
    use_pointlio = LaunchConfiguration("use_pointlio")
    use_pointlio_px4_visual_odom = LaunchConfiguration("use_pointlio_px4_visual_odom")
    use_px4_monitor = LaunchConfiguration("use_px4_monitor")
    use_px4_control_watchdog = LaunchConfiguration("use_px4_control_watchdog")
    use_position_compare = LaunchConfiguration("use_position_compare")
    use_base_link_to_base_tf = LaunchConfiguration("use_base_link_to_base_tf")
    pointlio_config_file = LaunchConfiguration("pointlio_config_file")

    # Point-LIO base_link 对应 IMU 原点，base 对应补偿后的机体中心。
    # 数值桥接和静态 TF 共用同一对象，避免外参配置不一致。
    base_link_to_base_translation = [-0.011, -0.02329, -0.05588]
    base_link_to_base_rotation_xyzw = [0.0, 0.0, 0.0, 1.0]

    # ========== 任务控制参数 ==========
    auto_arm = LaunchConfiguration("auto_arm")
    control_rate_hz = LaunchConfiguration("control_rate_hz")
    reference_capture_delay_sec = LaunchConfiguration("reference_capture_delay_sec")
    takeoff_altitude = LaunchConfiguration("takeoff_altitude")
    square_side_length = LaunchConfiguration("square_side_length")
    approach_speed = LaunchConfiguration("approach_speed")
    vertical_speed = LaunchConfiguration("vertical_speed")
    reach_xy_tol = LaunchConfiguration("reach_xy_tol")
    reach_z_tol = LaunchConfiguration("reach_z_tol")
    speed_xy_tol = LaunchConfiguration("speed_xy_tol")
    speed_z_tol = LaunchConfiguration("speed_z_tol")
    stable_time_sec = LaunchConfiguration("stable_time_sec")
    takeoff_hover_sec = LaunchConfiguration("takeoff_hover_sec")
    corner_hover_sec = LaunchConfiguration("corner_hover_sec")
    final_hover_sec = LaunchConfiguration("final_hover_sec")
    use_initial_heading_frame = LaunchConfiguration("use_initial_heading_frame")
    task_x_sign = LaunchConfiguration("task_x_sign")
    task_y_sign = LaunchConfiguration("task_y_sign")
    task_z_sign = LaunchConfiguration("task_z_sign")
    visual_odom_print_rate = LaunchConfiguration("visual_odom_print_rate")

    pointlio_share = get_package_share_directory("point_lio")
    pointlio_launch = os.path.join(pointlio_share, "launch", "mapping_headless.launch.py")
    default_pointlio_config = os.path.join(pointlio_share, "config", "mid360_mapping.yaml")

    # ========== Node 1: Point-LIO 建图 ==========
    pointlio_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pointlio_launch),
        launch_arguments={
            "point_lio_cfg_dir": pointlio_config_file,
            "rviz": "false",
        }.items(),
        condition=IfCondition(use_pointlio),
    )

    # ========== Node 2: Point-LIO -> PX4 视觉里程计桥接 ==========
    pointlio_to_px4_visual_odom = Node(
        package="bringup_square_mission",
        executable="pointlio_to_px4_visual_odom.py",
        output="screen",
        condition=IfCondition(use_pointlio_px4_visual_odom),
        parameters=[
            {
                "pointlio_topic": "/odom",
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

    # Point-LIO 动态发布 odom -> base_link；这里补全固定的 base_link -> base。
    # 桥接节点不读取 TF，因此不会改变或重复执行原有的 base -> PX4 NED 转换。
    base_link_to_base_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_base_tf",
        output="screen",
        condition=IfCondition(use_base_link_to_base_tf),
        arguments=[
            "--x", str(base_link_to_base_translation[0]),
            "--y", str(base_link_to_base_translation[1]),
            "--z", str(base_link_to_base_translation[2]),
            "--qx", str(base_link_to_base_rotation_xyzw[0]),
            "--qy", str(base_link_to_base_rotation_xyzw[1]),
            "--qz", str(base_link_to_base_rotation_xyzw[2]),
            "--qw", str(base_link_to_base_rotation_xyzw[3]),
            "--frame-id", "base_link",
            "--child-frame-id", "base",
        ],
    )

    # ========== Node 3: 正方形任务控制器（核心） ==========
    square_mission_controller = Node(
        package="bringup_square_mission",
        executable="square_mission_controller.py",
        output="screen",
        parameters=[
            {
                "auto_arm": auto_arm,
                "control_rate_hz": control_rate_hz,
                "reference_capture_delay_sec": reference_capture_delay_sec,
                "takeoff_altitude": takeoff_altitude,
                "square_side_length": square_side_length,
                "approach_speed": approach_speed,
                "vertical_speed": vertical_speed,
                "reach_xy_tol": reach_xy_tol,
                "reach_z_tol": reach_z_tol,
                "speed_xy_tol": speed_xy_tol,
                "speed_z_tol": speed_z_tol,
                "stable_time_sec": stable_time_sec,
                "takeoff_hover_sec": takeoff_hover_sec,
                "corner_hover_sec": corner_hover_sec,
                "final_hover_sec": final_hover_sec,
                "use_initial_heading_frame": use_initial_heading_frame,
                "task_x_sign": task_x_sign,
                "task_y_sign": task_y_sign,
                "task_z_sign": task_z_sign,
                "vehicle_local_position_topic": "/fmu/out/vehicle_local_position",
                "vehicle_status_topic": "/fmu/out/vehicle_status",
            }
        ],
    )

    # ========== Node 4: PX4 DDS 链路监控 ==========
    px4_dds_monitor = Node(
        package="bringup_square_mission",
        executable="px4_dds_monitor.py",
        output="screen",
        condition=IfCondition(use_px4_monitor),
    )

    # ========== Node 5: PX4 Offboard 控制看门狗 ==========
    px4_control_watchdog = Node(
        package="bringup_square_mission",
        executable="px4_control_watchdog.py",
        output="screen",
        condition=IfCondition(use_px4_control_watchdog),
        parameters=[{"control_source": "square_mission_controller"}],
    )

    # ========== Node 6: PX4/Point-LIO 位置对比调试 ==========
    px4_pointlio_position_compare = Node(
        package="bringup_square_mission",
        executable="px4_pointlio_position_compare.py",
        output="screen",
        condition=IfCondition(use_position_compare),
        parameters=[
            {
                "px4_topic": "/fmu/out/vehicle_local_position",
                "visual_odom_topic": "/fmu/in/vehicle_visual_odometry",
                "print_rate": 2.0,
            }
        ],
    )

    return LaunchDescription(
        [
            # ========== 功能开关 Launch Arguments ==========
            DeclareLaunchArgument("use_pointlio", default_value="true"),
            DeclareLaunchArgument("use_pointlio_px4_visual_odom", default_value="true"),
            DeclareLaunchArgument("use_px4_monitor", default_value="false"),
            DeclareLaunchArgument("use_px4_control_watchdog", default_value="true"),
            DeclareLaunchArgument("use_position_compare", default_value="false"),
            DeclareLaunchArgument("use_base_link_to_base_tf", default_value="true"),
            DeclareLaunchArgument("pointlio_config_file", default_value=default_pointlio_config),
            # ========== 任务控制参数 Launch Arguments ==========
            DeclareLaunchArgument("auto_arm", default_value="false"),
            DeclareLaunchArgument("control_rate_hz", default_value="50.0"),
            DeclareLaunchArgument("reference_capture_delay_sec", default_value="3.0"),
            DeclareLaunchArgument("takeoff_altitude", default_value="0.30"),
            DeclareLaunchArgument("square_side_length", default_value="1.0"),
            DeclareLaunchArgument("approach_speed", default_value="1.0"),
            DeclareLaunchArgument("vertical_speed", default_value="0.20"),
            DeclareLaunchArgument("reach_xy_tol", default_value="0.20"),
            DeclareLaunchArgument("reach_z_tol", default_value="0.10"),
            DeclareLaunchArgument("speed_xy_tol", default_value="0.20"),
            DeclareLaunchArgument("speed_z_tol", default_value="0.15"),
            DeclareLaunchArgument("stable_time_sec", default_value="0.6"),
            DeclareLaunchArgument("takeoff_hover_sec", default_value="0.6"),
            DeclareLaunchArgument("corner_hover_sec", default_value="0.6"),
            DeclareLaunchArgument("final_hover_sec", default_value="0.6"),
            DeclareLaunchArgument("use_initial_heading_frame", default_value="true"),
            DeclareLaunchArgument("task_x_sign", default_value="1.0"),
            DeclareLaunchArgument("task_y_sign", default_value="1.0"),
            DeclareLaunchArgument("task_z_sign", default_value="1.0"),
            DeclareLaunchArgument("visual_odom_print_rate", default_value="1.0"),
            # ========== 节点启动顺序（ROS 2 会在所有节点就绪后并行运行） ==========
            px4_dds_monitor,
            px4_control_watchdog,
            px4_pointlio_position_compare,
            base_link_to_base_tf,
            pointlio_to_px4_visual_odom,
            pointlio_mapping,
            square_mission_controller,
        ]
    )
