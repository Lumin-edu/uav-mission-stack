import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    use_pointlio = LaunchConfiguration("use_pointlio")
    use_pointlio_px4_visual_odom = LaunchConfiguration("use_pointlio_px4_visual_odom")
    use_hover_control = LaunchConfiguration("use_hover_control")
    use_px4_monitor = LaunchConfiguration("use_px4_monitor")
    use_px4_control_watchdog = LaunchConfiguration("use_px4_control_watchdog")
    use_position_compare = LaunchConfiguration("use_position_compare")

    pointlio_config_file = LaunchConfiguration("pointlio_config_file")
    takeoff_altitude = LaunchConfiguration("takeoff_altitude")
    hover_auto_arm = LaunchConfiguration("hover_auto_arm")
    offboard_rate_hz = LaunchConfiguration("offboard_rate_hz")
    visual_odom_print_rate = LaunchConfiguration("visual_odom_print_rate")
    reference_capture_delay_sec = LaunchConfiguration("reference_capture_delay_sec")
    use_base_link_to_base_tf = LaunchConfiguration("use_base_link_to_base_tf")

    # Point-LIO 的 base_link 数值对应 MID360 IMU 原点；base 是补偿后的机体中心。
    # 这组常量同时传给数值桥接节点和静态 TF，防止两条链路使用不同外参。
    base_link_to_base_translation = [-0.011, -0.02329, -0.05588]
    base_link_to_base_rotation_xyzw = [0.0, 0.0, 0.0, 1.0]

    pointlio_share = get_package_share_directory("point_lio")
    pointlio_launch = os.path.join(pointlio_share, "launch", "mapping_headless.launch.py")
    default_pointlio_config = os.path.join(pointlio_share, "config", "mid360_mapping.yaml")

    pointlio_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pointlio_launch),
        launch_arguments={
            "point_lio_cfg_dir": pointlio_config_file,
            "rviz": "false",
        }.items(),
        condition=IfCondition(use_pointlio),
    )

    pointlio_to_px4_visual_odom = Node(
        package="bringup_pointlio_hover",
        executable="pointlio_to_px4_visual_odom.py",
        output="screen",
        condition=IfCondition(use_pointlio_px4_visual_odom),
        parameters=[
            {
                "pointlio_topic": "/odom",
                "px4_local_topic": "/fmu/out/vehicle_local_position",
                "timesync_topic": "/fmu/out/timesync_status",
                "output_topic": "/fmu/in/vehicle_visual_odometry",
                # Point-LIO 保持输出 odom -> base_link；base_link 数值对应 IMU 原点。
                "pointlio_pose_frame": "base_link",
                # 修正后的机体中心统一命名为 base，并以此语义发送给 PX4。
                "vehicle_frame": "base",
                # T_base_link_base：base 原点在 Point-LIO base_link 坐标系中的坐标。
                # [-0.011, -0.02329, 0.04412] + [0, 0, -0.10]
                # = [-0.011, -0.02329, -0.05588] m。
                "base_link_to_base_translation": base_link_to_base_translation,
                # Point-LIO base_link 与机体 base 轴向一致，没有额外安装旋转。
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

    # Point-LIO 已发布动态 TF odom -> base_link。这里补充固定 TF base_link -> base，
    # TF 树即可自动得到经过杆臂补偿的 odom -> base，且不会重复发布动态父子关系。
    # 此静态 TF 只补全 TF 树；PX4 桥接节点不读取 TF，不会影响或重复执行 NED 转换。
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

    fixed_point_hover = Node(
        package="bringup_pointlio_hover",
        executable="fixed_point_hover.py",
        output="screen",
        parameters=[
            {
                "target_x": 0.0,
                "target_y": 0.0,
                "target_z": PythonExpression(["-1.0 * ", takeoff_altitude]),
                "target_yaw": 0.0,
                "auto_arm": hover_auto_arm,
                "use_current_position_reference": True,
                "reference_capture_delay_sec": reference_capture_delay_sec,
                "control_rate_hz": offboard_rate_hz,
                "vehicle_local_position_topic": "/fmu/out/vehicle_local_position",
                "vehicle_status_topic": "/fmu/out/vehicle_status",
            },
        ],
        condition=IfCondition(use_hover_control),
    )

    px4_dds_monitor = Node(
        package="bringup_pointlio_hover",
        executable="px4_dds_monitor.py",
        output="screen",
        condition=IfCondition(use_px4_monitor),
    )

    px4_control_watchdog = Node(
        package="bringup_pointlio_hover",
        executable="px4_control_watchdog.py",
        output="screen",
        condition=IfCondition(use_px4_control_watchdog),
        parameters=[{"control_source": "fixed_point_hover"}],
    )

    px4_pointlio_position_compare = Node(
        package="bringup_pointlio_hover",
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
            DeclareLaunchArgument("use_pointlio", default_value="true"),
            DeclareLaunchArgument("use_pointlio_px4_visual_odom", default_value="true"),
            DeclareLaunchArgument("use_hover_control", default_value="true"),
            DeclareLaunchArgument("use_px4_monitor", default_value="false"),
            DeclareLaunchArgument("use_px4_control_watchdog", default_value="true"),
            DeclareLaunchArgument("use_position_compare", default_value="false"),
            DeclareLaunchArgument("pointlio_config_file", default_value=default_pointlio_config),
            DeclareLaunchArgument("takeoff_altitude", default_value="0.2"),
            DeclareLaunchArgument("hover_auto_arm", default_value="false"),
            DeclareLaunchArgument("offboard_rate_hz", default_value="50.0"),
            DeclareLaunchArgument("visual_odom_print_rate", default_value="1.0"),
            DeclareLaunchArgument("reference_capture_delay_sec", default_value="3.0"),
            DeclareLaunchArgument("use_base_link_to_base_tf", default_value="true"),
            pointlio_mapping,
            base_link_to_base_tf,
            pointlio_to_px4_visual_odom,
            px4_dds_monitor,
            px4_control_watchdog,
            px4_pointlio_position_compare,
            fixed_point_hover,
        ]
    )
