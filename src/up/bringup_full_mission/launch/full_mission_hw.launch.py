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
    use_pointlio = LaunchConfiguration("use_pointlio")
    use_pointlio_px4_visual_odom = LaunchConfiguration("use_pointlio_px4_visual_odom")
    use_px4_monitor = LaunchConfiguration("use_px4_monitor")
    use_px4_control_watchdog = LaunchConfiguration("use_px4_control_watchdog")
    use_position_compare = LaunchConfiguration("use_position_compare")
    pointlio_config_file = LaunchConfiguration("pointlio_config_file")

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
        package="hx_bringup_full_mission",
        executable="pointlio_to_px4_visual_odom.py",
        output="screen",
        condition=IfCondition(use_pointlio_px4_visual_odom),
        parameters=[
            {
                "pointlio_topic": "/odom",
                "px4_local_topic": "/fmu/out/vehicle_local_position",
                "timesync_topic": "/fmu/out/timesync_status",
                "output_topic": "/fmu/in/vehicle_visual_odometry",
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
                "print_rate": LaunchConfiguration("visual_odom_print_rate"),
            }
        ],
    )

    full_mission_controller = Node(
        package="hx_bringup_full_mission",
        executable="full_mission_controller.py",
        output="screen",
        parameters=[
            {
                "auto_arm": LaunchConfiguration("auto_arm"),
                "control_rate_hz": LaunchConfiguration("control_rate_hz"),
                "reference_capture_delay_sec": LaunchConfiguration("reference_capture_delay_sec"),
                "selected_tasks": ParameterValue(LaunchConfiguration("selected_tasks"), value_type=str),
                "approach_speed": LaunchConfiguration("approach_speed"),
                "vertical_speed": LaunchConfiguration("vertical_speed"),
                "reach_xy_tol": LaunchConfiguration("reach_xy_tol"),
                "reach_z_tol": LaunchConfiguration("reach_z_tol"),
                "speed_xy_tol": LaunchConfiguration("speed_xy_tol"),
                "speed_z_tol": LaunchConfiguration("speed_z_tol"),
                "stable_time_sec": LaunchConfiguration("stable_time_sec"),
                "default_hover_sec": LaunchConfiguration("default_hover_sec"),
                "drop_hover_sec": LaunchConfiguration("drop_hover_sec"),
                "land_hover_sec": LaunchConfiguration("land_hover_sec"),
                "action_timeout_sec": LaunchConfiguration("action_timeout_sec"),
                "drop_command": ParameterValue(LaunchConfiguration("drop_command"), value_type=str),
                "drop_port": ParameterValue(LaunchConfiguration("drop_port"), value_type=str),
                "enable_d435i_drop_alignment": LaunchConfiguration("enable_d435i_drop_alignment"),
                "visual_correction_speed": LaunchConfiguration("visual_correction_speed"),
                "visual_align_tol": LaunchConfiguration("visual_align_tol"),
                "visual_align_stable_sec": LaunchConfiguration("visual_align_stable_sec"),
                "max_visual_offset": LaunchConfiguration("max_visual_offset"),
                "max_visual_step": LaunchConfiguration("max_visual_step"),
                "max_visual_correction_total": LaunchConfiguration("max_visual_correction_total"),
                "visual_offset_lowpass_alpha": LaunchConfiguration("visual_offset_lowpass_alpha"),
                "black_threshold": LaunchConfiguration("black_threshold"),
                "min_square_area": LaunchConfiguration("min_square_area"),
                "max_square_area": LaunchConfiguration("max_square_area"),
                "square_aspect_tol": LaunchConfiguration("square_aspect_tol"),
                "drop_camera_offset_body_x_right": LaunchConfiguration(
                    "drop_camera_offset_body_x_right"
                ),
                "drop_camera_offset_body_y_forward": LaunchConfiguration(
                    "drop_camera_offset_body_y_forward"
                ),
                "drop_camera_offset_body_z_up": LaunchConfiguration(
                    "drop_camera_offset_body_z_up"
                ),
                "start_drop_d435i_driver_on_alignment": LaunchConfiguration(
                    "start_drop_d435i_driver_on_alignment"
                ),
                "drop_d435i_camera_namespace": ParameterValue(
                    LaunchConfiguration("drop_d435i_camera_namespace"), value_type=str
                ),
                "drop_d435i_camera_name": ParameterValue(
                    LaunchConfiguration("drop_d435i_camera_name"), value_type=str
                ),
                "drop_d435i_serial_no": ParameterValue(
                    LaunchConfiguration("drop_d435i_serial_no"), value_type=str
                ),
                "drop_d435i_color_profile": ParameterValue(
                    LaunchConfiguration("drop_d435i_color_profile"), value_type=str
                ),
                "drop_d435i_depth_profile": ParameterValue(
                    LaunchConfiguration("drop_d435i_depth_profile"), value_type=str
                ),
                "drop_image_topic": ParameterValue(
                    LaunchConfiguration("drop_image_topic"), value_type=str
                ),
                "drop_camera_info_topic": ParameterValue(
                    LaunchConfiguration("drop_camera_info_topic"), value_type=str
                ),
                "enable_d435i_ring_alignment": LaunchConfiguration("enable_d435i_ring_alignment"),
                "ring_visual_correction_speed": LaunchConfiguration(
                    "ring_visual_correction_speed"
                ),
                "ring_align_tol": LaunchConfiguration("ring_align_tol"),
                "ring_align_stable_sec": LaunchConfiguration("ring_align_stable_sec"),
                "max_ring_lateral_offset": LaunchConfiguration("max_ring_lateral_offset"),
                "max_ring_step": LaunchConfiguration("max_ring_step"),
                "max_ring_correction_total": LaunchConfiguration("max_ring_correction_total"),
                "ring_offset_lowpass_alpha": LaunchConfiguration("ring_offset_lowpass_alpha"),
                "ring_white_threshold": LaunchConfiguration("ring_white_threshold"),
                "ring_min_area": LaunchConfiguration("ring_min_area"),
                "ring_max_area": LaunchConfiguration("ring_max_area"),
                "ring_min_circularity": LaunchConfiguration("ring_min_circularity"),
                "ring_diameter_m": LaunchConfiguration("ring_diameter_m"),
                "ring_depth_min_m": LaunchConfiguration("ring_depth_min_m"),
                "ring_depth_max_m": LaunchConfiguration("ring_depth_max_m"),
                "ring_camera_offset_body_x_right": LaunchConfiguration(
                    "ring_camera_offset_body_x_right"
                ),
                "ring_camera_offset_body_y_forward": LaunchConfiguration(
                    "ring_camera_offset_body_y_forward"
                ),
                "ring_camera_offset_body_z_up": LaunchConfiguration(
                    "ring_camera_offset_body_z_up"
                ),
                "ring_lateral_sign": LaunchConfiguration("ring_lateral_sign"),
                "start_ring_d435i_driver_on_alignment": LaunchConfiguration(
                    "start_ring_d435i_driver_on_alignment"
                ),
                "ring_d435i_camera_namespace": ParameterValue(
                    LaunchConfiguration("ring_d435i_camera_namespace"), value_type=str
                ),
                "ring_d435i_camera_name": ParameterValue(
                    LaunchConfiguration("ring_d435i_camera_name"), value_type=str
                ),
                "ring_d435i_serial_no": ParameterValue(
                    LaunchConfiguration("ring_d435i_serial_no"), value_type=str
                ),
                "ring_d435i_color_profile": ParameterValue(
                    LaunchConfiguration("ring_d435i_color_profile"), value_type=str
                ),
                "ring_d435i_depth_profile": ParameterValue(
                    LaunchConfiguration("ring_d435i_depth_profile"), value_type=str
                ),
                "ring_image_topic": ParameterValue(
                    LaunchConfiguration("ring_image_topic"), value_type=str
                ),
                "ring_camera_info_topic": ParameterValue(
                    LaunchConfiguration("ring_camera_info_topic"), value_type=str
                ),
                "ring_depth_topic": ParameterValue(
                    LaunchConfiguration("ring_depth_topic"), value_type=str
                ),
                "use_initial_heading_frame": LaunchConfiguration("use_initial_heading_frame"),
                "task_x_sign": LaunchConfiguration("task_x_sign"),
                "task_y_sign": LaunchConfiguration("task_y_sign"),
                "task_z_sign": LaunchConfiguration("task_z_sign"),
                "vehicle_local_position_topic": "/fmu/out/vehicle_local_position",
                "vehicle_status_topic": "/fmu/out/vehicle_status",
            }
        ],
    )

    px4_dds_monitor = Node(
        package="hx_bringup_full_mission",
        executable="px4_dds_monitor.py",
        output="screen",
        condition=IfCondition(use_px4_monitor),
    )

    px4_control_watchdog = Node(
        package="hx_bringup_full_mission",
        executable="px4_control_watchdog.py",
        output="screen",
        condition=IfCondition(use_px4_control_watchdog),
        parameters=[{"control_source": "full_mission_controller"}],
    )

    px4_pointlio_position_compare = Node(
        package="hx_bringup_full_mission",
        executable="px4_pointlio_position_compare.py",
        output="screen",
        condition=IfCondition(use_position_compare),
        parameters=[
            {
                "px4_topic": "/fmu/out/vehicle_local_position",
                "pointlio_topic": "/odom",
                "print_rate": 2.0,
                "pointlio_y_to_px4_y_sign": -1.0,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_pointlio", default_value="true"),
            DeclareLaunchArgument("use_pointlio_px4_visual_odom", default_value="true"),
            DeclareLaunchArgument("use_px4_monitor", default_value="false"),
            DeclareLaunchArgument("use_px4_control_watchdog", default_value="true"),
            DeclareLaunchArgument("use_position_compare", default_value="false"),
            DeclareLaunchArgument("pointlio_config_file", default_value=default_pointlio_config),
            DeclareLaunchArgument("visual_odom_print_rate", default_value="1.0"),
            DeclareLaunchArgument("auto_arm", default_value="false"),
            DeclareLaunchArgument("control_rate_hz", default_value="50.0"),
            DeclareLaunchArgument("reference_capture_delay_sec", default_value="3.0"),
            DeclareLaunchArgument("selected_tasks", default_value="6"),
            DeclareLaunchArgument("approach_speed", default_value="0.25"),
            DeclareLaunchArgument("vertical_speed", default_value="0.25"),
            DeclareLaunchArgument("reach_xy_tol", default_value="0.20"),
            DeclareLaunchArgument("reach_z_tol", default_value="0.10"),
            DeclareLaunchArgument("speed_xy_tol", default_value="0.20"),
            DeclareLaunchArgument("speed_z_tol", default_value="0.15"),
            DeclareLaunchArgument("stable_time_sec", default_value="0.6"),
            DeclareLaunchArgument("default_hover_sec", default_value="0.6"),
            DeclareLaunchArgument("drop_hover_sec", default_value="0.6"),
            DeclareLaunchArgument("land_hover_sec", default_value="0.6"),
            DeclareLaunchArgument("action_timeout_sec", default_value="8.0"),
            DeclareLaunchArgument("drop_command", default_value="/home/venom/venom/paotou.py"),
            DeclareLaunchArgument("drop_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("enable_d435i_drop_alignment", default_value="true"),
            DeclareLaunchArgument("visual_correction_speed", default_value="0.10"),
            DeclareLaunchArgument("visual_align_tol", default_value="0.06"),
            DeclareLaunchArgument("visual_align_stable_sec", default_value="0.6"),
            DeclareLaunchArgument("max_visual_offset", default_value="0.65"),
            DeclareLaunchArgument("max_visual_step", default_value="0.06"),
            DeclareLaunchArgument("max_visual_correction_total", default_value="0.70"),
            DeclareLaunchArgument("visual_offset_lowpass_alpha", default_value="0.25"),
            DeclareLaunchArgument("black_threshold", default_value="80"),
            DeclareLaunchArgument("min_square_area", default_value="800.0"),
            DeclareLaunchArgument("max_square_area", default_value="250000.0"),
            DeclareLaunchArgument("square_aspect_tol", default_value="0.35"),
            DeclareLaunchArgument("drop_camera_offset_body_x_right", default_value="0.0"),
            DeclareLaunchArgument("drop_camera_offset_body_y_forward", default_value="0.065"),
            DeclareLaunchArgument("drop_camera_offset_body_z_up", default_value="-0.12"),
            DeclareLaunchArgument("start_drop_d435i_driver_on_alignment", default_value="true"),
            DeclareLaunchArgument("drop_d435i_camera_namespace", default_value="d435i_down"),
            DeclareLaunchArgument("drop_d435i_camera_name", default_value="d435i_down"),
            DeclareLaunchArgument("drop_d435i_serial_no", default_value=""),
            DeclareLaunchArgument("drop_d435i_color_profile", default_value="640,480,30"),
            DeclareLaunchArgument("drop_d435i_depth_profile", default_value="640,480,30"),
            DeclareLaunchArgument(
                "drop_image_topic", default_value="/d435i_down/d435i_down/color/image_raw"
            ),
            DeclareLaunchArgument(
                "drop_camera_info_topic",
                default_value="/d435i_down/d435i_down/color/camera_info",
            ),
            DeclareLaunchArgument("enable_d435i_ring_alignment", default_value="true"),
            DeclareLaunchArgument("ring_visual_correction_speed", default_value="0.10"),
            DeclareLaunchArgument("ring_align_tol", default_value="0.06"),
            DeclareLaunchArgument("ring_align_stable_sec", default_value="0.6"),
            DeclareLaunchArgument("max_ring_lateral_offset", default_value="0.75"),
            DeclareLaunchArgument("max_ring_step", default_value="0.06"),
            DeclareLaunchArgument("max_ring_correction_total", default_value="0.70"),
            DeclareLaunchArgument("ring_offset_lowpass_alpha", default_value="0.25"),
            DeclareLaunchArgument("ring_white_threshold", default_value="185"),
            DeclareLaunchArgument("ring_min_area", default_value="1200.0"),
            DeclareLaunchArgument("ring_max_area", default_value="250000.0"),
            DeclareLaunchArgument("ring_min_circularity", default_value="0.25"),
            DeclareLaunchArgument("ring_diameter_m", default_value="0.90"),
            DeclareLaunchArgument("ring_depth_min_m", default_value="0.20"),
            DeclareLaunchArgument("ring_depth_max_m", default_value="5.00"),
            DeclareLaunchArgument("ring_camera_offset_body_x_right", default_value="0.0"),
            DeclareLaunchArgument("ring_camera_offset_body_y_forward", default_value="0.065"),
            DeclareLaunchArgument("ring_camera_offset_body_z_up", default_value="0.12"),
            DeclareLaunchArgument("ring_lateral_sign", default_value="1.0"),
            DeclareLaunchArgument("start_ring_d435i_driver_on_alignment", default_value="true"),
            DeclareLaunchArgument("ring_d435i_camera_namespace", default_value="d435i_front"),
            DeclareLaunchArgument("ring_d435i_camera_name", default_value="d435i_front"),
            DeclareLaunchArgument("ring_d435i_serial_no", default_value=""),
            DeclareLaunchArgument("ring_d435i_color_profile", default_value="640,480,30"),
            DeclareLaunchArgument("ring_d435i_depth_profile", default_value="640,480,30"),
            DeclareLaunchArgument(
                "ring_image_topic", default_value="/d435i_front/d435i_front/color/image_raw"
            ),
            DeclareLaunchArgument(
                "ring_camera_info_topic",
                default_value="/d435i_front/d435i_front/color/camera_info",
            ),
            DeclareLaunchArgument(
                "ring_depth_topic",
                default_value="/d435i_front/d435i_front/aligned_depth_to_color/image_raw",
            ),
            DeclareLaunchArgument("use_initial_heading_frame", default_value="true"),
            DeclareLaunchArgument("task_x_sign", default_value="1.0"),
            DeclareLaunchArgument("task_y_sign", default_value="1.0"),
            DeclareLaunchArgument("task_z_sign", default_value="1.0"),
            px4_dds_monitor,
            px4_control_watchdog,
            px4_pointlio_position_compare,
            pointlio_to_px4_visual_odom,
            pointlio_mapping,
            full_mission_controller,
        ]
    )
