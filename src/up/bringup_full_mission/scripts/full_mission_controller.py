#!/usr/bin/env python3

import math
import subprocess
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
import rclpy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


@dataclass(frozen=True)
class MissionWaypoint:
    x_right: float
    y_forward: float
    z_up: float
    label: str
    task_id: int = 0
    is_drop_point: bool = False
    is_land_point: bool = False
    starts_ring_alignment: bool = False


class FullMissionController(Node):
    STATE_WAITING_REFERENCE = "waiting_reference"
    STATE_MISSION = "mission"
    STATE_VISUAL_ALIGN = "visual_align"
    STATE_RING_ALIGN = "ring_align"
    STATE_ACTION = "action"
    STATE_COMPLETED = "completed"

    DROP_ANGLES = (90.0, 125.0, 175.0)

    def __init__(self) -> None:
        super().__init__("full_mission_controller")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.auto_arm = self.param_bool("auto_arm", False)
        self.control_rate_hz = self.param_float("control_rate_hz", 50.0)
        self.reference_capture_delay_sec = self.param_float("reference_capture_delay_sec", 3.0)
        self.approach_speed = self.param_float("approach_speed", 0.25)
        self.vertical_speed = self.param_float("vertical_speed", 0.25)
        self.reach_xy_tol = self.param_float("reach_xy_tol", 0.20)
        self.reach_z_tol = self.param_float("reach_z_tol", 0.10)
        self.speed_xy_tol = self.param_float("speed_xy_tol", 0.20)
        self.speed_z_tol = self.param_float("speed_z_tol", 0.15)
        self.stable_time_sec = self.param_float("stable_time_sec", 0.6)
        self.default_hover_sec = self.param_float("default_hover_sec", 0.6)
        self.drop_hover_sec = self.param_float("drop_hover_sec", 0.6)
        self.land_hover_sec = self.param_float("land_hover_sec", 0.6)
        self.action_timeout_sec = self.param_float("action_timeout_sec", 8.0)
        self.enable_d435i_drop_alignment = self.param_bool("enable_d435i_drop_alignment", True)
        self.visual_correction_speed = self.param_float("visual_correction_speed", 0.10)
        self.visual_align_tol = self.param_float("visual_align_tol", 0.06)
        self.visual_align_stable_sec = self.param_float("visual_align_stable_sec", 0.6)
        self.max_visual_offset = self.param_float("max_visual_offset", 0.65)
        self.max_visual_step = self.param_float("max_visual_step", 0.06)
        self.max_visual_correction_total = self.param_float("max_visual_correction_total", 0.70)
        self.visual_offset_lowpass_alpha = self.param_float("visual_offset_lowpass_alpha", 0.25)
        self.black_threshold = self.param_int("black_threshold", 80)
        self.min_square_area = self.param_float("min_square_area", 800.0)
        self.max_square_area = self.param_float("max_square_area", 250000.0)
        self.square_aspect_tol = self.param_float("square_aspect_tol", 0.35)
        self.camera_offset_body_x_right = self.param_float(
            "drop_camera_offset_body_x_right",
            self.param_float("camera_offset_body_x_right", 0.0),
        )
        self.camera_offset_body_y_forward = self.param_float(
            "drop_camera_offset_body_y_forward",
            self.param_float("camera_offset_body_y_forward", 0.065),
        )
        self.camera_offset_body_z_up = self.param_float(
            "drop_camera_offset_body_z_up",
            self.param_float("camera_offset_body_z_up", -0.12),
        )
        self.start_d435i_driver_on_alignment = self.param_bool(
            "start_drop_d435i_driver_on_alignment",
            self.param_bool("start_d435i_driver_on_alignment", True),
        )
        self.d435i_camera_namespace = self.param_string(
            "drop_d435i_camera_namespace",
            self.param_string("d435i_camera_namespace", "d435i_down"),
        )
        self.d435i_camera_name = self.param_string(
            "drop_d435i_camera_name",
            self.param_string("d435i_camera_name", "d435i_down"),
        )
        self.d435i_serial_no = self.param_string("drop_d435i_serial_no", "")
        self.d435i_color_profile = self.param_string(
            "drop_d435i_color_profile",
            self.param_string("d435i_color_profile", "640,480,30"),
        )
        self.d435i_depth_profile = self.param_string(
            "drop_d435i_depth_profile",
            self.param_string("d435i_depth_profile", "640,480,30"),
        )
        self.image_topic = self.param_string(
            "drop_image_topic",
            self.param_string("image_topic", "/d435i_down/d435i_down/color/image_raw"),
        )
        self.camera_info_topic = self.param_string(
            "drop_camera_info_topic",
            self.param_string("camera_info_topic", "/d435i_down/d435i_down/color/camera_info"),
        )
        self.enable_d435i_ring_alignment = self.param_bool("enable_d435i_ring_alignment", True)
        self.ring_visual_correction_speed = self.param_float("ring_visual_correction_speed", 0.10)
        self.ring_align_tol = self.param_float("ring_align_tol", 0.06)
        self.ring_align_stable_sec = self.param_float("ring_align_stable_sec", 0.6)
        self.max_ring_lateral_offset = self.param_float("max_ring_lateral_offset", 0.75)
        self.max_ring_step = self.param_float("max_ring_step", 0.06)
        self.max_ring_correction_total = self.param_float("max_ring_correction_total", 0.70)
        self.ring_offset_lowpass_alpha = self.param_float("ring_offset_lowpass_alpha", 0.25)
        self.ring_white_threshold = self.param_int("ring_white_threshold", 185)
        self.ring_min_area = self.param_float("ring_min_area", 1200.0)
        self.ring_max_area = self.param_float("ring_max_area", 250000.0)
        self.ring_min_circularity = self.param_float("ring_min_circularity", 0.25)
        self.ring_diameter_m = self.param_float("ring_diameter_m", 0.90)
        self.ring_depth_min_m = self.param_float("ring_depth_min_m", 0.20)
        self.ring_depth_max_m = self.param_float("ring_depth_max_m", 5.00)
        self.ring_camera_offset_body_x_right = self.param_float(
            "ring_camera_offset_body_x_right", 0.0
        )
        self.ring_camera_offset_body_y_forward = self.param_float(
            "ring_camera_offset_body_y_forward", 0.065
        )
        self.ring_camera_offset_body_z_up = self.param_float(
            "ring_camera_offset_body_z_up", 0.12
        )
        self.ring_lateral_sign = self.param_axis_sign("ring_lateral_sign", 1.0)
        self.start_ring_d435i_driver_on_alignment = self.param_bool(
            "start_ring_d435i_driver_on_alignment", True
        )
        self.ring_d435i_camera_namespace = self.param_string(
            "ring_d435i_camera_namespace", "d435i_front"
        )
        self.ring_d435i_camera_name = self.param_string("ring_d435i_camera_name", "d435i_front")
        self.ring_d435i_serial_no = self.param_string("ring_d435i_serial_no", "")
        self.ring_d435i_color_profile = self.param_string(
            "ring_d435i_color_profile", "640,480,30"
        )
        self.ring_d435i_depth_profile = self.param_string(
            "ring_d435i_depth_profile", "640,480,30"
        )
        self.ring_image_topic = self.param_string(
            "ring_image_topic", "/d435i_front/d435i_front/color/image_raw"
        )
        self.ring_camera_info_topic = self.param_string(
            "ring_camera_info_topic", "/d435i_front/d435i_front/color/camera_info"
        )
        self.ring_depth_topic = self.param_string(
            "ring_depth_topic",
            "/d435i_front/d435i_front/aligned_depth_to_color/image_raw",
        )
        self.use_initial_heading_frame = self.param_bool("use_initial_heading_frame", True)
        self.task_x_sign = self.param_axis_sign("task_x_sign", 1.0)
        self.task_y_sign = self.param_axis_sign("task_y_sign", 1.0)
        self.task_z_sign = self.param_axis_sign("task_z_sign", 1.0)
        self.selected_tasks_text = self.param_string("selected_tasks", "6")
        # The actuator script is site-specific and must be supplied explicitly
        # on the launch command line before enabling a drop task.
        self.drop_command = self.param_string("drop_command", "")
        self.drop_port = self.param_string("drop_port", "/dev/ttyUSB0")
        self.vehicle_local_position_topic = self.param_string(
            "vehicle_local_position_topic", "/fmu/out/vehicle_local_position"
        )
        self.vehicle_status_topic = self.param_string(
            "vehicle_status_topic", "/fmu/out/vehicle_status"
        )

        self.selected_tasks = self.parse_selected_tasks(self.selected_tasks_text)
        self.drop_task_ids = [task_id for task_id in self.selected_tasks if 1 <= task_id <= 5]
        self.landing_task_id = self.resolve_landing_task_id(self.selected_tasks)
        self.drop_angle_by_task = self.assign_drop_angles(self.drop_task_ids)

        self.validate_parameters()

        self.timer_period = 1.0 / self.control_rate_hz
        self.offboard_prestream_cycles = max(10, int(2.0 * self.control_rate_hz))
        self.stable_cycles_required = max(1, int(self.stable_time_sec * self.control_rate_hz))
        self.visual_stable_cycles_required = max(
            1, int(self.visual_align_stable_sec * self.control_rate_hz)
        )
        self.ring_stable_cycles_required = max(
            1, int(self.ring_align_stable_sec * self.control_rate_hz)
        )

        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos
        )
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos
        )

        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.camera_info: Optional[CameraInfo] = None
        self.ring_camera_info: Optional[CameraInfo] = None
        self.latest_visual_offset_body: Optional[tuple[float, float]] = None
        self.latest_visual_time = 0.0
        self.latest_square_pixel: Optional[tuple[float, float]] = None
        self.latest_ring_lateral_offset_body: Optional[float] = None
        self.latest_ring_time = 0.0
        self.latest_ring_pixel: Optional[tuple[float, float]] = None
        self.latest_ring_diameter_px = 0.0
        self.create_subscription(
            VehicleLocalPosition,
            self.vehicle_local_position_topic,
            self.vehicle_local_position_callback,
            qos,
        )
        self.create_subscription(
            VehicleStatus,
            self.vehicle_status_topic,
            self.vehicle_status_callback,
            qos,
        )
        self.create_subscription(Image, self.image_topic, self.image_callback, image_qos)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, image_qos)
        self.create_subscription(Image, self.ring_image_topic, self.ring_image_callback, image_qos)
        self.create_subscription(
            CameraInfo,
            self.ring_camera_info_topic,
            self.ring_camera_info_callback,
            image_qos,
        )

        self.state = self.STATE_WAITING_REFERENCE
        self.reference_capture_ready_sec = self.now_sec() + self.reference_capture_delay_sec
        self.reference_captured = False
        self.armed = False
        self.offboard_enabled = False
        self.offboard_setpoint_counter = 0
        self.last_arm_request_us = 0
        self.last_offboard_request_us = 0
        self.waiting_for_reference_logged = False
        self.waiting_for_manual_arm_logged = False
        self.waiting_for_offboard_logged = False
        self.completion_logged = False

        self.start_local: Optional[list[float]] = None
        self.task_waypoints: list[MissionWaypoint] = []
        self.local_waypoints: list[list[float]] = []
        self.commanded_local: Optional[list[float]] = None
        self.locked_yaw = 0.0
        self.waypoint_index = 0
        self.stable_cycles = 0
        self.hold_start_sec: Optional[float] = None
        self.action_started = False
        self.action_completed = False
        self.action_failed = False
        self.action_start_sec: Optional[float] = None
        self.action_process: Optional[subprocess.Popen] = None
        self.visual_stable_cycles = 0
        self.visual_base_local: Optional[list[float]] = None
        self.visual_correction_local_xy: Optional[list[float]] = None
        self.d435i_process: Optional[subprocess.Popen] = None
        self.d435i_start_requested = False
        self.ring_stable_cycles = 0
        self.ring_base_local: Optional[list[float]] = None
        self.ring_correction_local_xy: Optional[list[float]] = None
        self.ring_d435i_process: Optional[subprocess.Popen] = None
        self.ring_d435i_start_requested = False
        self.last_warn_sec = 0.0

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info(
            "Full mission controller ready. "
            f"selected_tasks={self.selected_tasks}, landing_task={self.landing_task_id}, "
            f"drop_angle_by_task={self.drop_angle_by_task}, auto_arm={self.auto_arm}, "
            f"approach_speed={self.approach_speed:.2f}, vertical_speed={self.vertical_speed:.2f}, "
            f"reach_xy_tol={self.reach_xy_tol:.2f}, "
            f"d435i_drop_alignment={self.enable_d435i_drop_alignment}, "
            f"task_signs=({self.task_x_sign:+.0f}, {self.task_y_sign:+.0f}, {self.task_z_sign:+.0f})"
        )

    def param_float(self, name: str, default: float) -> float:
        value = self.declare_parameter(name, default).value
        try:
            return float(value)
        except (TypeError, ValueError):
            self.get_logger().warn(f"Parameter {name}={value!r} is not a float; using {default}.")
            return float(default)

    def param_int(self, name: str, default: int) -> int:
        value = self.declare_parameter(name, default).value
        try:
            return int(value)
        except (TypeError, ValueError):
            self.get_logger().warn(f"Parameter {name}={value!r} is not an int; using {default}.")
            return int(default)

    def param_bool(self, name: str, default: bool) -> bool:
        value = self.declare_parameter(name, default).value
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    def param_string(self, name: str, default: str) -> str:
        return str(self.declare_parameter(name, default).value)

    def param_axis_sign(self, name: str, default: float) -> float:
        value = self.param_float(name, default)
        return -1.0 if value < 0.0 else 1.0

    def parse_selected_tasks(self, text: str) -> list[int]:
        selected: list[int] = []
        for raw_token in text.replace(";", ",").replace(" ", ",").split(","):
            token = raw_token.strip()
            if not token:
                continue
            try:
                task_id = int(token)
            except ValueError:
                self.get_logger().warn(f"Ignoring invalid selected task token: {token!r}")
                continue
            if task_id < 1 or task_id > 7:
                self.get_logger().warn(f"Ignoring selected task outside 1..7: {task_id}")
                continue
            if task_id not in selected:
                selected.append(task_id)
        return selected

    def resolve_landing_task_id(self, selected_tasks: list[int]) -> int:
        has_6 = 6 in selected_tasks
        has_7 = 7 in selected_tasks
        if has_6 and has_7:
            self.get_logger().warn("Both landing task 6 and 7 selected; using landing task 6.")
            return 6
        if has_7:
            return 7
        if not has_6:
            self.get_logger().warn("No landing task selected; defaulting to landing task 6.")
        return 6

    def assign_drop_angles(self, drop_task_ids: list[int]) -> dict[int, float]:
        angle_by_task: dict[int, float] = {}
        for index, task_id in enumerate(drop_task_ids):
            if index >= len(self.DROP_ANGLES):
                self.get_logger().warn(
                    f"Drop task {task_id} selected but no drop angle remains; skipping its drop action."
                )
                continue
            angle_by_task[task_id] = self.DROP_ANGLES[index]
        return angle_by_task

    def validate_parameters(self) -> None:
        if self.control_rate_hz <= 0.0:
            self.get_logger().warn("control_rate_hz must be positive; using 50 Hz.")
            self.control_rate_hz = 50.0
        if self.reference_capture_delay_sec < 0.0:
            self.get_logger().warn("reference_capture_delay_sec cannot be negative; using 0 s.")
            self.reference_capture_delay_sec = 0.0
        if self.approach_speed <= 0.01:
            self.get_logger().warn("approach_speed is too small; using 0.25 m/s.")
            self.approach_speed = 0.25
        if self.approach_speed > 0.25:
            self.get_logger().warn("approach_speed is capped at 0.25 m/s.")
            self.approach_speed = 0.25
        if self.vertical_speed <= 0.01:
            self.get_logger().warn("vertical_speed is too small; using 0.25 m/s.")
            self.vertical_speed = 0.25
        if self.vertical_speed > 0.25:
            self.get_logger().warn("vertical_speed is capped at 0.25 m/s.")
            self.vertical_speed = 0.25
        if self.reach_xy_tol <= 0.01:
            self.get_logger().warn("reach_xy_tol is too small; using 0.20 m.")
            self.reach_xy_tol = 0.20
        if self.reach_z_tol <= 0.01:
            self.get_logger().warn("reach_z_tol is too small; using 0.10 m.")
            self.reach_z_tol = 0.10
        if self.speed_xy_tol < 0.0:
            self.get_logger().warn("speed_xy_tol cannot be negative; using 0.20 m/s.")
            self.speed_xy_tol = 0.20
        if self.speed_z_tol < 0.0:
            self.get_logger().warn("speed_z_tol cannot be negative; using 0.15 m/s.")
            self.speed_z_tol = 0.15
        if self.stable_time_sec < 0.0:
            self.get_logger().warn("stable_time_sec cannot be negative; using 0 s.")
            self.stable_time_sec = 0.0
        if self.default_hover_sec < 0.0:
            self.default_hover_sec = 0.0
        if self.drop_hover_sec < 0.0:
            self.drop_hover_sec = 0.0
        if self.land_hover_sec < 0.0:
            self.land_hover_sec = 0.0
        if self.action_timeout_sec <= 0.0:
            self.get_logger().warn("action_timeout_sec must be positive; using 8 s.")
            self.action_timeout_sec = 8.0
        if self.visual_correction_speed <= 0.01:
            self.get_logger().warn("visual_correction_speed is too small; using 0.10 m/s.")
            self.visual_correction_speed = 0.10
        if self.visual_correction_speed > 0.10:
            self.get_logger().warn("visual_correction_speed is capped at 0.10 m/s.")
            self.visual_correction_speed = 0.10
        if self.ring_visual_correction_speed <= 0.01:
            self.get_logger().warn("ring_visual_correction_speed is too small; using 0.10 m/s.")
            self.ring_visual_correction_speed = 0.10
        if self.ring_visual_correction_speed > 0.10:
            self.get_logger().warn("ring_visual_correction_speed is capped at 0.10 m/s.")
            self.ring_visual_correction_speed = 0.10
        self.visual_align_tol = max(0.02, self.visual_align_tol)
        self.visual_align_stable_sec = max(0.0, self.visual_align_stable_sec)
        self.max_visual_offset = max(0.05, self.max_visual_offset)
        self.max_visual_step = max(0.01, self.max_visual_step)
        self.max_visual_correction_total = max(0.05, self.max_visual_correction_total)
        self.visual_offset_lowpass_alpha = max(
            0.01, min(1.0, self.visual_offset_lowpass_alpha)
        )
        self.black_threshold = max(1, min(254, self.black_threshold))
        self.min_square_area = max(10.0, self.min_square_area)
        self.max_square_area = max(self.min_square_area, self.max_square_area)
        self.square_aspect_tol = max(0.05, min(0.95, self.square_aspect_tol))
        self.ring_align_tol = max(0.02, self.ring_align_tol)
        self.ring_align_stable_sec = max(0.0, self.ring_align_stable_sec)
        self.max_ring_lateral_offset = max(0.05, self.max_ring_lateral_offset)
        self.max_ring_step = max(0.01, self.max_ring_step)
        self.max_ring_correction_total = max(0.05, self.max_ring_correction_total)
        self.ring_offset_lowpass_alpha = max(
            0.01, min(1.0, self.ring_offset_lowpass_alpha)
        )
        self.ring_white_threshold = max(1, min(254, self.ring_white_threshold))
        self.ring_min_area = max(10.0, self.ring_min_area)
        self.ring_max_area = max(self.ring_min_area, self.ring_max_area)
        self.ring_min_circularity = max(0.01, min(1.0, self.ring_min_circularity))
        self.ring_diameter_m = max(0.05, self.ring_diameter_m)
        self.ring_depth_min_m = max(0.01, self.ring_depth_min_m)
        self.ring_depth_max_m = max(self.ring_depth_min_m, self.ring_depth_max_m)
        if self.image_topic == self.ring_image_topic:
            self.get_logger().warn(
                "Drop D435i image topic equals ring D435i image topic; check camera namespaces."
            )
        if self.camera_info_topic == self.ring_camera_info_topic:
            self.get_logger().warn(
                "Drop D435i camera_info topic equals ring D435i camera_info topic; check camera namespaces."
            )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def warn_throttled(self, message: str, period: float = 2.0) -> None:
        now = self.now_sec()
        if now - self.last_warn_sec >= period:
            self.last_warn_sec = now
            self.get_logger().warn(message)

    def wrap_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def current_px4_yaw(self) -> float:
        heading = float(self.vehicle_local_position.heading)
        if math.isfinite(heading):
            return self.wrap_angle(heading)
        self.get_logger().warn("PX4 heading is not finite when capturing reference; using yaw=0.")
        return 0.0

    def build_task_waypoints(self) -> list[MissionWaypoint]:
        cruise_z = 1.42
        ring_z = 1.42
        work_z = 0.65
        waypoints = [
            MissionWaypoint(0.0, 0.0, cruise_z, "takeoff_cruise"),
            MissionWaypoint(-1.44, 0.0, cruise_z, "obstacle_approach_1"),
            MissionWaypoint(-2.88, 0.0, cruise_z, "obstacle_orbit_start"),
            MissionWaypoint(-2.88, 1.28, cruise_z, "target_1_cruise"),
            MissionWaypoint(-2.88, 1.28, work_z, "target_1_work", task_id=1, is_drop_point=True),
            MissionWaypoint(-2.88, 1.28, cruise_z, "target_1_recover"),
            MissionWaypoint(-1.44, 1.28, cruise_z, "target_2_cruise"),
            MissionWaypoint(-1.44, 1.28, work_z, "target_2_work", task_id=2, is_drop_point=True),
            MissionWaypoint(-1.44, 1.28, cruise_z, "target_2_recover"),
            MissionWaypoint(-1.44, -1.28, cruise_z, "target_3_cruise"),
            MissionWaypoint(-1.44, -1.28, work_z, "target_3_work", task_id=3, is_drop_point=True),
            MissionWaypoint(-1.44, -1.28, cruise_z, "target_3_recover"),
            MissionWaypoint(-2.88, -1.28, cruise_z, "target_4_cruise"),
            MissionWaypoint(-2.88, -1.28, work_z, "target_4_work", task_id=4, is_drop_point=True),
            MissionWaypoint(-2.88, -1.28, cruise_z, "target_4_recover"),
            MissionWaypoint(-4.8, -1.28, cruise_z, "special_target_pre_cruise"),
            MissionWaypoint(-4.8, -0.8, cruise_z, "special_target_cruise"),
            MissionWaypoint(-4.8, -0.8, work_z, "special_target_work", task_id=5, is_drop_point=True),
            MissionWaypoint(
                -4.8,
                -0.8,
                cruise_z,
                "special_target_recover",
                starts_ring_alignment=True,
            ),
            MissionWaypoint(-4.8, 1.6, ring_z, "ring_exit"),
            MissionWaypoint(-4.8, 1.7, cruise_z, "ring_recover"),
            MissionWaypoint(0.0, 1.7, cruise_z, "return_transfer"),
            MissionWaypoint(0.0, 1.28, cruise_z, "landing_6_overhead"),
        ]

        if self.landing_task_id == 6:
            waypoints.append(
                MissionWaypoint(0.0, 1.28, 0.0, "landing_6", task_id=6, is_land_point=True)
            )
        else:
            waypoints.extend(
                [
                    MissionWaypoint(0.0, -1.28, cruise_z, "landing_7_overhead"),
                    MissionWaypoint(0.0, -1.28, 0.0, "landing_7", task_id=7, is_land_point=True),
                ]
            )
        return waypoints

    def vehicle_local_position_callback(self, msg: VehicleLocalPosition) -> None:
        self.vehicle_local_position = msg
        if self.reference_captured or not (msg.xy_valid and msg.z_valid):
            return
        if self.now_sec() < self.reference_capture_ready_sec:
            return

        self.start_local = [float(msg.x), float(msg.y), float(msg.z)]
        self.locked_yaw = self.current_px4_yaw()
        self.task_waypoints = self.build_task_waypoints()
        self.local_waypoints = [self.task_waypoint_to_local(wp) for wp in self.task_waypoints]
        self.commanded_local = list(self.local_waypoints[0])
        self.reference_captured = True
        self.waiting_for_reference_logged = False
        self.transition_to(self.STATE_MISSION)

        self.get_logger().info(
            "Captured full mission reference: "
            f"start_local=({self.start_local[0]:.2f}, {self.start_local[1]:.2f}, {self.start_local[2]:.2f}), "
            f"locked_yaw={self.locked_yaw:.3f} rad, waypoints={len(self.task_waypoints)}"
        )
        self.log_active_waypoint()

    def vehicle_status_callback(self, msg: VehicleStatus) -> None:
        self.vehicle_status = msg
        self.armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        offboard_active = msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        if offboard_active and not self.offboard_enabled:
            self.get_logger().info("PX4 entered Offboard mode.")
        if self.offboard_enabled and not offboard_active:
            self.get_logger().warn("PX4 left Offboard mode; holding current setpoint.")
            self.waiting_for_offboard_logged = False
        self.offboard_enabled = offboard_active
        if self.armed:
            self.waiting_for_manual_arm_logged = False
        else:
            self.waiting_for_offboard_logged = False

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self.camera_info = msg

    def ring_camera_info_callback(self, msg: CameraInfo) -> None:
        self.ring_camera_info = msg

    def image_to_gray(self, msg: Image) -> Optional[np.ndarray]:
        encoding = msg.encoding.lower()
        try:
            if encoding == "rgb8":
                image = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
                return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
            if encoding == "bgr8":
                image = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
                return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            if encoding in ("mono8", "8uc1"):
                return np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
        except ValueError as exc:
            self.warn_throttled(f"Failed to parse image: {exc}")
            return None
        self.warn_throttled(f"Unsupported image encoding: {msg.encoding!r}")
        return None

    def image_callback(self, msg: Image) -> None:
        if self.state != self.STATE_VISUAL_ALIGN:
            return
        if self.camera_info is None or not self.reference_captured:
            return
        gray = self.image_to_gray(msg)
        if gray is None:
            return

        center = self.detect_square_center(gray)
        if center is None:
            return

        offset = self.pixel_to_body_offset(center[0], center[1])
        if offset is None:
            return
        if math.hypot(offset[0], offset[1]) > self.max_visual_offset:
            self.warn_throttled(
                f"Rejecting visual offset outside limit: ({offset[0]:+.2f}, {offset[1]:+.2f})"
            )
            return

        self.latest_square_pixel = center
        self.latest_visual_offset_body = offset
        self.latest_visual_time = self.now_sec()

    def ring_image_callback(self, msg: Image) -> None:
        if self.state != self.STATE_RING_ALIGN:
            return
        if self.ring_camera_info is None or not self.reference_captured:
            return
        gray = self.image_to_gray(msg)
        if gray is None:
            return

        detection = self.detect_ring_center(gray)
        if detection is None:
            return
        center_u, center_v, diameter_px = detection

        offset = self.ring_pixel_to_body_lateral_offset(center_u, diameter_px)
        if offset is None:
            return
        if abs(offset) > self.max_ring_lateral_offset:
            self.warn_throttled(
                f"Rejecting ring lateral offset outside limit: {offset:+.2f} m"
            )
            return

        self.latest_ring_pixel = (center_u, center_v)
        self.latest_ring_diameter_px = diameter_px
        self.latest_ring_lateral_offset_body = offset
        self.latest_ring_time = self.now_sec()

    def detect_square_center(self, gray: np.ndarray) -> Optional[tuple[float, float]]:
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        _, mask = cv2.threshold(blur, self.black_threshold, 255, cv2.THRESH_BINARY_INV)
        kernel = np.ones((5, 5), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.dilate(mask, kernel, iterations=1)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_center = None
        best_area = 0.0
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self.min_square_area or area > self.max_square_area:
                continue
            rect = cv2.minAreaRect(contour)
            (cx, cy), (w, h), _ = rect
            if w <= 1.0 or h <= 1.0:
                continue
            ratio = min(w, h) / max(w, h)
            if ratio < 1.0 - self.square_aspect_tol:
                continue
            if area > best_area:
                best_area = area
                best_center = (float(cx), float(cy))
        return best_center

    def detect_ring_center(self, gray: np.ndarray) -> Optional[tuple[float, float, float]]:
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        _, mask = cv2.threshold(blur, self.ring_white_threshold, 255, cv2.THRESH_BINARY)
        kernel = np.ones((5, 5), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best = None
        best_area = 0.0
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self.ring_min_area or area > self.ring_max_area:
                continue
            perimeter = float(cv2.arcLength(contour, True))
            if perimeter <= 1.0:
                continue
            circularity = 4.0 * math.pi * area / (perimeter * perimeter)
            if circularity < self.ring_min_circularity:
                continue
            (cx, cy), radius = cv2.minEnclosingCircle(contour)
            diameter_px = float(radius) * 2.0
            if diameter_px <= 2.0:
                continue
            if area > best_area:
                best_area = area
                best = (float(cx), float(cy), diameter_px)
        return best

    def current_height_up(self) -> Optional[float]:
        if self.start_local is None:
            return None
        return self.start_local[2] - float(self.vehicle_local_position.z)

    def pixel_to_body_offset(self, u: float, v: float) -> Optional[tuple[float, float]]:
        if self.camera_info is None:
            return None
        height_up = self.current_height_up()
        if height_up is None:
            return None
        camera_height = height_up + self.camera_offset_body_z_up
        if camera_height <= 0.05:
            return None
        fx = float(self.camera_info.k[0])
        fy = float(self.camera_info.k[4])
        cx = float(self.camera_info.k[2])
        cy = float(self.camera_info.k[5])
        if fx <= 0.0 or fy <= 0.0:
            return None

        camera_x = (u - cx) * camera_height / fx
        camera_y = (v - cy) * camera_height / fy
        body_x_right = camera_x + self.camera_offset_body_x_right
        body_y_forward = -camera_y + self.camera_offset_body_y_forward
        return (body_x_right, body_y_forward)

    def ring_pixel_to_body_lateral_offset(
        self, u: float, diameter_px: float
    ) -> Optional[float]:
        if self.ring_camera_info is None:
            return None
        fx = float(self.ring_camera_info.k[0])
        cx = float(self.ring_camera_info.k[2])
        if fx <= 0.0 or diameter_px <= 1.0:
            return None

        distance = fx * self.ring_diameter_m / diameter_px
        if distance < self.ring_depth_min_m or distance > self.ring_depth_max_m:
            self.warn_throttled(
                f"Rejecting ring distance estimate outside limit: {distance:.2f} m"
            )
            return None

        camera_x_right = self.ring_lateral_sign * (u - cx) * distance / fx
        return camera_x_right + self.ring_camera_offset_body_x_right

    def task_waypoint_to_local(self, waypoint: MissionWaypoint) -> list[float]:
        if self.start_local is None:
            return [0.0, 0.0, 0.0]

        x_right = self.task_x_sign * waypoint.x_right
        y_forward = self.task_y_sign * waypoint.y_forward
        z_up = self.task_z_sign * waypoint.z_up

        if self.use_initial_heading_frame:
            c = math.cos(self.locked_yaw)
            s = math.sin(self.locked_yaw)
            dx = y_forward * c - x_right * s
            dy = y_forward * s + x_right * c
        else:
            dx = y_forward
            dy = x_right

        return [
            self.start_local[0] + dx,
            self.start_local[1] + dy,
            self.start_local[2] - z_up,
        ]

    def body_offset_to_local_delta(self, x_right: float, y_forward: float) -> tuple[float, float]:
        if self.use_initial_heading_frame:
            c = math.cos(self.locked_yaw)
            s = math.sin(self.locked_yaw)
            return (y_forward * c - x_right * s, y_forward * s + x_right * c)
        return (y_forward, x_right)

    def transition_to(self, new_state: str) -> None:
        if self.state == new_state:
            return
        old_state = self.state
        self.state = new_state
        if new_state in (self.STATE_VISUAL_ALIGN, self.STATE_RING_ALIGN):
            self.visual_stable_cycles = 0
            self.ring_stable_cycles = 0
        self.get_logger().info(f"Mission state: {old_state} -> {new_state}")

    def active_waypoint(self) -> MissionWaypoint:
        return self.task_waypoints[self.waypoint_index]

    def active_target(self) -> list[float]:
        return self.local_waypoints[self.waypoint_index]

    def log_active_waypoint(self) -> None:
        waypoint = self.active_waypoint()
        target = self.active_target()
        self.get_logger().info(
            f"Waypoint {self.waypoint_index + 1}/{len(self.task_waypoints)} "
            f"{waypoint.label} task=({waypoint.x_right:.2f}, {waypoint.y_forward:.2f}, {waypoint.z_up:.2f}) "
            f"local=({target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f})"
        )

    def reset_waypoint_hold_state(self) -> None:
        self.stable_cycles = 0
        self.hold_start_sec = None

    def reset_action_state(self) -> None:
        self.action_started = False
        self.action_completed = False
        self.action_failed = False
        self.action_start_sec = None
        self.action_process = None

    def reset_visual_alignment_state(self) -> None:
        self.visual_stable_cycles = 0
        self.visual_base_local = None
        self.visual_correction_local_xy = None
        self.latest_visual_offset_body = None
        self.latest_square_pixel = None
        self.latest_visual_time = 0.0

    def reset_ring_alignment_state(self) -> None:
        self.ring_stable_cycles = 0
        self.ring_base_local = None
        self.ring_correction_local_xy = None
        self.latest_ring_lateral_offset_body = None
        self.latest_ring_pixel = None
        self.latest_ring_diameter_px = 0.0
        self.latest_ring_time = 0.0

    def advance_waypoint(self) -> None:
        if self.waypoint_index >= len(self.task_waypoints) - 1:
            self.complete_mission()
            return

        old = self.active_waypoint()
        self.waypoint_index += 1
        self.reset_waypoint_hold_state()
        self.reset_action_state()
        self.reset_visual_alignment_state()
        self.reset_ring_alignment_state()
        new = self.active_waypoint()
        self.get_logger().info(f"Waypoint complete: {old.label}. Advancing to {new.label}.")
        self.log_active_waypoint()

    def complete_mission(self) -> None:
        self.transition_to(self.STATE_COMPLETED)
        if not self.completion_logged:
            self.completion_logged = True
            self.get_logger().info(
                "Full mission completed. Holding final landing setpoint; disarm manually."
            )

    def hold_seconds_for_active_waypoint(self) -> float:
        waypoint = self.active_waypoint()
        if waypoint.is_land_point:
            return self.land_hover_sec
        if waypoint.is_drop_point:
            return self.drop_hover_sec
        return self.default_hover_sec

    def active_waypoint_needs_action(self) -> bool:
        waypoint = self.active_waypoint()
        if waypoint.is_drop_point:
            return waypoint.task_id in self.drop_angle_by_task
        return False

    def active_waypoint_needs_visual_alignment(self) -> bool:
        waypoint = self.active_waypoint()
        return (
            self.enable_d435i_drop_alignment
            and waypoint.is_drop_point
            and waypoint.task_id in self.drop_angle_by_task
        )

    def active_waypoint_needs_ring_alignment(self) -> bool:
        waypoint = self.active_waypoint()
        return self.enable_d435i_ring_alignment and waypoint.starts_ring_alignment

    def start_visual_alignment(self) -> None:
        target = self.active_target()
        self.visual_base_local = list(target)
        self.visual_correction_local_xy = [0.0, 0.0]
        self.latest_visual_offset_body = None
        self.latest_square_pixel = None
        self.latest_visual_time = 0.0
        self.visual_stable_cycles = 0
        self.start_d435i_driver_if_needed()
        self.transition_to(self.STATE_VISUAL_ALIGN)
        waypoint = self.active_waypoint()
        self.get_logger().info(
            f"Starting D435i XY alignment at task {waypoint.task_id}; "
            "Point-LIO remains the position source, D435i only corrects this waypoint setpoint."
        )

    def start_ring_alignment(self) -> None:
        target = self.active_target()
        self.ring_base_local = list(target)
        self.ring_correction_local_xy = [0.0, 0.0]
        self.latest_ring_lateral_offset_body = None
        self.latest_ring_pixel = None
        self.latest_ring_diameter_px = 0.0
        self.latest_ring_time = 0.0
        self.ring_stable_cycles = 0
        self.start_ring_d435i_driver_if_needed()
        self.transition_to(self.STATE_RING_ALIGN)
        self.get_logger().info(
            "Starting front D435i ring lateral alignment; "
            "Point-LIO remains the position source, front D435i only corrects lateral setpoint."
        )

    def start_d435i_driver_if_needed(self) -> None:
        if not self.start_d435i_driver_on_alignment or self.d435i_start_requested:
            return
        self.d435i_start_requested = True
        command = [
            "ros2",
            "launch",
            "realsense2_camera",
            "rs_launch.py",
            f"camera_namespace:={self.d435i_camera_namespace}",
            f"camera_name:={self.d435i_camera_name}",
            "enable_color:=true",
            "enable_depth:=true",
            "align_depth.enable:=true",
            "pointcloud.enable:=false",
            "enable_gyro:=false",
            "enable_accel:=false",
            f"rgb_camera.color_profile:={self.d435i_color_profile}",
            f"depth_module.depth_profile:={self.d435i_depth_profile}",
        ]
        if self.d435i_serial_no:
            command.append(self.realsense_serial_launch_arg(self.d435i_serial_no))
        try:
            self.d435i_process = subprocess.Popen(command)
        except OSError as exc:
            self.get_logger().error(f"Failed to start D435i driver: {exc}")
            return
        self.get_logger().info(f"D435i driver launch started: {' '.join(command)}")

    def start_ring_d435i_driver_if_needed(self) -> None:
        if not self.start_ring_d435i_driver_on_alignment or self.ring_d435i_start_requested:
            return
        self.ring_d435i_start_requested = True
        command = [
            "ros2",
            "launch",
            "realsense2_camera",
            "rs_launch.py",
            f"camera_namespace:={self.ring_d435i_camera_namespace}",
            f"camera_name:={self.ring_d435i_camera_name}",
            "enable_color:=true",
            "enable_depth:=false",
            "align_depth.enable:=false",
            "pointcloud.enable:=false",
            "enable_gyro:=false",
            "enable_accel:=false",
            f"rgb_camera.color_profile:={self.ring_d435i_color_profile}",
        ]
        if self.ring_d435i_serial_no:
            command.append(self.realsense_serial_launch_arg(self.ring_d435i_serial_no))
        try:
            self.ring_d435i_process = subprocess.Popen(command)
        except OSError as exc:
            self.get_logger().error(f"Failed to start front D435i driver: {exc}")
            return
        self.get_logger().info(f"Front D435i driver launch started: {' '.join(command)}")

    def realsense_serial_launch_arg(self, serial_no: str) -> str:
        return f'serial_no:="{serial_no}"'

    def stop_ring_d435i_driver_if_running(self) -> None:
        if self.ring_d435i_process is None:
            return
        if self.ring_d435i_process.poll() is None:
            self.ring_d435i_process.terminate()
            self.get_logger().info("Front D435i driver stopped after ring alignment.")

    def warn_if_d435i_process_exited(self) -> None:
        if self.d435i_process is None:
            return
        return_code = self.d435i_process.poll()
        if return_code is not None:
            self.warn_throttled(
                f"D435i driver launch process exited with code {return_code}. "
                "Check camera USB connection and realsense2_camera logs."
            )

    def warn_if_ring_d435i_process_exited(self) -> None:
        if self.ring_d435i_process is None:
            return
        return_code = self.ring_d435i_process.poll()
        if return_code is not None:
            self.warn_throttled(
                f"Front D435i driver launch process exited with code {return_code}. "
                "Check camera USB connection and realsense2_camera logs."
            )

    def publish_offboard_control_mode(self) -> None:
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.actuator = False
        msg.timestamp = self.now_us()
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self) -> None:
        if self.commanded_local is None:
            return

        msg = TrajectorySetpoint()
        msg.position = [
            float(self.commanded_local[0]),
            float(self.commanded_local[1]),
            float(self.commanded_local[2]),
        ]
        msg.yaw = float(self.locked_yaw)
        msg.timestamp = self.now_us()
        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command: int, **params: float) -> None:
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.now_us()
        self.vehicle_command_pub.publish(msg)

    def handle_offboard_entry(self) -> None:
        if not self.reference_captured:
            return

        if self.offboard_setpoint_counter < self.offboard_prestream_cycles:
            self.offboard_setpoint_counter += 1
            return

        now_us = self.now_us()
        if not self.armed:
            if self.auto_arm and now_us - self.last_arm_request_us >= 1_000_000:
                self.publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0
                )
                self.last_arm_request_us = now_us
                self.get_logger().info("Arm command sent by ROS because auto_arm=true.")
            elif not self.auto_arm and not self.waiting_for_manual_arm_logged:
                self.waiting_for_manual_arm_logged = True
                self.get_logger().info(
                    "Waiting for manual arm from RC before requesting Offboard mode."
                )
            return

        if not self.offboard_enabled and now_us - self.last_offboard_request_us >= 1_000_000:
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0
            )
            self.last_offboard_request_us = now_us
            self.waiting_for_offboard_logged = False
            self.get_logger().info("Offboard mode command sent.")
        elif not self.offboard_enabled and not self.waiting_for_offboard_logged:
            self.waiting_for_offboard_logged = True
            self.get_logger().info("Waiting for PX4 to enter Offboard mode.")

    def move_towards_xy_z(self, current: list[float], target: list[float]) -> list[float]:
        dx = target[0] - current[0]
        dy = target[1] - current[1]
        dz = target[2] - current[2]

        xy_dist = math.hypot(dx, dy)
        xy_step = self.approach_speed * self.timer_period
        if xy_dist <= xy_step or xy_dist <= 1.0e-6:
            next_x = target[0]
            next_y = target[1]
        else:
            scale = xy_step / xy_dist
            next_x = current[0] + dx * scale
            next_y = current[1] + dy * scale

        z_step = self.vertical_speed * self.timer_period
        if abs(dz) <= z_step:
            next_z = target[2]
        elif dz > 0.0:
            next_z = current[2] + z_step
        else:
            next_z = current[2] - z_step

        return [next_x, next_y, next_z]

    def move_towards_visual_target(self, current: list[float], target: list[float]) -> list[float]:
        dx = target[0] - current[0]
        dy = target[1] - current[1]
        dz = target[2] - current[2]

        xy_dist = math.hypot(dx, dy)
        xy_step = self.visual_correction_speed * self.timer_period
        if xy_dist <= xy_step or xy_dist <= 1.0e-6:
            next_x = target[0]
            next_y = target[1]
        else:
            scale = xy_step / xy_dist
            next_x = current[0] + dx * scale
            next_y = current[1] + dy * scale

        z_step = self.vertical_speed * self.timer_period
        if abs(dz) <= z_step:
            next_z = target[2]
        elif dz > 0.0:
            next_z = current[2] + z_step
        else:
            next_z = current[2] - z_step

        return [next_x, next_y, next_z]

    def move_towards_ring_target(self, current: list[float], target: list[float]) -> list[float]:
        dx = target[0] - current[0]
        dy = target[1] - current[1]
        dz = target[2] - current[2]

        xy_dist = math.hypot(dx, dy)
        xy_step = self.ring_visual_correction_speed * self.timer_period
        if xy_dist <= xy_step or xy_dist <= 1.0e-6:
            next_x = target[0]
            next_y = target[1]
        else:
            scale = xy_step / xy_dist
            next_x = current[0] + dx * scale
            next_y = current[1] + dy * scale

        z_step = self.vertical_speed * self.timer_period
        if abs(dz) <= z_step:
            next_z = target[2]
        elif dz > 0.0:
            next_z = current[2] + z_step
        else:
            next_z = current[2] - z_step

        return [next_x, next_y, next_z]

    def clamp_xy_vector(self, x: float, y: float, limit: float) -> tuple[float, float]:
        norm = math.hypot(x, y)
        if norm <= limit or norm <= 1.0e-6:
            return (x, y)
        scale = limit / norm
        return (x * scale, y * scale)

    def limited_visual_xy_target(self) -> Optional[list[float]]:
        if (
            self.latest_visual_offset_body is None
            or self.commanded_local is None
            or self.visual_base_local is None
        ):
            return None
        if self.now_sec() - self.latest_visual_time > 0.5:
            return None

        dx_local, dy_local = self.body_offset_to_local_delta(
            self.latest_visual_offset_body[0], self.latest_visual_offset_body[1]
        )

        estimated_target_x = float(self.vehicle_local_position.x) + dx_local
        estimated_target_y = float(self.vehicle_local_position.y) + dy_local
        desired_correction_x = estimated_target_x - self.visual_base_local[0]
        desired_correction_y = estimated_target_y - self.visual_base_local[1]
        desired_correction_x, desired_correction_y = self.clamp_xy_vector(
            desired_correction_x,
            desired_correction_y,
            self.max_visual_correction_total,
        )

        if self.visual_correction_local_xy is None:
            self.visual_correction_local_xy = [0.0, 0.0]

        previous_x = self.visual_correction_local_xy[0]
        previous_y = self.visual_correction_local_xy[1]
        proposed_x = previous_x + self.visual_offset_lowpass_alpha * (
            desired_correction_x - previous_x
        )
        proposed_y = previous_y + self.visual_offset_lowpass_alpha * (
            desired_correction_y - previous_y
        )
        step_x, step_y = self.clamp_xy_vector(
            proposed_x - previous_x,
            proposed_y - previous_y,
            self.max_visual_step,
        )
        correction_x = previous_x + step_x
        correction_y = previous_y + step_y
        correction_x, correction_y = self.clamp_xy_vector(
            correction_x,
            correction_y,
            self.max_visual_correction_total,
        )
        self.visual_correction_local_xy = [correction_x, correction_y]

        return [
            self.visual_base_local[0] + correction_x,
            self.visual_base_local[1] + correction_y,
            self.visual_base_local[2],
        ]

    def limited_ring_lateral_target(self) -> Optional[list[float]]:
        if (
            self.latest_ring_lateral_offset_body is None
            or self.commanded_local is None
            or self.ring_base_local is None
        ):
            return None
        if self.now_sec() - self.latest_ring_time > 0.5:
            return None

        dx_local, dy_local = self.body_offset_to_local_delta(
            self.latest_ring_lateral_offset_body, 0.0
        )
        desired_correction_x = dx_local
        desired_correction_y = dy_local
        desired_correction_x, desired_correction_y = self.clamp_xy_vector(
            desired_correction_x,
            desired_correction_y,
            self.max_ring_correction_total,
        )

        if self.ring_correction_local_xy is None:
            self.ring_correction_local_xy = [0.0, 0.0]

        previous_x = self.ring_correction_local_xy[0]
        previous_y = self.ring_correction_local_xy[1]
        proposed_x = previous_x + self.ring_offset_lowpass_alpha * (
            desired_correction_x - previous_x
        )
        proposed_y = previous_y + self.ring_offset_lowpass_alpha * (
            desired_correction_y - previous_y
        )
        step_x, step_y = self.clamp_xy_vector(
            proposed_x - previous_x,
            proposed_y - previous_y,
            self.max_ring_step,
        )
        correction_x = previous_x + step_x
        correction_y = previous_y + step_y
        correction_x, correction_y = self.clamp_xy_vector(
            correction_x,
            correction_y,
            self.max_ring_correction_total,
        )
        self.ring_correction_local_xy = [correction_x, correction_y]

        return [
            self.ring_base_local[0] + correction_x,
            self.ring_base_local[1] + correction_y,
            self.ring_base_local[2],
        ]

    def apply_ring_correction_to_passage(self, aligned_target: list[float]) -> None:
        if self.ring_base_local is None:
            return
        dx = aligned_target[0] - self.ring_base_local[0]
        dy = aligned_target[1] - self.ring_base_local[1]
        for index in range(self.waypoint_index + 1, len(self.task_waypoints)):
            label = self.task_waypoints[index].label
            if label not in ("ring_exit", "ring_recover"):
                continue
            self.local_waypoints[index][0] += dx
            self.local_waypoints[index][1] += dy

    def target_is_stable(self, target: list[float]) -> bool:
        dx = float(self.vehicle_local_position.x) - target[0]
        dy = float(self.vehicle_local_position.y) - target[1]
        dz = float(self.vehicle_local_position.z) - target[2]
        speed_xy = math.hypot(
            float(self.vehicle_local_position.vx),
            float(self.vehicle_local_position.vy),
        )
        speed_z = abs(float(self.vehicle_local_position.vz))
        return (
            math.hypot(dx, dy) <= self.reach_xy_tol
            and abs(dz) <= self.reach_z_tol
            and speed_xy <= self.speed_xy_tol
            and speed_z <= self.speed_z_tol
        )

    def count_stable_or_reset(self, target: list[float]) -> bool:
        if self.target_is_stable(target):
            self.stable_cycles += 1
        else:
            self.stable_cycles = 0
            self.hold_start_sec = None
        return self.stable_cycles >= self.stable_cycles_required

    def start_active_action(self) -> None:
        waypoint = self.active_waypoint()
        if waypoint.is_drop_point:
            angle = self.drop_angle_by_task.get(waypoint.task_id)
            if angle is None:
                self.action_completed = True
                return
            command = [
                "python3",
                self.drop_command,
                "--port",
                self.drop_port,
                "--angle",
                f"{angle:g}",
            ]
            self.get_logger().info(
                f"Executing drop action at task {waypoint.task_id}: {' '.join(command)}"
            )
            try:
                self.action_process = subprocess.Popen(
                    command,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                )
            except OSError as exc:
                self.action_failed = True
                self.action_completed = True
                self.get_logger().error(f"Failed to start drop action at task {waypoint.task_id}: {exc}")
                return
            self.action_start_sec = self.now_sec()

    def poll_active_action(self) -> None:
        waypoint = self.active_waypoint()
        if self.action_process is None:
            self.action_failed = True
            self.action_completed = True
            self.get_logger().error(f"Action process missing at waypoint {waypoint.label}.")
            return

        if self.action_start_sec is not None:
            elapsed = self.now_sec() - self.action_start_sec
            if elapsed > self.action_timeout_sec:
                self.action_process.kill()
                stdout, stderr = self.action_process.communicate()
                self.action_failed = True
                self.action_completed = True
                if stdout.strip():
                    self.get_logger().info(f"Drop stdout before timeout: {stdout.strip()}")
                if stderr.strip():
                    self.get_logger().warn(f"Drop stderr before timeout: {stderr.strip()}")
                self.get_logger().error(
                    f"Drop action timed out at task {waypoint.task_id} after "
                    f"{self.action_timeout_sec:.1f} s."
                )
                return

        returncode = self.action_process.poll()
        if returncode is None:
            return

        stdout, stderr = self.action_process.communicate()
        if stdout.strip():
            self.get_logger().info(f"Drop stdout: {stdout.strip()}")
        if stderr.strip():
            self.get_logger().warn(f"Drop stderr: {stderr.strip()}")
        if returncode != 0:
            self.action_failed = True
            self.get_logger().error(
                f"Drop action failed at task {waypoint.task_id}: returncode={returncode}"
            )
        else:
            self.get_logger().info(f"Drop action completed at task {waypoint.task_id}.")
        self.action_completed = True

    def update_action_state(self) -> None:
        if not self.active_waypoint_needs_action():
            self.action_completed = True
            self.transition_to(self.STATE_MISSION)
            self.advance_waypoint()
            return
        if not self.action_started:
            self.action_started = True
            self.start_active_action()
        else:
            self.poll_active_action()
        if self.action_completed:
            self.transition_to(self.STATE_MISSION)
            self.advance_waypoint()

    def update_visual_align_state(self) -> None:
        visual_target = self.limited_visual_xy_target()
        if visual_target is None:
            self.warn_if_d435i_process_exited()
            self.warn_throttled("Waiting for D435i black-square target detection at selected drop point.")
            self.visual_stable_cycles = 0
            return

        self.commanded_local = self.move_towards_visual_target(self.commanded_local, visual_target)
        offset_norm = math.hypot(*self.latest_visual_offset_body)
        if offset_norm <= self.visual_align_tol:
            self.visual_stable_cycles += 1
        else:
            self.visual_stable_cycles = 0

        if self.visual_stable_cycles < self.visual_stable_cycles_required:
            return

        aligned_target = list(visual_target)
        self.commanded_local = aligned_target
        self.local_waypoints[self.waypoint_index] = aligned_target
        self.reset_waypoint_hold_state()
        self.reset_action_state()
        self.reset_visual_alignment_state()
        self.get_logger().info(
            f"D435i alignment complete at task {self.active_waypoint().task_id}; "
            f"corrected_local=({aligned_target[0]:+.2f}, {aligned_target[1]:+.2f}, {aligned_target[2]:+.2f})."
        )
        self.transition_to(self.STATE_ACTION)

    def update_ring_align_state(self) -> None:
        ring_target = self.limited_ring_lateral_target()
        if ring_target is None:
            self.warn_if_ring_d435i_process_exited()
            self.warn_throttled("Waiting for front D435i white-ring lateral detection.")
            self.ring_stable_cycles = 0
            return

        self.commanded_local = self.move_towards_ring_target(self.commanded_local, ring_target)
        offset_abs = abs(self.latest_ring_lateral_offset_body)
        if offset_abs <= self.ring_align_tol:
            self.ring_stable_cycles += 1
        else:
            self.ring_stable_cycles = 0

        if self.ring_stable_cycles < self.ring_stable_cycles_required:
            return

        aligned_target = list(ring_target)
        self.commanded_local = aligned_target
        self.local_waypoints[self.waypoint_index] = aligned_target
        self.apply_ring_correction_to_passage(aligned_target)
        self.reset_waypoint_hold_state()
        self.reset_ring_alignment_state()
        self.stop_ring_d435i_driver_if_running()
        self.get_logger().info(
            "Front D435i ring alignment complete; "
            f"corrected_local=({aligned_target[0]:+.2f}, {aligned_target[1]:+.2f}, {aligned_target[2]:+.2f}). "
            "Front vision will not be used while passing through the ring."
        )
        self.transition_to(self.STATE_MISSION)
        self.advance_waypoint()

    def update_mission_target(self) -> None:
        if not self.reference_captured or self.commanded_local is None:
            return

        if self.state == self.STATE_COMPLETED:
            self.commanded_local = list(self.local_waypoints[-1])
            return

        if not self.offboard_enabled:
            return

        if self.state == self.STATE_ACTION:
            self.commanded_local = list(self.active_target())
            self.update_action_state()
            return

        if self.state == self.STATE_VISUAL_ALIGN:
            self.update_visual_align_state()
            return

        if self.state == self.STATE_RING_ALIGN:
            self.update_ring_align_state()
            return

        target = self.active_target()
        self.commanded_local = self.move_towards_xy_z(self.commanded_local, target)

        if not self.count_stable_or_reset(target):
            return

        if self.hold_start_sec is None:
            self.hold_start_sec = self.now_sec()
            self.get_logger().info(
                f"Waypoint {self.active_waypoint().label} stable; "
                f"holding for {self.hold_seconds_for_active_waypoint():.1f} s."
            )

        if self.now_sec() - self.hold_start_sec < self.hold_seconds_for_active_waypoint():
            return

        self.commanded_local = list(target)
        if self.active_waypoint_needs_action():
            if self.active_waypoint_needs_visual_alignment():
                self.start_visual_alignment()
            else:
                self.transition_to(self.STATE_ACTION)
            return
        if self.active_waypoint_needs_ring_alignment():
            self.start_ring_alignment()
            return
        self.advance_waypoint()

    def timer_callback(self) -> None:
        self.publish_offboard_control_mode()

        if not self.reference_captured:
            if not self.waiting_for_reference_logged:
                self.waiting_for_reference_logged = True
                self.get_logger().info(
                    "Waiting for valid PX4 local position before publishing setpoints."
                )
            return

        self.update_mission_target()
        self.publish_trajectory_setpoint()
        self.handle_offboard_entry()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FullMissionController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.d435i_process is not None and node.d435i_process.poll() is None:
            node.d435i_process.terminate()
        if node.ring_d435i_process is not None and node.ring_d435i_process.poll() is None:
            node.ring_d435i_process.terminate()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
