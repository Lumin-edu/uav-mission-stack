#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


@dataclass(frozen=True)
class SquareWaypoint:
    x_right: float
    y_forward: float
    z_up: float
    label: str


class SquareMissionController(Node):
    STATE_WAITING_REFERENCE = "waiting_reference"
    STATE_MISSION = "mission"
    STATE_COMPLETED = "completed"

    def __init__(self) -> None:
        super().__init__("square_mission_controller")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.auto_arm = self.param_bool("auto_arm", False)
        self.control_rate_hz = self.param_float("control_rate_hz", 50.0)
        self.reference_capture_delay_sec = self.param_float("reference_capture_delay_sec", 3.0)
        self.takeoff_altitude = self.param_float("takeoff_altitude", 0.30)
        self.square_side_length = self.param_float("square_side_length", 1.0)
        self.approach_speed = self.param_float("approach_speed", 1.0)
        self.vertical_speed = self.param_float("vertical_speed", 0.20)
        self.reach_xy_tol = self.param_float("reach_xy_tol", 0.20)
        self.reach_z_tol = self.param_float("reach_z_tol", 0.10)
        self.speed_xy_tol = self.param_float("speed_xy_tol", 0.20)
        self.speed_z_tol = self.param_float("speed_z_tol", 0.15)
        self.stable_time_sec = self.param_float("stable_time_sec", 0.6)
        self.takeoff_hover_sec = self.param_float("takeoff_hover_sec", 0.6)
        self.corner_hover_sec = self.param_float("corner_hover_sec", 0.6)
        self.final_hover_sec = self.param_float("final_hover_sec", 0.6)
        self.use_initial_heading_frame = self.param_bool("use_initial_heading_frame", True)
        self.task_x_sign = self.param_axis_sign("task_x_sign", 1.0)
        self.task_y_sign = self.param_axis_sign("task_y_sign", 1.0)
        self.task_z_sign = self.param_axis_sign("task_z_sign", 1.0)
        self.vehicle_local_position_topic = self.param_string(
            "vehicle_local_position_topic", "/fmu/out/vehicle_local_position"
        )
        self.vehicle_status_topic = self.param_string(
            "vehicle_status_topic", "/fmu/out/vehicle_status"
        )

        self.validate_parameters()

        self.timer_period = 1.0 / self.control_rate_hz
        self.offboard_prestream_cycles = max(10, int(2.0 * self.control_rate_hz))
        self.stable_cycles_required = max(1, int(self.stable_time_sec * self.control_rate_hz))

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
        self.task_waypoints: list[SquareWaypoint] = []
        self.local_waypoints: list[list[float]] = []
        self.commanded_local: Optional[list[float]] = None
        self.locked_yaw = 0.0
        self.waypoint_index = 0
        self.stable_cycles = 0
        self.hold_start_sec: Optional[float] = None

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info(
            "Square mission controller ready. "
            f"takeoff_altitude={self.takeoff_altitude:.2f} m, "
            f"square_side_length={self.square_side_length:.2f} m, "
            f"yaw=locked_to_initial_heading, auto_arm={self.auto_arm}, "
            f"approach_speed={self.approach_speed:.2f}, vertical_speed={self.vertical_speed:.2f}, "
            f"reach_xy_tol={self.reach_xy_tol:.2f}, "
            f"task_signs=({self.task_x_sign:+.0f}, {self.task_y_sign:+.0f}, {self.task_z_sign:+.0f})"
        )

    def param_float(self, name: str, default: float) -> float:
        value = self.declare_parameter(name, default).value
        try:
            return float(value)
        except (TypeError, ValueError):
            self.get_logger().warn(f"Parameter {name}={value!r} is not a float; using {default}.")
            return float(default)

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

    def validate_parameters(self) -> None:
        if self.control_rate_hz <= 0.0:
            self.get_logger().warn("control_rate_hz must be positive; using 50 Hz.")
            self.control_rate_hz = 50.0
        if self.reference_capture_delay_sec < 0.0:
            self.get_logger().warn("reference_capture_delay_sec cannot be negative; using 0 s.")
            self.reference_capture_delay_sec = 0.0
        if self.takeoff_altitude <= 0.0:
            self.get_logger().warn("takeoff_altitude must be positive; using 0.30 m.")
            self.takeoff_altitude = 0.30
        if self.square_side_length <= 0.0:
            self.get_logger().warn("square_side_length must be positive; using 1.0 m.")
            self.square_side_length = 1.0
        if self.approach_speed <= 0.01:
            self.get_logger().warn("approach_speed is too small; using 1.0 m/s.")
            self.approach_speed = 1.0
        if self.vertical_speed <= 0.01:
            self.get_logger().warn("vertical_speed is too small; using 0.20 m/s.")
            self.vertical_speed = 0.20
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
        if self.takeoff_hover_sec < 0.0:
            self.takeoff_hover_sec = 0.0
        if self.corner_hover_sec < 0.0:
            self.corner_hover_sec = 0.0
        if self.final_hover_sec < 0.0:
            self.final_hover_sec = 0.0

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

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

    def build_task_waypoints(self) -> list[SquareWaypoint]:
        side = self.square_side_length
        z = self.takeoff_altitude
        return [
            SquareWaypoint(0.0, 0.0, z, "takeoff_hover"),
            SquareWaypoint(0.0, side, z, "front_left_corner"),
            SquareWaypoint(side, side, z, "front_right_corner"),
            SquareWaypoint(side, 0.0, z, "rear_right_corner"),
            SquareWaypoint(0.0, 0.0, z, "return_takeoff_hover"),
            SquareWaypoint(0.0, 0.0, 0.0, "land_at_origin"),
        ]

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
            "Captured square mission reference: "
            f"start_local=({self.start_local[0]:.2f}, {self.start_local[1]:.2f}, {self.start_local[2]:.2f}), "
            f"locked_yaw={self.locked_yaw:.3f} rad"
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

    def task_waypoint_to_local(self, waypoint: SquareWaypoint) -> list[float]:
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

    def transition_to(self, new_state: str) -> None:
        if self.state == new_state:
            return
        old_state = self.state
        self.state = new_state
        self.get_logger().info(f"Mission state: {old_state} -> {new_state}")

    def active_waypoint(self) -> SquareWaypoint:
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

    def advance_waypoint(self) -> None:
        if self.waypoint_index >= len(self.task_waypoints) - 1:
            self.complete_mission()
            return

        old = self.active_waypoint()
        self.waypoint_index += 1
        self.reset_waypoint_hold_state()
        new = self.active_waypoint()
        self.get_logger().info(
            f"Waypoint complete: {old.label}. Advancing to {new.label}."
        )
        self.log_active_waypoint()

    def complete_mission(self) -> None:
        self.transition_to(self.STATE_COMPLETED)
        if not self.completion_logged:
            self.completion_logged = True
            self.get_logger().info(
                "Square mission completed. Holding final origin landing setpoint."
            )

    def hold_seconds_for_active_waypoint(self) -> float:
        if self.waypoint_index == 0:
            return self.takeoff_hover_sec
        if self.waypoint_index >= len(self.task_waypoints) - 1:
            return self.final_hover_sec
        return self.corner_hover_sec

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

    def update_mission_target(self) -> None:
        if not self.reference_captured or self.commanded_local is None:
            return

        if self.state == self.STATE_COMPLETED:
            self.commanded_local = list(self.local_waypoints[-1])
            return

        if not self.offboard_enabled:
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

        if self.now_sec() - self.hold_start_sec >= self.hold_seconds_for_active_waypoint():
            self.commanded_local = list(target)
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
    node = SquareMissionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
