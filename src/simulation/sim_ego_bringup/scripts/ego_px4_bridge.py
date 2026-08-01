#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from quadrotor_msgs.msg import PositionCommand
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from traj_utils.msg import Bspline


class SimEgoPx4Bridge(Node):
    """Simulation-only EGO trajectory bridge.

    This implementation intentionally lives in the simulation package. It has
    the same ROS-FLU/PX4-NED contract as the hardware bridge,
    but its automatic SITL entry policy is Offboard first, then arm. No
    hardware package or hardware confirmation is imported here.
    """

    def __init__(self) -> None:
        super().__init__("ego_px4_bridge")

        self.output_enabled = bool(self.declare_parameter("output_enabled", True).value)
        self.auto_arm = bool(self.declare_parameter("auto_arm", False).value)
        self.auto_offboard = bool(self.declare_parameter("auto_offboard", False).value)
        self.control_mode = str(
            self.declare_parameter("control_mode", "position").value
        ).strip().lower()
        if self.control_mode == "position_velocity":
            self.get_logger().warn(
                "control_mode='position_velocity' is deprecated; use 'position'."
            )
            self.control_mode = "position"
        self.control_rate_hz = float(self.declare_parameter("control_rate_hz", 50.0).value)
        self.offboard_prestream_sec = float(
            self.declare_parameter("offboard_prestream_sec", 2.0).value
        )
        self.command_timeout_sec = float(
            self.declare_parameter("command_timeout_sec", 0.30).value
        )
        self.odom_timeout_sec = float(
            self.declare_parameter("odom_timeout_sec", 0.30).value
        )
        self.velocity_position_gain = float(
            self.declare_parameter("velocity_position_gain", 1.0).value
        )
        self.velocity_limit = float(self.declare_parameter("velocity_limit", 1.0).value)
        self.takeoff_before_ego = bool(
            self.declare_parameter("takeoff_before_ego", True).value
        )
        self.takeoff_altitude = float(
            self.declare_parameter("takeoff_altitude", 1.0).value
        )
        self.takeoff_vertical_speed = float(
            self.declare_parameter("takeoff_vertical_speed", 0.3).value
        )
        self.takeoff_reach_xy_tol = float(
            self.declare_parameter("takeoff_reach_xy_tol", 0.20).value
        )
        self.takeoff_reach_z_tol = float(
            self.declare_parameter("takeoff_reach_z_tol", 0.10).value
        )
        self.takeoff_speed_xy_tol = float(
            self.declare_parameter("takeoff_speed_xy_tol", 0.20).value
        )
        self.takeoff_speed_z_tol = float(
            self.declare_parameter("takeoff_speed_z_tol", 0.15).value
        )
        self.takeoff_stable_sec = float(
            self.declare_parameter("takeoff_stable_sec", 0.8).value
        )
        self.reference_capture_delay_sec = float(
            self.declare_parameter("reference_capture_delay_sec", 3.0).value
        )
        self.takeoff_ready_topic = str(
            self.declare_parameter("takeoff_ready_topic", "/ego/takeoff_ready").value
        )
        self.localization_health_topic = str(
            self.declare_parameter("localization_health_topic", "").value
        )
        if self.control_mode not in {"position", "velocity"}:
            raise ValueError("control_mode must be 'position' or 'velocity'")
        if self.control_rate_hz <= 0.0 or self.offboard_prestream_sec <= 0.0:
            raise ValueError("control_rate_hz and offboard_prestream_sec must be positive")
        if self.velocity_position_gain < 0.0 or self.velocity_limit <= 0.0:
            raise ValueError("velocity_position_gain must be non-negative and velocity_limit positive")
        if self.takeoff_altitude <= 0.0 or self.takeoff_vertical_speed <= 0.0:
            raise ValueError("takeoff_altitude and takeoff_vertical_speed must be positive")
        if min(
            self.takeoff_reach_xy_tol,
            self.takeoff_reach_z_tol,
            self.takeoff_speed_xy_tol,
            self.takeoff_speed_z_tol,
            self.takeoff_stable_sec,
            self.reference_capture_delay_sec,
        ) < 0.0:
            raise ValueError("takeoff tolerances and timing parameters must be non-negative")

        self.world_y_to_px4_y_sign = float(
            self.declare_parameter("world_y_to_px4_y_sign", -1.0).value
        )
        self.world_z_to_px4_z_sign = float(
            self.declare_parameter("world_z_to_px4_z_sign", -1.0).value
        )
        rotation = list(self.declare_parameter("world_to_px4_rotation", [0.0]).value)
        if len(rotation) == 1 and float(rotation[0]) == 0.0:
            rotation = []
        if rotation and len(rotation) != 9:
            raise ValueError("world_to_px4_rotation must contain 9 row-major values")
        self.world_to_px4_rotation = [
            float(value)
            for value in (
                rotation
                if rotation
                else [
                    1.0,
                    0.0,
                    0.0,
                    0.0,
                    self.world_y_to_px4_y_sign,
                    0.0,
                    0.0,
                    0.0,
                    self.world_z_to_px4_z_sign,
                ]
            )
        ]
        self.yaw_sign = float(self.declare_parameter("yaw_sign", 1.0).value)
        self.yaw_offset = float(
            self.declare_parameter("yaw_offset", 0.0).value
        )
        self.use_initial_heading_frame = bool(
            self.declare_parameter("use_initial_heading_frame", True).value
        )
        self.lock_yaw_to_initial_heading = bool(
            self.declare_parameter("lock_yaw_to_initial_heading", True).value
        )
        self.align_to_px4_reference = bool(
            self.declare_parameter("align_to_px4_reference", True).value
        )

        self.ego_command_topic = str(
            self.declare_parameter("ego_command_topic", "/ego/position_cmd").value
        )
        self.planner_trajectory_topic = str(
            self.declare_parameter(
                "planner_trajectory_topic", "/ego/planning/bspline"
            ).value
        )
        self.planner_trajectory_timeout_sec = float(
            self.declare_parameter("planner_trajectory_timeout_sec", 2.0).value
        )
        if self.planner_trajectory_timeout_sec <= 0.0:
            raise ValueError("planner_trajectory_timeout_sec must be positive")
        self.ego_odom_topic = str(
            self.declare_parameter("ego_odom_topic", "/sim/pointlio/odom").value
        )
        self.px4_position_topic = str(
            self.declare_parameter(
                "px4_position_topic", "/fmu/out/vehicle_local_position"
            ).value
        )
        self.vehicle_status_topic = str(
            self.declare_parameter("vehicle_status_topic", "/fmu/out/vehicle_status").value
        )

        self.output_permitted = self.output_enabled
        if not self.output_enabled:
            self.get_logger().warn("PX4 SITL output is disabled.")
        else:
            self.get_logger().info("PX4 SITL output enabled.")

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        command_qos = QoSProfile(depth=10)
        self.mode_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", px4_qos
        )
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", px4_qos
        )
        self.command_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", px4_qos
        )
        ready_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.takeoff_ready_pub = self.create_publisher(
            Bool, self.takeoff_ready_topic, ready_qos
        )
        self.localization_health_required = bool(self.localization_health_topic)
        self.localization_healthy = not self.localization_health_required
        if self.localization_health_required:
            self.create_subscription(
                Bool,
                self.localization_health_topic,
                self.localization_health_callback,
                ready_qos,
            )
        self.create_subscription(
            PositionCommand, self.ego_command_topic, self.ego_command_callback, command_qos
        )
        self.trajectory_sub = self.create_subscription(
            Bspline,
            self.planner_trajectory_topic,
            self.planner_trajectory_callback,
            command_qos,
        )
        self.create_subscription(
            Odometry, self.ego_odom_topic, self.ego_odom_callback, command_qos
        )
        self.create_subscription(
            VehicleLocalPosition, self.px4_position_topic, self.px4_position_callback, px4_qos
        )
        self.create_subscription(
            VehicleStatus, self.vehicle_status_topic, self.vehicle_status_callback, px4_qos
        )

        self.latest_command: Optional[PositionCommand] = None
        self.latest_command_sec: Optional[float] = None
        self.latest_trajectory_id: Optional[int] = None
        self.trajectory_valid_until_sec: Optional[float] = None
        self.planner_guard_active = False
        self.latest_ego_odom: Optional[Odometry] = None
        self.latest_ego_odom_sec: Optional[float] = None
        self.latest_px4_position: Optional[VehicleLocalPosition] = None
        self.latest_vehicle_status: Optional[VehicleStatus] = None
        self.reference_world: Optional[tuple[float, float, float]] = None
        self.reference_px4: Optional[tuple[float, float, float]] = None
        self.reference_px4_heading: Optional[float] = None
        self.setpoint_cycles = 0
        self.offboard_prestream_cycles = max(
            10, int(self.offboard_prestream_sec * self.control_rate_hz)
        )
        self.last_arm_request_us = 0
        self.last_offboard_request_us = 0
        self.last_wait_warning_sec = -math.inf
        self.hold_position_ned: Optional[tuple[float, float, float]] = None
        self.hold_yaw: float = 0.0
        self.reference_capture_ready_sec = self.now_sec() + self.reference_capture_delay_sec
        self.takeoff_origin_ned: Optional[tuple[float, float, float]] = None
        self.takeoff_target_ned: Optional[tuple[float, float, float]] = None
        self.takeoff_command_ned: Optional[list[float]] = None
        self.takeoff_complete = not self.takeoff_before_ego
        self.takeoff_stable_since_sec: Optional[float] = None
        self.last_takeoff_ready_pub_sec = -math.inf
        self.create_timer(1.0 / self.control_rate_hz, self.timer_callback)
        self.get_logger().info(
            f"Simulation PX4 bridge: mode={self.control_mode}, "
            f"rate={self.control_rate_hz:.1f} Hz, "
            f"Offboard prestream={self.offboard_prestream_sec:.1f} s, "
            "entry_order=Offboard->arm (SITL auto-entry), "
            f"takeoff_before_ego={self.takeoff_before_ego}, "
            f"takeoff_altitude={self.takeoff_altitude:.2f} m, "
            f"lock_yaw_to_initial_heading={self.lock_yaw_to_initial_heading}, "
            f"planner_guard={self.planner_trajectory_topic}"
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def ego_command_callback(self, msg: PositionCommand) -> None:
        self.latest_command = msg
        self.latest_command_sec = self.now_sec()

    def planner_trajectory_callback(self, msg: Bspline) -> None:
        order = int(msg.order)
        knot_count = len(msg.knots)
        end_index = knot_count - 1 - order
        if order < 1 or knot_count <= 2 * order or end_index <= order:
            self.get_logger().error(
                f"Ignoring malformed EGO B-spline trajectory {int(msg.traj_id)}."
            )
            return
        duration = float(msg.knots[end_index]) - float(msg.knots[order])
        start_sec = float(msg.start_time.sec) + float(msg.start_time.nanosec) * 1e-9
        if not math.isfinite(duration) or duration <= 0.0 or not math.isfinite(start_sec):
            self.get_logger().error(
                f"Ignoring invalid EGO B-spline timing for trajectory {int(msg.traj_id)}."
            )
            return
        self.latest_trajectory_id = int(msg.traj_id)
        self.trajectory_valid_until_sec = (
            start_sec + duration + self.planner_trajectory_timeout_sec
        )
        if self.planner_guard_active:
            self.get_logger().info(
                f"Fresh EGO trajectory {self.latest_trajectory_id} restored planner control."
            )
        self.planner_guard_active = False

    def localization_health_callback(self, msg: Bool) -> None:
        was_healthy = self.localization_healthy
        self.localization_healthy = bool(msg.data)
        if was_healthy and not self.localization_healthy:
            self.get_logger().error(
                "Point-LIO odometry guard reported unhealthy localization; "
                "automatic PX4 state requests are inhibited."
            )

    def ego_odom_callback(self, msg: Odometry) -> None:
        self.latest_ego_odom = msg
        self.latest_ego_odom_sec = self.now_sec()
        self.try_capture_reference()

    def px4_position_callback(self, msg: VehicleLocalPosition) -> None:
        self.latest_px4_position = msg
        self.try_capture_reference()

    def vehicle_status_callback(self, msg: VehicleStatus) -> None:
        self.latest_vehicle_status = msg

    def px4_reference_is_valid(self) -> bool:
        msg = self.latest_px4_position
        return bool(
            msg is not None
            and msg.xy_valid
            and msg.z_valid
            and all(math.isfinite(value) for value in (msg.x, msg.y, msg.z))
        )

    def px4_heading_is_valid(self) -> bool:
        return self.latest_px4_position is not None and math.isfinite(
            float(self.latest_px4_position.heading)
        )

    def try_capture_reference(self) -> None:
        if self.reference_world is not None or self.latest_ego_odom is None:
            return
        if not self.localization_healthy:
            return
        if self.now_sec() < self.reference_capture_ready_sec:
            return
        if self.align_to_px4_reference and not self.px4_reference_is_valid():
            return
        if self.use_initial_heading_frame and not self.px4_heading_is_valid():
            return
        p = self.latest_ego_odom.pose.pose.position
        self.reference_world = (float(p.x), float(p.y), float(p.z))
        if self.align_to_px4_reference:
            px4 = self.latest_px4_position
            self.reference_px4 = (float(px4.x), float(px4.y), float(px4.z))
        else:
            self.reference_px4 = (0.0, 0.0, 0.0)
        self.reference_px4_heading = (
            self.wrap_angle(float(self.latest_px4_position.heading))
            if self.use_initial_heading_frame
            else 0.0
        )
        self.takeoff_origin_ned = tuple(self.reference_px4)
        self.takeoff_target_ned = (
            self.reference_px4[0],
            self.reference_px4[1],
            self.reference_px4[2] - self.takeoff_altitude,
        )
        self.takeoff_command_ned = list(self.takeoff_origin_ned)
        self.hold_position_ned = tuple(self.takeoff_origin_ned)
        self.hold_yaw = self.reference_px4_heading or 0.0
        self.setpoint_cycles = 0
        self.get_logger().info(
            "Captured simulation EGO/PX4 reference: "
            f"world={self.reference_world}, px4_ned={self.reference_px4}, "
            f"heading={self.reference_px4_heading:.3f}"
        )
        if self.takeoff_before_ego:
            self.get_logger().info(
                "Takeoff target captured: "
                f"origin_ned={self.takeoff_origin_ned}, "
                f"target_ned={self.takeoff_target_ned}"
            )
        else:
            self.publish_takeoff_ready(force=True)

    @staticmethod
    def wrap_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def transform_vector(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        m = self.world_to_px4_rotation
        mapped = (
            m[0] * x + m[1] * y + m[2] * z,
            m[3] * x + m[4] * y + m[5] * z,
            m[6] * x + m[7] * y + m[8] * z,
        )
        if self.use_initial_heading_frame:
            h = self.reference_px4_heading or 0.0
            c, s = math.cos(h), math.sin(h)
            mapped = (c * mapped[0] - s * mapped[1], s * mapped[0] + c * mapped[1], mapped[2])
        return mapped

    def velocity_command_world(self, command: PositionCommand) -> Optional[tuple[float, float, float]]:
        if self.latest_ego_odom is None:
            return None
        p = self.latest_ego_odom.pose.pose.position
        current = (float(p.x), float(p.y), float(p.z))
        if not all(math.isfinite(value) for value in current):
            return None
        e = (
            float(command.position.x) - current[0],
            float(command.position.y) - current[1],
            float(command.position.z) - current[2],
        )
        velocity = tuple(
            float(value) + self.velocity_position_gain * error
            for value, error in zip(
                (command.velocity.x, command.velocity.y, command.velocity.z), e
            )
        )
        norm = math.sqrt(sum(value * value for value in velocity))
        if norm > self.velocity_limit:
            scale = self.velocity_limit / norm
            velocity = tuple(value * scale for value in velocity)
        return velocity

    def make_setpoint(self, command: PositionCommand) -> Optional[TrajectorySetpoint]:
        if self.reference_world is None or self.reference_px4 is None:
            return None
        p, v, a = command.position, command.velocity, command.acceleration
        values = (p.x, p.y, p.z, v.x, v.y, v.z, a.x, a.y, a.z, command.yaw, command.yaw_dot)
        if not all(math.isfinite(float(value)) for value in values):
            return None
        msg = TrajectorySetpoint()
        msg.timestamp = self.now_us()
        if self.control_mode == "velocity":
            velocity = self.velocity_command_world(command)
            if velocity is None:
                return None
            msg.position = [math.nan, math.nan, math.nan]
            msg.velocity = list(self.transform_vector(*velocity))
            msg.acceleration = [math.nan, math.nan, math.nan]
        else:
            delta = (
                float(p.x) - self.reference_world[0],
                float(p.y) - self.reference_world[1],
                float(p.z) - self.reference_world[2],
            )
            position = self.transform_vector(*delta)
            msg.position = [
                self.reference_px4[0] + position[0],
                self.reference_px4[1] + position[1],
                self.reference_px4[2] + position[2],
            ]
            msg.velocity = list(self.transform_vector(float(v.x), float(v.y), float(v.z)))
            msg.acceleration = list(self.transform_vector(float(a.x), float(a.y), float(a.z)))
        heading = self.reference_px4_heading if self.use_initial_heading_frame else 0.0
        if self.lock_yaw_to_initial_heading:
            msg.yaw = self.wrap_angle(float(heading or 0.0))
            msg.yawspeed = 0.0
        else:
            msg.yaw = self.wrap_angle(
                self.yaw_sign * float(command.yaw) + self.yaw_offset + (heading or 0.0)
            )
            msg.yawspeed = self.yaw_sign * float(command.yaw_dot)
        return msg

    def publish_offboard_mode(self, force_position: bool = False) -> None:
        msg = OffboardControlMode()
        msg.timestamp = self.now_us()
        msg.position = force_position or self.control_mode == "position"
        msg.velocity = not force_position and self.control_mode == "velocity"
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.actuator = False
        self.mode_pub.publish(msg)

    def publish_vehicle_command(self, command: int, param1: float = 0.0, param2: float = 0.0) -> None:
        msg = VehicleCommand()
        msg.timestamp = self.now_us()
        msg.param1, msg.param2 = param1, param2
        msg.command = command
        msg.target_system = msg.source_system = 1
        msg.target_component = msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def publish_takeoff_ready(self, force: bool = False) -> None:
        if not self.takeoff_complete:
            return
        now = self.now_sec()
        if not force and now - self.last_takeoff_ready_pub_sec < 1.0:
            return
        msg = Bool()
        msg.data = True
        self.takeoff_ready_pub.publish(msg)
        self.last_takeoff_ready_pub_sec = now

    def make_position_setpoint(
        self, position: tuple[float, float, float] | list[float], yaw: float
    ) -> TrajectorySetpoint:
        msg = TrajectorySetpoint()
        msg.timestamp = self.now_us()
        msg.position = [float(value) for value in position]
        msg.velocity = [math.nan, math.nan, math.nan]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.yaw = float(yaw)
        msg.yawspeed = 0.0
        return msg

    def update_takeoff_command(self) -> None:
        if self.takeoff_command_ned is None or self.takeoff_target_ned is None:
            return
        offboard = bool(
            self.latest_vehicle_status is not None
            and self.latest_vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        )
        if not offboard:
            return
        step = self.takeoff_vertical_speed / self.control_rate_hz
        dz = self.takeoff_target_ned[2] - self.takeoff_command_ned[2]
        if abs(dz) <= step:
            self.takeoff_command_ned[2] = self.takeoff_target_ned[2]
        else:
            self.takeoff_command_ned[2] += math.copysign(step, dz)

    def takeoff_is_stable(self) -> bool:
        if self.latest_px4_position is None or self.takeoff_target_ned is None:
            return False
        px4 = self.latest_px4_position
        values = (px4.x, px4.y, px4.z, px4.vx, px4.vy, px4.vz)
        if not all(math.isfinite(float(value)) for value in values):
            return False
        return (
            math.hypot(px4.x - self.takeoff_target_ned[0], px4.y - self.takeoff_target_ned[1])
            <= self.takeoff_reach_xy_tol
            and abs(px4.z - self.takeoff_target_ned[2]) <= self.takeoff_reach_z_tol
            and math.hypot(px4.vx, px4.vy) <= self.takeoff_speed_xy_tol
            and abs(px4.vz) <= self.takeoff_speed_z_tol
        )

    def update_takeoff_completion(self) -> None:
        if self.takeoff_complete:
            return
        if not self.takeoff_is_stable():
            self.takeoff_stable_since_sec = None
            return
        if self.takeoff_stable_since_sec is None:
            self.takeoff_stable_since_sec = self.now_sec()
            self.get_logger().info(
                f"Takeoff altitude reached; verifying stability for {self.takeoff_stable_sec:.1f} s."
            )
            return
        if self.now_sec() - self.takeoff_stable_since_sec < self.takeoff_stable_sec:
            return
        self.takeoff_complete = True
        self.hold_position_ned = tuple(self.takeoff_target_ned)
        self.get_logger().info("Takeoff complete. EGO goal and trajectory control are now enabled.")
        self.publish_takeoff_ready(force=True)

    def publish_takeoff_setpoint(self) -> None:
        if self.takeoff_command_ned is None:
            return
        self.update_takeoff_command()
        self.publish_offboard_mode(force_position=True)
        self.setpoint_pub.publish(
            self.make_position_setpoint(self.takeoff_command_ned, self.hold_yaw)
        )
        self.setpoint_cycles += 1

    def command_is_fresh(self) -> bool:
        return bool(
            self.latest_command is not None
            and self.latest_command_sec is not None
            and self.now_sec() - self.latest_command_sec <= self.command_timeout_sec
        )

    def odom_is_fresh(self) -> bool:
        return bool(
            self.latest_ego_odom is not None
            and self.latest_ego_odom_sec is not None
            and self.now_sec() - self.latest_ego_odom_sec <= self.odom_timeout_sec
        )

    def planner_trajectory_is_valid(self) -> tuple[bool, str]:
        if self.latest_trajectory_id is None or self.trajectory_valid_until_sec is None:
            return False, "Waiting for the first EGO B-spline trajectory"
        if self.trajectory_sub.get_publisher_count() == 0:
            return False, "EGO planner trajectory publisher disappeared"
        if self.now_sec() > self.trajectory_valid_until_sec:
            return False, "EGO trajectory expired without a replacement"
        return True, ""

    def warn_waiting(self, message: str) -> None:
        now = self.now_sec()
        if now - self.last_wait_warning_sec >= 2.0:
            self.last_wait_warning_sec = now
            self.get_logger().warn(message)

    def make_hold_setpoint(self) -> Optional[TrajectorySetpoint]:
        """Create a stationary setpoint while planner/sensor data is unavailable."""
        if self.hold_position_ned is None:
            px4 = self.latest_px4_position
            if self.px4_reference_is_valid() and px4 is not None:
                self.hold_position_ned = (float(px4.x), float(px4.y), float(px4.z))
                if self.px4_heading_is_valid():
                    self.hold_yaw = self.wrap_angle(float(px4.heading))
            elif self.latest_command is not None and self.reference_world is not None:
                candidate = self.make_setpoint(self.latest_command)
                if candidate is not None and all(math.isfinite(float(v)) for v in candidate.position):
                    self.hold_position_ned = tuple(float(v) for v in candidate.position)
                    self.hold_yaw = float(candidate.yaw)
            elif self.reference_px4 is not None:
                self.hold_position_ned = tuple(float(v) for v in self.reference_px4)
        if self.hold_position_ned is None:
            return None
        msg = TrajectorySetpoint()
        msg.timestamp = self.now_us()
        msg.position = list(self.hold_position_ned)
        msg.velocity = [math.nan, math.nan, math.nan]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.yaw = self.hold_yaw
        msg.yawspeed = 0.0
        return msg

    def publish_hold(self, reason: str) -> None:
        self.publish_offboard_mode()
        hold = self.make_hold_setpoint()
        if hold is not None:
            self.setpoint_pub.publish(hold)
            self.setpoint_cycles += 1
        self.warn_waiting(f"{reason}; holding the last safe PX4 position.")

    def manage_px4_state(self) -> None:
        if (
            not self.localization_healthy
            or self.latest_vehicle_status is None
            or self.setpoint_cycles < self.offboard_prestream_cycles
        ):
            return
        now_us = self.now_us()
        armed = self.latest_vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        offboard = self.latest_vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        # PX4 SITL accepts automatic arming reliably after Offboard has been
        # confirmed. Hardware keeps the validated manual-arm -> Offboard order.
        if self.auto_offboard and not offboard:
            if now_us - self.last_offboard_request_us > 1_000_000:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.last_offboard_request_us = now_us
                self.get_logger().warn("Sent PX4 SITL Offboard request; waiting for confirmation.")
            return
        if not armed and self.auto_arm and now_us - self.last_arm_request_us > 1_000_000:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.last_arm_request_us = now_us
            self.get_logger().warn("Sent PX4 SITL arm request after Offboard confirmation.")

    def timer_callback(self) -> None:
        if not self.output_permitted:
            return
        if not self.localization_healthy:
            if self.reference_world is None:
                self.warn_waiting("Waiting for healthy Point-LIO odometry")
            else:
                self.publish_hold("Point-LIO odometry is unhealthy")
            return
        if self.reference_world is None:
            self.publish_offboard_mode(force_position=True)
            self.warn_waiting("Waiting for Point-LIO and a valid PX4 takeoff reference")
            return
        if not self.takeoff_complete:
            self.publish_takeoff_setpoint()
            self.manage_px4_state()
            self.update_takeoff_completion()
            return
        self.publish_takeoff_ready()
        trajectory_valid, trajectory_reason = self.planner_trajectory_is_valid()
        if not trajectory_valid:
            if not self.planner_guard_active and self.latest_trajectory_id is not None:
                self.get_logger().error(
                    f"{trajectory_reason}; rejecting traj_server output and switching to PX4 hold."
                )
            self.planner_guard_active = True
            self.publish_hold(trajectory_reason)
            self.manage_px4_state()
            return
        if not self.command_is_fresh():
            self.publish_hold("Waiting for a fresh EGO PositionCommand")
            self.manage_px4_state()
            return
        if not self.odom_is_fresh():
            self.publish_hold("Waiting for fresh EGO odometry")
            self.manage_px4_state()
            return
        if self.reference_world is None:
            self.publish_hold("Waiting for aligned EGO/PX4 reference")
            self.manage_px4_state()
            return
        setpoint = self.make_setpoint(self.latest_command)
        if setpoint is None:
            self.publish_hold("Invalid EGO PositionCommand")
            return
        self.publish_offboard_mode()
        self.setpoint_pub.publish(setpoint)
        self.setpoint_cycles += 1
        self.hold_position_ned = None
        self.manage_px4_state()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimEgoPx4Bridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
