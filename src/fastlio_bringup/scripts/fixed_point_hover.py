#!/usr/bin/env python3

import math

import rclpy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class FixedPointHover(Node):
    """Hold a relative local-position target through PX4 position Offboard mode."""

    def __init__(self) -> None:
        super().__init__("fastlio_fixed_point_hover")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.target_x = float(self.declare_parameter("target_x", 0.0).value)
        self.target_y = float(self.declare_parameter("target_y", 0.0).value)
        self.target_z = float(self.declare_parameter("target_z", -0.2).value)
        self.target_yaw = float(self.declare_parameter("target_yaw", 0.0).value)
        self.auto_arm = bool(self.declare_parameter("auto_arm", False).value)
        self.use_current_position_reference = bool(
            self.declare_parameter("use_current_position_reference", True).value
        )
        self.reference_capture_delay_sec = float(
            self.declare_parameter("reference_capture_delay_sec", 3.0).value
        )
        self.max_local_position_age_sec = self._positive_parameter(
            "max_local_position_age_sec", 0.5
        )
        self.max_vehicle_status_age_sec = self._positive_parameter(
            "max_vehicle_status_age_sec", 0.5
        )
        self.control_rate_hz = float(
            self.declare_parameter("control_rate_hz", 50.0).value
        )
        if self.control_rate_hz <= 0.0:
            self.get_logger().warning("control_rate_hz must be positive; using 50 Hz")
            self.control_rate_hz = 50.0
        self.offboard_prestream_cycles = max(10, int(2.0 * self.control_rate_hz))
        self.vehicle_local_position_topic = str(
            self.declare_parameter(
                "vehicle_local_position_topic", "/fmu/out/vehicle_local_position"
            ).value
        )
        self.vehicle_status_topic = str(
            self.declare_parameter(
                "vehicle_status_topic", "/fmu/out/vehicle_status"
            ).value
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
        self.last_local_position_receive_sec = None
        self.last_vehicle_status_receive_sec = None
        self.create_subscription(
            VehicleLocalPosition,
            self.vehicle_local_position_topic,
            self.vehicle_local_position_callback,
            qos,
        )
        self.create_subscription(
            VehicleStatus, self.vehicle_status_topic, self.vehicle_status_callback, qos
        )

        self.offboard_setpoint_counter = 0
        self.armed = False
        self.last_arm_request_time = 0
        self.last_offboard_request_time = 0
        self.waiting_for_manual_arm_logged = False
        self.waiting_for_reference_logged = False
        self.waiting_for_fresh_state_logged = False
        self.hover_reference_captured = False
        self.reference_capture_ready_sec = (
            self.now_sec() + self.reference_capture_delay_sec
        )
        self.target_local = [self.target_x, self.target_y, self.target_z]
        self.create_timer(1.0 / self.control_rate_hz, self.timer_callback)

        self.get_logger().info(
            "FAST-LIO fixed-point hover active: "
            f"target_delta=({self.target_x:.2f}, {self.target_y:.2f}, {self.target_z:.2f}), "
            f"auto_arm={self.auto_arm}, rate={self.control_rate_hz:.1f} Hz"
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _positive_parameter(self, name: str, default: float) -> float:
        value = float(self.declare_parameter(name, default).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be a finite positive value")
        return value

    @staticmethod
    def local_position_is_valid(msg: VehicleLocalPosition) -> bool:
        return bool(
            msg.xy_valid
            and msg.z_valid
            and all(math.isfinite(float(value)) for value in (msg.x, msg.y, msg.z))
        )

    def local_position_is_fresh(self) -> bool:
        return bool(
            self.last_local_position_receive_sec is not None
            and self.now_sec() - self.last_local_position_receive_sec
            <= self.max_local_position_age_sec
            and self.local_position_is_valid(self.vehicle_local_position)
        )

    def vehicle_status_is_fresh(self) -> bool:
        return bool(
            self.last_vehicle_status_receive_sec is not None
            and self.now_sec() - self.last_vehicle_status_receive_sec
            <= self.max_vehicle_status_age_sec
        )

    def vehicle_local_position_callback(self, msg: VehicleLocalPosition) -> None:
        self.vehicle_local_position = msg
        self.last_local_position_receive_sec = self.now_sec()
        if self.hover_reference_captured or not self.local_position_is_valid(msg):
            return
        if self.now_sec() < self.reference_capture_ready_sec:
            return
        if self.use_current_position_reference:
            self.target_local = [
                float(msg.x) + self.target_x,
                float(msg.y) + self.target_y,
                float(msg.z) + self.target_z,
            ]
        else:
            self.target_local = [self.target_x, self.target_y, self.target_z]
        self.hover_reference_captured = True
        self.waiting_for_reference_logged = False
        self.get_logger().info(
            "Captured PX4 local hover reference: "
            f"current=({msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f}), "
            f"target=({self.target_local[0]:.2f}, {self.target_local[1]:.2f}, "
            f"{self.target_local[2]:.2f})"
        )

    def vehicle_status_callback(self, msg: VehicleStatus) -> None:
        self.vehicle_status = msg
        self.last_vehicle_status_receive_sec = self.now_sec()
        self.armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        if self.armed:
            self.waiting_for_manual_arm_logged = False

    def publish_offboard_control_mode(self) -> None:
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.actuator = False
        msg.timestamp = int(self.now_sec() * 1e6)
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self) -> None:
        msg = TrajectorySetpoint()
        msg.position = [float(value) for value in self.target_local]
        msg.yaw = self.target_yaw
        msg.timestamp = int(self.now_sec() * 1e6)
        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command: int, **params: float) -> None:
        msg = VehicleCommand()
        msg.command = command
        for index in range(1, 8):
            setattr(msg, f"param{index}", params.get(f"param{index}", 0.0))
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.now_sec() * 1e6)
        self.vehicle_command_pub.publish(msg)

    def engage_offboard_mode(self) -> None:
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0
        )
        self.get_logger().info("Offboard mode command sent")

    def arm(self) -> None:
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0
        )
        self.get_logger().info("Arm command sent")

    def timer_callback(self) -> None:
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()
        now_us = int(self.now_sec() * 1e6)

        if self.offboard_setpoint_counter < self.offboard_prestream_cycles:
            self.offboard_setpoint_counter += 1
            return

        if self.use_current_position_reference and not self.hover_reference_captured:
            if not self.waiting_for_reference_logged:
                self.waiting_for_reference_logged = True
                self.get_logger().info(
                    "Waiting for valid PX4 local position before Offboard mode command."
                )
            return

        if not self.local_position_is_fresh() or not self.vehicle_status_is_fresh():
            if not self.waiting_for_fresh_state_logged:
                self.waiting_for_fresh_state_logged = True
                self.get_logger().warning(
                    "Waiting for fresh valid PX4 local position and vehicle status "
                    "before arm/Offboard requests."
                )
            return
        self.waiting_for_fresh_state_logged = False

        if not self.armed:
            if self.auto_arm and now_us - self.last_arm_request_time > 1_000_000:
                self.arm()
                self.last_arm_request_time = now_us
            elif not self.auto_arm and not self.waiting_for_manual_arm_logged:
                self.waiting_for_manual_arm_logged = True
                self.get_logger().info(
                    "Waiting for manual arm from RC before sending Offboard mode command."
                )
            return

        if self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            if now_us - self.last_offboard_request_time > 1_000_000:
                self.engage_offboard_mode()
                self.last_offboard_request_time = now_us
            return

        dx = self.vehicle_local_position.x - self.target_local[0]
        dy = self.vehicle_local_position.y - self.target_local[1]
        dz = self.vehicle_local_position.z - self.target_local[2]
        self.get_logger().debug(
            f"Distance to FAST-LIO hover target: {math.sqrt(dx * dx + dy * dy + dz * dz):.3f} m"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FixedPointHover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
