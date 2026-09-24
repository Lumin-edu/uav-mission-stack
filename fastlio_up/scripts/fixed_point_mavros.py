#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from mavros_setpoint_contract import (
    resolve_target,
    resolve_yaw,
)
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)


class FixedPointMavros(Node):
    """MAVROS fixed-point controller, independent of the PX4-DDS packages."""

    def __init__(self) -> None:
        super().__init__("fixed_point_mavros")
        self.target_x = float(self.declare_parameter("target_x", 0.0).value)
        self.target_y = float(self.declare_parameter("target_y", 0.0).value)
        # Task coordinates are (right, forward, up) relative to the captured pose.
        self.target_z = float(self.declare_parameter("target_z", 1.0).value)
        self.configured_yaw = float(self.declare_parameter("target_yaw", 0.0).value)
        self.hold_current_yaw = bool(
            self.declare_parameter("hold_current_yaw", True).value
        )
        self.auto_arm = bool(self.declare_parameter("auto_arm", False).value)
        self.auto_offboard = bool(self.declare_parameter("auto_offboard", True).value)
        self.reference_capture_delay_sec = float(
            self.declare_parameter("reference_capture_delay_sec", 3.0).value
        )
        self.control_rate_hz = max(
            10.0, float(self.declare_parameter("control_rate_hz", 50.0).value)
        )
        self.prestream_sec = max(
            1.0, float(self.declare_parameter("prestream_sec", 2.0).value)
        )
        self.max_speed = max(
            0.05, float(self.declare_parameter("max_speed", 0.8).value)
        )
        self.state_topic = str(
            self.declare_parameter("state_topic", "/mavros/state").value
        )
        self.local_odom_topic = str(
            self.declare_parameter(
                "local_odom_topic", "/mavros/local_position/odom"
            ).value
        )
        self.fastlio_odom_topic = str(
            self.declare_parameter("fastlio_odom_topic", "/Odometry").value
        )
        self.setpoint_topic = str(
            self.declare_parameter(
                "setpoint_topic", "/mavros/setpoint_position/local"
            ).value
        )
        self.arming_service = str(
            self.declare_parameter("arming_service", "/mavros/cmd/arming").value
        )
        self.mode_service = str(
            self.declare_parameter("mode_service", "/mavros/set_mode").value
        )

        state_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        # Match the validated ROS 1 controller: PoseStamped position setpoints.
        self.setpoint_pub = self.create_publisher(
            PoseStamped, self.setpoint_topic, 20
        )
        self.create_subscription(
            State, self.state_topic, self.state_callback, state_qos
        )
        self.create_subscription(
            Odometry, self.local_odom_topic, self.odom_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, self.fastlio_odom_topic, self.fastlio_odom_callback,
            qos_profile_sensor_data
        )
        self.arm_client = self.create_client(CommandBool, self.arming_service)
        self.mode_client = self.create_client(SetMode, self.mode_service)

        self.state = State()
        self.reference_enu: tuple[float, float, float] | None = None
        self.target_enu: tuple[float, float, float] | None = None
        self.reference_orientation: tuple[float, float, float, float] | None = None
        self.command_enu: tuple[float, float, float] | None = None
        self.fastlio_position = None
        self.px4_position = None
        self.current_yaw: float | None = None
        self.last_position_log = 0.0
        self.target_yaw: float | None = resolve_yaw(
            None,
            configured_yaw=self.configured_yaw,
            hold_current_yaw=self.hold_current_yaw,
        )
        self.reference_ready_at = time.monotonic() + self.reference_capture_delay_sec
        self.prestream_started = time.monotonic()
        self.target_enu = resolve_target(
            None,
            (self.target_x, self.target_y, self.target_z),
            orientation=None,
        )
        self.last_request = 0.0
        self.last_command_time = time.monotonic()
        self.timer = self.create_timer(1.0 / self.control_rate_hz, self.timer_callback)

        self.get_logger().info(
            "MAVROS fixed-point controller: "
            f"target=({self.target_x:.2f}, {self.target_y:.2f}, {self.target_z:.2f}), "
            f"setpoint={self.setpoint_topic}, auto_arm={self.auto_arm}, auto_offboard={self.auto_offboard}"
        )

    def state_callback(self, msg: State) -> None:
        self.state = msg

    def fastlio_odom_callback(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        values = (float(p.x), float(p.y), float(p.z))
        if all(math.isfinite(value) for value in values):
            self.fastlio_position = values

    def log_positions(self, now: float) -> None:
        if now - self.last_position_log < 1.0:
            return
        self.last_position_log = now
        fastlio = self.fastlio_position
        px4 = self.px4_position
        fastlio_text = "unavailable" if fastlio is None else "(%.3f, %.3f, %.3f)" % fastlio
        px4_text = "unavailable" if px4 is None else "(%.3f, %.3f, %.3f)" % px4
        self.get_logger().info(f"positions: FAST-LIO={fastlio_text} PX4/EKF local={px4_text}")

    def odom_callback(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        values = (float(p.x), float(p.y), float(p.z))
        if not all(math.isfinite(value) for value in values):
            return
        self.px4_position = values
        orientation = msg.pose.pose.orientation
        orientation_values = (
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        )
        self.current_yaw = resolve_yaw(
            orientation_values, configured_yaw=0.0, hold_current_yaw=True
        )
        if self.reference_enu is None and time.monotonic() >= self.reference_ready_at:
            captured_yaw = resolve_yaw(
                orientation_values,
                configured_yaw=self.configured_yaw,
                hold_current_yaw=self.hold_current_yaw,
            )
            if captured_yaw is None:
                self.get_logger().warning(
                    "Waiting for a valid MAVROS orientation before capturing takeoff yaw"
                )
                return
            self.reference_enu = values
            self.reference_orientation = orientation_values
            self.target_enu = resolve_target(
                values,
                (self.target_x, self.target_y, self.target_z),
                orientation=orientation_values,
            )
            self.target_yaw = captured_yaw
            self.command_enu = values
            self.last_command_time = time.monotonic()
            self.get_logger().info(
                f"Captured MAVROS local reference {values}; task target="
                f"({self.target_x:.2f}, {self.target_y:.2f}, {self.target_z:.2f}) "
                f"(right, forward, up); fixed target local={self.target_enu}; "
                f"hold yaw={self.target_yaw:.3f} rad"
            )
            # The pre-stream requirement starts once a usable target exists.
            self.prestream_started = time.monotonic()

    def publish_setpoint(self) -> None:
        # Before origin capture, stream the current local pose as the OFFBOARD
        # heartbeat, exactly like the validated ROS 1 implementation.
        position = self.command_enu
        yaw = self.target_yaw if self.target_yaw is not None else self.current_yaw
        if position is None and self.px4_position is not None:
            position = self.px4_position
        if position is None or yaw is None:
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = position
        msg.pose.orientation.z = math.sin(0.5 * yaw)
        msg.pose.orientation.w = math.cos(0.5 * yaw)
        self.setpoint_pub.publish(msg)

    def update_command(self, now: float) -> None:
        if self.command_enu is None or self.target_enu is None:
            self.last_command_time = now
            return
        if not self.state.armed or self.state.mode != "OFFBOARD":
            self.last_command_time = now
            return
        elapsed = min(max(now - self.last_command_time, 0.0), 0.2)
        self.last_command_time = now
        delta = tuple(self.target_enu[i] - self.command_enu[i] for i in range(3))
        distance = math.sqrt(sum(value * value for value in delta))
        if distance <= 1e-9:
            return
        step = min(self.max_speed * elapsed, distance)
        scale = step / distance
        self.command_enu = tuple(
            self.command_enu[i] + delta[i] * scale for i in range(3)
        )

    def request_arm(self) -> None:
        if not self.state.connected:
            return
        if not self.arm_client.service_is_ready():
            return
        request = CommandBool.Request()
        request.value = True
        future = self.arm_client.call_async(request)
        future.add_done_callback(lambda result: self._arm_result_callback(result))

    def request_offboard(self) -> None:
        # ROS 1 behavior: the pilot arms first in Position mode; only then do
        # we request OFFBOARD and let PX4 follow the streamed position target.
        if not self.state.connected or not self.state.armed:
            return
        if not self.mode_client.service_is_ready():
            return
        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = "OFFBOARD"
        future = self.mode_client.call_async(request)
        future.add_done_callback(lambda result: self._offboard_result_callback(result))

    def _arm_result_callback(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001  # ROS future may raise transport errors
            self.get_logger().warning(f"MAVROS arming request failed: {exc}")
            return
        if not response.success:
            self.get_logger().warning(
                f"MAVROS rejected arming request (result={response.result})"
            )

    def _offboard_result_callback(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001  # ROS future may raise transport errors
            self.get_logger().warning(f"MAVROS OFFBOARD request failed: {exc}")
            return
        if not response.mode_sent:
            self.get_logger().warning("MAVROS rejected OFFBOARD mode request")

    def timer_callback(self) -> None:
        # MAVROS/PX4 requires a continuous setpoint stream before OFFBOARD.
        now = time.monotonic()
        self.update_command(now)
        self.publish_setpoint()
        self.log_positions(now)
        if (
            now - self.prestream_started < self.prestream_sec
            or self.target_enu is None
            or self.target_yaw is None
        ):
            return
        if not self.state.connected:
            return
        if self.auto_arm and not self.state.armed and now - self.last_request > 1.0:
            self.request_arm()
            self.last_request = now
            return
        if (
            self.auto_offboard
            and self.state.armed
            and self.state.mode != "OFFBOARD"
            and now - self.last_request > 1.0
        ):
            self.request_offboard()
            self.last_request = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FixedPointMavros()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
