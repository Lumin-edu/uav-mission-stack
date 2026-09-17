#!/usr/bin/env python3

import math
import time

import rclpy
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from mavros_setpoint_contract import (
    FRAME_LOCAL_NED,
    make_position_target,
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
        # target_z is an ENU altitude delta when use_current_position_reference is true.
        self.target_z = float(self.declare_parameter("target_z", 1.0).value)
        self.configured_yaw = float(self.declare_parameter("target_yaw", 0.0).value)
        self.hold_current_yaw = bool(
            self.declare_parameter("hold_current_yaw", True).value
        )
        self.auto_arm = bool(self.declare_parameter("auto_arm", False).value)
        self.auto_offboard = bool(self.declare_parameter("auto_offboard", True).value)
        self.use_current_position_reference = bool(
            self.declare_parameter("use_current_position_reference", True).value
        )
        self.reference_capture_delay_sec = float(
            self.declare_parameter("reference_capture_delay_sec", 3.0).value
        )
        self.control_rate_hz = max(
            10.0, float(self.declare_parameter("control_rate_hz", 50.0).value)
        )
        self.prestream_sec = max(
            1.0, float(self.declare_parameter("prestream_sec", 2.0).value)
        )
        self.state_topic = str(
            self.declare_parameter("state_topic", "/mavros/state").value
        )
        self.local_odom_topic = str(
            self.declare_parameter(
                "local_odom_topic", "/mavros/local_position/odom"
            ).value
        )
        self.setpoint_topic = str(
            self.declare_parameter("setpoint_topic", "/mavros/setpoint_raw/local").value
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
        # These profiles match MAVROS' state, sensor, and raw-setpoint plugins.
        self.setpoint_pub = self.create_publisher(
            PositionTarget, self.setpoint_topic, qos_profile_sensor_data
        )
        self.create_subscription(
            State, self.state_topic, self.state_callback, state_qos
        )
        self.create_subscription(
            Odometry, self.local_odom_topic, self.odom_callback, qos_profile_sensor_data
        )
        self.arm_client = self.create_client(CommandBool, self.arming_service)
        self.mode_client = self.create_client(SetMode, self.mode_service)

        self.state = State()
        self.reference_enu: tuple[float, float, float] | None = None
        self.target_enu: tuple[float, float, float] | None = None
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
            use_current_position_reference=self.use_current_position_reference,
        )
        self.last_request = 0.0
        self.timer = self.create_timer(1.0 / self.control_rate_hz, self.timer_callback)

        self.get_logger().info(
            "MAVROS fixed-point controller: "
            f"target=({self.target_x:.2f}, {self.target_y:.2f}, {self.target_z:.2f}), "
            f"setpoint={self.setpoint_topic}, auto_arm={self.auto_arm}, auto_offboard={self.auto_offboard}"
        )

    def state_callback(self, msg: State) -> None:
        self.state = msg

    def odom_callback(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        values = (float(p.x), float(p.y), float(p.z))
        if not all(math.isfinite(value) for value in values):
            return
        if self.reference_enu is None and time.monotonic() >= self.reference_ready_at:
            orientation = msg.pose.pose.orientation
            captured_yaw = resolve_yaw(
                (orientation.x, orientation.y, orientation.z, orientation.w),
                configured_yaw=self.configured_yaw,
                hold_current_yaw=self.hold_current_yaw,
            )
            if captured_yaw is None:
                self.get_logger().warning(
                    "Waiting for a valid MAVROS orientation before capturing takeoff yaw"
                )
                return
            self.reference_enu = values
            self.target_enu = resolve_target(
                values,
                (self.target_x, self.target_y, self.target_z),
                use_current_position_reference=self.use_current_position_reference,
            )
            self.target_yaw = captured_yaw
            self.get_logger().info(
                f"Captured MAVROS ENU reference {values}; target ENU={self.target_enu}; "
                f"hold yaw={self.target_yaw:.3f} rad"
            )
            # The pre-stream requirement starts once a usable target exists.
            self.prestream_started = time.monotonic()

    def publish_setpoint(self) -> None:
        if self.target_enu is None or self.target_yaw is None:
            return
        # MAVROS accepts ROS ENU values on this ROS topic and performs the
        # MAVLink/PX4 ENU-to-NED conversion inside its setpoint plugin.
        fields = make_position_target(self.target_enu, self.target_yaw)
        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.coordinate_frame = FRAME_LOCAL_NED
        msg.type_mask = fields.type_mask
        msg.position.x, msg.position.y, msg.position.z = fields.position
        msg.velocity.x, msg.velocity.y, msg.velocity.z = fields.velocity
        (
            msg.acceleration_or_force.x,
            msg.acceleration_or_force.y,
            msg.acceleration_or_force.z,
        ) = fields.acceleration
        msg.yaw = fields.yaw
        msg.yaw_rate = fields.yaw_rate
        self.setpoint_pub.publish(msg)

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
        if not self.state.connected:
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
        self.publish_setpoint()
        now = time.monotonic()
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
