#!/usr/bin/env python3
"""运行时检查 PX4 Offboard 控制接口是否满足约定。

看门狗与控制器实现有意保持独立。它从 ROS 图和实际到达 PX4 输入话题的消息侧
进行检查，用于发现重复发布者、数据流超时，以及在 MPC 纯加速度模式中误填
位置或速度字段等问题。
"""

import math
from typing import Optional

import rclpy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class Px4ControlWatchdog(Node):
    """检查话题发布权、消息新鲜度以及 NaN/有限值字段语义。"""

    def __init__(self) -> None:
        super().__init__("px4_control_watchdog")
        self.control_source = str(self.declare_parameter("control_source", "hover").value)
        self.control_timeout_sec = float(self.declare_parameter("control_timeout_sec", 0.30).value)
        if self.control_timeout_sec <= 0.0:
            raise ValueError("control_timeout_sec must be positive")
        # 下列 PX4 输入话题必须各自只有一个发布者。若旧桥接节点同时运行，它可能
        # 在不触发任何 Python 异常的情况下覆盖 MPC 指令。
        self.required_topics = [
            "/fmu/in/offboard_control_mode",
            "/fmu/in/trajectory_setpoint",
            "/fmu/in/vehicle_command",
        ]
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.latest_mode: Optional[OffboardControlMode] = None
        self.latest_setpoint: Optional[TrajectorySetpoint] = None
        self.latest_mode_sec: Optional[float] = None
        self.latest_setpoint_sec: Optional[float] = None
        self.reported_ready = False
        self.create_subscription(
            OffboardControlMode,
            "/fmu/in/offboard_control_mode",
            self.mode_callback,
            px4_qos,
        )
        self.create_subscription(
            TrajectorySetpoint,
            "/fmu/in/trajectory_setpoint",
            self.setpoint_callback,
            px4_qos,
        )
        self.create_timer(2.0, self.report)
        self.get_logger().info(
            f"PX4 control watchdog active, expected control_source={self.control_source}, "
            f"timeout={self.control_timeout_sec:.2f}s"
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def age(self, stamp: Optional[float]) -> float:
        return math.inf if stamp is None else max(0.0, self.now_sec() - stamp)

    def mode_callback(self, msg: OffboardControlMode) -> None:
        self.latest_mode = msg
        self.latest_mode_sec = self.now_sec()

    def setpoint_callback(self, msg: TrajectorySetpoint) -> None:
        self.latest_setpoint = msg
        self.latest_setpoint_sec = self.now_sec()

    @staticmethod
    def _all_finite(values) -> bool:
        values = tuple(values)
        return len(values) == 3 and all(math.isfinite(float(value)) for value in values)

    @staticmethod
    def _all_nan(values) -> bool:
        values = tuple(values)
        return len(values) == 3 and all(math.isnan(float(value)) for value in values)

    def setpoint_matches_mode(self) -> bool:
        """检查 PX4 控制模式与对应设定值字段是否一致。

        本包约定 ``OffboardControlMode`` 中只能启用一种控制接口。加速度模式要求
        acceleration 为有限值、position/velocity 为 NaN；位置模式则相反。
        这遵循 PX4 的字段约定：NaN 表示该状态量不参与控制。
        """
        if self.latest_mode is None or self.latest_setpoint is None:
            return False
        active_modes = (
            getattr(self.latest_mode, "position", False),
            getattr(self.latest_mode, "velocity", False),
            getattr(self.latest_mode, "acceleration", False),
            getattr(self.latest_mode, "attitude", False),
            getattr(self.latest_mode, "body_rate", False),
            getattr(self.latest_mode, "actuator", False),
        )
        # PX4 内部虽然对控制字段规定了优先级，但这里若允许多个标志同时为 true，
        # 会掩盖发布者或配置错误，因此直接判定为不合法。
        if sum(bool(value) for value in active_modes) != 1:
            return False
        if self.latest_mode.acceleration:
            return self._all_nan(self.latest_setpoint.position) and self._all_nan(
                self.latest_setpoint.velocity
            ) and self._all_finite(self.latest_setpoint.acceleration)
        if self.latest_mode.position:
            return (
                self._all_finite(self.latest_setpoint.position)
                and self._all_nan(self.latest_setpoint.velocity)
                and self._all_nan(self.latest_setpoint.acceleration)
            )
        return False

    def report(self) -> None:
        """按固定周期输出就绪状态或首个错误，避免日志刷屏。"""
        failures = []
        details = []
        for topic in self.required_topics:
            publishers = self.get_publishers_info_by_topic(topic)
            subscriptions = self.get_subscriptions_info_by_topic(topic)
            details.append(f"{topic}: pubs={len(publishers)}, subs={len(subscriptions)}")
            if len(publishers) != 1:
                failures.append(f"expected exactly one publisher on {topic}, found {len(publishers)}")

        if self.age(self.latest_mode_sec) > self.control_timeout_sec:
            failures.append("OffboardControlMode is missing or stale")
        if self.age(self.latest_setpoint_sec) > self.control_timeout_sec:
            failures.append("TrajectorySetpoint is missing or stale")
        if not self.setpoint_matches_mode():
            failures.append("TrajectorySetpoint fields do not match OffboardControlMode")

        if failures:
            self.reported_ready = False
            self.get_logger().error(
                "PX4 offboard stream is not ready: "
                f"{failures[0]}. Details: {', '.join(details)}"
            )
            return

        if not self.reported_ready:
            self.reported_ready = True
            self.get_logger().info(
                "PX4 offboard stream is active with valid setpoints: " + ", ".join(details)
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Px4ControlWatchdog()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
