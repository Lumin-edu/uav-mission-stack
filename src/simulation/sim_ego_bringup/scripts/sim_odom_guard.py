#!/usr/bin/env python3

import math
import time
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class SimOdomGuard(Node):
    """Forward only physically plausible Point-LIO odometry to the planner."""

    def __init__(self) -> None:
        super().__init__("sim_odom_guard")
        self.input_topic = str(
            self.declare_parameter("input_topic", "/sim/pointlio/odom").value
        )
        self.output_topic = str(
            self.declare_parameter("output_topic", "/sim/pointlio/odom_safe").value
        )
        self.health_topic = str(
            self.declare_parameter(
                "health_topic", "/sim/pointlio/odom_healthy"
            ).value
        )
        self.max_speed_mps = float(self.declare_parameter("max_speed_mps", 2.5).value)
        self.max_jump_m = float(self.declare_parameter("max_jump_m", 1.0).value)
        self.max_validation_dt_sec = float(
            self.declare_parameter("max_validation_dt_sec", 0.25).value
        )
        self.input_timeout_sec = float(
            self.declare_parameter("input_timeout_sec", 0.5).value
        )
        self.max_consecutive_rejections = int(
            self.declare_parameter("max_consecutive_rejections", 3).value
        )
        self.max_horizontal_distance_m = float(
            self.declare_parameter("max_horizontal_distance_m", 12.0).value
        )
        self.max_vertical_distance_m = float(
            self.declare_parameter("max_vertical_distance_m", 3.0).value
        )
        if min(
            self.max_speed_mps,
            self.max_jump_m,
            self.max_validation_dt_sec,
            self.input_timeout_sec,
        ) <= 0.0:
            raise ValueError("odom guard motion limits must be positive")
        if self.max_consecutive_rejections < 1:
            raise ValueError("max_consecutive_rejections must be at least one")

        self.publisher = self.create_publisher(Odometry, self.output_topic, 10)
        health_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.health_publisher = self.create_publisher(Bool, self.health_topic, health_qos)
        self.create_subscription(Odometry, self.input_topic, self.odom_callback, 10)
        self.reference: Optional[tuple[float, float, float]] = None
        self.last_position: Optional[tuple[float, float, float]] = None
        self.last_raw_sample_sec: Optional[float] = None
        self.last_raw_wall_sec: Optional[float] = None
        self.last_warn_sec = -math.inf
        self.accepted_count = 0
        self.rejected_count = 0
        self.consecutive_rejections = 0
        self.fault_latched = False
        self.last_health: Optional[bool] = None
        self.create_timer(
            min(0.1, self.input_timeout_sec / 2.0), self.health_timer_callback
        )
        self.publish_health(False)
        self.get_logger().info(
            "Simulation odom guard: "
            f"{self.input_topic} -> {self.output_topic}, "
            f"speed<={self.max_speed_mps:.1f} m/s, jump<={self.max_jump_m:.1f} m, "
            f"horizontal<={self.max_horizontal_distance_m:.1f} m, "
            f"vertical<={self.max_vertical_distance_m:.1f} m, "
            f"input_timeout={self.input_timeout_sec:.2f} s, "
            f"fault_after={self.max_consecutive_rejections} rejects"
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def warn(self, message: str) -> None:
        now = self.now_sec()
        if now - self.last_warn_sec >= 1.0:
            self.last_warn_sec = now
            self.get_logger().warn(message)

    @staticmethod
    def stamp_sec(msg: Odometry) -> Optional[float]:
        value = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1e9
        return value if value > 0.0 and math.isfinite(value) else None

    @staticmethod
    def finite_position(msg: Odometry) -> Optional[tuple[float, float, float]]:
        p = msg.pose.pose.position
        values = (float(p.x), float(p.y), float(p.z))
        return values if all(math.isfinite(value) for value in values) else None

    def publish_health(self, healthy: bool, force: bool = False) -> None:
        if not force and self.last_health is healthy:
            return
        self.health_publisher.publish(Bool(data=healthy))
        self.last_health = healthy

    def reject(self, reason: str) -> None:
        self.rejected_count += 1
        self.consecutive_rejections += 1
        self.warn(f"Rejecting Point-LIO odometry: {reason}")
        self.publish_health(False)
        if self.consecutive_rejections >= self.max_consecutive_rejections:
            self.fault_latched = True
            self.get_logger().error(
                "Point-LIO odometry fault latched after "
                f"{self.consecutive_rejections} consecutive rejected samples. "
                "Safe odometry output will remain stopped until the simulation is restarted."
            )

    def health_timer_callback(self) -> None:
        if self.fault_latched or self.last_raw_wall_sec is None:
            return
        age = time.monotonic() - self.last_raw_wall_sec
        if age <= self.input_timeout_sec:
            return
        self.fault_latched = True
        self.publish_health(False)
        self.get_logger().error(
            "Point-LIO odometry input timed out after "
            f"{age:.2f} s (limit {self.input_timeout_sec:.2f} s). "
            "Safe odometry output will remain stopped until the simulation is restarted."
        )

    def odom_callback(self, msg: Odometry) -> None:
        position = self.finite_position(msg)
        if position is None:
            self.reject("non-finite position")
            return

        # Reception timeout must use a steady wall clock. /clock can appear or
        # jump when Gazebo starts, and mixing that jump with a pre-clock zero
        # timestamp would latch a false stale-input fault.
        now_wall = time.monotonic()
        sample_sec = self.stamp_sec(msg)
        if sample_sec is None:
            sample_sec = now_wall
        previous_raw_sample_sec = self.last_raw_sample_sec
        previous_raw_wall_sec = self.last_raw_wall_sec
        self.last_raw_sample_sec = sample_sec
        self.last_raw_wall_sec = now_wall

        if self.fault_latched:
            return

        if self.reference is None:
            self.reference = position
            self.last_position = position
            self.publisher.publish(msg)
            self.accepted_count += 1
            self.consecutive_rejections = 0
            self.publish_health(True)
            self.get_logger().info(
                "Captured safe odom reference: "
                f"({position[0]:+.3f}, {position[1]:+.3f}, {position[2]:+.3f})"
            )
            return

        horizontal = math.hypot(
            position[0] - self.reference[0], position[1] - self.reference[1]
        )
        vertical = abs(position[2] - self.reference[2])
        if horizontal > self.max_horizontal_distance_m:
            self.reject(f"horizontal drift {horizontal:.2f} m")
            return
        if vertical > self.max_vertical_distance_m:
            self.reject(f"vertical drift {vertical:.2f} m")
            return

        dt = None
        if previous_raw_sample_sec is not None and sample_sec > previous_raw_sample_sec:
            dt = sample_sec - previous_raw_sample_sec
        elif previous_raw_wall_sec is not None and now_wall > previous_raw_wall_sec:
            dt = now_wall - previous_raw_wall_sec
        if dt is None or dt <= 0.0:
            self.reject("non-monotonic timestamp")
            return

        delta = math.sqrt(
            sum((position[index] - self.last_position[index]) ** 2 for index in range(3))
        )
        # Use consecutive raw-sample timing and cap the validation interval. A
        # rejected sample must never enlarge the distance that a later sample
        # may move away from the last accepted estimator state.
        validation_dt = min(max(dt, 0.02), self.max_validation_dt_sec)
        allowed_jump = min(self.max_jump_m, self.max_speed_mps * validation_dt)
        if delta > allowed_jump:
            self.reject(f"jump {delta:.2f} m in {dt:.3f} s")
            return

        self.publisher.publish(msg)
        self.last_position = position
        self.accepted_count += 1
        self.consecutive_rejections = 0
        self.publish_health(True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimOdomGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
