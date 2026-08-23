#!/usr/bin/env python3
"""Safe fixed-point Offboard controller for a FAST-LIO local frame."""

import math
import time

import rospy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from mavros_msgs.msg import EstimatorStatus, State
from mavros_msgs.srv import SetMode
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


def _as_bool(value):
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def _valid_pose(pose):
    values = (pose.position.x, pose.position.y, pose.position.z,
              pose.orientation.x, pose.orientation.y,
              pose.orientation.z, pose.orientation.w)
    if not all(math.isfinite(float(value)) for value in values):
        return False
    return math.sqrt(sum(float(value) * float(value)
                        for value in values[3:])) > 1.0e-6


def _position(pose):
    return [pose.position.x, pose.position.y, pose.position.z]


def _quaternion(pose):
    values = [pose.orientation.x, pose.orientation.y,
              pose.orientation.z, pose.orientation.w]
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1.0e-9:
        raise ValueError("zero quaternion")
    return [value / norm for value in values]


def _yaw(pose):
    x, y, z, w = _quaternion(pose)
    return math.atan2(2.0 * (w * z + x * y),
                     1.0 - 2.0 * (y * y + z * z))


def _yaw_quaternion(yaw):
    return [0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)]


def _angle_error(first, second):
    return math.atan2(math.sin(first - second), math.cos(first - second))


def _transform_offset(_fastlio_q, px4_q, task_point):
    """Map a body RFU task offset into the PX4/MAVROS local ENU frame.

    Task coordinates follow the reference convention: (right, forward, up).
    FAST-LIO source coordinates are handled by the visual-odometry bridge.
    """
    px4_yaw = math.atan2(2.0 * (px4_q[3] * px4_q[2] +
                               px4_q[0] * px4_q[1]),
                          1.0 - 2.0 * (px4_q[1] ** 2 + px4_q[2] ** 2))
    right, forward, up = task_point
    return [math.sin(px4_yaw) * right + math.cos(px4_yaw) * forward,
            -math.cos(px4_yaw) * right + math.sin(px4_yaw) * forward, up]


class FixedPointOffboard(object):
    def __init__(self):
        self.rate_hz = float(rospy.get_param("~rate", 20.0))
        self.origin_delay = float(rospy.get_param("~origin_delay", 4.0))
        self.data_timeout = float(rospy.get_param("~data_timeout", 0.6))
        self.mode_retry_period = float(rospy.get_param(
            "~mode_retry_period", 1.0))
        self.max_speed = float(rospy.get_param("~max_speed", 0.8))
        self.position_tolerance = float(rospy.get_param(
            "~position_tolerance", 0.25))
        self.reached_hold_time = float(rospy.get_param(
            "~reached_hold_time", 1.0))
        self.frame_alignment_tolerance = float(rospy.get_param(
            "~frame_alignment_tolerance", 0.75))
        self.yaw_alignment_tolerance = float(rospy.get_param(
            "~yaw_alignment_tolerance", 0.5))
        self.max_distance = float(rospy.get_param("~max_distance", 20.0))
        self.require_vision_health = _as_bool(rospy.get_param(
            "~require_vision_health", True))
        self.require_estimator_health = _as_bool(rospy.get_param(
            "~require_estimator_health", False))
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.fastlio_topic = rospy.get_param("~fastlio_topic", "/Odometry")
        self.local_pose_topic = rospy.get_param(
            "~local_pose_topic", "/mavros/local_position/pose")
        self.vision_pose_topic = rospy.get_param(
            "~vision_pose_topic", "/mavros/vision_pose/pose")
        self.setpoint_topic = rospy.get_param(
            "~setpoint_topic", "/mavros/setpoint_position/local")
        self.vision_health_topic = rospy.get_param(
            "~vision_health_topic", "/fastlio_to_mavros_visual_odom/healthy")
        # ``target_*`` follows the ROS2 reference launch.  Keep x/y/z as
        # legacy aliases so existing ROS1 launch files remain usable.
        self.task_point = [
            float(rospy.get_param("~target_x", rospy.get_param("~x", 0.0))),
            float(rospy.get_param("~target_y", rospy.get_param("~y", 0.0))),
            float(rospy.get_param("~target_z", rospy.get_param("~z", 0.0))),
        ]
        self.target_yaw = float(rospy.get_param("~target_yaw", 0.0))
        # Keep the vehicle heading equal to the PX4 heading captured with the
        # hover reference.  target_yaw is an optional fixed offset, normally 0.
        self.lock_yaw = _as_bool(rospy.get_param("~lock_yaw", True))
        if self.max_speed <= 0.0 or self.rate_hz <= 0.0:
            raise ValueError("rate and max_speed must be positive")
        if self.position_tolerance <= 0.0 or self.reached_hold_time < 0.0:
            raise ValueError("invalid target tolerance")
        if math.sqrt(sum(value * value for value in self.task_point)) > self.max_distance:
            raise ValueError("task point exceeds max_distance")

        self.started_at = time.monotonic()
        self.last_fastlio = None
        self.last_fastlio_receive = None
        self.last_local_pose = None
        self.last_local_receive = None
        self.last_vision_pose = None
        self.last_vision_receive = None
        self.last_estimator = None
        self.last_estimator_receive = None
        self.state = State()
        self.vision_healthy = not self.require_vision_health
        self.origin_captured = False
        self.fastlio_origin = None
        self.fastlio_origin_q = None
        self.px4_origin = None
        self.px4_origin_q = None
        self.target_position = None
        self.target_q = None
        self.locked_yaw = None
        self.command_position = None
        self.reached_since = None
        self.target_reached = False
        self.last_mode_request = 0.0
        self.last_wait_log = 0.0
        self.last_command_time = time.monotonic()

        self.setpoint_pub = rospy.Publisher(self.setpoint_topic, PoseStamped,
                                            queue_size=20)
        self.target_pub = rospy.Publisher("~target", PoseStamped,
                                          queue_size=1, latch=True)
        self.origin_pub = rospy.Publisher("~origin", PoseStamped,
                                          queue_size=1, latch=True)
        self.state_pub = rospy.Publisher("~state", String,
                                         queue_size=1, latch=True)
        self.active_pub = rospy.Publisher("~active", Bool,
                                          queue_size=1, latch=True)
        self.reached_pub = rospy.Publisher("~target_reached", Bool,
                                           queue_size=1, latch=True)
        self.error_pub = rospy.Publisher("~target_error", Vector3Stamped,
                                         queue_size=1)
        rospy.Subscriber(self.fastlio_topic, Odometry, self._fastlio_cb,
                         queue_size=10)
        rospy.Subscriber(self.local_pose_topic, PoseStamped,
                         self._local_pose_cb, queue_size=10)
        rospy.Subscriber(self.vision_pose_topic, PoseStamped,
                         self._vision_pose_cb, queue_size=10)
        rospy.Subscriber("/mavros/state", State, self._state_cb,
                         queue_size=10)
        rospy.Subscriber("/mavros/estimator_status", EstimatorStatus,
                         self._estimator_cb, queue_size=10)
        rospy.Subscriber(self.vision_health_topic, Bool,
                         self._vision_health_cb, queue_size=10)
        self.mode_service = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self._publish_state("WAITING_FASTLIO")
        self.reached_pub.publish(Bool(data=False))
        rospy.loginfo("Fixed-point controller started; task RFU "
                      "(right=%.3f, forward=%.3f, up=%.3f), "
                      "target_yaw=%.3f, lock_yaw=%s",
                      self.task_point[0], self.task_point[1], self.task_point[2],
                      self.target_yaw, self.lock_yaw)

    def _publish_state(self, value):
        self.state_pub.publish(String(data=value))
        self.active_pub.publish(Bool(data=True))

    def _fastlio_cb(self, message):
        if _valid_pose(message.pose.pose):
            self.last_fastlio = message
            self.last_fastlio_receive = time.monotonic()

    def _local_pose_cb(self, message):
        if _valid_pose(message.pose):
            self.last_local_pose = message
            self.last_local_receive = time.monotonic()

    def _vision_pose_cb(self, message):
        if _valid_pose(message.pose):
            self.last_vision_pose = message
            self.last_vision_receive = time.monotonic()

    def _state_cb(self, message):
        self.state = message

    def _estimator_cb(self, message):
        self.last_estimator = message
        self.last_estimator_receive = time.monotonic()

    def _vision_health_cb(self, message):
        self.vision_healthy = bool(message.data) or not self.require_vision_health

    def _fresh(self, receive_time):
        return receive_time is not None and time.monotonic() - receive_time <= self.data_timeout

    def _make_pose(self, position, quaternion):
        message = PoseStamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.frame_id
        message.pose.position.x, message.pose.position.y, message.pose.position.z = position
        message.pose.orientation.x, message.pose.orientation.y = quaternion[0:2]
        message.pose.orientation.z, message.pose.orientation.w = quaternion[2:4]
        return message

    def _wait(self, source):
        if time.monotonic() - self.last_wait_log >= 2.0:
            rospy.loginfo("Waiting for %s before origin capture", source)
            self.last_wait_log = time.monotonic()

    def _capture_origin(self):
        if self.origin_captured:
            return True
        checks = ((self.last_fastlio_receive, "FAST-LIO"),
                  (self.last_local_receive, "PX4 local pose"),
                  (self.last_vision_receive, "external vision"))
        for receive_time, source in checks:
            if not self._fresh(receive_time):
                self._wait(source)
                return False
        if self.require_vision_health and not self.vision_healthy:
            self._wait("external vision health")
            return False
        if self.require_estimator_health:
            if not self._fresh(self.last_estimator_receive):
                self._wait("PX4 estimator status")
                return False
            if (not self.last_estimator.attitude_status_flag or
                    not self.last_estimator.pos_horiz_rel_status_flag or
                    not (self.last_estimator.pos_vert_agl_status_flag or
                         self.last_estimator.pos_vert_abs_status_flag)):
                self._wait("PX4 estimator position/attitude")
                return False
        fastlio_pose = self.last_fastlio.pose.pose
        px4_pose = self.last_local_pose.pose
        vision_pose = self.last_vision_pose.pose
        try:
            fastlio_q = _quaternion(fastlio_pose)
            px4_q = _quaternion(px4_pose)
            vision_yaw = _yaw(vision_pose)
            px4_yaw = _yaw(px4_pose)
        except ValueError:
            self._wait("valid quaternion")
            return False
        horizontal_error = math.sqrt((vision_pose.position.x - px4_pose.position.x) ** 2 +
                                     (vision_pose.position.y - px4_pose.position.y) ** 2)
        yaw_error = abs(_angle_error(vision_yaw, px4_yaw))
        if horizontal_error > self.frame_alignment_tolerance:
            rospy.logwarn_throttle(2.0, "vision/PX4 horizontal error %.3f m", horizontal_error)
            self._wait("vision/PX4 frame alignment")
            return False
        if yaw_error > self.yaw_alignment_tolerance:
            rospy.logwarn_throttle(2.0, "vision/PX4 yaw error %.3f rad", yaw_error)
            self._wait("vision/PX4 yaw alignment")
            return False
        self.fastlio_origin = _position(fastlio_pose)
        self.fastlio_origin_q = fastlio_q
        self.px4_origin = _position(px4_pose)
        self.px4_origin_q = px4_q
        task_offset = _transform_offset(fastlio_q, px4_q, self.task_point)
        self.target_position = [self.px4_origin[index] + task_offset[index]
                                for index in range(3)]
        self.locked_yaw = px4_yaw
        self.target_q = _yaw_quaternion(
            self.locked_yaw + self.target_yaw)
        self.command_position = list(self.px4_origin)
        self.last_command_time = time.monotonic()
        self.origin_captured = True
        self.origin_pub.publish(self._make_pose(self.px4_origin, self.target_q))
        self.target_pub.publish(self._make_pose(self.target_position, self.target_q))
        self._publish_state("WAITING_RC_ARM")
        rospy.loginfo("Origin captured; PX4 target=(%.3f, %.3f, %.3f)",
                      self.target_position[0], self.target_position[1],
                      self.target_position[2])
        return True

    def _heartbeat_target(self):
        if not self.origin_captured:
            if self.last_local_pose is not None:
                pose = self.last_local_pose.pose
                return _position(pose), _yaw_quaternion(_yaw(pose))
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]
        if (self.state.armed and self.state.mode == "OFFBOARD" and
                self.target_position is not None):
            now = time.monotonic()
            elapsed = min(max(now - self.last_command_time, 0.0), 0.2)
            self.last_command_time = now
            delta = [self.target_position[index] - self.command_position[index]
                     for index in range(3)]
            distance = math.sqrt(sum(value * value for value in delta))
            if distance > 1.0e-9:
                step = min(self.max_speed * elapsed, distance)
                scale = step / distance
                self.command_position = [
                    self.command_position[index] + delta[index] * scale
                    for index in range(3)]
        if self.lock_yaw or self.last_local_pose is None:
            command_q = self.target_q
        else:
            command_q = _yaw_quaternion(
                _yaw(self.last_local_pose.pose) + self.target_yaw)
        return self.command_position, command_q

    def _request_offboard(self):
        if (not self.origin_captured or not self.state.connected or
                not self.state.armed):
            return
        if self.require_vision_health and not self.vision_healthy:
            self._publish_state("WAITING_VISION")
            return
        if self.state.mode == "OFFBOARD":
            self._publish_state("TARGET_REACHED" if self.target_reached else "OFFBOARD")
            return
        now = time.monotonic()
        if now - self.last_mode_request < self.mode_retry_period:
            return
        self.last_mode_request = now
        self._publish_state("REQUEST_OFFBOARD")
        try:
            response = self.mode_service(base_mode=0, custom_mode="OFFBOARD")
            if not response.mode_sent:
                rospy.logwarn("PX4 rejected OFFBOARD request")
        except rospy.ServiceException as error:
            rospy.logwarn_throttle(2.0, "OFFBOARD service unavailable: %s", error)

    def _update_reached(self):
        if (not self.origin_captured or not self._fresh(self.last_local_receive) or
                self.target_position is None):
            self.reached_since = None
            self.target_reached = False
            self.reached_pub.publish(Bool(data=False))
            return
        current = _position(self.last_local_pose.pose)
        error = [self.target_position[index] - current[index] for index in range(3)]
        message = Vector3Stamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.frame_id
        message.vector.x, message.vector.y, message.vector.z = error
        self.error_pub.publish(message)
        within = math.sqrt(sum(value * value for value in error)) <= self.position_tolerance
        if self.state.mode != "OFFBOARD" or not self.state.armed or not within:
            self.reached_since = None
        elif self.reached_since is None:
            self.reached_since = time.monotonic()
        self.target_reached = (self.reached_since is not None and
                               time.monotonic() - self.reached_since >= self.reached_hold_time)
        self.reached_pub.publish(Bool(data=self.target_reached))

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if time.monotonic() - self.started_at >= self.origin_delay:
                self._capture_origin()
            position, quaternion = self._heartbeat_target()
            self.setpoint_pub.publish(self._make_pose(position, quaternion))
            self._update_reached()
            self._request_offboard()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("fixed_point_offboard")
    try:
        FixedPointOffboard().run()
    except ValueError as error:
        rospy.logfatal("Fixed-point parameter error: %s", error)
