#!/usr/bin/env python3
"""FAST-LIO -> MAVROS external-vision bridge.

The bridge keeps the reference implementation's stages:
  source odometry -> optional base_link/base lever arm -> initial reference
  -> source-frame to PX4-compatible local coordinates -> MAVROS PoseStamped.

The lever arm is intentionally zero by default.  Its parameters stay in the
interface so a later airframe calibration does not require changing the node.
"""

import math
import time

import rospy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


def as_bool(value):
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def normalize_quaternion(values):
    values = [float(value) for value in values]
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1.0e-12 or not math.isfinite(norm):
        raise ValueError("quaternion norm is zero or non-finite")
    return [value / norm for value in values]


def quaternion_multiply(lhs, rhs):
    x1, y1, z1, w1 = normalize_quaternion(lhs)
    x2, y2, z2, w2 = normalize_quaternion(rhs)
    return normalize_quaternion([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ])


def rotate_vector(quaternion, vector):
    x, y, z, w = normalize_quaternion(quaternion)
    vx, vy, vz = [float(value) for value in vector]
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return [vx + w * tx + y * tz - z * ty,
            vy + w * ty + z * tx - x * tz,
            vz + w * tz + x * ty - y * tx]


def finite_pose(pose):
    values = (pose.position.x, pose.position.y, pose.position.z,
              pose.orientation.x, pose.orientation.y,
              pose.orientation.z, pose.orientation.w)
    return all(math.isfinite(float(value)) for value in values)


def quaternion_to_yaw(quaternion):
    x, y, z, w = normalize_quaternion(quaternion)
    return math.atan2(2.0 * (w * z + x * y),
                     1.0 - 2.0 * (y * y + z * z))


class FastLioToMavrosVisualOdom(object):
    def __init__(self):
        self.fastlio_topic = rospy.get_param("~fastlio_topic", "/Odometry")
        self.output_topic = rospy.get_param(
            "~output_topic", "/mavros/vision_pose/pose")
        self.preview_topic = rospy.get_param(
            "~preview_topic", "/bringup/visual_odom")
        self.output_frame = rospy.get_param("~output_frame", "map")
        self.fastlio_pose_frame = rospy.get_param(
            "~fastlio_pose_frame", "body")
        self.vehicle_frame = rospy.get_param("~vehicle_frame", "base")

        # T_base_link_base.  Zero lever arm preserves the FAST-LIO pose while
        # retaining the same calibration boundary as the reference package.
        self.base_link_to_base_translation = self.vector_param(
            "~base_link_to_base_translation", [0.0, 0.0, 0.0], 3)
        self.base_link_to_base_rotation = normalize_quaternion(
            self.vector_param("~base_link_to_base_rotation_xyzw",
                              [0.0, 0.0, 0.0, 1.0], 4))

        # FAST-LIO's ROS1 odometry is passed to MAVROS as ENU.  MAVROS then
        # performs the standard ENU -> PX4 NED conversion internally.  This
        # identity default is therefore equivalent to the reference bridge's
        # direct PX4-NED mapping [x, -y, -z].
        self.source_to_enu = self.matrix_param(
            "~source_to_enu_rotation",
            [1.0, 0.0, 0.0,
             0.0, 1.0, 0.0,
             0.0, 0.0, 1.0])
        self.publish_orientation = as_bool(rospy.get_param(
            "~publish_orientation", True))
        self.publish_rate_limit = float(rospy.get_param(
            "~publish_rate_limit", 50.0))
        self.max_source_age = float(rospy.get_param("~max_source_age", 0.5))
        self.max_position_jump = float(rospy.get_param(
            "~max_position_jump", 3.0))
        self.max_output_abs_z = float(rospy.get_param(
            "~max_output_abs_z", 20.0))
        self.print_rate = float(rospy.get_param("~print_rate", 1.0))

        self.reference_captured = False
        self.reference_position = [0.0, 0.0, 0.0]
        self.last_output_position = None
        self.last_source_stamp = None
        self.last_publish_time = 0.0
        self.last_receive_time = None
        self.last_print_time = 0.0

        self.output_pub = rospy.Publisher(self.output_topic, PoseStamped,
                                          queue_size=20)
        self.preview_pub = rospy.Publisher(self.preview_topic, PoseStamped,
                                           queue_size=20)
        self.healthy_pub = rospy.Publisher("~healthy", Bool, queue_size=1,
                                           latch=True)
        self.state_pub = rospy.Publisher("~state", String, queue_size=1,
                                         latch=True)
        rospy.Subscriber(self.fastlio_topic, Odometry, self.fastlio_callback,
                         queue_size=20)
        rospy.Timer(rospy.Duration(0.1), self.timeout_callback)
        self.publish_state(False, "WAITING_FASTLIO")
        rospy.loginfo(
            "FAST-LIO visual bridge: input=%s output=%s frame=%s "
            "pose_interpretation=T_%s_%s vehicle_frame=%s "
            "t_base_link_base=(%.4f, %.4f, %.4f)",
            self.fastlio_topic, self.output_topic, self.output_frame,
            self.output_frame, self.fastlio_pose_frame, self.vehicle_frame,
            self.base_link_to_base_translation[0],
            self.base_link_to_base_translation[1],
            self.base_link_to_base_translation[2])

    @staticmethod
    def vector_param(name, default, size):
        raw = rospy.get_param(name, default)
        if isinstance(raw, str):
            raw = yaml.safe_load(raw)
        values = [float(value) for value in raw]
        if len(values) != size or not all(math.isfinite(value) for value in values):
            raise ValueError("%s must contain %d finite values" % (name, size))
        return values

    @staticmethod
    def matrix_param(name, default):
        raw = rospy.get_param(name, default)
        if isinstance(raw, str):
            raw = yaml.safe_load(raw)
        values = [float(value) for value in raw]
        if len(values) != 9 or not all(math.isfinite(value) for value in values):
            raise ValueError("%s must contain 9 finite values" % name)
        return values

    def publish_state(self, healthy, state):
        self.healthy_pub.publish(Bool(data=bool(healthy)))
        self.state_pub.publish(String(data=state))

    def corrected_pose(self, message):
        pose = message.pose.pose
        position = [pose.position.x, pose.position.y, pose.position.z]
        orientation = normalize_quaternion([
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w])
        # T_odom_base = T_odom_base_link * T_base_link_base.
        rotated_offset = rotate_vector(self.base_link_to_base_rotation,
                                       self.base_link_to_base_translation)
        # With the default identity rotation this is exactly zero.  Keeping
        # the composition explicit prevents a future calibration from turning
        # the lever arm into a world-frame constant.
        rotated_offset = rotate_vector(orientation, rotated_offset)
        corrected_position = [position[index] + rotated_offset[index]
                              for index in range(3)]
        corrected_orientation = quaternion_multiply(
            orientation, self.base_link_to_base_rotation)
        return corrected_position, corrected_orientation

    def map_position(self, corrected_position):
        offset = [corrected_position[index] - self.reference_position[index]
                  for index in range(3)]
        matrix = self.source_to_enu
        return [matrix[0] * offset[0] + matrix[1] * offset[1] + matrix[2] * offset[2],
                matrix[3] * offset[0] + matrix[4] * offset[1] + matrix[5] * offset[2],
                matrix[6] * offset[0] + matrix[7] * offset[1] + matrix[8] * offset[2]]

    def map_orientation(self, corrected_orientation):
        # For the default identity source-to-ENU rotation this is unchanged;
        # custom rotations still receive the corresponding static yaw.
        static_yaw = math.atan2(self.source_to_enu[3], self.source_to_enu[0])
        static_orientation = [0.0, 0.0, math.sin(0.5 * static_yaw),
                              math.cos(0.5 * static_yaw)]
        return quaternion_multiply(static_orientation, corrected_orientation)

    def should_publish(self):
        if self.publish_rate_limit <= 0.0:
            return True
        now = time.monotonic()
        if now - self.last_publish_time >= 1.0 / self.publish_rate_limit:
            self.last_publish_time = now
            return True
        return False

    def reject(self, reason):
        self.publish_state(False, reason)
        rospy.logwarn_throttle(2.0, "FAST-LIO visual odometry rejected: %s", reason)

    def fastlio_callback(self, message):
        stamp = message.header.stamp
        if stamp.is_zero():
            self.reject("STAMP")
            return
        age = (rospy.Time.now() - stamp).to_sec()
        if age > self.max_source_age or age < -0.1:
            self.reject("STALE_OR_FUTURE_STAMP")
            return
        if not finite_pose(message.pose.pose):
            self.reject("NONFINITE")
            return
        try:
            corrected_position, corrected_orientation = self.corrected_pose(message)
        except ValueError:
            self.reject("INVALID_POSE")
            return
        if not self.reference_captured:
            self.reference_position = list(corrected_position)
            self.reference_captured = True
            rospy.loginfo(
                "Captured corrected visual-odom reference: "
                "(%+.3f, %+.3f, %+.3f), child=%s",
                corrected_position[0], corrected_position[1],
                corrected_position[2], self.vehicle_frame)
        if not self.should_publish():
            return
        output_position = self.map_position(corrected_position)
        if (not all(math.isfinite(value) for value in output_position) or
                abs(output_position[2]) > self.max_output_abs_z):
            self.reject("OUTPUT_POSITION")
            return
        if self.last_output_position is not None:
            jump = math.sqrt(sum((output_position[index] -
                                  self.last_output_position[index]) ** 2
                                 for index in range(3)))
            if jump > self.max_position_jump:
                self.reject("POSITION_JUMP")
                return
        output = PoseStamped()
        output.header.stamp = stamp
        output.header.frame_id = self.output_frame
        output.pose.position.x, output.pose.position.y, output.pose.position.z = output_position
        if self.publish_orientation:
            output_orientation = self.map_orientation(corrected_orientation)
            output.pose.orientation.x, output.pose.orientation.y = output_orientation[0:2]
            output.pose.orientation.z, output.pose.orientation.w = output_orientation[2:4]
        else:
            output.pose.orientation.w = 1.0
        self.output_pub.publish(output)
        self.preview_pub.publish(output)
        self.last_output_position = output_position
        self.last_source_stamp = stamp
        self.last_receive_time = time.monotonic()
        self.publish_state(True, "OK")
        if (self.print_rate > 0.0 and
                time.monotonic() - self.last_print_time >= 1.0 / self.print_rate):
            self.last_print_time = time.monotonic()
            rospy.loginfo("visual odom published | ENU=(%+.2f, %+.2f, %+.2f)",
                          output_position[0], output_position[1], output_position[2])

    def timeout_callback(self, _event):
        if (self.last_receive_time is not None and
                time.monotonic() - self.last_receive_time > self.max_source_age):
            self.publish_state(False, "TIMEOUT")


if __name__ == "__main__":
    rospy.init_node("fastlio_to_mavros_visual_odom")
    try:
        FastLioToMavrosVisualOdom()
        rospy.spin()
    except ValueError as error:
        rospy.logfatal("FAST-LIO bridge parameter error: %s", error)
