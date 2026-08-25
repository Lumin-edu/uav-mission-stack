#!/usr/bin/env python3
"""Stable AprilTag offsets for the independent sysu mission.

The camera is mounted 0.115 m in front of the payload/body origin.  It is
assumed to look straight down, with image top pointing toward the nose:
camera x -> RFU right and camera y -> RFU backward.  Only horizontal tag
offsets are published; the mission owns altitude control.
"""

import math
import time

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Bool, String


class StablePoint(object):
    def __init__(self, sample_count, jump_limit):
        if sample_count < 1 or jump_limit <= 0.0:
            raise ValueError("invalid AprilTag stability parameters")
        self.sample_count = sample_count
        self.jump_limit = jump_limit
        self.samples = []

    def reset(self):
        self.samples = []

    def add(self, point):
        if self.samples:
            distance = math.sqrt(sum(
                (point[index] - self.samples[-1][index]) ** 2
                for index in range(2)))
            if distance > self.jump_limit:
                self.reset()
        self.samples.append(list(point))
        self.samples = self.samples[-self.sample_count:]
        if len(self.samples) < self.sample_count:
            return None
        return [sum(sample[index] for sample in self.samples) /
                float(len(self.samples)) for index in range(2)]


class TagTracker(object):
    def __init__(self):
        self.drop_tag_id = int(rospy.get_param("~drop_tag_id", 0))
        self.landing_tag_id = int(rospy.get_param(
            "~landing_tag_id", self.drop_tag_id))
        self.tag_timeout = float(rospy.get_param("~tag_timeout", 0.8))
        self.stable_samples = int(rospy.get_param("~stable_samples", 5))
        self.jump_limit = float(rospy.get_param("~jump_limit", 0.15))
        self.camera_right_offset = float(rospy.get_param(
            "~camera_right_offset", 0.0))
        self.camera_forward_offset = float(rospy.get_param(
            "~camera_forward_offset", 0.115))
        if self.tag_timeout <= 0.0:
            raise ValueError("tag_timeout must be positive")

        self.trackers = {
            "drop": StablePoint(self.stable_samples, self.jump_limit),
            "landing": StablePoint(self.stable_samples, self.jump_limit),
        }
        self.ids = {
            "drop": self.drop_tag_id,
            "landing": self.landing_tag_id,
        }
        self.last_receive = {"drop": None, "landing": None}
        self.publishers = {
            "drop": rospy.Publisher("/bringup/tag/drop_offset_rfu",
                                     Vector3Stamped, queue_size=5),
            "landing": rospy.Publisher("/bringup/tag/landing_offset_rfu",
                                        Vector3Stamped, queue_size=5),
        }
        self.healthy_publishers = {
            "drop": rospy.Publisher("/bringup/tag/drop_healthy", Bool,
                                     queue_size=1, latch=True),
            "landing": rospy.Publisher("/bringup/tag/landing_healthy", Bool,
                                        queue_size=1, latch=True),
        }
        self.state_pub = rospy.Publisher("/bringup/tag/state", String,
                                         queue_size=1, latch=True)
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray,
                         self._detections_cb, queue_size=10)
        rospy.Timer(rospy.Duration(0.1), self._health_timer)
        self._publish_health()
        self.state_pub.publish(String(data="WAITING_DETECTIONS"))
        rospy.loginfo(
            "AprilTag tracker: drop_id=%d landing_id=%d camera_offset_RF=(%.3f, %.3f)",
            self.drop_tag_id, self.landing_tag_id,
            self.camera_right_offset, self.camera_forward_offset)

    @staticmethod
    def _find_detection(detections, tag_id):
        for detection in detections:
            if any(int(value) == int(tag_id) for value in detection.id):
                return detection
        return None

    @staticmethod
    def _finite_pose(pose):
        return all(math.isfinite(float(value)) for value in (
            pose.position.x, pose.position.y, pose.position.z))

    def _detections_cb(self, message):
        now = time.monotonic()
        found_any = False
        for name in ("drop", "landing"):
            detection = self._find_detection(message.detections,
                                              self.ids[name])
            if detection is None:
                continue
            pose = detection.pose.pose.pose
            if not self._finite_pose(pose):
                continue

            # apriltag_ros uses the camera optical frame: x right, y down,
            # z along the optical axis.  For a downward camera whose image
            # top is the aircraft forward direction, RFU=(x, -y, -z).
            point = [pose.position.x + self.camera_right_offset,
                     -pose.position.y + self.camera_forward_offset]
            average = self.trackers[name].add(point)
            if average is None:
                continue

            output = Vector3Stamped()
            output.header = detection.pose.header
            if not output.header.frame_id:
                output.header = message.header
            output.header.stamp = message.header.stamp or rospy.Time.now()
            output.header.frame_id = "rfu"
            output.vector.x = average[0]
            output.vector.y = average[1]
            output.vector.z = 0.0
            self.publishers[name].publish(output)
            self.last_receive[name] = now
            found_any = True

        if found_any:
            self.state_pub.publish(String(data="STABLE_TAG"))
        self._publish_health()

    def _publish_health(self):
        now = time.monotonic()
        for name in ("drop", "landing"):
            receive_time = self.last_receive[name]
            healthy = (receive_time is not None and
                       now - receive_time <= self.tag_timeout)
            self.healthy_publishers[name].publish(Bool(data=healthy))

    def _health_timer(self, unused_event):
        del unused_event
        now = time.monotonic()
        for name in ("drop", "landing"):
            receive_time = self.last_receive[name]
            if receive_time is not None and now - receive_time > self.tag_timeout:
                self.trackers[name].reset()
        self._publish_health()


if __name__ == "__main__":
    rospy.init_node("bringup_tag_tracker")
    try:
        TagTracker()
        rospy.spin()
    except ValueError as error:
        rospy.logfatal("AprilTag tracker parameter error: %s", error)
