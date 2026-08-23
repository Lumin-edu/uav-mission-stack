#!/usr/bin/env python3
"""Compare MAVROS local pose with the FAST-LIO bridge's ENU pose."""

import math
import time

import rospy
from geometry_msgs.msg import PoseStamped


class Px4FastLioPositionCompare(object):
    def __init__(self):
        self.px4_topic = rospy.get_param(
            "~px4_topic", "/mavros/local_position/pose")
        self.visual_odom_topic = rospy.get_param(
            "~visual_odom_topic", "/bringup/visual_odom")
        self.print_rate = float(rospy.get_param("~print_rate", 2.0))
        self.warn_timeout = float(rospy.get_param("~warn_timeout", 1.0))
        self.direction_epsilon = float(rospy.get_param("~direction_epsilon", 0.15))
        self.latest_px4 = None
        self.latest_visual = None
        self.latest_px4_time = None
        self.latest_visual_time = None
        self.ref_px4 = None
        self.ref_visual = None
        rospy.Subscriber(self.px4_topic, PoseStamped, self.px4_callback,
                         queue_size=10)
        rospy.Subscriber(self.visual_odom_topic, PoseStamped,
                         self.visual_callback, queue_size=10)
        period = 1.0 / self.print_rate if self.print_rate > 0.0 else 0.5
        rospy.Timer(rospy.Duration(period), self.timer_callback)
        rospy.loginfo(
            "Comparing MAVROS local pose and FAST-LIO bridge: px4=%s, "
            "visual_odom=%s, print_rate=%.2f Hz", self.px4_topic,
            self.visual_odom_topic, self.print_rate)

    @staticmethod
    def position(message):
        return (float(message.pose.position.x), float(message.pose.position.y),
                float(message.pose.position.z))

    def px4_callback(self, message):
        self.latest_px4 = message
        self.latest_px4_time = time.monotonic()

    def visual_callback(self, message):
        self.latest_visual = message
        self.latest_visual_time = time.monotonic()

    def axis_direction(self, value):
        if value > self.direction_epsilon:
            return "+"
        if value < -self.direction_epsilon:
            return "-"
        return "0"

    def timer_callback(self, _event):
        if self.latest_px4 is None or self.latest_visual is None:
            rospy.logwarn_throttle(
                2.0, "Waiting for position data: px4_received=%s, "
                "fastlio_bridge_received=%s", self.latest_px4 is not None,
                self.latest_visual is not None)
            return
        now = time.monotonic()
        px4_age = now - self.latest_px4_time
        visual_age = now - self.latest_visual_time
        if px4_age > self.warn_timeout or visual_age > self.warn_timeout:
            rospy.logwarn("Stale position input: px4_age=%.2fs, fastlio_age=%.2fs",
                          px4_age, visual_age)
        px4 = self.position(self.latest_px4)
        fastlio = self.position(self.latest_visual)
        if not all(math.isfinite(value) for value in px4 + fastlio):
            rospy.logwarn("Non-finite position input; skipping comparison")
            return
        if self.ref_px4 is None:
            self.ref_px4 = px4
        if self.ref_visual is None:
            self.ref_visual = fastlio
        px4_rel = tuple(px4[i] - self.ref_px4[i] for i in range(3))
        fastlio_rel = tuple(fastlio[i] - self.ref_visual[i] for i in range(3))
        diff = tuple(fastlio_rel[i] - px4_rel[i] for i in range(3))
        norm = math.sqrt(sum(value * value for value in diff))
        axes = []
        mismatch = []
        for label, px4_value, fastlio_value in zip("xyz", px4_rel, fastlio_rel):
            px4_dir = self.axis_direction(px4_value)
            fastlio_dir = self.axis_direction(fastlio_value)
            axes.append("%s[px4=%s, fastlio=%s]" %
                        (label, px4_dir, fastlio_dir))
            if px4_dir != "0" and fastlio_dir != "0" and px4_dir != fastlio_dir:
                mismatch.append(label)
        axis_text = " ".join(axes)
        if mismatch:
            axis_text += " | direction mismatch on " + ",".join(mismatch)
        else:
            axis_text += " | direction looks consistent"
        rospy.loginfo(
            "ENU increment compare | PX4=(%+.2f, %+.2f, %+.2f) m | "
            "FAST-LIO=(%+.2f, %+.2f, %+.2f) m | diff=(%+.2f, %+.2f, %+.2f) m | "
            "|diff|=%.2f m | %s", px4_rel[0], px4_rel[1], px4_rel[2],
            fastlio_rel[0], fastlio_rel[1], fastlio_rel[2], diff[0], diff[1],
            diff[2], norm, axis_text)


if __name__ == "__main__":
    rospy.init_node("px4_fastlio_position_compare")
    Px4FastLioPositionCompare()
    rospy.spin()
