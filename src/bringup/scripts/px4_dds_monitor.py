#!/usr/bin/env python3
"""MAVROS equivalent of the reference PX4 DDS monitor."""

import math

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State


class Px4MavrosMonitor(object):
    def __init__(self):
        self.vehicle_status_topic = rospy.get_param(
            "~vehicle_status_topic", "/mavros/state")
        self.local_position_topic = rospy.get_param(
            "~vehicle_local_position_topic", "/mavros/local_position/pose")

        self.status_received = False
        self.local_position_received = False
        self.last_state = None
        self.last_pose = None

        rospy.Subscriber(self.vehicle_status_topic, State,
                         self.vehicle_status_callback, queue_size=10)
        rospy.Subscriber(self.local_position_topic, PoseStamped,
                         self.local_position_callback, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(2.0), self.timer_callback)
        rospy.loginfo(
            "Monitoring MAVROS topics: status=%s, local_position=%s "
            "(distance sensor disabled)", self.vehicle_status_topic,
            self.local_position_topic)

    def vehicle_status_callback(self, message):
        self.status_received = True
        self.last_state = message

    def local_position_callback(self, message):
        self.local_position_received = True
        self.last_pose = message.pose

    def timer_callback(self, _event):
        state = self.last_state
        pose = self.last_pose
        pose_text = "unavailable"
        if pose is not None:
            values = (pose.position.x, pose.position.y, pose.position.z)
            if all(math.isfinite(float(value)) for value in values):
                pose_text = "(%+.2f, %+.2f, %+.2f)" % values
        rospy.loginfo(
            "MAVROS status: state=%s, connected=%s, armed=%s, mode=%s, "
            "local_position=%s, pose_enu=%s",
            self.status_received, getattr(state, "connected", None),
            getattr(state, "armed", None), getattr(state, "mode", None),
            self.local_position_received, pose_text)


if __name__ == "__main__":
    rospy.init_node("px4_dds_monitor")
    Px4MavrosMonitor()
    rospy.spin()
