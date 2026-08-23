#!/usr/bin/env python3
"""Check that the ROS1 Offboard input publishers are present."""

import rospy


class Px4ControlWatchdog(object):
    def __init__(self):
        self.control_source = rospy.get_param("~control_source", "hover")
        self.setpoint_topic = rospy.get_param(
            "~setpoint_topic", "/mavros/setpoint_position/local")
        self.required_topics = [self.setpoint_topic]
        self.reported_ready = False
        rospy.Timer(rospy.Duration(2.0), self.report)
        rospy.loginfo("MAVROS control watchdog active, expected control_source=%s",
                      self.control_source)

    def report(self, _event):
        published = dict(rospy.get_published_topics())
        missing = [topic for topic in self.required_topics
                   if topic not in published]
        details = ["%s: publishers=%s" % (topic, topic in published)
                   for topic in self.required_topics]
        if missing:
            self.reported_ready = False
            rospy.logerr(
                "MAVROS will not move because no ROS node is publishing the "
                "required setpoint. Missing publishers: %s. %s",
                missing, ", ".join(details))
        elif not self.reported_ready:
            self.reported_ready = True
            rospy.loginfo("MAVROS setpoint publisher is present: %s",
                          ", ".join(details))


if __name__ == "__main__":
    rospy.init_node("px4_control_watchdog")
    Px4ControlWatchdog()
    rospy.spin()
