#!/usr/bin/env python3
"""Eight-point RFU mission controller for the ROS1 FAST-LIO bringup.

The base controller owns the FAST-LIO/PX4 reference capture, RFU-to-ENU
conversion, setpoint streaming, OFFBOARD request, and yaw lock.  This node
adds only sequential waypoint handling so the validated single-point path is
left unchanged.
"""

import importlib.util
import math
import os

import rospy
from std_msgs.msg import Bool, Int32


def _load_fixed_point_module():
    source_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "fixed_point_offboard.py")
    spec = importlib.util.spec_from_file_location(
        "bringup_fixed_point_offboard_for_up", source_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_fixed_point = _load_fixed_point_module()
FixedPointOffboard = _fixed_point.FixedPointOffboard
_transform_offset = _fixed_point._transform_offset
_yaw_quaternion = _fixed_point._yaw_quaternion
_position = _fixed_point._position


class UpMission(FixedPointOffboard):
    """Run exactly eight target points in YAML order."""

    def __init__(self):
        # The parent reads target_* for its initial validation.  The first
        # mission point is installed immediately after it initializes.
        super(UpMission, self).__init__()
        self.points = self._load_points(rospy.get_param("~points", []))
        if len(self.points) != 8:
            raise ValueError("up mission must contain exactly 8 points")

        for point in self.points:
            if math.sqrt(sum(value * value for value in point[0])) > self.max_distance:
                raise ValueError("up mission point exceeds max_distance")

        self.mission_index = 0
        self.mission_complete = False
        self._last_reached_event = False
        self.index_pub = rospy.Publisher("~waypoint_index", Int32,
                                          queue_size=1, latch=True)
        self.complete_pub = rospy.Publisher("~mission_complete", Bool,
                                             queue_size=1, latch=True)
        self.index_pub.publish(Int32(data=0))
        self.complete_pub.publish(Bool(data=False))
        rospy.loginfo("Eight-point UP mission loaded; points are RFU "
                      "(right, forward, up), count=%d", len(self.points))

    @staticmethod
    def _load_points(raw_points):
        if not isinstance(raw_points, list):
            raise ValueError("~points must be a YAML list")

        points = []
        for index, raw in enumerate(raw_points):
            if not isinstance(raw, dict):
                raise ValueError("mission point %d must be a map" % index)

            def get_value(primary, alias, default=None):
                if primary in raw:
                    return raw[primary]
                if alias in raw:
                    return raw[alias]
                if default is not None:
                    return default
                raise ValueError("mission point %d missing %s" %
                                 (index, primary))

            try:
                position = [float(get_value("x", "target_x")),
                            float(get_value("y", "target_y")),
                            float(get_value("z", "target_z"))]
                yaw = float(get_value("yaw", "target_yaw", 0.0))
            except (TypeError, ValueError):
                raise ValueError("mission point %d contains a non-numeric value" %
                                 index)

            values = position + [yaw]
            if not all(math.isfinite(value) for value in values):
                raise ValueError("mission point %d contains a non-finite value" %
                                 index)
            points.append((position, yaw))
        return points

    def _set_mission_target(self, reset_command=True):
        position_rfu, yaw_offset = self.points[self.mission_index]
        self.task_point = list(position_rfu)
        # Keep the parent's unlocked-yaw path consistent with this waypoint;
        # with lock_yaw=true this is added to the captured heading instead.
        self.target_yaw = yaw_offset
        task_offset = _transform_offset(self.fastlio_origin_q,
                                        self.px4_origin_q,
                                        self.task_point)
        self.target_position = [self.px4_origin[index] + task_offset[index]
                                for index in range(3)]
        self.target_q = _yaw_quaternion(self.locked_yaw + yaw_offset)
        if reset_command:
            if self.last_local_pose is not None:
                self.command_position = _position(self.last_local_pose.pose)
            else:
                self.command_position = list(self.px4_origin)
        self.target_pub.publish(self._make_pose(self.target_position,
                                                self.target_q))
        self.index_pub.publish(Int32(data=self.mission_index))
        rospy.loginfo("UP waypoint %d/8; RFU=(%.3f, %.3f, %.3f), "
                      "yaw_offset=%.3f", self.mission_index + 1,
                      position_rfu[0], position_rfu[1], position_rfu[2],
                      yaw_offset)

    def _capture_origin(self):
        if not self.origin_captured:
            self.task_point = list(self.points[0][0]) if self.points else [0.0, 0.0, 0.0]
        captured = super(UpMission, self)._capture_origin()
        if captured and self.mission_index == 0:
            # The parent computed the first target with the same transform;
            # republish it with the per-point yaw and mission status.
            self._set_mission_target(reset_command=False)
        return captured

    def _update_reached(self):
        super(UpMission, self)._update_reached()
        if not self.target_reached or self._last_reached_event:
            self._last_reached_event = self.target_reached
            return

        if self.mission_index >= len(self.points) - 1:
            self.mission_complete = True
            self.complete_pub.publish(Bool(data=True))
            self._publish_state("MISSION_COMPLETE")
            rospy.loginfo("UP mission complete at waypoint 8/8")
        else:
            self.mission_index += 1
            self.target_reached = False
            self.reached_since = None
            self._set_mission_target(reset_command=True)
            self.reached_pub.publish(Bool(data=False))
            self._publish_state("OFFBOARD")
        self._last_reached_event = True

    def _request_offboard(self):
        super(UpMission, self)._request_offboard()
        if self.mission_complete and self.state.mode == "OFFBOARD":
            self._publish_state("MISSION_COMPLETE")


if __name__ == "__main__":
    rospy.init_node("multi_point_up")
    try:
        UpMission().run()
    except ValueError as error:
        rospy.logfatal("UP mission parameter error: %s", error)
