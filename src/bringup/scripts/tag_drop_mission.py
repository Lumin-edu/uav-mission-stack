#!/usr/bin/env python3
"""Independent eight-point mission with point-5 AprilTag release and landing."""

import importlib.util
import math
import os
import time

import rospy
from geometry_msgs.msg import Vector3Stamped
from mavros_msgs.msg import ExtendedState
from mavros_msgs.srv import CommandBool, CommandTOL
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool


def _load_up_module():
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "up.py")
    spec = importlib.util.spec_from_file_location(
        "bringup_up_for_tag_drop", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_up = _load_up_module()
_fixed = _up._fixed_point
UpMission = _up.UpMission
FixedPointOffboard = _fixed.FixedPointOffboard
_position = _fixed._position
_transform_offset = _fixed._transform_offset
_yaw_quaternion = _fixed._yaw_quaternion


class TagDropMission(UpMission):
    """Run eight points, release at point five, then land at point eight."""

    def __init__(self):
        super(TagDropMission, self).__init__()
        self.drop_waypoint = int(rospy.get_param("~drop_waypoint", 5))
        if self.drop_waypoint < 1 or self.drop_waypoint > len(self.points):
            raise ValueError("drop_waypoint is outside the mission")
        self.drop_index = self.drop_waypoint - 1
        self.landing_index = len(self.points) - 1

        self.tag_timeout = float(rospy.get_param("~tag_timeout", 0.8))
        self.tag_xy_tolerance = float(rospy.get_param(
            "~tag_xy_tolerance", 0.15))
        self.tag_max_step = float(rospy.get_param("~tag_max_step", 0.20))
        self.tag_correction_speed = float(rospy.get_param(
            "~tag_correction_speed", 0.30))
        # Release as soon as the horizontal error is inside tolerance.  This
        # timer covers both detection and alignment, with a timeout fallback.
        self.drop_search_seconds = float(rospy.get_param(
            "~drop_search_seconds", 3.0))
        self.landing_height = float(rospy.get_param(
            "~landing_height", 0.0))
        self.landing_descent_speed = float(rospy.get_param(
            "~landing_descent_speed", 0.15))
        self.landing_altitude_tolerance = float(rospy.get_param(
            "~landing_altitude_tolerance", 0.08))
        self.landing_search_seconds = float(rospy.get_param(
            "~landing_search_seconds", 3.0))
        self.release_service_name = rospy.get_param(
            "~release_service", "/bringup_servo/set_released")
        self.land_service_name = rospy.get_param(
            "~land_service", "/mavros/cmd/land")
        self.arming_service_name = rospy.get_param(
            "~arming_service", "/mavros/cmd/arming")
        self.land_mode = str(rospy.get_param(
            "~land_mode", "AUTO.LAND")).strip()
        self.disarm_height_tolerance = float(rospy.get_param(
            "~disarm_height_tolerance", 0.05))
        self.disarm_retry_period = float(rospy.get_param(
            "~disarm_retry_period", 1.0))
        self.service_timeout = float(rospy.get_param(
            "~service_timeout", 2.0))
        if self.tag_timeout <= 0.0 or self.tag_xy_tolerance <= 0.0:
            raise ValueError("invalid AprilTag mission timeout/tolerance")
        if (self.tag_max_step <= 0.0 or self.drop_search_seconds <= 0.0 or
                self.tag_correction_speed <= 0.0 or
                self.landing_search_seconds <= 0.0):
            raise ValueError("invalid AprilTag correction parameters")
        if self.landing_height < 0.0 or self.landing_descent_speed <= 0.0:
            raise ValueError("invalid landing parameters")
        if (self.disarm_height_tolerance <= 0.0 or
                self.disarm_retry_period <= 0.0 or not self.land_mode):
            raise ValueError("invalid landing shutdown parameters")

        self.phase = "NAVIGATE"
        self.drop_offset = None
        self.landing_offset = None
        self.drop_receive_time = None
        self.landing_receive_time = None
        self.drop_altitude = None
        self.return_altitude = None
        self.landing_target_z = None
        self.landing_descent_z = None
        self.descent_last_time = None
        self.drop_search_started = None
        self.landing_search_started = None
        self.tag_hold_started = None
        self.release_done = False
        self.land_requested = False
        self.landing_reference_z = None
        self.last_land_mode_request = 0.0
        self.last_disarm_request = 0.0
        self.extended_state = ExtendedState()
        self.last_extended_state_receive = None

        self.phase_pub = rospy.Publisher("~mission_phase", String,
                                         queue_size=1, latch=True)
        self.release_pub = rospy.Publisher("~release_complete", Bool,
                                           queue_size=1, latch=True)
        self.landing_pub = rospy.Publisher("~landing_complete", Bool,
                                           queue_size=1, latch=True)
        rospy.Subscriber("/bringup/tag/drop_offset_rfu", Vector3Stamped,
                         self._drop_tag_cb, queue_size=5)
        rospy.Subscriber("/bringup/tag/landing_offset_rfu", Vector3Stamped,
                         self._landing_tag_cb, queue_size=5)
        rospy.Subscriber("/mavros/extended_state", ExtendedState,
                         self._extended_state_cb, queue_size=10)
        self.release_proxy = rospy.ServiceProxy(self.release_service_name,
                                                SetBool)
        self.land_proxy = rospy.ServiceProxy(self.land_service_name,
                                             CommandTOL)
        self.arming_proxy = rospy.ServiceProxy(self.arming_service_name,
                                               CommandBool)
        self.release_pub.publish(Bool(data=False))
        self.landing_pub.publish(Bool(data=False))
        self._set_phase("NAVIGATE")
        rospy.loginfo("Tag-drop mission enabled: drop waypoint=%d, landing "
                      "waypoint=%d, XY-only correction speed=%.2f m/s",
                      self.drop_waypoint, self.landing_index + 1,
                      self.tag_correction_speed)

    def _set_phase(self, phase):
        if self.phase != phase:
            rospy.loginfo("tag-drop mission phase: %s -> %s",
                          self.phase, phase)
        self.phase = phase
        self.phase_pub.publish(String(data=phase))

    @staticmethod
    def _valid_offset(message):
        return all(math.isfinite(float(value)) for value in (
            message.vector.x, message.vector.y))

    def _drop_tag_cb(self, message):
        if self._valid_offset(message):
            self.drop_offset = [float(message.vector.x),
                                float(message.vector.y)]
            self.drop_receive_time = time.monotonic()

    def _landing_tag_cb(self, message):
        if self._valid_offset(message):
            self.landing_offset = [float(message.vector.x),
                                   float(message.vector.y)]
            self.landing_receive_time = time.monotonic()

    def _extended_state_cb(self, message):
        self.extended_state = message
        self.last_extended_state_receive = time.monotonic()

    def _fresh(self, receive_time):
        return (receive_time is not None and
                time.monotonic() - receive_time <= self.tag_timeout)

    def _tag_error(self, offset):
        return math.hypot(offset[0], offset[1])

    def _hold_current(self, altitude=None):
        if self.last_local_pose is None or not self.origin_captured:
            return
        current = _position(self.last_local_pose.pose)
        self.target_position = [current[0], current[1],
                                current[2] if altitude is None else altitude]
        self.target_q = _yaw_quaternion(self.locked_yaw)

    def _set_tag_target(self, offset, altitude):
        if self.last_local_pose is None or not self.origin_captured:
            return False
        length = self._tag_error(offset)
        if length > self.tag_max_step:
            scale = self.tag_max_step / length
            offset = [offset[0] * scale, offset[1] * scale]
        # offset is already vehicle RFU -> tag RFU.  The z component is
        # intentionally zero: AprilTag controls horizontal position only.
        enu_offset = _transform_offset(
            self.fastlio_origin_q, self.px4_origin_q,
            [offset[0], offset[1], 0.0])
        current = _position(self.last_local_pose.pose)
        self.target_position = [current[0] + enu_offset[0],
                                current[1] + enu_offset[1], altitude]
        self.target_q = _yaw_quaternion(self.locked_yaw)
        return True

    def _begin_drop(self):
        self.drop_altitude = self.target_position[2]
        # Start the timer at waypoint 5. Detections collected during transit
        # must not satisfy the drop gate.
        self.drop_offset = None
        self.drop_receive_time = None
        self.drop_search_started = time.monotonic()
        self.tag_hold_started = None
        self._set_phase("WAIT_DROP_TAG")

    def _begin_return(self):
        """Continue from point 5 to points 6, 7, and 8 after release."""
        self.mission_index = self.drop_index + 1
        self.target_reached = False
        self.reached_since = None
        self._set_mission_target(reset_command=True)
        self.reached_pub.publish(Bool(data=False))
        self._set_phase("NAVIGATE")

    def _call_release(self):
        if self.release_done:
            return True
        try:
            rospy.wait_for_service(self.release_service_name,
                                   timeout=self.service_timeout)
            response = self.release_proxy(True)
        except (rospy.ROSException, rospy.ServiceException) as error:
            rospy.logerr("release service failed: %s", error)
            return False
        if not response.success:
            rospy.logerr("release rejected: %s", response.message)
            return False
        self.release_done = True
        self.release_pub.publish(Bool(data=True))
        rospy.loginfo("payload release completed")
        return True

    def _begin_landing_wait(self):
        """Start final-point landing search without moving to the origin."""
        self._set_mission_target(reset_command=False)
        self.return_altitude = self.target_position[2]
        self.landing_reference_z = self.target_position[2]
        self.landing_offset = None
        self.landing_receive_time = None
        self.landing_search_started = time.monotonic()
        self.tag_hold_started = None
        self._set_phase("WAIT_LANDING_TAG")

    def _begin_landing_descend(self):
        self.landing_target_z = self.return_altitude + self.landing_height
        self.landing_descent_z = self.return_altitude
        self.descent_last_time = time.monotonic()
        self.tag_hold_started = None
        self._set_phase("LANDING_DESCEND")

    def _call_land(self):
        if self.land_requested:
            return True
        try:
            rospy.wait_for_service(self.land_service_name,
                                   timeout=self.service_timeout)
            response = self.land_proxy(min_pitch=0.0,
                                       yaw=float(self.locked_yaw),
                                       latitude=0.0, longitude=0.0,
                                       altitude=0.0)
        except (rospy.ROSException, rospy.ServiceException) as error:
            rospy.logerr("MAVROS land service failed: %s", error)
            return False
        if not response.success:
            rospy.logerr("MAVROS rejected landing request, result=%d",
                          response.result)
            return False
        self.land_requested = True
        self._set_phase("LAND_REQUESTED")
        rospy.loginfo("landing request accepted; final PX4 z=%.3f, "
                      "disarm tolerance=%.3f m",
                      self.landing_reference_z,
                      self.disarm_height_tolerance)
        return True

    def _request_land_mode(self):
        if self.state.mode != "OFFBOARD":
            return True
        now = time.monotonic()
        if now - self.last_land_mode_request < self.mode_retry_period:
            return False
        self.last_land_mode_request = now
        try:
            response = self.mode_service(base_mode=0,
                                         custom_mode=self.land_mode)
        except rospy.ServiceException as error:
            rospy.logwarn_throttle(2.0, "AUTO.LAND mode request failed: %s",
                                   error)
            return False
        if not response.mode_sent:
            rospy.logwarn("PX4 rejected %s mode request", self.land_mode)
            return False
        rospy.loginfo("PX4 accepted %s mode; Offboard setpoints will stop "
                      "after mode feedback confirms the transition",
                      self.land_mode)
        return True

    def _finish_landing(self):
        if self.phase != "LAND_REQUESTED":
            return
        if not self.state.armed:
            self.landing_pub.publish(Bool(data=True))
            self.complete_pub.publish(Bool(data=True))
            self._set_phase("COMPLETE")
            rospy.loginfo("PX4 disarmed; landing complete and motors stopped")
            return

        self._request_land_mode()
        if self.state.mode == "OFFBOARD":
            return
        if (self.last_local_pose is None or
                not self._fresh(self.last_local_receive) or
                self.landing_reference_z is None):
            return

        current_z = float(self.last_local_pose.pose.position.z)
        height_error = abs(current_z - self.landing_reference_z)
        if height_error > self.disarm_height_tolerance:
            rospy.loginfo_throttle(
                1.0, "waiting to disarm: PX4 z=%.3f target=%.3f error=%.3f m",
                current_z, self.landing_reference_z, height_error)
            return

        if (self._fresh(self.last_extended_state_receive) and
                self.extended_state.landed_state !=
                ExtendedState.LANDED_STATE_ON_GROUND):
            rospy.loginfo_throttle(
                1.0, "z error %.3f m but PX4 has not reported ON_GROUND",
                height_error)
            return

        now = time.monotonic()
        if now - self.last_disarm_request < self.disarm_retry_period:
            return
        self.last_disarm_request = now
        try:
            response = self.arming_proxy(value=False)
        except rospy.ServiceException as error:
            rospy.logwarn_throttle(2.0, "PX4 disarm request failed: %s", error)
            return
        if response.success:
            rospy.loginfo("PX4 disarm accepted at z error %.3f m", height_error)
        else:
            rospy.logwarn("PX4 rejected disarm at z error %.3f m",
                          height_error)

    def _abort(self, reason):
        rospy.logerr("tag-drop mission aborted: %s", reason)
        self._hold_current()
        self._set_phase("ABORT")

    def _phase_tick(self):
        if self.phase == "WAIT_DROP_TAG":
            if self._fresh(self.drop_receive_time):
                self.tag_hold_started = None
                self._set_phase("DROP_ALIGN")
            elif (self.drop_search_started is not None and
                  time.monotonic() - self.drop_search_started >=
                  self.drop_search_seconds):
                rospy.logwarn("no drop AprilTag detected within %.1f s; "
                              "releasing at waypoint 5", self.drop_search_seconds)
                if self._call_release():
                    self._begin_return()
                else:
                    self._abort("payload release failed")
            return

        if self.phase == "DROP_ALIGN":
            if (self.drop_search_started is not None and
                    time.monotonic() - self.drop_search_started >=
                    self.drop_search_seconds):
                rospy.logwarn("drop AprilTag not aligned within %.1f s; "
                              "releasing by timeout",
                              self.drop_search_seconds)
                if self._call_release():
                    self._begin_return()
                else:
                    self._abort("payload release failed")
                return
            if not self._fresh(self.drop_receive_time):
                self._hold_current(self.drop_altitude)
                return
            self._set_tag_target(self.drop_offset, self.drop_altitude)
            if self._tag_error(self.drop_offset) <= self.tag_xy_tolerance:
                rospy.loginfo("drop AprilTag aligned; releasing immediately")
                if self._call_release():
                    self._begin_return()
                else:
                    self._abort("payload release failed")
            else:
                self.tag_hold_started = None
            return

        if self.phase == "WAIT_LANDING_TAG":
            if self._fresh(self.landing_receive_time):
                self.tag_hold_started = None
                self._set_phase("LANDING_ALIGN")
            elif (self.landing_search_started is not None and
                  time.monotonic() - self.landing_search_started >=
                  self.landing_search_seconds):
                rospy.logwarn("no landing AprilTag aligned within %.1f s; "
                              "landing at final waypoint anyway",
                              self.landing_search_seconds)
                self._call_land_or_abort()
            return

        if self.phase == "LANDING_ALIGN":
            if (self.landing_search_started is not None and
                    time.monotonic() - self.landing_search_started >=
                    self.landing_search_seconds):
                rospy.logwarn("landing AprilTag not aligned within %.1f s; "
                              "landing at final waypoint anyway",
                              self.landing_search_seconds)
                self._call_land_or_abort()
                return
            if not self._fresh(self.landing_receive_time):
                self._hold_current(self.return_altitude)
                self.tag_hold_started = None
                return
            self._set_tag_target(self.landing_offset, self.return_altitude)
            if self._tag_error(self.landing_offset) <= self.tag_xy_tolerance:
                rospy.loginfo("landing AprilTag aligned; landing immediately")
                self._call_land_or_abort()
            else:
                self.tag_hold_started = None
            return

        if self.phase == "LANDING_DESCEND":
            # Compatibility state for old parameters.  The final mission
            # point is already the commanded landing height, so land directly
            # after alignment or the timeout fallback.
            self._call_land_or_abort()

    def _call_land_or_abort(self):
        if self._call_land():
            return True
        self._abort("landing request failed")
        return False

    def _advance_navigation(self):
        if self.mission_index == self.drop_index:
            self._begin_drop()
            return
        if self.mission_index >= self.landing_index:
            self._begin_landing_wait()
            return
        self.mission_index += 1
        self.target_reached = False
        self.reached_since = None
        self._set_mission_target(reset_command=True)
        self.reached_pub.publish(Bool(data=False))

    def _update_reached(self):
        if self.phase == "NAVIGATE":
            FixedPointOffboard._update_reached(self)
            if self.target_reached:
                self._advance_navigation()
            return
        if self.phase == "LAND_REQUESTED":
            self._finish_landing()

    def _request_offboard(self):
        if self.phase in ("LAND_REQUESTED", "COMPLETE", "ABORT"):
            return
        super(TagDropMission, self)._request_offboard()

    def _heartbeat_target(self):
        """Use 0.30 m/s while applying AprilTag horizontal corrections."""
        normal_speed = self.max_speed
        if self.phase in ("DROP_ALIGN", "LANDING_ALIGN", "LANDING_DESCEND"):
            self.max_speed = self.tag_correction_speed
        try:
            return FixedPointOffboard._heartbeat_target(self)
        finally:
            self.max_speed = normal_speed

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if time.monotonic() - self.started_at >= self.origin_delay:
                self._capture_origin()
            self._phase_tick()
            # Keep the setpoint stream only until PX4 confirms it has left
            # OFFBOARD for AUTO.LAND.  No Offboard commands are published
            # during landing/disarmed completion.
            publish_setpoint = (self.phase not in ("COMPLETE", "ABORT") and
                                (self.phase != "LAND_REQUESTED" or
                                 self.state.mode == "OFFBOARD"))
            if publish_setpoint:
                position, quaternion = self._heartbeat_target()
                self.setpoint_pub.publish(self._make_pose(position, quaternion))
            self._update_reached()
            self._request_offboard()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("tag_drop_mission")
    try:
        TagDropMission().run()
    except ValueError as error:
        rospy.logfatal("Tag-drop mission parameter error: %s", error)
