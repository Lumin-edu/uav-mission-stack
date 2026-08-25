#!/usr/bin/env python3
"""Independent one-shot payload servo service for the sysu mission."""

import fcntl
import math
import os
import threading
import time

import rospy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, SetBoolResponse
from bringup.srv import SetServoAngle, SetServoAngleResponse

try:
    import serial
except ImportError:
    serial = None


def duty_for_angle(angle, pwm_min, pwm_max):
    if angle < 0.0 or angle > 180.0:
        raise ValueError("angle outside 0..180 degrees")
    if pwm_min < 0 or pwm_max <= pwm_min or pwm_max > 100:
        raise ValueError("invalid PWM range")
    return int((pwm_min + (pwm_max - pwm_min) * angle / 180.0) + 0.5)


def make_frame(channel, frequency_hz, duty):
    if not 0 <= channel <= 255 or not 1 <= frequency_hz <= 65535:
        raise ValueError("invalid servo protocol value")
    if not 0 <= duty <= 100:
        raise ValueError("duty outside 0..100")
    return bytes((0x5A, channel, frequency_hz >> 8,
                  frequency_hz & 0xFF, duty))


def frame_hex(frame):
    """Return a stable uppercase representation for protocol diagnostics."""
    return " ".join("%02X" % value for value in bytearray(frame))


class LockedSerial(object):
    def __init__(self, path, baudrate):
        self.path = path
        self.serial_port = None
        self.fd = None
        if serial is not None:
            port = serial.Serial(port=path, baudrate=baudrate, timeout=0.0,
                                 write_timeout=1.0, rtscts=False,
                                 dsrdtr=False, xonxoff=False)
            port.dtr = False
            port.rts = False
            try:
                fcntl.flock(port.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            except (OSError, IOError):
                port.close()
                raise RuntimeError("servo serial port is already locked")
            self.serial_port = port
            return
        flags = os.O_RDWR | os.O_NOCTTY
        if hasattr(os, "O_CLOEXEC"):
            flags |= os.O_CLOEXEC
        self.fd = os.open(path, flags)
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except Exception:
            os.close(self.fd)
            self.fd = None
            raise

    def write(self, data):
        if self.serial_port is not None:
            count = self.serial_port.write(data)
            self.serial_port.flush()
            if count != len(data):
                raise OSError("short servo serial write")
            return
        if self.fd is None:
            raise OSError("servo serial port is closed")
        offset = 0
        while offset < len(data):
            count = os.write(self.fd, data[offset:])
            if count <= 0:
                raise OSError("servo serial write failed")
            offset += count

    def close(self):
        if self.serial_port is not None:
            self.serial_port.close()
            self.serial_port = None
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None


class BringupServo(object):
    def __init__(self):
        self.port_path = str(rospy.get_param(
            "~port", "auto")).strip()
        self.port_candidates = rospy.get_param(
            "~port_candidates",
            ["/dev/robotac_servo", "/dev/ttyUSB0", "/dev/ttyUSB1",
             "/dev/ttyACM0", "/dev/ttyACM1"])
        self.baudrate = int(rospy.get_param("~baudrate", 115200))
        self.channel = int(rospy.get_param("~channel", 1))
        self.frequency_hz = int(rospy.get_param("~frequency_hz", 50))
        self.pwm_min = int(rospy.get_param("~pwm_min_duty", 3))
        self.pwm_max = int(rospy.get_param("~pwm_max_duty", 12))
        self.blocked_angle = float(rospy.get_param("~blocked_angle", 0.0))
        self.released_angle = float(rospy.get_param("~released_angle", 90.0))
        self.min_angle = float(rospy.get_param("~min_angle", 0.0))
        self.max_angle = float(rospy.get_param("~max_angle", 180.0))
        self.drive_seconds = float(rospy.get_param("~drive_seconds", 0.30))
        self.idle_duty = int(rospy.get_param("~idle_duty", 0))
        self.allow_repeat = bool(rospy.get_param("~allow_repeat", False))
        if (self.min_angle < 0.0 or self.max_angle > 180.0 or
                self.min_angle >= self.max_angle):
            raise ValueError("invalid servo angle limits")
        if not self.min_angle <= self.blocked_angle <= self.max_angle:
            raise ValueError("blocked_angle outside servo angle limits")
        if not self.min_angle <= self.released_angle <= self.max_angle:
            raise ValueError("released_angle outside servo angle limits")
        self.blocked_duty = duty_for_angle(self.blocked_angle,
                                           self.pwm_min, self.pwm_max)
        self.released_duty = duty_for_angle(self.released_angle,
                                            self.pwm_min, self.pwm_max)
        if self.blocked_duty == self.released_duty:
            raise ValueError("blocked and released commands are identical")

        self.lock = threading.Lock()
        self.port = None
        self.last_state = None
        self.last_angle = None
        self.connected_pub = rospy.Publisher("~connected", Bool,
                                             queue_size=1, latch=True)
        self.state_pub = rospy.Publisher("~state", String,
                                         queue_size=1, latch=True)
        self.service = rospy.Service("~set_released", SetBool,
                                     self._set_released)
        self.angle_service = rospy.Service("~set_angle", SetServoAngle,
                                           self._set_angle)
        rospy.on_shutdown(self._shutdown)
        self._publish(False, "IDLE")
        rospy.loginfo("Independent servo service ready: %s", self.port_path)

    def _publish(self, connected, state):
        self.connected_pub.publish(Bool(data=bool(connected)))
        self.state_pub.publish(String(data=state))

    def _connect(self):
        if self.port is not None:
            return True
        path = self._resolve_port()
        if path is None:
            rospy.logwarn_throttle(2.0, "no TEP-C servo serial device is available")
            return False
        try:
            self.port = LockedSerial(path, self.baudrate)
            return True
        except (OSError, IOError, RuntimeError) as error:
            rospy.logwarn_throttle(2.0, "servo port unavailable: %s", error)
            self.port = None
            return False

    def _resolve_port(self):
        if self.port_path and self.port_path != "auto":
            return self.port_path
        for candidate in self.port_candidates:
            candidate = str(candidate).strip()
            if candidate and os.path.exists(candidate):
                return candidate
        return None

    def _send_angle(self, angle, state):
        """Send one calibrated angle and then the protocol idle frame.

        The node deliberately does not call this method during startup.  A
        hardware command is emitted only after a service request (or the
        mission's release request).
        """
        try:
            angle = float(angle)
        except (TypeError, ValueError):
            return False, "angle is not a number", None
        if not math.isfinite(angle):
            return False, "angle must be finite", None
        if not self.min_angle <= angle <= self.max_angle:
            return False, "angle outside configured limits", None

        duty = duty_for_angle(angle, self.pwm_min, self.pwm_max)
        if not self._connect():
            self._publish(False, "ABORT")
            return False, "servo serial port unavailable", duty

        try:
            action_frame = make_frame(self.channel, self.frequency_hz, duty)
            idle_frame = make_frame(self.channel, self.frequency_hz,
                                     self.idle_duty)
            rospy.loginfo("servo HEX TX (%s, %.1f deg): %s; idle=%s",
                          state, angle, frame_hex(action_frame),
                          frame_hex(idle_frame))
            self.port.write(action_frame)
            rospy.sleep(self.drive_seconds)
            self.port.write(idle_frame)
        except (OSError, IOError, ValueError) as error:
            rospy.logerr("servo angle command failed: %s", error)
            self.port.close()
            self.port = None
            self._publish(False, "ABORT")
            return False, "servo angle command failed", duty

        self.last_angle = angle
        self.last_state = state
        self._publish(True, state)
        return True, "ANGLE_SENT", duty

    def _set_angle(self, request):
        """Set an arbitrary angle through ``/bringup_servo/set_angle``."""
        with self.lock:
            try:
                requested_angle = float(request.angle)
            except (TypeError, ValueError):
                return SetServoAngleResponse(False, "angle is not a number",
                                             0.0, 0)
            success, message, duty = self._send_angle(
                requested_angle, "ANGLE_%.1f" % requested_angle)
            if duty is None:
                duty = 0
            return SetServoAngleResponse(success, message,
                                         requested_angle, int(duty))

    def _set_released(self, request):
        requested = "RELEASED" if request.data else "BLOCKED"
        with self.lock:
            if self.last_state == requested and not self.allow_repeat:
                return SetBoolResponse(False, "repeated servo action refused")
            success, message, unused_duty = self._send_angle(
                self.released_angle if request.data else self.blocked_angle,
                requested)
            del unused_duty
            return SetBoolResponse(success, requested if success else message)

    def _shutdown(self):
        with self.lock:
            if self.port is not None:
                # No angle is sent on startup or shutdown.  The idle frame is
                # already emitted after each explicit command.
                self.port.close()
                self.port = None


if __name__ == "__main__":
    rospy.init_node("bringup_servo")
    try:
        BringupServo()
        rospy.spin()
    except ValueError as error:
        rospy.logfatal("servo configuration error: %s", error)
