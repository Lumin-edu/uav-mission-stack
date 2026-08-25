#!/usr/bin/env python3
"""Small self-contained V4L2 camera publisher for the sysu bringup."""

import time
import os

import cv2
import rospy
import yaml
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image


class CameraNode(object):
    def __init__(self):
        self.device_setting = str(rospy.get_param(
            "~video_device", "auto")).strip()
        self.device_candidates = rospy.get_param(
            "~video_device_candidates",
            ["/dev/video0", "/dev/video1", "/dev/video2", "/dev/video3",
             "/dev/robotac_rgb_camera"])
        self.width = int(rospy.get_param("~width", 1920))
        self.height = int(rospy.get_param("~height", 1080))
        self.fps = float(rospy.get_param("~framerate", 30.0))
        self.frame_id = rospy.get_param(
            "~camera_frame_id", "camera_rgb_optical_frame")
        self.camera_info_file = rospy.get_param("~camera_info_file", "")
        self.image_pub = rospy.Publisher("/camera/rgb/image_raw", Image,
                                         queue_size=2)
        self.info_pub = rospy.Publisher("/camera/rgb/camera_info", CameraInfo,
                                        queue_size=2)
        self.bridge = CvBridge()
        self.info = self._load_info()
        self.capture = None
        self.last_open_attempt = 0.0
        rospy.Timer(rospy.Duration(max(0.01, 1.0 / self.fps)),
                    self._timer)
        rospy.loginfo("sysu camera publisher configured: device=%s %dx%d %.1fHz",
                      self.device_setting, self.width, self.height, self.fps)

    def _load_info(self):
        info = CameraInfo()
        info.width = self.width
        info.height = self.height
        info.distortion_model = "plumb_bob"
        info.K = [0.0] * 9
        info.D = [0.0] * 5
        info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.P = [0.0] * 12
        if not self.camera_info_file:
            return info
        with open(self.camera_info_file, "r") as stream:
            data = yaml.safe_load(stream) or {}
        info.width = int(data.get("image_width", self.width))
        info.height = int(data.get("image_height", self.height))
        info.distortion_model = str(data.get("distortion_model", "plumb_bob"))
        info.K = [float(value) for value in
                  data.get("camera_matrix", {}).get("data", info.K)]
        info.D = [float(value) for value in
                  data.get("distortion_coefficients", {}).get("data", info.D)]
        info.R = [float(value) for value in
                  data.get("rectification_matrix", {}).get("data", info.R)]
        info.P = [float(value) for value in
                  data.get("projection_matrix", {}).get("data", info.P)]
        if len(info.K) != 9 or len(info.D) != 5 or len(info.R) != 9 or len(info.P) != 12:
            raise ValueError("camera calibration array length is invalid")
        return info

    def _open(self):
        now = time.monotonic()
        if now - self.last_open_attempt < 1.0:
            return False
        self.last_open_attempt = now
        device = self._resolve_device()
        if device is None:
            rospy.logwarn_throttle(2.0, "no TEP-C camera device is available")
            return False
        capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not capture.isOpened():
            capture.release()
            rospy.logwarn_throttle(2.0, "cannot open camera %s", device)
            return False
        # TEP-C USB cameras commonly expose 1920x1080 through MJPEG only.
        capture.set(cv2.CAP_PROP_FOURCC,
                    cv2.VideoWriter_fourcc(*"MJPG"))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        capture.set(cv2.CAP_PROP_FPS, self.fps)
        self.capture = capture
        rospy.loginfo("camera opened: %s", device)
        return True

    def _resolve_device(self):
        if self.device_setting and self.device_setting != "auto":
            return self.device_setting
        for candidate in self.device_candidates:
            candidate = str(candidate).strip()
            if candidate and os.path.exists(candidate):
                return candidate
        return None

    def _timer(self, unused_event):
        del unused_event
        if self.capture is None and not self._open():
            return
        ok, frame = self.capture.read()
        if not ok or frame is None:
            rospy.logwarn_throttle(2.0, "camera frame read failed")
            self.capture.release()
            self.capture = None
            return
        stamp = rospy.Time.now()
        image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        image.header.stamp = stamp
        image.header.frame_id = self.frame_id
        info = CameraInfo()
        info.header = image.header
        info.width = self.info.width
        info.height = self.info.height
        info.distortion_model = self.info.distortion_model
        info.D = list(self.info.D)
        info.K = list(self.info.K)
        info.R = list(self.info.R)
        info.P = list(self.info.P)
        self.image_pub.publish(image)
        self.info_pub.publish(info)


if __name__ == "__main__":
    rospy.init_node("bringup_camera")
    try:
        CameraNode()
        rospy.spin()
    except (IOError, OSError, ValueError) as error:
        rospy.logfatal("camera configuration error: %s", error)
