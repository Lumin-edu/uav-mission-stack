# ROS1 FAST-LIO fixed-point bringup

This package starts the MID360 driver, FAST-LIO, the FAST-LIO to MAVROS
external-vision bridge, fixed-point control, and optional diagnostics. MAVROS
is deliberately not included; start it separately first.

The bringup monitor intentionally does not subscribe to a MAVROS distance
sensor. The standard MAVROS PX4 plugin list also blacklists the
`distance_sensor` plugin; any `distance_sensor/*` parameters printed while
MAVROS loads `px4_config.yaml` are configuration entries only.

## Coordinate contract

`target_x`, `target_y`, `target_z` are task offsets in the vehicle RFU frame:

* `x`: right of the nose
* `y`: forward from the nose
* `z`: up

FAST-LIO publishes `nav_msgs/Odometry` on `/Odometry`. The bridge captures its
initial corrected pose, applies the zero default `base_link -> base` lever arm,
and publishes ROS ENU `PoseStamped` messages on `/mavros/vision_pose/pose`.
MAVROS converts that ENU pose to PX4 NED. The fixed-point controller converts
the RFU task offset using the captured MAVROS local yaw before publishing
`/mavros/setpoint_position/local`. `lock_yaw:=true` (the default) sends a
fixed setpoint yaw equal to the captured PX4 yaw; `target_yaw:=0.0` keeps it
exactly equal to capture.

## Launch

```bash
  source /opt/ros/noetic/setup.bash
  source /home/yundrone/sysu/devel/setup.bash
  roslaunch mavros px4.launch fcu_url:=/dev/ttyTHS0:921600

cd /home/yundrone/sysu
source /opt/ros/noetic/setup.bash
source devel/setup.bash

# Start MAVROS separately, using the vehicle's existing configuration.
# Then start MID360 + FAST-LIO + visual odometry + hover:
cd /home/yundrone/sysu
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch bringup hover_hw.launch \
  use_fastlio:=true \
  use_fastlio_mavros_visual_odom:=true \
  use_hover_control:=true \
  use_px4_monitor:=true \
  use_px4_control_watchdog:=true \
  use_position_compare:=false \
  target_x:=0.0 target_y:=0.0 target_z:=0.4 \
  lock_yaw:=true target_yaw:=0.0 \
  reference_capture_delay_sec:=3.0 \
  hover_auto_arm:=false
```

With `hover_auto_arm:=false`, arm manually from the RC. After the controller
has pre-streamed setpoints, it requests `OFFBOARD` through `/mavros/set_mode`.

FAST-LIO input topics are `/livox/lidar` and `/livox/imu`; the launch starts
`livox_ros_driver2` with `MID360_config.json` and `fastlio_mid360.yaml`.

## Eight-point mission

`up.launch` is an independent sequential mission entry point. It reuses the
MID360, FAST-LIO, and external-vision chain but does not start
`fixed_point_hover.py`. The eight targets are in
`config/up_targets.yaml`, in RFU coordinates relative to the captured origin.
The sample mission climbs to 0.4 m and visits an eight-point square path while
keeping the captured yaw (`lock_yaw:=true`). Do not run `hover_hw.launch` and
`up.launch` together, since both would publish local-position setpoints.

```bash
cd /home/yundrone/sysu
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch bringup up.launch \
  mission_file:=$(rospack find bringup)/config/up_targets.yaml \
  use_px4_monitor:=true \
  lock_yaw:=true \
  max_speed:=0.5
```

`max_speed` is the mission setpoint slew limit in m/s. It limits the norm of
the position setpoint motion between control cycles; PX4 velocity parameters
remain the final flight-controller limit.

Arm and start PX4 `OFFBOARD` according to the same procedure as the validated
single-point mode. The mission publishes its current zero-based waypoint index
on `/multi_point_up/waypoint_index` and latches completion on
`/multi_point_up/mission_complete`.

## AprilTag drop-and-return mission

`tag_drop_mission.launch` is a separate entry point. It does not start MAVROS
and must not be run together with `hover_hw.launch` or `up.launch`. It visits
the eight points in `config/tag_drop_targets.yaml`; point 5 is the drop point.

The TEP-C camera and servo are discovered independently. The default candidates
are `/dev/video0`, `/dev/video1`, `/dev/robotac_rgb_camera` for the camera and
`/dev/robotac_servo`, `/dev/ttyUSB0`, `/dev/ttyACM0` for the servo. Override them
with `video_device:=...` and `servo_port:=...` when the actual enumeration is
known.

At point 5 the mission clears old detections and starts a three-second search.
If a fresh Tag is detected, it applies the horizontal RFU correction only and
releases immediately once inside `tag_xy_tolerance`. If no Tag is detected in
three seconds, it releases at the point without correction. After release it
returns to the captured origin, requires a new landing Tag, aligns horizontally,
descends while the Tag remains fresh, and calls `/mavros/cmd/land`.

The camera is 0.115 m forward of the payload center. The tracker uses the
specified downward-camera convention (image top is aircraft forward) and maps
camera optical coordinates to RFU as `(right, forward) = (camera_x,
-camera_y)`, then adds the 0.115 m forward camera offset. The calibration is
copied into `config/tag_drop_camera.yaml`; replace it if the camera or
resolution changes.

Start MAVROS separately, then run:

```bash
cd /home/yundrone/sysu
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch bringup tag_drop_mission.launch \
  video_device:=auto \
  servo_port:=auto \
  drop_tag_id:=0 \
  landing_tag_id:=0 \
  max_speed:=0.5 \
  lock_yaw:=true
```

Useful checks before flight:

```bash
rostopic echo /tag_detections
rostopic echo /bringup/tag/drop_offset_rfu
rostopic echo /bringup/tag/landing_offset_rfu
rostopic echo /tag_drop_mission/mission_phase
rosservice call /bringup_servo/set_released "data: false"
```

For hardware-only testing, start `tag_drop_hardware_test.launch`. It does not
start MAVROS, FAST-LIO, or the mission controller. The service call above sends
the configured blocked position; `data: true` is the real release command and
must only be used with the mechanism secured and the payload removed.

For a servo-only test, start `servo_test.launch`. It sends no angle when the
node starts. Choose an angle explicitly through the generated service; for
example, 90 degrees sends duty 8 (`5A 01 00 32 08`) at 115200 baud:

```bash
roslaunch bringup servo_test.launch servo_port:=/dev/ttyUSB0
rosservice call /bringup_servo/set_angle "angle: 90.0"
```

The response reports the calibrated duty. The service accepts any angle from
0 to 180 degrees; with the default calibration, 0/90/180 degrees map to duty
3/8/12. The node sends the protocol idle frame after an explicit command, but
does not send an angle on startup or shutdown.

The servo transport uses `115200` baud and writes raw binary frames, not ASCII.
With the default calibration the diagnostic frames are:

```text
blocked:  5A 01 00 32 03
released: 5A 01 00 32 08
idle:     5A 01 00 32 00
```
