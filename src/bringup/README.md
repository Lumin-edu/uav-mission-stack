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
roslaunch bringup hover_hw.launch \
  use_fastlio:=true \
  use_fastlio_mavros_visual_odom:=true \
  use_hover_control:=true \
  use_px4_monitor:=true \
  use_px4_control_watchdog:=true \
  use_position_compare:=false \
  target_x:=0.0 target_y:=0.0 target_z:=0.4 \
  lock_yaw:=true target_yaw:=0.0 \
  reference_capture_delay_sec:=5.0 \
  hover_auto_arm:=false
```

With `hover_auto_arm:=false`, arm manually from the RC. After the controller
has pre-streamed setpoints, it requests `OFFBOARD` through `/mavros/set_mode`.

FAST-LIO input topics are `/livox/lidar` and `/livox/imu`; the launch starts
`livox_ros_driver2` with `MID360_config.json` and `fastlio_mid360.yaml`.
