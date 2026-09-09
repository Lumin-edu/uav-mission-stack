# bringup_ego

Standalone hardware bringup for MID-360, Point-LIO, EGO Planner, and PX4
Offboard control.

```text
/livox/lidar + /livox/imu
  -> Point-LIO /odom + /cloud_registered
  -> odom -> base_link (MID360 IMU)
  -> base_link -> base [-0.011, -0.02329, -0.05588]
  -> /ego/odom_base (aircraft center) + raw /cloud_registered
  -> EGO /ego/position_cmd (100 Hz B-spline samples)
  -> PX4 TrajectorySetpoint
```

The package starts no Gazebo nodes, synthetic sensors, scenario maps, fake
odometry, or `/sim/*` topics.

Point-LIO keeps its original output contract: `/odom` is `odom -> base_link`,
and `base_link` numerically represents the MID360 IMU origin. Point-LIO source
is not changed. Both the verified PX4 visual-odometry bridge and the EGO
planning adapter independently apply the same rigid transform:

```text
T_odom_base = T_odom_base_link * T_base_link_base
t_base_link_base = [-0.011, -0.02329, -0.05588] m
q_base_link_base_xyzw = [0, 0, 0, 1]
```

The static `base_link -> base` publisher only completes the TF tree; neither
numeric adapter reads that TF, so the lever arm is applied exactly once in each
independent output path. The planning chain consumes `/ego/odom_base`, which is
the Point-LIO pose transformed from the MID-360 IMU origin to the aircraft
center. The obstacle map consumes the original `/cloud_registered` topic, and
its coordinates are not shifted or vertically re-aligned. Point-LIO z is the
only planner height source.

The PX4 visual-odometry bridge consumes raw `/odom` and applies the same
base-link-to-base transform internally before the validated ROS-to-NED
conversion. This keeps the numeric bridge independent from the `/ego/odom_base`
topic and prevents double application of the lever arm.

## Control contract

The default `control_mode:=position` sends every EGO trajectory sample as:

```text
position + velocity feed-forward + acceleration feed-forward + yaw
```

Only `OffboardControlMode.position` is true. This selects PX4's position
controller; velocity and acceleration remain valid feed-forward fields in the
same `TrajectorySetpoint`. The optional `control_mode:=velocity` exists only for
controlled comparison tests and is not the hardware default.

The bridge publishes the Offboard heartbeat continuously before and after
arming. It follows the validated square-mission order: prestream for two
seconds, wait for manual arm, then request Offboard. It first climbs to the
configured `takeoff_altitude`; only after the position and velocity are stable
does it publish the startup goal and allow EGO trajectory commands to take
control. If an EGO command is temporarily unavailable, the bridge continuously
holds the last safe PX4 position.

Optional `auto_land_after_goal:=true` enables `auto_land_after_goal.py`. It
uses the same `/move_base_simple/goal` and the compensated `/ego/odom_base` pose,
waits for position and velocity to remain within tolerance, then sends
`VEHICLE_CMD_NAV_LAND`:

```text
goal stable -> /ego/landing_requested -> PX4 NAV_LAND
PX4 landed=true -> /ego/landing_complete -> stop Offboard heartbeat/setpoints
```

`VEHICLE_CMD_NAV_LAND` 在当前 PX4 版本中表示“当前位置降落”。因此命令发送
时飞机已经稳定在 EGO 目标点，PX4 会把该时刻的当前位置作为降落点，保持目标
点的 XY 下降；它不会执行 `NAV_RETURN_TO_LAUNCH`，也不会自动回到起飞点。

During descent the bridge keeps its heartbeat and blocks Offboard re-entry. It
stops publishing `/fmu/in/offboard_control_mode` and
`/fmu/in/trajectory_setpoint` only after PX4 reports
`/fmu/out/vehicle_land_detected.landed=true`.

Startup task parameters use the validated task frame:

```text
x: aircraft right
y: aircraft forward
z: up
```

Before publishing `/move_base_simple/goal`, `startup_goal.py` aligns them to the
EGO `odom`/ROS frame:

```text
ego_x_forward = task_y_forward
ego_y_left = -task_x_right
ego_z_up = task_z_up
```

The goal, `/odom`, and `/cloud_registered` therefore use the same Point-LIO
world frame. EGO consumes the estimator output directly.

The bridge captures the initial PX4 heading and converts the task frame to PX4
NED (`x=forward, y=right, z=down`). By default it also locks commanded yaw to
that initial heading, matching the verified square mission. Set
`lock_yaw_to_initial_heading:=false` only after separately validating yaw
tracking with the real Point-LIO installation.

The hardware Point-LIO profile keeps the validated MID-360 packet timing and
`use_imu_as_input: false`. Its LiDAR innovation gate rejects a single ICP
correction larger than `0.5 m` or `20 deg`, retains the IMU prediction, and
does not insert that frame into the map. These are estimator correction limits,
not aircraft speed or attitude limits. Do not replace this profile with the
Gazebo zero-duration-scan configuration.

## Build

```bash
cd /home/wu/sim-ego/uav-mission-stack
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  bringup_pointlio_hover bringup_ego
```

## Planning-only check

Start the Micro XRCE-DDS Agent in another terminal, then keep PX4 control output
disabled while checking the sensor, estimator, and planner chain:

```bash
cd /home/wu/sim-ego/uav-mission-stack
source install/setup.bash
ros2 launch bringup_ego ego_avoidance_hw.launch.py \
  goal_x:=1.0 goal_y:=0.0 goal_z:=0.5 \
  takeoff_before_ego:=false \
  output_enabled:=false \
  use_px4_monitor:=true \
  use_px4_control_watchdog:=true \
  use_position_compare:=true
```

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /odom
ros2 topic hz /ego/odom_base
ros2 topic hz /cloud_registered
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo base_link base
ros2 topic hz /fmu/in/vehicle_visual_odometry
ros2 topic hz /ego/position_cmd
ros2 topic echo /ego_hw/diagnostics --once
```

The diagnostic must report `ready for controlled test`, PX4 local position must
have `xy_valid: true` and `z_valid: true`, and exactly one node may publish
`/fmu/in/trajectory_setpoint`.

## Hardware output

After a propeller-off bench check, enable the two independent hardware guards.
With `auto_arm:=false`, arm from the RC only after all monitor outputs are
healthy. The bridge then requests Offboard, climbs to `takeoff_altitude`, waits
until stable, publishes the configured EGO goal, and hands control to EGO:

```bash
cd /home/wu/sim-ego/uav-mission-stack
source install/setup.bash
ros2 launch bringup_ego ego_avoidance_hw.launch.py \
  use_livox_driver:=true \
  use_pointlio:=true \
  use_pointlio_px4_visual_odom:=true \
  use_px4_monitor:=true \
  use_px4_control_watchdog:=true \
  use_position_compare:=true \
  takeoff_before_ego:=true \
  takeoff_altitude:=0.40 \
  goal_x:=0.0 goal_y:=2.0 goal_z:=0.40 \
  max_velocity:=0.3 max_acceleration:=0.5 \
  control_mode:=position \
  auto_land_after_goal:=true \
  output_enabled:=true \
  hardware_confirmation:=ENABLE_PX4_OUTPUT \
  auto_arm:=false \
  auto_offboard:=true
```

`takeoff_altitude` is relative to the captured takeoff origin. `goal_x`,
`goal_y`, and `goal_z` are EGO task-frame coordinates (`x=right`, `y=forward`,
`z=up`) and are not published until takeoff is stable.

Automatic landing is disabled by default. Initial thresholds are 0.25 m XY,
0.15 m Z, 0.15 m/s horizontal/vertical speed, stable for 1.0 s. Verify PX4
landing detection with propellers removed before enabling it on the aircraft.

With `start_with_goal:=false`, set each target in RViz using `2D Goal Pose`.
For a fixed altitude, the launch value `goal_z` is used when the clicked pose
has no positive z value.

Do not run another Offboard controller while this package owns PX4's setpoint
topics. The supplied hardware planner profile uses 0.40 m horizontal obstacle
inflation for the 0.55 m diameter aircraft.
