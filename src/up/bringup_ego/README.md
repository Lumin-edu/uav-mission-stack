# hx_bringup_ego

Standalone hardware bringup for MID-360, Point-LIO, EGO Planner, and PX4
Offboard control.

```text
/livox/lidar + /livox/imu
  -> Point-LIO /odom + /cloud_registered
  -> planner fusion: Point-LIO x/y + PX4 range-aided z
  -> /ego/odom_fused + /ego/cloud_registered_fused
  -> EGO /ego/position_cmd (100 Hz B-spline samples)
  -> PX4 TrajectorySetpoint
```

The package starts no Gazebo nodes, synthetic sensors, scenario maps, fake
odometry, or `/sim/*` topics.

The verified `hx_bringup_pointlio_hover/pointlio_to_px4_visual_odom.py`
continues to consume raw `/odom`. The planner-only fusion path must never feed
back into PX4 visual odometry. EGO, its obstacle map, the startup goal gate, and
the EGO-to-PX4 controller consume `/ego/odom_fused` and
`/ego/cloud_registered_fused` instead.

On flat ground, planner height is computed from PX4 NED height while retaining
the initial Point-LIO ROS-up origin:

```text
ego_z = initial_pointlio_z - (px4_z - initial_px4_z)
ego_vz = -px4_vz
cloud_z += ego_z - current_pointlio_z
```

The fusion health gate requires a valid range sensor by default. Before flight,
verify PX4 1.14 is configured for horizontal-only external vision and range
height:

```text
EKF2_EV_CTRL = 1
EKF2_HGT_REF = 2
EKF2_RNG_CTRL = 2
```

This range-height mode is restricted to a flat surface. Do not use it above
tables, steps, ramps, or changing terrain.

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

Task and EGO coordinates match the validated square mission:

```text
x: aircraft right
y: aircraft forward
z: up
```

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
  hx_bringup_pointlio_hover hx_bringup_ego
```

## Planning-only check

Start the Micro XRCE-DDS Agent in another terminal, then keep PX4 control output
disabled while checking the sensor, estimator, and planner chain:

```bash
cd /home/wu/sim-ego/uav-mission-stack
source install/setup.bash
ros2 launch hx_bringup_ego ego_avoidance_hw.launch.py \
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
ros2 topic hz /cloud_registered
ros2 topic hz /ego/odom_fused
ros2 topic hz /ego/cloud_registered_fused
ros2 topic echo /ego/height_fusion_healthy --once
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
ros2 launch hx_bringup_ego ego_avoidance_hw.launch.py \
  use_livox_driver:=true \
  use_pointlio:=true \
  use_pointlio_px4_visual_odom:=true \
  use_px4_monitor:=true \
  use_px4_control_watchdog:=true \
  use_position_compare:=true \
  require_rangefinder_height:=true \
  takeoff_before_ego:=true \
  takeoff_altitude:=0.40 \
  goal_x:=0.0 goal_y:=2.0 goal_z:=0.40 \
  max_velocity:=0.3 max_acceleration:=0.5 \
  control_mode:=position \
  output_enabled:=true \
  hardware_confirmation:=ENABLE_PX4_OUTPUT \
  auto_arm:=false \
  auto_offboard:=true
```

`takeoff_altitude` is relative to the captured takeoff origin. `goal_x`,
`goal_y`, and `goal_z` are EGO task-frame coordinates (`x=right`, `y=forward`,
`z=up`) and are not published until takeoff is stable.

With `start_with_goal:=false`, set each target in RViz using `2D Goal Pose`.
For a fixed altitude, the launch value `goal_z` is used when the clicked pose
has no positive z value.

Do not run another Offboard controller while this package owns PX4's setpoint
topics. The supplied hardware planner profile uses 0.40 m horizontal obstacle
inflation for the 0.55 m diameter aircraft.
