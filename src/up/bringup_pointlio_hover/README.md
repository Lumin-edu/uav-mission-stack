# hx_bringup_pointlio_hover

独立实机定点悬停 bringup，只保留悬停需要的节点：

```text
Point-LIO /odom
  -> scripts/pointlio_to_px4_visual_odom.py
  -> /fmu/in/vehicle_visual_odometry
  -> PX4 EKF2
  -> /fmu/out/vehicle_local_position
  -> scripts/fixed_point_hover.py
  -> /fmu/in/offboard_control_mode
  -> /fmu/in/trajectory_setpoint
```

## Build

```bash
cd ~/venom
source /opt/ros/humble/setup.bash
colcon build --packages-select hx_bringup_pointlio_hover --symlink-install
source install/setup.bash
```

## Launch

低高度手动解锁测试：

```bash
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
sudo MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
sudo ifconfig enp2s0 192.168.1.50

source install/setup.bash
ros2 launch hx_bringup_pointlio_hover hover_hw.launch.py \
  use_pointlio:=true \
  use_pointlio_px4_visual_odom:=true \
  use_hover_control:=true \
  use_px4_monitor:=true \
  use_px4_control_watchdog:=true \
  use_position_compare:=true \
  takeoff_altitude:=0.2 \
  reference_capture_delay_sec:=3.0 \
  hover_auto_arm:=false
```

`reference_capture_delay_sec` 表示 `fixed_point_hover` 启动后等待多少秒，再用之后收到的第一帧有效 `/fmu/out/vehicle_local_position` 计算悬停目标点。默认 `3.0`，用于让 Point-LIO 和 PX4 EKF local position 先稳定。

`takeoff_altitude` 是相对目标点向上的高度，单位 m；`hover_auto_arm:=false` 时不会由 ROS 自动解锁，需要遥控器手动解锁。

## Checks

```bash
ros2 topic hz /odom
ros2 topic hz /fmu/in/vehicle_visual_odometry
ros2 topic echo /fmu/out/vehicle_local_position --once
```

`/fmu/out/vehicle_local_position` 里至少要看到：

```text
xy_valid: true
z_valid: true
```

如果 `visual odom published | pos_ned=...` 日志太多，可以启动时设置：

```bash
visual_odom_print_rate:=0.0
```
