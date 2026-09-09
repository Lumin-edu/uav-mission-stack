# fastlio_bringup

独立的 FAST-LIO + PX4 无人机定点悬停 bringup。这个目录不导入或调用其他
bringup 包的脚本；Livox 驱动和 `MicroXRCEAgent` 由外部命令启动。

## 数据链路

```text
Livox driver
  -> /livox/lidar + /livox/imu
  -> fast_lio/fastlio_mapping
  -> /Odometry (camera_init -> body)
  -> fastlio_to_px4_visual_odom.py
  -> /fmu/in/vehicle_visual_odometry
  -> PX4 EKF2
  -> /fmu/out/vehicle_local_position
  -> fixed_point_hover.py
  -> PX4 Offboard position setpoint
```

FAST-LIO 的 `body` 是 MID-360 IMU 原点。桥接节点先使用
`T_world_base = T_world_body * T_body_base` 补偿到机体中心 `base`，再进行
初始参考和 ROS 到 PX4 NED 坐标转换。默认外参为：

```text
body_to_base_translation = [-0.011, -0.02329, -0.05588] m
body_to_base_rotation_xyzw = [0, 0, 0, 1]
```

默认坐标转换为 `(x, y, z) -> (N, E, D) = (x, -y, -z)`。补偿平移会随当前姿态
旋转，不能直接把固定杆臂量加到世界坐标。

PX4 视觉里程计默认只融合位置，`publish_orientation` 默认为 `false`；如果需要
融合姿态，可显式开启，桥接节点会发布完整的 FLU→FRD/NED 四元数，而不是仅发布偏航。

## 编译

```bash
cd /home/wu/sim-ego/uav-mission-stack
source /opt/ros/humble/setup.bash
colcon build --packages-select fastlio_bringup --symlink-install
source install/setup.bash
```

## 启动外部进程

先启动 Livox MID-360 驱动：

```bash
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

另一个终端启动 PX4 DDS Agent，例如：

```bash
sudo MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
```

确认 `/livox/lidar`、`/livox/imu` 和 PX4 `/fmu/out/*` 话题已经出现后，再启动本包：

```bash
source install/setup.bash
ros2 launch fastlio_bringup fastlio_hover.launch.py \
  use_fastlio:=true \
  use_visual_odom:=true \
  use_hover_control:=true \
  use_px4_control_watchdog:=true \
  use_px4_monitor:=true \
  use_position_compare:=true \
  takeoff_altitude:=0.2 \
  reference_capture_delay_sec:=3.0 \
  hover_auto_arm:=false
```

默认不会自动解锁。遥控器手动解锁后，控制器才会请求 Offboard。`takeoff_altitude`
是相对捕获位置的向上高度，单位米；PX4 NED 中对应负 Z。实飞时只能保留一个
PX4 Offboard 控制源。

## 有用参数

- `fastlio_config_file`: 本目录内的 FAST-LIO YAML，也可以传入另一份兼容配置。
- `fastlio_topic`: FAST-LIO 里程计，默认 `/Odometry`。
- `body_to_base_translation`: 机体中心相对 IMU 原点的平移。
- `max_output_abs_z`: 输出 Z 安全限制，默认 20 m。
- `max_position_jump`: 相邻视觉里程计位置跳变限制，默认 3 m。
- `use_body_to_base_tf`: 是否发布静态 `body -> base` TF；数值桥接始终执行一次补偿。
- `use_rviz`: 是否启动本目录内的 RViz 配置。

## 检查

```bash
ros2 topic hz /Odometry
ros2 topic hz /fmu/in/vehicle_visual_odometry
ros2 topic echo /fmu/out/vehicle_local_position --once
ros2 topic info -v /Odometry
ros2 topic info -v /fmu/in/vehicle_visual_odometry
```

桥接日志应显示 `FAST-LIO -> PX4 visual odometry bridge active`，并报告输入帧
`camera_init`、子帧 `body` 和输出机体 `base`。启用对比节点后，日志中的
`NED increment compare` 用于检查桥接输入与 PX4 本地位置的相对运动方向。

## 安全说明

- 默认 `hover_auto_arm:=false`，不会由 ROS 自动解锁。
- 控制器先连续发送至少 2 秒的 Offboard heartbeat/setpoint。
- 没有有效 `/fmu/out/vehicle_local_position` 时不会捕获目标或请求 Offboard。
- 非有限位姿、过大 Z 值和位置跳变会被桥接丢弃。
- `px4_control_watchdog.py` 只检查输入发布者，不会发送飞行控制命令。
