# hx_bringup_pointlio_hover

独立实机定点悬停 bringup，只保留悬停需要的节点：

```text
Point-LIO /odom
  -> T_odom_base_link * T_base_link_base
  -> scripts/pointlio_to_px4_visual_odom.py
  -> /fmu/in/vehicle_visual_odometry
  -> PX4 EKF2
  -> /fmu/out/vehicle_local_position
  -> scripts/fixed_point_hover.py
  -> /fmu/in/offboard_control_mode
  -> /fmu/in/trajectory_setpoint
```

## IMU 到机体中心补偿

Point-LIO 的状态位置是 MID360 内置 IMU 的测量原点，并继续使用 `base_link` 作为
输出子坐标系名称。补偿后的无人机机体中心统一命名为 `base`。桥接节点不修改
Point-LIO，在记录初始参考点和转换 PX4 NED 坐标之前计算：

```text
T_odom_base = T_odom_base_link * T_base_link_base
p_odom_base = p_odom_base_link
              + R_odom_base_link * t_base_link_base
```

当前 LiDAR 到 IMU 外参为：

```text
t_mid360_imu_lidar = [-0.011, -0.02329, 0.04412] m
R_mid360_imu_lidar = I
```

机体中心位于雷达正下方 `0.10 m`，且 IMU、雷达、`base_link` 和 `base` 轴向
一致，因此 hover launch 中配置为：

```yaml
base_link_to_base_translation: [-0.011, -0.02329, -0.05588]
base_link_to_base_rotation_xyzw: [0.0, 0.0, 0.0, 1.0]
```

这里的平移是 `base` 原点在 `base_link` 坐标系中的坐标，不是反方向。
补偿会随无人机姿态旋转，不能把这三个数直接加到 Point-LIO 世界坐标上。

不要同时设置 PX4 的外部视觉位置偏置，否则会重复补偿。

TF 树为：

```text
odom -> base_link       Point-LIO 动态发布，base_link 表示 IMU 测量原点
     -> base            hover launch 静态发布，base 表示机体中心
```

`base_link -> base` 和桥接节点使用同一组外参常量。PX4 `VehicleOdometry` 没有字符串
形式的子坐标系字段，但其中的位置和姿态数值都按修正后的 `base` 机体中心解释。
静态 TF 只用于补全 TF 树，桥接节点不会读取 TF；它只在进入原有 PX4 NED 转换前
执行一次 `base_link -> base` 数值补偿。`base -> PX4 NED` 的轴向、参考零点、航向
和话题路径均保持原逻辑不变。

## Build

```bash
cd /home/wu/sim-ego
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
ros2 topic info -v /odom
ros2 topic info -v /fmu/in/vehicle_visual_odometry
ros2 topic info -v /fmu/out/vehicle_local_position
```

桥接节点启动日志应包含：

```text
pose_interpretation=T_odom_base_link
output_body=base
t_base_link_base=(-0.011, -0.02329, -0.05588)
```

启用 `use_position_compare:=true` 时，比较节点订阅的是桥接节点实际发送给 PX4 的
`/fmu/in/vehicle_visual_odometry`，而不是原始 `/odom`。两侧都是补偿后的
`base` NED 位移，因此日志中的 `NED increment compare` 才可用于检查
PX4 EKF 输入和输出是否一致。

`/fmu/out/vehicle_local_position` 里至少要看到：

```text
xy_valid: true
z_valid: true
```

如果 `visual odom published | pos_ned=...` 日志太多，可以启动时设置：

```bash
visual_odom_print_rate:=0.0
```
