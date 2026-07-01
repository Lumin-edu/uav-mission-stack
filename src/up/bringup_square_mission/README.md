# hx_bringup_square_mission

独立正方形航点任务包。`hx_bringup_pointlio_hover` 不改动，本包复制已验证的 Point-LIO -> PX4 EKF2 visual odom 链路，只把 Offboard 控制器换成正方形任务。

```text
Point-LIO /odom
  -> scripts/pointlio_to_px4_visual_odom.py
  -> /fmu/in/vehicle_visual_odometry
  -> PX4 EKF2
  -> /fmu/out/vehicle_local_position
  -> scripts/square_mission_controller.py
  -> /fmu/in/offboard_control_mode
  -> /fmu/in/trajectory_setpoint
  -> /fmu/in/vehicle_command
```

## 编译

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select hx_bringup_square_mission --symlink-install
source install/setup.bash
```

## 启动

```bash
source install/setup.bash
ros2 launch hx_bringup_square_mission square_mission_hw.launch.py \
  use_pointlio:=true \
  use_pointlio_px4_visual_odom:=true \
  use_px4_monitor:=true \
  use_px4_control_watchdog:=true \
  use_position_compare:=true \
  auto_arm:=false \
  takeoff_altitude:=0.40 \
  square_side_length:=2.0 \
  approach_speed:=0.25
```

`auto_arm:=false` 时不会自动解锁。先确认 `/odom`、`/fmu/in/vehicle_visual_odometry`、`/fmu/out/vehicle_local_position` 正常，再遥控器手动解锁；控制器检测到已解锁后会请求 Offboard。

当前实测默认 `task_y_sign:=1.0` 时，任务 `+y` 为机头前方；不要额外添加 `task_y_sign:=-1.0`，否则正方形第一条边会向机头后方镜像。

## 路线

任务坐标以控制器捕获的起始 PX4 local position 作为 `0,0,0`，和定点悬停包的相对参考逻辑一致。

默认路线：

```text
0.  捕获当前位置作为原点
1.  (0.0, 0.0, 0.30) 起飞悬停
2.  (0.0, 1.0, 0.30) 前方 1m
3.  (1.0, 1.0, 0.30) 右侧 1m
4.  (1.0, 0.0, 0.30) 后退 1m
5.  (0.0, 0.0, 0.30) 回到起飞点上方
```

坐标约定：

```text
x: 初始机头右侧为正
y: 初始机头前方为正
z: 向上为正
```

PX4 local 使用 NED，所以 `z=0.30` 会转换成 `start_z - 0.30`。

所有 `TrajectorySetpoint.yaw` 都固定为起始 `vehicle_local_position.heading`，机头不跟随航线转向。

## 定位和高度

控制器只使用 PX4 EKF2 输出的 `/fmu/out/vehicle_local_position` 做位置反馈和到点判定，不直接用 Point-LIO 原始 `/odom` 控制飞机。

Point-LIO 点云通过 `/fmu/in/vehicle_visual_odometry` 输入 PX4；光流/定高传感器如果已经接入 PX4，则由 PX4 EKF2 按参数进行融合或选择。最终以 `/fmu/out/vehicle_local_position` 的 `xy_valid/z_valid` 和位置高度输出为准。

水平航点推进速度由 `approach_speed` 控制，默认 `1.0 m/s`；垂直爬升速度由 `vertical_speed` 控制，默认 `0.20 m/s`。

## 到点判定

每个航点都要求位置和速度稳定：

```text
水平误差 <= reach_xy_tol   默认 0.20 m
高度误差 <= reach_z_tol    默认 0.10 m
水平速度 <= speed_xy_tol   默认 0.20 m/s
垂直速度 <= speed_z_tol    默认 0.15 m/s
连续稳定 stable_time_sec   默认 0.6 s
```

到起飞高度后悬停 `takeoff_hover_sec`，默认 0.6s；每个角点悬停 `corner_hover_sec`，默认 0.6s；最后回到起飞点上方后保持 `final_hover_sec`，默认 0.6s，然后继续发布最终悬停 setpoint。

## 飞前检查

```bash
ros2 topic hz /odom
ros2 topic hz /fmu/in/vehicle_visual_odometry
ros2 topic echo /fmu/out/vehicle_local_position --once
ros2 topic hz /fmu/in/offboard_control_mode
ros2 topic hz /fmu/in/trajectory_setpoint
```

`/fmu/out/vehicle_local_position` 至少要看到：

```text
xy_valid: true
z_valid: true
```

打开 `use_position_compare:=true` 可以看 PX4 local 与 Point-LIO 增量方向对比。visual odom 参数保持与已验证悬停包一致。
