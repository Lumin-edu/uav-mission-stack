# hx_bringup_full_mission

完整航点任务包。Point-LIO 负责 PX4 EKF2 外部视觉定位；D435i 不发布定位信息，不修正 `/fmu/in/vehicle_visual_odometry`，只给当前 setpoint 提供有限修正量。

```text
Point-LIO /odom
  -> base_link 到 base 机体中心补偿
  -> scripts/pointlio_to_px4_visual_odom.py
  -> /fmu/in/vehicle_visual_odometry
  -> PX4 EKF2
  -> /fmu/out/vehicle_local_position
  -> scripts/full_mission_controller.py
  -> /fmu/in/trajectory_setpoint
```

## Point-LIO 机体中心补偿

Point-LIO 继续发布 `odom -> base_link`，其中 `base_link` 的数值对应 MID360 IMU
原点。修正后的机体中心统一命名为 `base`：

```text
T_odom_base = T_odom_base_link * T_base_link_base
t_base_link_base = [-0.011, -0.02329, -0.05588] m
q_base_link_base_xyzw = [0, 0, 0, 1]
```

launch 同时发布固定 TF `base_link -> base`，用于补全
`odom -> base_link -> base`。桥接节点在记录初始参考点前只做一次相同的数值补偿，
然后沿用本包原有的 `base -> PX4 NED` 轴向、零点和航向转换。静态 TF 不参与
PX4 消息计算，因此不会重复补偿。

位置对比节点订阅实际发送给 PX4 的 `/fmu/in/vehicle_visual_odometry`，不再使用
原始 `/odom`，这样比较的是修正后 `base` 的 NED 增量。

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch hx_bringup_full_mission full_mission_hw.launch.py \
  use_pointlio:=true \
  use_pointlio_px4_visual_odom:=true \
  use_px4_control_watchdog:=true \
  auto_arm:=false \
  selected_tasks:=1,3,5,6 \
  drop_port:=/dev/ttyUSB0 \
  drop_d435i_serial_no:=233622072879 \
  enable_d435i_drop_alignment:=true \
  start_drop_d435i_driver_on_alignment:=true \
  enable_d435i_ring_alignment:=false \
  start_ring_d435i_driver_on_alignment:=false \
  approach_speed:=0.15 \
  vertical_speed:=0.10 \
  visual_correction_speed:=0.10

## 视觉修正

本任务使用两台 D435i，话题必须分开，避免混用。

下视 D435i 只用于 1..5 作业点黑框修正：

```text
默认 namespace/name: d435i_down/d435i_down
RGB:         /d435i_down/d435i_down/color/image_raw
camera_info: /d435i_down/d435i_down/color/camera_info
```

下视修正流程：

```text
到达被 selected_tasks 选中的 1..5 作业点
  -> 悬停稳定
  -> 进入 STATE_VISUAL_ALIGN
  -> 下视 D435i 黑框 XY 修正
  -> 修正稳定 0.6 s
  -> 清空本次视觉缓存
  -> 执行抛投
  -> 继续下一个航点
```

前视 D435i 只用于圆环穿越前的左右修正：

```text
默认 namespace/name: d435i_front/d435i_front
RGB:         /d435i_front/d435i_front/color/image_raw
camera_info: /d435i_front/d435i_front/color/camera_info
```

前视圆环修正默认只启动 RGB，不启动 depth，减少 USB 带宽压力；左右偏差由圆环已知直径 0.90 m 和 RGB 外轮廓像素尺寸估算。

前视修正流程：

```text
到达特殊靶 5 回升点 (-4.80, -0.80, 1.42)
  -> 悬停稳定
  -> 启动前视 D435i
  -> 进入 STATE_RING_ALIGN
  -> 识别直径 0.90 m 的白色泡沫圆环
  -> 只修正机体左右 x_right，不修正上下和高度
  -> 左右偏差稳定 0.6 s
  -> 停止前视 D435i 进程
  -> 冻结修正后的横向位置
  -> 直接飞圆环出口，穿越过程中不再修正
```

如果两台 RealSense 都连接，强烈建议通过 serial 固定设备：

```text
drop_d435i_serial_no:=<下视相机序列号>
ring_d435i_serial_no:=<前视相机序列号>
```

## 高度和速度

```text
巡航/返航/回升高度: 1.42 m
穿环高度:           1.42 m
任务 1..5 作业高度: 0.65 m
降落点高度:         0.00 m
```

速度默认：

```text
approach_speed:=0.25
vertical_speed:=0.25
visual_correction_speed:=0.10
ring_visual_correction_speed:=0.10
```

代码里做了上限钳制：正常航点 XY/Z 速度不超过 0.25 m/s；下视作业点修正和前视圆环左右修正速度都不超过 0.10 m/s。

Yaw 在捕获任务原点时锁定为 PX4 当前 heading，后续每帧 `TrajectorySetpoint.yaw` 都发布同一个 `locked_yaw`，机头方向不变。

## 抛投接口

抛投串口默认使用 `/dev/ttyUSB0`，可以通过 launch 参数切换：

```text
drop_port:=/dev/ttyUSB0
drop_port:=/dev/ttyUSB1
```

实际执行命令格式：

```bash
python3 <your_drop_script> --port <drop_port> --angle <角度>
python3 <your_drop_script> --port /dev/ttyUSB0 --angle 90
```

抛投角度按选择顺序分配：

```text
第 1 个被选中的 1..5 任务: 90 deg
第 2 个被选中的 1..5 任务: 125 deg
第 3 个被选中的 1..5 任务: 175 deg
```

例如 `selected_tasks:=1,3,5,7` 表示：任务 1 抛 90 度，任务 3 抛 125 度，任务 5 抛 175 度，最后降落到 7。

## 完整航点顺序

坐标格式为 `(x_right, y_forward, z_up)`，即右侧为 X 正方向，机头前方为 Y 正方向，向上为 Z 正方向。任务原点是 Point-LIO 稳定后捕获的 PX4 当前 local 位置，yaw 锁定为起始 heading。

公共航线按顺序执行：

```text
01. 起飞巡航点:             ( 0.00,  0.00, 1.42)
02. 障碍物方向巡航点 1:     (-1.44,  0.00, 1.42)
03. 障碍物前绕行起始点:     (-2.88,  0.00, 1.42)
04. 靶 1 巡航点:            (-2.88,  1.28, 1.42)
05. 靶 1 作业点:            (-2.88,  1.28, 0.65)
06. 靶 1 回升点:            (-2.88,  1.28, 1.42)
07. 靶 2 巡航点:            (-1.44,  1.28, 1.42)
08. 靶 2 作业点:            (-1.44,  1.28, 0.65)
09. 靶 2 回升点:            (-1.44,  1.28, 1.42)
10. 靶 3 巡航点:            (-1.44, -1.28, 1.42)
11. 靶 3 作业点:            (-1.44, -1.28, 0.65)
12. 靶 3 回升点:            (-1.44, -1.28, 1.42)
13. 靶 4 巡航点:            (-2.88, -1.28, 1.42)
14. 靶 4 作业点:            (-2.88, -1.28, 0.65)
15. 靶 4 回升点:            (-2.88, -1.28, 1.42)
16. 特殊靶 5 前置巡航点:    (-4.80, -1.28, 1.42)
17. 特殊靶 5 巡航点:        (-4.80, -0.80, 1.42)
18. 特殊靶 5 作业点:        (-4.80, -0.80, 0.65)
19. 特殊靶 5 回升点:        (-4.80, -0.80, 1.42)
    到达并悬停后，在此处启动前视 D435i 圆环左右修正。
20. 圆环出口点:             (-4.80,  1.60, 1.42)
    实际执行时会叠加第 19 点获得的横向修正量。
21. 穿环完成回升点:         (-4.80,  1.70, 1.42)
    实际执行时会叠加第 19 点获得的横向修正量。
22. 返航中转巡航点:         ( 0.00,  1.70, 1.42)
23. 降落 6 上方巡航点:      ( 0.00,  1.28, 1.42)
```

如果选择降落点 6，继续执行：

```text
24. 降落点 6:               ( 0.00,  1.28, 0.00)
```

如果选择降落点 7，会先经过降落 6 上方巡航点，但不下降到 6，然后继续执行：

```text
24. 降落 7 上方巡航点:      ( 0.00, -1.28, 1.42)
25. 降落点 7:               ( 0.00, -1.28, 0.00)
```

## 坐标转换

下视 D435i 与已验证的 `hx_bringup_d435i_landing` 思路一致：

```text
黑框中心像素
  -> camera_info + 当前高度
  -> body_x_right/body_y_forward
  -> 按 locked_yaw 转 PX4 local XY
  -> 叠加到当前作业点 local setpoint
```

默认下视相机偏移：

```text
drop_camera_offset_body_x_right   =  0.000
drop_camera_offset_body_y_forward =  0.065
drop_camera_offset_body_z_up      = -0.120
```

前视 D435i 只使用左右方向：

```text
白色圆环外轮廓中心像素 u
  -> 已知圆环直径 0.90 m 估算距离
  -> body_x_right
  -> 按 locked_yaw 转 PX4 local XY
  -> 叠加到特殊靶 5 回升点、圆环出口点、穿环完成回升点
```

默认前视相机偏移：

```text
ring_camera_offset_body_x_right   = 0.000
ring_camera_offset_body_y_forward = 0.065
ring_camera_offset_body_z_up      = 0.120
ring_lateral_sign                 = 1.0
```

前视的 `y_forward/z_up` 偏移保留为参数，但当前穿环修正只使用左右偏移 `ring_camera_offset_body_x_right`，不修正上下和高度。

默认 `ring_lateral_sign:=1.0` 表示图像右侧为机体右侧。如果现场测试发现左右修正反了，启动时改成：

```text
ring_lateral_sign:=-1.0
```

## 启动

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch hx_bringup_full_mission full_mission_hw.launch.py \
  use_pointlio:=true \
  use_pointlio_px4_visual_odom:=true \
  use_px4_control_watchdog:=true \
  auto_arm:=false \
  selected_tasks:=1,3,5,7 \
  drop_port:=/dev/ttyUSB0 \
  approach_speed:=0.25 \
  vertical_speed:=0.25 \
  visual_correction_speed:=0.10 \
  ring_visual_correction_speed:=0.10
```

默认 `auto_arm:=false`，需要手动解锁。控制器检测到已解锁后会请求 Offboard。

如果已经手动启动某台 D435i，可以关闭对应自动启动：

```text
start_drop_d435i_driver_on_alignment:=false
start_ring_d435i_driver_on_alignment:=false
```

## 检查话题

```bash
ros2 topic echo /fmu/out/vehicle_local_position --once
ros2 topic hz /fmu/in/trajectory_setpoint
ros2 topic hz /d435i_down/d435i_down/color/image_raw
ros2 topic echo /d435i_down/d435i_down/color/camera_info --once
ros2 topic hz /d435i_front/d435i_front/color/image_raw
ros2 topic echo /d435i_front/d435i_front/color/camera_info --once
```

起飞前 `/fmu/out/vehicle_local_position` 需要：

```text
xy_valid: true
z_valid: true
dead_reckoning: false
```
