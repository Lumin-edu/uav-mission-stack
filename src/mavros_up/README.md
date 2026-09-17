# bringup_ego_mavros

这是基于 Point-LIO 外部定位的独立 MAVROS 定点悬停包，只实现定点位置控制，不包含 EGO 规划、多航点、速度模式或降落流程。目录内没有导入旧 `src/up`、PX4 uXRCE-DDS 或 `px4_msgs`。

数据链路为：

```text
MID360/IMU → Point-LIO /odom → /mavros/vision_pose/pose
             → MAVROS → PX4 EKF → /mavros/local_position/odom
             → 定点控制 → /mavros/setpoint_raw/local → PX4
```

Point-LIO 的机体坐标按 ROS FLU 处理：x 向前、y 向左、z 向上；`/odom` 中的位姿位置是 `base_link` 在 `odom` 世界坐标中的位置。桥接节点只做 `base_link`（MID360 IMU 原点）到机体中心 `base` 的刚体外参补偿，不手动转换 NED/FRD；MAVROS 在发送给 PX4 时负责坐标转换。

## 通信接口

`fixed_point_mavros.py` 使用以下 MAVROS 接口：

- 订阅 `/mavros/state`（`mavros_msgs/msg/State`）和 `/mavros/local_position/odom`（`nav_msgs/msg/Odometry`）；
- 发布 `/mavros/setpoint_raw/local`（`mavros_msgs/msg/PositionTarget`）；
- 按需调用 `/mavros/cmd/arming` 和 `/mavros/set_mode`；

`pointlio_to_mavros_vision_pose.py`：

- 订阅 Point-LIO `/odom`（`nav_msgs/msg/Odometry`）；
- 发布 `/mavros/vision_pose/pose`（`geometry_msgs/msg/PoseStamped`）；
- 默认以 50 Hz 向 MAVROS 提供外部视觉位姿；
- 默认外参位于 `config/pointlio_mavros_bridge.yaml`。

控制器发送的是“位置 + yaw”定点目标。`type_mask=2552` 忽略速度、加速度和 yaw rate，因此速度/加速度不是直接执行指令。任务坐标定义为 `(x_right, y_forward, z_up)`：机头右侧、机头前方、上方。第 3 秒捕获的 MAVROS 本地位置和姿态建立任务原点与固定航向。

## 目标点如何到达 PX4

控制器先把任务坐标转换为捕获姿态下的固定局部世界目标，不把 `target_x/y/z` 直接当成 ENU 世界轴：

```text
task_offset = (x_right, y_forward, z_up)
body_flu_offset = (y_forward, -x_right, z_up)
world_offset = R_capture_yaw(odom <- base_link) * body_flu_offset
target_local = captured_local_position + world_offset
```

节点启动后等待 `reference_capture_delay_sec`（默认 3 秒），用同一条有效 `/mavros/local_position/odom` 捕获位置、姿态和航向；任务目标只允许使用这个捕获点作为原点，偏移只转换一次，之后不随实时航向变化。因此 `target_x=1.0` 表示向捕获时机头右侧 1 m，`target_y=2.0` 表示向捕获时机头前方 2 m，`target_z=3.0` 表示向上 3 m。

例如捕获位置为 `(2.0, -1.0, 0.1)`、捕获航向为 0、任务偏移为 `(1, 2, 3)`，固定目标为 `(4.0, -2.0, 3.1)`。这里目标坐标是局部世界坐标；任务参数本身仍是 `(右, 前, 上)`。

虽然消息字段设置为 `coordinate_frame=FRAME_LOCAL_NED (1)`（这是 MAVLink/PX4 的目标帧），应用层填入的 `PositionTarget.position` 仍是 MAVROS ROS 侧局部 ENU 数值；本包不会再手动交换 x/y 或取 z 负号。MAVROS 的 `SetpointRawPlugin` 在收到 ROS 消息后执行标准转换并发送 `SET_POSITION_TARGET_LOCAL_NED`：

```text
x_ned = y_enu
y_ned = x_enu
z_ned = -z_enu
```

因此上例最终在 MAVLink/PX4 线上对应 `(north=-1.0, east=2.0, down=-1.1)`。这是唯一一次 ENU→NED 位置转换，避免了应用层二次转换。

这与 `bringup_full_mission` 的 PX4 NED 任务坐标结果等价。若捕获时 PX4 heading 为 `h`，MAVROS ROS yaw 满足 `yaw_enu=pi/2-h`，展开后的任务偏移为：

```text
x_ned = y_forward*cos(h) - x_right*sin(h)
y_ned = y_forward*sin(h) + x_right*cos(h)
z_ned = -z_up
```

航向也在 ROS ENU 中表达。默认 `hold_current_yaw=true`，控制器在捕获同一条本地里程计参考时，将四元数转换为 ENU yaw 并锁定，后续始终发布这个固定 yaw；设置为 `false` 时才使用参数 `target_yaw`。MAVROS 再按其姿态转换发送 PX4，常见等价关系为：

```text
yaw_ned = wrap(pi/2 - yaw_enu)
```

Point-LIO 的 FLU 描述机体坐标（x 前、y 左、z 上），桥接器将其位姿送入 MAVROS 外部视觉接口；控制器从 MAVROS 的局部里程计读取捕获姿态，将任务 `(右, 前, 上)` 转成固定局部世界目标。`hold_current_yaw=true` 时，航向锁定为捕获航向；目标不会随之后的实时机头方向重新旋转。

## 启动

先在另一个终端启动 MAVROS（连接串按实际串口/UDP 配置填写），例如：

```bash
source /opt/ros/humble/setup.bash
ros2 launch mavros px4.launch fcu_url:=serial:///dev/ttyACM0:921600
```

再启动本包。该 launch 默认自动启动 Point-LIO、外部位姿桥和定点控制器：

```bash
source install/setup.bash
ros2 launch bringup_ego_mavros fixed_point_mavros.launch.py \
  target_x:=1.0 target_y:=2.0 target_z:=3.0 \
  auto_arm:=false auto_offboard:=true
```

需要指定 Point-LIO 配置时：

```bash
ros2 launch bringup_ego_mavros fixed_point_mavros.launch.py \
  pointlio_config:=/absolute/path/to/mid360_mapping.yaml
```

如果 Point-LIO 已经由其他 launch 启动，可设置 `use_pointlio:=false`，桥接节点仍会订阅 `pointlio_odom_topic`。

控制器会在收到 MAVROS 本地里程计并等待 3 秒后，以捕获时航向为基准，把 `(target_x,target_y,target_z)` 解释为 `(右,前,上)` 任务偏移。上面的示例表示向捕获时机头右侧 1 m、前方 2 m、上方 3 m。

实际飞行前请确认 PX4 已启用外部视觉位置融合并允许 OFFBOARD。建议先保持 `auto_arm:=false`，依次确认：

```bash
ros2 topic hz /odom
ros2 topic hz /mavros/vision_pose/pose
ros2 topic hz /mavros/local_position/odom
ros2 topic echo /mavros/state --once
```

只有 `/mavros/local_position/odom` 稳定有效后再解锁。MAVROS 未安装时不能进行飞行联调。
