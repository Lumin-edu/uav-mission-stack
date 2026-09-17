# bringup_ego_mavros

这是基于 Point-LIO 外部定位的独立 MAVROS 定点悬停包，只实现定点位置控制，不包含 EGO 规划、多航点、速度模式或降落流程。目录内没有导入旧 `src/up`、PX4 uXRCE-DDS 或 `px4_msgs`。

数据链路为：

```text
MID360/IMU → Point-LIO /odom → /mavros/vision_pose/pose
             → MAVROS → PX4 EKF → /mavros/local_position/odom
             → 定点控制 → /mavros/setpoint_raw/local → PX4
```

Point-LIO 坐标按 ROS FLU/ENU 处理：机体坐标 x 向前、y 向左、z 向上。桥接节点只做 `base_link`（MID360 IMU 原点）到机体中心 `base` 的刚体外参补偿，不手动转换 NED/FRD；MAVROS 在发送给 PX4 时负责坐标转换。

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

控制器发送的是“位置 + yaw”定点目标。`type_mask=2552` 忽略速度、加速度和 yaw rate，因此速度/加速度不是直接执行指令。ROS 侧位置使用 ENU（z 向上）；`PositionTarget.coordinate_frame` 标为 `FRAME_LOCAL_NED`，由 MAVROS 插件负责发往 PX4 前的 ENU/NED 转换，代码不会再次手动翻转坐标。

## 目标点如何到达 PX4

控制器先在 MAVROS 的局部 ROS 坐标系中计算目标，不把 `target_x/y/z` 当成机体系指令：

```text
target_enu = captured_local_odom_enu + (target_x, target_y, target_z)
```

当 `use_current_position_reference=true`（默认）时，`captured_local_odom_enu` 是节点启动后等待 `reference_capture_delay_sec`（默认 3 秒）再取得的第一条有效 `/mavros/local_position/odom` 位置；之后参考点不会继续更新。因此默认 `target_z=1.0` 表示从这个第 3 秒参考位置向上 1 m。若设为 `false`，`target_x/y/z` 本身就是 MAVROS 局部 ENU 的绝对坐标。

例如捕获位置为 `(2.0, -1.0, 0.1)`、偏移为 `(0, 0, 1)`，发布到 `/mavros/setpoint_raw/local` 的位置就是 `(2.0, -1.0, 1.1)`。这些数值仍保持 ROS ENU 含义：x=东/局部 x，y=北/局部 y，z=上。

虽然消息字段设置为 `coordinate_frame=FRAME_LOCAL_NED (1)`（这是 MAVLink/PX4 的目标帧），本包不会再手动交换 x/y 或取 z 负号。MAVROS 的 `SetpointRawPlugin` 在收到 ROS 消息后执行标准转换并发送 `SET_POSITION_TARGET_LOCAL_NED`：

```text
x_ned = y_enu
y_ned = x_enu
z_ned = -z_enu
```

因此上例最终在 MAVLink/PX4 线上对应 `(north=-1.0, east=2.0, down=-1.1)`。这是唯一一次 ENU→NED 位置转换，避免了应用层二次转换。

航向也在 ROS ENU 中表达。默认 `hold_current_yaw=true`，控制器在捕获同一条本地里程计参考时，将四元数转换为 ENU yaw 并锁定，后续始终发布这个固定 yaw；设置为 `false` 时才使用参数 `target_yaw`。MAVROS 再按其姿态转换发送 PX4，常见等价关系为：

```text
yaw_ned = wrap(pi/2 - yaw_enu)
```

Point-LIO 的 FLU 仅描述机体坐标（x 前、y 左、z 上），桥接器将其位姿送入 MAVROS 外部视觉接口；定点目标使用的是 PX4 EKF 输出的局部世界 ENU（`/mavros/local_position/odom`），不能把两者的 x/y 轴直接混为一谈。`hold_current_yaw` 只锁定航向，不会把 `target_x/y` 按当前机头方向旋转。

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
  target_z:=1.0 auto_arm:=false auto_offboard:=true
```

需要指定 Point-LIO 配置时：

```bash
ros2 launch bringup_ego_mavros fixed_point_mavros.launch.py \
  pointlio_config:=/absolute/path/to/mid360_mapping.yaml
```

如果 Point-LIO 已经由其他 launch 启动，可设置 `use_pointlio:=false`，桥接节点仍会订阅 `pointlio_odom_topic`。

默认 `use_current_position_reference=true`，会在收到 MAVROS 本地里程计并等待 3 秒后，把当前点加上 `(target_x,target_y,target_z)` 作为目标；默认 `target_z=1.0` 表示相对当前高度上升 1 m。若要使用固定局部坐标目标，可设置 `use_current_position_reference:=false`。

实际飞行前请确认 PX4 已启用外部视觉位置融合并允许 OFFBOARD。建议先保持 `auto_arm:=false`，依次确认：

```bash
ros2 topic hz /odom
ros2 topic hz /mavros/vision_pose/pose
ros2 topic hz /mavros/local_position/odom
ros2 topic echo /mavros/state --once
```

只有 `/mavros/local_position/odom` 稳定有效后再解锁。MAVROS 未安装时不能进行飞行联调。
