# ROS1 FAST-LIO 定点与抛投任务

本目录是独立的 ROS1 `bringup` 包，负责启动 MID360、FAST-LIO、FAST-LIO 到
MAVROS 的视觉里程计桥接、定点任务、相机、AprilTag 和舵机。MAVROS 不由本
目录启动，需要单独启动。

## 坐标系

任务点使用 RFU 坐标，原点是任务启动后捕获的 PX4/MAVROS 本地位置：

- `target_x`：机头右方
- `target_y`：机头前方
- `target_z`：机头上方

FAST-LIO 输入话题是 `/Odometry`，视觉桥发布 ROS ENU 到
`/mavros/vision_pose/pose`，MAVROS 再转换给 PX4。任务点由捕获的 PX4 yaw
转换为 MAVROS local ENU 后发送到 `/mavros/setpoint_position/local`。默认
`lock_yaw:=true`，飞行过程中保持捕获时的 yaw。`base_link -> base` 杠杆
臂默认是 0 m。

## MAVROS 与基本启动

先单独启动 MAVROS：

```bash
source /opt/ros/noetic/setup.bash
source /home/yundrone/sysu/devel/setup.bash
roslaunch mavros px4.launch fcu_url:=/dev/ttyTHS0:921600
```

普通定点悬停：

```bash
cd /home/yundrone/sysu
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch bringup hover_hw.launch \
  use_fastlio:=true \
  use_fastlio_mavros_visual_odom:=true \
  use_hover_control:=true \
  target_x:=0.0 target_y:=0.0 target_z:=0.4 \
  lock_yaw:=true target_yaw:=0.0 \
  hover_auto_arm:=false
```

`hover_auto_arm:=false` 时由遥控器手动解锁；控制器预发布 setpoint 后请求
PX4 `OFFBOARD`。

## 八点抛投与降落任务

启动文件是 `launch/tag_drop_mission.launch`，任务点文件是
`config/tag_drop_targets.yaml`。

任务顺序：

1. 依次飞行到第 1 至第 5 点。
2. 第 5 点搜索 AprilTag ID 0 并进行水平 RFU 修正；对准后立即抛投。
3. 如果在 `drop_search_seconds` 内未识别或未对准，仍在第 5 点抛投。
4. 抛投后继续飞行第 6、7、8 点，不直接返航。
5. 到达第 8 点后搜索 AprilTag ID 0，进行水平修正；对准后立即降落。
6. 如果在 `landing_search_seconds` 内未识别或未对准，也在第 8 点降落。

默认第 8 点 `target_z=0.0`，表示回到捕获原点高度。AprilTag 修正只改水平
位置，最大修正速度默认是 `0.30 m/s`，比正常航线速度慢。

降落命令被 MAVROS 接受后，节点请求 `AUTO.LAND`。PX4 确认离开
`OFFBOARD` 后，节点停止发布 Offboard setpoint。当 PX4 local z 与第 8 点
实际目标高度误差不超过 5 cm，并且 PX4 报告已经在地面时，节点请求
`/mavros/cmd/arming false` 完成停桨。

### 可调 launch 参数

- `drop_search_seconds`：到达第 5 点后，抛投识别与对准的最长时间，默认 `3.0` 秒。
- `landing_search_seconds`：到达第 8 点后，降落识别与对准的最长时间，默认 `3.0` 秒。
- `tag_correction_speed`：AprilTag 水平修正速度上限，默认 `0.30` m/s。
- `max_speed`：正常航线速度上限，默认 `0.5` m/s。
- `tag_xy_tolerance`：水平对准容差，默认 `0.15` m，可从 launch 命令设置。
- `disarm_height_tolerance`：停桨高度误差，默认 `0.05` m。
- `video_device`：相机设备，默认自动选择。
- `servo_port`：舵机串口，默认自动选择。

例如抛投对准 5 秒、降落对准 8 秒：

```bash
cd /home/yundrone/sysu
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch bringup tag_drop_mission.launch \
  video_device:=/dev/video0 \
  servo_port:=/dev/ttyUSB0 \
  drop_tag_id:=0 landing_tag_id:=0 \
  drop_search_seconds:=5.0 \
  landing_search_seconds:=5.0 \
  tag_xy_tolerance:=0.15 \
  tag_correction_speed:=0.30 \
  disarm_height_tolerance:=0.05 \
  max_speed:=0.5 lock_yaw:=true
```

MAVROS 必须先启动。不要同时启动 `hover_hw.launch`、`up.launch` 和
`tag_drop_mission.launch`，否则会有多个节点同时发布位置 setpoint。

## 相机与 AprilTag 检查

相机默认分辨率 1920x1080、30 Hz。相机位于载荷中心前方 0.115 m，正下视，
图像上方对应机头前方。检查命令：

```bash
rostopic echo /tag_detections
rostopic echo /bringup/tag/drop_offset_rfu
rostopic echo /bringup/tag/landing_offset_rfu
rostopic echo /bringup/tag/state
rostopic echo /tag_drop_mission/mission_phase
```

上一轮日志中相机成功打开 `/dev/video0`，AprilTag 节点成功连接图像和相机
内参，但没有出现 ID 0 检测，因此 tracker 没有发布稳定 offset，任务按 3 秒
无识别兜底执行。这不表示相机节点故障，需要结合飞行高度、光照、Tag 尺寸
和朝向继续检查 `/tag_detections`。

## 舵机

舵机使用 115200 baud、原始 HEX 协议。启动不自动发送角度；手动测试：

```bash
roslaunch bringup servo_test.launch servo_port:=/dev/ttyUSB0
rosservice call /bringup_servo/set_angle "angle: 90.0"
```

默认标定中 0/90/180 度对应 duty 3/8/12，90 度帧为：

```text
5A 01 00 32 08
```

## 编译检查

```bash
cd /home/yundrone/sysu
source /opt/ros/noetic/setup.bash
catkin_make --pkg bringup
source devel/setup.bash
python3 -m py_compile src/bringup/scripts/*.py
rossrv show bringup/SetServoAngle
```
