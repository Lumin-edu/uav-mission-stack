# UAV Mission Stack

这是一个基于 ROS 2 Humble 的无人机实飞任务工作空间，当前上传版本以 MID-360 + Point-LIO + PX4 1.14 为主链路，重点包含 Point-LIO 到 PX4 EKF2 的视觉里程计桥接，以及定点悬停任务封装。

当前重点入口：

```text
src/up/bringup_pointlio_hover  # 定点悬停
src/up/bringup_square_mission  # 正方形航点
src/up/bringup_ego             # EGO 自主避障
```

本仓库暂不包含以下内容：

```text
src/bringup                 # 历史/协作中的 bringup 任务栈
src/test                    # 测试包，交给其他贡献者维护
src/up/bringup_pillar_orbit # 绕柱任务，暂不上传
build/
install/
log/
```

## Repository Layout

```text
uav-mission-stack/
├── src/
│   ├── driver/
│   │   └── livox_ros_driver2/        # Livox MID-360 ROS 2 驱动
│   ├── perception/
│   │   ├── Point-LIO/                # 当前推荐定位主链路
│   │   └── FAST_LIO/                 # 保留的 FAST-LIO ROS 2 上游包
│   ├── control/
│   │   └── px4_msgs/                 # PX4 1.14 对应 ROS 2 消息定义
│   ├── planning/
│   │   └── ego-planner-swarm/         # EGO-Planner EGO 规划器 ROS 2 (ros2_version 分支)
│   └── up/
│       ├── bringup_pointlio_hover/    # 本项目：Point-LIO + PX4 定点悬停
│       ├── bringup_square_mission/    # 本项目：正方形航点任务
│       ├── bringup_full_mission/      # 本项目：完整联合任务
│       └── bringup_ego/              # 本项目：EGO 自主避障
└── README.md
```

## Upstream Projects

`driver`、`perception`、`control` 下是集成进工作空间的上游开源项目。本仓库按普通源码目录保存这些项目，便于一次 clone 后直接构建；各项目原始来源如下：

| 本地路径 | 用途 | 上游链接 | 当前本地分支/提交 |
| --- | --- | --- | --- |
| `src/driver/livox_ros_driver2` | Livox MID-360 ROS/ROS 2 驱动 | [Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) | `master@13eb05e` |
| `src/perception/Point-LIO` | 当前定位主链路，输出 `/odom` | [HY-LiYihan/Point-LIO](https://github.com/HY-LiYihan/Point-LIO)；原始项目见 [hku-mars/Point-LIO](https://github.com/hku-mars/Point-LIO) | `main@f374d76` |
| `src/perception/FAST_LIO` | 保留的 FAST-LIO ROS 2 包，用于历史方案/对照调试 | [hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO) | `ROS2@a4743b0` |
| `src/control/px4_msgs` | PX4 uORB 的 ROS 2 消息定义，本地使用 PX4 1.14 对应分支 | [PX4/px4_msgs](https://github.com/PX4/px4_msgs) | `release/1.14@ffb6e80` |
| `src/planning/ego-planner-swarm` | EGO 规划器，支持局部 B-spline 轨迹优化与避障 | [ZJU-FAST-Lab/ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm) | `ros2_version@23a8d5a` |

`px4_msgs` 必须和飞控固件版本匹配。本项目当前按 PX4 `1.14` 使用；如果飞控升级，先同步检查 `src/control/px4_msgs` 分支和消息定义。

## Current Data Flow

定点悬停任务使用以下定位链路：

```text
MID-360
  -> livox_ros_driver2
  -> Point-LIO /odom
  -> pointlio_to_px4_visual_odom.py
  -> /fmu/in/vehicle_visual_odometry
  -> PX4 EKF2
  -> /fmu/out/vehicle_local_position
```

控制链路：

```text
/fmu/out/vehicle_local_position
  -> fixed_point_hover.py
  -> /fmu/in/offboard_control_mode
  -> /fmu/in/trajectory_setpoint
```

核心原则：PX4 只接受一个最终 Offboard 控制源。Point-LIO 只提供定位，实际控制由 `bringup_pointlio_hover` 中的控制脚本发布。

## Environment

当前工程按以下环境组织：

```text
Ubuntu 22.04
ROS 2 Humble
PX4 1.14
Livox MID-360
Micro XRCE-DDS Agent
```

进入工作空间根目录：

```bash
cd /home/wu/sim-ego/uav-mission-stack
source /opt/ros/humble/setup.bash
```

## Build

推荐先编译当前悬停实飞需要的包：

```bash
colcon build --packages-select \
  livox_ros_driver2 \
  point_lio \
  px4_msgs \
  hx_bringup_pointlio_hover \
  --symlink-install

source install/setup.bash
```

如果上游依赖已经构建过，只修改悬停 bringup 时可以只编译：

```bash
colcon build --packages-select hx_bringup_pointlio_hover --symlink-install
source install/setup.bash
```

## Hardware Bringup

常用实机启动项分终端执行。

启动 Livox MID-360 驱动：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

启动 PX4 Micro XRCE-DDS Agent：

```bash
sudo MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
```

如果 MID-360 使用有线网口，按现场网卡名设置，例如：

```bash
sudo ip addr add 192.168.1.50/24 dev enp2s0
sudo ip link set enp2s0 up
```

## Point-LIO Hover

低高度手动解锁测试：

```bash
source /opt/ros/humble/setup.bash
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

这个模式只验证 Point-LIO 到 PX4 EKF2 的视觉里程计融合，以及 PX4 Offboard 定点悬停。

`hover_auto_arm:=false` 表示 ROS 侧不会主动解锁，需要遥控器手动解锁。节点会持续发送 Offboard 心跳和定点 setpoint；检测到 PX4 已 armed 后，才会请求切入 Offboard。

悬停目标使用当前 PX4 local position 作为参考：

```text
target_x = current_px4_x
target_y = current_px4_y
target_z = current_px4_z - takeoff_altitude
```

PX4 local 通常是 NED，`z` 向下为正；所以 `current_px4_z - takeoff_altitude` 表示向上起飞。

更多参数见：

```text
src/up/bringup_pointlio_hover/README.md
```

## Preflight Checks

1. 确认 Point-LIO 有里程计：

```bash
ros2 topic hz /odom
ros2 topic echo /odom --once
```

2. 确认视觉里程计已送入 PX4：

```bash
ros2 topic hz /fmu/in/vehicle_visual_odometry
ros2 topic echo /fmu/in/vehicle_visual_odometry --once
```

launch 终端应周期性出现：

```text
visual odom published
```

3. 确认 PX4 EKF2 输出可控 local position：

```bash
ros2 topic echo /fmu/out/vehicle_local_position --once
```

必须看到：

```text
xy_valid: true
z_valid: true
```

4. 确认 Offboard 心跳和 setpoint 持续发布：

```bash
ros2 topic hz /fmu/in/offboard_control_mode
ros2 topic hz /fmu/in/trajectory_setpoint
```

5. 确认 PX4 状态：

```bash
ros2 topic echo /fmu/out/vehicle_status --once
```

`hover_auto_arm:=false` 时，前面全部正常后再遥控器手动解锁。

## EKF Fusion Checks

PX4 不会因为 `/fmu/in/vehicle_visual_odometry` 有数据就一定完成融合，最终以 `/fmu/out/vehicle_local_position` 为准。

基础通过条件：

```text
/fmu/in/vehicle_visual_odometry 有频率
/fmu/out/vehicle_local_position 中 xy_valid=true
/fmu/out/vehicle_local_position 中 z_valid=true
```

如果 PX4 固件发布 estimator aid source 话题，可以进一步检查视觉里程计 aid 状态：

```bash
ros2 topic list | grep estimator_aid_src
ros2 topic echo /fmu/out/estimator_aid_src_ev_pos --once
```

不同 PX4 版本的话题名可能不同；如果没有该话题，就以 `xy_valid/z_valid`、`px4_dds_monitor.py` 日志和 `px4_pointlio_position_compare.py` 的增量方向对比为准。

## Point-LIO / MID-360 Parameters

Point-LIO 配置默认使用：

```text
install/point_lio/share/point_lio/config/mid360_mapping.yaml
```

源码中的对应文件在：

```text
src/perception/Point-LIO/config/mid360_mapping.yaml
```

常见关注项：

```yaml
common:
  lid_topic: "/livox/lidar"
  imu_topic: "/livox/imu"

preprocess:
  lidar_type: 1
  scan_line: 4
  blind: 0.2

mapping:
  fov_degree: 360.0
  det_range: 100.0
  extrinsic_est_en: false
```

实机最重要的是外参：

```text
MID-360 -> body / IMU / PX4 body
```

如果外参错，常见现象是静止还好，一转弯地图就歪，PX4 local 方向和实际运动不一致。

## Debug Commands

查看 topic 是否连通：

```bash
ros2 topic info /odom -v
ros2 topic info /fmu/in/vehicle_visual_odometry -v
ros2 topic info /fmu/out/vehicle_local_position -v
ros2 topic info /fmu/in/offboard_control_mode -v
ros2 topic info /fmu/in/trajectory_setpoint -v
```

查看频率：

```bash
ros2 topic hz /odom
ros2 topic hz /fmu/in/vehicle_visual_odometry
ros2 topic hz /fmu/out/vehicle_local_position
ros2 topic hz /fmu/in/offboard_control_mode
ros2 topic hz /fmu/in/trajectory_setpoint
```

方向不确定时，启动 launch 时打开：

```text
use_position_compare:=true
```

看 `px4_pointlio_position_compare.py` 日志里的增量方向对比。

## Notes

- `driver/perception/control` 下的上游项目按源码目录保存，便于本仓库一次 clone 后直接构建。
- `src/up/bringup_pointlio_hover` 是当前上传版本的核心任务封装。
- 更换 PX4 固件版本时，先同步 `px4_msgs` 分支，再检查所有 `/fmu/in/*`、`/fmu/out/*` 消息字段是否兼容。
