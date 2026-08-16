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

## 一键拉取与部署（实机，不含仿真）

本节只覆盖真实硬件链路：Ubuntu 22.04、ROS 2 Humble、PX4 1.14、Livox
MID-360、Point-LIO、EGO-Planner 和本仓库的实机 bringup。`src/simulation`
不参与下面的依赖安装、`rosdep` 扫描或 `colcon build`。

仓库内已经包含以下 ROS 2 源码，不需要再次单独 clone：

```text
src/driver/livox_ros_driver2
src/perception/Point-LIO
src/perception/FAST_LIO             # 可选，不是当前推荐定位链路
src/control/px4_msgs
src/planning/ego-planner-swarm/src/planner
src/up/*
```

PX4 固件不是这个 GitHub 仓库的子目录，需要与工作空间并列拉取。下面命令适合
在一台全新的 Ubuntu 22.04 + ROS 2 Humble 计算机上执行；已有工作空间时不要重复
clone，也不要用 `git checkout` 覆盖本地修改。

### 1. 拉取工作空间和 PX4 1.14.4

```bash
set -e

WORKSPACE_ROOT=/home/wu/sim-ego/uav-mission-stack
PX4_ROOT=/home/wu/sim-ego/PX4-Autopilot

mkdir -p /home/wu/sim-ego
if [ ! -d "$WORKSPACE_ROOT/.git" ]; then
  git clone https://github.com/Lumin-edu/uav-mission-stack.git "$WORKSPACE_ROOT"
fi

if [ ! -d "$PX4_ROOT/.git" ]; then
  git clone --branch v1.14.4 --recursive \
    https://github.com/PX4/PX4-Autopilot.git "$PX4_ROOT"
else
  echo "PX4 directory already exists: $PX4_ROOT"
  echo "Verify it is the v1.14.x firmware expected by src/control/px4_msgs."
fi

git -C "$PX4_ROOT" submodule update --init --recursive
```

`src/control/px4_msgs` 当前按 PX4 1.14 消息定义构建；飞控固件和该消息包不要跨
大版本混用。

### 2. 安装系统、ROS 和 PX4 依赖

ROS 2 Humble 本身需要先按官方文档安装；下面命令补齐本项目编译和实机通信所需的
工具。PX4 官方安装脚本使用 `--no-sim-tools`，不会为本项目安装或构建仿真工具。

```bash
set -e

source /opt/ros/humble/setup.bash

sudo apt-get update
sudo apt-get install -y \
  build-essential cmake git ninja-build pkg-config \
  python3-colcon-common-extensions python3-rosdep python3-vcstool \
  python3-pip python3-dev libeigen3-dev libpcl-dev libomp-dev libapr1-dev \
  ros-humble-pcl-conversions ros-humble-pcl-ros \
  ros-humble-rviz2 ros-humble-tf2-ros \
  ros-humble-micro-xrce-dds-agent

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update

PX4_ROOT=/home/wu/sim-ego/PX4-Autopilot
cd "$PX4_ROOT"
bash Tools/setup/ubuntu.sh --no-sim-tools
```

如果使用 USB 串口连接飞控，建议加入串口用户组并重新登录：

```bash
sudo usermod -aG dialout "$USER"
```

### 3. 安装仓库内 ROS 依赖

这里显式列出实机源码路径，因此不会扫描 `src/simulation`：

```bash
set -e

WORKSPACE_ROOT=/home/wu/sim-ego/uav-mission-stack
cd "$WORKSPACE_ROOT"
source /opt/ros/humble/setup.bash

rosdep install --from-paths \
  src/driver/livox_ros_driver2 \
  src/perception/Point-LIO \
  src/control/px4_msgs \
  src/planning/ego-planner-swarm/src/planner \
  src/up \
  --ignore-src --rosdistro humble -r -y
```

默认主链路不编译 `src/perception/FAST_LIO`。如果需要对照调试，再额外加入：

```bash
rosdep install --from-paths src/perception/FAST_LIO \
  --ignore-src --rosdistro humble -r -y
```

### 4. 编译实机 ROS 2 工作空间

`--base-paths` 是关键：它只让 colcon 发现实机目录，不会把 `src/simulation` 的
仿真包加入构建图。

```bash
set -e

WORKSPACE_ROOT=/home/wu/sim-ego/uav-mission-stack
cd "$WORKSPACE_ROOT"
source /opt/ros/humble/setup.bash

HARDWARE_BASE_PATHS=(
  src/driver/livox_ros_driver2
  src/perception/Point-LIO
  src/control/px4_msgs
  src/planning/ego-planner-swarm/src/planner
  src/up
)

colcon list --base-paths "${HARDWARE_BASE_PATHS[@]}"

colcon build --symlink-install \
  --base-paths "${HARDWARE_BASE_PATHS[@]}" \
  --packages-up-to \
    hx_bringup_pointlio_hover \
    hx_bringup_square_mission \
    hx_bringup_full_mission \
    hx_bringup_ego \
    hx_bringup_ego_multi_mission \
    hx_bringup_ego_mpc \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

如果只想验证当前推荐的 Point-LIO + EGO 主链路，可以缩小构建范围：

```bash
colcon build --symlink-install \
  --base-paths \
    src/driver/livox_ros_driver2 \
    src/perception/Point-LIO \
    src/control/px4_msgs \
    src/planning/ego-planner-swarm/src/planner \
    src/up \
  --packages-up-to livox_ros_driver2 point_lio hx_bringup_ego \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 5. 编译 PX4 固件

PX4 的目标板必须按实际飞控型号选择。下面以常见的 FMUv6X 为例；如果是
FMUv5、FMUv6C 或其他板型，只替换 `PX4_BOARD`，不要把不同板型固件直接刷写。

```bash
set -e

PX4_ROOT=/home/wu/sim-ego/PX4-Autopilot
PX4_BOARD=px4_fmu-v6x_default
cd "$PX4_ROOT"

git describe --tags --always
make "$PX4_BOARD"
```

连接飞控并确认 DFU/串口权限后，才执行刷写：

```bash
cd /home/wu/sim-ego/PX4-Autopilot
make px4_fmu-v6x_default upload
```

固件刷写命令和 ROS 2 工作空间编译相互独立；刷写前先确认飞控型号、固件目标和
串口设备，避免误刷。

## Build

已经完成上述一键部署后，日常只修改 ROS 代码时使用以下增量编译命令。它仍然
不扫描 `src/simulation`：

```bash
cd /home/wu/sim-ego/uav-mission-stack
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --base-paths \
    src/driver/livox_ros_driver2 \
    src/perception/Point-LIO \
    src/control/px4_msgs \
    src/planning/ego-planner-swarm/src/planner \
    src/up \
  --packages-up-to \
  livox_ros_driver2 \
  point_lio \
  px4_msgs \
  ego_planner \
  hx_bringup_pointlio_hover \
  hx_bringup_square_mission \
  hx_bringup_full_mission \
  hx_bringup_ego \
  hx_bringup_ego_multi_mission \
  hx_bringup_ego_mpc \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

如果上游依赖已经构建过，只修改悬停 bringup 时可以只编译：

```bash
colcon build --symlink-install \
  --base-paths \
    src/driver/livox_ros_driver2 \
    src/control/px4_msgs \
    src/perception/Point-LIO \
    src/up \
  --packages-up-to livox_ros_driver2 hx_bringup_pointlio_hover \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
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
配置
  sudo nmcli connection add \
    type ethernet \
    ifname enp2s0 \
    con-name enp2s0-static \
    ipv4.method manual \
    ipv4.addresses 192.168.1.50/24 \
    ipv4.gateway "" \
    ipv4.never-default yes \
    ipv4.route-metric 1000 \
    ipv6.method disabled \
    connection.autoconnect yes
启用
  sudo nmcli connection up enp2s0-static
检查
  ip -4 addr show enp2s0
  ip route
  nmcli connection show enp2s0-static
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

## EGO 实机部署

先启动 Micro XRCE-DDS Agent，再启动 EGO bringup。第一次建议保持
`output_enabled:=false`，只检查 MID-360、Point-LIO、融合里程计、点云和 EGO
轨迹；确认诊断正常后，再打开 PX4 输出安全门。

### EGO 规划链路检查（不向 PX4 输出）

```bash
cd /home/wu/sim-ego/uav-mission-stack
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch hx_bringup_ego ego_avoidance_hw.launch.py \
  use_livox_driver:=true \
  use_pointlio:=true \
  use_pointlio_px4_visual_odom:=true \
  use_px4_monitor:=true \
  use_px4_control_watchdog:=true \
  use_position_compare:=true \
  require_rangefinder_height:=true \
  output_enabled:=false \
  auto_land_after_goal:=false
```

该启动文件会连接以下实机链路：

```text
MID-360 /livox/lidar + /livox/imu
  -> Point-LIO /odom + /cloud_registered
  -> base_link(IMU) -> base(机体中心)补偿
  -> /ego/odom_fused + /ego/cloud_registered_fused
  -> EGO /ego/position_cmd
```

### EGO 控制输出（确认安全后）

确认 `/ego/odom_fused`、`/ego/cloud_registered_fused`、PX4
`xy_valid/z_valid` 和监控输出都正常后，再使用双重安全门：

```bash
ros2 launch hx_bringup_ego ego_avoidance_hw.launch.py \
  use_livox_driver:=true \
  use_pointlio:=true \
  use_pointlio_px4_visual_odom:=true \
  use_px4_monitor:=true \
  use_px4_control_watchdog:=true \
  require_rangefinder_height:=true \
  takeoff_before_ego:=true \
  output_enabled:=true \
  hardware_confirmation:=ENABLE_PX4_OUTPUT \
  auto_arm:=false \
  auto_offboard:=true \
  auto_land_after_goal:=false
```

`auto_arm:=false` 时仍由遥控器执行解锁；`auto_land_after_goal` 默认关闭，只有
完成单独的落地测试后才建议改为 `true`。EGO 的目标、坐标系、杆臂补偿和自动
降落逻辑见 [src/up/bringup_ego/README.md](src/up/bringup_ego/README.md)。

其他实机入口：

```bash
# 正方形航点
ros2 launch hx_bringup_square_mission square_mission_hw.launch.py

# 完整航点任务
ros2 launch hx_bringup_full_mission full_mission_hw.launch.py

# EGO 多任务点
ros2 launch hx_bringup_ego_multi_mission ego_multi_mission_hw.launch.py

# EGO MPC 外环（独立包）
ros2 launch hx_bringup_ego_mpc ego_mpc_hw.launch.py
```

`bringup_ego` 和 `bringup_ego_mpc` 使用 `output_enabled` 与
`hardware_confirmation` 安全门；`bringup_square_mission`、
`bringup_full_mission` 和多任务包使用各自控制器的参数。执行任何会向 PX4
发布 setpoint 的启动文件前，都要先阅读对应 README，并用 `--show-args` 核对
默认值和输出开关。

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
