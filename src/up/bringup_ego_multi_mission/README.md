# hx_bringup_ego_multi_mission

实机 EGO 多任务点启动包。它将已经验证过的单任务实机节点、配置和坐标
补偿逻辑复制到本包内独立维护，不依赖 `hx_bringup_ego` 或
`hx_bringup_pointlio_hover`，也不修改 Point-LIO 或 EGO 源码。

## 数据链路

```text
MID-360 -> Point-LIO /odom + /cloud_registered
         -> base_link(IMU) 到 base(机体中心)补偿
         -> PX4 视觉里程计
         -> /ego/odom_base + raw /cloud_registered (Point-LIO z used directly)
         -> EGO 局部规划
         -> /ego/position_cmd -> PX4 TrajectorySetpoint
```

本包不启动 Gazebo、仿真传感器、`scenario_map` 或静态 map，也不等待静态
occupancy。EGO 仍然只使用实机 Point-LIO 的实时障碍点云，和已经验证的单
任务实机模式一致。

本包不启动单目标 `startup_goal.py`，由 `multi_waypoint_runner.py` 独占
`/move_base_simple/goal`，每次只发布一个目标；只有当前目标位置、速度在
容差内并稳定指定时间后才发布下一个目标。
任意航点超时或里程计失效都会停止路线，不会跳过航点。

## 航点格式

默认文件为 `config/multi_waypoints.yaml`，坐标使用任务坐标系：

```text
x = 机体右侧，y = 机头前方，z = 向上
```

程序发布给 EGO 前转换为 Point-LIO ROS FLU：

```text
ros_x = task_y
ros_y = -task_x
ros_z = task_z
ros_yaw = -task_yaw
```

`z` 与单任务模式一致，表示相对于起飞时捕获的 Point-LIO/PX4 原点的高度。
请先在无桨环境检查航点、方向、限速和每个航点的超时设置，再进行飞行。

## 构建

```bash
cd /home/wu/sim-ego/uav-mission-stack
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select hx_bringup_ego_multi_mission
source install/setup.bash
```

## 实机启动

默认 `output_enabled:=false`、`auto_arm:=false`，用于先检查链路：

```bash
ros2 launch hx_bringup_ego_multi_mission ego_multi_mission_hw.launch.py \
  use_livox_driver:=true \
  use_pointlio:=true \
  use_pointlio_px4_visual_odom:=true \
  output_enabled:=false \
  auto_arm:=false \
  start_with_waypoints:=true
```

确认 `/odom`、`/ego/odom_base`、`/cloud_registered` 和 `/ego/position_cmd` 正常后，经过无桨检查再
开启 PX4 输出：

```bash
ros2 launch hx_bringup_ego_multi_mission ego_multi_mission_hw.launch.py \
  output_enabled:=true \
  hardware_confirmation:=ENABLE_PX4_OUTPUT \
  auto_arm:=false \
  auto_offboard:=true \
  route_file:=/path/to/your/route.yaml
```

可通过启动参数调整 `route_reach_xy`、`route_reach_z`、
`route_settle_sec`、`route_waypoint_timeout_sec` 以及速度限制。修改航点时
请复制并编辑本包的 YAML，而不是依赖仿真目录中的文件。
