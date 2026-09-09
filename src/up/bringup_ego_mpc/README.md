# `hx_bringup_ego_mpc`

这是一个独立的实机 EGO + MPC 启动包。它不依赖 `bringup_ego`、
`bringup_ego_multi_mission` 或其它自定义 bringup 包，也不会修改这些包。

## 控制职责

```text
MID-360 / Point-LIO
  -> /odom + /cloud_registered
  -> base_link(IMU) -> base(机体中心)补偿
  -> /ego/odom_base + raw /cloud_registered (Point-LIO z used directly)
  -> EGO 规划器 -> /ego/planning/bspline
  -> ego_mpc_controller -> /fmu/in/trajectory_setpoint
  -> PX4 姿态、角速度、电机内环
```

MPC 的状态为 PX4 NED 系的
`x = [pN, pE, pD, vN, vE, vD]`，输入为
`u = [aN, aE, aD]`。离散模型为：

```text
EGO B-spline + PX4 NED 位置/速度
  -> 带全时域硬约束的线性 MPC (OSQP)
  -> TrajectorySetpoint.acceleration
  -> PX4 姿态、角速度、电机内环
```

```text
p(k+1) = p(k) + v(k) dt + 0.5 a(k) dt^2
v(k+1) = v(k) + a(k) dt
```

每个控制周期使用 EGO B-spline 的未来 `mpc_horizon` 个采样点，同时惩罚位置误差、速度误差、加速度误差和 jerk。求解失败、轨迹过期、定位不健康时自动退回 PX4 位置保持模式。

OSQP 在整个预测时域约束速度、总/水平/垂向加速度和 jerk。由于标准 QP 只能表达线性约束，总量限制使用严格位于二范数球内部的 L1 多面体，水平加速度使用内接正多边形；这会在极限附近略微保守，但不会允许多面体顶点越过物理上限。QP 内部约束还会按 `mpc_solver_constraint_tolerance` 向内预留数值裕量，返回前再逐拍按真实二范数复核。

## PX4 输出约定

正常 MPC 控制时：

```text
OffboardControlMode.position     = false
OffboardControlMode.velocity     = false
OffboardControlMode.acceleration = true
TrajectorySetpoint.position      = [NaN, NaN, NaN]
TrajectorySetpoint.velocity      = [NaN, NaN, NaN]
TrajectorySetpoint.acceleration  = [aN, aE, aD]
```

因此本包不会接管姿态、角速度、推力或电机。起飞和安全保持阶段使用位置 setpoint；任何时候只有 `ego_mpc_controller.py` 发布 PX4 控制 setpoint。

## 坐标和杆臂补偿

Point-LIO/EGO 使用 ROS FLU：`x` 前、`y` 左、`z` 上；PX4 使用 NED：`x` 前、`y` 右、`z` 下。默认转换矩阵为：

```text
[ 1  0  0 ]
[ 0 -1  0 ]
[ 0  0 -1 ]
```

`base_link` 是 MID-360 IMU 原点，`base` 是机体中心。启动文件中的
`base_link_to_base_translation = [-0.011, -0.02329, -0.05588]` 与 Point-LIO 外参保持一致，并同时用于无人机中心里程计适配器、PX4 视觉里程计和静态 TF `base_link -> base`。机体中心在雷达正下方约 10 cm 的补偿已经包含在该参数组合中，修改时必须同步检查三个入口。

## 代码阅读入口

建议按下列数据流阅读核心代码：

```text
EGO Bspline 消息
  -> reference_sampler.py：De Boor 求值及 p_ref/v_ref/a_ref 解析求导
  -> ego_mpc_controller.py：参考轨迹对齐到 PX4 NED、状态机和安全门控
  -> constrained_mpc.py：凝聚预测模型、QP 代价、全时域硬约束和 OSQP 复核
  -> ego_mpc_controller.py：纯加速度 TrajectorySetpoint 发布
  -> px4_control_watchdog.py：控制模式、NaN 字段和话题独占检查
```

- `scripts/constrained_mpc.py`：MPC 的纯数值核心。重点阅读 `_system_matrices()`、`_prediction_matrices()`、`_cost_matrices()`、`_constraint_matrices()` 和 `solve()`。决策变量是绝对加速度序列 `U`；`a_ref` 是代价中的加速度跟踪目标，不是显式的 `a_ref + delta_a` 控制变量。
- `scripts/ego_mpc_controller.py`：ROS/PX4 控制主节点。重点阅读 `references_in_ned()`、`make_mpc_setpoint()` 和 `timer_callback()`，分别对应坐标转换、纯加速度输出约定和安全状态机。
- `scripts/reference_sampler.py`：把 EGO B-spline 消息转换为预测时域内的连续位置、速度和加速度参考。
- `scripts/px4_control_watchdog.py`：从控制器外部检查每个 PX4 输入话题只有一个发布者，并验证加速度模式下 position/velocity 为 NaN。
- `scripts/pointlio_to_px4_visual_odom.py`：执行 `base_link` 到 `base` 杆臂补偿、初始原点/航向对齐和 ROS 到 PX4 NED 转换，输出 PX4 外部视觉里程计。
- `scripts/rigid_transform.py`：无 ROS 依赖的四元数、向量旋转和刚体位姿复合工具，是两条杆臂补偿链路共用的数学实现。

配置入口是 `config/mpc_params.yaml`；`config/mpc_params_5ms.yaml` 是 5 m/s 参数模板。启动参数映射和各节点连接关系位于 `launch/ego_mpc_hw.launch.py`。

## 构建和启动

```bash
cd /home/wu/sim-ego/uav-mission-stack
source /opt/ros/humble/setup.bash
python3 -m pip install --user 'osqp>=0.6.3,<2'
colcon build --symlink-install --packages-select hx_bringup_ego_mpc
source install/setup.bash
ros2 launch hx_bringup_ego_mpc ego_mpc_hw.launch.py \
  output_enabled:=true \
  hardware_confirmation:=ENABLE_PX4_OUTPUT
```

MPC 和 EGO planner 的速度、加速度、jerk、预测时域和求解器参数统一放在
[`config/mpc_params.yaml`](/home/wu/sim-ego/uav-mission-stack/src/up/bringup_ego_mpc/config/mpc_params.yaml)。
启动时可以替换配置文件，例如使用 5 m/s 模板：

```bash
ros2 launch hx_bringup_ego_mpc ego_mpc_hw.launch.py \
  mpc_config:=/home/wu/sim-ego/uav-mission-stack/src/up/bringup_ego_mpc/config/mpc_params_5ms.yaml
```

`mpc_params_5ms.yaml` 只是参数模板，不代表已经满足实机飞行条件。修改
`ego_mpc_controller` 段时，必须同步检查 `ego_planner` 段的速度和加速度限制；
planner 生成的 B-spline 必须落在 MPC 的可行域内。

默认 `output_enabled=false`、`auto_arm=false`、`auto_offboard=false`，且不自动发布起始目标。实机启用前必须确认 PX4 参数、遥控器/安全开关和视觉里程计状态。建议先用默认关闭输出启动，检查：

```bash
ros2 topic info /fmu/in/trajectory_setpoint
  ros2 topic echo /ego/odom_base
ros2 topic echo /ego/planning/bspline
ros2 topic echo /ego_hw/diagnostics
ros2 topic echo /ego_mpc/solver_diagnostics
```

只允许看到一个 `/fmu/in/trajectory_setpoint` 发布者，并确认 MPC 日志显示 `output_acceleration_only=True`。watchdog 还必须看到实际的 `OffboardControlMode` 和 `TrajectorySetpoint` 持续到达；仅有 publisher 不代表已经在发送控制。

## 主要参数

`mpc_params.yaml` 中的 `control_rate_hz=50`、`mpc_dt=0.05`、`mpc_horizon=20` 对应 1 秒预测时域。`mpc_dt` 是预测采样周期：第一拍 jerk 使用实际控制周期 `1/control_rate_hz`，其余预测拍使用 `mpc_dt`。首次进入 MPC、轨迹恢复、定位恢复和 reset 恢复都会从零加速度重新约束 jerk。带有效起始时间的 B-spline 以“起始时间 + 轨迹时长 + `mpc_trajectory_timeout_sec`”判定过期，不会仅因消息接收时间超过超时值而提前失效。`mpc_px4_timeout_sec`、`mpc_reset_recovery_sec`、`mpc_max_eph` 和 `mpc_max_epv` 是状态安全门。

OSQP 默认使用 `solver_eps_abs=1e-4`、`solver_eps_rel=1e-4`、`solver_max_iter=4000`、`solver_time_limit_ms=18` 和 `solver_constraint_tolerance=5e-4`。求解时限必须严格小于 `1000/control_rate_hz`，否则节点拒绝启动。只有状态为 `solved`、在时限内且完整预测序列通过物理复核的结果才会发送；其他结果全部切换到位置保持。

PX4 的 `xy/z/vxy/vz/heading_reset_counter` 发生变化时，控制器会同步平移保存的参考点、起飞目标和保持点，并短暂回到位置保持模式，避免把 EKF 原点跳变误认为飞行器运动。

## 已知边界

当前 OSQP 已把速度、总/水平/垂向加速度和 jerk 约束应用到完整预测时域，不再依赖“无约束求解后只裁剪第一拍”。剩余边界是 Python/ROS 2/OSQP 组合不是硬实时系统：必须在实际飞控计算机上确认 `/ego_mpc/solver_diagnostics` 的最坏求解时间始终低于控制周期，并验证连续重规划时不会频繁触发 time-limit 保持。高动态飞行前仍建议迁移到 C++ OSQP 或 acados，并在 SITL、拆桨台架和系留低高度测试中验证 PX4 acceleration 模式。
