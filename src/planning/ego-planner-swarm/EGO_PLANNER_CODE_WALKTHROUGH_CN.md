# EGO-Planner-Swarm 核心代码导读

本文对应当前仓库中的 ROS 2 实现，不是只按论文或原版 ROS 1 代码推导。阅读重点是单机局部避障闭环，同时说明多机分支和本仓库增加的静态地图、目标合法性检查等行为。

核心文件如下：

- `src/planner/plan_manage/src/ego_replan_fsm.cpp`：目标接收、FSM、周期重规划、安全检查、轨迹发布。
- `src/planner/plan_manage/src/planner_manager.cpp`：全局参考轨迹、局部初值、rebound 优化、时间重分配。
- `src/planner/plan_env/src/grid_map.cpp`：深度/点云进入占据地图、局部更新、膨胀与超时检测。
- `src/planner/plan_env/include/plan_env/grid_map.h`：地图缓存和碰撞查询 API。
- `src/planner/path_searching/src/dyn_a_star.cpp`：为碰撞段寻找绕障拓扑路径。
- `src/planner/bspline_opt/src/uniform_bspline.cpp`：B-spline 求值、求导、参数化和动力学可行性检查。
- `src/planner/bspline_opt/src/bspline_optimizer.cpp`：ESDF-free 障碍约束生成、代价函数及 L-BFGS 优化。
- `src/planner/plan_manage/src/traj_server.cpp`：B-spline 采样、yaw 生成和 `PositionCommand` 发布。
- `src/planner/plan_manage/launch/advanced_param.launch.py`：仿真默认参数和主要话题 remap。

## 1. 先建立整体图景

### 1.1 节点和数据流

```text
目标 /move_base_simple/goal 或预设 waypoint
                    |
                    v
              EGOReplanFSM (100 Hz)
                    |
       无避障全局多项式参考轨迹
                    |
          选择 planning_horizon 内局部目标
                    |
      旧轨迹/多项式采样 -> 三次 B-spline 初值
                    |
点云/深度 -> GridMap -> 膨胀占据查询
                    |
       A* 绕障几何 -> base_point/direction
                    |
  L-BFGS: 平滑 + 碰撞 + 动力学 + 终点 (+ 多机)
                    |
        planning/bspline (整条位置 B-spline)
                    |
              traj_server (100 Hz)
                    |
      position + velocity + acceleration + yaw
                    |
       PositionCommand -> controller / PX4 bridge
```

仿真 launch 中至少有两个规划侧节点：

| 节点 | 职责 | 主要输入 | 主要输出 |
|---|---|---|---|
| `ego_planner_node` | 地图、FSM、规划、优化 | odom、depth/cloud、goal、其他无人机轨迹 | `planning/bspline` |
| `traj_server` | 按当前时间采样轨迹并生成 yaw | `planning/bspline` | `PositionCommand` |

`ego_planner_node.cpp` 只创建一个普通 `rclcpp::Node`，再调用 `EGOReplanFSM::init()`。地图和优化器不是独立 ROS 节点，而是由 `EGOPlannerManager::initPlanModules()` 在同一节点内创建的对象。

### 1.2 三层轨迹不要混淆

代码里同时存在三种“路径/轨迹”：

1. **全局多项式参考轨迹**：`GlobalTrajData::global_traj_`。从当前位置到最终目标，负责提供前进方向和局部目标，不查询障碍物。
2. **局部 B-spline 轨迹**：`LocalTrajData::position_traj_`。这是实际优化、发布和执行的轨迹。
3. **A* 路径**：只在初值穿过障碍物时，为碰撞控制点生成绕障方向；它不是最终发布轨迹，也不是常驻全局规划器。

一句话概括当前架构：**全局多项式管“往哪里走”，局部 B-spline 管“怎样安全地飞过去”，A* 只给局部优化器提供绕障几何提示。**

## 2. FSM：什么时候规划、重规划和停止

### 2.1 状态定义

`EGOReplanFSM::FSM_EXEC_STATE` 定义七个状态：

```text
INIT -> WAIT_TARGET
          | planNextWaypoint() 成功
          +-> GEN_NEW_TRAJ -> EXEC_TRAJ

WAIT_TARGET -> SEQUENTIAL_START -> EXEC_TRAJ
               (代码保留，但当前目标入口通常不会走到)

EXEC_TRAJ -> REPLAN_TRAJ -> EXEC_TRAJ
    |
    +-> EMERGENCY_STOP -> GEN_NEW_TRAJ
                         (仅 fail-safe 允许恢复时)
```

`exec_timer_` 每 10 ms 调用 `execFSMCallback()`，即名义 100 Hz。`safety_timer_` 每 50 ms 调用 `checkCollisionCallback()`，即名义 20 Hz。

注意：回调开头会 cancel FSM timer，结束前再 reset，目的是避免一次规划尚未完成时重复进入同一回调。因此实际 FSM 周期是“回调执行时间 + 10 ms”，重优化期间不会保持严格 100 Hz。

更重要的是，`ego_planner_node` 使用 `rclcpp::spin(node)` 的单线程 executor。FSM、safety timer、地图更新、odom 和 cloud 回调并不并发；同步执行的 L-BFGS/A* 规划会暂时阻塞安全检查和新传感器数据处理。因此这里的 “safety” 是另一个定时回调，不是独立实时 watchdog。

### 2.2 起点从哪里来

里程计订阅为 `odom_world`，launch 通常 remap 到每架无人机的 odometry。回调更新：

```text
odom_pos_    <- odom.pose.pose.position
odom_vel_    <- odom.twist.twist.linear
odom_orient_ <- odom.pose.pose.orientation
```

首次局部规划 `planFromGlobalTraj()` 的边界状态为：

```text
start_pt_  = odom_pos_
start_vel_ = odom_vel_
start_acc_ = 0
```

所以首次规划的位置和速度来自实时里程计，加速度没有估计，直接置零。

周期重规划 `planFromCurrentTraj()` 则不是直接使用最新 odom，而是在旧局部轨迹的当前时间 `t_cur` 上采样：

```text
start_pt_  = old_position_traj(t_cur)
start_vel_ = old_velocity_traj(t_cur)
start_acc_ = old_acceleration_traj(t_cur)
```

这样做的意图是让新旧轨迹在切换时保持位置、速度、加速度连续。不过后续 `parameterizeToBspline()` 用超定方程做最小二乘拟合，边界状态不是逐项精确满足；优化阶段固定的是拟合所得前三个控制点。因此这里只能说近似接续旧轨迹，不能把它理解为严格的 `p/v/a` 连续性保证。另一个代价是：如果飞行器已经明显偏离参考轨迹，新规划的数学起点仍在旧轨迹上，真实状态误差要由控制器吸收；当前 FSM 没有基于 odom-track error 的显式重规划条件。

### 2.3 目标从哪里来

`fsm/flight_type` 支持：

| 值 | 类型 | 来源 |
|---|---|---|
| `1` | `MANUAL_TARGET` | `/move_base_simple/goal` |
| `2` | `PRESET_TARGET` | launch 中 `fsm/waypointN_{x,y,z}`；实机触发模式下等 `/traj_start_trigger` |
| `3` | `REFENCE_PATH` | 枚举存在，但当前 `init()` 没有实现对应订阅/分支 |

手动目标只使用消息中的位置。若 `z <= 0.05`，使用 `fsm/manual_goal_z`；`z < -0.1` 直接忽略。可用以下开关拒绝非法目标：

- `fsm/validate_goal_occupancy=true`：调用 `getInflateOccupancy(end_wp)`。
- `fsm/require_occupancy_for_goal=true`：在上述检查前还要求 `hasOccupancyObservation()` 为真。

默认仿真 launch 没有打开这两个检查，所以默认情况下目标可落在障碍物或地图外，之后可能表现为局部规划持续失败。

这里的“已有观测”接口有明显边界：`hasOccupancyObservation()` 在 `use_static_map=true` 时只看 `has_static_map_`，否则只看独立点云的 `has_cloud_`，完全不读取深度融合的 `has_first_depth_`。因此 depth-only 配置即使已经形成有效 log-odds 地图，打开 `require_occupancy_for_goal` 后仍会持续拒绝目标；静态地图模式下，它也只以静态图是否到达为准。这更像特定数据入口的到达 flag，不是通用的“地图有效”判据。

### 2.4 全局目标如何变成局部目标

`planNextWaypoint()` 调用 `planGlobalTraj()`，生成从当前 odom 状态到目标的 minimum-snap/单段多项式参考。长直线每约 4 m 插入中间点，但这些点仍在起终点直线上，整个过程不查地图。

每次局部规划前，`getLocalTarget()` 从 `global_data_.last_progress_time_` 沿全局参考前向采样：

- 步长 `t_step = planning_horizon / 20 / max_vel`；
- 找到与当前局部起点距离首次达到 `planning_horizon` 的参考点；
- 若全局目标已经在 horizon 内，就直接选最终目标；
- 距终点小于制动距离 `v_max^2 / (2 a_max)` 时，局部目标速度置零，否则继承全局参考速度。

因此 `planning_horizon` 同时影响局部问题的空间长度和重规划滚动窗口。

### 2.5 首次规划什么时候开始

启动后先由 `INIT` 等第一帧 odom，再进入 `WAIT_TARGET`。手动目标回调和预设航点的 `readGivenWps()` 最终都会调用 `planNextWaypoint()`。全局参考生成成功时，该函数若看到当前状态是 `WAIT_TARGET`，会立刻切到 `GEN_NEW_TRAJ`。该状态每个 FSM tick 最多连续调用十次 `planFromGlobalTraj()`，成功才进入 `EXEC_TRAJ`。

`WAIT_TARGET` 分支本身还写有 `WAIT_TARGET -> SEQUENTIAL_START`，并在后者对 `drone_id>=1` 等待前一架无人机轨迹。但当前代码中只有 `planNextWaypoint()` 会把 `have_target_` 设为 true，而它同时已经把 `WAIT_TARGET` 改成 `GEN_NEW_TRAJ`。因此按现有入口，`SEQUENTIAL_START` 通常不可达，多机“依次起飞”的门控并未真正生效。若以后恢复这一设计，应让目标接收只置 flag，由 FSM 统一决定进入哪个首次规划状态。

`fsm/realworld_experiment=false` 时 `have_trigger_` 初始为 true。设为 true 时，只有 `PRESET_TARGET` 分支创建 `/traj_start_trigger` 订阅并在初始化期间等待它；`MANUAL_TARGET` 的 goal callback 会直接切到 `GEN_NEW_TRAJ`，实际绕过 `WAIT_TARGET` 中的 trigger 检查。

### 2.6 什么情况下触发 replan

当前实现有四类触发源。

**1. 固定时间滚动重规划**

在 `EXEC_TRAJ` 中，只要当前局部目标还不是最终目标，并且：

```text
t_cur > fsm/thresh_replan_time
```

就进入 `REPLAN_TRAJ`。默认仿真值为 1.0 s。这是主要的滚动优化节拍，不是“执行到轨迹百分之多少”触发，而是从当前局部轨迹 `start_time_` 起经过固定秒数触发。

**2. 最后一段仍离终点较远**

当局部目标已经等于最终目标时，如果：

```text
distance(end_pt_, planned_pos(t_cur)) > fsm/thresh_no_replan_meter
and t_cur > fsm/thresh_replan_time
```

仍会重规划。进入终点邻域后，则允许剩余轨迹执行完，不再周期重规划。

**3. 安全定时回调发现静态障碍或其他无人机冲突**

安全定时回调从 `t_cur` 起以 0.01 s 步长采样未来轨迹。若当前时间还在轨迹前 2/3，只检查到 2/3 分界；进入最后 1/3 后则检查至轨迹末尾。每个样本调用 `getInflateOccupancy()`。

静态占据确实随未来采样 `t` 查询；但当前 safety callback 的 swarm 分支在循环外计算自己的 `p_cur`，对方位置也始终按当前绝对时间预测，循环内没有随 `t` 推进。因此这一分支实际检查的是当前相对距离，并非完整的未来 pairwise trajectory collision。未来轨迹交叉主要由下一条所述的广播回调检查。

发现冲突后先立即调用一次 `planFromCurrentTraj()`：

- 成功：直接替换轨迹，保持 `EXEC_TRAJ`；
- 失败且预计碰撞时间 `< emergency_time`：进入 `EMERGENCY_STOP`；
- 失败但还有更多时间：进入 `REPLAN_TRAJ`，持续尝试。

**4. 收到另一架无人机的新轨迹**

`BroadcastBsplineCallback()` 存入对方 B-spline 后，`EGOPlannerManager::checkCollision()` 以 0.03 s 步长检查双方时间重叠段。若距离小于 `swarm_clearance`，切到 `REPLAN_TRAJ`。

### 2.7 重规划从旧轨迹的哪一段开始

触发时刻计算为：

```text
t_cur = now - old_local_traj.start_time
```

新规划的边界状态取旧轨迹的 `p(t_cur), v(t_cur), a(t_cur)`。构造初值时优先复用旧轨迹从 `t_cur` 到旧轨迹末尾的部分，再从旧末尾接一段到新局部目标的多项式，并按近似弧长重采样。

因此不是把整条轨迹推倒重来，也不是等旧轨迹执行到某个控制点；它在“当前绝对时间对应的旧轨迹状态”处切入，再保留尚未执行的几何趋势。

当前实现没有补偿本轮规划耗时：`start_pt/vel/acc` 在进入优化前按一次 `now` 采样，优化完成后又用新的 `now` 作为新轨迹 `start_time`。若一次优化耗时为 `T_plan`，新轨迹零时刻仍对应较早采样的状态，而真实系统已经沿旧轨迹前进约 `T_plan`。低速时影响小，高速或优化偶发变慢时可能造成切换误差。

### 2.8 规划失败后怎么处理

失败分多层回退：

1. 重规划先尝试“复用旧轨迹”初值：`callReboundReplan(false, false)`。
2. 失败后改用新的单段多项式初值：`callReboundReplan(true, false)`。
3. 再失败则加入随机中间点生成另一拓扑初值：`callReboundReplan(true, true)`。
4. FSM 首次规划每个 tick 最多试十次；失败后留在 `SEQUENTIAL_START` 或 `GEN_NEW_TRAJ`。
5. 普通 `REPLAN_TRAJ` 每个 tick 走上述回退链一次；失败则留在 `REPLAN_TRAJ`，旧轨迹仍由 `traj_server` 继续采样。
6. 安全定时回调发现迫近碰撞且抢救式重规划失败，才进入急停。

优化器内部还有两种重启：

- 优化后仍碰撞：重新生成 `base_point/direction`，碰撞权重乘 2，最多重启三次；
- 优化过程中发现新障碍：让 L-BFGS 提前退出，补充新约束后 rebound，最多 20 次。

### 2.9 急停和任务结束

`EmergencyStop(stop_pos)` 本身会创建六个完全相同的控制点，得到位置恒定、速度和加速度为零的三次 B-spline，并由 `callEmergencyStop()` 发布给 `traj_server`。但进入 `EMERGENCY_STOP` 不等于这个函数一定会被调用：状态分支只有在 `flag_escape_emergency_` 为 true 时才发布静止轨迹，随后立即把该 flag 清零。

两种急停后的行为不同：

- 普通迫近碰撞：安全回调只切换状态，没有同时把 `flag_escape_emergency_` 置 true。当前正常入口经 `GEN_NEW_TRAJ` 成功启动后，该 flag 已被置 true，首次急停会发布静止轨迹；但保留的 `SEQUENTIAL_START` 成功分支没有设置它，若将来修复前述状态可达性并启用顺序起飞，首次普通急停可能不发布静止轨迹，而是继续执行旧轨迹。之后只有在 odom 速度已经低于 0.1 m/s 且 `fsm/fail_safe=true` 时才会转到 `GEN_NEW_TRAJ`。这是潜伏在该分支中的安全缺陷，恢复它之前必须修正。
- 深度/里程计融合超时：安全定时回调显式把 `flag_escape_emergency_` 置 true、把 `enable_fail_safe_` 置 false，因此会发布一次静止轨迹并保持停止，需外部重启任务。

任务结束的判定使用**规划轨迹时间**，不是 odom 到目标距离：当局部目标等于最终目标，且 `t_cur > duration - 0.01`，清除目标并回到 `WAIT_TARGET`。`traj_server` 此后仍持续发布终点悬停命令。

## 3. 地图与障碍物接口

### 3.1 地图里真正存了什么

`GridMap` 维护三个主要体素缓存：

| 缓存 | 类型 | 含义 |
|---|---|---|
| `occupancy_buffer_` | `double` | 深度 raycast 的 log-odds 占据概率 |
| `occupancy_buffer_inflate_` | `char` | 当前局部深度/点云的膨胀占据层 |
| `static_occupancy_buffer_inflate_` | `char` | 本仓库增加的静态累计点云膨胀层 |

地图原点为：

```text
(-map_size_x/2, -map_size_y/2, ground_height)
```

它是固定大小的 dense 3D array，不是动态哈希地图，也没有 ESDF 缓存。

### 3.2 深度图怎样进入地图

根据 `grid_map/pose_type`，深度图与 `PoseStamped` 或 `Odometry` 近似同步。处理链为：

```text
depth + pose
  -> projectDepthImage()
  -> 相机内参反投影到世界系点
  -> raycastProcess()
  -> 光线终点记 hit，沿途体素记 miss
  -> log-odds 累积并 clamp 到 [p_min, p_max]
  -> clearAndInflateLocalMap()
```

超过 `max_ray_length` 或地图边界的端点按 free/miss 处理；真实量测端点按 hit 处理。`p_occ` 转成 logit 后作为原始占据阈值。

`updateOccupancyCallback()` 每 0.05 s 处理一次待更新深度，即地图融合名义 20 Hz。开启深度融合后，若超过 `grid_map/odom_depth_timeout` 没有新同步数据，会设置超时标志，由 FSM 安全定时回调触发永久急停。

### 3.3 独立点云怎样进入地图

`grid_map/cloud` 的 `PointCloud2` 走 `cloudCallback()`，假设点已经在地图/world 坐标系：

1. 必须先收到 `grid_map/odom`，用于确定局部更新中心。
2. 清空 `camera_pos +/- local_update_range` 内旧的动态膨胀缓存。
3. 仅接收局部范围内的点。
4. 每个点直接写入 `occupancy_buffer_inflate_`，不更新 `occupancy_buffer_`。
5. x/y 膨胀 `ceil(obstacles_inflation/resolution)` 个 voxel，z 固定膨胀一层。

这条路径不做 raycast，所以看不到 free-space 概率，也没有时间融合；每帧点云相当于重建一个局部二值障碍层。输入点云必须已完成坐标变换，否则障碍会被写到错误位置。

还有两个状态边界：`cloudCallback()` 在检查 odom 之前就把 `has_cloud_` 设为 true。如果点云先于 odom 到达，`hasOccupancyObservation()` 可能返回 true，但这一帧并未写入障碍。反过来，深度图即使已经融合，该函数也不检查 `has_first_depth_`，在没有独立点云时仍返回 false。因此对目标合法性做严格门控时，不能把该接口当作“地图中一定已有有效 voxel”的充分必要条件。

### 3.4 静态累计点云

当 `grid_map/use_static_map=true` 时，节点以 transient-local QoS 订阅 `grid_map/static_cloud`。收到点云后清空并重建 `static_occupancy_buffer_inflate_`，使用 `grid_map/static_map_inflation` 做各向同性立方体膨胀。

当前 `getInflateOccupancy()` 对动态和静态膨胀层做 OR，因此规划器、A*、目标验证和安全检查会自动同时看到两层。

### 3.5 障碍物怎样膨胀

深度 log-odds 路径使用：

```text
inf_step = ceil(obstacles_inflation / resolution)
```

然后 `inflatePoint()` 枚举 `[-inf_step, inf_step]^3`，实际形状是轴对齐立方体，不是欧氏球体。

独立点云路径的 x/y 同上，但 z 方向写死为 `[-1, 1]` voxel。两条通路的垂直膨胀规则不一致，语义地图扩展时必须先决定是否保留这种差异。

虚拟天花板也直接写入动态膨胀占据层，因而对所有碰撞查询都等同真实障碍。

### 3.6 查询某位置是否碰撞

主接口是：

```cpp
int GridMap::getInflateOccupancy(Eigen::Vector3d pos)
```

返回值语义：

- `1`：动态或静态膨胀层占据；
- `0`：地图内且膨胀层不占据；
- `-1`：地图外。

不少调用点直接把它转成 `bool`，所以地图外的 `-1` 也会被视为碰撞。这对限制规划范围是合理的，但写新逻辑时不要把它误解成纯布尔接口。

`getOccupancy()` 查询的是未膨胀 log-odds 层；实际规划、A* 和安全检查主要使用 `getInflateOccupancy()`。

### 3.7 未知空间怎样处理

深度地图初始化为略低于 `clamp_min_log` 的 unknown 值，但 `getInflateOccupancy()` 不检查 unknown，只检查两个膨胀二值层。因此地图内未知体素默认当作可通行；只有地图外返回 `-1` 并被当作碰撞。

`isUnknown()`、`isKnownFree()` 等接口存在，但当前主优化和安全链路没有用它们。这是做保守探索或语义未知区代价时的重要接入口。

## 4. 为什么不建 ESDF 仍然有碰撞梯度

### 4.1 不是从占据栅格直接求梯度

当前 `GridMap` 没有距离场，也没有“查询距离及梯度”的 API。`bspline_optimizer.cpp` 文件头仍有“通过 ESDF 计算碰撞梯度”的遗留注释，但实际代码不是这样做的。

EGO 的方法是先把离散占据关系转成每个控制点的几何约束：

```text
控制点/相邻控制点连线穿过膨胀障碍
             |
             v
提取进入/离开障碍的控制点片段 [in_id, out_id]
             |
             v
A* 在 free voxel 中从入口绕到出口
             |
             v
对每个碰撞控制点，找 A* 路径与局部法平面的交点
             |
             v
沿“控制点 -> 交点”方向回查障碍边界
             |
             v
保存 base_point 和归一化 direction
```

数据保存在 `ControlPoints`：

```cpp
base_point[i][j]  // 障碍边界附近基点
direction[i][j]   // 从控制点指向绕障侧的单位方向
```

### 4.2 碰撞距离和解析梯度

对控制点 `Q_i`、基点 `P_ij` 和单位方向 `v_ij`：

```text
d_ij = (Q_i - P_ij)^T v_ij
e_ij = clearance - d_ij
```

`e_ij <= 0` 时不惩罚；安全距离不足时使用分段三次/二次代价。靠近边界的一段为：

```text
f_collision = e_ij^3
df/dQ_i = -3 e_ij^2 v_ij
```

距离误差更大时切为连续的二次形式，避免三次代价增长过猛。

所以梯度来自**A* 绕障路径构造出来的方向向量**，而不是 ESDF 的空间梯度。占据地图只回答 0/1，A* 和障碍边界回查负责把 0/1 变成可微的局部推动方向。

### 4.3 rebound 的含义

优化进行几轮且轨迹足够平滑后，`calcDistanceCostRebound()` 会再次检查控制点是否撞上新障碍。若发现原约束未覆盖的新碰撞：

1. 为新碰撞段再跑 A*；
2. 追加新的 `base_point/direction`；
3. 令 L-BFGS 提前退出；
4. 用更新后的碰撞几何重新优化。

这就是 “rebound”：不是一次固定代价的优化，而是边优化、边发现新接触面、边更新推离方向。

## 5. 轨迹表示

### 5.1 优化变量是什么

局部位置轨迹是三次 B-spline，优化变量是三维控制点 `Q_i`，矩阵形状为 `3 x N`。不是优化 polynomial coefficient，也不直接优化每个采样时刻的位置。

在 rebound 阶段：

```text
start_id = order = 3
end_id   = number_of_control_points
```

即前三个控制点固定，其余控制点全部进入 L-BFGS，包括尾部。因此：

- 参数化所得前三个控制点在优化中硬保持，所以参数化轨迹自身的起始状态不会再被 L-BFGS 改动；
- 局部终点不是硬固定，而是通过 `calcTerminalCost()` 软拉向 `local_target_pt_`。

在 refine 阶段，变量范围是 `[order, N-order)`，首尾各三个控制点固定。

### 5.2 位置、速度和加速度怎样计算

`evaluateDeBoorT(t)` 把从零开始的轨迹时间转换到 knot domain，再用 de Boor 算法求位置。

B-spline 的导数仍是 B-spline，导数控制点为：

```text
Q_i^(1) = p (Q_{i+1} - Q_i) / (u_{i+p+1} - u_{i+1})
```

代码依次构造：

```text
position_traj
velocity_traj     = position_traj.getDerivative()
acceleration_traj = velocity_traj.getDerivative()
```

规划器和 `traj_server` 都用同一套 `UniformBspline` 求值，避免发布端另做数值差分。

### 5.3 当前状态怎样变成控制点边界

`parameterizeToBspline()` 接收：

- 一串希望 B-spline 插值的 `point_set`；
- 起点/终点速度；
- 起点/终点加速度；
- knot interval `ts`。

它建立一个 `(K+4) x (K+2)` 的超定线性方程 `A Q = b`：位置行使用三次均匀 B-spline 的 `[1,4,1]/6`，速度行使用 `[-1,0,1]/(2 ts)`，加速度行使用 `[1,-2,1]/ts^2`，再用列主元 QR 最小二乘求出 `K+2` 个控制点。

这一步把 odom 或旧轨迹采样得到的 `p/v/a` 边界近似编码进前三个控制点。由于位置样本和四个端点导数约束一起参与超定拟合，它们一般不能全部精确满足；后端固定前三点只能保持这次拟合结果，不能保证新轨迹严格等于请求的起始 `p/v/a`。

### 5.4 时间分配

初始 knot interval 近似为：

```text
ts = control_points_distance / max_vel * 1.5
```

若相邻采样点过远或点数少于 7，会继续缩小 `ts`，保证控制点密度。

优化后用 B-spline 控制点差分检查每个坐标轴的速度和加速度上界。若不可行：

```text
ratio = max(max_vel_actual / vel_limit,
            sqrt(max_acc_actual / acc_limit))
```

然后 `lengthenTime(ratio)` 拉长内部 knot 时间，重新参数化控制点，再做一次 smoothness + fitness + feasibility refine。单机和 `drone_id=0` 会 refine；其他 swarm agent 明确禁用 refine，以避免各机时间轴在已交换轨迹后改变。

### 5.5 全局多项式与局部 B-spline 的关系

全局 polynomial coefficient 只用于参考轨迹和 B-spline 初值，最终不会发布给 controller。实际下游收到的是控制点、knot、order 和 `start_time`。

## 6. 优化代价函数

### 6.1 当前实际目标函数

`combineCostRebound()` 的真实组合为：

```text
J_rebound = lambda_smooth      * J_smooth
          + lambda_collision'  * J_collision
          + lambda_feasibility * J_feasibility
          + lambda_collision'  * J_swarm
          + lambda_collision   * J_terminal
```

其中 `lambda_collision'` 初始等于 `lambda_collision`，碰撞后重启时会翻倍。

注意两个容易误读的点：

- 终点项没有独立参数，复用了 `lambda_collision`。
- 多机项也复用了动态变化的碰撞权重。

时间拉伸后的 refine 目标为：

```text
J_refine = lambda_smooth      * J_smooth
         + lambda_fitness     * J_fitness
         + lambda_feasibility * J_feasibility
```

### 6.2 平滑代价

默认 `calcSmoothnessCost(..., true)` 最小化控制点三阶差分：

```text
j_i = Q_{i+3} - 3 Q_{i+2} + 3 Q_{i+1} - Q_i
J_smooth = sum ||j_i||^2
```

它对应三次均匀 B-spline 的 jerk 平滑趋势。代码没有显式除以 `ts^3`，因此这个代价的数值尺度会随控制点间隔变化；调 `control_points_distance`、`max_vel` 或时间分配时，`lambda_smooth` 的等效意义也会变化。

函数也支持二阶差分的 acceleration smoothness，但主路径使用 jerk 分支。

### 6.3 静态障碍碰撞代价

如上一章所述，使用 `base_point/direction` 的有向投影距离和 `dist0` 安全距离。它不对轨迹所有连续点直接求距离场代价，而是在控制点索引 `[order, N-order)` 上施加推离力；rebound 的后三个控制点虽然是优化变量，却没有静态距离代价。优化后还会离散采样 B-spline 做占据复核，但这不是连续轨迹的数学碰撞保证。

复核步长由轨迹起终点的弦长估算，而不是由曲线弧长或速度上界确定：

```text
t_step = duration / (|p_end - p_start| / map_resolution)
```

弯曲、回环或起终点很近时，这个步长可能过大甚至退化，只检查少量样本并漏掉中段障碍。refine 后使用同样的采样方式，而且两处都只复核前 2/3 轨迹。工程上应把它看作启发式离散检查，而不是“硬碰撞已完整验证”。

`optimization/dist0` 是优化几何安全距离；`grid_map/obstacles_inflation` 已经先扩大了占据体。因此实际保守程度由两者叠加决定，不应把二者都直接设成完整机体半径而不做飞行验证。

### 6.4 动力学可行性代价

当前编译走 `#else` 的分段二次形式。近似控制点速度和加速度为：

```text
v_i = (Q_{i+1} - Q_i) / ts
a_i = (Q_{i+2} - 2 Q_{i+1} + Q_i) / ts^2
```

对 x/y/z 每个分量，只有超过 `max_vel` 或 `max_acc` 才产生平方惩罚和解析梯度。

这里约束的是各轴绝对值，不是三维速度/加速度向量范数。优化后 `UniformBspline::checkFeasibility()` 也按轴检查；对当前均匀 knot，其导数控制点公式会化简为同样的差分形式，主要区别是后者执行硬判定并允许 `manager/feasibility_tolerance`。

### 6.5 局部终点代价

三次均匀 B-spline 末端位置用最后三个控制点计算：

```text
p_end = (Q_{N-3} + 4 Q_{N-2} + Q_{N-1}) / 6
J_terminal = ||p_end - local_target||^2
```

这是位置软约束。局部终点速度被用于生成初始控制点边界，但 rebound 优化的尾部控制点是自由变量，所以最终速度没有独立硬约束或终端速度代价。

### 6.6 多机代价

对其他无人机在相同绝对时间的预测位置，使用竖直方向半轴更长的椭球距离：

```text
d_ellip = sqrt(dx^2/b^2 + dy^2/b^2 + dz^2/a^2), a=2, b=1
```

若小于 `2 * swarm_clearance`，产生平方惩罚。只检查控制点时间对应的前约 2/3 局部轨迹。

这要求所有无人机 `start_time` 位于一致时钟。广播回调会拒绝与本地时间差超过 0.25 s 的新轨迹。

### 6.7 fitness 代价

`J_fitness` 只在时间拉伸后的 refine 阶段使用，使新控制点对应的 B-spline 位置贴近拉伸前参考点。沿参考切线方向容忍度大、垂直方向惩罚强，目的是允许沿路径方向调整时间，同时阻止轨迹横向漂移回障碍物。

### 6.8 动态物体代价当前未启用

`calcMovingObjCost()` 和 `ObjPredictor` 类确实存在，但当前主链有三个事实：

1. `EGOPlannerManager::obj_predictor_` 没有实例化；
2. `combineCostRebound()` 中 `calcMovingObjCost()` 被注释；
3. launch 中 `obj_generator_node` 也被注释，没有启动。

因此当前版本不会对普通动态人/车做轨迹预测避障。launch 的 `use_dynamic` 控制的是四旋翼动力学仿真还是 `poscmd_2_odom` 理想跟踪，不代表动态障碍规划已开启。

### 6.9 权重和关键参数

默认 `advanced_param.launch.py`：

| 参数 | 默认值 | 实际作用 |
|---|---:|---|
| `optimization/lambda_smooth` | 1.0 | rebound/refine 平滑项 |
| `optimization/lambda_collision` | 0.5 | 静态碰撞、多机、终点项；碰撞重启时部分动态翻倍 |
| `optimization/lambda_feasibility` | 0.1 | 速度/加速度软约束 |
| `optimization/lambda_fitness` | 1.0 | 仅 refine 贴合原轨迹 |
| `optimization/dist0` | 0.5 m | 控制点到障碍几何的期望净空 |
| `optimization/swarm_clearance` | 0.5 m | 多机距离参数 |
| `manager/control_points_distance` | 0.4 m | 初值采样密度和初始 `ts` |
| `manager/feasibility_tolerance` | 0.05 | 后验可行性容差 |
| `fsm/thresh_replan_time` | 1.0 s | 周期滚动重规划时间 |
| `fsm/thresh_no_replan_meter` | 1.0 m | 终点邻域内停止周期重规划 |

## 7. 轨迹发布与控制接口

### 7.1 planner 发布什么

规划成功后 `callReboundReplan()` 发布 `traj_utils/msg/Bspline`：

```text
order      = 3
traj_id    = local trajectory sequence id
start_time = updateTrajInfo() 时的当前 ROS 时间
pos_pts    = 位置 B-spline 控制点
knots      = 完整 knot vector
```

消息还定义了 `yaw_pts` 和 `yaw_dt`，但当前 planner 不填充它们。

planner 发布的是**整条连续轨迹描述**，不是固定频率的 setpoint。局部规划一成功就发布一次；重规划成功再发布新的一条，以新的 `traj_id/start_time` 整体替换旧轨迹。

### 7.2 traj_server 怎样执行

`traj_server` 收到消息后重建位置 B-spline，并解析 knot；随后构造一阶和二阶导数轨迹。10 ms timer 每次计算：

```text
t_cur = ROS_TIME_now - msg.start_time
position     = p(t_cur)
velocity     = dp/dt(t_cur)
acceleration = d2p/dt2(t_cur)
```

然后发布 `quadrotor_msgs/PositionCommand`，字段包括 position、velocity、acceleration、yaw、yaw_dot、trajectory_id 和状态标志。仿真 launch remap 到 `drone_<id>_planning/pos_cmd`，由 SO3 controller 或理想 `poscmd_2_odom` 接收；实机栈可再由 PX4 bridge 转换。

轨迹结束后不发布 COMPLETED，而是继续以 `TRAJECTORY_STATUS_READY` 发布终点位置、零速度、零加速度，实现悬停保持。

### 7.3 yaw 怎样规划

yaw 没有进入 B-spline 优化。`traj_server` 在线计算一个前视方向：

```text
dir = p(min(t_cur + time_forward, duration)) - p(t_cur)
yaw_target = atan2(dir_y, dir_x)
```

默认 `time_forward=1.0 s`。若方向长度小于 0.1 m，则保持上次 yaw。随后：

- 处理 `[-pi, pi]` wrap；
- 限制最大 yaw rate 为 `pi rad/s`；
- 对 yaw 和 yaw_dot 做 0.5/0.5 的简单低通。

所以当前系统是“机头大致朝轨迹前方”，不是联合优化 yaw，也没有观测目标朝向或相机可见性约束。

### 7.4 时间戳怎样对齐

规划器在 `updateTrajInfo(pos, now)` 时设置 `local_data_.start_time_ = now`，随后把同一值写入 B-spline 消息。`traj_server` 用 ROS time 相减得到执行进度。

这要求 planner、traj_server 和 swarm peers 使用一致 clock。当前 planner/FSM/optimizer 多数使用默认 `rclcpp::Clock()`，即 system time；`traj_server` 的采样循环却显式创建 `RCL_ROS_TIME` clock，地图部分又使用 `node_->now()`。在没有 `/clock` 且 ROS time 回落到系统时间时通常一致；一旦启用 `use_sim_time`，这种混用可能造成时间类型不兼容或轨迹进度错误。多机或仿真部署应统一使用节点 clock，并实测广播轨迹的 0.25 s 新鲜度检查。

## 8. 当前实现中容易被名字误导的地方

以下参数/字段存在，但当前主路径没有真正使用或语义不同：

| 名称 | 当前实际情况 |
|---|---|
| `fsm/planning_horizen_time` | 读取但未参与局部目标或 replan 判定 |
| `manager/max_jerk` | 读取但没有 jerk 上限约束；只有 jerk 平滑代价 |
| `bspline/limit_vel`, `limit_acc`, `limit_ratio` | launch 设置，但当前代码没有声明/读取这些 ROS 参数；物理上限来自 `manager/*` 和 `optimization/*` |
| `Bspline.yaw_pts`, `yaw_dt` | 消息保留字段，planner 不填，traj_server 不读 |
| `REFENCE_PATH=3` | 枚举存在，当前初始化分支未实现 |
| `use_dynamic` | 控制飞行器仿真模型，不是动态障碍代价开关 |
| `ObjPredictor` / `calcMovingObjCost` | 代码存在但主目标函数未接通 |
| `getInflateOccupancy()` | 返回 `-1/0/1`，不是严格 bool；地图外会在 bool 语境中算碰撞 |
| “ESDF gradient” 注释 | 遗留描述；实际梯度来自 `base_point/direction` |
| safety timer 的 swarm 循环 | 循环未来 `t` 时没有推进双方预测时间，实际主要检查当前距离 |
| safety timer | 与规划、地图和传感器回调共用单线程 executor，长规划期间不会并发运行 |
| `EMERGENCY_STOP` | 只有 `flag_escape_emergency_` 为 true 才发布静止轨迹；当前通常不可达的 `SEQUENTIAL_START` 成功时未设置该 flag，若恢复该分支，普通急停可能继续执行旧轨迹 |
| `hasOccupancyObservation()` | 只在静态图 flag 和独立点云 flag 中二选一，不把已融合深度视为占据观测 |

另一个重要边界是：安全检查和优化后的离散碰撞复核主要关注局部轨迹前 2/3。这是 receding-horizon 假设的一部分：后 1/3 预期会在下一轮被替换。若重规划回调卡住、传感器延迟或控制器继续执行旧轨迹，尾段安全余量会比“整条轨迹均已验证”更弱。优化后的离散复核还按起终点弦长确定时间步长，在高曲率或近回环轨迹上存在漏检风险。

## 9. 如果把语义接进 EGO-Planner

语义接入应分地图、代价和决策三层，不建议把类别 ID 直接塞进现有二值占据缓存后到处写条件分支。

### 9.1 地图层：类别相关膨胀和查询

当前最自然的位置是 `GridMap`：

```text
semantic point/depth detection
  -> voxel semantic class / confidence / timestamp
  -> class-specific inflation
  -> collision query returns more than 0/1
```

建议新增与几何缓存对齐的结构，例如：

```text
semantic_class_buffer_[voxel]
semantic_confidence_buffer_[voxel]
semantic_last_seen_[voxel]
```

并提供结构化查询，而不是破坏现有 `getInflateOccupancy()`：

```cpp
SemanticVoxel GridMap::querySemantic(const Eigen::Vector3d& pos) const;
double GridMap::semanticClearance(SemanticClass cls) const;
```

类别膨胀可采用：

- 墙、柱：稳定静态障碍，使用机体半径 + 定位误差；
- 人、车：更大安全半径，并随速度/预测不确定度增长；
- 门、通道：不要简单膨胀成不可通行，应保留通行宽度和中心线偏好；
- 草、树叶、烟雾等：根据任务定义为软风险而非硬占据。

注意深度路径和独立点云路径当前 z 膨胀不一致。做按类膨胀时最好统一成明确的 3D kernel/footprint API。

### 9.2 代价层：语义风险如何产生梯度

最直接的改动点是 `BsplineOptimizer::combineCostRebound()`，新增：

```text
J = ... + lambda_semantic * J_semantic
```

但不能只返回类别权重，还需要可微方向。可按数据性质选择两种方案：

**硬语义障碍**

沿用 EGO 的 ESDF-free 机制：在 `ControlPoints` 的约束中附加 `class_id`、`clearance`、`risk_weight`。A* 生成的 `base_point/direction` 不变，但每个类别采用不同净空和代价系数：

```text
e = clearance(class) - (Q-P)^T v
J_semantic_obstacle = weight(class) * phi(e)
```

这最符合现有代码结构，也最容易保持实时性。

**软语义区域**

对通道中心、可观测区域、禁飞概率、地表风险等，单独提供连续 cost/gradient 查询。例如语义栅格预计算局部距离/风险梯度，或用走廊中心线的解析投影。不要强行把“偏好走廊”编码成占据，否则 A* 和安全定时回调都会把软偏好误当硬碰撞。

函数落点建议：

- 扩展 `ControlPoints`：保存类别、单约束净空和权重；
- 新增 `calcSemanticCost()`：保持与 `calcDistanceCostRebound()` 同样的 cost + gradient 契约；
- 在 `combineCostRebound()` 中接入；
- 给语义项独立 `optimization/lambda_semantic`，不要继续复用 `lambda_collision`。

### 9.3 动态人车：要同时接时间

普通语义占据不足以处理移动人/车。当前 dormant 的 `ObjPredictor` 路径可以作为参考，但需要真正完成：

1. 实例化 predictor，并接入带 `class_id/velocity/covariance` 的 track；
2. 在控制点对应绝对时间预测目标位置；
3. 按类别和预测协方差生成时变椭球；
4. 恢复或重写 `calcMovingObjCost()` 并加入主目标；
5. 安全定时回调也要使用同一预测模型，否则优化器认为安全而硬检查只看当前占据，判据会不一致。

对于人，代价通常应同时考虑更大净空、速度相关前向缓冲和不确定度；对于车，应考虑航向相关的非球形 footprint。

### 9.4 决策层：语义目标和 replan 策略

决策入口集中在 FSM：

- `waypointCallback()`：把“目标物/房间/门”解析为可达几何目标，并做占据与语义合法性校验；
- `getLocalTarget()`：不再只按欧氏 horizon 取点，可根据门、通道、观测位姿改变局部目标；
- `EXEC_TRAJ` replan 条件：加入语义事件，例如人进入警戒区、门状态变化、目标置信度下降；
- `checkCollisionCallback()`：区分必须急停的硬障碍和可降速/绕行的语义风险。

建议把决策输出收敛成一个明确结构：

```text
SemanticLocalGoal {
  position, velocity,
  desired_yaw / view_target,
  corridor_id,
  risk_policy,
  valid_until
}
```

这样语义不会散落成多个全局 flag，并能进一步把 yaw/可观测性送入优化器或单独的 yaw planner。

### 9.5 推荐实施顺序

1. **先做只读语义地图与可视化**：确认坐标、时间戳、类别稳定性，不改变飞行行为。
2. **做类别相关硬膨胀**：复用现有二值查询，最容易验证安全收益。
3. **把 per-constraint clearance/weight 放入 `ControlPoints`**：形成真正语义化的 ESDF-free 碰撞代价。
4. **加入软语义 cost**：通道、可观测性、目标偏好。
5. **最后做动态预测和 FSM 策略**：这会同时影响时间同步、优化和 fail-safe，测试面最大。

## 10. 建议的源码阅读顺序

带着一次真实任务闭环阅读：

1. `single_run_in_sim.launch.py` 与 `advanced_param.launch.py`：先画出 topic 和节点。
2. `EGOReplanFSM::init()`、`waypointCallback()`、`execFSMCallback()`：看目标怎样启动系统。
3. `planFromGlobalTraj()`、`planFromCurrentTraj()`、`getLocalTarget()`：看滚动窗口和轨迹接续。
4. `GridMap::cloudCallback()` 与 `updateOccupancyCallback()`：根据你的传感器链选择重点通路。
5. `GridMap::getInflateOccupancy()`：确认所有碰撞判定最终落在哪个缓存。
6. `EGOPlannerManager::reboundReplan()`：看 INIT -> OPTIMIZE -> REFINE 主线。
7. `BsplineOptimizer::initControlPoints()`：精读 ESDF-free 的 A*、基点和方向生成。
8. `calcDistanceCostRebound()` 与 `combineCostRebound()`：把论文公式对到真实目标函数。
9. `UniformBspline`：理解控制点、导数和时间伸缩。
10. `traj_server.cpp`：看整条轨迹怎样变成 100 Hz 控制命令和 yaw。

读完后应能回答这几个检查题：

- 新目标到来时，全局参考轨迹是否避障？答案：不避障，局部优化负责避障。
- 周期 replan 从 odom 还是旧轨迹接？答案：首次用 odom，正常 replan 用旧轨迹当前 `p/v/a`。
- 地图查询能否直接给距离和梯度？答案：不能，只给膨胀占据；梯度由 A* 几何约束生成。
- controller 收到控制点还是 setpoint？答案：`traj_server` 收控制点，controller 收 100 Hz `PositionCommand`。
- yaw 是否由 optimizer 联合规划？答案：否，由执行端按前视轨迹方向在线生成。
- 当前是否真的启用了普通动态障碍预测？答案：没有，多机轨迹避碰启用，`ObjPredictor` 代价未接通。

FSM、占据查询、B-spline 控制点、ESDF-free 碰撞代价这四条主线，共同决定了 EGO-Planner 是一个能持续飞行和恢复的机器人规划系统，而不只是一次性求解器。
