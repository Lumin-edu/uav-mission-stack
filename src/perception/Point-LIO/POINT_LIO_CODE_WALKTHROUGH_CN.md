# Point-LIO 核心代码导读

本文对应当前仓库中的实现，而不是只按论文描述推导。核心文件如下：

- `include/common_lib.h`：状态流形、状态索引、测量包、平面拟合。
- `src/Estimator.cpp`：过程模型、过程 Jacobian、LiDAR/IMU 残差。
- `include/IKFoM/IKFoM_toolkit/esekfom/esekfom.hpp`：状态传播和 Kalman 更新。
- `src/laserMapping.cpp`：按点时间推进、点级更新、异常更新回滚、地图写入。
- `src/li_initialization.cpp`：LiDAR/IMU 缓冲、固定时差修正和帧级同步。
- `src/IMU_Processing.cpp`：静止初始化；这里没有传统的整帧去畸变。
- `src/preprocess.cpp`：把每个点的相对时间写入 `PointType::curvature`，统一为 ms。

## 1. 总体数据流

```text
LiDAR callback                       IMU callback
  | 预处理、点时间 -> curvature        | 固定时差修正
  v                                   v
lidar_buffer/time_buffer            imu_deque
             \                       /
              \--- sync_packages ---/
                       |
              等 IMU 覆盖 LiDAR 帧末端
                       |
              静止初始化 / 建初始地图
                       |
          点按 curvature 排序并按时间分组
                       |
          传播到 IMU 时刻，再传播到点时刻
                       |
        iVox 近邻 -> 拟合平面 -> 点面残差
                       |
              ESEKF 点时间组更新
                       |
     更新可接受：发布并入图；否则只发布传播结果
```

这里的关键不是“先把整帧去畸变，再做一次帧级 ICP”，而是让状态沿扫描时间连续向前走，在每个点point时间组上立即构造 scan-to-map 约束并更新。

## 2. 状态量定义

### 2.1 IMU 作为输入：`state_input`，24 维

由 `use_imu_as_input=true` 选择：

| 误差索引 | 状态 | 含义 |
|---|---|---|
| 0:3 | `pos` | IMU/body 在世界系的位置 `p_WI` |
| 3:6 | `rot` | IMU/body 到世界系的旋转 `R_WI` |
| 6:9 | `offset_R_L_I` | LiDAR 到 IMU 的旋转 `R_IL` |
| 9:12 | `offset_T_L_I` | LiDAR 原点在 IMU 系的位置 `t_IL` |
| 12:15 | `vel` | 世界系速度 `v_WI` |
| 15:18 | `bg` | 陀螺零偏 |
| 18:21 | `ba` | 加计零偏 |
| 21:24 | `gravity` | 世界系重力向量 |

`rot` 和 `offset_R_L_I` 的名义量属于 SO(3)，但误差量用 3 维李代数表示，因此状态流形的自由度仍是 24。

### 2.2 IMU 作为观测：`state_output`，30 维

这是配置中的默认模式（`use_imu_as_input=false`）：

| 误差索引 | 状态 | 含义 |
|---|---|---|
| 0:15 | `pos, rot, offset_R_L_I, offset_T_L_I, vel` | 与 24 维模式相同 |
| 15:18 | `omg` | 当前机体系角速度 |
| 18:21 | `acc` | 去掉 bias 后的机体系比力/加速度状态 |
| 21:24 | `gravity` | 世界系重力向量 |
| 24:27 | `bg` | 陀螺零偏 |
| 27:30 | `ba` | 加计零偏 |

两种模式最大的差别是：24 维模式直接用 IMU 读数驱动运动方程；30 维模式先把 `omg`、`acc` 作为连续状态传播，再把 IMU 读数作为 6 维观测校正它们和 bias。

### 2.3 外参与坐标变换

代码中的点变换是：

```text
p_I = R_IL p_L + t_IL
p_W = R_WI p_I + p_WI
```

当 `extrinsic_est_en=false` 时，使用配置中的 `Lidar_R_wrt_IMU` 和 `Lidar_T_wrt_IMU`，LiDAR Jacobian 的外参 6 列置零。当它为 true 时，外参进入状态并由点面残差联合估计。

## 3. IMU 传播

### 3.1 24 维输入模式

`Estimator.cpp::get_f_input()` 实现：

```text
p_dot = v
R_dot = R * Exp((gyro_m - bg) dt)
v_dot = R (acc_m - ba) + g
bg_dot = 0, ba_dot = 0, g_dot = 0
```

对应的主要连续时间 Jacobian 块为：

```text
d(p_dot)/d(v)   = I
d(v_dot)/d(dR) = -R [acc_m-ba]x
d(v_dot)/d(ba) = -R
d(v_dot)/d(g)  = I
d(R_dot)/d(bg) = -I
```

过程噪声非零块位于姿态、速度、陀螺 bias、加计 bias，对应 `gyr_cov_input`、`acc_cov_input`、`b_gyr_cov`、`b_acc_cov`。

### 3.2 30 维输出模式

`Estimator.cpp::get_f_output()` 实现：

```text
p_dot = v
R_dot = R * Exp(omg dt)
v_dot = R acc + g
omg_dot = 0, acc_dot = 0, bg_dot = 0, ba_dot = 0, g_dot = 0
```

主要 Jacobian 块为：

```text
d(p_dot)/d(v)   = I
d(v_dot)/d(dR) = -R [acc]x
d(v_dot)/d(acc)= R
d(v_dot)/d(g)  = I
d(R_dot)/d(omg)= I
```

IMU 观测在 `h_model_IMU_output()` 中定义：

```text
r_gyro = gyro_m - omg - bg
r_acc  = acc_m * |g| / acc_norm - acc - ba
```

因此每个 gyro 轴的 H 在 `omg` 和 `bg` 上为 1，每个 acc 轴的 H 在 `acc` 和 `ba` 上为 1。代码直接从协方差相应列构造 `P H^T`。

`acc_norm=1.0` 表示驱动输出单位是 g，代码乘以 `|g|`；如果输入已经是 m/s^2，应把 `acc_norm` 配成约 9.81。

### 3.3 名义状态与协方差为什么分开传播

`esekfom::predict(dt, Q, input, predict_state, prop_cov)` 允许只推进状态或只推进协方差：

```text
x <- x boxplus (f(x,u) dt)
P <- F P F^T + Q dt^2
```

点级主循环用两个时间游标：一个记录名义状态已经传播到哪里，另一个记录协方差已经传播到哪里。这样状态可以准确落在每个点的时间上，而协方差可按 `prop_at_freq_of_imu` 选择 IMU 频率或点更新频率累计。

IMU 样本之间没有线性插值，采用上一样本零阶保持。最后不足一个 IMU 周期的时间段，直接用当前保持值传播到点时间。

## 4. 初始化

`IMU_init()` 在启动阶段累计至少 `MAX_INI_COUNT=100` 个 IMU 样本的在线均值。主循环用：

```text
tmp_gravity = -mean_acc / |mean_acc| * |g|
```

再由 `Set_init()` 求一个旋转，使测得的重力方向与配置的世界重力方向对齐。初始化期间要求设备静止，否则线加速度会被误当成重力，初始 roll/pitch 会偏。

当前代码虽然统计了 `mean_gyr`，但没有把它写入 `bg`；初始陀螺 bias 仍为状态默认值，并依赖后续融合收敛。初始化也没有实际计算注释中所说的 IMU 方差。

初始姿态完成后，系统继续保持静止并积累 `init_map_size` 个世界系点，建立第一份 iVox 地图，然后才允许正常运动。

## 5. 点级时间更新

### 5.1 点时间编码

预处理将每个点相对帧起点的时间写入 `curvature`，统一为毫秒。绝对点时间为：

```text
t_point = lidar_beg_time + curvature / 1000.0
```

点云先按 `curvature` 排序。`time_compressing()` 把相同时间戳的连续点压成一组；每组只做一次传播和一次 LiDAR 更新。若每个点时间都不同，则组大小为 1，即真正逐点更新。

### 5.2 默认 30 维模式的顺序

对每个点时间组：

1. 消费所有时间不晚于当前点的 IMU 样本。
2. 将名义状态传播到各 IMU 时刻。
3. 将协方差传播到 IMU 时刻，并执行 6 维 IMU 观测更新。
4. 用最后一个 IMU 状态保持值传播剩余小段到当前点时刻。
5. 对当前点组查询地图、拟合平面、构造点面残差并做 ESEKF 更新。
6. 用更新后的当前时刻状态把该组点变换到世界系。

### 5.3 24 维输入模式的顺序

区别只在 IMU 部分：读取 `imu_last` 的 gyro/acc，扣除状态 bias 后直接进入过程模型；到达 IMU 时刻和点时刻时分别积分，不执行 `update_iterated_dyn_share_IMU()`。

`ImuProcess::Process()` 只是完成静止初始化并原样返回点云，没有生成传统意义上“统一到帧首/帧末”的去畸变点云。变量名 `feats_undistort` 是历史命名；真正的运动处理发生在上述按点状态推进中。

## 6. 点面残差构造

对当前点 `p_L`：

1. 按当前状态变换得到 `p_W`。
2. 在 iVox 中取 `NUM_MATCH_POINTS=5` 个最近邻。
3. 用 5 个点解平面 `n^T x+d=0`，归一化 `n`。
4. 要求每个近邻到拟合平面的距离不超过 `plane_thr`。
5. 计算当前点到平面的绝对距离 `pd2=|n^T p_W+d|`。
6. 要求 `|p_L| > match_s * pd2^2`，即 `pd2 < sqrt(|p_L|/match_s)`。

滤波器使用的创新为：

```text
z = -(n^T p_W + d)
```

若在线估计外参，12 列几何 Jacobian 对应：

```text
H = [n^T, A^T, B^T, C^T]
C = R_WI^T n
A = [p_I]x C
B = [p_L]x R_IL^T C
```

分别对应位置、机体姿态、外参旋转、外参平移。固定外参时后 6 列为零，姿态项使用预先计算的 `[p_I]x R_WI^T n`。

LiDAR 的 H 只显式覆盖前 12 维，但 Kalman 增益使用完整协方差的前 12 列，所以速度、重力、bias、omega/acc 可以通过互协方差间接被 LiDAR 修正。

## 7. ESEKF 更新

测量维数小于状态维数时，代码走标准形式：

```text
S = H P H^T + R
K = P H^T S^-1
dx = K z
x <- x boxplus dx
P <- P - K H P
```

测量维数不小于状态维数时，代码切到信息形式以避免求逆一个很大的测量矩阵。

需要注意两点：

- `maximum_iter` 在初始化中被设为 1。函数名虽为 `update_iterated_dyn_share_modified()`，当前行为并不是多轮重线性化 IEKF。
- 大测量维分支把 `M_Noise` 乘在 `H^T H` 上，等价于把它当信息权重 `R^-1`；小测量维分支却把同一量加到 `H P H^T` 对角线上，等价于把它当方差 `R`。两条分支对 `laser_point_cov` 的语义不一致。点时间戳大多严格递增、组较小时通常走小测量分支，但多线雷达同时间组较大时应重点核查这一点。

## 8. 时间同步

### 8.1 时间轴修正

IMU callback 修改时间戳：

```text
t_imu_aligned = t_imu_raw
              - timediff_imu_wrt_lidar
              - time_diff_lidar_to_imu
              - time_lag_IMU_wtr_lidar
```

当前自动估计时差的代码已注释，通常只有配置项 `common.time_diff_lidar_to_imu` 非零。配置正号的含义必须按上式理解：正值会把 IMU 时间戳向前移。

### 8.2 帧级同步条件

`sync_packages()` 从点云最大 `curvature` 得到帧末时间，并要求：

```text
last_timestamp_imu >= lidar_end_time
```

不满足就保留当前 LiDAR 帧并等待 IMU。同步函数还会检查：

- 点云时长是否接近 `lidar_time_inte`；
- IMU 队首是否明显晚于 LiDAR 帧首；
- 在线队列是否超过上限，超过时丢弃最旧数据；`offline_map` 不做此裁剪。

初始化完成前，帧末之前的 IMU 被放进 `MeasureGroup::imu` 供均值统计；完成后，点级循环直接消费全局 `imu_deque`。

## 9. 退化与异常处理

### 9.1 已实现的保护

- 邻域不足 5 点：该点不生成约束。
- 邻域不够平：5 个近邻中任一点超过 `plane_thr`，拒绝该平面。
- 点面距离过大：由 `match_s` 距离相关门限拒绝。
- 当前时间组无有效点：LiDAR 更新返回 false，保留传播状态并继续下一组。
- IMU 饱和：默认模式下接近 `satu_acc/satu_gyro` 99% 的轴不参与该次 IMU 更新。
- LiDAR 创新门限：启用 `lidar_innovation_gate_en` 后，若单次点组更新造成的位置或姿态修正过大，则恢复更新前的状态和协方差。
- 无可接受 LiDAR 约束：仍发布惯性传播里程计，但整帧不写入 iVox，避免污染地图。
- 时间倒退：回调丢弃本条倒退消息。
- 在线积压：LiDAR/IMU 队列超过上限时丢弃最旧数据并告警。
- 2D 模式：每次传播/更新后强制保留 x、y、yaw，清零 z 相关运动量。这是硬约束，不是统计观测。

### 9.2 没有实现的显式退化判定

代码没有对 `H^T R^-1 H`、Hessian 或协方差做特征值/条件数检查，也没有检测走廊、单平面、纯旋转等几何可观方向。因此“有很多有效点”不等于“六自由度都可观”。例如只有地面平面时，点面约束主要限制 z、roll、pitch，对平面内平移和 yaw 很弱。

创新门限只能拦截已经产生的大跳变，不能识别小幅但持续的不可观漂移；而且 `mid360.yaml` 未显式启用该门限，使用参数默认值时是关闭的，`mid360_mapping.yaml` 才显式设为 true。

### 9.3 值得关注的边界

- 平面拟合后直接用 `normvec.norm()` 归一化，没有显式检查接近零或非有限值。
- Kalman 更新直接调用矩阵 `inverse()`，没有 LDLT/LLT 失败检查、正定性检查或 NaN 回退。
- 时间倒退日志写着 `clear buffer/deque`，实际只丢弃当前消息，并未清空队列或重置滤波器。
- IMU 缺少覆盖时会等待，但 IMU 队首晚于帧首只告警，不补插值、不回退该帧。
- 创新门限按“当前点时间组”回滚；若同一帧前面的点组已经成功更新，后续点组触发门限时只撤销后一个点组，之前的 LiDAR 修正仍保留，但整帧不会入图。
- 协方差更新采用简化式 `P-KHP`，没有 Joseph 形式及显式对称化，长时间数值误差需要运行时监控。

## 10. 参数理解建议

| 参数 | 主要作用 | 调整风险 |
|---|---|---|
| `lidar_meas_cov` | 点面更新测量噪声 | 太小会过度相信地图匹配；还受大小测量分支语义不一致影响 |
| `plane_thr` | 近邻平面一致性 | 太大接纳弯曲/边缘，太小导致有效点不足 |
| `match_s` | 点面残差距离门限 | 数值越大，允许的 `pd2` 越小 |
| `acc/gyr_cov_*` | IMU 驱动状态的过程不确定性 | 太小导致状态不愿跟随新运动，太大导致预测噪声增大 |
| `imu_meas_*_cov` | 默认模式 IMU 观测噪声 | 太小会强迫 `omg/acc+bias` 跟随原始读数 |
| `b_acc/b_gyr_cov` | bias 随机游走 | 太小 bias 难适应温漂，太大容易吸收真实运动 |
| `time_diff_lidar_to_imu` | 固定时间轴平移 | 符号或量值错误会直接造成点级残差系统性偏差 |
| `max_lidar_*_correction` | 更新后跳变门限 | 太严会频繁退回纯惯导，太松无法阻止错误匹配入状态 |

实际诊断时应同时观察有效点数、接受更新数、队列丢包/同步告警、单次修正量和协方差特征值。当前日志已有前四类中的大部分，但协方差谱和几何信息矩阵谱尚未输出。
