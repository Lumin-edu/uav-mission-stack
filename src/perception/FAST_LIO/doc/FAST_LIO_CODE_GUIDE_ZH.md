# FAST-LIO2 算法与源码导读

本文对应当前目录中的 ROS 2 实现，目标是解释代码实际执行的逻辑，而不只复述论文框图。

## 1. 先建立整体认识

FAST-LIO2 是一个紧耦合 LiDAR-Inertial Odometry（LIO）前端。它用 IMU 给出高频运动预测，
再把 LiDAR 点到局部地图平面的距离直接作为迭代误差状态卡尔曼滤波器（Iterated ESEKF）的
观测。滤波后的当前扫描再增量加入 ikd-Tree 地图。

它不是完整的全局 SLAM 系统：本目录中没有回环检测、全局位姿图优化或回环后的地图纠正。
长时间运行仍会累积漂移，只是 LiDAR 与 IMU 的紧耦合使局部里程计非常快且稳健。

一帧数据的主链路如下：

```text
LiDAR 回调 -> 点格式/逐点时间预处理 ---+
                                      +-> 时间同步成 MeasureGroup
IMU 回调 -----------------------------+
       -> 启动静止初始化
       -> IMU 前向传播 x、P，并记录帧内轨迹
       -> 把所有 LiDAR 点补偿到扫描末时刻
       -> 原始点体素降采样
       -> 当前状态投点到世界系
       -> ikd-Tree 查 5 个近邻并拟合局部平面
       -> 点到平面残差 + 雅可比
       -> Iterated ESEKF 多轮修正
       -> 用修正后的状态发布里程计
       -> 当前帧增量插入 ikd-Tree
```

对应的代码入口是 `LaserMappingNode::timer_callback()`。

## 2. 源码文件分工

| 文件 | 作用 | 建议阅读重点 |
| --- | --- | --- |
| `src/laserMapping.cpp` | ROS 节点、同步、点面观测、主循环、增量建图 | `sync_packages`、`h_share_model`、`timer_callback` |
| `src/IMU_Processing.hpp` | IMU 初始化、状态传播、点云去畸变 | `IMU_init`、`UndistortPcl` |
| `include/use-ikfom.hpp` | 状态流形、连续运动模型及雅可比 | `state_ikfom`、`get_f`、`df_dx`、`df_dw` |
| `include/IKFoM_toolkit/esekfom/esekfom.hpp` | 通用流形 ESEKF | `predict`、`update_iterated_dyn_share_modified` |
| `include/ikd-Tree/ikd_Tree.cpp` | 可增量更新、懒删除和重建的 kd-tree | `Add_Points`、`Nearest_Search`、`Rebuild` |
| `src/preprocess.cpp` | 适配不同雷达，生成逐点相对时间，可选特征提取 | 各 lidar handler |
| `include/common_lib.h` | 公共点类型、测量包、平面拟合 | `MeasureGroup`、`esti_plane` |

## 3. 坐标系与变换

代码涉及三套坐标系：

- `L`：LiDAR 坐标系。
- `I`：IMU/机体坐标系。
- `W`：初始化时建立的世界坐标系，ROS 消息中叫 `camera_init`。

状态中的外参含义为：

- `offset_R_L_I = R_IL`：把 LiDAR 向量旋转到 IMU 系。
- `offset_T_L_I = p_IL`：LiDAR 原点在 IMU 系中的坐标。

LiDAR 点投到世界系的完整公式是：

```text
P_I = R_IL P_L + p_IL
P_W = R_WI P_I + p_WI
```

也就是源码中的：

```cpp
s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos
```

注意，`laserMapping.cpp` 中很多变量沿用旧命名：`point_body`、`feats_down_body` 实际是
扫描末时刻的 LiDAR 系点，不是已经变换到 IMU 系的点。

## 4. 状态为什么是 23 维

名义状态定义在多个流形的直积上：

| 局部误差下标 | 状态 | 自由度 | 含义 |
| --- | --- | ---: | --- |
| 0..2 | `pos` | 3 | 世界系 IMU 位置 |
| 3..5 | `rot` | 3 | IMU 到世界的旋转 `R_WI` |
| 6..8 | `offset_R_L_I` | 3 | LiDAR 到 IMU 的外参旋转 `R_IL` |
| 9..11 | `offset_T_L_I` | 3 | LiDAR 在 IMU 系的外参平移 `p_IL` |
| 12..14 | `vel` | 3 | 世界系速度 |
| 15..17 | `bg` | 3 | 陀螺零偏 |
| 18..20 | `ba` | 3 | 加速度计零偏 |
| 21..22 | `grav` | 2 | 固定模长重力在球面 `S^2` 上的方向 |

合计为 `3+3+3+3+3+3+3+2=23` 个局部自由度。

名义状态展开是 24 维，因为重力仍保存为 3 维向量；但其模长固定，只允许在球面切平面
中用 2 个自由度修正。这就是 `get_f()` 返回 24 行，而滤波协方差 `P` 是 `23 x 23` 的原因。

旋转不能直接做普通向量加法。IKFoM 的 `boxplus` 用指数映射注入小旋转，`boxminus` 把
两个流形状态的差取回局部切空间，因此每次更新后旋转仍是合法的 `SO(3)` 元素。

## 5. 预处理与测量同步

### 5.1 逐点时间是去畸变的基础

不同雷达驱动的时间字段单位不同。`Preprocess::process()` 把它们统一为毫秒，并复用
`PointXYZINormal::curvature` 保存“该点相对扫描帧首的时间偏移”。这里的 `curvature`
不是曲率。

Livox 的 `offset_time` 是 ns，因此代码除以 `1e6` 得到 ms。Velodyne/Ouster 根据配置的
`timestamp_unit` 缩放。若旋转雷达没有逐点时间，代码会按扫描线方位角和转速估算；若走
`default_handler()`，所有点偏移为 0，实际无法进行有效的帧内运动补偿。

FAST-LIO2 默认 `feature_extract_enable=false`。此时不使用 LOAM 式边缘/平面特征提取，
而是保留通过盲区、重复点和抽样检查的原始点，后面直接建立点面约束。

### 5.2 `sync_packages()` 如何组一帧

1. 取 LiDAR 队首帧，以最后一个点的相对时间估计 `lidar_end_time`。
2. 若 IMU 最新时间还没越过帧末，保留这帧并返回等待。
3. 收集所有时间不晚于帧末的 IMU，形成 `MeasureGroup`。
4. 弹出本帧 LiDAR，交给 IMU 前端处理。

这个等待条件很重要。没有覆盖到帧末的 IMU，既无法把滤波状态传播到统一时刻，也无法
补偿扫描后半段点的运动。

## 6. IMU 初始化

`ImuProcess::IMU_init()` 假设系统启动时近似静止，跨 `MAX_INI_COUNT` 个测量包在线统计：

```text
mean_acc ~= -R_IW g_W + ba
mean_gyr ~= bg
```

当前实现忽略初始化阶段的加计零偏，把 `mean_acc` 的反方向作为初始重力方向，并把模长
固定到 `9.81 m/s^2`；`mean_gyr` 直接作为初始陀螺零偏。配置外参也在这里写入滤波状态。

因此启动阶段的运动会把真实角速度混进 `bg`，把线加速度混进重力方向，后续容易出现姿态
和速度漂移。实际使用时应让设备在启动的前若干帧保持静止。

## 7. IMU 预测模型

记 IMU 测量为 `omega_m`、`a_m`，零偏为 `bg`、`ba`。连续时间名义模型是：

```text
p_dot = v
R_dot = R [omega_m - bg]x
v_dot = R (a_m - ba) + g
bg_dot = n_bg
ba_dot = n_ba
g_dot = 0
R_IL_dot = 0
p_IL_dot = 0
```

`get_f()` 给出上述名义状态导数，`df_dx()` 给状态雅可比，`df_dw()` 把 12 维过程噪声
`[n_g, n_a, n_bg, n_ba]` 注入系统。

`UndistortPcl()` 对相邻 IMU 两端测量取均值，使用中值法得到区间输入，然后调用：

```cpp
kf_state.predict(dt, Q, in);
```

`predict()` 同时完成：

```text
x <- x (+) (f(x,u) dt)
P <- F P F^T + G Q G^T
```

其中 `(+ )` 是流形上的积分，不是对所有状态做普通相加。

## 8. 点云运动去畸变

机械旋转雷达的一帧通常持续约 0.1 秒，固态雷达也会在一个积分窗口内依次发射点。运动中
每个点的采样位姿不同，若把整帧当成同一位姿，墙面会被拉弯，点面残差也会系统性变大。

代码先在 IMU 时间线上前向传播，并保存每个 IMU 时刻的 `R、p、v、a、omega`。然后从
扫描末尾向前遍历点云，在相邻 IMU 锚点内插出点时刻 `i` 的位姿，把该点统一变换到帧末
LiDAR 系 `L_e`：

```text
P_Ii = R_IL P_Li + p_IL
P_W  = R_WIi P_Ii + p_WIi
P_Le = R_IL^T (R_WIe^T (P_W - p_WIe) - p_IL)
```

完成后，整帧点云和滤波预测状态都对应同一个 `lidar_end_time`，可以建立刚体 scan-to-map
观测。源码注释中的 backward propagation 指从晚到早遍历点，并不是让 EKF 全程反向积分。

## 9. 直接 scan-to-map 观测

### 9.1 数据关联与平面拟合

对每个降采样后的当前扫描点：

1. 用当前 IEKF 迭代状态把点投到世界系。
2. 在 ikd-Tree 地图中找 5 个最近点。
3. 若第 5 个点的平方距离大于 5，拒绝该对应。
4. 用 5 个点最小二乘拟合平面 `n^T P + d = 0`。
5. 若任一近邻离拟合平面超过 `0.1 m`，认为局部不是可靠平面。
6. 再按当前点的点面距离和量程评分，拒绝动态物体、边缘或错误关联。

平面拟合在 `esti_plane()` 中完成。它先固定常数项求：

```text
[x_j y_j z_j] [a b c]^T = -1
```

然后将 `[a,b,c,1]` 整体归一化，得到单位法向量 `n` 和常数 `d`。

### 9.2 残差

当前点的世界坐标是：

```text
P_W(x) = R_WI (R_IL P_L + p_IL) + p_WI
```

标量观测残差为：

```text
r(x) = n_W^T P_W(x) + d
```

理想情况下点落在地图平面上，所以目标是 `r=0`。代码将创新向量写成 `h=-r`。

### 9.3 为什么 H 只有 12 列

单个点面残差直接依赖位置、机体旋转、外参旋转和外参平移，不直接依赖速度、IMU 零偏
和重力。因此 `h_share_model()` 只构造：

```text
H_i = [H_p, H_R, H_Rext, H_text]    // 1 x 12
```

源码中的中间量为：

```text
C = R_WI^T n_W
A = [R_IL P_L + p_IL]x C
B = [P_L]x R_IL^T C
```

按 IKFoM 的右扰动约定，H 的 12 列依次装入 `n_W、A、B、C`。关闭
`extrinsic_est_en` 时，后 6 列置零。

速度、零偏和重力仍可能被 LiDAR 间接修正，因为传播后的协方差 `P` 含有它们与位姿之间
的交叉相关项。这正是紧耦合滤波区别于“单独 ICP 位姿 + 后处理融合”的关键。

## 10. 迭代 ESEKF 更新

普通 EKF 只在 IMU 预测状态处线性化一次。点面残差对旋转非线性，而且数据关联也依赖
当前位姿，因此 FAST-LIO 在一帧内多轮迭代：

1. 固定 IMU 传播先验 `x_prior、P_prior`。
2. 在当前迭代状态 `x_i` 下重新投点、计算残差和 H。
3. 将先验协方差搬运到 `x_i` 的流形切空间。
4. 求误差增量 `delta_x`，通过 `boxplus` 注入状态。
5. 检查 23 个误差分量是否小于阈值；收敛或达到最大次数后更新协方差。

设每个点使用相同方差 `R`。有效点数 `N` 通常远大于状态维数 23，直接求逆
`N x N` 的创新协方差代价很高。代码使用等价的信息形式：

```text
P_info = (H^T H + R P_prior^-1)^-1
K h    = P_info H^T h
K H    = P_info H^T H
```

这样主要矩阵求逆固定在 `23 x 23`，点数只影响 `H^T H` 和 `H^T h` 的线性累加成本。
这就是 FAST-LIO 能处理大量原始点、又保持高频运行的重要原因。

当前实现还有一个数据关联策略：初轮执行 kNN；若上一轮修正量较大，暂时复用对应；修正量
降到阈值后再做一次 kNN，以新对应继续检验收敛。这样兼顾稳定性和查询开销。

## 11. ikd-Tree 增量地图

普通 PCL kd-tree 适合静态点集，频繁插入后通常需要整体重建。ikd-Tree 支持：

- 单点/批量增量插入。
- kNN、半径和轴对齐盒查询。
- 体素内只保留最靠近体素中心的代表点。
- 子树级懒删除。
- 失衡或无效点比例过高时局部重建。
- 大子树后台重建，并用操作日志同步重建期间的新修改。

`map_incremental()` 只在 LiDAR 更新完成后插入当前帧，避免当前扫描先进入地图再与自己匹配。
它复用最后一轮 IEKF 的近邻结果，判断当前体素是否已有更靠近中心的点。

局部地图由 `lasermap_fov_segment()` 维护为一个滑动立方体。当 LiDAR 接近盒边缘时，盒子
向运动方向平移，离开新盒子的长条区域通过 `Delete_Point_Boxes()` 批量懒删除。这样内存和
近邻搜索规模不会随总行驶距离无限增长。

## 12. 初始化后每帧的准确时序

读 `timer_callback()` 时，可以按以下顺序打断点：

1. `sync_packages(Measures)`：是否拿到完整测量包。
2. `p_imu->Process(...)`：IMU 预测与去畸变。
3. `lasermap_fov_segment()`：滑动局部地图。
4. `downSizeFilterSurf.filter(...)`：扫描点降采样。
5. `ikdtree.Build(...)`：仅第一帧有效扫描建立地图。
6. `kf.update_iterated_dyn_share_modified(...)`：紧耦合迭代更新。
7. `h_share_model(...)`：第 6 步内部每轮回调，可观察对应、残差和 H。
8. `map_incremental()`：用最终状态插入当前扫描。

建议重点观察这些量：

| 变量 | 含义 |
| --- | --- |
| `feats_undistort` | 已统一到帧末 LiDAR 系的点云 |
| `feats_down_body` | 上述点云的体素降采样版本 |
| `feats_down_world` | 当前迭代状态下投到世界系的点 |
| `Nearest_Points[i]` | 第 i 个扫描点的地图近邻 |
| `normvec[i].intensity` | 带符号点面残差，而不是反射强度 |
| `effct_feat_num` | 本轮通过筛选的有效点面约束数 |
| `state_point` | 当前帧末的后验状态 |
| `pos_lid` | 世界系 LiDAR 位置 `p_WI + R_WI p_IL` |

## 13. 参数如何影响算法

- `mapping.gyr_cov`、`mapping.acc_cov`：IMU 白噪声。越大表示越不信任短期 IMU 传播。
- `mapping.b_gyr_cov`、`mapping.b_acc_cov`：零偏随机游走。过小会让偏置难以跟踪温漂。
- `filter_size_surf`：当前扫描体素尺寸。越小观测更多、计算更慢。
- `filter_size_map`：地图体素尺寸。越小地图更密、kNN 和内存成本更高。
- `max_iteration`：每帧 IEKF 最大迭代次数。
- `mapping.extrinsic_est_en`：是否让 LiDAR 点面观测在线修正外参。
- `mapping.det_range`、`cube_side_length`：局部地图滑窗与保留范围。
- `common.time_offset_lidar_to_imu`：LiDAR 到 IMU 的固定时间偏移。
- `common.time_sync_en`：简单软件时间对齐，只适合作为没有硬件同步时的退路。

`LASER_POINT_COV=0.001` 在源码中是所有点面观测共用的方差。它越小，LiDAR 平面约束相对
IMU 先验越强。这个值与地图噪声、点云量程误差和体素尺寸共同决定更新强度。

## 14. 最容易误解的几点

1. FAST-LIO2 的默认输入是原始点，不要求 LOAM 特征。
2. “ICP and iterated Kalman filter” 注释不代表两个串联模块；点面残差就在滤波器内部更新。
3. `curvature` 保存逐点时间，`normvec.intensity` 保存残差，两者都复用了 PCL 字段。
4. 状态是 23 自由度，不是 24 自由度；重力 3 维存储但只有 2 维可变。
5. 去畸变后的点在帧末 LiDAR 系，不是世界系；进入 `h_share_model()` 后才按迭代状态投到世界系。
6. 当前地图是滑动局部地图，不包含回环和全局一致性优化。
7. 时间同步和可靠外参往往比继续调滤波参数更重要；逐点时间错误会直接破坏去畸变。

## 15. 推荐阅读顺序

第一次读建议遵循：

```text
use-ikfom.hpp 的状态定义
 -> laserMapping.cpp::timer_callback
 -> sync_packages
 -> IMU_Processing.hpp::IMU_init / UndistortPcl
 -> use-ikfom.hpp::get_f / df_dx / df_dw
 -> laserMapping.cpp::h_share_model
 -> esekfom.hpp::update_iterated_dyn_share_modified
 -> laserMapping.cpp::map_incremental / lasermap_fov_segment
 -> ikd_Tree.cpp 的公开增删查与 Rebuild
```

按这个顺序，先掌握业务数据流，再下钻滤波模板和树内部实现，不容易被大量模板代码、旧变量名
和性能统计变量打断。
