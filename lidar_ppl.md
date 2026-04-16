# LiDAR 点云处理流水线

## 总体架构

```
原始消息 → 适配器转换 → 预处理 → IMU同步 → 去畸变 → ESKF迭代更新 → 地图更新 → 关键帧 → 回环检测
```

核心模块调用链（重构后）：

```
SlamEngine (core/engine/)         ← 平台无关 SLAM 编排器（零 ROS 依赖头文件）
  ├── LaserMapping (lio/)         ← LIO 前端
  │     ├── PointCloudPreprocess  ← 点云预处理
  │     ├── ImuProcess (p_imu_)   ← IMU 初始化 + 去畸变委托
  │     ├── LidarPipeline         ← Phase 2: 去畸变/观测/地图更新（无 ROS）
  │     │     ├── UndistortFrame  ← 从 ImuProcess::UndistortPcl 迁出
  │     │     ├── BuildObservation← 从 LaserMapping::ObsModel 迁出
  │     │     └── UpdateMap       ← 从 LaserMapping::MapIncremental 迁出
  │     ├── LocalMap              ← Phase 1: 线程安全 IVox 包装
  │     ├── ESKF (kf_)            ← 状态估计
  │     └── IVox (via LocalMap)   ← 局部点云地图
  ├── LoopClosing (lc_)           ← 回环检测 + 位姿图优化
  ├── G2P5 (g2p5_)                ← 2D 栅格地图
  └── PangolinWindow (ui_)        ← 可视化

LocEngine (core/engine/)          ← 平台无关定位编排器
  ├── Localization (loc/)         ← 定位管线（LIO + lidar_loc + PGO）
  └── GnssRtkHandler              ← Phase 4: RTK/GNSS 四层假固检测
```

---

## 0. 双库架构与 ROS 隔离

项目分为两个库：

| 库 | 类型 | ROS 依赖 | 内容 |
|----|------|---------|------|
| `lightning_core` | STATIC | 无 | 纯 C++17 算法：ESKF、LoopClosing、LidarPipeline、LocalMap、GnssRtkHandler、PGO、TiledMap 等 |
| `lightning.libs` | SHARED | 需要 | ROS 薄层：LaserMapping、PointCloudPreprocess、SlamEngine/LocEngine 实现、BagIO、UI |

**规则**：
- `adapter/ros2/ros2_converter.{h,cc}` 是**唯一**可以 include ROS 消息头文件的源文件
- `core/engine/slam_engine.h` 和 `loc_engine.h` 头文件零 ROS include
- 平台无关类型定义在 `core/types/platform_types.h`：`ImuFrame`、`PointCloudFrame`、`GnssRtkFrame`、`RawPoint`

---

## 1. 消息订阅与入口

### 在线模式 (`run_slam_online.cc`)

重构后使用 `SlamEngine` + `adapter::ros2` 转换层：

```cpp
// 创建引擎（零 ROS 头文件）
core::SlamEngine::Options eng_opts;
eng_opts.online_mode_ = true;
auto engine = std::make_shared<core::SlamEngine>(eng_opts);
engine->Init(FLAGS_config);
engine->StartMapping("new_map");

// ROS 2 订阅 → 平台无关类型 → Engine
auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, qos, [&engine](auto msg) {
        engine->FeedImu(adapter::ros2::FromImu(*msg));  // Imu → ImuFrame
    });

auto cloud_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic, qos, [&engine, time_scale](auto msg) {
        engine->FeedLidar(adapter::ros2::FromPointCloud2(*msg, time_scale));
    });
```

`SlamEngine::FeedImu()` / `FeedLidar()` 接受平台无关的 `core::ImuFrame` / `core::PointCloudFrame`，内部转换为 `IMUPtr` / `CloudPtr` 后推入 `LaserMapping`。

### 离线模式 (`run_slam_offline.cc`)

仍通过 `SlamSystem`（ROS 耦合）读取 bag：

```cpp
RosbagIO rosbag(FLAGS_input_bag);
rosbag
    .AddImuHandle(imu_topic, [&slam](IMUPtr imu) {
        slam.ProcessIMU(imu); return true;
    })
    .AddPointCloud2Handle(lidar_topic, [&slam](auto msg) {
        slam.ProcessLidar(msg); return true;
    })
    .Go();  // 顺序遍历 db3
```

### 适配器转换函数 (`adapter/ros2/ros2_converter`)

| 函数 | 输入 | 输出 |
|------|------|------|
| `FromImu()` | `sensor_msgs::msg::Imu` | `core::ImuFrame` |
| `FromPointCloud2()` | `sensor_msgs::msg::PointCloud2` | `core::PointCloudFrame` |
| `FromNavSatFix()` | `sensor_msgs::msg::NavSatFix` | `core::GnssRtkFrame` |
| `LocResultToGeoMsg()` | `LocalizationResult` | `geometry_msgs::msg::TransformStamped` |

---

## 2. SlamEngine 消息分发

### FeedImu

```cpp
void SlamEngine::FeedImu(const ImuFrame& imu) {
    auto imu_ptr = std::make_shared<IMU>();
    imu_ptr->timestamp           = imu.timestamp_sec;
    imu_ptr->linear_acceleration = imu.acc;
    imu_ptr->angular_velocity    = imu.gyro;
    lio_->ProcessIMU(imu_ptr);  // 推入 imu_buffer_，在线模式下高频外推 kf_imu_
}
```

### FeedLidar

```cpp
void SlamEngine::FeedLidar(const PointCloudFrame& frame) {
    // PointCloudFrame → PCL CloudPtr
    CloudPtr cloud(new PointCloudType());
    cloud->header.stamp = frame.timestamp_sec * 1e9;  // ns
    for (const auto& p : frame.points) {
        PointType pt;
        pt.x = p.x; pt.y = p.y; pt.z = p.z;
        pt.intensity = p.intensity;
        pt.time = p.time_offset_sec;  // 注意：这里存的是秒，不是毫秒
        cloud->push_back(pt);
    }

    lio_->ProcessPointCloud2(cloud);  // 预处理 → lidar_buffer_
    lio_->Run();                      // 主处理循环

    auto kf = lio_->GetKeyframe();
    if (kf != cur_kf_) {
        cur_kf_ = kf;
        HandleNewKeyframe(cur_kf_);  // 分发给回环/栅格/UI
    }
}
```

### HandleNewKeyframe

```cpp
void SlamEngine::HandleNewKeyframe(Keyframe::Ptr kf) {
    if (lc_)   lc_->AddKF(kf);           // 回环检测
    if (g2p5_) g2p5_->PushKeyframe(kf);  // 2D 栅格
    if (ui_)   ui_->UpdateKF(kf);        // 可视化
}
```

---

## 3. 点云预处理 (`PointCloudPreprocess`)

**输入**：`sensor_msgs::msg::PointCloud2` 或 `livox_ros_driver2::msg::CustomMsg`
**输出**：`CloudPtr`（`pcl::PointCloud<PointXYZIT>`）

```cpp
struct PointXYZIT {
    float x, y, z;
    float intensity;
    double time;   // 相对扫描起始时间，单位：毫秒（Velodyne）或微秒转换后（Livox）
};
```

各雷达的处理差异：

| 雷达类型 | `lidar_type` | 时间戳来源 | 时间单位 |
|---------|-------------|-----------|---------|
| Livox (AVIA) | 1 | `offset_time` | 微秒 → `/1e6` 秒 |
| Velodyne | 2 | `time` 字段或按 yaw 角推估 | 秒，乘以 `time_scale`（默认 `1e-3`） |
| Ouster | 3 | `t` | 纳秒 → `/1e6` |
| RoboSense | 4 | `timestamp`（Linux 绝对时间） | 秒，减去首点时间得相对值 |

通用滤波步骤（所有雷达类型）：
1. **盲区过滤**：距离 < `blind_` 的点丢弃
2. **高度 ROI 过滤**：`z < height_min_` 或 `z > height_max_` 的点丢弃
3. **降采样**：每 `point_filter_num_` 个点保留一个

Velodyne 在缺少每点时间戳时，根据 yaw 角和扫描角速度（`3.61 rad/s`）推算相对时间：

```cpp
point.time = (yaw_fp[layer] - yaw_angle) / omega_l;  // 单位：秒
```

预处理完成后，点云推入 `lidar_buffer_`，时间戳推入 `time_buffer_`。

---

## 4. IMU / 点云同步 (`SyncPackages`)

**目的**：找到完整覆盖一帧点云时间段 `[lidar_begin_time, lidar_end_time]` 的 IMU 序列。

```
lidar_begin_time ←────── lidar scan ──────→ lidar_end_time
     │ imu_0 │ imu_1 │ imu_2 │ imu_3 │ imu_4 │
     ↑                                         ↑
  last_imu_                          must have IMU past here
```

同步逻辑（`LaserMapping::SyncPackages`）：

```cpp
bool SyncPackages() {
    // 1. 从 lidar_buffer_ 取出待处理帧
    measures_.scan_ = lidar_buffer_.front();
    measures_.lidar_begin_time_ = time_buffer_.front();

    // 2. 计算点云结束时间
    //    = begin_time + 最后一个点的相对时间戳
    double last_pt_time = measures_.scan_->points.back().time / 1000.0;
    lidar_end_time_ = measures_.lidar_begin_time_ + last_pt_time;

    // 3. 异常时间戳保护：超过 5 倍平均扫描时间 → 用均值代替
    if ((lidar_end_time_ - measures_.lidar_begin_time_) > 5 * lo::lidar_time_interval) {
        lidar_end_time_ = measures_.lidar_begin_time_ + lo::lidar_time_interval;
    }

    // 4. 更新运行时扫描时间间隔（Phase 3: 传给 LidarPipeline）
    lo::lidar_time_interval = lidar_mean_scantime_;
    lidar_pipeline_->SetLidarTimeInterval(lidar_mean_scantime_);

    // 5. 检查 IMU 是否覆盖整个点云时间段
    if (last_timestamp_imu_ < lidar_end_time_) return false;

    // 6. 收集该时间段内所有 IMU
    measures_.imu_.clear();
    while (!imu_buffer_.empty() && imu_buffer_.front()->timestamp < lidar_end_time_) {
        measures_.imu_.push_back(imu_buffer_.front());
        imu_buffer_.pop_front();
    }
    return true;
}
```

同步结果存入 `MeasureGroup`：

```cpp
struct MeasureGroup {
    double lidar_begin_time_;
    double lidar_end_time_;
    std::deque<IMUPtr> imu_;   // 覆盖该帧的 IMU 序列
    CloudPtr scan_;            // 预处理后的点云
};
```

---

## 5. IMU 初始化 (`IMUInit`)

前 `max_init_count_`（约 20）帧不做 SLAM，只积累 IMU 数据：

```cpp
mean_acc_ += (imu->linear_acceleration - mean_acc_) / N;
mean_gyr_ += (imu->angular_velocity - mean_gyr_) / N;
```

初始化完成时：
- **重力向量**：`grav_ = -mean_acc_.normalized() * 9.81`（从加速度均值推估初始姿态）
- **陀螺零偏**：`bg_ = mean_gyr_`（静止时角速度即为零偏）
- **加速度计尺度因子**：`acc_scale_factor_` 根据均值范数判断（`0.5~1.5` → `9.81`，`7~12` → `1.0`）
- **初始化 LidarPipeline 状态**（Phase 2）：`pipeline_->SetAccScaleFactor()` + `pipeline_->ResetUndistortState()`

---

## 6. 点云去畸变 (`LidarPipeline::UndistortFrame`)

Phase 2 重构：从 `ImuProcess::UndistortPcl` 迁移至 `LidarPipeline::UndistortFrame`。

**调用链**：
```
ImuProcess::Process()
  └─ if (pipeline_)  pipeline_->UndistortFrame(meas, kf_state, scan, Q_)
     else            UndistortPcl(meas, kf_state, scan)  // fallback
```

**问题**：点云采集期间（约 100ms）传感器在运动，不同时刻的点处于不同位置，需补偿到**扫描结束时刻**。

**方法**：前向 IMU 积分记录每个 IMU 时刻的位姿，再对每个点按其时间戳反向插值补偿。

### 第一步：前向 IMU 积分

```cpp
for (auto it = v_imu.begin(); it < v_imu.end() - 1; it++) {
    double dt = tail->timestamp - head->timestamp;
    Vec3d angvel_avr = 0.5 * (head->angular_velocity + tail->angular_velocity);
    Vec3d acc_avr = 0.5 * (head->linear_acceleration + tail->linear_acceleration);
    acc_avr = acc_avr * acc_scale_factor_;  // 尺度修正

    kf_state.Predict(dt, Q, gyro, acc);  // ESKF 预测

    // 保存该时刻的位姿（供后续插值）
    imu_pose_.emplace_back(Pose6D{
        dt, acc_s_last_, angvel_last_,
        imu_state.vel_, imu_state.pos_, imu_state.rot_.matrix()
    });
}
```

### 第二步：反向插值补偿

点云按时间戳升序排列后，从末尾向前遍历，对每个点找到对应的 IMU 时间段，计算补偿变换：

```
R_i = R_head · exp(ω_avg · dt_point)          // 点时刻的旋转
T_ei = pos_head + vel_head · dt + 0.5·a·dt² - pos_end  // 点时刻的位置偏移

p_compensate = R_LI⁻¹ · (R_end⁻¹ · (R_i · (R_LI · p_lidar + t_LI) + T_ei) - t_LI)
              (补偿到扫描结束时刻的 LiDAR 坐标系)
```

其中 `R_LI`, `t_LI` 为 LiDAR-IMU 外参（默认为单位阵/零）。`dt` 通过 `lidar_time_interval_`（Phase 3: 运行时由 LaserMapping 估算并注入）保护防止异常值。

去畸变结果保存为 `scan_undistort_`（坐标仍在 LiDAR/Body 系）。

---

## 7. ESKF 预测与更新

### 状态向量（12 维）

```
x = [pos(3), rot(3), vel(3), bg(3)]
```

`grav_`, `ba`（加速度零偏）、`offset_R/T`（外参）不在线估计，为固定量。

### 预测步

在 `LidarPipeline::UndistortFrame` 内部的 `kf_state.Predict()` 中完成：

```cpp
// 运动方程（连续时间）：
pos' = vel
rot' = [ω - bg]×   (角速度减零偏)
vel' = R · a + g    (重力补偿后的加速度积分)
bg'  = 0            (随机游走模型)

// 协方差传播：
F = I + f_x · dt
P = F · P · F^T + (dt · f_w) · Q · (dt · f_w)^T
```

### 更新步（`ESKF::Update`）

调用 `ObsModel`（见第 8 节，Phase 2 后委托给 `LidarPipeline::BuildObservation`）构建观测矩阵后，迭代执行 IEKF（Iterated EKF）：

```cpp
for (int i = 0; i < maximum_iter_; i++) {
    // 1. 调用观测函数，计算 H^T H 和 H^T r
    lidar_obs_func_(x_, obs_model);  // → LidarPipeline::BuildObservation

    // 2. 特征值分解，检测退化方向（如走廊）
    Eigen::SelfAdjointEigenSolver<Mat6d> es(HTH_sym);
    // 低特征值方向 → nullity，对应退化，膨胀协方差

    // 3. 卡尔曼更新
    // P_temp = P⁻¹/R + H^T H（信息矩阵）
    // dx = P_temp⁻¹ · (H^T r + H^T H · dx_prev)

    // 4. 状态更新（Manifold 运算）
    x_ = x_.boxplus(dx);

    // 5. 收敛判断
    if (||dx||_trans < eps_trans && ||dx||_rot < eps_rot) break;
}
// 更新协方差：Joseph 形式
P_ = (I - K·H) · P
```

Phase 3: `maximum_iter_` (`num_max_iterations_`) 和 `esti_plane_threshold_` 不再是全局 `extern` 变量，而是从 YAML 读取后传入 ESKF 和 LidarPipeline。

---

## 8. 观测模型（`LidarPipeline::BuildObservation`）

Phase 2 重构：从 `LaserMapping::ObsModel` 迁移至 `LidarPipeline::BuildObservation`。

调用链：`ESKF::Update` → `lidar_obs_func_` → `LaserMapping::ObsModel` → `LidarPipeline::BuildObservation`

### 8.1 流程

```
scan_down_body_ (Body 系)
    │
    ↓ 并行转换：p_world = R · (R_LI · p + t_LI) + t
    │
scan_down_world_ (World 系)
    │
    ↓ LocalMap::GetClosestPoint()：KNN 搜索 5 个最近点
    │
    ↓ math::esti_plane()：SVD 拟合平面 n^T · x + d = 0
    │
    ↓ 计算点到平面距离 r = n^T · p_world + d
    │
    ↓ 加权判断：有效点满足 ||p_body||² > 81 · r²（距离越远容差越小）
    │
    ↓ 构建雅可比矩阵 J
```

### 8.2 雅可比矩阵推导

对位姿 `[t, R]` 求导，点面 ICP 的 1×6 雅可比为：

```
J = [n^T,  (point_crossmat · R^T · n)^T]
     ↑ 对平移         ↑ 对旋转（反对称矩阵）
```

### 8.3 汇总信息矩阵

并行遍历所有有效点（`std::execution::par_unseq`）：

```cpp
obs.HTH_ += J^T · J · plane_icp_weight_;   // 6×6 Fisher 信息矩阵（累加）
obs.HTr_ += J^T · r · plane_icp_weight_;   // 6×1 信息向量（累加）
```

### 8.4 点点 ICP 补充（可选，`enable_icp_part_`）

当 `enable_icp_part_ = true` 时，额外添加 3D 点点 ICP 项：

```cpp
// 3×6 雅可比
J.block<3,3>(0,0) = I;
J.block<3,3>(0,3) = -(R · R_LI) · hat(q);

// 残差
e = q_world - nearest_point[0];

obs.HTH_ += J^T · J · icp_weight_;
obs.HTr_ += -J^T · e · icp_weight_;
```

有效点数 < 20 时，`obs.valid_ = false`，跳过本次更新。

---

## 9. LocalMap 与 IVox 局部地图

### LocalMap（Phase 1: 线程安全包装）

```cpp
class LocalMap {
    // 写操作 → unique_lock
    void AddPoints(const PointVector& pts);
    // 读操作 → shared_lock
    bool GetClosestPoint(const PointType& pt, PointVector& neighbors, ...) const;
    size_t NumValidGrids() const;

    mutable std::shared_mutex rw_mutex_;
    std::shared_ptr<IVoxType> ivox_;
};
```

### IVox 数据结构

```cpp
IVox<3, IVoxNodeType::DEFAULT, PointType>
```

- **哈希表** `grids_map_`：体素坐标 Key → 体素迭代器
- **LRU 链表** `grids_cache_`：缓存淘汰策略，容量超过 `capacity_`（默认 100 万）时删除最久未用体素

体素坐标：`key = round(pos / resolution)`

### 最近邻搜索

```cpp
bool GetClosestPoint(const PointType& pt, PointVector& closest_pt, int max_num = 5) {
    // 1. 计算所属体素
    auto key = Pos2Grid(pt);
    // 2. 搜索邻域体素（NEARBY18 = 19 个体素）
    for (const auto& delta : nearby_grids_) {
        auto iter = grids_map_.find(key + delta);
        if (iter != grids_map_.end())
            iter->second->KNNPointByCondition(candidates, pt, max_num, max_range);
    }
    // 3. 取前 max_num 个最近点
    std::nth_element(...);
}
```

邻域类型（配置 `ivox_nearby_type`）：

| 值 | 类型 | 体素数 |
|----|------|--------|
| 0 | CENTER | 1 |
| 6 | NEARBY6 | 7 |
| 18 | NEARBY18 | 19 |
| 26 | NEARBY26 | 27 |

### 地图更新（`LidarPipeline::UpdateMap`）

Phase 2 重构：从 `LaserMapping::MapIncremental` 迁移至 `LidarPipeline::UpdateMap`。

调用链：`LaserMapping::MapIncremental` → `lidar_pipeline_->UpdateMap(...)`

对 `scan_down_body_` 中每个点：
1. 转换到世界坐标系
2. 计算体素中心，与最近邻比较决定是否需要添加
3. 按 `PointAction` 分类：`Add`（需降采样添加）、`NoNeedDownsample`（直接添加）
4. 批量写入 `LocalMap::AddPoints()`

---

## 10. 关键帧创建

### 触发条件

```cpp
double trans = (last_kf_pose.translation() - cur_pose.translation()).norm();
double rot   = (last_kf_pose.so3().inverse() * cur_pose.so3()).log().norm();

if (trans > kf_dis_th_    // 默认 2.0 m
 || rot   > kf_angle_th_) // 默认 15°（rad）
    MakeKF();
// 定位模式下，超过 2 秒也会强制建帧
if (!is_in_slam_mode_ && (cur_time - last_kf_time) > 2.0)
    MakeKF();
```

### 关键帧内容

```cpp
struct Keyframe {
    int id_;
    CloudPtr cloud_;       // scan_undistort_（去畸变的原始密集点云）
    NavState lio_state_;   // ESKF 输出的状态（位姿 + 速度 + 零偏）
    SE3 lio_pose_;         // LIO 估计位姿（前端）
    SE3 opt_pose_;         // 图优化后位姿（后端更新）
};
```

关键帧创建后依次触发：
1. `MapIncremental()` → `LidarPipeline::UpdateMap()` → 更新 LocalMap/IVox
2. `SlamEngine::HandleNewKeyframe()`:
   - `lc_->AddKF(kf)` → 送回环检测
   - `g2p5_->PushKeyframe(kf)` → 更新 2D 栅格地图
   - `ui_->UpdateKF(kf)` → 更新可视化

---

## 11. 回环检测

### 候选帧检测

每隔 `loop_kf_gap_`（默认 50）个关键帧触发一次检测：

```cpp
for (auto kf : all_keyframes_) {
    // 条件 1：ID 间隔 > closest_id_th_（50）
    if (|kf.id - cur_kf.id| < closest_id_th_) break;
    // 条件 2：XY 平面距离 < max_range_（30 m）
    if ((kf.opt_pose.t - cur_kf.opt_pose.t).head<2>().norm() < max_range_)
        candidates.push_back(kf);
}
```

### 候选配准（多分辨率 NDT）

子地图构建：取候选帧周围 ±40 个关键帧的点云拼合。

```cpp
std::vector<double> resolutions{10.0, 5.0, 2.0, 1.0};
for (auto r : resolutions) {
    ndt.setResolution(r);
    ndt.align(*output, Tw2);
    Tw2 = ndt.getFinalTransformation();
    c.ndt_score_ = ndt.getTransformationProbability();
}
```

NDT 分数 > `ndt_score_th_`（默认 1.0）的候选进入图优化。

### 位姿图优化（`miao` 库）

使用内置轻量图优化库 `miao`（`src/core/miao/`）：

- **顶点**：所有关键帧的 SE3 位姿
- **运动边**：相邻关键帧的 LIO 相对位姿（高权重）
- **回环边**：NDT 配准的相对位姿（低权重）
- **RTK 先验边**（Phase 4）：`EdgePositionPrior` — 3D 位置先验约束

优化后更新所有关键帧的 `opt_pose_`。

---

## 12. RTK/GNSS 融合（Phase 4）

### GnssRtkHandler — 四层假固检测

```cpp
class GnssRtkHandler {
    // 四层过滤：
    // 1. SolutionType 检查：只接受 FIX 和 FLOAT
    // 2. 精度检查：pos_std_enu 各轴 < 阈值
    // 3. HDOP / 卫星数检查
    // 4. 卡方一致性检查（与 PGO 已有约束对比）
    bool Accept(const GnssRtkFrame& obs);
};
```

### PGO 集成

- `PGOImpl` 内维护 RTK 观测队列
- `AddRtkFactors()`: 为每个有效 RTK 观测添加 `EdgePositionPrior`（3×6 Jacobian）
- `UpdateRtkChi2()`: 更新卡方统计量用于假固检测
- `enu_to_map_`: ENU → Map 坐标变换（从首个 FIX 解算）
- `AssignRtkObsIfNeeded()`: 将 RTK 观测分配到最近的关键帧

---

## 13. 完整调用链（时序）

```
ROS2 Subscription / RosbagIO
  │
  ├─ IMU 消息
  │    └─ SlamEngine::FeedImu(ImuFrame)
  │         └─ LaserMapping::ProcessIMU(IMUPtr)
  │              ├─ imu_buffer_.push_back()
  │              └─ kf_imu_.Predict()  [在线模式：高频外推 100Hz]
  │
  └─ PointCloud2 消息
       └─ SlamEngine::FeedLidar(PointCloudFrame)
            ├─ PointCloudFrame → CloudPtr 转换
            ├─ LaserMapping::ProcessPointCloud2(cloud)
            │    ├─ PointCloudPreprocess::Process()     [格式统一 + 滤波 + 降采样]
            │    └─ lidar_buffer_.push_back()
            │
            └─ LaserMapping::Run()
                 ├─ SyncPackages()                      [IMU + 点云时间对齐]
                 │    └─ lo::lidar_time_interval 更新
                 │    └─ lidar_pipeline_->SetLidarTimeInterval()
                 │
                 ├─ ImuProcess::Process()
                 │    ├─ IMUInit() (前 20 帧)            [估计重力 + 陀螺零偏 + acc scale]
                 │    │    └─ pipeline_->ResetUndistortState()
                 │    └─ LidarPipeline::UndistortFrame() [前向积分 → 反向补偿去畸变]
                 │         └─ ESKF::Predict() × N       [每个 IMU 间隔预测一次]
                 │
                 ├─ VoxelGrid 降采样                    [scan_undistort_ → scan_down_body_]
                 │
                 ├─ ESKF::Update(LIDAR)
                 │    └─ 迭代调用 ObsModel()
                 │         └─ LidarPipeline::BuildObservation()
                 │              ├─ 并行 Body→World 投影
                 │              ├─ LocalMap::GetClosestPoint()   [KNN 搜索]
                 │              ├─ math::esti_plane()            [SVD 平面拟合]
                 │              ├─ 计算点面距离残差 r
                 │              ├─ 累加 H^T H, H^T r (点面 ICP)
                 │              └─ [可选] 累加点点 ICP 项
                 │    └─ 卡尔曼增益 → 状态更新 → 协方差更新
                 │
                 ├─ 关键帧判断（位移/旋转阈值）
                 │    └─ MakeKF()
                 │         └─ LidarPipeline::UpdateMap()   [更新 LocalMap/IVox]
                 │
                 ├─ kf_imu_ 前推到最新 IMU 时刻
                 │
                 └─ SlamEngine::HandleNewKeyframe()
                      ├─ LoopClosing::AddKF()          [回环候选 + NDT 配准 + 图优化]
                      ├─ G2P5::PushKeyframe()          [2D 栅格地图更新]
                      └─ PangolinWindow::UpdateKF()    [3D 可视化更新]
```

---

## 14. 关键数据类型汇总

| 变量名 | 类型 | 含义 | 所属 |
|--------|------|------|------|
| `scan_` | `CloudPtr (PointXYZIT)` | 预处理后原始点云（Body 系） | MeasureGroup |
| `scan_undistort_` | `CloudPtr` | 去畸变后点云（Body 系，所有点补偿到扫描结束时刻） | LaserMapping |
| `scan_down_body_` | `CloudPtr` | 体素降采样后点云（Body 系，用于配准） | LaserMapping |
| `scan_down_world_` | `CloudPtr` | 投影到 World 系的降采样点云 | LaserMapping |
| `measures_` | `MeasureGroup` | 一帧点云 + 对应 IMU 序列 | LaserMapping |
| `state_point_` | `NavState` | ESKF 输出的当前状态（pos, rot, vel, bg） | LaserMapping |
| `kf_` | `ESKF` | 点云帧时刻的 ESKF（主估计器） | LaserMapping |
| `kf_imu_` | `ESKF` | IMU 最新时刻的 ESKF（高频外推用） | LaserMapping |
| `local_map_` | `shared_ptr<LocalMap>` | 线程安全 IVox 包装（Phase 1） | LaserMapping |
| `lidar_pipeline_` | `shared_ptr<LidarPipeline>` | 去畸变/观测/地图更新管线（Phase 2） | LaserMapping |
| `all_keyframes_` | `vector<Keyframe::Ptr>` | 全局关键帧列表（前端位姿 + 点云） | LaserMapping |

### 平台无关类型（`core/types/platform_types.h`）

| 类型 | 字段 | 用途 |
|------|------|------|
| `ImuFrame` | `timestamp_sec`, `acc`, `gyro` | Engine FeedImu 输入 |
| `PointCloudFrame` | `timestamp_sec`, `seq`, `points[]` | Engine FeedLidar 输入 |
| `RawPoint` | `x, y, z, intensity, time_offset_sec` | PointCloudFrame 内的点 |
| `GnssRtkFrame` | `timestamp_sec`, `solution_type`, `lat/lon/alt`, `pos_std_enu` | Engine FeedRtk 输入 |

---

## 15. Phase 重构变更摘要

| Phase | 变更 | 涉及文件 |
|-------|------|---------|
| Phase 1 | LocalMap 线程安全包装 | `core/lidar/local_map.{h,cc}` |
| Phase 2 | LidarPipeline 提取 | `core/lidar/lidar_pipeline.{h,cc}` |
| Phase 3 | 消除 extern 全局变量 | `common/options.h`, `LaserMapping`, `LidarPipeline` |
| Phase 4 | RTK/GNSS 集成 | `core/gnss/gnss_rtk_handler.{h,cc}`, `pgo_impl.{h,cc}`, `edge_position_prior.h` |
| Phase 5 | Engine API + 平台无关验证 | `core/engine/slam_engine.{h,cc}`, `loc_engine.{h,cc}`, `adapter/ros2/ros2_converter.{h,cc}`, `core/types/platform_types.h` |
