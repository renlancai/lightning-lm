# LiDAR 点云处理流水线

## 总体架构

```
原始消息 → 预处理 → IMU同步 → 去畸变 → ESKF迭代更新 → 地图更新 → 关键帧 → 回环检测
```

核心模块调用链：

```
SlamSystem
  ├── LaserMapping (lio_)         ← LIO 前端
  │     ├── PointCloudPreprocess  ← 点云预处理
  │     ├── ImuProcess (p_imu_)   ← IMU 同步 + 去畸变
  │     ├── ESKF (kf_)            ← 状态估计
  │     └── IVox (ivox_)          ← 局部点云地图
  ├── LoopClosing (lc_)           ← 回环检测 + 位姿图优化
  ├── G2P5 (g2p5_)                ← 2D 栅格地图
  └── PangolinWindow (ui_)        ← 可视化
```

---

## 1. 消息订阅与入口

### 在线模式 (`run_slam_online.cc`)

```cpp
SlamSystem::Options options;
options.online_mode_ = true;
SlamSystem slam(options);
slam.Init(FLAGS_config);
slam.StartSLAM("new_map");
slam.Spin();  // ROS2 事件循环
```

`SlamSystem::Init()` 中建立订阅：
- `sensor_msgs::msg::PointCloud2` → `SlamSystem::ProcessLidar()`
- `livox_ros_driver2::msg::CustomMsg` → `SlamSystem::ProcessLidar()`（Livox 专用）
- `sensor_msgs::msg::Imu` → `SlamSystem::ProcessIMU()`

### 离线模式 (`run_slam_offline.cc`)

```cpp
RosbagIO rosbag(FLAGS_input_bag);
rosbag
    .AddImuHandle(imu_topic, [&slam](IMUPtr imu) {
        slam.ProcessIMU(imu); return true;
    })
    .AddPointCloud2Handle(lidar_topic, [&slam](auto msg) {
        slam.ProcessLidar(msg); return true;
    })
    .Go();  // 顺序遍历 bag，无 Spin
```

`RosbagIO::Go()` 直接读取 sqlite3 db3 文件，反序列化每条消息后调用注册回调。IMU 消息在此处转换为内部 `IMUPtr` 结构体（`linear_acceleration`, `angular_velocity`, `timestamp`）。

---

## 2. SlamSystem 消息分发

### IMU

```cpp
void SlamSystem::ProcessIMU(const lightning::IMUPtr& imu) {
    lio_->ProcessIMU(imu);  // 直接推入 imu_buffer_
}
```

### 点云

```cpp
void SlamSystem::ProcessLidar(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud) {
    lio_->ProcessPointCloud2(cloud);  // 预处理 → lidar_buffer_
    lio_->Run();                      // 触发主处理循环

    auto kf = lio_->GetKeyframe();
    if (kf != cur_kf_) {
        cur_kf_ = kf;
        if (lc_)    lc_->AddKF(cur_kf_);        // 送回环检测
        if (g2p5_)  g2p5_->PushKeyframe(cur_kf_); // 送栅格地图
        if (ui_)    ui_->UpdateKF(cur_kf_);      // 更新 UI
    }
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

同步逻辑：

```cpp
bool SyncPackages() {
    // 1. 从 lidar_buffer_ 取出待处理帧
    measures_.scan_ = lidar_buffer_.front();
    measures_.lidar_begin_time_ = time_buffer_.front();

    // 2. 计算点云结束时间
    //    = begin_time + 最后一个点的相对时间戳
    double last_pt_time = measures_.scan_->points.back().time / 1000.0;
    lidar_end_time_ = measures_.lidar_begin_time_ + last_pt_time;

    // 3. 检查 IMU 是否覆盖整个点云时间段
    if (last_timestamp_imu_ < lidar_end_time_) return false;

    // 4. 收集该时间段内所有 IMU
    measures_.imu_.clear();
    while (!imu_buffer_.empty() &&
           imu_buffer_.front()->timestamp < lidar_end_time_) {
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

---

## 6. 点云去畸变 (`UndistortPcl`)

**问题**：点云采集期间（约 100ms）传感器在运动，不同时刻的点处于不同位置，需补偿到**扫描结束时刻**。

**方法**：前向 IMU 积分记录每个 IMU 时刻的位姿，再对每个点按其时间戳反向插值补偿。

### 第一步：前向 IMU 积分

```cpp
for (auto it = v_imu.begin(); it < v_imu.end() - 1; it++) {
    double dt = tail->timestamp - head->timestamp;
    Vec3d angvel_avr = 0.5 * (head->angular_velocity + tail->angular_velocity);
    Vec3d acc_avr = 0.5 * (head->linear_acceleration + tail->linear_acceleration);

    kf_state.Predict(dt, Q_, angvel_avr, acc_avr);  // ESKF 预测

    // 保存该时刻的位姿（供后续插值）
    imu_pose_.emplace_back(Pose6D{
        dt, acc_avr, angvel_avr,
        imu_state.vel_, imu_state.pos_, imu_state.rot_.matrix()
    });
}
```

### 第二步：反向插值补偿

点云按时间戳升序排列后，从末尾向前遍历，对每个点找到对应的 IMU 时间段，计算补偿变换：

```
R_i = R_head · exp(ω_avg · dt_point)          // 点时刻的旋转
T_i = pos_head + vel_head · dt + 0.5·a·dt²    // 点时刻的位置偏移

p_compensate = R_end⁻¹ · (R_i · (R_LI · p_lidar + t_LI) + T_i - pos_end) - t_LI
              (补偿到扫描结束时刻的 LiDAR 坐标系)
```

其中 `R_LI`, `t_LI` 为 LiDAR-IMU 外参（默认为单位阵/零）。

去畸变结果保存为 `scan_undistort_`（坐标仍在 LiDAR/Body 系）。

---

## 7. ESKF 预测与更新

### 状态向量（12 维）

```
x = [pos(3), rot(3), vel(3), bg(3)]
```

`grav_`, `ba`（加速度零偏）、`offset_R/T`（外参）不在线估计，为固定量。

### 预测步

在 `UndistortPcl` 内部的 `kf_state.Predict()` 中完成：

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

调用 `ObsModel`（见第 8 节）构建观测矩阵后，迭代执行 IEKF（Iterated EKF）：

```cpp
for (int i = 0; i < maximum_iter_; i++) {
    // 1. 调用观测函数，计算 H^T H 和 H^T r
    lidar_obs_func_(x_, obs_model);

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

---

## 8. 观测模型（点面 ICP）

**`ObsModel`** 在每次 ESKF 迭代时被调用，构建点面 ICP 的 H 矩阵和残差。

### 8.1 流程

```
scan_down_body_ (Body 系)
    │
    ↓ PointBodyToWorld：p_world = R · (R_LI · p + t_LI) + t
    │
scan_down_world_ (World 系)
    │
    ↓ IVox::GetClosestPoint()：KNN 搜索 5 个最近点
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
J = [n^T,  (-R^T · n)^T × p_body]
     ↑ 对平移         ↑ 对旋转（反对称矩阵）
```

### 8.3 汇总信息矩阵

并行遍历所有有效点：

```cpp
obs.HTH_ += J^T · J;   // 6×6 Fisher 信息矩阵（累加）
obs.HTr_ += J^T · r;   // 6×1 信息向量（累加）
```

有效点数 < 20 时，`obs.valid_ = false`，跳过本次更新。

---

## 9. IVox 局部地图

### 数据结构

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

### 地图更新（`MapIncremental`）

每创建一个关键帧时调用，将 `scan_down_world_` 中的点添加到 IVox：

```cpp
ivox_->AddPoints(scan_down_world_->points);
```

若体素已存在则追加，否则新建体素，同时维护 LRU 缓存。

---

## 10. 关键帧创建

### 触发条件

```cpp
double trans = (last_kf_pose.translation() - cur_pose.translation()).norm();
double rot   = (last_kf_pose.so3().inverse() * cur_pose.so3()).log().norm();

if (trans > kf_dis_th_    // 默认 2.0 m
 || rot   > kf_angle_th_) // 默认 15°（rad）
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
1. `MapIncremental()` → 更新 IVox
2. `lc_->AddKF(kf)` → 送回环检测
3. `g2p5_->PushKeyframe(kf)` → 更新 2D 栅格地图
4. `ui_->UpdateKF(kf)` → 更新可视化

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

优化后更新所有关键帧的 `opt_pose_`。

---

## 12. 完整调用链（时序）

```
ROS2 Subscription / RosbagIO
  │
  ├─ IMU 消息
  │    └─ SlamSystem::ProcessIMU()
  │         └─ LaserMapping::ProcessIMU()
  │              └─ imu_buffer_.push_back()
  │
  └─ PointCloud2 消息
       └─ SlamSystem::ProcessLidar()
            ├─ LaserMapping::ProcessPointCloud2()
            │    ├─ PointCloudPreprocess::Process()     [格式统一 + 滤波 + 降采样]
            │    └─ lidar_buffer_.push_back()
            │
            └─ LaserMapping::Run()
                 ├─ SyncPackages()                      [IMU + 点云时间对齐]
                 ├─ ImuProcess::Process()
                 │    ├─ IMUInit() (前 N 帧)            [估计重力 + 陀螺零偏]
                 │    └─ UndistortPcl()                 [前向积分 → 反向补偿去畸变]
                 │         └─ ESKF::Predict() × N      [每个 IMU 间隔预测一次]
                 │
                 ├─ VoxelGrid 降采样                    [scan_undistort_ → scan_down_body_]
                 │
                 ├─ ESKF::Update(LIDAR)
                 │    └─ 迭代调用 ObsModel()
                 │         ├─ PointBodyToWorld()        [投影到世界系]
                 │         ├─ IVox::GetClosestPoint()   [KNN 搜索]
                 │         ├─ math::esti_plane()        [SVD 平面拟合]
                 │         ├─ 计算点面距离残差 r
                 │         └─ 累加 H^T H, H^T r
                 │    └─ 卡尔曼增益 → 状态更新 → 协方差更新
                 │
                 ├─ 关键帧判断（位移/旋转阈值）
                 │    └─ MakeKF()
                 │         └─ IVox::AddPoints()         [更新局部地图]
                 │
                 └─ 分发关键帧
                      ├─ LoopClosing::AddKF()          [回环候选 + NDT 配准 + 图优化]
                      ├─ G2P5::PushKeyframe()          [2D 栅格地图更新]
                      └─ PangolinWindow::UpdateKF()    [3D 可视化更新]
```

---

## 13. 关键数据类型汇总

| 变量名 | 类型 | 含义 |
|--------|------|------|
| `scan_` | `CloudPtr (PointXYZIT)` | 预处理后原始点云（Body 系） |
| `scan_undistort_` | `CloudPtr` | 去畸变后点云（Body 系，所有点补偿到扫描结束时刻） |
| `scan_down_body_` | `CloudPtr` | 体素降采样后点云（Body 系，用于配准） |
| `scan_down_world_` | `CloudPtr` | 投影到 World 系的降采样点云 |
| `measures_` | `MeasureGroup` | 一帧点云 + 对应 IMU 序列 |
| `state_point_` | `NavState` | ESKF 输出的当前状态（pos, rot, vel, bg） |
| `kf_` | `ESKF` | 点云帧时刻的 ESKF（主估计器） |
| `kf_imu_` | `ESKF` | IMU 最新时刻的 ESKF（高频外推用） |
| `ivox_` | `IVox<3>` | 局部点云地图（哈希体素 + LRU） |
| `all_keyframes_` | `vector<Keyframe::Ptr>` | 全局关键帧列表（前端位姿 + 点云） |
