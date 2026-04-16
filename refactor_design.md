# Lightning-LM 重构设计文档

## 1. 重构目标与约束

### 1.1 三项核心需求

| 需求 | 当前痛点 | 目标 |
|------|---------|------|
| LiDAR 处理封装 | 去畸变散在 `ImuProcess`，观测构建散在 `LaserMapping`，地图管理无统一接口 | 独立 `LidarPipeline` 类，零 ROS 依赖，可直接剥离 |
| 平台无关 | `#include <sensor_msgs/...>` 侵入核心算法层，全局 extern 变量跨模块耦合 | 核心层纯 C++17，ROS/非ROS 通过 Adapter 接入 |
| RTK/GNSS 量测 | `PGOFrame` 中 RTK 字段全部注释；无假固定检测；无坐标系转换 | 完整 RTK 量测通路 + 多层假固定检测 + 卡方异常拒绝 |

### 1.2 不在本次重构范围内

- 算法本身（ESKF 数学、IVox 结构、miao 优化器）不修改
- 界面（Pangolin UI）不变
- 地图文件格式不变

---

## 2. 整体分层架构

### 2.1 目标架构

```
┌──────────────────────────────────────────────────────────────┐
│  Platform Adapter Layer  (src/adapter/)                       │
│                                                               │
│  ROS2Adapter  │  ROS1Adapter  │  FileAdapter  │  BagAdapter  │
│  ─────────────────────────────────────────────────────────── │
│  输入：ROS msgs / bag / 自定义流                               │
│  输出：平台无关数据帧 (PointCloudFrame / ImuFrame / RtkFrame)  │
└──────────────────────────┬───────────────────────────────────┘
                           │ 平台无关纯数据类型
                           ▼
┌──────────────────────────────────────────────────────────────┐
│  Core Engine API  (src/core/)          零 ROS 依赖            │
│                                                               │
│  SlamEngine          │  LocEngine                            │
│  ─────────────────────────────────────────────────────────── │
│  统一对外接口：FeedImu() / FeedLidar() / FeedRtk()            │
└──┬──────────┬───────────┬──────────┬──────────┬─────────────┘
   │          │           │          │          │
   ▼          ▼           ▼          ▼          ▼
LidarPipeline LocalMap  ESKF     FusionPGO  GnssRtkHandler
(本文档核心)  (IVox+    (不变)   (扩展RTK)  (新增)
              TiledMap)
```

### 2.2 当前层级映射关系

| 当前文件 | 重构后归属 | 改动类型 |
|---------|----------|---------|
| `wrapper/bag_io.cc` | `adapter/ros2/bag_adapter.cc` | 移动，接口不变 |
| `wrapper/ros_utils.h` | `adapter/ros2/ros2_converter.h` | 移动 |
| `core/system/slam.cc` | `core/engine/slam_engine.cc` | 重写（去 ROS 依赖） |
| `core/system/loc_system.cc` | `core/engine/loc_engine.cc` | 重写 |
| `core/lio/laser_mapping.cc` | 拆分为 `LidarPipeline` + `LocalMap` | 拆分 |
| `core/lio/imu_processing.hpp` | 并入 `LidarPipeline` | 合并 |
| `core/lio/pointcloud_preprocess.cc` | 移入 `adapter/` 层 | 移动 |
| `core/localization/pose_graph/pgo_impl.cc` | 扩展 RTK 支持 | 扩展 |
| `common/options.h` (全局 extern) | 消除，改为各类 Options struct | 重构 |

---

## 3. 平台无关数据类型（新增）

文件：`src/core/types/platform_types.h`

```cpp
#pragma once
#include "common/eigen_types.h"
#include <vector>
#include <cstdint>

namespace lightning::core {

// ─── LiDAR 原始点（格式统一后，不含 ROS 类型）───────────────────
struct RawPoint {
    float x, y, z;
    float intensity;
    double time_offset_sec;  // 相对本帧起始时间，单位秒
};

// ─── LiDAR 帧 ─────────────────────────────────────────────────
struct PointCloudFrame {
    std::vector<RawPoint> points;
    double timestamp_sec = 0;    // 帧起始时间（主机时钟，Unix 秒）
    uint64_t seq = 0;
};

// ─── IMU 帧 ───────────────────────────────────────────────────
struct ImuFrame {
    double timestamp_sec = 0;
    Vec3d acc;   // m/s²，Body 系
    Vec3d gyro;  // rad/s，Body 系
};

// ─── GNSS RTK 帧 ──────────────────────────────────────────────
struct GnssRtkFrame {
    double timestamp_sec = 0;

    enum class SolutionType : uint8_t {
        INVALID = 0,
        SPP     = 1,   // 单点定位
        SBAS    = 2,
        FLOAT   = 4,   // RTK 浮点解
        FIX     = 5,   // RTK 固定解
    };

    SolutionType solution_type = SolutionType::INVALID;
    int    num_satellites    = 0;
    double hdop              = 99.0;
    double age_of_corr_sec   = 0.0;  // 差分龄期

    // 位置：WGS84（lat/lon/alt，角度制）
    double lat_deg = 0, lon_deg = 0, alt_m = 0;
    Vec3d  pos_std_enu;   // 位置标准差，ENU 顺序，单位 m（来自接收机 covariance）

    // 双天线航向（可选）
    bool   heading_valid  = false;
    double heading_rad    = 0;
    double heading_std_rad = 0;
};

}  // namespace lightning::core
```

---

## 4. LidarPipeline 类设计（核心改动）

### 4.1 职责边界

| 职责 | 当前位置 | 重构后位置 |
|------|---------|----------|
| 去畸变（前向积分 + 反向补偿） | `ImuProcess::UndistortPcl` | `LidarPipeline::UndistortFrame` |
| 降采样 | `LaserMapping::Run` 内 inline | `LidarPipeline::Downsample` |
| 点面 ICP 雅可比构建 | `LaserMapping::ObsModel` | `LidarPipeline::BuildObservation` |
| 点点 ICP 雅可比构建 | `LaserMapping::ObsModel` | `LidarPipeline::BuildObservation` |
| IVox 地图增量更新 | `LaserMapping::MapIncremental` | `LocalMap::Update` |
| IVox KNN 查询 | `LaserMapping::ObsModel` 内直接调用 | `LocalMap::QueryKNN` |

**注意**：`LidarPipeline` 不持有 ESKF，不决定关键帧，只做点云处理和观测构建。与 ESKF 的交互通过回调函数注入，保持单向依赖。

### 4.2 接口设计

文件：`src/core/lidar/lidar_pipeline.h`

```cpp
#pragma once
#include "core/types/platform_types.h"
#include "core/lio/eskf.hpp"
#include "core/lio/pose6d.h"
#include "core/ivox3d/ivox3d.h"
#include "common/nav_state.h"
#include <functional>

namespace lightning::core {

// ─── 单帧去畸变后的点云（内部流通格式）──────────────────────────
struct UndistortedFrame {
    CloudPtr cloud_body;    // Body 系，所有点已补偿到帧尾时刻
    CloudPtr cloud_down_body;   // 降采样后，Body 系
    double   timestamp_sec = 0; // 帧尾时间
};

// ─── ObsModel 输出 ──────────────────────────────────────────────
struct ObservationResult {
    bool   valid         = false;
    Mat6d  HTH;                  // 信息矩阵（6×6），由各点 J^T J 累加
    Vec6d  HTr;                  // 信息向量（6×1），由各点 J^T r 累加
    double residual_median = 0;  // 用于 IEKF 收敛判断
    double residual_max    = 0;
    int    num_surf        = 0;  // 有效点面约束数
    int    num_point       = 0;  // 有效点点约束数
};

// ════════════════════════════════════════════════════════════════
class LidarPipeline {
public:
    struct Options {
        // 去畸变
        double imu_dt_max = 0.1;         // 超过此 dt 的 IMU 间隔报警

        // 降采样
        float voxel_size_scan = 0.5;

        // 地图增量更新
        float voxel_size_map = 0.5;

        // 观测模型
        bool  enable_point_plane  = true;
        bool  enable_point_point  = false;
        float plane_icp_weight    = 1.0;
        float point_icp_weight    = 100.0;
        float plane_threshold     = 0.1;   // 平面拟合门限
        int   min_effective_surfs = 20;    // 有效点面数量下限
        int   num_nearest_pts     = 5;     // KNN 数
        int   min_nearest_pts     = 3;     // 有效平面最小点数
        float max_point_dist      = 0.5;   // 点点约束最大距离

        // 外参（LiDAR → IMU/Body）
        Vec3d  t_lidar_imu = Vec3d::Zero();
        Mat3d  R_lidar_imu = Mat3d::Identity();
    };

    explicit LidarPipeline(Options options = Options());

    // ── 喂入 IMU（在处理点云前调用，内部维护 IMU buffer）──────────
    void FeedImu(const ImuFrame& imu);

    // ── Step 1：同步 IMU + 点云帧，返回是否同步成功 ───────────────
    // 成功后内部 ready_ = true，可调用后续接口
    bool Sync(const PointCloudFrame& raw_frame, ESKF& kf);

    // ── Step 2：去畸变 + 降采样 ────────────────────────────────────
    // 调用前须已 Sync()，内部完成前向 IMU 积分 + 反向点补偿
    // out_frame.cloud_body：去畸变原始密度点云
    // out_frame.cloud_down_body：降采样后点云（用于配准）
    bool Process(ESKF& kf, UndistortedFrame& out_frame);

    // ── Step 3：观测构建（ESKF 每次迭代均调用）─────────────────────
    // state：当前 IEKF 线性化点
    // frame：Process() 输出的帧
    // local_map：LocalMap 引用，内部做 KNN 查询
    ObservationResult BuildObservation(const NavState&         state,
                                       const UndistortedFrame& frame,
                                       LocalMap&               local_map);

    // ── Step 4：更新 Local Map（ESKF 收敛后调用）──────────────────
    void UpdateMap(const UndistortedFrame& frame,
                   const NavState&         final_state,
                   LocalMap&               local_map);

    // 获取上次同步的时间信息（供外部判断关键帧）
    double GetLastFrameEndTime() const { return last_frame_end_time_; }
    double GetLastFrameBeginTime() const { return last_frame_begin_time_; }

    // 重置（清空 IMU buffer，重新初始化）
    void Reset();

    // 更新外参
    void SetExtrinsic(const Vec3d& t, const Mat3d& R);

private:
    Options options_;

    // IMU buffer（用于去畸变）
    std::deque<ImuFrame>  imu_buffer_;
    ImuFrame              last_imu_;
    double                last_lidar_end_time_ = 0;

    // 前向积分结果（每帧重建）
    std::vector<Pose6D> imu_poses_;

    // 同步后的数据
    PointCloudFrame pending_frame_;
    bool            ready_ = false;
    double          last_frame_begin_time_ = 0;
    double          last_frame_end_time_   = 0;

    // 工作缓冲（避免每帧分配）
    std::vector<PointVector> nearest_points_;
    std::vector<float>       residuals_;
    std::vector<Vec4f>       plane_coefs_;
    std::vector<bool>        selected_surf_;
    std::vector<bool>        selected_icp_;

    // 内部方法
    bool SyncImu(double lidar_end_time, std::vector<ImuFrame>& out_imu_seq);
    void ForwardPropagate(const std::vector<ImuFrame>& imu_seq,
                          ESKF& kf,
                          double lidar_begin_time,
                          double lidar_end_time);
    void BackwardUndistort(const std::vector<ImuFrame>& imu_seq,
                           const NavState& end_state,
                           CloudPtr& cloud);
};

}  // namespace lightning::core
```

### 4.3 LocalMap 类设计

文件：`src/core/lidar/local_map.h`

```cpp
namespace lightning::core {

class LocalMap {
public:
    struct Options {
        float   ivox_resolution  = 0.5;
        int     ivox_nearby_type = 18;     // 6 / 18 / 26
        size_t  capacity         = 1000000;
        float   voxel_size_map   = 0.5;    // 地图增量时的点云分辨率
    };

    using IVoxType = IVox<3, IVoxNodeType::DEFAULT, PointType>;

    explicit LocalMap(Options options = Options());

    // KNN 查询（线程安全，供 ObsModel 并行调用）
    bool QueryKNN(const PointType& query,
                  PointVector&     neighbors,
                  int              k         = 5,
                  double           max_range = 5.0) const;

    // 增量添加点云（World 系）
    void AddPoints(const PointVector& pts);

    // 清空地图
    void Reset();

    // 有效体素数
    int NumValidGrids() const;

private:
    Options     options_;
    IVoxType    ivox_;
    mutable std::shared_mutex rw_mutex_;   // 读多写少，用读写锁
};

}  // namespace lightning::core
```

**关键改动**：`LocalMap` 的读写锁解决了原版 `FIXME 这并发可能有点问题` 的数据竞争——`QueryKNN` 获取读锁，`AddPoints` 获取写锁。

---

## 5. Platform Adapter 层设计

### 5.1 目录结构

```
src/adapter/
├── common/
│   └── coordinate_utils.h       # WGS84 ↔ ECEF ↔ ENU 坐标转换
├── ros2/
│   ├── ros2_converter.h          # ROS2 消息 → 平台无关类型
│   ├── ros2_converter.cc
│   ├── bag_adapter.h             # 替代现有 bag_io.h
│   └── bag_adapter.cc
└── file/
    └── pcd_adapter.h             # 读取离线 PCD（非 ROS 场景）
```

### 5.2 ROS2 转换器

文件：`src/adapter/ros2/ros2_converter.h`

```cpp
namespace lightning::adapter::ros2 {

class Ros2Converter {
public:
    // ── LiDAR → PointCloudFrame ──────────────────────────────────
    // type: 1=Livox, 2=Velodyne, 3=Ouster, 4=RoboSense
    static core::PointCloudFrame FromPointCloud2(
        const sensor_msgs::msg::PointCloud2& msg,
        int     lidar_type,
        double  blind_dist,
        double  time_scale,       // Velodyne 时间戳缩放
        float   height_min,
        float   height_max,
        int     point_filter_num);

    static core::PointCloudFrame FromLivox(
        const livox_ros_driver2::msg::CustomMsg& msg,
        double  blind_dist,
        float   height_min,
        float   height_max,
        int     point_filter_num);

    // ── IMU ─────────────────────────────────────────────────────
    static core::ImuFrame FromImu(const sensor_msgs::msg::Imu& msg);

    // ── RTK：支持 Novatel、ublox、nmea 等常见消息 ─────────────────
    // (以下为示例，实际按所用接收机添加)
    static core::GnssRtkFrame FromNovatelBestPos(
        const novatel_oem7_msgs::msg::BESTPOS& msg);

    // 通用：从标准 NavSatFix（不含 heading，solution_type 仅 SPP/FIX）
    static core::GnssRtkFrame FromNavSatFix(
        const sensor_msgs::msg::NavSatFix& msg);

    // ── 时间戳 ───────────────────────────────────────────────────
    static double ToSec(const builtin_interfaces::msg::Time& t) {
        return t.sec + 1e-9 * t.nanosec;
    }
};

}  // namespace lightning::adapter::ros2
```

### 5.3 去除全局 extern 变量

现有 `options.h` 中存在大量 `namespace lo / fasterlio / pgo` 下的 `extern` 全局变量，例如：

```cpp
// 旧（反模式）
namespace lo {
    extern float lidar_time_interval;  // 被多个模块写/读
}
```

**重构方案**：将这些变量移入各自类的 `Options` struct 或实例成员，通过构造函数传入：

```cpp
// 新（推荐）
struct LidarPipeline::Options {
    float lidar_time_interval = 0.1;  // 取代 lo::lidar_time_interval
    // ...
};
```

唯一需要跨模块共享的动态值（如 `lidar_time_interval` 由扫描数据在线估计），改为：
- 由 `LidarPipeline` 估计后，通过返回值 or callback 传给调用方
- 调用方（`SlamEngine`）持有该值并向下传递，不用全局变量

---

## 6. GNSS RTK 假固定检测与融合设计

### 6.1 多层检测框架

```
RTK 原始量测
     │
     ▼
Layer 1：接收机质量门控
  - solution_type == FIX（可配置降级为 FLOAT）
  - num_satellites >= 6
  - hdop < 3.0
  - age_of_corr < 2.0s
     │ 通过
     ▼
Layer 2：位置跳变检测
  - 用 LIO/ESKF 当前状态预测 RTK 时刻位置
  - |RTK_pos - predicted_pos|_2D > jump_th → 拒绝
  - 短时间内大跳变（速度暗示）也拒绝
     │ 通过
     ▼
Layer 3：时序一致性门
  - 连续 3 帧通过 Layer1+2 才正式 ACCEPT
  - 连续 5 帧拒绝 → 进入 blackout（暂停使用，等待恢复）
     │ ACCEPT
     ▼
Layer 4：PGO 卡方检验（事后）
  - 加入 PGO 优化
  - 计算残差 chi2 = r^T Ω r（Ω = Σ_rtk^{-1}）
  - chi2 > 7.81（3自由度，p=0.05）→ 标记为 outlier，不参与本次优化
```

### 6.2 GnssRtkHandler 类

文件：`src/core/gnss/gnss_rtk_handler.h`

```cpp
#pragma once
#include "core/types/platform_types.h"
#include "common/nav_state.h"

namespace lightning::core {

class GnssRtkHandler {
public:
    struct Options {
        // Layer 1：接收机质量门控
        int    min_satellites     = 6;
        double max_hdop           = 3.0;
        double max_age_sec        = 2.0;
        bool   require_fix        = true;   // false 则接受 FLOAT
        bool   accept_float       = false;  // 接受浮点解时，噪声自动放大 10 倍

        // Layer 2：跳变检测
        double jump_3d_th_m       = 5.0;   // 三维跳变阈值（m）
        double jump_2d_th_m       = 3.0;   // 水平跳变阈值（m）
        double max_rtk_lio_gap_sec = 1.0;  // 允许 RTK 与 LIO 的最大时间差

        // Layer 3：时序一致性
        int min_consecutive_valid   = 3;
        int max_consecutive_invalid = 5;
        int blackout_recovery_count = 10;

        // Layer 4：PGO 卡方门限
        double chi2_reject_th = 7.81;   // 3 自由度，p=0.05

        // 坐标
        // Datum 由外部在 GNSS 初始化后第一帧固定解时自动设置，或手动设置
        bool   auto_set_datum = true;

        // RTK 观测噪声（fix 解，ENU 方向，标准差 m）
        Vec3d fix_pos_std   = Vec3d(0.05, 0.05, 0.10);   // 水平 5cm，高程 10cm
        Vec3d float_pos_std = Vec3d(0.30, 0.30, 0.60);
        double heading_std_rad = 0.01;  // 双天线航向噪声（rad）
    };

    // 验证结果
    enum class ValidResult {
        ACCEPT,
        REJECT_QUALITY,     // Layer 1 拒绝
        REJECT_JUMP,        // Layer 2 拒绝
        REJECT_CONSISTENCY, // Layer 3 拒绝（处于 blackout）
        PENDING,            // Layer 3 等待积累
    };

    struct ProcessedRtk {
        ValidResult result;

        // ENU 坐标（已减去 datum，已做 WGS84→ECEF→ENU 变换）
        Vec3d      pos_enu;
        Mat3d      pos_cov;          // 3×3 协方差（m²）

        bool       heading_valid = false;
        double     heading_rad   = 0;
        double     heading_cov   = 0;

        GnssRtkFrame::SolutionType solution_type;
        double     chi2          = 0;  // 由 PGO 回填
        bool       is_inlier     = true;
    };

    explicit GnssRtkHandler(Options options = Options());

    // 更新 LIO 当前状态（用于 Layer 2 跳变检测）
    void UpdateCurrentState(const NavState& state);

    // 处理一帧 RTK，返回验证结果和 ENU 坐标
    ProcessedRtk Process(const GnssRtkFrame& rtk);

    // PGO 优化后回填 chi2，决定是否为 inlier（Layer 4）
    void UpdateChi2(double chi2, ProcessedRtk& rtk_obs);

    // 手动设置 Datum（lat/lon/alt 角度制）
    void SetDatum(double lat_deg, double lon_deg, double alt_m);

    bool IsDatumSet() const { return datum_set_; }
    bool IsInBlackout() const { return in_blackout_; }

    int TotalAccepted()  const { return total_accepted_; }
    int TotalRejected()  const { return total_rejected_; }

private:
    Options options_;

    // Datum（首帧固定解）
    bool   datum_set_  = false;
    Vec3d  datum_llh_;          // lat(rad), lon(rad), alt(m)
    Vec3d  datum_ecef_;
    Mat3d  R_ecef_enu_;         // ECEF → ENU 旋转

    // 状态追踪
    NavState last_state_;
    bool     last_state_set_ = false;

    int  consecutive_valid_   = 0;
    int  consecutive_invalid_ = 0;
    bool in_blackout_         = false;
    int  blackout_recovery_   = 0;

    int total_accepted_ = 0;
    int total_rejected_ = 0;

    // 坐标转换工具
    Vec3d LlhToEcef(double lat_rad, double lon_rad, double alt_m) const;
    Vec3d EcefToEnu(const Vec3d& ecef) const;
    Mat3d BuildRotEcefEnu(double lat_rad, double lon_rad) const;

    // Layer 检查
    bool CheckQuality(const GnssRtkFrame& rtk) const;
    bool CheckJump(const Vec3d& pos_enu) const;
};

}  // namespace lightning::core
```

### 6.3 坐标变换实现要点

文件：`src/adapter/common/coordinate_utils.h`

```
WGS84 (lat, lon, alt)
    │ ① LlhToEcef
    ▼
ECEF (X, Y, Z)
    │ ② EcefToEnu (减 datum_ecef，再乘 R_ecef_enu)
    ▼
Local ENU (East, North, Up)
    │ ③ 仅使用 EN（忽略 Up / 或全 3D）
    ▼
PGO 观测量
```

关键公式：
```
// ① WGS84 → ECEF
a = 6378137.0; b = 6356752.31424518; e² = 1 - (b/a)²
N = a / sqrt(1 - e² sin²(lat))
X = (N + alt) cos(lat) cos(lon)
Y = (N + alt) cos(lat) sin(lon)
Z = (N(1-e²) + alt) sin(lat)

// ② ECEF → ENU（相对 datum）
Δecef = ecef - datum_ecef
R_ecef_enu = [-sin(lon),           cos(lon),          0        ]
             [-sin(lat)cos(lon), -sin(lat)sin(lon), cos(lat)   ]
             [ cos(lat)cos(lon),  cos(lat)sin(lon), sin(lat)   ]
enu = R_ecef_enu * Δecef
```

### 6.4 RTK 融入 PGO

在 `PGOFrame` 中恢复 RTK 字段（现已注释，直接解注释并更新类型）：

```cpp
// pgo_impl.h 中 PGOFrame 新增
struct RtkObs {
    bool       set       = false;
    bool       valid     = false;
    bool       inlier    = true;

    Vec3d      pos_enu;        // Layer1-3 通过后的 ENU 坐标
    Mat3d      pos_cov;        // 3×3 协方差（对角阵或从接收机获取）

    bool       heading_valid = false;
    double     heading_rad   = 0;
    double     heading_cov   = 0;

    GnssRtkFrame::SolutionType solution_type;
    double     chi2 = 0;       // PGO 优化后计算
};
RtkObs rtk_obs;
```

在 `PGOImpl` 中新增 `AddRtkFactors()`：

```cpp
void PGOImpl::AddRtkFactors() {
    for (auto& frame : frames_) {
        if (!frame->rtk_obs.set || !frame->rtk_obs.valid || !frame->rtk_obs.inlier)
            continue;

        // 构建 3 自由度位置先验边（仅约束 translation）
        auto edge = std::make_shared<miao::EdgePosition3D>();
        edge->SetVertex(0, vertices_[frame->frame_id_]);
        edge->SetMeasurement(frame->rtk_obs.pos_enu);

        // 协方差 = pos_cov，取对角线或直接用 3×3
        Eigen::Matrix3d info = frame->rtk_obs.pos_cov.inverse();
        edge->SetInformation(info);

        // 双天线：如有 heading，再添加 1-DOF 偏航约束
        if (frame->rtk_obs.heading_valid) {
            auto edge_yaw = std::make_shared<miao::EdgeYaw1D>();
            edge_yaw->SetVertex(0, vertices_[frame->frame_id_]);
            edge_yaw->SetMeasurement(frame->rtk_obs.heading_rad);
            edge_yaw->SetInformation(
                Eigen::Matrix<double,1,1>::Identity() /
                (frame->rtk_obs.heading_cov * frame->rtk_obs.heading_cov));
            optimizer_->AddEdge(edge_yaw);
        }

        optimizer_->AddEdge(edge);
        rtk_edges_.push_back(edge);
    }
}

// 优化后：计算 chi2 并回填，用于 Layer 4 拒绝
void PGOImpl::UpdateRtkChi2() {
    for (int i = 0; i < rtk_edges_.size(); i++) {
        double chi2 = rtk_edges_[i]->Chi2();
        frames_with_rtk_[i]->rtk_obs.chi2 = chi2;
        if (chi2 > options_.rtk_chi2_reject_th) {
            frames_with_rtk_[i]->rtk_obs.inlier = false;
            LOG(WARNING) << "RTK obs chi2=" << chi2
                         << " > " << options_.rtk_chi2_reject_th
                         << ", marked as outlier";
        }
    }
}
```

---

## 7. Core Engine 接口设计

### 7.1 SlamEngine（替代 SlamSystem）

文件：`src/core/engine/slam_engine.h`

```cpp
namespace lightning::core {

class SlamEngine {
public:
    struct Options {
        bool with_loop_closing   = true;
        bool with_gridmap        = false;
        bool with_ui             = true;
        bool online_mode         = true;

        LidarPipeline::Options lidar_options;
        LocalMap::Options      map_options;
        // loop closing, pgo options 等通过 YAML 加载
    };

    explicit SlamEngine(Options options = Options());

    bool Init(const std::string& yaml_path);
    void StartMapping(const std::string& map_name);
    void SaveMap(const std::string& path = "");

    // ── 统一输入接口（平台无关）──────────────────────────────────
    void FeedImu(const ImuFrame& imu);
    void FeedLidar(const PointCloudFrame& frame);

    // Online 模式下的 Spin（内部起 executor）
    void Spin();

private:
    Options                         options_;
    std::shared_ptr<LidarPipeline>  lidar_pipeline_;
    std::shared_ptr<LocalMap>       local_map_;
    std::shared_ptr<ESKF>           kf_;
    std::shared_ptr<LoopClosing>    loop_closing_;
    // ...
};

}  // namespace lightning::core
```

### 7.2 LocEngine（替代 LocSystem，新增 RTK 支持）

```cpp
namespace lightning::core {

class LocEngine {
public:
    struct Options {
        bool with_ui         = false;
        bool pub_tf          = true;
        bool enable_rtk      = false;

        LidarPipeline::Options  lidar_options;
        GnssRtkHandler::Options rtk_options;
    };

    explicit LocEngine(Options options = Options());
    bool Init(const std::string& yaml_path);

    void FeedImu(const ImuFrame& imu);
    void FeedLidar(const PointCloudFrame& frame);
    void FeedRtk(const GnssRtkFrame& rtk);       // 新增

    void Spin();
    void Finish();

private:
    Options                           options_;
    std::shared_ptr<LidarPipeline>    lidar_pipeline_;
    std::shared_ptr<LocalMap>         local_map_;
    std::shared_ptr<loc::Localization> localization_;
    std::shared_ptr<GnssRtkHandler>   rtk_handler_;     // 新增
};

}  // namespace lightning::core
```

### 7.3 ROS2 节点薄化

重构后 `SlamSystem` 和 `LocSystem` 变为纯粹的 ROS2 包装：

```cpp
// src/adapter/ros2/ros2_slam_node.cc
class Ros2SlamNode : public rclcpp::Node {
public:
    Ros2SlamNode() : Node("lightning_slam") {
        engine_ = std::make_shared<core::SlamEngine>(load_options());
        engine_->Init(get_parameter("config").as_string());
        engine_->StartMapping("new_map");

        imu_sub_  = create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, 10,
            [this](auto msg) {
                engine_->FeedImu(Ros2Converter::FromImu(*msg)); // 转换在 adapter 层
            });

        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic_, 10,
            [this](auto msg) {
                engine_->FeedLidar(Ros2Converter::FromPointCloud2(*msg, ...));
            });
    }
private:
    std::shared_ptr<core::SlamEngine> engine_;
    // subscriptions...
};
```

---

## 8. 重构执行路线图

### Phase 0：准备（2 天）

- [ ] 新建 `src/adapter/` 目录结构
- [ ] 新增 `src/core/types/platform_types.h`
- [ ] 新增 `src/adapter/common/coordinate_utils.h`（含单元测试）
- [ ] 迁移 `ros_utils.h` → `adapter/ros2/ros2_converter.h`，新增 FromImu / FromPointCloud2 接口

### Phase 1：LocalMap 抽取（3 天）

- [ ] 新建 `src/core/lidar/local_map.h/cc`，封装 `IVox` + 读写锁
- [ ] 修复 `MapIncremental` 中的 data race（写锁保护 `AddPoints`）
- [ ] `LaserMapping` 中将 `ivox_` 改为持有 `LocalMap` 实例
- [ ] 全量回归测试：NCLT/VBR 离线建图结果不变

### Phase 2：LidarPipeline 抽取（5 天）

- [ ] 新建 `src/core/lidar/lidar_pipeline.h/cc`
- [ ] 将 `ImuProcess::UndistortPcl` 迁移为 `LidarPipeline::UndistortFrame`
- [ ] 将 `LaserMapping::ObsModel` 迁移为 `LidarPipeline::BuildObservation`
- [ ] 将 `LaserMapping::MapIncremental` 迁移为 `LidarPipeline::UpdateMap`（调用 `LocalMap`）
- [ ] `LaserMapping` 改为持有 `LidarPipeline` + `LocalMap`，原有逻辑通过它们调用
- [ ] 消除 `ImuProcess` 对 `rclcpp` 的 **无** 依赖（实际无，但需验证头文件传递）
- [ ] 全量回归测试

### Phase 3：消除全局 extern（3 天）

- [ ] 将 `lo::lidar_time_interval` 移入 `LidarPipeline::Options`（在线估计后由 `SlamEngine` 持有）
- [ ] 将 `fasterlio::NUM_MAX_ITERATIONS` 等移入 `ESKF::Options`
- [ ] 将 `pgo::*` 噪声参数移入 `PGOImpl::Options`（已有部分，补全即可）
- [ ] 删除 `common/options.h` 中的全部 `extern`

### Phase 4：RTK 集成（5 天）

- [ ] 实现 `GnssRtkHandler`（含 Layer 1-3 检测 + 坐标转换）
- [ ] 实现 `coordinate_utils.h` 中 LLH→ECEF→ENU 转换，编写单测（与已知坐标对比）
- [ ] 恢复 `PGOFrame::rtk_obs` 字段
- [ ] 实现 `PGOImpl::AddRtkFactors()` + `UpdateRtkChi2()`（Layer 4）
- [ ] 新增 `LocEngine::FeedRtk()` 接口
- [ ] 新增 ROS2 适配器：`FromNavSatFix` / `FromNovatelBestPos`

### Phase 5：Engine API + 平台无关验证（3 天）

- [ ] 实现 `SlamEngine` / `LocEngine`，内部调用 `LidarPipeline` + `LocalMap` + ESKF
- [ ] `Ros2SlamNode` / `Ros2LocNode` 薄化为 adapter 调用
- [ ] 编写非 ROS 的离线测试程序（直接从 PCD 文件读入，不依赖任何 ROS 头文件）
- [ ] 验证去 ROS 依赖后 `src/core/` 可独立编译

---

## 9. 关键边界约定

### 9.1 模块依赖图（重构后）

```
platform_types.h ← 所有模块均可包含，零外部依赖

LidarPipeline
  ├── depends: IVox, Pose6D, NavState, ESKF（接口）
  └── NOT depends: rclcpp, sensor_msgs, livox_ros_driver2

LocalMap
  ├── depends: IVox, CloudPtr
  └── NOT depends: ROS

GnssRtkHandler
  ├── depends: platform_types, NavState, Eigen
  └── NOT depends: ROS

ESKF / NavState
  └── NOT changed（保持稳定）

PGOImpl
  ├── depends: miao, NavState, LocalizationResult
  └── RTK 扩展: GnssRtkHandler 的输出作为输入（不依赖 GnssRtkHandler 本身）

Adapter/ROS2
  └── depends: rclcpp, sensor_msgs, LidarPipeline, ESKF ... (唯一含 ROS 的层)
```

### 9.2 数据流（重构后，以定位模式为例）

```
ROS2 Subscription
    │
    ▼ Ros2Converter::FromImu / FromPointCloud2 / FromNavSatFix
    │                  (adapter 层，含 ROS 类型)
    ▼
LocEngine::FeedImu / FeedLidar / FeedRtk
    │                  (core 层，无 ROS 类型)
    │
    ├─ FeedImu      → LidarPipeline::FeedImu (buffer)
    │
    ├─ FeedLidar
    │    ├─ LidarPipeline::Sync (IMU/Lidar 时间对齐)
    │    ├─ LidarPipeline::Process (去畸变 + 降采样)
    │    ├─ ESKF::Update
    │    │    └─ LidarPipeline::BuildObservation (每次 IEKF 迭代)
    │    │         └─ LocalMap::QueryKNN
    │    ├─ LidarPipeline::UpdateMap
    │    │    └─ LocalMap::AddPoints
    │    └─ loc::Localization → PGO::ProcessLidarOdom
    │
    └─ FeedRtk
         ├─ GnssRtkHandler::Process (Layer 1-3)
         │    └─ ACCEPT → PGO::ProcessRtk
         │         └─ PGOImpl::AddRtkFactors + UpdateRtkChi2 (Layer 4)
         └─ REJECT_* → 记录统计，输出警告
```

---

## 10. TDD 重构安全网设计

### 10.0 核心原则：特征化测试优先

本次重构**不修改任何算法**，只做结构迁移。因此：

> **重构后的数值输出必须与重构前完全一致（浮点误差 < 1e-6），轨迹应逐帧可复现。任何数值偏差都是 bug，不是"精度损失"。**

这与新功能开发的 TDD 不同——这里使用 **特征化测试（Characterization Test）** 模式：
1. **重构前**：先捕获当前代码的"黄金输出"
2. **重构中**：每个 Phase 完成后，验证输出与黄金输出一致
3. **新功能**（RTK 集成）：用传统 TDD，先写失败测试，再实现

```
重构 Phase 1-3:  [捕获黄金输出] → [重构] → [对比黄金输出] → ✓ 继续 / ✗ 回滚
新功能 Phase 4:  [写失败测试] → [实现] → [测试通过] → ✓ 继续
```

---

### 10.1 三级安全网（测试金字塔）

```
         ┌──────────────────────────────┐
Level 3  │  端到端轨迹回归（整包 bag）    │  最慢，终极裁判
         │  指标：轨迹 RMSE diff < 0.1m  │
         └──────────────────────────────┘
         ┌──────────────────────────────┐
Level 2  │  黄金快照测试（关键中间值）    │  中速，每帧数值对比
         │  指标：Frobenius 范数 < 1e-6  │
         └──────────────────────────────┘
         ┌──────────────────────────────┐
Level 1  │  纯数学单元测试（无数据依赖）  │  最快，秒级，每次 CI 必跑
         │  指标：精确数值验证            │
         └──────────────────────────────┘
```

---

### 10.2 Level 1：纯数学单元测试

**不依赖任何数据，输入由测试代码构造，输出与手算或参考实现对比。**

#### 10.2.1 测试目录结构

```
tests/
├── unit/
│   ├── CMakeLists.txt
│   ├── eskf_math_test.cc           # ESKF 预测/更新数学正确性
│   ├── undistort_math_test.cc      # 去畸变插值数学正确性
│   ├── obsmodel_math_test.cc       # 点面 ICP 雅可比正确性
│   ├── local_map_thread_test.cc    # LocalMap 并发安全
│   ├── coordinate_utils_test.cc    # WGS84→ECEF→ENU 坐标变换
│   └── gnss_rtk_handler_test.cc    # RTK 假固定检测各层逻辑
├── snapshot/
│   ├── CMakeLists.txt
│   ├── golden/                     # 黄金输出文件（二进制，纳入 git lfs）
│   │   ├── undistort_frame_100.bin
│   │   ├── obsmodel_hthr_100.bin
│   │   └── eskf_state_100.bin
│   ├── capture_golden.cc           # 生成黄金输出的工具程序
│   └── verify_snapshot.cc          # 对比黄金输出的测试程序
└── regression/
    ├── run_regression.sh           # 整包 bag 跑完，计算轨迹 diff
    └── baseline_traj.tum           # 重构前的基准轨迹（TUM 格式）
```

#### 10.2.2 ESKF 数学测试

```cpp
// tests/unit/eskf_math_test.cc
// ── 测试 1：静止预测，协方差应增长 ───────────────────────────────
TEST(EskfMath, PredictStationary) {
    ESKF kf;
    NavState s0;
    s0.pos_ = Vec3d(0, 0, 0);
    s0.rot_ = SO3();
    s0.vel_ = Vec3d(0, 0, 0);

    ImuFrame imu;
    imu.acc  = Vec3d(0, 0, 9.81);  // 静止，加速度 = 重力
    imu.gyro = Vec3d(0, 0, 0);
    double dt = 0.01;

    auto p_before = kf.GetCov().diagonal().norm();
    kf.Predict(dt, imu.acc, imu.gyro);
    auto p_after  = kf.GetCov().diagonal().norm();

    EXPECT_GT(p_after, p_before);  // 协方差必须增长
    // 静止时位置不变（在 dt=0.01s 内）
    EXPECT_NEAR(kf.GetState().pos_.norm(), 0.0, 1e-6);
}

// ── 测试 2：匀速直线运动，Predict 10 步后位置正确 ───────────────
TEST(EskfMath, PredictConstantVelocity) {
    ESKF kf;
    NavState s0;
    s0.vel_ = Vec3d(1.0, 0, 0);  // 1 m/s 沿 X
    kf.SetState(s0);

    ImuFrame imu;
    imu.acc  = Vec3d(0, 0, 9.81);
    imu.gyro = Vec3d(0, 0, 0);

    for (int i = 0; i < 100; i++) kf.Predict(0.01, imu.acc, imu.gyro);
    // 1s 后位置应约为 1m（误差来自重力补偿的 Euler 积分）
    EXPECT_NEAR(kf.GetState().pos_.x(), 1.0, 0.01);
}

// ── 测试 3：退化检测（平面场景），低特征值方向协方差应膨胀 ───────
TEST(EskfMath, DegeneracyDetection) {
    // 构造只有 Z 轴约束的 HTH（走廊场景：X/Y 方向无约束）
    Mat6d HTH = Mat6d::Zero();
    HTH(2, 2) = 1000.0;  // 只有 Z 方向有强约束
    Vec6d HTr = Vec6d::Zero();
    HTr(2) = 0.01;

    ESKF kf;
    auto obs = ESKF::CustomObservationModel{HTH, HTr, true};
    kf.Update([&obs](const NavState&, ESKF::CustomObservationModel& o){ o = obs; });

    // X/Y 方向协方差不应被 HTH 压缩（退化方向保持原协方差）
    EXPECT_GT(kf.GetCov()(0, 0), kf.GetCov()(2, 2));
}
```

#### 10.2.3 去畸变数学测试

```cpp
// tests/unit/undistort_math_test.cc

// ── 测试 1：零运动，去畸变不改变点坐标 ─────────────────────────
TEST(Undistort, ZeroMotion) {
    // 构造静止 IMU 序列（只有重力）
    std::vector<ImuFrame> imu_seq = MakeStaticImuSeq(0.1 /*sec*/, 0.01 /*dt*/);

    PointCloudFrame pcl_frame = MakeRandomPointCloud(100, 0.1 /*scan duration*/);
    UndistortedFrame result;
    LidarPipeline pipeline;
    for (auto& imu : imu_seq) pipeline.FeedImu(imu);
    ESKF kf; // 零速度零旋转初始状态
    pipeline.Sync(pcl_frame, kf);
    pipeline.Process(kf, result);

    // 每个点坐标与原始偏差 < 1mm
    for (int i = 0; i < result.cloud_body->size(); i++) {
        auto& orig = pcl_frame.points[i];
        auto& comp = result.cloud_body->points[i];
        EXPECT_NEAR(comp.x, orig.x, 1e-3);
        EXPECT_NEAR(comp.y, orig.y, 1e-3);
        EXPECT_NEAR(comp.z, orig.z, 1e-3);
    }
}

// ── 测试 2：匀速旋转，帧头部点应被补偿，帧尾部点不补偿 ──────────
TEST(Undistort, ConstantRotation) {
    // ω = 0.1 rad/s 绕 Z 轴，扫描时长 0.1s，总旋转 = 0.01 rad
    double omega = 0.1;   // rad/s
    double scan_duration = 0.1;  // s

    std::vector<ImuFrame> imu_seq = MakeConstantYawImuSeq(omega, scan_duration, 0.01);

    // 帧头部的点（time_offset ≈ 0）：需要被补偿约 omega*scan_duration rad
    // 帧尾部的点（time_offset ≈ scan_duration）：补偿量 ≈ 0
    // 在纯 Z 旋转下，xy 平面点的角度偏差应等于 omega * (scan_duration - time_offset)

    // ... 验证补偿后所有点在 world 系下角度一致
    PointCloudFrame pcl_frame = MakeArcPointCloud(omega, scan_duration);
    UndistortedFrame result;
    LidarPipeline pipeline;
    // ...
    // 验证去畸变后点云在 world 系下分布均匀（标准差应 < 未去畸变的一半）
    double std_before = ComputeAngularStdDev(pcl_frame, omega);
    double std_after  = ComputeAngularStdDev(result, 0.0);
    EXPECT_LT(std_after, std_before * 0.1);
}
```

#### 10.2.4 观测模型（点面 ICP）雅可比测试

```cpp
// tests/unit/obsmodel_math_test.cc

// ── 测试 1：点在平面上，残差应为 0 ──────────────────────────────
TEST(ObsModel, PointOnPlane_ZeroResidual) {
    // 构造平面 n = [0,0,1], d = 0（XY 平面）
    Vec4f plane_coef(0, 0, 1, 0);
    // 构造在该平面上的点
    PointType pt; pt.x = 1.0; pt.y = 2.0; pt.z = 0.0;

    float residual = plane_coef.head<3>().dot(Vec3f(pt.x, pt.y, pt.z)) + plane_coef(3);
    EXPECT_NEAR(residual, 0.0f, 1e-6f);
}

// ── 测试 2：雅可比数值验证（有限差分） ──────────────────────────
// 这是最重要的测试：用数值微分验证解析雅可比
TEST(ObsModel, JacobianNumericalVerification) {
    Vec4f plane_coef(0, 0, 1, 0.5);  // n=[0,0,1], d=0.5
    PointType pt_body; pt_body.x = 0.5; pt_body.y = 0.5; pt_body.z = 0.5;

    NavState state;
    state.pos_ = Vec3d(0.1, 0.2, 0.3);
    state.rot_ = SO3::exp(Vec3d(0.01, 0.02, 0.03));

    // 解析雅可比（来自 ObsModel 实现）
    Vec6d J_analytic = ComputePointPlaneJacobian(pt_body, plane_coef, state);

    // 数值雅可比
    Vec6d J_numeric = ComputeNumericalJacobian(pt_body, plane_coef, state, 1e-6);

    for (int i = 0; i < 6; i++) {
        EXPECT_NEAR(J_analytic(i), J_numeric(i), 1e-4)
            << "Jacobian mismatch at dim " << i;
    }
}
```

**注意**：雅可比数值验证是整个测试套件中**最高价值的测试**——它确保 ObsModel 搬移后数学不变。

#### 10.2.5 LocalMap 并发安全测试

```cpp
// tests/unit/local_map_thread_test.cc
TEST(LocalMap, ConcurrentReadWrite_NoCrash) {
    LocalMap map;
    // 预填 10000 个随机点
    PointVector initial_pts = MakeRandomPoints(10000);
    map.AddPoints(initial_pts);

    std::atomic<bool> stop{false};
    std::atomic<int> read_count{0};
    std::vector<std::thread> readers;

    // 10 个读线程（QueryKNN）
    for (int i = 0; i < 10; i++) {
        readers.emplace_back([&]() {
            while (!stop) {
                PointType query = RandomPoint();
                PointVector neighbors;
                map.QueryKNN(query, neighbors, 5, 10.0);
                read_count.fetch_add(1);
            }
        });
    }

    // 1 个写线程（AddPoints）
    std::thread writer([&]() {
        while (!stop) {
            map.AddPoints(MakeRandomPoints(100));
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });

    std::this_thread::sleep_for(std::chrono::seconds(5));
    stop = true;
    writer.join();
    for (auto& t : readers) t.join();

    EXPECT_GT(read_count.load(), 1000);  // 确保读操作确实发生了
    // 若有 data race，TSAN 或 ASAN 会在这里报告
}

// KNN 结果正确性（对比暴力搜索）
TEST(LocalMap, KNN_MatchesBruteForce) {
    LocalMap map;
    PointVector pts = MakeGridPoints(100);  // 10×10×1 的规则网格
    map.AddPoints(pts);

    PointType query; query.x = 5.1; query.y = 5.1; query.z = 0.0;
    PointVector knn_result;
    map.QueryKNN(query, knn_result, 5, 10.0);

    // 暴力搜索
    PointVector brute = BruteForceKNN(pts, query, 5);

    ASSERT_EQ(knn_result.size(), brute.size());
    for (int i = 0; i < knn_result.size(); i++) {
        EXPECT_NEAR(DistSq(knn_result[i], query), DistSq(brute[i], query), 1e-4);
    }
}
```

#### 10.2.6 坐标变换测试

```cpp
// tests/unit/coordinate_utils_test.cc

// 参考值来自 PROJ / GeographicLib 或手算
TEST(CoordinateUtils, WGS84_to_ECEF_KnownPoint) {
    // 上海（约）：31.2°N, 121.5°E, 4m
    double lat = 31.2 * M_PI / 180.0;
    double lon = 121.5 * M_PI / 180.0;
    double alt = 4.0;

    Vec3d ecef = LlhToEcef(lat, lon, alt);
    // 参考值（GeographicLib 计算）
    EXPECT_NEAR(ecef.x(), -2848022.0, 1.0);   // ±1m
    EXPECT_NEAR(ecef.y(),  4652816.0, 1.0);
    EXPECT_NEAR(ecef.z(),  3284745.0, 1.0);
}

TEST(CoordinateUtils, ENU_Datum_SamePoint_IsZero) {
    double lat = 31.2 * M_PI / 180.0;
    double lon = 121.5 * M_PI / 180.0;
    double alt = 4.0;

    GnssRtkHandler handler;
    handler.SetDatum(lat * 180.0 / M_PI, lon * 180.0 / M_PI, alt);

    GnssRtkFrame rtk;
    rtk.solution_type = GnssRtkFrame::SolutionType::FIX;
    rtk.num_satellites = 10;
    rtk.hdop = 1.0;
    rtk.lat_deg = lat * 180.0 / M_PI;
    rtk.lon_deg = lon * 180.0 / M_PI;
    rtk.alt_m = alt;

    auto result = handler.Process(rtk);
    // Datum 点本身的 ENU 坐标应为 (0, 0, 0)
    EXPECT_NEAR(result.pos_enu.norm(), 0.0, 1e-3);
}

TEST(CoordinateUtils, ENU_NorthIs_PlusY) {
    // 在 Datum 点正北 100m 处，ENU 的 N 分量（Y）应 ≈ 100m
    double lat0 = 31.2 * M_PI / 180.0;
    double lon0 = 121.5 * M_PI / 180.0;
    // 往北 100m ≈ lat + 100/6378137 rad
    double lat1 = lat0 + 100.0 / 6378137.0;

    GnssRtkHandler handler;
    handler.SetDatum(lat0 * 180.0 / M_PI, lon0 * 180.0 / M_PI, 0.0);
    // ... 构造 rtk 帧 ...
    EXPECT_NEAR(result.pos_enu.y(), 100.0, 0.1);  // North ≈ ENU.Y
    EXPECT_NEAR(result.pos_enu.x(), 0.0, 0.1);    // East ≈ 0
}
```

#### 10.2.7 RTK 假固定检测逻辑测试

```cpp
// tests/unit/gnss_rtk_handler_test.cc

// Layer 1：各质量门控条件
TEST(GnssRtk, Layer1_RejectFloat) {
    GnssRtkHandler::Options opts; opts.require_fix = true;
    GnssRtkHandler handler(opts);
    handler.SetDatum(31.2, 121.5, 0.0);

    GnssRtkFrame rtk = MakeValidRtk();
    rtk.solution_type = GnssRtkFrame::SolutionType::FLOAT;
    auto r = handler.Process(rtk);
    EXPECT_EQ(r.result, GnssRtkHandler::ValidResult::REJECT_QUALITY);
}

TEST(GnssRtk, Layer1_RejectFewSats) {
    GnssRtkHandler handler; handler.SetDatum(31.2, 121.5, 0.0);
    GnssRtkFrame rtk = MakeValidRtk();
    rtk.num_satellites = 3;  // < min_satellites(6)
    EXPECT_EQ(handler.Process(rtk).result, GnssRtkHandler::ValidResult::REJECT_QUALITY);
}

// Layer 2：位置跳变检测
TEST(GnssRtk, Layer2_RejectPositionJump) {
    GnssRtkHandler handler; handler.SetDatum(31.2, 121.5, 0.0);
    // 先建立正常状态
    NavState state;
    state.pos_ = Vec3d(10.0, 10.0, 0.0);
    handler.UpdateCurrentState(state);

    GnssRtkFrame rtk = MakeValidRtk();
    // RTK 报告位置在 ENU (100, 10, 0)，与 LIO 预测偏差 90m
    rtk.lat_deg = 31.2 + 100.0 * 180.0 / (M_PI * 6378137.0);  // 约 100m North
    auto r = handler.Process(rtk);
    EXPECT_EQ(r.result, GnssRtkHandler::ValidResult::REJECT_JUMP);
}

// Layer 3：时序一致性（累积 3 帧才 ACCEPT）
TEST(GnssRtk, Layer3_PendingUntilConsecutive) {
    GnssRtkHandler handler; handler.SetDatum(31.2, 121.5, 0.0);
    GnssRtkFrame rtk = MakeValidRtk();

    EXPECT_EQ(handler.Process(rtk).result, GnssRtkHandler::ValidResult::PENDING);
    EXPECT_EQ(handler.Process(rtk).result, GnssRtkHandler::ValidResult::PENDING);
    EXPECT_EQ(handler.Process(rtk).result, GnssRtkHandler::ValidResult::ACCEPT);
}

// Layer 3：Blackout 机制（连续 5 帧拒绝进入 blackout）
TEST(GnssRtk, Layer3_BlackoutAfterManyRejects) {
    GnssRtkHandler handler; handler.SetDatum(31.2, 121.5, 0.0);
    GnssRtkFrame bad_rtk = MakeValidRtk();
    bad_rtk.num_satellites = 2;  // 确保 Layer1 拒绝

    for (int i = 0; i < 5; i++) handler.Process(bad_rtk);
    EXPECT_TRUE(handler.IsInBlackout());

    // 恢复正常输入，需要 blackout_recovery_count 帧才退出 blackout
    GnssRtkFrame good_rtk = MakeValidRtk();
    for (int i = 0; i < 9; i++) handler.Process(good_rtk);
    EXPECT_TRUE(handler.IsInBlackout());  // 还没到 10 帧
    handler.Process(good_rtk);
    EXPECT_FALSE(handler.IsInBlackout());
}
```

---

### 10.3 Level 2：黄金快照测试

**原理**：在重构前，用当前代码跑一段 bag（前 100 帧），将关键中间值序列化到文件（golden data）。重构后，用相同输入再跑一次，对比数值。

#### 10.3.1 捕获点选择

选择下表中 5 个检测点，它们覆盖了重构涉及的所有核心路径：

| 检测点 ID | 捕获位置 | 数据内容 | 验证精度 |
|----------|---------|---------|---------|
| CP-1 | `SyncPackages()` 返回后 | `measures_.imu_` 序列时间戳列表 | 精确匹配 |
| CP-2 | `UndistortPcl()` 返回后 | `scan_undistort_` 前 50 个点的 xyz | < 1e-4 m |
| CP-3 | `ObsModel()` 第一次迭代完成后 | `HTH`（6×6）和 `HTr`（6×1）的 Frobenius 范数，`valid_num` | < 1e-6 |
| CP-4 | `ESKF::Update()` 返回后 | `state_point_.pos_`, `.vel_`, `.bg_` | < 1e-6 |
| CP-5 | `MakeKF()` 被调用时 | keyframe `lio_pose_`（SE3：平移 + 四元数） | < 1e-5 |

#### 10.3.2 黄金输出捕获工具

```cpp
// tests/snapshot/capture_golden.cc
// 用法：./capture_golden --bag /path/to/test.bag --num_frames 100 --output tests/snapshot/golden/

int main(int argc, char** argv) {
    // 1. 初始化旧版 LaserMapping（重构前代码）
    LaserMapping lio;
    lio.Init(config_path);

    // 2. 注入捕获钩子
    //    （通过修改 LaserMapping 临时添加 debug callback，或用宏开关）
    GoldenWriter writer(output_dir);
    lio.SetDebugCallback([&](int frame_id, const CheckPoint& cp) {
        writer.Write(frame_id, cp);
    });

    // 3. 播放 bag 前 N 帧
    RosbagIO bag(bag_path);
    bag.AddPointCloud2Handle(topic, [&](auto msg) {
        lio.ProcessPointCloud2(msg);
        lio.Run();
        return true;
    });
    bag.AddImuHandle(imu_topic, [&](auto imu) {
        lio.ProcessIMU(imu);
        return true;
    });
    bag.GoN(num_frames);

    writer.Flush();
    LOG(INFO) << "Golden data written to " << output_dir;
}
```

`CheckPoint` 结构体：
```cpp
struct CheckPoint {
    int    frame_id;

    // CP-2：去畸变后点云（只保存前 50 个点）
    std::vector<std::array<float, 3>> undistort_pts;

    // CP-3：ObsModel 第一次迭代
    Eigen::Matrix<double, 6, 6> HTH;
    Eigen::Matrix<double, 6, 1> HTr;
    int valid_num;

    // CP-4：ESKF 更新后状态
    Vec3d pos, vel, bg;
    Eigen::Quaterniond rot;

    // CP-5：关键帧位姿（只在 is_keyframe=true 时有效）
    bool is_keyframe = false;
    Vec3d kf_pos;
    Eigen::Quaterniond kf_rot;
};
```

#### 10.3.3 快照对比测试

```cpp
// tests/snapshot/verify_snapshot.cc
class SnapshotTest : public ::testing::TestWithParam<int> {};

TEST_P(SnapshotTest, FrameMatchesGolden) {
    int frame_id = GetParam();

    // 加载黄金数据
    CheckPoint golden = GoldenReader::Read(golden_dir, frame_id);

    // 运行重构后的代码（LidarPipeline）
    CheckPoint actual = RunRefactoredCode(bag_path, frame_id);

    // CP-2：去畸变点云（< 1e-4 m）
    ASSERT_EQ(actual.undistort_pts.size(), golden.undistort_pts.size());
    for (int i = 0; i < actual.undistort_pts.size(); i++) {
        EXPECT_NEAR(actual.undistort_pts[i][0], golden.undistort_pts[i][0], 1e-4)
            << "undistort pt[" << i << "].x mismatch at frame " << frame_id;
    }

    // CP-3：ObsModel HTH
    EXPECT_LT((actual.HTH - golden.HTH).norm(), 1e-6)
        << "HTH mismatch at frame " << frame_id;
    EXPECT_LT((actual.HTr - golden.HTr).norm(), 1e-6);
    EXPECT_EQ(actual.valid_num, golden.valid_num);

    // CP-4：ESKF 状态
    EXPECT_LT((actual.pos - golden.pos).norm(), 1e-6)
        << "pos mismatch at frame " << frame_id;
    EXPECT_LT((actual.rot * golden.rot.inverse()).vec().norm(), 1e-6)
        << "rot mismatch at frame " << frame_id;

    // CP-5：关键帧
    if (golden.is_keyframe) {
        EXPECT_TRUE(actual.is_keyframe) << "missed keyframe at frame " << frame_id;
        EXPECT_LT((actual.kf_pos - golden.kf_pos).norm(), 1e-5);
    }
}

INSTANTIATE_TEST_SUITE_P(
    Frames0to99, SnapshotTest,
    ::testing::Range(0, 100)
);
```

**快照文件管理**：
- 使用 `git lfs` 管理 `.bin` 文件（典型大小 < 5 MB）
- 快照仅在 **Phase 0 完成后**（重构第一刀落下前）生成一次，之后不再更新
- 如果需要算法升级（不是重构），必须重新生成快照并在 PR 中说明原因

---

### 10.4 Level 3：端到端轨迹回归

#### 10.4.1 基准轨迹捕获

**在重构第一行代码之前**，运行完整 bag，保存轨迹为 TUM 格式：

```bash
# 生成基准轨迹（只需做一次，结果纳入 git）
./run_slam_offline \
    --input_bag /data/test_dataset.db3 \
    --config config/config.yaml \
    --output_traj tests/regression/baseline_traj.tum

# 验证基准轨迹有效（有效帧数 > 500）
wc -l tests/regression/baseline_traj.tum
```

TUM 格式：`timestamp tx ty tz qx qy qz qw`（每帧一行）

#### 10.4.2 回归测试脚本

```bash
#!/bin/bash
# tests/regression/run_regression.sh

set -e
BAG=/data/test_dataset.db3
CONFIG=config/config.yaml
BASELINE=tests/regression/baseline_traj.tum
TRAJ_OUT=/tmp/refactored_traj.tum
# 允许的最大轨迹偏差（RMSE，单位 m）
MAX_RMSE=0.05   # 50mm；重构不改算法，应远小于此值

# 1. 运行重构后代码
./run_slam_offline \
    --input_bag $BAG \
    --config $CONFIG \
    --output_traj $TRAJ_OUT

# 2. 计算轨迹 diff（用 evo，对比两条轨迹的 RMSE，不需要 GT）
evo_res $TRAJ_OUT $BASELINE \
    --align --correct_scale \
    --save_results /tmp/regression_result.zip

# 3. 提取 RMSE 并判断
RMSE=$(python3 -c "
import zipfile, json
with zipfile.ZipFile('/tmp/regression_result.zip') as z:
    with z.open('stats.json') as f:
        data = json.load(f)
print(data['rmse'])
")

echo "Trajectory RMSE diff: ${RMSE} m (threshold: ${MAX_RMSE} m)"
if python3 -c "assert $RMSE < $MAX_RMSE, 'RMSE too large!'"; then
    echo "✓ Regression test PASSED"
else
    echo "✗ Regression test FAILED — possible algorithm behavior change!"
    exit 1
fi
```

**注意**：这里比较的是"重构后轨迹 vs 重构前轨迹"（非 GT），RMSE 阈值设为 **50mm**。若重构严格正确，实际 RMSE 应 < 1mm（仅浮点舍入差异）。超过 1mm 即应怀疑，超过 50mm 则 CI 失败。

---

### 10.5 各重构 Phase 的测试门控

每个 Phase 合并到主分支的前置条件（**全部通过才能合并**）：

#### Phase 0 门控（准备阶段）
```
✓ coordinate_utils_test.cc 全部通过
✓ 黄金快照已生成（100 帧，文件在 git lfs 中）
✓ baseline_traj.tum 已生成并纳入 git
✓ 编译无警告（-Wall -Wextra）
```

#### Phase 1 门控（LocalMap 抽取）
```
✓ local_map_thread_test.cc 全部通过（含 TSAN 检测）
   → 必须用 -fsanitize=thread 编译运行，确认无 data race 报告
✓ 快照测试 CP-3（ObsModel HTH/HTr）全部通过（100 帧）
✓ 回归测试 RMSE < 50mm
```

**注意**：若修复了原来的 data race，在**单线程**测试场景下快照应完全匹配。加读写锁后，单线程行为不变。

#### Phase 2 门控（LidarPipeline 抽取）
```
✓ undistort_math_test.cc 全部通过
✓ obsmodel_math_test.cc 全部通过（含雅可比数值验证）
✓ 快照测试 CP-2 / CP-3 / CP-4 / CP-5 全部通过（100 帧）
   → 这是最严格的 gate：ObsModel 搬移后 HTH 必须逐帧相同
✓ lidar_pipeline_test.cc（零运动 + 匀速旋转场景）通过
✓ 回归测试 RMSE < 50mm
✓ 编译 src/core/ 时不能出现任何 rclcpp 相关符号
   → 用 nm / objdump 检查：nm libcore.a | grep rclcpp | wc -l 应为 0
```

#### Phase 3 门控（消除 extern）
```
✓ 所有 Level 1 单元测试通过（Options 默认值验证）
✓ 快照测试全部通过（100 帧）
   → extern 默认值与 Options struct 默认值必须完全一致
✓ 回归测试 RMSE < 50mm
✓ grep -r "extern" src/common/options.h 输出为空
```

**重点风险**：全局 extern 变量在运行时由某处代码写入（如 `fasterlio::NUM_MAX_ITERATIONS = yaml["...")`）。消除后，这个写入必须通过 Options 传入，值相同。用快照测试的 CP-3（迭代次数隐含在 ObsModel 调用次数中）可以验证。

#### Phase 4 门控（RTK 集成）
```
✓ gnss_rtk_handler_test.cc 全部通过（Layer 1-4 全覆盖）
✓ coordinate_utils_test.cc 全部通过
✓ RTK 关闭（enable_rtk=false）时，回归测试 RMSE < 50mm
   → RTK 集成不能影响 RTK 关闭时的 LIO 路径
✓ 假固定注入测试（手动构造 bag 帧，验证 Layer 2 触发）
```

#### Phase 5 门控（Engine API + 平台无关验证）
```
✓ test_core_no_ros 可执行文件成功编译（不链接 rclcpp）
   → CMakeLists.txt 中单独 target，不依赖任何 ROS 包
✓ 快照测试全部通过（通过 FileAdapter 播放，不用 ROS）
✓ 回归测试 RMSE < 50mm
```

---

### 10.6 测试基础设施

#### 10.6.1 CMake 集成

```cmake
# tests/CMakeLists.txt

# Level 1 单元测试（无数据依赖，每次编译后都运行）
add_executable(unit_tests
    unit/eskf_math_test.cc
    unit/undistort_math_test.cc
    unit/obsmodel_math_test.cc
    unit/local_map_thread_test.cc
    unit/coordinate_utils_test.cc
    unit/gnss_rtk_handler_test.cc
)
target_link_libraries(unit_tests
    lightning_core   # 只链接 core，不含 rclcpp
    GTest::GTest GTest::Main
)
# TSAN 版本（用于线程安全测试）
add_executable(unit_tests_tsan ${unit_test_sources})
target_compile_options(unit_tests_tsan PRIVATE -fsanitize=thread -g)
target_link_options(unit_tests_tsan PRIVATE -fsanitize=thread)

# Level 2 快照测试（需要测试数据，CI 中条件运行）
add_executable(snapshot_tests snapshot/verify_snapshot.cc)
target_link_libraries(snapshot_tests lightning_core lightning_adapter GTest::GTest)
# 需要设置环境变量 GOLDEN_DIR 和 BAG_PATH

# ctest 注册
add_test(NAME UnitTests    COMMAND unit_tests)
add_test(NAME UnitTestTSAN COMMAND unit_tests_tsan)
add_test(NAME SnapshotTests COMMAND snapshot_tests
    ENVIRONMENT "GOLDEN_DIR=${CMAKE_SOURCE_DIR}/tests/snapshot/golden"
)
```

#### 10.6.2 CI 触发策略

| 触发条件 | 运行级别 | 耗时（估计） |
|---------|---------|------------|
| 每次 `git push` | Level 1 单元测试 | < 30s |
| PR 合并前 | Level 1 + Level 2 快照测试 | < 5min |
| Phase 完成节点（手动触发） | Level 1 + 2 + 3 回归测试 | 10-30min（视 bag 大小） |
| TSAN 检测 | Phase 1 完成后，专项触发 | < 2min |

#### 10.6.3 合成数据生成工具

为了使 Level 1 测试完全脱离真实数据，需要一个合成数据生成器：

```cpp
// tests/utils/synthetic_data.h
namespace test_utils {

// 生成匀速旋转 IMU 序列（绕 Z 轴）
std::vector<ImuFrame> MakeConstantYawImuSeq(
    double omega_z,      // rad/s
    double duration_sec,
    double dt_sec);

// 生成纯平移 IMU 序列
std::vector<ImuFrame> MakeConstantVelImuSeq(
    const Vec3d& velocity,
    double duration_sec,
    double dt_sec);

// 生成在平面上的点云（用于 ObsModel 测试）
PointCloudFrame MakePlanarPointCloud(
    const Vec4f& plane_coef,  // ax+by+cz+d=0
    int num_points,
    double scan_duration);

// 生成 KNN 测试用的规则网格点云
PointVector MakeGridPoints(int n_per_axis, float spacing = 1.0f);

// 构造一个"好的" RTK 帧（默认参数满足所有 Layer 1 条件）
GnssRtkFrame MakeValidRtk(double lat = 31.2, double lon = 121.5, double alt = 4.0);

}  // namespace test_utils
```

---

### 10.7 重构过程中的实操流程

每个 Phase 的标准操作步骤：

```
1. 切新分支：git checkout -b refactor/phase-N

2. 运行当前测试（确保基线通过）：
   ctest -R UnitTests --output-on-failure

3. 进行代码重构（仅结构变化，不改逻辑）

4. 再次运行单元测试（Level 1）：
   ctest -R UnitTests

5. 运行快照测试（Level 2）：
   GOLDEN_DIR=... BAG_PATH=... ctest -R SnapshotTests

6. 如有失败：
   a. 检查 CP-2（去畸变）：diff 通常来自时间戳处理或 IMU 插值顺序
   b. 检查 CP-3（ObsModel HTH）：diff 通常来自点投影到世界系的外参乘法顺序
   c. 检查 CP-4（ESKF 状态）：diff 通常来自 boxplus 调用时机
   d. 调试工具：在 capture 程序中对比两版代码的中间输出，逐行定位

7. 全部通过后，运行回归测试（Level 3）：
   bash tests/regression/run_regression.sh

8. 提 PR，在 PR 描述中贴上：
   - 快照测试通过截图
   - 回归测试 RMSE 数值
   - TSAN 报告（Phase 1 必须无 race）
```

---

### 10.8 常见重构错误与检测方法

| 错误类型 | 快照检测点 | 典型现象 |
|---------|----------|---------|
| UndistortPcl 中 `time_offset` 单位错误（ms/s 混用） | CP-2 | 第 1-2 帧正常，后续帧点云漂移，RMSE 急剧增大 |
| ObsModel 中外参 `R_LI` / `t_LI` 乘法顺序颠倒 | CP-3 | 第 1 帧 HTH 就不匹配，`valid_num` 可能为 0 |
| LocalMap 构造函数 `ivox_resolution` 参数错误 | CP-3 | `valid_num` 下降（KNN 找不到近邻），HTH 范数偏小 |
| `extern` 变量迁移时默认值不一致 | CP-3/CP-4 | 迭代次数变化导致 HTH 累加次数不同 |
| 新旧代码并存时调用路径未切换（旧代码仍在运行） | CP-4 | 快照测试通过但实际没测到新代码（用 nm 或 coverage 验证） |
| `shared_mutex` 造成读锁未释放 | 线程测试 | 写线程饥饿，性能下降，TSAN 死锁报告 |

---

## 附录 A：需要新增的 `miao` 图优化类型

| 类型 | 用途 | 参数维度 |
|-----|-----|---------|
| `EdgePosition3D` | RTK 位置先验约束 | 测量：Vec3d（ENU），信息：Mat3d |
| `EdgeYaw1D` | 双天线航向约束 | 测量：double（rad），信息：1×1 |

两者均为 `EdgeSE3Prior` 的降维特化版本，可参照现有 `edge_se3_prior.h` 实现。

## 附录 B：假固定典型场景与对应检测层

| 场景 | 现象 | 触发层 |
|------|------|-------|
| 遮挡后首次重捕获，整数模糊度未收敛 | solution_type=FLOAT | Layer 1（需配置 require_fix=true）|
| 多路径效应导致坐标跳 | 单帧位置跳变 3-5m | Layer 2（jump_2d_th_m） |
| 天线遮挡后虚假固定 | num_satellites 突降 | Layer 1（min_satellites） |
| 差分信号超龄（远离基站） | age_of_corr > 3s | Layer 1（max_age_sec） |
| 系统性偏差（建筑物附近）| 与 LIO 持续偏差，chi2 大 | Layer 4（卡方检验） |
| 短时间多次跳变（极端环境）| N 帧连续拒绝 | Layer 3（blackout 机制）|
