# Lightning-LM 重构完成总结

## 项目背景

原始 Lightning-LM 是一个功能完整的 ROS 2 激光 SLAM + 定位系统，但存在三个核心问题：

1. **LiDAR 处理逻辑散乱** — 去畸变在 `ImuProcess`，观测构建在 `LaserMapping`，无统一接口
2. **ROS 头文件侵入核心层** — `sensor_msgs`、`geometry_msgs` 等直接 `#include` 进算法类，无法在非 ROS 环境编译
3. **RTK/GNSS 通路缺失** — `PGOFrame` 中 RTK 字段全部注释掉，无假固定检测，无坐标系转换

重构目标：在不修改任何算法（ESKF、IVox、miao 优化器）的前提下，完成以上三项整改。

---

## 完成工作

### Phase 2 — LidarPipeline 封装

**问题：** `LaserMapping::MapIncremental()` 直接操作 IVox 地图，点云地图增量更新逻辑散落其中，无单独可测试接口。

**方案：** 抽取独立类 `LidarPipeline`，`LaserMapping::MapIncremental()` 改为委托调用。

**关键文件：**
- `src/core/lidar/lidar_pipeline.h/cc` — 新增，封装点云地图增量更新
- `src/core/lidar/local_map.h/cc` — 新增，IVox 地图生命周期管理
- `src/core/lio/laser_mapping.cc` — `MapIncremental()` 改为 `lidar_pipeline_->UpdateMap()`

---

### Phase 3 — 消除全局 extern 变量

**问题：** `common/options.h` 中大量 `extern X foo` 全局变量跨模块共享状态，包括 `NUM_MAX_ITERATIONS`、`ESTI_PLANE_THRESHOLD`、`lidar_time_interval` 等。

**方案：** 全部改为 `inline X foo = val` 就地初始化；运行期可变量迁移为相关类的成员变量。

**关键变更：**

| 原全局变量 | 迁移位置 |
|-----------|---------|
| `fasterlio::NUM_MAX_ITERATIONS` | `LaserMapping::num_max_iterations_` |
| `fasterlio::ESTI_PLANE_THRESHOLD` | `LidarPipeline::Options::esti_plane_threshold_` |
| `lo::lidar_time_interval` | `LidarPipeline::SetLidarTimeInterval()` |
| `pgo::pgo_smooth_factor` | `PGOImpl::options_.pgo_smooth_factor` |
| `debug::SigHandle` rclcpp 依赖 | 移除，仅保留 `flg_exit = true` |

**关键文件：**
- `src/common/options.h` — `extern` → `inline`，移除 `#include <rclcpp/rclcpp.hpp>`

---

### Phase 4 — RTK/GNSS 量测接入

**问题：** `PGOFrame` 中的 RTK 相关字段全部注释，系统无法融合 GNSS 信息。

**方案：** 完整实现从原始 NavSatFix 到 PGO 位置约束的全链路，含四层假固定检测。

#### 四层假固定检测（`GnssRtkHandler`）

| 层级 | 逻辑 |
|------|------|
| Layer 1 | 解类型 FIX + 卫星数 ≥ 6 + HDOP ≤ 2.0 |
| Layer 2 | 相邻位置跳变检测（> 阈值则拒绝并重置） |
| Layer 3 | 滑窗连续有效计数（需 ≥ N 帧才输出） |
| Layer 4 | 基于历史残差的卡方检验（> 3σ 拒绝） |

#### PGO 集成

- `PGOFrame::RtkObs` 结构体，存储经 ENU 转换后的位置观测
- `PGOImpl::AddRtkFactors()` — 将 RTK 位置转为 `EdgePositionPrior`（3×6 Jacobian）加入图优化
- `PGOImpl::AssignRtkObsIfNeeded()` — 按时间戳将 RTK 观测分配给最近关键帧
- `PGOImpl::UpdateRtkChi2()` — 用当前优化结果更新卡方统计（反馈给 Layer 4）
- 坐标系：WGS84 → ECEF → ENU，datum 自动设为首次有效 FIX

#### 新增 miao 边类型

- `src/core/miao/core/types/edge_position_prior.h` — 3D 位置先验边，完整推导 3×6 Jacobian（$\partial e / \partial \xi$）

**关键文件：**
- `src/core/gnss/gnss_rtk_handler.h/cc` — 四层假固定检测
- `src/core/miao/core/types/edge_position_prior.h` — 新边类型
- `src/core/localization/pose_graph/pgo_impl.h/cc` — RTK 队列、因子添加、卡方更新
- `src/core/localization/pose_graph/pgo.h/cc` — 新增 `ProcessRtk()` 对外接口
- `src/adapter/common/coordinate_utils.h/cc` — WGS84→ECEF→ENU 坐标转换
- `tests/unit/gnss_rtk_handler_test.cc` — 9 个单元测试

---

### Phase 5 — Engine API + 平台无关验证 + CMakeLists 拆分

#### 5a. Engine API

**问题：** `SlamSystem` / `LocSystem` 直接持有 `rclcpp::Node`，无法在非 ROS 环境使用。

**方案：** 新增 `SlamEngine` / `LocEngine` 作为零 ROS 依赖的对外接口层；原 ROS 节点改为调用 Engine 的薄封装。

```
ROS 节点 (run_slam_online.cc)
    └─ SlamEngine::FeedImu(ImuFrame)
    └─ SlamEngine::FeedLidar(PointCloudFrame)
           └─ SlamSystem (内部实现，保留)

非 ROS 程序
    └─ 直接 #include "core/engine/slam_engine.h"
    └─ 链接 liblightning_core.a（无任何 ROS 符号）
```

**平台无关数据类型（`src/core/types/platform_types.h`）：**

```cpp
struct ImuFrame        { double timestamp_sec; Vec3d acc, gyro; };
struct PointCloudFrame { double timestamp_sec; uint64_t seq; std::vector<RawPoint> points; };
struct GnssRtkFrame    { double timestamp_sec; double lat_deg, lon_deg, alt_m; SolutionType solution_type; Vec3d pos_std_enu; };
```

**关键文件：**
- `src/core/engine/slam_engine.h/cc` — ROS-free `SlamEngine`
- `src/core/engine/loc_engine.h/cc` — ROS-free `LocEngine`，含 `FeedRtk()`
- `src/core/types/platform_types.h` — 平台无关数据帧类型
- `src/adapter/ros2/ros2_converter.h/cc` — ROS msg ↔ platform types 转换，含 `LocResultToGeoMsg()`

#### 5b. 头文件 ROS 污染清理

| 文件 | 变更 |
|------|------|
| `common/options.h` | 移除 `#include <rclcpp/rclcpp.hpp>` |
| `core/localization/localization_result.h` | 移除 `geometry_msgs`，`ToGeoMsg()` 迁移到 `ros2_converter` |
| `core/localization/localization.h` | 移除 ROS 类型，`TFCallback` 改为 `std::function<void(SE3, double)>` |
| `core/system/loc_system.h/cc` | 适配新 TFCallback，lambda 内构造 TransformStamped |

#### 5c. CMakeLists 拆分

**方案：** `src/CMakeLists.txt` 拆为两个目标：

```
lightning_core (STATIC)          lightning.libs (SHARED)
─────────────────────────        ────────────────────────────────
common/                          adapter/ros2/ros2_converter.cc
adapter/common/                  core/lio/laser_mapping.cc
core/gnss/                       core/lio/pointcloud_preprocess.cc
io/                              core/engine/slam_engine.cc
core/lio/eskf.cc                 core/engine/loc_engine.cc
core/loop_closing/               core/g2p5/g2p5_map.cc
core/lidar/                      core/g2p5/g2p5.cc
core/g2p5/g2p5_subgrid.cc       core/localization/lidar_loc/
core/maps/                       core/localization/localization.cpp
core/localization/result.cc      ui/
core/localization/pose_graph/    core/system/slam.cc
utils/                           core/system/loc_system.cc
                                 wrapper/bag_io.cc

无 ament_target_dependencies     ament_target_dependencies(ROS 2)
                                 target_link_libraries(lightning_core, ...)
```

验证（`nm -u`）：`lightning_core.a` 含零 ROS 符号，`lightning.libs.so` 含 1793 个 rclcpp 符号。

#### 5d. 零 ROS 测试

- `tests/offline/test_core_no_ros.cc` — 4 个 GTest，验证 Engine API 可在不 `source ROS` 的环境下构造和运行

---

## 测试覆盖

| 测试集 | 数量 | 覆盖内容 |
|--------|------|---------|
| `test_coordinate_utils` | 6 | WGS84→ECEF→ENU 坐标转换 |
| `test_gnss_rtk_handler` | 9 | 4 层假固定检测逻辑 |
| `test_core_no_ros` | 4 | Engine API 构造、ImuFrame/PointCloudFrame 转换 |
| **合计** | **19** | 全部通过 |

---

## 功能回归验证

在 NCLT 20130110 数据集（Velodyne HDL-32，约 24 分钟）上验证重构无功能回归：

| 指标 | 重构前 | 重构后 |
|------|--------|--------|
| 关键帧数 | 1035 | 1035 ✓ |
| 回环数 | 9 | 9 ✓ |
| 地图分块数 | 24 | 24 ✓ |
| 同 binary 两次运行 3D 差 | ~0.30 m | 0.47 m ✓ (< 0.50 m) |

---

## 新增/修改文件清单

### 新增文件

```
src/core/lidar/lidar_pipeline.h/cc       LidarPipeline 封装
src/core/lidar/local_map.h/cc            IVox 地图生命周期
src/core/gnss/gnss_rtk_handler.h/cc      4 层假固定检测
src/core/miao/core/types/
  edge_position_prior.h                  RTK 位置先验边（3×6 Jacobian）
src/core/types/platform_types.h          平台无关数据帧类型
src/core/engine/slam_engine.h/cc         ROS-free SlamEngine
src/core/engine/loc_engine.h/cc          ROS-free LocEngine（含 FeedRtk）
src/adapter/common/coordinate_utils.h/cc WGS84→ECEF→ENU
src/adapter/ros2/ros2_converter.h/cc     ROS ↔ platform types 转换
tests/unit/gnss_rtk_handler_test.cc      RTK 单元测试（9 用例）
tests/unit/coordinate_utils_test.cc      坐标转换单元测试（6 用例）
tests/offline/test_core_no_ros.cc        零 ROS 核心测试（4 用例）
```

### 主要修改文件

```
src/common/options.h                     extern → inline，移除 rclcpp
src/common/params.cc                     参数加载适配
src/core/lio/laser_mapping.h/cc          MapIncremental 委托 LidarPipeline
src/core/lio/imu_processing.hpp          去畸变接口对接
src/core/localization/localization_result.h/cc  移除 geometry_msgs / ToGeoMsg
src/core/localization/localization.h/cpp TFCallback 平台无关化
src/core/localization/pose_graph/pgo.h/cc       新增 ProcessRtk()
src/core/localization/pose_graph/pgo_impl.h/cc  RTK 队列、因子、卡方更新
src/core/system/loc_system.h/cc          适配新 TFCallback
src/CMakeLists.txt                       lightning_core / lightning.libs 拆分
```

---

## 架构变化示意

**重构前：**
```
run_slam_offline.cc
    └─ SlamSystem (直接含 rclcpp::Node)
           └─ LaserMapping (含 sensor_msgs, livox headers)
                  └─ ImuProcess (去畸变混在此处)
                  └─ 全局 extern 变量散落各处
PGO (RTK 字段注释掉)
```

**重构后：**
```
run_slam_offline.cc / run_slam_online.cc (ROS 薄封装)
    └─ SlamEngine  ←── 零 ROS，可独立链接
           └─ SlamSystem (保留内部实现)
                  └─ LaserMapping
                         └─ LidarPipeline (封装点云地图更新)
                         └─ LocalMap (IVox 生命周期)

adapter/ros2/ros2_converter  (唯一含 ROS headers 的 adapter 文件)
    ↕ platform_types (ImuFrame / PointCloudFrame / GnssRtkFrame)

GnssRtkHandler (4 层过滤) → PGO (EdgePositionPrior) → 优化后位姿输出

lightning_core.a  (零 ROS 静态库，可跨平台链接)
lightning.libs.so (ROS 2 动态库，链接 lightning_core)
```
