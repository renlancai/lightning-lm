# Lightning-LM 使用手册

## 目录

1. [环境要求](#1-环境要求)
2. [编译](#2-编译)
   - [本机编译](#21-本机编译)
   - [Docker 方式](#22-docker-方式)
3. [快速开始：建图](#3-快速开始建图)
   - [离线建图（推荐调试用）](#31-离线建图)
   - [在线建图（实车）](#32-在线建图)
4. [快速开始：定位](#4-快速开始定位)
   - [离线定位](#41-离线定位)
   - [在线定位](#42-在线定位)
5. [配置文件说明](#5-配置文件说明)
   - [传感器选择](#51-传感器选择)
   - [关键参数](#52-关键参数)
6. [RTK/GNSS 支持](#6-rtkgnss-支持)
7. [地图管理](#7-地图管理)
8. [测试](#8-测试)
   - [单元测试](#81-单元测试)
   - [零 ROS 核心测试](#82-零-ros-核心测试)
   - [功能回归基准](#83-功能回归基准)
9. [非 ROS 集成（Engine API）](#9-非-ros-集成engine-api)
10. [常见问题](#10-常见问题)

---

## 1. 环境要求

| 项目 | 要求 |
|------|------|
| 操作系统 | Ubuntu 22.04 |
| ROS 版本 | ROS 2 Humble |
| 编译器 | GCC 11+ / C++17 |

---

## 2. 编译

### 2.1 本机编译

**安装系统依赖：**

```bash
sudo apt install \
    libopencv-dev libpcl-dev pcl-tools \
    libyaml-cpp-dev libgoogle-glog-dev libgflags-dev \
    libboost-all-dev libeigen3-dev \
    ros-humble-pcl-conversions ros-humble-pcl-ros
```

**安装 Pangolin（可视化依赖，不需要 UI 可跳过）：**

```bash
git clone --recurse-submodules https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc) && sudo make install
sudo ldconfig
```

**创建 ROS 2 工作空间并编译：**

```bash
mkdir -p ~/slam_ws/src && cd ~/slam_ws/src
git clone https://github.com/gaoxiang12/lightning-lm.git

cd ~/slam_ws
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

> 后续每次开终端都需要执行 `source /opt/ros/humble/setup.bash && source ~/slam_ws/install/setup.bash`，
> 或者写入 `~/.bashrc`。

编译产物包含两个库：
- `liblightning_core.a` — 纯 C++17 静态库，零 ROS 依赖
- `liblightning.libs.so` — ROS 2 动态库，链接 `lightning_core`

### 2.2 Docker 方式

**方式一：拉取预编译镜像（推荐）**

```bash
docker pull docker.cnb.cool/gpf2025/slam:demo
docker run -it --name slam_demo \
    -v /path/to/your/data:/data \
    docker.cnb.cool/gpf2025/slam:demo
```

**方式二：本地构建镜像**

```bash
cd docker
docker build -t lightning_lm_image .
docker run -it --name slam_demo \
    -v /path/to/your/data:/data \
    lightning_lm_image
```

**在容器内使用：**

```bash
# 进入已运行的容器
docker exec -it slam_demo bash

# 激活环境
source /opt/ros/humble/setup.bash
source /root/slam_ws/install/setup.bash
```

---

## 3. 快速开始：建图

### 3.1 离线建图

从 ROS 2 bag 文件离线处理，速度快、结果确定，适合调试和验证。

```bash
ros2 run lightning run_slam_offline \
    --config ./config/default_nclt.yaml \
    --input_bag /path/to/your.db3
```

建图完成后，地图默认保存到配置文件中 `system.map_path` 指定的目录（默认 `./data/new_map/`）。

**无显示器运行（服务器/Docker 无 GUI 环境）：**

```bash
# 先修改配置：将 system.with_ui: true 改为 false
sed 's/with_ui: true/with_ui: false/' config/default_nclt.yaml > /tmp/noui.yaml
ros2 run lightning run_slam_offline \
    --config /tmp/noui.yaml \
    --input_bag /path/to/your.db3
```

### 3.2 在线建图

实车运行，实时订阅 ROS 2 话题。

```bash
ros2 run lightning run_slam_online \
    --config ./config/default_nclt.yaml
```

**运行中保存地图：**

```bash
# 在另一个终端执行
ros2 service call /lightning/save_map lightning/srv/SaveMap "{map_id: new_map}"
```

---

## 4. 快速开始：定位

定位需要先有地图，确保 `system.map_path` 指向已建好的地图目录。

### 4.1 离线定位

```bash
ros2 run lightning run_loc_offline \
    --config ./config/default_nclt.yaml \
    --input_bag /path/to/your.db3
```

### 4.2 在线定位

```bash
ros2 run lightning run_loc_online \
    --config ./config/default_nclt.yaml
```

---

## 5. 配置文件说明

`config/` 目录下提供了多个传感器的参考配置：

| 文件 | 适用场景 |
|------|---------|
| `default_nclt.yaml` | NCLT 数据集，Velodyne HDL-32 |
| `default_livox.yaml` | Livox Mid-360 / Avia |
| `default_robosense.yaml` | RoboSense RS-LiDAR-32 |
| `default_vbr.yaml` | VBR 数据集 |

### 5.1 传感器选择

修改配置文件中的以下字段：

```yaml
common:
  lidar_topic: "points_raw"   # 点云话题名
  imu_topic: "imu_raw"        # IMU 话题名

fasterlio:
  lidar_type: 2    # 1=Livox, 2=Velodyne, 3=Ouster, 4=RoboSense
```

### 5.2 关键参数

**LIO 前端（`fasterlio` 段）：**

| 参数 | 说明 | 常用值 |
|------|------|-------|
| `lidar_type` | 雷达型号 | 1/2/3/4 |
| `scan_line` | 线束数 | 16/32/64/128 |
| `blind` | 近距离盲区（米），小于此距离的点丢弃 | 0.1~1.0 |
| `time_scale` | 点云时间戳缩放，**Velodyne 必须设置** | `1e-3` |
| `point_filter_num` | 点云稀疏化步长，越大越稀 | 4~8 |
| `filter_size_scan` | 当前帧降采样体素大小（米） | 0.3~0.5 |
| `filter_size_map` | 地图降采样体素大小（米） | 0.3~0.5 |
| `extrinsic_T` | LiDAR 相对 IMU 的平移外参（米） | 根据标定结果填写 |
| `extrinsic_R` | LiDAR 相对 IMU 的旋转外参（行优先 3×3） | 根据标定结果填写 |

**系统功能开关（`system` 段）：**

| 参数 | 说明 | 默认 |
|------|------|------|
| `with_loop_closing` | 是否启用回环检测 | `true` |
| `with_ui` | 是否启用 Pangolin 3D 可视化 | `true` |
| `with_g2p5` | 是否生成 2D 栅格地图 | `false` |
| `map_path` | 地图读写路径 | `./data/new_map/` |

**回环检测（`loop_closing` 段）：**

| 参数 | 说明 |
|------|------|
| `max_range` | 候选回环帧的最大搜索距离（米） |
| `ndt_score_th` | NDT 配准分值阈值，越小越严 |
| `with_height` | 是否考虑高度差，**多楼层/楼梯场景设为 `false`** |

**PGO 噪声（`pgo` 段）：**

| 参数 | 说明 |
|------|------|
| `lidar_loc_pos_noise` | 激光定位位置噪声（米） |
| `lidar_odom_pos_noise` | 激光里程计位置噪声（米） |
| `smooth_factor` | 输出位姿平滑系数，越小越平滑但延迟更大 |

---

## 6. RTK/GNSS 支持

系统内置四层假固定检测滤波器（`GnssRtkHandler`），通过 PGO 后端融合 RTK 位置约束。

**在线模式下接入 RTK：**

`run_loc_online` 会自动订阅 `sensor_msgs/NavSatFix` 话题（话题名通过配置文件 `common.gnss_topic` 指定）。RTK 观测经过以下四层过滤后融入 PGO：

| 层级 | 过滤条件 |
|------|---------|
| Layer 1 | 解类型必须为 FIX，卫星数 ≥ 6，HDOP ≤ 2.0 |
| Layer 2 | 位置跳变检测（相邻帧距离 > 阈值则拒绝）|
| Layer 3 | 连续有效帧数 ≥ 3 才接受 |
| Layer 4 | 卡方检验：残差 > 3σ 则拒绝 |

**坐标系：** RTK 原始经纬高自动转换为 ENU 局部坐标，首次有效 FIX 自动设为原点（datum）。

**PGO 噪声（`pgo` 段）：**

```yaml
pgo:
  rtk_pos_noise: 0.1        # RTK 位置噪声 1σ（米），越小权重越大
```

---

## 7. 地图管理

**地图目录结构：**

```
data/new_map/
├── index.txt          # 地图元信息（分块索引）
├── chunk_0_0.pcd      # 分块点云（Tiled Map 格式）
├── chunk_0_1.pcd
└── ...
```

**更换定位用地图：**

修改配置文件中的 `system.map_path`，指向已建好的地图目录，然后启动定位程序即可。

**从 ROS 1 bag 转换：**

```bash
pip install rosbags
rosbags-convert --src your_bag.bag --dst output_dir/
```

---

## 8. 测试

### 8.1 单元测试

单元测试独立于 ROS，可直接用 CMake 编译运行：

```bash
cd tests/unit
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4

./test_coordinate_utils    # 坐标转换（WGS84 ↔ ECEF ↔ ENU），6 个用例
./test_gnss_rtk_handler    # RTK 假固定检测（4 层过滤），9 个用例
```

### 8.2 零 ROS 核心测试

`test_core_no_ros` 验证 `lightning_core` 静态库不引入任何 ROS 依赖，同时测试 Engine API 可独立构造：

```bash
# colcon 编译后直接运行（无需 ROS 环境）
./bin/test_core_no_ros

# 4 个用例：
#   Phase5_ZeroRos.SlamEngineConstructs
#   Phase5_ZeroRos.LocEngineConstructs
#   Phase5_ZeroRos.ImuFrameRoundTrip
#   Phase5_ZeroRos.PointCloudFrameConvertible
```

确认静态库无 ROS 符号：

```bash
nm -u build/lightning/src/liblightning_core.a | \
    grep -E 'rclcpp|sensor_msgs|geometry_msgs' || echo "clean"
```

### 8.3 功能回归基准

在 NCLT 20130110 数据集上的回归指标：

| 指标 | 基准值 |
|------|-------|
| 关键帧总数 | 1035 |
| 检测回环数 | 9 |
| 同 binary 两次运行终点 3D 差 | < 0.50 m |

```bash
ros2 run lightning run_slam_offline \
    --config /tmp/nclt_noui.yaml \
    --input_bag /data/NCLT/20130110/20130110.db3 2>&1 | \
    grep -E 'called times|loops:'
# 期望输出：called times : 1035，optimize finished, loops: 9
```

---

## 9. 非 ROS 集成（Engine API）

`lightning_core` 是零 ROS 依赖的静态库，可在任意 C++17 项目中使用。通过 `SlamEngine` / `LocEngine` 直接喂数据：

```cpp
#include "core/engine/slam_engine.h"
#include "core/engine/loc_engine.h"
#include "core/types/platform_types.h"

// ── 建图 ──────────────────────────────────────────────
lightning::SlamEngine slam;
slam.Init("config/default_nclt.yaml");

// 喂 IMU
core::ImuFrame imu;
imu.timestamp_sec = 1234567890.123;
imu.acc  = {0, 0, 9.8};
imu.gyro = {0, 0, 0};
slam.FeedImu(imu);

// 喂点云
core::PointCloudFrame cloud;
cloud.timestamp_sec = imu.timestamp_sec;
// ... 填充 cloud.points
slam.FeedLidar(cloud);

// ── 定位 ──────────────────────────────────────────────
lightning::LocEngine loc;
loc.Init("config/default_nclt.yaml", "/path/to/map");

// 额外融合 RTK（可选）
core::GnssRtkFrame rtk;
rtk.timestamp_sec   = 1234567890.0;
rtk.lat_deg = 48.123; rtk.lon_deg = 11.456; rtk.alt_m = 530.0;
rtk.solution_type = core::GnssRtkFrame::SolutionType::FIX;
loc.FeedRtk(rtk);
```

CMakeLists.txt 中只需链接 `lightning_core`，无需 `ament_target_dependencies`：

```cmake
find_package(lightning_core REQUIRED)
target_link_libraries(your_target lightning_core)
```

---

## 10. 常见问题

**Q: 运行报错 `Pangolin X11: Failed to open X display`**

在无 GUI 环境（服务器、Docker headless）下，需关闭可视化：

```yaml
system:
  with_ui: false
```

**Q: Velodyne 点云没有时间戳，轨迹发散**

检查 `fasterlio.time_scale`，Velodyne 驱动输出的时间单位为毫秒，需设为 `1e-3`：

```yaml
fasterlio:
  time_scale: 1e-3
```

**Q: 建图时回环检测误匹配导致地图扭曲**

适当调高 `loop_closing.ndt_score_th`（建议 1.3~2.0），或减小 `max_range`：

```yaml
loop_closing:
  ndt_score_th: 1.5
  max_range: 20.0
```

**Q: 多楼层/楼梯建图出现楼层混叠**

```yaml
loop_closing:
  with_height: false
```

**Q: `libpango_plot.so.0: cannot open shared object file`**

首次编译安装 Pangolin 后需刷新动态库缓存：

```bash
sudo ldconfig
```

**Q: 定位启动后迟迟不收敛**

- 检查 `system.map_path` 路径是否正确
- 适当调低 `lidar_loc.min_init_confidence`（如从 1.8 降至 1.5）
- 检查点云话题名与配置文件 `common.lidar_topic` 是否一致

**Q: RTK 量测始终被拒绝**

- 检查 Layer 1：解类型是否为 FIX（`NavSatStatus::STATUS_FIX`）
- 检查 Layer 3：连续有效帧数需 ≥ 3，刚启动时会有短暂抑制期
- 适当放宽 `pgo.rtk_pos_noise` 增大容忍区间
