# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Lightning-LM is a complete 3D LiDAR SLAM (mapping) and localization system for ROS 2. It features:
- **AA-FasterLIO** LIO front-end for fast laser-inertial odometry
- Real-time loop closure detection and back-end optimization
- 3D-to-2D map conversion (g2p5) for grid maps
- High-frequency IMU-based pose output (100Hz)
- Dynamic/static map layer separation for dynamic scene handling
- Tiled map partitions for large-scale environments
- Lightweight in-house graph optimization library (`miao`, derived from g2o)
- RTK/GNSS fusion with 4-layer false-fix detection (Phase 4)

## Build

### Dependencies (Ubuntu 22.04)
```bash
./scripts/install_dep.sh
```
Key deps: ROS 2 Humble+, Pangolin, OpenCV, PCL, yaml-cpp, glog, gflags, pcl_conversions, GTest.

### Build & Source
```bash
colcon build
source install/setup.bash
```

### Single-package build (faster iteration)
```bash
colcon build --packages-select lightning
```

### Run Programs
```bash
# Offline SLAM (faster, processes bag fully)
ros2 run lightning run_slam_offline --config ./config/default_nclt.yaml --input_bag [bag_file]

# Online SLAM (real-time sensor input)
ros2 run lightning run_slam_online --config ./config/default_nclt.yaml

# Offline localization
ros2 run lightning run_loc_offline --config ./config/default_nclt.yaml --input_bag [bag_file]

# Online localization
ros2 run lightning run_loc_online --config ./config/default_nclt.yaml

# Save map during online SLAM
ros2 service call /lightning/save_map lightning/srv/SaveMap "{map_id: new_map}"
```

### Docker
```bash
# Pull pre-built image
docker pull docker.cnb.cool/gpf2025/slam:demo

# Or build locally
cd docker && docker build -t lighting_lm_image .
```

### Tests
Tests only compile when GTest is found. Run individual tests:
```bash
# Unit tests (no ROS needed at runtime)
./build/unit/coordinate_utils_test
./build/unit/gnss_rtk_handler_test

# Zero-ROS engine smoke test
./build/unit/test_core_no_ros

# Snapshot verification (requires golden data + SLAM run)
./build/unit/verify_snapshot
```

## Code Architecture

All source code lives under `src/`. There are **two libraries** — a ROS-free core and a ROS-dependent wrapper:

### `lightning_core` (STATIC, no ROS deps)
Pure C++17 algorithms and types. Any file in this library must NOT include ROS headers.
- `common/` — `nav_state`, `options`, `params`, `eigen_types`, `std_types`, `point_def`, `keyframe`, `imu`
- `adapter/common/` — `coordinate_utils` (platform-agnostic coordinate transforms)
- `core/lio/eskf` — Error-state Kalman filter (12-dim state)
- `core/loop_closing/` — Loop closure detection
- `core/lidar/` — `lidar_pipeline`, `local_map` (extracted from LaserMapping)
- `core/gnss/` — `gnss_rtk_handler` (4-layer false-fix detection)
- `core/g2p5/g2p5_subgrid` — Grid map subgrid
- `core/maps/` — `tiled_map`, `tiled_map_chunk`
- `core/localization/localization_result` — Result type (ROS-free after refactor)
- `core/localization/pose_graph/` — `pgo`, `pgo_impl`, `pose_extrapolator`
- `io/` — `yaml_io`, `file_io`
- `utils/` — `timer`, `pointcloud_utils`
- `core/miao/` — Lightweight graph optimization (its own CMakeLists, linked as `miao.core` + `miao.utils`)
- `core/types/platform_types.h` — `ImuFrame`, `PointCloudFrame`, `GnssRtkFrame`, `RawPoint`

### `lightning.libs` (SHARED, depends on ROS)
Thin ROS-dependent layer over `lightning_core`. Links against `lightning_core` and ROS 2 packages.
- `adapter/ros2/ros2_converter` — **The ONLY file that may include ROS message headers**. Converts between ROS messages and platform-agnostic types.
- `core/lio/laser_mapping`, `pointcloud_preprocess` — LIO pipeline internals (need sensor_msgs)
- `core/engine/slam_engine`, `loc_engine` — Platform-agnostic orchestrators (zero ROS includes in headers)
- `core/g2p5/g2p5_map`, `g2p5` — Full g2p5 pipeline
- `core/localization/lidar_loc/` — NDT-OMP scan-to-map matching
- `core/localization/localization` — Top-level localization class
- `ui/` — Pangolin-based visualization
- `core/system/slam`, `loc_system` — Legacy ROS-coupled orchestrators
- `wrapper/bag_io` — ROS 2 db3 bag reading

### `src/app/` — Executables
Entry points: `run_slam_offline.cc`, `run_slam_online.cc`, `run_loc_offline.cc`, `run_loc_online.cc`, plus debug utilities (`run_frontend_offline`, `run_loop_offline`, `test_ui`).

### `config/` — YAML Config Files
Per-dataset configs: `default_nclt.yaml`, `default_vbr.yaml`, `default_livox.yaml`, `default_robosense.yaml`. Key sections:
- `common`: topic names, dataset type
- `fasterlio`: lidar_type (1=Livox, 2=Velodyne, 3=Ouster, 4=RoboSense), scan params, ESKF noise
- `system`: feature flags (`with_loop_closing`, `with_ui`, `with_g2p5`), map_path
- `loop_closing`: loop detection thresholds
- `lidar_loc`: scan-to-map matching settings
- `maps`: tiled map loading/unloading policy
- `pgo`: noise parameters for pose graph fusion (including RTK noise params)

## Key Design Patterns

- **Two-library split**: `lightning_core` (no ROS) + `lightning.libs` (ROS wrapper). New algorithm code goes in `lightning_core`; ROS adapters go in `adapter/ros2/`.
- **Platform-agnostic engine API**: `SlamEngine` and `LocEngine` have zero ROS includes in their headers. They expose `FeedImu()`, `FeedLidar()`, `FeedRtk()` taking `core::ImuFrame`/`PointCloudFrame`/`GnssRtkFrame`. The `app/` entry points are thin ROS wrappers that call `adapter::ros2::FromImu()` etc. then feed into the engine.
- **Offline vs Online modes**: Offline processes bags synchronously (single-threaded, deterministic). Online uses async message queues (`AsyncMessageProcess<T>`) for real-time throughput.
- **Keyframe-driven pipeline**: LIO generates keyframes; loop closing and localization operate at keyframe granularity.
- **ESKF state**: 12-dimensional (pos, vel, rot, gyro bias, accel bias) — `ba`, `grav`, `offset_R/t` are NOT estimated online.
- **Namespace conventions**: All code in `namespace lightning`. Sub-namespaces: `lightning::loc`, `lightning::ui`, `lightning::g2p5`, `lightning::sys`, `lightning::core`, `lightning::adapter::ros2`.
- **GCC two-constructor pattern**: `explicit Foo(Options opts = Options())` fails if `Options` uses default member initializers on GCC. Use:
  ```cpp
  // .h:
  Foo();
  explicit Foo(Options opts);
  // .cc:
  Foo::Foo() : Foo(Options{}) {}
  Foo::Foo(Options opts) : options_(opts) {}
  ```
- **Options as inline globals** (Phase 3): `extern X foo` replaced with `inline X foo = val` in `common/options.h`. Namespace-scoped globals like `fasterlio::NUM_MAX_ITERATIONS` were moved to class members.

## Debugging Tips

- Set `fasterlio.lidar_type` correctly for your sensor (1/2/3/4).
- `fasterlio.time_scale` is critical for Velodyne — check if per-point timestamps exist and are correct (code in `core/lio/pointcloud_preprocess`).
- Topic names configured via `common.lidar_topic` and `common.imu_topic`.
- `loop_closing.with_height: false` required for multi-floor or staircase environments.
- Convert ROS 1 bags: `rosbags-convert --src [bag.bag] --dst [output_dir]`
