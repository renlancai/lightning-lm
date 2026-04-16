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

## Build

### Dependencies (Ubuntu 22.04)
```bash
./scripts/install_dep.sh
```
Key deps: ROS 2 Humble+, Pangolin, OpenCV, PCL, yaml-cpp, glog, gflags, pcl_conversions.

### Build & Source
```bash
colcon build
source install/setup.bash
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

## Code Architecture

All source code lives under `src/`. The main shared library is `lightning.libs`.

### `src/app/` — Executables
Entry points: `run_slam_offline.cc`, `run_slam_online.cc`, `run_loc_offline.cc`, `run_loc_online.cc`, plus debug utilities.

### `src/common/` — Shared Types & Config
- `options.h/cc` — Global runtime configuration namespaces (`fasterlio`, `pgo`, `lo`, `map`, `ui`, `lidar_loc`)
- `params.h/cc` — YAML parameter loading into options
- `eigen_types.h`, `std_types.h`, `point_def.h` — Core type aliases
- `keyframe.h`, `nav_state.h`, `imu.h` — Data structures for SLAM state

### `src/core/` — Core Algorithms
- **`lio/`** — LIO front-end:
  - `laser_mapping.cc/h` — Main LIO pipeline (IVox map, ESKF update, keyframe management)
  - `eskf.cc/hpp` — Error-state Kalman filter (12-dim state: pos, vel, rot, bg, ba)
  - `imu_processing.hpp` — IMU pre-integration and initialization
  - `pointcloud_preprocess.cc/h` — LiDAR type-specific point cloud parsing (Velodyne/Ouster/Livox/RoboSense)
- **`loop_closing/`** — Loop closure detection and correction
- **`localization/`** — Localization pipeline:
  - `localization.cpp/h` — Top-level class integrating LIO odom + lidar_loc + PGO
  - `lidar_loc/` — NDT-OMP based scan-to-map matching
  - `pose_graph/` — PGO (pose graph optimization), pose extrapolator, smoother
- **`maps/`** — Tiled map management (`tiled_map.cc/h`, `tiled_map_chunk.cc/h`)
- **`g2p5/`** — 3D-to-2D grid map conversion
- **`ivox3d/`** — Incremental voxel map data structure
- **`miao/`** — Lightweight graph optimization library (replaces g2o): `core/` contains graph, solver, sparse matrix, robust kernels; `utils/` has sampling helpers

### `src/system/` (inside `core/system/`)
- `slam.cc/h` — `SlamSystem`: top-level SLAM orchestrator (owns LIO, LoopClosing, UI, G2P5)
- `loc_system.cc/h` — `LocSystem`: top-level localization orchestrator

### `src/ui/` — Visualization
Pangolin-based 3D viewer (`pangolin_window.cc/h`, `pangolin_window_impl.cc/h`, `ui_car`, `ui_cloud`, `ui_trajectory`).

### `src/io/` — I/O Utilities
- `yaml_io.cc/h` — YAML config reading
- `file_io.cc/h` — Map/PCD file I/O

### `src/wrapper/` — ROS 2 Bag I/O
`bag_io.cc/h` — Reads ROS 2 db3 bags for offline processing.

### `src/utils/` — Utilities
`timer.cc/h`, `pointcloud_utils.cc/h`, `sync.h`, `async_message_process.h`.

### `thirdparty/`
- `livox_ros_driver` — Livox LiDAR ROS 2 driver

### `config/` — YAML Config Files
Per-dataset configs: `default_nclt.yaml`, `default_vbr.yaml`, `default_livox.yaml`, `default_robosense.yaml`. Key sections:
- `common`: topic names, dataset type
- `fasterlio`: lidar_type (1=Livox, 2=Velodyne, 3=Ouster, 4=RoboSense), scan params, ESKF noise
- `system`: feature flags (`with_loop_closing`, `with_ui`, `with_g2p5`), map_path
- `loop_closing`: loop detection thresholds
- `lidar_loc`: scan-to-map matching settings
- `maps`: tiled map loading/unloading policy
- `pgo`: noise parameters for pose graph fusion

## Key Design Patterns

- **Offline vs Online modes**: Offline processes bags synchronously (single-threaded, deterministic). Online uses async message queues (`AsyncMessageProcess<T>`) for real-time throughput.
- **Keyframe-driven pipeline**: LIO generates keyframes; loop closing and localization operate at keyframe granularity.
- **ESKF state**: 12-dimensional (pos, vel, rot, gyro bias, accel bias) — `ba`, `grav`, `offset_R/t` are NOT estimated online.
- **Namespace conventions**: All code in `namespace lightning`. Sub-namespaces: `lightning::loc`, `lightning::ui`, `lightning::g2p5`, `lightning::sys`.

## Debugging Tips

- Set `fasterlio.lidar_type` correctly for your sensor (1/2/3/4).
- `fasterlio.time_scale` is critical for Velodyne — check if per-point timestamps exist and are correct (code in `core/lio/pointcloud_preprocess`).
- Topic names configured via `common.lidar_topic` and `common.imu_topic`.
- `loop_closing.with_height: false` required for multi-floor or staircase environments.
- Convert ROS 1 bags: `rosbags-convert --src [bag.bag] --dst [output_dir]`
