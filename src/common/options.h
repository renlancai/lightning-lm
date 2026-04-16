//
// Created by xiang on 24-4-8.
//

#pragma once
#ifndef LIGHTNING_MODULE_OPTIONS_H
#define LIGHTNING_MODULE_OPTIONS_H

#include <string>

#include <common/constant.h>
#include <common/eigen_types.h>

/// 配置参数
namespace lightning {

namespace debug {

/// debug and save
inline bool flg_exit = false;     // ctrl-c中断
inline bool flg_pause = false;    // 暂停
inline bool flg_next = false;     // 暂停后，放行单个消息(单步调试)
inline float play_speed = 10.0f;  // 播放速度

/// Signal handler: sets flg_exit.  Callers that need rclcpp::shutdown() should
/// add it themselves (SlamSystem/LocSystem register this and their own cleanup).
inline void SigHandle(int /*sig*/) {
    debug::flg_exit = true;
}

}  // namespace debug

namespace lo {
inline float lidar_time_interval = 0.1f;          // 雷达的扫描时间
inline bool use_dr_rotation = true;
inline int relative_pose_check_cloud_size = 2;
inline double parking_speed = 0.05;
inline double parking_count = 5;
inline double kf_pose_check_distance = 0.5;
inline double kf_pose_check_angle = 5.0;
inline double pose2d_roll_limit = 8.0;
inline double pose2d_pitch_limit = 10.0;
inline double pose2d_relative_z_limit = 0.25;
}  // namespace lo

/// 地图配置
namespace map {
inline std::string map_path = "";          // 地图路径
inline Vec3d map_origin = Vec3d::Zero();   // 地图原点
}  // namespace map

/// PGO 配置
namespace pgo {

constexpr int PGO_MAX_FRAMES = 5;                               // PGO所持的最大帧数
constexpr int PGO_MAX_SIZE_OF_RELATIVE_POSE_QUEUE = 10000;      // PGO 相对定位队列最大长度
constexpr int PGO_MAX_SIZE_OF_RTK_POSE_QUEUE = 200;             // PGO RTK观测队列最大长度
constexpr double PGO_DISTANCE_TH_LAST_FRAMES = 2.5;             // PGO 滑窗时，最近两帧的最小距离
constexpr double PGO_ANGLE_TH_LAST_FRAMES = 10 * M_PI / 360.0;  // PGO 滑窗时，最近两帧的最小角度

/// 噪声参数（legacy — PGOImpl::Options is the canonical source）
inline double lidar_loc_pos_noise = 0.3;
inline double lidar_loc_ang_noise = 1.0 * constant::kDEG2RAD;
inline double lidar_loc_outlier_th = 30.0;
inline double lidar_odom_pos_noise = 0.3;
inline double lidar_odom_ang_noise = 1.0 * constant::kDEG2RAD;
inline double lidar_odom_outlier_th = 0.01;
inline double dr_pos_noise = 1.0;
inline double dr_ang_noise = 0.5 * constant::kDEG2RAD;
inline double dr_pos_noise_ratio = 1.0;
inline double pgo_frame_converge_pos_th = 0.05;
inline double pgo_frame_converge_ang_th = 1.0 * constant::kDEG2RAD;
inline double pgo_smooth_factor = 0.01;
}  // namespace pgo

// ui
namespace ui {
inline int pgo_res_rows = 16;   // pgo发送滑窗数据给ui的矩阵的行数
inline float opacity = 0.2f;   // 点云透明度
}  // namespace ui

// lidar_loc
namespace lidar_loc {
inline int grid_search_angle_step = 20;        // 角度网格搜索步数（关键参数）
inline double grid_search_angle_range = 20.0;  // 角度搜索半径(角度制，关键参数,左右各有)
}  // namespace lidar_loc

/// fasterlio 配置
namespace fasterlio {

/// fixed params
constexpr double INIT_TIME = 0.1;
constexpr int NUM_MATCH_POINTS = 5;      // required matched points in current
constexpr int MIN_NUM_MATCH_POINTS = 3;  // minimum matched points in current

/// configurable params — Phase 3: these are now loaded directly into
/// LaserMapping / LidarPipeline options; these inline globals are kept only
/// as fallback defaults for any remaining consumers.
inline int NUM_MAX_ITERATIONS = 4;          // max iterations of ekf
inline float ESTI_PLANE_THRESHOLD = 0.1f;  // plane threshold
}  // namespace fasterlio

}  // namespace lightning

#endif
