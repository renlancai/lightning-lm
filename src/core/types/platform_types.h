#pragma once

#include "common/eigen_types.h"
#include <vector>
#include <cstdint>

namespace lightning::core {

// ─── LiDAR 原始点（格式统一后，不含 ROS 类型）───────────────────────
struct RawPoint {
    float x, y, z;
    float intensity;
    double time_offset_sec;  // 相对本帧起始时间，单位秒
};

// ─── LiDAR 帧 ────────────────────────────────────────────────────────
struct PointCloudFrame {
    std::vector<RawPoint> points;
    double timestamp_sec = 0;    // 帧起始时间（Unix 秒）
    uint64_t seq = 0;
};

// ─── IMU 帧 ──────────────────────────────────────────────────────────
struct ImuFrame {
    double timestamp_sec = 0;
    Vec3d acc;   // m/s²，Body 系
    Vec3d gyro;  // rad/s，Body 系
};

// ─── GNSS RTK 帧 ─────────────────────────────────────────────────────
struct GnssRtkFrame {
    double timestamp_sec = 0;

    enum class SolutionType : uint8_t {
        INVALID = 0,
        SPP     = 1,
        SBAS    = 2,
        FLOAT   = 4,
        FIX     = 5,
    };

    SolutionType solution_type  = SolutionType::INVALID;
    int    num_satellites       = 0;
    double hdop                 = 99.0;
    double age_of_corr_sec      = 0.0;

    // WGS84（角度制）
    double lat_deg = 0, lon_deg = 0, alt_m = 0;
    Vec3d  pos_std_enu = Vec3d::Zero();  // ENU 标准差，单位 m

    // 双天线航向（可选）
    bool   heading_valid   = false;
    double heading_rad     = 0;
    double heading_std_rad = 0;
};

}  // namespace lightning::core
