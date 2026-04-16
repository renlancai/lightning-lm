#pragma once

/**
 * Platform Adapter — ROS 2 ↔ platform-agnostic types.
 *
 * This is the ONLY file in src/ that may include ROS message headers.
 * Core algorithm code must NOT include this header.
 */

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include "common/imu.h"
#include "core/localization/localization_result.h"
#include "core/types/platform_types.h"

namespace lightning::adapter::ros2 {

// ── Time helpers (kept for backward-compat with bag_io / online nodes) ───────
inline double ToSec(const builtin_interfaces::msg::Time& t) {
    return static_cast<double>(t.sec) + 1e-9 * static_cast<double>(t.nanosec);
}
inline uint64_t ToNanoSec(const builtin_interfaces::msg::Time& t) {
    return static_cast<uint64_t>(t.sec) * 1'000'000'000ULL + t.nanosec;
}

// ── IMU ───────────────────────────────────────────────────────────────────────

/// Convert ROS Imu message to platform-agnostic ImuFrame.
core::ImuFrame FromImu(const sensor_msgs::msg::Imu& msg);

/// Produce the legacy lightning::IMU (still required by LaserMapping internals).
lightning::IMUPtr FromImuLegacy(const sensor_msgs::msg::Imu::SharedPtr& msg);

// ── PointCloud2 ───────────────────────────────────────────────────────────────

/**
 * Convert a ROS PointCloud2 to a platform-agnostic PointCloudFrame.
 *
 * @param msg        The incoming ROS message.
 * @param time_scale Factor to convert the per-point `time` field to seconds.
 *                   Velodyne (rosbag2 driver): typically 1e-3 ms → s.
 *                   Ouster / Livox: 1e-9 ns → s.
 *                   Default 1.0 assumes the field is already in seconds.
 */
core::PointCloudFrame FromPointCloud2(const sensor_msgs::msg::PointCloud2& msg,
                                      double time_scale = 1.0);

// ── NavSatFix ─────────────────────────────────────────────────────────────────

/// Convert a ROS NavSatFix message to a platform-agnostic GnssRtkFrame.
/// solution_type is inferred from NavSatStatus; covariance diagonal is used
/// as ENU position std-dev when covariance_type is not UNKNOWN.
core::GnssRtkFrame FromNavSatFix(const sensor_msgs::msg::NavSatFix& msg);

// ── LocalizationResult → ROS ─────────────────────────────────────────────────

/// Convert a LocalizationResult to a geometry_msgs::TransformStamped.
/// (Moved from LocalizationResult::ToGeoMsg() to keep the core ROS-free.)
geometry_msgs::msg::TransformStamped LocResultToGeoMsg(const loc::LocalizationResult& r);

}  // namespace lightning::adapter::ros2
