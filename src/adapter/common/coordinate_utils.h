#pragma once

#include "common/eigen_types.h"
#include <cmath>

namespace lightning::adapter {

/**
 * WGS84 → ECEF
 * @param lat_rad  纬度（弧度）
 * @param lon_rad  经度（弧度）
 * @param alt_m    椭球高（米）
 * @return ECEF 坐标（米）
 */
Vec3d LlhToEcef(double lat_rad, double lon_rad, double alt_m);

/**
 * 构造 ECEF → ENU 旋转矩阵（基于 datum 点的经纬度）
 * @param lat_rad  datum 点纬度（弧度）
 * @param lon_rad  datum 点经度（弧度）
 * @return 3×3 旋转矩阵，将 ECEF 差向量转到 ENU
 */
Mat3d BuildRotEcefEnu(double lat_rad, double lon_rad);

/**
 * ECEF → ENU（相对 datum）
 * @param point_ecef   待转换点 ECEF 坐标（米）
 * @param datum_ecef   datum 点 ECEF 坐标（米）
 * @param R_ecef_enu   BuildRotEcefEnu 返回的旋转矩阵
 * @return ENU 坐标（米）
 */
Vec3d EcefToEnu(const Vec3d& point_ecef,
                const Vec3d& datum_ecef,
                const Mat3d& R_ecef_enu);

}  // namespace lightning::adapter
