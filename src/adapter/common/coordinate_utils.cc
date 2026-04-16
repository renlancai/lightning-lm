#include "adapter/common/coordinate_utils.h"
#include <cmath>

namespace lightning::adapter {

// WGS84 椭球参数
static constexpr double kWgs84A  = 6378137.0;           // 长半轴（米）
static constexpr double kWgs84E2 = 0.00669437999014;    // 第一偏心率²

Vec3d LlhToEcef(double lat_rad, double lon_rad, double alt_m) {
    const double sin_lat = std::sin(lat_rad);
    const double cos_lat = std::cos(lat_rad);
    const double sin_lon = std::sin(lon_rad);
    const double cos_lon = std::cos(lon_rad);

    const double N = kWgs84A / std::sqrt(1.0 - kWgs84E2 * sin_lat * sin_lat);

    return Vec3d{
        (N + alt_m) * cos_lat * cos_lon,
        (N + alt_m) * cos_lat * sin_lon,
        (N * (1.0 - kWgs84E2) + alt_m) * sin_lat
    };
}

Mat3d BuildRotEcefEnu(double lat_rad, double lon_rad) {
    const double sin_lat = std::sin(lat_rad);
    const double cos_lat = std::cos(lat_rad);
    const double sin_lon = std::sin(lon_rad);
    const double cos_lon = std::cos(lon_rad);

    Mat3d R;
    // 行：East, North, Up
    R << -sin_lon,              cos_lon,             0.0,
         -sin_lat * cos_lon,   -sin_lat * sin_lon,   cos_lat,
          cos_lat * cos_lon,    cos_lat * sin_lon,   sin_lat;
    return R;
}

Vec3d EcefToEnu(const Vec3d& point_ecef,
                const Vec3d& datum_ecef,
                const Mat3d& R_ecef_enu) {
    return R_ecef_enu * (point_ecef - datum_ecef);
}

}  // namespace lightning::adapter
