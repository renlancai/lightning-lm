#include <gtest/gtest.h>
#include <cmath>
#include "adapter/common/coordinate_utils.h"

using namespace lightning;
using namespace lightning::adapter;

// ── 测试 1：LlhToEcef 已知参考值（上海，Python WGS84 公式验证）──────
TEST(CoordinateUtils, LlhToEcef_Shanghai) {
    const double lat = 31.2 * M_PI / 180.0;
    const double lon = 121.5 * M_PI / 180.0;
    const double alt = 4.0;

    Vec3d ecef = LlhToEcef(lat, lon, alt);

    // 参考值由 Python WGS84 公式独立计算
    EXPECT_NEAR(ecef.x(), -2853124.759, 1.0);
    EXPECT_NEAR(ecef.y(),  4655876.451, 1.0);
    EXPECT_NEAR(ecef.z(),  3284882.648, 1.0);
}

// ── 测试 2：datum 点本身的 ENU 坐标应为 (0, 0, 0) ──────────────────
TEST(CoordinateUtils, EcefToEnu_SamePointIsZero) {
    const double lat = 31.2 * M_PI / 180.0;
    const double lon = 121.5 * M_PI / 180.0;
    const double alt = 4.0;

    Vec3d datum = LlhToEcef(lat, lon, alt);
    Mat3d R     = BuildRotEcefEnu(lat, lon);
    Vec3d enu   = EcefToEnu(datum, datum, R);

    EXPECT_NEAR(enu.norm(), 0.0, 1e-3);
}

// ── 测试 3：正北 ~100m → ENU.y ≈ 99.6m，ENU.x ≈ 0 ─────────────────
// 注：lat 增量等效距离 < 100m（地球曲率），Python 参考值 99.5989m
TEST(CoordinateUtils, EcefToEnu_NorthIsPositiveY) {
    const double lat0 = 31.2 * M_PI / 180.0;
    const double lon0 = 121.5 * M_PI / 180.0;
    const double alt  = 4.0;

    // 正北 100m 对应的 lat 增量（弧度）
    const double dlat = 100.0 / 6378137.0;

    Vec3d datum = LlhToEcef(lat0, lon0, alt);
    Mat3d R     = BuildRotEcefEnu(lat0, lon0);
    Vec3d pt    = LlhToEcef(lat0 + dlat, lon0, alt);
    Vec3d enu   = EcefToEnu(pt, datum, R);

    EXPECT_NEAR(enu.y(), 99.5989, 0.1);  // North ≈ ENU.Y
    EXPECT_NEAR(enu.x(),  0.0,    0.1);  // East ≈ 0
}

// ── 测试 4：正东 ~100m → ENU.x ≈ 100m，ENU.y ≈ 0 ──────────────────
TEST(CoordinateUtils, EcefToEnu_EastIsPositiveX) {
    const double lat0 = 31.2 * M_PI / 180.0;
    const double lon0 = 121.5 * M_PI / 180.0;
    const double alt  = 4.0;

    // 正东 100m 对应的 lon 增量（弧度）
    const double dlon = 100.0 / (6378137.0 * std::cos(lat0));

    Vec3d datum = LlhToEcef(lat0, lon0, alt);
    Mat3d R     = BuildRotEcefEnu(lat0, lon0);
    Vec3d pt    = LlhToEcef(lat0, lon0 + dlon, alt);
    Vec3d enu   = EcefToEnu(pt, datum, R);

    EXPECT_NEAR(enu.x(), 100.0, 0.2);  // East ≈ ENU.X
    EXPECT_NEAR(enu.y(),   0.0, 0.2);  // North ≈ 0
}

// ── 测试 5：正上 100m → ENU.z = 100m，x/y ≈ 0 ─────────────────────
TEST(CoordinateUtils, EcefToEnu_UpIsPositiveZ) {
    const double lat = 31.2 * M_PI / 180.0;
    const double lon = 121.5 * M_PI / 180.0;

    Vec3d datum = LlhToEcef(lat, lon, 0.0);
    Mat3d R     = BuildRotEcefEnu(lat, lon);
    Vec3d pt    = LlhToEcef(lat, lon, 100.0);
    Vec3d enu   = EcefToEnu(pt, datum, R);

    EXPECT_NEAR(enu.z(), 100.0, 0.1);
    EXPECT_NEAR(enu.x(),   0.0, 0.1);
    EXPECT_NEAR(enu.y(),   0.0, 0.1);
}

// ── 测试 6：旋转矩阵应为正交矩阵（R * R^T ≈ I）─────────────────────
TEST(CoordinateUtils, BuildRotEcefEnu_IsOrthogonal) {
    const double lat = 31.2 * M_PI / 180.0;
    const double lon = 121.5 * M_PI / 180.0;
    Mat3d R = BuildRotEcefEnu(lat, lon);

    Mat3d I = R * R.transpose();
    EXPECT_NEAR((I - Mat3d::Identity()).norm(), 0.0, 1e-10);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
