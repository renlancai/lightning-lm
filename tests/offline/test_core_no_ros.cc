// Phase 5: Zero-ROS compile check.
//
// This file verifies that the SlamEngine and LocEngine *headers* have no
// ROS dependencies.  It includes only the engine headers (and standard
// library) — no <rclcpp/...>, no <sensor_msgs/...>.
//
// Build:  compiled as part of tests/ when GTest is found (see CMakeLists).
// The binary links against lightning.libs, which does use ROS internally, but
// the *source code* here is provably ROS-free.

#include <gtest/gtest.h>

// ── Engine headers (must not pull in any ROS type) ───────────────────────────
#include "core/engine/slam_engine.h"
#include "core/engine/loc_engine.h"

// Ensure no ROS macros leaked in:
#ifdef RCLCPP_VERSION_MAJOR
#error "ROS rclcpp headers were pulled in via engine headers — Phase 5 violated!"
#endif
#ifdef __ROS_MSG_TYPES_HPP__
#error "sensor_msgs pulled in via engine headers — Phase 5 violated!"
#endif

// ── Instantiation smoke tests ────────────────────────────────────────────────

TEST(Phase5_ZeroRos, SlamEngineConstructs) {
    lightning::core::SlamEngine::Options opts;
    opts.with_ui           = false;
    opts.with_loop_closing = false;
    opts.with_gridmap      = false;
    lightning::core::SlamEngine engine(opts);
    // Just verify construction doesn't crash and GetAllKeyframes is callable
    EXPECT_TRUE(engine.GetAllKeyframes().empty());
}

TEST(Phase5_ZeroRos, LocEngineConstructs) {
    lightning::core::LocEngine::Options opts;
    opts.pub_tf     = false;
    opts.enable_rtk = false;
    lightning::core::LocEngine engine(opts);
    // FeedImu/FeedLidar/FeedRtk must be callable (no-op before Init/SetInitPose)
    lightning::core::ImuFrame imu{};
    engine.FeedImu(imu);   // should silently no-op

    lightning::core::PointCloudFrame frame{};
    engine.FeedLidar(frame);  // should silently no-op
}

TEST(Phase5_ZeroRos, ImuFrameRoundTrip) {
    lightning::core::ImuFrame f;
    f.timestamp_sec = 1234.5;
    f.acc  = Eigen::Vector3d(0.1, 0.2, 9.8);
    f.gyro = Eigen::Vector3d(0.01, -0.01, 0.0);

    EXPECT_DOUBLE_EQ(f.timestamp_sec, 1234.5);
    EXPECT_NEAR(f.acc.z(), 9.8, 1e-9);
}

TEST(Phase5_ZeroRos, PointCloudFrameConvertible) {
    lightning::core::PointCloudFrame frame;
    frame.timestamp_sec = 100.0;
    frame.points.resize(3);
    frame.points[0] = {1.f, 2.f, 3.f, 100.f, 0.0};
    frame.points[1] = {4.f, 5.f, 6.f, 200.f, 0.01};
    frame.points[2] = {7.f, 8.f, 9.f, 300.f, 0.02};

    EXPECT_EQ(frame.points.size(), 3u);
    EXPECT_FLOAT_EQ(frame.points[2].z, 9.f);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
