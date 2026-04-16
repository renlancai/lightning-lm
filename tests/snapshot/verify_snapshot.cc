/**
 * verify_snapshot — Phase 1+ snapshot regression test.
 *
 * Compares the output of the refactored code against golden data captured
 * during Phase 0.  Run after each refactoring phase with the same bag.
 *
 * Environment variables (or gflags):
 *   --golden_dir  – path to the directory with golden_NNNN.bin files
 *   --bag         – bag to replay
 *   --config      – YAML config
 */

#ifndef LIGHTNING_SNAPSHOT_DEBUG
#error "verify_snapshot must be compiled with -DLIGHTNING_SNAPSHOT_DEBUG"
#endif

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include "core/lio/laser_mapping.h"
#include "wrapper/bag_io.h"
#include "tests/snapshot/golden_io.h"

DEFINE_string(golden_dir,  "tests/snapshot/golden",    "Golden data directory");
DEFINE_string(bag,         "",                          "Bag file to replay");
DEFINE_string(config,      "./config/default_nclt.yaml","Config YAML");
DEFINE_int32 (num_frames,  100,                         "Number of frames to verify");

namespace lsnap = lightning::snapshot;
using lightning::LaserMapping;
using lightning::RosbagIO;
using lightning::IMUPtr;

// ── Global actual-run results ─────────────────────────────────────────────────
static std::vector<lsnap::CheckPoint> g_actual;
static bool g_replay_ok = false;

static lsnap::CheckPoint ConvertCp(const LaserMapping::CheckPoint& src) {
    lsnap::CheckPoint dst;
    dst.frame_id       = src.frame_id;
    dst.imu_timestamps = src.imu_timestamps;
    dst.undistort_pts  = src.undistort_pts;
    dst.HTH            = src.HTH;
    dst.HTr            = src.HTr;
    dst.valid_num      = src.valid_num;
    dst.pos            = src.pos;
    dst.vel            = src.vel;
    dst.bg             = src.bg;
    dst.rot            = src.rot;
    dst.is_keyframe    = src.is_keyframe;
    dst.kf_pos         = src.kf_pos;
    dst.kf_rot         = src.kf_rot;
    return dst;
}

static void ReplayBag() {
    LaserMapping lio;
    if (!lio.Init(FLAGS_config)) {
        LOG(ERROR) << "LaserMapping::Init failed";
        return;
    }

    lio.SetDebugCallback([](const LaserMapping::CheckPoint& src) {
        g_actual.push_back(ConvertCp(src));
    });

    int lidar_cnt = 0;
    RosbagIO bag(FLAGS_bag);
    bag.AddImuHandle("imu_raw", [&](IMUPtr imu) {
            if (lidar_cnt >= FLAGS_num_frames) return false;
            lio.ProcessIMU(imu);
            return true;
        })
       .AddPointCloud2Handle("points_raw", [&](auto msg) {
            if (lidar_cnt >= FLAGS_num_frames) return false;
            lio.ProcessPointCloud2(msg);
            lio.Run();
            ++lidar_cnt;
            return lidar_cnt < FLAGS_num_frames;
        })
       .Go();

    g_replay_ok = (lidar_cnt > 0);
    LOG(INFO) << "Replay done: " << lidar_cnt << " frames captured";
}

// ── Test fixture ──────────────────────────────────────────────────────────────
class SnapshotTest : public ::testing::TestWithParam<int> {
public:
    static void SetUpTestSuite() {
        if (!FLAGS_bag.empty()) ReplayBag();
    }
};

TEST_P(SnapshotTest, FrameMatchesGolden) {
    const int frame_id = GetParam();
    ASSERT_FALSE(FLAGS_golden_dir.empty()) << "Set --golden_dir";
    ASSERT_TRUE(g_replay_ok || !FLAGS_bag.empty()) << "Set --bag for replay";

    lsnap::CheckPoint golden;
    try {
        golden = lsnap::GoldenReader::Read(FLAGS_golden_dir, frame_id);
    } catch (const std::exception& e) {
        GTEST_SKIP() << "No golden data for frame " << frame_id << ": " << e.what();
    }

    const lsnap::CheckPoint* actual = nullptr;
    for (const auto& cp : g_actual) {
        if (cp.frame_id == frame_id) { actual = &cp; break; }
    }
    ASSERT_NE(actual, nullptr) << "frame " << frame_id << " not found in replay";

    // CP-1: IMU timestamp count
    EXPECT_EQ(actual->imu_timestamps.size(), golden.imu_timestamps.size())
        << "IMU count mismatch at frame " << frame_id;

    // CP-2: Undistorted points (< 1e-4 m)
    ASSERT_EQ(actual->undistort_pts.size(), golden.undistort_pts.size())
        << "undistort point count mismatch at frame " << frame_id;
    for (size_t i = 0; i < actual->undistort_pts.size(); ++i) {
        EXPECT_NEAR(actual->undistort_pts[i][0], golden.undistort_pts[i][0], 1e-4f)
            << "undistort pt[" << i << "].x at frame " << frame_id;
        EXPECT_NEAR(actual->undistort_pts[i][1], golden.undistort_pts[i][1], 1e-4f)
            << "undistort pt[" << i << "].y at frame " << frame_id;
        EXPECT_NEAR(actual->undistort_pts[i][2], golden.undistort_pts[i][2], 1e-4f)
            << "undistort pt[" << i << "].z at frame " << frame_id;
    }

    // CP-3: HTH / HTr (< 1e-6)
    EXPECT_LT((actual->HTH - golden.HTH).norm(), 1e-6)
        << "HTH mismatch at frame " << frame_id;
    EXPECT_LT((actual->HTr - golden.HTr).norm(), 1e-6)
        << "HTr mismatch at frame " << frame_id;
    EXPECT_EQ(actual->valid_num, golden.valid_num)
        << "valid_num mismatch at frame " << frame_id;

    // CP-4: ESKF state (< 1e-6)
    EXPECT_LT((actual->pos - golden.pos).norm(), 1e-6)
        << "pos mismatch at frame " << frame_id;
    EXPECT_LT((actual->vel - golden.vel).norm(), 1e-6)
        << "vel mismatch at frame " << frame_id;
    EXPECT_LT((actual->bg  - golden.bg ).norm(), 1e-6)
        << "bg  mismatch at frame " << frame_id;
    EXPECT_LT((actual->rot * golden.rot.inverse()).vec().norm(), 1e-6)
        << "rot mismatch at frame " << frame_id;

    // CP-5: Keyframe
    if (golden.is_keyframe) {
        EXPECT_TRUE(actual->is_keyframe)
            << "missed keyframe at frame " << frame_id;
        EXPECT_LT((actual->kf_pos - golden.kf_pos).norm(), 1e-5)
            << "kf_pos mismatch at frame " << frame_id;
    }
}

INSTANTIATE_TEST_SUITE_P(
    Frames0to99, SnapshotTest,
    ::testing::Range(0, 100));

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    testing::InitGoogleTest(&argc, argv);
    google::ParseCommandLineFlags(&argc, &argv, true);
    return RUN_ALL_TESTS();
}
