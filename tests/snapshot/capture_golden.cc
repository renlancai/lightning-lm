/**
 * capture_golden — Phase 0 snapshot capture tool.
 *
 * Runs the CURRENT (pre-refactor) LaserMapping on the first N lidar frames
 * of a bag and serialises CheckPoints to tests/snapshot/golden/.
 *
 * Build with:  -DLIGHTNING_SNAPSHOT_DEBUG=1
 * Usage:
 *   ./capture_golden \
 *       --config  config/default_nclt.yaml \
 *       --bag     /data/nclt.db3 \
 *       --frames  100 \
 *       --output  tests/snapshot/golden
 */

#ifndef LIGHTNING_SNAPSHOT_DEBUG
#error "capture_golden must be compiled with -DLIGHTNING_SNAPSHOT_DEBUG"
#endif

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "core/lio/laser_mapping.h"
#include "wrapper/bag_io.h"

#include "tests/snapshot/golden_io.h"

DEFINE_string(config,  "./config/default_nclt.yaml", "Config YAML");
DEFINE_string(bag,     "",    "Input bag file (.db3)");
DEFINE_int32 (frames,  100,   "Number of lidar frames to capture");
DEFINE_string(output,  "tests/snapshot/golden", "Output directory");

namespace lsnap = lightning::snapshot;
using lightning::LaserMapping;
using lightning::RosbagIO;
using lightning::IMUPtr;

// Map from LaserMapping::CheckPoint (debug struct) to snapshot::CheckPoint
static lsnap::CheckPoint Convert(const LaserMapping::CheckPoint& src) {
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

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold  = google::INFO;
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (FLAGS_bag.empty()) {
        LOG(ERROR) << "--bag is required";
        return 1;
    }

    LaserMapping lio;
    if (!lio.Init(FLAGS_config)) {
        LOG(ERROR) << "LaserMapping::Init failed";
        return 1;
    }

    lsnap::GoldenWriter writer(FLAGS_output);
    int lidar_frame_count = 0;

    lio.SetDebugCallback([&](const LaserMapping::CheckPoint& cp) {
        writer.Write(Convert(cp));
        LOG(INFO) << "Captured frame " << cp.frame_id
                  << "  is_kf=" << cp.is_keyframe
                  << "  pos=" << cp.pos.transpose();
    });

    RosbagIO bag(FLAGS_bag);

    // Resolve topic names from LaserMapping config (fallback to common names).
    const std::string imu_topic   = "imu_raw";
    const std::string lidar_topic = "points_raw";

    bag.AddImuHandle(imu_topic, [&](IMUPtr imu) {
            if (lidar_frame_count >= FLAGS_frames) return false;
            lio.ProcessIMU(imu);
            return true;
        })
       .AddPointCloud2Handle(lidar_topic, [&](auto msg) {
            if (lidar_frame_count >= FLAGS_frames) return false;
            lio.ProcessPointCloud2(msg);
            lio.Run();
            lidar_frame_count++;
            return lidar_frame_count < FLAGS_frames;
        })
       .Go();

    LOG(INFO) << "Done. Captured " << lidar_frame_count
              << " frames → " << FLAGS_output;
    return 0;
}
