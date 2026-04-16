// Phase 5: LocEngine implementation.

#include "core/engine/loc_engine.h"

#include "common/imu.h"
#include "common/nav_state.h"
#include "common/point_def.h"
#include "core/localization/localization.h"
#include "io/yaml_io.h"

namespace lightning::core {

LocEngine::LocEngine() : LocEngine(Options{}) {}
LocEngine::LocEngine(Options options) : options_(options) {}

LocEngine::~LocEngine() {}

bool LocEngine::Init(const std::string& yaml_path) {
    YAML_IO yaml(yaml_path);
    std::string map_path = yaml.GetValue<std::string>("system", "map_path");

    loc::Localization::Options loc_opts;
    loc_opts.online_mode_ = false;  // offline default; online apps set their own subscription loop
    loc_ = std::make_shared<loc::Localization>(loc_opts);

    if (!loc_->Init(yaml_path, map_path)) {
        LOG(ERROR) << "LocEngine: failed to init Localization";
        return false;
    }

    if (options_.enable_rtk) {
        rtk_handler_ = std::make_shared<GnssRtkHandler>();
        LOG(INFO) << "LocEngine: RTK handler enabled";
    }

    return true;
}

void LocEngine::SetInitPose(const SE3& pose) {
    if (loc_) {
        loc_->SetExternalPose(pose.unit_quaternion(), pose.translation());
        loc_started_ = true;
    }
}

void LocEngine::FeedImu(const ImuFrame& imu) {
    if (!loc_started_) return;

    auto imu_ptr                 = std::make_shared<IMU>();
    imu_ptr->timestamp           = imu.timestamp_sec;
    imu_ptr->linear_acceleration = imu.acc;
    imu_ptr->angular_velocity    = imu.gyro;

    loc_->ProcessIMUMsg(imu_ptr);
}

void LocEngine::FeedLidar(const PointCloudFrame& frame) {
    if (!loc_started_) return;

    // Convert platform-agnostic PointCloudFrame → PCL CloudPtr.
    // PointType = PointXYZIT; '.time' holds per-point time offset (sec).
    // cloud->header.stamp is nanoseconds (math::ToSec = t * 1e-9).
    CloudPtr cloud(new PointCloudType());
    cloud->header.stamp = static_cast<uint64_t>(frame.timestamp_sec * 1e9);
    cloud->reserve(frame.points.size());

    for (const auto& p : frame.points) {
        PointType pt;
        pt.x         = p.x;
        pt.y         = p.y;
        pt.z         = p.z;
        pt.intensity = p.intensity;
        pt.time      = p.time_offset_sec;
        cloud->push_back(pt);
    }

    loc_->ProcessCloud(cloud);
}

void LocEngine::FeedRtk(const GnssRtkFrame& rtk) {
    if (!loc_started_ || !rtk_handler_) return;

    // Pass an empty NavState; GnssRtkHandler layer 2 skips the jump check when
    // pose_is_ok_ == false, which is safe for the first few frames.
    NavState dummy_state;
    auto obs = rtk_handler_->Process(rtk, dummy_state);

    if (obs.valid) {
        loc_->ProcessRtk(obs);
    }
}

void LocEngine::Finish() {
    if (loc_) { loc_->Finish(); }
}

}  // namespace lightning::core
