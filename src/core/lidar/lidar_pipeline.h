// Phase 2: LidarPipeline — encapsulates the three ROS-free LiDAR processing
// steps previously scattered across ImuProcess and LaserMapping.
//
//   UndistortFrame    ← ImuProcess::UndistortPcl
//   BuildObservation  ← LaserMapping::ObsModel
//   UpdateMap         ← LaserMapping::MapIncremental  (via LocalMap)
//
// None of these methods depend on rclcpp / sensor_msgs / livox_ros_driver2.

#pragma once

#include <execution>
#include <vector>

#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/measure_group.h"
#include "common/nav_state.h"
#include "common/point_def.h"
#include "core/lidar/local_map.h"
#include "core/lio/eskf.hpp"
#include "core/lio/imu_filter.h"
#include "core/lio/pose6d.h"
#include "utils/timer.h"

namespace lightning {

class LidarPipeline {
   public:
    struct Options {
        bool  enable_icp_part_      = true;
        double plane_icp_weight_    = 1.0;
        double icp_weight_          = 100.0;
        double filter_size_map_min_ = 0.0;
        float  esti_plane_threshold_ = 0.1f;  // Phase 3: migrated from fasterlio::ESTI_PLANE_THRESHOLD
    };

    LidarPipeline();
    explicit LidarPipeline(Options opts);

    // ── Setup ─────────────────────────────────────────────────────────────
    void SetLocalMap(std::shared_ptr<LocalMap> map) { local_map_ = std::move(map); }
    void SetExtrinsic(const Vec3d& t, const Mat3d& R);
    void SetUseIMUFilter(bool b) { use_imu_filter_ = b; }
    void SetAccScaleFactor(double f) { acc_scale_factor_ = f; }
    void SetOptions(const Options& opts) { options_ = opts; }

    // Phase 3: runtime lidar time interval (estimated online by LaserMapping)
    void SetLidarTimeInterval(float t) { lidar_time_interval_ = t; }
    float GetLidarTimeInterval() const { return lidar_time_interval_; }

    // Called when IMU init completes — seeds the carry-over state with the
    // last IMU measurement so the first UndistortFrame call is well-defined.
    void ResetUndistortState(const IMUPtr& last_imu);

    // ── Pipeline steps ─────────────────────────────────────────────────────

    // Step 1 (migrated from ImuProcess::UndistortPcl)
    // Q: process-noise covariance (caller owns it — e.g. ImuProcess::Q_)
    void UndistortFrame(const MeasureGroup& meas, ESKF& kf_state, CloudPtr& scan_out,
                        const Eigen::Matrix<double, 12, 12>& Q);

    // Step 2 (migrated from LaserMapping::ObsModel)
    // Builds the IEKF observation (HTH, HTr) from the downsampled scan.
    // scan_down_body / scan_down_world / nearest_points must be pre-sized.
    void BuildObservation(const CloudPtr& scan_down_body, CloudPtr& scan_down_world,
                          std::vector<PointVector>& nearest_points,
                          NavState& s, ESKF::CustomObservationModel& obs);

    // Step 3 (migrated from LaserMapping::MapIncremental)
    // Decides which world-frame points to add and writes them to LocalMap.
    void UpdateMap(const CloudPtr& scan_down_body, CloudPtr& scan_down_world,
                   const std::vector<PointVector>& nearest_points,
                   const NavState& state, bool flg_EKF_inited);

    // ── Statistics (read after BuildObservation) ───────────────────────────
    int GetEffectFeatSurf()  const { return effect_feat_surf_; }
    int GetEffectFeatICP()   const { return effect_feat_icp_;  }

   private:
    Options options_;

    // ── Extrinsics (lidar ↔ IMU, set once from YAML) ─────────────────────
    Mat3d offset_R_lidar_fixed_{Mat3d::Identity()};
    Vec3d offset_t_lidar_fixed_{Vec3d::Zero()};

    // ── LocalMap ─────────────────────────────────────────────────────────
    std::shared_ptr<LocalMap> local_map_;

    // ── Undistortion state (moved from ImuProcess) ───────────────────────
    IMUPtr            last_imu_;
    std::vector<Pose6D> imu_pose_;
    double            last_lidar_end_time_ = 0.0;
    Vec3d             angvel_last_{Vec3d::Zero()};
    Vec3d             acc_s_last_{Vec3d::Zero()};
    double            acc_scale_factor_    = 1.0;
    bool              use_imu_filter_      = true;
    IMUFilter         filter_;
    float             lidar_time_interval_ = 0.1f;  // Phase 3: runtime value, set by LaserMapping

    // ── Observation buffers (moved from LaserMapping) ────────────────────
    std::vector<Vec4f> corr_pts_;
    std::vector<Vec4f> corr_norm_;
    std::vector<float> residuals_;
    std::vector<char>  point_selected_surf_;
    std::vector<char>  point_selected_icp_;
    std::vector<Vec4f> plane_coef_;
    int effect_feat_surf_ = 0;
    int effect_feat_icp_  = 0;
};

}  // namespace lightning
