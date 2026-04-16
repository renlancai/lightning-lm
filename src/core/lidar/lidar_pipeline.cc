#include "core/lidar/lidar_pipeline.h"
#include "core/lightning_math.hpp"
#include "common/options.h"
#include "utils/timer.h"

namespace lightning {

LidarPipeline::LidarPipeline() : LidarPipeline(Options{}) {}

LidarPipeline::LidarPipeline(Options opts) : options_(opts) {
    last_imu_.reset(new IMU());
}

void LidarPipeline::SetExtrinsic(const Vec3d& t, const Mat3d& R) {
    offset_t_lidar_fixed_ = t;
    offset_R_lidar_fixed_ = R;
}

void LidarPipeline::ResetUndistortState(const IMUPtr& last_imu) {
    last_imu_             = last_imu;
    last_lidar_end_time_  = 0.0;
    angvel_last_.setZero();
    acc_s_last_.setZero();
    imu_pose_.clear();
}

// ─────────────────────────────────────────────────────────────────────────────
// Step 1 – Undistortion  (migrated from ImuProcess::UndistortPcl)
// ─────────────────────────────────────────────────────────────────────────────
void LidarPipeline::UndistortFrame(const MeasureGroup& meas, ESKF& kf_state,
                                   CloudPtr& pcl_out,
                                   const Eigen::Matrix<double, 12, 12>& Q) {
    auto v_imu = meas.imu_;
    v_imu.push_front(last_imu_);
    const double& imu_end_time = v_imu.back()->timestamp;

    const double& pcl_beg_time = meas.lidar_begin_time_;
    const double& pcl_end_time = meas.lidar_end_time_;

    auto imu_state = kf_state.GetX();
    imu_pose_.clear();
    imu_pose_.emplace_back(0.0, acc_s_last_, angvel_last_,
                           imu_state.vel_, imu_state.pos_,
                           imu_state.rot_.matrix());

    Vec3d angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
    Mat3d R_imu;
    double dt = 0;
    Vec3d acc  = Vec3d::Zero();
    Vec3d gyro = Vec3d::Zero();

    if (use_imu_filter_) {
        for (auto& imu : v_imu) {
            auto imu_f = filter_.Filter(*imu);
            *imu = imu_f;
        }
    }

    for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++) {
        auto&& head = *(it_imu);
        auto&& tail = *(it_imu + 1);

        if (tail->timestamp < last_lidar_end_time_) continue;

        angvel_avr = .5 * (head->angular_velocity + tail->angular_velocity);
        acc_avr    = .5 * (head->linear_acceleration + tail->linear_acceleration);
        acc_avr    = acc_avr * acc_scale_factor_;

        if (head->timestamp < last_lidar_end_time_) {
            dt = tail->timestamp - last_lidar_end_time_;
        } else {
            dt = tail->timestamp - head->timestamp;
        }

        acc  = acc_avr;
        gyro = angvel_avr;

        if (dt > 0.1) {
            LOG(ERROR) << "get abnormal dt: " << dt;
            kf_state.SetTime((*it_imu)->timestamp);
            break;
        }

        kf_state.Predict(dt, Q, gyro, acc);

        imu_state    = kf_state.GetX();
        angvel_last_ = angvel_avr - imu_state.bg_;
        acc_s_last_  = imu_state.rot_ * acc_avr;
        for (int i = 0; i < 3; i++) acc_s_last_[i] += imu_state.grav_[i];

        double offs_t = tail->timestamp - pcl_beg_time;
        imu_pose_.emplace_back(
            Pose6D(offs_t, acc_s_last_, angvel_last_,
                   imu_state.vel_, imu_state.pos_, imu_state.rot_.matrix()));
    }

    double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
    dt = note * (pcl_end_time - imu_end_time);
    kf_state.Predict(dt, Q, gyro, acc);

    imu_state            = kf_state.GetX();
    last_imu_            = meas.imu_.back();
    last_lidar_end_time_ = pcl_end_time;

    pcl_out = meas.scan_;
    std::sort(pcl_out->points.begin(), pcl_out->points.end(),
              [](const PointType& p1, const PointType& p2) { return p1.time < p2.time; });

    if (pcl_out->empty()) return;

    auto it_pcl = pcl_out->points.end() - 1;
    for (auto it_kp = imu_pose_.end() - 1; it_kp != imu_pose_.begin(); it_kp--) {
        auto head       = it_kp - 1;
        auto tail       = it_kp;
        R_imu           = head->rot;
        vel_imu         = head->vel;
        pos_imu         = head->pos;
        acc_imu         = tail->acc;
        angvel_avr      = tail->gyr;

        for (; it_pcl->time / double(1000) > head->offset_time &&
               it_pcl != pcl_out->points.begin();
             it_pcl--) {
            dt = it_pcl->time / double(1000) - head->offset_time;
            if (dt < 0 || dt > lidar_time_interval_) continue;

            Mat3d R_i(R_imu * math::exp(angvel_avr, dt).matrix());
            Vec3d P_i(it_pcl->x, it_pcl->y, it_pcl->z);
            Vec3d T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos_);
            Vec3d p_compensate =
                offset_R_lidar_fixed_.transpose() *
                (imu_state.rot_.inverse() *
                     (R_i * (offset_R_lidar_fixed_ * P_i + offset_t_lidar_fixed_) + T_ei) -
                 offset_t_lidar_fixed_);

            it_pcl->x = p_compensate(0);
            it_pcl->y = p_compensate(1);
            it_pcl->z = p_compensate(2);
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Step 2 – Observation  (migrated from LaserMapping::ObsModel)
// ─────────────────────────────────────────────────────────────────────────────
void LidarPipeline::BuildObservation(const CloudPtr& scan_down_body,
                                     CloudPtr& scan_down_world,
                                     std::vector<PointVector>& nearest_points,
                                     NavState& s,
                                     ESKF::CustomObservationModel& obs) {
    int cnt_pts = scan_down_body->size();

    std::vector<size_t> index(cnt_pts);
    for (size_t i = 0; i < index.size(); ++i) index[i] = i;

    Timer::Evaluate(
        [&, this]() {
            Mat3f R_wl = (s.rot_.matrix() * offset_R_lidar_fixed_).cast<float>();
            Vec3f t_wl = (s.rot_ * offset_t_lidar_fixed_ + s.pos_).cast<float>();

            // resize working buffers if needed
            residuals_.resize(cnt_pts, 0);
            point_selected_surf_.resize(cnt_pts, 1);
            point_selected_icp_.resize(cnt_pts, 1);
            plane_coef_.resize(cnt_pts, Vec4f::Zero());

            std::for_each(std::execution::par_unseq, index.begin(), index.end(),
                          [&](const size_t& i) {
                              PointType& point_body  = scan_down_body->points[i];
                              PointType& point_world = scan_down_world->points[i];

                              Vec3f p_body        = point_body.getVector3fMap();
                              point_world.getVector3fMap() = R_wl * p_body + t_wl;
                              point_world.intensity        = point_body.intensity;

                              auto& points_near = nearest_points[i];
                              points_near.clear();
                              local_map_->GetClosestPoint(point_world, points_near,
                                                         fasterlio::NUM_MATCH_POINTS);
                              point_selected_surf_[i] =
                                  points_near.size() >= fasterlio::MIN_NUM_MATCH_POINTS;
                              point_selected_icp_[i] = point_selected_surf_[i];

                              if (point_selected_surf_[i]) {
                                  point_selected_surf_[i] = math::esti_plane(
                                      plane_coef_[i], points_near,
                                      options_.esti_plane_threshold_);
                              }

                              if (point_selected_surf_[i]) {
                                  auto temp = point_world.getVector4fMap();
                                  temp[3]   = 1.0;
                                  float pd2 = plane_coef_[i].dot(temp);
                                  if (p_body.norm() > 81 * pd2 * pd2) {
                                      point_selected_surf_[i] = true;
                                      residuals_[i]           = pd2;
                                  } else {
                                      point_selected_surf_[i] = false;
                                  }
                              }
                          });
        },
        "    ObsModel (Lidar Match)");

    effect_feat_surf_ = 0;
    effect_feat_icp_  = 0;

    corr_pts_.resize(cnt_pts);
    corr_norm_.resize(cnt_pts);
    for (int i = 0; i < cnt_pts; i++) {
        if (point_selected_surf_[i]) {
            corr_norm_[effect_feat_surf_]    = plane_coef_[i];
            corr_pts_[effect_feat_surf_]     = scan_down_body->points[i].getVector4fMap();
            corr_pts_[effect_feat_surf_][3]  = residuals_[i];
            effect_feat_surf_++;
        }
        if (point_selected_icp_[i]) effect_feat_icp_++;
    }
    corr_pts_.resize(effect_feat_surf_);
    corr_norm_.resize(effect_feat_surf_);

    if (effect_feat_surf_ < 20) {
        obs.valid_ = false;
        LOG(WARNING) << "No enough effective surface points: " << effect_feat_surf_
                     << ", icp: " << effect_feat_icp_ << ", required: 20";
        return;
    }

    index.resize(effect_feat_surf_);
    const Mat3f off_R = offset_R_lidar_fixed_.cast<float>();
    const Vec3f off_t = offset_t_lidar_fixed_.cast<float>();
    const Mat3f Rt    = s.rot_.matrix().transpose().cast<float>();

    obs.HTH_.setZero();
    obs.HTr_.setZero();

    std::vector<Mat6d> JTJ(effect_feat_surf_);
    std::vector<Vec6d> JTr(effect_feat_surf_);
    std::vector<double> res_sq(index.size());

    std::for_each(std::execution::par_unseq, index.begin(), index.end(),
                  [&](const size_t& i) {
                      Vec3f point_this_be = corr_pts_[i].head<3>();
                      Vec3f point_this    = off_R * point_this_be + off_t;
                      Mat3f point_crossmat = math::SKEW_SYM_MATRIX(point_this);
                      Vec3f norm_vec      = corr_norm_[i].head<3>();
                      Vec3f C(Rt * norm_vec);
                      Vec3f A(point_crossmat * C);

                      Eigen::Matrix<double, 1, ESKF::pose_obs_dim_> J;
                      J.setZero();
                      J << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2];

                      float  res = -corr_pts_[i][3];
                      double w   = 1.0;
                      JTJ[i]     = (J.transpose() * J).eval() * w;
                      JTr[i]     = J.transpose() * res * w;
                      res_sq[i]  = res * res;
                  });

    for (int i = 0; i < static_cast<int>(index.size()); ++i) {
        obs.HTH_ += JTJ[i] * options_.plane_icp_weight_;
        obs.HTr_ += JTr[i] * options_.plane_icp_weight_;
    }

    if (!res_sq.empty()) {
        std::sort(res_sq.begin(), res_sq.end());
        obs.lidar_residual_mean_ = res_sq[res_sq.size() / 2];
        obs.lidar_residual_max_  = res_sq[res_sq.size() - 1];
    }

    if (!options_.enable_icp_part_) return;

    JTJ.resize(cnt_pts);
    JTr.resize(cnt_pts);
    std::vector<size_t> idx2(cnt_pts);
    for (size_t i = 0; i < idx2.size(); ++i) idx2[i] = i;

    std::for_each(std::execution::par_unseq, idx2.begin(), idx2.end(),
                  [&](const size_t& i) {
                      if (!point_selected_icp_[i]) return;

                      Vec3d q  = scan_down_body->points[i].getVector3fMap().cast<double>();
                      Vec3d qs = scan_down_world->points[i].getVector3fMap().cast<double>();

                      Eigen::Matrix<double, 3, ESKF::pose_obs_dim_> J;
                      J.setZero();
                      J.block<3, 3>(0, 0) = Mat3d::Identity();
                      J.block<3, 3>(0, 3) =
                          -(s.rot_.matrix() * offset_R_lidar_fixed_) * SO3::hat(q);

                      Vec3d e = qs - nearest_points[i][0].getVector3fMap().cast<double>();
                      if (e.norm() > 0.5) {
                          point_selected_icp_[i] = false;
                          return;
                      }
                      JTJ[i] = J.transpose() * J;
                      JTr[i] = -J.transpose() * e;
                  });

    for (int i = 0; i < cnt_pts; ++i) {
        if (!point_selected_icp_[i]) continue;
        obs.HTH_ += JTJ[i] * options_.icp_weight_;
        obs.HTr_ += JTr[i] * options_.icp_weight_;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Step 3 – Map update  (migrated from LaserMapping::MapIncremental)
// ─────────────────────────────────────────────────────────────────────────────
void LidarPipeline::UpdateMap(const CloudPtr& scan_down_body, CloudPtr& scan_down_world,
                              const std::vector<PointVector>& nearest_points,
                              const NavState& state, bool flg_EKF_inited) {
    const double fsm = options_.filter_size_map_min_;

    // Helper: transform a body-frame point to world frame using current state
    auto body_to_world = [&](const PointType& pi, PointType& po) {
        Vec3d p_global(state.rot_ * (offset_R_lidar_fixed_ * pi.getVector3fMap().cast<double>() +
                                     offset_t_lidar_fixed_) +
                       state.pos_);
        po.x         = p_global(0);
        po.y         = p_global(1);
        po.z         = p_global(2);
        po.intensity = pi.intensity;
    };

    size_t cur_pts = scan_down_body->size();

    enum class PointAction : uint8_t { None, Add, NoNeedDownsample };
    std::vector<PointAction> actions(cur_pts, PointAction::None);

    std::vector<size_t> index(cur_pts);
    for (size_t i = 0; i < cur_pts; ++i) index[i] = i;

    std::for_each(std::execution::par_unseq, index.begin(), index.end(),
                  [&](const size_t& i) {
                      body_to_world(scan_down_body->points[i], scan_down_world->points[i]);

                      const PointType& point_world = scan_down_world->points[i];
                      if (!nearest_points[i].empty() && flg_EKF_inited) {
                          const PointVector& points_near = nearest_points[i];
                          Eigen::Vector3f center =
                              ((point_world.getVector3fMap() / fsm).array().floor() + 0.5f) *
                              fsm;
                          Eigen::Vector3f dis_2_center =
                              points_near[0].getVector3fMap() - center;

                          if (fabs(dis_2_center.x()) > 0.5 * fsm &&
                              fabs(dis_2_center.y()) > 0.5 * fsm &&
                              fabs(dis_2_center.z()) > 0.5 * fsm) {
                              actions[i] = PointAction::NoNeedDownsample;
                              return;
                          }

                          bool need_add = true;
                          float dist = math::calc_dist(point_world.getVector3fMap(), center);
                          if (points_near.size() >= fasterlio::NUM_MATCH_POINTS) {
                              for (int ri = 0; ri < fasterlio::NUM_MATCH_POINTS; ri++) {
                                  if (math::calc_dist(points_near[ri].getVector3fMap(),
                                                      center) < dist + 1e-6f) {
                                      need_add = false;
                                      break;
                                  }
                              }
                          }
                          if (need_add) actions[i] = PointAction::Add;
                      } else {
                          actions[i] = PointAction::Add;
                      }
                  });

    PointVector points_to_add, point_no_need_downsample;
    points_to_add.reserve(cur_pts);
    point_no_need_downsample.reserve(cur_pts);
    for (size_t i = 0; i < cur_pts; ++i) {
        if      (actions[i] == PointAction::Add)              points_to_add.emplace_back(scan_down_world->points[i]);
        else if (actions[i] == PointAction::NoNeedDownsample) point_no_need_downsample.emplace_back(scan_down_world->points[i]);
    }

    Timer::Evaluate(
        [&, this]() {
            local_map_->AddPoints(points_to_add);
            local_map_->AddPoints(point_no_need_downsample);
        },
        "    IVox Add Points");
}

}  // namespace lightning
