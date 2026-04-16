#include "core/gnss/gnss_rtk_handler.h"
#include "adapter/common/coordinate_utils.h"

#include <cmath>
#include <glog/logging.h>

namespace lightning::core {

GnssRtkHandler::GnssRtkHandler() : GnssRtkHandler(Options{}) {}
GnssRtkHandler::GnssRtkHandler(Options opts) : options_(std::move(opts)) {}

void GnssRtkHandler::SetDatum(double lat_deg, double lon_deg, double alt_m) {
    datum_lat_rad_ = lat_deg * M_PI / 180.0;
    datum_lon_rad_ = lon_deg * M_PI / 180.0;
    datum_alt_m_   = alt_m;
    datum_ecef_    = adapter::LlhToEcef(datum_lat_rad_, datum_lon_rad_, datum_alt_m_);
    R_ecef_enu_    = adapter::BuildRotEcefEnu(datum_lat_rad_, datum_lon_rad_);
    datum_set_     = true;
    LOG(INFO) << "GnssRtkHandler: datum set to (" << lat_deg << ", " << lon_deg << ", " << alt_m << ")";
}

Vec3d GnssRtkHandler::LlhToEnu(double lat_deg, double lon_deg, double alt_m) const {
    const double lat_rad = lat_deg * M_PI / 180.0;
    const double lon_rad = lon_deg * M_PI / 180.0;
    Vec3d ecef = adapter::LlhToEcef(lat_rad, lon_rad, alt_m);
    return adapter::EcefToEnu(ecef, datum_ecef_, R_ecef_enu_);
}

bool GnssRtkHandler::CheckReceiverQuality(const GnssRtkFrame& frame) {
    using ST = GnssRtkFrame::SolutionType;

    // Solution type check
    bool type_ok = false;
    if (options_.require_fix) {
        type_ok = (frame.solution_type == ST::FIX);
    } else {
        type_ok = (frame.solution_type == ST::FIX) ||
                  (options_.accept_float && frame.solution_type == ST::FLOAT);
    }
    if (!type_ok) {
        VLOG(2) << "RTK L1 reject: solution_type " << static_cast<int>(frame.solution_type);
        return false;
    }

    if (frame.num_satellites < options_.min_satellites) {
        VLOG(2) << "RTK L1 reject: num_satellites " << frame.num_satellites;
        return false;
    }
    if (frame.hdop > options_.max_hdop) {
        VLOG(2) << "RTK L1 reject: hdop " << frame.hdop;
        return false;
    }
    if (frame.age_of_corr_sec > options_.max_age_sec) {
        VLOG(2) << "RTK L1 reject: age_of_corr " << frame.age_of_corr_sec;
        return false;
    }
    return true;
}

bool GnssRtkHandler::CheckPositionJump(const Vec3d& pos_enu, const NavState& lio_state,
                                        double timestamp_sec) {
    if (!lio_state.pose_is_ok_) return true;  // can't predict, pass through

    const double dt = timestamp_sec - lio_state.timestamp_;
    if (std::fabs(dt) > options_.max_rtk_lio_gap_sec) {
        VLOG(2) << "RTK L2 skip: LIO state too old, dt=" << dt;
        return true;  // can't judge, allow
    }

    // Predict LIO position at RTK timestamp
    Vec3d lio_pos_pred = lio_state.pos_ + lio_state.vel_ * dt;

    // Compare 3D and 2D distances
    double dist3d = (pos_enu - lio_pos_pred).norm();
    double dist2d = (pos_enu.head<2>() - lio_pos_pred.head<2>()).norm();

    if (dist3d > options_.jump_3d_th_m) {
        LOG(WARNING) << "RTK L2 reject: 3D jump " << dist3d << " m > " << options_.jump_3d_th_m;
        return false;
    }
    if (dist2d > options_.jump_2d_th_m) {
        LOG(WARNING) << "RTK L2 reject: 2D jump " << dist2d << " m > " << options_.jump_2d_th_m;
        return false;
    }
    return true;
}

bool GnssRtkHandler::CheckTemporalConsistency(bool layer12_ok) {
    if (in_blackout_) {
        blackout_counter_++;
        if (blackout_counter_ >= options_.blackout_recovery_count) {
            in_blackout_ = false;
            consecutive_valid_   = 0;
            consecutive_invalid_ = 0;
            blackout_counter_    = 0;
            LOG(INFO) << "RTK L3: blackout recovery complete";
        }
        return false;
    }

    if (layer12_ok) {
        consecutive_valid_++;
        consecutive_invalid_ = 0;
    } else {
        consecutive_invalid_++;
        consecutive_valid_ = 0;
        if (consecutive_invalid_ >= options_.max_consecutive_invalid) {
            in_blackout_     = true;
            blackout_counter_ = 0;
            LOG(WARNING) << "RTK L3: entering blackout after "
                         << consecutive_invalid_ << " consecutive rejections";
        }
        return false;
    }

    return consecutive_valid_ >= options_.min_consecutive_valid;
}

GnssRtkHandler::RtkObservation GnssRtkHandler::Process(const GnssRtkFrame& frame,
                                                         const NavState& lio_state) {
    stats_received_++;
    RtkObservation obs;
    obs.timestamp_sec  = frame.timestamp_sec;
    obs.solution_type  = frame.solution_type;

    // Auto-set datum from first valid FIX
    if (options_.auto_set_datum && !datum_set_ &&
        frame.solution_type == GnssRtkFrame::SolutionType::FIX) {
        SetDatum(frame.lat_deg, frame.lon_deg, frame.alt_m);
    }

    if (!datum_set_) {
        stats_rejected_++;
        VLOG(2) << "RTK: datum not set, skipping";
        return obs;
    }

    // Convert to ENU
    Vec3d pos_enu = LlhToEnu(frame.lat_deg, frame.lon_deg, frame.alt_m);
    obs.pos_enu = pos_enu;

    // Layer 1
    bool l1 = CheckReceiverQuality(frame);
    // Layer 2 (only if L1 passed)
    bool l2 = l1 && CheckPositionJump(pos_enu, lio_state, frame.timestamp_sec);
    // Layer 3
    bool l3 = CheckTemporalConsistency(l1 && l2);

    if (!l3) {
        stats_rejected_++;
        return obs;
    }

    // Build covariance
    bool is_fix = (frame.solution_type == GnssRtkFrame::SolutionType::FIX);
    Vec3d std_enu = (frame.pos_std_enu.norm() > 1e-6)
                        ? frame.pos_std_enu
                        : (is_fix ? options_.fix_pos_std : options_.float_pos_std);
    if (!is_fix) std_enu *= 10.0;  // inflate float noise

    obs.pos_cov = std_enu.cwiseProduct(std_enu).asDiagonal();

    // Heading
    obs.heading_valid    = frame.heading_valid;
    obs.heading_rad      = frame.heading_rad;
    obs.heading_cov      = frame.heading_std_rad > 0
                               ? frame.heading_std_rad * frame.heading_std_rad
                               : options_.heading_std_rad * options_.heading_std_rad;

    obs.valid   = true;
    obs.inlier  = true;  // default; set to false by UpdateLastChi2 if needed
    last_obs_   = obs;

    stats_accepted_++;
    return obs;
}

void GnssRtkHandler::UpdateLastChi2(double chi2) {
    last_obs_.chi2   = chi2;
    last_obs_.inlier = (chi2 <= options_.chi2_reject_th);
    if (!last_obs_.inlier) {
        LOG(WARNING) << "RTK L4: chi2 " << chi2 << " > " << options_.chi2_reject_th
                     << ", marking as outlier";
        // Penalise temporal consistency state
        consecutive_valid_ = std::max(0, consecutive_valid_ - 1);
    }
}

}  // namespace lightning::core
