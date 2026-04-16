#pragma once

#include "common/eigen_types.h"
#include "common/nav_state.h"
#include "core/types/platform_types.h"

#include <deque>
#include <cstdint>

namespace lightning::core {

/**
 * GnssRtkHandler: 4-layer RTK false-fixed detection.
 *
 *   Layer 1 – Receiver quality gating    (solution type, satellites, HDOP, age)
 *   Layer 2 – Position jump detection    (compare with LIO predicted position)
 *   Layer 3 – Temporal consistency       (require N consecutive valid frames)
 *   Layer 4 – PGO chi-square rejection   (post-optimization residual check)
 *
 * Usage:
 *   GnssRtkHandler handler(options);
 *   handler.SetDatum(lat_deg, lon_deg, alt_m);           // set ENU datum once
 *
 *   auto result = handler.Process(rtk_frame, lio_state);
 *   if (result.valid && result.inlier)
 *       pgo->ProcessRtk(result);
 *
 *   // After PGO optimization, update chi2 for last accepted observation:
 *   handler.UpdateLastChi2(chi2_value);
 */
class GnssRtkHandler {
public:
    struct Options {
        // Layer 1: receiver quality
        int    min_satellites    = 6;
        double max_hdop          = 3.0;
        double max_age_sec       = 2.0;
        bool   require_fix       = true;
        bool   accept_float      = false;  // if true, scale noise 10×

        // Layer 2: position jump
        double jump_3d_th_m          = 5.0;
        double jump_2d_th_m          = 3.0;
        double max_rtk_lio_gap_sec   = 1.0;

        // Layer 3: temporal consistency
        int min_consecutive_valid    = 3;
        int max_consecutive_invalid  = 5;
        int blackout_recovery_count  = 10;

        // Layer 4: chi-square rejection
        double chi2_reject_th = 7.81;  // 3-DOF, p=0.05

        // coordinate
        bool auto_set_datum = true;

        // RTK observation noise std-dev (ENU, metres)
        Vec3d fix_pos_std   = Vec3d(0.05, 0.05, 0.10);
        Vec3d float_pos_std = Vec3d(0.30, 0.30, 0.60);
        double heading_std_rad = 0.01;
    };

    struct RtkObservation {
        bool   valid   = false;  // passed layers 1-3
        bool   inlier  = true;   // passed layer 4 (PGO chi2)

        Vec3d  pos_enu = Vec3d::Zero();
        Mat3d  pos_cov = Mat3d::Identity();

        bool   heading_valid   = false;
        double heading_rad     = 0.0;
        double heading_cov     = 0.0;

        GnssRtkFrame::SolutionType solution_type = GnssRtkFrame::SolutionType::INVALID;
        double chi2 = 0.0;
        double timestamp_sec = 0.0;
    };

    explicit GnssRtkHandler(Options opts);
    GnssRtkHandler();

    // Set the ENU datum point (first call or explicit set)
    void SetDatum(double lat_deg, double lon_deg, double alt_m);
    bool DatumSet() const { return datum_set_; }

    // Main processing — returns observation (valid=false if rejected)
    RtkObservation Process(const GnssRtkFrame& frame, const NavState& lio_state);

    // Called by PGO after optimization to feed back chi2 for layer 4
    void UpdateLastChi2(double chi2);

    // Statistics
    int TotalReceived()  const { return stats_received_; }
    int TotalAccepted()  const { return stats_accepted_; }
    int TotalRejected()  const { return stats_rejected_; }

private:
    // Layer 1
    bool CheckReceiverQuality(const GnssRtkFrame& frame);

    // Layer 2
    bool CheckPositionJump(const Vec3d& pos_enu, const NavState& lio_state,
                           double timestamp_sec);

    // Layer 3
    bool CheckTemporalConsistency(bool layer12_ok);

    // Convert frame LLH → ENU
    Vec3d LlhToEnu(double lat_deg, double lon_deg, double alt_m) const;

    Options options_;

    // ENU datum
    bool   datum_set_ = false;
    double datum_lat_rad_ = 0.0;
    double datum_lon_rad_ = 0.0;
    double datum_alt_m_   = 0.0;
    Vec3d  datum_ecef_    = Vec3d::Zero();
    Mat3d  R_ecef_enu_    = Mat3d::Identity();

    // Layer 3 state
    int  consecutive_valid_   = 0;
    int  consecutive_invalid_ = 0;
    bool in_blackout_         = false;
    int  blackout_counter_    = 0;

    // Last accepted observation (for chi2 feedback)
    RtkObservation last_obs_;

    // Statistics
    int stats_received_ = 0;
    int stats_accepted_ = 0;
    int stats_rejected_ = 0;
};

}  // namespace lightning::core
