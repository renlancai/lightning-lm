// Phase 5: LocEngine — platform-agnostic localization orchestrator.
//
// This header has ZERO ROS includes.
// It adds FeedRtk() on top of FeedImu / FeedLidar, integrating GnssRtkHandler
// (Phase 4) with the PGO backend.

#pragma once

#include <atomic>
#include <memory>
#include <string>

#include "common/eigen_types.h"
#include "core/gnss/gnss_rtk_handler.h"  // ROS-free
#include "core/types/platform_types.h"

namespace lightning {
namespace loc { class Localization; }  // forward-declare (localization.h has ROS types)

namespace core {

class LocEngine {
   public:
    struct Options {
        bool pub_tf      = true;   ///< publish TF transform (online only)
        bool enable_rtk  = false;  ///< enable RTK / GNSS fusion
    };

    LocEngine();
    explicit LocEngine(Options options);
    ~LocEngine();

    /// Load YAML config and initialise all subsystems.
    bool Init(const std::string& yaml_path);

    /// Set initial pose for localisation (call after Init).
    void SetInitPose(const SE3& pose);

    // ── Platform-agnostic feed methods ──────────────────────────────────────
    void FeedImu(const ImuFrame& imu);
    void FeedLidar(const PointCloudFrame& frame);

    /// Phase 4: feed an RTK/GNSS observation through the multi-layer filter
    /// and, if accepted, inject into PGO.
    void FeedRtk(const GnssRtkFrame& rtk);

    /// Finish localization (flush queues, save temp map).
    void Finish();

   private:
    Options options_;
    std::atomic_bool loc_started_{false};

    std::shared_ptr<loc::Localization> loc_;
    std::shared_ptr<GnssRtkHandler>    rtk_handler_;
};

}  // namespace core
}  // namespace lightning
