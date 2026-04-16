// Phase 5: SlamEngine — platform-agnostic SLAM orchestrator.
//
// This header has ZERO ROS includes.  Heavy types (LaserMapping, LoopClosing,
// PangolinWindow, G2P5) are forward-declared so callers that only #include
// this header do not transitively pull in any ROS dependency.
//
// Usage:
//   core::SlamEngine engine(opts);
//   engine.Init(yaml_path);
//   engine.StartMapping("my_map");
//   engine.FeedImu(imu_frame);      // core::ImuFrame
//   engine.FeedLidar(pc_frame);     // core::PointCloudFrame
//   engine.SaveMap();

#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "common/keyframe.h"
#include "core/types/platform_types.h"

namespace lightning {

// Forward declarations — keep this header ROS-free
class LaserMapping;
class LoopClosing;
namespace ui { class PangolinWindow; }
namespace g2p5 { class G2P5; }

namespace core {

class SlamEngine {
   public:
    struct Options {
        bool online_mode_      = true;   ///< true → async (online); false → sync (offline)
        bool with_loop_closing = true;
        bool with_gridmap      = false;
        bool with_ui           = true;
        bool step_on_kf        = true;
    };

    SlamEngine();
    explicit SlamEngine(Options options);
    ~SlamEngine();

    /// Load YAML config and initialise all subsystems.
    bool Init(const std::string& yaml_path);

    /// Begin mapping with the given map name.
    void StartMapping(const std::string& map_name = "new_map");

    /// Save current map.  Empty path → ./data/<map_name>/.
    void SaveMap(const std::string& path = "");

    // ── Platform-agnostic feed methods ──────────────────────────────────────
    void FeedImu(const ImuFrame& imu);
    void FeedLidar(const PointCloudFrame& frame);

    // ── Accessors ────────────────────────────────────────────────────────────
    std::vector<Keyframe::Ptr> GetAllKeyframes() const;

   private:
    void HandleNewKeyframe(Keyframe::Ptr kf);

    Options              options_;
    std::atomic_bool     running_{false};
    std::string          map_name_;

    std::shared_ptr<LaserMapping>       lio_;
    std::shared_ptr<LoopClosing>        lc_;
    std::shared_ptr<ui::PangolinWindow> ui_;
    std::shared_ptr<g2p5::G2P5>         g2p5_;

    Keyframe::Ptr cur_kf_;
};

}  // namespace core
}  // namespace lightning
