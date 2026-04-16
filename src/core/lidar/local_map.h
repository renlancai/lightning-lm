// Phase 1: LocalMap — thread-safe wrapper around IVox.
//
// Rules:
//  • GetClosestPoint / NumValidGrids → shared (read) lock
//  • AddPoints                       → unique (write) lock
//
// All other LaserMapping logic is unchanged.

#pragma once

#include <shared_mutex>

#include "common/point_def.h"
#include "core/ivox3d/ivox3d.h"

namespace lightning {

class LocalMap {
   public:
    using IVoxType = IVox<3, IVoxNodeType::DEFAULT, PointType>;
    using Options  = IVoxType::Options;   // reuse IVox options directly

    explicit LocalMap(const Options& opts);

    // ── write interface (unique lock) ─────────────────────────────────────
    void AddPoints(const PointVector& pts);

    // ── read interface (shared lock) ──────────────────────────────────────
    bool GetClosestPoint(const PointType& pt, PointVector& neighbors,
                         int max_num = 5, double max_range = 5.0) const;

    size_t NumValidGrids() const;

   private:
    mutable std::shared_mutex rw_mutex_;
    std::shared_ptr<IVoxType>  ivox_;
};

}  // namespace lightning
