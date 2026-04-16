#include "core/lidar/local_map.h"

namespace lightning {

LocalMap::LocalMap(const Options& opts) : ivox_(std::make_shared<IVoxType>(opts)) {}

void LocalMap::AddPoints(const PointVector& pts) {
    std::unique_lock<std::shared_mutex> lk(rw_mutex_);
    ivox_->AddPoints(pts);
}

bool LocalMap::GetClosestPoint(const PointType& pt, PointVector& neighbors,
                               int max_num, double max_range) const {
    std::shared_lock<std::shared_mutex> lk(rw_mutex_);
    return ivox_->GetClosestPoint(pt, neighbors, max_num, max_range);
}

size_t LocalMap::NumValidGrids() const {
    std::shared_lock<std::shared_mutex> lk(rw_mutex_);
    return ivox_->NumValidGrids();
}

}  // namespace lightning
