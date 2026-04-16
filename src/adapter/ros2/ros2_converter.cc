#include "adapter/ros2/ros2_converter.h"

#include <pcl_conversions/pcl_conversions.h>

#include "common/point_def.h"

namespace lightning::adapter::ros2 {

core::ImuFrame FromImu(const sensor_msgs::msg::Imu& msg) {
    core::ImuFrame f;
    f.timestamp_sec = ToSec(msg.header.stamp);
    f.acc  = {msg.linear_acceleration.x,
              msg.linear_acceleration.y,
              msg.linear_acceleration.z};
    f.gyro = {msg.angular_velocity.x,
              msg.angular_velocity.y,
              msg.angular_velocity.z};
    return f;
}

lightning::IMUPtr FromImuLegacy(const sensor_msgs::msg::Imu::SharedPtr& msg) {
    auto imu                  = std::make_shared<lightning::IMU>();
    imu->timestamp            = ToSec(msg->header.stamp);
    imu->linear_acceleration  = {msg->linear_acceleration.x,
                                  msg->linear_acceleration.y,
                                  msg->linear_acceleration.z};
    imu->angular_velocity     = {msg->angular_velocity.x,
                                  msg->angular_velocity.y,
                                  msg->angular_velocity.z};
    return imu;
}

core::PointCloudFrame FromPointCloud2(const sensor_msgs::msg::PointCloud2& msg,
                                      double time_scale) {
    core::PointCloudFrame frame;
    frame.timestamp_sec = ToSec(msg.header.stamp);
    frame.seq           = ToNanoSec(msg.header.stamp);

    pcl::PointCloud<PointType> pcl_cloud;
    pcl::fromROSMsg(msg, pcl_cloud);

    frame.points.reserve(pcl_cloud.size());
    for (const auto& p : pcl_cloud) {
        core::RawPoint rp;
        rp.x              = p.x;
        rp.y              = p.y;
        rp.z              = p.z;
        rp.intensity      = p.intensity;
        rp.time_offset_sec = static_cast<double>(p.time) * time_scale;
        frame.points.push_back(rp);
    }
    return frame;
}

core::GnssRtkFrame FromNavSatFix(const sensor_msgs::msg::NavSatFix& msg) {
    using namespace sensor_msgs::msg;
    core::GnssRtkFrame frame;
    frame.timestamp_sec = ToSec(msg.header.stamp);
    frame.lat_deg       = msg.latitude;
    frame.lon_deg       = msg.longitude;
    frame.alt_m         = msg.altitude;

    using ST = core::GnssRtkFrame::SolutionType;
    switch (msg.status.status) {
        case NavSatStatus::STATUS_FIX:
            frame.solution_type = ST::FIX;
            break;
        case NavSatStatus::STATUS_SBAS_FIX:
            frame.solution_type = ST::SBAS;
            break;
        default:
            frame.solution_type = ST::INVALID;
            break;
    }

    if (msg.position_covariance_type != NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
        frame.pos_std_enu = {
            std::sqrt(std::max(0.0, msg.position_covariance[0])),
            std::sqrt(std::max(0.0, msg.position_covariance[4])),
            std::sqrt(std::max(0.0, msg.position_covariance[8]))
        };
    }
    return frame;
}

geometry_msgs::msg::TransformStamped LocResultToGeoMsg(const loc::LocalizationResult& r) {
    geometry_msgs::msg::TransformStamped msg;
    msg.header.frame_id = "map";
    msg.header.stamp.sec    = static_cast<int32_t>(r.timestamp_);
    msg.header.stamp.nanosec = static_cast<uint32_t>((r.timestamp_ - msg.header.stamp.sec) * 1e9);
    msg.child_frame_id = "base_link";

    msg.transform.translation.x = r.pose_.translation().x();
    msg.transform.translation.y = r.pose_.translation().y();
    msg.transform.translation.z = r.pose_.translation().z();
    msg.transform.rotation.x = r.pose_.so3().unit_quaternion().x();
    msg.transform.rotation.y = r.pose_.so3().unit_quaternion().y();
    msg.transform.rotation.z = r.pose_.so3().unit_quaternion().z();
    msg.transform.rotation.w = r.pose_.so3().unit_quaternion().w();
    return msg;
}

}  // namespace lightning::adapter::ros2
