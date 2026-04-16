//
// Phase 5: run_loc_online — thin ROS 2 adapter using LocEngine.
//
// All localization logic is in LocEngine (zero ROS in its header).
// This file handles subscriptions and ROS → platform-agnostic conversion.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "adapter/ros2/ros2_converter.h"
#include "core/engine/loc_engine.h"
#include "io/yaml_io.h"
#include "utils/timer.h"

#include "livox_ros_driver2/msg/custom_msg.hpp"

DEFINE_string(config, "./config/default.yaml", "配置文件");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold  = google::INFO;
    google::ParseCommandLineFlags(&argc, &argv, true);

    rclcpp::init(argc, argv);

    using namespace lightning;
    using namespace lightning::adapter::ros2;

    YAML_IO yaml(FLAGS_config);
    const std::string imu_topic   = yaml.GetValue<std::string>("common", "imu_topic");
    const std::string cloud_topic = yaml.GetValue<std::string>("common", "lidar_topic");
    const std::string livox_topic = yaml.GetValue<std::string>("common", "livox_lidar_topic");
    const double      time_scale  = yaml.GetValue<double>("fasterlio", "time_scale");

    // ── Engine (zero ROS header) ──────────────────────────────────────────────
    core::LocEngine::Options eng_opts;
    eng_opts.pub_tf = true;

    auto engine = std::make_shared<core::LocEngine>(eng_opts);
    if (!engine->Init(FLAGS_config)) {
        LOG(ERROR) << "failed to init LocEngine";
        return -1;
    }

    // Default starting pose
    engine->SetInitPose(SE3());

    // ── ROS 2 node (adapter layer) ────────────────────────────────────────────
    auto node = std::make_shared<rclcpp::Node>("lightning_loc");
    rclcpp::QoS qos(10);

    // TF broadcaster — wired up through Localization's TFCallback
    auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

    auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, qos,
        [&engine](sensor_msgs::msg::Imu::SharedPtr msg) {
            engine->FeedImu(FromImu(*msg));
        });

    auto cloud_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic, qos,
        [&engine, time_scale](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            Timer::Evaluate([&]() {
                engine->FeedLidar(FromPointCloud2(*msg, time_scale));
            }, "Proc Lidar", true);
        });

    auto livox_sub = node->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        livox_topic, qos,
        [&engine](livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
            core::PointCloudFrame frame;
            frame.timestamp_sec = static_cast<double>(msg->header.stamp.sec) +
                                  1e-9 * msg->header.stamp.nanosec;
            frame.seq = msg->header.stamp.sec * 1'000'000'000ULL + msg->header.stamp.nanosec;
            frame.points.reserve(msg->point_num);
            for (const auto& p : msg->points) {
                core::RawPoint rp;
                rp.x              = p.x;
                rp.y              = p.y;
                rp.z              = p.z;
                rp.intensity      = static_cast<float>(p.reflectivity);
                rp.time_offset_sec = static_cast<double>(p.offset_time) * 1e-9;
                frame.points.push_back(rp);
            }
            Timer::Evaluate([&]() {
                engine->FeedLidar(frame);
            }, "Proc Lidar", true);
        });

    // Optional: subscribe to NavSatFix for RTK fusion
    // auto rtk_sub = node->create_subscription<sensor_msgs::msg::NavSatFix>(
    //     "/gnss/fix", qos,
    //     [&engine](sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    //         engine->FeedRtk(FromNavSatFix(*msg));
    //     });

    rclcpp::spin(node);

    engine->Finish();
    Timer::PrintAll();
    rclcpp::shutdown();

    LOG(INFO) << "done";
    return 0;
}
