// Phase 5: SlamEngine implementation.
//
// NOTE: This .cc file may transitively include ROS headers through
// laser_mapping.h / g2p5_map.h.  That is acceptable — only the *header*
// slam_engine.h must remain ROS-free.

#include "core/engine/slam_engine.h"

#include <yaml-cpp/yaml.h>
#include <filesystem>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/io/pcd_io.h>

#include "common/imu.h"
#include "common/point_def.h"
#include "core/g2p5/g2p5.h"
#include "core/lio/laser_mapping.h"
#include "core/loop_closing/loop_closing.h"
#include "core/maps/tiled_map.h"
#include "ui/pangolin_window.h"
#include "utils/timer.h"

namespace lightning::core {

SlamEngine::SlamEngine() : SlamEngine(Options{}) {}
SlamEngine::SlamEngine(Options options) : options_(options) {}

SlamEngine::~SlamEngine() {
    if (ui_) { ui_->Quit(); }
}

bool SlamEngine::Init(const std::string& yaml_path) {
    lio_ = std::make_shared<LaserMapping>();
    if (!lio_->Init(yaml_path)) {
        LOG(ERROR) << "SlamEngine: failed to init LIO";
        return false;
    }

    auto yaml = YAML::LoadFile(yaml_path);
    options_.with_loop_closing = yaml["system"]["with_loop_closing"].as<bool>();
    options_.with_ui           = yaml["system"]["with_ui"].as<bool>();
    options_.with_gridmap      = yaml["system"]["with_g2p5"].as<bool>();
    options_.step_on_kf        = yaml["system"]["step_on_kf"].as<bool>();

    if (options_.with_loop_closing) {
        LoopClosing::Options lc_opts;
        lc_opts.online_mode_ = options_.online_mode_;
        lc_ = std::make_shared<LoopClosing>(lc_opts);
        lc_->Init(yaml_path);
    }

    if (options_.with_ui) {
        LOG(INFO) << "SlamEngine: starting 3-D UI";
        ui_ = std::make_shared<ui::PangolinWindow>();
        ui_->Init();
        lio_->SetUI(ui_);
    }

    if (options_.with_gridmap) {
        g2p5::G2P5::Options gopt;
        gopt.online_mode_ = options_.online_mode_;
        g2p5_ = std::make_shared<g2p5::G2P5>(gopt);
        g2p5_->Init(yaml_path);

        if (lc_) {
            lc_->SetLoopClosedCB([this]() { g2p5_->RedrawGlobalMap(); });
        }

        g2p5_->SetMapUpdateCallback([this](g2p5::G2P5MapPtr map) {
            cv::Mat image = map->ToCV();
            cv::imshow("map", image);
            cv::waitKey(options_.step_on_kf ? 0 : 10);
        });
    }

    return true;
}

void SlamEngine::StartMapping(const std::string& map_name) {
    map_name_ = map_name;
    running_  = true;
}

void SlamEngine::SaveMap(const std::string& path) {
    std::string save_path = path.empty() ? ("./data/" + map_name_ + "/") : path;
    LOG(INFO) << "SlamEngine: saving map to " << save_path;

    if (!std::filesystem::exists(save_path)) {
        std::filesystem::create_directories(save_path);
    } else {
        std::filesystem::remove_all(save_path);
        std::filesystem::create_directories(save_path);
    }

    auto global_map = lio_->GetGlobalMap(!options_.with_loop_closing);

    TiledMap::Options tm_options;
    tm_options.map_path_ = save_path;
    TiledMap tm(tm_options);
    SE3 start_pose = lio_->GetAllKeyframes().front()->GetOptPose();
    tm.ConvertFromFullPCD(global_map, start_pose, save_path);

    pcl::io::savePCDFileBinaryCompressed(save_path + "/global.pcd", *global_map);

    if (options_.with_gridmap && g2p5_) {
        auto map          = g2p5_->GetNewestMap()->ToROS();
        const int width   = static_cast<int>(map.info.width);
        const int height  = static_cast<int>(map.info.height);

        cv::Mat nav_image(height, width, CV_8UC1);
        for (int y = 0; y < height; ++y) {
            const int row = y * width;
            for (int x = 0; x < width; ++x) {
                int8_t d = map.data[row + x];
                if (d == 0)        nav_image.at<uchar>(height - 1 - y, x) = 255;
                else if (d == 100) nav_image.at<uchar>(height - 1 - y, x) = 0;
                else               nav_image.at<uchar>(height - 1 - y, x) = 128;
            }
        }
        cv::imwrite(save_path + "/map.pgm", nav_image);

        std::ofstream yaml_f(save_path + "/map.yaml");
        if (yaml_f) {
            YAML::Emitter emitter;
            emitter << YAML::BeginMap;
            emitter << YAML::Key << "image"          << YAML::Value << "map.pgm";
            emitter << YAML::Key << "mode"           << YAML::Value << "trinary";
            emitter << YAML::Key << "width"          << YAML::Value << map.info.width;
            emitter << YAML::Key << "height"         << YAML::Value << map.info.height;
            emitter << YAML::Key << "resolution"     << YAML::Value << float(0.05);
            std::vector<double> orig{map.info.origin.position.x,
                                     map.info.origin.position.y, 0.0};
            emitter << YAML::Key << "origin"         << YAML::Value << orig;
            emitter << YAML::Key << "negate"         << YAML::Value << 0;
            emitter << YAML::Key << "occupied_thresh"<< YAML::Value << 0.65;
            emitter << YAML::Key << "free_thresh"    << YAML::Value << 0.25;
            emitter << YAML::EndMap;
            yaml_f << emitter.c_str();
        }
    }

    LOG(INFO) << "SlamEngine: map saved";
}

void SlamEngine::FeedImu(const ImuFrame& imu) {
    if (!running_) return;

    auto imu_ptr                 = std::make_shared<IMU>();
    imu_ptr->timestamp           = imu.timestamp_sec;
    imu_ptr->linear_acceleration = imu.acc;
    imu_ptr->angular_velocity    = imu.gyro;

    lio_->ProcessIMU(imu_ptr);
}

void SlamEngine::FeedLidar(const PointCloudFrame& frame) {
    if (!running_) return;

    // Convert platform-agnostic PointCloudFrame → PCL CloudPtr.
    // PointType = PointXYZIT whose '.time' field stores per-point time offset (sec).
    // cloud->header.stamp is in nanoseconds (read via math::ToSec = t * 1e-9).
    CloudPtr cloud(new PointCloudType());
    cloud->header.stamp = static_cast<uint64_t>(frame.timestamp_sec * 1e9);
    cloud->reserve(frame.points.size());

    for (const auto& p : frame.points) {
        PointType pt;
        pt.x         = p.x;
        pt.y         = p.y;
        pt.z         = p.z;
        pt.intensity = p.intensity;
        pt.time      = p.time_offset_sec;
        cloud->push_back(pt);
    }

    lio_->ProcessPointCloud2(cloud);
    lio_->Run();

    auto kf = lio_->GetKeyframe();
    if (kf != cur_kf_) {
        cur_kf_ = kf;
        if (cur_kf_) {
            HandleNewKeyframe(cur_kf_);
        }
    }
}

void SlamEngine::HandleNewKeyframe(Keyframe::Ptr kf) {
    if (lc_)   { lc_->AddKF(kf);         }
    if (g2p5_) { g2p5_->PushKeyframe(kf); }
    if (ui_)   { ui_->UpdateKF(kf);       }
}

std::vector<Keyframe::Ptr> SlamEngine::GetAllKeyframes() const {
    if (lio_) return lio_->GetAllKeyframes();
    return {};
}

}  // namespace lightning::core
