# ── lightning_core: pure C++17, no ROS dependencies ──────────────────────────
add_library(lightning_core STATIC
        common/nav_state.cc
        common/constant.h
        common/options.cc
        common/params.cc

        adapter/common/coordinate_utils.cc

        core/gnss/gnss_rtk_handler.cc

        io/yaml_io.cc
        io/file_io.cc

        core/lio/eskf.cc
        core/loop_closing/loop_closing.cc

        core/lidar/local_map.cc
        core/lidar/lidar_pipeline.cc

        core/g2p5/g2p5_subgrid.cc

        core/maps/tiled_map.cc
        core/maps/tiled_map_chunk.cc

        core/localization/localization_result.cc
        core/localization/pose_graph/pgo.cc
        core/localization/pose_graph/pgo_impl.cc
        core/localization/pose_graph/pose_extrapolator.cc

        utils/timer.cc
        utils/pointcloud_utils.cc
)

include_directories(${PROJECT_SOURCE_DIR}/src/core/miao)
add_subdirectory(core/miao)

target_link_libraries(lightning_core
        ${third_party_libs}
        ${PCL_LIBRARIES}
        miao.core
        miao.utils
)

# ── lightning.libs: ROS-dependent layer (thin wrapper over lightning_core) ────
add_library(${PROJECT_NAME}.libs SHARED
        adapter/ros2/ros2_converter.cc

        core/lio/laser_mapping.cc
        core/lio/pointcloud_preprocess.cc

        core/engine/slam_engine.cc
        core/engine/loc_engine.cc

        core/g2p5/g2p5_map.cc
        core/g2p5/g2p5.cc

        core/localization/lidar_loc/lidar_loc.cc
        core/localization/lidar_loc/pclomp/ndt_omp.cpp
        core/localization/lidar_loc/pclomp/voxel_grid_covariance_omp.cpp

        core/localization/localization.cpp

        ui/pangolin_window.cc
        ui/pangolin_window_impl.cc
        ui/ui_car.cc
        ui/ui_cloud.cc
        ui/ui_trajectory.cc

        core/system/slam.cc
        core/system/loc_system.cc

        wrapper/bag_io.cc
)

ament_target_dependencies(
        ${PROJECT_NAME}.libs
        "rclcpp"
        "std_msgs"
        "geometry_msgs"
        "sensor_msgs"
        "nav_msgs"
        "std_srvs"
        "message_filters"
        "pcl_conversions"
        "tf2_ros"
        "tf2"
)

add_dependencies(${PROJECT_NAME}.libs
        livox_interfaces2__rosidl_typesupport_cpp
        ${PROJECT_NAME}__rosidl_typesupport_cpp
)

target_link_libraries(${PROJECT_NAME}.libs
        lightning_core
        ${third_party_libs}
        ${PCL_LIBRARIES}
        miao.core
        miao.utils
        livox_interfaces2__rosidl_typesupport_cpp
        ${PROJECT_NAME}__rosidl_typesupport_cpp
)

install(TARGETS ${PROJECT_NAME}.libs lightning_core miao.core miao.utils
        DESTINATION lib)

# ── Snapshot debug hooks: compile into lightning.libs when GTest is present ──
# The hooks are no-ops unless SetDebugCallback() is called (capture_golden only).
find_package(GTest QUIET)
if(GTest_FOUND)
    target_compile_definitions(${PROJECT_NAME}.libs PUBLIC LIGHTNING_SNAPSHOT_DEBUG)
endif()

add_subdirectory(app)
