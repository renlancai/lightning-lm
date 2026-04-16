add_executable(run_frontend_offline
        run_frontend_offline.cc
)

target_link_libraries(run_frontend_offline
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)

add_executable(run_loop_offline
        run_loop_offline.cc
)

target_link_libraries(run_loop_offline
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)

add_executable(run_slam_offline
        run_slam_offline.cc
)

target_link_libraries(run_slam_offline
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)

add_executable(run_slam_online
        run_slam_online.cc
)

target_link_libraries(run_slam_online
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)

add_executable(run_loc_offline
        run_loc_offline.cc
)

target_link_libraries(run_loc_offline
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)

add_executable(run_loc_online
        run_loc_online.cc
)

target_link_libraries(run_loc_online
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)

add_executable(test_ui
        test_ui.cc
)

target_link_libraries(test_ui
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)

install(TARGETS run_slam_offline run_slam_online run_loc_offline run_loc_online
        DESTINATION lib/${PROJECT_NAME})

# ── Snapshot tools (Phase 0 — compiled with LIGHTNING_SNAPSHOT_DEBUG) ─────────
find_package(GTest QUIET)
if(GTest_FOUND)

add_executable(capture_golden
    ${PROJECT_SOURCE_DIR}/tests/snapshot/capture_golden.cc
)
target_compile_definitions(capture_golden PRIVATE LIGHTNING_SNAPSHOT_DEBUG)
target_include_directories(capture_golden PRIVATE
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}
)
target_link_libraries(capture_golden
    ${PROJECT_NAME}.libs
    ${third_party_libs}
)

add_executable(verify_snapshot
    ${PROJECT_SOURCE_DIR}/tests/snapshot/verify_snapshot.cc
)
target_compile_definitions(verify_snapshot PRIVATE LIGHTNING_SNAPSHOT_DEBUG)
target_include_directories(verify_snapshot PRIVATE
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}
)
target_link_libraries(verify_snapshot
    ${PROJECT_NAME}.libs
    ${third_party_libs}
    GTest::GTest
    GTest::Main
)

# ── Phase 5: zero-ROS engine smoke test ──────────────────────────────────────
# Source includes only slam_engine.h / loc_engine.h (no ROS headers).
# The binary links against lightning.libs (which uses ROS internally), but
# the *source* is verified to compile without any ROS include.
add_executable(test_core_no_ros
    ${PROJECT_SOURCE_DIR}/tests/offline/test_core_no_ros.cc
)
target_include_directories(test_core_no_ros PRIVATE
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}
)
target_link_libraries(test_core_no_ros
    ${PROJECT_NAME}.libs
    ${third_party_libs}
    GTest::GTest
    GTest::Main
)

endif()  # GTest_FOUND

