cmake_minimum_required(VERSION 3.16)

find_package(GTest REQUIRED)
find_package(Eigen3 REQUIRED)

# Repo root: two levels up from tests/unit/
get_filename_component(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

# ── coordinate_utils 单元测试（零 ROS 依赖）──────────────────────────
add_executable(test_coordinate_utils
    coordinate_utils_test.cc
    ${REPO_ROOT}/src/adapter/common/coordinate_utils.cc
)

target_include_directories(test_coordinate_utils PRIVATE
    ${REPO_ROOT}/src
    ${REPO_ROOT}/thirdparty
    ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(test_coordinate_utils
    GTest::GTest
    GTest::Main
    glog
    gflags
)

target_compile_features(test_coordinate_utils PRIVATE cxx_std_17)

add_test(NAME CoordinateUtils COMMAND test_coordinate_utils)

# ── gnss_rtk_handler 单元测试（零 ROS 依赖）──────────────────────────
add_executable(test_gnss_rtk_handler
    gnss_rtk_handler_test.cc
    ${REPO_ROOT}/src/core/gnss/gnss_rtk_handler.cc
    ${REPO_ROOT}/src/adapter/common/coordinate_utils.cc
    ${REPO_ROOT}/src/common/nav_state.cc
)

target_include_directories(test_gnss_rtk_handler PRIVATE
    ${REPO_ROOT}/src
    ${REPO_ROOT}/thirdparty
    ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(test_gnss_rtk_handler
    GTest::GTest
    GTest::Main
    glog
    gflags
)

target_compile_features(test_gnss_rtk_handler PRIVATE cxx_std_17)

add_test(NAME GnssRtkHandler COMMAND test_gnss_rtk_handler)
