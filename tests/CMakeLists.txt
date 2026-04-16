cmake_minimum_required(VERSION 3.16)
project(lightning_tests)

enable_testing()

# ── Level 1: pure unit tests (no ROS, no data) ───────────────────────────────
add_subdirectory(unit)

# ── Level 2: offline engine compile/smoke tests ───────────────────────────────
add_subdirectory(offline)
