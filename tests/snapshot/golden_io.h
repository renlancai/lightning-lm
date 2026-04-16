#pragma once

/**
 * Golden snapshot I/O for Phase 0 characterization tests.
 *
 * Binary format per frame:
 *   [magic:4B][version:4B][frame_id:4B]
 *   [n_imu:4B][imu_ts × n_imu : f64]
 *   [n_pts:4B][pts × n_pts : 3×f32]
 *   [HTH:36×f64][HTr:6×f64][valid_num:4B]
 *   [pos:3×f64][vel:3×f64][bg:3×f64][rot:4×f64 (xyzw)]
 *   [is_kf:4B][kf_pos:3×f64][kf_rot:4×f64 (xyzw)]
 *
 * One file per frame: golden_NNNN.bin  (NNNN = zero-padded frame id)
 */

#include <Eigen/Dense>
#include <array>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace lightning::snapshot {

static constexpr uint32_t kMagic   = 0x474F4C44u;  // "GOLD"
static constexpr uint32_t kVersion = 1u;

struct CheckPoint {
    int frame_id = -1;

    // CP-1
    std::vector<double> imu_timestamps;

    // CP-2
    std::vector<std::array<float, 3>> undistort_pts;

    // CP-3
    Eigen::Matrix<double, 6, 6> HTH{Eigen::Matrix<double, 6, 6>::Zero()};
    Eigen::Matrix<double, 6, 1> HTr{Eigen::Matrix<double, 6, 1>::Zero()};
    int valid_num = 0;

    // CP-4
    Eigen::Vector3d pos{Eigen::Vector3d::Zero()};
    Eigen::Vector3d vel{Eigen::Vector3d::Zero()};
    Eigen::Vector3d bg{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond rot{Eigen::Quaterniond::Identity()};

    // CP-5
    bool is_keyframe = false;
    Eigen::Vector3d kf_pos{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond kf_rot{Eigen::Quaterniond::Identity()};
};

// ── helpers ────────────────────────────────────────────────────────────────
namespace detail {

template <typename T>
inline void write_pod(std::ofstream& f, const T& v) {
    f.write(reinterpret_cast<const char*>(&v), sizeof(T));
}
template <typename T>
inline void read_pod(std::ifstream& f, T& v) {
    f.read(reinterpret_cast<char*>(&v), sizeof(T));
}

inline std::string frame_filename(const std::string& dir, int id) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "golden_%04d.bin", id);
    return dir + "/" + buf;
}

}  // namespace detail

// ── GoldenWriter ────────────────────────────────────────────────────────────
class GoldenWriter {
public:
    explicit GoldenWriter(const std::string& dir) : dir_(dir) {
        std::filesystem::create_directories(dir_);
    }

    void Write(const CheckPoint& cp) {
        std::ofstream f(detail::frame_filename(dir_, cp.frame_id),
                        std::ios::binary | std::ios::trunc);
        if (!f) throw std::runtime_error("GoldenWriter: cannot open file");

        detail::write_pod(f, kMagic);
        detail::write_pod(f, kVersion);
        detail::write_pod(f, static_cast<int32_t>(cp.frame_id));

        // CP-1
        auto n_imu = static_cast<int32_t>(cp.imu_timestamps.size());
        detail::write_pod(f, n_imu);
        for (double ts : cp.imu_timestamps) detail::write_pod(f, ts);

        // CP-2
        auto n_pts = static_cast<int32_t>(cp.undistort_pts.size());
        detail::write_pod(f, n_pts);
        for (const auto& p : cp.undistort_pts) {
            detail::write_pod(f, p[0]);
            detail::write_pod(f, p[1]);
            detail::write_pod(f, p[2]);
        }

        // CP-3
        for (int r = 0; r < 6; ++r)
            for (int c = 0; c < 6; ++c) detail::write_pod(f, cp.HTH(r, c));
        for (int r = 0; r < 6; ++r) detail::write_pod(f, cp.HTr(r));
        detail::write_pod(f, static_cast<int32_t>(cp.valid_num));

        // CP-4
        for (int i = 0; i < 3; ++i) detail::write_pod(f, cp.pos[i]);
        for (int i = 0; i < 3; ++i) detail::write_pod(f, cp.vel[i]);
        for (int i = 0; i < 3; ++i) detail::write_pod(f, cp.bg[i]);
        detail::write_pod(f, cp.rot.x());
        detail::write_pod(f, cp.rot.y());
        detail::write_pod(f, cp.rot.z());
        detail::write_pod(f, cp.rot.w());

        // CP-5
        detail::write_pod(f, static_cast<int32_t>(cp.is_keyframe ? 1 : 0));
        for (int i = 0; i < 3; ++i) detail::write_pod(f, cp.kf_pos[i]);
        detail::write_pod(f, cp.kf_rot.x());
        detail::write_pod(f, cp.kf_rot.y());
        detail::write_pod(f, cp.kf_rot.z());
        detail::write_pod(f, cp.kf_rot.w());
    }

private:
    std::string dir_;
};

// ── GoldenReader ────────────────────────────────────────────────────────────
class GoldenReader {
public:
    static CheckPoint Read(const std::string& dir, int frame_id) {
        std::ifstream f(detail::frame_filename(dir, frame_id),
                        std::ios::binary);
        if (!f) throw std::runtime_error(
            "GoldenReader: file not found for frame " + std::to_string(frame_id));

        uint32_t magic{}, version{};
        detail::read_pod(f, magic);
        detail::read_pod(f, version);
        if (magic != kMagic)
            throw std::runtime_error("GoldenReader: bad magic");
        if (version != kVersion)
            throw std::runtime_error("GoldenReader: unsupported version");

        CheckPoint cp;
        int32_t fid{};
        detail::read_pod(f, fid);
        cp.frame_id = fid;

        // CP-1
        int32_t n_imu{};
        detail::read_pod(f, n_imu);
        cp.imu_timestamps.resize(n_imu);
        for (int i = 0; i < n_imu; ++i) detail::read_pod(f, cp.imu_timestamps[i]);

        // CP-2
        int32_t n_pts{};
        detail::read_pod(f, n_pts);
        cp.undistort_pts.resize(n_pts);
        for (int i = 0; i < n_pts; ++i) {
            detail::read_pod(f, cp.undistort_pts[i][0]);
            detail::read_pod(f, cp.undistort_pts[i][1]);
            detail::read_pod(f, cp.undistort_pts[i][2]);
        }

        // CP-3
        for (int r = 0; r < 6; ++r)
            for (int c = 0; c < 6; ++c) detail::read_pod(f, cp.HTH(r, c));
        for (int r = 0; r < 6; ++r) detail::read_pod(f, cp.HTr(r));
        int32_t vn{};
        detail::read_pod(f, vn);
        cp.valid_num = vn;

        // CP-4
        for (int i = 0; i < 3; ++i) detail::read_pod(f, cp.pos[i]);
        for (int i = 0; i < 3; ++i) detail::read_pod(f, cp.vel[i]);
        for (int i = 0; i < 3; ++i) detail::read_pod(f, cp.bg[i]);
        double qx{}, qy{}, qz{}, qw{};
        detail::read_pod(f, qx); detail::read_pod(f, qy);
        detail::read_pod(f, qz); detail::read_pod(f, qw);
        cp.rot = Eigen::Quaterniond{qw, qx, qy, qz};

        // CP-5
        int32_t is_kf{};
        detail::read_pod(f, is_kf);
        cp.is_keyframe = (is_kf != 0);
        for (int i = 0; i < 3; ++i) detail::read_pod(f, cp.kf_pos[i]);
        double kqx{}, kqy{}, kqz{}, kqw{};
        detail::read_pod(f, kqx); detail::read_pod(f, kqy);
        detail::read_pod(f, kqz); detail::read_pod(f, kqw);
        cp.kf_rot = Eigen::Quaterniond{kqw, kqx, kqy, kqz};

        return cp;
    }
};

}  // namespace lightning::snapshot
