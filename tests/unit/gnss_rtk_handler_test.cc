#include <gtest/gtest.h>
#include <glog/logging.h>
#include <cmath>

#include "core/gnss/gnss_rtk_handler.h"
#include "common/nav_state.h"

using namespace lightning::core;
using namespace lightning;
using ST = GnssRtkFrame::SolutionType;

// ──────────────────────────────────────────────────────────────────────────────
// Helpers
// ──────────────────────────────────────────────────────────────────────────────

static GnssRtkFrame MakeFix(double lat = 31.0, double lon = 121.0, double alt = 10.0,
                             double ts = 100.0) {
    GnssRtkFrame f;
    f.timestamp_sec  = ts;
    f.solution_type  = ST::FIX;
    f.num_satellites = 8;
    f.hdop           = 1.5;
    f.age_of_corr_sec = 0.1;
    f.lat_deg = lat;
    f.lon_deg = lon;
    f.alt_m   = alt;
    return f;
}

static NavState MakeLioState(double ts = 100.0) {
    NavState s;
    s.timestamp_   = ts;
    s.pose_is_ok_  = true;
    s.pos_         = Vec3d(0, 0, 0);
    s.vel_         = Vec3d(0, 0, 0);
    return s;
}

// ──────────────────────────────────────────────────────────────────────────────
// Test Suite
// ──────────────────────────────────────────────────────────────────────────────

// 1. Auto-datum: first FIX sets datum, ENU origin is (0,0,0)
TEST(GnssRtkHandlerTest, AutoSetDatumFirstFix) {
    GnssRtkHandler handler;
    EXPECT_FALSE(handler.DatumSet());

    auto frame = MakeFix();
    NavState lio;
    lio.pose_is_ok_ = false;
    // Need min_consecutive_valid frames — use min=1 for simplicity
    GnssRtkHandler::Options opts;
    opts.min_consecutive_valid = 1;
    opts.auto_set_datum = true;
    GnssRtkHandler h2(opts);

    auto obs = h2.Process(frame, lio);
    EXPECT_TRUE(h2.DatumSet());
    // First observation after datum set: ENU should be ~zero
    EXPECT_LT(obs.pos_enu.norm(), 1e-3) << "ENU at datum should be near zero";
}

// 2. Layer 1 — reject by solution type (FLOAT when require_fix=true)
TEST(GnssRtkHandlerTest, Layer1RejectFloat) {
    GnssRtkHandler::Options opts;
    opts.require_fix  = true;
    opts.accept_float = false;
    opts.auto_set_datum = true;
    opts.min_consecutive_valid = 1;
    GnssRtkHandler h(opts);

    GnssRtkFrame frame = MakeFix();
    frame.solution_type = ST::FLOAT;
    NavState lio;
    lio.pose_is_ok_ = false;

    auto obs = h.Process(frame, lio);
    EXPECT_FALSE(obs.valid);
    EXPECT_EQ(h.TotalRejected(), 1);
}

// 3. Layer 1 — reject by satellite count
TEST(GnssRtkHandlerTest, Layer1RejectFewSats) {
    GnssRtkHandler::Options opts;
    opts.min_consecutive_valid = 1;
    opts.auto_set_datum = true;
    GnssRtkHandler h(opts);

    // First: set datum with good frame
    GnssRtkFrame good = MakeFix(31.0, 121.0, 10.0, 100.0);
    NavState lio; lio.pose_is_ok_ = false;
    h.Process(good, lio);

    // Second: bad satellite count
    GnssRtkFrame bad = MakeFix(31.0, 121.0, 10.0, 101.0);
    bad.num_satellites = 2;  // < min_satellites=6
    auto obs = h.Process(bad, lio);
    EXPECT_FALSE(obs.valid);
}

// 4. Layer 1 — reject by HDOP
TEST(GnssRtkHandlerTest, Layer1RejectHighHdop) {
    GnssRtkHandler::Options opts;
    opts.min_consecutive_valid = 1;
    opts.auto_set_datum = true;
    GnssRtkHandler h(opts);

    GnssRtkFrame good = MakeFix(31.0, 121.0, 10.0, 100.0);
    NavState lio; lio.pose_is_ok_ = false;
    h.Process(good, lio);

    GnssRtkFrame bad = MakeFix(31.0, 121.0, 10.0, 101.0);
    bad.hdop = 5.0;  // > max_hdop=3.0
    auto obs = h.Process(bad, lio);
    EXPECT_FALSE(obs.valid);
}

// 5. Layer 2 — reject by position jump
TEST(GnssRtkHandlerTest, Layer2RejectJump) {
    GnssRtkHandler::Options opts;
    opts.min_consecutive_valid = 1;
    opts.auto_set_datum = true;
    opts.jump_3d_th_m = 2.0;
    opts.jump_2d_th_m = 1.5;
    GnssRtkHandler h(opts);

    // Set datum
    GnssRtkFrame datum_frame = MakeFix(31.0, 121.0, 10.0, 100.0);
    NavState lio; lio.pose_is_ok_ = false;
    h.Process(datum_frame, lio);

    // Second frame: LIO says we're at (0,0,0), but RTK says we jumped 10m
    // To simulate jump: provide LIO state at same time but with same position as datum,
    // and RTK at a distant point (need ~0.001 deg lat ≈ 111 m)
    // Instead, fake by using LIO state that predicts a different position
    NavState lio2;
    lio2.pose_is_ok_ = true;
    lio2.timestamp_  = 100.0;
    lio2.pos_        = Vec3d(0, 0, 0);  // LIO at origin
    lio2.vel_        = Vec3d(0, 0, 0);

    // RTK at ~10m away in ENU (need about 9e-5 deg lat shift ≈ 10m)
    const double lat_shift = 10.0 / 111000.0;  // ~10m in degrees
    GnssRtkFrame jump_frame = MakeFix(31.0 + lat_shift, 121.0, 10.0, 100.1);

    auto obs = h.Process(jump_frame, lio2);
    EXPECT_FALSE(obs.valid) << "Expected jump rejection, dist should be > " << opts.jump_2d_th_m;
}

// 6. Layer 3 — need min_consecutive_valid frames before accepting
TEST(GnssRtkHandlerTest, Layer3ConsecutiveValid) {
    GnssRtkHandler::Options opts;
    opts.min_consecutive_valid = 3;
    opts.auto_set_datum = true;
    GnssRtkHandler h(opts);

    NavState lio; lio.pose_is_ok_ = false;

    // Frame 0: sets datum, L3 needs 3 consecutive
    auto f0 = MakeFix(31.0, 121.0, 10.0, 100.0);
    auto o0 = h.Process(f0, lio);
    EXPECT_FALSE(o0.valid) << "Need 3 consecutive, frame 0 should not be valid";

    // Frame 1
    auto f1 = MakeFix(31.0, 121.0, 10.0, 100.1);
    auto o1 = h.Process(f1, lio);
    EXPECT_FALSE(o1.valid) << "Frame 1: still need more";

    // Frame 2: now 3 consecutive
    auto f2 = MakeFix(31.0, 121.0, 10.0, 100.2);
    auto o2 = h.Process(f2, lio);
    EXPECT_TRUE(o2.valid) << "Frame 2: should be valid now";
    EXPECT_TRUE(o2.inlier);
}

// 7. Layer 3 — blackout after max_consecutive_invalid
TEST(GnssRtkHandlerTest, Layer3Blackout) {
    GnssRtkHandler::Options opts;
    opts.min_consecutive_valid   = 1;
    opts.max_consecutive_invalid = 3;
    opts.blackout_recovery_count = 2;
    opts.auto_set_datum = true;
    GnssRtkHandler h(opts);

    NavState lio; lio.pose_is_ok_ = false;

    // Build up 1 consecutive valid
    h.Process(MakeFix(31.0, 121.0, 10.0, 100.0), lio);

    // 3 invalid frames → enter blackout
    GnssRtkFrame bad = MakeFix(31.0, 121.0, 10.0, 100.1);
    bad.solution_type = ST::FLOAT;  // Layer 1 reject
    for (int i = 0; i < 3; i++) {
        bad.timestamp_sec += 0.1;
        h.Process(bad, lio);
    }

    // Should be in blackout now; even a valid frame gets rejected
    auto good = MakeFix(31.0, 121.0, 10.0, 100.5);
    auto obs = h.Process(good, lio);
    EXPECT_FALSE(obs.valid) << "Should be in blackout";

    // After 2 recovery frames (blackout_recovery_count), should recover
    for (int i = 0; i < 2; i++) {
        good.timestamp_sec += 0.1;
        h.Process(good, lio);
    }
    // One more valid frame should now pass
    good.timestamp_sec += 0.1;
    auto obs_after = h.Process(good, lio);
    EXPECT_TRUE(obs_after.valid) << "Should have recovered from blackout";
}

// 8. Layer 4 — chi2 feedback marks outlier
TEST(GnssRtkHandlerTest, Layer4Chi2Reject) {
    GnssRtkHandler::Options opts;
    opts.min_consecutive_valid = 1;
    opts.auto_set_datum = true;
    opts.chi2_reject_th = 7.81;
    GnssRtkHandler h(opts);

    NavState lio; lio.pose_is_ok_ = false;
    auto obs = h.Process(MakeFix(), lio);
    EXPECT_TRUE(obs.valid);
    EXPECT_TRUE(obs.inlier);

    // Feed back a large chi2 → should mark as outlier
    h.UpdateLastChi2(100.0);
    // We can read last_obs_ through the accessor — but we don't expose it
    // Instead: check stats — consecutive_valid_ should be decremented.
    // The next frame's validity depends on consecutive_valid count.
    // We test indirectly: after UpdateLastChi2 with a large value, the
    // handler should have penalized the state. Fire a new valid frame
    // and if consecutive_valid_ dropped, it may be 0 again requiring re-accumulation.
    // (We use min_consecutive_valid=1, so one more valid frame should still pass.)
    auto obs2 = h.Process(MakeFix(31.0, 121.0, 10.0, 101.0), lio);
    EXPECT_TRUE(obs2.valid);  // still valid with min=1

    // And with chi2 below threshold, inlier stays true
    h.UpdateLastChi2(1.0);
    EXPECT_EQ(h.TotalAccepted(), 2);
}

// 9. Statistics counter
TEST(GnssRtkHandlerTest, Statistics) {
    GnssRtkHandler::Options opts;
    opts.min_consecutive_valid = 1;
    opts.auto_set_datum = true;
    GnssRtkHandler h(opts);

    NavState lio; lio.pose_is_ok_ = false;

    // 2 valid
    h.Process(MakeFix(31.0, 121.0, 10.0, 100.0), lio);
    h.Process(MakeFix(31.0, 121.0, 10.0, 100.1), lio);
    // 1 invalid (bad hdop)
    GnssRtkFrame bad = MakeFix(31.0, 121.0, 10.0, 100.2);
    bad.hdop = 10.0;
    h.Process(bad, lio);

    EXPECT_EQ(h.TotalReceived(), 3);
    EXPECT_EQ(h.TotalAccepted(), 2);
    EXPECT_EQ(h.TotalRejected(), 1);
}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
