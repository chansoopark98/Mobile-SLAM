// P3 candidate R6 (AccuracyConfig::r6_span_velocity): motion-prediction velocity over a time span.
// Synthetic poses only (no fixture): a short input pair must not inflate the velocity, uniform
// timing must reproduce the original UpdateHistory velocity bit for bit, a restart or a gap must
// not reach across the discontinuity, and the rotation path of MotionPrediction is untouched.

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "c8/geometry/egomotion.h"
#include "c8/hmatrix.h"
#include "c8/hpoint.h"
#include "c8/quaternion.h"
#include "config/accuracy.h"
#include "config/params.h"
#include "vio_tracker/span_velocity.h"
#include "vio_tracker/tracker_modules.h"
#include "vio_tracker/vio_tracker.h"
#include "vio_tracker/vio_tracker_internal.h"

namespace slam_refact::vio_tracker {
namespace {

// G1 binds its gtest XML to the binary under test: the harness passes the hash of the candidate
// binary in SR_BINARY_SHA256 and it is recorded as a property of the whole run.
class BindBinaryEnvironment : public ::testing::Environment {
 public:
  void SetUp() override {
    const char* sha = std::getenv("SR_BINARY_SHA256");
    if (sha != nullptr && *sha != '\0') {
      ::testing::Test::RecordProperty("binary_sha256", std::string(sha));
    }
  }
};
const ::testing::Environment* const kBindBinary =
    ::testing::AddGlobalTestEnvironment(new BindBinaryEnvironment);

constexpr int64_t kFrameNs = 33333333;

c8::HMatrix PoseAt(float x, float y, float z) { return c8::cameraMotion(c8::HPoint3(x, y, z), c8::Quaternion()); }

FrameRecord RecordAt(int64_t t_ns, c8::Quaternion q = c8::Quaternion()) {
  FrameRecord r;
  r.time_ns = t_ns;
  r.xr_device_rotation = q;
  return r;
}

// Two histories driven side by side: `orig` with UpdateHistory only, `r6` with ApplySpanVelocity.
struct Pair {
  FrameState orig[2];
  FrameState r6[2];
  SpanVelocity span;
  void Step(int64_t t_ns, float x, float y, float z, bool restart = false, bool world = false) {
    const FrameRecord now = RecordAt(t_ns);
    UpdateHistory(orig, PoseAt(x, y, z), now, world);
    UpdateHistory(r6, PoseAt(x, y, z), now, world);
    ApplySpanVelocity(&span, r6, restart || world, world);
  }
};

float Len(const c8::HVector3& v) { return std::sqrt(v.x() * v.x() + v.y() * v.y() + v.z() * v.z()); }

TEST(R6SpanVelocity, ShortPairDoesNotInflateVelocity) {
  // Real motion 0.3 u/s along x at 30 Hz. Frame 10 arrives 3.3 ms after frame 9 (S2 frame 287 ->
  // 288) with a tracker jitter of 0.004 u in x that the real motion does not explain.
  Pair h;
  int64_t t = 0;
  for (int k = 0; k < 10; ++k) {
    h.Step(t, static_cast<float>(0.3 * t * 1e-9), 0.f, 0.f);
    t += kFrameNs;
  }
  t = t - kFrameNs + 3300000;
  h.Step(t, static_cast<float>(0.3 * t * 1e-9 + 0.004), 0.f, 0.f);
  EXPECT_GT(Len(h.orig[1].velocity), 1.0f) << "the original estimate must show the inflation";
  // Exact expectation: the last seven intervals are six nominal ones and the 3.3 ms one, so the
  // median is kFrameNs; the spans from the newest pose are 3.3 ms (error ~30 ms) and 36.6 ms (error
  // 3.3 ms): the reference is the pose two frames back, and v = dx / real dt of that span.
  EXPECT_EQ(h.span.nominal_interval_ns(), kFrameNs);
  const int64_t t8 = 8 * kFrameNs;
  const double expect_vx = (0.3 * t * 1e-9 + 0.004 - 0.3 * t8 * 1e-9) / ((t - t8) * 1e-9);
  EXPECT_NEAR(h.r6[1].velocity.x(), expect_vx, 1e-4);
  EXPECT_NEAR(expect_vx, 0.3 + 0.004 / ((t - t8) * 1e-9), 1e-9);  // = 0.409 u/s, not 1.5 u/s
  EXPECT_EQ(h.r6[1].velocity.y(), 0.0f);
  // The prediction step of the next frame (one nominal interval later) follows the velocity.
  const FrameRecord next = RecordAt(t + kFrameNs);
  const FrameState po = PredictMotionWithParams(h.orig[1], next, 0.f, 0.f);
  const FrameState pr = PredictMotionWithParams(h.r6[1], next, 0.f, 0.f);
  const float step_o = po.pose(0, 3) - h.orig[1].pose(0, 3);
  const float step_r = pr.pose(0, 3) - h.r6[1].pose(0, 3);
  EXPECT_GT(step_o, 0.05f);
  EXPECT_LT(step_r, 0.02f);
  EXPECT_GT(step_r, 0.0f);
}

TEST(R6SpanVelocity, UniformDtEqualsOriginalExactly) {
  Pair h;
  uint32_t lcg = 12345u;
  const auto rnd = [&lcg]() {
    lcg = lcg * 1664525u + 1013904223u;
    return static_cast<float>(lcg >> 8) / static_cast<float>(1u << 24) - 0.5f;
  };
  int64_t t = 1000000000LL;
  float x = 0.f, y = 0.f, z = 0.f;
  int estimated = 0;
  for (int k = 0; k < 80; ++k) {
    x += 0.01f + 0.004f * rnd();
    y += 0.003f * rnd();
    z += 0.01f * rnd();
    h.Step(t, x, y, z);
    t += kFrameNs;
    if (h.span.nominal_interval_ns() != 0) ++estimated;
    for (int i = 0; i < 3; ++i) {
      ASSERT_EQ(h.r6[1].velocity.data()[i], h.orig[1].velocity.data()[i]) << k << " v" << i;
      ASSERT_EQ(h.r6[1].acceleration.data()[i], h.orig[1].acceleration.data()[i]) << k << " a" << i;
    }
  }
  EXPECT_GT(estimated, 70);  // the span path was active, not just the fallback
}

TEST(R6SpanVelocity, RestartAndGapDoNotReachAcrossTheDiscontinuity) {
  Pair h;
  int64_t t = 0;
  for (int k = 0; k < 10; ++k) {
    h.Step(t, 100.f + 0.01f * k, 0.f, 0.f);
    t += kFrameNs;
  }
  // Pose correction: the coordinates jump to 0, poses around 100 are in the old frame. 665 keeps
  // the previous velocity on that frame.
  const c8::HVector3 kept = h.r6[1].velocity;
  h.Step(t, 0.f, 0.f, 0.f, /*restart=*/true, /*world=*/true);
  t += kFrameNs;
  for (int i = 0; i < 3; ++i) EXPECT_EQ(h.r6[1].velocity.data()[i], kept.data()[i]);
  // Next frame: only the corrected pose is a reference; the velocity is the 0.01 u step.
  h.Step(t, 0.01f, 0.f, 0.f);
  t += kFrameNs;
  EXPECT_NEAR(h.r6[1].velocity.x(), 0.01f / (kFrameNs * 1e-9f), 1e-3f);
  // A map reset without a world transform (restart only): R6 keeps the ORIGINAL velocity of that
  // frame, which UpdateHistory computed across the reset (the jump, ~ -1500 u/s). That is deliberate:
  // R6 changes only the span rule, so the reset-crossing velocity, the acceleration and the next
  // frame's prediction are the original values bit for bit.
  h.Step(t, -50.f, 0.f, 0.f, /*restart=*/true, /*world=*/false);
  EXPECT_LT(h.orig[1].velocity.x(), -1000.f);
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(h.r6[1].velocity.data()[i], h.orig[1].velocity.data()[i]);
    EXPECT_EQ(h.r6[1].acceleration.data()[i], h.orig[1].acceleration.data()[i]);
  }
  {
    const FrameRecord next = RecordAt(t + kFrameNs);
    const FrameState po = PredictMotionWithParams(h.orig[1], next, 0.5f, 0.2f);
    const FrameState pr = PredictMotionWithParams(h.r6[1], next, 0.5f, 0.2f);
    for (int i = 0; i < 16; ++i) EXPECT_EQ(pr.pose.data()[i], po.pose.data()[i]);
  }
  t += kFrameNs;
  h.Step(t, -49.99f, 0.f, 0.f);
  t += kFrameNs;
  EXPECT_NEAR(h.r6[1].velocity.x(), 0.01f / (kFrameNs * 1e-9f), 1e-1f);
  // Gap: 0.5 s without a frame. The interval is far above nominal and a longer span is farther
  // still, so the pair itself is used with its real dt (same as the original).
  t += 500000000LL;
  h.Step(t, -49.99f + 0.15f, 0.f, 0.f);
  EXPECT_EQ(h.r6[1].velocity.x(), h.orig[1].velocity.x());
  EXPECT_NEAR(h.r6[1].velocity.x(), 0.15f / 0.5333333f, 2e-2f);
}

TEST(R6SpanVelocity, RotationPathUnchanged) {
  Pair h;
  int64_t t = 0;
  for (int k = 0; k < 8; ++k) {
    h.Step(t, 0.01f * k, 0.f, 0.f);
    t += kFrameNs;
  }
  t = t - kFrameNs + 3300000;  // short pair with a position jitter
  h.Step(t, 0.07f + 0.004f, 0.f, 0.f);
  ASSERT_NE(h.r6[1].velocity.x(), h.orig[1].velocity.x());
  // ApplySpanVelocity only writes velocity / acceleration.
  for (int i = 0; i < 16; ++i) EXPECT_EQ(h.r6[1].pose.data()[i], h.orig[1].pose.data()[i]);
  EXPECT_EQ(h.r6[1].time_ns, h.orig[1].time_ns);
  // MotionPrediction: the rotation depends on the poses and device rotations, not the velocity.
  const c8::Quaternion dev = c8::Quaternion(std::cos(0.2f), 0.f, std::sin(0.2f), 0.f);
  const FrameRecord next = RecordAt(t + kFrameNs, dev);
  c8::Quaternion qo, qr;
  const FrameState po = PredictMotionWithParams(h.orig[1], next, 0.5f, 0.2f, &qo);
  const FrameState pr = PredictMotionWithParams(h.r6[1], next, 0.5f, 0.2f, &qr);
  EXPECT_EQ(qo.w(), qr.w());
  EXPECT_EQ(qo.x(), qr.x());
  EXPECT_EQ(qo.y(), qr.y());
  EXPECT_EQ(qo.z(), qr.z());
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) EXPECT_EQ(po.pose(r, c), pr.pose(r, c)) << r << "," << c;
  }
  EXPECT_NE(po.pose(0, 3), pr.pose(0, 3));  // only the position differs
}

// Seven distinct intervals (ms, oldest -> newest): 31 38 29 35 33 27 11. Sorted 11 27 29 31 33 35
// 38, median (index 3) = 31 ms. Spans from the newest pose: 11 (error 20), 38 (error 7), 71, 106,
// 135, 173 (errors 40, 75, 104, 142); the reference is the second earlier pose (index 2), span 38 ms.
TEST(R6SpanVelocity, MedianReferenceAndVelocityMatchHandComputedValues) {
  const int64_t kMs = 1000000;
  const int64_t intervals_ms[7] = {31, 38, 29, 35, 33, 27, 11};
  const double xs[8] = {1.0, 1.25, 1.5, 2.0, 2.5, 3.0, 3.875, 4.0};  // x of poses 0..7, oldest first
  SpanVelocity span;
  int64_t t = 5 * kMs;
  span.Push(t, static_cast<float>(xs[0]), 0.f, 0.f);
  for (int i = 0; i < 7; ++i) {
    t += intervals_ms[i] * kMs;
    span.Push(t, static_cast<float>(xs[i + 1]), 0.f, 0.f);
  }
  EXPECT_EQ(span.nominal_interval_ns(), 31 * kMs);
  c8::HVector3 v;
  ASSERT_TRUE(span.Estimate(&v));
  // Newest pose 7 (x = 4.0) over pose 5 (x = 3.0) across 27 + 11 = 38 ms (intervals 6 and 7).
  const double expected = (4.0 - 3.0) / 0.038;
  EXPECT_NEAR(v.x(), expected, expected * 4 * 1.2e-7);  // a few float ulp
  EXPECT_EQ(v.y(), 0.0f);
  EXPECT_EQ(v.z(), 0.0f);
}

// Non-increasing time restarts the ring: a later forward jump cannot select a pose of the earlier
// time segment (here the 99 ms pose, 34 ms before the new 133 ms pose, would be the closest span).
TEST(R6SpanVelocity, NonIncreasingTimeRestartsTheRing) {
  const int64_t kMs = 1000000;
  SpanVelocity span;
  for (int k = 0; k < 6; ++k) span.Push(k * 33 * kMs, 100.f + k, 0.f, 0.f);  // 0 .. 165 ms
  ASSERT_EQ(span.nominal_interval_ns(), 33 * kMs);
  span.Push(100 * kMs, 500.f, 0.f, 0.f);  // time went backwards
  EXPECT_EQ(span.pose_count(), 1);
  c8::HVector3 v;
  EXPECT_FALSE(span.Estimate(&v));
  span.Push(133 * kMs, 501.f, 0.f, 0.f);  // forward again
  ASSERT_TRUE(span.Estimate(&v));
  EXPECT_NEAR(v.x(), 1.0 / 0.033, 1e-3);  // reference = the 100 ms pose, not a pre-backjump one
  // The same timestamp twice is also a restart (and no zero interval enters the median).
  span.Push(133 * kMs, 502.f, 0.f, 0.f);
  EXPECT_EQ(span.pose_count(), 1);
  EXPECT_EQ(span.nominal_interval_ns(), 33 * kMs);
}

// ---- through VioTracker: the flag gate ----
struct FakeModules : public TrackerModules {
  c8::HMatrix tam = c8::HMatrixGen::i();
  bool map_reset = false;
  FrameState last_prediction;
  void Detect(const c8::Gr8Pyramid&, const c8::DetectionConfig&, c8::FrameWithPoints*) override {}
  frontend::DetectStats detect_stats() const override { return {}; }
  void TrackMap(TrackerInput*, const FrameState& predicted, MapTrackerOutput* out,
                compat::RandomNumbers*) override {
    last_prediction = predicted;
    out->state = 1;
  }
  void UpdateMap(const TrackerInput&, const MapTrackerOutput&, int64_t, MapBuilderOutput* out) override {
    *out = MapBuilderOutput();
    out->map_reset = map_reset;
  }
  c8::HMatrix ApplyMapBuilderOutput(const MapBuilderOutput&) override { return tam; }
  void RecreateMapTrackerAndBuilder() override {}
  void VisibleMapPoints(std::vector<VisibleMapPoint>* out) const override { out->clear(); }
  int ground_match_count() const override { return 0; }
  int num_keyframes() const override { return 0; }
  std::array<int, 5> num_points_by_source() const override { return {0, 0, 0, 0, 0}; }
};

class ScopedAccuracy {
 public:
  ScopedAccuracy(bool enabled, bool r6) : previous_(config::Accuracy()) {
    AccuracyConfig cfg;
    cfg.enabled = enabled;
    cfg.r6_span_velocity = r6;
    config::SetAccuracy(cfg);
  }
  ~ScopedAccuracy() { config::SetAccuracy(previous_); }

 private:
  AccuracyConfig previous_;
};

// Step of the prediction fed to the MapTracker, frame by frame, for a 0.3 u/s motion whose input
// times are the nominal ones except one 3.3 ms pair with a position jitter at frame 10.
std::vector<float> PredictionSteps(bool enabled, bool r6) {
  ScopedAccuracy scoped(enabled, r6);
  config::RegisterDefaults();
  auto f = std::make_unique<FakeModules>();
  FakeModules* fake = f.get();
  auto tracker = NewVioTrackerWithModules(false, false, std::move(f));
  c8::Gr8Pyramid pyr;
  pyr.levels.push_back(c8::LevelLayout{});
  compat::RandomNumbers rng;
  StatusOut status;
  TrackDebug debug;
  std::vector<float> steps;
  const int64_t t0 = 5000000000LL;
  int64_t t = t0;
  float prev_x = 0.f;
  for (int k = 0; k < 16; ++k) {
    const bool shortened = k == 10;
    t += k == 0 ? 0 : (shortened ? 3300000 : kFrameNs);
    const float x = static_cast<float>(0.3 * (t - t0) * 1e-9) + (shortened ? 0.004f : 0.f);
    fake->tam = PoseAt(x, 0.f, 0.f);
    TrackingSensorFrame frame;
    frame.time_ns = t;
    frame.device_pose = c8::Quaternion(1, 0, 0, 0);
    tracker->Track(frame, pyr, &rng, &status, &debug);
    if (k > 0) steps.push_back(fake->last_prediction.pose(0, 3) - prev_x);
    prev_x = x;
  }
  return steps;
}

TEST(R6SpanVelocity, FlagGateThroughTracker) {
  const std::vector<float> base = PredictionSteps(false, false);
  // r6 without `enabled`, and enabled without r6: the original prediction, bit for bit.
  EXPECT_EQ(PredictionSteps(false, true), base);
  EXPECT_EQ(PredictionSteps(true, false), base);
  // enabled + r6: the frame after the short pair predicts a small step instead of the inflated one.
  const std::vector<float> on = PredictionSteps(true, true);
  ASSERT_EQ(on.size(), base.size());
  const size_t after_short = 10;  // steps[i] is the prediction of frame i + 1: frame 11 follows frame 10
  EXPECT_GT(base[after_short], 0.05f);
  EXPECT_LT(on[after_short], 0.02f);
  for (size_t i = 0; i < 8; ++i) EXPECT_EQ(on[i], base[i]) << i;  // uniform part: identical
}


// One tracker driven frame by frame; the same script runs with R6 off and on.
struct TrackerRun {
  TrackerRun(bool enabled, bool r6) : scoped(enabled, r6) {
    cfg.enabled = enabled;
    cfg.r6_span_velocity = r6;
    config::RegisterDefaults();
    auto f = std::make_unique<FakeModules>();
    fake = f.get();
    tracker = NewVioTrackerWithModules(false, false, std::move(f));
    pyr.levels.push_back(c8::LevelLayout{});
  }
  void Track(int64_t t_ns, float x, bool map_reset = false) {
    config::SetAccuracy(cfg);  // process-global: each run sets its own before every frame
    fake->tam = PoseAt(x, 0.f, 0.f);
    fake->map_reset = map_reset;
    TrackingSensorFrame frame;
    frame.time_ns = t_ns;
    frame.device_pose = c8::Quaternion(1, 0, 0, 0);
    tracker->Track(frame, pyr, &rng, &status, &debug);
  }
  const FrameState& hist() { return VioTrackerAccess::state(*tracker).hist[1]; }
  SpanVelocity& span() { return VioTrackerAccess::state(*tracker).span; }
  ScopedAccuracy scoped;
  AccuracyConfig cfg;
  FakeModules* fake = nullptr;
  std::unique_ptr<VioTracker> tracker;
  c8::Gr8Pyramid pyr;
  compat::RandomNumbers rng;
  StatusOut status;
  TrackDebug debug;
};

void ExpectSameVelocityAndPrediction(TrackerRun& a, TrackerRun& b, const char* what) {
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(a.hist().velocity.data()[i], b.hist().velocity.data()[i]) << what << " v" << i;
    EXPECT_EQ(a.hist().acceleration.data()[i], b.hist().acceleration.data()[i]) << what << " a" << i;
  }
  for (int i = 0; i < 16; ++i) {
    EXPECT_EQ(a.tracker->last_prediction().pose.data()[i], b.tracker->last_prediction().pose.data()[i])
        << what << " pred " << i;
  }
}

// Map reset through the tracker: the reset frame's velocity (computed across the reset by the
// original UpdateHistory) and every later prediction equal the R6-off values bit for bit; the frame
// right after the reset is a 3.3 ms pair, which would pick a pre-reset pose if the ring survived.
TEST(R6SpanVelocity, MapResetKeepsTheOriginalVelocityAndPrediction) {
  TrackerRun off(true, false), on(true, true);
  int64_t t = 5000000000LL;
  float x = 0.f;
  const auto both = [&](int64_t tt, float xx, bool reset) {
    off.Track(tt, xx, reset);
    on.Track(tt, xx, reset);
  };
  for (int k = 0; k < 10; ++k) {
    both(t, x, false);
    t += kFrameNs;
    x += 0.01f;
  }
  both(t, -50.f, true);  // map reset: pose jumps
  ASSERT_LT(on.hist().velocity.x(), -1000.f);  // the reset-crossing original velocity is kept
  ExpectSameVelocityAndPrediction(off, on, "reset frame");
  EXPECT_EQ(on.span().pose_count(), 1);
  t += 3300000;
  both(t, -49.99f, false);
  ExpectSameVelocityAndPrediction(off, on, "3.3 ms after reset");
  t += kFrameNs;
  both(t, -49.98f, false);
  ExpectSameVelocityAndPrediction(off, on, "next prediction");
}

// SetDisableVio: the ring is cut on both transitions and disabled frames are not pushed. After
// switching on again the 3.3 ms pair has no earlier reference, so it equals the R6-off values.
TEST(R6SpanVelocity, DisableVioTransitionCutsTheRing) {
  TrackerRun off(true, false), on(true, true);
  int64_t t = 5000000000LL;
  float x = 0.f;
  const auto both = [&](int64_t tt, float xx) {
    off.Track(tt, xx);
    on.Track(tt, xx);
  };
  for (int k = 0; k < 10; ++k) {
    both(t, x);
    t += kFrameNs;
    x += 0.01f;
  }
  ASSERT_GT(on.span().pose_count(), 5);
  off.tracker->SetDisableVio(true);
  on.tracker->SetDisableVio(true);
  EXPECT_EQ(on.span().pose_count(), 0);
  for (int k = 0; k < 3; ++k) {  // disabled frames: not pushed
    both(t, 0.f);
    t += kFrameNs;
    EXPECT_EQ(on.span().pose_count(), 0);
  }
  off.tracker->SetDisableVio(false);
  on.tracker->SetDisableVio(false);
  EXPECT_EQ(on.span().pose_count(), 0);
  both(t, 5.f);
  EXPECT_EQ(on.span().pose_count(), 1);
  ExpectSameVelocityAndPrediction(off, on, "first frame after enabling");
  t += 3300000;
  both(t, 5.004f);  // short pair: only the one new pose can be the reference
  ExpectSameVelocityAndPrediction(off, on, "short pair after enabling");
  EXPECT_EQ(on.span().pose_count(), 2);
}

}  // namespace
}  // namespace slam_refact::vio_tracker
