// Accuracy-mode modules that do not need the tracking core (contract ACC-2, ACC-3):
// MetricScaleTracker on a synthetic trajectory with a known metric scale, FloorDetector on synthetic
// planes, the hit test geometry, and the faithful default of AccuracyConfig / GlueConfig.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

#include "engine/floor_detector.h"
#include "engine/metric_scale_tracker.h"
#include "gtest/gtest.h"
#include "slam_refact/engine_config.h"

namespace slam_refact {
namespace {

constexpr double kG = 9.80665;

// Camera moving on a smooth 3D path (metres) with a slow yaw; the published trajectory is
// metres / scale. The accelerometer gets the true specific force in the device (= GL camera) frame
// plus white noise and the Chrome 0.1 m/s^2 quantization, delivered `lag_s` late.
struct Synthetic {
  double scale;   // metres per published unit
  double lag_s;   // IMU stamped this much later than the frames
  double noise;   // m/s^2 white noise
  double hf_amp = 0.0;  // m, extra 1.9 Hz (12 rad/s) side-to-side motion (x), seen by the IMU
  double eis = 1.0;     // gain of that 1.9 Hz term in the published track (EIS-like attenuation)
};

constexpr double kHfOmega = 12.0;  // rad/s

std::array<double, 3> Position(double t, double hf = 0.0) {
  return {0.6 * std::sin(1.3 * t) + 0.2 * std::sin(3.1 * t) + hf * std::sin(kHfOmega * t),
          1.4 + 0.15 * std::sin(2.2 * t + 0.4),
          0.5 * std::cos(0.9 * t) + 0.25 * std::sin(2.7 * t)};
}

std::array<double, 3> Acceleration(double t) {
  return {-0.6 * 1.69 * std::sin(1.3 * t) - 0.2 * 9.61 * std::sin(3.1 * t),
          -0.15 * 4.84 * std::sin(2.2 * t + 0.4),
          -0.5 * 0.81 * std::cos(0.9 * t) - 0.25 * 7.29 * std::sin(2.7 * t)};
}

double Yaw(double t) { return 0.3 * std::sin(0.4 * t); }

// world_from_camera rotation about +y (yaw), wxyz.
std::array<double, 4> Rotation(double t) {
  const double h = 0.5 * Yaw(t);
  return {std::cos(h), 0.0, std::sin(h), 0.0};
}

void RunSynthetic(const Synthetic& s, MetricScaleTracker* tracker, double seconds) {
  std::mt19937 rng(7);
  std::normal_distribution<double> n(0.0, s.noise);
  // IMU at 60 Hz, frames at 30 Hz, interleaved in time order.
  double t_imu = 0.0, t_frame = 0.0;
  while (t_frame < seconds) {
    if (t_imu - s.lag_s <= t_frame) {
      const double t = t_imu - s.lag_s;  // physical time of the sample
      std::array<double, 3> a = Acceleration(t);
      a[0] -= s.hf_amp * kHfOmega * kHfOmega * std::sin(kHfOmega * t);
      // Specific force in the world = a + g up; device frame = R^T (yaw about y).
      const double fx = a[0], fy = a[1] + kG, fz = a[2];
      const double c = std::cos(Yaw(t)), sn = std::sin(Yaw(t));
      std::array<double, 3> dev = {c * fx - sn * fz, fy, sn * fx + c * fz};
      for (double& v : dev) v = std::round((v + n(rng)) / 0.1) * 0.1;
      tracker->PushImu(static_cast<int64_t>(t_imu * 1e9), dev);
      t_imu += 1.0 / 60.0;
    } else {
      const std::array<double, 3> p = Position(t_frame, s.eis * s.hf_amp);
      tracker->PushFrame(static_cast<int64_t>(t_frame * 1e9),
                         {p[0] / s.scale, p[1] / s.scale, p[2] / s.scale}, Rotation(t_frame));
      t_frame += 1.0 / 30.0;
    }
  }
}

TEST(MetricScaleTracker, RecoversKnownScale) {
  for (const double scale : {0.25, 0.47, 0.9}) {
    MetricScaleTracker tracker;
    RunSynthetic({scale, 0.0, 0.05}, &tracker, 20.0);
    ASSERT_TRUE(tracker.initialized()) << "scale " << scale;
    EXPECT_NEAR(tracker.scale() / scale, 1.0, 0.05) << "scale " << scale;
  }
}

TEST(MetricScaleTracker, EstimatesImuLag) {
  MetricScaleTracker tracker;
  RunSynthetic({0.47, 0.12, 0.05}, &tracker, 20.0);
  ASSERT_TRUE(tracker.initialized());
  EXPECT_NEAR(tracker.scale() / 0.47, 1.0, 0.05);
  // The IMU lags the frames: the best lag is about +0.12 s of sample time behind frame time.
  EXPECT_NEAR(tracker.lag_s(), 0.12, 0.02);
}

// Low band (0.1-0.8 Hz) option: recovers the scale on the same trajectory, and stays unbiased when
// the published track loses half of a 1.9 Hz motion the IMU still sees (EIS-like attenuation),
// where the default 0.4-2.5 Hz band over-estimates the scale.
TEST(MetricScaleTracker, LowBandRecoversScale) {
  MetricScaleParams p;
  p.low_band = true;
  p.window_s = 8.0;
  for (const double scale : {0.25, 0.47}) {
    MetricScaleTracker tracker(p);
    RunSynthetic({scale, 0.0, 0.05}, &tracker, 40.0);
    ASSERT_TRUE(tracker.initialized()) << "scale " << scale;
    EXPECT_NEAR(tracker.scale() / scale, 1.0, 0.07) << "scale " << scale;
  }
}

TEST(MetricScaleTracker, LowBandUnderEisLikeAttenuation) {
  // 2 cm of 1.9 Hz motion (2.9 m/s^2) in the IMU, half of it in the published track.
  const Synthetic eis{0.47, 0.0, 0.05, 0.02, 0.5};
  MetricScaleTracker mid;
  RunSynthetic(eis, &mid, 40.0);
  MetricScaleParams p;
  p.low_band = true;
  p.window_s = 8.0;
  MetricScaleTracker low(p);
  RunSynthetic(eis, &low, 40.0);
  ASSERT_TRUE(low.initialized());
  const double low_err = std::fabs(std::log(low.scale() / 0.47));
  EXPECT_LT(low_err, 0.10);
  if (mid.initialized()) {
    EXPECT_LT(low_err, std::fabs(std::log(mid.scale() / 0.47)));
  }
}

TEST(MetricScaleTracker, LowBandFixedLag) {
  MetricScaleParams p;
  p.low_band = true;
  p.window_s = 8.0;
  p.fixed_lag_s = 0.1;
  MetricScaleTracker tracker(p);
  RunSynthetic({0.47, 0.1, 0.05}, &tracker, 40.0);
  ASSERT_TRUE(tracker.initialized());
  EXPECT_NEAR(tracker.scale() / 0.47, 1.0, 0.07);
  EXPECT_DOUBLE_EQ(tracker.lag_s(), 0.1);
}

TEST(MetricScaleTracker, DefaultsUnchangedByLowBandFields) {
  const MetricScaleParams p;
  EXPECT_FALSE(p.low_band);
  EXPECT_FALSE(std::isfinite(p.fixed_lag_s));
  const AccuracyConfig a;
  EXPECT_FALSE(a.metric_low_band);
  EXPECT_FALSE(std::isfinite(a.metric_fixed_lag_s));
}

TEST(MetricScaleTracker, MapResetDropsConvergence) {
  MetricScaleTracker tracker;
  RunSynthetic({0.47, 0.0, 0.05}, &tracker, 20.0);
  ASSERT_TRUE(tracker.initialized());
  tracker.Perturb();
  EXPECT_FALSE(tracker.initialized());
  EXPECT_EQ(tracker.accepted_windows(), 0);
  EXPECT_NEAR(tracker.scale() / 0.47, 1.0, 0.05);  // kept as the starting value
}

TEST(MetricScaleTracker, StaysAtPriorWithoutMotion) {
  MetricScaleTracker tracker;
  for (int k = 0; k < 600; ++k) {
    const double t = k / 60.0;
    tracker.PushImu(static_cast<int64_t>(t * 1e9), {0.0, kG, 0.0});
    if (k % 2 == 0) tracker.PushFrame(static_cast<int64_t>(t * 1e9), {0.0, 2.0, 0.0}, {1, 0, 0, 0});
  }
  EXPECT_FALSE(tracker.initialized());
  EXPECT_NEAR(tracker.scale(), MetricScaleParams{}.prior_scale, 1e-9);
}

std::vector<WorldPointOut> PlanePoints(float y, float x0, float x1, float z0, float z1, int n,
                                       uint64_t id0, std::mt19937* rng, float noise) {
  std::uniform_real_distribution<float> ux(x0, x1), uz(z0, z1);
  std::normal_distribution<float> ny(0.f, noise);
  std::vector<WorldPointOut> out;
  for (int k = 0; k < n; ++k) {
    out.push_back({id0 + static_cast<uint64_t>(k), Vec3f{ux(*rng), y + ny(*rng), uz(*rng)}, 1.f});
  }
  return out;
}

TEST(FloorDetector, FindsLowestPlaneAndTable) {
  std::mt19937 rng(3);
  FloorDetectorParams p;
  p.bin = 0.02f;
  FloorDetector det(p);
  std::vector<WorldPointOut> pts = PlanePoints(0.0f, -2.f, 2.f, -3.f, 1.f, 200, 1, &rng, 0.005f);
  const std::vector<WorldPointOut> table = PlanePoints(0.75f, 0.5f, 1.3f, -1.f, -0.4f, 60, 1000, &rng, 0.005f);
  pts.insert(pts.end(), table.begin(), table.end());
  // Clutter: random points at any height.
  std::uniform_real_distribution<float> u(-2.f, 2.f), h(0.05f, 1.2f);
  for (int k = 0; k < 40; ++k) pts.push_back({5000u + static_cast<uint64_t>(k), Vec3f{u(rng), h(rng), u(rng)}, 1.f});
  pts.push_back({0, Vec3f{0.f, 2.f, 0.f}, 0.f});  // 0-padding is ignored
  det.Update(Vec3f{0.f, 1.4f, 0.f}, pts);
  const SurfaceState& s = det.state();
  ASSERT_TRUE(s.floor_detected);
  EXPECT_NEAR(s.floor.height, 0.0f, 0.01f);
  EXPECT_GE(s.floor.inliers, 180);
  EXPECT_NEAR(s.floor.area, 16.0f, 3.0f);
  ASSERT_FALSE(s.others.empty());
  EXPECT_NEAR(s.others[0].height, 0.75f, 0.01f);
}

TEST(FloorDetector, NoFloorWithoutSupport) {
  FloorDetector det;
  std::vector<WorldPointOut> pts;
  for (int k = 0; k < 10; ++k) pts.push_back({1u + static_cast<uint64_t>(k), Vec3f{0.1f * k, 0.f, 0.f}, 1.f});
  det.Update(Vec3f{0.f, 2.f, 0.f}, pts);
  EXPECT_FALSE(det.state().floor_detected);
  EXPECT_TRUE(det.HitTest(Vec3f{0.f, 2.f, 0.f}, Vec3f{0.f, -1.f, 0.f},
                          static_cast<uint32_t>(HitType::kDetectedSurface) |
                              static_cast<uint32_t>(HitType::kEstimatedSurface))
                  .empty());
}

TEST(FloorDetector, HitTestTypesAndGeometry) {
  std::mt19937 rng(5);
  FloorDetectorParams p;
  p.bin = 0.02f;
  FloorDetector det(p);
  det.Update(Vec3f{0.f, 1.5f, 0.f}, PlanePoints(0.f, -1.f, 1.f, -1.f, 1.f, 300, 1, &rng, 0.002f));
  ASSERT_TRUE(det.state().floor_detected);
  const uint32_t all = 7u;
  // Straight down: inside the hull.
  std::vector<HitResult> h = det.HitTest(Vec3f{0.f, 1.5f, 0.f}, Vec3f{0.f, -1.f, 0.f}, all);
  ASSERT_FALSE(h.empty());
  bool detected = false;
  for (const HitResult& r : h) {
    if (r.type == HitType::kDetectedSurface) {
      detected = true;
      EXPECT_NEAR(r.position.y, det.state().floor.height, 1e-5f);
      EXPECT_NEAR(r.distance, 1.5f, 0.01f);
      EXPECT_EQ(r.surface, 0);
    }
  }
  EXPECT_TRUE(detected);
  // Shallow ray far outside the hull: the infinite floor plane (estimated surface).
  h = det.HitTest(Vec3f{0.f, 1.5f, 0.f}, Vec3f{1.f, -0.1f, 0.f},
                  static_cast<uint32_t>(HitType::kEstimatedSurface) |
                      static_cast<uint32_t>(HitType::kDetectedSurface));
  ASSERT_EQ(h.size(), 1u);
  EXPECT_EQ(h[0].type, HitType::kEstimatedSurface);
  EXPECT_NEAR(h[0].position.x, 15.f, 0.2f);
  // Upwards: nothing.
  EXPECT_TRUE(det.HitTest(Vec3f{0.f, 1.5f, 0.f}, Vec3f{0.f, 1.f, 0.f}, all).empty());
}

// Exact grid of n x n points at height y (ids from id0): equal-support surfaces without sampling noise.
std::vector<WorldPointOut> GridPlane(float y, int n, float step, uint64_t id0) {
  std::vector<WorldPointOut> out;
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      out.push_back({id0 + static_cast<uint64_t>(i * n + j),
                     Vec3f{(static_cast<float>(i) - 0.5f * (n - 1)) * step, y,
                           (static_cast<float>(j) - 0.5f * (n - 1)) * step},
                     1.f});
    }
  }
  return out;
}

// Surfaces with equal inlier counts must be ordered by a total order (inliers desc, then height asc),
// not by whatever the standard library's std::sort does with ties.
// Scope: with 3 surfaces std::sort is an insertion sort and keeps the (already height ordered) input
// order of ties even with an inliers-only comparator, so this case checks the ordering contract but
// cannot tell the old comparator apart. OtherSurfacesTieBreakWithManyTiedSurfaces below does (it
// needs more tied elements than the library's insertion-sort threshold).
TEST(FloorDetector, OtherSurfacesTieBreakByHeight) {
  FloorDetectorParams p;
  p.bin = 0.02f;
  const std::vector<WorldPointOut> floor = GridPlane(0.0f, 9, 0.3f, 1);
  const std::vector<WorldPointOut> low = GridPlane(0.6f, 6, 0.3f, 1000);
  const std::vector<WorldPointOut> mid = GridPlane(1.0f, 6, 0.3f, 2000);
  const std::vector<WorldPointOut> high = GridPlane(1.4f, 6, 0.3f, 3000);
  for (int order = 0; order < 2; ++order) {
    FloorDetector det(p);
    std::vector<WorldPointOut> pts = floor;
    const std::vector<const std::vector<WorldPointOut>*> seq =
        order == 0 ? std::vector<const std::vector<WorldPointOut>*>{&low, &mid, &high}
                   : std::vector<const std::vector<WorldPointOut>*>{&high, &mid, &low};
    for (const auto* t : seq) pts.insert(pts.end(), t->begin(), t->end());
    det.Update(Vec3f{0.f, 2.2f, 0.f}, pts);
    const SurfaceState& s = det.state();
    ASSERT_TRUE(s.floor_detected);
    ASSERT_EQ(s.others.size(), 3u);
    for (const HorizontalSurface& o : s.others) EXPECT_EQ(o.inliers, 36);
    EXPECT_NEAR(s.others[0].height, 0.6f, 0.01f);
    EXPECT_NEAR(s.others[1].height, 1.0f, 0.01f);
    EXPECT_NEAR(s.others[2].height, 1.4f, 0.01f);
  }
  // More support still wins over height.
  FloorDetector det(p);
  std::vector<WorldPointOut> pts = floor;
  const std::vector<WorldPointOut> big = GridPlane(1.4f, 8, 0.3f, 4000);
  for (const auto* t : {&low, &big}) pts.insert(pts.end(), t->begin(), t->end());
  det.Update(Vec3f{0.f, 2.2f, 0.f}, pts);
  ASSERT_EQ(det.state().others.size(), 2u);
  EXPECT_EQ(det.state().others[0].inliers, 64);
  EXPECT_NEAR(det.state().others[0].height, 1.4f, 0.01f);
}

// Hits at exactly the same distance are ordered by hit type, then surface, not by std::sort's tie order.
TEST(FloorDetector, HitTestEqualDistanceOrder) {
  FloorDetectorParams p;
  p.bin = 0.02f;
  FloorDetector det(p);
  // 7 x 7 grid, 0.5 apart: the point at the origin is the only one within 0.06 rad of the straight-down
  // ray from (0, 2, 0), so the FEATURE_POINT depth is exactly the floor plane distance (2).
  det.Update(Vec3f{0.f, 2.f, 0.f}, GridPlane(0.0f, 7, 0.5f, 1));
  ASSERT_TRUE(det.state().floor_detected);
  const std::vector<HitResult> h = det.HitTest(Vec3f{0.f, 2.f, 0.f}, Vec3f{0.f, -1.f, 0.f}, 7u);
  ASSERT_EQ(h.size(), 2u);
  EXPECT_EQ(h[0].distance, h[1].distance);
  EXPECT_EQ(h[0].type, HitType::kFeaturePoint);
  EXPECT_EQ(h[1].type, HitType::kDetectedSurface);
}

// Many surfaces with tied support, points inserted in a shuffled order: `others` is exactly
// (inliers desc, height asc). 20 surfaces (> the 16-element insertion-sort threshold of libstdc++, and
// many equal keys) make std::sort partition, so an inliers-only comparator reorders the ties.
TEST(FloorDetector, OtherSurfacesTieBreakWithManyTiedSurfaces) {
  FloorDetectorParams p;
  p.bin = 0.02f;
  constexpr int kSurfaces = 20;
  struct Plane {
    float y;
    int n;      // n x n grid
    int count;  // n * n
  };
  std::vector<Plane> planes;
  std::vector<WorldPointOut> pts = GridPlane(0.0f, 9, 0.3f, 1);  // floor, 81 points
  for (int k = 0; k < kSurfaces; ++k) {
    const int n = (k % 5 == 2) ? 6 : (k % 7 == 3) ? 4 : 5;  // 36, 16 or 25 points
    const Plane pl{0.3f + 0.1f * static_cast<float>(k), n, n * n};
    planes.push_back(pl);
    const std::vector<WorldPointOut> g = GridPlane(pl.y, n, 0.3f, 1000u * static_cast<uint64_t>(k + 1));
    pts.insert(pts.end(), g.begin(), g.end());
  }
  // Expected order, built independently of the detector: support descending, height ascending.
  std::vector<int> expect(kSurfaces);
  for (int k = 0; k < kSurfaces; ++k) expect[static_cast<size_t>(k)] = k;
  std::stable_sort(expect.begin(), expect.end(), [&](int a, int b) {
    return planes[static_cast<size_t>(a)].count > planes[static_cast<size_t>(b)].count;
  });  // stable over the height-ascending index order = (count desc, height asc)
  std::mt19937 rng(23);
  for (int order = 0; order < 3; ++order) {
    std::vector<WorldPointOut> in = pts;
    if (order == 1) std::reverse(in.begin(), in.end());
    if (order == 2) std::shuffle(in.begin(), in.end(), rng);
    FloorDetector det(p);
    det.Update(Vec3f{0.f, 3.5f, 0.f}, in);
    const SurfaceState& s = det.state();
    ASSERT_TRUE(s.floor_detected) << "order " << order;
    ASSERT_EQ(s.others.size(), static_cast<size_t>(kSurfaces)) << "order " << order;
    for (size_t i = 0; i < expect.size(); ++i) {
      const Plane& want = planes[static_cast<size_t>(expect[i])];
      ASSERT_EQ(s.others[i].inliers, want.count) << "order " << order << " rank " << i;
      EXPECT_NEAR(s.others[i].height, want.y, 0.01f) << "order " << order << " rank " << i;
    }
  }
}

// The FEATURE_POINT depth is a Gaussian-weighted average accumulated in double over the cloud. The
// public contract is that it does not depend on the insertion order of the points (nor on the
// container's hash layout). With ordinary data the double accumulation error (~1e-16) disappears when
// the result is rounded to float, so this case checks the geometry/ordering contract only; the next
// test uses data that makes the accumulation order visible.
TEST(FloorDetector, FeaturePointDepthIndependentOfInsertionOrder) {
  std::mt19937 rng(11);
  std::uniform_real_distribution<float> u(-0.05f, 0.05f), d(0.f, 0.3f);
  std::vector<WorldPointOut> pts = GridPlane(0.0f, 9, 0.3f, 1);
  for (int k = 0; k < 200; ++k) {
    pts.push_back({7000u + static_cast<uint64_t>(k) * 7919u, Vec3f{u(rng), -d(rng) + 0.2f, u(rng)}, 1.f});
  }
  std::vector<WorldPointOut> shuffled = pts;
  std::shuffle(shuffled.begin(), shuffled.end(), rng);
  FloorDetector a, b;
  a.Update(Vec3f{0.f, 2.f, 0.f}, pts);
  b.Update(Vec3f{0.f, 2.f, 0.f}, shuffled);
  const Vec3f o{0.01f, 2.f, 0.02f}, dir{0.f, -1.f, 0.f};
  const std::vector<HitResult> ha = a.HitTest(o, dir, 1u);
  const std::vector<HitResult> hb = b.HitTest(o, dir, 1u);
  ASSERT_EQ(ha.size(), 1u);
  ASSERT_EQ(hb.size(), 1u);
  EXPECT_EQ(ha[0].distance, hb[0].distance);
}

// Order-sensitive accumulation: eight points exactly on the ray (weight exactly 1), one with a depth of
// 2^60-class magnitude, six of 128 (half a double ulp of the big one) and one of 2^36. Accumulating in
// the order big, smalls, X loses the smalls to round-to-even; the reverse order keeps them, and the
// two results differ after the cast to float (checked below as a precondition, with a plain double
// loop, so the test cannot pass vacuously). The detector must accumulate in ascending id order whatever
// the order the points were inserted in: an unordered container (insertion/hash dependent) gives the
// other result for at least one of the insertion orders.
TEST(FloorDetector, FeaturePointDepthAccumulatesInIdOrder) {
  const float kBig = 1.2537689859843686e18f;
  const float kMid = 68719476736.0f;  // 2^36
  // ids ascending == accumulation order of the contract.
  const std::vector<float> depth_by_id = {kBig, 128.f, 128.f, 128.f, 128.f, 128.f, 128.f, kMid};
  double asc = 0.0, desc = 0.0;
  for (float v : depth_by_id) asc += static_cast<double>(v);
  for (size_t i = depth_by_id.size(); i-- > 0;) desc += static_cast<double>(depth_by_id[i]);
  const float want = static_cast<float>(asc / 8.0);
  ASSERT_NE(want, static_cast<float>(desc / 8.0)) << "data no longer sensitive to the accumulation order";

  std::vector<WorldPointOut> pts;
  for (size_t i = 0; i < depth_by_id.size(); ++i) {
    pts.push_back({100u + static_cast<uint64_t>(i) * 37u, Vec3f{0.f, -depth_by_id[i], 0.f}, 1.f});
  }
  const Vec3f origin{0.f, 0.f, 0.f}, dir{0.f, -1.f, 0.f};
  std::mt19937 rng(5);
  for (int order = 0; order < 6; ++order) {
    std::vector<WorldPointOut> in = pts;
    if (order == 1) std::reverse(in.begin(), in.end());
    if (order >= 2) std::shuffle(in.begin(), in.end(), rng);
    FloorDetector det;
    det.Update(Vec3f{0.f, 0.f, 0.f}, in);
    const std::vector<HitResult> h = det.HitTest(origin, dir, static_cast<uint32_t>(HitType::kFeaturePoint));
    ASSERT_EQ(h.size(), 1u) << "order " << order;
    EXPECT_EQ(h[0].distance, want) << "insertion order " << order;
  }
}

TEST(AccuracyConfig, DefaultsAreTheOriginal) {
  const EngineConfig e;
  EXPECT_FALSE(e.accuracy.enabled);
  EXPECT_FALSE(e.accuracy.metric_output);
  EXPECT_FALSE(e.accuracy.local_ba);
  EXPECT_FALSE(e.accuracy.multiview_triangulation);
  EXPECT_EQ(e.accuracy.ba_ground_points, 0);
  const GlueConfig g;
  EXPECT_FALSE(g.detect_surfaces);
}

}  // namespace
}  // namespace slam_refact
