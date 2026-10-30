// Bundle code tests (US-009, architecture.md §7.3 map_builder: "iterative BA · bundleAdjustSchur
// convergence (synthetic), residuals = public source on fixed inputs"). Both BA paths are off in
// the default configuration; these tests exercise them directly.

#include <array>
#include <cmath>
#include <cstdio>
#include <vector>

#include "c8/hmatrix.h"
#include "c8/hpoint.h"
#include "ceres/rotation.h"
#include "gtest/gtest.h"
#include "config/params.h"
#include "map/world_map.h"
#include "map_builder/map_builder.h"
#include "map_builder_test_util.h"
#include "reality/engine/geometry/bundle-residual-analytic.h"
#include "reality/engine/geometry/bundle-residual.h"
#include "reality/engine/geometry/bundle.h"
#include "reality/engine/geometry/observed-point.h"

namespace slam_refact::map_builder {
namespace {

using testing::MakeFrame;
using testing::Pose;
using testing::Project;
using testing::RayFeature;

// Public TriangulatePointMultiviewAnalytic on fixed inputs: residual = projection formula of the
// public header comment, Jacobian = central finite differences of that residual.
TEST(MapBuilderBundle, TriangulateResidualEqualsProjectionAndJacobianIsDerivative) {
  const std::array<double, 2> ray = {0.12, -0.07};
  const std::array<double, 12> proj = {0.98, -0.05, 0.19, 0.3,  0.04, 0.99, 0.08, -0.2,
                                       -0.19, -0.07, 0.98, 0.1};
  const std::array<double, 3> x = {0.4, -0.3, 3.2};
  const c8::TriangulatePointMultiviewAnalytic f(ray, proj);
  auto eval = [&](const std::array<double, 3>& p, double* r, double* j) {
    const double* params[1] = {p.data()};
    double* jacs[1] = {j};
    return f.Evaluate(params, r, j != nullptr ? jacs : nullptr);
  };
  double r[2];
  double jac[6];
  ASSERT_TRUE(eval(x, r, jac));
  const double w = proj[8] * x[0] + proj[9] * x[1] + proj[10] * x[2] + proj[11];
  const double u = (proj[0] * x[0] + proj[1] * x[1] + proj[2] * x[2] + proj[3]) / w;
  const double v = (proj[4] * x[0] + proj[5] * x[1] + proj[6] * x[2] + proj[7]) / w;
  EXPECT_NEAR(r[0], u - ray[0], 1e-14);
  EXPECT_NEAR(r[1], v - ray[1], 1e-14);
  for (int k = 0; k < 3; ++k) {
    std::array<double, 3> hi = x;
    std::array<double, 3> lo = x;
    hi[static_cast<size_t>(k)] += 1e-6;
    lo[static_cast<size_t>(k)] -= 1e-6;
    double rh[2];
    double rl[2];
    eval(hi, rh, nullptr);
    eval(lo, rl, nullptr);
    EXPECT_NEAR(jac[k], (rh[0] - rl[0]) / 2e-6, 1e-8);
    EXPECT_NEAR(jac[3 + k], (rh[1] - rl[1]) / 2e-6, 1e-8);
  }
}

// Public SnavelyReprojectionResidual (bundleAdjustSchur) on fixed inputs vs the same model
// written out: R(angle-axis) * X + t, scaled ray difference.
TEST(MapBuilderBundle, SnavelyResidualOnFixedInputs) {
  c8::ObservedPoint obs{c8::HPoint2(0.11f, -0.05f), 1, 0.0f, 1.0f};
  const c8::SnavelyReprojectionResidual res(obs);
  const double camera[6] = {0.02, -0.1, 0.05, 0.3, -0.2, 0.4};
  const double point[3] = {0.5, 0.2, 3.0};
  double r[2];
  ASSERT_TRUE(res(camera, point, r));
  double rot[9];
  ceres::AngleAxisToRotationMatrix(camera, rot);  // column-major
  double p[3];
  for (int i = 0; i < 3; ++i) {
    p[i] = rot[i] * point[0] + rot[3 + i] * point[1] + rot[6 + i] * point[2] + camera[3 + i];
  }
  EXPECT_NEAR(r[0], 500.0 * p[0] / p[2] - static_cast<double>(0.11f * 500.0f), 1e-9);
  EXPECT_NEAR(r[1], 500.0 * p[1] / p[2] - static_cast<double>(-0.05f * 500.0f), 1e-9);
}

struct Scene {
  std::vector<c8::HMatrix> cameras;
  std::vector<c8::HPoint3> points;
};

Scene MakeScene() {
  Scene s;
  s.cameras = {Pose(0.0f, 0.0f, 0.0f, 0.0f), Pose(-0.1f, 0.4f, 0.05f, 0.0f),
               Pose(0.08f, -0.35f, -0.03f, 0.1f)};
  for (int i = 0; i < 40; ++i) {
    s.points.emplace_back(-1.0f + 0.05f * static_cast<float>(i),
                          -0.5f + 0.025f * static_cast<float>((i * 13) % 40),
                          2.5f + 0.05f * static_cast<float>((i * 7) % 40));
  }
  return s;
}

float MaxPointError(const std::vector<c8::HPoint3>& a, const std::vector<c8::HPoint3>& b) {
  float e = 0.0f;
  for (size_t i = 0; i < a.size(); ++i) {
    const float dx = a[i].x() - b[i].x();
    const float dy = a[i].y() - b[i].y();
    const float dz = a[i].z() - b[i].z();
    e = std::fmax(e, std::sqrt(dx * dx + dy * dy + dz * dz));
  }
  return e;
}

TEST(MapBuilderBundle, BundleAdjustSchurConvergesOnSyntheticScene) {
  const Scene scene = MakeScene();
  c8::Vector<c8::ObservedPoint> obs;
  c8::Vector<int> obs_point;
  c8::Vector<int> obs_camera;
  for (size_t c = 0; c < scene.cameras.size(); ++c) {
    for (size_t i = 0; i < scene.points.size(); ++i) {
      obs.push_back({Project(scene.cameras[c], scene.points[i]), 0, 0.0f, 1.0f});
      obs_point.push_back(static_cast<int>(i));
      obs_camera.push_back(static_cast<int>(c));
    }
  }
  c8::Vector<c8::HPoint3> points;
  for (size_t i = 0; i < scene.points.size(); ++i) {
    const float n = 0.08f * static_cast<float>(static_cast<int>(i % 5) - 2);
    points.emplace_back(scene.points[i].x() + n, scene.points[i].y() - n, scene.points[i].z() + n);
  }
  c8::Vector<c8::HMatrix> cameras(scene.cameras.begin(), scene.cameras.end());
  const c8::Vector<int> fixed = {1, 1, 1};
  const float before = MaxPointError(points, scene.points);
  float residual = -1.0f;
  const bool ok = c8::bundleAdjustSchur(obs, obs_point, &points, false, obs_camera, &cameras,
                                        fixed, 0.0f, &residual);
  const float after = MaxPointError(points, scene.points);
  std::printf("[bundleAdjustSchur] ok=%d max point error %.4g -> %.4g, residual %.4g\n", ok,
              before, after, residual);
  EXPECT_TRUE(ok);
  EXPECT_GT(before, 0.1f);
  EXPECT_LT(after, 1e-3f);
}

// IterativeBundleAdjustment (936 [H]), point step: triangulated points displaced by up to 0.12
// (zero mean) are moved back onto their exact observations. The camera step is switched off with
// BundleAdjustment.minNumPointsForKeyframeToAdjust so the cameras stay at the ground truth.
TEST(MapBuilderBundle, IterativeBundleAdjustmentPointStepConvergesOnSyntheticMap) {
  const Scene scene = MakeScene();
  map::WorldMaps maps;
  map::WorldMap& wm = maps.active();
  std::vector<map::KeyframeId> kfs;
  for (size_t c = 0; c < scene.cameras.size(); ++c) {
    std::vector<RayFeature> rays;
    for (size_t i = 0; i < scene.points.size(); ++i) {
      const c8::HPoint2 r = Project(scene.cameras[c], scene.points[i]);
      rays.push_back({r.x(), r.y(), 1, static_cast<uint32_t>(i + 1)});
    }
    kfs.push_back(wm.AddKeyframe(MakeFrame(rays), static_cast<int64_t>(5 * c)));
    wm.SetKeyframePose(kfs.back(), scene.cameras[c]);
  }
  std::vector<map::PointId> ids;
  for (size_t i = 0; i < scene.points.size(); ++i) {
    const map::PointId id = wm.AddPoint(0, scene.points[i], map::PointSource::kTriangulated);
    for (const map::KeyframeId kf : kfs) {
      wm.AddObservation(id, static_cast<uint32_t>(i), kf);
    }
    ids.push_back(id);
  }
  std::vector<c8::HPoint3> displaced;
  for (size_t i = 0; i < ids.size(); ++i) {
    const float n = 0.06f * static_cast<float>(static_cast<int>(i % 5) - 2);
    const float m = 0.06f * static_cast<float>(static_cast<int>(i % 3) - 1);
    map::WorldPoint& p = wm.MutablePoint(ids[i]);
    p.position = c8::HPoint3(scene.points[i].x() + n, scene.points[i].y() + m,
                             scene.points[i].z() - n);
    displaced.push_back(p.position);
  }
  const float before = MaxPointError(displaced, scene.points);
  {
    const testing::ScopedParamOverrides overrides(
        {{"BundleAdjustment.minNumPointsForKeyframeToAdjust", ParamValue(100000)}});
    IterativeBundleAdjustment(wm);
  }
  std::vector<c8::HPoint3> after_pts;
  for (const map::PointId id : ids) {
    after_pts.push_back(wm.Point(id).position);
  }
  const float after = MaxPointError(after_pts, scene.points);
  std::printf("[iterative BA] max point error %.4g -> %.4g\n", before, after);
  EXPECT_GT(before, 0.1f);
  EXPECT_LT(after, 1e-2f);
}

}  // namespace
}  // namespace slam_refact::map_builder
