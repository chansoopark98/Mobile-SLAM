// Global iterative bundle adjustment of 936 (S3 §3.5, 936.c:3291-4157). Off by default
// (MapBuilder.bundleAdjustmentIterativeEnabled = 0); kept for option parity (US-009).
//
// cameras: every keyframe except the newest re-estimates its pose with the public
//   poseEstimationAnalytical (1146) from its TRIANGULATED/CLOUD, non-retired points
//   (ObservedPoint{ray, point min_scale, 0, 1}, 936.c:3330-3420) when there are at least
//   BundleAdjustment.minNumPointsForKeyframeToAdjust; the pose is applied (655) only when the
//   camera moved <= 0.02 (936.c:3526-3534).
// points: unless useFastPointAdjustmentInGlobalIterative (then only 1127), every
//   TRIANGULATED/CLOUD non-retired point (1147 order) is re-triangulated with the public
//   TriangulatePointMultiviewAnalytic residual of each valid observation (ray, rows 0..2 of
//   camera_from_world as doubles, 936.c:3849-3866) under CauchyLoss(0.001f) (936.c:3895-3898);
//   the result is applied (1150 + dirty set) when usable and 0.001 < |dX|^2 < 5 (936.c:4062-4085).
// Ceres options are S3 §3.5 INFERRED values (S3 U3): LM, DENSE_NORMAL_CHOLESKY, 5 iterations,
// 0.2 s, function/gradient tolerance 1e-10, parameter tolerance 1e-6, one thread.

#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include "ceres/ceres.h"
#include "c8/hmatrix.h"
#include "c8/hpoint.h"
#include "common/check.h"
#include "common/stage_timer.h"
#include "config/settings.h"
#include "map/world_map.h"
#include "map/world_map_internal.h"
#include "map_builder/map_builder.h"
#include "reality/engine/geometry/bundle-residual-analytic.h"
#include "reality/engine/geometry/bundle.h"
#include "reality/engine/geometry/observed-point.h"

namespace slam_refact::map_builder {

namespace {

bool AdjustableSource(map::PointSource s) {
  return s == map::PointSource::kTriangulated || s == map::PointSource::kCloud;
}

// Pool lookup by id without abort (634 semantics: any pool slot with this id).
const map::WorldPoint* FindPoint(const map::WorldMap& wm, map::PointId id) {
  if (id.id() == 0) {
    return nullptr;
  }
  for (const map::PointPool& pool : map::WorldMapAccess::impl(wm).pools) {
    if (id.index() < pool.points.size() && pool.points[id.index()].id == id.id()) {
      return &pool.points[id.index()];
    }
  }
  return nullptr;
}

void AdjustCameras(map::WorldMap& wm) {
  StageTimer timer("global-iterative-bundle-adjustment-cameras");
  const config::BundleAdjustmentSettings& ba = config::BundleAdjustmentSettings::Get();
  const uint32_t newest_id = wm.newest().id();
  const std::vector<map::Keyframe>& slots = map::WorldMapAccess::impl(wm).keyframes;
  for (size_t slot = 0; slot < slots.size(); ++slot) {
    const map::Keyframe& kf = slots[slot];
    if (kf.id == 0 || kf.id == newest_id) {
      continue;
    }
    c8::Vector<c8::ObservedPoint> rays;
    c8::Vector<c8::HPoint3> points;
    for (const c8::FramePoint& fp : kf.frame.points()) {
      const map::WorldPoint* p = FindPoint(wm, fp.id());
      if (p == nullptr || !AdjustableSource(p->source) || p->state == map::PointState::kDead) {
        continue;
      }
      rays.push_back({c8::HPoint2(fp.x(), fp.y()), static_cast<int>(p->min_scale), 0.0f, 1.0f});
      points.push_back(p->position);
    }
    if (rays.size() < static_cast<uint32_t>(ba.min_num_points_for_keyframe_to_adjust)) {
      continue;
    }
    c8::HMatrix pose = kf.frame.extrinsic();
    c8::PoseEstimationAnalyticalParams params;  // {true, 1.0, false, 0.0, false} (936.c:3505-3522)
    params.suppressOutliers = true;
    params.scale = 1.0;
    params.useImageTargetCostFunction = false;
    params.cameraMotionWeight = 0.0;
    params.logReport = false;
    const bool ok = c8::poseEstimationAnalytical(rays, points, {}, &pose, params);  // 1146
    const auto& before = kf.frame.extrinsic().data();
    const auto& after = pose.data();
    const float dz = before[14] - after[14];
    const float dy = before[13] - after[13];
    const float dx = before[12] - after[12];
    if (ok && std::sqrt(dz * dz + dy * dy + dx * dx + 0.0f) <= 0.02f) {
      wm.SetKeyframePose(map::KeyframeId(slot, kf.id), pose);  // 655
    }
  }
}

void AdjustPoints(map::WorldMap& wm) {
  StageTimer timer("global-iterative-bundle-adjustment-points");
  if (config::BundleAdjustmentSettings::Get().use_fast_point_adjustment_in_global_iterative) {
    wm.AdjustDirtyPoints();
    return;
  }
  const std::vector<map::Keyframe>& slots = map::WorldMapAccess::impl(wm).keyframes;
  for (map::PointId id : wm.AllPointIds()) {  // 1147
    const map::WorldPoint* p = FindPoint(wm, id);
    SR_CHECK(p != nullptr, "Invalid point requested");
    if (!AdjustableSource(p->source) || p->state == map::PointState::kDead) {
      continue;
    }
    const auto& pos = p->position.raw();
    std::array<double, 3> x = {static_cast<double>(pos[0] / pos[3]),
                               static_cast<double>(pos[1] / pos[3]),
                               static_cast<double>(pos[2] / pos[3])};
    ceres::Problem problem;
    int residuals = 0;
    for (const auto& [kf_id, obs_idx] : p->observations) {
      if (!(kf_id.id() != 0 && kf_id.index() < slots.size() &&
            slots[kf_id.index()].id == kf_id.id())) {
        continue;
      }
      const map::Keyframe& kf = slots[kf_id.index()];
      const c8::HMatrix camera_from_world = kf.frame.extrinsic().inv();
      const auto& inv = camera_from_world.data();
      const std::array<double, 12> projection = {
          inv[0], inv[4], inv[8], inv[12], inv[1], inv[5], inv[9], inv[13],
          inv[2], inv[6], inv[10], inv[14]};
      const c8::FramePoint& fp = kf.frame.points()[obs_idx];
      const std::array<double, 2> ray = {static_cast<double>(fp.x() / 1.0f),
                                         static_cast<double>(fp.y() / 1.0f)};
      problem.AddResidualBlock(new c8::TriangulatePointMultiviewAnalytic(ray, projection),
                               new ceres::CauchyLoss(static_cast<double>(0.001f)), x.data());
      ++residuals;
    }
    if (residuals == 0) {
      continue;
    }
    ceres::Solver::Options options;
    options.minimizer_type = ceres::TRUST_REGION;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options.max_num_iterations = 5;
    options.max_solver_time_in_seconds = 0.2;
    options.function_tolerance = 1e-10;
    options.gradient_tolerance = 1e-10;
    options.parameter_tolerance = 1e-6;
    options.num_threads = 1;
    options.minimizer_progress_to_stdout = false;
    options.logging_type = ceres::SILENT;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    const float nx = static_cast<float>(x[0]);
    const float ny = static_cast<float>(x[1]);
    const float nz = static_cast<float>(x[2]);
    const float ex = pos[0] / pos[3] - nx;
    const float ey = pos[1] / pos[3] - ny;
    const float ez = pos[2] / pos[3] - nz;
    const float d2 = ex * ex + ey * ey + ez * ez;
    if (summary.IsSolutionUsable() && 0.001f < d2 && d2 < 5.0f) {
      map::SetPointPosition(wm, id, c8::HPoint3(nx, ny, nz));  // 1150
      map::WorldMapAccess::impl(wm).dirty.insert(id);
    }
  }
}

}  // namespace

void IterativeBundleAdjustment(map::WorldMap& wm) {
  const int iterations = config::BundleAdjustmentSettings::Get().num_global_iterative_iterations;
  for (int i = 0; i < iterations; ++i) {
    AdjustCameras(wm);
    AdjustPoints(wm);
  }
}

}  // namespace slam_refact::map_builder
