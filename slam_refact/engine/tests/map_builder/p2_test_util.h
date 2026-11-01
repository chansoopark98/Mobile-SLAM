// Helpers of the P2 candidate tests (F1 / F2, build/xr-drift-p2-20261031/f2-contract-v3.3.md section 7):
// synthetic exact-pinhole scenes without noise, a scoped accuracy configuration and an independent
// (test-side) geometry computation used to check the evaluator's reported quantities.

#ifndef SLAM_REFACT_TESTS_MAP_BUILDER_P2_TEST_UTIL_H_
#define SLAM_REFACT_TESTS_MAP_BUILDER_P2_TEST_UTIL_H_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

#include "Eigen/Core"
#include "Eigen/LU"
#include "c8/hmatrix.h"
#include "c8/hpoint.h"
#include "config/accuracy.h"
#include "map/f2_confirm.h"
#include "map/world_map.h"
#include "map_builder_test_util.h"
#include "slam_refact/accuracy_counters.h"
#include "slam_refact/engine_config.h"

namespace slam_refact::p2 {

using V3 = Eigen::Vector3d;
using M3 = Eigen::Matrix3d;

// Installs an AccuracyConfig (enabled, F1/F2 as asked) for one scope, zeroes the counters, and
// restores the previous configuration.
class ScopedAccuracy {
 public:
  ScopedAccuracy(bool f1, bool f2) : previous_(config::Accuracy()) {
    AccuracyConfig cfg;
    cfg.enabled = true;
    cfg.f1_ground_plane = f1;
    cfg.f2_ground_confirmation = f2;
    config::SetAccuracy(cfg);
    ResetCounters();
  }
  ~ScopedAccuracy() { config::SetAccuracy(previous_); }
  ScopedAccuracy(const ScopedAccuracy&) = delete;
  ScopedAccuracy& operator=(const ScopedAccuracy&) = delete;

 private:
  AccuracyConfig previous_;
};

inline const double kPi = 3.14159265358979323846;
inline double Deg(double d) { return d * kPi / 180.0; }

// ---- pure-evaluator scenes (double precision) ---------------------------------------------------

// View of the point X from a camera at `center` with rotation `r_wc` (exact, noise free).
inline map::F2View ViewTo(const V3& x_world, const V3& center, const M3& r_wc, uint32_t kf_id) {
  map::F2View v;
  v.kf_id = kf_id;
  v.center = center;
  v.r_wc = r_wc;
  const V3 c = r_wc.transpose() * (x_world - center);
  v.x = c.x() / c.z();
  v.y = c.y() / c.z();
  v.obs_idx = 0;
  v.kf_serial = kf_id;
  v.pose_rev = 1;
  return v;
}

constexpr uint32_t kCreator = 10;

// Three views of one point. The creator sits at the origin (identity rotation), its ray is
// (0, -1/3, 1) and the plane is y = -scale (so d_plane = 3 * scale... see below): with
// scale = d_plane / 3 the creator is `scale` above the plane and d_plane == d_plane exactly.
// The point lies on the creator's ray at z-depth d_plane * ratio, so d_mv == d_plane * ratio.
// The other two cameras are at (-1.2, 0, 0) and (1, 0.1, -0.3) times scale.
inline map::F2EvalInput MakeInput(double ratio = 1.0, double d_plane = 3.0) {
  const double s = d_plane / 3.0;
  map::F2EvalInput in;
  in.creator_kf_id = kCreator;
  in.creator_alive = true;
  in.plane.n = V3(0.0, 1.0, 0.0);
  in.plane.offset = -s;
  const V3 x = d_plane * ratio * V3(0.0, -1.0 / 3.0, 1.0);
  const M3 id = M3::Identity();
  in.views.push_back(ViewTo(x, V3::Zero(), id, kCreator));
  in.views.push_back(ViewTo(x, s * V3(-1.2, 0.0, 0.0), id, kCreator + 1));
  in.views.push_back(ViewTo(x, s * V3(1.0, 0.1, -0.3), id, kCreator + 2));
  return in;
}

// Test-side geometry, written independently of F2Evaluate (plain inverse and atan2).
struct Indep {
  V3 x = V3::Zero();
  double theta_max = 0.0;
  double rho_max = 0.0;
  double d_mv = 0.0;
  double d_plane = 0.0;
  bool all_in_front = true;
  bool plane_ok = true;
};
inline Indep IndepGeometry(const map::F2EvalInput& in) {
  Indep r;
  std::vector<V3> c;
  std::vector<V3> d;
  for (const map::F2View& v : in.views) {
    c.push_back(v.center);
    const V3 w = v.r_wc * V3(v.x, v.y, 1.0);
    d.push_back(w / w.norm());
  }
  M3 a = M3::Zero();
  V3 b = V3::Zero();
  for (size_t i = 0; i < d.size(); ++i) {
    const M3 p = M3::Identity() - d[i] * d[i].transpose();
    a += p;
    b += p * c[i];
  }
  r.x = a.inverse() * b;
  auto angle = [](const V3& u, const V3& w) { return std::atan2(u.cross(w).norm(), u.dot(w)); };
  for (size_t i = 0; i < d.size(); ++i) {
    for (size_t j = i + 1; j < d.size(); ++j) {
      r.theta_max = std::max(r.theta_max, angle(d[i], d[j]));
    }
    r.rho_max = std::max(r.rho_max, angle(d[i], (r.x - c[i])));
    const V3 cam = in.views[i].r_wc.transpose() * (r.x - c[i]);
    if (!((r.x - c[i]).dot(d[i]) > 0.0 && cam.z() > 0.0)) {
      r.all_in_front = false;
    }
    if (in.views[i].kf_id == in.creator_kf_id) {
      r.d_mv = cam.z();
      const V3 w0 = in.views[i].r_wc * V3(in.views[i].x, in.views[i].y, 1.0);
      const double den = in.plane.n.dot(w0);
      const double height = in.plane.n.dot(c[i]) - in.plane.offset;
      r.plane_ok = den != 0.0 && height > 0.0 && (in.plane.offset - in.plane.n.dot(c[i])) / den > 0.0;
      r.d_plane = den != 0.0 ? (in.plane.offset - in.plane.n.dot(c[i])) / den : 0.0;
    }
  }
  return r;
}

// ---- WorldMap scenes (float poses, exact pinhole rays) --------------------------------------------

struct ScenePoint {
  V3 world;                 // true position
  std::vector<int> observers;  // keyframe indices; the first one is the creator
};

struct SceneResult {
  std::vector<map::KeyframeId> kfs;
  std::vector<map::PointId> points;
  std::vector<double> d_true;   // true z-depth of the point in its creating keyframe
  std::vector<double> d_plane;  // z-depth of the seed on the assumed plane in its creating keyframe
  std::vector<c8::HMatrix> poses;   // keyframe poses of the scene
  std::vector<ScenePoint> pts;      // true positions and observers (for independent checks)
};


// Keyframes at `poses` (identity rotation) carrying the rays of the points they observe, then one
// ground point per ScenePoint. The seed is placed on the plane y = assumed_y along the creator's ray
// (the MapBuilder's [D] construction); the rays are those of the true position. `first_frame` is
// the creation frame of the first keyframe.
struct SceneKfs {
  std::vector<map::KeyframeId> kfs;
  std::vector<std::vector<int>> idx_of;  // [point][keyframe] keypoint index, -1 if not observed
};

inline SceneKfs AddSceneKeyframes(map::WorldMap* wm, const std::vector<c8::HMatrix>& poses,
                                  const std::vector<ScenePoint>& pts, int64_t first_frame = 0) {
  SceneKfs out;
  std::vector<std::vector<map_builder::testing::RayFeature>> rays(poses.size());
  out.idx_of.assign(pts.size(), std::vector<int>(poses.size(), -1));
  for (size_t p = 0; p < pts.size(); ++p) {
    for (int k : pts[p].observers) {
      const c8::HPoint2 r = map_builder::testing::Project(
          poses[static_cast<size_t>(k)], c8::HPoint3(static_cast<float>(pts[p].world.x()),
                                                    static_cast<float>(pts[p].world.y()),
                                                    static_cast<float>(pts[p].world.z())));
      out.idx_of[p][static_cast<size_t>(k)] = static_cast<int>(rays[static_cast<size_t>(k)].size());
      rays[static_cast<size_t>(k)].push_back(
          {r.x(), r.y(), 1, static_cast<uint32_t>(1000 + p * 16 + static_cast<size_t>(k))});
    }
  }
  for (size_t k = 0; k < poses.size(); ++k) {
    out.kfs.push_back(wm->AddKeyframe(map_builder::testing::MakeFrame(rays[k]),
                                      first_frame + static_cast<int64_t>(5 * k)));
    wm->SetKeyframePose(out.kfs.back(), poses[k]);
  }
  return out;
}

// Adds ground point number `p` of `pts` (seed on y = assumed_y, observations in listed order).
// Returns its id; *d_plane receives the z-depth of the seed in the creating keyframe.
inline map::PointId AddScenePoint(map::WorldMap* wm, const SceneKfs& kfs,
                                  const std::vector<c8::HMatrix>& poses,
                                  const std::vector<ScenePoint>& pts, size_t p, double assumed_y,
                                  double* d_plane = nullptr) {
  const size_t creator = static_cast<size_t>(pts[p].observers.front());
  const c8::HMatrix& cp = poses[creator];
  const V3 center(cp(0, 3), cp(1, 3), cp(2, 3));
  const c8::FramePoint& fp = wm->GetKeyframe(kfs.kfs[creator]).frame.points()[
      static_cast<size_t>(kfs.idx_of[p][creator])];
  const double ry = fp.y();
  const double s = (assumed_y - center.y()) / ry;
  const V3 seed = center + s * V3(fp.x(), ry, 1.0);
  const map::PointId pid =
      wm->AddPoint(wm->GetKeyframe(kfs.kfs[creator]).seq,
                   c8::HPoint3(static_cast<float>(seed.x()), static_cast<float>(seed.y()),
                               static_cast<float>(seed.z())),
                   map::PointSource::kGround);
  for (int k : pts[p].observers) {
    wm->AddObservation(pid, static_cast<uint32_t>(kfs.idx_of[p][static_cast<size_t>(k)]),
                       kfs.kfs[static_cast<size_t>(k)]);
  }
  if (d_plane != nullptr) {
    *d_plane = s;
  }
  return pid;
}

inline SceneResult BuildScene(map::WorldMap* wm, const std::vector<c8::HMatrix>& poses,
                              const std::vector<ScenePoint>& pts, double assumed_y,
                              int64_t first_frame = 0) {
  SceneResult out;
  out.poses = poses;
  out.pts = pts;
  const SceneKfs kfs = AddSceneKeyframes(wm, poses, pts, first_frame);
  out.kfs = kfs.kfs;
  for (size_t p = 0; p < pts.size(); ++p) {
    double d_plane = 0.0;
    out.points.push_back(AddScenePoint(wm, kfs, poses, pts, p, assumed_y, &d_plane));
    out.d_true.push_back(pts[p].world.z() -
                         poses[static_cast<size_t>(pts[p].observers.front())](2, 3));
    out.d_plane.push_back(d_plane);
  }
  return out;
}

inline c8::HMatrix PoseAt(double x, double y, double z) {
  return map_builder::testing::Pose(0.0f, static_cast<float>(x), static_cast<float>(y),
                                    static_cast<float>(z));
}

// The scene of the contract's T-height: group A is created by a keyframe at height 1.0 above the
// floor y = -1 and seen by five keyframes, group B by a keyframe at height 1.4 and seen by three.
// `floor_y` is the true surface the rays come from; the seeds always assume y = -1.
inline SceneResult BuildHeightScene(map::WorldMap* wm, double floor_y, int per_group = 100) {
  const std::vector<c8::HMatrix> poses = {
      PoseAt(0.0, 0.0, 0.0), PoseAt(0.5, 0.0, 0.0), PoseAt(1.0, 0.0, 0.0),
      PoseAt(0.25, 0.4, 0.2), PoseAt(0.75, 0.4, 0.2), PoseAt(1.25, 0.4, 0.2)};
  std::vector<ScenePoint> pts;
  for (int i = 0; i < per_group; ++i) {  // group A: creator K0 (height 1.0)
    const double z = 3.6 + 0.025 * static_cast<double>((i * 7) % 60);
    pts.push_back({V3(-1.0 + 0.03 * static_cast<double>(i), floor_y, z), {0, 1, 2, 3, 4}});
  }
  for (int i = 0; i < per_group; ++i) {  // group B: creator K3 (height 1.4)
    const double z = 4.4 + 0.025 * static_cast<double>((i * 11) % 60);
    pts.push_back({V3(-0.8 + 0.03 * static_cast<double>(i), floor_y, z), {3, 4, 5}});
  }
  return BuildScene(wm, poses, pts, -1.0);
}

inline map::F2FinalizeArgs Args(int64_t frame, bool hold = false, bool kf = true) {
  map::F2FinalizeArgs a;
  a.frame = frame;
  a.hold = hold;
  a.kf_path = kf;
  return a;
}

}  // namespace slam_refact::p2

#endif  // SLAM_REFACT_TESTS_MAP_BUILDER_P2_TEST_UTIL_H_
