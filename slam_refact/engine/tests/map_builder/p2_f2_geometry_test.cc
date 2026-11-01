// P2 candidate F2: geometric tests of the multi-view confirmation (f2-contract-v3.3.md section 7).
// Synthetic exact-pinhole scenes. The G1 gate requires every test name of this file and of the other
// p2_*_test.cc files; none of them is skipped or disabled.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <string>
#include <vector>

#include "Eigen/Geometry"
#include "gtest/gtest.h"
#include "map/f2_confirm.h"
#include "map/world_map.h"
#include "p2_test_util.h"

namespace slam_refact::p2 {
namespace {

using map::F2EvalInput;
using map::F2EvalResult;
using map::F2Reason;
using map::F2Status;

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

void ExpectReason(const F2EvalResult& r, F2Status status, F2Reason reason) {
  EXPECT_EQ(static_cast<int>(r.status), static_cast<int>(status));
  EXPECT_EQ(static_cast<int>(r.reason), static_cast<int>(reason));
}

// ---------------------------------------------------------------------------------------------
// T-height / T-nonfloor / T-lowparallax (WorldMap scenes)

TEST(P2F2Geometry, T_height_confirmed_and_errors) {
  ScopedAccuracy acc(false, true);
  map::WorldMap wm;
  const SceneResult sc = BuildHeightScene(&wm, -1.0);
  map::F2Finalize(wm, Args(10));
  // Independent precondition (steps 1-5 of the contract) from the synthetic scene itself, not from
  // F2's reasons: at least 3 distinct keyframes, the point in front of every camera, and a largest
  // ray angle of at least 3 degrees (rays from the camera centres to the true point).
  size_t precondition_ok = 0;
  size_t confirmed = 0;
  double max_seed_err = 0.0;
  double max_tri_err = 0.0;
  for (size_t i = 0; i < sc.points.size(); ++i) {
    const map::F2PointState& f = wm.Point(sc.points[i]).f2;
    ASSERT_TRUE(f.evaluated);
    {
      const ScenePoint& sp = sc.pts[i];
      std::vector<V3> dirs;
      bool front = true;
      for (const int k : sp.observers) {
        const c8::HMatrix& pose = sc.poses[static_cast<size_t>(k)];
        const V3 rel = sp.world - V3(pose(0, 3), pose(1, 3), pose(2, 3));
        front = front && rel.z() > 0.0;
        dirs.push_back(rel.normalized());
      }
      double theta = 0.0;
      for (size_t a = 0; a < dirs.size(); ++a) {
        for (size_t b = a + 1; b < dirs.size(); ++b) {
          theta = std::max(theta, std::atan2(dirs[a].cross(dirs[b]).norm(), dirs[a].dot(dirs[b])));
        }
      }
      if (sp.observers.size() >= 3 && front && theta >= Deg(3.0)) {
        ++precondition_ok;
      }
    }
    confirmed += f.state == F2Status::kConfirmed ? 1 : 0;
    ASSERT_TRUE(std::isfinite(f.d_plane) && std::isfinite(f.d_mv) && std::isfinite(sc.d_true[i]) &&
                sc.d_true[i] > 0.0)
        << "point " << i << ": the error metrics need finite depths";
    max_seed_err = std::max(max_seed_err, std::fabs(f.d_plane / sc.d_true[i] - 1.0));  // (1)
    max_tri_err = std::max(max_tri_err, std::fabs(f.d_mv / sc.d_true[i] - 1.0));       // (2)
  }
  EXPECT_EQ(sc.points.size(), 200u);
  EXPECT_EQ(precondition_ok, sc.points.size()) << "the scene itself satisfies the preconditions";
  EXPECT_GT(confirmed, 0u) << "T-height: zero confirmed points is a FAIL";
  EXPECT_GE(static_cast<double>(confirmed), 0.9 * static_cast<double>(precondition_ok));
  ASSERT_TRUE(std::isfinite(max_seed_err) && std::isfinite(max_tri_err));
  EXPECT_LE(max_seed_err, 1e-4) << "seed error |d_plane/d_true - 1| on the real floor";
  EXPECT_LE(max_tri_err, 1e-4) << "triangulation error |d_mv/d_true - 1|";
  // The scene holds seeds created at height 1.0 (K0) and, after the camera rose, at height 1.4 (K3).
  EXPECT_EQ(Counters().f2_eval[static_cast<size_t>(F2Reason::kConfirmed)], confirmed);
  std::printf("T-height: confirmed %zu of %zu (preconditions %zu), seed err max %.3g, tri err max %.3g\n",
              confirmed, sc.points.size(), precondition_ok, max_seed_err, max_tri_err);
}

TEST(P2F2Geometry, T_height_nonfloor_variant_contradicted) {
  ScopedAccuracy acc(false, true);
  map::WorldMap wm;
  // The surface is 0.3 above the plane the seeds assume: every seed is placed too far.
  const SceneResult sc = BuildHeightScene(&wm, -0.7);
  map::F2Finalize(wm, Args(10));
  size_t contradicted = 0;
  for (size_t i = 0; i < sc.points.size(); ++i) {
    const map::F2PointState& f = wm.Point(sc.points[i]).f2;
    const double seed_err = f.d_plane / sc.d_true[i] - 1.0;
    EXPECT_GT(seed_err, 0.20) << "point " << i;  // about +27% (height 1.4) .. +43% (height 1.0)
    EXPECT_LT(seed_err, 0.50) << "point " << i;
    EXPECT_NE(f.state, F2Status::kConfirmed) << "point " << i;
    contradicted += f.state == F2Status::kContradicted ? 1 : 0;
    EXPECT_EQ(static_cast<int>(f.last_reason), static_cast<int>(F2Reason::kDepthRatioOutOfBand));
  }
  EXPECT_EQ(contradicted, sc.points.size());
}

TEST(P2F2Geometry, T_nonfloor_depth_ratio_out_of_band) {
  // Preconditions 1-7 hold and the plane depth is 30% off: CONTRADICTED with that reason, and no point
  // of the non-floor scene is confirmed.
  ScopedAccuracy acc(false, true);
  F2EvalResult r = map::F2Evaluate(MakeInput(1.30));
  ExpectReason(r, F2Status::kContradicted, F2Reason::kDepthRatioOutOfBand);
  EXPECT_NEAR(r.ratio, 1.30, 1e-9);
  map::WorldMap wm;
  const SceneResult sc = BuildHeightScene(&wm, -0.7, 20);
  map::F2Finalize(wm, Args(3));
  EXPECT_EQ(Counters().f2_eval[static_cast<size_t>(F2Reason::kConfirmed)], 0u);
  EXPECT_EQ(Counters().f2_eval[static_cast<size_t>(F2Reason::kDepthRatioOutOfBand)],
            sc.points.size());
}

TEST(P2F2Geometry, T_lowparallax_degenerate_pure_rotation) {
  // One world direction seen from the same centre with three different camera rotations (pure
  // rotation), and three parallel rays from shifted centres: the rays do not determine a point.
  F2EvalInput rot;
  rot.creator_kf_id = kCreator;
  rot.plane.offset = -1.0;
  const V3 w = V3(0.0, -1.0 / 3.0, 1.0).normalized();
  for (int i = 0; i < 3; ++i) {
    const double yaw = 0.2 * i;
    M3 r;
    r << std::cos(yaw), 0.0, std::sin(yaw), 0.0, 1.0, 0.0, -std::sin(yaw), 0.0, std::cos(yaw);
    const V3 cam = r.transpose() * w;
    map::F2View v;
    v.kf_id = kCreator + static_cast<uint32_t>(i);
    v.center = V3::Zero();
    v.r_wc = r;
    v.x = cam.x() / cam.z();
    v.y = cam.y() / cam.z();
    rot.views.push_back(v);
  }
  ExpectReason(map::F2Evaluate(rot), F2Status::kUnconfirmed, F2Reason::kDegenerate);

  F2EvalInput par;
  par.creator_kf_id = kCreator;
  par.plane.offset = -1.0;
  for (int i = 0; i < 3; ++i) {
    map::F2View v;
    v.kf_id = kCreator + static_cast<uint32_t>(i);
    v.center = V3(0.5 * i, 0.0, 0.0);
    v.x = 0.0;
    v.y = -1.0 / 3.0;
    par.views.push_back(v);
  }
  ExpectReason(map::F2Evaluate(par), F2Status::kUnconfirmed, F2Reason::kDegenerate);
}

TEST(P2F2Geometry, T_lowparallax_two_degrees) {
  // Normal conditioning, largest ray angle 2 degrees: low_parallax, never contradicted.
  const double theta = Deg(2.0);
  F2EvalInput in = MakeInput(1.0);
  const V3 x = V3(0.0, -1.0, 3.0);
  const V3 d0 = V3(0.0, -1.0 / 3.0, 1.0).normalized();
  auto rot_x = [](const V3& v, double a) {
    return V3(v.x(), v.y() * std::cos(a) - v.z() * std::sin(a), v.y() * std::sin(a) + v.z() * std::cos(a));
  };
  const V3 d1 = rot_x(d0, theta / 2.0);
  const V3 d2 = rot_x(d0, -theta / 2.0);
  in.views.clear();
  const M3 id = M3::Identity();
  in.views.push_back(ViewTo(x, V3::Zero(), id, kCreator));
  in.views.push_back(ViewTo(x, x - 3.0 * d1, id, kCreator + 1));
  in.views.push_back(ViewTo(x, x - 3.0 * d2, id, kCreator + 2));
  const F2EvalResult r = map::F2Evaluate(in);
  ExpectReason(r, F2Status::kUnconfirmed, F2Reason::kLowParallax);
  EXPECT_NEAR(r.theta_max, theta, 1e-12);
  EXPECT_NE(static_cast<int>(r.status), static_cast<int>(F2Status::kContradicted));
}

// ---------------------------------------------------------------------------------------------
// Boundaries (absolute epsilon 1e-9, double; d_plane = 3)

TEST(P2F2Geometry, Boundary_ratio_1_05) {
  const double d_plane = 3.0;
  auto at = [&](double r) { return map::F2Evaluate(MakeInput(r, d_plane)); };
  const F2EvalResult a = at(1.05);
  ExpectReason(a, F2Status::kConfirmed, F2Reason::kConfirmed);
  EXPECT_NEAR(a.d_mv, 3.15, 1e-12);
  ExpectReason(at(1.05 + 0.5e-9), F2Status::kConfirmed, F2Reason::kConfirmed);
  const F2EvalResult out = at(1.05 + 2e-9);
  ExpectReason(out, F2Status::kContradicted, F2Reason::kDepthRatioOutOfBand);
  EXPECT_NEAR(out.ratio - 1.05, 2e-9, 1e-11);
}

TEST(P2F2Geometry, Boundary_ratio_0_95) {
  auto at = [&](double r) { return map::F2Evaluate(MakeInput(r, 3.0)); };
  const F2EvalResult a = at(0.95);
  ExpectReason(a, F2Status::kConfirmed, F2Reason::kConfirmed);
  EXPECT_NEAR(a.d_mv, 2.85, 1e-12);
  ExpectReason(at(0.95 - 0.5e-9), F2Status::kConfirmed, F2Reason::kConfirmed);
  const F2EvalResult out = at(0.95 - 2e-9);
  ExpectReason(out, F2Status::kContradicted, F2Reason::kDepthRatioOutOfBand);
  EXPECT_NEAR(0.95 - out.ratio, 2e-9, 1e-11);
}

TEST(P2F2Geometry, Boundary_ratio_counterexample_1_052) {
  // d_plane = 1, d_mv = 1.052: 5.2% apart, whichever way the ratio is read.
  const F2EvalResult r = map::F2Evaluate(MakeInput(1.052, 1.0));
  ExpectReason(r, F2Status::kContradicted, F2Reason::kDepthRatioOutOfBand);
  EXPECT_NEAR(r.d_plane, 1.0, 1e-12);
  EXPECT_NEAR(r.d_mv, 1.052, 1e-12);
}

namespace {

// Largest ray angle exactly `theta`: the creator's ray in the middle, the other two rays rotated
// by +-theta/2 about the x axis, all meeting at (0, -1, 3) (z-depth 3 = d_plane).
F2EvalInput ParallaxInput(double theta) {
  F2EvalInput in = MakeInput(1.0);
  const V3 x(0.0, -1.0, 3.0);
  const V3 d0 = V3(0.0, -1.0 / 3.0, 1.0).normalized();
  auto rot_x = [](const V3& v, double a) {
    return V3(v.x(), v.y() * std::cos(a) - v.z() * std::sin(a), v.y() * std::sin(a) + v.z() * std::cos(a));
  };
  const M3 id = M3::Identity();
  in.views.clear();
  in.views.push_back(ViewTo(x, V3::Zero(), id, kCreator));
  in.views.push_back(ViewTo(x, x - 3.0 * rot_x(d0, theta / 2.0), id, kCreator + 1));
  in.views.push_back(ViewTo(x, x - 3.0 * rot_x(d0, -theta / 2.0), id, kCreator + 2));
  return in;
}

// One ray (view 1) shifted by `t` in y (out of the plane of the cameras and the point, so the
// depth hardly moves): the triangulation compromises and the largest ray residual grows with t.
F2EvalInput ResidualInput(double t) {
  F2EvalInput in = MakeInput(1.0);
  in.views[1].y += t;
  return in;
}

}  // namespace

TEST(P2F2Geometry, Boundary_parallax_3deg) {
  const double three = Deg(3.0);
  for (const double d : {0.0, -0.5e-9, -2e-9, +2e-9}) {
    const F2EvalInput in = ParallaxInput(three + d);
    const F2EvalResult r = map::F2Evaluate(in);
    const Indep ind = IndepGeometry(in);
    EXPECT_NEAR(r.theta_max, three + d, 1e-12) << "delta " << d;
    EXPECT_NEAR(ind.theta_max, r.theta_max, 1e-12);
    if (d >= -0.5e-9) {
      ExpectReason(r, F2Status::kConfirmed, F2Reason::kConfirmed);  // 3 deg and 3 deg - 0.5e-9 rad pass
    } else {
      ExpectReason(r, F2Status::kUnconfirmed, F2Reason::kLowParallax);  // 3 deg - 2e-9 rad
    }
  }
}

TEST(P2F2Geometry, Boundary_residual_0_75deg) {
  const double target = Deg(0.75);
  // Bisect the ray shift t so that the largest ray residual equals target + delta to 1e-13 rad.
  auto solve = [&](double want) {
    double lo = 0.0;
    double hi = 0.2;
    for (int i = 0; i < 200; ++i) {
      const double mid = 0.5 * (lo + hi);
      (map::F2Evaluate(ResidualInput(mid)).rho_max < want ? lo : hi) = mid;
    }
    return 0.5 * (lo + hi);
  };
  for (const double d : {0.0, 0.5e-9, 2e-9}) {
    const F2EvalInput in = ResidualInput(solve(target + d));
    const F2EvalResult r = map::F2Evaluate(in);
    const Indep ind = IndepGeometry(in);
    EXPECT_NEAR(r.rho_max, target + d, 1e-12) << "delta " << d;
    EXPECT_NEAR(ind.rho_max, r.rho_max, 1e-12);
    EXPECT_GT(r.theta_max, Deg(3.0));
    if (d <= 0.5e-9) {
      ExpectReason(r, F2Status::kConfirmed, F2Reason::kConfirmed);  // 0.75 deg and + 0.5e-9 rad pass
    } else {
      ExpectReason(r, F2Status::kUnconfirmed, F2Reason::kLargeResidual);  // + 2e-9 rad
    }
  }
}

TEST(P2F2Geometry, Boundary_kf_count) {
  F2EvalInput in = MakeInput(1.0);
  ExpectReason(map::F2Evaluate(in), F2Status::kConfirmed, F2Reason::kConfirmed);  // 3 distinct
  F2EvalInput two = in;
  two.views.pop_back();
  const F2EvalResult r2 = map::F2Evaluate(two);
  ExpectReason(r2, F2Status::kUnconfirmed, F2Reason::kTooFewKfs);
  EXPECT_EQ(r2.distinct_kfs, 2);
  EXPECT_FALSE(r2.no_creator);
  // A second observation of the same keyframe (another index) counts once: still two keyframes.
  F2EvalInput dup = two;
  dup.views.push_back(two.views[1]);
  dup.views.back().obs_idx = 7;
  const F2EvalResult rd = map::F2Evaluate(dup);
  ExpectReason(rd, F2Status::kUnconfirmed, F2Reason::kTooFewKfs);
  EXPECT_EQ(rd.distinct_kfs, 2);
}

// ---------------------------------------------------------------------------------------------
// First-match order and degenerate / invalid inputs

namespace {

F2EvalInput NanInput() {
  F2EvalInput in = MakeInput(1.0);
  in.views[1].x = std::numeric_limits<double>::quiet_NaN();
  in.views.pop_back();  // also only 2 keyframes
  return in;
}

F2EvalInput TwoKfParallel() {
  F2EvalInput in;
  in.creator_kf_id = kCreator;
  in.plane.offset = -1.0;
  for (int i = 0; i < 2; ++i) {
    map::F2View v;
    v.kf_id = kCreator + static_cast<uint32_t>(i);
    v.center = V3(0.5 * i, 0.0, 0.0);
    v.y = -1.0 / 3.0;
    in.views.push_back(v);
  }
  return in;
}

F2EvalInput ParallelAndBehind() {
  F2EvalInput in = TwoKfParallel();
  map::F2View v = in.views[0];
  v.kf_id = kCreator + 2;
  v.center = V3(1.0, 0.0, 0.0);
  in.views.push_back(v);
  for (map::F2View& w : in.views) {  // rays point away from the plane: (x, y, 1) = (0, +1/3, 1)
    w.y = 1.0 / 3.0;
  }
  return in;
}

// Rays that meet 3 behind the cameras (z < 0) from centres 0.03 apart (a fraction of a degree).
F2EvalInput BehindLowParallax() {
  F2EvalInput in;
  in.creator_kf_id = kCreator;
  in.plane.offset = -1.0;
  const V3 x(0.0, -1.0, -3.0);
  const M3 id = M3::Identity();
  in.views.push_back(ViewTo(x, V3::Zero(), id, kCreator));
  in.views.push_back(ViewTo(x, V3(-0.03, 0.0, 0.0), id, kCreator + 1));
  in.views.push_back(ViewTo(x, V3(0.03, 0.0, 0.0), id, kCreator + 2));
  return in;
}

// Parallax 2 deg and the creator's ray 2 deg out of the plane of the other two: small angle, big residual.
F2EvalInput LowParallaxLargeResidual() {
  F2EvalInput in = ParallaxInput(Deg(2.0));
  const V3 x(0.0, -1.0, 3.0);
  const M3 id = M3::Identity();
  const V3 d0 = V3(0.0, -1.0 / 3.0, 1.0).normalized();
  V3 tilted = d0;
  tilted.x() += std::tan(Deg(2.0)) * 1.0;  // sideways, perpendicular to the rotation plane
  in.views[0] = ViewTo(x, V3::Zero(), id, kCreator);
  in.views[0].x = tilted.x() / tilted.z();
  in.views[0].y = tilted.y() / tilted.z();
  return in;
}

// Five degree perturbation of one ray (large residual, parallax fine) and a creator ray that
// points up (lambda <= 0).
F2EvalInput LargeResidualPlaneInvalid() {
  F2EvalInput in = MakeInput(1.0);
  in.views[1].x += std::tan(Deg(5.0));
  in.views[0].y = +1.0 / 3.0;  // creator ray upward: the plane is behind it
  return in;
}

// Creator ray upward (plane invalid) and a ratio that would be out of band.
F2EvalInput PlaneInvalidRatioOut() {
  F2EvalInput in = MakeInput(1.30);
  const V3 x = in.views[0].center + 3.9 * V3(0.0, +1.0 / 3.0, 1.0);
  const M3 id = M3::Identity();
  in.views[0] = ViewTo(x, V3::Zero(), id, kCreator);
  in.views[1] = ViewTo(x, V3(-1.2, 0.0, 0.0), id, kCreator + 1);
  in.views[2] = ViewTo(x, V3(1.0, 0.1, -0.3), id, kCreator + 2);
  return in;
}

}  // namespace

TEST(P2F2Geometry, Order_first_match_pairs) {
  // (NaN + 2 KF) -> invalid_input
  ExpectReason(map::F2Evaluate(NanInput()), F2Status::kUnconfirmed, F2Reason::kInvalidInput);
  // (2 KF + parallel) -> too_few_kfs
  ExpectReason(map::F2Evaluate(TwoKfParallel()), F2Status::kUnconfirmed, F2Reason::kTooFewKfs);
  // (parallel + negative depth) -> degenerate
  ExpectReason(map::F2Evaluate(ParallelAndBehind()), F2Status::kUnconfirmed, F2Reason::kDegenerate);
  // (negative depth + low parallax) -> negative_depth
  {
    const F2EvalInput in = BehindLowParallax();
    const Indep ind = IndepGeometry(in);
    EXPECT_FALSE(ind.all_in_front);
    EXPECT_LT(ind.theta_max, Deg(3.0));
    ExpectReason(map::F2Evaluate(in), F2Status::kUnconfirmed, F2Reason::kNegativeDepth);
  }
  // (low parallax + large residual) -> low_parallax
  {
    const F2EvalInput in = LowParallaxLargeResidual();
    const Indep ind = IndepGeometry(in);
    EXPECT_TRUE(ind.all_in_front);
    EXPECT_LT(ind.theta_max, Deg(3.0));
    EXPECT_GT(ind.rho_max, Deg(0.75));
    ExpectReason(map::F2Evaluate(in), F2Status::kUnconfirmed, F2Reason::kLowParallax);
  }
  // (large residual + lambda <= 0) -> large_residual
  {
    const F2EvalInput in = LargeResidualPlaneInvalid();
    const Indep ind = IndepGeometry(in);
    EXPECT_GT(ind.theta_max, Deg(3.0));
    EXPECT_GT(ind.rho_max, Deg(0.75));
    EXPECT_FALSE(ind.plane_ok);
    ExpectReason(map::F2Evaluate(in), F2Status::kUnconfirmed, F2Reason::kLargeResidual);
  }
  // (lambda <= 0 + ratio out of band) -> plane_invalid
  {
    const F2EvalInput in = PlaneInvalidRatioOut();
    const Indep ind = IndepGeometry(in);
    EXPECT_GT(ind.theta_max, Deg(3.0));
    EXPECT_LT(ind.rho_max, Deg(0.75));
    EXPECT_FALSE(ind.plane_ok);
    EXPECT_GT(std::fabs(ind.d_mv / ind.d_plane - 1.0), 0.05);  // the ratio would be out of band too
    ExpectReason(map::F2Evaluate(in), F2Status::kUnconfirmed, F2Reason::kPlaneInvalid);
  }
  // ratio out of band alone -> CONTRADICTED
  ExpectReason(map::F2Evaluate(MakeInput(1.2)), F2Status::kContradicted, F2Reason::kDepthRatioOutOfBand);
}

TEST(P2F2Geometry, Order_no_contradicted_for_reasons_1_to_7) {
  const std::vector<F2EvalInput> inputs = {
      NanInput(),           TwoKfParallel(),           ParallelAndBehind(),
      BehindLowParallax(),  LowParallaxLargeResidual(), LargeResidualPlaneInvalid(),
      PlaneInvalidRatioOut(), ParallaxInput(Deg(2.0)),   ResidualInput(0.3)};
  std::vector<int> reasons;
  for (const F2EvalInput& in : inputs) {
    const F2EvalResult r = map::F2Evaluate(in);
    reasons.push_back(static_cast<int>(r.reason));
    EXPECT_EQ(static_cast<int>(r.status), static_cast<int>(F2Status::kUnconfirmed))
        << "reason " << static_cast<int>(r.reason);
  }
  // Reasons 1..7 are all produced by this list.
  for (int want = 1; want <= 7; ++want) {
    EXPECT_NE(std::find(reasons.begin(), reasons.end(), want), reasons.end()) << "reason " << want;
  }
}

TEST(P2F2Geometry, Degenerate_invalid_input_nan_inf) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<F2EvalInput> bad;
  for (const double v : {nan, inf, -inf}) {
    F2EvalInput a = MakeInput(1.0);
    a.views[2].x = v;  // ray
    bad.push_back(a);
    F2EvalInput b = MakeInput(1.0);
    b.views[1].center.y() = v;  // pose
    bad.push_back(b);
    F2EvalInput c = MakeInput(1.0);
    c.views[0].r_wc(1, 1) = v;  // pose (rotation)
    bad.push_back(c);
    F2EvalInput d = MakeInput(1.0);
    d.plane.offset = v;  // plane value
    bad.push_back(d);
    F2EvalInput e = MakeInput(1.0);
    e.plane.n.x() = v;
    bad.push_back(e);
  }
  for (const F2EvalInput& in : bad) {
    ExpectReason(map::F2Evaluate(in), F2Status::kUnconfirmed, F2Reason::kInvalidInput);
  }
}

TEST(P2F2Geometry, Degenerate_parallel_rays) {
  // All rays parallel, or all rays identical from the same centre.
  F2EvalInput par = MakeInput(1.0);
  for (size_t i = 0; i < par.views.size(); ++i) {
    par.views[i].x = 0.0;
    par.views[i].y = -1.0 / 3.0;
  }
  ExpectReason(map::F2Evaluate(par), F2Status::kUnconfirmed, F2Reason::kDegenerate);
  F2EvalInput same = par;
  for (map::F2View& v : same.views) {
    v.center = V3::Zero();
  }
  ExpectReason(map::F2Evaluate(same), F2Status::kUnconfirmed, F2Reason::kDegenerate);
  std::vector<V3> centers = {V3::Zero(), V3(1, 0, 0), V3(2, 0, 0)};
  std::vector<V3> dirs(3, V3(0, 0, 1));
  V3 x;
  EXPECT_FALSE(map::F2Triangulate(centers, dirs, &x));
}

TEST(P2F2Geometry, Degenerate_negative_depth) {
  // The point is behind the second camera: its z-depth there is negative while the rays still meet.
  F2EvalInput in = MakeInput(1.0);
  const V3 x = 3.0 * V3(0.0, -1.0 / 3.0, 1.0);
  const M3 id = M3::Identity();
  in.views[1] = ViewTo(x, V3(0.0, 0.0, 6.0), id, kCreator + 1);  // x - c = (0,-1,-3)
  const Indep ind = IndepGeometry(in);
  EXPECT_FALSE(ind.all_in_front);
  EXPECT_GT(ind.theta_max, Deg(3.0));
  const F2EvalResult r = map::F2Evaluate(in);
  ExpectReason(r, F2Status::kUnconfirmed, F2Reason::kNegativeDepth);
  EXPECT_GT(r.d_mv, 0.0);  // the creator itself sees the point in front
}

TEST(P2F2Geometry, Degenerate_large_residual) {
  F2EvalInput in = MakeInput(1.0);
  in.views[2].x += std::tan(Deg(5.0));  // one ray perturbed by 5 degrees
  const F2EvalResult r = map::F2Evaluate(in);
  ExpectReason(r, F2Status::kUnconfirmed, F2Reason::kLargeResidual);
  EXPECT_GT(r.rho_max, Deg(0.75));
  EXPECT_NEAR(IndepGeometry(in).rho_max, r.rho_max, 1e-12);
}

TEST(P2F2Geometry, Degenerate_plane_invalid) {
  // lambda <= 0: creator ray points away from the plane.
  F2EvalInput up = MakeInput(1.0);
  const V3 x = 3.0 * V3(0.0, +1.0 / 3.0, 1.0);
  const M3 id = M3::Identity();
  up.views[0] = ViewTo(x, V3::Zero(), id, kCreator);
  up.views[1] = ViewTo(x, V3(-1.2, 0.0, 0.0), id, kCreator + 1);
  up.views[2] = ViewTo(x, V3(1.0, 0.1, -0.3), id, kCreator + 2);
  ExpectReason(map::F2Evaluate(up), F2Status::kUnconfirmed, F2Reason::kPlaneInvalid);
  // The creator is below the plane.
  F2EvalInput below = MakeInput(1.0);
  below.plane.offset = +0.5;
  ExpectReason(map::F2Evaluate(below), F2Status::kUnconfirmed, F2Reason::kPlaneInvalid);
  // The ray is parallel to the plane.
  F2EvalInput flat = MakeInput(1.0);
  const V3 xf = 3.0 * V3(0.0, 0.0, 1.0);
  flat.views[0] = ViewTo(xf, V3::Zero(), id, kCreator);
  flat.views[1] = ViewTo(xf, V3(-1.2, 0.0, 0.0), id, kCreator + 1);
  flat.views[2] = ViewTo(xf, V3(1.0, 0.1, -0.3), id, kCreator + 2);
  ExpectReason(map::F2Evaluate(flat), F2Status::kUnconfirmed, F2Reason::kPlaneInvalid);
}

// ---------------------------------------------------------------------------------------------
// Independence (a), (a2), (b), (c)

namespace {

// Example scene of the contract: creator at the origin, plane y = -1, point (0, -1, 3), d = 3.
F2EvalInput ExampleScene(const V3& creator_center = V3::Zero()) {
  F2EvalInput in;
  in.creator_kf_id = kCreator;
  in.plane.offset = -1.0;
  const double height = creator_center.y() + 1.0;
  const V3 x = creator_center + 3.0 * height * V3(0.0, -1.0 / 3.0, 1.0);
  const M3 id = M3::Identity();
  in.views.push_back(ViewTo(x, creator_center, id, kCreator));
  in.views.push_back(ViewTo(x, creator_center + V3(-1.2, 0.0, 0.0), id, kCreator + 1));
  in.views.push_back(ViewTo(x, creator_center + V3(1.0, 0.1, -0.3), id, kCreator + 2));
  return in;
}

}  // namespace

TEST(P2F2Geometry, Independence_a_pose_only_scale) {
  F2EvalInput in = ExampleScene();
  const F2EvalResult before = map::F2Evaluate(in);
  ExpectReason(before, F2Status::kConfirmed, F2Reason::kConfirmed);
  EXPECT_NEAR(before.d_plane, 3.000, 1e-12);
  EXPECT_NEAR(before.d_mv, 3.000, 1e-12);
  // The non-creator keyframes move away from the creator by 1.1; rays, rotations and the plane stay.
  in.views[1].center *= 1.1;
  in.views[2].center *= 1.1;
  const F2EvalResult after = map::F2Evaluate(in);
  EXPECT_NEAR(after.d_plane, 3.000, 1e-12);  // unchanged: creator pose and plane did not change
  EXPECT_NEAR(after.d_mv, 3.300, 1e-12);
  EXPECT_NEAR(after.ratio, 1.100, 1e-12);
  ExpectReason(after, F2Status::kContradicted, F2Reason::kDepthRatioOutOfBand);
}

TEST(P2F2Geometry, Independence_a2_origin_scale) {
  // Creator centre (0, 0.5, 0): height 1.5, d_plane = d_mv = 4.5. Every keyframe (the creator
  // included) is scaled by 1.1 about the origin; the plane y = -1 stays.
  F2EvalInput in = ExampleScene(V3(0.0, 0.5, 0.0));
  const F2EvalResult before = map::F2Evaluate(in);
  ExpectReason(before, F2Status::kConfirmed, F2Reason::kConfirmed);
  EXPECT_NEAR(before.d_plane, 4.5, 1e-12);
  EXPECT_NEAR(before.d_mv, 4.5, 1e-12);
  for (map::F2View& v : in.views) {
    v.center *= 1.1;
  }
  const F2EvalResult after = map::F2Evaluate(in);
  EXPECT_NEAR(after.d_plane, 4.650, 1e-12);  // 3 * (0.55 + 1)
  EXPECT_NEAR(after.d_mv, 4.950, 1e-12);
  EXPECT_NEAR(after.ratio, 1.0645, 5e-5);
  ExpectReason(after, F2Status::kContradicted, F2Reason::kDepthRatioOutOfBand);
}

TEST(P2F2Geometry, Independence_b_similarity_invariance) {
  // The whole configuration, plane included, under a similarity (s = 1.1, rotation, translation).
  const F2EvalInput base = ExampleScene();
  const F2EvalResult r0 = map::F2Evaluate(base);
  ExpectReason(r0, F2Status::kConfirmed, F2Reason::kConfirmed);
  const double s = 1.1;
  const Eigen::AngleAxisd rot(0.7, V3(0.3, -0.5, 0.8).normalized());
  const M3 r = rot.toRotationMatrix();
  const V3 t(2.0, -3.0, 5.0);
  F2EvalInput sim = base;
  for (map::F2View& v : sim.views) {
    v.center = s * r * v.center + t;
    v.r_wc = r * v.r_wc;  // rays (x, y) are camera coordinates: unchanged
  }
  sim.plane.n = r * base.plane.n;
  sim.plane.offset = s * base.plane.offset + sim.plane.n.dot(t);
  const F2EvalResult r1 = map::F2Evaluate(sim);
  ExpectReason(r1, F2Status::kConfirmed, F2Reason::kConfirmed);
  EXPECT_NEAR(r1.d_plane, 3.300, 1e-11);
  EXPECT_NEAR(r1.d_mv, 3.300, 1e-11);
  EXPECT_NEAR(r1.ratio, 1.000, 1e-11);
  EXPECT_NEAR(r1.theta_max, r0.theta_max, 1e-11);
  EXPECT_NEAR(r1.rho_max, r0.rho_max, 1e-7);  // acos near 1 resolves only ~1.5e-8 rad
}

TEST(P2F2Geometry, Independence_c_seed_perturbation_bit_identical) {
  ScopedAccuracy acc(false, true);
  auto run = [](double scale, double* d_mv, double* d_plane, double* ratio) {
    map::WorldMap wm;
    const std::vector<c8::HMatrix> poses = {PoseAt(0.0, 0.0, 0.0), PoseAt(-1.2, 0.0, 0.0),
                                            PoseAt(1.0, 0.1, -0.3)};
    const std::vector<ScenePoint> pts = {ScenePoint{V3(0.0, -1.0, 3.0), {0, 1, 2}}};
    const SceneResult sc = BuildScene(&wm, poses, pts, -1.0);
    // Perturb the seed's stored position and the initial depth of every observation by `scale`.
    map::WorldPoint& p = wm.MutablePoint(sc.points[0]);
    p.position = c8::HPoint3(p.position.x() * static_cast<float>(scale),
                             p.position.y() * static_cast<float>(scale),
                             p.position.z() * static_cast<float>(scale));
    for (const auto& [kf, idx] : p.observations) {
      c8::FramePoint& fp = (*wm.MutableKeyframe(kf).frame.mutablePoints())[idx];
      fp.updateInitialDepthDuringDeserialization(fp.initialDepth() * scale);
    }
    map::F2Finalize(wm, Args(1));
    const map::F2PointState& f = wm.Point(sc.points[0]).f2;
    EXPECT_TRUE(f.evaluated);
    EXPECT_EQ(static_cast<int>(f.last_reason), static_cast<int>(F2Reason::kConfirmed));
    *d_mv = f.d_mv;
    *d_plane = f.d_plane;
    *ratio = f.ratio;
  };
  double mv1, pl1, r1, mv_lo, pl_lo, r_lo, mv_hi, pl_hi, r_hi;
  run(1.0, &mv1, &pl1, &r1);
  run(0.9, &mv_lo, &pl_lo, &r_lo);
  run(1.1, &mv_hi, &pl_hi, &r_hi);
  EXPECT_NEAR(mv1, 3.000, 1e-5);
  EXPECT_NEAR(pl1, 3.000, 1e-5);
  EXPECT_NEAR(r1, 1.000, 1e-5);
  EXPECT_EQ(mv_lo, mv1);  // bit identical: EXPECT_EQ on doubles
  EXPECT_EQ(mv_hi, mv1);
  EXPECT_EQ(pl_lo, pl1);
  EXPECT_EQ(pl_hi, pl1);
  EXPECT_EQ(r_lo, r1);
  EXPECT_EQ(r_hi, r1);
}

}  // namespace
}  // namespace slam_refact::p2
