// Accuracy-mode local BA bookkeeping (review E-05 / NOW-4). The mode is off by default; these
// tests call LocalBundleAdjustment directly with an AccuracyConfig installed for one scope.
//
// SetPointPosition (1150) unlinks every observation whose ray residual exceeds the world-map limit.
// When that removes the last observation of a point, the point must stay in the dirty set so the
// next AdjustDirtyPoints (1127) retires it; the BA must not leave a live point without observations.

#include <vector>

#include "c8/hmatrix.h"
#include "c8/hpoint.h"
#include "common/check.h"
#include "config/accuracy.h"
#include "gtest/gtest.h"
#include "map/world_map.h"
#include "map_builder/map_builder.h"
#include "map_builder_test_util.h"
#include "slam_refact/engine_config.h"

namespace slam_refact::map_builder {
namespace {

using testing::MakeFrame;
using testing::Pose;
using testing::Project;
using testing::RayFeature;

// Installs an AccuracyConfig for one scope and restores the previous one.
class ScopedAccuracy {
 public:
  explicit ScopedAccuracy(const AccuracyConfig& cfg) : previous_(config::Accuracy()) {
    config::SetAccuracy(cfg);
  }
  ~ScopedAccuracy() { config::SetAccuracy(previous_); }
  ScopedAccuracy(const ScopedAccuracy&) = delete;
  ScopedAccuracy& operator=(const ScopedAccuracy&) = delete;

 private:
  AccuracyConfig previous_;
};

constexpr int kGoodPoints = 12;

struct Scene {
  map::WorldMap wm;
  std::vector<map::KeyframeId> kfs;
  std::vector<map::PointId> good;
  map::PointId bad;
};

// Three keyframes, kGoodPoints consistent points seen by all of them, and one point (the last
// feature of k0 and k1) whose two rays are inconsistent (same camera orientation, y-ray +0.3 vs
// -0.3, sideways baseline): no position gives both a ray residual below the world-map limit.
void BuildScene(Scene* sc) {
  const std::vector<c8::HMatrix> truth = {Pose(0.0f, 0.0f, 0.0f, 0.0f), Pose(0.05f, 0.4f, 0.0f, 0.05f),
                                          Pose(-0.03f, -0.35f, 0.02f, 0.1f)};
  std::vector<c8::HPoint3> world;
  for (int i = 0; i < kGoodPoints; ++i) {
    world.emplace_back(-1.0f + 0.15f * static_cast<float>(i),
                       -0.5f + 0.07f * static_cast<float>((i * 5) % 12),
                       2.5f + 0.1f * static_cast<float>((i * 7) % 12));
  }
  std::vector<std::vector<RayFeature>> rays(3);
  for (size_t k = 0; k < 3; ++k) {
    for (int i = 0; i < kGoodPoints; ++i) {
      const c8::HPoint2 r = Project(truth[k], world[static_cast<size_t>(i)]);
      rays[k].push_back({r.x(), r.y(), 1, static_cast<uint32_t>(100 + i)});
    }
  }
  rays[0].push_back({0.0f, 0.3f, 1, 500});
  rays[1].push_back({0.0f, -0.3f, 1, 500});

  map::WorldMap& wm = sc->wm;
  for (size_t k = 0; k < 3; ++k) {
    sc->kfs.push_back(wm.AddKeyframe(MakeFrame(rays[k]), static_cast<int64_t>(5 * k)));
  }
  // k2 starts off its true pose so the BA has cost to remove and is applied.
  wm.SetKeyframePose(sc->kfs[0], truth[0]);
  wm.SetKeyframePose(sc->kfs[1], truth[1]);
  wm.SetKeyframePose(sc->kfs[2], Pose(-0.03f, -0.32f, 0.02f, 0.1f));
  for (int i = 0; i < kGoodPoints; ++i) {
    const map::PointId p =
        wm.AddPoint(0, world[static_cast<size_t>(i)], map::PointSource::kTriangulated);
    for (size_t k = 0; k < 3; ++k) {
      wm.AddObservation(p, static_cast<uint32_t>(i), sc->kfs[k]);
    }
    sc->good.push_back(p);
  }
  sc->bad = wm.AddPoint(0, c8::HPoint3(0.0f, 0.9f, 3.0f), map::PointSource::kTriangulated);
  wm.AddObservation(sc->bad, kGoodPoints, sc->kfs[0]);
  wm.AddObservation(sc->bad, kGoodPoints, sc->kfs[1]);
  SR_CHECK(wm.Point(sc->bad).observations.size() == 2u, "scene setup");
}

void ExpectGoodPointsIntact(const Scene& sc) {
  for (const map::PointId p : sc.good) {
    EXPECT_NE(sc.wm.Point(p).state, map::PointState::kDead);
    EXPECT_FALSE(sc.wm.Point(p).observations.empty());
  }
}

// SetPointPosition (1150) unlinks both observations of the contradictory point (quadratic loss:
// the compromise leaves both far off). The point must be retired by the next 1127.
TEST(MapBuilderLocalBa, PointWhoseObservationsAreAllUnlinkedByBaIsRetired) {
  AccuracyConfig acc;
  acc.enabled = true;
  acc.local_ba = true;
  acc.ba_huber_px = 1.0e4;
  const ScopedAccuracy scoped(acc);
  Scene sc;
  BuildScene(&sc);

  const LocalBaResult ba = LocalBundleAdjustment(sc.wm, sc.kfs[2]);
  ASSERT_TRUE(ba.applied);
  ASSERT_TRUE(sc.wm.Point(sc.bad).observations.empty());

  sc.wm.AdjustDirtyPoints();  // the common one at the end of MapBuilder::Update (936.c:5758)
  EXPECT_EQ(sc.wm.Point(sc.bad).state, map::PointState::kDead)
      << "point without observations stayed alive after the BA";
  ExpectGoodPointsIntact(sc);
}

// Default Huber loss: the contradictory point keeps one exactly satisfied observation, 1150 unlinks
// the other, and the ba_outlier_px step (set below the float rounding of that residual) unlinks
// the last one. Unlinking a point's last observation does not dirty it, so the BA must.
TEST(MapBuilderLocalBa, PointLosingItsLastObservationToTheOutlierStepIsRetired) {
  AccuracyConfig acc;
  acc.enabled = true;
  acc.local_ba = true;
  acc.ba_outlier_px = 1.0e-9;
  const ScopedAccuracy scoped(acc);
  Scene sc;
  BuildScene(&sc);

  const LocalBaResult ba = LocalBundleAdjustment(sc.wm, sc.kfs[2]);
  ASSERT_TRUE(ba.applied);
  ASSERT_GE(ba.unlinked, 1);
  ASSERT_TRUE(sc.wm.Point(sc.bad).observations.empty());

  sc.wm.AdjustDirtyPoints();
  EXPECT_EQ(sc.wm.Point(sc.bad).state, map::PointState::kDead)
      << "point without observations stayed alive after the outlier step";
  // ba_outlier_px = 1e-9 is below the float rounding of any residual, so the good points lose all
  // their observations too (ExpectGoodPointsIntact does not apply). The invariant checked instead:
  // no point is left alive without observations.
  for (const map::PointId p : sc.good) {
    EXPECT_TRUE(sc.wm.Point(p).state == map::PointState::kDead || !sc.wm.Point(p).observations.empty());
  }
}

}  // namespace
}  // namespace slam_refact::map_builder
