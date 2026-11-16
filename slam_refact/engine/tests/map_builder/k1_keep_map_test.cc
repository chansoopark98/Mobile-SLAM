// XR commercialization candidate K1 (AccuracyConfig::k1_keep_map): when the reset rule of the
// MapBuilder fires on a NON-EMPTY map, the map is kept and a keyframe is added at the current pose
// instead of discarding everything. Synthetic frames only (the drivers of update_test.cc).
//
// Scenario (every test): frame 0 creates the first map by the reset path (state 0), frames 1..12
// track (state 1, keyframes at 0, 5, 10), frames 13..19 are lost (state 0). The reset rule fires on
// frame 19: more than numFramesToResetNonEmptyMap (5) frames since the state change at 13, 24
// ground candidates, 7+ consecutive frame-tracker successes and 3 <= 5 keyframe slots. Frames 20..
// track again (state 1).

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <vector>

#include "c8/geometry/egomotion.h"
#include "c8/hmatrix.h"
#include "c8/hpoint.h"
#include "c8/quaternion.h"
#include "common/tracking_types.h"
#include "compat/random_numbers.h"
#include "config/accuracy.h"
#include "gtest/gtest.h"
#include "map/world_map.h"
#include "map/world_map_internal.h"
#include "map_builder/map_builder.h"
#include "map_builder/map_builder_internal.h"
#include "map_builder_test_util.h"
#include "map_tracker/map_tracker.h"
#include "map_tracker/map_tracker_internal.h"
#include "../map_tracker/scene_util.h"
#include "slam_refact/engine_config.h"

namespace slam_refact::map_builder {
namespace {

using testing::AddRays;
using testing::Pose;
using testing::RayFeature;
using testing::TestIntrinsics;

constexpr int kGround = 24;
constexpr int kSky = 40;
constexpr int kLastTrackingFrame = 12;  // frames 1..12 are state 1
constexpr int kEventFrame = 19;         // the reset rule fires here

// Installs an AccuracyConfig for one scope and restores the previous one (also on ASSERT exits).
class ScopedK1 {
 public:
  ScopedK1(bool enabled, bool k1) : previous_(config::Accuracy()) {
    AccuracyConfig cfg;
    cfg.enabled = enabled;
    cfg.k1_keep_map = k1;
    config::SetAccuracy(cfg);
  }
  ~ScopedK1() { config::SetAccuracy(previous_); }
  ScopedK1(const ScopedK1&) = delete;
  ScopedK1& operator=(const ScopedK1&) = delete;

 private:
  AccuracyConfig previous_;
};

std::unique_ptr<TrackerInput> MakeInput(int frame) {
  auto in = std::make_unique<TrackerInput>(TestIntrinsics(), c8::Quaternion(1.f, 0.f, 0.f, 0.f),
                                           static_cast<int64_t>(frame) * 33333333);
  std::vector<RayFeature> rays;
  for (int i = 0; i < kGround; ++i) {
    rays.push_back({-0.4f + 0.035f * static_cast<float>(i), -0.1f - 0.01f * static_cast<float>(i % 7),
                    1, static_cast<uint32_t>(frame * 1000 + i)});
  }
  for (int i = 0; i < kSky; ++i) {
    rays.push_back({-0.4f + 0.02f * static_cast<float>(i), 0.2f + 0.003f * static_cast<float>(i),
                    1, static_cast<uint32_t>(frame * 1000 + 500 + i)});
  }
  AddRays(&in->frame_points, rays);
  return in;
}

MapTrackerOutput MakeTrk(int32_t state, int64_t consecutive, const c8::HMatrix& pose) {
  MapTrackerOutput trk;
  trk.pose = pose;
  trk.state = state;
  trk.frame_tracker_ok = true;
  trk.consecutive_frame_tracker_ok = consecutive;
  for (uint32_t i = 0; i < kGround; ++i) {
    trk.ground_idx.push_back(i);
  }
  for (uint32_t i = 0; i < kSky; ++i) {
    trk.sky_idx.push_back(kGround + i);
  }
  return trk;
}

int StateOf(int f) {
  if (f == 0) return 0;
  if (f <= kLastTrackingFrame) return 1;
  if (f <= kEventFrame) return 0;
  return 1;
}

// Tracking frames follow a slow path; lost frames (13..19) sit at a different pose whose height is
// `lost_y`; later tracking frames continue from the lost pose at height 0.
c8::HMatrix PoseAt(int f, float lost_y) {
  if (f <= kLastTrackingFrame) {
    return Pose(0.002f * static_cast<float>(f), 0.01f * static_cast<float>(f), 0.f, 0.f);
  }
  if (f <= kEventFrame) {
    return Pose(0.05f, 0.5f, lost_y, 0.3f);
  }
  return Pose(0.05f, 0.5f + 0.002f * static_cast<float>(f - kEventFrame), 0.f, 0.3f);
}

struct Frame {
  bool reset = false;
  bool keyframe = false;
  std::vector<map::PointId> new_points;
  int live_keyframes = 0;
};

class Driver {
 public:
  Frame Step(int f, float lost_y) {
    MapBuilderOutput out;
    mb_.Update(&maps_, *MakeInput(f), MakeTrk(StateOf(f), 7 + f, PoseAt(f, lost_y)), f * 33, &out);
    Frame r;
    r.reset = out.map_reset;
    r.keyframe = out.keyframe_created;
    r.new_points = out.new_points;
    r.live_keyframes = maps_.active().NumKeyframes();
    return r;
  }
  Frame StepWith(int f, const MapTrackerOutput& trk) {
    MapBuilderOutput out;
    mb_.Update(&maps_, *MakeInput(f), trk, f * 33, &out);
    Frame r;
    r.reset = out.map_reset;
    r.keyframe = out.keyframe_created;
    r.new_points = out.new_points;
    r.live_keyframes = maps_.active().NumKeyframes();
    return r;
  }
  std::vector<Frame> Run(int from, int to, float lost_y = 0.f) {  // [from, to]
    std::vector<Frame> v;
    for (int f = from; f <= to; ++f) {
      v.push_back(Step(f, lost_y));
    }
    return v;
  }
  map::WorldMaps& maps() { return maps_; }

 private:
  map::WorldMaps maps_;
  MapBuilder mb_;
};

void ExpectSameFrame(const Frame& a, const Frame& b, int f) {
  EXPECT_EQ(a.reset, b.reset) << "frame " << f;
  EXPECT_EQ(a.keyframe, b.keyframe) << "frame " << f;
  EXPECT_EQ(a.live_keyframes, b.live_keyframes) << "frame " << f;
  ASSERT_EQ(a.new_points.size(), b.new_points.size()) << "frame " << f;
  for (size_t i = 0; i < a.new_points.size(); ++i) {
    EXPECT_TRUE(a.new_points[i] == b.new_points[i]) << "frame " << f << " point " << i;
  }
}

// ---- 1. predicate -------------------------------------------------------------------------------

TEST(K1Predicate, FalseWithAccuracyDisabled) {
  const ScopedK1 cfg(false, true);
  EXPECT_FALSE(KeepMapInsteadOfReset(3, Pose(0.f, 0.f, 0.f, 0.f)));
}

TEST(K1Predicate, FalseWithAccuracyEnabledButFlagOff) {
  const ScopedK1 cfg(true, false);
  EXPECT_FALSE(KeepMapInsteadOfReset(3, Pose(0.f, 0.f, 0.f, 0.f)));
}

TEST(K1Predicate, FalseForAnEmptyMap) {
  const ScopedK1 cfg(true, true);
  EXPECT_FALSE(KeepMapInsteadOfReset(0, Pose(0.f, 0.f, 0.f, 0.f)));
}

TEST(K1Predicate, TrueForANonEmptyMapAtNormalHeight) {
  const ScopedK1 cfg(true, true);
  EXPECT_TRUE(KeepMapInsteadOfReset(1, Pose(0.f, 0.f, 0.f, 0.f)));
  EXPECT_TRUE(KeepMapInsteadOfReset(3, Pose(0.f, 0.f, -0.9f, 0.f)));
  EXPECT_TRUE(KeepMapInsteadOfReset(3, Pose(0.f, 0.f, -0.99f, 0.f)));
  EXPECT_TRUE(KeepMapInsteadOfReset(3, Pose(0.f, 0.f, 0.5f, 0.f)));
}

TEST(K1Predicate, FalseWhenTheCameraIsBelowTheGroundPlane) {
  const ScopedK1 cfg(true, true);
  EXPECT_FALSE(KeepMapInsteadOfReset(3, Pose(0.f, 0.f, -1.2f, 0.f)));
}

// The comparison is made in double: -0.999f (= -0.99900001...) is below -0.999, so it resets.
TEST(K1Predicate, BoundaryFloatMinus0999IsBelowThePlaneAndMinus0998IsNot) {
  const ScopedK1 cfg(true, true);
  EXPECT_FALSE(KeepMapInsteadOfReset(3, Pose(0.f, 0.f, -0.999f, 0.f)));
  EXPECT_TRUE(KeepMapInsteadOfReset(3, Pose(0.f, 0.f, -0.998f, 0.f)));
}

// ---- 2. first map -------------------------------------------------------------------------------

// Empty map; the reset rule fires on frame 2 (frames 0 and 1 have too few consecutive successes).
struct FirstMap {
  Frame out;
  int keyframes = 0;
  std::array<int, 5> by_source{};
  std::vector<c8::HPoint3> positions;
};

FirstMap RunFirstMap() {
  const c8::HMatrix pose = Pose(0.2f, 0.1f, 0.3f, -0.4f);
  FirstMap r;
  map::WorldMaps maps;
  MapBuilder mb;
  for (int f = 0; f <= 2; ++f) {
    MapBuilderOutput out;
    mb.Update(&maps, *MakeInput(f), MakeTrk(0, f == 2 ? 7 : 3, pose), f * 33, &out);
    if (f < 2) {
      EXPECT_FALSE(out.map_reset) << "frame " << f;
      EXPECT_FALSE(out.keyframe_created) << "frame " << f;
      continue;
    }
    r.out.reset = out.map_reset;
    r.out.keyframe = out.keyframe_created;
    r.out.new_points = out.new_points;
    for (const map::PointId id : out.new_points) {
      r.positions.push_back(maps.active().Point(id).position);
    }
  }
  r.keyframes = maps.active().NumKeyframes();
  r.by_source = maps.active().NumPointsBySource();
  return r;
}

TEST(K1FirstMap, EmptyMapIsStillCreatedByTheResetPathWithTheFlagOn) {
  FirstMap off;
  FirstMap on;
  {
    const ScopedK1 cfg(true, false);
    off = RunFirstMap();
  }
  {
    const ScopedK1 cfg(true, true);
    on = RunFirstMap();
  }
  EXPECT_TRUE(on.out.reset);
  EXPECT_TRUE(on.out.keyframe);
  EXPECT_EQ(on.out.reset, off.out.reset);
  EXPECT_EQ(on.out.keyframe, off.out.keyframe);
  EXPECT_EQ(on.keyframes, 1);
  EXPECT_EQ(on.keyframes, off.keyframes);
  EXPECT_EQ(on.by_source, off.by_source);
  ASSERT_FALSE(on.positions.empty());
  ASSERT_EQ(on.positions.size(), off.positions.size());
  for (size_t i = 0; i < on.positions.size(); ++i) {
    EXPECT_EQ(on.positions[i].x(), off.positions[i].x()) << i;
    EXPECT_EQ(on.positions[i].y(), off.positions[i].y()) << i;
    EXPECT_EQ(on.positions[i].z(), off.positions[i].z()) << i;
  }
}

// ---- 3. non-empty map is retained ---------------------------------------------------------------

TEST(K1KeepMap, EventFrameKeepsTheMapAndAddsAKeyframeAtTheCurrentPose) {
  const ScopedK1 cfg(true, true);
  Driver d;
  const std::vector<Frame> before = d.Run(0, kEventFrame - 1);
  ASSERT_TRUE(before[0].reset);
  for (int f = 1; f < kEventFrame; ++f) {
    ASSERT_FALSE(before[static_cast<size_t>(f)].reset) << "frame " << f;
  }
  map::WorldMap& wm = d.maps().active();
  const int kfs_before = wm.NumKeyframes();
  ASSERT_EQ(kfs_before, 3);  // keyframes at frames 0, 5, 10
  const std::vector<map::PointId> ids_before = wm.AllPointIds();
  ASSERT_FALSE(ids_before.empty());
  std::vector<c8::HPoint3> pos_before;
  for (const map::PointId id : ids_before) {
    pos_before.push_back(wm.Point(id).position);
  }
  std::vector<map::KeyframeId> kf_before = wm.AllKeyframesByCreation();
  ASSERT_EQ(kf_before.size(), 3u);

  const Frame ev = d.Step(kEventFrame, 0.f);
  EXPECT_FALSE(ev.reset);
  EXPECT_TRUE(ev.keyframe);
  EXPECT_FALSE(ev.new_points.empty());
  EXPECT_EQ(ev.live_keyframes, kfs_before + 1);

  // The same map object keeps every earlier keyframe and point where it was.
  EXPECT_EQ(&d.maps().active(), &wm);
  for (const map::KeyframeId k : kf_before) {
    EXPECT_TRUE(wm.IsKeyframeValid(k));
  }
  for (size_t i = 0; i < ids_before.size(); ++i) {
    const map::WorldPoint* p = wm.FindPoint(ids_before[i]);
    ASSERT_NE(p, nullptr) << "point " << i;
    EXPECT_NE(p->state, map::PointState::kDead) << "point " << i;
    EXPECT_EQ(p->position.x(), pos_before[i].x()) << "point " << i;
    EXPECT_EQ(p->position.y(), pos_before[i].y()) << "point " << i;
    EXPECT_EQ(p->position.z(), pos_before[i].z()) << "point " << i;
  }

  // New ground points: on the plane y = -1 and on the camera rays of the current pose (not of the
  // first keyframe's pose).
  const c8::HMatrix now = PoseAt(kEventFrame, 0.f);
  const c8::HMatrix first = PoseAt(0, 0.f);
  int ground = 0;
  float max_dev_first = 0.f;
  for (const map::PointId id : ev.new_points) {
    const map::WorldPoint& p = wm.Point(id);
    if (p.source != map::PointSource::kGround) {
      continue;
    }
    ++ground;
    EXPECT_NEAR(p.position.y(), -1.0f, 1e-4f);
    const size_t obs = p.observations.begin()->second;
    const c8::FramePoint& fp = wm.GetKeyframe(p.observations.begin()->first).frame.points()[obs];
    const c8::HPoint3 c = now.inv() * p.position;
    ASSERT_GT(c.z(), 0.0f);
    EXPECT_NEAR(c.x() / c.z(), fp.x(), 1e-4f);
    EXPECT_NEAR(c.y() / c.z(), fp.y(), 1e-4f);
    const c8::HPoint3 c0 = first.inv() * p.position;
    max_dev_first = std::max(max_dev_first, std::fabs(c0.x() / c0.z() - fp.x()));
  }
  EXPECT_EQ(ground, kGround);
  EXPECT_GT(max_dev_first, 0.01f);  // the first keyframe's pose would give other rays
}

// A rejected relocalization leaves matches in trk.tracked on the (lost) event frame. They must not
// become observations of the extension keyframe (that would re-place old points); the tracking
// statistics of the visible ids are still counted, as on any lost frame.
TEST(K1KeepMap, RejectedRelocalizationMatchesAreNotAttachedToTheExtensionKeyframe) {
  const ScopedK1 cfg(true, true);
  Driver d;
  d.Run(0, kEventFrame - 1);
  map::WorldMap& wm = d.maps().active();
  std::vector<map::PointId> old_ground;
  for (const map::PointId id : wm.AllPointIds()) {
    if (wm.Point(id).source == map::PointSource::kGround) {
      old_ground.push_back(id);
    }
  }
  ASSERT_GE(old_ground.size(), 5u);
  old_ground.resize(5);
  const std::vector<map::PointId> all_before = wm.AllPointIds();
  std::vector<c8::HPoint3> pos_before;
  for (const map::PointId id : all_before) {
    pos_before.push_back(wm.Point(id).position);
  }
  std::vector<uint32_t> count_before;
  for (const map::PointId id : old_ground) {
    count_before.push_back(wm.Point(id).tracking_count);
  }

  MapTrackerOutput trk = MakeTrk(StateOf(kEventFrame), 7 + kEventFrame, PoseAt(kEventFrame, 0.f));
  for (size_t i = 0; i < old_ground.size(); ++i) {
    trk.tracked[static_cast<uint32_t>(i)] = old_ground[i];  // indices 0..4 are ground candidates
    trk.visible_ids.push_back(old_ground[i]);
    trk.visible_matched.push_back(1);
  }
  const Frame ev = d.StepWith(kEventFrame, trk);
  ASSERT_FALSE(ev.reset);
  ASSERT_TRUE(ev.keyframe);

  // (a) old points did not move.
  for (size_t i = 0; i < all_before.size(); ++i) {
    const map::WorldPoint& p = wm.Point(all_before[i]);
    EXPECT_EQ(p.position.x(), pos_before[i].x()) << "point " << i;
    EXPECT_EQ(p.position.y(), pos_before[i].y()) << "point " << i;
    EXPECT_EQ(p.position.z(), pos_before[i].z()) << "point " << i;
  }
  // (b) the new keyframe observes none of them; its frame points there are unlinked or new.
  const map::KeyframeId newest = wm.newest();
  for (const map::PointId id : old_ground) {
    EXPECT_EQ(wm.Point(id).observations.count(newest), 0u);
  }
  for (uint32_t i = 0; i < old_ground.size(); ++i) {
    const map::PointId linked = wm.GetKeyframe(newest).frame.points()[i].id();
    const bool unlinked = linked == map::PointId::nullId();
    const bool is_new = std::find(ev.new_points.begin(), ev.new_points.end(), linked) !=
                        ev.new_points.end();
    EXPECT_TRUE(unlinked || is_new) << "index " << i;
  }
  // (c) every ground candidate became a new ground point, the "matched" ones included.
  int ground = 0;
  for (const map::PointId id : ev.new_points) {
    ground += wm.Point(id).source == map::PointSource::kGround ? 1 : 0;
  }
  EXPECT_EQ(ground, kGround);
  // (d) the tracking statistics of the visible ids were still counted.
  for (size_t i = 0; i < old_ground.size(); ++i) {
    EXPECT_EQ(wm.Point(old_ground[i]).tracking_count, count_before[i] + 1) << i;
  }
}

// ---- 4. flag off resets as before ---------------------------------------------------------------

TEST(K1FlagOff, ResetsOnTheSameFrameAndMatchesTheOnRunBeforeTheEvent) {
  std::vector<Frame> on_frames;
  {
    const ScopedK1 cfg(true, true);
    Driver d;
    on_frames = d.Run(0, kEventFrame);
  }
  for (const bool enabled : {false, true}) {
    const ScopedK1 cfg(enabled, false);
    Driver d;
    const std::vector<Frame> off_frames = d.Run(0, kEventFrame);
    // Identical until the event.
    for (int f = 0; f < kEventFrame; ++f) {
      ExpectSameFrame(off_frames[static_cast<size_t>(f)], on_frames[static_cast<size_t>(f)], f);
    }
    // The OFF run resets exactly where the ON run extends.
    const Frame& off_ev = off_frames[static_cast<size_t>(kEventFrame)];
    const Frame& on_ev = on_frames[static_cast<size_t>(kEventFrame)];
    EXPECT_TRUE(off_ev.reset) << "enabled=" << enabled;
    EXPECT_TRUE(off_ev.keyframe) << "enabled=" << enabled;
    EXPECT_FALSE(on_ev.reset);
    EXPECT_TRUE(on_ev.keyframe);
    EXPECT_EQ(off_ev.live_keyframes, 1) << "enabled=" << enabled;
    EXPECT_EQ(on_ev.live_keyframes, 4);
    // No reset anywhere else in the OFF run (the event frame is the first one after frame 0).
    for (int f = 1; f < kEventFrame; ++f) {
      EXPECT_FALSE(off_frames[static_cast<size_t>(f)].reset) << "frame " << f;
    }
  }
}

TEST(K1FlagOff, ResetDiscardsTheOldPoints) {
  const ScopedK1 cfg(true, false);
  Driver d;
  d.Run(0, kEventFrame - 1);
  map::WorldMap& wm = d.maps().active();
  const std::vector<map::PointId> old_ids = wm.AllPointIds();
  const std::vector<map::KeyframeId> old_kfs = wm.AllKeyframesByCreation();
  ASSERT_FALSE(old_ids.empty());
  const Frame ev = d.Step(kEventFrame, 0.f);
  ASSERT_TRUE(ev.reset);
  map::WorldMap& after = d.maps().active();
  for (const map::KeyframeId k : old_kfs) {
    EXPECT_FALSE(after.IsKeyframeValid(k));
  }
  for (const map::PointId id : old_ids) {
    const map::WorldPoint* p = after.FindPoint(id);
    const bool gone = p == nullptr || p->state == map::PointState::kDead;
    const bool recycled = p != nullptr && p->state != map::PointState::kDead &&
                          std::find(ev.new_points.begin(), ev.new_points.end(), id) !=
                              ev.new_points.end();
    EXPECT_TRUE(gone || recycled);
  }
  EXPECT_EQ(after.AllPointIds().size(), ev.new_points.size());
}

// ---- 5. camera at/below the ground plane --------------------------------------------------------

TEST(K1KeepMap, CameraBelowThePlaneStillResetsWithTheFlagOn) {
  const ScopedK1 cfg(true, true);
  Driver d;
  const std::vector<Frame> frames = d.Run(0, kEventFrame, -1.2f);
  for (int f = 1; f < kEventFrame; ++f) {
    EXPECT_FALSE(frames[static_cast<size_t>(f)].reset) << "frame " << f;
  }
  const Frame& ev = frames[static_cast<size_t>(kEventFrame)];
  EXPECT_TRUE(ev.reset);
  EXPECT_TRUE(ev.keyframe);
  EXPECT_EQ(ev.live_keyframes, 1);
}

// ---- 6. keyframe timing after the event ---------------------------------------------------------

// A keyframe is wanted (no tracked points) every 5 frames from frame 24 on in state 1; the
// relocalization hold (30 frames after the last seeding from a lost state, while the state change
// is younger than 30) blocks 49 only when last_reset_frame_ is the event frame. If it were still the
// frame-0 reset, the hold would already apply from frame 30 and swallow the keyframes 34, 39, 44.
TEST(K1KeepMap, KeyframeTimingAfterTheEventEqualsTheTimingAfterAReset) {
  std::vector<Frame> on_frames;
  std::vector<Frame> off_frames;
  {
    const ScopedK1 cfg(true, true);
    Driver d;
    on_frames = d.Run(0, kEventFrame + 40);
  }
  {
    const ScopedK1 cfg(true, false);
    Driver d;
    off_frames = d.Run(0, kEventFrame + 40);
  }
  ASSERT_EQ(on_frames.size(), off_frames.size());
  int keyframes_between = 0;
  for (int f = kEventFrame + 1; f <= kEventFrame + 40; ++f) {
    const size_t i = static_cast<size_t>(f);
    EXPECT_EQ(on_frames[i].keyframe, off_frames[i].keyframe) << "frame " << f;
    EXPECT_FALSE(on_frames[i].reset) << "frame " << f;
    if (f >= 31 && f <= 48) {
      keyframes_between += on_frames[i].keyframe ? 1 : 0;
    }
  }
  // Not vacuous: keyframes do occur inside the window that a stale last_reset_frame_ would hold.
  EXPECT_GE(keyframes_between, 3);
  EXPECT_TRUE(off_frames[34].keyframe);
  EXPECT_TRUE(on_frames[34].keyframe);
}


// ---- K1F: k1_fast_extend ------------------------------------------------------------------------

class ScopedK1F {
 public:
  ScopedK1F(bool enabled, bool k1, bool k1f) : previous_(config::Accuracy()) {
    AccuracyConfig cfg;
    cfg.enabled = enabled;
    cfg.k1_keep_map = k1;
    cfg.k1_fast_extend = k1f;
    config::SetAccuracy(cfg);
  }
  ~ScopedK1F() { config::SetAccuracy(previous_); }
  ScopedK1F(const ScopedK1F&) = delete;
  ScopedK1F& operator=(const ScopedK1F&) = delete;

 private:
  AccuracyConfig previous_;
};

ResetInputs BaseInputs() {
  ResetInputs in;
  in.state = 0;
  in.frame_tracker_ok = true;
  in.num_candidates = 24;
  in.frame_count = 20;
  in.last_state_change_frame = 10;  // 10 frames since the change (> 5)
  in.keyframe_slots = 8;
  in.consecutive_frame_tracker_ok = 7;
  return in;
}

const c8::HMatrix kNormalPose = Pose(0.f, 0.f, 0.f, 0.f);

TEST(K1FPredicate, FalseWhenAnyFlagOrAccuracyIsMissing) {
  {
    const ScopedK1F cfg(false, true, true);
    EXPECT_FALSE(ExtendMapEarly(BaseInputs(), kNormalPose));
  }
  {
    const ScopedK1F cfg(true, true, false);
    EXPECT_FALSE(ExtendMapEarly(BaseInputs(), kNormalPose));
  }
  {
    const ScopedK1F cfg(true, false, true);
    EXPECT_FALSE(ExtendMapEarly(BaseInputs(), kNormalPose));
  }
}

TEST(K1FPredicate, FalseForAnEmptyMap) {
  const ScopedK1F cfg(true, true, true);
  ResetInputs in = BaseInputs();
  in.keyframe_slots = 0;
  EXPECT_FALSE(ExtendMapEarly(in, kNormalPose));
}

TEST(K1FPredicate, UsesTheSmallMapSuccessCountForALargeMap) {
  const ScopedK1F cfg(true, true, true);
  ResetInputs in = BaseInputs();
  in.consecutive_frame_tracker_ok = 6;
  EXPECT_FALSE(ExtendMapEarly(in, kNormalPose));
  in.consecutive_frame_tracker_ok = 7;
  EXPECT_TRUE(ExtendMapEarly(in, kNormalPose));
  in.consecutive_frame_tracker_ok = 19;
  EXPECT_TRUE(ExtendMapEarly(in, kNormalPose));
}

TEST(K1FPredicate, KeepsTheOtherConditionsOfTheResetRule) {
  const ScopedK1F cfg(true, true, true);
  ResetInputs in = BaseInputs();
  in.frame_count = 15;  // 5 frames since the change: not more than numFramesToResetNonEmptyMap
  EXPECT_FALSE(ExtendMapEarly(in, kNormalPose));
  in = BaseInputs();
  in.state = 1;
  EXPECT_FALSE(ExtendMapEarly(in, kNormalPose));
  in = BaseInputs();
  in.frame_tracker_ok = false;
  EXPECT_FALSE(ExtendMapEarly(in, kNormalPose));
  in = BaseInputs();
  in.num_candidates = 19;
  EXPECT_FALSE(ExtendMapEarly(in, kNormalPose));
}

TEST(K1FPredicate, FalseWhenTheCameraIsBelowTheGroundPlane) {
  const ScopedK1F cfg(true, true, true);
  EXPECT_TRUE(ExtendMapEarly(BaseInputs(), kNormalPose));
  EXPECT_FALSE(ExtendMapEarly(BaseInputs(), Pose(0.f, 0.f, -1.2f, 0.f)));
}

// A map with 6 keyframe slots (tracking frames 0..28), then lost frames 29.. whose consecutive
// frame-tracker success count is given per lost frame j (frame 29 + j).
struct BigRun {
  std::vector<Frame> lost;     // outputs of the lost frames
  std::vector<Frame> after;    // state-1 frames after the lost phase
  size_t slots_before = 0;
  int live_before = 0;
};

enum class Mode { kOff, kK1, kK1F };

template <typename ConsecFn>
BigRun RunBig(Mode mode, float lost_y, ConsecFn consec, int lost_frames, int post_frames) {
  const ScopedK1F cfg(true, mode != Mode::kOff, mode == Mode::kK1F);
  map::WorldMaps maps;
  MapBuilder mb;
  BigRun r;
  auto step = [&](int f, const MapTrackerOutput& trk) {
    MapBuilderOutput out;
    mb.Update(&maps, *MakeInput(f), trk, f * 33, &out);
    Frame fr;
    fr.reset = out.map_reset;
    fr.keyframe = out.keyframe_created;
    fr.new_points = out.new_points;
    fr.live_keyframes = maps.active().NumKeyframes();
    return fr;
  };
  for (int f = 0; f <= 28; ++f) {
    step(f, MakeTrk(f == 0 ? 0 : 1, 7 + f,
                    Pose(0.002f * static_cast<float>(f), 0.01f * static_cast<float>(f), 0.f, 0.f)));
  }
  r.slots_before = map::KeyframeSlotCount(maps.active());
  r.live_before = maps.active().NumKeyframes();
  for (int j = 0; j < lost_frames; ++j) {
    r.lost.push_back(step(29 + j, MakeTrk(0, consec(j), Pose(0.05f, 0.5f, lost_y, 0.3f))));
  }
  for (int k = 0; k < post_frames; ++k) {
    const int f = 29 + lost_frames + k;
    r.after.push_back(step(f, MakeTrk(1, 7 + k, Pose(0.05f, 0.5f + 0.002f * static_cast<float>(k + 1),
                                                       0.f, 0.3f))));
  }
  return r;
}

// Consecutive count 1, 2, 3, ... on lost frame j = 0, 1, 2, ...
int CountUp(int j) { return j + 1; }

// First lost frame (as its consecutive count) on which a reset or a keyframe happens, 0 = none.
int FirstEventConsecutive(const BigRun& r) {
  for (size_t j = 0; j < r.lost.size(); ++j) {
    if (r.lost[j].reset || r.lost[j].keyframe) {
      return CountUp(static_cast<int>(j));
    }
  }
  return 0;
}

TEST(K1FUpdate, OffResetsOnlyAtTwentyConsecutiveSuccessesForALargeMap) {
  const BigRun r = RunBig(Mode::kOff, 0.f, CountUp, 25, 0);
  ASSERT_GT(r.slots_before, 5u);
  EXPECT_EQ(FirstEventConsecutive(r), 20);
  EXPECT_TRUE(r.lost[19].reset);
  EXPECT_EQ(r.lost[19].live_keyframes, 1);
}

TEST(K1FUpdate, K1OnlyExtendsAtTwentyConsecutiveSuccessesForALargeMap) {
  const BigRun r = RunBig(Mode::kK1, 0.f, CountUp, 25, 0);
  ASSERT_GT(r.slots_before, 5u);
  EXPECT_EQ(FirstEventConsecutive(r), 20);
  EXPECT_FALSE(r.lost[19].reset);
  EXPECT_TRUE(r.lost[19].keyframe);
  EXPECT_EQ(r.lost[19].live_keyframes, r.live_before + 1);
}

TEST(K1FUpdate, K1FExtendsAtSevenConsecutiveSuccessesForALargeMap) {
  const BigRun r = RunBig(Mode::kK1F, 0.f, CountUp, 25, 0);
  ASSERT_GT(r.slots_before, 5u);
  const int first = FirstEventConsecutive(r);
  EXPECT_EQ(first, 7);  // 7 successes and 6 (> 5) lost frames
  ASSERT_GE(first, 1);
  EXPECT_LT(first, 20);
  const Frame& ev = r.lost[static_cast<size_t>(first - 1)];
  EXPECT_FALSE(ev.reset);
  EXPECT_TRUE(ev.keyframe);
  EXPECT_FALSE(ev.new_points.empty());
  EXPECT_EQ(ev.live_keyframes, r.live_before + 1);
}

TEST(K1FUpdate, K1FBelowThePlaneStillWaitsForTheOriginalReset) {
  const BigRun r = RunBig(Mode::kK1F, -1.2f, CountUp, 25, 0);
  ASSERT_GT(r.slots_before, 5u);
  EXPECT_EQ(FirstEventConsecutive(r), 20);
  EXPECT_TRUE(r.lost[19].reset);
}

// ---- K1F on a small map ---------------------------------------------------------------------------

TEST(K1FUpdate, SmallMapGivesTheSameOutputsAsK1Only) {
  std::vector<Frame> k1;
  std::vector<Frame> k1f;
  {
    const ScopedK1F cfg(true, true, false);
    Driver d;
    k1 = d.Run(0, kEventFrame + 40);
  }
  {
    const ScopedK1F cfg(true, true, true);
    Driver d;
    k1f = d.Run(0, kEventFrame + 40);
  }
  ASSERT_EQ(k1.size(), k1f.size());
  for (size_t f = 0; f < k1.size(); ++f) {
    ExpectSameFrame(k1[f], k1f[f], static_cast<int>(f));
  }
}

// ---- K1F keyframe timing --------------------------------------------------------------------------

// The OFF run is given 20 consecutive successes on the same lost frame (7th) so that it resets on
// the very frame where K1F extends; every later input is identical. The keyframe timing must then
// be the same, i.e. K1F set last_reset_frame_ like the reset does.
TEST(K1FUpdate, KeyframeTimingAfterAFastExtensionEqualsTheTimingAfterAReset) {
  auto consec_k1f = [](int j) { return j + 1; };
  auto consec_off = [](int j) { return j == 6 ? 20 : j + 1; };
  const BigRun k1f = RunBig(Mode::kK1F, 0.f, consec_k1f, 7, 40);
  const BigRun off = RunBig(Mode::kOff, 0.f, consec_off, 7, 40);
  ASSERT_GT(k1f.slots_before, 5u);
  ASSERT_TRUE(k1f.lost[6].keyframe);
  ASSERT_FALSE(k1f.lost[6].reset);
  ASSERT_TRUE(off.lost[6].reset);
  int keyframes_in_window = 0;
  for (size_t k = 0; k < k1f.after.size(); ++k) {
    EXPECT_EQ(k1f.after[k].keyframe, off.after[k].keyframe) << "post frame " << k;
    EXPECT_FALSE(k1f.after[k].reset) << "post frame " << k;
    keyframes_in_window += k1f.after[k].keyframe ? 1 : 0;
  }
  EXPECT_GE(keyframes_in_window, 5);
}


// ---- pinned behaviour: repeated extensions (code re-review H1) ----------------------------------------
// Documents the current behaviour, it does not approve it: the builder can see state 0 on two
// consecutive frames (the tracker enters state 1 and loses the map again within one Track call), so
// last_state_change_frame_ does not move while the success counter keeps counting.

// The 6-slot map of RunBig (tracking frames 0..28), then caller-driven frames.
class BigWorld {
 public:
  explicit BigWorld(const AccuracyConfig& cfg) {
    config::SetAccuracy(cfg);
    for (int f = 0; f <= 28; ++f) {
      Step(f, MakeTrk(f == 0 ? 0 : 1, 7 + f,
                      Pose(0.002f * static_cast<float>(f), 0.01f * static_cast<float>(f), 0.f, 0.f)));
    }
  }
  Frame Step(int f, const MapTrackerOutput& trk) {
    MapBuilderOutput out;
    mb_.Update(&maps_, *MakeInput(f), trk, f * 33, &out);
    Frame fr;
    fr.reset = out.map_reset;
    fr.keyframe = out.keyframe_created;
    fr.new_points = out.new_points;
    fr.live_keyframes = maps_.active().NumKeyframes();
    return fr;
  }
  Frame Lost(int f, int64_t consecutive) {
    return Step(f, MakeTrk(0, consecutive, Pose(0.05f, 0.5f, 0.f, 0.3f)));
  }
  Frame Tracking(int f) { return Step(f, MakeTrk(1, 40, Pose(0.05f, 0.52f, 0.f, 0.3f))); }
  map::WorldMaps& maps() { return maps_; }

 private:
  map::WorldMaps maps_;
  MapBuilder mb_;
};

constexpr int kFirstLostFrame = 29;

// Live points only (AllPointIds also lists dead ones, whose pool slots are recycled).
std::vector<map::PointId> LivePoints(const map::WorldMap& wm) {
  std::vector<map::PointId> v;
  for (const map::PointId id : wm.AllPointIds()) {
    if (wm.Point(id).state != map::PointState::kDead) {
      v.push_back(id);
    }
  }
  return v;
}

// Marks every point of the map as tenured with a perfect hit record, as real tracking would have
// done. Untracked NEW points are culled by age on every keyframe (original behaviour), which would
// otherwise hide whether an extension itself disturbs the old map.
void TenureAll(map::WorldMap& wm) {
  for (const map::PointId id : wm.AllPointIds()) {
    map::WorldPoint& p = wm.MutablePoint(id);
    if (p.state == map::PointState::kDead) {
      continue;
    }
    p.state = map::PointState::kTenured;
    p.tracking_count = 100;
    p.tracking_hits = 100;
  }
}


AccuracyConfig Cfg(bool k1, bool k1f) {
  AccuracyConfig cfg;
  cfg.enabled = true;
  cfg.k1_keep_map = k1;
  cfg.k1_fast_extend = k1f;
  return cfg;
}

// Lost frames 29.. with count(j); the first extension is on j = 6; then 5 more lost frames.
void ExpectExtensionRepeats(bool k1f, int64_t (*count)(int)) {
  const AccuracyConfig previous = config::Accuracy();
  {
    BigWorld w(Cfg(true, k1f));
    map::WorldMap& wm = w.maps().active();
    ASSERT_GT(map::KeyframeSlotCount(wm), 5u);
    TenureAll(wm);
    for (int j = 0; j < 6; ++j) {
      const Frame fr = w.Lost(kFirstLostFrame + j, count(j));
      ASSERT_FALSE(fr.reset || fr.keyframe) << "j " << j;
    }
    // Snapshot right before the first extension (ordinary culling of the lost frames before it
    // belongs to the original behaviour).
    const int live0 = wm.NumKeyframes();
    const std::vector<map::PointId> ids = LivePoints(wm);
    const std::vector<map::KeyframeId> kfs = wm.AllKeyframesByCreation();
    for (int j = 6; j <= 11; ++j) {  // the first extension and 5 repeats
      const Frame fr = w.Lost(kFirstLostFrame + j, count(j));
      EXPECT_TRUE(fr.keyframe) << "j " << j;
      EXPECT_FALSE(fr.reset) << "j " << j;
      EXPECT_EQ(fr.live_keyframes, live0 + (j - 5)) << "j " << j;
    }
    for (const map::KeyframeId k : kfs) {
      EXPECT_TRUE(wm.IsKeyframeValid(k));
    }
    for (const map::PointId id : ids) {
      const map::WorldPoint* p = wm.FindPoint(id);
      ASSERT_NE(p, nullptr);
      EXPECT_NE(p->state, map::PointState::kDead);
    }
  }
  config::SetAccuracy(previous);
}

int64_t CountUpK1F(int j) { return j + 1; }       // 7 successes on j = 6
int64_t CountHighK1(int j) { return 20 + j; }     // already >= 20

TEST(K1FUpdate, ExtensionRepeatsEveryFrameWhileTheBuilderKeepsSeeingStateZero) {
  ExpectExtensionRepeats(true, CountUpK1F);   // K1 + K1F
  ExpectExtensionRepeats(false, CountHighK1);  // K1 only, counter >= 20
}

TEST(K1FUpdate, OneTrackedFrameAfterAnExtensionDelaysTheNextExtension) {
  const AccuracyConfig previous = config::Accuracy();
  {
    BigWorld w(Cfg(true, true));
    int f = kFirstLostFrame;
    for (int j = 0; j < 6; ++j) {
      ASSERT_FALSE(w.Lost(f++, j + 1).keyframe);
    }
    const int event = f;
    ASSERT_TRUE(w.Lost(f++, 7).keyframe);  // the extension (frame 35)
    const Frame tracked = w.Tracking(f++);  // state 1 at event + 1
    EXPECT_FALSE(tracked.reset);
    EXPECT_FALSE(tracked.keyframe);
    // State 0 again from event + 2 (the state change moves there), counter still high: nothing
    // until more than numFramesToResetNonEmptyMap (5) frames have passed since that change, i.e.
    // up to event + 7; the next extension is on event + 8.
    for (; f < event + 8; ++f) {
      const Frame fr = w.Lost(f, 30 + f);
      EXPECT_FALSE(fr.reset) << "frame " << f;
      EXPECT_FALSE(fr.keyframe) << "frame " << f;
    }
    const Frame next = w.Lost(f, 30 + f);
    EXPECT_EQ(f, event + 8);
    EXPECT_FALSE(next.reset);
    EXPECT_TRUE(next.keyframe);
  }
  config::SetAccuracy(previous);
}

// The rejected-relocalization matches are dropped on a K1F extension of a large map as well.
TEST(K1FUpdate, RejectedRelocalizationMatchesAreNotAttachedToAFastExtensionKeyframe) {
  const AccuracyConfig previous = config::Accuracy();
  {
    BigWorld w(Cfg(true, true));
    map::WorldMap& wm = w.maps().active();
    ASSERT_GT(map::KeyframeSlotCount(wm), 5u);
    TenureAll(wm);
    for (int j = 0; j < 6; ++j) {
      ASSERT_FALSE(w.Lost(kFirstLostFrame + j, j + 1).keyframe);
    }
    std::vector<map::PointId> old_ground;
    for (const map::PointId id : LivePoints(wm)) {
      if (wm.Point(id).source == map::PointSource::kGround) {
        old_ground.push_back(id);
      }
    }
    ASSERT_GE(old_ground.size(), 5u);
    old_ground.resize(5);
    const std::vector<map::PointId> all_before = LivePoints(wm);
    std::vector<c8::HPoint3> pos_before;
    for (const map::PointId id : all_before) {
      pos_before.push_back(wm.Point(id).position);
    }
    std::vector<uint32_t> count_before;
    for (const map::PointId id : old_ground) {
      count_before.push_back(wm.Point(id).tracking_count);
    }
    MapTrackerOutput trk = MakeTrk(0, 7, Pose(0.05f, 0.5f, 0.f, 0.3f));
    for (size_t i = 0; i < old_ground.size(); ++i) {
      trk.tracked[static_cast<uint32_t>(i)] = old_ground[i];
      trk.visible_ids.push_back(old_ground[i]);
      trk.visible_matched.push_back(1);
    }
    const Frame ev = w.Step(kFirstLostFrame + 6, trk);
    ASSERT_FALSE(ev.reset);
    ASSERT_TRUE(ev.keyframe);
    for (size_t i = 0; i < all_before.size(); ++i) {
      const map::WorldPoint& p = wm.Point(all_before[i]);
      EXPECT_EQ(p.position.x(), pos_before[i].x()) << "point " << i;
      EXPECT_EQ(p.position.y(), pos_before[i].y()) << "point " << i;
      EXPECT_EQ(p.position.z(), pos_before[i].z()) << "point " << i;
    }
    const map::KeyframeId newest = wm.newest();
    for (const map::PointId id : old_ground) {
      EXPECT_EQ(wm.Point(id).observations.count(newest), 0u);
    }
    int ground = 0;
    for (const map::PointId id : ev.new_points) {
      ground += wm.Point(id).source == map::PointSource::kGround ? 1 : 0;
    }
    EXPECT_EQ(ground, kGround);
    for (size_t i = 0; i < old_ground.size(); ++i) {
      EXPECT_EQ(wm.Point(old_ground[i]).tracking_count, count_before[i] + 1) << i;
    }
  }
  config::SetAccuracy(previous);
}

}  // namespace
}  // namespace slam_refact::map_builder

// ---- 7. tracker side ----------------------------------------------------------------------------

namespace slam_refact::map_tracker {
namespace {

namespace mtt = slam_refact::map_tracker::testing;

constexpr int64_t kFrameNs = 50000000;

class K1TrackerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    world_ = mtt::GridPoints(12, 9);
    desc_ = mtt::RandomDescriptors(world_.size(), 21);
    mtt::BuildScene(&scene_, world_, desc_, c8::HMatrixGen::i(), map::PointSource::kDepth);
  }

  // A keyframe created from the lost state (the K1 event: map_reset false), then one Track call
  // with the camera (tracker pose and observations) at height y0.
  MapTrackerOutput TrackAfterKeyframe(float y0) {
    MapTrackerAccess::Impl& impl = MapTrackerAccess::impl(tracker_);
    const c8::HMatrix truth = c8::HMatrixGen::translation(0.f, y0, 0.f);
    impl.pose = truth;
    MapBuilderOutput ev;
    ev.keyframe_created = true;
    ev.map_reset = false;
    for (const map::PointId id : scene_.ids) {
      if (!(id == map::PointId())) {
        ev.new_points.push_back(id);
        if (ev.new_points.size() == 5) break;
      }
    }
    tracker_.ApplyMapBuilderOutput(ev);
    EXPECT_TRUE(impl.map_initialized_pending);
    TrackerInput in = mtt::MakeInput(world_, desc_, truth, 0);
    FrameState pred;
    pred.pose = tracker_.pose();
    pred.time_ns = kFrameNs;
    pred.xr_device_rotation = c8::rotation(truth);
    MapTrackerOutput out;
    tracker_.Track(scene_.map, &in, pred, &out, &rng_);
    return out;
  }

  std::vector<c8::HPoint3> world_;
  std::vector<mtt::Descriptor> desc_;
  mtt::Scene scene_;
  MapTracker tracker_;
  compat::RandomNumbers rng_{compat::kEngineRngSeed};
};

TEST_F(K1TrackerTest, KeyframeFromTheLostStateEntersTrackingWithoutReBasingANormalHeight) {
  const MapTrackerOutput out = TrackAfterKeyframe(-0.5f);
  EXPECT_EQ(out.state, 1);
  EXPECT_NEAR(out.pose(1, 3), -0.5f, 0.01f);  // not re-based to 0
}

TEST_F(K1TrackerTest, KeyframeFromTheLostStateReBasesAPoseBelowThePlane) {
  const MapTrackerOutput out = TrackAfterKeyframe(-1.2f);
  // The original re-bases the pose height to 0 here although the camera (and the kept map) is at
  // -1.2: the pose no longer fits the map and tracking drops back to LOST. This is why K1 leaves
  // this case to the reset.
  EXPECT_NEAR(out.pose(1, 3), 0.0f, 1e-3f);
  EXPECT_EQ(out.state, 0);
}

}  // namespace
}  // namespace slam_refact::map_tracker
