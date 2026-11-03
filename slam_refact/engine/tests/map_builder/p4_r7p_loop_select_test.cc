// P4 candidate R7P (AccuracyConfig::r7p_loop_select): the loop close candidate selection judges every
// distance-list entry by its own slot (loop_close.cc FindTarget S5). Compiled only with
// SLAM_REFACT_LOOP_CLOSE=ON. Synthetic scenes only (no registered fixture is run here).
//
// Design: build/xr-drift-p4-20261103/lc-selfloop-analysis.md plan A and lc-selfloop-review2.md.
// Pre-declared expectations (fixed before the first run):
//  - with R7 and R7P on the newest keyframe is never the candidate, whatever free slots precede it;
//  - with R7P off (R7 on) the original selection is kept, i.e. it selects the newest keyframe itself
//    as soon as a free slot precedes it, and the first keyframe when none does;
//  - S2 (the 2-hop covisible exclusion) is applied to the slot of the entry, not to its list position;
//  - R7P needs R7 and `enabled`: otherwise the selection is the original one and nothing is printed;
//  - each accepted loop prints one `[r7p-loop]` line to stderr, nothing else changes.

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <random>
#include <regex>
#include <string>
#include <vector>

#include "c8/hmatrix.h"
#include "c8/hpoint.h"
#include "config/accuracy.h"
#include "gtest/gtest.h"
#include "map/world_map.h"
#include "map_builder/loop_close.h"
#include "map_builder_test_util.h"
#include "reality/engine/features/frame-point.h"
#include "reality/engine/features/image-descriptor.h"

namespace slam_refact::map_builder {
namespace {

// G1 binds its gtest XML to the binary under test (see p3_r7_pose_graph_test.cc).
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

using testing::Pose;
using testing::Project;
using testing::TestIntrinsics;

using Desc = std::array<uint8_t, 32>;

class ScopedAccuracy {
 public:
  ScopedAccuracy(bool enabled, bool r7, bool r7p) : previous_(config::Accuracy()) {
    AccuracyConfig cfg;
    cfg.enabled = enabled;
    cfg.r7_pose_graph = r7;
    cfg.r7p_loop_select = r7p;
    config::SetAccuracy(cfg);
  }
  ~ScopedAccuracy() { config::SetAccuracy(previous_); }
  ScopedAccuracy(const ScopedAccuracy&) = delete;
  ScopedAccuracy& operator=(const ScopedAccuracy&) = delete;

 private:
  AccuracyConfig previous_;
};

c8::FeatureSet OrbSet(const Desc& d) {
  c8::FeatureSet set;
  set.orbFeature = c8::OrbFeature(d);
  return set;
}

// A global descriptor that is `bits` bits away from the all-zero descriptor of the newest keyframe.
Desc DescriptorAtDistance(int bits) {
  Desc d{};
  for (int b = 0; b < bits; ++b) {
    d[static_cast<size_t>(b / 8)] = static_cast<uint8_t>(d[static_cast<size_t>(b / 8)] | (1u << (b % 8)));
  }
  return d;
}

// The loop scene of p3_r7_pose_graph_test.cc (candidate keyframe sees `n` world points, four
// keyframes follow, the newest keyframe sees the same appearances from another pose), with
//  - slot layout: first = slot 0, middle[k] = slot k + 1, newest = slot 5;
//  - the newest keyframe's global descriptor all zero, so a distance is a bit count;
//  - first at `first_bits`, every middle keyframe at 256 bits unless set in `middle_bits`;
//  - the middle keyframes in `remove` (indices 0..3) removed again, which leaves a free slot in
//    front of the newest keyframe;
//  - `covisible` (middle index or -1): 12 world points seen by that middle keyframe and by the
//    newest one, which makes it a 1-hop covisible neighbour of the newest keyframe (S2).
struct Scene {
  map::WorldMaps maps;
  map::KeyframeId first;
  std::vector<map::KeyframeId> middle;
  map::KeyframeId newest;

  Scene(int first_bits, const std::vector<int>& middle_bits, const std::vector<int>& remove,
        int covisible = -1) {
    map::WorldMap& w = maps.active();
    const c8::HMatrix pose_first = Pose(0.0f, 0.0f, 0.0f, 0.0f);
    const c8::HMatrix pose_true = Pose(0.01f, 0.03f, 0.005f, 0.02f);
    const c8::HMatrix pose_stored = c8::HMatrixGen::translation(0.04f, -0.01f, 0.03f) *
                                    c8::HMatrixGen::xRadians(0.31f) * c8::HMatrixGen::yRadians(-0.52f) *
                                    c8::HMatrixGen::zRadians(0.17f);
    constexpr int kPoints = 80;
    c8::FrameWithPoints fa(TestIntrinsics());
    c8::FrameWithPoints fb(TestIntrinsics());
    std::vector<c8::HPoint3> truth;
    for (int i = 0; i < kPoints; ++i) {
      const c8::HPoint3 x(-1.2f + 0.03f * static_cast<float>(i),
                          -0.5f + 0.03f * static_cast<float>((i * 7) % 30),
                          2.0f + 0.06f * static_cast<float>((i * 11) % 30));
      truth.push_back(x);
      const Desc d = testing::Descriptor(static_cast<uint32_t>(100 + i));
      fa.addImageRayPoint(Project(pose_first, x), 1, 0.25f, 0.5f, OrbSet(d));
      fb.addImageRayPoint(Project(pose_true, x), 1, 0.25f, 0.5f, OrbSet(d));
    }
    first = w.AddKeyframe(fa, 0);
    w.SetKeyframePose(first, pose_first);
    w.MutableKeyframe(first).global_descriptor = DescriptorAtDistance(first_bits);
    for (int i = 0; i < kPoints; ++i) {
      const map::PointId p =
          w.AddPoint(w.GetKeyframe(first).seq, truth[static_cast<size_t>(i)], map::PointSource::kTriangulated);
      w.AddObservation(p, static_cast<uint32_t>(i), first);
    }
    for (int k = 1; k <= 4; ++k) {
      c8::FrameWithPoints ff(TestIntrinsics());
      for (int i = 0; i < 40; ++i) {
        ff.addImageRayPoint(c8::HPoint2(-0.5f + 0.025f * static_cast<float>(i), 0.1f * static_cast<float>(k) - 0.2f),
                            1, 0.25f, 0.5f, OrbSet(testing::Descriptor(static_cast<uint32_t>(1000 * k + i))));
      }
      const map::KeyframeId kf = w.AddKeyframe(ff, 5 * k);
      w.SetKeyframePose(kf, Pose(0.0f, 0.1f * static_cast<float>(k), 0.f, 0.f));
      const int bits = static_cast<size_t>(k - 1) < middle_bits.size() ? middle_bits[static_cast<size_t>(k - 1)] : 256;
      if (bits >= 256) {
        w.MutableKeyframe(kf).global_descriptor.fill(0xFF);
      } else {
        w.MutableKeyframe(kf).global_descriptor = DescriptorAtDistance(bits);
      }
      middle.push_back(kf);
    }
    newest = w.AddKeyframe(fb, 30);
    w.SetKeyframePose(newest, pose_stored);  // descriptor stays all zero
    if (covisible >= 0) {
      for (uint32_t i = 0; i < 12; ++i) {
        const map::PointId p = w.AddPoint(w.GetKeyframe(first).seq,
                                          c8::HPoint3(0.1f * static_cast<float>(i), 0.2f, 3.0f + 0.1f * static_cast<float>(i)),
                                          map::PointSource::kTriangulated);
        w.AddObservation(p, i, middle[static_cast<size_t>(covisible)]);
        w.AddObservation(p, i, newest);
      }
    }
    for (const int r : remove) {
      w.RemoveKeyframe(middle[static_cast<size_t>(r)]);
    }
  }

  map::WorldMap& wm() { return maps.active(); }
};

struct Attempt {
  bool applied = false;
  LoopCloseReport rep;
  MapBuilderOutput out;
};

Attempt Close(Scene* scene) {
  Attempt a;
  LoopCloseState st;
  a.applied = LoopClose(scene->wm(), &st, &a.out, &a.rep);
  return a;
}

// 1. The newest keyframe is never its own candidate, wherever the free slots are.
TEST(P4R7pLoopSelect, NewestEntryIsNeverACandidate) {
  const ScopedAccuracy on(true, true, true);
  const std::vector<std::vector<int>> layouts = {{}, {0}, {1}, {2}, {3}, {0, 1}, {0, 2}, {1, 3}, {0, 1, 2}};
  for (const std::vector<int>& remove : layouts) {
    Scene scene(8, {}, remove);
    ASSERT_EQ(scene.wm().NumKeyframeSlots(), 6u);
    ASSERT_EQ(scene.newest.index(), 5u);
    const Attempt a = Close(&scene);
    EXPECT_FALSE(a.rep.candidate == scene.newest) << "removed " << remove.size() << " keyframes";
    EXPECT_TRUE(a.rep.candidate == scene.first) << "removed " << remove.size() << " keyframes";
    EXPECT_NE(a.rep.candidate.index(), scene.newest.index());
    EXPECT_FLOAT_EQ(a.rep.candidate_distance, 8.0f);
    EXPECT_EQ(a.rep.num_live, 6u - remove.size());  // first, four middle keyframes and the newest, minus the removed
  }
}

// 2. A free slot in front of the newest keyframe: the original reads the compressed list at slot
// indices and selects the newest keyframe itself; R7P selects the first keyframe and the loop is a
// real one (the pose graph runs over first, the three remaining middle keyframes and the newest).
TEST(P4R7pLoopSelect, FreeSlotAheadOfTheNewestStillSelectsANonSelfCandidate) {
  Scene scene(8, {}, {0});
  ASSERT_EQ(scene.wm().KeyframeAtSlot(1).id, 0u);  // the free slot, in front of the newest keyframe
  ASSERT_EQ(scene.newest.index(), 5u);
  {
    const ScopedAccuracy on(true, true, true);
    const Attempt a = Close(&scene);
    ASSERT_TRUE(a.applied);
    EXPECT_EQ(a.rep.outcome, LoopCloseOutcome::kApplied);
    EXPECT_TRUE(a.rep.candidate == scene.first);
    EXPECT_TRUE(a.rep.pose_graph.applied);
    EXPECT_EQ(a.rep.pose_graph.nodes, 5);
    EXPECT_EQ(a.rep.pose_graph.edges, 4);
  }
}

// 3. S2 by slot. Slot 1 is free, so the list is [slot 0, 2, 3, 4, 5(newest)] and the positions of
// the entries differ from their slots by one. The covisible neighbour is excluded wherever its entry
// sits in the list, and the entry that merely stands at a position equal to a covisible slot is not.
TEST(P4R7pLoopSelect, S2IsAppliedToTheSlotOfTheEntry) {
  const ScopedAccuracy on(true, true, true);
  {  // middle[1] (slot 2, closest at 4 bits) is covisible: excluded; middle[2] (slot 3, 2 bits, list
     // position 2 = the covisible slot) is selected, which a position based S2 would have excluded.
    Scene scene(8, {256, 4, 2, 256}, {0}, /*covisible=*/1);
    const Attempt a = Close(&scene);
    EXPECT_TRUE(a.rep.candidate == scene.middle[2]);
    EXPECT_FLOAT_EQ(a.rep.candidate_distance, 2.0f);
  }
  {  // middle[2] (slot 3) is covisible: excluded; middle[1] (slot 2, 4 bits) is selected.
    Scene scene(8, {256, 4, 2, 256}, {0}, /*covisible=*/2);
    const Attempt a = Close(&scene);
    EXPECT_TRUE(a.rep.candidate == scene.middle[1]);
    EXPECT_FLOAT_EQ(a.rep.candidate_distance, 4.0f);
  }
  {  // no covisibility: the closest entry is selected
    Scene scene(8, {256, 4, 2, 256}, {0});
    const Attempt a = Close(&scene);
    EXPECT_TRUE(a.rep.candidate == scene.middle[2]);
  }
}

// 4. R7P off (R7 on) is the original selection: the newest keyframe itself when a free slot precedes
// it (and the pose graph guard refuses the self loop), the first keyframe when none does.
TEST(P4R7pLoopSelect, OffReproducesTheOriginalSelectionIncludingTheSelfLoop) {
  const ScopedAccuracy off(true, true, false);
  {
    Scene scene(8, {}, {0});
    const Attempt a = Close(&scene);
    EXPECT_TRUE(a.rep.candidate == scene.newest);  // the original's self loop
    EXPECT_FLOAT_EQ(a.rep.candidate_distance, 0.0f);
    EXPECT_FALSE(a.rep.pose_graph.applied);  // pose_graph.cc: a self loop relates no two keyframes
  }
  {
    Scene scene(8, {}, {});  // no free slot: the original already skips the newest entry
    const Attempt a = Close(&scene);
    EXPECT_TRUE(a.rep.candidate == scene.first);
  }
  {  // with R7P on the layout without a free slot gives the same candidate and the same report
    Scene s_off(8, {}, {});
    const Attempt a_off = Close(&s_off);
    const ScopedAccuracy on(true, true, true);
    Scene s_on(8, {}, {});
    const Attempt a_on = Close(&s_on);
    EXPECT_TRUE(a_on.rep.candidate == a_off.rep.candidate);
    EXPECT_EQ(a_on.rep.candidate_distance, a_off.rep.candidate_distance);
    EXPECT_EQ(a_on.rep.num_matches, a_off.rep.num_matches);
    EXPECT_EQ(a_on.rep.num_correspondences, a_off.rep.num_correspondences);
    EXPECT_EQ(a_on.rep.pose_graph.nodes, a_off.rep.pose_graph.nodes);
    EXPECT_EQ(a_on.rep.pose_graph.applied, a_off.rep.pose_graph.applied);
  }
}

// 5. R7P refines R7: without R7, or without `enabled`, the key is inert and nothing is printed.
TEST(P4R7pLoopSelect, WithoutR7OrEnabledTheKeyIsInert) {
  for (const auto& [enabled, r7] : std::vector<std::pair<bool, bool>>{{true, false}, {false, true}, {false, false}}) {
    const ScopedAccuracy cfg(enabled, r7, true);
    Scene scene(8, {}, {0});
    ::testing::internal::CaptureStderr();
    const Attempt a = Close(&scene);
    const std::string err = ::testing::internal::GetCapturedStderr();
    EXPECT_TRUE(a.rep.candidate == scene.newest) << "enabled " << enabled << " r7 " << r7;
    EXPECT_EQ(err.find("[r7p"), std::string::npos) << err;
  }
}

// 6. The diagnostic lines: one `[r7p-loop]` line per accepted loop, `[r7p-attempt]` for a refused one,
// nothing at all with R7P off.
TEST(P4R7pLoopSelect, DiagnosticLinesOnStderr) {
  {
    const ScopedAccuracy on(true, true, true);
    Scene scene(8, {}, {0});
    ::testing::internal::CaptureStderr();
    const Attempt a = Close(&scene);
    const std::string err = ::testing::internal::GetCapturedStderr();
    ASSERT_TRUE(a.applied);
    const std::regex line(
        R"(^\[r7p-loop\] newest=(\d+) cand=(\d+) gap=(-?\d+) nodes=(\d+) merges=(\d+) applied=([01])\n$)");
    std::smatch m;
    ASSERT_TRUE(std::regex_match(err, m, line)) << err;
    EXPECT_EQ(std::stoul(m[1]), scene.newest.id());
    EXPECT_EQ(std::stoul(m[2]), scene.first.id());
    // gap = creation-order distance (Keyframe::seq): first is created first, newest five keyframes later
    // (four middle keyframes in between; a removed one keeps its sequence number).
    EXPECT_EQ(std::stol(m[3]), static_cast<long>(scene.wm().GetKeyframe(scene.newest).seq) -
                                   static_cast<long>(scene.wm().GetKeyframe(scene.first).seq));
    EXPECT_EQ(std::stol(m[3]), 5);
    EXPECT_EQ(std::stoi(m[4]), a.rep.pose_graph.nodes);
    EXPECT_EQ(std::stoul(m[5]), a.rep.num_merges);
    EXPECT_EQ(std::stoi(m[6]), a.rep.pose_graph.applied ? 1 : 0);
  }
  {  // no candidate below the threshold: an attempt line, no loop line
    const ScopedAccuracy on(true, true, true);
    Scene scene(256, {}, {0});
    ::testing::internal::CaptureStderr();
    const Attempt a = Close(&scene);
    const std::string err = ::testing::internal::GetCapturedStderr();
    EXPECT_FALSE(a.applied);
    EXPECT_EQ(a.rep.outcome, LoopCloseOutcome::kNoCandidate);
    EXPECT_EQ(err, "[r7p-attempt] outcome=no_candidate\n");
  }
  {
    const ScopedAccuracy off(true, true, false);
    Scene scene(8, {}, {0});
    ::testing::internal::CaptureStderr();
    Close(&scene);
    EXPECT_EQ(::testing::internal::GetCapturedStderr(), "");
  }
}

// 7. Property: for random distances and free-slot layouts R7P selects the nearest entry (strictly
// below the threshold 80, first slot wins a tie) among the live slots other than the newest one.
TEST(P4R7pLoopSelect, SelectsTheNearestNonNewestSlotForRandomLayouts) {
  const ScopedAccuracy on(true, true, true);
  std::mt19937 gen(4242);
  for (int trial = 0; trial < 60; ++trial) {
    std::uniform_int_distribution<int> bits(1, 120);
    const int first_bits = bits(gen);
    std::vector<int> middle_bits;
    for (int k = 0; k < 4; ++k) middle_bits.push_back(bits(gen));
    std::vector<int> remove;
    for (int k = 0; k < 4; ++k) {
      if (std::uniform_int_distribution<int>(0, 2)(gen) == 0) remove.push_back(k);
    }
    Scene scene(first_bits, middle_bits, remove);
    map::WorldMap& wm = scene.wm();
    if (wm.NumKeyframeSlots() < 5) continue;  // below S1
    float best = 80.0f;
    map::KeyframeId expected;
    for (size_t s = 0; s < wm.NumKeyframeSlots(); ++s) {
      const map::Keyframe& kf = wm.KeyframeAtSlot(s);
      if (kf.id == 0 || s == scene.newest.index()) continue;
      int distance = 0;
      for (uint8_t byte : kf.global_descriptor) distance += __builtin_popcount(byte);
      if (static_cast<float>(distance) < best) {
        best = static_cast<float>(distance);
        expected = map::KeyframeId(static_cast<uint32_t>(s), kf.id);
      }
    }
    const Attempt a = Close(&scene);
    if (best < 80.0f) {
      EXPECT_TRUE(a.rep.candidate == expected) << "trial " << trial;
      EXPECT_FLOAT_EQ(a.rep.candidate_distance, best) << "trial " << trial;
      EXPECT_FALSE(a.rep.candidate == scene.newest) << "trial " << trial;
    } else {
      EXPECT_EQ(a.rep.outcome, LoopCloseOutcome::kNoCandidate) << "trial " << trial;
    }
  }
}

}  // namespace
}  // namespace slam_refact::map_builder
