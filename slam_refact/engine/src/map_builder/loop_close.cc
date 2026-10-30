// Loop close of MapBuilder::Update (func 936 [I], G936:4159-5754; US-025 LC-3).
//
// Basis: docs/spec/12-loop-close.md only. Step labels S1..S12 are the spec §3.1 / §3.2 ones;
// S0 (the MapBuilder.loopCloseEnabled gate) is checked by the caller. Citations G936:L / D:L are
// the spec's. Anything the spec marks UNKNOWN is not guessed here; the deterministic choices made
// for U2 (uninitialized distance-list tail) and U3 (set traversal with erase, deviation N2) are listed in docs/design/us025-lc3-impl.md and repeated at the code.
//
// Compiled only with SLAM_REFACT_LOOP_CLOSE=ON (map_builder/CMakeLists.txt).

#include "map_builder/loop_close.h"

#include <algorithm>
#include <cfloat>
#include <cstdint>
#include <cstring>
#include <deque>
#include <set>
#include <vector>

#include "c8/hmatrix.h"
#include "c8/hpoint.h"
#include "common/check.h"
#include "common/stage_timer.h"
#include "config/settings.h"
#include "reality/engine/features/frame-point.h"
#include "reality/engine/features/image-descriptor.h"
#include "reality/engine/features/local-matcher.h"
#include "reality/engine/geometry/bundle.h"
#include "reality/engine/geometry/observed-point.h"

namespace slam_refact::map_builder {

namespace {

using Entry = LoopCloseDistEntry;

constexpr size_t kMinSlots = 5;          // S1  G936:4196
constexpr size_t kRingSize = 3;          // G936:4823
constexpr int kMatcherBinsX = 15;        // S8  D:107567-107579
constexpr int kMatcherBinsY = 20;
constexpr float kMatcherRadius = 1.0f;
constexpr float kMatcherDescriptorDistance = 32.0f;

static_assert(c8::OrbFeature::size() == 32, "the agate input is 32-byte ORB descriptors");

// popcount of the 256-bit xor, as float (S3, G936:4277-4326).
float Hamming256(const std::array<uint8_t, 32>& a, const std::array<uint8_t, 32>& b) {
  int bits = 0;
  for (size_t i = 0; i < a.size(); i += 8) {
    uint64_t x;
    uint64_t y;
    std::memcpy(&x, a.data() + i, sizeof(x));
    std::memcpy(&y, b.data() + i, sizeof(y));
    bits += __builtin_popcountll(x ^ y);
  }
  return static_cast<float>(bits);
}

// ---- find-loop-close-target (S1..S6) -------------------------------------------------------------

bool FindTarget(map::WorldMap& wm, LoopCloseState* st, LoopCloseReport* rep,
                map::KeyframeId* candidate) {
  StageTimer timer("find-loop-close-target");
  const size_t n_slots = wm.NumKeyframeSlots();
  rep->num_slots = n_slots;
  if (n_slots < kMinSlots) {  // S1: slot count, free slots included (G936:4196)
    rep->outcome = LoopCloseOutcome::kTooFewSlots;
    return false;
  }

  const map::KeyframeId newest = wm.newest();
  rep->newest = newest;
  // S2 (G936:4247-4256).
  SR_CHECK(newest.index() < n_slots && wm.KeyframeAtSlot(newest.index()).id == newest.id(),
           "[keyframe-descriptor] Keyframe with this id doesn't exist");

  // S3: distance list of the live slots, compressed (G936:4262-4326).
  // U2 (UNVERIFIED): the original allocates malloc(n_slots * 12) without initializing it and the
  // selection loop below reads it at slot indices, so with free slots the entries past `live` are
  // whatever the heap held. Default model (LoopCloseState::model_stale_dist_tail = false): the tail
  // is id 0 entries, skipped by the selection (lc-g G-C: 92% of the original's picks; the rest is
  // unreproducible heap content, UNKNOWN). true keeps the previous attempt's storage instead.
  std::vector<Entry>& dist = st->dist_storage;
  if (!st->model_stale_dist_tail || dist.size() != n_slots) {
    dist.assign(n_slots, Entry{FLT_MAX, 0, 0});
  }
  const std::array<uint8_t, 32>& newest_desc = wm.KeyframeAtSlot(newest.index()).global_descriptor;
  size_t live = 0;
  for (size_t s = 0; s < n_slots; ++s) {
    const map::Keyframe& kf = wm.KeyframeAtSlot(s);
    if (kf.id != 0) {
      dist[live++] = Entry{Hamming256(kf.global_descriptor, newest_desc),
                           static_cast<uint32_t>(s), kf.id};
    }
  }
  rep->num_live = live;

  // S4: ring reset on a new newest handle (G936:4336-4338), 2-hop covisible exclusion set.
  if (st->last_newest != newest) {
    st->last_newest = newest;
    st->ring.fill(0);
    st->cursor = 0;
  }
  const int min_shared = config::WorldMapSettings::Get().required_points_for_covisibility;
  std::set<map::KeyframeId> s1;  // order (id, index), like the original tree (G936:4375-4380)
  for (const map::KeyframeId& h1 : wm.CovisibleKeyframes(newest, min_shared)) {  // 1143
    for (const map::KeyframeId& h2 : wm.CovisibleKeyframes(h1, min_shared)) {
      s1.insert(h2);
    }
    s1.insert(h1);
  }
  s1.erase(newest);
  std::set<uint32_t> s2;  // slot indices only (G936:4606-4745)
  for (const map::KeyframeId& h : s1) {
    s2.insert(static_cast<uint32_t>(h.index()));
  }

  // S5: candidate selection (G936:4747-4816). The compressed list is read at the slot index m
  // and `m == newest.index` / S2 are applied to that position (spec §0 3, U2); strict `<`.
  float best = config::WorldMapsSettings::Get().loop_closing_distance_threshold;
  map::KeyframeId cand;
  const size_t iterations = std::max<size_t>(n_slots, 1);
  for (size_t m = 0; m < iterations; ++m) {
    if (m == newest.index()) {
      continue;
    }
    const Entry e = dist[m];
    if (e.id == 0) {
      continue;
    }
    if (!(e.distance < best)) {
      continue;
    }
    if (s2.count(static_cast<uint32_t>(m)) != 0) {
      continue;
    }
    if (e.id == st->ring[0] || e.id == st->ring[1] || e.id == st->ring[2]) {
      continue;
    }
    best = e.distance;
    cand = map::KeyframeId(e.index, e.id);
  }

  // S6: the threshold is read again (G936:4817-4819); the ring advances before the pose stage.
  if (!(best < config::WorldMapsSettings::Get().loop_closing_distance_threshold)) {
    rep->outcome = LoopCloseOutcome::kNoCandidate;
    return false;
  }
  st->ring[st->cursor] = cand.id();
  st->cursor = static_cast<uint32_t>((st->cursor + 1) % kRingSize);
  rep->candidate = cand;
  rep->candidate_distance = best;
  *candidate = cand;
  return true;
}

// ---- find-loop-close-pose (S7..S11) --------------------------------------------------------------

struct PoseResult {
  std::vector<c8::PointMatch> matches;
  c8::HMatrix pose = c8::HMatrixGen::i();
};

bool FindPose(map::WorldMap& wm, map::KeyframeId newest, map::KeyframeId cand, LoopCloseReport* rep,
              PoseResult* result) {
  StageTimer timer("find-loop-close-pose");
  // S7: a stale or invalid handle is a silent not-found (G936:4851-4880).
  if (!wm.IsKeyframeValid(newest) || !wm.IsKeyframeValid(cand)) {
    rep->outcome = LoopCloseOutcome::kInvalidHandle;
    return false;
  }
  const map::Keyframe& k_cand = wm.GetKeyframe(cand);
  const map::Keyframe& k_new = wm.GetKeyframe(newest);

  // S8: local matcher over the candidate keyframe (D:107567-107579).
  c8::LocalMatcher<c8::OrbFeature> matcher(kMatcherBinsX, kMatcherBinsY, kMatcherRadius,
                                           kMatcherDescriptorDistance);
  matcher.useRatioTest(true);
  matcher.useScaleFilter(false);
  matcher.setQueryPointsPointer(k_cand.frame);  // frameBoundsExcludingEdge + BinMap reset (1114)
  c8::MatchResultFrequency frequency;
  {
    StageTimer match_timer("local-match");
    matcher.findMatches(k_new.frame.points(), k_new.frame.store(), &result->matches, &frequency);
  }
  SR_CHECK(wm.IsKeyframeValid(cand) && wm.IsKeyframeValid(newest),
           "[WorldMap::getKeyframe] WorldMapKeyFrameId mismatch");
  rep->num_matches = result->matches.size();
  const config::WorldMapsSettings& settings = config::WorldMapsSettings::Get();
  if (result->matches.size() < static_cast<uint32_t>(settings.loop_closing_matches_required)) {
    rep->outcome = LoopCloseOutcome::kTooFewMatches;  // S9
    return false;
  }

  // S10: 3D-2D correspondences from the candidate's world points (G936:5060-5167).
  std::vector<map::PointId> cand_point_ids;  // pointIds(K.pts) (1140)
  cand_point_ids.reserve(k_cand.frame.points().size());
  for (const c8::FramePoint& fp : k_cand.frame.points()) {
    cand_point_ids.push_back(fp.id());
  }
  c8::Vector<c8::ObservedPoint> rays;
  c8::Vector<c8::HPoint3> world_points;
  for (const c8::PointMatch& m : result->matches) {  // dictionaryIdx ascending
    const size_t o = m.wordsIdx;       // newest keyframe observation
    const size_t q = m.dictionaryIdx;  // candidate keyframe observation
    SR_CHECK(o < k_new.frame.points().size() && q < cand_point_ids.size(),
             "[loop_close] match index out of range");
    if (k_cand.frame.worldPointIndices().count(q) != 0) {
      const map::WorldPoint& p = wm.Point(cand_point_ids[q]);  // 634: aborts on an invalid id
      world_points.push_back(p.position);
      const c8::FramePoint& fp = k_new.frame.points()[o];
      rays.push_back(c8::ObservedPoint{c8::HPoint2(fp.x(), fp.y()), 1, 0.0f, 1.0f});
    }
  }
  rep->num_correspondences = world_points.size();
  if (world_points.size() <
      static_cast<uint32_t>(settings.loop_closing_pose_estimation_points_required)) {
    rep->outcome = LoopCloseOutcome::kTooFewCorrespondences;  // S11
    return false;
  }

  // 1146 from the candidate's pose; the result is adopted without any post check (G936:5367-5560).
  c8::HMatrix pose = k_cand.frame.extrinsic();
  c8::PoseEstimationAnalyticalParams params;
  params.suppressOutliers = true;
  params.scale = 1.0;
  params.useImageTargetCostFunction = false;
  params.cameraMotionWeight = 0.0;
  params.logReport = false;
  const std::deque<c8::HMatrix> last_two;
  if (!c8::poseEstimationAnalytical(rays, world_points, last_two, &pose, params, nullptr)) {
    rep->outcome = LoopCloseOutcome::kPoseEstimationFailed;
    return false;
  }
  result->pose = pose;
  return true;
}

// ---- report: correction, updateFromLoopCloseResult (S12) -----------------------------------------

void ApplyResult(map::WorldMap& wm, map::KeyframeId newest, map::KeyframeId cand,
                 const PoseResult& result, MapBuilderOutput* out, LoopCloseReport* rep) {
  // 660: C = P_old^-1 * P_new, computed before the keyframe pose changes (G936:5585-5586). The
  // vendored operator* is the original's product: lc-g's recorded 660 in/out (7/7) match its
  // summation order bit for bit and not the 789 order of spec §9 (U10 resolved). inv() aborts on
  // a non-invertible P_old.
  const c8::HMatrix old_pose = wm.GetKeyframe(newest).frame.extrinsic();
  out->world_correction = old_pose.inv() * result.pose;
  out->world_transform_updated = true;

  StageTimer timer("updateFromLoopCloseResult");
  wm.SetKeyframePose(newest, result.pose);  // 655
  SR_CHECK(wm.IsKeyframeValid(cand) && wm.IsKeyframeValid(newest),
           "[WorldMap::getKeyframe] WorldMapKeyFrameId mismatch");

  // Snapshots after 655, before any link changes (1140 at G936:5639, 5641).
  std::vector<map::PointId> new_point_ids;
  for (const c8::FramePoint& fp : wm.GetKeyframe(newest).frame.points()) {
    new_point_ids.push_back(fp.id());
  }
  std::vector<map::PointId> cand_point_ids;
  for (const c8::FramePoint& fp : wm.GetKeyframe(cand).frame.points()) {
    cand_point_ids.push_back(fp.id());
  }

  // Merge / associate in match order (dictionaryIdx ascending; G936:5645-5713). The live sets and
  // the live observation handle are read at each step, the snapshots only where the spec says.
  for (const c8::PointMatch& m : result.matches) {
    const size_t o = m.wordsIdx;
    const size_t q = m.dictionaryIdx;
    const map::Keyframe& k_cand = wm.GetKeyframe(cand);
    const map::Keyframe& k_new = wm.GetKeyframe(newest);
    if (k_cand.frame.worldPointIndices().count(q) != 0) {
      const map::PointId eb = k_new.frame.points()[o].id();
      const map::PointId db = cand_point_ids[q];
      if (eb != db) {
        if (k_new.frame.worldPointIndices().count(o) != 0) {
          wm.MergePoint(eb, db);  // 1124: eb -> db
        } else {
          wm.AddObservation(db, static_cast<uint32_t>(o), newest);  // 658
        }
      }
    } else if (k_new.frame.worldPointIndices().count(o) != 0) {
      wm.AddObservation(new_point_ids[o], static_cast<uint32_t>(q), cand);  // 658
    }
  }

  // Unlink every non-TRIANGULATED observation of the newest keyframe (G936:5715-5743).
  // U3 (UNKNOWN): the original erases the std::set node it is standing on (658 with a null point)
  // and then reads the freed node's right child to find the next one, so which observations it
  // visits depends on allocator state. Here the traversal is a snapshot of the set in ascending
  // order, i.e. every observation is visited exactly once.
  const std::set<size_t> observations = wm.GetKeyframe(newest).frame.worldPointIndices();
  for (size_t k : observations) {
    const map::PointId pid = wm.GetKeyframe(newest).frame.points()[k].id();
    if (wm.Point(pid).source != map::PointSource::kTriangulated) {
      wm.AddObservation(map::PointId::nullId(), static_cast<uint32_t>(k), newest);  // 658(null)
      ++rep->num_unlinked;
    }
  }
  wm.AdjustDirtyPoints();  // 1127 (the common one follows in MapBuilder::Update)
}

}  // namespace

void ComputeKeyframeGlobalDescriptor(LoopCloseState* state, map::Keyframe* kf) {
  if (!state->agate) {
    state->agate = std::make_unique<AgateDescriptor>();
  }
  const auto& orbs = kf->frame.store().getFeatures<c8::OrbFeature>();
  std::vector<uint8_t> bytes(orbs.size() * kAgateOrbBytes);
  for (size_t i = 0; i < orbs.size(); ++i) {
    std::memcpy(bytes.data() + i * kAgateOrbBytes, orbs[i].data(), kAgateOrbBytes);
  }
  kf->global_descriptor = state->agate->Extract(bytes.empty() ? nullptr : bytes.data(), orbs.size());
}

bool LoopClose(map::WorldMap& wm, LoopCloseState* state, MapBuilderOutput* out,
               LoopCloseReport* report) {
  StageTimer timer("loop-close");
  LoopCloseReport local;
  LoopCloseReport* rep = report != nullptr ? report : &local;
  *rep = LoopCloseReport{};
  out->world_transform_updated = false;  // out+0x1c is overwritten whenever [I] runs (spec §6)

  map::KeyframeId cand;
  if (!FindTarget(wm, state, rep, &cand)) {
    return false;
  }
  const map::KeyframeId newest = wm.newest();
  PoseResult result;
  if (!FindPose(wm, newest, cand, rep, &result)) {
    return false;
  }
  ApplyResult(wm, newest, cand, result, out, rep);
  rep->outcome = LoopCloseOutcome::kApplied;
  return true;
}

}  // namespace slam_refact::map_builder
