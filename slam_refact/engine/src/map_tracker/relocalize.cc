// relocalize (1274, cloud-empty path) and computeFramePoseUsingRobustP2P (1265), S2 §3.2.
// Read from Ghidra G1274 / G1265, G1550 and the f1274 / f1265 disassembly.

#include <cfloat>
#include <cstdint>
#include <vector>

#include "c8/geometry/egomotion.h"
#include "c8/hpoint.h"
#include "common/check.h"
#include "config/settings.h"
#include "map/world_map_internal.h"
#include "map_tracker/map_tracker.h"
#include "map_tracker/map_tracker_internal.h"
#include "reality/engine/features/descriptor-bit-counter.h"
#include "reality/engine/features/image-descriptor.h"
#include "reality/engine/geometry/pose-pnp.h"

namespace slam_refact::map_tracker {

namespace {

int Hamming256(const uint8_t* a, const uint8_t* b) {
  int d = 0;
  for (int i = 0; i < 32; ++i) {
    d += __builtin_popcount(static_cast<unsigned>(a[i] ^ b[i]));
  }
  return d;
}

}  // namespace

void ComputeFramePoseUsingRobustP2P(const TrackerInput& frame,
                                    const std::vector<c8::HPoint3>& world_points,
                                    const std::vector<map::PointId>& ids,
                                    const std::vector<c8::HPoint2>& rays,
                                    const c8::HMatrix& tracking_pose, compat::RandomNumbers* rng,
                                    c8::RobustP2PScratchSpace* scratch, RelocResult* out) {
  out->pose = tracking_pose;
  // Scratch ownership (S2 review MEDIUM-1). The original keeps one process-wide static
  // (F1265:109, Q16). The only scratch field that carries over between calls is confidenceGrid:
  // robustP2P builds it from the frame bounds of the first call and never clears it; every other
  // field is cleared by RobustP2PScratchSpace::reset or written before it is read
  // (pose-pnp.cc:753-826). The frame bounds depend only on the intrinsics, which are fixed for
  // one Engine (Engine::Configure rejects an intrinsics change), so the scratch owned by the
  // MapTracker (MapTracker::Impl::p2p_scratch) produces the same grid as the process-wide static
  // for every call of a single-engine process (the replay harness, one XREngine per page). It
  // differs only when engines with different intrinsics share one process, where the original
  // would reuse the first engine's grid; there each engine now uses its own. Replay traces are
  // unchanged (VERIFIED, build/slam-refact-20261016/review-fixes).
  SR_CHECK(scratch != nullptr, "[map_tracker] robustP2P scratch missing");

  const config::WorldMapsSettings& w = config::WorldMapsSettings::Get();
  c8::RobustP2PParams params;
  params.isImageTracking = false;
  params.skipFinalPoseOptimization = false;
  params.ransac.initialInlierGuess = w.ransac_initial_inlier_guess;
  params.ransac.numSamplesPerSolve = 2;
  params.ransac.confidence = 0.99f;
  params.ransac.inlierThreshold = w.ransac_inlier_threshold;
  params.ransac.maxNumIteration = static_cast<size_t>(w.ransac_max_num_iteration);
  params.ransac.minNumIteration = 100;
  params.ransac.minInlierRequired = static_cast<size_t>(w.ransac_min_inlier_required);
  params.ransac.minBaseline2d = 0.01f;
  params.ransac.countFailure = w.ransac_count_failure;
  params.simAnn.initialTemperature = 1.0f;
  params.simAnn.coolingRate = 0.9847666f;

  c8::HPoint2 lower_left;
  c8::HPoint2 upper_right;
  c8::frameBounds(frame.frame_points.intrinsic(), &lower_left, &upper_right);

  c8::HMatrix cam = c8::HMatrixGen::i();
  std::vector<uint8_t> inliers;
  out->success = c8::robustP2P(world_points, rays, lower_left, upper_right, tracking_pose, params,
                               &cam, &out->confidence, &inliers, rng, scratch);
  if (out->success) {
    out->inlier_ids.clear();
    out->inlier_world_points.clear();
    for (size_t i = 0; i < inliers.size(); ++i) {
      if (inliers[i] != 0) {
        out->inlier_ids.push_back(ids[i]);
        out->inlier_world_points.push_back(world_points[i]);
      }
    }
    out->pose = cam;
  }
}

void Relocalize(RelocResult* out, const map::WorldMap& map, const TrackerInput& frame,
                const c8::HMatrix& tracking_pose, compat::RandomNumbers* rng) {
  // Frozen entry point without an owner for the scratch: a call-local scratch, so the confidence
  // grid comes from this frame's bounds. MapTracker::Track uses the overload below.
  c8::RobustP2PScratchSpace scratch;
  Relocalize(out, map, frame, tracking_pose, rng, &scratch);
}

void Relocalize(RelocResult* out, const map::WorldMap& map, const TrackerInput& frame,
                const c8::HMatrix& tracking_pose, compat::RandomNumbers* rng,
                c8::RobustP2PScratchSpace* scratch) {
  SR_CHECK(map.CloudEmpty(), "[map_tracker] cloud relocalization is VPS-only");
  const map::WorldMapAccess::Impl& m = map::WorldMapAccess::impl(map);

  // 1550 over pools TRIANGULATED, GROUND, SKY, DEPTH: every slot not on the free list, in slot
  // order, with its majority descriptor (lazily recomputed: bit = oneCount > count >> 1).
  std::vector<map::PointId> dict_ids;
  std::vector<std::array<uint8_t, 32>> dict_desc;
  for (int pool_index = 0; pool_index < 4; ++pool_index) {
    const map::PointPool& pool = m.pools[pool_index];
    std::vector<uint8_t> alive(pool.majority.size(), 1);
    for (uint32_t slot : pool.free_slots) {
      alive[slot] = 0;
    }
    for (size_t slot = 0; slot < pool.majority.size(); ++slot) {
      if (alive[slot] == 0) {
        continue;
      }
      const map::MajorityRecord& rec = pool.majority[slot];
      std::array<uint8_t, 32> desc = rec.majority;
      if (!rec.majority_valid) {
        c8::getMajorityFromOneCounts<32>(rec.one_counts, rec.count >> 1, desc.data());
      }
      dict_ids.emplace_back(slot, pool.points[slot].id);
      dict_desc.push_back(desc);
    }
  }

  // global-match-map-brute-force-with-reader (F1274:259-361).
  const config::WorldMapsSettings& w = config::WorldMapsSettings::Get();
  const int threshold = w.full_map_global_matcher_distance_threshold;
  const int initial = threshold > 0 ? threshold + 1 : 257;
  const c8::FrameWithPoints& f = frame.frame_points;
  std::vector<c8::PointMatch> matches;
  if (!dict_ids.empty()) {
    for (const auto kp : f.store().keypointIndices<c8::GorbFeature>()) {
      const uint8_t* desc = f.store().get<c8::GorbFeature>(kp).data();
      int best_d = initial;
      int64_t best = -1;
      for (size_t c = 0; c < dict_desc.size(); ++c) {
        const int d = Hamming256(dict_desc[c].data(), desc);
        if (d < best_d) {
          best = static_cast<int64_t>(c);
          best_d = d;
        }
      }
      if (best >= 0) {
        c8::PointMatch pm;
        pm.wordsIdx = kp;
        pm.dictionaryIdx = static_cast<size_t>(best);
        pm.rotation = FLT_MAX;
        pm.descriptorDist = static_cast<float>(best_d);
        pm.dictionaryScale = 0xff;
        matches.push_back(pm);
      }
    }
  }

  // Correspondences (F1274:1128-1398): no ratio test, no dedup.
  std::vector<c8::HPoint3> world_points;
  std::vector<c8::HPoint2> rays;
  std::vector<map::PointId> ids;
  world_points.reserve(matches.size());
  rays.reserve(matches.size());
  ids.reserve(matches.size());
  for (const c8::PointMatch& pm : matches) {
    const map::PointId id = dict_ids[pm.dictionaryIdx];
    world_points.push_back(map.Point(id).position);
    const c8::FramePoint& fp = f.points()[pm.wordsIdx];
    rays.push_back(c8::HPoint2(fp.x(), fp.y()));
    ids.push_back(id);
  }

  *out = RelocResult{};
  out->pose = tracking_pose;
  ComputeFramePoseUsingRobustP2P(frame, world_points, ids, rays, tracking_pose, rng, scratch, out);
}

}  // namespace slam_refact::map_tracker
