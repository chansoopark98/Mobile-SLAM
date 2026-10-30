// trackPoints (1272) and the map-points-subset selection (§5.4), S2 §3.1. Read from Ghidra G1272
// and the f1272 disassembly (float evaluation order taken from the wasm stack).

#include <cfloat>
#include <cmath>
#include <cstdint>
#include <deque>
#include <map>
#include <vector>

#include "c8/hmatrix.h"
#include "c8/hpoint.h"
#include "common/c8_adapters.h"
#include "common/check.h"
#include "compat/libcxx_algorithms.h"
#include "config/settings.h"
#include "map/world_map.h"
#include "map_tracker/map_tracker.h"
#include "map_tracker/map_tracker_internal.h"
#include "reality/engine/features/local-matcher.h"
#include "reality/engine/geometry/bundle.h"

namespace slam_refact::map_tracker {

namespace {

// §5.2 (F1272:2066-2143): mean of squared ray distances, float, sum += (dx^2 + dy^2). The camera
// point is pose.inv() * P without normalization; the projection is x / z.
float MeanSquaredRayError(const c8::HMatrix& pose, const std::vector<c8::ObservedPoint>& obs,
                          const std::vector<c8::HPoint3>& world) {
  const c8::HMatrix inv = pose.inv();
  float sum = 0.0f;
  for (size_t i = 0; i < obs.size(); ++i) {
    const c8::HPoint3 pc = inv * world[i];
    const auto& r = pc.raw();
    const auto& o = obs[i].position.raw();
    const float dx = r[0] / r[2] - o[0] / o[2];
    const float dy = r[1] / r[2] - o[1] / o[2];
    sum = sum + (dx * dx + dy * dy);
  }
  return sum / static_cast<float>(obs.size());
}

template <typename Descriptor>
void MatchProjected(const c8::FrameWithPoints& frame, const c8::FrameWithPoints& projected,
                    float radius, std::vector<c8::PointMatch>* matches,
                    c8::MatchResultFrequency* frequency) {
  c8::LocalMatcher<Descriptor> matcher(15, 20);
  matcher.setSearchRadius(radius);
  matcher.setQueryPointsPointer(frame);
  matcher.findMatches(projected, matches, frequency);
}

}  // namespace

void TrackedBimap::Insert(uint32_t frame_index, map::PointId id) {
  const auto by_id_it = by_id.find(id);
  if (by_id_it != by_id.end()) {
    const auto old_frame = by_frame.find(by_id_it->second);
    if (old_frame != by_frame.end()) {
      by_frame.erase(old_frame);
    }
    by_id.erase(id);
  }
  by_frame.emplace(frame_index, id);
  by_id.emplace(id, frame_index);
}

void SelectSubset(const map::WorldMap& map, MapTrackResult* result) {
  const config::WorldMapsSettings& w = config::WorldMapsSettings::Get();
  const size_t n_max = static_cast<size_t>(
      map.CloudEmpty() ? w.max_subset_points : w.cloud_map_max_subset_points);
  result->subset_obs.clear();
  result->subset_matches.clear();
  result->subset_world_points.clear();
  const size_t m = result->all_obs.size();

  std::vector<uint32_t> order(m);
  for (size_t i = 0; i < m; ++i) {
    order[i] = static_cast<uint32_t>(i);
  }
  if (m >= n_max) {
    std::vector<float> score(m, 0.0f);
    const std::vector<c8::FramePoint>& proj = result->projected.frame.points();
    int ground_count = 0;
    for (size_t i = 0; i < m; ++i) {
      const map::WorldPoint& p = map.Point(result->all_matches[i].id);
      float& s = score[i];
      switch (p.source) {
        case map::PointSource::kTriangulated:
          s = w.subset_is_triangulated_weight + s;
          s = w.subset_is_non_skypoint_weight + s;
          break;
        case map::PointSource::kDepth:
          s = w.subset_is_depth_point_weight + s;
          s = w.subset_is_non_skypoint_weight + s;
          break;
        case map::PointSource::kSky:
          break;
        default:
          s = w.subset_is_non_skypoint_weight + s;
          if (p.source == map::PointSource::kGround && ground_count < w.encouraged_n_ground_points) {
            s = s + w.encouraged_ground_point_weight;
            ++ground_count;
          } else if (p.source == map::PointSource::kCloud) {
            s = w.subset_is_cloud_point_weight + s;
          }
          break;
      }
      if (p.state == map::PointState::kTenured) {
        s = w.subset_is_tenured_weight + s;
      }
      const float confidence =
          p.tracking_count == 0
              ? 0.0f
              : static_cast<float>(p.tracking_hits) / static_cast<float>(p.tracking_count);
      s = confidence * w.subset_confidence_weight + s;
      if (w.subset_lower_residual_weight > 0.0f) {
        // Projected ray of the matched word (input pose) vs the observed frame ray.
        const c8::HPoint2 pr = proj[result->word_indices[i]].position();
        const auto& a = pr.raw();
        const auto& b = result->all_obs[i].position.raw();
        const float dx = a[0] / a[2] - b[0] / b[2];
        const float dy = a[1] / a[2] - b[1] / b[2];
        s = -w.subset_lower_residual_weight * (dx * dx + dy * dy) + s;
      }
    }
    // 1555: libc++ std::sort, comparator score[a] > score[b] (unstable).
    libcxx::sort(order.begin(), order.end(),
                 [&score](uint32_t a, uint32_t b) { return score[a] > score[b]; });
    order.resize(n_max);
  }
  for (uint32_t i : order) {
    result->subset_obs.push_back(result->all_obs[i]);
    result->subset_matches.push_back(result->all_matches[i]);
    result->subset_world_points.push_back(result->all_world_points[i]);
  }
}

void TrackPoints(const map::WorldMap& map, const TrackerInput& frame, const c8::HMatrix& pose,
                 const std::vector<map::PointId>& ids, MapTrackResult* result) {
  // Reset (F1272:239-276); the caller-provided outputs are cleared, not detached.
  std::vector<map::PointId>* visible_out = result->visible_out;
  std::vector<uint8_t>* matched_out = result->matched_out;
  *result = MapTrackResult{};
  result->visible_out = visible_out;
  result->matched_out = matched_out;
  if (visible_out != nullptr) {
    visible_out->clear();
  }
  if (matched_out != nullptr) {
    matched_out->clear();
  }
  result->pose = pose;

  const c8::FrameWithPoints& f = frame.frame_points;
  if (f.points().empty()) {
    return;
  }
  const PinholeIntrinsics intrinsics = ToPinholeIntrinsics(f.intrinsic());
  result->projected = map::ProjectPoints(map, ids, intrinsics, result->pose);
  const c8::FrameWithPoints& projected = result->projected.frame;
  const std::vector<c8::FramePoint>& proj = projected.points();

  const config::WorldMapsSettings& w = config::WorldMapsSettings::Get();
  // matchMinimum >= 1 keeps MeanSquaredRayError off an empty subset (0 / 0) (S2 review LOW-2).
  SR_CHECK(w.match_minimum >= 1, "WorldMaps.matchMinimum must be >= 1");
  const float radius = map.CloudEmpty() ? w.match_radius_max : w.cloud_map_match_radius_max;

  std::vector<c8::PointMatch> gorb_matches;
  c8::MatchResultFrequency freq_gorb;
  MatchProjected<c8::GorbFeature>(f, projected, radius, &gorb_matches, &freq_gorb);
  std::vector<c8::PointMatch> orb_matches;
  c8::MatchResultFrequency freq_orb;
  if (gorb_matches.size() < static_cast<size_t>(w.find_orb_matches_threshold)) {
    MatchProjected<c8::OrbFeature>(f, projected, radius, &orb_matches, &freq_orb);
  }

  // collate-matches: ORB first, then GORB; per word the smallest descriptorDist (first on ties),
  // emitted in word order (F1272:585-688).
  result->frequency = freq_gorb + freq_orb;
  std::vector<c8::PointMatch> all = orb_matches;
  all.insert(all.end(), gorb_matches.begin(), gorb_matches.end());
  std::vector<int32_t> best_by_word(proj.size(), -1);
  for (size_t i = 0; i < all.size(); ++i) {
    int32_t& b = best_by_word[all[i].wordsIdx];
    if (b == -1 || all[i].descriptorDist < all[static_cast<size_t>(b)].descriptorDist) {
      b = static_cast<int32_t>(i);
    }
  }
  std::vector<c8::PointMatch> matches;
  for (int32_t b : best_by_word) {
    if (b != -1) {
      matches.push_back(all[static_cast<size_t>(b)]);
    }
  }

  // dedup-dictionary-matches (F1272:699-994): one match per map point id (smallest
  // descriptorDist, first on ties), emitted in id order, then sorted by word index (1551).
  if (w.dedup_dictionary_matches) {
    std::map<map::PointId, size_t> by_id;
    for (size_t i = 0; i < matches.size(); ++i) {
      const map::PointId id = proj[matches[i].wordsIdx].id();
      const auto it = by_id.find(id);
      if (it == by_id.end()) {
        by_id.emplace(id, i);
      } else if (matches[i].descriptorDist < matches[it->second].descriptorDist) {
        it->second = i;
      }
    }
    std::vector<c8::PointMatch> dedup;
    dedup.reserve(by_id.size());
    for (const auto& kv : by_id) {
      dedup.push_back(matches[kv.second]);
    }
    libcxx::sort(dedup.begin(), dedup.end(), [](const c8::PointMatch& a, const c8::PointMatch& b) {
      return a.wordsIdx < b.wordsIdx;
    });
    matches = std::move(dedup);
  }
  if (matches.size() < static_cast<size_t>(w.match_minimum)) {
    return;
  }

  std::vector<uint8_t> matched_mask(proj.size(), 0);
  const c8::HMatrix& projected_pose = projected.extrinsic();
  for (const c8::PointMatch& pm : matches) {
    matched_mask[pm.wordsIdx] = 1;
    const map::PointId id = proj[pm.wordsIdx].id();
    const uint32_t dict = static_cast<uint32_t>(pm.dictionaryIdx);
    result->matched_ids.push_back(id);
    result->dict_map[dict] = id;
    result->all_matches.push_back({dict, id});
    result->word_indices.push_back(static_cast<uint32_t>(pm.wordsIdx));
    result->all_world_points.push_back(projected_pose * proj[pm.wordsIdx].position3());

    // §5.3 match weight (F1272:1333-1396).
    float weight = (256.0f - pm.descriptorDist) * (1.0f / 256.0f);
    const map::WorldPoint& p = map.Point(id);
    if (p.source == map::PointSource::kTriangulated) {
      if (w.weight_by_depth_confidence) {
        double certainty_sum = 0.0;
        for (const auto& obs : p.observations) {
          // 1272 indexes the keyframe slot directly. Observations only name live keyframes
          // (1144 unlinks them before freeing the slot), so the id-checked accessor returns the
          // same keyframe and turns a dangling id (an out-of-range read of a cleared frame in the
          // original) into an abort (S2 review LOW-6).
          const map::Keyframe& kf = map.GetKeyframe(obs.first);
          certainty_sum += kf.frame.points()[obs.second].depthWithCertainty().certainty;
        }
        const float mean_log = std::log10(static_cast<float>(
            certainty_sum / static_cast<double>(static_cast<uint32_t>(p.observations.size()))));
        const float t = (mean_log - w.untrusted_depth_certainty) /
                        (w.trusted_depth_certainty - w.untrusted_depth_certainty);
        // wasm select(0, f32.min(t, 1), t < 0): a NaN t stays NaN.
        const float clamped = t < 0.0f ? 0.0f : (std::isnan(t) ? t : (t < 1.0f ? t : 1.0f));
        weight = weight * (w.min_weight_for_depth_certainty + clamped * w.depth_weight_coef);
      }
    } else if (p.source == map::PointSource::kSky) {
      weight = weight * w.skypoint_weight_coef;
    }
    const c8::FramePoint& fp = f.points()[pm.dictionaryIdx];
    c8::ObservedPoint o;
    o.position = c8::HPoint2(fp.x(), fp.y());
    o.scale = pm.dictionaryScale;
    o.descriptorDist = pm.descriptorDist;
    o.weight = weight;
    result->all_obs.push_back(o);
  }

  SelectSubset(map, result);
  for (const TrackedMatch& sm : result->subset_matches) {
    result->num_loaded += map.Point(sm.id).loaded_from_map ? 1u : 0u;
  }

  float residual = FLT_MAX;
  if (result->subset_obs.size() >= static_cast<size_t>(w.match_minimum)) {
    const float r0 =
        MeanSquaredRayError(result->pose, result->subset_obs, result->subset_world_points);
    c8::HMatrix p = result->pose;
    c8::PoseEstimationAnalyticalParams params;
    params.suppressOutliers = true;
    params.scale = 1.0;
    params.useImageTargetCostFunction = false;
    params.cameraMotionWeight = 0.0;
    params.logReport = false;
    const std::deque<c8::HMatrix> last_two;
    c8::poseEstimationAnalytical(result->subset_obs, result->subset_world_points, last_two, &p,
                                 params, &result->bundle_out);
    const float r1 = MeanSquaredRayError(p, result->subset_obs, result->subset_world_points);
    residual = r0;
    if (r1 < r0) {
      result->pose = p;
      residual = r1;
    }
  }
  result->residual = residual;

  // verify-point-visibility with the final pose over the projected ids (F1272:2339-2429). A point
  // is reported visible when 1549 accepted at least one keyframe view of it: 1272 tests the
  // per-point view count 1549 keeps at result+24 (incremented per accepted view), not the status
  // word at result+60. Status 2 (behind the camera / outside the frame at the final pose) and 3
  // (no observation in scale range) have no views and are not reported.
  std::vector<map::PointId> projected_ids;
  projected_ids.reserve(proj.size());
  for (const c8::FramePoint& fp : proj) {
    projected_ids.push_back(fp.id());
  }
  const map::VisibilityResult vis =
      map::VerifyPointVisibility(map, intrinsics, result->pose, projected_ids);
  for (size_t i = 0; i < projected_ids.size(); ++i) {
    if (!vis.views[i].empty()) {
      if (visible_out != nullptr) {
        visible_out->push_back(projected_ids[i]);
      }
      if (matched_out != nullptr) {
        matched_out->push_back(matched_mask[i]);
      }
    }
  }
}

}  // namespace slam_refact::map_tracker
