// EpipolarMatcher (func 1112) with its closed helpers 1122/1123 and the 1120/1121 sort
// (architecture.md §3.6; S3 §3.3, §3.3.1).
//
// Citations: 1112.c:L = build/slam-refact-20261015/spec/map-builder/annotated/1112.c (Ghidra C),
// DCMP = re/wabt/xr-slam.dcmp, OBJ = wasm-objdump -d of the pinned xr-slam.wasm.
// The public parts (epipolarDepthImagePrework 1115, epipolarDepthPointPrework 1116,
// depthOfEpipolarPoint 1118, BinMap::binsOnLine 1117, LocalMatcher::findMatches 1128/1129) are the
// vendored reference runtime code. Differences of the closed code from the public LocalMatcher are noted
// inline (VERIFIED from 1112.c).

#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <vector>

#include "c8/geometry/egomotion.h"
#include "c8/hmatrix.h"
#include "c8/hpoint.h"
#include "c8/hvector.h"
#include "common/c8_adapters.h"
#include "common/check.h"
#include "common/stage_timer.h"
#include "compat/libcxx_algorithms.h"
#include "config/settings.h"
#include "map/world_map.h"
#include "map/world_map_internal.h"
#include "map_builder/map_builder.h"
#include "map_builder/map_builder_internal.h"
#include "reality/engine/features/feature-scales.h"
#include "reality/engine/features/image-descriptor.h"
#include "reality/engine/features/local-matcher.h"
#include "reality/engine/geometry/epipolar.h"

namespace slam_refact::map_builder {

namespace {

using map::PointId;
using map::PointSource;

// match-new-points LocalMatcher constants (1112.c:2262, 0x3d4ccccd / 0x42000000; not config).
constexpr float kNewPointsMatchRadius = 0.05f;
constexpr float kNewPointsDescriptorDistance = 32.0f;

std::vector<map::Keyframe>& Slots(map::WorldMap& wm) {
  return map::WorldMapAccess::impl(wm).keyframes;
}

bool KeyframeValid(map::WorldMap& wm, map::KeyframeId id) {
  const std::vector<map::Keyframe>& slots = Slots(wm);
  return id.id() != 0 && id.index() < slots.size() && slots[id.index()].id == id.id();
}

// 634-style lookup without abort: the point source when the id names a live pool slot, else 0.
// The binary scans pools TRI..CLOUD for a slot with this id (1112.c:1788-1820); state is not
// checked (a frame point never refers to a retired point).
int32_t PointTypeOrZero(const map::WorldMap& wm, PointId id) {
  if (id.id() == 0) {
    return 0;
  }
  const auto& pools = map::WorldMapAccess::impl(wm).pools;
  for (const map::PointPool& pool : pools) {
    if (id.index() < pool.points.size() && pool.points[id.index()].id == id.id()) {
      return static_cast<int32_t>(pool.points[id.index()].source);
    }
  }
  return 0;
}

bool IsGroundOrDepth(int32_t t) { return t == 2 || t == 4; }
bool IsUntriangulated(int32_t t) { return t == 2 || t == 3 || t == 4; }

// sqrt(dz^2 + dy^2 + dx^2 + 0) of (point / w - P) (1112.c:1936-1945).
float DistanceToPoint(const map::WorldPoint& p, const c8::HPoint3& q) {
  const auto& r = p.position.raw();
  const float dz = r[2] / r[3] - q.raw()[2];
  const float dy = r[1] / r[3] - q.raw()[1];
  const float dx = r[0] / r[3] - q.raw()[0];
  return std::sqrt(dz * dz + dy * dy + dx * dx + 0.0f);
}

template <typename Descriptor>
std::shared_ptr<void>& EpipolarCache(map::Keyframe& kf);
template <>
std::shared_ptr<void>& EpipolarCache<c8::OrbFeature>(map::Keyframe& kf) {
  return kf.epipolar_matcher_orb;
}
template <>
std::shared_ptr<void>& EpipolarCache<c8::GorbFeature>(map::Keyframe& kf) {
  return kf.epipolar_matcher_gorb;
}

template <typename Descriptor>
std::shared_ptr<void>& NewPointsCache(map::Keyframe& kf);
template <>
std::shared_ptr<void>& NewPointsCache<c8::OrbFeature>(map::Keyframe& kf) {
  return kf.new_points_matcher_orb;
}
template <>
std::shared_ptr<void>& NewPointsCache<c8::GorbFeature>(map::Keyframe& kf) {
  return kf.new_points_matcher_gorb;
}

// "match-keyframes" + "epipolar-match" + "sort-and-filter-duplicates" (1112.c:296-1745):
// words = keyframe a, dictionary = keyframe b.
template <typename Descriptor>
std::vector<c8::PointMatch> MatchKeyframes(map::Keyframe& a, map::Keyframe& b) {
  StageTimer match_timer("match-keyframes");
  std::shared_ptr<void>& cache = EpipolarCache<Descriptor>(b);
  if (cache == nullptr) {
    auto created = std::make_shared<EpipolarKeyframeMatcher>();
    created->radius = config::EpipolarMatcherSettings::Get().radius;
    c8::HPoint2 lo;
    c8::HPoint2 hi;
    b.frame.frameBoundsExcludingEdge(&lo, &hi);
    created->map.reset<c8::FramePoint>(lo.x(), lo.y(), hi.x() + 0.0f, hi.y() + 0.0f, b.frame.points(),
                       &b.frame.store().keypointIndices<Descriptor>());  // 1114
    cache = created;
  }
  auto* matcher = static_cast<EpipolarKeyframeMatcher*>(cache.get());
  matcher->dictionary = &b.frame;

  StageTimer epipolar_timer("epipolar-match");
  const c8::FrameWithPoints& words = a.frame;
  const c8::FrameWithPoints& dict = *matcher->dictionary;
  const float radius = matcher->radius;
  const float max_distance =
      static_cast<float>(config::EpipolarMatcherSettings::Get().descriptor_distance);
  const c8::EpipolarDepthImagePrework iw =
      c8::epipolarDepthImagePrework(words.extrinsic(), dict.extrinsic());  // 1115
  const c8::Vector<size_t>& words_kp = words.store().keypointIndices<Descriptor>();
  const auto& words_desc = words.store().getFeatures<Descriptor>();

  // Dedup by dictionary index like LocalMatcher::findMatches, except that a strictly better
  // match also removes the index from the suppressed set (1112.c:669-746; the public code keeps
  // it suppressed).
  std::map<size_t, c8::PointMatch> dedup;
  std::set<size_t> suppressed;
  for (size_t d = 0; d < words_desc.size(); ++d) {
    const c8::FramePoint& wp = words.points()[words_kp[d]];
    const c8::EpipolarDepthPointPrework pw =
        c8::epipolarDepthPointPrework(iw, c8::HPoint2(wp.x(), wp.y()));  // 1116
    matcher->map.binsOnLine(pw.lineSegment, radius, &matcher->bins);    // 1117
    c8::PointMatch best;
    c8::PointMatch second;
    for (size_t bin : matcher->bins) {
      for (size_t i : matcher->map.map_[bin]) {
        const c8::FramePoint& dp = dict.points()[i];
        const c8::HVector2 c = c8::closestLineSegmentPoint(pw, c8::HVector2(dp.x(), dp.y()));
        const float ex = c.x() - dp.x();
        const float ey = c.y() - dp.y();
        if (!(ex * ex + ey * ey <= radius * radius)) {
          continue;
        }
        const c8::EpipolarDepthResult r = c8::depthOfEpipolarPoint(iw, pw, c, dp.scale());  // 1118
        if (r.status == c8::EpipolarDepthResult::Status::OK) {
          // Predicted dictionary scale: z of the words point at that depth in the dictionary
          // camera over dMinForScale(words scale, depth) (1112.c:451-475).
          const double depth = r.depthInWords;
          const c8::HPoint3 q =
              iw.projectToDictionary *
              c8::HPoint3(static_cast<float>(depth * static_cast<double>(wp.x())),
                          static_cast<float>(depth * static_cast<double>(wp.y())),
                          static_cast<float>(depth));
          const float z_dict = q.raw()[2] / q.raw()[3];
          const uint8_t predicted =
              c8::scaleForDist(c8::dMinForScale(wp.scale(), static_cast<float>(depth)), z_dict);
          if (predicted != dp.scale()) {
            continue;
          }
        } else if (wp.scale() != dp.scale()) {
          continue;
        }
        const float h = static_cast<float>(
            words_desc[d].hammingDistance(dict.store().get<Descriptor>(i)));
        if (max_distance < h) {
          continue;
        }
        // findNearestTwoNeighbors update order.
        if (h < second.descriptorDist) {
          second.dictionaryIdx = i;
          second.dictionaryScale = dp.scale();
          second.descriptorDist = h;
          second.rotation = 0.0f;
        }
        if (second.descriptorDist < best.descriptorDist) {
          std::swap(best, second);
        }
      }
    }
    const float threshold = matcher->use_ratio_test
                                ? second.descriptorDist * c8::LocalMatcher<Descriptor>::RATIO_TEST_TH
                                : second.descriptorDist;
    if (!(best.descriptorDist < threshold) || best.dictionaryIdx == static_cast<size_t>(-1)) {
      continue;
    }
    best.wordsIdx = words_kp[d];
    auto found = dedup.find(best.dictionaryIdx);
    if (found == dedup.end()) {
      dedup[best.dictionaryIdx] = best;
    } else if (found->second.descriptorDist == best.descriptorDist) {
      suppressed.insert(best.dictionaryIdx);
    } else if (best.descriptorDist < found->second.descriptorDist) {
      found->second = best;
      suppressed.erase(best.dictionaryIdx);
    }
  }
  std::vector<c8::PointMatch> matches;
  matches.reserve(dedup.size());
  for (const auto& [dict_idx, m] : dedup) {
    if (suppressed.count(dict_idx) == 0) {
      matches.push_back(m);
    }
  }

  StageTimer sort_timer("sort-and-filter-duplicates");
  SortEpipolarMatches(&matches);
  // Keep a match only when neither its words nor its dictionary index was used (1112.c:1636-1745).
  std::set<size_t> used_words;
  std::set<size_t> used_dict;
  std::vector<c8::PointMatch> kept;
  for (const c8::PointMatch& m : matches) {
    if (used_words.count(m.wordsIdx) != 0 || used_dict.count(m.dictionaryIdx) != 0) {
      continue;
    }
    used_words.insert(m.wordsIdx);
    used_dict.insert(m.dictionaryIdx);
    kept.push_back(m);
  }
  return kept;
}

// Triangulate one kept match and merge it into the map (1112.c:1785-2223, S3 §3.3.1).
// Appends to *new_pts when a fresh TRIANGULATED point is created for two unmapped observations.
void TriangulateAndMerge(map::WorldMap& wm, map::KeyframeId a_id, map::KeyframeId b_id,
                         const RayPairPrework& pre_ab, const RayPairPrework& pre_ba,
                         const c8::HMatrix& a_inv, const c8::HMatrix& b_inv,
                         const c8::HMatrix& ego_ab, const c8::PointMatch& m,
                         std::vector<PointId>* new_pts) {
  const config::MapBuilderSettings& s = config::MapBuilderSettings::Get();
  map::Keyframe& a = Slots(wm)[a_id.index()];
  map::Keyframe& b = Slots(wm)[b_id.index()];
  const uint32_t b_obs = static_cast<uint32_t>(m.dictionaryIdx);
  const uint32_t a_obs = static_cast<uint32_t>(m.wordsIdx);
  const c8::FramePoint& bp = b.frame.points()[b_obs];
  const c8::FramePoint& ap = a.frame.points()[a_obs];
  const PointId b_point = bp.id();
  const PointId a_point = ap.id();
  const int32_t a_type = PointTypeOrZero(wm, a_point);
  const int32_t b_type = PointTypeOrZero(wm, b_point);
  const bool a_exists = a_type != 0;
  const bool b_exists = b_type != 0;

  const c8::HPoint3 ray_a(ap.x(), ap.y(), 1.0f);
  const c8::HPoint3 ray_b(bp.x(), bp.y(), 1.0f);
  const c8::HPoint3 o1 = RayPairPoint(ray_a, ray_b, pre_ab);  // 1123 (A, B)
  const c8::HPoint3 o2 = RayPairPoint(ray_b, ray_a, pre_ba);  // 1123 (B, A)
  SR_CHECK(a_type != static_cast<int32_t>(PointSource::kCloud) &&
               b_type != static_cast<int32_t>(PointSource::kCloud),
           "[map_builder] cloud points are not supported");

  // Midpoint (1112.c:1902-1905).
  const auto& r1 = o1.raw();
  const auto& r2 = o2.raw();
  const float px = (r1[0] / r1[3] + r2[0] / r2[3]) * 0.5f;
  const float py = (r1[1] / r1[3] + r2[1] / r2[3]) * 0.5f;
  const float pz = (r1[2] / r1[3] + r2[2] / r2[3]) * 0.5f;
  const c8::HPoint3 p(px, py, pz);

  // Rotation-only parallax: |flatten(R_ab^T proj_a(P)) - proj_b(P)| (1112.c:1906-1930).
  const c8::HPoint3 ca = a_inv * p;
  const float xa = ca.raw()[0] / ca.raw()[2];
  const float ya = ca.raw()[1] / ca.raw()[2];
  const c8::HPoint3 cb = b_inv * p;
  const float xb = cb.raw()[0] / cb.raw()[2];
  const float yb = cb.raw()[1] / cb.raw()[2];
  const auto& e = ego_ab.data();
  const float hz = e[10] + e[8] * xa + ya * e[9] + 0.0f;
  const float dx = (e[2] + e[0] * xa + ya * e[1] + 0.0f) / hz - xb;
  const float dy = (e[6] + e[4] * xa + ya * e[5] + 0.0f) / hz - yb;
  if (!(s.min_parallax_threshold <= std::sqrt(dx * dx + dy * dy))) {
    return;
  }
  const float max_dist = s.existing_to_triangulated_point_max_distance;

  if (a_exists && b_exists) {
    if (a_point == b_point) {
      if (IsGroundOrDepth(b_type) && DistanceToPoint(wm.Point(a_point), p) <= max_dist) {
        const PointId fresh = wm.AddPoint(b.seq, p, PointSource::kTriangulated);
        wm.MergePoint(a_point, fresh);  // 1124
      }
      return;
    }
    const bool a_untri = IsUntriangulated(a_type);
    const bool b_untri = IsUntriangulated(b_type);
    if (a_untri && b_untri) {
      const PointId fresh = wm.AddPoint(b.seq, p, PointSource::kTriangulated);
      wm.MergePoint(b_point, fresh);
      wm.MergePoint(a_point, fresh);
    } else if (b_untri) {
      wm.MergePoint(b_point, a_point);
    } else if (a_untri) {
      wm.MergePoint(a_point, b_point);
    } else if (wm.Point(a_point).observations.size() <= wm.Point(b_point).observations.size()) {
      wm.MergePoint(a_point, b_point);  // fewer observations into more; tie A -> B
    } else {
      wm.MergePoint(b_point, a_point);
    }
    return;
  }
  if (a_exists) {
    if (IsGroundOrDepth(a_type)) {
      if (DistanceToPoint(wm.Point(a_point), p) <= max_dist) {
        const PointId fresh = wm.AddPoint(a.seq, p, PointSource::kTriangulated);
        wm.AddObservation(fresh, b_obs, b_id);
        wm.MergePoint(a_point, fresh);
      }
    } else {
      wm.AddObservation(a_point, b_obs, b_id);
    }
    return;
  }
  if (b_exists) {
    if (IsGroundOrDepth(b_type)) {
      if (DistanceToPoint(wm.Point(b_point), p) <= max_dist) {
        const PointId fresh = wm.AddPoint(b.seq, p, PointSource::kTriangulated);
        wm.AddObservation(fresh, a_obs, a_id);
        wm.MergePoint(b_point, fresh);
      }
    } else {
      wm.AddObservation(b_point, a_obs, a_id);
    }
    return;
  }
  const PointId fresh = wm.AddPoint(b.seq, p, PointSource::kTriangulated);
  map::WorldPoint& fp = wm.MutablePoint(fresh);
  ++fp.tracking_count;
  ++fp.tracking_hits;
  new_pts->push_back(fresh);
  wm.AddObservation(fresh, b_obs, b_id);
  wm.AddObservation(fresh, a_obs, a_id);
}

// "match-new-points" (1112.c:2226-2494): every keyframe slot except the dictionary keyframe B
// (compared by id; keyframe A and free slots are not skipped, which answers S3 U7) projects the
// new points and matches them with a LocalMatcher(15x20, radius 0.05, distance 32) over its frame.
template <typename Descriptor>
void MatchNewPoints(map::WorldMap& wm, uint32_t b_id, const std::vector<PointId>& new_pts) {
  StageTimer timer("match-new-points");
  std::vector<map::Keyframe>& slots = Slots(wm);
  for (size_t slot = 0; slot < slots.size(); ++slot) {
    map::Keyframe& k = slots[slot];
    if (k.id == b_id) {
      continue;
    }
    const map::ProjectedPoints proj = map::ProjectPoints(
        wm, new_pts, ToPinholeIntrinsics(k.frame.intrinsic()), k.frame.extrinsic());  // 632
    std::shared_ptr<void>& cache = NewPointsCache<Descriptor>(k);
    if (cache == nullptr) {
      auto created = std::make_shared<c8::LocalMatcher<Descriptor>>(
          15, 20, kNewPointsMatchRadius, kNewPointsDescriptorDistance);
      created->setQueryPointsPointer(k.frame);  // 1114
      cache = created;
    }
    auto* matcher = static_cast<c8::LocalMatcher<Descriptor>*>(cache.get());
    matcher->ensureQueryPointsPointer(k.frame);
    c8::Vector<c8::PointMatch> matches;
    matcher->findMatches(proj.frame, &matches);  // 1128 (ORB) / 1129 (GORB)
    std::vector<uint8_t> matched(proj.frame.points().size(), 0);
    for (const c8::PointMatch& m : matches) {
      matched[m.wordsIdx] = 1;
      wm.AddObservation(proj.frame.points()[m.wordsIdx].id(),
                        static_cast<uint32_t>(m.dictionaryIdx), map::KeyframeId(slot, k.id));
    }
    for (size_t i = 0; i < matched.size(); ++i) {
      map::WorldPoint& p = wm.MutablePoint(proj.frame.points()[i].id());  // 1125
      ++p.tracking_count;
      if (matched[i] != 0) {
        ++p.tracking_hits;
      }
    }
  }
}

// One keyframe pair (1112.c:296-2535) for one descriptor type.
template <typename Descriptor>
void ProcessPair(map::WorldMap& wm, map::KeyframeId a_id, map::KeyframeId b_id) {
  StageTimer timer("gen-map-pts");
  map::Keyframe& a = Slots(wm)[a_id.index()];
  map::Keyframe& b = Slots(wm)[b_id.index()];
  const std::vector<c8::PointMatch> kept = MatchKeyframes<Descriptor>(a, b);
  if (kept.empty()) {
    return;
  }
  const c8::HMatrix pose_a = a.frame.extrinsic();
  const c8::HMatrix pose_b = b.frame.extrinsic();
  const RayPairPrework pre_ab = MakeRayPairPrework(pose_a, pose_b);  // 1122
  const RayPairPrework pre_ba = MakeRayPairPrework(pose_b, pose_a);
  const c8::HMatrix ego_ab = c8::egomotion(pose_a, pose_b);          // 660
  const c8::HMatrix a_inv = pose_a.inv();
  const c8::HMatrix b_inv = pose_b.inv();
  std::vector<PointId> new_pts;
  for (const c8::PointMatch& m : kept) {
    TriangulateAndMerge(wm, a_id, b_id, pre_ab, pre_ba, a_inv, b_inv, ego_ab, m, &new_pts);
  }
  if (new_pts.empty()) {
    return;
  }
  wm.AdjustDirtyPoints();  // 1127
  MatchNewPoints<Descriptor>(wm, b_id.id(), new_pts);
  for (PointId id : new_pts) {
    SR_CHECK(id.id() != 0, "Invalid map point");
    if (wm.Point(id).tracking_hits < 2) {
      wm.Retire(id);  // 1130(..., 2)
    }
  }
}

}  // namespace

// ---------------------------------------------------------------------------------------------
// 1122 / 1123

RayPairPrework MakeRayPairPrework(const c8::HMatrix& first, const c8::HMatrix& second) {
  RayPairPrework w;
  // E = essentialMatrixForCameras(first, second) = [t]x R of proj = egomotion(first, second).inv()
  // (DCMP:138798-138813, operand order as decompiled).
  const c8::HMatrix cam = c8::egomotion(first, second);
  const c8::HMatrix proj = cam.inv();
  const auto& iv = proj.data();
  const float t = iv[0], j = iv[1], i = iv[2];
  const float l = iv[4], m = iv[5], k = iv[6];
  const float o = iv[8], p = iv[9], n = iv[10];
  const float r = iv[12], s = iv[13], q = iv[14];
  std::array<float, 16>& e = w.essential;
  const float u1 = r * 0.0f;
  const float v1 = s * 0.0f;
  e[14] = (u1 - v1) + 0.0f;
  e[13] = ((q * 0.0f + 0.0f) - u1) + 0.0f;
  e[12] = ((v1 + q * -0.0f) + 0.0f) + 0.0f;
  const float u2 = n * 0.0f;
  const float v2 = o * 0.0f;
  const float w2 = p * 0.0f;
  e[11] = ((u2 + v2) + w2) + 0.0f;
  e[10] = ((u2 + r * p) - s * o) + 0.0f;
  e[9] = ((q * o + w2) - r * n) + 0.0f;
  e[8] = ((s * n + v2) - p * q) + 0.0f;
  const float n3 = k * 0.0f;
  const float o3 = l * 0.0f;
  const float p3 = m * 0.0f;
  e[7] = ((n3 + o3) + p3) + 0.0f;
  e[6] = ((n3 + r * m) - s * l) + 0.0f;
  e[5] = ((q * l + p3) - r * k) + 0.0f;
  e[4] = ((s * k + o3) - m * q) + 0.0f;
  const float k4 = i * 0.0f;
  const float l4 = t * 0.0f;
  const float m4 = j * 0.0f;
  e[3] = ((k4 + l4) + m4) + 0.0f;
  e[2] = ((k4 + r * j) - s * t) + 0.0f;
  e[1] = ((q * t + m4) - r * i) + 0.0f;
  e[0] = ((s * i + l4) - j * q) + 0.0f;
  // m[15] keeps the identity initialization of the output HMatrix (1.0, as [t]x of the public
  // HMatrixGen::cross has [3][3] = 1); 1123 multiplies this column by 0.
  e[15] = 1.0f;
  // E.t() of a noInvert matrix: plain transpose (func 1579).
  for (int c = 0; c < 4; ++c) {
    for (int rr = 0; rr < 4; ++rr) {
      w.essential_t[c * 4 + rr] = e[rr * 4 + c];
    }
  }
  // rotationMat(first) and translation(first).
  const auto& b = first.data();
  w.rotation_first = {b[0], b[1], b[2], 0.0f, b[4], b[5], b[6], 0.0f,
                      b[8], b[9], b[10], 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  w.center_first = {b[12], b[13], b[14]};
  // Block of the second pose (DCMP:138860-138895, statement order kept).
  const auto& c = second.data();
  const float dz = c[14] - b[14];
  const float dy = c[13] - b[13];
  const float dx = c[12] - b[12];
  const float w10 = c[10] * 0.0f;
  const float x8 = c[8] * 0.0f;
  const float y9 = c[9] * 0.0f;
  const float z6 = c[6] * 0.0f;
  const float a4 = c[4] * 0.0f;
  const float b5 = c[5] * 0.0f;
  const float d2 = c[2] * 0.0f;
  const float e0 = c[0] * 0.0f;
  const float f1 = c[1] * 0.0f;
  std::array<float, 16>& g = w.second_block;
  g[10] = (c[10] + (x8 + y9)) + 0.0f;
  g[9] = ((w10 + x8) + c[9]) + 0.0f;
  g[8] = ((w10 + c[8]) + y9) + 0.0f;
  g[6] = (c[6] + (a4 + b5)) + 0.0f;
  g[5] = ((z6 + a4) + c[5]) + 0.0f;
  g[4] = ((z6 + c[4]) + b5) + 0.0f;
  g[2] = (c[2] + (e0 + f1)) + 0.0f;
  g[1] = ((d2 + e0) + c[1]) + 0.0f;
  g[0] = ((d2 + c[0]) + f1) + 0.0f;
  g[15] = ((dy * -0.0f - dx * 0.0f) - dz * 0.0f) + 1.0f;
  const float ndx = -dx;
  g[11] = ((c[8] * ndx - c[9] * dy) - c[10] * dz) + 0.0f;
  g[7] = ((c[4] * ndx - c[5] * dy) - c[6] * dz) + 0.0f;
  g[3] = ((c[0] * ndx - c[1] * dy) - c[2] * dz) + 0.0f;
  g[12] = 0.0f;
  g[13] = 0.0f;
  g[14] = 0.0f;
  return w;
}

c8::HPoint3 RayPairPoint(const c8::HPoint3& ray_first, const c8::HPoint3& ray_second,
                         const RayPairPrework& w) {
  // Instruction order of OBJ func[1123]; each f32x4 lane is written out.
  const auto& r1 = ray_first.raw();
  const auto& r2 = ray_second.raw();
  const auto& e = w.essential;
  const auto& et = w.essential_t;
  const auto& rf = w.rotation_first;
  const auto& g = w.second_block;
  const float n = r2[0] / r2[2];
  const float o = r2[1] / r2[2];
  // v = Et * (n, o, 1, 0) = c3*0 + (c2 + (c0*n + c1*o)).
  float v[3];
  for (int i = 0; i < 3; ++i) {
    v[i] = et[12 + i] * 0.0f + (et[8 + i] + (et[i] * n + et[4 + i] * o));
  }
  const float k = r1[0] / r1[2];
  const float l = r1[1] / r1[2];
  const float u16 = v[0] - k * 0.0f;
  const float u17 = l * 0.0f - v[1];
  const float u18 = k * v[1] - v[0] * l;
  const float p1 = v[0] * u16 - u17 * v[1];
  const float p0 = v[1] * u18 - u16 * v[2];
  const float p2 = v[2] * u17 - u18 * v[0];
  // wv = Rf * (p0, p2, p1): c3*0 + (c2*p1 + (c0*p0 + c1*p2)).
  float wv[3];
  for (int i = 0; i < 3; ++i) {
    wv[i] = rf[12 + i] * 0.0f + (rf[8 + i] * p1 + (rf[i] * p0 + rf[4 + i] * p2));
  }
  // ev = E * (k, l, 1, 0).
  float ev[2];
  for (int i = 0; i < 2; ++i) {
    ev[i] = e[12 + i] * 0.0f + (e[8 + i] + (e[i] * k + e[4 + i] * l));
  }
  const float g0 = n * ev[1] - ev[0] * o;
  const float g1 = o * 0.0f - ev[1];
  const float g2 = ev[0] - n * 0.0f;
  const float num = -(g[11] * g0 + (g[3] * g1 + g[7] * g2));
  // hv = G * (g1, g2, g0): c3*0 + (c2*g0 + (c0*g1 + c1*g2)).
  float hv[3];
  for (int i = 0; i < 3; ++i) {
    hv[i] = g[12 + i] * 0.0f + (g[8 + i] * g0 + (g[i] * g1 + g[4 + i] * g2));
  }
  const float den = wv[2] * hv[2] + (wv[1] * hv[1] + (wv[0] * hv[0] + 0.0f));
  const float pr = num / den;
  c8::HPoint3 out;
  out.raw()[0] = w.center_first[0] + wv[0] * pr;
  out.raw()[1] = w.center_first[1] + wv[1] * pr;
  out.raw()[2] = w.center_first[2] + wv[2] * pr;
  out.raw()[3] = 1.0f;
  return out;
}

// ---------------------------------------------------------------------------------------------
// 1120 / 1121

bool EpipolarMatchLess(const c8::PointMatch& a, const c8::PointMatch& b) {
  if (a.descriptorDist != b.descriptorDist) {
    return a.descriptorDist < b.descriptorDist;
  }
  if (a.dictionaryScale != b.dictionaryScale) {
    return a.dictionaryScale < b.dictionaryScale;
  }
  if (static_cast<uint32_t>(a.wordsIdx) != static_cast<uint32_t>(b.wordsIdx)) {
    return static_cast<uint32_t>(a.wordsIdx) < static_cast<uint32_t>(b.wordsIdx);
  }
  return static_cast<uint32_t>(a.dictionaryIdx) < static_cast<uint32_t>(b.dictionaryIdx);
}

void SortEpipolarMatches(std::vector<c8::PointMatch>* matches) {
  slam_refact::libcxx::sort(matches->begin(), matches->end(), EpipolarMatchLess);
}

// ---------------------------------------------------------------------------------------------
// 1112

void EpipolarMatcher(map::WorldMap& wm, std::vector<KfPair>* queue, bool vps_mode) {
  int processed = 0;
  while (processed < config::MapBuilderSettings::Get().keyframe_pair_process_count &&
         !queue->empty()) {
    StageTimer timer("new-pts");
    const KfPair pair = queue->back();  // LIFO
    if (!KeyframeValid(wm, pair.a) || !KeyframeValid(wm, pair.b_newest)) {
      queue->pop_back();  // not counted
      continue;
    }
    map::Keyframe& a = Slots(wm)[pair.a.index()];
    map::Keyframe& b = Slots(wm)[pair.b_newest.index()];
    if (vps_mode) {
      // 1112.c:2545-2563: centers of B (iVar84) and A (iVar59).
      const auto& ta = a.frame.extrinsic().data();
      const auto& tb = b.frame.extrinsic().data();
      const float dz = ta[14] - tb[14];
      const float dy = ta[13] - tb[13];
      const float dx = ta[12] - tb[12];
      if (!(config::WorldMapsSettings::Get().min_keyframe_pair_baseline_for_vps_triangulation <=
            std::sqrt(dz * dz + dy * dy + dx * dx + 0.0f))) {
        queue->pop_back();
        continue;
      }
    }
    const c8::FeatureStore& sa = a.frame.store();
    const c8::FeatureStore& sb = b.frame.store();
    const bool use_orb = sb.numDescriptors<c8::GorbFeature>() + sa.numDescriptors<c8::GorbFeature>() <
                         sb.numDescriptors<c8::OrbFeature>() + sa.numDescriptors<c8::OrbFeature>();
    if (use_orb) {
      ProcessPair<c8::OrbFeature>(wm, pair.a, pair.b_newest);
    } else {
      ProcessPair<c8::GorbFeature>(wm, pair.a, pair.b_newest);
    }
    wm.AdjustDirtyPoints();  // 1127
    queue->pop_back();
    ++processed;
  }
}

}  // namespace slam_refact::map_builder
