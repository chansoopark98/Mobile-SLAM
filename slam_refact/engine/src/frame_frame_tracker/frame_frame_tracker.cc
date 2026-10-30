// FrameFrameTracker::track (1266), classifyHorizon (1267), depth point indices (1269) —
// architecture.md §3.5, S4 §3.7. Read from DCMP func1266 / G1266 and the func1266 disassembly
// (operation order of the height update and the epipolar residual); RNG use per WP2-PRE §1.3.

#include "frame_frame_tracker/frame_frame_tracker.h"

#include <cfloat>
#include <cmath>
#include <utility>

#include "c8/geometry/egomotion.h"
#include "c8/hvector.h"
#include "common/check.h"
#include "config/settings.h"
#include "frame_frame_tracker/frame_frame_tracker_internal.h"
#include "frame_frame_tracker/ground_homography.h"
#include "reality/engine/geometry/p2p.h"

namespace slam_refact::frame_frame_tracker {

namespace {

using Impl = FrameFrameTrackerAccess::Impl;

constexpr size_t kInitialDepthTrials = 459;        // computeNeededTrials(0.1), S4 §5.8
constexpr float kMinPairRayDistance = 0.05f;       // f32 0x1.99999ap-5
constexpr float kLogOneMinusConfidence = -0x1.26bb2p+2f;  // logf(1 - 0.99f)
constexpr float kMaxNeededTrials = 2500.0f;
constexpr uint32_t kMinNeededTrials = 100;
constexpr double kMinEpipolarLineNorm = 1e-9;

// Lane-wise ray of an HPoint2 (x / w, y / w).
float RayX(const c8::HPoint2& r) { return r.raw()[0] / r.raw()[2]; }
float RayY(const c8::HPoint2& r) { return r.raw()[1] / r.raw()[2]; }

// RANSAC trial count of the depth egomotion (inline computeNeededTrials, DCMP func1266).
uint32_t NeededTrials(uint32_t inliers, uint32_t matches) {
  const float p = static_cast<float>(inliers) / static_cast<float>(matches);
  float denom = 1.0f - p * p + -0x1.0c6f7ap-20f;
  if (!(denom > FLT_MIN)) {
    denom = FLT_MIN;
  }
  float needed = std::ceil(kLogOneMinusConfidence / std::log(denom));
  if (!(needed < kMaxNeededTrials)) {
    needed = kMaxNeededTrials;
  }
  uint32_t n = (needed < 4294967296.0f && needed >= 0.0f) ? static_cast<uint32_t>(needed) : 0u;
  return n > kMinNeededTrials ? n : kMinNeededTrials;
}

}  // namespace

// ---------------------------------------------------------------------------------------------

std::vector<int32_t> ClassifyHorizon(const c8::Quaternion& q, const std::vector<c8::HPoint3>& rays) {
  const c8::HMatrix m = GravityNormalRotation(q);
  const std::vector<c8::HPoint3> lifted = m * rays;
  std::vector<int32_t> out;
  out.reserve(lifted.size());
  for (const c8::HPoint3& p : lifted) {
    const float v = p.raw()[2] / p.raw()[3];
    int32_t c;
    if (v >= 0.2f) {
      c = 1;
    } else if (v > 0.0f) {
      c = 2;
    } else if (v == 0.0f) {
      c = 3;
    } else if (v > -0.2f) {
      c = 4;
    } else {
      c = 5;
    }
    out.push_back(c);
  }
  return out;
}

std::vector<int32_t> ClassifyHorizonRays(const c8::Quaternion& q,
                                         const std::vector<c8::HPoint2>& rays) {
  return ClassifyHorizon(q, c8::extrude<3>(rays));
}

std::vector<std::pair<uint32_t, uint32_t>> ShuffledDepthPairs(uint32_t n,
                                                              compat::RandomNumbers* rng) {
  std::vector<std::pair<uint32_t, uint32_t>> pairs;
  for (uint32_t i = 0; i + 1 < n; ++i) {
    for (uint32_t j = i + 1; j < n; ++j) {
      pairs.emplace_back(i, j);
    }
  }
  rng->shuffle(pairs.begin(), pairs.end());
  return pairs;
}

std::vector<uint32_t> DepthPointIndices(size_t n) {
  const config::FrameFrameTrackerSettings& s = config::FrameFrameTrackerSettings::Get();
  std::vector<uint32_t> out;
  const uint32_t fraction = static_cast<uint32_t>(s.depth_points_fraction);
  SR_CHECK(fraction != 0, "[frame-frame-tracker] depthPointsFraction is 0");
  out.reserve(n / fraction);
  if (s.include_depth_points) {
    for (uint32_t c = 0; c < n; ++c) {
      if (c % fraction == 0) {
        out.push_back(c);
      }
    }
  }
  return out;
}

// ---------------------------------------------------------------------------------------------

FrameFrameTracker::FrameFrameTracker() : impl_(std::make_unique<Impl>()) {}

FrameFrameTracker::~FrameFrameTracker() = default;

float FrameFrameTracker::height() const { return impl_->height; }

void FrameFrameTracker::set_height(float height) { impl_->height = height; }

// 664 map-reset event: +0xb50 = 1, 913 on the previous frame, previous rays and classes cleared
// (DCMP func664); 1229 is empty.
void FrameFrameTracker::ResetPartial() {
  impl_->height = 1.0f;
  impl_->prev_frame.clear();
  impl_->prev_rays.clear();
  impl_->prev_classes.clear();
}

void FrameFrameTracker::SetPrevDepths(const std::vector<float>& depth_by_frame_point) {
  impl_->prev_depths.Assign(depth_by_frame_point);
}

int FrameFrameTracker::last_ground_match_count() const {
  return static_cast<int>(impl_->ground_matches.size());
}

compat::RandomNumbers* FrameFrameTracker::rng() { return &impl_->rng; }

// 1266.
void FrameFrameTracker::Track(c8::HMatrix* delta_out, const c8::FrameWithPoints& cur,
                              const c8::Quaternion& q_cur, std::vector<uint32_t>* ground_idx,
                              std::vector<uint32_t>* sky_idx, std::vector<uint32_t>* depth_idx,
                              bool* success) {
  Impl& t = *impl_;
  const config::FrameFrameTrackerSettings& s = config::FrameFrameTrackerSettings::Get();

  // ---- local-matcher: words = current frame, dictionary = previous frame (GORB first) ----
  t.matches.clear();
  c8::MatchResultFrequency freq;
  if (cur.store().has<c8::GorbFeature>() && t.prev_frame.store().has<c8::GorbFeature>()) {
    t.gorb_matcher.setQueryPointsPointer(t.prev_frame);
    t.gorb_matcher.findMatches(cur.points(), cur.store(), &t.matches, &freq);
  } else if (cur.store().has<c8::OrbFeature>() && t.prev_frame.store().has<c8::OrbFeature>()) {
    t.orb_matcher.setQueryPointsPointer(t.prev_frame);
    t.orb_matcher.findMatches(cur.points(), cur.store(), &t.matches, &freq);
  }

  std::vector<c8::HPoint2> cur_rays;
  cur_rays.reserve(cur.points().size());
  for (const c8::FramePoint& p : cur.points()) {
    cur_rays.emplace_back(p.x(), p.y());
  }
  std::vector<int32_t> cur_classes = ClassifyHorizonRays(q_cur, cur_rays);

  t.ground_matches.clear();
  t.sky_matches.clear();
  for (const c8::PointMatch& m : t.matches) {
    SR_CHECK(m.dictionaryIdx < t.prev_classes.size(), "[frame-frame-tracker] match index");
    const int32_t prev_class = t.prev_classes[m.dictionaryIdx];
    if (prev_class == 1 && cur_classes[m.wordsIdx] == 1) {
      t.ground_matches.push_back(m);
    } else if (prev_class == 5 && cur_classes[m.wordsIdx] == 5) {
      t.sky_matches.push_back(m);
    }
  }

  t.depth_matches.clear();
  if (!t.prev_depths.empty()) {
    for (const c8::PointMatch& m : t.matches) {
      // The binary reads depths[dictionaryIdx] unchecked (G1266:454-459). The depth list can be
      // stale: MapTracker's inertial path re-clones prev_frame without clearing it (F926:1227-1251
      // leaves +0xf18 untouched) and SetPrevDepths only runs on a tracked frame. An index in
      // [size, capacity) reads the stale float left in the vector's storage by an earlier, longer
      // list (PrevDepthList models the libc++ storage; flip6-s2 frame 326: 16 depth matches in
      // the original vs 10 when treated as "no depth"). An index past the capacity is an
      // out-of-bounds heap read in the original (not reproducible); the engine treats it as "no
      // depth" (WP5b fix, replaces an abort the original does not have; US026-CC-1).
      if (t.prev_depths.Depth(m.dictionaryIdx) > 0.0f) {
        t.depth_matches.push_back(m);
      }
    }
  }

  *success = false;
  c8::HMatrix delta = c8::HMatrixGen::i();
  // Matrix used by the height update: best depth solution, or the ground camera motion.
  c8::HMatrix motion = c8::HMatrixGen::i();
  bool depth_done = false;

  // ---- depth-egomotion ----
  if (static_cast<int64_t>(t.depth_matches.size()) >=
      static_cast<int64_t>(s.min_depths_for_depth_egomotion)) {
    const c8::HMatrix r_rel = t.prev_q_b.inverse().times(q_cur).toRotationMat();
    const uint32_t num_matches = static_cast<uint32_t>(t.matches.size());
    const float pct = s.min2d_inliers_percent_for_depth_egomotion * static_cast<float>(num_matches);
    const uint32_t pct_count =
        (pct < 4294967296.0f && pct >= 0.0f) ? static_cast<uint32_t>(pct) : 0u;
    const uint32_t min2d = static_cast<uint32_t>(s.min2d_inliers_for_depth_egomotion);
    const uint32_t required = min2d > pct_count ? min2d : pct_count;

    // 3D points of the depth matches in the previous camera: ((x / w) * d, (d * y) / w, d).
    std::vector<c8::HPoint3> pts3;
    pts3.reserve(t.depth_matches.size());
    for (const c8::PointMatch& m : t.depth_matches) {
      const float d = t.prev_depths.Depth(m.dictionaryIdx);
      const auto& r = t.prev_rays[m.dictionaryIdx].raw();
      pts3.emplace_back(r[0] / r[2] * d, d * r[1] / r[2], d);
    }
    const std::vector<std::pair<uint32_t, uint32_t>> pairs =
        ShuffledDepthPairs(static_cast<uint32_t>(t.depth_matches.size()), &t.rng);

    const size_t num_pairs = pairs.size();
    size_t max_trials = num_pairs < kInitialDepthTrials ? num_pairs : kInitialDepthTrials;
    uint32_t best2d = 0;
    float best_max_res = 0.0f;
    c8::HMatrix best = c8::HMatrixGen::i();
    std::vector<uint8_t> best_mask(t.matches.size(), 0);
    std::vector<uint8_t> mask;
    size_t trials = 0;
    for (size_t ua = 0; max_trials != 0 && ua < num_pairs; ++ua) {
      const uint32_t pi = pairs[ua].first;
      const uint32_t pj = pairs[ua].second;
      const c8::HPoint2& ray_i = cur_rays[t.depth_matches[pi].wordsIdx];
      const c8::HPoint2& ray_j = cur_rays[t.depth_matches[pj].wordsIdx];
      const float dx = RayX(ray_i) - RayX(ray_j);
      const float dy = RayY(ray_i) - RayY(ray_j);
      c8::HMatrix sols[2] = {c8::HMatrixGen::i(), c8::HMatrixGen::i()};
      if (!(std::sqrt(dx * dx + dy * dy) < kMinPairRayDistance) &&
          c8::p2pImpl(pts3[pi], pts3[pj], ray_i, ray_j, r_rel, &sols[0], &sols[1])) {
        bool evaluated = false;
        for (const c8::HMatrix& m : sols) {
          const float tn = c8::HVector3(m(0, 3), m(1, 3), m(2, 3)).l2Norm();
          if (tn > s.max_motion_threshold) {
            continue;
          }
          if ((m * pts3[pi]).raw()[2] < 0.0f || (m * pts3[pj]).raw()[2] < 0.0f) {
            continue;
          }
          // compute-2d-inliers over all matches.
          const bool rotation_only = tn < s.max_translation_for_homography_depth_egomotion;
          const c8::HMatrix e =
              rotation_only ? c8::HMatrixGen::i() : c8::HMatrixGen::cross(m(0, 3), m(1, 3), m(2, 3)) * m;
          mask.clear();
          mask.reserve(t.matches.size());
          uint32_t count = 0;
          float max_res = 0.0f;
          for (const c8::PointMatch& pm : t.matches) {
            const auto& pr = t.prev_rays[pm.dictionaryIdx].raw();
            const c8::HPoint2& cr = cur_rays[pm.wordsIdx];
            float res;
            if (rotation_only) {
              const auto v = (m * c8::HVector3(pr[0], pr[1], pr[2])).raw();
              const float ex = v[0] / v[2] - RayX(cr);
              const float ey = v[1] / v[2] - RayY(cr);
              res = std::sqrt(ex * ex + ey * ey);
            } else {
              res = FLT_MAX;
              const auto l = (e * c8::HPoint3(pr[0], pr[1], pr[2])).raw();
              const float lx = l[0] / l[3];
              const float ly = l[1] / l[3];
              const float norm2 = lx * lx + ly * ly;
              if (!(static_cast<double>(norm2) < kMinEpipolarLineNorm)) {
                res = std::fabs(lx * RayX(cr) + RayY(cr) * ly + l[2] / l[3]) / std::sqrt(norm2);
              }
            }
            if (res < s.min2d_inliers_residual_for_depth_egomotion) {
              max_res = max_res < res ? res : max_res;
              ++count;
              mask.push_back(1);
            } else {
              mask.push_back(0);
            }
          }
          evaluated = true;
          if (count < (required > best2d ? required : best2d)) {
            continue;
          }
          // compute-3d-inliers: the inverse of the solution (Q9).
          const c8::HMatrix minv = m.inv();
          int32_t n3d = 0;
          for (size_t k = 0; k < t.depth_matches.size(); ++k) {
            const auto q = (minv * pts3[k]).raw();
            if (q[2] / q[3] < 0.0f) {
              continue;
            }
            const c8::HPoint2& cr = cur_rays[t.depth_matches[k].wordsIdx];
            const float ex = RayX(cr) - q[0] / q[2];
            const float ey = RayY(cr) - q[1] / q[2];
            if (ex * ex + ey * ey > s.min3d_inlier_sq_threshold) {
              continue;
            }
            if (++n3d >= s.min_depths_for_depth_egomotion) {
              break;
            }
          }
          if (n3d < s.min_depths_for_depth_egomotion) {
            continue;
          }
          if (!(count > best2d || (count == best2d && max_res < best_max_res))) {
            continue;
          }
          best = m.inv();
          best_mask.swap(mask);
          best2d = count;
          best_max_res = max_res;
          const uint32_t needed = NeededTrials(count, num_matches);
          max_trials = num_pairs < needed ? num_pairs : needed;
        }
        trials += evaluated ? 1 : 0;
      }
      if (trials >= max_trials) {
        break;
      }
    }

    if (ground_idx != nullptr) {
      ground_idx->clear();
    }
    if (best2d >= required) {
      if (ground_idx != nullptr) {
        for (size_t j = 0; j < best_mask.size(); ++j) {
          const c8::PointMatch& pm = t.matches[j];
          if (best_mask[j] && cur_classes[pm.wordsIdx] == 1 &&
              t.prev_classes[pm.dictionaryIdx] == 1) {
            ground_idx->push_back(static_cast<uint32_t>(pm.wordsIdx));
          }
        }
      }
      *success = true;
      t.prev_depths.clear();
      delta = best;
      motion = best;
      depth_done = true;
    }
  }

  // ---- ground-egomotion (1279) ----
  if (!depth_done) {
    std::vector<c8::HPoint2> prev_ground;
    std::vector<c8::HPoint2> cur_ground;
    prev_ground.reserve(t.ground_matches.size());
    cur_ground.reserve(t.ground_matches.size());
    for (const c8::PointMatch& pm : t.ground_matches) {
      prev_ground.push_back(t.prev_rays[pm.dictionaryIdx]);
      cur_ground.push_back(cur_rays[pm.wordsIdx]);
    }
    std::vector<uint8_t> ground_mask;
    c8::HMatrix h = c8::HMatrixGen::i();
    const bool ok = RobustGroundHomography(t.prev_q_a, prev_ground, q_cur, cur_ground, &t.scratch,
                                           &t.rng, &h, &motion, &ground_mask);
    *success = ok;
    if (ground_idx != nullptr) {
      ground_idx->clear();
      if (ok) {
        for (size_t k = 0; k < ground_mask.size(); ++k) {
          if (ground_mask[k]) {
            ground_idx->push_back(static_cast<uint32_t>(t.ground_matches[k].wordsIdx));
          }
        }
      }
    }
    t.prev_depths.clear();
    delta = t.call_count == 0 ? q_cur.toRotationMat() : c8::scaleTranslation(t.height, motion);
  }

  // ---- motion gate, fallback and height update ----
  const float translation = c8::HVector3(delta(0, 3), delta(1, 3), delta(2, 3)).l2Norm();
  if (translation > s.max_motion_threshold) {
    *success = false;
  }
  if (!*success) {
    delta = t.prev_q_b.inverse().times(q_cur).toRotationMat();
  } else {
    // h *= 1 + R(q_prev)[1,:] . t(motion), with the binary's evaluation order (func1266 wat).
    const c8::HMatrix r = t.prev_q_a.toRotationMat();
    const float r10 = r(1, 0);
    const float r11 = r(1, 1);
    const float r12 = r(1, 2);
    const auto& m = motion.data();
    const float g1 = m[15] * 0.0f + (r12 * m[14] + (r10 * m[12] + m[13] * r11));
    const float g2 = (m[11] * 0.0f + (r12 * m[10] + (r10 * m[8] + m[9] * r11))) * 0.0f;
    const float g3 = (m[3] * 0.0f + (r12 * m[2] + (r10 * m[0] + m[1] * r11))) * 0.0f;
    const float g4 = (m[7] * 0.0f + (r12 * m[6] + (r10 * m[4] + m[5] * r11))) * 0.0f;
    t.height = t.height * ((g1 + (g2 + (g3 + g4))) + 1.0f);
  }

  // ---- bookkeeping ----
  t.prev_frame.clearAndClone(cur);  // 990
  t.prev_q_a = q_cur;
  t.prev_q_b = q_cur;
  t.prev_rays = std::move(cur_rays);
  t.prev_classes = std::move(cur_classes);
  t.last_translation = translation;
  t.last_success = *success;
  const float w = c8::Quaternion::fromHMatrix(delta).w();
  t.last_rotation = std::acos(w) + std::acos(w);
  ++t.call_count;
  if (sky_idx != nullptr && s.include_sky_points) {
    sky_idx->clear();
    for (const c8::PointMatch& pm : t.sky_matches) {
      sky_idx->push_back(static_cast<uint32_t>(pm.wordsIdx));
    }
  }
  const std::vector<uint32_t> depth_points = DepthPointIndices(t.prev_rays.size());
  if (depth_idx != nullptr) {
    *depth_idx = depth_points;
  }
  *delta_out = delta;
}

}  // namespace slam_refact::frame_frame_tracker
