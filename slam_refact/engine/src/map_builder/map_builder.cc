// MapBuilder::Update (func 936) and its policy pieces (architecture.md §3.6; S3 §3, §3.1-3.4).
//
// Citations: 936.c:L = build/slam-refact-20261015/spec/map-builder/annotated/936.c (Ghidra C of
// func 936, same line numbers as the S3 spec), DCMP = re/wabt/xr-slam.dcmp.
// Out of scope and replaced by SR_CHECK (architecture.md §1.2): VPS (depth keyframe reseed
// 936.c:463-688, VPS correction 1132, the force-keyframe record 936.c:2786-2868), cloud maps and
// loop close without SLAM_REFACT_LOOP_CLOSE (US-025 compiles it in; loop_close.cc). VpsUpdate
// (1134) is not called: it only consumes VPS responses, which world SLAM never has (S3 U6,
// INFERRED no-op).

#include "map_builder/map_builder.h"

#include <cmath>
#include <cstdint>
#include <map>
#include <set>
#include <vector>

#include "c8/geometry/egomotion.h"
#include "c8/hmatrix.h"
#include "c8/hpoint.h"
#include "common/c8_adapters.h"
#include "common/check.h"
#include "common/stage_timer.h"
#include "config/accuracy.h"
#include "config/settings.h"
#include "map/world_map.h"
#include "map/world_map_internal.h"
#include "map_builder/loop_close.h"
#include "map_builder/map_builder_internal.h"

namespace slam_refact::map_builder {

namespace {

// wasm f32 -> i32 conversion as emitted for these casts: |v| < 2^31 truncates, else 0x80000000
// (936.c:1236-1241, 1255-1260).
int32_t TruncToI32(float v) {
  if (std::fabs(v) < 2147483648.0f) {
    return static_cast<int32_t>(v);
  }
  return INT32_MIN;
}

// Live keyframe count = raw slots minus free slots (936.c:3267). Free slots have id 0.
int LiveKeyframes(const map::WorldMap& wm) { return wm.NumKeyframes(); }

bool KeyframeIdValid(const map::WorldMap& wm, map::KeyframeId id) {
  const auto& slots = map::WorldMapAccess::impl(wm).keyframes;
  return id.id() != 0 && id.index() < slots.size() && slots[id.index()].id == id.id();
}

}  // namespace

// ---------------------------------------------------------------------------------------------
// Policy predicates

bool ShouldResetMap(const ResetInputs& in) {
  const config::MapBuilderSettings& s = config::MapBuilderSettings::Get();
  bool lost_enough = false;
  if (in.state == 0 && in.frame_tracker_ok &&
      static_cast<uint32_t>(s.keyframe_minimum_new_points) <= in.num_candidates) {
    lost_enough = true;
    // int32 difference of the two int64 counters, signed compare (936.c:711-714).
    if (static_cast<int32_t>(in.frame_count) - static_cast<int32_t>(in.last_state_change_frame) <=
        s.num_frames_to_reset_non_empty_map) {
      lost_enough = in.keyframe_slots == 0;
    }
  }
  bool small_map_ok = false;
  if (in.keyframe_slots <= static_cast<uint32_t>(s.map_good_key_frame_count)) {
    small_map_ok = static_cast<int64_t>(s.num_consecutive_frame_tracking_successes_to_reset_bad_map) <=
                   in.consecutive_frame_tracker_ok;
  }
  return lost_enough &&
         (small_map_ok ||
          static_cast<int64_t>(s.num_consecutive_frame_tracking_successes_to_reset_good_map) <=
              in.consecutive_frame_tracker_ok);
}

bool AnyGyroAbove(const std::vector<TrackingSensorEvent>& events, float max_deg_s) {
  for (const TrackingSensorEvent& e : events) {
    if (e.kind == ImuEvent::Kind::kGyroscope &&
        max_deg_s < std::sqrt(e.z * e.z + e.y * e.y + e.x * e.x)) {
      return true;
    }
  }
  return false;
}

int CountNotIn(const std::vector<uint32_t>& indices,
               const std::map<uint32_t, map::PointId>& matches) {
  int n = 0;
  for (uint32_t i : indices) {
    n += matches.count(i) == 0 ? 1 : 0;
  }
  return n;
}

bool RelocalizationHold(int64_t frame_count, int64_t last_reset_frame,
                        int64_t last_state_change_frame) {
  const int delay = config::MapBuilderSettings::Get().relocalization_delay;
  const int32_t since_change =
      static_cast<int32_t>(frame_count) - static_cast<int32_t>(last_state_change_frame);
  if (static_cast<int64_t>(delay) <= frame_count - last_reset_frame) {
    return since_change < delay;
  }
  return false;
}

bool NeedKeyframe(uint32_t tracked, int n_new) {
  const config::MapBuilderSettings& s = config::MapBuilderSettings::Get();
  // 936.c:884-893: unsigned compare against keyframeLowMapPointCount, then the new-point test.
  if (tracked < static_cast<uint32_t>(s.keyframe_low_map_point_count)) {
    return true;
  }
  if (n_new < s.keyframe_minimum_new_points) {
    return false;
  }
  return s.keyframe_count_variability * static_cast<float>(tracked) <= static_cast<float>(n_new);
}

bool DecideKeyframe(const KeyframeDecisionInputs& in) {
  const config::MapBuilderSettings& s = config::MapBuilderSettings::Get();
  const bool reloc_hold =
      RelocalizationHold(in.frame_count, in.last_reset_frame, in.last_state_change_frame);
  if (in.map_reset) {
    return true;  // 936.c:904: the reset frame always gets a keyframe
  }
  if (s.no_kf_in_extreme_gyro && in.imu != nullptr &&
      AnyGyroAbove(*in.imu, config::WorldMapsSettings::Get().max_gyro_magnitude_keyframe)) {
    return false;
  }
  if (in.state != 1) {
    return false;
  }
  const int32_t since_change =
      static_cast<int32_t>(in.frame_count) - static_cast<int32_t>(in.last_state_change_frame);
  if (since_change < s.keyframe_delay || reloc_hold) {
    return false;
  }
  return static_cast<int64_t>(s.keyframe_cooldown) <= in.frame_count - in.last_keyframe_frame &&
         in.need;
}

// ---------------------------------------------------------------------------------------------
// Ground plane (936.c:1998-2170)

GroundPlane GroundPlaneInCamera(const c8::HMatrix& world_from_camera) {
  // Points (0,-1,0), (1,-1,0), (0,-1,1) (DCMP:105272-105277) into the camera frame.
  const c8::HMatrix camera_from_world = world_from_camera.inv();
  const c8::HPoint3 p0 = camera_from_world * c8::HPoint3(0.0f, -1.0f, 0.0f);
  const c8::HPoint3 p1 = camera_from_world * c8::HPoint3(1.0f, -1.0f, 0.0f);
  const c8::HPoint3 p2 = camera_from_world * c8::HPoint3(0.0f, -1.0f, 1.0f);
  const float ax = p1.x() - p0.x();
  const float ay = p1.y() - p0.y();
  const float az = p1.z() - p0.z();
  const float bx = p2.x() - p0.x();
  const float by = p2.y() - p0.y();
  const float bz = p2.z() - p0.z();
  GroundPlane g;
  g.nx = ay * bz - az * by;
  g.ny = az * bx - ax * bz;
  g.nz = ax * by - ay * bx;
  g.p0x = p0.x();
  g.p0y = p0.y();
  g.p0z = p0.z();
  return g;
}

c8::HPoint3 GroundPointInWorld(const GroundPlane& g, const c8::HMatrix& world_from_camera,
                               float ray_x, float ray_y) {
  // 936.c:2137-2146 with the f936 wat association (Ghidra prints the nested sums flat):
  //   den = n.z*rz + (n.x*rx + n.y*ry); s = (n.z*p0z + (n.x*p0x + p0y*n.y)) / den.
  const float rz = 1.0f;
  float den = g.nz * rz + (g.nx * ray_x + g.ny * ray_y);
  if (den == 0.0f) {
    den = 1.0f;
  }
  const float s = (g.nz * g.p0z + (g.nx * g.p0x + g.p0y * g.ny)) / den;
  return world_from_camera * c8::HPoint3(ray_x * s, ray_y * s, rz * s);
}

std::vector<uint32_t> PruneGroundCandidates(const map::WorldMap& wm, map::KeyframeId kf_id,
                                            const std::vector<uint32_t>& candidates) {
  StageTimer timer("remove-ground-points-behind-triangulated");
  const map::Keyframe& kf = wm.GetKeyframe(kf_id);
  const config::MapBuilderSettings& s = config::MapBuilderSettings::Get();
  const float eps = s.pruning_ground_points_epsilon;

  // Visible map points in the new keyframe (632 -> 1548, every map point).
  const map::ProjectedPoints vis =
      map::ProjectPoints(wm, {}, ToPinholeIntrinsics(kf.frame.intrinsic()), kf.frame.extrinsic());

  // Grid over the keyframe bounds (pixel corners -0.5 .. w-0.5), cell = eps (936.c:1205-1290).
  c8::HPoint2 lo;
  c8::HPoint2 hi;
  kf.frame.frameBounds(&lo, &hi);
  const int32_t nx = TruncToI32((hi.x() + 0.0f - lo.x()) / eps);
  const int32_t ny = TruncToI32((hi.y() + 0.0f - lo.y()) / eps);

  // Triangulated points with mean observation certainty >= threshold (936.c:1318-1400).
  std::vector<c8::HPoint2> tri_rays;
  for (const c8::FramePoint& fp : vis.frame.points()) {
    const map::WorldPoint& p = wm.Point(fp.id());
    if (p.state == map::PointState::kDead || p.source != map::PointSource::kTriangulated) {
      continue;
    }
    double sum = 0.0;
    for (const auto& [obs_kf, obs_idx] : p.observations) {
      sum += wm.GetKeyframe(obs_kf).frame.points()[obs_idx].depthWithCertainty().certainty;
    }
    if (s.pruning_ground_points_triangulated_certainty_threshold <=
        sum / static_cast<double>(p.observations.size())) {
      tri_rays.emplace_back(fp.x(), fp.y());
    }
  }
  if (tri_rays.empty()) {
    return candidates;
  }
  SR_CHECK(nx > 0 && ny > 0, "[map_builder] ground pruning grid is empty");

  c8::Vector<c8::HPoint2> cand_rays;
  cand_rays.reserve(candidates.size());
  for (uint32_t c : candidates) {
    SR_CHECK(c < kf.frame.points().size(), "[map_builder] ground candidate out of range");
    const c8::FramePoint& fp = kf.frame.points()[c];
    cand_rays.emplace_back(fp.x(), fp.y());
  }
  c8::BinMap grid(nx, ny);  // 1133
  grid.reset<c8::HPoint2>(lo.x(), lo.y(), hi.x() + 0.0f, hi.y() + 0.0f, cand_rays);

  std::set<uint32_t> keep(candidates.begin(), candidates.end());
  const float eps2 = eps * eps;
  const size_t ux = static_cast<uint32_t>(nx);
  const size_t uy = static_cast<uint32_t>(ny);
  for (const c8::HPoint2& t : tri_rays) {
    const size_t cx = grid.bin_.xBin(t.x());
    const size_t cy = grid.bin_.yBin(t.y());
    for (int dx = -1; dx <= 1; ++dx) {
      const size_t col = cx + static_cast<size_t>(static_cast<int64_t>(dx));
      if (!(col < ux)) {
        continue;
      }
      for (int dy = -1; dy <= 1; ++dy) {
        const size_t row = cy + static_cast<size_t>(static_cast<int64_t>(dy));
        if (!(row < uy)) {
          continue;
        }
        for (size_t ci : grid.map_[row * ux + col]) {
          const c8::HPoint2& c = cand_rays[ci];
          const float ex = c.x() - t.x();
          const float ey = c.y() - t.y();
          if (ex * ex + ey * ey <= eps2) {
            keep.erase(candidates[ci]);
          }
        }
      }
    }
  }
  return std::vector<uint32_t>(keep.begin(), keep.end());
}

// ---------------------------------------------------------------------------------------------
// Pair queue and keyframe pruning

void TrimPairQueue(std::vector<KfPair>* queue, int max_pair_size) {
  if (0.9 * static_cast<double>(max_pair_size) < static_cast<double>(queue->size())) {
    if (4 < max_pair_size + 2) {
      queue->erase(queue->begin(), queue->begin() + max_pair_size / 3);
    }
    if (max_pair_size > 0) {
      queue->reserve(static_cast<size_t>(max_pair_size));
    }
  }
}

bool IsKeyframeRedundant(int total, int redundant, float percentage) {
  return percentage * static_cast<float>(total) < static_cast<float>(redundant);
}

void PruneKeyframes(map::WorldMap& wm, const std::set<map::KeyframeId>& protected_kfs) {
  StageTimer timer("prune-keyframes");
  const config::MapBuilderSettings& s = config::MapBuilderSettings::Get();
  const std::vector<map::KeyframeId> all = wm.AllKeyframesByCreation();  // 1142
  // max(n, 1) - 1 oldest keyframes, newest excluded (936.c:3027-3060).
  const size_t count = (all.size() < 2 ? 1 : all.size()) - 1;
  std::vector<map::KeyframeId> cand;
  for (size_t i = 0; i < count; ++i) {
    if (protected_kfs.count(all[i]) == 0) {
      cand.push_back(all[i]);
    }
  }
  // Pass 1: redundancy.
  for (map::KeyframeId h : cand) {
    const map::Keyframe& kf = wm.GetKeyframe(h);
    int total = 0;
    int redundant = 0;
    for (size_t idx : kf.frame.worldPointIndices()) {
      const c8::FramePoint& o = kf.frame.points()[idx];
      SR_CHECK(o.id().id() != 0, "Invalid point id");
      const map::WorldPoint& p = wm.Point(o.id());
      uint32_t q = 0;
      for (const auto& [k, scale] : p.scale_by_kf) {
        q += scale <= o.scale() ? 1u : 0u;
      }
      ++total;
      redundant += static_cast<float>(s.redundancy_point_count) <= static_cast<float>(q) ? 1 : 0;
    }
    if (IsKeyframeRedundant(total, redundant, s.redundancy_point_percentage) &&
        wm.GetKeyframe(h).prune_protect == 0) {
      wm.RemoveKeyframe(h);  // 1144
    }
  }
  // Pass 2: size cap, oldest first.
  const int max_keyframes = config::WorldMapsSettings::Get().max_keyframes_in_map;
  for (map::KeyframeId h : cand) {
    if (LiveKeyframes(wm) <= max_keyframes) {
      break;
    }
    if (KeyframeIdValid(wm, h) && wm.GetKeyframe(h).prune_protect == 0) {
      wm.RemoveKeyframe(h);
    }
  }
}

// ---------------------------------------------------------------------------------------------
// MapBuilder

MapBuilder::MapBuilder() {
  // 641:141-158: pair queue capacity maxPairSize.
  const int max_pair_size = config::MapBuilderSettings::Get().max_pair_size;
  if (max_pair_size > 0) {
    pair_queue_.reserve(static_cast<size_t>(max_pair_size));
  }
}

MapBuilder::~MapBuilder() = default;

// 653 (called with clear flag 1 from 936.c:730).
void MapBuilder::ResetMap(map::WorldMaps* ctx, MapBuilderOutput* out) {
  ctx->ResetMap();
  pair_queue_.clear();
  seeded_from_depth_kf_ = false;
  vps_baseline_mode_ = false;
  out->map_reset = true;
  last_reset_frame_ = frame_count_;
  pending_vps_.clear();
  vps_corrected_kfs_.clear();
}

void MapBuilder::Update(map::WorldMaps* ctx, const TrackerInput& frm, const MapTrackerOutput& trk,
                        int64_t frame_ms, MapBuilderOutput* out) {
  StageTimer timer("map-keyframe");
  // frame_ms keys the VPS record map only (936.c:2813-2827), which world SLAM never writes.
  (void)frame_ms;
  const config::MapBuilderSettings& s = config::MapBuilderSettings::Get();

  // out->Clear (936.c:398-446).
  out->map_reset = false;
  out->keyframe_created = false;
  out->new_points.clear();
  out->reloc_kf_request = false;
  out->world_transform_updated = false;
  out->world_correction = c8::HMatrixGen::i();
  out->pose_correction_applied = false;
  out->pose_correction = c8::HMatrixGen::i();

  // 936.c:449-456.
  prev_state_ = state_;
  ++frame_count_;
  state_ = trk.state;
  if (state_ != prev_state_) {
    last_state_change_frame_ = frame_count_;
  }
  // 936.c:457-461 (cloud map or VPS-localized frame) cannot occur: CloudEmpty() and trk +0x1c4
  // is a VPS flag not modeled in MapTrackerOutput.
  SR_CHECK(!trk.depth_kf_reloc_latch,
           "[map_builder] depth keyframe reseed is VPS/depth only (architecture.md §1.2)");
  if (!s.reloc_to_kf_with_depth_points && !s.use_two_vps_localizations_for_map_scaling &&
      seeded_from_depth_kf_ && trk.state == 1) {
    vps_baseline_mode_ = true;  // 936.c:689-695
  }

  // [A] map reset (936.c:696-731).
  if (ctx->active().CloudEmpty() && (!seeded_from_depth_kf_ || vps_baseline_mode_) &&
      pending_vps_.empty() && reset_allowed_) {
    ResetInputs in;
    in.state = trk.state;
    in.frame_tracker_ok = trk.frame_tracker_ok;
    in.num_candidates = trk.ground_idx.size();
    in.frame_count = frame_count_;
    in.last_state_change_frame = last_state_change_frame_;
    in.keyframe_slots = map::KeyframeSlotCount(ctx->active());
    in.consecutive_frame_tracker_ok = trk.consecutive_frame_tracker_ok;
    if (ShouldResetMap(in)) {
      ResetMap(ctx, out);
    }
  }
  map::WorldMap& wm = ctx->active();

  // [B] tracking statistics (936.c:732-768).
  std::map<uint32_t, map::PointId> matches;
  if (!out->map_reset) {
    if (trk.frame_tracker_ok) {
      SR_CHECK(trk.visible_matched.size() >= trk.visible_ids.size(),
               "[map_builder] visible_matched shorter than visible_ids");
      for (size_t i = 0; i < trk.visible_ids.size(); ++i) {
        map::WorldPoint& p = wm.MutablePoint(trk.visible_ids[i]);  // 1125
        ++p.tracking_count;
        if (trk.visible_matched[i] != 0) {
          ++p.tracking_hits;
        }
      }
    }
    matches = trk.tracked;  // 937
  }

  // [C] keyframe decision (936.c:770-958).
  bool create = force_keyframe_;
  if (!create) {
    const uint32_t tracked = static_cast<uint32_t>(matches.size());
    int n_new;
    if (!seeded_from_depth_kf_) {
      n_new = CountNotIn(trk.ground_idx, matches) +
              (s.consider_sky_pts_in_keyframe_creation_check ? CountNotIn(trk.sky_idx, matches)
                                                             : 0);
    } else {
      n_new = CountNotIn(trk.depth_idx, matches);
    }
    KeyframeDecisionInputs in;
    in.map_reset = out->map_reset;
    in.state = trk.state;
    in.imu = &trk.predicted.sensor_events;
    in.frame_count = frame_count_;
    in.last_state_change_frame = last_state_change_frame_;
    in.last_reset_frame = last_reset_frame_;
    in.last_keyframe_frame = wm.last_keyframe_frame();
    in.need = NeedKeyframe(tracked, n_new);
    create = DecideKeyframe(in);
  }
  if (!create) {
    // NO_KF (936.c:914-933).
    EpipolarMatcher(wm, &pair_queue_, vps_baseline_mode_);  // 1112
    wm.CullAndTenureAll();                                  // 1141
    wm.AdjustDirtyPoints();                                 // 1127
    return;
  }

  // CREATE (936.c:955-1135). world_matches is frm+0x98 = worldPointIndices().size() (D-7/D-22:
  // the original per-frame path never fills that set, so the value is expected to be 0).
  const int world_matches = static_cast<int>(frm.frame_points.worldPointIndices().size());
#ifdef SLAM_REFACT_LOOP_CLOSE
  if (s.loop_close_enabled && !loop_close_) {
    loop_close_ = std::make_unique<LoopCloseState>();
  }
#endif
  const map::KeyframeId nk = wm.AddKeyframe(frm.frame_points, frame_count_);  // 990 + 654
#ifdef SLAM_REFACT_LOOP_CLOSE
  if (s.loop_close_enabled) {
    // 654 calls func 1263 on the stored ORB descriptors here (spec/12-loop-close.md §2.5). It is
    // done at this call site because sr_map cannot depend on sr_map_builder (US025-CC-1).
    ComputeKeyframeGlobalDescriptor(loop_close_.get(), &wm.MutableKeyframe(nk));
  }
#endif
  wm.SetKeyframePose(nk, trk.pose);                                           // 655
  for (const auto& [obs_idx, ph] : matches) {
    wm.AddObservation(ph, obs_idx, nk);  // 658
  }
  wm.AdjustDirtyPoints();
  const map::Keyframe& kf = wm.GetKeyframe(nk);  // slots never reallocate (642 reserve 100)
  const c8::HMatrix pose = kf.frame.extrinsic();

  // [D] ground points (936.c:1135-2362).
  if (!seeded_from_depth_kf_ && ground_points_enabled_ &&
      (wm.CloudEmpty() || s.enable_ground_points_for_cloud_map)) {
    std::vector<uint32_t> cand = trk.ground_idx;
    if (trk.state == 1 && s.is_pruning_ground_points_in_map_tracking) {
      ground_prune_pose_ = pose;  // 936.c:1173
      cand = PruneGroundCandidates(wm, nk, cand);
    }
    // inv() aborts on a non-invertible pose like the flag checks of 936.c:2007-2018.
    const GroundPlane plane = GroundPlaneInCamera(pose);
    std::vector<c8::HPoint3> world;
    world.reserve(cand.size());
    for (uint32_t c : cand) {
      SR_CHECK(c < kf.frame.points().size(), "[map_builder] ground candidate out of range");
      const c8::FramePoint& fp = kf.frame.points()[c];
      world.push_back(GroundPointInWorld(plane, pose, fp.x(), fp.y()));
    }
    for (size_t i = 0; i < cand.size(); ++i) {
      if (kf.frame.points()[cand[i]].id() == map::PointId::nullId()) {
        const map::PointId ph = wm.AddPoint(kf.seq, world[i], map::PointSource::kGround);  // 657
        wm.AddObservation(ph, cand[i], nk);
        out->new_points.push_back(ph);
      }
    }
  }

  // [E] sky points (936.c:2364-2640).
  if (!seeded_from_depth_kf_ && wm.CloudEmpty() &&
      (!s.require_minimum_for_skypoints || world_matches < s.min_world_point_matches_for_skypoints)) {
    const float dist = s.skypoints_distance;
    c8::Vector<c8::HPoint3> cam;
    cam.reserve(trk.sky_idx.size());
    for (uint32_t j : trk.sky_idx) {
      SR_CHECK(j < kf.frame.points().size(), "[map_builder] sky index out of range");
      const c8::FramePoint& fp = kf.frame.points()[j];
      const float y = fp.y() / 1.0f;
      const float x = fp.x() / 1.0f;
      const float n = std::sqrt(y * y + x * x + 0.0f + 1.0f);
      cam.emplace_back((x / n) * dist, (y / n) * dist, (1.0f / n) * dist);
    }
    const c8::Vector<c8::HPoint3> world = pose * cam;  // 564
    const int limit = std::min(static_cast<int>(trk.sky_idx.size()), s.max_new_skypoints);
    for (int k = 0; k < limit; ++k) {
      const uint32_t j = trk.sky_idx[static_cast<size_t>(k)];
      if (kf.frame.points()[j].id() == map::PointId::nullId()) {
        const map::PointId ph = wm.AddPoint(kf.seq, world[static_cast<size_t>(k)],
                                            map::PointSource::kSky);
        wm.AddObservation(ph, j, nk);
        out->new_points.push_back(ph);
      }
    }
  }
  SR_CHECK(!(vps_baseline_mode_ && wm.CloudEmpty() && !trk.depth_idx.empty() && !force_keyframe_),
           "[map_builder] VPS correction (1132) is not supported (architecture.md §1.2)");
  if (!out->new_points.empty()) {
    wm.AdjustDirtyPoints();  // 936.c:2665
  }
  out->keyframe_created = true;  // 936.c:2677

  // [F] pairing (936.c:2678-2785).
  TrimPairQueue(&pair_queue_, s.max_pair_size);
  const map::KeyframeId newest = wm.newest();
  const std::vector<map::KeyframeId> partners =
      s.pair_all_keyframes
          ? wm.AllKeyframesByCreation()                                                    // 1142
          : wm.CovisibleKeyframes(newest,
                                  config::WorldMapSettings::Get().required_points_for_covisibility);
  for (map::KeyframeId c : partners) {
    if (c.id() != newest.id()) {
      pair_queue_.push_back({c, newest});
    }
  }
  EpipolarMatcher(wm, &pair_queue_, vps_baseline_mode_);  // 1112
  wm.CullAndTenureAll();                                  // 1141
  SR_CHECK(!force_keyframe_, "[map_builder] VPS keyframe records are not supported");

  // [G] prune-keyframes (936.c:2869-3286).
  if (!config::WorldMapsSettings::Get().prevent_pruning) {
    std::set<map::KeyframeId> protected_kfs;
    for (const auto& [ms, kf_id] : pending_vps_) {
      protected_kfs.insert(kf_id);
    }
    protected_kfs.insert(vps_corrected_kfs_.begin(), vps_corrected_kfs_.end());
    PruneKeyframes(wm, protected_kfs);
  }
  // Accuracy mode (ACC-1): joint BA of the newest keyframes; not part of the original.
  if (config::Accuracy().enabled && config::Accuracy().local_ba) {
    const LocalBaResult ba = LocalBundleAdjustment(wm, nk);
    if (ba.applied) {
      out->pose_correction_applied = true;
      out->pose_correction = ba.newest_correction;
    }
  }
  // [H] global iterative BA, default off (936.c:3291-4157).
  if (s.bundle_adjustment_iterative_enabled) {
    IterativeBundleAdjustment(wm);
  }
  // [I] loop close, default off (936.c:4159-5754; spec/12-loop-close.md, US-025). With the option
  // off nothing below the condition is executed or read.
  if (s.loop_close_enabled) {
#ifdef SLAM_REFACT_LOOP_CLOSE
    LoopClose(wm, loop_close_.get(), out);
#else
    SR_CHECK(false, "[map_builder] loop close is unsupported in this build (SLAM_REFACT_LOOP_CLOSE=OFF)");
#endif
  }
  wm.AdjustDirtyPoints();  // 936.c:5758
}

}  // namespace slam_refact::map_builder
