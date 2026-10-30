// MapTracker::track (926), the MapBuilder event application (664) and the ground-height window
// (§5.6). Read from Ghidra G926 / G664 / G1273 / G1260 and the f926 disassembly; S2 §3, §5.6, §6.

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <map>
#include <set>
#include <utility>
#include <vector>

#include "c8/geometry/egomotion.h"
#include "c8/hmatrix.h"
#include "c8/hpoint.h"
#include "c8/quaternion.h"
#include "common/check.h"
#include "config/settings.h"
#include "frame_frame_tracker/frame_frame_tracker_internal.h"
#include "map_tracker/map_tracker.h"
#include "map_tracker/map_tracker_internal.h"

namespace slam_refact::map_tracker {

namespace {

using frame_frame_tracker::FrameFrameTrackerAccess;

// 1260 applyTrackResult: move the match vectors into the tracker, pose_ = result pose.
void ApplyTrackResult(MapTrackerAccess::Impl* t, MapTrackResult* r) {
  t->last_tracked_ids = std::move(r->matched_ids);
  r->matched_ids.clear();
  t->all_obs = std::move(r->all_obs);
  r->all_obs.clear();
  t->all_matches = std::move(r->all_matches);
  r->all_matches.clear();
  t->subset_obs = std::move(r->subset_obs);
  r->subset_obs.clear();
  t->subset_matches = std::move(r->subset_matches);
  r->subset_matches.clear();
  t->last_world_points = std::move(r->subset_world_points);
  r->subset_world_points.clear();
  t->pose = r->pose;
}

}  // namespace

// f926 (section D): the operands and their association follow the wasm stack order.
MotionGate ComputeMotionGate(const c8::HMatrix& frame_delta, const c8::Quaternion& prev_rotation,
                             const c8::Quaternion& rotation) {
  MotionGate g;
  const float tx = frame_delta(0, 3);
  const float ty = frame_delta(1, 3);
  const float tz = frame_delta(2, 3);
  g.translation = std::sqrt(tz * tz + (ty * ty + (tx * tx + 0.0f)));

  // Conjugate of the normalized previous device rotation (n > 0 ? 1 / n : 1).
  const float pw0 = prev_rotation.w();
  const float px0 = prev_rotation.x();
  const float py0 = prev_rotation.y();
  const float pz0 = prev_rotation.z();
  const float n = std::sqrt(pz0 * pz0 + (py0 * py0 + (pw0 * pw0 + px0 * px0)));
  const float s = n > 0.0f ? 1.0f / n : 1.0f;
  const float pw = pw0 * s;
  const float nz = s * -pz0;
  const float ny = s * -py0;
  const float nx = s * -px0;
  const float rw = rotation.w();
  const float rx = rotation.x();
  const float ry = rotation.y();
  const float rz = rotation.z();
  // d = conj * rotation (Hamilton product), then normalized the same way.
  const float dz = rz * pw + ((rw * nz + rx * ny) - ry * nx);
  const float dy = rz * nx + (ry * pw + (rw * ny - nz * rx));
  const float dw = ((rw * pw - nx * rx) - ry * ny) - rz * nz;
  const float dx = (ry * nz + (rw * nx + rx * pw)) - rz * ny;
  const float m = std::sqrt(dz * dz + (dy * dy + (dw * dw + dx * dx)));
  const float u = m > 0.0f ? 1.0f / m : 1.0f;
  const float a = dz * u;
  const float b = dx * u;
  const float c = dy * u;
  g.device_rotation2 = a * a + (b * b + c * c);

  const c8::Quaternion v = c8::Quaternion::fromHMatrix(frame_delta);  // 520
  g.visual_rotation2 = v.z() * v.z() + (v.x() * v.x() + v.y() * v.y());
  return g;
}

namespace {

void InsertTracked(MapTrackerAccess::Impl* t, const std::vector<TrackedMatch>& matches) {
  for (const TrackedMatch& m : matches) {
    t->tracked.Insert(m.frame_index, m.id);  // 1273
  }
}

// |v| of a sensor event as the binary evaluates it: sqrt(z^2 + (x^2 + y^2)).
float EventMagnitude(const TrackingSensorEvent& e) {
  return std::sqrt(e.z * e.z + (e.x * e.x + e.y * e.y));
}

bool EventExceedsLimits(const TrackingSensorEvent& e, const config::MapTrackerSettings& s) {
  if (e.kind == ImuEvent::Kind::kGyroscope) {
    return EventMagnitude(e) > s.max_gyro_magnitude;
  }
  if (e.kind == ImuEvent::Kind::kLinearAcceleration) {
    return EventMagnitude(e) > s.max_linear_acceleration_magnitude;
  }
  return false;
}

}  // namespace

bool UpdateGroundHeight(std::vector<float>* heights, size_t ground_candidates, float pose_ty,
                        float* ground_height) {
  const config::MapTrackerSettings& s = config::MapTrackerSettings::Get();
  // k = 0 would read h[i - 1] at i = 0 (the original has the same out-of-range read); a config
  // error, not a tracking state (S2 review LOW-2).
  SR_CHECK(s.min_ground_points_for_scale >= 1, "MapTracker.minGroundPointsForScale must be >= 1");
  const size_t k = static_cast<size_t>(s.min_ground_points_for_scale);
  if (ground_candidates < k || heights->size() < k) {
    return false;
  }
  std::stable_sort(heights->begin(), heights->end());  // 1257
  const std::vector<float>& h = *heights;
  float best = FLT_MAX;
  size_t best_index = 0;
  for (size_t i = 0; i <= h.size() - k; ++i) {
    const float spread = h[i + k - 1] - h[i];
    if (spread < best) {
      best = spread;
      best_index = i;
    }
  }
  const float median = h[static_cast<size_t>(s.min_ground_points_for_scale / 2) + best_index];
  const float alpha = s.low_pass_ground_height_alpha;
  const float v = *ground_height * (1.0f - alpha) + alpha * (pose_ty - median);
  // Out of [-100, 100] resets to 1.0 (not a clamp, F926:2495-2503).
  *ground_height = v <= 100.0f ? (v >= -100.0f ? v : 1.0f) : 1.0f;
  return true;
}

MapTracker::MapTracker() : impl_(std::make_unique<Impl>()) {}

MapTracker::~MapTracker() = default;

const c8::HMatrix& MapTracker::pose() const { return impl_->pose; }

frame_frame_tracker::FrameFrameTracker& MapTracker::frame_tracker() {
  return impl_->frame_tracker;
}

// 664 (S2 §6.5).
c8::HMatrix MapTracker::ApplyMapBuilderOutput(const MapBuilderOutput& o) {
  Impl& t = *impl_;
  if (o.map_reset) {
    t.frame_tracker.ResetPartial();
  }
  if (o.keyframe_created) {
    t.map_initialized_pending = true;
  }
  if (o.reloc_kf_request) {
    t.reloc_kf_id = o.reloc_kf_id;
    t.reloc_kf_pending = true;
  }
  if (o.pose_correction_applied) {  // accuracy mode (ACC-1): keep tracked ids and frame tracker
    t.pose = o.pose_correction * t.pose;
    t.frame_start_pose = t.pose;
  }
  if (o.world_transform_updated) {
    // 664 (G664:108-268): pose_ = pose_ * C, right multiplication (operand roles: static reading,
    // the original never ran this path, spec/12-loop-close.md §5.2 / N2). Vendored product.
    t.pose = t.pose * o.world_correction;
    t.frame_start_pose = t.pose;
    t.last_tracked_ids.clear();
    t.frame_tracker.ResetPartial();
  } else if (!o.new_points.empty()) {
    t.last_tracked_ids.insert(t.last_tracked_ids.end(), o.new_points.begin(),
                              o.new_points.end());
  }
  return t.pose;
}

void MapTracker::Track(map::WorldMap& map, TrackerInput* frame, const FrameState& predicted,
                       MapTrackerOutput* out, compat::RandomNumbers* rng) {
  Impl& t = *impl_;
  const config::MapTrackerSettings& s = config::MapTrackerSettings::Get();
  const config::FrameFrameTrackerSettings& fs = config::FrameFrameTrackerSettings::Get();
  const c8::FrameWithPoints& fp = frame->frame_points;
  const c8::Quaternion& rotation = frame->pose;  // frame +0x3a4

  t.intrinsic = fp.intrinsic();
  t.frame_start_pose = t.pose;
  ++t.frame_count;
  // out reset (F926:317-332); pose and predicted keep their previous values.
  out->consecutive_frame_tracker_ok = 0;
  out->frame_tracker_ok = false;
  out->state = 0;
  out->ground_idx.clear();
  out->sky_idx.clear();
  out->depth_idx.clear();
  out->visible_ids.clear();
  out->visible_matched.clear();
  out->tracked.clear();
  out->lost_reloc_attempted = false;
  out->depth_kf_reloc_latch = false;
  out->depth_kf_reloc_id = map::KeyframeId();
  out->depth_kf_reloc_success = false;

  // ---- A. depth-keyframe relocalization: not part of world SLAM (architecture.md §1.2 M2).
  SR_CHECK(!t.reloc_kf_pending, "[map_tracker] depth-keyframe relocalization is not supported");

  // ---- B. extreme IMU motion flag (F926:986-1019).
  if (!predicted.sensor_events.empty()) {
    bool extreme = false;
    for (const TrackingSensorEvent& e : predicted.sensor_events) {
      if (EventExceedsLimits(e, s)) {
        extreme = true;
        break;
      }
    }
    t.extreme_motion = extreme;
    out->predicted = predicted;
  }

  // ---- C. frame-to-frame motion: inertial or visual (F926:1020-1260).
  bool inertial = s.inertial_tracking && t.state == 1 &&
                  (t.frame_count - t.last_state_change_frame) >
                      static_cast<int64_t>(s.initial_frame_delay);
  if (inertial && s.use_fft_for_extreme_movement) {
    for (const TrackingSensorEvent& e : predicted.sensor_events) {
      if (EventExceedsLimits(e, s)) {
        inertial = false;
        break;
      }
    }
  }
  if (inertial && t.extreme_motion) {
    inertial = false;
  }
  if (!inertial) {
    bool ok = false;
    t.frame_tracker.Track(&t.frame_delta, fp, rotation, &t.ground_idx, &t.sky_idx, &t.depth_idx,
                          &ok);
    t.frame_tracker_ok = ok;
  } else {
    t.frame_tracker_ok = true;
    t.frame_delta = c8::egomotion(t.pose, predicted.pose);  // 660
    std::vector<c8::HPoint2> rays;
    rays.reserve(fp.points().size());
    for (const c8::FramePoint& p : fp.points()) {
      rays.emplace_back(p.x(), p.y());
    }
    const std::vector<int32_t> classes =
        frame_frame_tracker::ClassifyHorizonRays(predicted.xr_device_rotation, rays);
    const size_t n = fp.points().size();
    const int ground_fraction = fs.inertial_tracking_ground_points_fraction;
    // i % 0 traps in the original (wasm i32.rem_s) and is UB here (S2 review LOW-2).
    SR_CHECK(ground_fraction >= 1,
             "FrameFrameTracker.inertialTrackingGroundPointsFraction must be >= 1");
    t.ground_idx.clear();
    for (size_t i = 0; i < n; ++i) {
      if (static_cast<int32_t>(i) % ground_fraction == 0 && classes[i] == 1) {
        t.ground_idx.push_back(static_cast<uint32_t>(i));
      }
    }
    const int sky_fraction = fs.inertial_tracking_sky_points_fraction;
    t.sky_idx.clear();
    if (fs.include_sky_points) {
      SR_CHECK(sky_fraction >= 1, "FrameFrameTracker.inertialTrackingSkyPointsFraction must be >= 1");
      for (size_t i = 0; i < n; ++i) {
        if (static_cast<int32_t>(i) % sky_fraction == 0 && classes[i] == 5) {
          t.sky_idx.push_back(static_cast<uint32_t>(i));
        }
      }
    }
    t.depth_idx = frame_frame_tracker::DepthPointIndices(n);  // 1269

    // FrameFrameTracker bookkeeping (F926:1227-1251).
    FrameFrameTrackerAccess::Impl& ft = FrameFrameTrackerAccess::impl(t.frame_tracker);
    ft.matches.clear();
    ft.ground_matches.clear();
    ft.sky_matches.clear();
    ft.prev_frame.clearAndClone(fp);  // 990
    ft.prev_q_a = predicted.xr_device_rotation;
    ft.prev_q_b = predicted.xr_device_rotation;
    ft.prev_rays = rays;
    ft.prev_classes = classes;
    ft.last_success = true;
    ft.last_translation =
        c8::HVector3(t.frame_delta(0, 3), t.frame_delta(1, 3), t.frame_delta(2, 3)).l2Norm();
    const float w = c8::Quaternion::fromHMatrix(t.frame_delta).w();
    ft.last_rotation = std::acos(w) + std::acos(w);
    ++ft.call_count;
  }
  out->ground_idx = t.ground_idx;
  out->sky_idx = t.sky_idx;
  out->depth_idx = t.depth_idx;

  // ---- D. compose and correct the tilt (F926:1270-1491).
  const c8::HMatrix composed = t.pose * t.frame_delta;
  bool settling = true;
  if (t.state == 1) {
    settling = (t.frame_count - t.last_state_change_frame) <=
               static_cast<int64_t>(s.initial_frame_delay);
  }
  // Small-motion rule as the f926 wasm evaluates it: the composed pose is taken when the frame
  // moved (|t| > 0.001), while settling, or when the visual rotation (|v|^2 > 1e-6) or the device
  // rotation (|d|^2 > 0.001) is large; otherwise only the device yaw is re-applied when
  // |d|^2 > 2e-5, and the pose is kept unchanged below that.
  const MotionGate gate = ComputeMotionGate(t.frame_delta, t.prev_device_rotation, rotation);
  if (static_cast<double>(gate.translation) > 0.001 || settling ||
      static_cast<double>(gate.visual_rotation2) > 1e-6 ||
      static_cast<double>(gate.device_rotation2) > 0.001) {
    t.pose = ReplaceYaw(composed, rotation);
  } else if (static_cast<double>(gate.device_rotation2) > 2e-5) {
    t.pose = ReplaceYaw(t.pose, rotation);
  }
  out->frame_tracker_ok = t.frame_tracker_ok;
  t.consecutive_frame_tracker_ok = t.frame_tracker_ok ? t.consecutive_frame_tracker_ok + 1 : 0;
  out->pose = t.pose;

  // ---- E. map-initialized event (F926:1501-1571).
  if (t.map_initialized_pending && t.state == 0) {
    if (static_cast<double>(t.frame_start_pose(1, 3)) < -0.999) {
      t.ground_y = -1.0f;
      t.pose = c8::cameraMotion(c8::HMatrixGen::translation(t.pose(0, 3), 0.0f, t.pose(2, 3)),
                                c8::rotationMat(t.pose));
      t.frame_tracker.set_height(1.0f);
      t.ground_height = 1.0f;
      out->pose = t.pose;
    }
    t.state = 1;
    t.last_state_change_frame = t.frame_count;
  }
  t.map_initialized_pending = false;
  t.tracked.Clear();

  if (t.state == 0) {
    // ---- F. state-lost (F926:1584-1759).
    t.anchor_pose = ReplaceYaw(t.anchor_pose, rotation);
    const c8::HMatrix backup = t.pose;
    t.last_tracked_ids.clear();
    RelocResult r;
    // The external-extrinsic branch needs external_extrinsic_valid_, which world SLAM never sets;
    // the cloud is always empty, so relocalization runs on every lost frame.
    t.last_reloc_time_ns = frame->time_ns;
    Relocalize(&r, map, *frame, t.pose, rng, &t.p2p_scratch);
    out->lost_reloc_attempted = true;
    t.pose = r.pose;
    t.last_tracked_ids = std::move(r.inlier_ids);
    t.last_world_points = std::move(r.inlier_world_points);
    if (!r.success) {
      t.pose = backup;
    } else {
      t.track_result.visible_out = &out->visible_ids;
      t.track_result.matched_out = &out->visible_matched;
      TrackPoints(map, *frame, t.pose, {}, &t.track_result);
      InsertTracked(&t, t.track_result.all_matches);
      if (t.track_result.residual > s.relocalization_threshold) {
        t.pose = backup;
      } else {
        t.state = 1;
        t.last_state_change_frame = t.frame_count;
        ApplyTrackResult(&t, &t.track_result);
        t.anchor_pose = t.pose;
      }
    }
  } else {
    // ---- G. state-tracking (F926:1760-2647).
    const c8::HMatrix before = t.pose;
    // track-last-points
    t.track_result.visible_out = &out->visible_ids;
    t.track_result.matched_out = &out->visible_matched;
    TrackPoints(map, *frame, t.pose, t.last_tracked_ids, &t.track_result);
    InsertTracked(&t, t.track_result.all_matches);
    t.pose = t.track_result.pose;
    t.last_world_points = std::move(t.track_result.subset_world_points);
    t.track_result.subset_world_points.clear();
    if (t.track_result.residual >= s.tracking_lost_threshold) {
      t.pose = before;
    }

    // Secret relocalization is cloud-only (map.CloudEmpty() is always true here).

    // sample-track-points (F926:1922-2202).
    std::vector<map::PointId> all = map.AllPointIds();  // 1147
    const size_t max_points = static_cast<size_t>(s.max_points_for_second_pass_tracking);
    if (s.sample_points_for_second_pass_tracking && all.size() > max_points) {
      std::set<map::PointId> keep(t.last_tracked_ids.begin(), t.last_tracked_ids.end());
      rng->shuffle(all.begin(), all.end());  // 805
      while (keep.size() < max_points) {
        keep.insert(all.back());
        all.pop_back();
      }
      t.last_tracked_ids.assign(keep.begin(), keep.end());
    } else {
      t.last_tracked_ids.clear();
    }

    // track-sampled-points (F926:2203-2220).
    t.track_result.visible_out = &out->visible_ids;
    t.track_result.matched_out = &out->visible_matched;
    TrackPoints(map, *frame, t.pose, t.last_tracked_ids, &t.track_result);
    InsertTracked(&t, t.track_result.all_matches);
    const float residual = t.track_result.residual;
    ApplyTrackResult(&t, &t.track_result);

    if (residual < s.tracking_lost_threshold) {
      t.last_good_residual = residual;
      // map-tracker-set-frame-tracker-scale (§5.6, F926:2223-2636). Matches of point types
      // TRIANGULATED, DEPTH, CLOUD (mask 0x32), keyed by frame point index (last value wins).
      std::map<uint32_t, c8::HPoint3> position_by_frame;
      std::map<uint32_t, map::PointId> id_by_frame;
      for (const TrackedMatch& m : t.all_matches) {
        const map::WorldPoint& p = map.Point(m.id);
        if (p.source == map::PointSource::kTriangulated || p.source == map::PointSource::kDepth ||
            p.source == map::PointSource::kCloud) {
          position_by_frame[m.frame_index] = p.position;
          id_by_frame[m.frame_index] = m.id;
        }
      }
      if (out->ground_idx.size() >= static_cast<size_t>(s.min_ground_points_for_scale)) {
        std::vector<float> heights;
        for (const auto& kv : id_by_frame) {
          if (std::find(out->ground_idx.begin(), out->ground_idx.end(), kv.first) !=
              out->ground_idx.end()) {
            heights.push_back(map.Point(kv.second).position.y());
          }
        }
        const float ty = t.pose(1, 3);
        if (UpdateGroundHeight(&heights, out->ground_idx.size(), ty, &t.ground_height)) {
          t.ground_y = ty - t.ground_height;
        }
      }
      // Previous-frame map depth per frame point (camera z), -FLT_MAX where unmatched.
      const FrameFrameTrackerAccess::Impl& ft = FrameFrameTrackerAccess::impl(t.frame_tracker);
      std::vector<float> depths(ft.prev_frame.points().size(), -FLT_MAX);
      const c8::HMatrix inv = t.pose.inv();
      for (const auto& kv : position_by_frame) {
        SR_CHECK(kv.first < depths.size(), "[map_tracker] tracked frame index out of range");
        const c8::HPoint3 flat(kv.second.x(), kv.second.y(), kv.second.z());
        depths[kv.first] = (inv * flat).raw()[2];
      }
      t.frame_tracker.SetPrevDepths(depths);
      t.frame_tracker.set_height(t.pose(1, 3) - t.ground_y);
    } else {
      t.pose = ReplaceYaw(t.anchor_pose, rotation);
      t.state = 0;
      t.lost_residual = residual;
      t.last_state_change_frame = t.frame_count;
    }
    t.anchor_pose = t.pose;
  }

  // Output (F926:2648-2701).
  out->state = t.state;
  out->consecutive_frame_tracker_ok = t.consecutive_frame_tracker_ok;
  out->pose = t.pose;
  out->tracked = t.tracked.by_frame;
  t.prev_device_rotation = rotation;
  // The cloud relocalization delay update (§5.7) only acts on a non-empty cloud; nothing to do.
}

}  // namespace slam_refact::map_tracker
