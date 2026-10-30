// verifyPointVisibility (1549) and projectPoints (1548) — architecture.md §3.3, S2 §3.3.
// Read from DCMP func1549 / func1548 (wasm-decompile) and the f1549 disassembly; where the spec
// pseudo-code (S2 §3.3) and the binary differ, the binary is followed (noted inline).

#include <cstdint>
#include <vector>

#include "c8/geometry/egomotion.h"
#include "common/c8_adapters.h"
#include "common/check.h"
#include "config/settings.h"
#include "map/world_map.h"
#include "map/world_map_internal.h"
#include "reality/engine/features/feature-scales.h"

namespace slam_refact::map {

namespace {

// World-space forward axis of a camera: flatten(pose * (0,0,1,1)) - raw translation column.
c8::HVector3 ForwardAxis(const c8::HMatrix& pose) {
  const c8::HPoint3 f = pose * c8::HPoint3(0.0f, 0.0f, 1.0f);
  return c8::HVector3(f.x() - pose(0, 3), f.y() - pose(1, 3), f.z() - pose(2, 3));
}

}  // namespace

VisibilityResult VerifyPointVisibility(const WorldMap& map, const PinholeIntrinsics& intrinsics,
                                       const c8::HMatrix& pose, const std::vector<PointId>& ids) {
  const WorldMapAccess::Impl& m = WorldMapAccess::impl(map);
  const size_t n = ids.size();
  VisibilityResult r;
  r.status.assign(n, 0);
  r.scale.assign(n, 0xff);
  r.views.resize(n);

  // Keyframes whose forward axis is within the viewing angle of the camera (unnormalized cosine).
  std::vector<uint8_t> kf_ok(m.keyframes.size(), 0);
  if (!m.keyframes.empty()) {
    const c8::HVector3 cam_fwd = ForwardAxis(pose);
    const float min_cos = config::WorldMapsSettings::Get().max_viewing_angle_kf_to_cam;
    for (size_t slot = 0; slot < m.keyframes.size(); ++slot) {
      const Keyframe& kf = m.keyframes[slot];
      if (kf.id == 0) {
        continue;
      }
      if (min_cos <= cam_fwd.dot(ForwardAxis(kf.frame.extrinsic()))) {
        kf_ok[slot] = 1;
      }
    }
  }
  if (n == 0) {
    return r;
  }

  c8::HPoint2 lower_left;
  c8::HPoint2 upper_right;
  c8::frameBounds(ToC8Intrinsics(intrinsics), &lower_left, &upper_right);
  const c8::HMatrix camera_from_world = pose.inv();

  for (size_t i = 0; i < n; ++i) {
    const PointId id = ids[i];
    if (id.id() == 0) {
      continue;  // status 0
    }
    const WorldPoint* found = nullptr;
    for (const PointPool& pool : m.pools) {
      if (id.index() < pool.points.size() && pool.points[id.index()].id == id.id()) {
        found = &pool.points[id.index()];
        break;
      }
    }
    if (found == nullptr || found->state == PointState::kDead) {
      continue;  // status 0
    }
    const WorldPoint& p = *found;
    const c8::HPoint3 cam = camera_from_world * p.position;
    const auto& pc = cam.raw();
    const float z = pc[2] / pc[3];
    if (z <= 0.0f) {
      r.status[i] = 2;
      continue;
    }
    const float x = pc[0] / pc[2];
    const float y = pc[1] / pc[2];
    if (!(x >= lower_left.x()) || !(x <= upper_right.x()) || !(y >= lower_left.y()) ||
        !(y <= upper_right.y())) {
      r.status[i] = 2;
      continue;
    }
    if (p.source == PointSource::kCloud && p.observations.empty()) {
      r.status[i] = 1;
      continue;
    }
    // Per observation (binary order): a keyframe that fails the angle test is skipped; a scale
    // range failure sets status 3 only while no observation has been accepted; an accepted view
    // sets status 1. A point whose observations are all in rejected keyframes stays at status 0
    // (S2 §3.3 pseudo-code sets 3 there; DCMP func1549 does not).
    for (const auto& [kf_id, obs_idx] : p.observations) {
      SR_CHECK(kf_id.index() < kf_ok.size(), "[world_map] keyframe slot out of range");
      if (kf_ok[kf_id.index()] == 0) {
        continue;
      }
      const c8::FramePoint& fp = m.keyframes[kf_id.index()].frame.points()[obs_idx];
      SR_CHECK(fp.scale() < c8::NUM_SCALES, "[world_map] keyframe point scale out of range");
      const double d = fp.depthWithCertainty().certainty > 0.0
                           ? fp.depthWithCertainty().depthInWords
                           : static_cast<double>(fp.depth());
      const float dmin = c8::dMinForScale(fp.scale(), static_cast<float>(d));
      if (z < dmin || z >= c8::dMaxForDMin(dmin)) {
        if (r.status[i] == 0) {
          r.status[i] = 3;
        }
        continue;
      }
      const uint8_t s = c8::scaleForDist(dmin, z);
      if (s < r.scale[i]) {
        r.scale[i] = s;
      }
      r.views[i].push_back({kf_id, obs_idx});
      r.status[i] = 1;
    }
  }
  return r;
}

ProjectedPoints ProjectPoints(const WorldMap& map, std::vector<PointId> ids,
                              const PinholeIntrinsics& intrinsics, const c8::HMatrix& pose) {
  const WorldMapAccess::Impl& m = WorldMapAccess::impl(map);
  if (ids.empty()) {
    ids = map.AllPointIds();  // 1147
  }
  const VisibilityResult vis = VerifyPointVisibility(map, intrinsics, pose, ids);

  // Views per keyframe slot over all points; points with views are counted for the reservation.
  std::vector<uint32_t> kf_use(m.keyframes.size(), 0);
  size_t with_views = 0;
  for (const std::vector<PointView>& views : vis.views) {
    for (const PointView& v : views) {
      ++kf_use[v.kf.index()];
    }
    with_views += views.empty() ? 0 : 1;
  }

  ProjectedPoints out;
  out.frame = c8::FrameWithPoints(ToC8Intrinsics(intrinsics), pose);
  out.frame.reserve(with_views);
  for (size_t i = 0; i < ids.size(); ++i) {
    if (vis.status[i] != 1) {
      continue;
    }
    const WorldPoint& p = map.Point(ids[i]);
    if (p.source == PointSource::kCloud) {
      // Cloud descriptors are VPS-only (architecture.md §1.2); cloud points never exist here.
      SR_CHECK(false, "[world_map] cloud point projection is not supported");
    }
    const std::vector<PointView>& views = vis.views[i];
    // Most-used keyframe; ties go to the smaller slot.
    size_t best = 0;
    for (size_t v = 1; v < views.size(); ++v) {
      const uint32_t cand_use = kf_use[views[v].kf.index()];
      const uint32_t best_use = kf_use[views[best].kf.index()];
      if (cand_use > best_use ||
          (cand_use == best_use && views[v].kf.index() < views[best].kf.index())) {
        best = v;
      }
    }
    const Keyframe& kf = m.keyframes[views[best].kf.index()];
    SR_CHECK(kf.id == views[best].kf.id(), "[world_map] projected view keyframe id mismatch");
    const c8::FramePoint& fp = kf.frame.points()[views[best].point_index];
    out.frame.addWorldMapPoint(ids[i], p.position, vis.scale[i], fp.angle(), fp.gravityAngle(),
                               kf.frame.store().getClone(views[best].point_index));
  }
  return out;
}

}  // namespace slam_refact::map
