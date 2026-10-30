// WorldMap / WorldMaps (architecture.md §3.3, WP2-M1). Each method follows the closed function
// named in its comment; the decompile references are build/slam-refact-20261015/re/ghidra/
// export/c/<addr>_unnamed_function_<N>.c (G<N>) and re/wabt/xr-slam.dcmp (DCMP).
//
// Arithmetic: products and dot products use the vendored c8 HMatrix/HPoint operators. The wasm
// stack order of the inlined products in 1127/1150/1549 was checked against the public unrolled
// order (e.g. 1549 pushes zz, yy, xx, 0 and adds right to left = ((0 + xx) + yy) + zz).

#include "map/world_map.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "Eigen/Cholesky"
#include "Eigen/Core"
#include "common/check.h"
#include "compat/libcxx_algorithms.h"
#include "config/accuracy.h"
#include "config/settings.h"
#include "map/world_map_internal.h"
#include "reality/engine/features/descriptor-bit-counter.h"
#include "reality/engine/features/image-descriptor.h"
#include "slam_refact/replay_diagnostics.h"

namespace slam_refact::map {

namespace {

using Impl = WorldMapAccess::Impl;

constexpr int kNumPools = 5;

int PoolIndex(PointSource source) {
  const int s = static_cast<int>(source);
  SR_CHECK(s >= 1 && s <= kNumPools, "[world_map] Invalid point source");
  return s - 1;
}

PointPool& Pool(Impl& m, PointSource source) { return m.pools[PoolIndex(source)]; }

// 634 / 1125 lookup: the first pool (TRI, GROUND, SKY, DEPTH, CLOUD) whose slot holds this id.
// The point's state is not checked; a retired slot still answers to its last id.
WorldPoint* FindPointIn(Impl& m, PointId id) {
  for (PointPool& pool : m.pools) {
    if (id.index() < pool.points.size() && pool.points[id.index()].id == id.id()) {
      return &pool.points[id.index()];
    }
  }
  return nullptr;
}

WorldPoint& PointOrAbort(Impl& m, PointId id) {
  WorldPoint* p = FindPointIn(m, id);
  SR_CHECK(p != nullptr, "[WorldMap::getPoint] WorldMapPointId out of range or id mismatch");
  return *p;
}

bool KeyframeValid(const Impl& m, KeyframeId id) {
  return id.id() != 0 && id.index() < m.keyframes.size() &&
         m.keyframes[id.index()].id == id.id();
}

// Keyframe by slot only (655, 1143, 1144, 1150, 1549 index the slot vector without an id check).
Keyframe& SlotAt(Impl& m, size_t slot) {
  SR_CHECK(slot < m.keyframes.size(), "[world_map] keyframe slot out of range");
  return m.keyframes[slot];
}

// Camera-frame depth of a world point: (camera_from_world * p).z / w (655, 658, 1150).
// Accuracy mode (ACC-1): Gauss-Newton multi-view triangulation of a TRIANGULATED point on the
// current keyframe poses, minimising the Huber-weighted ray reprojection error of every valid
// observation, started from `init` (the original's mean of per-view depths). Returns false (and
// the caller keeps `init`) with fewer than two usable rays, a ray angle below 1 degree, a point
// behind a camera, or a singular system.
bool TriangulateMultiview(const Impl& m, const WorldPoint& p, const c8::HPoint3& init,
                          c8::HPoint3* out) {
  struct Ray {
    Eigen::Matrix3d r_cw;
    Eigen::Vector3d t_cw;
    Eigen::Vector3d center;
    double x;
    double y;
  };
  std::vector<Ray> rays;
  for (const auto& [kf_id, obs_idx] : p.observations) {
    if (kf_id.id() == 0 || kf_id.index() >= m.keyframes.size() ||
        m.keyframes[kf_id.index()].id != kf_id.id()) {
      continue;
    }
    const Keyframe& kf = m.keyframes[kf_id.index()];
    const std::array<float, 16>& d = kf.frame.extrinsic().data();  // world_from_camera, col-major
    Eigen::Matrix3d r_wc;
    for (int c = 0; c < 3; ++c) {
      for (int r = 0; r < 3; ++r) {
        r_wc(r, c) = static_cast<double>(d[static_cast<size_t>(c * 4 + r)]);
      }
    }
    const Eigen::Vector3d c_w(d[12], d[13], d[14]);
    const c8::FramePoint& fp = kf.frame.points()[obs_idx];
    rays.push_back({r_wc.transpose(), -r_wc.transpose() * c_w, c_w, static_cast<double>(fp.x()),
                    static_cast<double>(fp.y())});
  }
  if (rays.size() < 2) {
    return false;
  }
  const auto& raw = init.raw();
  Eigen::Vector3d x(raw[0] / raw[3], raw[1] / raw[3], raw[2] / raw[3]);
  constexpr double kHuber = 0.004;  // ray units (about 2 px at a 500 px focal length)
  for (int it = 0; it < 10; ++it) {
    Eigen::Matrix3d h = Eigen::Matrix3d::Zero();
    Eigen::Vector3d b = Eigen::Vector3d::Zero();
    for (const Ray& ray : rays) {
      const Eigen::Vector3d xc = ray.r_cw * x + ray.t_cw;
      if (xc.z() <= 1e-6) {
        return false;
      }
      const double iz = 1.0 / xc.z();
      const Eigen::Vector2d res(xc.x() * iz - ray.x, xc.y() * iz - ray.y);
      Eigen::Matrix<double, 2, 3> jc;
      jc << iz, 0.0, -xc.x() * iz * iz, 0.0, iz, -xc.y() * iz * iz;
      const Eigen::Matrix<double, 2, 3> j = jc * ray.r_cw;
      const double n = res.norm();
      const double w = n <= kHuber ? 1.0 : kHuber / n;
      h += w * j.transpose() * j;
      b += w * j.transpose() * res;
    }
    const Eigen::LDLT<Eigen::Matrix3d> ldlt(h);
    if (ldlt.info() != Eigen::Success || !(ldlt.vectorD().minCoeff() > 1e-12)) {
      return false;
    }
    const Eigen::Vector3d dx = -ldlt.solve(b);
    if (!dx.allFinite()) {
      return false;
    }
    x += dx;
    if (dx.norm() < 1e-7 * (1.0 + x.norm())) {
      break;
    }
  }
  double max_angle = 0.0;
  for (size_t i = 0; i < rays.size(); ++i) {
    const Eigen::Vector3d xc = rays[i].r_cw * x + rays[i].t_cw;
    if (xc.z() <= 1e-6) {
      return false;
    }
    for (size_t j = i + 1; j < rays.size(); ++j) {
      const Eigen::Vector3d a = (x - rays[i].center).normalized();
      const Eigen::Vector3d c = (x - rays[j].center).normalized();
      max_angle = std::max(max_angle, std::acos(std::min(1.0, std::max(-1.0, a.dot(c)))));
    }
  }
  if (max_angle < 1.0 * 3.14159265358979323846 / 180.0) {
    return false;
  }
  *out = c8::HPoint3(static_cast<float>(x.x()), static_cast<float>(x.y()),
                     static_cast<float>(x.z()));
  return true;
}

// Accuracy mode (ACC-1): least-squares position of a GROUND point on the original's ground plane
// y = -1 (936 [D]) from every valid observation: (x, z) minimising the Huber-weighted ray
// reprojection error, started from `init`. Returns false with no usable ray.
bool TriangulateOnGroundPlane(const Impl& m, const WorldPoint& p, const c8::HPoint3& init,
                              c8::HPoint3* out) {
  constexpr double kGroundY = -1.0;
  struct Ray {
    Eigen::Matrix3d r_cw;
    Eigen::Vector3d t_cw;
    double x;
    double y;
  };
  std::vector<Ray> rays;
  for (const auto& [kf_id, obs_idx] : p.observations) {
    if (kf_id.id() == 0 || kf_id.index() >= m.keyframes.size() ||
        m.keyframes[kf_id.index()].id != kf_id.id()) {
      continue;
    }
    const Keyframe& kf = m.keyframes[kf_id.index()];
    const std::array<float, 16>& d = kf.frame.extrinsic().data();
    Eigen::Matrix3d r_wc;
    for (int c = 0; c < 3; ++c) {
      for (int r = 0; r < 3; ++r) {
        r_wc(r, c) = static_cast<double>(d[static_cast<size_t>(c * 4 + r)]);
      }
    }
    const Eigen::Vector3d c_w(d[12], d[13], d[14]);
    const c8::FramePoint& fp = kf.frame.points()[obs_idx];
    rays.push_back({r_wc.transpose(), -r_wc.transpose() * c_w, static_cast<double>(fp.x()),
                    static_cast<double>(fp.y())});
  }
  if (rays.empty()) {
    return false;
  }
  const auto& raw = init.raw();
  Eigen::Vector2d xz(raw[0] / raw[3], raw[2] / raw[3]);
  constexpr double kHuber = 0.004;
  for (int it = 0; it < 10; ++it) {
    Eigen::Matrix2d h = Eigen::Matrix2d::Zero();
    Eigen::Vector2d b = Eigen::Vector2d::Zero();
    const Eigen::Vector3d x(xz.x(), kGroundY, xz.y());
    for (const Ray& ray : rays) {
      const Eigen::Vector3d xc = ray.r_cw * x + ray.t_cw;
      if (xc.z() <= 1e-6) {
        return false;
      }
      const double iz = 1.0 / xc.z();
      const Eigen::Vector2d res(xc.x() * iz - ray.x, xc.y() * iz - ray.y);
      Eigen::Matrix<double, 2, 3> jc;
      jc << iz, 0.0, -xc.x() * iz * iz, 0.0, iz, -xc.y() * iz * iz;
      const Eigen::Matrix<double, 2, 3> j3 = jc * ray.r_cw;
      Eigen::Matrix2d j;
      j << j3(0, 0), j3(0, 2), j3(1, 0), j3(1, 2);
      const double n = res.norm();
      const double w = n <= kHuber ? 1.0 : kHuber / n;
      h += w * j.transpose() * j;
      b += w * j.transpose() * res;
    }
    const Eigen::LDLT<Eigen::Matrix2d> ldlt(h);
    if (ldlt.info() != Eigen::Success || !(ldlt.vectorD().minCoeff() > 1e-12)) {
      return false;
    }
    const Eigen::Vector2d dx = -ldlt.solve(b);
    if (!dx.allFinite()) {
      return false;
    }
    xz += dx;
    if (dx.norm() < 1e-7 * (1.0 + xz.norm())) {
      break;
    }
  }
  *out = c8::HPoint3(static_cast<float>(xz.x()), static_cast<float>(kGroundY),
                     static_cast<float>(xz.y()));
  return true;
}

float CameraDepth(const c8::HMatrix& world_from_camera, const c8::HPoint3& p) {
  return (world_from_camera.inv() * p).z();
}

void ReleaseMatcherCaches(Keyframe* kf) {
  kf->new_points_matcher_orb.reset();
  kf->new_points_matcher_gorb.reset();
  kf->epipolar_matcher_orb.reset();
  kf->epipolar_matcher_gorb.reset();
}

// GORB bits of keyframe point kp_index into / out of the point's majority record (658).
void MajorityAdd(Impl& m, const WorldPoint& p, PointId pid, const Keyframe& kf, size_t kp_index) {
  if (p.source == PointSource::kCloud) {
    return;
  }
  const c8::FeatureStore& store = kf.frame.store();
  SR_CHECK(kp_index < store.numKeypoints(), "[world_map] keypoint index out of range");
  if (!store.has<c8::GorbFeature>(kp_index)) {
    return;
  }
  PointPool& pool = Pool(m, p.source);
  SR_CHECK(pid.index() < pool.majority.size(), "[world_map] majority record out of range");
  MajorityRecord& rec = pool.majority[pid.index()];
  c8::addOneCounts<32>(store.get<c8::GorbFeature>(kp_index).data(), &rec.one_counts);
  rec.majority_valid = false;
  ++rec.count;
}

void MajoritySubtract(Impl& m, const WorldPoint& p, PointId pid, const Keyframe& kf,
                      size_t kp_index) {
  if (p.source == PointSource::kCloud) {
    return;
  }
  const c8::FeatureStore& store = kf.frame.store();
  SR_CHECK(kp_index < store.numKeypoints(), "[world_map] keypoint index out of range");
  if (!store.has<c8::GorbFeature>(kp_index)) {
    return;
  }
  PointPool& pool = Pool(m, p.source);
  SR_CHECK(pid.index() < pool.majority.size(), "[world_map] majority record out of range");
  MajorityRecord& rec = pool.majority[pid.index()];
  if (rec.count == 0) {
    // The original only prints this diagnostic and continues (658).
    std::fprintf(stderr, "[majority image descriptor] Failed to remove a descriptor from an "
                         "empty majority\n");
    return;
  }
  c8::subtractOneCounts<32>(store.get<c8::GorbFeature>(kp_index).data(), &rec.one_counts);
  rec.majority_valid = false;
  --rec.count;
}

// 4071: erase kf from the point's observations and scale maps; abort if it was not observed.
void RemoveObservationRecord(WorldPoint* p, KeyframeId kf) {
  SR_CHECK(p->observations.count(kf) != 0, "[world_map] point is not observed by keyframe");
  p->observations.erase(kf);
  p->scale_by_kf.erase(kf);
}

// 1130 setPointState. DEAD unlinks every observation, frees the slot and drops the point from
// the dirty set; cloud points are never retired.
void SetPointState(WorldMap* map, Impl& m, PointId id, PointState state) {
  WorldPoint& p = PointOrAbort(m, id);
  if (p.state == state) {
    return;
  }
  if (state == PointState::kDead) {
    if (p.source == PointSource::kCloud) {
      return;
    }
    const std::map<KeyframeId, uint32_t> observations = p.observations;
    for (const auto& [kf, obs_idx] : observations) {
      map->AddObservation(PointId::nullId(), obs_idx, kf);
    }
    Pool(m, p.source).free_slots.push_back(static_cast<uint32_t>(id.index()));
    m.dirty.erase(id);
  }
  p.state = state;
}

// 1562, called by 1141 for every slot of pools TRI..DEPTH.
void TenurePoint(WorldMap* map, Impl& m, PointId id, const WorldPoint& p) {
  if (p.source == PointSource::kCloud) {
    return;
  }
  const config::WorldMapsSettings& s = config::WorldMapsSettings::Get();
  if (p.state == PointState::kNew) {
    // Unsigned arithmetic and compares (VERIFIED i32.lt_u, func 1562 @0x161f2b).
    const uint32_t age = m.keyframe_seq + ~p.birth_kf_seq;
    if (age < static_cast<uint32_t>(s.tenure_keyframe_count)) {
      return;
    }
    const bool fail =
        p.tracking_count < static_cast<uint32_t>(s.required_tracking_count) ||
        !(s.required_tracking_hit_ratio * static_cast<float>(p.tracking_count) <=
          static_cast<float>(p.tracking_hits)) ||
        (p.source == PointSource::kSky && !s.allow_tenured_skypoints);
    if (fail) {
      if (!s.enable_untenure_instead_of_prune) {
        SetPointState(map, m, id, PointState::kDead);
      }
      return;
    }
    WorldPoint& q = PointOrAbort(m, id);
    if (q.state != PointState::kTenured) {
      q.state = PointState::kTenured;
    }
  } else if (p.state == PointState::kTenured) {
    if (!(s.required_tenured_tracking_hit_ratio * static_cast<float>(p.tracking_count) >
          static_cast<float>(p.tracking_hits))) {
      return;
    }
    if (!s.enable_untenure_instead_of_prune) {
      SetPointState(map, m, id, PointState::kDead);
      return;
    }
    WorldPoint& q = PointOrAbort(m, id);
    if (q.state != PointState::kNew) {
      q.state = PointState::kNew;
    }
  }
}

// 1142 / 1143 comparator: ascending keyframe creation frame (DCMP:418105, 417575).
void SortByCreation(const Impl& m, std::vector<KeyframeId>* ids) {
  slam_refact::libcxx::sort(ids->begin(), ids->end(), [&m](KeyframeId a, KeyframeId b) {
    SR_CHECK(a.index() < m.keyframes.size() && b.index() < m.keyframes.size(),
             "[world_map] keyframe slot out of range");
    return m.keyframes[a.index()].creation_frame < m.keyframes[b.index()].creation_frame;
  });
}

}  // namespace

// ---------------------------------------------------------------------------------------------
// WorldMap

WorldMap::WorldMap() : impl_(std::make_unique<Impl>()) {
  impl_->id_generator = &impl_->own_generator;
  impl_->map_id = impl_->NextId();
  // 642: capacity reservations. Keyframe slots must not reallocate while matchers hold pointers
  // to keyframe frames (S3 §2.3); 100 slots exceed the 64-keyframe cap plus free slots.
  impl_->keyframes.reserve(100);
  impl_->free_keyframe_slots.reserve(100);
  impl_->pools[0].points.reserve(5000);
  for (int i = 1; i < 4; ++i) {
    impl_->pools[i].points.reserve(1000);
  }
}

WorldMap::~WorldMap() = default;

// 654.
KeyframeId WorldMap::AddKeyframe(const c8::FrameWithPoints& f, int64_t frame_count) {
  Impl& m = *impl_;
  size_t slot = m.keyframes.size();
  bool reuse = false;
  if (!m.free_keyframe_slots.empty() && m.free_keyframe_slots.back() != m.keyframes.size()) {
    slot = m.free_keyframe_slots.back();
    m.free_keyframe_slots.pop_back();
    SR_CHECK(slot < m.keyframes.size(), "[world_map] free keyframe slot out of range");
    reuse = true;
  }
  if (!reuse) {
    m.keyframes.emplace_back();
  }
  Keyframe& kf = m.keyframes[slot];
  kf.frame.clearAndClone(f);  // 663 (new slot) / 1015 (reused slot)
  if (reuse) {
    ReleaseMatcherCaches(&kf);  // +0x3ac..+0x3cc; prune_protect is kept, as in 654
  }
  const uint32_t id = m.NextId();
  kf.id = id;
  kf.seq = m.keyframe_seq++;
  kf.creation_frame = frame_count;
  // 4067: a keyframe source frame must not carry world point links.
  SR_CHECK(kf.frame.worldPointIndices().empty(), "[world_map] keyframe frame has world points");
  const KeyframeId kf_id(slot, id);
  m.newest = kf_id;
  m.last_keyframe_frame = frame_count;
  return kf_id;
}

// 655: setExtrinsic (inline initial-depth update), world point depths, dirty set, both
// directions of every existing point view.
void WorldMap::SetKeyframePose(KeyframeId kf_id, const c8::HMatrix& world_from_camera) {
  Impl& m = *impl_;
  Keyframe& kf = SlotAt(m, kf_id.index());
  (void)world_from_camera.inv();  // aborts on noInvert / invertFailed, like 655
  kf.frame.setExtrinsic(world_from_camera);
  for (size_t idx : kf.frame.worldPointIndices()) {
    const PointId pid = kf.frame.points()[idx].id();
    const WorldPoint& p = PointOrAbort(m, pid);
    (*kf.frame.mutablePoints())[idx].setDepth(CameraDepth(kf.frame.extrinsic(), p.position));
    m.dirty.insert(pid);
  }
  std::vector<KeyframeId> viewers;
  viewers.reserve(kf.frame.pointsViewedByKeyframe().size());
  for (const auto& entry : kf.frame.pointsViewedByKeyframe()) {
    viewers.push_back(entry.first);
  }
  for (KeyframeId other_id : viewers) {
    Keyframe& other = SlotAt(m, other_id.index());
    kf.frame.updateCurrentPointViews(other_id, other.frame);
    other.frame.updateCurrentPointViews(kf_id, kf.frame);
  }
}

// 657 (world SLAM: sources TRI..DEPTH, id from the generator).
PointId WorldMap::AddPoint(uint32_t kf_seq, const c8::HPoint3& p, PointSource source) {
  SR_CHECK(source != PointSource::kCloud, "[world_map] cloud points are not supported");
  Impl& m = *impl_;
  PointPool& pool = Pool(m, source);
  size_t slot;
  if (pool.free_slots.empty()) {
    slot = pool.points.size();
    pool.points.emplace_back();
    pool.majority.emplace_back();
  } else {
    slot = pool.free_slots.back();
    pool.free_slots.pop_back();
    SR_CHECK(slot < pool.majority.size(), "[world_map] free point slot out of range");
    pool.majority[slot] = MajorityRecord{};
  }
  SR_CHECK(slot < pool.points.size(), "[world_map] point slot out of range");
  const uint32_t id = m.NextId();
  WorldPoint& w = pool.points[slot];
  w.source = source;
  w.min_scale = 0xff;
  w.tracking_hits = 0;
  w.tracking_count = 0;
  w.state = PointState::kNew;
  w.birth_kf_seq = kf_seq;
  w.id = id;
  w.scale_by_kf.clear();
  w.observations.clear();
  w.position = p;
  // loaded_from_map (+0x44) is not reset on slot reuse (657); it is never set in world SLAM.
  return PointId(slot, id);
}

// 658 (→ 4069/4070/4071/4072/4074).
void WorldMap::AddObservation(PointId point, uint32_t obs_idx, KeyframeId kf_id) {
  Impl& m = *impl_;
  SR_CHECK(KeyframeValid(m, kf_id), "Associating invalid keyframe id");
  Keyframe& kf = m.keyframes[kf_id.index()];
  SR_CHECK(obs_idx < kf.frame.points().size(), "[world_map] observation index out of range");
  const PointId old_id = kf.frame.points()[obs_idx].id();
  if (old_id == point) {
    return;
  }
  if (old_id != PointId::nullId()) {
    WorldPoint* old_p = FindPointIn(m, old_id);
    SR_CHECK(old_id.id() != 0 && old_p != nullptr, "Associating invalid world point id");
    for (const auto& [other_id, other_idx] : old_p->observations) {
      if (other_id == kf_id) {
        continue;
      }
      SlotAt(m, other_id.index()).frame.removePointView(kf_id, other_idx);
      kf.frame.removePointView(other_id, obs_idx);
      m.dirty.insert(old_id);
    }
    kf.frame.setWorldPoint(obs_idx, PointId::nullId());
    RemoveObservationRecord(old_p, kf_id);
    MajoritySubtract(m, *old_p, old_id, kf, obs_idx);
  }
  if (point == PointId::nullId()) {
    return;
  }
  WorldPoint& p = PointOrAbort(m, point);
  // The point may already be observed at another index of this keyframe: unlink that one first.
  const auto existing = p.observations.find(kf_id);
  if (existing != p.observations.end()) {
    const uint32_t prev_idx = existing->second;
    for (const auto& [other_id, other_idx] : p.observations) {
      if (other_id == kf_id) {
        continue;
      }
      SlotAt(m, other_id.index()).frame.removePointView(kf_id, other_idx);
      kf.frame.removePointView(other_id, prev_idx);
    }
    kf.frame.setWorldPoint(prev_idx, PointId::nullId());
    RemoveObservationRecord(&p, kf_id);
    MajoritySubtract(m, p, point, kf, prev_idx);
  }
  kf.frame.setWorldPoint(obs_idx, point);
  const uint8_t scale = kf.frame.points()[obs_idx].scale();
  if (scale < p.min_scale) {
    p.min_scale = scale;
  }
  p.observations.emplace(kf_id, obs_idx);
  p.scale_by_kf.emplace(kf_id, scale);
  MajorityAdd(m, p, point, kf, obs_idx);
  if (p.source == PointSource::kDepth) {
    kf.frame.setKnownDepth(obs_idx, CameraDepth(kf.frame.extrinsic(), p.position));
  }
  for (const auto& [other_id, other_idx] : p.observations) {
    if (other_id == kf_id) {
      continue;
    }
    Keyframe& other = SlotAt(m, other_id.index());
    other.frame.addPointView(kf_id, kf.frame, obs_idx, other_idx);
    kf.frame.addPointView(other_id, other.frame, other_idx, obs_idx);
  }
  m.dirty.insert(point);
  (*kf.frame.mutablePoints())[obs_idx].setDepth(CameraDepth(kf.frame.extrinsic(), p.position));
}

const WorldPoint& WorldMap::Point(PointId id) const {
  const WorldPoint* p = FindPointIn(const_cast<Impl&>(*impl_), id);
  SR_CHECK(p != nullptr, "[WorldMap::getPoint] WorldMapPointId out of range or id mismatch");
  return *p;
}

WorldPoint& WorldMap::MutablePoint(PointId id) {
  WorldPoint* p = FindPointIn(*impl_, id);
  SR_CHECK(p != nullptr, "Invalid point requested");
  return *p;
}

const Keyframe& WorldMap::GetKeyframe(KeyframeId id) const {
  SR_CHECK(KeyframeValid(*impl_, id), "[WorldMap::getKeyframe] invalid keyframe id");
  return impl_->keyframes[id.index()];
}

Keyframe& WorldMap::MutableKeyframe(KeyframeId id) {
  SR_CHECK(KeyframeValid(*impl_, id), "[WorldMap::getMutableKeyframe] invalid keyframe id");
  return impl_->keyframes[id.index()];
}

// 1130 with state DEAD.
void WorldMap::Retire(PointId id) { SetPointState(this, *impl_, id, PointState::kDead); }

// 1124: retire src, then move its observations to dst where dst has no view from that keyframe.
void WorldMap::MergePoint(PointId src, PointId dst) {
  Impl& m = *impl_;
  const std::map<KeyframeId, uint32_t> src_observations = PointOrAbort(m, src).observations;
  PointOrAbort(m, dst);
  SetPointState(this, m, src, PointState::kDead);
  for (const auto& [kf_id, obs_idx] : src_observations) {
    if (PointOrAbort(m, dst).observations.count(kf_id) == 0) {
      AddObservation(dst, obs_idx, kf_id);
    }
  }
}

// 1127 (WorldMap.adjustPoints / adjustUntriangulatedPoints). Repeats until the dirty set stays
// empty, since unlinking inside 1150 re-dirties points.
void WorldMap::AdjustDirtyPoints() {
  Impl& m = *impl_;
  const config::WorldMapSettings& s = config::WorldMapSettings::Get();
  if (!s.adjust_points) {
    return;
  }
  while (!m.dirty.empty()) {
    const std::set<PointId> batch = std::move(m.dirty);
    m.dirty.clear();
    for (PointId id : batch) {
      const WorldPoint& p = PointOrAbort(m, id);
      if (p.observations.empty()) {
        SetPointState(this, m, id, PointState::kDead);
        continue;
      }
      bool use_initial_depth;
      switch (p.source) {
        case PointSource::kTriangulated:
          use_initial_depth = false;
          break;
        case PointSource::kGround:
        case PointSource::kSky:
          if (!s.adjust_untriangulated_points) {
            continue;
          }
          use_initial_depth = true;
          break;
        case PointSource::kDepth:
          continue;
        default:
          SR_CHECK(false, "[world_map] cloud point adjustment is not supported");
          continue;
      }
      double sum_x = 0.0;
      double sum_y = 0.0;
      double sum_z = 0.0;
      int n = 0;
      for (const auto& [kf_id, obs_idx] : p.observations) {
        const Keyframe& kf = SlotAt(m, kf_id.index());
        const c8::FramePoint& fp = kf.frame.points()[obs_idx];
        const double d =
            use_initial_depth ? fp.initialDepth() : fp.depthWithCertainty().depthInWords;
        const c8::HPoint3 cam(static_cast<float>(d * static_cast<double>(fp.x())),
                              static_cast<float>(d * static_cast<double>(fp.y())),
                              static_cast<float>(d));
        const c8::HPoint3 world = kf.frame.extrinsic() * cam;
        sum_z += static_cast<double>(world.z());
        sum_y += static_cast<double>(world.y());
        sum_x += static_cast<double>(world.x());
        ++n;
      }
      const double inv_n = 1.0 / static_cast<double>(n);
      c8::HPoint3 position(static_cast<float>(sum_x * inv_n), static_cast<float>(sum_y * inv_n),
                           static_cast<float>(sum_z * inv_n));
      if (config::Accuracy().enabled && config::Accuracy().multiview_triangulation) {
        c8::HPoint3 refined;
        if (p.source == PointSource::kTriangulated && TriangulateMultiview(m, p, position, &refined)) {
          position = refined;
        } else if (p.source == PointSource::kGround && config::Accuracy().ba_ground_points == 1 &&
                   TriangulateOnGroundPlane(m, p, position, &refined)) {
          position = refined;
        } else if (p.source == PointSource::kGround && config::Accuracy().ba_ground_points == 2 &&
                   TriangulateMultiview(m, p, position, &refined)) {
          position = refined;
        }
      }
      SetPointPosition(id, position);
    }
  }
}

// 1144.
void WorldMap::RemoveKeyframe(KeyframeId kf_id) {
  Impl& m = *impl_;
  SR_CHECK(KeyframeValid(m, kf_id), "[world_map] removing an invalid keyframe");
  const std::set<size_t> world_indices = m.keyframes[kf_id.index()].frame.worldPointIndices();
  for (size_t idx : world_indices) {
    const PointId pid = m.keyframes[kf_id.index()].frame.points()[idx].id();
    AddObservation(PointId::nullId(), static_cast<uint32_t>(idx), kf_id);
    if (PointOrAbort(m, pid).observations.size() < 2) {
      SetPointState(this, m, pid, PointState::kDead);
    }
  }
  m.free_keyframe_slots.push_back(static_cast<uint32_t>(kf_id.index()));
  Keyframe& kf = m.keyframes[kf_id.index()];
  kf.id = 0;
  kf.frame.clear();  // 913
  ReleaseMatcherCaches(&kf);
}

// 1142.
std::vector<KeyframeId> WorldMap::AllKeyframesByCreation() const {
  const Impl& m = *impl_;
  std::vector<KeyframeId> out;
  for (size_t slot = 0; slot < m.keyframes.size(); ++slot) {
    if (m.keyframes[slot].id != 0) {
      out.emplace_back(slot, m.keyframes[slot].id);
    }
  }
  SortByCreation(m, &out);
  return out;
}

// 1143. min_shared is WorldMap.requiredPointsForCovisibility in the original (default 10).
std::vector<KeyframeId> WorldMap::CovisibleKeyframes(KeyframeId kf_id, int min_shared) const {
  const Impl& m = *impl_;
  std::vector<KeyframeId> out;
  if (m.keyframes.size() <= 1) {
    return out;
  }
  const Keyframe& kf = SlotAt(const_cast<Impl&>(m), kf_id.index());
  for (const auto& [other_id, shared] : kf.frame.pointsViewedByKeyframe()) {
    if (static_cast<uint32_t>(min_shared) <= static_cast<uint32_t>(shared.size())) {
      out.push_back(other_id);
    }
  }
  SortByCreation(m, &out);
  return out;
}

// 1147: every slot of every pool, in pool order, including retired slots.
std::vector<PointId> WorldMap::AllPointIds() const {
  const Impl& m = *impl_;
  size_t total = 0;
  for (const PointPool& pool : m.pools) {
    total += pool.points.size();
  }
  std::vector<PointId> out;
  out.reserve(total);
  for (const PointPool& pool : m.pools) {
    for (size_t slot = 0; slot < pool.points.size(); ++slot) {
      out.emplace_back(slot, pool.points[slot].id);
    }
  }
  return out;
}

// 1141 → 1562 over pools TRI, GROUND, SKY, DEPTH.
void WorldMap::CullAndTenureAll() {
  Impl& m = *impl_;
  for (int pool_index = 0; pool_index < 4; ++pool_index) {
    for (size_t slot = 0; slot < m.pools[pool_index].points.size(); ++slot) {
      const WorldPoint& p = m.pools[pool_index].points[slot];
      TenurePoint(this, m, PointId(slot, p.id), p);
    }
  }
}

uint32_t WorldMap::keyframe_seq_counter() const { return impl_->keyframe_seq; }

int64_t WorldMap::last_keyframe_frame() const { return impl_->last_keyframe_frame; }

KeyframeId WorldMap::newest() const { return impl_->newest; }

int WorldMap::NumKeyframes() const {
  int n = 0;
  for (const Keyframe& kf : impl_->keyframes) {
    n += kf.id != 0 ? 1 : 0;
  }
  return n;
}

std::array<int, 5> WorldMap::NumPointsBySource() const {
  std::array<int, 5> out{};
  for (int i = 0; i < kNumPools; ++i) {
    for (const WorldPoint& p : impl_->pools[i].points) {
      out[i] += p.state != PointState::kDead ? 1 : 0;
    }
  }
  return out;
}

// ---------------------------------------------------------------------------------------------
// Module-internal functions (world_map_internal.h)

void WorldMapAccess::BindGenerator(WorldMap& map, std::mt19937* generator) {
  map.impl_->id_generator = generator;
  map.impl_->map_id = map.impl_->NextId();
}

void SetPointPosition(WorldMap& map, PointId id, const c8::HPoint3& position) {
  map.SetPointPosition(id, position);
}

size_t KeyframeSlotCount(const WorldMap& map) { return map.NumKeyframeSlots(); }

// ---------------------------------------------------------------------------------------------
// Slot-level public access (WP2-CC-1, WP3-CC-1)

size_t WorldMap::NumKeyframeSlots() const { return impl_->keyframes.size(); }

const Keyframe& WorldMap::KeyframeAtSlot(size_t slot) const {
  return SlotAt(*impl_, slot);
}

Keyframe& WorldMap::MutableKeyframeAtSlot(size_t slot) { return SlotAt(*impl_, slot); }

bool WorldMap::IsKeyframeValid(KeyframeId id) const { return KeyframeValid(*impl_, id); }

const WorldPoint* WorldMap::FindPoint(PointId id) const {
  return FindPointIn(*impl_, id);
}

void WorldMap::MarkPointDirty(PointId id) { impl_->dirty.insert(id); }

// 1150.
void WorldMap::SetPointPosition(PointId id, const c8::HPoint3& position) {
  WorldMap& map = *this;
  Impl& m = *impl_;
  const float max_residual =
      config::WorldMapSettings::Get().max_internal_squared_residual_for_triangulated_pts;
  WorldPoint& p = map.MutablePoint(id);
  p.position = position;
  std::vector<std::pair<KeyframeId, uint32_t>> unlink;
  for (const auto& [kf_id, obs_idx] : p.observations) {
    Keyframe& kf = SlotAt(m, kf_id.index());
    const c8::HPoint3 cam = kf.frame.extrinsic().inv() * position;
    c8::FramePoint& fp = (*kf.frame.mutablePoints())[obs_idx];
    const auto& r = cam.raw();
    const float dy = r[1] / r[2] - fp.y();
    const float dx = r[0] / r[2] - fp.x();
    float sq = 0.0f;
    sq = sq + dx * dx;
    sq = sq + dy * dy;
    if (sq > max_residual) {
      unlink.emplace_back(kf_id, obs_idx);
    }
    fp.setDepth(r[2] / r[3]);
  }
  for (const auto& [kf_id, obs_idx] : unlink) {
    map.AddObservation(PointId::nullId(), obs_idx, kf_id);
  }
}

MajorityRecord& MutableMajorityRecord(WorldMap& map, PointSource source, uint32_t slot) {
  PointPool& pool = Pool(WorldMapAccess::impl(map), source);
  SR_CHECK(slot < pool.majority.size(), "[world_map] majority record out of range");
  return pool.majority[slot];
}

// ---------------------------------------------------------------------------------------------
// WorldMaps

namespace {

// diagnostics::SetReplayIdGeneratorSeed (exact-input replays only).
std::optional<uint32_t>& ReplayIdGeneratorSeed() {
  static std::optional<uint32_t> seed;
  return seed;
}

}  // namespace

struct WorldMaps::Impl {
  // MapBuilder +0x10 mt19937 / +0x9d4 generator (G641). The original seeds it with one
  // getentropy() word per run; here kIdGeneratorSeed, or a replay's recorded seed.
  std::mt19937 generator{ReplayIdGeneratorSeed().value_or(kIdGeneratorSeed)};
  std::unique_ptr<WorldMap> active;
  // VioTracker +7240: a second draw right after the first map's id (VioTracker ctor, DCMP:28431).
  // Not read by world SLAM, but it advances the id generator, so every later map / keyframe / point
  // id depends on it. Known gap: the 476 re-creation (disableVio on, DCMP:26247) also makes a new map
  // plus this draw from the new MapBuilder's generator; RecreateMapTrackerAndBuilder does not
  // reproduce that (ids are per-run random in the original anyway; world SLAM default never takes it).
  uint32_t maps_id = 0;
};

WorldMaps::WorldMaps() : impl_(std::make_unique<Impl>()) {
  impl_->active = std::make_unique<WorldMap>();
  WorldMapAccess::BindGenerator(*impl_->active, &impl_->generator);
  impl_->maps_id = impl_->generator();
}

WorldMaps::~WorldMaps() = default;

WorldMap& WorldMaps::active() { return *impl_->active; }

const WorldMap& WorldMaps::active() const { return *impl_->active; }

// 653: the new map (with its id drawn) is created before the old one is destroyed.
void WorldMaps::ResetMap() {
  auto fresh = std::make_unique<WorldMap>();
  WorldMapAccess::BindGenerator(*fresh, &impl_->generator);
  impl_->active = std::move(fresh);
}

}  // namespace slam_refact::map

namespace slam_refact::diagnostics {

void SetReplayIdGeneratorSeed(std::optional<uint32_t> seed) { map::ReplayIdGeneratorSeed() = seed; }

}  // namespace slam_refact::diagnostics
