// P2 candidate F2 (f2-contract-v3.3.md). See f2_confirm.h.

#include "map/f2_confirm.h"

#include <algorithm>
#include <cmath>
#include <set>
#include <utility>

#include "Eigen/Cholesky"
#include "config/accuracy.h"

namespace slam_refact {
namespace {

AccuracyCounters& Store() {
  static AccuracyCounters counters;
  return counters;
}

}  // namespace

const AccuracyCounters& Counters() { return Store(); }
void ResetCounters() { Store() = AccuracyCounters{}; }

}  // namespace slam_refact

namespace slam_refact::map {
namespace {

using Impl = WorldMapAccess::Impl;

int64_t g_applied_frame = -1;
uint64_t g_track_bonus = 0;

constexpr int kGroundPool = static_cast<int>(PointSource::kGround) - 1;

bool KeyframeValid(const Impl& m, KeyframeId id) {
  return id.id() != 0 && id.index() < m.keyframes.size() && m.keyframes[id.index()].id == id.id();
}

bool AllFinite(const Eigen::Vector3d& v) { return v.allFinite(); }

double AngleBetween(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
  return std::acos(std::clamp(a.dot(b), -1.0, 1.0));
}

uint64_t Fnv(uint64_t h, uint64_t v) {
  for (int i = 0; i < 8; ++i) {
    h ^= (v >> (8 * i)) & 0xffu;
    h *= 1099511628211ull;
  }
  return h;
}

}  // namespace

AccuracyCounters& MutableCounters() {
  return const_cast<AccuracyCounters&>(Counters());
}

bool F2Enabled() {
  const AccuracyConfig& a = config::Accuracy();
  return a.enabled && a.f2_ground_confirmation;
}

bool F2Triangulate(const std::vector<Eigen::Vector3d>& centers,
                   const std::vector<Eigen::Vector3d>& dirs, Eigen::Vector3d* x) {
  if (centers.size() != dirs.size() || centers.empty()) {
    return false;
  }
  Eigen::Matrix3d a = Eigen::Matrix3d::Zero();
  Eigen::Vector3d b = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < dirs.size(); ++i) {
    const Eigen::Matrix3d p = Eigen::Matrix3d::Identity() - dirs[i] * dirs[i].transpose();
    a += p;
    b += p * centers[i];
  }
  const Eigen::LDLT<Eigen::Matrix3d> ldlt(a);
  if (ldlt.info() != Eigen::Success || !(ldlt.vectorD().minCoeff() > kF2MinPivot)) {
    return false;
  }
  const Eigen::Vector3d sol = ldlt.solve(b);
  if (!AllFinite(sol)) {
    return false;
  }
  *x = sol;
  return true;
}

F2EvalResult F2Evaluate(const F2EvalInput& in) {
  F2EvalResult r;
  auto fail = [&r](F2Reason reason) {
    r.status = F2Status::kUnconfirmed;
    r.reason = reason;
    return r;
  };

  // 1. invalid_input: computed on every given view before anything else.
  bool finite = in.plane.n.allFinite() && std::isfinite(in.plane.offset);
  for (const F2View& v : in.views) {
    finite = finite && v.center.allFinite() && v.r_wc.allFinite() && std::isfinite(v.x) &&
             std::isfinite(v.y);
  }
  if (!finite) {
    return fail(F2Reason::kInvalidInput);
  }

  // 2. too_few_kfs: distinct keyframes (first occurrence of an id is used), creator included.
  std::vector<const F2View*> views;
  for (const F2View& v : in.views) {
    const bool seen = std::any_of(views.begin(), views.end(),
                                  [&v](const F2View* o) { return o->kf_id == v.kf_id; });
    if (!seen) {
      views.push_back(&v);
    }
  }
  r.distinct_kfs = static_cast<int>(views.size());
  const F2View* creator = nullptr;
  for (const F2View* v : views) {
    if (v->kf_id == in.creator_kf_id) {
      creator = v;
    }
  }
  if (!in.creator_alive || creator == nullptr) {
    r.no_creator = true;
    return fail(F2Reason::kTooFewKfs);
  }
  if (r.distinct_kfs < kF2MinKeyframes) {
    return fail(F2Reason::kTooFewKfs);
  }

  // 3. degenerate: directions d_i = R_wc (x, y, 1) / |.|, midpoint triangulation.
  std::vector<Eigen::Vector3d> centers;
  std::vector<Eigen::Vector3d> dirs;
  for (const F2View* v : views) {
    const Eigen::Vector3d w = v->r_wc * Eigen::Vector3d(v->x, v->y, 1.0);
    const double n = w.norm();
    if (!(n > 0.0) || !std::isfinite(n)) {
      return fail(F2Reason::kDegenerate);
    }
    centers.push_back(v->center);
    dirs.push_back(w / n);
  }
  if (!F2Triangulate(centers, dirs, &r.x)) {
    return fail(F2Reason::kDegenerate);
  }

  // 4. negative_depth: in front of every camera (along the ray and in camera z), and d_mv > 0.
  r.d_mv = (creator->r_wc.transpose() * (r.x - creator->center)).z();
  for (size_t i = 0; i < views.size(); ++i) {
    const Eigen::Vector3d rel = r.x - views[i]->center;
    const double z = (views[i]->r_wc.transpose() * rel).z();
    if (!(rel.dot(dirs[i]) > 0.0) || !(z > 0.0)) {
      return fail(F2Reason::kNegativeDepth);
    }
  }
  if (!(r.d_mv > 0.0)) {
    return fail(F2Reason::kNegativeDepth);
  }

  // 5. low_parallax: largest angle between two rays.
  r.theta_max = 0.0;
  for (size_t i = 0; i < dirs.size(); ++i) {
    for (size_t j = i + 1; j < dirs.size(); ++j) {
      r.theta_max = std::max(r.theta_max, AngleBetween(dirs[i], dirs[j]));
    }
  }
  if (r.theta_max < kF2MinParallaxRad - kF2Epsilon) {
    return fail(F2Reason::kLowParallax);
  }

  // 6. large_residual: largest angle between a ray and the direction to the triangulated point.
  r.rho_max = 0.0;
  for (size_t i = 0; i < dirs.size(); ++i) {
    r.rho_max = std::max(r.rho_max, AngleBetween(dirs[i], (r.x - centers[i]).normalized()));
  }
  if (r.rho_max > kF2MaxResidualRad + kF2Epsilon) {
    return fail(F2Reason::kLargeResidual);
  }

  // 7. plane_invalid: d_plane is the z-depth where the creator's observation ray meets the plane.
  const Eigen::Vector3d w0 = creator->r_wc * Eigen::Vector3d(creator->x, creator->y, 1.0);
  const double den = in.plane.n.dot(w0);
  const double height = in.plane.n.dot(creator->center) - in.plane.offset;
  if (den == 0.0 || !(height > 0.0)) {
    return fail(F2Reason::kPlaneInvalid);
  }
  r.d_plane = (in.plane.offset - in.plane.n.dot(creator->center)) / den;
  if (!(r.d_plane > 0.0) || !std::isfinite(r.d_plane)) {
    return fail(F2Reason::kPlaneInvalid);
  }

  // 8. depth_ratio_out_of_band: z-depths against each other, r = d_mv / d_plane.
  r.ratio = r.d_mv / r.d_plane;
  if (std::fabs(r.ratio - 1.0) > kF2RatioTolerance + kF2Epsilon) {
    r.status = F2Status::kContradicted;
    r.reason = F2Reason::kDepthRatioOutOfBand;
    return r;
  }
  r.status = F2Status::kConfirmed;
  r.reason = F2Reason::kConfirmed;
  return r;
}

// ---------------------------------------------------------------------------------------------
// WorldMap integration

F2Built F2BuildInput(const Impl& m, const WorldPoint& p) {
  F2Built b;
  b.input.plane.n = Eigen::Vector3d(0.0, 1.0, 0.0);
  b.input.plane.offset = p.f2.plane_y;
  b.input.creator_kf_id = p.f2.creator.id();
  b.input.creator_alive = !p.f2.creator_lost && KeyframeValid(m, p.f2.creator) &&
                          m.keyframes[p.f2.creator.index()].kf_serial == p.f2.creator_serial;
  uint64_t h = 1469598103934665603ull;
  for (const auto& [kf_id, obs_idx] : p.observations) {
    if (!KeyframeValid(m, kf_id)) {
      continue;
    }
    const Keyframe& kf = m.keyframes[kf_id.index()];
    if (obs_idx >= kf.frame.points().size()) {
      continue;
    }
    const std::array<float, 16>& d = kf.frame.extrinsic().data();  // world_from_camera, col-major
    F2View v;
    v.kf_id = kf_id.id();
    for (int c = 0; c < 3; ++c) {
      for (int r = 0; r < 3; ++r) {
        v.r_wc(r, c) = static_cast<double>(d[static_cast<size_t>(c * 4 + r)]);
      }
    }
    v.center = Eigen::Vector3d(d[12], d[13], d[14]);
    const c8::FramePoint& fp = kf.frame.points()[obs_idx];
    v.x = static_cast<double>(fp.x());
    v.y = static_cast<double>(fp.y());
    v.obs_idx = obs_idx;
    v.kf_serial = kf.kf_serial;
    v.pose_rev = kf.pose_rev;
    b.input.views.push_back(v);
    b.basis.push_back({v.kf_id, v.kf_serial, v.obs_idx, v.pose_rev});
    h = Fnv(h, v.kf_id);
    h = Fnv(h, v.kf_serial);
    h = Fnv(h, v.obs_idx);
    h = Fnv(h, v.pose_rev);
  }
  b.basis_hash = h;
  return b;
}

bool F2EvaluatePointNow(const WorldMap& wm, PointId id, F2EvalResult* out) {
  const WorldPoint* p = wm.FindPoint(id);
  if (p == nullptr) {
    return false;
  }
  *out = F2Evaluate(F2BuildInput(WorldMapAccess::impl(wm), *p).input);
  return true;
}

F2Status F2StateOf(const WorldMap& wm, const WorldPoint& p) {
  // Side-effect free: id_reuse is counted once, when F2Finalize meets the reused identifier.
  if (!p.f2.evaluated || p.f2.state == F2Status::kUnconfirmed) {
    return p.f2.state;
  }
  if (p.f2.point_serial_at_eval != p.point_serial) {
    return F2Status::kUnconfirmed;
  }
  const F2Built b = F2BuildInput(WorldMapAccess::impl(wm), p);
  if (b.basis_hash == p.f2.basis_hash && b.input.creator_alive == !p.f2.creator_lost) {
    return p.f2.state;
  }
  return F2Status::kUnconfirmed;  // changed since the evaluation: wait for the next finalize
}

void F2NoteChange(Impl& m, PointId id, const WorldPoint& p, F2Invalidation why) {
  if (!F2Enabled() || p.source != PointSource::kGround) {
    return;
  }
  if (m.f2_in_merge && why == kInvObsAdded) {
    return;  // MergePoint reports one 'merge' event for dst
  }
  m.f2_dirty.emplace(id, p.point_serial);
  ++MutableCounters().f2_invalidation[why];
}

void F2NoteMapReset(const WorldMap& old_map) {
  if (!F2Enabled()) {
    return;
  }
  const Impl& m = WorldMapAccess::impl(old_map);
  for (const WorldPoint& p : m.pools[kGroundPool].points) {
    if (p.state != PointState::kDead && p.f2.evaluated) {
      ++MutableCounters().f2_invalidation[kInvMapReset];
    }
  }
}

namespace {

void ProcessPoint(Impl& m, PointId id, WorldPoint& p, int64_t frame) {
  (void)id;
  F2PointState& f = p.f2;
  AccuracyCounters& c = MutableCounters();
  if (p.state == PointState::kDead || p.source != PointSource::kGround) {
    return;
  }
  if (f.creator_lost && f.evaluated) {
    return;  // no creator, the plane depth is undefined: never confirmed again (contract 6.3)
  }
  // The creating keyframe: still there, and the same object (a reused random id is not).
  bool reuse_event = false;  // one reuse event per point per finalize, however many keys show it
  if (!f.creator_lost && KeyframeValid(m, f.creator) &&
      m.keyframes[f.creator.index()].kf_serial != f.creator_serial) {
    reuse_event = true;
    f.creator_lost = true;
  }
  F2Built b = F2BuildInput(m, p);
  bool basis_reuse = false;
  for (const F2Basis& old : f.basis) {
    for (const F2Basis& cur : b.basis) {
      if (old.kf_id == cur.kf_id && old.kf_serial != cur.kf_serial) {
        basis_reuse = true;
      }
    }
  }
  if (reuse_event || basis_reuse) {
    ++c.f2_id_reuse;
  }
  const uint32_t tracker_epoch = m.serials->tracker_epoch;
  const uint32_t map_epoch = m.serials->map_epoch;
  const bool same = f.evaluated && !basis_reuse && f.basis_hash == b.basis_hash &&
                    f.tracker_epoch == tracker_epoch && f.map_epoch == map_epoch &&
                    f.point_serial_at_eval == p.point_serial && !f.creator_lost == b.input.creator_alive;
  if (same) {
    return;  // same basis: a contradiction (or confirmation) is not re-decided
  }
  const F2Status before = f.state;
  f.state = F2Status::kUnconfirmed;
  const F2EvalResult r = F2Evaluate(b.input);
  if (!b.input.creator_alive) {
    f.creator_lost = true;
  }
  f.state = r.status;
  f.last_reason = r.reason;
  f.no_creator = r.no_creator;
  f.evaluated = true;
  f.basis = std::move(b.basis);
  f.basis_hash = b.basis_hash;
  f.point_serial_at_eval = p.point_serial;
  f.tracker_epoch = tracker_epoch;
  f.map_epoch = map_epoch;
  f.eval_frame = frame;
  f.applied_frame = frame + 1;
  f.d_mv = r.d_mv;
  f.d_plane = r.d_plane;
  f.ratio = r.ratio;
  f.theta_max = r.theta_max;
  f.rho_max = r.rho_max;
  ++c.f2_eval[static_cast<size_t>(r.reason)];
  if (r.no_creator) {
    ++c.f2_no_creator;
  }
  if (r.status == F2Status::kConfirmed && before != F2Status::kConfirmed) {
    ++c.f2_newly_confirmed;
  }
}

}  // namespace

void F2Finalize(WorldMap& wm, const F2FinalizeArgs& args) {
  if (!F2Enabled()) {
    return;
  }
  Impl& m = WorldMapAccess::impl(wm);
  AccuracyCounters& c = MutableCounters();
  ++(args.kf_path ? c.f2_path_kf : c.f2_path_nonkf);
  g_applied_frame = args.frame + 1;

  // A tracker epoch change (LOST / relocalization / engine rebuild) puts every ground point back
  // to unconfirmed in this Update (contract 6.3); the evaluation waits while the tracker is held.
  if (m.serials->tracker_epoch != m.f2_last_tracker_epoch) {
    m.f2_last_tracker_epoch = m.serials->tracker_epoch;
    for (WorldPoint& p : m.pools[kGroundPool].points) {
      if (p.state != PointState::kDead && p.f2.evaluated) {
        p.f2.state = F2Status::kUnconfirmed;
        if (!p.f2.creator_lost) {
          p.f2.evaluated = false;
        }
        ++c.f2_invalidation[m.serials->epoch_reason];
      }
    }
    m.serials->epoch_reason = kInvTrackerEpoch;
    m.f2_all_dirty = true;
  }
  if (args.hold) {
    ++c.f2_hold_frames;
    c.f2_hold_points += m.f2_dirty.size() + (m.f2_all_dirty ? m.pools[kGroundPool].points.size() : 0);
    return;
  }

  const std::set<std::pair<PointId, uint64_t>> work = std::move(m.f2_dirty);
  m.f2_dirty.clear();
  for (const auto& [id, serial] : work) {
    WorldPoint* p = nullptr;
    if (id.index() < m.pools[kGroundPool].points.size() &&
        m.pools[kGroundPool].points[id.index()].id == id.id()) {
      p = &m.pools[kGroundPool].points[id.index()];
    }
    if (p == nullptr) {
      continue;  // slot holds another id now: not a reuse of this id
    }
    if (p->point_serial != serial) {
      ++c.f2_id_reuse;  // same (slot, id), a different point
      continue;
    }
    ProcessPoint(m, id, *p, args.frame);
  }
  if (m.f2_all_dirty) {
    m.f2_all_dirty = false;
    std::vector<WorldPoint>& pts = m.pools[kGroundPool].points;
    for (size_t slot = 0; slot < pts.size(); ++slot) {
      ProcessPoint(m, PointId(slot, pts[slot].id), pts[slot], args.frame);
    }
  }
  c.f2_dirty_remainder += m.f2_dirty.size();
}

void F2BumpTrackerEpoch(WorldMap& wm) {
  if (!F2Enabled()) {
    return;
  }
  ++WorldMapAccess::impl(wm).serials->tracker_epoch;
}

void F2NoteBuilderStart(WorldMap& wm) {
  if (!F2Enabled()) {
    return;
  }
  Impl& m = WorldMapAccess::impl(wm);
  for (const WorldPoint& p : m.pools[kGroundPool].points) {
    if (p.state != PointState::kDead) {
      ++m.serials->tracker_epoch;
      m.serials->epoch_reason = kInvRebuild;
      return;
    }
  }
}

int64_t F2AppliedFrame() { return g_applied_frame; }
void F2TrackBegin() { g_track_bonus = 0; }
void F2NoteBonusGranted() { ++g_track_bonus; }
void F2TrackEnd(bool state_changed) {
  if (state_changed && g_track_bonus > 0) {
    ++MutableCounters().used_state_before_epoch;
  }
}

}  // namespace slam_refact::map
