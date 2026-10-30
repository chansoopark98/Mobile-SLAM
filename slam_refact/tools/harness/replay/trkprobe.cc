// See trkprobe.h. Harness-only (US-026 root-cause A-3, REPORT tier).
#include "trkprobe.h"

#include <cfloat>
#include <cmath>
#include <cstdlib>
#include <cstdarg>
#include <cstdint>
#include <cstring>
#include <random>
#include <string>
#include <type_traits>
#include <vector>

#include "common/tracking_types.h"
#include "frame_frame_tracker/frame_frame_tracker_internal.h"
#include "map_builder/map_builder.h"
#include "map_tracker/map_tracker_internal.h"
#include "metric_scale/metric_scale_estimator_internal.h"
#include "vio_tracker/vio_tracker_internal.h"

// ---- private-member access without touching engine/src (explicit instantiation ignores access) ----------
namespace {
template <typename Tag, typename Tag::type M>
struct Rob {
  friend typename Tag::type Get(Tag) { return M; }
};
struct MbStateTag {
  using type = int32_t slam_refact::map_builder::MapBuilder::*;
  friend type Get(MbStateTag);
};
struct MbFrameCountTag {
  using type = int64_t slam_refact::map_builder::MapBuilder::*;
  friend type Get(MbFrameCountTag);
};
struct MbLastChangeTag {
  using type = int64_t slam_refact::map_builder::MapBuilder::*;
  friend type Get(MbLastChangeTag);
};
}  // namespace
template struct Rob<MbStateTag, &slam_refact::map_builder::MapBuilder::state_>;
template struct Rob<MbFrameCountTag, &slam_refact::map_builder::MapBuilder::frame_count_>;
template struct Rob<MbLastChangeTag, &slam_refact::map_builder::MapBuilder::last_state_change_frame_>;

namespace slam_refact::harness::trkprobe {
namespace {

using slam_refact::frame_frame_tracker::FrameFrameTracker;
using slam_refact::frame_frame_tracker::FrameFrameTrackerAccess;
using slam_refact::map_builder::MapBuilder;
using slam_refact::map_tracker::MapTracker;
using slam_refact::map_tracker::MapTrackerAccess;
using slam_refact::vio_tracker::VioTracker;
using slam_refact::vio_tracker::VioTrackerAccess;

VioTracker* g_vt = nullptr;
MapTracker* g_mt = nullptr;
MapBuilder* g_mb = nullptr;

std::string g_stages;       // comma separated JSON objects, chronological
int g_mt_calls = 0;         // MapTracker::Track calls since the last EmitFrame
int64_t g_fft_rng_total = 0;  // mt19937 draws of the FFT generator, summed over measured calls
int64_t g_gl_rng_total = 0;   // mt19937 draws of the engine's global generator (shared by MapTracker / Relocalize)
bool g_rng_unmeasured = false;

__attribute__((format(printf, 1, 2))) std::string Fmt(const char* fmt, ...) {
  va_list ap, ap2;
  va_start(ap, fmt);
  va_copy(ap2, ap);
  const int n = std::vsnprintf(nullptr, 0, fmt, ap);
  va_end(ap);
  std::string out(static_cast<size_t>(n > 0 ? n : 0) + 1, '\0');
  std::vsnprintf(&out[0], out.size(), fmt, ap2);
  va_end(ap2);
  out.resize(static_cast<size_t>(n > 0 ? n : 0));
  return out;
}

std::string Num(double v) {
  if (!std::isfinite(v)) return "null";
  return Fmt("%.9g", v);
}

struct Fnv {
  uint64_t h = 0xcbf29ce484222325ull;
  void Bytes(const void* p, size_t n) {
    const unsigned char* b = static_cast<const unsigned char*>(p);
    for (size_t i = 0; i < n; ++i) h = (h ^ b[i]) * 0x100000001b3ull;
  }
  std::string Hex() const { return Fmt("%016llx", static_cast<unsigned long long>(h)); }
};

std::string HashFloats(const float* p, size_t n) {
  Fnv f;
  f.Bytes(p, n * sizeof(float));
  return f.Hex();
}

std::string PoseStr(const c8::HMatrix& m) {
  const std::array<float, 16>& d = m.data();
  return Fmt("{\"t\":[%s,%s,%s],\"h\":\"%s\"}", Num(d[12]).c_str(), Num(d[13]).c_str(), Num(d[14]).c_str(),
             HashFloats(d.data(), 16).c_str());
}

std::string QuatStr(const c8::Quaternion& q) {
  return Fmt("[%s,%s,%s,%s]", Num(q.w()).c_str(), Num(q.x()).c_str(), Num(q.y()).c_str(), Num(q.z()).c_str());
}

// ---- RNG draw accounting: c8::RandomNumbers keeps a std::mt19937 as its first member; the number of raw draws
// between two states is the number of steps that turn the first copy into the second. -------------------------
static_assert(std::is_trivially_copyable<std::mt19937>::value, "mt19937 must be copyable bytewise");
struct Snap {
  std::mt19937 m;
  explicit Snap(const void* rng) { std::memcpy(&m, rng, sizeof(m)); }
};
std::string RngHash(const Snap& s) {
  Fnv f;
  f.Bytes(&s.m, sizeof(s.m));
  return f.Hex();
}
// -1 when the later state is not reachable within the cap.
int64_t Draws(const Snap& before, const Snap& after) {
  std::mt19937 a = before.m;
  constexpr int64_t kCap = 400000;
  for (int64_t n = 0; n <= kCap; ++n) {
    if (std::memcmp(&a, &after.m, sizeof(a)) == 0) return n;
    a();
  }
  g_rng_unmeasured = true;
  return -1;
}

// TRKPROBE_STALE_DEPTHS=1: the historical what-if described in trkprobe.h (not an engine-equivalent replay).
bool WhatIfStaleDepths() {
  static const bool on = [] {
    const char* e = std::getenv("TRKPROBE_STALE_DEPTHS");
    return e != nullptr && std::string(e) == "1";
  }();
  return on;
}

void Add(const std::string& obj) {
  if (!g_stages.empty()) g_stages += ",";
  g_stages += obj;
}

const compat::RandomNumbers& FftRng(const FrameFrameTracker& t) { return FrameFrameTrackerAccess::impl(t).rng; }

std::string FftStateStr(const FrameFrameTracker& t) {
  const auto& ft = FrameFrameTrackerAccess::impl(t);
  return Fmt(
      "{\"matches\":%zu,\"ground\":%zu,\"sky\":%zu,\"depthMatches\":%zu,\"prevDepths\":%zu,\"prevRays\":%zu,"
      "\"lastOk\":%d,\"lastT\":%s,\"lastRot\":%s,\"height\":%s,\"calls\":%d}",
      ft.matches.size(), ft.ground_matches.size(), ft.sky_matches.size(), ft.depth_matches.size(),
      ft.prev_depths.size(), ft.prev_rays.size(), ft.last_success ? 1 : 0, Num(ft.last_translation).c_str(),
      Num(ft.last_rotation).c_str(), Num(ft.height).c_str(), ft.call_count);
}

}  // namespace
}  // namespace slam_refact::harness::trkprobe

// ---- linker --wrap capture points ----------------------------------------------------------------------------
namespace tp = slam_refact::harness::trkprobe;
using slam_refact::TrackerInput;

extern "C" {

// VioTracker(bool, bool): the engine's one tracker (root of MetricScale, mt_out, mb_out, input).
void __real__ZN11slam_refact11vio_tracker10VioTrackerC1Ebb(void*, bool, bool);
void __wrap__ZN11slam_refact11vio_tracker10VioTrackerC1Ebb(void* self, bool a, bool b) {
  __real__ZN11slam_refact11vio_tracker10VioTrackerC1Ebb(self, a, b);
  tp::g_vt = static_cast<tp::VioTracker*>(self);
  tp::g_mt = nullptr;
  tp::g_mb = nullptr;
}
void __real__ZN11slam_refact11vio_tracker10VioTrackerD1Ev(void*);
void __wrap__ZN11slam_refact11vio_tracker10VioTrackerD1Ev(void* self) {
  if (tp::g_vt == self) {
    tp::g_vt = nullptr;
    tp::g_mt = nullptr;  // owned by the dying tracker's modules
    tp::g_mb = nullptr;
  }
  __real__ZN11slam_refact11vio_tracker10VioTrackerD1Ev(self);
}
void __real__ZN11slam_refact11vio_tracker10VioTrackerD2Ev(void*);
void __wrap__ZN11slam_refact11vio_tracker10VioTrackerD2Ev(void* self) {
  if (tp::g_vt == self) {
    tp::g_vt = nullptr;
    tp::g_mt = nullptr;  // owned by the dying tracker's modules
    tp::g_mb = nullptr;
  }
  __real__ZN11slam_refact11vio_tracker10VioTrackerD2Ev(self);
}

// MapTracker::Track(WorldMap&, TrackerInput*, const FrameState&, MapTrackerOutput*, RandomNumbers*)
void __real__ZN11slam_refact11map_tracker10MapTracker5TrackERNS_3map8WorldMapEPNS_12TrackerInputERKNS_10FrameStateEPNS_16MapTrackerOutputEPN2c813RandomNumbersE(
    void*, void*, void*, const void*, void*, void*);
void __wrap__ZN11slam_refact11map_tracker10MapTracker5TrackERNS_3map8WorldMapEPNS_12TrackerInputERKNS_10FrameStateEPNS_16MapTrackerOutputEPN2c813RandomNumbersE(
    void* self, void* map, void* frame, const void* predicted, void* out, void* rng) {
  using namespace tp;
  auto* mt = static_cast<MapTracker*>(self);
  g_mt = mt;
  ++g_mt_calls;
  const auto& im = MapTrackerAccess::impl(*mt);
  const auto* ti = static_cast<const TrackerInput*>(frame);
  const auto* pred = static_cast<const slam_refact::FrameState*>(predicted);
  const Snap gl0(rng);
  const Snap ff0(&FftRng(im.frame_tracker));
  const std::string pre = Fmt(
      "\"pose0\":%s,\"state0\":%d,\"frameCount0\":%lld,\"lastChange0\":%lld,\"consecOk0\":%lld,\"nPts\":%zu,"
      "\"pred\":%s,\"predQ\":%s,\"nEvents\":%zu,\"fft0\":%s,\"glRng0\":\"%s\",\"fftRng0\":\"%s\"",
      PoseStr(im.pose).c_str(), im.state, static_cast<long long>(im.frame_count),
      static_cast<long long>(im.last_state_change_frame), static_cast<long long>(im.consecutive_frame_tracker_ok),
      ti->frame_points.points().size(), PoseStr(pred->pose).c_str(), QuatStr(pred->xr_device_rotation).c_str(),
      pred->sensor_events.size(), FftStateStr(im.frame_tracker).c_str(), RngHash(gl0).c_str(),
      RngHash(ff0).c_str());
  __real__ZN11slam_refact11map_tracker10MapTracker5TrackERNS_3map8WorldMapEPNS_12TrackerInputERKNS_10FrameStateEPNS_16MapTrackerOutputEPN2c813RandomNumbersE(
      self, map, frame, predicted, out, rng);
  const Snap gl1(rng);
  const Snap ff1(&FftRng(im.frame_tracker));
  const int64_t dgl = Draws(gl0, gl1);
  const int64_t dff = Draws(ff0, ff1);
  g_gl_rng_total += dgl > 0 ? dgl : 0;
  g_fft_rng_total += dff > 0 ? dff : 0;
  const auto* o = static_cast<const slam_refact::MapTrackerOutput*>(out);
  Add(Fmt("{\"s\":\"mapTracker\",%s,\"pose1\":%s,\"state1\":%d,\"frameTrackerOk\":%d,\"delta\":%s,"
          "\"nGround\":%zu,\"nSky\":%zu,\"nDepth\":%zu,\"nVisible\":%zu,\"nTracked\":%zu,\"trackedHash\":\"%s\","
          "\"anchor\":%s,\"groundHeight\":%s,\"groundY\":%s,\"lastGoodResidual\":%s,\"lostResidual\":%s,"
          "\"fft1\":%s,\"glDraws\":%lld,\"fftDraws\":%lld,\"glRng1\":\"%s\",\"fftRng1\":\"%s\"}",
          pre.c_str(), PoseStr(im.pose).c_str(), im.state, im.frame_tracker_ok ? 1 : 0,
          PoseStr(im.frame_delta).c_str(), o->ground_idx.size(), o->sky_idx.size(), o->depth_idx.size(),
          o->visible_ids.size(), o->tracked.size(),
          [&] {
            Fnv f;
            for (const auto& kv : o->tracked) {
              const uint32_t k = kv.first, idx = kv.second.index(), id = kv.second.id();
              f.Bytes(&k, 4);
              f.Bytes(&idx, 4);
              f.Bytes(&id, 4);
            }
            return f.Hex();
          }()
              .c_str(),
          PoseStr(im.anchor_pose).c_str(), Num(im.ground_height).c_str(), Num(im.ground_y).c_str(),
          Num(im.last_good_residual).c_str(), Num(im.lost_residual).c_str(),
          FftStateStr(im.frame_tracker).c_str(), static_cast<long long>(dgl), static_cast<long long>(dff),
          RngHash(gl1).c_str(), RngHash(ff1).c_str()));
}

// FrameFrameTracker::Track(HMatrix*, const FrameWithPoints&, const Quaternion&, vector<u32>* x3, bool*)
void __real__ZN11slam_refact19frame_frame_tracker17FrameFrameTracker5TrackEPN2c87HMatrixERKNS2_15FrameWithPointsERKNS2_10QuaternionEPSt6vectorIjSaIjEESE_SE_Pb(
    void*, void*, const void*, const void*, void*, void*, void*, bool*);
void __wrap__ZN11slam_refact19frame_frame_tracker17FrameFrameTracker5TrackEPN2c87HMatrixERKNS2_15FrameWithPointsERKNS2_10QuaternionEPSt6vectorIjSaIjEESE_SE_Pb(
    void* self, void* delta, const void* cur, const void* q, void* g, void* s, void* d, bool* success) {
  using namespace tp;
  auto* t = static_cast<FrameFrameTracker*>(self);
  const auto* fp = static_cast<const c8::FrameWithPoints*>(cur);
  const auto* qq = static_cast<const c8::Quaternion*>(q);
  auto& fi = FrameFrameTrackerAccess::impl(*t);
  const std::string st0 = FftStateStr(*t);
  const Snap r0(&FftRng(*t));
  // Stale-depth analysis (A-3, read-only counters): prev_depths can be shorter than the previous frame
  // (SetPrevDepths only runs on a tracked frame). The original reads depths[dictionaryIdx] unchecked past size();
  // inside capacity() that is the stale content of an earlier, longer assignment (clear()/shrinking assign never
  // overwrite it). Since US026-CC-1 the engine models exactly this itself (PrevDepthList), so these counters
  // describe reads the engine performs in a default replay.
  const size_t d0 = fi.prev_depths.size();
  const size_t cap0 = fi.prev_depths.capacity();
  const size_t pr0 = fi.prev_rays.size();
  std::vector<float> stale;  // raw floats at [d0, min(cap0, pr0))
  for (size_t k = d0; k < cap0 && k < pr0; ++k) stale.push_back(fi.prev_depths.data()[k]);
  bool whatif = false;
  // Opt-in experiment, off by default (see trkprobe.h). Historical since US026-CC-1: the engine already reads the
  // stale tail, and this assignment can reallocate the modeled storage (PrevDepthList::Assign).
  if (WhatIfStaleDepths() && d0 > 0 && d0 < pr0) {
    std::vector<float> ext(pr0, -FLT_MAX);
    for (size_t k = 0; k < d0; ++k) ext[k] = fi.prev_depths[k];
    for (size_t k = 0; k < stale.size(); ++k) ext[d0 + k] = stale[k];
    fi.prev_depths = ext;
    whatif = true;
  }
  __real__ZN11slam_refact19frame_frame_tracker17FrameFrameTracker5TrackEPN2c87HMatrixERKNS2_15FrameWithPointsERKNS2_10QuaternionEPSt6vectorIjSaIjEESE_SE_Pb(
      self, delta, cur, q, g, s, d, success);
  const Snap r1(&FftRng(*t));
  const int64_t dd = Draws(r0, r1);
  size_t oob = 0, in_cap = 0, in_cap_pos = 0, beyond = 0;
  if (d0 > 0) {
    for (const c8::PointMatch& m : fi.matches) {
      if (m.dictionaryIdx < d0) continue;
      ++oob;
      if (m.dictionaryIdx - d0 < stale.size()) {
        ++in_cap;
        in_cap_pos += stale[m.dictionaryIdx - d0] > 0.0f ? 1 : 0;
      } else if (m.dictionaryIdx >= cap0) {
        ++beyond;
      }
    }
  }
  Add(Fmt("{\"s\":\"fft\",\"nCur\":%zu,\"qCur\":%s,\"state0\":%s,\"success\":%d,\"delta\":%s,\"state1\":%s,"
          "\"rngDraws\":%lld,\"stale\":{\"d0\":%zu,\"cap0\":%zu,\"prevRays0\":%zu,\"oobMatches\":%zu,\"oobInCap\":%zu,"
          "\"oobInCapPositive\":%zu,\"oobBeyondCap\":%zu,\"whatIf\":%d}}",
          fp->points().size(), QuatStr(*qq).c_str(), st0.c_str(), *success ? 1 : 0,
          PoseStr(*static_cast<const c8::HMatrix*>(delta)).c_str(), FftStateStr(*t).c_str(),
          static_cast<long long>(dd), d0, cap0, pr0, oob, in_cap, in_cap_pos, beyond, whatif ? 1 : 0));
}

// RobustGroundHomography(q_prev, prev_rays, q_cur, cur_rays, scratch, rng, h, cam_motion, mask) -> bool
bool __real__ZN11slam_refact19frame_frame_tracker22RobustGroundHomographyERKN2c810QuaternionERKSt6vectorINS1_6HPointILm2EEESaIS7_EES4_SB_PNS1_28RobustHomographyScratchSpaceEPNS1_13RandomNumbersEPNS1_7HMatrixESH_PS5_IhSaIhEE(
    const void*, const void*, const void*, const void*, void*, void*, void*, void*, void*);
bool __wrap__ZN11slam_refact19frame_frame_tracker22RobustGroundHomographyERKN2c810QuaternionERKSt6vectorINS1_6HPointILm2EEESaIS7_EES4_SB_PNS1_28RobustHomographyScratchSpaceEPNS1_13RandomNumbersEPNS1_7HMatrixESH_PS5_IhSaIhEE(
    const void* qp, const void* pr, const void* qc, const void* cr, void* scratch, void* rng, void* h, void* cam,
    void* mask) {
  using namespace tp;
  const auto* prev = static_cast<const std::vector<c8::HPoint2>*>(pr);
  const auto* cur = static_cast<const std::vector<c8::HPoint2>*>(cr);
  const Snap r0(rng);
  Fnv in;  // fingerprint of the ray inputs (prev then cur)
  for (const c8::HPoint2& p : *prev) in.Bytes(p.raw().data(), sizeof(float) * p.raw().size());
  for (const c8::HPoint2& p : *cur) in.Bytes(p.raw().data(), sizeof(float) * p.raw().size());
  const bool ok = __real__ZN11slam_refact19frame_frame_tracker22RobustGroundHomographyERKN2c810QuaternionERKSt6vectorINS1_6HPointILm2EEESaIS7_EES4_SB_PNS1_28RobustHomographyScratchSpaceEPNS1_13RandomNumbersEPNS1_7HMatrixESH_PS5_IhSaIhEE(
      qp, pr, qc, cr, scratch, rng, h, cam, mask);
  const Snap r1(rng);
  const int64_t dd = Draws(r0, r1);
  const auto* sc = static_cast<const c8::RobustHomographyScratchSpace*>(scratch);
  Fnv pairs;
  size_t npairs = 0;
  for (const auto& pr2 : sc->hSampleInds) {
    pairs.Bytes(&pr2.first, sizeof(pr2.first));
    pairs.Bytes(&pr2.second, sizeof(pr2.second));
    ++npairs;
  }
  size_t inl = 0;
  const auto* m = static_cast<const std::vector<uint8_t>*>(mask);
  if (m != nullptr) {
    for (uint8_t v : *m) inl += v != 0;
  }
  Add(Fmt("{\"s\":\"groundHomography\",\"n\":%zu,\"inRays\":\"%s\",\"qPrev\":%s,\"qCur\":%s,\"ok\":%d,"
          "\"inliers\":%zu,\"nPairs\":%zu,\"pairsHash\":\"%s\",\"nSamples\":%zu,\"h\":%s,\"cam\":%s,\"rngDraws\":%lld,"
          "\"rng0\":\"%s\",\"rng1\":\"%s\"}",
          prev->size(), in.Hex().c_str(), QuatStr(*static_cast<const c8::Quaternion*>(qp)).c_str(),
          QuatStr(*static_cast<const c8::Quaternion*>(qc)).c_str(), ok ? 1 : 0, inl, npairs, pairs.Hex().c_str(),
          sc->aPtsSample.size(), PoseStr(*static_cast<const c8::HMatrix*>(h)).c_str(),
          PoseStr(*static_cast<const c8::HMatrix*>(cam)).c_str(), static_cast<long long>(dd), RngHash(r0).c_str(),
          RngHash(r1).c_str()));
  return ok;
}

// TrackPoints(const WorldMap&, const TrackerInput&, const HMatrix& pose, const vector<PointId>& ids, MapTrackResult*)
void __real__ZN11slam_refact11map_tracker11TrackPointsERKNS_3map8WorldMapERKNS_12TrackerInputERKN2c87HMatrixERKSt6vectorINS8_15WorldMapPointIdESaISD_EEPNS0_14MapTrackResultE(
    const void*, const void*, const void*, const void*, void*);
void __wrap__ZN11slam_refact11map_tracker11TrackPointsERKNS_3map8WorldMapERKNS_12TrackerInputERKN2c87HMatrixERKSt6vectorINS8_15WorldMapPointIdESaISD_EEPNS0_14MapTrackResultE(
    const void* map, const void* frame, const void* pose, const void* ids, void* result) {
  using namespace tp;
  const auto* idv = static_cast<const std::vector<c8::WorldMapPointId>*>(ids);
  const std::string pose0 = PoseStr(*static_cast<const c8::HMatrix*>(pose));
  const size_t nids = idv->size();
  __real__ZN11slam_refact11map_tracker11TrackPointsERKNS_3map8WorldMapERKNS_12TrackerInputERKN2c87HMatrixERKSt6vectorINS8_15WorldMapPointIdESaISD_EEPNS0_14MapTrackResultE(
      map, frame, pose, ids, result);
  const auto* r = static_cast<const slam_refact::map_tracker::MapTrackResult*>(result);
  Add(Fmt("{\"s\":\"trackPoints\",\"nIds\":%zu,\"pose0\":%s,\"residual\":%s,\"pose1\":%s,\"nLoaded\":%u,"
          "\"nAllObs\":%zu,\"nAllMatches\":%zu,\"nSubsetObs\":%zu,\"nSubsetMatches\":%zu,\"nMatchedIds\":%zu}",
          nids, pose0.c_str(), Num(r->residual).c_str(), PoseStr(r->pose).c_str(), r->num_loaded,
          r->all_obs.size(), r->all_matches.size(), r->subset_obs.size(), r->subset_matches.size(),
          r->matched_ids.size()));
}

// Relocalize(RelocResult*, const WorldMap&, const TrackerInput&, const HMatrix&, RandomNumbers*, RobustP2PScratchSpace*)
void __real__ZN11slam_refact11map_tracker10RelocalizeEPNS_11RelocResultERKNS_3map8WorldMapERKNS_12TrackerInputERKN2c87HMatrixEPNSA_13RandomNumbersEPNSA_21RobustP2PScratchSpaceE(
    void*, const void*, const void*, const void*, void*, void*);
void __wrap__ZN11slam_refact11map_tracker10RelocalizeEPNS_11RelocResultERKNS_3map8WorldMapERKNS_12TrackerInputERKN2c87HMatrixEPNSA_13RandomNumbersEPNSA_21RobustP2PScratchSpaceE(
    void* out, const void* map, const void* frame, const void* pose, void* rng, void* scratch) {
  using namespace tp;
  const Snap r0(rng);
  const std::string pose0 = PoseStr(*static_cast<const c8::HMatrix*>(pose));
  __real__ZN11slam_refact11map_tracker10RelocalizeEPNS_11RelocResultERKNS_3map8WorldMapERKNS_12TrackerInputERKN2c87HMatrixEPNSA_13RandomNumbersEPNSA_21RobustP2PScratchSpaceE(
      out, map, frame, pose, rng, scratch);
  const Snap r1(rng);
  const auto* r = static_cast<const slam_refact::RelocResult*>(out);
  Add(Fmt("{\"s\":\"relocalize\",\"pose0\":%s,\"success\":%d,\"pose1\":%s,\"confidence\":%s,\"nInliers\":%zu,"
          "\"rngDraws\":%lld}",
          pose0.c_str(), r->success ? 1 : 0, PoseStr(r->pose).c_str(), Num(r->confidence).c_str(),
          r->inlier_ids.size(), static_cast<long long>(Draws(r0, r1))));
}

// MapBuilder::Update(WorldMaps*, const TrackerInput&, const MapTrackerOutput&, int64_t, MapBuilderOutput*)
void __real__ZN11slam_refact11map_builder10MapBuilder6UpdateEPNS_3map9WorldMapsERKNS_12TrackerInputERKNS_16MapTrackerOutputElPNS_16MapBuilderOutputE(
    void*, void*, const void*, const void*, int64_t, void*);
void __wrap__ZN11slam_refact11map_builder10MapBuilder6UpdateEPNS_3map9WorldMapsERKNS_12TrackerInputERKNS_16MapTrackerOutputElPNS_16MapBuilderOutputE(
    void* self, void* ctx, const void* frm, const void* trk, int64_t frame_ms, void* out) {
  using namespace tp;
  auto* mb = static_cast<MapBuilder*>(self);
  g_mb = mb;
  const int s0 = mb->*Get(MbStateTag());
  const long long fc0 = mb->*Get(MbFrameCountTag());
  __real__ZN11slam_refact11map_builder10MapBuilder6UpdateEPNS_3map9WorldMapsERKNS_12TrackerInputERKNS_16MapTrackerOutputElPNS_16MapBuilderOutputE(
      self, ctx, frm, trk, frame_ms, out);
  const auto* o = static_cast<const slam_refact::MapBuilderOutput*>(out);
  Add(Fmt("{\"s\":\"mapBuilder\",\"state0\":%d,\"frameCount0\":%lld,\"state1\":%d,\"frameCount1\":%lld,"
          "\"lastChange1\":%lld,\"mapReset\":%d,\"keyframeCreated\":%d,\"nNewPoints\":%zu,\"worldTransformUpdated\":%d}",
          s0, fc0, static_cast<int>(mb->*Get(MbStateTag())), static_cast<long long>(mb->*Get(MbFrameCountTag())),
          static_cast<long long>(mb->*Get(MbLastChangeTag())), o->map_reset ? 1 : 0, o->keyframe_created ? 1 : 0,
          o->new_points.size(), o->world_transform_updated ? 1 : 0));
}

}  // extern "C"

namespace slam_refact::harness::trkprobe {
namespace {

std::string Hex(const Eigen::VectorXf& v) { return HashFloats(v.data(), static_cast<size_t>(v.size())); }
std::string Hex(const Eigen::MatrixXf& m) { return HashFloats(m.data(), static_cast<size_t>(m.size())); }

}  // namespace

void ResetFrame() {
  g_stages.clear();
  g_mt_calls = 0;
}

void EmitFrame(std::FILE* f, int frame_index) {
  std::string line = Fmt("{\"i\":%d", frame_index);
  if (g_vt == nullptr) {
    line += ",\"vt\":false";
  } else {
    auto& s = VioTrackerAccess::state(*g_vt);
    const auto& o = s.mt_out;
    line += Fmt(",\"trkState\":%d,\"trkFrameTrackerOk\":%d,\"trkConsecutiveOk\":%lld,\"trkCandidates\":%zu,"
                "\"trkSky\":%zu",
                o.state, o.frame_tracker_ok ? 1 : 0, static_cast<long long>(o.consecutive_frame_tracker_ok),
                o.ground_idx.size(), o.sky_idx.size());
    // frmWorldMatches: u32 at TrackerInput+0x98 = FrameWithPoints worldPointIndices().size() (engine
    // map_builder.cc:450 names the same quantity world_matches).
    line += s.input ? Fmt(",\"frmWorldMatches\":%zu", s.input->frame_points.worldPointIndices().size())
                    : std::string(",\"frmWorldMatches\":null");
    line += g_mb ? Fmt(",\"mbState\":%d,\"mbFrameCount\":%lld", static_cast<int>(g_mb->*Get(MbStateTag())),
                       static_cast<long long>(g_mb->*Get(MbFrameCountTag())))
                 : std::string(",\"mbState\":null");
    if (g_mt != nullptr) {
      const auto& ft = FrameFrameTrackerAccess::impl(MapTrackerAccess::impl(*g_mt).frame_tracker);
      uint32_t hb;
      std::memcpy(&hb, &ft.height, 4);
      line += Fmt(",\"fftAll\":%zu,\"fftGround\":%zu,\"fftSky\":%zu,\"fftLastOk\":%d,\"fftHeight\":%.6f,"
                  "\"fftHeightBits\":\"%08x\",\"fftCalls\":%d",
                  ft.matches.size(), ft.ground_matches.size(), ft.sky_matches.size(), ft.last_success ? 1 : 0,
                  static_cast<double>(ft.height), hb, ft.call_count);
    }
    // MetricScaleEstimator (poke fields): hashes as config-poke.js / mse_precision_golden.mjs:
    // mseKfHash = fnv(x[9]) + fnv(P[81]) of the position Kalman filter, mseImuHash = fnv(x[6]) + fnv(P[36]).
    const auto& mse = metric_scale::MetricScaleEstimatorAccess::state(s.mse);
    line += Fmt(",\"mseScale\":%s,\"mseResidual\":%s,\"mseSum\":%s,\"mseHave\":%d,\"mseActive\":%d,"
                "\"mseKfHash\":\"%s%s\",\"mseImuHash\":\"%s%s\",\"vtLatch\":%d,\"vtLatchedScale\":%s",
                Num(mse.scale).c_str(), Num(mse.fit_residual).c_str(), Num(mse.acc_sum).c_str(),
                mse.have_estimate ? 1 : 0, mse.active ? 1 : 0, Hex(mse.kf.x).c_str(), Hex(mse.kf.p).c_str(),
                Hex(mse.imu.x).c_str(), Hex(mse.imu.p).c_str(), s.scale_latched ? 1 : 0,
                Num(s.latched_scale).c_str());
  }
  line += Fmt(",\"mtCalls\":%d,\"fftRngTotal\":%lld,\"glRngTotal\":%lld,\"rngUnmeasured\":%d,\"stages\":[%s]}\n",
              g_mt_calls, static_cast<long long>(g_fft_rng_total), static_cast<long long>(g_gl_rng_total),
              g_rng_unmeasured ? 1 : 0, g_stages.c_str());
  std::fputs(line.c_str(), f);
  ResetFrame();
}

}  // namespace slam_refact::harness::trkprobe
