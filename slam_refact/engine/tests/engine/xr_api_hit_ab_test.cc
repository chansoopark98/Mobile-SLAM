// XrController hitTest A/B oracle (contract FULL-7, RM-3b, review M2; preregistered in
// build/slam-refact-full-20261019/preregistration/rm3b-hit-ab-gate.v2.md, which supersedes v1).
//
// v2 (RM-3b2, pre-run review V2-1..V2-5): the comparator fails closed on non-finite values; tier T no
// longer injects the camera position as a point for original hits at distance 0 (0/0 -> NaN) but a point
// whose weight underflows on every ray; the synthetic recording covers zero-depth, padding-only,
// zero-world-point and LIMITED samples; mutants M11 (handedness toggle dropped) and M12 (non-finite
// output) were added; the held gate checks its own integrity (sha256 of the TSV, the gate file, this
// binary and the reviewer designation) and re-runs the original-side properties O-1..O-6 first.
//
// Why a second hit oracle: FEATURE_POINT is invariant under a similarity transform of camera and
// points, so a single-configuration recording cannot tell a wrong origin / facing / scale model
// from the right one when the test rebuilds the internal state with the same model it checks, and
// the mirror toggle cannot be told from "no mirror" at a centred principal point. The recording
// (api-oracle.js option "ab") runs hitTest -> updateCameraProjectionMatrix / configure -> hitTest
// ... inside ONE onUpdate of the original SDK, so every step sees the same internal state and the
// steps differ only by the coordinate system (origin x z, scale, yaw, pitch, roll, mirror, left
// handed, all together, and back).
//
// Replay (exact input). The published pose and world points of the callback (base configuration)
// are mapped back to the internal frame and injected into a GlueRunner through a scripted
// TrackerCore; the recorded call sequence is applied (GlueRunner::UpdateCameraProjectionMatrix /
// Configure) and GlueRunner::XrHitTest answers the recorded queries. Errors of the base-configuration
// model that are common to all steps cancel in the A/B relation by construction; errors that differ
// between configurations (origin, facing, scale, handedness, mirror) do not.
//   tier T (tight)       for every query one point is injected: the internal hit that the recording
//                        implies (the base-configuration hit of the same screen point, or of its mirror
//                        partner while the mirror is on, mapped back with the base chain). FEATURE_POINT
//                        then returns exactly that point, so the engine answer is the coordinate chain
//                        of the glue (bookkeeping of origin / facing / scale / handedness / mirror) and
//                        nothing else; float32 noise only.
//   tier E (end to end)  the full feature set of the callback is injected and the recorded hits are
//                        recomputed by the weighted FEATURE_POINT estimate. This is ill conditioned
//                        (sigma 0.02 weights amplify float32 noise of the reconstructed state by
//                        up to ~1e3), hence its looser tolerance.
//
// Test groups
//   HitAbNull / HitAbMutation  instrument checks on a SYNTHETIC recording made by the engine itself
//                              (no original data, no verdict): resolution (round trip noise),
//                              comparator self identity, determinism, and detection of model errors
//   HitAbOriginal              properties of the ORIGINAL recording itself (no engine): the A/B steps
//                              that must coincide do (original-side null-run), pitch / roll are ignored
//   HitAbIntegrity             the gate's own checks (recording sha256 and class counts, SHA-256 known answers,
//                              the order of the release checks on synthetic files), no engine
//   HitAbHeld                  the gate: engine replay against the original recording. Compiled only
//                              into sr_tests_xr_api_hit_ab_held (label held) and fails closed until StartRelease
//                              passes (shared device sr_held; SLAM_REFACT_RM3B_REVIEWER = sha256 of the registered
//                              reviewer designation file, SLAM_REFACT_RM3B_GATE = <gate_id>.v<N>, and the rest of its
//                              comment; SLAM_REFACT_RM3B_PREREG_DIR optionally replaces the default directory).

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <sys/stat.h>
#include <unistd.h>

#include "c8/geometry/egomotion.h"
#include "c8/geometry/homography.h"
#include "c8/hmatrix.h"
#include "c8/hpoint.h"
#include "c8/quaternion.h"
#include "engine/hit_test.h"
#include "slam_refact/engine_config.h"
#include "slam_refact/glue_runner.h"
#include "slam_refact/xr_controller.h"
#include "../support/sr_held.h"

namespace slam_refact {
namespace {

const std::string kOracleAbDir = SR_FULL_ORACLE_AB_DIR;
const std::string kPreregDir = kOracleAbDir + "/../preregistration";

// ---- registered numbers (rm3b-hit-ab-gate.v2.md section 3; tolerances unchanged from v1) ----
// Error bound of a position or distance: tolerance * max(1, expected distance); rotations compare bitwise.
constexpr double kTolT = 3e-5;   // tier T: chain and glue bookkeeping, one injected point per query
constexpr double kTolE = 2e-3;   // tier E: end to end, the full feature set of the recorded callback
constexpr double kBaseTol = 1e-5;  // instrument self-consistency of the base pose, not a gate

// ------------------------------------------------------------------------------------------------
// Recording.

struct ExpHit {
  int type = 0;
  double pos[3] = {0, 0, 0};
  double rot[4] = {0, 0, 0, 0};  // w x y z
  double distance = 0;
};

struct AbQuery {
  double x = 0, y = 0;
  std::vector<ExpHit> hits;
};

struct AbCall {
  bool update = false;
  bool has_origin = false, has_facing = false;
  double origin[3] = {0, 0, 0};
  double facing[4] = {1, 0, 0, 0};
  int mirror = -1, left = -1;  // configure: -1 absent
};

struct AbStep {
  std::string id;
  std::vector<AbCall> calls;
  std::vector<AbQuery> queries;
};

struct AbWorldPoint {
  uint64_t id = 0;
  float conf = 0;
  Vec3f pos;
};

struct AbSample {
  int frame = 0;
  TrackingStatus status = TrackingStatus::kNormal;
  Vec3f position;
  QuatWxyz rotation;
  std::array<float, 16> intrinsics{};
  std::vector<AbWorldPoint> points;
  std::vector<AbStep> steps;
};

std::vector<std::string> Split(const std::string& line) {
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string c;
  while (std::getline(ss, c, '\t')) out.push_back(c);
  return out;
}

double D(const std::string& s) { return std::stod(s); }

std::vector<AbSample> ReadAb(const std::string& path) {
  std::ifstream in(path);
  EXPECT_TRUE(in.good()) << path;
  std::vector<AbSample> samples;
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    const std::vector<std::string> c = Split(line);
    if (c[0] == "S") {
      samples.emplace_back();
      AbSample& s = samples.back();
      s.frame = std::stoi(c[2]);
      s.status = c[3] == "NORMAL" ? TrackingStatus::kNormal : TrackingStatus::kLimited;
      s.position = Vec3f{std::stof(c[5]), std::stof(c[6]), std::stof(c[7])};
      s.rotation = QuatWxyz{std::stof(c[8]), std::stof(c[9]), std::stof(c[10]), std::stof(c[11])};
      for (size_t k = 0; k < 16; ++k) s.intrinsics[k] = std::stof(c[12 + k]);
    } else if (c[0] == "W") {
      samples.back().points.push_back(AbWorldPoint{static_cast<uint64_t>(std::stoull(c[1])),
                                                   std::stof(c[2]),
                                                   Vec3f{std::stof(c[3]), std::stof(c[4]), std::stof(c[5])}});
    } else if (c[0] == "T") {
      samples.back().steps.push_back(AbStep{c[1], {}, {}});
    } else if (c[0] == "U") {
      AbCall call;
      call.update = true;
      call.has_origin = c[1] != "-";
      call.has_facing = c[4] != "-";
      if (call.has_origin) {
        for (int k = 0; k < 3; ++k) call.origin[k] = D(c[1 + static_cast<size_t>(k)]);
      }
      if (call.has_facing) {
        for (int k = 0; k < 4; ++k) call.facing[k] = D(c[4 + static_cast<size_t>(k)]);
      }
      samples.back().steps.back().calls.push_back(call);
    } else if (c[0] == "G") {
      AbCall call;
      call.mirror = c[1] == "-" ? -1 : std::stoi(c[1]);
      call.left = c[2] == "-" ? -1 : std::stoi(c[2]);
      samples.back().steps.back().calls.push_back(call);
    } else if (c[0] == "Q") {
      AbQuery q;
      q.x = D(c[1]);
      q.y = D(c[2]);
      const size_t n = std::stoul(c[3]);
      for (size_t k = 0; k < n; ++k) {
        const size_t b = 4 + 9 * k;
        ExpHit h;
        h.type = std::stoi(c[b]);
        for (size_t j = 0; j < 3; ++j) h.pos[j] = D(c[b + 1 + j]);
        for (size_t j = 0; j < 4; ++j) h.rot[j] = D(c[b + 4 + j]);
        h.distance = D(c[b + 8]);
        q.hits.push_back(h);
      }
      samples.back().steps.back().queries.push_back(q);
    }
  }
  return samples;
}

// ------------------------------------------------------------------------------------------------
// Base configuration (page-runtime.js: origin (0, 2, 0), facing identity, responsive scale = origin.y)
// and the inverse of the 704 chain for it (author's model: it cancels in the A/B relation).

CoordinateSystem BaseCoords() {
  CoordinateSystem c;
  c.origin = Vec3f{0.f, 2.f, 0.f};
  c.facing = QuatWxyz{1.f, 0.f, 0.f, 0.f};
  c.scale = 2.f;
  return c;
}

Vec3f InversePosition(const CoordinateSystem& c, const Vec3f& pub) {
  const float s = c.scale > 0.f ? c.scale : 1.f;
  const c8::Quaternion oq =
      c8::groundPlaneRotation(c8::Quaternion(c.facing.w, c.facing.x, c.facing.y, c.facing.z));
  const c8::HMatrix o = c8::cameraMotion(c8::HVector3(c.origin.x, c.origin.y, c.origin.z), oq);
  const c8::HPoint3 p = o.inv() * c8::HPoint3(pub.x, pub.y, pub.z);
  float x = p.x() / s, y = p.y() / s, z = p.z() / s;
  if (c.mirrored_display) x = -x;
  if (!c.left_handed_axes) z = -z;
  return Vec3f{x, y, z};
}

QuatWxyz InverseRotation(const CoordinateSystem& c, const QuatWxyz& pub) {
  const c8::Quaternion oq =
      c8::groundPlaneRotation(c8::Quaternion(c.facing.w, c.facing.x, c.facing.y, c.facing.z));
  const c8::Quaternion q = oq.inverse().times(c8::Quaternion(pub.w, pub.x, pub.y, pub.z));
  float w = q.w(), x = q.x(), y = q.y(), z = q.z();
  if (c.mirrored_display) {
    y = -y;
    z = -z;
  }
  if (!c.left_handed_axes) {
    w = -w;
    z = -z;
  }
  return QuatWxyz{w, x, y, z};
}

struct InternalState {
  Vec3f position;
  QuatWxyz rotation;
  std::vector<WorldPointOut> points;
  TrackingStatus status = TrackingStatus::kNormal;
};

InternalState Reconstruct(const AbSample& s) {
  const CoordinateSystem base = BaseCoords();
  InternalState st;
  st.position = InversePosition(base, s.position);
  st.rotation = InverseRotation(base, s.rotation);
  st.status = s.status;
  for (const AbWorldPoint& p : s.points) {
    st.points.push_back(WorldPointOut{p.id, InversePosition(base, p.pos), p.conf});
  }
  return st;
}

// ------------------------------------------------------------------------------------------------
// Production replay: GlueRunner with a scripted core that returns the injected internal state.

struct Injection {
  bool identity = true;  // first response: identity pose (recenterPose then makes the next ones absolute)
  InternalState state;
};

class InjectedCore final : public TrackerCore {
 public:
  explicit InjectedCore(const Injection* inj) : inj_(inj) {}
  TrackerStep Track(const FrameInput&) override {
    TrackerStep t;
    t.status = inj_->identity ? TrackingStatus::kNormal : inj_->state.status;
    if (!inj_->identity) {
      const c8::HMatrix m = c8::cameraMotion(
          c8::HVector3(inj_->state.position.x, inj_->state.position.y, inj_->state.position.z),
          c8::Quaternion(inj_->state.rotation.w, inj_->state.rotation.x, inj_->state.rotation.y,
                         inj_->state.rotation.z));
      const std::array<float, 16>& d = m.data();
      for (size_t k = 0; k < 16; ++k) t.pose.m[k] = d[k];
      t.world_points_internal = inj_->state.points;
    }
    return t;
  }

 private:
  const Injection* inj_;
};

// What the registered recording contains (gate file section 1; numbers from class-counts.py, engine free).
// z / n / e classify the tier-T source hit of every query: z = original hit at distance 0 (the weights of
// every point underflow: the hit is the camera position), n = a hit at distance > 0, e = no hit.
struct Registered {
  const char* tsv_sha256;
  size_t samples, queries, with_hits, z, n, e;
  size_t min_samples_with_hits;
};

constexpr Registered kReg1 = {"7918675c7a1c6f3f3e1d53e11f064cbf94dab0bac501e76d0d9186ebee567240", 51, 5508, 5508, 228, 5280, 0, 40};
constexpr Registered kReg2 = {"2d399848918b1d0114921048d47549cb274e5b89d6b1e250e87853d1450091d9", 51, 5508, 5508, 336, 5172, 0, 40};
constexpr Registered kReg3 = {"34d0eb2d7754458da7daa830c4d97ffe4547c476038e4348bb97426e0f988bfe", 29, 3132, 2160, 156, 2004, 972, 15};

struct DatasetCfg {
  const char* name;
  std::string tsv;
  EngineConfig engine;
  int display_w = 0, display_h = 0;  // 0 = the capture size
  const Registered* reg = nullptr;
};

EngineConfig Flip6() {
  EngineConfig e;
  e.capture_width = 480;
  e.capture_height = 640;
  e.model = "K";  // reduced UA (api1 / ab1, flip6-s2)
  return e;
}

EngineConfig TumPixel7() {
  EngineConfig e;
  e.capture_width = 240;
  e.capture_height = 320;
  e.manufacturer = "Google";
  e.model = "Pixel 7";
  return e;
}

DatasetCfg Ab1() { return {"hab1", kOracleAbDir + "/tsv-hab1/hit_ab.tsv", Flip6(), 0, 0, &kReg1}; }
DatasetCfg Ab2() { return {"hab2", kOracleAbDir + "/tsv-hab2/hit_ab.tsv", Flip6(), 360, 640, &kReg2}; }
DatasetCfg Ab3() { return {"hab3", kOracleAbDir + "/tsv-hab3/hit_ab.tsv", TumPixel7(), 0, 0, &kReg3}; }

struct StepOut {
  std::vector<std::vector<XrHitResult>> queries;
};

void ApplyCall(GlueRunner* glue, const AbCall& c) {
  if (c.update) {
    CameraProjectionUpdate u;
    if (c.has_origin) {
      u.origin = Vec3f{static_cast<float>(c.origin[0]), static_cast<float>(c.origin[1]),
                       static_cast<float>(c.origin[2])};
    }
    if (c.has_facing) {
      u.facing = QuatWxyz{static_cast<float>(c.facing[0]), static_cast<float>(c.facing[1]),
                          static_cast<float>(c.facing[2]), static_cast<float>(c.facing[3])};
    }
    glue->UpdateCameraProjectionMatrix(u);
  } else {
    XrConfigureOptions o;
    if (c.mirror >= 0) o.mirrored_display = c.mirror == 1;
    if (c.left >= 0) o.left_handed_axes = c.left == 1;
    EXPECT_TRUE(glue->Configure(o).ok());
  }
}

// Replays the steps of `sample` (its calls and queries) on the internal state `st`. `base` receives
// the published frame of the base configuration (the instrument self-check).
std::vector<StepOut> ReplayProduction(const DatasetCfg& cfg, const InternalState& st,
                                      const AbSample& sample, PublishedFrame* base) {
  Injection inj;
  GlueConfig g;
  g.frames_until_recenter = 0;  // FULL-7: no startup recenter, the first response is the injected pose
  GlueRunner glue(cfg.engine, g, [&inj](const EngineConfig&) { return std::make_unique<InjectedCore>(&inj); });
  if (cfg.display_w > 0) {
    CameraProjectionUpdate u;
    u.pixel_rect_width = cfg.display_w;
    u.pixel_rect_height = cfg.display_h;
    glue.UpdateCameraProjectionMatrix(u);
  }
  const std::vector<uint8_t> px(16, 0);
  int k = 0;
  auto step = [&]() {
    ++k;
    return glue.ProcessFrame(1000000000LL + k * 33000000LL, 0, GrayImageView{px.data(), 4, 4, 4});
  };
  step();  // fills the pipeline
  step();  // Track #0: identity pose -> the initial transform of recenterPose
  inj.identity = false;
  inj.state = st;
  const PublishedFrame f = step();
  if (base != nullptr) *base = f;
  std::vector<StepOut> out;
  for (const AbStep& s : sample.steps) {
    for (const AbCall& c : s.calls) ApplyCall(&glue, c);
    StepOut so;
    for (const AbQuery& q : s.queries) {
      so.queries.push_back(glue.XrHitTest(static_cast<float>(q.x), static_cast<float>(q.y), {}));
    }
    out.push_back(std::move(so));
  }
  return out;
}

// Tier T replay (see the header): one injected point per query.
size_t PartnerIndex(const AbStep& base, size_t j) {
  const AbQuery& q = base.queries[j];
  for (size_t k = 0; k < base.queries.size(); ++k) {
    if (std::fabs(base.queries[k].x + q.x - 1.0) < 1e-9 && std::fabs(base.queries[k].y - q.y) < 1e-9) return k;
  }
  ADD_FAILURE() << "no mirror partner of query " << j;
  return j;
}

// Classes of the tier-T source hit of a query (gate file section 2, v2): z = the original hit lies at the
// camera (distance exactly 0: every weight underflowed), n = a hit at distance > 0, e = no hit.
struct TierTCounts {
  size_t zero = 0, nonzero = 0, nosource = 0;
};

// V2-2. For a zero-depth source hit the v1 injection (the camera position as the point) gives
// flatten(0/0) = NaN in the engine and compared nothing. The point injected now has normalized image
// coordinates (8, 8) at depth 3 of the camera: its Gaussian weight exp(-0.5 * (d / 0.02)^2) is exactly 0
// for every query ray (d >= 7), so the engine takes the same path as the original (weight sum 0 -> depth 0
// -> hit at the camera) and the coordinate chain of the camera position is still tested.
constexpr float kFarNormalized = 8.0f, kFarDepth = 3.0f;

WorldPointOut FarPoint(const InternalState& camera) {
  const c8::HMatrix m = c8::cameraMotion(
      c8::HVector3(camera.position.x, camera.position.y, camera.position.z),
      c8::Quaternion(camera.rotation.w, camera.rotation.x, camera.rotation.y, camera.rotation.z));
  const c8::HPoint3 w =
      m * c8::HPoint3(kFarNormalized * kFarDepth, kFarNormalized * kFarDepth, kFarDepth);
  return WorldPointOut{1, Vec3f{w.x(), w.y(), w.z()}, 1.f};
}

// `v1_zero_depth` reproduces the v1 injection (negative control N5 only).
std::vector<StepOut> ReplayTierT(const DatasetCfg& cfg, const InternalState& camera, const AbSample& sample,
                                 TierTCounts* counts = nullptr, bool v1_zero_depth = false) {
  Injection inj;
  GlueConfig g;
  g.frames_until_recenter = 0;
  GlueRunner glue(cfg.engine, g, [&inj](const EngineConfig&) { return std::make_unique<InjectedCore>(&inj); });
  if (cfg.display_w > 0) {
    CameraProjectionUpdate u;
    u.pixel_rect_width = cfg.display_w;
    u.pixel_rect_height = cfg.display_h;
    glue.UpdateCameraProjectionMatrix(u);
  }
  const std::vector<uint8_t> px(16, 0);
  int k = 0;
  auto step = [&]() {
    ++k;
    glue.ProcessFrame(1000000000LL + k * 33000000LL, 0, GrayImageView{px.data(), 4, 4, 4});
  };
  step();
  step();  // Track #0: identity pose
  inj.identity = false;
  inj.state = camera;
  EXPECT_EQ(sample.steps.front().id, "base0");
  const AbStep& base = sample.steps.front();
  const CoordinateSystem base_coords = BaseCoords();
  bool mirror = false;  // the base configuration has no mirror
  std::vector<StepOut> out;
  for (const AbStep& s : sample.steps) {
    for (const AbCall& c : s.calls) {
      ApplyCall(&glue, c);
      if (!c.update && c.mirror >= 0) mirror = c.mirror == 1;
    }
    StepOut so;
    for (size_t j = 0; j < s.queries.size(); ++j) {
      // The internal ray of a mirrored query is the one of the partner screen point (x' = 1 - x).
      const size_t src = mirror ? PartnerIndex(base, j) : j;
      const std::vector<ExpHit>& src_hits = base.queries[src].hits;
      if (counts != nullptr) {
        if (src_hits.empty()) {
          ++counts->nosource;
        } else if (src_hits[0].distance == 0.0) {
          ++counts->zero;
        } else {
          ++counts->nonzero;
        }
      }
      inj.state.points.clear();
      for (const ExpHit& h : src_hits) {
        if (h.distance == 0.0 && !v1_zero_depth) {
          inj.state.points.push_back(FarPoint(camera));
          continue;
        }
        inj.state.points.push_back(WorldPointOut{
            1, InversePosition(base_coords, Vec3f{static_cast<float>(h.pos[0]), static_cast<float>(h.pos[1]),
                                                  static_cast<float>(h.pos[2])}),
            1.f});
      }
      step();  // a new response: the hit test state is this point
      so.queries.push_back(glue.XrHitTest(static_cast<float>(s.queries[j].x), static_cast<float>(s.queries[j].y), {}));
    }
    out.push_back(std::move(so));
  }
  return out;
}

#ifndef SR_HITAB_HELD

// ------------------------------------------------------------------------------------------------
// Mutant models (test-local copies of the chain with one deliberate error each; the hit algorithm
// is the production FeaturePointHitTest). With no flag the model equals the production chain.

struct Flags {
  bool full_facing = false;        // facing used as a whole instead of its yaw (groundPlaneRotation)
  bool ignore_scale = false;
  bool ignore_origin = false;
  bool ignore_handedness = false;
  bool mirror_sign = false;        // mirror does not negate x
  bool mirror_no_query_flip = false;  // mirror does not flip the screen x of the query
};

XrHitResult ModelRewrite(const CoordinateSystem& c, const InternalHit& h, const Flags& f) {
  XrHitResult out;
  out.type = h.type;
  float px = h.position.x, py = h.position.y, pz = h.position.z;
  float qw = 0.f, qx = 0.f, qy = 0.f, qz = 0.f;
  if (!c.left_handed_axes && !f.ignore_handedness) {
    pz = -pz;
    qw = -qw;
    qz = -qz;
  }
  if (c.mirrored_display && !f.mirror_sign) px = -px;
  const float s = (c.scale > 0.f && !f.ignore_scale) ? c.scale : 1.f;
  const c8::Quaternion fq(c.facing.w, c.facing.x, c.facing.y, c.facing.z);
  const c8::Quaternion oq = f.full_facing ? fq : c8::groundPlaneRotation(fq);
  const c8::HVector3 origin = f.ignore_origin ? c8::HVector3(0.f, 0.f, 0.f)
                                              : c8::HVector3(c.origin.x, c.origin.y, c.origin.z);
  const c8::HMatrix o = c8::cameraMotion(origin, oq);
  const c8::HPoint3 np = o * c8::HPoint3(s * px, s * py, s * pz);
  out.position = Vec3f{np.x(), np.y(), np.z()};
  out.rotation = QuatWxyz{qw, qx, qy, qz};
  out.distance = h.distance * s;
  return out;
}

std::vector<StepOut> ReplayModel(const DatasetCfg& cfg, const InternalState& st, const AbSample& sample,
                                 const Flags& flags) {
  (void)cfg;
  HitTestState hs;
  hs.camera_position = st.position;
  hs.camera_rotation = st.rotation;
  hs.projection = sample.intrinsics;
  for (const WorldPointOut& p : st.points) hs.points.push_back(p.position);
  CoordinateSystem c = BaseCoords();
  std::vector<StepOut> out;
  for (const AbStep& s : sample.steps) {
    for (const AbCall& call : s.calls) {
      if (call.update) {
        if (call.has_origin) {
          c.origin = Vec3f{static_cast<float>(call.origin[0]), static_cast<float>(call.origin[1]),
                           static_cast<float>(call.origin[2])};
        }
        if (call.has_facing) {
          c.facing = QuatWxyz{static_cast<float>(call.facing[0]), static_cast<float>(call.facing[1]),
                              static_cast<float>(call.facing[2]), static_cast<float>(call.facing[3])};
        }
        c.scale = c.origin.y;
      } else {
        if (call.mirror >= 0) c.mirrored_display = call.mirror == 1;
        if (call.left >= 0) c.left_handed_axes = call.left == 1;
      }
    }
    StepOut so;
    for (const AbQuery& q : s.queries) {
      std::vector<XrHitResult> hits;
      const bool query_mirror = c.mirrored_display && !flags.mirror_no_query_flip;
      for (const InternalHit& h : FeaturePointHitTest(hs, static_cast<float>(q.x), static_cast<float>(q.y), 0,
                                                      query_mirror)) {
        hits.push_back(ModelRewrite(c, h, flags));
      }
      so.queries.push_back(std::move(hits));
    }
    out.push_back(std::move(so));
  }
  return out;
}

#endif  // SR_HITAB_HELD

// ------------------------------------------------------------------------------------------------
// Comparator (one for every group). V2-1: fail closed. A non-finite value (NaN, +-inf) of the engine
// answer is a violation of its own (`NaN > tol` is false, so v1 passed it), and no aggregate may swallow
// it (`std::max` ignores NaN, so v1 could not even see it in the resolution).

struct Stats {
  size_t queries = 0;
  size_t count_mismatch = 0, type_mismatch = 0, pos_fail = 0, dist_fail = 0, rot_fail = 0;
  size_t nonfinite_actual = 0;    // engine answers with a NaN / inf position or distance
  size_t nonfinite_expected = 0;  // recording values that are not finite (an instrument error)
  double max_pos_rel = 0, max_dist_rel = 0;
  TierTCounts t;  // tier T only: classes of the injected source hits
  std::map<std::string, size_t> bad_steps;
  bool ok() const { return failures() == 0 && std::isfinite(max_pos_rel) && std::isfinite(max_dist_rel); }
  size_t failures() const {
    return count_mismatch + type_mismatch + pos_fail + dist_fail + rot_fail + nonfinite_actual + nonfinite_expected;
  }
};

bool SameBits(float a, float b) { return std::memcmp(&a, &b, sizeof(float)) == 0; }

// max that cannot lose a NaN: a non-finite sample makes the maximum +inf.
void MaxInto(double* m, double v) {
  if (!std::isfinite(v)) {
    *m = INFINITY;
  } else if (v > *m) {
    *m = v;
  }
}

bool FiniteExpected(const ExpHit& h) {
  bool f = std::isfinite(h.distance);
  for (int k = 0; k < 3; ++k) f = f && std::isfinite(h.pos[k]);
  for (int k = 0; k < 4; ++k) f = f && std::isfinite(h.rot[k]);
  return f;
}

bool FiniteActual(const XrHitResult& h) {
  return std::isfinite(h.position.x) && std::isfinite(h.position.y) && std::isfinite(h.position.z) &&
         std::isfinite(h.distance);
}

void Compare(const AbSample& expected, const std::vector<StepOut>& actual, double tolerance, Stats* st) {
  ASSERT_EQ(actual.size(), expected.steps.size());
  for (size_t i = 0; i < expected.steps.size(); ++i) {
    const AbStep& es = expected.steps[i];
    ASSERT_EQ(actual[i].queries.size(), es.queries.size());
    for (size_t j = 0; j < es.queries.size(); ++j) {
      ++st->queries;
      const std::vector<ExpHit>& want = es.queries[j].hits;
      const std::vector<XrHitResult>& got = actual[i].queries[j];
      bool bad = false;
      if (got.size() != want.size()) {
        ++st->count_mismatch;
        st->bad_steps[es.id]++;
        continue;
      }
      for (size_t k = 0; k < want.size(); ++k) {
        if (static_cast<int>(got[k].type) != want[k].type) {
          ++st->type_mismatch;
          bad = true;
          continue;
        }
        if (!FiniteExpected(want[k])) {
          ++st->nonfinite_expected;
          bad = true;
          continue;
        }
        if (!FiniteActual(got[k])) {
          ++st->nonfinite_actual;
          MaxInto(&st->max_pos_rel, NAN);
          MaxInto(&st->max_dist_rel, NAN);
          bad = true;
          continue;
        }
        const double tol = tolerance * std::max(1.0, want[k].distance);
        const double dx = got[k].position.x - want[k].pos[0], dy = got[k].position.y - want[k].pos[1],
                     dz = got[k].position.z - want[k].pos[2];
        const double dp = std::sqrt(dx * dx + dy * dy + dz * dz);
        const double dd = std::fabs(got[k].distance - want[k].distance);
        const double scale = std::max(1.0, want[k].distance);
        MaxInto(&st->max_pos_rel, dp / scale);
        MaxInto(&st->max_dist_rel, dd / scale);
        if (!(dp <= tol)) {
          ++st->pos_fail;
          bad = true;
        }
        if (!(dd <= tol)) {
          ++st->dist_fail;
          bad = true;
        }
        if (!SameBits(got[k].rotation.w, static_cast<float>(want[k].rot[0])) ||
            !SameBits(got[k].rotation.x, static_cast<float>(want[k].rot[1])) ||
            !SameBits(got[k].rotation.y, static_cast<float>(want[k].rot[2])) ||
            !SameBits(got[k].rotation.z, static_cast<float>(want[k].rot[3]))) {
          ++st->rot_fail;
          bad = true;
        }
      }
      if (bad) st->bad_steps[es.id]++;
    }
  }
}

std::string Summary(const Stats& st) {
  std::string by_step;
  for (const auto& [id, n] : st.bad_steps) by_step += (by_step.empty() ? "" : ",") + id + ":" + std::to_string(n);
  char buf[640];
  std::snprintf(buf, sizeof(buf),
                "queries=%zu count_mismatch=%zu type_mismatch=%zu pos_fail=%zu dist_fail=%zu rot_fail=%zu "
                "nonfinite_actual=%zu nonfinite_expected=%zu max_pos_rel=%.3g max_dist_rel=%.3g bad_steps=[%s]",
                st.queries, st.count_mismatch, st.type_mismatch, st.pos_fail, st.dist_fail, st.rot_fail,
                st.nonfinite_actual, st.nonfinite_expected, st.max_pos_rel, st.max_dist_rel, by_step.c_str());
  return buf;
}

Stats ReplayAllE(const DatasetCfg& cfg, const std::vector<AbSample>& rec, double tolerance) {
  Stats st;
  for (const AbSample& s : rec) Compare(s, ReplayProduction(cfg, Reconstruct(s), s, nullptr), tolerance, &st);
  return st;
}

Stats ReplayAllT(const DatasetCfg& cfg, const std::vector<AbSample>& rec, double tolerance) {
  Stats st;
  for (const AbSample& s : rec) Compare(s, ReplayTierT(cfg, Reconstruct(s), s, &st.t), tolerance, &st);
  return st;
}

// ------------------------------------------------------------------------------------------------
// Properties of a recording that need no engine: the original-side null-run (HitAbOriginal) and the
// class counts. Both are compiled into the held binary too: the gate runs them first (V2-4).

bool SameExp(const ExpHit& a, const ExpHit& b, double tol) {
  if (a.type != b.type) return false;
  double d = std::fabs(a.distance - b.distance);
  for (int k = 0; k < 3; ++k) d = std::max(d, std::fabs(a.pos[k] - b.pos[k]));
  return d <= tol;
}

bool BitEqual(const ExpHit& a, const ExpHit& b) {
  auto same = [](double x, double y) { return std::memcmp(&x, &y, sizeof(double)) == 0; };
  bool eq = a.type == b.type && same(a.distance, b.distance);
  for (int k = 0; k < 3; ++k) eq = eq && same(a.pos[k], b.pos[k]);
  for (int k = 0; k < 4; ++k) eq = eq && same(a.rot[k], b.rot[k]);
  return eq;
}

const AbStep& StepNamed(const AbSample& s, const char* id) {
  for (const AbStep& step : s.steps) {
    if (step.id == id) return step;
  }
  ADD_FAILURE() << "no step " << id;
  return s.steps.front();
}

struct OriginalInvariants {
  size_t samples = 0, with_hits = 0, queries = 0;
  size_t null_identity_diff = 0, restore_diff = 0, pitch_diff = 0, roll_diff = 0, combo_diff = 0;
  size_t lefthanded_diff = 0, mirror_pair_diff = 0, scale_same = 0, origin_same = 0, yaw_same = 0;
  size_t mirror_pairs = 0;
};

OriginalInvariants CheckOriginal(const std::vector<AbSample>& rec) {
  OriginalInvariants r;
  for (const AbSample& s : rec) {
    ++r.samples;
    const AbStep& base = StepNamed(s, "base0");
    bool any = false;
    for (size_t q = 0; q < base.queries.size(); ++q) {
      const std::vector<ExpHit>& b = base.queries[q].hits;
      if (b.empty()) continue;
      any = true;
      ++r.queries;
      const double d = std::max(1.0, b[0].distance);
      // A: bitwise coincidence of steps that must coincide (original-side null-run). Pitch and roll
      // of the facing are dropped by groundPlaneRotation, so those steps equal the base / the yaw step.
      auto bit = [&](const char* id, const ExpHit& ref, size_t* diff) {
        const std::vector<ExpHit>& o = StepNamed(s, id).queries[q].hits;
        if (o.size() != 1 || !BitEqual(o[0], ref)) ++*diff;
      };
      bit("null-identity", b[0], &r.null_identity_diff);
      bit("restore", b[0], &r.restore_diff);
      bit("pitch25", b[0], &r.pitch_diff);
      bit("roll20", b[0], &r.roll_diff);
      {
        const std::vector<ExpHit>& y = StepNamed(s, "yaw40").queries[q].hits;
        if (y.size() != 1) {
          ++r.combo_diff;
        } else {
          bit("yaw40-pitch25-roll20", y[0], &r.combo_diff);
        }
      }
      // B: the steps that must differ do (the recording discriminates).
      if (SameExp(StepNamed(s, "origin-xz").queries[q].hits.at(0), b[0], 1e-3)) ++r.origin_same;
      if (SameExp(StepNamed(s, "scale-3.5").queries[q].hits.at(0), b[0], 1e-3)) ++r.scale_same;
      if (SameExp(StepNamed(s, "yaw40").queries[q].hits.at(0), b[0], 1e-3)) ++r.yaw_same;
      // C: left handed = z negated about origin z = 0, everything else equal.
      const ExpHit& l = StepNamed(s, "lefthanded").queries[q].hits.at(0);
      if (std::fabs(l.pos[0] - b[0].pos[0]) > 1e-4 * d || std::fabs(l.pos[1] - b[0].pos[1]) > 1e-4 * d ||
          std::fabs(l.pos[2] + b[0].pos[2]) > 1e-4 * d || std::fabs(l.distance - b[0].distance) > 1e-4 * d) {
        ++r.lefthanded_diff;
      }
      // D: mirror on at (x, y) = mirror off at the partner (1 - x, y) with x negated (centred principal
      // point, origin x 0 and facing identity in both steps).
      const std::vector<ExpHit>& m = StepNamed(s, "mirror").queries[q].hits;
      const std::vector<ExpHit>& n = base.queries[PartnerIndex(base, q)].hits;
      if (!m.empty() && !n.empty()) {
        ++r.mirror_pairs;
        const double dn = std::max(1.0, n[0].distance);
        if (std::fabs(m[0].pos[0] + n[0].pos[0]) > 1e-4 * dn || std::fabs(m[0].pos[1] - n[0].pos[1]) > 1e-4 * dn ||
            std::fabs(m[0].pos[2] - n[0].pos[2]) > 1e-4 * dn || std::fabs(m[0].distance - n[0].distance) > 1e-4 * dn) {
          ++r.mirror_pair_diff;
        }
      }
    }
    r.with_hits += any ? 1 : 0;
  }
  return r;
}

// O-1..O-6 as one predicate (gate file section 3). `discriminates` = the recording moves (almost) every hit
// under an origin / scale / yaw change.
bool OriginalHolds(const OriginalInvariants& r, size_t min_samples_with_hits) {
  const bool discriminates = r.origin_same < r.queries / 100 + 1 && r.scale_same < r.queries / 100 + 1 &&
                             r.yaw_same < r.queries / 100 + 1;
  return r.with_hits >= min_samples_with_hits && r.null_identity_diff == 0 && r.restore_diff == 0 &&
         r.pitch_diff == 0 && r.roll_diff == 0 && r.combo_diff == 0 && r.lefthanded_diff == 0 &&
         r.mirror_pair_diff == 0 && r.mirror_pairs == r.queries && discriminates;
}

std::string OriginalLine(const char* name, const OriginalInvariants& r) {
  char buf[640];
  std::snprintf(buf, sizeof(buf),
                "[HitAbOriginal %s] samples=%zu with_hits=%zu queries=%zu | must coincide (bitwise): null_identity=%zu "
                "restore=%zu pitch25=%zu roll20=%zu combo_vs_yaw40=%zu | lefthanded_rule_violations=%zu "
                "mirror_pairs=%zu mirror_pair_violations=%zu | must differ but coincide: origin=%zu scale=%zu yaw=%zu",
                name, r.samples, r.with_hits, r.queries, r.null_identity_diff, r.restore_diff, r.pitch_diff,
                r.roll_diff, r.combo_diff, r.lefthanded_diff, r.mirror_pairs, r.mirror_pair_diff, r.origin_same,
                r.scale_same, r.yaw_same);
  return buf;
}

// Engine-free classes of a recording; the tier-T source of a query is its base0 hit (the partner's while
// the mirror is on). Same rule as ReplayTierT and as class-counts.py.
struct RecordingClasses {
  size_t samples = 0, queries = 0, with_hits = 0, zero = 0, nonzero = 0, nosource = 0;
  size_t samples_limited = 0, samples_without_world_points = 0, samples_padding_only = 0, samples_mixed = 0;
};

RecordingClasses ClassifyRecording(const std::vector<AbSample>& rec) {
  RecordingClasses c;
  for (const AbSample& s : rec) {
    ++c.samples;
    c.samples_limited += s.status == TrackingStatus::kLimited ? 1 : 0;
    c.samples_without_world_points += s.points.empty() ? 1 : 0;
    bool padding_only = !s.points.empty();
    for (const AbWorldPoint& p : s.points) padding_only = padding_only && p.id == 0;
    c.samples_padding_only += padding_only ? 1 : 0;
    const AbStep& base = s.steps.front();
    bool mirror = false;
    size_t z = 0, n = 0;
    for (const AbStep& st : s.steps) {
      for (const AbCall& call : st.calls) {
        if (!call.update && call.mirror >= 0) mirror = call.mirror == 1;
      }
      for (size_t j = 0; j < st.queries.size(); ++j) {
        ++c.queries;
        c.with_hits += st.queries[j].hits.empty() ? 0 : 1;
        const std::vector<ExpHit>& src = base.queries[mirror ? PartnerIndex(base, j) : j].hits;
        if (src.empty()) {
          ++c.nosource;
        } else if (src[0].distance == 0.0) {
          ++c.zero;
          ++z;
        } else {
          ++c.nonzero;
          ++n;
        }
      }
    }
    c.samples_mixed += (z > 0 && n > 0) ? 1 : 0;
  }
  return c;
}

std::string ClassLine(const char* name, const RecordingClasses& c) {
  char buf[400];
  std::snprintf(buf, sizeof(buf),
                "[classes %s] samples=%zu queries=%zu with_hits=%zu zero_depth(Z)=%zu nonzero(N)=%zu no_source(E)=%zu "
                "limited_samples=%zu samples_without_world_points=%zu padding_only_samples=%zu mixed_samples=%zu",
                name, c.samples, c.queries, c.with_hits, c.zero, c.nonzero, c.nosource, c.samples_limited,
                c.samples_without_world_points, c.samples_padding_only, c.samples_mixed);
  return buf;
}

// ------------------------------------------------------------------------------------------------
// Integrity of the gate (V2-4). SHA-256 (FIPS 180-4) so that no dependency is added; known-answer tests below.

std::string Sha256Hex(const std::string& msg) {
  static const uint32_t k[64] = {
      0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
      0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
      0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
      0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
      0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
      0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
      0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
      0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};
  uint32_t h[8] = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
  auto rotr = [](uint32_t x, int n) { return (x >> n) | (x << (32 - n)); };
  auto block = [&](const uint8_t* p) {
    uint32_t w[64];
    for (int i = 0; i < 16; ++i) {
      w[i] = (uint32_t{p[4 * i]} << 24) | (uint32_t{p[4 * i + 1]} << 16) | (uint32_t{p[4 * i + 2]} << 8) | p[4 * i + 3];
    }
    for (int i = 16; i < 64; ++i) {
      const uint32_t s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
      const uint32_t s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
      w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4], f = h[5], g = h[6], hh = h[7];
    for (int i = 0; i < 64; ++i) {
      const uint32_t s1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
      const uint32_t ch = (e & f) ^ (~e & g);
      const uint32_t t1 = hh + s1 + ch + k[i] + w[i];
      const uint32_t s0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
      const uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
      const uint32_t t2 = s0 + maj;
      hh = g;
      g = f;
      f = e;
      e = d + t1;
      d = c;
      c = b;
      b = a;
      a = t1 + t2;
    }
    h[0] += a;
    h[1] += b;
    h[2] += c;
    h[3] += d;
    h[4] += e;
    h[5] += f;
    h[6] += g;
    h[7] += hh;
  };
  const uint8_t* data = reinterpret_cast<const uint8_t*>(msg.data());
  const size_t full = msg.size() / 64;
  for (size_t i = 0; i < full; ++i) block(data + 64 * i);
  uint8_t tail[128] = {0};
  const size_t rest = msg.size() - 64 * full;
  if (rest > 0) std::memcpy(tail, data + 64 * full, rest);
  tail[rest] = 0x80;
  const size_t tail_len = rest < 56 ? 64 : 128;
  const uint64_t bits = static_cast<uint64_t>(msg.size()) * 8;
  for (int i = 0; i < 8; ++i) tail[tail_len - 1 - static_cast<size_t>(i)] = static_cast<uint8_t>(bits >> (8 * i));
  for (size_t off = 0; off < tail_len; off += 64) block(tail + off);
  char out[65];
  for (int i = 0; i < 8; ++i) std::snprintf(out + 8 * i, 9, "%08x", h[i]);
  return out;
}

bool ReadWholeFile(const std::string& path, std::string* out) {
  std::ifstream in(path, std::ios::binary);
  if (!in.good()) return false;
  std::stringstream ss;
  ss << in.rdbuf();
  *out = ss.str();
  return true;
}

// Start of the first real run (protocol §12.5, §12.6). The release decision and the one-run marker are the shared device sr_held
// (tests/support/sr_held.h, docs/design/sr-held-design.md). The gate is designated by the caller (SLAM_REFACT_RM3B_GATE=<gate_id>.v<N>);
// this source names no gate id or version. The order is: every check that can refuse the run, THEN the marker (a refusal never uses
// up the one run):
//   1. sr_held::CheckRelease  env value = sha256 of the registered designation file, manifest, review, pinned binary (this
//                             executable), gate file; no side effect
//   2. every recording TSV has the sha256 the code constants register (in code, not in a file the coordinator could edit)
//   3. sr_held::HeldGate      the same checks again, then the O_EXCL marker; the code never removes it
// The recordings are not a manifest input tree: oracle-ab holds more than the three TSVs, so their sha256 stays a code constant.
namespace sh = slam_refact::sr_held;

// "<gate_id>.v<N>" -> gate_id, N. A name the caller designated; anything else is refused.
bool SplitGateName(const std::string& name, std::string* gate_id, int* version) {
  const size_t p = name.rfind(".v");
  if (p == std::string::npos || p == 0 || p + 2 >= name.size()) return false;
  const std::string digits = name.substr(p + 2);
  if (digits.size() > 6 || digits[0] == '0' || !std::all_of(digits.begin(), digits.end(), [](char c) { return c >= '0' && c <= '9'; })) return false;
  *gate_id = name.substr(0, p);
  *version = std::atoi(digits.c_str());
  return sh::IsId(*gate_id);
}

struct ReleaseInputs {
  std::string env_value;                 // SLAM_REFACT_RM3B_REVIEWER
  std::string prereg_dir;                // holds the manifest, the designation file, the review and the gate file
  std::string gate_name;                 // "<gate_id>.v<N>", SLAM_REFACT_RM3B_GATE
  std::vector<std::string> pins;         // files manifest held.pins registers: the held binary (this executable)
  std::vector<std::pair<std::string, std::string>> data_files;  // path, registered sha256 (code constants)
};

struct ReleaseResult {
  bool ok = false;
  std::string code;              // "" when ok; an sr_held refusal code, "ARGS" or "RECORDING"
  std::string reason;            // first failed check
  std::vector<std::string> log;  // every check performed, for the head of real-run.log
};

ReleaseResult StartRelease(const ReleaseInputs& in) {
  ReleaseResult r;
  auto fail = [&r](const std::string& code, const std::string& why) {
    r.ok = false;
    r.code = code;
    r.reason = why;
    r.log.push_back("[RM3B2-INTEGRITY] FAIL " + code + ": " + why);
    return r;
  };
  sh::Request rq;
  rq.prereg_dir = in.prereg_dir;
  rq.kind = "main";
  rq.env_value = in.env_value;
  rq.pins = in.pins;
  if (!SplitGateName(in.gate_name, &rq.gate_id, &rq.version)) {
    return fail("ARGS", "HELD: the gate is not designated as <gate_id>.v<N> (SLAM_REFACT_RM3B_GATE='" + in.gate_name + "')");
  }
  const sh::Outcome check = sh::CheckRelease(rq);
  for (const std::string& l : check.log) r.log.push_back(l);
  if (!check.ok) return fail(check.code, check.why);
  for (const auto& [path, want] : in.data_files) {
    std::string data;
    if (!ReadWholeFile(path, &data)) return fail("RECORDING", "recording unreadable: " + path);
    const std::string actual = Sha256Hex(data);
    r.log.push_back("[RM3B2-INTEGRITY] recording " + path + " sha256 actual=" + actual + " registered=" + want);
    if (actual != want) return fail("RECORDING", "recording sha256 differs from the registered one: " + path);
  }
  const sh::Outcome claimed = sh::HeldGate(rq);
  if (!claimed.ok) return fail(claimed.code, claimed.why);
  r.log.push_back("[RM3B2-INTEGRITY] first-run marker created for " + in.gate_name + ": " + claimed.marker);
  r.ok = true;
  r.log.push_back("[RM3B2-INTEGRITY] OK: release conditions hold");
  return r;
}

#ifndef SR_HITAB_HELD

// ------------------------------------------------------------------------------------------------
// Synthetic recording made by the engine itself (instrument checks).

class Rng {
 public:
  explicit Rng(uint32_t seed) : s_(seed) {}
  double Uniform(double lo, double hi) {
    s_ ^= s_ << 13;
    s_ ^= s_ >> 17;
    s_ ^= s_ << 5;
    return lo + (hi - lo) * (static_cast<double>(s_) / 4294967296.0);
  }

 private:
  uint32_t s_;
};

c8::Quaternion AxisQuat(double deg, char axis) {
  const double h = deg * M_PI / 360.0;
  const float c = static_cast<float>(std::cos(h)), s = static_cast<float>(std::sin(h));
  return axis == 'x' ? c8::Quaternion(c, s, 0.f, 0.f)
                     : axis == 'y' ? c8::Quaternion(c, 0.f, s, 0.f) : c8::Quaternion(c, 0.f, 0.f, s);
}

AbCall Update(double ox, double oy, double oz, const c8::Quaternion& q) {
  AbCall c;
  c.update = true;
  c.has_origin = c.has_facing = true;
  c.origin[0] = ox;
  c.origin[1] = oy;
  c.origin[2] = oz;
  c.facing[0] = q.w();
  c.facing[1] = q.x();
  c.facing[2] = q.y();
  c.facing[3] = q.z();
  return c;
}

AbCall Configure(int mirror, int left) {
  AbCall c;
  c.mirror = mirror;
  c.left = left;
  return c;
}

// The step list of api-oracle.js (same arguments up to the last double digit).
std::vector<AbStep> DefaultSteps() {
  const c8::Quaternion id(1.f, 0.f, 0.f, 0.f);
  const c8::Quaternion combo = AxisQuat(40, 'y').times(AxisQuat(25, 'x')).times(AxisQuat(20, 'z'));
  const c8::Quaternion lh_q = AxisQuat(-65, 'y').times(AxisQuat(15, 'x'));
  std::vector<AbStep> s;
  s.push_back({"base0", {}, {}});
  s.push_back({"null-identity", {Update(0, 2, 0, id)}, {}});
  s.push_back({"origin-xz", {Update(0.7, 2, -1.3, id)}, {}});
  s.push_back({"scale-3.5", {Update(0.7, 3.5, -1.3, id)}, {}});
  s.push_back({"yaw40", {Update(0, 2, 0, AxisQuat(40, 'y'))}, {}});
  s.push_back({"pitch25", {Update(0, 2, 0, AxisQuat(25, 'x'))}, {}});
  s.push_back({"roll20", {Update(0, 2, 0, AxisQuat(20, 'z'))}, {}});
  s.push_back({"yaw40-pitch25-roll20", {Update(0, 2, 0, combo)}, {}});
  s.push_back({"mirror", {Update(0, 2, 0, id), Configure(1, -1)}, {}});
  s.push_back({"lefthanded", {Configure(0, 1)}, {}});
  s.push_back({"lh-mirror-origin-yaw", {Configure(1, 1), Update(-0.4, 1.2, 0.9, lh_q)}, {}});
  s.push_back({"restore", {Configure(0, 0), Update(0, 2, 0, id)}, {}});
  // The point list of api-oracle.js: every point has its mirror partner (1 - x, y).
  const double pts[9][2] = {{0.5, 0.5}, {0.3, 0.4}, {0.7, 0.4}, {0.2, 0.75}, {0.8, 0.75},
                            {0.8, 0.25}, {0.2, 0.25}, {0.1, 0.9}, {0.9, 0.9}};
  for (AbStep& step : s) {
    for (const auto& p : pts) step.queries.push_back(AbQuery{p[0], p[1], {}});
  }
  return s;
}

ExpHit ToExp(const XrHitResult& h) {
  ExpHit e;
  e.type = static_cast<int>(h.type);
  e.pos[0] = h.position.x;
  e.pos[1] = h.position.y;
  e.pos[2] = h.position.z;
  e.rot[0] = h.rotation.w;
  e.rot[1] = h.rotation.x;
  e.rot[2] = h.rotation.y;
  e.rot[3] = h.rotation.z;
  e.distance = h.distance;
  return e;
}

// V2-3: the input classes of the original recordings that the v1 synthetic recording lacked.
//   kRandom        100 points in view at depth 1.5..6 plus 8 padding points (the v1 sample)
//   kZeroDepthFar  24 points at normalized image coordinates (+-8, +-8): every weight underflows to 0, so the
//                  engine's own answer is a hit at the camera with distance 0 (the original's zero-depth hit)
//   kPaddingOnly   only the 8 padding points (id 0, internal origin) with the camera near the origin, like
//                  hab1 sample 0
//   kNoPoints      no world point at all (hab3: 9 of 29 samples)
//   kLimited       as kRandom with tracking status LIMITED (hab3: 20 of 29 samples)
//   kPartial       a cluster of points at one side of the image: the rays near it get weight, the far ones
//                  get exactly 0 (zero-depth and regular queries in one sample)
enum class Kind { kRandom, kZeroDepthFar, kPaddingOnly, kNoPoints, kLimited, kPartial };

// The "truth": a random internal state injected directly (no reconstruction); the recording keeps
// what a client of the engine sees: the published pose, world points, intrinsics and the hits.
AbSample SyntheticSample(const DatasetCfg& cfg, uint32_t seed, Kind kind = Kind::kRandom) {
  Rng r(seed);
  InternalState st;
  const c8::Quaternion q = AxisQuat(r.Uniform(-180, 180), 'y')
                               .times(AxisQuat(r.Uniform(-25, 25), 'x'))
                               .times(AxisQuat(r.Uniform(-15, 15), 'z'));
  const double near_origin = kind == Kind::kPaddingOnly ? 0.02 : 1.0;
  st.position = Vec3f{static_cast<float>(r.Uniform(-1.5, 1.5) * near_origin),
                      static_cast<float>(r.Uniform(-0.5, 0.5) * near_origin),
                      static_cast<float>(r.Uniform(-1.5, 1.5) * near_origin)};
  st.rotation = QuatWxyz{q.w(), q.x(), q.y(), q.z()};
  st.status = kind == Kind::kLimited ? TrackingStatus::kLimited : TrackingStatus::kNormal;
  const c8::HMatrix cam = c8::cameraMotion(c8::HVector3(st.position.x, st.position.y, st.position.z), q);
  uint64_t id = 1;
  auto padding = [&st](int n) {
    for (int i = 0; i < n; ++i) st.points.push_back(WorldPointOut{0, Vec3f{0.f, 0.f, 0.f}, 0.f});
  };
  switch (kind) {
    case Kind::kRandom:
    case Kind::kLimited:
      for (int i = 0; i < 100; ++i) {
        const double z = r.Uniform(1.5, 6.0);
        const c8::HPoint3 w = cam * c8::HPoint3(static_cast<float>(r.Uniform(-0.7, 0.7) * z),
                                                static_cast<float>(r.Uniform(-0.9, 0.9) * z), static_cast<float>(z));
        st.points.push_back(WorldPointOut{id++, Vec3f{w.x(), w.y(), w.z()}, 0.5f});
      }
      padding(8);
      break;
    case Kind::kZeroDepthFar:
      for (int i = 0; i < 24; ++i) {
        const double z = r.Uniform(1.5, 6.0);
        const double sx = (i & 1) ? 8.0 : -8.0, sy = (i & 2) ? 8.0 : -8.0;
        const c8::HPoint3 w = cam * c8::HPoint3(static_cast<float>(sx * z), static_cast<float>(sy * z), static_cast<float>(z));
        st.points.push_back(WorldPointOut{id++, Vec3f{w.x(), w.y(), w.z()}, 0.5f});
      }
      break;
    case Kind::kPaddingOnly:
      padding(8);
      break;
    case Kind::kNoPoints:
      break;
    case Kind::kPartial:
      for (int i = 0; i < 40; ++i) {
        const double z = r.Uniform(2.0, 4.0);
        const c8::HPoint3 w = cam * c8::HPoint3(static_cast<float>(r.Uniform(-0.55, -0.45) * z),
                                                static_cast<float>(r.Uniform(-0.35, -0.25) * z), static_cast<float>(z));
        st.points.push_back(WorldPointOut{id++, Vec3f{w.x(), w.y(), w.z()}, 0.5f});
      }
      padding(8);
      break;
  }

  AbSample s;
  s.steps = DefaultSteps();
  PublishedFrame base;
  const std::vector<StepOut> truth = ReplayProduction(cfg, st, s, &base);
  s.position = base.position;
  s.rotation = base.rotation;
  s.intrinsics = base.intrinsics;
  s.status = base.status;
  for (const WorldPointOut& p : base.world_points) s.points.push_back(AbWorldPoint{p.id, p.confidence, p.position});
  for (size_t i = 0; i < s.steps.size(); ++i) {
    for (size_t j = 0; j < s.steps[i].queries.size(); ++j) {
      for (const XrHitResult& h : truth[i].queries[j]) s.steps[i].queries[j].hits.push_back(ToExp(h));
    }
  }
  return s;
}

constexpr size_t kSynthetic = 24;  // random samples
constexpr size_t kExtraPerKind = 2;
constexpr size_t kExtraKinds = 5;

std::vector<AbSample> SyntheticOfKind(const DatasetCfg& cfg, Kind kind, size_t n, uint32_t seed0) {
  std::vector<AbSample> out;
  for (size_t i = 0; i < n; ++i) out.push_back(SyntheticSample(cfg, seed0 + 104729u * static_cast<uint32_t>(i), kind));
  return out;
}

// kSynthetic random samples (the v1 recording, same seeds) plus kExtraPerKind samples of each other class.
std::vector<AbSample> SyntheticRecording(const DatasetCfg& cfg, size_t n_random) {
  std::vector<AbSample> out;
  for (size_t i = 0; i < n_random; ++i) out.push_back(SyntheticSample(cfg, 0x1234567u + 7919u * static_cast<uint32_t>(i)));
  uint32_t seed0 = 0x0BADC0DEu;
  for (const Kind k : {Kind::kZeroDepthFar, Kind::kPaddingOnly, Kind::kNoPoints, Kind::kLimited, Kind::kPartial}) {
    for (AbSample& s : SyntheticOfKind(cfg, k, kExtraPerKind, seed0)) out.push_back(std::move(s));
    seed0 += 1000003u;
  }
  return out;
}

Stats ReplayModelAll(const DatasetCfg& cfg, const std::vector<AbSample>& rec, const Flags& f) {
  Stats st;
  for (const AbSample& s : rec) Compare(s, ReplayModel(cfg, Reconstruct(s), s, f), kTolE, &st);
  return st;
}

DatasetCfg SyntheticCfg() { return {"synthetic", "", Flip6(), 0, 0, nullptr}; }

constexpr size_t kQueriesPerSample = 12 * 9;

// ---- N0: the synthetic recording contains the input classes of the original recordings (V2-3) ----
TEST(HitAbNull, SyntheticRecordingCoversTheInputClasses) {
  const DatasetCfg cfg = SyntheticCfg();
  const std::vector<AbSample> rec = SyntheticRecording(cfg, kSynthetic);
  const RecordingClasses c = ClassifyRecording(rec);
  std::printf("%s\n", ClassLine("N0 synthetic", c).c_str());
  EXPECT_EQ(c.samples, kSynthetic + kExtraPerKind * kExtraKinds);
  EXPECT_GT(c.zero, 0u);
  EXPECT_GT(c.nonzero, 0u);
  EXPECT_GT(c.nosource, 0u);
  EXPECT_GE(c.samples_limited, kExtraPerKind);
  EXPECT_GE(c.samples_without_world_points, kExtraPerKind);
  EXPECT_GE(c.samples_padding_only, kExtraPerKind);
  EXPECT_GE(c.samples_mixed, kExtraPerKind);
  // The zero-depth kind is zero-depth for every query (a pure class for the Z-class mutants).
  const RecordingClasses z = ClassifyRecording(SyntheticOfKind(cfg, Kind::kZeroDepthFar, 4, 0x5EED0001u));
  std::printf("%s\n", ClassLine("N0 zero-depth-only", z).c_str());
  EXPECT_EQ(z.nonzero, 0u);
  EXPECT_EQ(z.nosource, 0u);
  EXPECT_EQ(z.zero, z.queries);
}

// ---- N1: the comparator applied to a recording against itself finds nothing ----
TEST(HitAbNull, ComparatorSelfIdentity) {
  const DatasetCfg cfg = SyntheticCfg();
  const std::vector<AbSample> rec = SyntheticRecording(cfg, kSynthetic);
  Stats st;
  for (const AbSample& s : rec) {
    std::vector<StepOut> same;
    for (const AbStep& step : s.steps) {
      StepOut so;
      for (const AbQuery& q : step.queries) {
        std::vector<XrHitResult> hits;
        for (const ExpHit& e : q.hits) {
          XrHitResult h;
          h.type = static_cast<XrHitType>(e.type);
          h.position = Vec3f{static_cast<float>(e.pos[0]), static_cast<float>(e.pos[1]), static_cast<float>(e.pos[2])};
          h.rotation = QuatWxyz{static_cast<float>(e.rot[0]), static_cast<float>(e.rot[1]),
                                static_cast<float>(e.rot[2]), static_cast<float>(e.rot[3])};
          h.distance = static_cast<float>(e.distance);
          hits.push_back(h);
        }
        so.queries.push_back(hits);
      }
      same.push_back(so);
    }
    Compare(s, same, kTolT, &st);
  }
  std::printf("[N1 synthetic] %s\n", Summary(st).c_str());
  EXPECT_EQ(st.queries, rec.size() * kQueriesPerSample);
  EXPECT_TRUE(st.ok());
  EXPECT_EQ(st.max_pos_rel, 0.0);
}

// ---- N1b (V2-1): the comparator fails closed on a non-finite value ----
// The reviewer's probe P1: NaN position and distance used to give ok = 1 (`NaN > tol` is false and
// std::max drops NaN). Every non-finite variant must now be a violation, and the aggregates must show it.
TEST(HitAbNull, ComparatorFailsClosedOnNonFiniteValues) {
  auto make = [](double dist) {
    AbSample s;
    s.steps.push_back(AbStep{"base0", {}, {}});
    AbQuery q;
    q.x = q.y = 0.5;
    ExpHit e;
    e.type = 1;
    e.pos[0] = 1;
    e.pos[1] = 2;
    e.pos[2] = 3;
    e.rot[0] = -0.0;
    e.rot[3] = -0.0;
    e.distance = dist;
    q.hits.push_back(e);
    s.steps[0].queries.push_back(q);
    return s;
  };
  auto actual = [](float px, float py, float pz, float d) {
    std::vector<StepOut> got(1);
    XrHitResult h;
    h.type = XrHitType::kFeaturePoint;
    h.position = Vec3f{px, py, pz};
    h.rotation = QuatWxyz{-0.f, 0.f, 0.f, -0.f};
    h.distance = d;
    got[0].queries.push_back({h});
    return got;
  };
  const float nan = NAN, inf = INFINITY;
  // control: an equal finite answer passes
  {
    Stats st;
    Compare(make(4), actual(1, 2, 3, 4), kTolT, &st);
    EXPECT_TRUE(st.ok()) << Summary(st);
  }
  struct Case {
    const char* name;
    std::vector<StepOut> got;
  };
  const Case cases[] = {{"NaN position and distance (probe P1)", actual(nan, nan, nan, nan)},
                        {"NaN position only", actual(1, nan, 3, 4)},
                        {"NaN distance only", actual(1, 2, 3, nan)},
                        {"+inf distance", actual(1, 2, 3, inf)},
                        {"-inf position", actual(1, -inf, 3, 4)}};
  for (const Case& c : cases) {
    Stats st;
    Compare(make(4), c.got, kTolT, &st);
    std::printf("[N1b %s] ok=%d %s\n", c.name, st.ok() ? 1 : 0, Summary(st).c_str());
    EXPECT_FALSE(st.ok()) << c.name;
    EXPECT_EQ(st.nonfinite_actual, 1u) << c.name;
    EXPECT_FALSE(std::isfinite(st.max_pos_rel)) << c.name << ": the aggregate must not lose the value";
    EXPECT_EQ(st.bad_steps["base0"], 1u) << c.name;
  }
  // a non-finite expected value is an instrument error, also a violation
  {
    Stats st;
    Compare(make(nan), actual(1, 2, 3, 4), kTolT, &st);
    EXPECT_FALSE(st.ok());
    EXPECT_EQ(st.nonfinite_expected, 1u);
  }
  // a NaN in the rotation compares by bits and fails against the finite recorded value
  {
    std::vector<StepOut> got = actual(1, 2, 3, 4);
    got[0].queries[0][0].rotation.w = nan;
    Stats st;
    Compare(make(4), got, kTolT, &st);
    EXPECT_FALSE(st.ok());
    EXPECT_EQ(st.rot_fail, 1u);
  }
}

// ---- N2: both replays are deterministic ----
TEST(HitAbNull, ReplayDeterminism) {
  const DatasetCfg cfg = SyntheticCfg();
  const std::vector<AbSample> rec = SyntheticRecording(cfg, 6);
  size_t diff = 0, hits = 0;
  auto count = [&](const std::vector<StepOut>& a, const std::vector<StepOut>& b) {
    ASSERT_EQ(a.size(), b.size());
    for (size_t i = 0; i < a.size(); ++i) {
      for (size_t j = 0; j < a[i].queries.size(); ++j) {
        ASSERT_EQ(a[i].queries[j].size(), b[i].queries[j].size());
        for (size_t k = 0; k < a[i].queries[j].size(); ++k) {
          ++hits;
          diff += std::memcmp(&a[i].queries[j][k], &b[i].queries[j][k], sizeof(XrHitResult)) != 0;
        }
      }
    }
  };
  for (const AbSample& s : rec) {
    const InternalState st = Reconstruct(s);
    count(ReplayProduction(cfg, st, s, nullptr), ReplayProduction(cfg, st, s, nullptr));
    count(ReplayTierT(cfg, st, s), ReplayTierT(cfg, st, s));
  }
  std::printf("[N2 synthetic] samples=%zu hits=%zu differing_between_two_replays=%zu\n", rec.size(), hits, diff);
  EXPECT_GT(hits, 0u);
  EXPECT_EQ(diff, 0u);
}

// ---- N3: resolution of the instrument = float32 round trip of the reconstruction (published values
// -> internal state -> replay) against the engine's own direct answers; tolerance >= 3 x resolution ----
TEST(HitAbNull, RoundTripResolutionTierT) {
  const DatasetCfg cfg = SyntheticCfg();
  const std::vector<AbSample> rec = SyntheticRecording(cfg, kSynthetic);
  const Stats st = ReplayAllT(cfg, rec, kTolT);
  const double resolution = std::max(st.max_pos_rel, st.max_dist_rel);
  std::printf("[N3T synthetic] %s resolution=%.3g tolerance=%.3g tolerance/resolution=%.3g | classes Z=%zu N=%zu E=%zu\n",
              Summary(st).c_str(), resolution, kTolT, resolution > 0 ? kTolT / resolution : INFINITY, st.t.zero,
              st.t.nonzero, st.t.nosource);
  EXPECT_EQ(st.queries, rec.size() * kQueriesPerSample);
  EXPECT_TRUE(st.ok());
  EXPECT_GT(st.t.zero, 0u);
  EXPECT_LE(3.0 * resolution, kTolT) << "tolerance < 3 x resolution: the measurement formula must change";
}

// The zero-depth class alone (the injection changed for it in v2): its own resolution, same 3x rule.
TEST(HitAbNull, RoundTripResolutionTierTZeroDepthClass) {
  const DatasetCfg cfg = SyntheticCfg();
  const std::vector<AbSample> rec = SyntheticOfKind(cfg, Kind::kZeroDepthFar, 6, 0x5EED0001u);
  const Stats st = ReplayAllT(cfg, rec, kTolT);
  const double resolution = std::max(st.max_pos_rel, st.max_dist_rel);
  std::printf("[N3T-Z synthetic zero-depth only] %s resolution=%.3g tolerance=%.3g tolerance/resolution=%.3g | classes Z=%zu N=%zu\n",
              Summary(st).c_str(), resolution, kTolT, resolution > 0 ? kTolT / resolution : INFINITY, st.t.zero,
              st.t.nonzero);
  EXPECT_TRUE(st.ok());
  EXPECT_EQ(st.t.nonzero, 0u);
  EXPECT_EQ(st.t.zero, st.queries);
  EXPECT_LE(3.0 * resolution, kTolT);
}

TEST(HitAbNull, RoundTripResolutionTierE) {
  const DatasetCfg cfg = SyntheticCfg();
  const std::vector<AbSample> rec = SyntheticRecording(cfg, kSynthetic);
  const Stats st = ReplayAllE(cfg, rec, kTolE);
  const double resolution = std::max(st.max_pos_rel, st.max_dist_rel);
  std::printf("[N3E synthetic] %s resolution=%.3g tolerance=%.3g tolerance/resolution=%.3g\n", Summary(st).c_str(),
              resolution, kTolE, resolution > 0 ? kTolE / resolution : INFINITY);
  EXPECT_TRUE(st.ok());
  EXPECT_LE(3.0 * resolution, kTolE) << "tolerance < 3 x resolution: the measurement formula must change";
  // The base pose itself: the engine's forward of the reconstructed state equals the recorded pose.
  double worst = 0;
  for (const AbSample& s : rec) {
    PublishedFrame base;
    ReplayProduction(cfg, Reconstruct(s), s, &base);
    const double dx = base.position.x - s.position.x, dy = base.position.y - s.position.y,
                 dz = base.position.z - s.position.z;
    worst = std::max(worst, std::sqrt(dx * dx + dy * dy + dz * dz) /
                                std::max(1.0, static_cast<double>(std::fabs(s.position.z))));
  }
  std::printf("[N3E synthetic] base pose round trip max rel error %.3g\n", worst);
  EXPECT_LE(worst, kBaseTol);
}

// ---- N4: the test-local model without a flag equals the production chain (mutant framework check;
// the model recomputes from the same state, so it agrees to the noise of tier E) ----
TEST(HitAbNull, ModelWithoutFlagEqualsProduction) {
  const DatasetCfg cfg = SyntheticCfg();
  const std::vector<AbSample> rec = SyntheticRecording(cfg, kSynthetic);
  const Stats st = ReplayModelAll(cfg, rec, Flags{});
  std::printf("[N4 synthetic] %s\n", Summary(st).c_str());
  EXPECT_TRUE(st.ok());
}

// ---- N5 (V2-2): negative control. The v1 injection (the camera position as the point) is what made tier T
// blind on zero-depth hits. With the fail-closed comparator it no longer passes silently: it is rejected as
// non-finite. The v2 injection on the same recording passes the comparator and keeps its detection power
// (M7Z..M11Z below). ----
TEST(HitAbNull, NegativeControlV1ZeroDepthInjectionIsRejectedAndV2IsNot) {
  const DatasetCfg cfg = SyntheticCfg();
  const std::vector<AbSample> rec = SyntheticOfKind(cfg, Kind::kZeroDepthFar, 4, 0x5EED0001u);
  Stats v1, v2;
  for (const AbSample& s : rec) {
    Compare(s, ReplayTierT(cfg, Reconstruct(s), s, nullptr, true), kTolT, &v1);
    Compare(s, ReplayTierT(cfg, Reconstruct(s), s, nullptr, false), kTolT, &v2);
  }
  std::printf("[N5 v1 injection on zero-depth hits] ok=%d %s\n", v1.ok() ? 1 : 0, Summary(v1).c_str());
  std::printf("[N5 v2 injection on zero-depth hits] ok=%d %s\n", v2.ok() ? 1 : 0, Summary(v2).c_str());
  EXPECT_FALSE(v1.ok());
  EXPECT_EQ(v1.nonfinite_actual, v1.queries) << "the v1 injection gives 0/0 for every zero-depth query";
  EXPECT_TRUE(v2.ok());
}

// ---- mutations: each model error must be detected (the baselines N3T / N3E pass on the same recording) ----
void ExpectDetected(const char* id, const Stats& st, std::initializer_list<const char*> expected_steps) {
  std::printf("[%s synthetic] %s\n", id, Summary(st).c_str());
  EXPECT_FALSE(st.ok()) << id << " not detected";
  for (const char* step : expected_steps) {
    EXPECT_GT(st.bad_steps.count(step), 0u) << id << ": step " << step << " not flagged";
  }
}

// Model-flag mutants (tier E comparator: they replace the glue chain by a wrong one).
TEST(HitAbMutation, M1OriginIgnored) {
  const DatasetCfg cfg = SyntheticCfg();
  Flags f;
  f.ignore_origin = true;
  ExpectDetected("M1", ReplayModelAll(cfg, SyntheticRecording(cfg, kSynthetic), f), {"origin-xz", "base0"});
}

TEST(HitAbMutation, M2FacingUsedAsAWholeSoPitchAndRollMatter) {
  const DatasetCfg cfg = SyntheticCfg();
  Flags f;
  f.full_facing = true;
  ExpectDetected("M2", ReplayModelAll(cfg, SyntheticRecording(cfg, kSynthetic), f),
                 {"pitch25", "roll20", "yaw40-pitch25-roll20"});
}

TEST(HitAbMutation, M3ScaleIgnored) {
  const DatasetCfg cfg = SyntheticCfg();
  Flags f;
  f.ignore_scale = true;
  ExpectDetected("M3", ReplayModelAll(cfg, SyntheticRecording(cfg, kSynthetic), f), {"scale-3.5", "base0"});
}

TEST(HitAbMutation, M4MirrorDoesNotNegateX) {
  const DatasetCfg cfg = SyntheticCfg();
  Flags f;
  f.mirror_sign = true;
  ExpectDetected("M4", ReplayModelAll(cfg, SyntheticRecording(cfg, kSynthetic), f), {"mirror", "lh-mirror-origin-yaw"});
}

TEST(HitAbMutation, M5MirrorDoesNotFlipTheQueryX) {
  const DatasetCfg cfg = SyntheticCfg();
  Flags f;
  f.mirror_no_query_flip = true;
  ExpectDetected("M5", ReplayModelAll(cfg, SyntheticRecording(cfg, kSynthetic), f), {"mirror", "lh-mirror-origin-yaw"});
}

TEST(HitAbMutation, M6HandednessIgnored) {
  const DatasetCfg cfg = SyntheticCfg();
  Flags f;
  f.ignore_handedness = true;
  // Right-handed steps change; the two left-handed steps (lefthanded, lh-mirror-origin-yaw) cannot.
  ExpectDetected("M6", ReplayModelAll(cfg, SyntheticRecording(cfg, kSynthetic), f), {"base0", "mirror", "origin-xz"});
}

// Input mutants of the production path: the engine side runs a call sequence that differs from the
// recorded one; the expectation stays. Run against both tiers, each with the error size it must see.
using Perturb = std::function<void(AbStep*)>;

Stats PerturbedT(const std::vector<AbSample>& rec, const Perturb& f) {
  const DatasetCfg cfg = SyntheticCfg();
  Stats st;
  for (const AbSample& s : rec) {
    AbSample engine_side = s;
    for (AbStep& step : engine_side.steps) f(&step);
    Compare(s, ReplayTierT(cfg, Reconstruct(s), engine_side, &st.t), kTolT, &st);
  }
  return st;
}

Stats PerturbedE(const std::vector<AbSample>& rec, const Perturb& f) {
  const DatasetCfg cfg = SyntheticCfg();
  Stats st;
  for (const AbSample& s : rec) {
    AbSample engine_side = s;
    for (AbStep& step : engine_side.steps) f(&step);
    Compare(s, ReplayProduction(cfg, Reconstruct(s), engine_side, nullptr), kTolE, &st);
  }
  return st;
}

Perturb OriginX(double dx) {
  return [dx](AbStep* s) { if (s->id == "origin-xz") s->calls[0].origin[0] += dx; };
}

Perturb YawTo(double deg) {
  return [deg](AbStep* s) {
    if (s->id != "yaw40") return;
    const c8::Quaternion q = AxisQuat(deg, 'y');
    s->calls[0].facing[0] = q.w();
    s->calls[0].facing[2] = q.y();
  };
}

Perturb ScaleTo(double y) {
  return [y](AbStep* s) { if (s->id == "scale-3.5") s->calls[0].origin[1] = y; };
}

Perturb DropMirror() {
  return [](AbStep* s) { for (AbCall& c : s->calls) c.mirror = c.mirror == 1 ? 0 : c.mirror; };
}

// M11 (V2-3): the handedness (axis) toggle is dropped: the engine stays right handed.
Perturb DropHandedness() {
  return [](AbStep* s) { for (AbCall& c : s->calls) c.left = c.left == 1 ? 0 : c.left; };
}

TEST(HitAbMutation, M7OriginOffset) {
  const std::vector<AbSample> rec = SyntheticRecording(SyntheticCfg(), kSynthetic);
  ExpectDetected("M7T origin.x +0.001", PerturbedT(rec, OriginX(0.001)), {"origin-xz"});
  ExpectDetected("M7E origin.x +0.02", PerturbedE(rec, OriginX(0.02)), {"origin-xz"});
}

TEST(HitAbMutation, M8YawOff) {
  const std::vector<AbSample> rec = SyntheticRecording(SyntheticCfg(), kSynthetic);
  ExpectDetected("M8T yaw +0.05 deg", PerturbedT(rec, YawTo(40.05)), {"yaw40"});
  ExpectDetected("M8E yaw +0.5 deg", PerturbedE(rec, YawTo(40.5)), {"yaw40"});
}

TEST(HitAbMutation, M9ScaleOff) {
  const std::vector<AbSample> rec = SyntheticRecording(SyntheticCfg(), kSynthetic);
  ExpectDetected("M9T scale +0.1 %", PerturbedT(rec, ScaleTo(3.5035)), {"scale-3.5"});
  ExpectDetected("M9E scale +1 %", PerturbedE(rec, ScaleTo(3.535)), {"scale-3.5"});
}

TEST(HitAbMutation, M10MirrorToggleDropped) {
  const std::vector<AbSample> rec = SyntheticRecording(SyntheticCfg(), kSynthetic);
  ExpectDetected("M10T mirror dropped", PerturbedT(rec, DropMirror()), {"mirror", "lh-mirror-origin-yaw"});
  ExpectDetected("M10E mirror dropped", PerturbedE(rec, DropMirror()), {"mirror", "lh-mirror-origin-yaw"});
}

TEST(HitAbMutation, M11HandednessToggleDropped) {
  const std::vector<AbSample> rec = SyntheticRecording(SyntheticCfg(), kSynthetic);
  ExpectDetected("M11T handedness dropped", PerturbedT(rec, DropHandedness()), {"lefthanded", "lh-mirror-origin-yaw"});
  ExpectDetected("M11E handedness dropped", PerturbedE(rec, DropHandedness()), {"lefthanded", "lh-mirror-origin-yaw"});
}

// M12 (V2-1): the engine answers with a non-finite value (here injected into the engine side output of the
// production replay; the engine itself is untouched). Every poisoned query must be a violation and the
// aggregate must show it.
enum class Poison { kPositionNaN, kDistanceNaN, kDistanceInf };

std::vector<StepOut> Poisoned(std::vector<StepOut> out, Poison p, size_t* poisoned) {
  size_t n = 0;
  for (StepOut& so : out) {
    for (std::vector<XrHitResult>& q : so.queries) {
      for (XrHitResult& h : q) {
        if (n++ % 7 != 0) continue;
        ++*poisoned;
        if (p == Poison::kPositionNaN) h.position.x = NAN;
        if (p == Poison::kDistanceNaN) h.distance = NAN;
        if (p == Poison::kDistanceInf) h.distance = INFINITY;
      }
    }
  }
  return out;
}

Stats PoisonedRun(const std::vector<AbSample>& rec, bool tier_t, Poison p, size_t* poisoned) {
  const DatasetCfg cfg = SyntheticCfg();
  Stats st;
  for (const AbSample& s : rec) {
    const std::vector<StepOut> out = tier_t ? ReplayTierT(cfg, Reconstruct(s), s) : ReplayProduction(cfg, Reconstruct(s), s, nullptr);
    Compare(s, Poisoned(out, p, poisoned), tier_t ? kTolT : kTolE, &st);
  }
  return st;
}

TEST(HitAbMutation, M12NonFiniteEngineOutput) {
  const std::vector<AbSample> rec = SyntheticRecording(SyntheticCfg(), kSynthetic);
  const struct {
    const char* name;
    bool tier_t;
    Poison p;
  } runs[] = {{"M12T position NaN", true, Poison::kPositionNaN}, {"M12T distance NaN", true, Poison::kDistanceNaN},
              {"M12T distance inf", true, Poison::kDistanceInf}, {"M12E position NaN", false, Poison::kPositionNaN},
              {"M12E distance NaN", false, Poison::kDistanceNaN}, {"M12E distance inf", false, Poison::kDistanceInf}};
  for (const auto& run : runs) {
    size_t poisoned = 0;
    const Stats st = PoisonedRun(rec, run.tier_t, run.p, &poisoned);
    ExpectDetected(run.name, st, {"base0", "mirror"});
    EXPECT_EQ(st.nonfinite_actual, poisoned) << run.name << ": every poisoned hit must be counted";
    EXPECT_FALSE(std::isfinite(std::max(st.max_pos_rel, st.max_dist_rel))) << run.name;
  }
}

// V2-2 mutants on the zero-depth class only (every query of the recording is a zero-depth query): the v1
// tier T could not see M7 / M10 here (reviewer probe P2). Errors big enough for the camera position
// (M7 0.001, M10 mirror, M9 0.1 %, M8 0.05 deg, M11 handedness).
TEST(HitAbMutation, M7M8M9M10M11OnTheZeroDepthClass) {
  const DatasetCfg cfg = SyntheticCfg();
  const std::vector<AbSample> rec = SyntheticOfKind(cfg, Kind::kZeroDepthFar, 6, 0x5EED0001u);
  const RecordingClasses c = ClassifyRecording(rec);
  EXPECT_EQ(c.zero, c.queries);
  const Stats base = ReplayAllT(cfg, rec, kTolT);
  EXPECT_TRUE(base.ok()) << Summary(base);
  ExpectDetected("M7Z origin.x +0.001", PerturbedT(rec, OriginX(0.001)), {"origin-xz"});
  ExpectDetected("M8Z yaw +0.05 deg", PerturbedT(rec, YawTo(40.05)), {"yaw40"});
  ExpectDetected("M9Z scale +0.1 %", PerturbedT(rec, ScaleTo(3.5035)), {"scale-3.5"});
  ExpectDetected("M10Z mirror dropped", PerturbedT(rec, DropMirror()), {"mirror", "lh-mirror-origin-yaw"});
  ExpectDetected("M11Z handedness dropped", PerturbedT(rec, DropHandedness()), {"lefthanded", "lh-mirror-origin-yaw"});
}

// ---- HitAbOriginal: properties of the ORIGINAL recording (no engine involved) ----
void ExpectOriginal(const DatasetCfg& cfg) {
  const std::vector<AbSample> rec = ReadAb(cfg.tsv);
  const OriginalInvariants r = CheckOriginal(rec);
  std::printf("%s\n", OriginalLine(cfg.name, r).c_str());
  EXPECT_GE(r.with_hits, cfg.reg->min_samples_with_hits);
  EXPECT_EQ(r.null_identity_diff, 0u);
  EXPECT_EQ(r.restore_diff, 0u);
  EXPECT_EQ(r.pitch_diff, 0u);
  EXPECT_EQ(r.roll_diff, 0u);
  EXPECT_EQ(r.combo_diff, 0u);
  EXPECT_EQ(r.lefthanded_diff, 0u);
  EXPECT_EQ(r.mirror_pair_diff, 0u);
  EXPECT_EQ(r.mirror_pairs, r.queries);
  // The recording discriminates: an origin / scale / yaw change moves (almost) every hit.
  EXPECT_LT(r.origin_same, r.queries / 100 + 1);
  EXPECT_LT(r.scale_same, r.queries / 100 + 1);
  EXPECT_LT(r.yaw_same, r.queries / 100 + 1);
  EXPECT_TRUE(OriginalHolds(r, cfg.reg->min_samples_with_hits));
}

TEST(HitAbOriginal, Hab1FlipS2) { ExpectOriginal(Ab1()); }
TEST(HitAbOriginal, Hab2FlipS2Crop360x640) { ExpectOriginal(Ab2()); }
TEST(HitAbOriginal, Hab3TumRoom1) { ExpectOriginal(Ab3()); }

// ---- HitAbIntegrity: the gate's own checks (V2-4), engine free ----
// The registered recordings are the ones the code constants name, and their classes are the registered ones.
void ExpectRegistered(const DatasetCfg& cfg) {
  std::string data;
  ASSERT_TRUE(ReadWholeFile(cfg.tsv, &data)) << cfg.tsv;
  EXPECT_EQ(Sha256Hex(data), cfg.reg->tsv_sha256) << cfg.tsv;
  const std::vector<AbSample> rec = ReadAb(cfg.tsv);
  const RecordingClasses c = ClassifyRecording(rec);
  std::printf("%s\n", ClassLine(cfg.name, c).c_str());
  EXPECT_EQ(c.samples, cfg.reg->samples);
  EXPECT_EQ(c.queries, cfg.reg->queries);
  EXPECT_EQ(c.with_hits, cfg.reg->with_hits);
  EXPECT_EQ(c.zero, cfg.reg->z);
  EXPECT_EQ(c.nonzero, cfg.reg->n);
  EXPECT_EQ(c.nosource, cfg.reg->e);
  EXPECT_EQ(c.zero + c.nonzero + c.nosource, c.queries);
}

TEST(HitAbIntegrity, RegisteredRecordingHab1) { ExpectRegistered(Ab1()); }
TEST(HitAbIntegrity, RegisteredRecordingHab2) { ExpectRegistered(Ab2()); }
TEST(HitAbIntegrity, RegisteredRecordingHab3) { ExpectRegistered(Ab3()); }

TEST(HitAbIntegrity, Sha256KnownAnswers) {
  EXPECT_EQ(Sha256Hex(""), "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
  EXPECT_EQ(Sha256Hex("abc"), "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  EXPECT_EQ(Sha256Hex("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq"),
            "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");
  // padding boundaries: 55 | 56 | 57 | 63 | 64 | 65 | 119 | 120 bytes (hashlib values)
  EXPECT_EQ(Sha256Hex(std::string(55, 'a')), "9f4390f8d30c2dd92ec9f095b65e2b9ae9b0a925a5258e241c9f1e910f734318");
  EXPECT_EQ(Sha256Hex(std::string(56, 'a')), "b35439a4ac6f0948b6d6f9e3c6af0f5f590ce20f1bde7090ef7970686ec6738a");
  EXPECT_EQ(Sha256Hex(std::string(57, 'a')), "f13b2d724659eb3bf47f2dd6af1accc87b81f09f59f2b75e5c0bed6589dfe8c6");
  EXPECT_EQ(Sha256Hex(std::string(63, 'a')), "7d3e74a05d7db15bce4ad9ec0658ea98e3f06eeecf16b4c6fff2da457ddc2f34");
  EXPECT_EQ(Sha256Hex(std::string(64, 'a')), "ffe054fe7ae0cb6dc65c3af9b61d5209f439851db43d0ba5997337df154668eb");
  EXPECT_EQ(Sha256Hex(std::string(65, 'a')), "635361c48bb9eab14198e76ea8ab7f1a41685d6ad62aa9146d301d4f17eb0ae0");
  EXPECT_EQ(Sha256Hex(std::string(119, 'a')), "31eba51c313a5c08226adf18d4a359cfdfd8d2e816b13f4af952f7ea6584dcfb");
  EXPECT_EQ(Sha256Hex(std::string(120, 'a')), "2f3d335432c70b580af0e8e1b3674a7c020d683aa5f73aaaedfdc55af904c21c");
  EXPECT_EQ(Sha256Hex(std::string(1000000, 'a')), "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");
}

// The held binary pins itself by the sha256 of /proc/self/exe: that path must give the executable's bytes
// (compare the printed value with `sha256sum` of this binary).
TEST(HitAbIntegrity, SelfExecutableIsReadableAndHashedLikeTheFile) {
  std::string a, b;
  ASSERT_TRUE(ReadWholeFile("/proc/self/exe", &a));
  ASSERT_TRUE(ReadWholeFile("/proc/self/exe", &b));
  EXPECT_GT(a.size(), 1000000u);
  const std::string sha = Sha256Hex(a);
  EXPECT_EQ(sha, Sha256Hex(b));
  std::printf("[N6 self executable] bytes=%zu sha256=%s\n", a.size(), sha.c_str());
}

// The start of the first real run (StartRelease) on synthetic files: no engine, no original data. The release logic itself (every bypass of
// protocol §12.5) is the conformance of the shared device (tests/support/sr_held_conformance.cc, 198 cases); what this file adds is the
// ORDER of the checks: a refusal of any kind leaves no marker, so it does not use up the one run of the gate.
struct ReleaseFixture {
  std::string root, prereg, exe, tsv, marker;
  ReleaseInputs in;
};

void PutFile(const std::string& path, const std::string& body) {
  std::filesystem::create_directories(std::filesystem::path(path).parent_path());
  std::ofstream f(path, std::ios::binary);
  f << body;
}

bool Exists(const std::string& path) {
  struct stat st;
  return ::lstat(path.c_str(), &st) == 0;
}

ReleaseFixture MakeReleaseFixture(const std::string& tag, int version) {
  ReleaseFixture f;
  f.root = (std::filesystem::temp_directory_path() / ("sr_rm3b_release_" + tag + "_" + std::to_string(getpid()))).string();
  std::filesystem::remove_all(f.root);
  f.prereg = f.root + "/prereg";
  f.exe = f.root + "/exe.bin";
  f.tsv = f.root + "/hit_ab.tsv";
  const std::string v = std::to_string(version);
  const std::string exe_bytes = "the held binary", tsv_bytes = "a recording", gate_md = "the gate file", review_md = "the review";
  PutFile(f.exe, exe_bytes);
  PutFile(f.tsv, tsv_bytes);
  PutFile(f.prereg + "/g1.v" + v + ".md", gate_md);
  PutFile(f.prereg + "/g1.v" + v + ".review.md", review_md);
  auto manifest = [&](const std::string& designation_sha) {
    return "{\"gate_id\":\"g1\",\"version\":" + v + ",\"file\":\"g1.v" + v + ".md\",\"sha256\":\"" + Sha256Hex(gate_md) +
           "\",\"created_at\":\"2026-01-01T00:00:00+09:00\",\"held\":{\"kinds\":[\"main\"],\"pins\":{\"exe.bin\":\"" + Sha256Hex(exe_bytes) +
           "\"},\"inputs\":{},\"seed\":false},\"review\":{\"reviewer\":\"test\",\"at\":\"2026-01-02T00:00:00+09:00\",\"file\":\"g1.v" + v +
           ".review.md\",\"sha256\":\"" + Sha256Hex(review_md) + "\",\"designation_file\":\"g1.v" + v +
           ".review-designation.json\",\"designation_file_sha256\":\"" + designation_sha + "\"},\"first_real_run_at\":null}";
  };
  // The designation pins the manifest without its review block, so it can be computed before the designation's own sha256 is known.
  sh::Json parsed;
  std::string err, canon;
  EXPECT_TRUE(sh::ParseJson(manifest(std::string(64, '0')), &parsed, &err)) << err;
  EXPECT_TRUE(sh::CanonManifestSha(parsed, &canon));
  const std::string designation = "{\"gate\":\"g1.v" + v + "\",\"first_real_run_permitted\":true,\"pinned\":{\"gate_md_sha256\":\"" +
                                  Sha256Hex(gate_md) + "\",\"review_file_sha256\":\"" + Sha256Hex(review_md) +
                                  "\",\"manifest_before_review_sha256\":\"" + canon + "\"}}";
  PutFile(f.prereg + "/g1.v" + v + ".review-designation.json", designation);
  PutFile(f.prereg + "/g1.v" + v + ".manifest.json", manifest(Sha256Hex(designation)));
  f.marker = f.prereg + "/g1.v" + v + ".first-run.main.marker";
  f.in.env_value = Sha256Hex(designation);
  f.in.prereg_dir = f.prereg;
  f.in.gate_name = "g1.v" + v;
  f.in.pins = {f.exe};
  f.in.data_files = {{f.tsv, Sha256Hex(tsv_bytes)}};  // the "code constant" of the recording
  return f;
}

TEST(HitAbIntegrity, ReleaseStartsOnceAndTheMarkerIsExclusive) {
  const ReleaseFixture f = MakeReleaseFixture("once", 1);
  const ReleaseResult first = StartRelease(f.in);
  EXPECT_TRUE(first.ok) << first.code << ": " << first.reason;
  EXPECT_TRUE(Exists(f.marker));
  EXPECT_GE(first.log.size(), 5u);
  for (const std::string& l : first.log) std::printf("[N6 release] %s\n", l.c_str());
  const ReleaseResult second = StartRelease(f.in);
  EXPECT_FALSE(second.ok);
  EXPECT_EQ(second.code, "MARKER_EXISTS");
  std::printf("[N6 release] second start: %s\n", second.code.c_str());
}

// A recording that differs from its registered sha256 refuses the run BEFORE the marker exists; after the recording is repaired the
// same gate still starts (the refusal did not use up its one run).
TEST(HitAbIntegrity, RecordingMismatchRefusesBeforeTheMarkerAndKeepsTheRun) {
  const ReleaseFixture f = MakeReleaseFixture("recording", 1);
  PutFile(f.tsv, "a recording, edited");
  const ReleaseResult refused = StartRelease(f.in);
  EXPECT_FALSE(refused.ok);
  EXPECT_EQ(refused.code, "RECORDING");
  EXPECT_NE(refused.reason.find("recording sha256 differs"), std::string::npos) << refused.reason;
  EXPECT_FALSE(Exists(f.marker)) << "the refusal created the one-run marker";
  std::printf("[N6 release] edited recording: %s, marker %s\n", refused.code.c_str(), Exists(f.marker) ? "CREATED" : "absent");
  std::filesystem::remove(f.tsv);
  const ReleaseResult missing = StartRelease(f.in);
  EXPECT_FALSE(missing.ok);
  EXPECT_EQ(missing.code, "RECORDING");
  EXPECT_FALSE(Exists(f.marker));
  PutFile(f.tsv, "a recording");
  const ReleaseResult started = StartRelease(f.in);
  EXPECT_TRUE(started.ok) << started.code << ": " << started.reason;
  EXPECT_TRUE(Exists(f.marker));
}

// Any other refusal (env, gate designation, pinned binary) also happens before the marker.
TEST(HitAbIntegrity, ReleaseRefusalLeavesNoMarker) {
  struct Case {
    const char* name;
    const char* code;
    std::function<void(ReleaseFixture*)> damage;
  };
  const Case cases[] = {
      {"environment value not set", "ENV_MISSING", [](ReleaseFixture* f) { f->in.env_value = ""; }},
      {"environment value is any other string (v1 bypass)", "ENV_MISMATCH", [](ReleaseFixture* f) { f->in.env_value = "x"; }},
      {"gate not designated as <gate_id>.v<N>", "ARGS", [](ReleaseFixture* f) { f->in.gate_name = ""; }},
      {"gate designated with a leading zero version", "ARGS", [](ReleaseFixture* f) { f->in.gate_name = "g1.v01"; }},
      {"another gate designated", "MANIFEST_UNREADABLE", [](ReleaseFixture* f) { f->in.gate_name = "g1.v99"; }},
      {"the held binary differs from the registered one", "PIN_MISMATCH", [](ReleaseFixture* f) { PutFile(f->exe, "rebuilt"); }},
  };
  int n = 0;
  for (const Case& c : cases) {
    ReleaseFixture f = MakeReleaseFixture("refuse" + std::to_string(n++), 1);
    c.damage(&f);
    const ReleaseResult refused = StartRelease(f.in);
    EXPECT_FALSE(refused.ok) << c.name;
    EXPECT_EQ(refused.code, c.code) << c.name << ": " << refused.reason;
    EXPECT_FALSE(Exists(f.marker)) << c.name << ": the refusal created the one-run marker";
    std::printf("[N6 release] %-52s %s\n", c.name, refused.code.c_str());
  }
}

#endif  // SR_HITAB_HELD

#ifdef SR_HITAB_HELD

// ------------------------------------------------------------------------------------------------
// The gate. Fails closed (V2-4): before the first engine answer, StartRelease must pass (see its comment), and
// the recording must be the registered one (sha256, sample / query / class counts) and satisfy the original-side
// properties O-1..O-6. The results of those checks head the output (real-run.log).

ReleaseResult g_release;

class GateEnvironment : public ::testing::Environment {
 public:
  void SetUp() override {
    ReleaseInputs in;
    const char* env = std::getenv("SLAM_REFACT_RM3B_REVIEWER");
    in.env_value = env == nullptr ? "" : env;
    const char* gate = std::getenv("SLAM_REFACT_RM3B_GATE");  // <gate_id>.v<N>, designated by the caller
    in.gate_name = gate == nullptr ? "" : gate;
    const char* prereg = std::getenv("SLAM_REFACT_RM3B_PREREG_DIR");
    in.prereg_dir = prereg != nullptr && *prereg != '\0' ? prereg : kPreregDir;
    in.pins = {sh::SelfExe()};
    for (const DatasetCfg& c : {Ab1(), Ab2(), Ab3()}) in.data_files.push_back({c.tsv, c.reg->tsv_sha256});
    g_release = StartRelease(in);
    for (const std::string& line : g_release.log) std::printf("%s\n", line.c_str());
    std::fflush(stdout);
  }
};

const ::testing::Environment* const kGateEnvironment = ::testing::AddGlobalTestEnvironment(new GateEnvironment);

void RunGate(const char* tier, const DatasetCfg& cfg) {
  if (!g_release.ok) {
    FAIL() << g_release.reason;
  }
  const std::vector<AbSample> rec = ReadAb(cfg.tsv);
  const Registered& reg = *cfg.reg;
  const RecordingClasses c = ClassifyRecording(rec);
  std::printf("%s\n", ClassLine(cfg.name, c).c_str());
  ASSERT_EQ(c.samples, reg.samples);
  ASSERT_EQ(c.queries, reg.queries);
  ASSERT_EQ(c.with_hits, reg.with_hits);
  ASSERT_EQ(c.zero, reg.z);
  ASSERT_EQ(c.nonzero, reg.n);
  ASSERT_EQ(c.nosource, reg.e);
  const OriginalInvariants o = CheckOriginal(rec);
  std::printf("%s\n", OriginalLine(cfg.name, o).c_str());
  if (!OriginalHolds(o, reg.min_samples_with_hits)) {
    FAIL() << "original-side property O-1..O-6 violated: the recording device is at fault, no engine comparison";
  }
  const bool t = std::string(tier) == "T";
  const Stats st = t ? ReplayAllT(cfg, rec, kTolT) : ReplayAllE(cfg, rec, kTolE);
  std::printf("[G-HAB-%s %s] samples=%zu tolerance=%.3g %s", tier, cfg.name, rec.size(), t ? kTolT : kTolE,
              Summary(st).c_str());
  if (t) std::printf(" | tierT_classes zero_depth=%zu nonzero=%zu no_source=%zu", st.t.zero, st.t.nonzero, st.t.nosource);
  std::printf("\n");
  EXPECT_EQ(st.queries, reg.queries);
  EXPECT_EQ(st.count_mismatch, 0u);
  EXPECT_EQ(st.type_mismatch, 0u);
  EXPECT_EQ(st.pos_fail, 0u);
  EXPECT_EQ(st.dist_fail, 0u);
  EXPECT_EQ(st.rot_fail, 0u);
  EXPECT_EQ(st.nonfinite_actual, 0u);
  EXPECT_EQ(st.nonfinite_expected, 0u);
  EXPECT_TRUE(std::isfinite(st.max_pos_rel) && std::isfinite(st.max_dist_rel));
  if (t) {
    // Registered minimum of discriminating queries (gate file section 3): all non-zero-depth queries are
    // evaluated, and the zero-depth ones are evaluated through the underflow injection (not skipped).
    EXPECT_EQ(st.t.nonzero, reg.n);
    EXPECT_EQ(st.t.zero, reg.z);
    EXPECT_EQ(st.t.nosource, reg.e);
  }
}

TEST(HitAbHeld, GateTHab1FlipS2) { RunGate("T", Ab1()); }
TEST(HitAbHeld, GateTHab2FlipS2Crop360x640) { RunGate("T", Ab2()); }
TEST(HitAbHeld, GateTHab3TumRoom1) { RunGate("T", Ab3()); }
TEST(HitAbHeld, GateEHab1FlipS2) { RunGate("E", Ab1()); }
TEST(HitAbHeld, GateEHab2FlipS2Crop360x640) { RunGate("E", Ab2()); }
TEST(HitAbHeld, GateEHab3TumRoom1) { RunGate("E", Ab3()); }

#endif  // SR_HITAB_HELD

}  // namespace
}  // namespace slam_refact
