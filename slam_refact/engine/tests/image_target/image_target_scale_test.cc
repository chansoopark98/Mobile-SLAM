// US-015: gate `it-scale-gate` (build/slam-refact-full-20261019/preregistration/it-scale-gate.v1.md). It replaces the
// role of IT-P2 ("scale == 0.5", a public-source observation) by tests of the scale and world pose of a located image
// target against ground truth that does not come from the engine: virtual feature photos of a planar target are taken
// from an analytic camera orbit, and the VIO extrinsic (tam) the tracker receives is scripted from the same orbit.
//
// WHAT IS UNDER TEST: the engine's use of the public reference runtime scale logic (ImageTrackingState::computeScale, the 0.8 / 0.2
// smoothing, the world pose latch) through VioTracker::TrackWithTargets (tam hand-over, yaw offset, applyToLocatedImages).
// It is NOT a parity test with the closed binary 994 (scale-solver, image-world-tracker); that part is not reconstructed
// (docs/spec/09-image-world-tracker.md, docs/design/blocked-registry.md).
//
// DEV and REGISTERED data are separate (preregistration-protocol.md section 12.5 / 12.6): the tests named ItsDev.* read the
// dev dataset only (its_gate_data.py --variant dev) and refuse a directory with the registered tree sha256. The test named
// ItsGate.* reads the registered dataset (derived from the coordinator seed) and refuses to start unless its_held.h releases
// it. Nothing in this file runs the registered data before that.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "c8/geometry/egomotion.h"
#include "c8/hmatrix.h"
#include "c8/pixels/gr8-pyramid.h"
#include "config/params.h"
#include "its_held.h"
#include "image_target/image_target_spec.h"
#include "image_target/target_loader.h"
#include "reality/engine/imagedetection/detection-image-tracker.h"
#include "test_util.h"
#include "vio_tracker/tracker_modules.h"
#include "vio_tracker/vio_tracker.h"
#include "vio_tracker/vio_tracker_internal.h"

#ifndef SR_ITS_DEV_DIR
#error "SR_ITS_DEV_DIR (dev dataset directory) must be defined by CMake"
#endif
#ifndef SR_ITS_REG_DIR
#error "SR_ITS_REG_DIR (registered dataset directory) must be defined by CMake"
#endif
#ifndef SR_ITS_PREREG_DIR
#error "SR_ITS_PREREG_DIR must be defined by CMake"
#endif

namespace slam_refact {
namespace {

using image_target::testing::SyntheticTarget;
using image_target::testing::TestK;
using image_target::testing::TestPinholeK;
using vio_tracker::TrackerModules;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;

// ---- gate constants (fixed in it-scale-gate.v1.md section 3, before any registered data exists) ----
constexpr int kWarmup = 60;             // frames; the 0.8 / 0.2 smoothing needs >= 17 updates from the largest initial error
constexpr int kMinSamples = 40;         // frames judged per scenario (last_seen == 0, frame >= kWarmup)
constexpr double kMinCoverage = 0.95;   // fraction of the frames >= kWarmup that carry last_seen == 0
constexpr double kPriorScale = 0.5;     // ImageTrackingState::scaleGuess_ initial value (public source)
constexpr double kBaselineThreshold = 3e-2;  // computeScale: n > 3e-2 (tam units, public source)
// tolerances
constexpr double kTolScale = 0.02;      // S1: |scale / k - 1|
constexpr double kTolPos = 0.02;        // S2: |p - p_gt| / k
constexpr double kTolRotDeg = 0.5;      // S2: rotation error (degrees)
constexpr double kTolSize = 0.02;       // S2: |width / (w_model k) - 1| and the height
constexpr double kTolStationary = 0.02; // S3: max |p - mean p| / k
constexpr double kTolPairRel = 1e-3;    // S4 / S5: relative scale and position deviation of a metamorphic pair
constexpr double kTolPairRotDeg = 0.05; // S4 / S5: rotation deviation (degrees)
constexpr double kTolInitial = 1e-6;    // S0: |scale / 0.5 - 1|
// representation floors (one float32 ulp of the quantity): scale and size relative, position relative to k, angle in degrees
constexpr double kUlpRel = 1.1920929e-7;
constexpr double kUlpDeg = 6.0e-5;

std::string Fmt(const char* f, ...) {
  char buf[1024];
  va_list ap;
  va_start(ap, f);
  std::vsnprintf(buf, sizeof(buf), f, ap);
  va_end(ap);
  return buf;
}

// ---------------------------------------------------------------------------------------------------------------
// scenario table
// ---------------------------------------------------------------------------------------------------------------
struct Scenario {
  std::string id, kind, pair_of;
  double k = 1, rho = 1.3, span_deg = 30, speed = 0.05, tilt_deg = 0, tilt_az_deg = 0, height_ratio = 0.1;
  double world_yaw_deg = 0, world_pitch_deg = 0, world_roll_deg = 0, wx = 0, wy = 0, wz = 0, noise_px = 0, pair_c = 0;
  uint32_t target_seed = 1;
  int n_frames = 100, loss_start = 0, loss_len = 0;
};

struct Dataset {
  std::string dir, variant;
  std::vector<Scenario> rows;
  const Scenario* Find(const std::string& id) const {
    for (const Scenario& s : rows) {
      if (s.id == id) return &s;
    }
    return nullptr;
  }
};

std::vector<std::string> Split(const std::string& line, char sep) {
  std::vector<std::string> out;
  std::string cur;
  for (char c : line) {
    if (c == sep) {
      out.push_back(cur);
      cur.clear();
    } else {
      cur.push_back(c);
    }
  }
  out.push_back(cur);
  return out;
}

bool LoadDataset(const std::string& dir, Dataset* ds, std::string* why) {
  ds->dir = dir;
  std::string variant;
  if (!its_held::ReadFile(dir + "/variant.txt", &variant)) {
    *why = "no variant.txt in " + dir;
    return false;
  }
  while (!variant.empty() && (variant.back() == '\n' || variant.back() == ' ')) variant.pop_back();
  ds->variant = variant;
  std::string text;
  if (!its_held::ReadFile(dir + "/scenarios.tsv", &text)) {
    *why = "no scenarios.tsv in " + dir;
    return false;
  }
  std::istringstream in(text);
  std::string line;
  std::getline(in, line);
  const std::vector<std::string> header = Split(line, '\t');
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    const std::vector<std::string> c = Split(line, '\t');
    if (c.size() != header.size()) {
      *why = "bad row: " + line;
      return false;
    }
    std::map<std::string, std::string> m;
    for (size_t i = 0; i < c.size(); ++i) m[header[i]] = c[i];
    Scenario s;
    s.id = m["id"];
    s.kind = m["kind"];
    s.pair_of = m["pair_of"];
    s.k = std::stod(m["k"]);
    s.target_seed = static_cast<uint32_t>(std::stoul(m["target_seed"]));
    s.rho = std::stod(m["rho"]);
    s.span_deg = std::stod(m["span_deg"]);
    s.speed = std::stod(m["speed"]);
    s.n_frames = std::stoi(m["n_frames"]);
    s.tilt_deg = std::stod(m["tilt_deg"]);
    s.tilt_az_deg = std::stod(m["tilt_az_deg"]);
    s.height_ratio = std::stod(m["height_ratio"]);
    s.world_yaw_deg = std::stod(m["world_yaw_deg"]);
    s.world_pitch_deg = std::stod(m["world_pitch_deg"]);
    s.world_roll_deg = std::stod(m["world_roll_deg"]);
    s.wx = std::stod(m["wx"]);
    s.wy = std::stod(m["wy"]);
    s.wz = std::stod(m["wz"]);
    s.loss_start = std::stoi(m["loss_start"]);
    s.loss_len = std::stoi(m["loss_len"]);
    s.noise_px = std::stod(m["noise_px"]);
    s.pair_c = std::stod(m["pair_c"]);
    ds->rows.push_back(s);
  }
  return !ds->rows.empty();
}

// ---------------------------------------------------------------------------------------------------------------
// analytic geometry (double precision; converted to c8::HMatrix at the end)
// ---------------------------------------------------------------------------------------------------------------
struct Rigid {
  double R[3][3];
  double t[3];
};

Rigid Mul(const Rigid& a, const Rigid& b) {
  Rigid o;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      o.R[i][j] = a.R[i][0] * b.R[0][j] + a.R[i][1] * b.R[1][j] + a.R[i][2] * b.R[2][j];
    }
    o.t[i] = a.R[i][0] * b.t[0] + a.R[i][1] * b.t[1] + a.R[i][2] * b.t[2] + a.t[i];
  }
  return o;
}

Rigid RotY(double deg) {
  const double c = std::cos(deg * kDeg), s = std::sin(deg * kDeg);
  return Rigid{{{c, 0, s}, {0, 1, 0}, {-s, 0, c}}, {0, 0, 0}};
}
Rigid RotX(double deg) {
  const double c = std::cos(deg * kDeg), s = std::sin(deg * kDeg);
  return Rigid{{{1, 0, 0}, {0, c, -s}, {0, s, c}}, {0, 0, 0}};
}
Rigid RotZ(double deg) {
  const double c = std::cos(deg * kDeg), s = std::sin(deg * kDeg);
  return Rigid{{{c, -s, 0}, {s, c, 0}, {0, 0, 1}}, {0, 0, 0}};
}
Rigid RotAxis(double ax, double ay, double az, double deg) {  // Rodrigues, (ax, ay, az) unit
  const double c = std::cos(deg * kDeg), s = std::sin(deg * kDeg), v = 1 - c;
  return Rigid{{{c + ax * ax * v, ax * ay * v - az * s, ax * az * v + ay * s},
                {ay * ax * v + az * s, c + ay * ay * v, ay * az * v - ax * s},
                {az * ax * v - ay * s, az * ay * v + ax * s, c + az * az * v}},
               {0, 0, 0}};
}

c8::HMatrix ToHMatrix(const Rigid& p) {
  const auto f = [](double v) { return static_cast<float>(v); };
  return c8::HMatrix{{f(p.R[0][0]), f(p.R[0][1]), f(p.R[0][2]), f(p.t[0])},
                     {f(p.R[1][0]), f(p.R[1][1]), f(p.R[1][2]), f(p.t[1])},
                     {f(p.R[2][0]), f(p.R[2][1]), f(p.R[2][2]), f(p.t[2])},
                     {0.f, 0.f, 0.f, 1.f}};
}

double Tri(double u) {  // 0 -> 0, 1/4 -> 1, 3/4 -> -1, 1 -> 0 (constant |slope| 4)
  u -= std::floor(u);
  return u < 0.25 ? 4 * u : (u < 0.75 ? 2 - 4 * u : 4 * u - 4);
}

// World transform of the target frame (G): Ry(yaw) Rx(pitch) Rz(roll), translation w.
Rigid TargetInWorld(const Scenario& s) {
  Rigid g = Mul(Mul(RotY(s.world_yaw_deg), RotX(s.world_pitch_deg)), RotZ(s.world_roll_deg));
  g.t[0] = s.wx;
  g.t[1] = s.wy;
  g.t[2] = s.wz;
  return g;
}

// Camera pose in the TARGET frame (target plane z = 0, camera at z < 0 looking at +z; c8 camera axes x right, y up,
// z forward, columns [right, up, forward]): the public tests place the camera the same way (detection-image-tracker-test.cc).
Rigid CameraInTarget(const Scenario& s, int i) {
  const double r0 = s.rho * s.k;
  const double span = s.span_deg * kDeg;
  const double omega = s.speed / r0;
  // Orbit angle travelled (rad). Every modulation below is tied to it, so the per-frame step of the camera is proportional
  // to `speed` (a "slow" scenario really is slow).
  const double x = omega * i;
  const double theta = span * Tri(x / (4 * span));
  const double r = r0 * (1 + 0.06 * std::sin(2.4 * x + 0.7));
  const double h = s.height_ratio * r0 * std::sin(1.3 * x + 1.1);
  double c[3] = {r * std::sin(theta), h, -r * std::cos(theta)};
  if (s.tilt_deg != 0) {  // tilt the orbit about an axis in the target plane
    const Rigid q = RotAxis(std::cos(s.tilt_az_deg * kDeg), std::sin(s.tilt_az_deg * kDeg), 0.0, s.tilt_deg);
    const double v[3] = {c[0], c[1], c[2]};
    for (int a = 0; a < 3; ++a) c[a] = q.R[a][0] * v[0] + q.R[a][1] * v[1] + q.R[a][2] * v[2];
  }
  double f[3] = {-c[0], -c[1], -c[2]};  // forward = towards the target origin
  const double fn = std::sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
  for (double& v : f) v /= fn;
  const double up[3] = {0, 1, 0};
  double right[3] = {up[1] * f[2] - up[2] * f[1], up[2] * f[0] - up[0] * f[2], up[0] * f[1] - up[1] * f[0]};  // up x forward
  const double rn = std::sqrt(right[0] * right[0] + right[1] * right[1] + right[2] * right[2]);
  for (double& v : right) v /= rn;
  const double u[3] = {f[1] * right[2] - f[2] * right[1], f[2] * right[0] - f[0] * right[2],
                       f[0] * right[1] - f[1] * right[0]};  // forward x right
  Rigid p;
  for (int a = 0; a < 3; ++a) {
    p.R[a][0] = right[a];
    p.R[a][1] = u[a];
    p.R[a][2] = f[a];
    p.t[a] = c[a];
  }
  return p;
}

// ---------------------------------------------------------------------------------------------------------------
// virtual photos
// ---------------------------------------------------------------------------------------------------------------
class NoiseRng {  // xorshift64* + Box-Muller: the same numbers on every platform
 public:
  explicit NoiseRng(uint64_t seed) : s_(seed * 2685821657736338717ull + 1) {}
  double Uniform() {
    s_ ^= s_ >> 12;
    s_ ^= s_ << 25;
    s_ ^= s_ >> 27;
    return static_cast<double>((s_ * 2685821657736338717ull) >> 11) / 9007199254740992.0;
  }
  double Normal() {
    const double u1 = std::max(Uniform(), 1e-300), u2 = Uniform();
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * kPi * u2);
  }

 private:
  uint64_t s_;
};

// detection-image-tracker-test.cc image(): the photo a camera at `cam` (pose in the target frame) takes of the target whose
// model points (target units) are scaled by k.
c8::FrameWithPoints Photo(const c8::TargetWithPoints& src, double k, const c8::HMatrix& cam, double noise_px,
                          NoiseRng* rng) {
  const c8_PixelPinholeCameraModel kc = TestK();
  c8::FrameWithPoints ret{kc};
  const c8::HMatrix kk = c8::HMatrixGen::intrinsic(kc);
  const c8::HMatrix inv = cam.inv();
  for (size_t i = 0; i < src.points().size(); ++i) {
    const auto& pt = src.points()[i];
    const c8::HPoint3 wpt{static_cast<float>(pt.x() * k), static_cast<float>(pt.y() * k), 0.0f};
    if ((inv * wpt).z() < 0) continue;
    c8::HPoint2 im = (kk * inv * wpt).flatten();
    if (noise_px > 0) {
      im = c8::HPoint2{im.x() + static_cast<float>(noise_px * rng->Normal()),
                       im.y() + static_cast<float>(noise_px * rng->Normal())};
    }
    if (im.x() < 0 || im.y() < 0 || im.x() >= kc.pixelsWidth || im.y() >= kc.pixelsHeight) continue;
    ret.addImagePixelPoint(im, pt.scale(), pt.angle(), pt.gravityAngle(), src.store().getClone(i));
  }
  return ret;
}

// A loaded target (DetectionImage must not move once it is in a map: matcher pointers).
struct LoadedTarget {
  explicit LoadedTarget(uint32_t seed) {
    const image_target::GrayImage img = SyntheticTarget(480, 640, seed);
    image_target::ImageTargetSpec spec;
    spec.name = "t0";
    spec.type = image_target::ImageTargetType::kPlanar;
    images.insert(std::make_pair(std::string("t0"), image_target::LoadDetectionImage(spec, img.view(), TestPinholeK(),
                                                                                      ShaderNumericModel::kIeee)));
    targets.insert(std::make_pair(std::string("t0"), &images.at("t0")));
    images.at("t0").framePoints().frameBounds(&llb, &urb);
  }
  DetectionImageMap images;
  DetectionImagePtrMap targets;
  c8::HPoint2 llb{0.f, 0.f}, urb{0.f, 0.f};
  const c8::TargetWithPoints& Source() const { return images.at("t0").framePoints(); }
};

LoadedTarget& TargetFor(uint32_t seed) {
  static std::map<uint32_t, std::unique_ptr<LoadedTarget>> cache;
  auto& slot = cache[seed];
  if (!slot) slot = std::make_unique<LoadedTarget>(seed);
  return *slot;
}

// ---------------------------------------------------------------------------------------------------------------
// the engine side: VioTracker with doubles that hand over a prepared photo and a scripted tam
// ---------------------------------------------------------------------------------------------------------------
struct ScriptedModules : public TrackerModules {
  const c8::FrameWithPoints* photo = nullptr;
  c8::HMatrix tam = c8::HMatrixGen::i();

  void Detect(const c8::Gr8Pyramid&, const c8::DetectionConfig&, c8::FrameWithPoints* frame) override {
    if (photo != nullptr) frame->clearAndClone(*photo);
  }
  frontend::DetectStats detect_stats() const override { return {}; }
  void TrackMap(TrackerInput*, const FrameState&, MapTrackerOutput* out, compat::RandomNumbers*) override {
    out->state = 1;
  }
  void UpdateMap(const TrackerInput&, const MapTrackerOutput&, int64_t, MapBuilderOutput* out) override {
    *out = MapBuilderOutput{};
  }
  c8::HMatrix ApplyMapBuilderOutput(const MapBuilderOutput&) override { return tam; }
  void RecreateMapTrackerAndBuilder() override {}
  void VisibleMapPoints(std::vector<vio_tracker::VisibleMapPoint>* out) const override { out->clear(); }
  int ground_match_count() const override { return 0; }
  int num_keyframes() const override { return 0; }
  std::array<int, 5> num_points_by_source() const override { return {0, 0, 0, 0, 0}; }
};

struct Frame {
  bool present = false;
  int last_seen = -1, first_seen = 0, status = 0;
  float scale = 0.f, width = 0.f, height = 0.f;
  c8::HMatrix pose = c8::HMatrixGen::i();  // as the tracker reports it: Y * world pose
};

struct RunRecord {
  std::vector<Frame> frames;
  c8::HMatrix yaw = c8::HMatrixGen::i();  // YawOffsetFromPose(first tam) (what the tracker applied)
};

// What a scenario provides to every run that uses it.
struct ScenarioData {
  std::vector<c8::FrameWithPoints> photos;
  std::vector<c8::HMatrix> cams;  // true camera pose in the world (= the unmutated tam)
  int min_points = 1 << 30;       // smallest photo (frames outside a loss window)
  double step_frac = 0;           // fraction of the frame pairs whose camera step exceeds kBaselineThreshold * 1.1
  double max_step = 0;
};

ScenarioData BuildScenario(const Scenario& s, LoadedTarget& t) {
  ScenarioData d;
  NoiseRng rng(s.target_seed + 17u);
  const Rigid g = TargetInWorld(s);
  for (int i = 0; i < s.n_frames; ++i) {
    const Rigid ct = CameraInTarget(s, i);
    const bool lost = s.loss_len > 0 && i >= s.loss_start && i < s.loss_start + s.loss_len;
    if (lost) {
      d.photos.emplace_back(TestK());
    } else {
      d.photos.push_back(Photo(t.Source(), s.k, ToHMatrix(ct), s.noise_px, &rng));
      d.min_points = std::min(d.min_points, static_cast<int>(d.photos.back().points().size()));
    }
    d.cams.push_back(ToHMatrix(Mul(g, ct)));
  }
  int above = 0;
  for (int i = 1; i < s.n_frames; ++i) {
    const double step = std::sqrt([&] {
      const c8::HVector3 a = c8::translation(d.cams[i]) - c8::translation(d.cams[i - 1]);
      return static_cast<double>(a.x()) * a.x() + static_cast<double>(a.y()) * a.y() + static_cast<double>(a.z()) * a.z();
    }());
    d.max_step = std::max(d.max_step, step);
    if (step > kBaselineThreshold * 1.1) ++above;
  }
  d.step_frac = static_cast<double>(above) / std::max(1, s.n_frames - 1);
  return d;
}

// ---- the scripted tam: the true camera pose, or one of the input mutations of the dev reports ----
enum class MutKind { kNone, kScale, kZeroTranslation, kLag, kFlipX };
struct Mutation {
  MutKind kind = MutKind::kNone;
  double c = 1.0;
  std::string name = "control";
};

c8::HMatrix Mutated(const Mutation& m, int i, const std::vector<c8::HMatrix>& cams) {
  const c8::HMatrix& cam = cams[i];
  switch (m.kind) {
    case MutKind::kScale:
      return c8::scaleTranslation(static_cast<float>(m.c), cam);
    case MutKind::kZeroTranslation:
      return c8::rotationMat(cam);
    case MutKind::kLag:
      return cams[std::max(0, i - 1)];
    case MutKind::kFlipX: {
      const c8::HVector3 t = c8::translation(cam);
      return c8::cameraMotion(c8::HPoint3(-t.x(), t.y(), t.z()), c8::rotation(cam));
    }
    case MutKind::kNone:
      break;
  }
  return cam;
}

using TamFn = std::function<c8::HMatrix(int i, const std::vector<c8::HMatrix>& cams)>;

RunRecord RunTracker(const ScenarioData& d, LoadedTarget& t, const TamFn& tam_fn) {
  config::RegisterDefaults();
  auto m = std::make_unique<ScriptedModules>();
  ScriptedModules* modules = m.get();
  auto tracker = vio_tracker::NewVioTrackerWithModules(false, false, std::move(m));
  c8::Gr8Pyramid pyr;
  pyr.levels.push_back(c8::LevelLayout{});
  compat::RandomNumbers rng;
  vio_tracker::StatusOut status;
  vio_tracker::TrackDebug debug;
  RunRecord run;
  for (size_t i = 0; i < d.photos.size(); ++i) {
    modules->photo = &d.photos[i];
    modules->tam = tam_fn(static_cast<int>(i), d.cams);
    if (i == 0) run.yaw = vio_tracker::YawOffsetFromPose(modules->tam);
    TrackingSensorFrame frame;
    frame.time_ns = 1000000000LL + static_cast<int64_t>(i) * 33333333LL;
    frame.intrinsic = TestK();
    tracker->TrackWithTargets(frame, pyr, t.targets, &rng, &status, &debug);
    Frame f;
    const auto& located = tracker->located_images();
    if (located.size() == 1) {
      const c8::LocatedImage& l = located[0];
      f.present = true;
      f.last_seen = l.targetSpace.lastSeen;
      f.first_seen = l.targetSpace.firstSeen;
      f.status = static_cast<int>(l.targetSpace.status);
      f.scale = l.targetSpace.scale;
      f.width = l.width;
      f.height = l.height;
      f.pose = l.pose;
    }
    run.frames.push_back(f);
  }
  return run;
}

// ---------------------------------------------------------------------------------------------------------------
// metrics
// ---------------------------------------------------------------------------------------------------------------
double Norm3(const c8::HVector3& v) {
  return std::sqrt(static_cast<double>(v.x()) * v.x() + static_cast<double>(v.y()) * v.y() + static_cast<double>(v.z()) * v.z());
}

// Angle of the relative rotation, 2 atan2(|v|, |w|) (the stable form; acos((tr - 1) / 2) has a 0.026 deg floor in float:
// image-target.md section 7.5).
double RotationErrorDeg(const c8::HMatrix& a, const c8::HMatrix& b) {
  const c8::Quaternion q = c8::Quaternion::fromHMatrix(c8::rotationMat(a.inv() * b));
  const double vn = std::sqrt(static_cast<double>(q.x()) * q.x() + static_cast<double>(q.y()) * q.y() +
                              static_cast<double>(q.z()) * q.z());
  return 2.0 * std::atan2(vn, std::fabs(static_cast<double>(q.w()))) / kDeg;
}

double PositionError(const c8::HMatrix& a, const c8::HMatrix& b) { return Norm3(c8::translation(a) - c8::translation(b)); }

struct Stats {
  double max_scale = 0, max_pos = 0, max_rot = 0, max_size = 0, max_stat = 0;
  int n = 0;  // frames judged
  double coverage = 0;
  double last_scale = 0;
};

// S1 / S2 / S3 quantities of one run of a main scenario against ground truth (frames >= kWarmup with last_seen == 0).
Stats MainStats(const Scenario& s, const LoadedTarget& t, const RunRecord& run) {
  Stats st;
  const c8::HMatrix gt = run.yaw * ToHMatrix(TargetInWorld(s));
  const double w_model = t.urb.x() - t.llb.x(), h_model = t.urb.y() - t.llb.y();
  std::vector<c8::HVector3> pos;
  int total = 0;
  for (size_t i = kWarmup; i < run.frames.size(); ++i) {
    ++total;
    const Frame& f = run.frames[i];
    if (!f.present || f.last_seen != 0) continue;
    ++st.n;
    st.last_scale = f.scale;
    st.max_scale = std::max(st.max_scale, std::fabs(f.scale / s.k - 1.0));
    st.max_pos = std::max(st.max_pos, PositionError(f.pose, gt) / s.k);
    st.max_rot = std::max(st.max_rot, RotationErrorDeg(f.pose, gt));
    st.max_size = std::max(st.max_size, std::max(std::fabs(f.width / (w_model * s.k) - 1.0),
                                                  std::fabs(f.height / (h_model * s.k) - 1.0)));
    pos.push_back(c8::translation(f.pose));
  }
  st.coverage = total > 0 ? static_cast<double>(st.n) / total : 0.0;
  if (!pos.empty()) {
    double mx = 0, my = 0, mz = 0;
    for (const auto& p : pos) {
      mx += p.x();
      my += p.y();
      mz += p.z();
    }
    mx /= pos.size();
    my /= pos.size();
    mz /= pos.size();
    for (const auto& p : pos) {
      st.max_stat = std::max(st.max_stat, std::sqrt((p.x() - mx) * (p.x() - mx) + (p.y() - my) * (p.y() - my) +
                                                   (p.z() - mz) * (p.z() - mz)) / s.k);
    }
  }
  return st;
}

struct PairDev {
  double scale = 0, pos = 0, rot = 0;
  int n = 0;
};

// Frame-by-frame deviation of run B from run A under the pair's metamorphic relation: B = A with the world scaled by c
// (positions c x, scale c x) or rotated about the vertical axis (outputs identical).
PairDev ComparePair(const Scenario& s, const RunRecord& a, const RunRecord& b, double k_world) {
  PairDev d;
  const bool scaled = s.kind == "scale_pair" || s.kind == "null_scale_pair";
  const double c = scaled ? s.pair_c : 1.0;
  for (size_t i = kWarmup; i < a.frames.size(); ++i) {
    const Frame& fa = a.frames[i];
    const Frame& fb = b.frames[i];
    if (!fa.present || !fb.present || fa.last_seen != 0 || fb.last_seen != 0) continue;
    ++d.n;
    d.scale = std::max(d.scale, std::fabs(fb.scale / (c * fa.scale) - 1.0));
    d.pos = std::max(d.pos, PositionError(fb.pose, scaled ? c8::scaleTranslation(static_cast<float>(c), fa.pose) : fa.pose) /
                                (c * k_world));
    d.rot = std::max(d.rot, RotationErrorDeg(fb.pose, fa.pose));
  }
  return d;
}

// ---------------------------------------------------------------------------------------------------------------
// the gates
// ---------------------------------------------------------------------------------------------------------------
struct Verdict {
  std::string id;      // S0 .. S5, or P1 .. P6 (premises)
  std::string scen;    // scenario id or "*"
  bool pass = false;
  std::string detail;
};

struct EvalResult {
  std::vector<Verdict> gates, premises;
  std::vector<std::string> reports;
  std::map<std::string, RunRecord> runs;
  bool GatePass(const std::string& id) const {
    bool seen = false;
    for (const Verdict& v : gates) {
      if (v.id != id) continue;
      seen = true;
      if (!v.pass) return false;
    }
    return seen;
  }
  bool GateFailed(const std::string& id) const {
    for (const Verdict& v : gates) {
      if (v.id == id && !v.pass) return true;
    }
    return false;
  }
  bool PremisesPass() const {
    for (const Verdict& v : premises) {
      if (!v.pass) return false;
    }
    return !premises.empty();
  }
  std::string Text() const {
    std::string out;
    for (const Verdict& v : premises) out += Fmt("[ITS-PREMISE] %s %s %s %s\n", v.id.c_str(), v.scen.c_str(), v.pass ? "PASS" : "FAIL", v.detail.c_str());
    for (const Verdict& v : gates) out += Fmt("[ITS-GATE] %s %s %s %s\n", v.id.c_str(), v.scen.c_str(), v.pass ? "PASS" : "FAIL", v.detail.c_str());
    for (const std::string& r : reports) out += "[ITS-REPORT] " + r + "\n";
    return out;
  }
};

void DumpTsv(const std::string& dir, const Scenario& s, const RunRecord& run) {
  if (dir.empty()) return;
  std::string out = "frame\tpresent\tlast_seen\tfirst_seen\tstatus\tscale\twidth\theight\tx\ty\tz\n";
  for (size_t i = 0; i < run.frames.size(); ++i) {
    const Frame& f = run.frames[i];
    const c8::HVector3 p = c8::translation(f.pose);
    out += Fmt("%zu\t%d\t%d\t%d\t%d\t%.9g\t%.9g\t%.9g\t%.9g\t%.9g\t%.9g\n", i, f.present, f.last_seen, f.first_seen, f.status, f.scale,
               f.width, f.height, p.x(), p.y(), p.z());
  }
  FILE* fp = std::fopen((dir + "/" + s.id + ".tsv").c_str(), "w");
  if (fp != nullptr) {
    std::fwrite(out.data(), 1, out.size(), fp);
    std::fclose(fp);
  }
}

// Runs every scenario of the dataset once and evaluates S0 .. S5 and the premises P1 .. P6. `mut` mutates the scripted tam of
// every run (dev reports only; the registered run always passes the unmutated control).
EvalResult Evaluate(const Dataset& ds, const Mutation& mut, const std::string& tsv_dir = std::string()) {
  EvalResult ev;
  std::map<std::string, ScenarioData> data;
  const auto data_of = [&](const Scenario& s) -> const ScenarioData& {
    auto it = data.find(s.id);
    if (it == data.end()) it = data.emplace(s.id, BuildScenario(s, TargetFor(s.target_seed))).first;
    return it->second;
  };
  const auto run_of = [&](const Scenario& s, const ScenarioData& d, const TamFn& tam) -> const RunRecord& {
    return ev.runs.emplace(s.id, RunTracker(d, TargetFor(s.target_seed), tam)).first->second;
  };
  const TamFn base_tam = [&](int i, const std::vector<c8::HMatrix>& cams) { return Mutated(mut, i, cams); };

  // premises P5: the table has what the gates need
  std::map<std::string, int> kinds;
  for (const Scenario& s : ds.rows) ++kinds[s.kind];
  const bool table_ok = kinds["main"] >= 4 && kinds["zero"] >= 1 && kinds["slow"] >= 1 && kinds["scale_pair"] >= 1 &&
                        kinds["yaw_pair"] >= 1 && kinds["noise"] >= 2 && kinds["loss"] >= 1;
  ev.premises.push_back({"P5", "*", table_ok,
                         Fmt("main %d zero %d slow %d scale_pair %d yaw_pair %d noise %d loss %d", kinds["main"], kinds["zero"],
                             kinds["slow"], kinds["scale_pair"], kinds["yaw_pair"], kinds["noise"], kinds["loss"])});

  // ---- main scenarios and the noise rows ----
  for (const Scenario& s : ds.rows) {
    if (s.kind != "main" && s.kind != "noise") continue;
    LoadedTarget& t = TargetFor(s.target_seed);
    const ScenarioData& d = data_of(s);
    const RunRecord& run = run_of(s, d, base_tam);
    DumpTsv(tsv_dir, s, run);
    const Stats st = MainStats(s, t, run);
    // Only the exact-photo rows are GATE. A noisy row is ONE random realisation of the feature noise (protocol rule 8: a small
    // sample is a REPORT), so its numbers are reported, not judged.
    const bool gate = s.kind == "main";
    const std::string detail_s1 = Fmt("n %d coverage %.3f max |scale / k - 1| %.3e (k %.3f)", st.n, st.coverage, st.max_scale, s.k);
    const std::string detail_s2 = Fmt("n %d position/k %.3e rotation %.3e deg size %.3e (k %.3f)", st.n, st.max_pos, st.max_rot, st.max_size, s.k);
    const std::string detail_s3 = Fmt("n %d max |p - mean p| / k %.3e (k %.3f)", st.n, st.max_stat, s.k);
    const std::string line =
        Fmt("n %d cov %.3f scale %.3e pos %.3e rot %.3e deg size %.3e stat %.3e (k %.3f sigma %.2f px)", st.n, st.coverage,
            st.max_scale, st.max_pos, st.max_rot, st.max_size, st.max_stat, s.k, s.noise_px);
    if (gate) {
      // premises of this row
      ev.premises.push_back({"P1", s.id, d.step_frac >= 0.80, Fmt("camera step > %.3f in %.2f of the pairs (>= 0.80), max step %.3f", kBaselineThreshold * 1.1, d.step_frac, d.max_step)});
      ev.premises.push_back({"P2", s.id, std::fabs(s.k - kPriorScale) / s.k >= 0.05, Fmt("|k - 0.5| / k = %.3f (>= 0.05)", std::fabs(s.k - kPriorScale) / s.k)});
      ev.premises.push_back({"P3", s.id, s.n_frames - kWarmup >= kMinSamples, Fmt("%d frames after the warm-up (>= %d)", s.n_frames - kWarmup, kMinSamples)});
      ev.premises.push_back({"P4", s.id, d.min_points >= 150, Fmt("smallest photo %d points (>= 150)", d.min_points)});
      const bool enough = st.n >= kMinSamples && st.coverage >= kMinCoverage;
      ev.gates.push_back({"S1", s.id, enough && st.max_scale <= kTolScale, detail_s1});
      ev.gates.push_back({"S2", s.id, enough && st.max_pos <= kTolPos && st.max_rot <= kTolRotDeg && st.max_size <= kTolSize, detail_s2});
      ev.gates.push_back({"S3", s.id, enough && st.max_stat <= kTolStationary, detail_s3});
    } else {
      ev.reports.push_back("S8 " + s.id + " " + line);
    }
    if (s.kind == "main" && !ev.runs.empty() && s.id == ds.rows.front().id) {
      std::string prof = "convergence profile of " + s.id + ": scale at frames 0,1,2,3,5,10,20,40:";
      for (int i : {0, 1, 2, 3, 5, 10, 20, 40}) prof += Fmt(" %.4f", run.frames[i].scale);
      ev.reports.push_back(prof + Fmt(" (k %.3f, prior 0.5)", s.k));
    }
  }

  // ---- S0: zero baseline and sub-threshold baseline keep the public initial guess ----
  for (const Scenario& s : ds.rows) {
    if (s.kind != "zero" && s.kind != "slow") continue;
    LoadedTarget& t = TargetFor(s.target_seed);
    const ScenarioData& d = data_of(s);
    TamFn tam = base_tam;
    if (s.kind == "zero") {
      tam = [&](int i, const std::vector<c8::HMatrix>& cams) { return c8::rotationMat(Mutated(mut, i, cams)); };
    }
    const RunRecord& run = run_of(s, d, tam);
    DumpTsv(tsv_dir, s, run);
    double worst = 0;
    int n = 0;
    for (const Frame& f : run.frames) {
      if (!f.present) continue;
      ++n;
      worst = std::max(worst, std::fabs(f.scale / kPriorScale - 1.0));
    }
    ev.gates.push_back({"S0", s.id, n >= kMinSamples && worst <= kTolInitial,
                        Fmt("frames with a located image %d, max |scale / 0.5 - 1| = %.3e (k %.3f, max camera step %.4f)", n, worst, s.k, d.max_step)});
    if (s.kind == "slow") {
      ev.premises.push_back({"P1s", s.id, d.max_step < kBaselineThreshold * 0.9,
                             Fmt("max camera step %.4f < %.4f (slow scenario below the baseline threshold)", d.max_step, kBaselineThreshold * 0.9)});
    }
    const Stats st = MainStats(s, t, run);
    ev.reports.push_back(Fmt("S0 cost of the prior %s: with the scale stuck at 0.5 the published size is wrong by %.3f (relative) and the world position by %.3f of k (k %.3f; the ratio k / 0.5 = %.3f)", s.id.c_str(), st.max_size, st.max_pos, s.k, s.k / kPriorScale));
  }

  // ---- S4 / S5: metamorphic pairs on the photos of their base scenario ----
  for (const Scenario& s : ds.rows) {
    if (s.kind != "scale_pair" && s.kind != "yaw_pair") continue;
    const Scenario* base = ds.Find(s.pair_of);
    if (base == nullptr || ev.runs.count(base->id) == 0) {
      ev.premises.push_back({"P6", s.id, false, "base scenario " + s.pair_of + " missing or not a main row"});
      continue;
    }
    LoadedTarget& t = TargetFor(base->target_seed);
    const ScenarioData& d = data_of(*base);
    TamFn tam;
    if (s.kind == "scale_pair") {
      const double c = s.pair_c;
      tam = [&, c](int i, const std::vector<c8::HMatrix>& cams) { return c8::scaleTranslation(static_cast<float>(c), Mutated(mut, i, cams)); };
    } else {
      const c8::HMatrix r = ToHMatrix(RotY(s.pair_c));
      tam = [&, r](int i, const std::vector<c8::HMatrix>& cams) { return r * Mutated(mut, i, cams); };
    }
    const RunRecord run = RunTracker(d, t, tam);
    ev.runs.emplace(s.id, run);
    DumpTsv(tsv_dir, s, run);
    const PairDev pd = ComparePair(s, ev.runs.at(base->id), run, base->k);
    const bool relation_ok = s.kind == "scale_pair" ? s.pair_c >= 1.2 : std::fabs(s.pair_c) >= 20.0;
    ev.premises.push_back({"P6", s.id, relation_ok && pd.n >= kMinSamples,
                           Fmt("pair parameter %.3f, %d frames compared (>= %d)", s.pair_c, pd.n, kMinSamples)});
    ev.gates.push_back({s.kind == "scale_pair" ? "S4" : "S5", s.id,
                        pd.n >= kMinSamples && pd.scale <= kTolPairRel && pd.pos <= kTolPairRel && pd.rot <= kTolPairRotDeg,
                        Fmt("n %d scale dev %.3e pos dev %.3e rot dev %.3e deg (pair of %s, c/psi %.3f)", pd.n, pd.scale, pd.pos, pd.rot,
                            s.pair_of.c_str(), s.pair_c)});
  }

  // ---- S6 (REPORT): the target disappears for loss_len frames ----
  for (const Scenario& s : ds.rows) {
    if (s.kind != "loss") continue;
    LoadedTarget& t = TargetFor(s.target_seed);
    const ScenarioData& d = data_of(s);
    const RunRecord& run = run_of(s, d, base_tam);
    DumpTsv(tsv_dir, s, run);
    const c8::HMatrix gt = run.yaw * ToHMatrix(TargetInWorld(s));
    double lost_pos = 0, after_pos = 0;
    int lost_present = 0, lost_frames = 0, last_ls = 0;
    for (int i = s.loss_start; i < s.loss_start + s.loss_len; ++i) {
      ++lost_frames;
      const Frame& f = run.frames[i];
      if (!f.present) continue;
      ++lost_present;
      last_ls = f.last_seen;
      lost_pos = std::max(lost_pos, PositionError(f.pose, gt) / s.k);
    }
    for (int i = s.loss_start + s.loss_len + 4; i < std::min<int>(s.n_frames, s.loss_start + s.loss_len + 20); ++i) {
      if (run.frames[i].present) after_pos = std::max(after_pos, PositionError(run.frames[i].pose, gt) / s.k);
    }
    (void)t;
    ev.reports.push_back(Fmt("S6 %s (lost %d frames from %d, k %.3f): located image kept in %d of them (last_seen up to %d); world position "
                             "error during the loss up to %.3e of k (the pose latched at the first localisation, made with the prior 0.5), "
                             "up to %.3e of k from 4 frames after re-acquisition",
                             s.id.c_str(), lost_frames, s.loss_start, s.k, lost_present, last_ls, lost_pos, after_pos));
  }
  return ev;
}

const Dataset& DevDataset() {
  static Dataset ds;
  static bool loaded = false;
  if (!loaded) {
    loaded = true;
    std::string why;
    const char* env_dir = std::getenv("SLAM_REFACT_ITS_DEV_DIR");
    if (!LoadDataset(env_dir != nullptr ? env_dir : SR_ITS_DEV_DIR, &ds, &why)) ds.rows.clear();
  }
  return ds;
}

// The dev path never runs registered data: the variant must say dev and the tree must not be the registered one.
void RequireDevImpl(const Dataset& ds) {
  ASSERT_FALSE(ds.rows.empty()) << "dev dataset " << ds.dir << " missing (run its_gate_data.py --variant dev)";
  ASSERT_EQ(ds.variant, "dev") << "the dev tests accept only variant dev, got " << ds.variant;
  ASSERT_FALSE(its_held::IsRegisteredTree(SR_ITS_PREREG_DIR, "it-scale-gate.v1.manifest.json", "registered", ds.dir))
      << ds.dir << " has the registered tree sha256: refused on the dev path";
}

#define RequireDev(ds)                      \
  do {                                      \
    RequireDevImpl(ds);                     \
    if (::testing::Test::HasFatalFailure()) return; \
  } while (0)

[[maybe_unused]] const char* TsvDir() {
  const char* d = std::getenv("SLAM_REFACT_ITS_OUT");
  return d != nullptr ? d : "";
}

#ifndef SR_ITS_MUTANT_BUILD
// ---------------------------------------------------------------------------------------------------------------
// dev tests (label unit): control, null-run, input mutations, reports
// ---------------------------------------------------------------------------------------------------------------
TEST(ItsDev, ControlPassesOnDevData) {
  const Dataset& ds = DevDataset();
  RequireDev(ds);
  const EvalResult ev = Evaluate(ds, Mutation{}, TsvDir());
  std::printf("%s", ev.Text().c_str());
  EXPECT_TRUE(ev.PremisesPass());
  for (const char* g : {"S0", "S1", "S2", "S3", "S4", "S5"}) EXPECT_TRUE(ev.GatePass(g)) << g;
}

// Determinism: the same scenario twice gives bit-identical outputs (the instrument has no run-to-run noise).
TEST(ItsDev, RunsAreBitIdentical) {
  const Dataset& ds = DevDataset();
  RequireDev(ds);
  const Scenario& s = *ds.Find("dB");
  LoadedTarget& t = TargetFor(s.target_seed);
  const ScenarioData d = BuildScenario(s, t);
  const TamFn tam = [](int i, const std::vector<c8::HMatrix>& cams) { return cams[i]; };
  const RunRecord a = RunTracker(d, t, tam), b = RunTracker(d, t, tam);
  ASSERT_EQ(a.frames.size(), b.frames.size());
  for (size_t i = 0; i < a.frames.size(); ++i) {
    EXPECT_EQ(a.frames[i].scale, b.frames[i].scale) << i;
    EXPECT_EQ(a.frames[i].pose.data(), b.frames[i].pose.data()) << i;
    EXPECT_EQ(a.frames[i].last_seen, b.frames[i].last_seen) << i;
  }
  std::printf("[ITS-NULL] N3 determinism: dB run twice, %zu frames, scale and pose bit-identical\n", a.frames.size());
}

// Null-run (protocol section 3): inputs where no difference is expected, to measure what the instrument itself reports.
//   N1 truth = prior (k = 0.5): any deviation of scale / pose / size is estimator and float noise.
//   N2 the pairs: dE1 (c = 1.6) and dE2 (psi = 75) must have deviation 0 in theory; dE1n (c = 1) and dE2n (psi = 0) are identical runs.
//   N4 zero / sub-threshold baseline: scale must be 0.5 exactly.
// resolution = max(null value, float32 representation floor); tolerance >= 3 x resolution is asserted for every tier.
TEST(ItsDev, NullRunResolutionRule) {
  const Dataset& ds = DevDataset();
  RequireDev(ds);
  const Scenario& n0 = *ds.Find("dN0");
  LoadedTarget& t0 = TargetFor(n0.target_seed);
  const ScenarioData d0 = BuildScenario(n0, t0);
  const RunRecord r0 = RunTracker(d0, t0, [](int i, const std::vector<c8::HMatrix>& cams) { return cams[i]; });
  const Stats s0 = MainStats(n0, t0, r0);
  // pairs
  struct PairNull {
    const char* id;
    PairDev dev;
  };
  std::vector<PairNull> pairs;
  std::map<std::string, RunRecord> bases;
  for (const char* bid : {"dB", "dC"}) {
    const Scenario& b = *ds.Find(bid);
    LoadedTarget& t = TargetFor(b.target_seed);
    const ScenarioData d = BuildScenario(b, t);
    bases[bid] = RunTracker(d, t, [](int i, const std::vector<c8::HMatrix>& cams) { return cams[i]; });
    for (const Scenario& p : ds.rows) {
      if (p.pair_of != bid || (p.kind != "scale_pair" && p.kind != "yaw_pair" && p.kind != "null_scale_pair" && p.kind != "null_yaw_pair")) continue;
      TamFn tam;
      if (p.kind == "scale_pair" || p.kind == "null_scale_pair") {
        const double c = p.pair_c;
        tam = [c](int i, const std::vector<c8::HMatrix>& cams) { return c8::scaleTranslation(static_cast<float>(c), cams[i]); };
      } else {
        const c8::HMatrix r = ToHMatrix(RotY(p.pair_c));
        tam = [r](int i, const std::vector<c8::HMatrix>& cams) { return r * cams[i]; };
      }
      pairs.push_back({p.id.c_str(), ComparePair(p, bases[bid], RunTracker(d, t, tam), b.k)});
    }
  }
  double pair_scale = 0, pair_pos = 0, pair_rot = 0;
  for (const PairNull& p : pairs) {
    std::printf("[ITS-NULL] N2 %s: n %d scale dev %.3e pos dev %.3e rot dev %.3e deg\n", p.id, p.dev.n, p.dev.scale, p.dev.pos, p.dev.rot);
    pair_scale = std::max(pair_scale, p.dev.scale);
    pair_pos = std::max(pair_pos, p.dev.pos);
    pair_rot = std::max(pair_rot, p.dev.rot);
  }
  std::printf("[ITS-NULL] N1 truth=prior (dN0, k %.3f): n %d scale %.3e pos %.3e rot %.3e deg size %.3e stationarity %.3e\n", n0.k, s0.n,
              s0.max_scale, s0.max_pos, s0.max_rot, s0.max_size, s0.max_stat);
  // N4: the zero / slow rows of the control
  const EvalResult ev = Evaluate(ds, Mutation{});
  double s0_worst = 0;
  for (const Verdict& v : ev.gates) {
    if (v.id != "S0") continue;
    const size_t at = v.detail.find("= ");
    s0_worst = std::max(s0_worst, std::atof(v.detail.c_str() + at + 2));
    std::printf("[ITS-NULL] N4 %s: %s\n", v.scen.c_str(), v.detail.c_str());
  }
  struct Tier {
    const char* name;
    double resolution, tolerance;
  };
  const Tier tiers[] = {
      {"S0 scale == 0.5 (relative)", std::max(s0_worst, kUlpRel), kTolInitial},
      {"S1 scale (relative)", std::max(s0.max_scale, kUlpRel), kTolScale},
      {"S2 position / k", std::max(s0.max_pos, kUlpRel), kTolPos},
      {"S2 rotation (deg)", std::max(s0.max_rot, kUlpDeg), kTolRotDeg},
      {"S2 size (relative)", std::max(s0.max_size, kUlpRel), kTolSize},
      {"S3 stationarity / k", std::max(s0.max_stat, kUlpRel), kTolStationary},
      {"S4 S5 scale and position (relative)", std::max(std::max(pair_scale, pair_pos), kUlpRel), kTolPairRel},
      {"S4 S5 rotation (deg)", std::max(pair_rot, kUlpDeg), kTolPairRotDeg},
  };
  for (const Tier& tr : tiers) {
    const double ratio = tr.tolerance / tr.resolution;
    std::printf("[ITS-NULL] tier \"%s\": resolution %.3e tolerance %.3e ratio %.1f %s\n", tr.name, tr.resolution, tr.tolerance, ratio,
                ratio >= 3.0 ? "OK" : "VIOLATES THE 3x RULE");
    EXPECT_GE(ratio, 3.0) << tr.name;
  }
}

// Input mutations (dev data): each one changes only the scripted tam. The gate must FAIL where it says; the negative
// control (x1.005) must not.
struct InputMutant {
  Mutation m;
  std::vector<const char*> must_fail;
  std::vector<const char*> must_pass;
};

std::vector<InputMutant> InputMutants() {
  return {
      {{MutKind::kScale, 1.10, "tam-translation-x1.10"}, {"S1", "S2"}, {}},
      {{MutKind::kScale, 0.90, "tam-translation-x0.90"}, {"S1", "S2"}, {}},
      {{MutKind::kScale, 1.05, "tam-translation-x1.05"}, {"S1", "S2"}, {}},
      {{MutKind::kScale, 1.005, "negative-control-x1.005"}, {}, {"S0", "S1", "S2", "S3", "S4", "S5"}},
      {{MutKind::kZeroTranslation, 1.0, "tam-rotation-only"}, {"S1", "S2"}, {"S0"}},
      {{MutKind::kLag, 1.0, "tam-one-frame-lag"}, {"S2", "S3"}, {}},
      {{MutKind::kFlipX, 1.0, "tam-x-translation-negated"}, {"S2"}, {"S0"}},
  };
}

TEST(ItsDev, InputMutationsAreDetected) {
  const Dataset& ds = DevDataset();
  RequireDev(ds);
  for (const InputMutant& im : InputMutants()) {
    const EvalResult ev = Evaluate(ds, im.m);
    std::string line = "[ITS-MUTATION] " + im.m.name + ":";
    for (const char* g : {"S0", "S1", "S2", "S3", "S4", "S5"}) line += std::string(" ") + g + (ev.GatePass(g) ? "=pass" : "=FAIL");
    std::printf("%s\n", line.c_str());
    for (const char* g : im.must_fail) EXPECT_TRUE(ev.GateFailed(g)) << im.m.name << " must FAIL " << g;
    for (const char* g : im.must_pass) EXPECT_TRUE(ev.GatePass(g)) << im.m.name << " must pass " << g;
  }
}

// REPORT: sensitivity of S1 / S2 to a mis-scaled tam translation (what the gate can and cannot see).
TEST(ItsDev, SensitivityReport) {
  const Dataset& ds = DevDataset();
  RequireDev(ds);
  for (double c : {1.005, 1.01, 1.02, 1.03, 1.05}) {
    const EvalResult ev = Evaluate(ds, Mutation{MutKind::kScale, c, "scale"});
    std::printf("[ITS-SENS] tam translation x%.3f: S1 %s S2 %s\n", c, ev.GatePass("S1") ? "pass" : "FAIL", ev.GatePass("S2") ? "pass" : "FAIL");
  }
}

TEST(ItsDev, ReportsOnDevData) {
  const Dataset& ds = DevDataset();
  RequireDev(ds);
  const EvalResult ev = Evaluate(ds, Mutation{});
  for (const std::string& r : ev.reports) std::printf("[ITS-REPORT] %s\n", r.c_str());
  SUCCEED();
}
#else
// ---------------------------------------------------------------------------------------------------------------
// code mutant executables: the same test file with a copy of one engine source that has one planted bug
// ---------------------------------------------------------------------------------------------------------------
#ifndef SR_ITS_EXPECT_FAIL
#error "SR_ITS_EXPECT_FAIL (comma separated gate ids that the mutant must fail) is required for a mutant build"
#endif
TEST(ItsDev, CodeMutantIsDetected) {
  const Dataset& ds = DevDataset();
  RequireDev(ds);
  const EvalResult ev = Evaluate(ds, Mutation{});
  std::printf("%s", ev.Text().c_str());
  std::stringstream want(SR_ITS_EXPECT_FAIL);
  std::string id;
  while (std::getline(want, id, ',')) {
    EXPECT_TRUE(ev.GateFailed(id)) << "the mutant must FAIL " << id;
  }
}
#endif  // SR_ITS_MUTANT_BUILD

#ifndef SR_ITS_MUTANT_BUILD
// ---------------------------------------------------------------------------------------------------------------
// the gate itself: registered data behind the HELD device (label held; the only test that reads the registered table)
// ---------------------------------------------------------------------------------------------------------------
TEST(ItsGate, RegisteredData) {
  its_held::Request rq;
  rq.prereg_dir = SR_ITS_PREREG_DIR;
  rq.manifest_name = "it-scale-gate.v1.manifest.json";
  rq.gate_id = "it-scale-gate";
  rq.gate_version = "1";
  const char* env = std::getenv(its_held::kEnvName);
  rq.env_value = env != nullptr ? env : "";
  rq.pins = {its_held::SelfExe()};
  rq.kind = "registered";
  rq.data_dir = SR_ITS_REG_DIR;
  std::vector<std::string> log;
  const std::string held = its_held::HeldGate(rq, &log);
  for (const std::string& l : log) std::printf("%s\n", l.c_str());
  ASSERT_EQ(held, "") << held;

  Dataset ds;
  std::string why;
  ASSERT_TRUE(LoadDataset(SR_ITS_REG_DIR, &ds, &why)) << why;
  ASSERT_EQ(ds.variant, "registered");
  const EvalResult ev = Evaluate(ds, Mutation{}, TsvDir());
  std::printf("%s", ev.Text().c_str());
  EXPECT_TRUE(ev.PremisesPass()) << "a premise failed: the measurement chain is void (gate file section 7)";
  for (const char* g : {"S0", "S1", "S2", "S3", "S4", "S5"}) EXPECT_TRUE(ev.GatePass(g)) << g;
}
#endif

}  // namespace
}  // namespace slam_refact
