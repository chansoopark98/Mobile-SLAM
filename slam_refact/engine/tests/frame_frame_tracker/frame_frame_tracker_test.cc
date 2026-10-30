// FrameFrameTracker (1266) tests: V-FFT (a)-(c) on synthetic scenes (pre-registered,
// architecture.md §6.6.4: sigma 1e-3 ray noise, 20-60 ground points, 20 % outliers; (a) accept /
// reject equals the noise-free reference, (b) rotation <= 0.05 deg and translation direction
// <= 1 deg, (c) on failure delta translation exactly 0), ClassifyHorizon (1267) against the
// original-binary histograms (WP2-PRE probe-fft-1267-r1), 1269, the height update (Q9) and the
// depth egomotion path.

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "c8/geometry/egomotion.h"
#include "c8/hmatrix.h"
#include "c8/quaternion.h"
#include "frame_frame_tracker/frame_frame_tracker.h"
#include "frame_frame_tracker/frame_frame_tracker_internal.h"
#include "frame_frame_tracker/ground_homography.h"
#include "json_lite.h"
#include "reality/engine/features/image-descriptor.h"

namespace slam_refact::frame_frame_tracker {
namespace {

using slam_refact::testing::JsonValue;
using slam_refact::testing::ReadJson;

constexpr double kPi = 3.14159265358979323846;
const std::string kDataDir = std::string(SR_TEST_DATA_DIR) + "/frame-frame-tracker";

// ---------------------------------------------------------------------------------------------
// 1267 against the binary: the probe's quaternions and rays are regenerated with its LCG
// (probe_fft_1267.mjs), the class histogram per quaternion must match exactly.

struct ProbeLcg {
  uint32_t seed = 1267;
  double Next() {
    seed = seed * 1664525u + 1013904223u;
    return static_cast<double>(seed) / 4294967296.0;
  }
};

c8::Quaternion AxisQuat(double ax, double ay, double az, double ang) {
  const double s = std::sin(ang / 2);
  return c8::Quaternion(static_cast<float>(std::cos(ang / 2)), static_cast<float>(ax * s),
                        static_cast<float>(ay * s), static_cast<float>(az * s));
}

TEST(ClassifyHorizon, MatchesBinaryHistograms) {
  JsonValue cases;
  ASSERT_TRUE(ReadJson(kDataDir + "/probe-fft-1267-r1/cases.json", &cases));
  ASSERT_EQ(cases.size(), 44u);
  ProbeLcg lcg;
  std::vector<c8::Quaternion> quats = {c8::Quaternion(), AxisQuat(1, 0, 0, kPi / 4),
                                       AxisQuat(1, 0, 0, -kPi / 4), AxisQuat(1, 0, 0, kPi / 2)};
  for (int k = 0; k < 40; ++k) {
    const double u1 = lcg.Next();
    const double u2 = lcg.Next() * 2 * kPi;
    const double u3 = lcg.Next() * 2 * kPi;
    const double a = std::sqrt(1 - u1);
    const double b = std::sqrt(u1);
    quats.emplace_back(static_cast<float>(b * std::cos(u3)), static_cast<float>(a * std::sin(u2)),
                       static_cast<float>(a * std::cos(u2)), static_cast<float>(b * std::sin(u3)));
  }
  for (size_t qi = 0; qi < quats.size(); ++qi) {
    const JsonValue& c = cases[qi];
    EXPECT_EQ(quats[qi].x(), c["q"][0].AsFloat());
    EXPECT_EQ(quats[qi].w(), c["q"][3].AsFloat());
    std::vector<c8::HPoint3> rays;
    for (int k = 0; k < 64; ++k) {
      const float x = static_cast<float>((lcg.Next() - 0.5) * 1.6);
      const float y = static_cast<float>((lcg.Next() - 0.5) * 1.6);
      rays.emplace_back(x, y, 1.0f);
    }
    rays.emplace_back(0.0f, 0.0f, 1.0f);
    rays.emplace_back(0.0f, 0.2f, 1.0f);
    rays.emplace_back(0.0f, -0.2f, 1.0f);
    const std::vector<int32_t> cls = ClassifyHorizon(quats[qi], rays);
    ASSERT_EQ(cls.size(), rays.size());
    std::array<long long, 5> hist{};
    for (int32_t v : cls) {
      ASSERT_GE(v, 1);
      ASSERT_LE(v, 5);
      ++hist[v - 1];
    }
    for (int b = 0; b < 5; ++b) {
      EXPECT_EQ(hist[b], c["hist"][b].AsInt()) << "quat " << qi << " class " << b + 1;
    }
  }
}

TEST(DepthPointIndices, EverySecondPoint) {
  EXPECT_EQ(DepthPointIndices(7), std::vector<uint32_t>({0, 2, 4, 6}));
  EXPECT_TRUE(DepthPointIndices(0).empty());
}

// ---------------------------------------------------------------------------------------------
// Synthetic two-frame scenes. World: y up, ground y = -height; cameras look along +z of
// R(q) (world_from_camera rotation), rays (x/z, y/z) with y up.

struct Camera {
  c8::Quaternion q;
  c8::HPoint3 c;
  c8::HMatrix Pose() const {
    return c8::HMatrixGen::translation(c.x(), c.y(), c.z()) * q.toRotationMat();
  }
};

struct Scene {
  Camera a;
  Camera b;
  float height = 1.5f;
  std::vector<c8::HPoint3> world;
};

c8_PixelPinholeCameraModel Intrinsics() {
  c8_PixelPinholeCameraModel k;
  k.pixelsWidth = 640;
  k.pixelsHeight = 480;
  k.centerPointX = 319.5f;
  k.centerPointY = 239.5f;
  k.focalLengthHorizontal = 500.0f;
  k.focalLengthVertical = 500.0f;
  return k;
}

c8::Quaternion Pitch(double deg) { return AxisQuat(1, 0, 0, deg * kPi / 180.0); }

c8::Quaternion YawPitch(double yaw_deg, double pitch_deg) {
  return AxisQuat(0, 1, 0, yaw_deg * kPi / 180.0).times(Pitch(pitch_deg));
}

bool Project(const Camera& cam, const c8::HPoint3& p, c8::HPoint2* ray) {
  const auto pc = (cam.Pose().inv() * p).raw();
  if (pc[2] <= 0.1f) {
    return false;
  }
  const float x = pc[0] / pc[2];
  const float y = pc[1] / pc[2];
  if (std::fabs(x) > 0.6f || std::fabs(y) > 0.44f) {
    return false;
  }
  *ray = c8::HPoint2(x, y);
  return true;
}

// Ground points visible in both cameras and below the horizon (lift >= 0.25) in both.
Scene MakeGroundScene(int n, uint32_t seed, const Camera& b) {
  Scene s;
  s.a = {Pitch(40.0), c8::HPoint3(0.0f, 0.0f, 0.0f)};
  s.b = b;
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> ux(-0.55f, 0.55f);
  std::uniform_real_distribution<float> uy(-0.42f, 0.30f);
  const c8::HMatrix pose_a = s.a.Pose();
  while (static_cast<int>(s.world.size()) < n) {
    const c8::HPoint3 ray_w = pose_a * c8::HPoint3(ux(rng), uy(rng), 1.0f);
    const float dy = ray_w.y() - s.a.c.y();
    if (dy >= -1e-3f) {
      continue;
    }
    const float t = (-s.height - s.a.c.y()) / dy;
    const c8::HPoint3 p(s.a.c.x() + t * (ray_w.x() - s.a.c.x()), -s.height,
                        s.a.c.z() + t * (ray_w.z() - s.a.c.z()));
    c8::HPoint2 ra;
    c8::HPoint2 rb;
    if (!Project(s.a, p, &ra) || !Project(s.b, p, &rb)) {
      continue;
    }
    const std::vector<c8::HPoint3> both = {c8::HPoint3(ra.x(), ra.y(), 1.0f),
                                           c8::HPoint3(rb.x(), rb.y(), 1.0f)};
    if (ClassifyHorizon(s.a.q, {both[0]})[0] != 1 || ClassifyHorizon(s.b.q, {both[1]})[0] != 1) {
      continue;
    }
    const auto lift_a = (c8::HMatrixGen::xDegrees(-90.0f) * s.a.q.toRotationMat() * both[0]).raw();
    const auto lift_b = (c8::HMatrixGen::xDegrees(-90.0f) * s.b.q.toRotationMat() * both[1]).raw();
    if (lift_a[2] < 0.25f || lift_b[2] < 0.25f) {
      continue;
    }
    s.world.push_back(p);
  }
  return s;
}

std::array<uint8_t, 32> RandomDescriptor(std::mt19937* rng) {
  std::array<uint8_t, 32> d;
  for (auto& byte : d) {
    byte = static_cast<uint8_t>((*rng)() & 0xff);
  }
  return d;
}

struct Frames {
  c8::FrameWithPoints a{Intrinsics()};
  c8::FrameWithPoints b{Intrinsics()};
};

// noise: ray sigma; outlier_fraction: share of points whose frame-b ray is displaced by
// 0.02-0.06 (inside the matcher radius, so they are matched but geometrically wrong).
Frames MakeFrames(const Scene& s, float noise, float outlier_fraction, uint32_t seed) {
  Frames f;
  std::mt19937 rng(seed);
  std::normal_distribution<float> gauss(0.0f, noise > 0.0f ? noise : 1.0f);
  std::uniform_real_distribution<float> uang(0.0f, 6.2831853f);
  std::uniform_real_distribution<float> umag(0.02f, 0.06f);
  const int num_outliers = static_cast<int>(outlier_fraction * static_cast<float>(s.world.size()));
  for (size_t i = 0; i < s.world.size(); ++i) {
    c8::HPoint2 ra;
    c8::HPoint2 rb;
    Project(s.a, s.world[i], &ra);
    Project(s.b, s.world[i], &rb);
    float ax = ra.x();
    float ay = ra.y();
    float bx = rb.x();
    float by = rb.y();
    if (noise > 0.0f) {
      ax += gauss(rng);
      ay += gauss(rng);
      bx += gauss(rng);
      by += gauss(rng);
    }
    if (static_cast<int>(i) < num_outliers) {
      const float ang = uang(rng);
      const float mag = umag(rng);
      bx += mag * std::cos(ang);
      by += mag * std::sin(ang);
    }
    const std::array<uint8_t, 32> desc = RandomDescriptor(&rng);
    c8::FeatureSet sa;
    sa.gorbFeature = c8::GorbFeature(desc);
    f.a.addImageRayPoint(c8::HPoint2(ax, ay), 0, 0.0f, 0.0f, std::move(sa));
    c8::FeatureSet sb;
    sb.gorbFeature = c8::GorbFeature(desc);
    f.b.addImageRayPoint(c8::HPoint2(bx, by), 0, 0.0f, 0.0f, std::move(sb));
  }
  return f;
}

struct TrackOut {
  bool success = false;
  c8::HMatrix delta = c8::HMatrixGen::i();
  std::vector<uint32_t> ground;
  float height_after = 0.0f;
};

TrackOut RunPair(const Scene& s, const Frames& f, const std::vector<float>* prev_depths = nullptr) {
  FrameFrameTracker t;
  t.set_height(s.height);
  TrackOut o;
  std::vector<uint32_t> ground;
  std::vector<uint32_t> sky;
  std::vector<uint32_t> depth;
  bool ok = false;
  c8::HMatrix d = c8::HMatrixGen::i();
  t.Track(&d, f.a, s.a.q, &ground, &sky, &depth, &ok);
  if (prev_depths != nullptr) {
    t.SetPrevDepths(*prev_depths);
  }
  t.Track(&o.delta, f.b, s.b.q, &o.ground, &sky, &depth, &o.success);
  o.height_after = t.height();
  return o;
}

double RotationErrorDeg(const c8::HMatrix& a, const c8::HMatrix& b) {
  const c8::HMatrix d = c8::rotationMat(a).inv() * c8::rotationMat(b);
  const double tr = static_cast<double>(d(0, 0)) + d(1, 1) + d(2, 2);
  const double c = std::max(-1.0, std::min(1.0, (tr - 1.0) / 2.0));
  return std::acos(c) * 180.0 / kPi;
}

double TranslationDirectionErrorDeg(const c8::HMatrix& a, const c8::HMatrix& b) {
  const double ax = a(0, 3), ay = a(1, 3), az = a(2, 3);
  const double bx = b(0, 3), by = b(1, 3), bz = b(2, 3);
  const double c = (ax * bx + ay * by + az * bz) /
                   (std::sqrt(ax * ax + ay * ay + az * az) * std::sqrt(bx * bx + by * by + bz * bz));
  return std::acos(std::max(-1.0, std::min(1.0, c))) * 180.0 / kPi;
}

Camera MovedCamera() {
  return {YawPitch(2.0, 41.0), c8::HPoint3(0.10f, -0.03f, 0.08f)};
}

// V-FFT scenes for (a) and (b): 20-60 ground points, sigma 1e-3, 20 % outliers, camera at
// height 1.5 pitched 40 deg moving 0.13 m with 2 deg yaw.
struct VfftRun {
  int n;
  TrackOut ref;
  TrackOut noisy;
  c8::HMatrix truth = c8::HMatrixGen::i();
};

// Opt-in input dump for the isolated original-binary check of V-FFT (b)
// (slam_refact/tools/harness/vfft_b_original_probe.mjs). Active only when SR_VFFT_B_DUMP names an
// output file; it replays the same pair on a fresh tracker, records the 1266 intermediate state
// (rays, matches, horizon classes, ground matches = 1279 inputs) and calls 1279 once more on a
// fresh seed-5489 generator (the state the second Track call sees: the first call has no matches
// and 1279 draws nothing for n < 15). Floats are written with 9 significant digits (exact
// round trip). The test assertions do not read anything from here.
void DumpFloats(std::FILE* f, const float* v, size_t n) {
  std::fputc('[', f);
  for (size_t i = 0; i < n; ++i) {
    std::fprintf(f, "%s%.9g", i ? "," : "", static_cast<double>(v[i]));
  }
  std::fputc(']', f);
}

void DumpMatrix(std::FILE* f, const c8::HMatrix& m) {
  std::fputc('[', f);
  for (int r = 0; r < 4; ++r) {
    const float row[4] = {m(r, 0), m(r, 1), m(r, 2), m(r, 3)};
    std::fputs(r ? "," : "", f);
    DumpFloats(f, row, 4);
  }
  std::fputc(']', f);
}

void DumpRays(std::FILE* f, const std::vector<c8::HPoint2>& rays) {
  std::fputc('[', f);
  for (size_t i = 0; i < rays.size(); ++i) {
    std::fputs(i ? "," : "", f);
    DumpFloats(f, rays[i].raw().data(), 3);
  }
  std::fputc(']', f);
}

void DumpInts(std::FILE* f, const std::vector<int32_t>& v) {
  std::fputc('[', f);
  for (size_t i = 0; i < v.size(); ++i) {
    std::fprintf(f, "%s%d", i ? "," : "", v[i]);
  }
  std::fputc(']', f);
}

void DumpVfftPair(std::FILE* f, bool first, int n, uint32_t scene_seed, const char* kind,
                  const Scene& s, const Frames& fr, const TrackOut& run_pair,
                  const c8::HMatrix& truth) {
  FrameFrameTracker t;
  t.set_height(s.height);
  const auto& impl = FrameFrameTrackerAccess::impl(t);
  std::vector<uint32_t> ground;
  std::vector<uint32_t> sky;
  std::vector<uint32_t> depth;
  bool ok = false;
  c8::HMatrix d = c8::HMatrixGen::i();
  t.Track(&d, fr.a, s.a.q, &ground, &sky, &depth, &ok);
  const std::vector<c8::HPoint2> rays_a = impl.prev_rays;
  const std::vector<int32_t> classes_a = impl.prev_classes;
  const size_t first_call_matches = impl.matches.size();
  const size_t first_call_depth_matches = impl.depth_matches.size();
  t.Track(&d, fr.b, s.b.q, &ground, &sky, &depth, &ok);
  std::vector<c8::HPoint2> prev_ground;
  std::vector<c8::HPoint2> cur_ground;
  for (const c8::PointMatch& pm : impl.ground_matches) {
    prev_ground.push_back(rays_a[pm.dictionaryIdx]);
    cur_ground.push_back(impl.prev_rays[pm.wordsIdx]);
  }
  compat::RandomNumbers rng(compat::kEngineRngSeed);
  c8::RobustHomographyScratchSpace scratch;
  c8::HMatrix h = c8::HMatrixGen::i();
  c8::HMatrix motion = c8::HMatrixGen::i();
  std::vector<uint8_t> mask;
  const bool ret = RobustGroundHomography(s.a.q, prev_ground, s.b.q, cur_ground, &scratch, &rng,
                                          &h, &motion, &mask);
  bool same_delta = true;
  for (int r = 0; r < 4; ++r) {
    for (int k = 0; k < 4; ++k) {
      same_delta = same_delta && d(r, k) == run_pair.delta(r, k);
    }
  }
  std::fprintf(f, "%s\n{\"n\":%d,\"scene_seed\":%u,\"frames_seed\":7,\"kind\":\"%s\",", first ? "" : ",",
               n, scene_seed, kind);
  std::fprintf(f, "\"first_call_matches\":%zu,\"first_call_depth_matches\":%zu,", first_call_matches,
               first_call_depth_matches);
  std::fprintf(f, "\"second_call_depth_matches\":%zu,", impl.depth_matches.size());
  std::fprintf(f, "\"height\":%.9g,\"q_a\":", static_cast<double>(s.height));
  const float qa[4] = {s.a.q.x(), s.a.q.y(), s.a.q.z(), s.a.q.w()};
  const float qb[4] = {s.b.q.x(), s.b.q.y(), s.b.q.z(), s.b.q.w()};
  DumpFloats(f, qa, 4);
  std::fputs(",\"q_b\":", f);
  DumpFloats(f, qb, 4);
  std::fputs(",\"rays_a\":", f);
  DumpRays(f, rays_a);
  std::fputs(",\"rays_b\":", f);
  DumpRays(f, impl.prev_rays);
  std::fputs(",\"classes_a\":", f);
  DumpInts(f, classes_a);
  std::fputs(",\"classes_b\":", f);
  DumpInts(f, impl.prev_classes);
  std::fputs(",\"matches\":[", f);
  for (size_t i = 0; i < impl.matches.size(); ++i) {
    std::fprintf(f, "%s[%zu,%zu]", i ? "," : "", impl.matches[i].wordsIdx,
                 impl.matches[i].dictionaryIdx);
  }
  std::fputs("],\"ground_matches\":[", f);
  for (size_t i = 0; i < impl.ground_matches.size(); ++i) {
    std::fprintf(f, "%s[%zu,%zu]", i ? "," : "", impl.ground_matches[i].wordsIdx,
                 impl.ground_matches[i].dictionaryIdx);
  }
  std::fputs("],\"prev_ground\":", f);
  DumpRays(f, prev_ground);
  std::fputs(",\"cur_ground\":", f);
  DumpRays(f, cur_ground);
  std::fprintf(f, ",\"ours\":{\"track_success\":%s,\"track_delta_equals_run_pair\":%s,\"delta\":",
               ok ? "true" : "false", same_delta ? "true" : "false");
  DumpMatrix(f, d);
  std::fprintf(f, ",\"ret1279\":%s,\"h\":", ret ? "true" : "false");
  DumpMatrix(f, h);
  std::fputs(",\"cam_motion\":", f);
  DumpMatrix(f, motion);
  std::fputs(",\"mask\":[", f);
  for (size_t i = 0; i < mask.size(); ++i) {
    std::fprintf(f, "%s%d", i ? "," : "", mask[i]);
  }
  std::fprintf(f, "],\"rot_err_deg\":%.17g,\"dir_err_deg\":%.17g},\"truth\":",
               RotationErrorDeg(run_pair.delta, truth), TranslationDirectionErrorDeg(run_pair.delta, truth));
  DumpMatrix(f, truth);
  std::fputc('}', f);
}

std::vector<VfftRun> RunVfftScenes() {
  static bool dumped = false;
  const char* dump_path = std::getenv("SR_VFFT_B_DUMP");
  std::FILE* dump = nullptr;
  if (dump_path != nullptr && !dumped) {
    dump = std::fopen(dump_path, "w");
    dumped = dump != nullptr;
    if (dump != nullptr) {
      std::fputs("{\"source\":\"frame_frame_tracker_test.cc RunVfftScenes\",\"scenes\":[", dump);
    }
  }
  std::vector<VfftRun> runs;
  bool first = true;
  for (int n : {20, 30, 40, 50, 60}) {
    const Scene s = MakeGroundScene(n, 100u + static_cast<uint32_t>(n), MovedCamera());
    VfftRun r{n, RunPair(s, MakeFrames(s, 0.0f, 0.0f, 7u)), RunPair(s, MakeFrames(s, 1e-3f, 0.2f, 7u)),
              c8::egomotion(s.a.Pose(), s.b.Pose())};
    if (dump != nullptr) {
      DumpVfftPair(dump, first, n, 100u + static_cast<uint32_t>(n), "ref", s,
                   MakeFrames(s, 0.0f, 0.0f, 7u), r.ref, r.truth);
      DumpVfftPair(dump, false, n, 100u + static_cast<uint32_t>(n), "noisy", s,
                   MakeFrames(s, 1e-3f, 0.2f, 7u), r.noisy, r.truth);
      first = false;
    }
    runs.push_back(r);
  }
  if (dump != nullptr) {
    std::fputs("\n]}\n", dump);
    std::fclose(dump);
  }
  return runs;
}

// V-FFT (a): accept / reject equals the noise-free reference (which recovers the exact motion).
TEST(FrameFrameTrackerVfft, AcceptanceMatchesNoiseFreeReference) {
  for (const VfftRun& r : RunVfftScenes()) {
    ASSERT_TRUE(r.ref.success) << "n " << r.n;
    EXPECT_LE(RotationErrorDeg(r.ref.delta, r.truth), 1e-3) << "n " << r.n;
    EXPECT_LE(TranslationDirectionErrorDeg(r.ref.delta, r.truth), 1e-2) << "n " << r.n;
    EXPECT_EQ(r.noisy.success, r.ref.success) << "n " << r.n;
  }
}

// V-FFT (b): on acceptance, rotation <= 0.05 deg and translation direction <= 1 deg.
TEST(FrameFrameTrackerVfft, AcceptedMotionAccuracy) {
  for (const VfftRun& r : RunVfftScenes()) {
    if (!r.noisy.success) {
      continue;
    }
    const double rot = RotationErrorDeg(r.noisy.delta, r.truth);
    const double dir = TranslationDirectionErrorDeg(r.noisy.delta, r.truth);
    EXPECT_LE(rot, 0.05) << "n " << r.n;
    EXPECT_LE(dir, 1.0) << "n " << r.n;
  }
}

// V-FFT (a), (c): rejected scenes (too few ground matches; all matches outliers) match the
// noise-free reference and fall back to the device rotation with a translation of exactly 0.
TEST(FrameFrameTrackerVfft, RejectionFallsBackToRotationOnly) {
  struct Case {
    int n;
    float outliers;
  };
  for (const Case& c : {Case{12, 0.0f}, Case{40, 1.0f}}) {
    const Scene s = MakeGroundScene(c.n, 500u + static_cast<uint32_t>(c.n), MovedCamera());
    const TrackOut ref = RunPair(s, MakeFrames(s, 0.0f, c.outliers, 11u));
    const TrackOut noisy = RunPair(s, MakeFrames(s, 1e-3f, c.outliers, 11u));
    EXPECT_FALSE(ref.success) << "n " << c.n;
    EXPECT_EQ(noisy.success, ref.success) << "n " << c.n;  // (a)
    ASSERT_FALSE(noisy.success);
    EXPECT_EQ(noisy.delta(0, 3), 0.0f);  // (c)
    EXPECT_EQ(noisy.delta(1, 3), 0.0f);
    EXPECT_EQ(noisy.delta(2, 3), 0.0f);
    const c8::HMatrix expect = s.a.q.inverse().times(s.b.q).toRotationMat();
    for (int r = 0; r < 4; ++r) {
      for (int k = 0; k < 4; ++k) {
        EXPECT_EQ(noisy.delta(r, k), expect(r, k));
      }
    }
    EXPECT_EQ(noisy.height_after, s.height);
  }
}

// First call: no previous frame, rotation-only fallback (identity previous rotation).
TEST(FrameFrameTracker, FirstCallIsRotationOnly) {
  const Scene s = MakeGroundScene(30, 9u, MovedCamera());
  const Frames f = MakeFrames(s, 0.0f, 0.0f, 3u);
  FrameFrameTracker t;
  c8::HMatrix d = c8::HMatrixGen::i();
  std::vector<uint32_t> ground = {99};
  std::vector<uint32_t> sky = {99};
  std::vector<uint32_t> depth;
  bool ok = true;
  t.Track(&d, f.a, s.a.q, &ground, &sky, &depth, &ok);
  EXPECT_FALSE(ok);
  EXPECT_TRUE(ground.empty());
  EXPECT_TRUE(sky.empty());
  EXPECT_EQ(depth, DepthPointIndices(30));
  const c8::HMatrix expect = c8::Quaternion().inverse().times(s.a.q).toRotationMat();
  for (int r = 0; r < 4; ++r) {
    for (int k = 0; k < 4; ++k) {
      EXPECT_EQ(d(r, k), expect(r, k));
    }
  }
}

// Q9: after a ground success, height *= 1 + R(q_prev)[1,:] . t_cam (t in camera-height units).
// A camera that drops by 0.1 from height 1.5 ends near height 1.4.
TEST(FrameFrameTracker, HeightFollowsVerticalMotion) {
  const Camera lower = {YawPitch(1.0, 40.5), c8::HPoint3(0.06f, -0.10f, 0.05f)};
  const Scene s = MakeGroundScene(40, 77u, lower);
  const TrackOut o = RunPair(s, MakeFrames(s, 0.0f, 0.0f, 5u));
  ASSERT_TRUE(o.success);
  EXPECT_NEAR(o.height_after, 1.4f, 2e-3f);
  // Ground inliers are current-frame point indices (1.1 x best residual mask of 1279).
  EXPECT_FALSE(o.ground.empty());
  for (uint32_t idx : o.ground) {
    EXPECT_LT(idx, s.world.size());
  }
}

TEST(FrameFrameTracker, ResetPartialRestoresHeightAndForgetsPreviousFrame) {
  const Scene s = MakeGroundScene(30, 21u, MovedCamera());
  const Frames f = MakeFrames(s, 0.0f, 0.0f, 3u);
  FrameFrameTracker t;
  t.set_height(2.0f);
  c8::HMatrix d = c8::HMatrixGen::i();
  std::vector<uint32_t> ground;
  std::vector<uint32_t> sky;
  std::vector<uint32_t> depth;
  bool ok = false;
  t.Track(&d, f.a, s.a.q, &ground, &sky, &depth, &ok);
  t.ResetPartial();
  EXPECT_EQ(t.height(), 1.0f);
  t.Track(&d, f.b, s.b.q, &ground, &sky, &depth, &ok);
  EXPECT_FALSE(ok);  // nothing to match against
  EXPECT_EQ(t.last_ground_match_count(), 0);
}

// Depth egomotion scenes: noise-free points above the horizon (no ground matches, so the result
// comes from the depth path alone) with previous-frame depths.
struct DepthRun {
  bool ok = false;
  c8::HMatrix delta = c8::HMatrixGen::i();
  c8::HMatrix truth = c8::HMatrixGen::i();
  size_t matches = 0;
  size_t depth_matches = 0;
};

// Frames a (previous) and b of the depth scene plus the true depth of every world point in camera a.
struct DepthScene {
  Scene s;
  Frames f;
  std::vector<float> depths;
};

DepthScene MakeDepthScene(const Camera& b) {
  DepthScene d;
  Scene& s = d.s;
  s.a = {Pitch(-5.0), c8::HPoint3(0.0f, 0.0f, 0.0f)};
  s.b = b;
  std::mt19937 rng(42u);
  std::uniform_real_distribution<float> ux(-0.5f, 0.5f);
  std::uniform_real_distribution<float> uy(0.12f, 0.4f);
  std::uniform_real_distribution<float> ud(2.0f, 6.0f);
  const c8::HMatrix pose_a = s.a.Pose();
  while (s.world.size() < 150) {
    const float depth = ud(rng);
    const c8::HPoint3 p = pose_a * c8::HPoint3(ux(rng) * depth, uy(rng) * depth, depth);
    c8::HPoint2 rb;
    if (Project(s.b, p, &rb) && rb.y() > 0.05f) {
      s.world.push_back(p);
    }
  }
  d.f = MakeFrames(s, 0.0f, 0.0f, 13u);
  for (const c8::HPoint3& p : s.world) {
    d.depths.push_back((pose_a.inv() * p).z());
  }
  return d;
}

DepthRun RunDepthScene(const Camera& b) {
  const DepthScene sc = MakeDepthScene(b);
  const Scene& s = sc.s;
  const Frames& f = sc.f;
  FrameFrameTracker t;
  std::vector<uint32_t> ground;
  std::vector<uint32_t> sky;
  std::vector<uint32_t> depth;
  DepthRun r;
  t.Track(&r.delta, f.a, s.a.q, &ground, &sky, &depth, &r.ok);
  t.SetPrevDepths(sc.depths);
  t.Track(&r.delta, f.b, s.b.q, &ground, &sky, &depth, &r.ok);
  const auto& impl = FrameFrameTrackerAccess::impl(t);
  r.matches = impl.matches.size();
  r.depth_matches = impl.depth_matches.size();
  EXPECT_EQ(impl.ground_matches.size(), 0u);
  EXPECT_TRUE(impl.prev_depths.empty());  // consumed every frame
  r.truth = c8::egomotion(s.a.Pose(), s.b.Pose());
  return r;
}

// Small inter-frame motion: the cur_from_prev solution M passes both inlier tests and the
// tracker returns M^-1 = prev_from_cur, the exact egomotion.
TEST(FrameFrameTracker, DepthEgomotionSmallMotionIsExact) {
  const DepthRun r = RunDepthScene({YawPitch(0.03, -5.02), c8::HPoint3(0.004f, 0.001f, 0.003f)});
  EXPECT_GE(r.matches, 90u);
  EXPECT_EQ(r.depth_matches, r.matches);
  ASSERT_TRUE(r.ok);
  for (int row = 0; row < 3; ++row) {
    for (int k = 0; k < 4; ++k) {
      EXPECT_NEAR(r.delta(row, k), r.truth(row, k), 1e-4f) << row << "," << k;
    }
  }
}

// Q9 (S4 §3.7.2, VERIFIED code): 3D inliers are counted with M^-1 of the cur_from_prev solution,
// i.e. the previous-frame points are moved the wrong way. With a 1.5 deg / 8 cm motion no
// hypothesis passes, the depth path fails although the data are exact, and (no ground matches)
// the tracker falls back to the rotation-only delta.
TEST(FrameFrameTracker, DepthEgomotionQ9RejectsLargerMotion) {
  const DepthRun r = RunDepthScene({YawPitch(1.5, -4.0), c8::HPoint3(0.05f, 0.02f, 0.06f)});
  EXPECT_GE(r.matches, 90u);
  EXPECT_EQ(r.depth_matches, r.matches);
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(r.delta(0, 3), 0.0f);
  EXPECT_EQ(r.delta(1, 3), 0.0f);
  EXPECT_EQ(r.delta(2, 3), 0.0f);
}

// ---------------------------------------------------------------------------------------------
// PrevDepthList (+0xf18): libc++ storage model, US026-CC-1. F926:2586-2592 = clear() +
// resize(n, -FLT_MAX) + per-point depths; G1266:738/:1746 = clear().

std::vector<float> Iota(size_t n, float base) {
  std::vector<float> v(n);
  for (size_t i = 0; i < n; ++i) {
    v[i] = base + static_cast<float>(i);
  }
  return v;
}

// Shorter list after a longer one: the tail keeps the old floats and stays readable.
TEST(PrevDepthList, ShrinkKeepsStaleTail) {
  PrevDepthList l;
  l.Assign(Iota(10, 1.0f));  // 1..10, capacity 10
  l.Assign(Iota(4, 100.0f));
  EXPECT_EQ(l.size(), 4u);
  EXPECT_EQ(l.capacity(), 10u);
  EXPECT_EQ(l.Depth(0), 100.0f);
  EXPECT_EQ(l.Depth(3), 103.0f);
  EXPECT_EQ(l.Depth(4), 5.0f);  // stale value of the earlier list
  EXPECT_EQ(l.Depth(9), 10.0f);
  EXPECT_EQ(l.values(), Iota(4, 100.0f));
}

// Growing past the capacity reallocates: capacity = max(2 * capacity, n), the old floats are gone
// and the new tail is "no depth".
TEST(PrevDepthList, GrowReallocatesWithoutStaleTail) {
  PrevDepthList l;
  l.Assign(Iota(4, 1.0f));
  EXPECT_EQ(l.capacity(), 4u);
  l.Assign(Iota(5, 50.0f));  // 5 <= 2 * 4
  EXPECT_EQ(l.capacity(), 8u);
  EXPECT_EQ(l.Depth(4), 54.0f);
  EXPECT_LE(l.Depth(5), 0.0f);  // fresh tail
  EXPECT_LE(l.Depth(7), 0.0f);
  l.Assign(Iota(20, 7.0f));  // n > 2 * capacity -> exactly n
  EXPECT_EQ(l.capacity(), 20u);
  EXPECT_EQ(l.Depth(19), 26.0f);
  // Growing again after a shrink: the stale tail is dropped by the reallocation.
  l.Assign(Iota(3, 1.0f));
  l.Assign(Iota(21, 1.0f));
  EXPECT_EQ(l.capacity(), 40u);
  EXPECT_LE(l.Depth(21), 0.0f);
}

// clear() (G1266:738 / :1746): logical size 0, storage and capacity kept, stale floats readable.
TEST(PrevDepthList, ClearKeepsStorage) {
  PrevDepthList l;
  l.Assign(Iota(6, 2.0f));
  l.clear();
  EXPECT_TRUE(l.empty());
  EXPECT_EQ(l.size(), 0u);
  EXPECT_EQ(l.capacity(), 6u);
  EXPECT_EQ(l.Depth(0), 2.0f);
  EXPECT_EQ(l.Depth(5), 7.0f);
  l.Assign(Iota(2, 90.0f));  // n <= capacity after clear: first n overwritten, tail kept
  EXPECT_EQ(l.Depth(1), 91.0f);
  EXPECT_EQ(l.Depth(2), 4.0f);
}

// Index >= capacity: unreproducible heap read in the original, modeled as "no depth"; an empty
// list never reads out of bounds.
TEST(PrevDepthList, PastCapacityIsNoDepth) {
  PrevDepthList l;
  EXPECT_EQ(l.capacity(), 0u);
  EXPECT_LE(l.Depth(0), 0.0f);
  l.Assign(Iota(5, 1.0f));
  EXPECT_LE(l.Depth(5), 0.0f);
  EXPECT_LE(l.Depth(1000000), 0.0f);
  l.clear();
  EXPECT_LE(l.Depth(5), 0.0f);
}

// US026-CC-1 through Track (the PrevDepthList tests above call the class directly): when the depth
// list is shorter than the previous-frame point list (an earlier tracked frame set it, a later frame
// did not, see Track), a match whose previous-frame index lies in [size, capacity) reads the depth a
// longer earlier list left in the storage and counts as a depth match; an index past the capacity is
// "no depth". Reverting the lookup to the logical size (idx < size) drops the stale-tail matches.
TEST(FrameFrameTracker, TrackReadsStaleDepthTailOfAnEarlierLongerList) {
  const DepthScene sc = MakeDepthScene({YawPitch(0.03, -5.02), c8::HPoint3(0.004f, 0.001f, 0.003f)});
  constexpr size_t kCapacity = 100;  // storage left by the earlier, longer list
  constexpr size_t kSize = 40;       // the current, shorter list
  ASSERT_GT(sc.depths.size(), kCapacity);
  FrameFrameTracker t;
  std::vector<uint32_t> ground;
  std::vector<uint32_t> sky;
  std::vector<uint32_t> depth;
  c8::HMatrix d = c8::HMatrixGen::i();
  bool ok = false;
  // Frame 1 (prev_frame := a); a 100-entry list for it (shorter than a's 150 points); frame 2 consumes
  // the list (clear(): size 0, storage kept) and makes b the previous frame.
  t.Track(&d, sc.f.a, sc.s.a.q, &ground, &sky, &depth, &ok);
  t.SetPrevDepths(std::vector<float>(sc.depths.begin(), sc.depths.begin() + kCapacity));
  t.Track(&d, sc.f.b, sc.s.b.q, &ground, &sky, &depth, &ok);
  const auto& impl = FrameFrameTrackerAccess::impl(t);
  ASSERT_TRUE(impl.prev_depths.empty());
  ASSERT_EQ(impl.prev_depths.capacity(), kCapacity);
  // A shorter list for b: 40 fresh depths over the first 40 slots, slots 40..99 keep the old floats.
  t.SetPrevDepths(std::vector<float>(kSize, 7.0f));
  ASSERT_EQ(impl.prev_depths.size(), kSize);
  ASSERT_EQ(impl.prev_depths.capacity(), kCapacity);
  // Frame 3 (cur = a, previous = b): the matches' dictionary indices are b's point indices.
  t.Track(&d, sc.f.a, sc.s.a.q, &ground, &sky, &depth, &ok);
  size_t in_size = 0, in_stale_tail = 0, past_capacity = 0;
  for (const c8::PointMatch& m : impl.matches) {
    if (m.dictionaryIdx < kSize) {
      ++in_size;
    } else if (m.dictionaryIdx < kCapacity) {
      ++in_stale_tail;
    } else {
      ++past_capacity;
    }
  }
  // The scene must exercise all three regions, otherwise the counts below prove nothing.
  ASSERT_GT(in_size, 0u);
  ASSERT_GT(in_stale_tail, 0u);
  ASSERT_GT(past_capacity, 0u);
  EXPECT_EQ(impl.depth_matches.size(), in_size + in_stale_tail);
  for (const c8::PointMatch& m : impl.depth_matches) {
    EXPECT_LT(m.dictionaryIdx, kCapacity);
  }
}

}  // namespace
}  // namespace slam_refact::frame_frame_tracker
