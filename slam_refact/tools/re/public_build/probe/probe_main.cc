// Standalone probe for public xr_reference/xr_reference@4edfa484 world-SLAM sources (no Bazel, no OpenCV,
// no Cap'n Proto). Runs each component on fixed synthetic input and prints deterministic results.
//
// Sections:
//   [A] Gr8Gl CPU half: decode a GPU-format Gr8Pyramid (R=gray, G=blurred gray, B=NMS score,
//       A=subpixel nibble pair), per-level budgets, retainBest 60px bins, ORB angles.
//       The score channel is synthetic (hand placed); the GPU stage1-3 shaders are NOT run.
//   [B] ORB (center-of-mass angle) and GORB (gravity angle) 32-byte descriptors, Hamming distance,
//       DescriptorLshIndex nearest neighbour.
//   [C] Bundle: poseEstimationAnalytical (motion-only, Ceres), bundleAdjustSchur (local BA),
//       robustPnP (RANSAC P3P + final pose opt), Bundle.* defaults read through globalParams().
//   [D] Epipolar: essential matrix, distanceFromEpipolar, epipolar depth triangulation,
//       ground-plane prior (groundPointTriangulationPrework).
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <array>
#include <cstring>
#include <deque>

#include "c8/geometry/egomotion.h"
#include "c8/hmatrix.h"
#include "c8/parameter-data.h"
#include "c8/pixels/gr8-pyramid.h"
#include "c8/quaternion.h"
#include "c8/random-numbers.h"
#include "reality/engine/features/descriptor-lsh-index.h"
#include "reality/engine/features/detection-config.h"
#include "reality/engine/features/frame-point.h"
#include "reality/engine/features/gr8gl.h"
#include "reality/engine/geometry/bundle.h"
#include "reality/engine/geometry/epipolar.h"
#include "reality/engine/geometry/pose-pnp.h"

namespace {

using namespace c8;  // NOLINT: probe-only translation unit.

uint32_t lcg_state = 12345u;
uint32_t NextRand() {
  lcg_state = lcg_state * 1664525u + 1013904223u;
  return lcg_state >> 8;
}
float Uniform(float lo, float hi) { return lo + (hi - lo) * (NextRand() % 1000001) / 1000000.0f; }

uint64_t Fnv1a(const uint8_t *data, size_t n, uint64_t h = 1469598103934665603ull) {
  for (size_t i = 0; i < n; ++i) {
    h ^= data[i];
    h *= 1099511628211ull;
  }
  return h;
}

// Order-insensitive hash of (x, y, scale, angle, descriptor) records: std::nth_element in
// retainBest leaves an implementation-defined order (libstdc++ vs libc++), so compare sets too.
uint64_t SetHash(const ImagePoints &pts, bool gorb) {
  Vector<std::array<uint8_t, 48>> recs;
  for (size_t i = 0; i < pts.size(); ++i) {
    std::array<uint8_t, 48> r{};
    const auto loc = pts[i].location();
    const int32_t q[4] = {static_cast<int32_t>(std::lround(loc.pt.x * 1000)),
      static_cast<int32_t>(std::lround(loc.pt.y * 1000)), loc.scale,
      static_cast<int32_t>(std::lround((gorb ? loc.gravityAngle : loc.angle) * 100))};
    std::memcpy(r.data(), q, sizeof(q));
    const auto &fs = pts[i].features();
    const uint8_t *d = gorb ? fs.gorbFeature->data() : fs.orbFeature->data();
    std::memcpy(r.data() + 16, d, 32);
    recs.push_back(r);
  }
  std::sort(recs.begin(), recs.end());
  uint64_t h = 1469598103934665603ull;
  for (const auto &r : recs) {
    h = Fnv1a(r.data(), r.size(), h);
  }
  return h;
}

void PrintMatrix(const char *tag, const HMatrix &m) {
  std::printf("%s", tag);
  for (int r = 0; r < 3; ++r) {
    std::printf(" [%+.6f %+.6f %+.6f %+.6f]", m(r, 0), m(r, 1), m(r, 2), m(r, 3));
  }
  std::printf("\n");
}

double PoseError(const HMatrix &a, const HMatrix &b) {
  double e = 0.0;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 4; ++c) {
      e = std::max(e, std::abs(static_cast<double>(a(r, c)) - b(r, c)));
    }
  }
  return e;
}

// ------------------------------------------------------------------------------------------------
// [A] + [B]
struct SyntheticPyramid {
  int cols = 0;
  int rows = 0;
  Vector<uint8_t> data;
  Gr8Pyramid pyramid;
};

SyntheticPyramid MakePyramid() {
  // Layout: 4 levels (scale 1.44) packed left-to-right in one RGBA atlas, as GlRealityFrame does.
  const int w0 = 320, h0 = 240;
  Vector<LevelLayout> levels;
  int c = 0;
  float w = w0, h = h0;
  for (int l = 0; l < 4; ++l) {
    LevelLayout layout;
    layout.r = 0;
    layout.c = c;
    layout.w = static_cast<int>(std::round(w));
    layout.h = static_cast<int>(std::round(h));
    levels.push_back(layout);
    c += layout.w;
    w /= 1.44f;
    h /= 1.44f;
  }
  SyntheticPyramid s;
  s.cols = c;
  s.rows = h0;
  s.data.assign(static_cast<size_t>(s.cols) * s.rows * 4, 0);
  for (const auto &layout : levels) {
    const float scale = static_cast<float>(w0) / layout.w;
    for (int y = 0; y < layout.h; ++y) {
      for (int x = 0; x < layout.w; ++x) {
        const float bx = x * scale, by = y * scale;
        // Deterministic texture in base-image coordinates.
        float v = 128.0f + 60.0f * std::sin(bx * 0.11f) * std::cos(by * 0.07f)
          + 40.0f * std::sin((bx + 2.0f * by) * 0.031f) + ((static_cast<int>(bx / 16) +
          static_cast<int>(by / 16)) % 2 ? 20.0f : -20.0f);
        v = std::min(255.0f, std::max(0.0f, v));
        uint8_t *p = &s.data[(static_cast<size_t>(layout.r + y) * s.cols + layout.c + x) * 4];
        p[0] = static_cast<uint8_t>(v);
        p[1] = static_cast<uint8_t>(v);  // "blurred" channel: identical copy for the probe.
        p[2] = 0;
        p[3] = 0x77;  // subpixel nibbles 7/14-0.5 = 0.0 offset in x and y.
      }
    }
    // Synthetic NMS score channel: a 13px grid of candidate corners with varying response.
    for (int y = 25; y < layout.h - 25; y += 13) {
      for (int x = 25; x < layout.w - 25; x += 13) {
        uint8_t *p = &s.data[(static_cast<size_t>(layout.r + y) * s.cols + layout.c + x) * 4];
        p[2] = static_cast<uint8_t>(1 + (x * 7 + y * 13) % 250);
      }
    }
  }
  s.pyramid.data = ConstRGBA8888PlanePixels(s.rows, s.cols, s.cols * 4, s.data.data());
  s.pyramid.levels = levels;
  return s;
}

void RunFeatures() {
  std::printf("== [A] Gr8Gl CPU half (synthetic GPU-format pyramid)\n");
  SyntheticPyramid s = MakePyramid();
  for (size_t l = 0; l < s.pyramid.levels.size(); ++l) {
    const auto &L = s.pyramid.levels[l];
    std::printf("A.level %zu r=%d c=%d w=%d h=%d\n", l, L.r, L.c, L.w, L.h);
  }

  // retainBest directly: 30 points in one 60px bin + 4 isolated weak points.
  {
    Vector<ImagePointLocation> kps;
    for (int i = 0; i < 30; ++i) {
      kps.emplace_back(30.f + i, 30.f, 31.f, 0, -1.f, -1.f, 200.f - i, -1);
    }
    kps.emplace_back(150.f, 150.f, 31.f, 0, -1.f, -1.f, 10.f, -1);
    kps.emplace_back(250.f, 150.f, 31.f, 0, -1.f, -1.f, 11.f, -1);
    kps.emplace_back(150.f, 200.f, 31.f, 0, -1.f, -1.f, 12.f, -1);
    kps.emplace_back(250.f, 200.f, 31.f, 0, -1.f, -1.f, 13.f, -1);
    Gr8Gl::retainBest(kps, 8, 320, 240, 21, true);
    std::printf("A.retainBest kept=%zu:", kps.size());
    for (const auto &k : kps) {
      std::printf(" (%.0f,%.0f,r=%.0f)", k.pt.x, k.pt.y, k.response);
    }
    std::printf("\n");
  }

  Gr8Gl gr8 = Gr8Gl::create();
  ImagePoints pts = gr8.detectAndCompute(s.pyramid, 500);
  int per_level[8] = {0};
  uint64_t h = 1469598103934665603ull;
  for (size_t i = 0; i < pts.size(); ++i) {
    const auto loc = pts[i].location();
    per_level[loc.scale]++;
    const auto &orb = pts[i].features().get<OrbFeature>();
    h = Fnv1a(orb.data(), orb.size(), h);
  }
  std::printf(
    "A.detectAndCompute n=%zu per_level=[%d,%d,%d,%d] orb_desc_fnv=%016" PRIx64 "\n",
    pts.size(), per_level[0], per_level[1], per_level[2], per_level[3], h);
  std::printf("A.detectAndCompute orb_set_fnv=%016" PRIx64 "\n", SetHash(pts, false));
  for (size_t i = 0; i < std::min<size_t>(5, pts.size()); ++i) {
    const auto loc = pts[i].location();
    const auto &orb = pts[i].features().get<OrbFeature>();
    std::printf(
      "A.kp[%zu] x=%.3f y=%.3f scale=%u size=%.3f angle=%.3f resp=%.0f desc0..7=", i, loc.pt.x,
      loc.pt.y, loc.scale, loc.size, loc.angle, loc.response);
    for (int b = 0; b < 8; ++b) {
      std::printf("%02x", orb.data()[b]);
    }
    std::printf("\n");
  }

  std::printf("== [B] ORB/GORB descriptors, Hamming, LSH\n");
  // GORB: same keypoints, gravity angles from a device pose (camera pitched down 30deg).
  c8_PixelPinholeCameraModel k{320, 240, 159.5f, 119.5f, 260.f, 260.f};
  Quaternion device = Quaternion::fromPitchYawRollDegrees(-30.f, 0.f, 0.f);
  FrameWithPoints frame(k, HMatrixGen::i(), device);
  ImagePoints gpts = gr8.detectAndCompute(s.pyramid, frame, DETECTION_CONFIG_ALL_ORB_GORB, 0.0f);
  int n_gorb = 0, same = 0;
  uint64_t gh = 1469598103934665603ull;
  for (size_t i = 0; i < gpts.size(); ++i) {
    const auto &fs = gpts[i].features();
    if (fs.gorbFeature.has_value() && fs.orbFeature.has_value()) {
      ++n_gorb;
      gh = Fnv1a(fs.gorbFeature->data(), 32, gh);
      same += fs.gorbFeature->hammingDistance(*fs.orbFeature) == 0;
    }
  }
  std::printf(
    "B.gorb n=%zu with_orb_and_gorb=%d identical_orb_gorb=%d gorb_desc_fnv=%016" PRIx64 "\n",
    gpts.size(), n_gorb, same, gh);
  std::printf("B.gorb gorb_set_fnv=%016" PRIx64 " orb_set_fnv=%016" PRIx64 "\n",
    SetHash(gpts, true), SetHash(gpts, false));
  for (size_t i = 0; i < std::min<size_t>(3, gpts.size()); ++i) {
    const auto loc = gpts[i].location();
    std::printf(
      "B.kp[%zu] angle=%.3f gravityAngle=%.3f hamming(orb,gorb)=%d\n", i, loc.angle,
      loc.gravityAngle,
      gpts[i].features().gorbFeature->hammingDistance(*gpts[i].features().orbFeature));
  }

  // LSH index over the ORB descriptors (64 slots), query with a 3-bit-corrupted copy.
  const size_t n = std::min<size_t>(64, pts.size());
  Vector<ImageDescriptor32> dict(n);
  for (size_t i = 0; i < n; ++i) {
    std::memcpy(dict[i].mutableData(), pts[i].features().get<OrbFeature>().data(), 32);
  }
  DescriptorLshIndex<64, ImageDescriptor32> lsh(dict.data(), n, 16, 1, 1, 1 << 16);
  int lsh_hits = 0, bf_hits = 0;
  for (size_t q = 0; q < n; q += 7) {
    ImageDescriptor32 query;
    std::memcpy(query.mutableData(), dict[q].data(), 32);
    query.mutableData()[0] ^= 0x01;
    query.mutableData()[9] ^= 0x10;
    query.mutableData()[31] ^= 0x80;
    auto r = lsh.findNearest(query);
    auto b = lsh.findNearestBruteForce(query);
    lsh_hits += r.index == static_cast<int>(q);
    bf_hits += b.index == static_cast<int>(q) && b.distance == 3;
  }
  std::printf("B.lsh dict=%zu queries=%zu lsh_exact=%d bruteforce_exact_d3=%d\n", n,
    (n + 6) / 7, lsh_hits, bf_hits);
}

// ------------------------------------------------------------------------------------------------
// [C]
Vector<HPoint3> MakeWorld(int n) {
  Vector<HPoint3> pts;
  for (int i = 0; i < n; ++i) {
    // Separate statements: argument evaluation order differs between GCC and clang.
    const float x = Uniform(-2.f, 2.f);
    const float y = Uniform(-1.5f, 1.5f);
    const float z = Uniform(3.f, 8.f);
    pts.emplace_back(x, y, z);
  }
  return pts;
}

Vector<ObservedPoint> Observe(const Vector<HPoint3> &pts, const HMatrix &cam, float noise) {
  Vector<ObservedPoint> out;
  for (const auto &p : pts) {
    ObservedPoint o;
    auto r = (cam.inv() * p).flatten();
    const float nx = Uniform(-noise, noise);
    const float ny = Uniform(-noise, noise);
    o.position = HPoint2(r.x() + nx, r.y() + ny);
    o.scale = 0;
    o.descriptorDist = 0.f;
    o.weight = 1.f;
    out.push_back(o);
  }
  return out;
}

void RunBundle() {
  std::printf("== [C] Bundle / pose (Ceres)\n");
  Vector<HPoint3> world = MakeWorld(60);
  HMatrix cam_true = cameraMotion(
    HPoint3(0.05f, -0.02f, 0.10f), Quaternion::fromPitchYawRollDegrees(2.f, -3.f, 1.f));
  Vector<ObservedPoint> obs = Observe(world, cam_true, 0.0f);

  // Motion-only pose: start from identity, last two poses identity (no stability prior branch).
  HMatrix est = HMatrixGen::i();
  std::deque<HMatrix> last_two = {HMatrixGen::i(), HMatrixGen::i()};
  PoseEstimationResidualOutput res;
  bool ok = poseEstimationAnalytical(obs, world, last_two, &est, {}, &res);
  std::printf("C.poseAnalytical ok=%d max_abs_err=%.3e n_point_res=%zu\n", ok,
    PoseError(est, cam_true), res.pointResiduals.size());
  PrintMatrix("C.poseAnalytical.est", est);
  PrintMatrix("C.poseAnalytical.true", cam_true);
  // Same problem initialised at the true pose: RotationTarget/ZeroTarget priors are anchored at
  // the initial estimate, so with a correct initial pose the solution must stay at the truth.
  HMatrix est_true_init = cam_true;
  bool ok_true_init =
    poseEstimationAnalytical(obs, world, last_two, &est_true_init, {}, nullptr);
  std::printf("C.poseAnalytical.initAtTruth ok=%d max_abs_err=%.3e\n", ok_true_init,
    PoseError(est_true_init, cam_true));
  const char *keys_d[] = {"Bundle.rotationRobust", "Bundle.rotationWt", "Bundle.positionRobust",
    "Bundle.positionWt", "Bundle.poseEstCauchyLoss", "Bundle.poseEstFunctionTolerance",
    "Bundle.poseEstGradientTolerance", "Bundle.poseEstParameterTolerance",
    "Bundle.poseEstHuberLossComposed"};
  for (const char *key : keys_d) {
    if (globalParams().contains(key)) {
      std::printf("C.param %s=%.10g\n", key, globalParams().get<double>(key));
    } else {
      std::printf("C.param %s=<unset>\n", key);
    }
  }
  for (const char *key : {"Bundle.poseEstMaxNumIterations"}) {
    std::printf("C.param %s=%s\n", key,
      globalParams().contains(key) ? std::to_string(globalParams().get<int>(key)).c_str() : "<unset>");
  }
  std::printf("C.param Bundle.useNewPoseEstimationAnalytic=%s\n",
    globalParams().contains("Bundle.useNewPoseEstimationAnalytic")
      ? (globalParams().get<bool>("Bundle.useNewPoseEstimationAnalytic") ? "true" : "false")
      : "<unset>");

  // Local BA: 3 cameras (cam0 fixed), 40 points, perturbed init.
  Vector<HPoint3> pts = MakeWorld(40);
  Vector<HMatrix> cams_true = {HMatrixGen::i(),
    cameraMotion(HPoint3(0.3f, 0.0f, 0.0f), Quaternion::fromPitchYawRollDegrees(0.f, -2.f, 0.f)),
    cameraMotion(HPoint3(0.6f, 0.05f, 0.1f), Quaternion::fromPitchYawRollDegrees(1.f, -4.f, 0.f))};
  Vector<ObservedPoint> ba_obs;
  Vector<int> obs_to_pt, obs_to_cam;
  for (int ci = 0; ci < 3; ++ci) {
    auto o = Observe(pts, cams_true[ci], 0.0f);
    for (int pi = 0; pi < static_cast<int>(pts.size()); ++pi) {
      ba_obs.push_back(o[pi]);
      obs_to_pt.push_back(pi);
      obs_to_cam.push_back(ci);
    }
  }
  Vector<HPoint3> pts_init;
  for (const auto &p : pts) {
    pts_init.emplace_back(p.x() * 1.02f + 0.01f, p.y() * 0.99f, p.z() * 1.03f);
  }
  Vector<HMatrix> cams_init = {cams_true[0],
    cameraMotion(HPoint3(0.32f, 0.01f, -0.01f), Quaternion::fromPitchYawRollDegrees(0.f, -1.f, 0.f)),
    cameraMotion(HPoint3(0.58f, 0.03f, 0.12f), Quaternion::fromPitchYawRollDegrees(0.f, -5.f, 0.f))};
  Vector<int> cam_fixed = {1, 0, 0};
  float residual = -1.f;
  bool ba_ok = bundleAdjustSchur(
    ba_obs, obs_to_pt, &pts_init, false, obs_to_cam, &cams_init, cam_fixed, 10.f, &residual);
  double pt_err = 0.0;
  for (size_t i = 0; i < pts.size(); ++i) {
    pt_err = std::max<double>(pt_err, pts_init[i].dist(pts[i]));
  }
  std::printf(
    "C.bundleAdjustSchur ok=%d residual=%.3e cam1_err=%.3e cam2_err=%.3e max_pt_err=%.3e\n", ba_ok,
    residual, PoseError(cams_init[1], cams_true[1]), PoseError(cams_init[2], cams_true[2]), pt_err);

  // robustPnP: 80 points, 25% outliers.
  Vector<HPoint3> pnp_world = MakeWorld(80);
  HMatrix pnp_true = cameraMotion(
    HPoint3(-0.1f, 0.05f, 0.2f), Quaternion::fromPitchYawRollDegrees(-3.f, 4.f, 2.f));
  Vector<HPoint2> rays;
  for (size_t i = 0; i < pnp_world.size(); ++i) {
    auto r = (pnp_true.inv() * pnp_world[i]).flatten();
    if (i % 4 == 0) {
      const float ox = Uniform(0.05f, 0.2f);
      const float oy = Uniform(0.05f, 0.2f);
      r = HPoint2(r.x() + ox, r.y() - oy);
    }
    rays.push_back(r);
  }
  HMatrix pnp_est = HMatrixGen::i();
  Vector<uint8_t> inliers;
  RandomNumbers random;
  RobustPoseScratchSpace sc;
  RobustPnPParams params;
  params.minZ = -100.f;
  params.maxZ = 100.f;
  params.maxOutOfPlaneRotationDegrees = 90.f;
  bool pnp_ok = robustPnP(pnp_world, rays, HMatrixGen::i(), params, &pnp_est, &inliers, &random, &sc);
  int n_in = 0, n_in_true = 0;
  for (size_t i = 0; i < inliers.size(); ++i) {
    n_in += inliers[i] != 0;
    n_in_true += inliers[i] != 0 && i % 4 != 0;
  }
  std::printf("C.robustPnP ok=%d inliers=%d (true_inliers_among=%d of 60) max_abs_err=%.3e\n",
    pnp_ok, n_in, n_in_true, PoseError(pnp_est, pnp_true));
}

// ------------------------------------------------------------------------------------------------
// [D]
void RunEpipolar() {
  std::printf("== [D] Epipolar / triangulation\n");
  HMatrix cam1 = HMatrixGen::i();
  HMatrix cam2 = cameraMotion(
    HPoint3(0.2f, 0.0f, 0.0f), Quaternion::fromPitchYawRollDegrees(0.f, -2.f, 0.f));
  HMatrix e = essentialMatrixForCameras(cam1, cam2);
  Vector<HPoint3> pts = MakeWorld(10);
  double max_epi = 0.0;
  for (const auto &p : pts) {
    auto r1 = (cam1.inv() * p).flatten();
    auto r2 = (cam2.inv() * p).flatten();
    max_epi = std::max<double>(max_epi, distanceFromEpipolar(e, r1, r2));
  }
  auto off = distanceFromEpipolar(
    e, (cam1.inv() * pts[0]).flatten(), HPoint2((cam2.inv() * pts[0]).flatten().x() + 0.01f,
    (cam2.inv() * pts[0]).flatten().y() + 0.01f));
  std::printf("D.essential max_dist_true=%.3e dist_offset_0.01=%.3e\n", max_epi, off);

  // Epipolar depth (words = cam2, dictionary = cam1).
  auto iw = epipolarDepthImagePrework(cam2, cam1);
  int ok = 0;
  double max_rel = 0.0;
  for (const auto &p : pts) {
    auto words_ray = (cam2.inv() * p).flatten();
    auto dict_ray = (cam1.inv() * p).flatten();
    auto pw = epipolarDepthPointPrework(iw, words_ray);
    auto on_line = closestLineSegmentPoint(pw, HVector2(dict_ray.x(), dict_ray.y()));
    auto d = depthOfEpipolarPoint(iw, pw, on_line, 0);
    double true_depth = (cam2.inv() * p).z();
    if (d.status == EpipolarDepthResult::Status::OK) {
      ++ok;
      max_rel = std::max(max_rel, std::abs(d.depthInWords - true_depth) / true_depth);
    }
  }
  std::printf("D.epipolarDepth ok=%d/%zu max_rel_depth_err=%.3e pureRotation=%d\n", ok, pts.size(),
    max_rel, iw.isPureRotation ? 1 : 0);

  // Ground prior (y=-1 plane), camera pitched down 45 deg.
  HMatrix down45 = HMatrixGen::xDegrees(-45.f);
  auto g = groundPointTriangulationPrework(down45);
  std::printf("D.groundPrework pitch-45 = (%.6f, %.6f, %.6f)\n", g.x(), g.y(), g.z());
}

}  // namespace

int main() {
  std::printf("probe: public xr_reference@4edfa484 world-SLAM closure, __cplusplus=%ld\n", __cplusplus);
  RunFeatures();
  RunBundle();
  RunEpipolar();
  std::printf("probe: done\n");
  return 0;
}
