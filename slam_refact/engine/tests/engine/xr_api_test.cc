// XrController API unit tests (contract FULL-1, docs/spec/06-xr-controller-api.md): display
// intrinsics (US-002), hit test (US-003), configure / updateCameraProjectionMatrix / recenter
// (US-004), lighting (US-005), tracking status events and the front camera rejection (US-006).
// The GlueRunner cases use a scripted TrackerCore (no engine modules); constants from original runs
// are copied with their source path.

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "c8/pixels/pixel-transforms.h"
#include "engine/display_intrinsics.h"
#include "engine/hit_test.h"
#include "engine/lighting.h"
#include "engine/startup_recenter.h"
#include "frontend/roi_draw.h"
#include "slam_refact/glue_runner.h"
#include "slam_refact/xr_controller.h"

namespace slam_refact {
namespace {

// ------------------------------------------------------------------------------------------------
// Scripted core: identity rotation, position (0, 0, 0) unless set, a configurable point list.
struct Script {
  int cores_created = 0;
  std::vector<EngineConfig> configs;  // EngineConfig of every created core
  std::vector<bool> lighting_requested;
  std::vector<WorldPointOut> points{WorldPointOut{7, Vec3f{0.f, 0.f, 2.f}, 0.5f}};
  // Status / reason of call n (0-based over all cores); last entry repeats.
  std::vector<std::pair<TrackingStatus, TrackingReason>> statuses{
      {TrackingStatus::kNormal, TrackingReason::kUnspecified}};
  float exposure = 0.25f;
  int calls = 0;
  // Camera translation (internal coordinates) of call n; empty = the origin. The last entry repeats.
  std::vector<Vec3f> positions;
  // imu_events of every Track call (FULL-7 optional sensors).
  std::vector<std::vector<ImuEvent>> imu;
};

class ScriptedCore final : public TrackerCore {
 public:
  ScriptedCore(Script* s, const EngineConfig& cfg) : s_(s) {
    ++s_->cores_created;
    s_->configs.push_back(cfg);
  }
  TrackerStep Track(const FrameInput& in) override {
    TrackerStep t;
    const size_t k = std::min(static_cast<size_t>(s_->calls), s_->statuses.size() - 1);
    ++s_->calls;
    t.status = s_->statuses[k].first;
    t.reason = s_->statuses[k].second;
    t.world_points_internal = s_->points;
    if (!s_->positions.empty()) {
      const Vec3f& p = s_->positions[std::min(static_cast<size_t>(s_->calls - 1), s_->positions.size() - 1)];
      t.pose.m[12] = p.x;
      t.pose.m[13] = p.y;
      t.pose.m[14] = p.z;
    }
    s_->imu.push_back(in.imu_events);
    s_->lighting_requested.push_back(in.estimate_lighting);
    if (in.estimate_lighting) {
      t.has_lighting = true;
      t.lighting_exposure = s_->exposure;
    }
    return t;
  }

 private:
  Script* s_;
};

EngineConfig Flip6Config() {
  EngineConfig e;
  e.capture_width = 480;
  e.capture_height = 640;
  e.model = "K";  // reduced UA: unregistered device -> iPhone X focal 506.4104
  return e;
}

std::unique_ptr<GlueRunner> MakeGlue(Script* s, EngineConfig e = Flip6Config(), GlueConfig g = {}) {
  return std::make_unique<GlueRunner>(e, g, [s](const EngineConfig& cfg) {
    return std::make_unique<ScriptedCore>(s, cfg);
  });
}

PublishedFrame Step(GlueRunner* glue, int64_t k) {
  const std::vector<uint8_t> px(16, 0);
  return glue->ProcessFrame(1000000000 + k * 33000000, 0, GrayImageView{px.data(), 4, 4, 4});
}

// Runs past the startup recenter (frame 5 rebuilds the core, frame 6 re-publishes).
void Warm(GlueRunner* glue, int frames = 9) {
  for (int k = 0; k < frames; ++k) Step(glue, k);
}

void ExpectMatrixNear(const std::array<float, 16>& got, const std::array<double, 16>& want,
                      double tol) {
  for (int i = 0; i < 16; ++i) EXPECT_NEAR(got[i], want[i], tol) << "element " << i;
}

// ------------------------------------------------------------------------------------------------
// US-002 display intrinsics.

// build/slam-refact-20261015/reference/runs/flip6-s2/r1/events.json intrinsicsChanges[0]:
// texture 480x640, near 0.01, far 1000 (page-runtime.js:147), UA reduced (model K).
constexpr std::array<double, 16> kFlip6S2R1 = {
    2.1100432872772217, 0, 0, 0, 0, 1.582532525062561, 0, 0, 0, 0, -1.0000200271606445, -1,
    0,                  0, -0.02000020071864128, 0};
// build/slam-refact-20261015/reference/runs/tum-room1-full/r1/events.json intrinsicsChanges[0]:
// texture 240x320, UA pixel7 (Google / Pixel 7).
constexpr std::array<double, 16> kTumRoom1FullR1 = {
    2.174260377883911, 0, 0, 0, 0, 1.6306953430175781, 0, 0, 0, 0, -1.0000200271606445, -1,
    0,                 0, -0.02000020071864128, 0};

TEST(DisplayIntrinsics, Flip6S2R1MatchesOriginal) {
  const PinholeIntrinsics capture = CaptureIntrinsicsFor(Flip6Config());
  EXPECT_FLOAT_EQ(capture.fx, 506.4104f);
  ExpectMatrixNear(DisplayProjection(capture, GraphicsIntrinsics{480, 640, 0.01f, 1000.f}),
                   kFlip6S2R1, 1e-6);
}

TEST(DisplayIntrinsics, TumRoom1FullR1MatchesOriginal) {
  EngineConfig e;
  e.capture_width = 240;
  e.capture_height = 320;
  e.manufacturer = "Google";
  e.model = "Pixel 7";
  ExpectMatrixNear(DisplayProjection(CaptureIntrinsicsFor(e), GraphicsIntrinsics{240, 320, 0.01f, 1000.f}),
                   kTumRoom1FullR1, 1e-6);
}

TEST(DisplayIntrinsics, CropPathFollowsCropAndScaleAndToClipSpace) {
  // 360x640 display of a 480x640 capture: scale = max(0.75, 1) = 1, the width is cropped.
  const PinholeIntrinsics capture{480, 640, 239.5f, 319.5f, 506.4104f, 506.4104f};
  const PinholeIntrinsics d = CropAndScaleIntrinsics(capture, 360, 640);
  EXPECT_EQ(d.width, 360);
  EXPECT_FLOAT_EQ(d.cx, 179.5f);
  EXPECT_FLOAT_EQ(d.cy, 319.5f);
  EXPECT_FLOAT_EQ(d.fx, 506.4104f);
  const std::array<float, 16> m = DisplayProjection(capture, GraphicsIntrinsics{360, 640, 0.1f, 100.f});
  EXPECT_NEAR(m[0], 2.0 * 506.4104 / 360.0, 1e-6);
  EXPECT_NEAR(m[5], 2.0 * 506.4104 / 640.0, 1e-6);
  EXPECT_NEAR(m[8], 0.0, 1e-7);
  EXPECT_NEAR(m[9], 0.0, 1e-7);
  EXPECT_NEAR(m[10], (100.0 + 0.1) / (0.1 - 100.0), 1e-6);
  EXPECT_EQ(m[11], -1.0f);
  EXPECT_NEAR(m[14], 2.0 * 100.0 * 0.1 / (0.1 - 100.0), 1e-6);
  // Off-centre principal point: c02 = 1 - (2 cx + 1) / w.
  const PinholeIntrinsics off{480, 640, 250.0f, 300.0f, 500.f, 510.f};
  const std::array<float, 16> o = ToClipSpace(off, 0.01f, 1000.f);
  EXPECT_FLOAT_EQ(o[8], 1.0f - (2.0f * 250.0f + 1.0f) / 480.0f);
  EXPECT_FLOAT_EQ(o[9], -1.0f + (2.0f * 300.0f + 1.0f) / 640.0f);
}

TEST(DisplayIntrinsics, NonPositiveFarUsesTheFixedFallback) {
  const std::array<float, 16> m = DisplayProjection(CaptureIntrinsicsFor(Flip6Config()),
                                                    GraphicsIntrinsics{480, 640, 0.01f, 0.f});
  const std::array<double, 16> want = {2.92424, 0, 0, 0, 0, 1.64488, 0, 0, 0, 0.0015625, -1.0006,
                                       -1, 0, 0, -0.60018, 0};
  ExpectMatrixNear(m, want, 1e-6);
}

TEST(DisplayIntrinsics, GluePublishesTheProjectionOfEachResponse) {
  Script s;
  auto glue = MakeGlue(&s);
  const PublishedFrame first = Step(glue.get(), 0);  // pipeline not full: no response yet
  for (float v : first.intrinsics) EXPECT_EQ(v, 0.f);
  const PublishedFrame second = Step(glue.get(), 1);
  ExpectMatrixNear(second.intrinsics, kFlip6S2R1, 1e-6);
  CameraProjectionUpdate u;
  u.pixel_rect_width = 360;  // merged field by field; height, near, far kept
  glue->UpdateCameraProjectionMatrix(u);
  const PublishedFrame third = Step(glue.get(), 2);
  EXPECT_NEAR(third.intrinsics[0], 2.0 * 506.4104 / 360.0, 1e-6);
  EXPECT_NEAR(third.intrinsics[5], kFlip6S2R1[5], 1e-6);
  EXPECT_EQ(s.cores_created, 1);  // updateCameraProjectionMatrix never rebuilds the engine
}

// ------------------------------------------------------------------------------------------------
// US-003 hit test.

TEST(XrHitTest, EmptyBeforeTheFirstResponseAndAfterRecenter) {
  Script s;
  auto glue = MakeGlue(&s);
  EXPECT_TRUE(glue->XrHitTest(0.5f, 0.5f, {}).empty());
  Warm(glue.get());
  EXPECT_EQ(glue->XrHitTest(0.5f, 0.5f, {}).size(), 1u);
  glue->Recenter();  // new XREngine: no lastCall until the next processed frame
  EXPECT_TRUE(glue->XrHitTest(0.5f, 0.5f, {}).empty());
  Step(glue.get(), 20);  // fills the pipeline only
  EXPECT_TRUE(glue->XrHitTest(0.5f, 0.5f, {}).empty());
  Step(glue.get(), 21);
  EXPECT_EQ(glue->XrHitTest(0.5f, 0.5f, {}).size(), 1u);
}

TEST(XrHitTest, CentreHitOnASinglePointInFrontOfTheCamera) {
  // Camera at the internal origin looking +z, one point at z = 2: zEst = 2 and the centre ray
  // (c02 = 0 for cx = 239.5) gives the internal hit (0, 0, 2). Default coordinates: right-handed
  // z flip, scale = origin.y = 2, origin (0, 2, 0) -> (0, 2, -4), distance 2 * 2.
  Script s;
  auto glue = MakeGlue(&s);
  Warm(glue.get());
  const std::vector<XrHitResult> h = glue->XrHitTest(0.5f, 0.5f, {});
  ASSERT_EQ(h.size(), 1u);
  EXPECT_EQ(h[0].type, XrHitType::kFeaturePoint);
  EXPECT_NEAR(h[0].position.x, 0.0, 1e-5);
  EXPECT_NEAR(h[0].position.y, 2.0, 1e-5);
  EXPECT_NEAR(h[0].position.z, -4.0, 1e-5);
  EXPECT_NEAR(h[0].distance, 4.0, 1e-5);
  // rotation is never set: the zero quaternion with w and z negated by makeRightHanded.
  EXPECT_EQ(h[0].rotation.w, 0.f);
  EXPECT_EQ(h[0].rotation.x, 0.f);
  EXPECT_EQ(h[0].rotation.y, 0.f);
  EXPECT_EQ(h[0].rotation.z, 0.f);
  EXPECT_TRUE(std::signbit(h[0].rotation.w));
  EXPECT_TRUE(std::signbit(h[0].rotation.z));
}

TEST(XrHitTest, GaussianWeightOnTheNormalizedPlaneAndPointsBehindTheCamera) {
  // Two points on the centre ray direction at depths 2 and 4 (same normalized coordinates) weigh
  // the same: zEst = 3. A point behind the camera at (0, 0, -1) has normalized (0, 0) too and is
  // not filtered (hit-test-performer.cc:84-105): zEst = (2 + 4 - 1) / 3.
  Script s;
  s.points = {WorldPointOut{1, Vec3f{0.f, 0.f, 2.f}, 1.f}, WorldPointOut{2, Vec3f{0.f, 0.f, 4.f}, 1.f}};
  auto glue = MakeGlue(&s);
  Warm(glue.get());
  std::vector<XrHitResult> h = glue->XrHitTest(0.5f, 0.5f, {});
  ASSERT_EQ(h.size(), 1u);
  EXPECT_NEAR(h[0].distance, 2.0 * 3.0, 1e-5);
  s.points.push_back(WorldPointOut{3, Vec3f{0.f, 0.f, -1.f}, 1.f});
  Step(glue.get(), 30);
  h = glue->XrHitTest(0.5f, 0.5f, {});
  ASSERT_EQ(h.size(), 1u);
  EXPECT_NEAR(h[0].distance, 2.0 * 5.0 / 3.0, 1e-5);
  // A point 3 sigma away on the normalized plane (x / z = 0.06) gets weight exp(-4.5).
  s.points = {WorldPointOut{1, Vec3f{0.f, 0.f, 2.f}, 1.f}, WorldPointOut{2, Vec3f{0.24f, 0.f, 4.f}, 1.f}};
  Step(glue.get(), 31);
  h = glue->XrHitTest(0.5f, 0.5f, {});
  ASSERT_EQ(h.size(), 1u);
  const double w = std::exp(-4.5);
  EXPECT_NEAR(h[0].position.z, -2.0 * (2.0 + 4.0 * w) / (1.0 + w), 1e-4);
}

TEST(XrHitTest, TypeMasks) {
  Script s;
  auto glue = MakeGlue(&s);
  Warm(glue.get());
  EXPECT_EQ(glue->XrHitTest(0.5f, 0.5f, {}).size(), 1u);
  EXPECT_EQ(glue->XrHitTest(0.5f, 0.5f, {XrHitType::kFeaturePoint}).size(), 1u);
  EXPECT_TRUE(glue->XrHitTest(0.5f, 0.5f, {XrHitType::kEstimatedSurface}).empty());
  EXPECT_TRUE(glue->XrHitTest(0.5f, 0.5f, {XrHitType::kDetectedSurface}).empty());
  EXPECT_TRUE(
      glue->XrHitTest(0.5f, 0.5f, {XrHitType::kEstimatedSurface, XrHitType::kDetectedSurface})
          .empty());
  EXPECT_EQ(glue->XrHitTest(0.5f, 0.5f, {XrHitType::kEstimatedSurface, XrHitType::kFeaturePoint})
                .size(),
            1u);
  // An unknown type name stays UNSPECIFIED: the mask is non-zero without the FEATURE_POINT bit.
  EXPECT_TRUE(glue->XrHitTest(0.5f, 0.5f, {XrHitType::kUnspecified}).empty());
  EXPECT_EQ(HitTypeMask({XrHitType::kFeaturePoint, XrHitType::kDetectedSurface}), 0xAu);
  EXPECT_TRUE(ContainsResultType(0, XrHitType::kDetectedSurface));
}

struct Combo {
  bool left_handed;
  bool mirrored;
};

std::vector<XrHitResult> HitWith(const Combo& c, float x, float y) {
  Script s;
  s.points = {WorldPointOut{1, Vec3f{-0.5f, 0.2f, 2.f}, 1.f}, WorldPointOut{2, Vec3f{0.4f, -0.1f, 3.f}, 1.f},
              WorldPointOut{3, Vec3f{0.f, 0.f, 2.5f}, 1.f}};
  GlueConfig g;
  g.left_handed_axes = c.left_handed;
  g.mirrored_display = c.mirrored;
  auto glue = MakeGlue(&s, Flip6Config(), g);
  Warm(glue.get());
  return glue->XrHitTest(x, y, {});
}

TEST(XrHitTest, LeftHandedAxesAndMirroredDisplayCombinations) {
  // Reference: right-handed, not mirrored. Expected relations from XREngine::query and 704:
  //   mirrored: the query uses x' = 1 - x and the result x is negated (origin x = 0, facing id);
  //   leftHandedAxes: no makeRightHanded (z keeps its sign, rotation +0).
  const float x = 0.3f, y = 0.42f;
  const XrHitResult base = HitWith({false, false}, x, y).at(0);
  const XrHitResult base_flip = HitWith({false, false}, 1.f - x, y).at(0);
  const XrHitResult lh = HitWith({true, false}, x, y).at(0);
  const XrHitResult mi = HitWith({false, true}, x, y).at(0);
  const XrHitResult lh_mi = HitWith({true, true}, x, y).at(0);
  // leftHandedAxes: z negated (origin z = 0), x / y and distance unchanged.
  EXPECT_FLOAT_EQ(lh.position.x, base.position.x);
  EXPECT_FLOAT_EQ(lh.position.y, base.position.y);
  EXPECT_FLOAT_EQ(lh.position.z, -base.position.z);
  EXPECT_FLOAT_EQ(lh.distance, base.distance);
  // mirroredDisplay: the hit of the mirrored screen point, x negated.
  EXPECT_FLOAT_EQ(mi.position.x, -base_flip.position.x);
  EXPECT_FLOAT_EQ(mi.position.y, base_flip.position.y);
  EXPECT_FLOAT_EQ(mi.position.z, base_flip.position.z);
  EXPECT_FLOAT_EQ(mi.distance, base_flip.distance);
  // Both.
  EXPECT_FLOAT_EQ(lh_mi.position.x, -base_flip.position.x);
  EXPECT_FLOAT_EQ(lh_mi.position.z, -base_flip.position.z);
  // Rotation: (-0, 0, 0, -0) with the right-handed conversion, (+0, 0, 0, +0) left-handed;
  // mirror does not touch the hit rotation.
  for (const XrHitResult& r : {base, mi}) {
    EXPECT_TRUE(std::signbit(r.rotation.w) && std::signbit(r.rotation.z));
  }
  for (const XrHitResult& r : {lh, lh_mi}) {
    EXPECT_FALSE(std::signbit(r.rotation.w) || std::signbit(r.rotation.z));
  }
  // The screen x is not symmetric here, so mirroring really changes the queried ray.
  EXPECT_GT(std::fabs(base.position.x + base_flip.position.x) +
                std::fabs(base.position.z - base_flip.position.z),
            1e-3);
}

TEST(XrHitTest, SurfaceTypesOnlyWithTheSurfacesExtension) {
  // A grid of points on the internal plane y = -1 (published floor y = 0 with origin y 2 and
  // scale 2). Off: ESTIMATED / DETECTED are empty like the original web runtime; on (ACC-3
  // extension): the floor detector answers them for the same display ray.
  Script s;
  s.points.clear();
  uint64_t id = 1;
  for (int i = 0; i < 12; ++i) {
    for (int j = 0; j < 12; ++j) {
      s.points.push_back(WorldPointOut{id++, Vec3f{-1.5f + 0.25f * i, -1.f, 0.6f + 0.3f * j}, 1.f});
    }
  }
  for (const bool surfaces : {false, true}) {
    Script run = s;
    GlueConfig g;
    g.detect_surfaces = surfaces;
    auto glue = MakeGlue(&run, Flip6Config(), g);
    Warm(glue.get(), 15);
    // One of the two vertical screen positions looks down at the floor (the screen y convention
    // of the display ray is the original's; spec 06 §2.1).
    size_t surface_hits = 0;
    for (const float y : {0.15f, 0.85f}) {
      const std::vector<XrHitResult> h =
          glue->XrHitTest(0.5f, y, {XrHitType::kEstimatedSurface, XrHitType::kDetectedSurface});
      if (!surfaces) {
        EXPECT_TRUE(h.empty());
        continue;
      }
      surface_hits += h.size();
      for (const XrHitResult& r : h) {
        EXPECT_TRUE(r.type == XrHitType::kDetectedSurface ||
                    r.type == XrHitType::kEstimatedSurface);
        EXPECT_NEAR(r.position.y, glue->surfaces().floor.height, 1e-3);
      }
      // FEATURE_POINT is still the original algorithm's single hit, first.
      const std::vector<XrHitResult> all = glue->XrHitTest(0.5f, y, {});
      ASSERT_FALSE(all.empty());
      EXPECT_EQ(all[0].type, XrHitType::kFeaturePoint);
      EXPECT_EQ(all.size(), 1u + h.size());
    }
    if (surfaces) {
      ASSERT_TRUE(glue->surfaces().floor_detected);
      EXPECT_GT(surface_hits, 0u);
    }
  }
}

// ------------------------------------------------------------------------------------------------
// US-004 configure / updateCameraProjectionMatrix / recenter.

TEST(XrConfigure, ScaleAndWorldTrackingAreFixedAfterStart) {
  Script s;
  auto glue = MakeGlue(&s);
  XrConfigureOptions abs;
  abs.absolute_scale = true;
  EXPECT_TRUE(glue->Configure(abs).ok());  // before start: the next core is absolute
  EXPECT_EQ(s.cores_created, 2);
  EXPECT_TRUE(s.configs.back().estimate_scale);
  XrConfigureOptions both;
  both.disable_world_tracking = true;
  const XrError e = glue->Configure(both);
  EXPECT_EQ(e.message, "[XR] Unable to perform metric scale estimation if world tracking is disabled.");
  Step(glue.get(), 0);  // started
  XrConfigureOptions resp;
  resp.absolute_scale = false;
  EXPECT_EQ(glue->Configure(resp).message, "[XR] Scale can only be changed before calling XR8.run()");
  XrConfigureOptions dwt;
  dwt.disable_world_tracking = false;
  EXPECT_EQ(glue->Configure(dwt).message,
            "[XR] World tracking can only be changed before calling XR8.run()");
  // The failed call above left a = R = true in the JS state (the throw comes after the
  // assignments), so every later configure throws the same error, as in the original.
  XrConfigureOptions fine;
  fine.enable_lighting = true;
  EXPECT_EQ(glue->Configure(fine).message,
            "[XR] Unable to perform metric scale estimation if world tracking is disabled.");
}

TEST(XrConfigure, RuntimeOptionsAreAcceptedAfterStart) {
  Script s;
  auto glue = MakeGlue(&s);
  Step(glue.get(), 0);
  XrConfigureOptions fine;
  fine.enable_lighting = true;
  fine.enable_world_points = false;
  fine.left_handed_axes = true;
  fine.mirrored_display = true;
  EXPECT_TRUE(glue->Configure(fine).ok());
}

TEST(XrConfigure, AbsoluteScaleUsesOriginYOneAfterUpdateCameraProjectionMatrix) {
  // configure({scale: 'absolute'}) alone keeps d (the scale) and P; updateCameraProjectionMatrix
  // then sets P.y = 1, d = 1 (xA). The startup recenter publishes the configured origin.
  Script s;
  auto glue = MakeGlue(&s);
  XrConfigureOptions abs;
  abs.absolute_scale = true;
  ASSERT_TRUE(glue->Configure(abs).ok());
  EXPECT_FLOAT_EQ(Step(glue.get(), 0).position.y, 0.f);  // no response yet
  EXPECT_FLOAT_EQ(Step(glue.get(), 1).position.y, 2.f);
  glue->UpdateCameraProjectionMatrix(CameraProjectionUpdate{});
  EXPECT_FLOAT_EQ(Step(glue.get(), 2).position.y, 1.f);
}

TEST(XrConfigure, EnableWorldPointsFalseEmptiesWorldPoints) {
  Script s;
  auto glue = MakeGlue(&s);
  Warm(glue.get());
  EXPECT_EQ(Step(glue.get(), 20).world_points.size(), 1u);  // glue default: on
  XrConfigureOptions off;
  off.enable_world_points = false;
  ASSERT_TRUE(glue->Configure(off).ok());
  EXPECT_TRUE(Step(glue.get(), 21).world_points.empty());
  // The hit test still uses the feature set of the last response: the implementation's choice, a
  // regression pin only (the original takes it from the closed featureSetExecutor, UNVERIFIED).
  EXPECT_EQ(glue->XrHitTest(0.5f, 0.5f, {}).size(), 1u);
  XrConfigureOptions on;
  on.enable_world_points = true;
  ASSERT_TRUE(glue->Configure(on).ok());
  EXPECT_EQ(Step(glue.get(), 22).world_points.size(), 1u);
}

TEST(XrConfigure, LeftHandedAndMirroredApplyToTheNextResponse) {
  Script s;
  auto glue = MakeGlue(&s);
  Warm(glue.get());
  const PublishedFrame a = Step(glue.get(), 20);
  ASSERT_EQ(a.world_points.size(), 1u);
  EXPECT_FLOAT_EQ(a.world_points[0].position.z, -4.f);  // internal z 2, right-handed, scale 2
  XrConfigureOptions o;
  o.left_handed_axes = true;
  o.mirrored_display = true;
  ASSERT_TRUE(glue->Configure(o).ok());
  const PublishedFrame b = Step(glue.get(), 21);
  EXPECT_FLOAT_EQ(b.world_points[0].position.z, 4.f);
  EXPECT_EQ(s.cores_created, 2);  // only the startup recenter rebuilt the core
}

TEST(XrConfigure, UpdateCameraProjectionMatrixOriginAndFacing) {
  Script s;
  auto glue = MakeGlue(&s);
  Warm(glue.get());
  CameraProjectionUpdate u;
  u.origin = Vec3f{1.f, 3.f, -2.f};
  u.facing = QuatWxyz{std::cos(0.25f), 0.f, std::sin(0.25f), 0.f};  // yaw 0.5 rad
  glue->UpdateCameraProjectionMatrix(u);
  const PublishedFrame f = Step(glue.get(), 20);
  // Camera at the internal origin -> the configured origin; responsive scale = origin.y = 3.
  EXPECT_NEAR(f.position.x, 1.0, 1e-6);
  EXPECT_NEAR(f.position.y, 3.0, 1e-6);
  EXPECT_NEAR(f.position.z, -2.0, 1e-6);
  // The point (0, 0, 2) -> right-handed (0, 0, -2) -> scale 3 -> rotated by yaw 0.5 about y.
  ASSERT_EQ(f.world_points.size(), 1u);
  EXPECT_NEAR(f.world_points[0].position.x, 1.0 + (-6.0) * std::sin(0.5), 1e-5);
  EXPECT_NEAR(f.world_points[0].position.z, -2.0 + (-6.0) * std::cos(0.5), 1e-5);
  // Identity camera rotation -> makeRightHanded (-1, 0, 0, 0) -> facing * q = -facing.
  EXPECT_NEAR(f.rotation.w, -std::cos(0.25f), 1e-6);
  EXPECT_NEAR(f.rotation.y, -std::sin(0.25f), 1e-6);
  EXPECT_EQ(s.cores_created, 2);
}

TEST(XrConfigure, RecenterRestoresTheRecenterPointButKeepsTheConfiguredSystem) {
  // XA: P = p, l = f, c8EmAsm_recenter() without configureXr; rebuildEngine re-applies the last
  // configured coordinate system. The next configure() sends P (the recenter point).
  Script s;
  auto glue = MakeGlue(&s);
  Warm(glue.get());
  CameraProjectionUpdate u;
  u.origin = Vec3f{0.f, 4.f, 0.f};
  u.update_recenter_point = false;
  glue->UpdateCameraProjectionMatrix(u);
  const int cores = s.cores_created;
  glue->Recenter();
  EXPECT_EQ(s.cores_created, cores + 1);
  Step(glue.get(), 20);
  EXPECT_FLOAT_EQ(Step(glue.get(), 21).position.y, 4.f);  // configured system is still y 4
  ASSERT_TRUE(glue->Configure(XrConfigureOptions{}).ok());  // sends P = recenter point, d = 4
  EXPECT_FLOAT_EQ(Step(glue.get(), 22).position.y, 2.f);
}

TEST(XrConfigure, OriginYZeroKeepsTheConfiguredCoordinates) {
  // origin.y = 0 gives scale 0, which configureXr does not send (fA truthy-checks the scale). The
  // glue then keeps the construction values (user origin, axes, scale 1 in the 704 chain) instead
  // of falling back to the default coordinate system (origin (0, 2, 0), right-handed).
  Script s;
  GlueConfig g;
  g.origin = Vec3f{1.f, 0.f, 3.f};
  g.left_handed_axes = true;
  auto glue = MakeGlue(&s, Flip6Config(), g);
  Warm(glue.get());
  const PublishedFrame f = Step(glue.get(), 20);
  EXPECT_FLOAT_EQ(f.position.x, 1.f);
  EXPECT_FLOAT_EQ(f.position.y, 0.f);
  EXPECT_FLOAT_EQ(f.position.z, 3.f);
  // The internal point (0, 0, 2) keeps its z sign (left-handed) and is not scaled.
  ASSERT_EQ(f.world_points.size(), 1u);
  EXPECT_FLOAT_EQ(f.world_points[0].position.x, 1.f);
  EXPECT_FLOAT_EQ(f.world_points[0].position.y, 0.f);
  EXPECT_FLOAT_EQ(f.world_points[0].position.z, 5.f);
  // hitTest uses the same coordinates.
  const std::vector<XrHitResult> h = glue->XrHitTest(0.5f, 0.5f, {});
  ASSERT_EQ(h.size(), 1u);
  EXPECT_NEAR(h[0].position.z, 5.0, 1e-5);
  EXPECT_NEAR(h[0].distance, 2.0, 1e-5);
}

TEST(XrConfigure, AbsoluteRecenterRestoresTheOriginYTheCallerGave) {
  // xA copies p = P before `P.y = 1, d = 1`: after recenter() + a configure() the configured
  // origin.y is the caller's 2 again (scale stays 1), not 1.
  Script s;
  EngineConfig e = Flip6Config();
  e.estimate_scale = true;
  auto glue = MakeGlue(&s, e);
  Warm(glue.get());
  EXPECT_FLOAT_EQ(Step(glue.get(), 20).position.y, 1.f);  // absolute: origin.y forced to 1
  glue->Recenter();
  Step(glue.get(), 21);
  EXPECT_FLOAT_EQ(Step(glue.get(), 22).position.y, 1.f);  // recenter does not send configureXr
  ASSERT_TRUE(glue->Configure(XrConfigureOptions{}).ok());
  EXPECT_FLOAT_EQ(Step(glue.get(), 23).position.y, 2.f);  // P = p
}

TEST(XrConfigure, InjectedCoreWithoutFrameSizeHasZeroIntrinsics) {
  // A GlueRunner around an injected core does not need a capture size (no CaptureIntrinsicsFor).
  Script s;
  GlueRunner glue(EngineConfig{}, GlueConfig{}, [&s](const EngineConfig& cfg) {
    return std::make_unique<ScriptedCore>(&s, cfg);
  });
  PublishedFrame f;
  for (int k = 0; k < 4; ++k) f = Step(&glue, k);
  for (float v : f.intrinsics) EXPECT_EQ(v, 0.f);
  EXPECT_EQ(f.world_points.size(), 1u);
}

TEST(XrConfigure, SurfaceStateRestartsWhenTheCoordinateSystemChanges) {
  // floor_ and the stored camera are published coordinates; a runtime change of axes, mirror,
  // origin, facing or scale makes them stale, so detection restarts and finds the floor again in
  // the new coordinates.
  Script s;
  s.points.clear();
  uint64_t id = 1;
  for (int i = 0; i < 12; ++i) {
    for (int j = 0; j < 12; ++j) {
      s.points.push_back(WorldPointOut{id++, Vec3f{-1.5f + 0.25f * i, -1.f, 0.6f + 0.3f * j}, 1.f});
    }
  }
  GlueConfig g;
  g.detect_surfaces = true;
  auto glue = MakeGlue(&s, Flip6Config(), g);
  Warm(glue.get(), 15);
  ASSERT_TRUE(glue->surfaces().floor_detected);
  const float old_height = glue->surfaces().floor.height;
  EXPECT_NEAR(old_height, 0.0, 1e-3);  // internal y = -1, origin y 2, scale 2
  XrConfigureOptions same;
  same.left_handed_axes = false;  // no change: nothing restarts
  ASSERT_TRUE(glue->Configure(same).ok());
  EXPECT_TRUE(glue->surfaces().floor_detected);
  CameraProjectionUpdate u;
  u.origin = Vec3f{0.f, 4.f, 0.f};  // new origin and scale 4: the floor is at published y 4 - 4 = 0
  glue->UpdateCameraProjectionMatrix(u);
  EXPECT_FALSE(glue->surfaces().floor_detected);
  EXPECT_TRUE(glue->HitTest(0.5f, 0.85f, 0xFu).empty());  // stored camera dropped too
  for (int k = 0; k < 20 && !glue->surfaces().floor_detected; ++k) Step(glue.get(), 100 + k);
  ASSERT_TRUE(glue->surfaces().floor_detected);
  EXPECT_NEAR(glue->surfaces().floor.height, 0.0, 1e-3);
}

TEST(XrConfigure, DisableWorldTrackingBeforeStartRebuildsWithDisableVio) {
  Script s;
  auto glue = MakeGlue(&s);
  XrConfigureOptions o;
  o.disable_world_tracking = true;
  ASSERT_TRUE(glue->Configure(o).ok());
  EXPECT_TRUE(s.configs.back().disable_vio);
}

// ------------------------------------------------------------------------------------------------
// US-005 lighting.

TEST(Lighting, PublicEstimatorTestVector) {
  // PUB:reality/engine/lighting/lighting-estimator-test.cc: 5x5 of a stride-6 buffer -> -0.36.
  const uint8_t src[] = {1,   2,   26, 50, 0,   0,   63, 201, 56, 45, 124, 250, 123, 45, 7,
                         23,  200, 87, 100, 200, 60,  5,  6,   150, 34, 57,  2,   5,   78, 87};
  EXPECT_FLOAT_EQ(ExposureScore(5, 5, 6, src, 1), -0.36f);
}

TEST(Lighting, RowsPastTheTaskSplitAreNotScoredButCounted) {
  // rows > 64: 16 tasks of rows / 16 rows; 100 rows -> 96 scored, divisor 100 * cols.
  const int rows = 100, cols = 3;
  std::vector<uint8_t> img(static_cast<size_t>(rows) * cols * 4, 128);
  for (int r = 96; r < rows; ++r) img[static_cast<size_t>(r) * cols * 4] = 255;  // not scored
  img[0] = 255;   // scored high
  img[4] = 0;     // scored low
  img[8] = 240;   // scored high
  EXPECT_FLOAT_EQ(ExposureScore(rows, cols, cols * 4, img.data(), 4), 1.0f / 300.0f);
}

// The pre-2026-10-22 hand port of estimateExposureScore (pixel-transforms.cc:1927-1956), kept here as
// an independent reference now that lighting.cc calls the vendored c8::LightingEstimator.
float LegacyExposureScore(int rows, int cols, int row_bytes, const uint8_t* pixels, int channels) {
  const int n_tasks = rows > 64 ? 16 : 1;
  int32_t scores[16];
  const int32_t row_inc = row_bytes * (rows / n_tasks);
  const uint8_t* row_start = pixels;
  for (int i = 0; i < n_tasks; ++i, row_start += row_inc) {
    scores[i] = 0;
    const uint8_t* stop = row_start + row_inc;
    const uint8_t* start = row_start;
    const uint8_t* end = row_start + cols * channels;
    for (; start < stop; start += row_bytes, end += row_bytes) {
      for (const uint8_t* pix = start; pix < end; pix += channels) {
        scores[i] += (*pix >= c8::EXPOSURE_SCORE_HIGH) - (*pix <= c8::EXPOSURE_SCORE_LOW);
      }
    }
  }
  int32_t score = 0;
  for (int i = 0; i < n_tasks; ++i) score += scores[i];
  return static_cast<float>(score) / (cols * rows);
}

TEST(Lighting, VendoredEstimatorMatchesTheFormerHandPortBitForBit) {
  // Deterministic LCG buffers; shapes cover rows <= 64 (1 task), rows > 64 with and without a
  // remainder past the 16-way split, 1 and 4 channels, padded strides, and a nonzero start offset.
  uint32_t state = 12345u;
  auto next = [&state]() {
    state = state * 1664525u + 1013904223u;
    return static_cast<uint8_t>(state >> 24);
  };
  struct Shape { int rows, cols, channels, pad; };
  const Shape shapes[] = {{5, 5, 1, 1}, {64, 17, 4, 0}, {65, 17, 4, 3}, {100, 3, 4, 0},
                          {240, 135, 4, 0}, {257, 31, 1, 5}, {320, 180, 4, 8}, {3, 1, 4, 0}};
  int compared = 0;
  for (const Shape& sh : shapes) {
    const int row_bytes = sh.cols * sh.channels + sh.pad;
    std::vector<uint8_t> buf(static_cast<size_t>(sh.rows) * row_bytes);
    for (uint8_t& b : buf) b = next();
    const float want = LegacyExposureScore(sh.rows, sh.cols, row_bytes, buf.data(), sh.channels);
    const float got = ExposureScore(sh.rows, sh.cols, row_bytes, buf.data(), sh.channels);
    uint32_t a, b;
    std::memcpy(&a, &want, 4);
    std::memcpy(&b, &got, 4);
    EXPECT_EQ(a, b) << sh.rows << "x" << sh.cols << " ch" << sh.channels;
    ++compared;
  }
  EXPECT_EQ(compared, 8);
}

TEST(Lighting, PublishedOnlyWhenEnabled) {
  Script s;
  auto glue = MakeGlue(&s);
  Warm(glue.get());
  PublishedFrame f = Step(glue.get(), 20);
  EXPECT_FALSE(f.has_lighting);
  EXPECT_FALSE(s.lighting_requested.back());
  XrConfigureOptions o;
  o.enable_lighting = true;
  ASSERT_TRUE(glue->Configure(o).ok());
  f = Step(glue.get(), 21);
  EXPECT_TRUE(s.lighting_requested.back());
  ASSERT_TRUE(f.has_lighting);
  EXPECT_EQ(f.lighting.exposure, 0.25f);
  EXPECT_EQ(f.lighting.temperature, 6500.0f);
}

TEST(Lighting, OutputFollowsTheJsStateNotTheRequestMask) {
  // YA reads the response's lighting when the JS flag h is on. configure({enableLighting: true,
  // disableWorldTracking: false}) after start throws after setting h, so configureXr is not sent
  // and the request mask stays off: the original then publishes the default {0, 0}.
  Script s;
  auto glue = MakeGlue(&s);
  Warm(glue.get());
  XrConfigureOptions o;
  o.enable_lighting = true;
  o.disable_world_tracking = false;
  EXPECT_EQ(glue->Configure(o).message,
            "[XR] World tracking can only be changed before calling XR8.run()");
  PublishedFrame f = Step(glue.get(), 20);
  EXPECT_FALSE(s.lighting_requested.back());
  ASSERT_TRUE(f.has_lighting);
  EXPECT_EQ(f.lighting.exposure, 0.f);
  EXPECT_EQ(f.lighting.temperature, 0.f);
  // A configure that reaches configureXr turns the mask on: the estimate follows.
  ASSERT_TRUE(glue->Configure(XrConfigureOptions{}).ok());
  f = Step(glue.get(), 21);
  EXPECT_TRUE(s.lighting_requested.back());
  ASSERT_TRUE(f.has_lighting);
  EXPECT_EQ(f.lighting.exposure, 0.25f);
  EXPECT_EQ(f.lighting.temperature, 6500.0f);
  // A repeated response is read again with the current h: still the last response's lighting.
  glue->Recenter();
  f = Step(glue.get(), 22);  // pipeline refilling: repeated
  EXPECT_TRUE(f.repeated);
  ASSERT_TRUE(f.has_lighting);
  EXPECT_EQ(f.lighting.exposure, 0.25f);
  // h off: no lighting, also for a repeated response.
  XrConfigureOptions off;
  off.enable_lighting = false;
  ASSERT_TRUE(glue->Configure(off).ok());
  EXPECT_FALSE(Step(glue.get(), 23).has_lighting);
  glue->Recenter();
  f = Step(glue.get(), 24);
  EXPECT_TRUE(f.repeated);
  EXPECT_FALSE(f.has_lighting);
}

TEST(Lighting, StartupRecenterRepublishesTheCallbackAfterTheConstantWindow) {
  // The evidence gate (RM-3a G-L1) derives the single callback where "previous fresh callback's
  // frame" differs from the original from kInitialFramesUntilRecenter: callback 0 only fills the
  // pipeline, callbacks 1..5 process frames (the 5th rebuilds the core), callback 6 finds an empty
  // pipeline and publishes the 5th response again.
  static_assert(kInitialFramesUntilRecenter == 5, "tracking-controller.cc:85");
  Script s;
  s.exposure = 0.25f;
  auto glue = MakeGlue(&s);
  XrConfigureOptions o;
  o.enable_lighting = true;
  ASSERT_TRUE(glue->Configure(o).ok());
  std::vector<PublishedFrame> f;
  for (int k = 0; k <= kInitialFramesUntilRecenter + 2; ++k) {
    s.exposure = 0.01f * static_cast<float>(k);
    f.push_back(Step(glue.get(), k));
  }
  EXPECT_TRUE(f[0].repeated);
  for (int k = 1; k <= kInitialFramesUntilRecenter; ++k) EXPECT_FALSE(f[k].repeated) << k;
  EXPECT_TRUE(f[kInitialFramesUntilRecenter + 1].repeated);
  EXPECT_EQ(f[kInitialFramesUntilRecenter + 1].lighting.exposure,
            f[kInitialFramesUntilRecenter].lighting.exposure);
  EXPECT_FALSE(f[kInitialFramesUntilRecenter + 2].repeated);
}

TEST(Lighting, NoLightingBeforeTheFirstResponse) {
  Script s;
  auto glue = MakeGlue(&s);
  XrConfigureOptions o;
  o.enable_lighting = true;
  ASSERT_TRUE(glue->Configure(o).ok());
  EXPECT_FALSE(Step(glue.get(), 0).has_lighting);  // `reality` does not exist yet
  EXPECT_TRUE(Step(glue.get(), 1).has_lighting);
}

// ------------------------------------------------------------------------------------------------
// US-006 tracking status events and the front camera.

TEST(TrackingStatusEvent, RaisedOnlyWhenStatusOrReasonChanges) {
  Script s;
  s.statuses = {{TrackingStatus::kLimited, TrackingReason::kInitializing},
                {TrackingStatus::kLimited, TrackingReason::kInitializing},
                {TrackingStatus::kNormal, TrackingReason::kUnspecified},
                {TrackingStatus::kNormal, TrackingReason::kUnspecified},
                {TrackingStatus::kLimited, TrackingReason::kUnspecified},
                {TrackingStatus::kLimited, TrackingReason::kInitializing}};
  auto glue = MakeGlue(&s);
  std::vector<bool> changed;
  for (int k = 0; k < 8; ++k) changed.push_back(Step(glue.get(), k).tracking_status_changed);
  // Frame 0: no response. Frames 1..6 publish calls 0..5; frame 6 is the startup re-publication
  // (repeat of call 4 = frame 5's response) and frame 7 publishes call 5.
  EXPECT_EQ(changed, (std::vector<bool>{false, true, false, true, false, true, false, true}));
}

TEST(FrontCamera, WorldTrackingOnTheFrontCameraIsRejected) {
  Script s;
  auto glue = MakeGlue(&s);
  EXPECT_TRUE(glue->OnCameraStatusChange("requesting", CameraDirection::kBack).ok());
  const XrError e = glue->OnCameraStatusChange("requesting", CameraDirection::kFront);
  EXPECT_EQ(e.code, "slam-front-camera-unsupported");
  EXPECT_EQ(e.message, "[XR] World tracking is only supported on the back camera");
  EXPECT_TRUE(glue->OnCameraStatusChange("hasStream", CameraDirection::kFront).ok());
  // The status change marks the session as started (AA).
  XrConfigureOptions abs;
  abs.absolute_scale = true;
  EXPECT_FALSE(glue->Configure(abs).ok());
}

TEST(FrontCamera, AllowedWithDisableWorldTracking) {
  Script s;
  auto glue = MakeGlue(&s);
  XrConfigureOptions o;
  o.disable_world_tracking = true;
  ASSERT_TRUE(glue->Configure(o).ok());
  EXPECT_TRUE(glue->OnCameraStatusChange("requesting", CameraDirection::kFront).ok());
}

// ------------------------------------------------------------------------------------------------
// FULL-7 (RM-3a2): non-finite origin / facing.

TEST(XrConfigureDeathTest, NonFiniteOriginOrFacingIsRejected) {
  // The original passes NaN / Infinity to capnp unchecked (no valid pose can come out of it); the
  // glue rejects them at construction and in updateCameraProjectionMatrix. The embind layer turns
  // the same input into a JS error before it reaches this check (xr_api.test.mjs).
  const float nan = std::nanf("");
  const float inf = std::numeric_limits<float>::infinity();
  Script s;
  for (const Vec3f& origin : {Vec3f{nan, 2.f, 0.f}, Vec3f{0.f, nan, 0.f}, Vec3f{0.f, 2.f, nan},
                              Vec3f{inf, 2.f, 0.f}, Vec3f{0.f, -inf, 0.f}}) {
    GlueConfig g;
    g.origin = origin;
    EXPECT_DEATH(MakeGlue(&s, Flip6Config(), g), "origin must be finite");
  }
  GlueConfig f;
  f.facing = QuatWxyz{1.f, nan, 0.f, 0.f};
  EXPECT_DEATH(MakeGlue(&s, Flip6Config(), f), "facing must be finite");
  auto glue = MakeGlue(&s);
  CameraProjectionUpdate u;
  u.origin = Vec3f{0.f, nan, 0.f};
  EXPECT_DEATH(glue->UpdateCameraProjectionMatrix(u), "origin must be finite");
  CameraProjectionUpdate v;
  v.facing = QuatWxyz{1.f, 0.f, inf, 0.f};
  EXPECT_DEATH(glue->UpdateCameraProjectionMatrix(v), "facing must be finite");
  // "A rejected update changes nothing" is NOT asserted here (RM-3c, API-2 review MINOR-7): the
  // rejection is an SR_CHECK abort, EXPECT_DEATH runs the statement in a child process, so the
  // parent's `glue` is never touched by the rejected calls and a check on it afterwards proves
  // nothing. After an abort there is no C++ state to preserve. The state-preserving behaviour exists
  // only at the embind layer, which validates before calling the glue and throws a JS error; that is
  // asserted in tests/wasm/xr_api.test.mjs ("still usable after the rejected calls").
}

// RM-3c (API-2 review MINOR-9): the injected-atlas row counts are public (image_types.h) and equal
// the internal layout's, so a native producer does not depend on internal headers.
TEST(PyramidAtlasRows, PublicCountsEqualTheEngineLayout) {
  for (const auto& wh : std::vector<std::pair<int, int>>{{480, 640}, {240, 320}, {360, 640}, {640, 480}}) {
    const frontend::PyramidLayout layout = frontend::ComputePyramidLayout(wh.first, wh.second);
    EXPECT_EQ(PyramidAtlasRows(wh.first, wh.second), layout.rows) << wh.first << "x" << wh.second;
    EXPECT_EQ(PyramidAtlasRoiRows(wh.first, wh.second), frontend::RoiAtlasRows(layout));
    // Image-target ROIs need more than the plain pyramid: the contract is not vacuous.
    EXPECT_GT(PyramidAtlasRoiRows(wh.first, wh.second), PyramidAtlasRows(wh.first, wh.second));
  }
  EXPECT_EQ(PyramidAtlasRows(0, 640), 0);
  EXPECT_EQ(PyramidAtlasRoiRows(480, -1), 0);
}

TEST(XrConfigure, FiniteZeroOriginYStillAccepted) {
  // origin.y = 0 stays valid (scale 0 is simply not sent, TEST above); only non-finite values abort.
  Script s;
  GlueConfig g;
  g.origin = Vec3f{0.f, 0.f, 0.f};
  EXPECT_NO_FATAL_FAILURE(MakeGlue(&s, Flip6Config(), g));
}

// ------------------------------------------------------------------------------------------------
// FULL-7 (RM-4b): framesUntilRecenter of an engineInit re-run.

TEST(StartupRecenter, DefaultIsTheOriginalsFiveFrames) {
  static_assert(kInitialFramesUntilRecenter == 5, "tracking-controller.cc:85");
  EXPECT_EQ(GlueConfig{}.frames_until_recenter, kInitialFramesUntilRecenter);
  Script s;
  auto glue = MakeGlue(&s);
  EXPECT_EQ(glue->frames_until_recenter(), 5);
  std::vector<int> remaining;
  for (int k = 0; k < 9; ++k) {
    Step(glue.get(), k);
    remaining.push_back(glue->frames_until_recenter());
  }
  // Frame 0 only fills the pipeline; frames 1..5 are processed (5 -> 0); the 5th rebuilds the core.
  EXPECT_EQ(remaining, (std::vector<int>{5, 4, 3, 2, 1, 0, 0, 0, 0}));
  EXPECT_EQ(s.cores_created, 2);
}

TEST(StartupRecenter, ZeroSkipsTheStartupRecenterLikeAnEngineInitRerun) {
  // The original keeps framesUntilRecenter in its EngineData singleton: a video size or
  // orientation change re-runs engineInit (rebuildEngine only) without a new startup recenter.
  // A replacement runner with frames_until_recenter = 0 publishes the tracked pose from its first
  // response on and never rebuilds the core by itself.
  Script s;
  s.positions = {Vec3f{0.f, 0.f, 0.f}, Vec3f{0.5f, 0.f, 0.f}};
  GlueConfig g;
  g.frames_until_recenter = 0;
  auto glue = MakeGlue(&s, Flip6Config(), g);
  EXPECT_EQ(glue->frames_until_recenter(), 0);
  Step(glue.get(), 0);
  Step(glue.get(), 1);  // call 0: the initial transform (recenterPose) -> the configured origin
  const PublishedFrame f = Step(glue.get(), 2);  // call 1: 0.5 internal x, scale 2
  EXPECT_EQ(s.cores_created, 1);
  EXPECT_NEAR(f.position.x, 1.0, 1e-6);
  EXPECT_NEAR(f.position.y, 2.0, 1e-6);
  // The default runner forces the first 5 responses to the origin.
  Script d;
  d.positions = s.positions;
  auto dflt = MakeGlue(&d);
  Step(dflt.get(), 0);
  Step(dflt.get(), 1);
  EXPECT_FLOAT_EQ(Step(dflt.get(), 2).position.x, 0.f);
}

TEST(StartupRecenter, ARemainingCountContinuesLikeTheSingletonCounter) {
  Script s;
  GlueConfig g;
  g.frames_until_recenter = 2;
  auto glue = MakeGlue(&s, Flip6Config(), g);
  Step(glue.get(), 0);
  EXPECT_EQ(glue->frames_until_recenter(), 2);
  Step(glue.get(), 1);
  EXPECT_EQ(glue->frames_until_recenter(), 1);
  EXPECT_EQ(s.cores_created, 1);
  Step(glue.get(), 2);  // passes 1 -> 0: rebuildEngine
  EXPECT_EQ(glue->frames_until_recenter(), 0);
  EXPECT_EQ(s.cores_created, 2);
  Step(glue.get(), 3);
  Step(glue.get(), 4);
  EXPECT_EQ(s.cores_created, 2);
}

TEST(StartupRecenterDeathTest, NegativeCounterIsRejected) {
  Script s;
  GlueConfig g;
  g.frames_until_recenter = -1;
  EXPECT_DEATH(MakeGlue(&s, Flip6Config(), g), "frames_until_recenter");
}

// ------------------------------------------------------------------------------------------------
// FULL-7 (RM-4b): optional DeviceMotion sensors (sensors.ts updateDeviceMotion).

// The events pushed before frame k are staged with it and reach the core when the pipeline's next
// frame is processed (1-frame lag): Track call n sees the events staged with frame n.
std::vector<ImuEvent> EventsStagedWithFrame(GlueRunner* glue, Script* s, int k) {
  Step(glue, k);
  Step(glue, k + 1);
  return s->imu.empty() ? std::vector<ImuEvent>{} : s->imu.back();
}

TEST(OptionalSensors, OnlyThePresentKindsEnterTheQueue) {
  struct Case {
    bool lin, acc, gyro;
    size_t events;
  };
  const Case cases[] = {{false, true, false, 1}, {false, false, true, 1}, {true, false, false, 1},
                        {false, false, false, 0}, {true, true, true, 3},  {false, true, true, 2}};
  for (const Case& c : cases) {
    Script s;
    auto glue = MakeGlue(&s);
    Step(glue.get(), 0);  // fills the pipeline (the queue is taken at every stage)
    DeviceMotionSample m;
    m.receive_time_ns = 1000000000;
    m.interval_ms = 16.666;
    if (c.lin) m.acceleration = Vec3f{0.1f, 0.2f, 0.3f};
    if (c.acc) m.acceleration_including_gravity = Vec3f{0.f, 9.81f, 0.f};
    if (c.gyro) m.rotation_rate_abg = Vec3f{1.f, 2.f, 3.f};
    glue->PushDeviceMotion(m);
    const std::vector<ImuEvent> ev = EventsStagedWithFrame(glue.get(), &s, 1);
    EXPECT_EQ(ev.size(), c.events) << c.lin << c.acc << c.gyro;
    if (c.acc && !c.lin && !c.gyro) {
      ASSERT_EQ(ev.size(), 1u);
      EXPECT_EQ(ev[0].kind, ImuEvent::Kind::kAccelerometer);
    }
    if (c.gyro && !c.lin && !c.acc) {
      ASSERT_EQ(ev.size(), 1u);
      EXPECT_EQ(ev[0].kind, ImuEvent::Kind::kGyroscope);
    }
  }
}

// ------------------------------------------------------------------------------------------------
// FULL-7 (RM-3b, review M3): xrHitTest under accuracy.metricOutput.

TEST(MetricHitTest, HitsAreInTheUnitOfThePublishedPoseAndWorldPoints) {
  // The pose and world points of a metric response are metric_pos + s * (x - responsive camera
  // position); before this fix xrHitTest still answered in responsive units (the original has no
  // metric mode, so the hit has to follow the published frame it is read against).
  Script s;
  s.points = {WorldPointOut{1, Vec3f{0.f, 0.f, 2.f}, 1.f}};
  s.positions = {Vec3f{0.f, 0.f, 0.f}, Vec3f{0.2f, 0.f, 0.1f}};
  EngineConfig e = Flip6Config();
  e.accuracy.enabled = true;
  e.accuracy.metric_output = true;
  Script r = s;
  auto metric = MakeGlue(&s, e);
  auto resp = MakeGlue(&r);
  Warm(metric.get(), 12);
  Warm(resp.get(), 12);
  const PublishedFrame fm = Step(metric.get(), 20);
  const PublishedFrame fr = Step(resp.get(), 20);
  ASSERT_TRUE(fm.metric);
  ASSERT_EQ(fm.world_points.size(), 1u);
  ASSERT_EQ(fr.world_points.size(), 1u);
  const double scale = fm.metric_scale;
  ASSERT_GT(scale, 0.0);
  // Sanity: the unit really differs (otherwise the test proves nothing).
  ASSERT_GT(std::fabs(fm.position.z - fr.position.z) + std::fabs(fm.position.y - fr.position.y), 1e-3);
  const std::vector<XrHitResult> hm = metric->XrHitTest(0.5f, 0.5f, {});
  const std::vector<XrHitResult> hr = resp->XrHitTest(0.5f, 0.5f, {});
  ASSERT_EQ(hm.size(), 1u);
  ASSERT_EQ(hr.size(), 1u);
  // The centre ray lands on the single point: the hit coincides with the published world point in
  // either unit.
  EXPECT_NEAR(hr[0].position.x, fr.world_points[0].position.x, 1e-4);
  EXPECT_NEAR(hr[0].position.z, fr.world_points[0].position.z, 1e-4);
  EXPECT_NEAR(hm[0].position.x, fm.world_points[0].position.x, 1e-4);
  EXPECT_NEAR(hm[0].position.y, fm.world_points[0].position.y, 1e-4);
  EXPECT_NEAR(hm[0].position.z, fm.world_points[0].position.z, 1e-4);
  // Distance: camera to hit in metres = s * responsive distance; the metric camera is the
  // published metric position.
  EXPECT_NEAR(hm[0].distance, scale * hr[0].distance, 1e-4 * std::max(1.0, static_cast<double>(hm[0].distance)));
  const double dx = hm[0].position.x - fm.position.x, dy = hm[0].position.y - fm.position.y,
               dz = hm[0].position.z - fm.position.z;
  EXPECT_NEAR(std::sqrt(dx * dx + dy * dy + dz * dz), hm[0].distance, 1e-4 * std::max(1.0, static_cast<double>(hm[0].distance)));
  // Rotation is not a position: unchanged.
  EXPECT_EQ(hm[0].rotation.w, hr[0].rotation.w);
  EXPECT_EQ(hm[0].rotation.z, hr[0].rotation.z);
}

TEST(MetricHitTest, WithoutMetricOutputTheHitIsUnchanged) {
  // Default (and accuracy-enabled without metricOutput): bit-identical to the responsive hit.
  Script a, b;
  a.positions = b.positions = {Vec3f{0.f, 0.f, 0.f}, Vec3f{0.2f, 0.f, 0.1f}};
  EngineConfig e = Flip6Config();
  e.accuracy.enabled = true;
  e.accuracy.metric_output = false;
  auto ga = MakeGlue(&a, e);
  auto gb = MakeGlue(&b);
  Warm(ga.get(), 12);
  Warm(gb.get(), 12);
  const std::vector<XrHitResult> ha = ga->XrHitTest(0.4f, 0.55f, {});
  const std::vector<XrHitResult> hb = gb->XrHitTest(0.4f, 0.55f, {});
  ASSERT_EQ(ha.size(), 1u);
  ASSERT_EQ(hb.size(), 1u);
  EXPECT_EQ(std::memcmp(&ha[0].position, &hb[0].position, sizeof(Vec3f)), 0);
  EXPECT_EQ(std::memcmp(&ha[0].distance, &hb[0].distance, sizeof(float)), 0);
}

TEST(MetricHitTest, SurfaceHitsAreInMetres) {
  // detect_surfaces keeps the floor in the published (metric) frame; the display ray must be in
  // it too, or the floor intersection is wrong. The floor of the grid scene is at internal
  // y = -1: published y = 0 responsive; in metres it is where the detector put it.
  Script s;
  s.points.clear();
  uint64_t id = 1;
  for (int i = 0; i < 12; ++i) {
    for (int j = 0; j < 12; ++j) {
      s.points.push_back(WorldPointOut{id++, Vec3f{-1.5f + 0.25f * i, -1.f, 0.6f + 0.3f * j}, 1.f});
    }
  }
  EngineConfig e = Flip6Config();
  e.accuracy.enabled = true;
  e.accuracy.metric_output = true;
  GlueConfig g;
  g.detect_surfaces = true;
  auto glue = MakeGlue(&s, e, g);
  Warm(glue.get(), 25);
  ASSERT_TRUE(glue->surfaces().floor_detected);
  size_t surface_hits = 0;
  for (const float y : {0.15f, 0.85f}) {
    for (const XrHitResult& r :
         glue->XrHitTest(0.5f, y, {XrHitType::kEstimatedSurface, XrHitType::kDetectedSurface})) {
      ++surface_hits;
      EXPECT_NEAR(r.position.y, glue->surfaces().floor.height, 1e-3);
      // The ray starts at the published (metric) camera: distance = |hit - published position|.
      const Vec3f& cam = glue->last_published().position;
      const double dx = r.position.x - cam.x, dy = r.position.y - cam.y, dz = r.position.z - cam.z;
      EXPECT_NEAR(std::sqrt(dx * dx + dy * dy + dz * dz), r.distance, 1e-3 * std::max(1.0, static_cast<double>(r.distance)));
    }
  }
  EXPECT_GT(surface_hits, 0u);
}

// RM-3c (API-2 review MAJOR-2): the three tests above never push device motion, so the metric
// tracker keeps its prior scale for the whole session and metric_pos == s * p exactly; the map
// `metric_pos + s * (x - p)` then coincides with the plain enlargement `s * x` and a mutant that
// returns `s * x` passes. Here the accelerometer drives the scale away from the prior, which
// re-anchors the metric position (metric_pos != s * p) and separates the two maps.

// Scripted camera motion in internal units (the core's pose translation at frame time t_ns) and
// the accelerometer a phone in this motion would measure, for a true metres-per-published-unit
// `s_true`: published position = origin + scale_ * internal, so the published acceleration is
// scale_ * x'' and the accelerometer reads s_true * that + gravity (identity rotation: device
// frame = published camera frame).
struct MotionScript {
  static constexpr double kPublishedPerInternal = 2.0;  // origin.y = 2 (GlueConfig default)
  // Internal world axes are x right, y up, z forward; the published frame is GL (z backward).
  static constexpr double kPublishedSign[3] = {1.0, 1.0, -1.0};
  static constexpr double kAmp[3][2] = {{0.040, 0.030}, {0.020, 0.012}, {0.030, 0.025}};
  static constexpr double kHz[3][2] = {{0.90, 1.60}, {0.70, 1.30}, {1.20, 1.90}};
  static constexpr double kPhase[3][2] = {{0.0, 0.7}, {0.4, 1.9}, {1.1, 0.2}};
  static constexpr double kTwoPi = 6.283185307179586;

  static double Pos(int axis, double t) {
    double v = 0;
    for (int k = 0; k < 2; ++k) v += kAmp[axis][k] * std::sin(kTwoPi * kHz[axis][k] * t + kPhase[axis][k]);
    return v;
  }
  static double Acc(int axis, double t) {
    double v = 0;
    for (int k = 0; k < 2; ++k) {
      const double w = kTwoPi * kHz[axis][k];
      v -= kAmp[axis][k] * w * w * std::sin(w * t + kPhase[axis][k]);
    }
    return v;
  }
  static double TimeS(int64_t t_ns) { return static_cast<double>(t_ns - 1000000000) * 1e-9; }
};

// Frame k of Step() is at 1e9 + k * 33e6 ns; Track call k is that frame (1-frame publish lag).
Vec3f ScriptedPosition(int k) {
  const double t = MotionScript::TimeS(1000000000 + static_cast<int64_t>(k) * 33000000);
  return Vec3f{static_cast<float>(MotionScript::Pos(0, t)), static_cast<float>(MotionScript::Pos(1, t)),
               static_cast<float>(MotionScript::Pos(2, t))};
}

void PushAccelerometerFor(GlueRunner* glue, int k, double s_true) {
  for (const int64_t back : {int64_t{16500000}, int64_t{0}}) {
    const int64_t ns = 1000000000 + static_cast<int64_t>(k) * 33000000 - back;
    const double t = MotionScript::TimeS(ns);
    DeviceMotionSample m;
    m.receive_time_ns = ns;
    m.interval_ms = 16.5;
    m.acceleration_including_gravity =
        Vec3f{static_cast<float>(s_true * MotionScript::kPublishedPerInternal * MotionScript::kPublishedSign[0] * MotionScript::Acc(0, t)),
              static_cast<float>(s_true * MotionScript::kPublishedPerInternal * MotionScript::kPublishedSign[1] * MotionScript::Acc(1, t) + 9.80665),
              static_cast<float>(s_true * MotionScript::kPublishedPerInternal * MotionScript::kPublishedSign[2] * MotionScript::Acc(2, t))};
    glue->PushDeviceMotion(m);
  }
}

TEST(MetricHitTest, HitFollowsTheMapOfAScaleThatTheAccelerometerMoved) {
  constexpr double kTrueScale = 0.45;  // metres per published unit; the prior is 0.7
  constexpr int kFrames = 330;         // 10.9 s
  Script s;
  s.points = {WorldPointOut{1, Vec3f{0.f, 0.f, 2.f}, 1.f}};
  for (int k = 0; k < kFrames + 2; ++k) s.positions.push_back(ScriptedPosition(k));
  Script r = s;
  EngineConfig e = Flip6Config();
  e.accuracy.enabled = true;
  e.accuracy.metric_output = true;
  GlueConfig g;
  g.frames_until_recenter = 0;  // no startup recenter window: the metric tracker is fed from frame 1
  auto metric = MakeGlue(&s, e, g);
  auto resp = MakeGlue(&r, Flip6Config(), g);
  PublishedFrame fm, fr;
  for (int k = 0; k < kFrames; ++k) {
    PushAccelerometerFor(metric.get(), k, kTrueScale);
    fm = Step(metric.get(), k);
    fr = Step(resp.get(), k);
  }
  ASSERT_TRUE(fm.metric);
  ASSERT_TRUE(fm.metric_initialized) << "the scale did not converge: scenario is vacuous";
  const double scale = fm.metric_scale;
  std::printf("[MetricHitTest scale] prior=0.7 true=%.2f estimated=%.4f sigma_log=%.3f lag=%.3f\n", kTrueScale, scale,
              fm.metric_sigma_log, fm.metric_lag_s);
  // Non-vacuity 1: the accelerometer moved the scale well away from the prior.
  ASSERT_GT(std::fabs(scale - 0.7), 0.1);
  // Non-vacuity 2: the metric position is no longer s * p, so `s * x` and `metric_pos + s * (x - p)`
  // differ by more than the test tolerance at every point of this frame.
  const double off = std::sqrt(std::pow(fm.position.x - scale * fr.position.x, 2) +
                               std::pow(fm.position.y - scale * fr.position.y, 2) +
                               std::pow(fm.position.z - scale * fr.position.z, 2));
  std::printf("[MetricHitTest offset] |metric_pos - s * p| = %.4f m\n", off);
  ASSERT_GT(off, 0.1);
  ASSERT_EQ(fm.world_points.size(), 1u);
  ASSERT_EQ(fr.world_points.size(), 1u);
  // The world point follows the same map; this is the independent anchor of the expectation.
  EXPECT_NEAR(fm.world_points[0].position.x, fm.position.x + scale * (fr.world_points[0].position.x - fr.position.x), 1e-4);
  EXPECT_NEAR(fm.world_points[0].position.y, fm.position.y + scale * (fr.world_points[0].position.y - fr.position.y), 1e-4);
  EXPECT_NEAR(fm.world_points[0].position.z, fm.position.z + scale * (fr.world_points[0].position.z - fr.position.z), 1e-4);
  size_t compared = 0;
  for (const auto& q : std::vector<std::pair<float, float>>{{0.5f, 0.5f}, {0.45f, 0.55f}, {0.55f, 0.45f}, {0.5f, 0.6f}}) {
    const std::vector<XrHitResult> hm = metric->XrHitTest(q.first, q.second, {});
    const std::vector<XrHitResult> hr = resp->XrHitTest(q.first, q.second, {});
    ASSERT_EQ(hm.size(), hr.size());
    for (size_t i = 0; i < hm.size(); ++i) {
      ++compared;
      // Responsive hit (the original's) mapped like the response it is read against.
      EXPECT_NEAR(hm[i].position.x, fm.position.x + scale * (hr[i].position.x - fr.position.x), 1e-4);
      EXPECT_NEAR(hm[i].position.y, fm.position.y + scale * (hr[i].position.y - fr.position.y), 1e-4);
      EXPECT_NEAR(hm[i].position.z, fm.position.z + scale * (hr[i].position.z - fr.position.z), 1e-4);
      EXPECT_NEAR(hm[i].distance, scale * hr[i].distance, 1e-4 * std::max(1.0, static_cast<double>(hm[i].distance)));
      const double dx = hm[i].position.x - fm.position.x, dy = hm[i].position.y - fm.position.y,
                   dz = hm[i].position.z - fm.position.z;
      EXPECT_NEAR(std::sqrt(dx * dx + dy * dy + dz * dz), hm[i].distance,
                  1e-4 * std::max(1.0, static_cast<double>(hm[i].distance)));
    }
  }
  EXPECT_GT(compared, 0u);
}

}  // namespace
}  // namespace slam_refact
