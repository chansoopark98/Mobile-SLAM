// US-023 (contract FULL-8): landscape captures and rotation 0 / 90 / 180 / 270 (R16 lifted).
// Checked-in constants only (label unit). The expectations are written independently of the code under
// test: double-precision 3x3 algebra for the pose, spelled-out tables for the projection reorder and the
// hit input turn (sources: PUB:reality/engine/geometry/orientation.cc:37-110, 437-453,
// reality/engine/executor/xr-engine.cc:111-135, reality/app/xr/js/tracking-controller.cc:152-167).
// What this file does NOT show: that the original behaves this way (that is the recorded-original gate,
// tests/engine/orientation_gate_test.cc and build/slam-refact-full-20261019/preregistration/orient-*.md).

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <vector>

#include "engine/device_orientation.h"
#include "engine/hit_test.h"
#include "slam_refact/engine.h"
#include "slam_refact/glue_runner.h"

namespace slam_refact {
namespace {

struct Mat3 {
  double a[3][3];
};

Mat3 Mul(const Mat3& x, const Mat3& y) {
  Mat3 r{};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      for (int k = 0; k < 3; ++k) r.a[i][j] += x.a[i][k] * y.a[k][j];
  return r;
}

Mat3 FromQuat(double w, double x, double y, double z) {
  const double n = std::sqrt(w * w + x * x + y * y + z * z);
  w /= n;
  x /= n;
  y /= n;
  z /= n;
  return Mat3{{{1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)},
               {2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)},
               {2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)}}};
}

// In-plane rotation about z by `deg` (0, 90, 180, 270), counter-clockwise: HMatrixGen::z90() is
// [[0, -1], [1, 0]] (hmatrix.cc:516-525).
Mat3 Rz(int deg) {
  const int c = deg == 0 ? 1 : deg == 180 ? -1 : 0;
  const int s = deg == 90 ? 1 : deg == 270 ? -1 : 0;
  return Mat3{{{double(c), double(-s), 0}, {double(s), double(c), 0}, {0, 0, 1}}};
}

// The in-plane angle updateCameraExtrinsics multiplies by, per rotation (EngineConfig::rotation_deg).
int ExtrinsicAngle(int rotation_deg) {
  switch (RotationToDeviceOrientation(rotation_deg)) {
    case DeviceOrientation::kLandscapeRight: return 90;
    case DeviceOrientation::kPortraitUpsideDown: return 180;
    case DeviceOrientation::kLandscapeLeft: return 270;
    default: return 0;
  }
}

void ExpectMatNear(const Mat3& got, const Mat3& want, double tol, const char* what) {
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) EXPECT_NEAR(got.a[i][j], want.a[i][j], tol) << what << " (" << i << "," << j << ")";
}

TEST(DeviceOrientation, RotationMapping) {
  // tracking-controller.cc:152-167, spelled out.
  struct Row { int rotation; DeviceOrientation want; };
  const Row rows[] = {{0, DeviceOrientation::kPortrait},          {90, DeviceOrientation::kLandscapeLeft},
                      {-270, DeviceOrientation::kLandscapeLeft},  {180, DeviceOrientation::kPortraitUpsideDown},
                      {-180, DeviceOrientation::kPortraitUpsideDown}, {270, DeviceOrientation::kLandscapeRight},
                      {-90, DeviceOrientation::kLandscapeRight},  {45, DeviceOrientation::kUnspecified},
                      {360, DeviceOrientation::kUnspecified},     {-1, DeviceOrientation::kUnspecified}};
  for (const Row& r : rows) {
    EXPECT_EQ(RotationToDeviceOrientation(r.rotation), r.want) << r.rotation;
    EXPECT_EQ(IsSupportedRotation(r.rotation), r.want != DeviceOrientation::kUnspecified) << r.rotation;
  }
  EXPECT_TRUE(IsLandscape(DeviceOrientation::kLandscapeLeft));
  EXPECT_TRUE(IsLandscape(DeviceOrientation::kLandscapeRight));
  EXPECT_FALSE(IsLandscape(DeviceOrientation::kPortraitUpsideDown));
  EXPECT_FALSE(IsLandscape(DeviceOrientation::kPortrait));
}

TEST(DeviceOrientation, GraphicsSizeIsExchangedForLandscapeOnly) {
  for (int rotation : {0, 180, -180}) {
    int w = 640, h = 480;
    PortraitGraphicsSize(RotationToDeviceOrientation(rotation), &w, &h);
    EXPECT_EQ(w, 640);
    EXPECT_EQ(h, 480);
  }
  for (int rotation : {90, 270, -90, -270}) {
    int w = 640, h = 480;
    PortraitGraphicsSize(RotationToDeviceOrientation(rotation), &w, &h);
    EXPECT_EQ(w, 480);
    EXPECT_EQ(h, 640);
  }
}

std::array<float, 16> DistinctProjection() {
  std::array<float, 16> m{};
  for (int k = 0; k < 16; ++k) m[static_cast<size_t>(k)] = 100.f + static_cast<float>(k);
  return m;
}

TEST(DeviceOrientation, ProjectionReorderTables) {
  const std::array<float, 16> in = DistinctProjection();
  const float r0x = in[0], r0z = in[8], r1y = in[5], r1z = in[9];
  // Element positions: (0,0) = [0], (0,2) = [8], (1,1) = [5], (1,2) = [9].
  auto expect = [&](DeviceOrientation o, float m00, float m02, float m11, float m12) {
    const std::array<float, 16> out = PortraitToDeviceProjection(o, in);
    EXPECT_EQ(out[0], m00);
    EXPECT_EQ(out[8], m02);
    EXPECT_EQ(out[5], m11);
    EXPECT_EQ(out[9], m12);
    for (int k = 0; k < 16; ++k) {  // every other element is untouched
      if (k != 0 && k != 8 && k != 5 && k != 9) {
        EXPECT_EQ(out[static_cast<size_t>(k)], in[static_cast<size_t>(k)]) << k;
      }
    }
  };
  expect(DeviceOrientation::kPortrait, r0x, r0z, r1y, r1z);
  expect(DeviceOrientation::kUnspecified, r0x, r0z, r1y, r1z);
  expect(DeviceOrientation::kLandscapeRight, r1y, r1z, r0x, -r0z);   // orientation.cc:54-58
  expect(DeviceOrientation::kPortraitUpsideDown, r0x, -r0z, r1y, -r1z);  // :60-62
  expect(DeviceOrientation::kLandscapeLeft, r1y, -r1z, r0x, r0z);    // :64-68
}

TEST(DeviceOrientation, ProjectionReorderMetamorphic) {
  const std::array<float, 16> in = DistinctProjection();
  // Upside-down twice is the identity; landscape right then left (both reorder the same 4 slots) swaps
  // m00 and m11 twice and flips each skew sign twice only when applied to the portrait matrix, so test
  // the invariants that hold for any input: the diagonal pair (m00, m11) is exchanged by both landscape
  // orientations and kept by upside-down.
  EXPECT_EQ(PortraitToDeviceProjection(DeviceOrientation::kPortraitUpsideDown,
                                       PortraitToDeviceProjection(DeviceOrientation::kPortraitUpsideDown, in)),
            in);
  for (DeviceOrientation o : {DeviceOrientation::kLandscapeLeft, DeviceOrientation::kLandscapeRight}) {
    const std::array<float, 16> out = PortraitToDeviceProjection(o, in);
    EXPECT_EQ(out[0], in[5]);
    EXPECT_EQ(out[5], in[0]);
  }
  // Left and right differ exactly in the signs of the two skew terms.
  const std::array<float, 16> l = PortraitToDeviceProjection(DeviceOrientation::kLandscapeLeft, in);
  const std::array<float, 16> r = PortraitToDeviceProjection(DeviceOrientation::kLandscapeRight, in);
  EXPECT_EQ(l[8], -r[8]);
  EXPECT_EQ(l[9], -r[9]);
}

TEST(DeviceOrientation, PoseIsPostMultipliedByTheInPlaneRotation) {
  // A generic rigid pose.
  // A generic unit quaternion (0.8, 0.1, -0.45, 0.3 normalised in double).
  const double qn = std::sqrt(0.8 * 0.8 + 0.1 * 0.1 + 0.45 * 0.45 + 0.3 * 0.3);
  const double qw = 0.8 / qn, qx = 0.1 / qn, qy = -0.45 / qn, qz = 0.3 / qn;
  for (int rotation : {0, 90, 180, 270, -90, -180, -270}) {
    c8::HVector3 t(0.25f, -1.5f, 3.0f);
    c8::Quaternion q(static_cast<float>(qw), static_cast<float>(qx), static_cast<float>(qy), static_cast<float>(qz));
    const Mat3 before = FromQuat(q.w(), q.x(), q.y(), q.z());
    PortraitToDevicePose(RotationToDeviceOrientation(rotation), &t, &q);
    const Mat3 after = FromQuat(q.w(), q.x(), q.y(), q.z());
    ExpectMatNear(after, Mul(before, Rz(ExtrinsicAngle(rotation))), 2e-6, "R' = R * Rz");
    // The correction has no translation: the camera position is unchanged.
    EXPECT_FLOAT_EQ(t.x(), 0.25f);
    EXPECT_FLOAT_EQ(t.y(), -1.5f);
    EXPECT_FLOAT_EQ(t.z(), 3.0f);
  }
  // 90 followed by 270 (LANDSCAPE_RIGHT then LANDSCAPE_LEFT) is the identity; 90 twice is 180.
  c8::HVector3 t(1.f, 2.f, 3.f);
  c8::Quaternion q(static_cast<float>(qw), static_cast<float>(qx), static_cast<float>(qy), static_cast<float>(qz));
  const Mat3 start = FromQuat(q.w(), q.x(), q.y(), q.z());
  PortraitToDevicePose(DeviceOrientation::kLandscapeRight, &t, &q);
  PortraitToDevicePose(DeviceOrientation::kLandscapeLeft, &t, &q);
  ExpectMatNear(FromQuat(q.w(), q.x(), q.y(), q.z()), start, 4e-6, "R * Rz(90) * Rz(270)");
}

TEST(DeviceOrientation, HitInputTurn) {
  // xr-engine.cc:111-135 spelled out: (x, y) are the mirror-corrected display coordinates.
  float x = 0.f, y = 0.f;
  DeviceToPortraitHitInput(DeviceOrientation::kPortrait, 0.2f, 0.7f, &x, &y);
  EXPECT_EQ(x, 0.2f);
  EXPECT_EQ(y, 0.7f);
  DeviceToPortraitHitInput(DeviceOrientation::kLandscapeRight, 0.2f, 0.7f, &x, &y);
  EXPECT_EQ(x, 0.7f);
  EXPECT_EQ(y, 1 - 0.2f);
  DeviceToPortraitHitInput(DeviceOrientation::kPortraitUpsideDown, 0.2f, 0.7f, &x, &y);
  EXPECT_EQ(x, 1 - 0.2f);
  EXPECT_EQ(y, 1 - 0.7f);
  DeviceToPortraitHitInput(DeviceOrientation::kLandscapeLeft, 0.2f, 0.7f, &x, &y);
  EXPECT_EQ(x, 1 - 0.7f);
  EXPECT_EQ(y, 0.2f);
  // The turn of the display centre is the centre for every orientation.
  for (DeviceOrientation o : {DeviceOrientation::kLandscapeLeft, DeviceOrientation::kLandscapeRight,
                              DeviceOrientation::kPortraitUpsideDown}) {
    DeviceToPortraitHitInput(o, 0.5f, 0.5f, &x, &y);
    EXPECT_EQ(x, 0.5f);
    EXPECT_EQ(y, 0.5f);
  }
}

HitTestState SampleHitState(DeviceOrientation o) {
  HitTestState s;
  s.camera_position = Vec3f{0.1f, 0.2f, -0.3f};
  s.camera_rotation = QuatWxyz{0.96f, 0.05f, 0.2f, -0.1f};
  const float fx = 2.1f, fy = 1.5f;
  s.projection = {fx, 0, 0, 0, 0, fy, 0, 0, 0.01f, -0.02f, -1.0001f, -1, 0, 0, -0.02f, 0};
  uint32_t lcg = 12345;
  auto rnd = [&lcg] { lcg = lcg * 1664525u + 1013904223u; return static_cast<float>((lcg >> 8) & 0xFFFF) / 65536.0f; };
  for (int k = 0; k < 60; ++k) s.points.push_back(Vec3f{rnd() * 4 - 2, rnd() * 4 - 2, rnd() * 3 + 1});
  s.orientation = o;
  return s;
}

TEST(DeviceOrientation, FeaturePointHitInputIsTurnedBeforeTheRay) {
  const uint32_t mask = HitTypeMask({XrHitType::kFeaturePoint});
  for (float x : {0.1f, 0.5f, 0.83f}) {
    for (float y : {0.2f, 0.5f, 0.9f}) {
      const HitTestState portrait = SampleHitState(DeviceOrientation::kPortrait);
      const std::vector<InternalHit> base_left =
          FeaturePointHitTest(portrait, 1 - y, x, mask, false);  // LANDSCAPE_LEFT turns (x, y) into (1 - y, x)
      const std::vector<InternalHit> left =
          FeaturePointHitTest(SampleHitState(DeviceOrientation::kLandscapeLeft), x, y, mask, false);
      ASSERT_EQ(base_left.size(), 1u);
      ASSERT_EQ(left.size(), 1u);
      EXPECT_EQ(std::memcmp(&base_left[0].position, &left[0].position, sizeof(Vec3f)), 0);
      EXPECT_EQ(base_left[0].distance, left[0].distance);
      const std::vector<InternalHit> base_right = FeaturePointHitTest(portrait, y, 1 - x, mask, false);
      const std::vector<InternalHit> right =
          FeaturePointHitTest(SampleHitState(DeviceOrientation::kLandscapeRight), x, y, mask, false);
      EXPECT_EQ(std::memcmp(&base_right[0].position, &right[0].position, sizeof(Vec3f)), 0);
      const std::vector<InternalHit> base_flip = FeaturePointHitTest(portrait, 1 - x, 1 - y, mask, false);
      const std::vector<InternalHit> flip =
          FeaturePointHitTest(SampleHitState(DeviceOrientation::kPortraitUpsideDown), x, y, mask, false);
      EXPECT_EQ(std::memcmp(&base_flip[0].position, &flip[0].position, sizeof(Vec3f)), 0);
      // Mirror first: mirrored LANDSCAPE_LEFT at (x, y) is LANDSCAPE_LEFT at (1 - x, y) unmirrored.
      const std::vector<InternalHit> mirrored =
          FeaturePointHitTest(SampleHitState(DeviceOrientation::kLandscapeLeft), x, y, mask, true);
      const std::vector<InternalHit> unmirrored =
          FeaturePointHitTest(SampleHitState(DeviceOrientation::kLandscapeLeft), 1 - x, y, mask, false);
      EXPECT_EQ(std::memcmp(&mirrored[0].position, &unmirrored[0].position, sizeof(Vec3f)), 0);
    }
  }
  // Metamorphic: a landscape query is not the portrait query (the turn changes the result).
  const std::vector<InternalHit> p = FeaturePointHitTest(SampleHitState(DeviceOrientation::kPortrait), 0.2f, 0.8f, mask, false);
  const std::vector<InternalHit> l = FeaturePointHitTest(SampleHitState(DeviceOrientation::kLandscapeLeft), 0.2f, 0.8f, mask, false);
  EXPECT_NE(std::memcmp(&p[0].position, &l[0].position, sizeof(Vec3f)), 0);
}

// ---- GlueRunner with a scripted core ----

// Call k (1-based): a rotation about a tilted axis growing with k, translation 0.1 k.
class ScriptedCore final : public TrackerCore {
 public:
  TrackerStep Track(const FrameInput&) override {
    ++calls_;
    const double a = 0.15 * calls_;
    // R = Rz(a) * Rx(0.4): column-major 4x4.
    const double ca = std::cos(a), sa = std::sin(a), cx = std::cos(0.4), sx = std::sin(0.4);
    const double r[3][3] = {{ca, -sa * cx, sa * sx}, {sa, ca * cx, -ca * sx}, {0, sx, cx}};
    TrackerStep s;
    for (int row = 0; row < 3; ++row)
      for (int col = 0; col < 3; ++col) s.pose.m[4 * col + row] = static_cast<float>(r[row][col]);
    s.pose.m[12] = 0.1f * calls_;
    s.pose.m[13] = 0.05f * calls_;
    s.pose.m[14] = 0.2f * calls_;
    s.tam_extrinsic = s.pose;
    s.predicted_pose = s.pose;
    s.status = TrackingStatus::kNormal;
    for (int k = 0; k < 40; ++k) {
      s.world_points_internal.push_back(WorldPointOut{static_cast<uint64_t>(k + 1),
                                                      Vec3f{0.1f * k - 2.f, 0.07f * k - 1.f, 1.5f + 0.05f * k}, 0.5f});
    }
    return s;
  }

 private:
  int calls_ = 0;
};

EngineConfig ScriptedConfig(int rotation) {
  EngineConfig c;
  const bool landscape = RotationToDeviceOrientation(rotation) == DeviceOrientation::kLandscapeLeft ||
                         RotationToDeviceOrientation(rotation) == DeviceOrientation::kLandscapeRight;
  c.capture_width = landscape ? 640 : 480;
  c.capture_height = landscape ? 480 : 640;
  c.rotation_deg = rotation;
  c.manufacturer = "";
  c.model = "K";
  return c;
}

std::vector<PublishedFrame> RunScripted(int rotation, bool mirrored, GlueRunner** keep = nullptr) {
  GlueConfig g;
  g.frames_until_recenter = 0;
  g.mirrored_display = mirrored;
  static std::vector<std::unique_ptr<GlueRunner>> keepalive;
  keepalive.push_back(std::make_unique<GlueRunner>(
      ScriptedConfig(rotation), g, [](const EngineConfig&) -> std::unique_ptr<TrackerCore> { return std::make_unique<ScriptedCore>(); }));
  GlueRunner* glue = keepalive.back().get();
  if (keep != nullptr) *keep = glue;
  std::vector<PublishedFrame> out;
  uint8_t pixel = 0;
  for (int k = 0; k < 8; ++k) {
    FrameInput in;
    in.time_ns = 1000 + k;
    in.video_time_ns = in.time_ns;
    in.gray = GrayImageView{&pixel, 1, 1, 1};
    out.push_back(glue->Publish(in));
  }
  return out;
}

TEST(GlueOrientation, PublishedPoseIsThePortraitPoseTimesTheInPlaneRotation) {
  const std::vector<PublishedFrame> portrait = RunScripted(0, false);
  for (int rotation : {90, 180, 270, -90, -180, -270}) {
    const std::vector<PublishedFrame> turned = RunScripted(rotation, false);
    ASSERT_EQ(turned.size(), portrait.size());
    for (size_t k = 2; k < turned.size(); ++k) {
      SCOPED_TRACE(::testing::Message() << "rotation " << rotation << " frame " << k);
      // The camera position is not touched by the correction (the later 704 steps are position-wise the same).
      EXPECT_EQ(std::memcmp(&turned[k].position, &portrait[k].position, sizeof(Vec3f)), 0);
      const Mat3 base = FromQuat(portrait[k].rotation.w, portrait[k].rotation.x, portrait[k].rotation.y, portrait[k].rotation.z);
      const Mat3 got = FromQuat(turned[k].rotation.w, turned[k].rotation.x, turned[k].rotation.y, turned[k].rotation.z);
      // makeRightHanded is a conjugation by diag(1, 1, -1), which commutes with an in-plane rotation, and
      // recenterAndScale pre-multiplies: the whole published rotation is R_portrait * Rz.
      ExpectMatNear(got, Mul(base, Rz(ExtrinsicAngle(rotation))), 5e-6, "published rotation");
      // World points are not turned.
      ASSERT_EQ(turned[k].world_points.size(), portrait[k].world_points.size());
      for (size_t i = 0; i < turned[k].world_points.size(); ++i) {
        EXPECT_EQ(std::memcmp(&turned[k].world_points[i].position, &portrait[k].world_points[i].position, sizeof(Vec3f)), 0);
      }
    }
  }
}

TEST(GlueOrientation, RotationIsNotAnIdentityForTheScriptedPose) {
  // Guard against a vacuous comparison: the scripted pose is not symmetric under the in-plane turns.
  const std::vector<PublishedFrame> portrait = RunScripted(0, false);
  const std::vector<PublishedFrame> turned = RunScripted(90, false);
  EXPECT_NE(std::memcmp(&portrait[5].rotation, &turned[5].rotation, sizeof(QuatWxyz)), 0);
}

TEST(GlueOrientation, PublishedIntrinsicsFollowTheOrientation) {
  // Default graphics size = the capture size as given (canvas follows the video): 480x640 portrait,
  // 640x480 landscape; landscape exchanges it back before the portrait projection is computed.
  const std::vector<PublishedFrame> portrait = RunScripted(0, false);
  const std::array<float, 16>& p = portrait[3].intrinsics;
  EXPECT_NE(p[0], 0.f);
  for (int rotation : {90, 180, 270, -90}) {
    const std::vector<PublishedFrame> turned = RunScripted(rotation, false);
    const std::array<float, 16>& m = turned[3].intrinsics;
    SCOPED_TRACE(rotation);
    const DeviceOrientation o = RotationToDeviceOrientation(rotation);
    EXPECT_EQ(m, PortraitToDeviceProjection(o, p));  // the same portrait projection, reordered
    if (o == DeviceOrientation::kLandscapeLeft || o == DeviceOrientation::kLandscapeRight) {
      EXPECT_EQ(m[0], p[5]);
      EXPECT_EQ(m[5], p[0]);
    } else {
      EXPECT_EQ(m[0], p[0]);
      EXPECT_EQ(m[5], p[5]);
    }
    EXPECT_EQ(m[10], p[10]);  // depth terms untouched
    EXPECT_EQ(m[14], p[14]);
  }
}

TEST(GlueOrientation, XrHitTestTurnsTheInputByTheOrientationOfTheResponse) {
  GlueRunner* portrait_glue = nullptr;
  RunScripted(0, false, &portrait_glue);
  for (int rotation : {90, 270, 180}) {
    GlueRunner* glue = nullptr;
    RunScripted(rotation, false, &glue);
    for (float x : {0.15f, 0.6f}) {
      for (float y : {0.3f, 0.75f}) {
        float px = 0.f, py = 0.f;
        DeviceToPortraitHitInput(RotationToDeviceOrientation(rotation), x, y, &px, &py);
        const std::vector<XrHitResult> a = glue->XrHitTest(x, y, {XrHitType::kFeaturePoint});
        const std::vector<XrHitResult> b = portrait_glue->XrHitTest(px, py, {XrHitType::kFeaturePoint});
        ASSERT_EQ(a.size(), 1u);
        ASSERT_EQ(b.size(), 1u);
        EXPECT_EQ(std::memcmp(&a[0].position, &b[0].position, sizeof(Vec3f)), 0) << rotation << " " << x << " " << y;
      }
    }
  }
}


// ---- US-024: the engine derives is_ios from the manufacturer string it was configured with ----

class EventSink final : public TrackerCore {
 public:
  explicit EventSink(std::vector<std::vector<ImuEvent>>* out) : out_(out) {}
  TrackerStep Track(const FrameInput& in) override {
    out_->push_back(in.imu_events);
    TrackerStep s;
    s.status = TrackingStatus::kNormal;
    return s;
  }

 private:
  std::vector<std::vector<ImuEvent>>* out_;
};

TEST(IosPath, GlueRunnerDerivesIsIosFromTheManufacturer) {
  // upper(manufacturer) == "APPLE" (func 647 compares the upper-cased string with the C string "APPLE": exact match).
  struct Row { const char* manufacturer; bool ios; };
  const Row rows[] = {{"Apple", true}, {"APPLE", true}, {"apple", true}, {"aPpLe", true}, {"", false},
                      {"Google", false}, {"Apple Inc.", false}, {"Appl", false}, {"Samsung", false}};
  for (const Row& r : rows) {
    EngineConfig cfg;
    cfg.capture_width = 480;
    cfg.capture_height = 640;
    cfg.manufacturer = r.manufacturer;
    cfg.model = "K";
    GlueConfig g;
    g.frames_until_recenter = 0;
    std::vector<std::vector<ImuEvent>> sink;
    GlueRunner glue(cfg, g, [&sink](const EngineConfig&) -> std::unique_ptr<TrackerCore> { return std::make_unique<EventSink>(&sink); });
    DeviceMotionSample s;
    s.receive_time_ns = 1000;
    s.acceleration_including_gravity = Vec3f{1.f, 2.f, 3.f};
    s.interval_ms = 16.0;
    glue.PushDeviceMotion(s);
    uint8_t px = 0;
    glue.ProcessFrame(1, 1, GrayImageView{&px, 1, 1, 1});
    glue.ProcessFrame(2, 2, GrayImageView{&px, 1, 1, 1});
    ASSERT_EQ(sink.size(), 1u) << r.manufacturer;
    ASSERT_EQ(sink[0].size(), 1u) << r.manufacturer;
    const ImuEvent& e = sink[0][0];
    EXPECT_EQ(e.x, r.ios ? -1.f : 1.f) << r.manufacturer;
    EXPECT_EQ(e.y, r.ios ? -2.f : 2.f) << r.manufacturer;
    EXPECT_EQ(e.z, r.ios ? 3.f : -3.f) << r.manufacturer;
    EXPECT_EQ(e.interval_ns, r.ios ? 16000000000LL : 16666667LL) << r.manufacturer;
  }
}

TEST(DeviceOrientation, NegatedZeroSkewTermsKeepTheirSign) {
  // updateCameraIntrinsics negates with `-` (orientation.cc:50-51): a negated +0 skew term is -0 (0x80000000), the others stay +0.
  std::array<float, 16> m{};
  m[0] = 2.f; m[5] = 1.5f; m[10] = -1.f; m[11] = -1.f;  // m02 = m12 = +0
  struct Row { DeviceOrientation o; uint32_t m02, m12; };
  const Row rows[] = {{DeviceOrientation::kLandscapeLeft, 0x80000000u, 0u},
                      {DeviceOrientation::kLandscapeRight, 0u, 0x80000000u},
                      {DeviceOrientation::kPortraitUpsideDown, 0x80000000u, 0x80000000u},
                      {DeviceOrientation::kPortrait, 0u, 0u}};
  for (const Row& r : rows) {
    const std::array<float, 16> out = PortraitToDeviceProjection(r.o, m);
    uint32_t b8, b9;
    std::memcpy(&b8, &out[8], 4);
    std::memcpy(&b9, &out[9], 4);
    EXPECT_EQ(b8, r.m02) << static_cast<int>(r.o);
    EXPECT_EQ(b9, r.m12) << static_cast<int>(r.o);
  }
}

// ---- The real Engine ----

std::vector<uint8_t> NoiseImage(int w, int h, uint32_t seed) {
  std::vector<uint8_t> img(static_cast<size_t>(w) * static_cast<size_t>(h));
  uint32_t s = seed;
  for (size_t i = 0; i < img.size(); ++i) {
    s = s * 1664525u + 1013904223u;
    img[i] = static_cast<uint8_t>(64 + ((s >> 16) & 0x7F));
  }
  return img;
}

TEST(EngineOrientation, LandscapeAndRotatedCapturesConstructAndTrackWithThePortraitK) {
  struct Case { int w, h, rotation; };
  const Case cases[] = {{480, 640, 0}, {480, 640, 180}, {640, 480, 90}, {640, 480, 270}, {640, 480, -90}, {640, 480, 0}};
  PinholeIntrinsics reference;
  bool have_reference = false;
  for (const Case& c : cases) {
    SCOPED_TRACE(::testing::Message() << c.w << "x" << c.h << " rotation " << c.rotation);
    EngineConfig cfg;
    cfg.capture_width = c.w;
    cfg.capture_height = c.h;
    cfg.rotation_deg = c.rotation;
    cfg.manufacturer = "";
    cfg.model = "K";
    Engine engine(cfg);
    const std::vector<uint8_t> img = NoiseImage(c.w, c.h, 7);
    for (int k = 0; k < 2; ++k) {
      FrameInput in;
      in.time_ns = 1000000000LL + k * 50000000LL;
      in.video_time_ns = in.time_ns;
      in.gray = GrayImageView{img.data(), c.w, c.h, c.w};
      in.device_pose = QuatWxyz{0.70710677f, 0.70710677f, 0.f, 0.f};
      const TrackerStep step = engine.Track(in);
      for (float v : step.pose.m) EXPECT_TRUE(std::isfinite(v));
      if (!have_reference) {
        reference = step.intrinsics;
        have_reference = true;
      }
      // The tracker runs in the portrait frame: the K rule exchanges a landscape capture, so the
      // processing K is the same for every geometry.
      EXPECT_EQ(step.intrinsics.width, reference.width);
      EXPECT_EQ(step.intrinsics.height, reference.height);
      EXPECT_EQ(std::memcmp(&step.intrinsics.fx, &reference.fx, sizeof(float)), 0);
      EXPECT_EQ(std::memcmp(&step.intrinsics.cx, &reference.cx, sizeof(float)), 0);
      EXPECT_EQ(std::memcmp(&step.intrinsics.cy, &reference.cy, sizeof(float)), 0);
    }
  }
}

TEST(EngineOrientationDeathTest, ImageSizeMustMatchTheCaptureSize) {
  EngineConfig cfg;
  cfg.capture_width = 640;
  cfg.capture_height = 480;
  cfg.rotation_deg = 90;
  cfg.model = "K";
  Engine engine(cfg);
  const std::vector<uint8_t> portrait_image = NoiseImage(480, 640, 1);  // the portrait frame for a landscape engine
  FrameInput in;
  in.time_ns = 1000;
  in.gray = GrayImageView{portrait_image.data(), 480, 640, 480};
  EXPECT_DEATH(engine.Track(in), "");
}

}  // namespace
}  // namespace slam_refact
