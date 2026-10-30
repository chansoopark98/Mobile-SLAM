// Engine (real modules) unit tests on checked-in data only (architecture.md §7.3 engine/glue,
// §7.4 IT-3, D-14, R16, M12):
//   * D-14 double-apply guard: compile-time (no ImuEvent overload of func 647) and run-time
//     (Engine::Track aborts on post-647 kind 2 and on time-reversed events, via
//     ValidatePostPrepareEvents);
//   * geometry: an unsupported rotation or a missing capture size aborts (SR_CHECK; R16 was lifted by FULL-8);
//   * determinism and RNG reseed: two engines (= a rebuilt engine) give bit-identical steps;
//   * partial reset (476): Configure(disable_vio = true) recreates MapTracker/MapBuilder and the
//     output becomes 3DoF (translation exactly 0); fixed fields abort on change;
//   * US-010 API (IT-3): GlueRunner pushImu / processFrame / getters / reset on TUM cap-001..003
//     injected pyramids with synthetic IMU: finite poses, statuses, P5, frame-5 re-publication.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <type_traits>
#include <vector>

#include "engine/device_pose.h"
#include "frontend/frontend.h"
#include "slam_refact/engine.h"
#include "slam_refact/glue_runner.h"
#include "test_util.h"

namespace slam_refact {
namespace {

constexpr int kCaptureRows = 660;
constexpr int64_t kT0 = 1000000000LL;      // harness T0 = 1 s (0 is a sentinel)
constexpr int64_t kFrameNs = 50000000LL;   // TUM 20 Hz

// D-14 compile-time half: func 647 only accepts capnp-numbered RawSensorEvent.
static_assert(!std::is_invocable_v<decltype(&frontend::PrepareTrackingSensorEvents),
                                   std::vector<ImuEvent>, bool>);
static_assert(std::is_invocable_v<decltype(&frontend::PrepareTrackingSensorEvents),
                                  std::vector<RawSensorEvent>, bool>);

EngineConfig TumConfig() {
  EngineConfig c;
  c.capture_width = 240;
  c.capture_height = 320;
  c.manufacturer = "Google";
  c.model = "Pixel 7";
  c.pyramid_model = ShaderNumericModel::kSwiftShaderApprox;
  return c;
}

// TUM c2 captures 1..3 (GPU readbacks, 1024 x 660 RGBA).
const std::vector<frontend::RgbaAtlas>& Atlases() {
  static const std::vector<frontend::RgbaAtlas>* atlases = [] {
    auto* v = new std::vector<frontend::RgbaAtlas>(3);
    for (int k = 1; k <= 3; ++k) {
      char name[80];
      std::snprintf(name, sizeof(name), "/frontend/capture/tum-room1-full/c2/cap-%03d.rgba.gz", k);
      const bool ok = frontend::testing::LoadCaptureAtlas(
          frontend::testing::TestDataDir() + name, kCaptureRows, &(*v)[static_cast<size_t>(k - 1)]);
      if (!ok) std::abort();
    }
    return v;
  }();
  return *atlases;
}

PyramidAtlasView AtlasView(int frame) {
  const frontend::RgbaAtlas& a = Atlases()[static_cast<size_t>(frame % 3)];
  return PyramidAtlasView{a.rgba.data(), a.size, a.rows};
}

// Post-647 events of one frame: still device, gravity along +y (portrait upright).
std::vector<ImuEvent> StillEvents(int64_t t_end) {
  std::vector<ImuEvent> ev;
  for (int k = 2; k >= 0; --k) {
    const int64_t t = t_end - k * 16666667LL;
    ev.push_back(ImuEvent{ImuEvent::Kind::kAccelerometer, t, 16666667, 0.f, 9.81f, 0.f});
    ev.push_back(ImuEvent{ImuEvent::Kind::kGyroscope, t, 16666667, 0.f, 0.f, 0.f});
  }
  return ev;
}

FrameInput EngineFrame(int k) {
  FrameInput in;
  in.time_ns = kT0 + k * kFrameNs;
  in.video_time_ns = in.time_ns;
  in.pyramid = AtlasView(k);
  in.device_pose = QuatWxyz{0.70710677f, 0.70710677f, 0.f, 0.f};  // getQuaternion(0, 90, 0)
  in.imu_events = StillEvents(in.time_ns);
  return in;
}

bool Finite(const HMatrix4f& m) {
  for (float v : m.m) {
    if (!std::isfinite(v)) return false;
  }
  return true;
}

TEST(EngineDeathTest, PostPrepareKindTwoAborts) {
  Engine e(TumConfig());
  FrameInput in = EngineFrame(0);
  in.imu_events.push_back(ImuEvent{static_cast<ImuEvent::Kind>(2), in.time_ns + 1, 16666667, 0.f,
                                   0.f, 0.f});  // a raw capnp gyroscope (2) that skipped 647
  EXPECT_DEATH(e.Track(in), "MAGNETOMETER");
}

TEST(EngineDeathTest, TimeReversedEventsAbort) {
  Engine e(TumConfig());
  FrameInput in = EngineFrame(0);
  in.imu_events.push_back(
      ImuEvent{ImuEvent::Kind::kAccelerometer, in.time_ns - 1000000000LL, 16666667, 0.f, 9.81f, 0.f});
  EXPECT_DEATH(e.Track(in), "not sorted");
}

// R16 was lifted by FULL-8 (US-023: landscape captures and rotation 90 / 180 / 270 are supported, see
// tests/engine/orientation_test.cc); what still aborts is a rotation the original maps to no device
// orientation (UNSPECIFIED) and a missing capture size.
TEST(EngineDeathTest, UnsupportedGeometryAborts) {
  EngineConfig c = TumConfig();
  c.capture_width = 0;
  EXPECT_DEATH(Engine e(c), "capture size missing");
  EngineConfig r = TumConfig();
  r.rotation_deg = 45;
  EXPECT_DEATH(Engine e(r), "rotation_deg must be");
}

TEST(EngineDeathTest, ConfigureRejectsFixedFields) {
  Engine e(TumConfig());
  EngineConfig c = TumConfig();
  c.estimate_scale = true;
  EXPECT_DEATH(e.Configure(c), "estimate_scale");
}

// S2 review LOW-1: the intrinsics are fixed at construction, so Configure must not accept a
// different override silently; re-sending the same configuration stays accepted.
TEST(EngineDeathTest, ConfigureRejectsIntrinsicsOverrideChange) {
  EngineConfig with = TumConfig();
  with.intrinsics_override = PinholeIntrinsics{240, 320, 119.5f, 159.5f, 260.f, 260.f};
  Engine a(TumConfig());
  EXPECT_DEATH(a.Configure(with), "intrinsics_override is fixed");
  Engine b(with);
  EngineConfig other = with;
  other.intrinsics_override->fx = 261.f;
  EXPECT_DEATH(b.Configure(other), "intrinsics_override is fixed");
  EXPECT_DEATH(b.Configure(TumConfig()), "intrinsics_override is fixed");
}

TEST(EngineTest, ConfigureAcceptsUnchangedIntrinsicsOverride) {
  EngineConfig with = TumConfig();
  with.intrinsics_override = PinholeIntrinsics{240, 320, 119.5f, 159.5f, 260.f, 260.f};
  Engine e(with);
  e.Configure(with);
  EXPECT_TRUE(Finite(e.Track(EngineFrame(0)).pose));
}

// S2 review MEDIUM-3: a CPU-path frame must have the capture size (engineInit).
TEST(EngineDeathTest, FrameSizeDifferentFromCaptureAborts) {
  Engine e(TumConfig());  // 240 x 320
  std::vector<uint8_t> gray(320 * 240, 100);
  FrameInput in = EngineFrame(0);
  in.pyramid = PyramidAtlasView{};
  in.gray = GrayImageView{gray.data(), 320, 240, 320};
  EXPECT_DEATH(e.Track(in), "differs from the capture geometry");
}

// A rebuilt engine (rebuildEngine) starts from the same state, including RandomNumbers(5489).
TEST(EngineTest, FreshEnginesAreBitIdentical) {
  Engine a(TumConfig());
  Engine b(TumConfig());
  for (int k = 0; k < 6; ++k) {
    const TrackerStep sa = a.Track(EngineFrame(k));
    const TrackerStep sb = b.Track(EngineFrame(k));
    ASSERT_TRUE(Finite(sa.pose)) << k;
    EXPECT_EQ(std::memcmp(sa.pose.m, sb.pose.m, sizeof(sa.pose.m)), 0) << k;
    EXPECT_EQ(sa.status, sb.status);
    EXPECT_EQ(sa.num_keypoints, sb.num_keypoints);
    EXPECT_EQ(sa.world_points_internal.size(), sb.world_points_internal.size());
  }
}

TEST(EngineTest, TracksInjectedPyramidsWithKeypoints) {
  Engine e(TumConfig());
  const TrackerStep s0 = e.Track(EngineFrame(0));
  EXPECT_TRUE(s0.yaw_offset_initialized);
  EXPECT_GT(s0.num_keypoints, 0);
  EXPECT_EQ(s0.reason, TrackingReason::kUnspecified);  // responsive world SLAM (P4)
  for (int k = 1; k < 5; ++k) {
    const TrackerStep s = e.Track(EngineFrame(k));
    EXPECT_TRUE(Finite(s.pose)) << k;
    EXPECT_NE(s.status, TrackingStatus::kUnspecified) << k;
  }
}

TEST(EngineTest, DisableVioPartialResetGivesThreeDof) {
  Engine e(TumConfig());
  for (int k = 0; k < 3; ++k) e.Track(EngineFrame(k));
  EngineConfig c = TumConfig();
  c.disable_vio = true;
  e.Configure(c);
  for (int k = 3; k < 6; ++k) {
    const TrackerStep s = e.Track(EngineFrame(k));
    // 3DoF: tamExtrinsic = cameraMotion(0, xr rotation); output translation exactly 0 (V-3DOF).
    EXPECT_EQ(s.pose.m[12], 0.f) << k;
    EXPECT_EQ(s.pose.m[13], 0.f) << k;
    EXPECT_EQ(s.pose.m[14], 0.f) << k;
    EXPECT_EQ(s.map_tracker_state, 0) << k;  // recreated MapTracker never ran
    EXPECT_TRUE(Finite(s.pose)) << k;
  }
  c.disable_vio = false;
  e.Configure(c);  // turning it off: no recreation, tracking resumes
  EXPECT_TRUE(Finite(e.Track(EngineFrame(6)).pose));
}

// IT-3 / US-010: configure (constructor), pushImu, processFrame, getPose/getStatus/getWorldPoints
// (last_published), reset.
class GlueApiTest : public ::testing::Test {
 protected:
  void PushStill(GlueRunner* g, int64_t t) {
    DeviceMotionSample m;
    m.receive_time_ns = t;
    m.acceleration_including_gravity = Vec3f{0.f, 9.81f, 0.f};
    m.rotation_rate_abg = Vec3f{0.f, 0.f, 0.f};
    m.interval_ms = 16.666;
    g->PushDeviceMotion(m);
    g->PushDeviceOrientation(DeviceOrientationSample{t, 0.0, 90.0, 0.0});
  }

  PublishedFrame Frame(GlueRunner* g, int k) {
    const int64_t t = kT0 + k * kFrameNs;
    for (int j = 2; j >= 0; --j) PushStill(g, t - j * 16666667LL);
    return g->ProcessFrame(t, t, GrayImageView{gray_.data(), 240, 320, 240}, AtlasView(k));
  }

  std::vector<uint8_t> gray_ = std::vector<uint8_t>(240 * 320, 128);
};

TEST_F(GlueApiTest, ProcessFramePublishesFinitePosesWithStartupRecenter) {
  GlueRunner g(TumConfig(), GlueConfig{});
  std::vector<PublishedFrame> out;
  for (int k = 0; k < 12; ++k) out.push_back(Frame(&g, k));  // stage 0 = bootstrap
  EXPECT_TRUE(out[0].repeated);
  for (int i = 0; i <= 4; ++i) {  // published frames 0-4 = stages 1-5 (P5)
    const PublishedFrame& f = out[static_cast<size_t>(i + 1)];
    EXPECT_EQ(f.position.x, 0.f) << i;
    EXPECT_EQ(f.position.y, 2.f) << i;
    EXPECT_EQ(f.position.z, 0.f) << i;
  }
  EXPECT_TRUE(out[6].repeated);  // frame 5 = frame 4 re-published
  EXPECT_EQ(std::memcmp(&out[6].rotation, &out[5].rotation, sizeof(QuatWxyz)), 0);
  for (const PublishedFrame& f : out) {
    EXPECT_TRUE(std::isfinite(f.position.x) && std::isfinite(f.position.y) &&
                std::isfinite(f.position.z) && std::isfinite(f.rotation.w));
  }
  const PublishedFrame& last = g.last_published();
  EXPECT_EQ(last.source_frame_index, out.back().source_frame_index);
  EXPECT_NE(last.status, TrackingStatus::kUnspecified);
  EXPECT_EQ(last.reason, TrackingReason::kUnspecified);
  ASSERT_TRUE(last.step.has_value());
  EXPECT_EQ(last.step->world_points_internal.size(), last.world_points.size());
  // Frame 6 = first frame of the rebuilt engine: heading 0 (P6) -> rotation about y only from the
  // device tilt; the published camera stays at the origin (recenterPose of the first response).
  EXPECT_FLOAT_EQ(out[7].position.y, 2.f);

  g.Reset();
  const PublishedFrame after = Frame(&g, 12);
  EXPECT_TRUE(after.repeated);  // pipeline cleared by the rebuild
  EXPECT_FALSE(Frame(&g, 13).repeated);
}

// 647 devicePose = staged.times(Quaternion()) (precision lane, 2026-10-16): the staged f32
// quaternion is renormalized. Input = a staged device quaternion of the flip6-s2 capture run
// (w270-mp stageCalls, input frame 5); expected bits = the f647 wat evaluation
// n = sqrt(z^2 + (y^2 + (w^2 + x^2))), inv = 1 / n, q * inv, evaluated independently in f32.
// Without the renormalization the exact-input replays leave the original at the first frame
// whose staged quaternion is not already normalized (flip6-s2 trace frame 6, s3 frame 3).
TEST(EngineDevicePose, RenormalizedAs647) {
  const auto bits = [](float v) {
    uint32_t u;
    std::memcpy(&u, &v, 4);
    return u;
  };
  const auto from_bits = [](uint32_t u) {
    float v;
    std::memcpy(&v, &u, 4);
    return v;
  };
  const QuatWxyz staged{from_bits(0xbf5a9109u), from_bits(0xbf053181u), from_bits(0x3c65cfbau),
                        from_bits(0x3c5715dbu)};
  const c8::Quaternion q = TrackingDevicePose(staged);
  EXPECT_EQ(bits(q.w()), 0xbf5a910bu);
  EXPECT_EQ(bits(q.x()), 0xbf053182u);
  EXPECT_EQ(bits(q.y()), 0x3c65cfbcu);
  EXPECT_EQ(bits(q.z()), 0x3c5715ddu);
  // An exactly normalized quaternion is unchanged.
  const c8::Quaternion id = TrackingDevicePose(QuatWxyz{1.0f, 0.0f, 0.0f, 0.0f});
  EXPECT_EQ(bits(id.w()), bits(1.0f));
  EXPECT_EQ(bits(id.x()), bits(0.0f));
}

}  // namespace
}  // namespace slam_refact
