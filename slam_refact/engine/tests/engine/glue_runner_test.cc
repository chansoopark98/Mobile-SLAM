// GlueRunner unit tests with a fixed-output TrackerCore (architecture.md §4.3-§4.4, §7.3
// "engine/glue"): 1-frame lag, startup recenter at the 5th processed frame (P5, frame 5
// re-published, rebuild semantics: new core + pipeline.clear()), Recenter() keeping
// framesUntilRecenter, 733/704 composition, world point exposure and 0-padding (Q10), absolute
// origin.y = 1 (N4), pipeline pixel ownership.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include "slam_refact/glue_runner.h"

namespace slam_refact {
namespace {

struct FakeLog {
  int cores_created = 0;
  std::vector<int64_t> tracked_times;   // time_ns of every Track call (all cores)
  std::vector<int> core_of_call;        // core number (1-based) per Track call
  std::vector<uint8_t> first_pixels;    // gray[0] seen by every Track call
  std::vector<std::vector<uint8_t>> grays;  // width x height gray bytes (row by row) per Track call
  std::vector<int> gray_strides;            // gray stride seen by every Track call
};

// Pose of call n of one core (1-based): translation (0.1, 0.2, 0.3) * n, identity rotation.
class FakeCore final : public TrackerCore {
 public:
  FakeCore(FakeLog* log, TrackingReason reason) : log_(log), reason_(reason) {
    id_ = ++log_->cores_created;
  }
  TrackerStep Track(const FrameInput& in) override {
    ++calls_;
    log_->tracked_times.push_back(in.time_ns);
    log_->core_of_call.push_back(id_);
    log_->first_pixels.push_back(in.gray.data != nullptr ? in.gray.data[0] : 0);
    std::vector<uint8_t> gray;
    for (int y = 0; in.gray.data != nullptr && y < in.gray.height; ++y) {
      for (int x = 0; x < in.gray.width; ++x) {
        gray.push_back(in.gray.data[static_cast<size_t>(y) * in.gray.stride + x]);
      }
    }
    log_->grays.push_back(gray);
    log_->gray_strides.push_back(in.gray.stride);
    TrackerStep s;
    for (int k = 0; k < 16; ++k) s.pose.m[k] = (k % 5 == 0) ? 1.f : 0.f;
    s.pose.m[12] = 0.1f * static_cast<float>(calls_);
    s.pose.m[13] = 0.2f * static_cast<float>(calls_);
    s.pose.m[14] = 0.3f * static_cast<float>(calls_);
    s.tam_extrinsic = s.pose;
    s.predicted_pose = s.pose;
    s.status = TrackingStatus::kNormal;
    s.reason = reason_;
    // One real point and one 0-padding entry (631 Q10).
    s.world_points_internal.push_back(WorldPointOut{7, Vec3f{1.f, 1.f, 1.f}, 0.5f});
    s.world_points_internal.push_back(WorldPointOut{0, Vec3f{0.f, 0.f, 0.f}, 0.f});
    return s;
  }

 private:
  FakeLog* log_;
  TrackingReason reason_;
  int id_ = 0;
  int calls_ = 0;
};

EngineConfig TumConfig(bool estimate_scale = false) {
  EngineConfig c;
  c.estimate_scale = estimate_scale;
  c.capture_width = 240;
  c.capture_height = 320;
  c.manufacturer = "Google";
  c.model = "Pixel 7";
  return c;
}

class GlueRunnerFakeTest : public ::testing::Test {
 protected:
  std::unique_ptr<GlueRunner> MakeRunner(bool estimate_scale = false,
                                         TrackingReason reason = TrackingReason::kUnspecified) {
    FakeLog* log = &log_;
    return std::make_unique<GlueRunner>(
        TumConfig(estimate_scale), GlueConfig{},
        [log, reason](const EngineConfig&) -> std::unique_ptr<TrackerCore> {
          return std::make_unique<FakeCore>(log, reason);
        });
  }

  // Stage k: time_ns = 1000 + k, gray pixel value k.
  PublishedFrame Stage(GlueRunner* g, int k) {
    pixel_ = static_cast<uint8_t>(k);
    FrameInput in;
    in.time_ns = 1000 + k;
    in.video_time_ns = 1000 + k;
    in.gray = GrayImageView{&pixel_, 1, 1, 1};
    return g->Publish(in);
  }

  FakeLog log_;
  uint8_t pixel_ = 0;
};

TEST_F(GlueRunnerFakeTest, StartupRecenterLagAndRepublish) {
  auto g = MakeRunner();
  EXPECT_EQ(log_.cores_created, 1);
  std::vector<PublishedFrame> out;
  for (int k = 0; k < 10; ++k) out.push_back(Stage(g.get(), k));

  // Stage 0 = bootstrap: pipeline size 1 -> nothing processed.
  EXPECT_TRUE(out[0].repeated);
  EXPECT_FALSE(out[0].step.has_value());
  // Stages 1-5 process stages 0-4 (1-frame lag) and publish at the origin (P5).
  for (int k = 1; k <= 5; ++k) {
    EXPECT_FALSE(out[k].repeated) << k;
    EXPECT_EQ(out[k].source_frame_index, k - 1) << k;
    EXPECT_EQ(log_.tracked_times[static_cast<size_t>(k - 1)], 1000 + k - 1) << k;
    EXPECT_EQ(out[k].position.x, 0.f);
    EXPECT_EQ(out[k].position.y, 2.f);
    EXPECT_EQ(out[k].position.z, 0.f);
  }
  // The 5th processed frame rebuilt the core and cleared the pipeline (stage 5 dropped).
  EXPECT_EQ(log_.cores_created, 2);
  // Stage 6: pipeline holds only stage 6 -> stage-5 response re-published.
  EXPECT_TRUE(out[6].repeated);
  EXPECT_EQ(out[6].position.y, out[5].position.y);
  EXPECT_EQ(out[6].rotation.w, out[5].rotation.w);
  EXPECT_EQ(out[6].source_frame_index, out[5].source_frame_index);
  // Stage 7 processes stage 6 on the new core; stage 5 was never tracked.
  ASSERT_EQ(log_.tracked_times.size(), 8u);
  EXPECT_EQ(log_.tracked_times[5], 1006);
  EXPECT_EQ(log_.core_of_call[5], 2);
  EXPECT_EQ(out[7].source_frame_index, 6);
  // First frame of a core: recenterPose initial transform = itself -> origin (0, 2, 0).
  EXPECT_FLOAT_EQ(out[7].position.y, 2.f);
  // Second frame of the new core: delta (0.1, 0.2, 0.3) -> RH (z flip) -> scale 2 + origin.
  EXPECT_NEAR(out[8].position.x, 0.2f, 1e-6f);
  EXPECT_NEAR(out[8].position.y, 2.4f, 1e-6f);
  EXPECT_NEAR(out[8].position.z, -0.6f, 1e-6f);
  // Identity rotation after makeRightHanded (w, z negated) is still the identity rotation.
  EXPECT_FLOAT_EQ(std::fabs(out[8].rotation.w), 1.f);
  EXPECT_EQ(g->last_published().source_frame_index, out[9].source_frame_index);
}

TEST_F(GlueRunnerFakeTest, RecenterClearsPipelineButKeepsStartupCounter) {
  auto g = MakeRunner();
  for (int k = 0; k < 10; ++k) Stage(g.get(), k);
  EXPECT_EQ(log_.cores_created, 2);
  g->Recenter();
  EXPECT_EQ(log_.cores_created, 3);
  const PublishedFrame a = Stage(g.get(), 10);
  EXPECT_TRUE(a.repeated);  // pipeline cleared
  const PublishedFrame b = Stage(g.get(), 11);
  const PublishedFrame c = Stage(g.get(), 12);
  EXPECT_EQ(b.source_frame_index, 10);
  EXPECT_EQ(log_.core_of_call.back(), 3);
  // framesUntilRecenter stays 0: no origin overwrite and no further rebuild.
  EXPECT_NEAR(c.position.x, 0.2f, 1e-6f);
  EXPECT_EQ(log_.cores_created, 3);
  g->Reset();
  EXPECT_EQ(log_.cores_created, 4);
}

TEST_F(GlueRunnerFakeTest, RecenterDuringStartupDoesNotResetCounter) {
  auto g = MakeRunner();
  Stage(g.get(), 0);
  Stage(g.get(), 1);  // processed 1
  g->Recenter();      // core 2
  for (int k = 2; k < 8; ++k) Stage(g.get(), k);
  // After the recenter stage 2 is alone (repeat); 3 -> 2 (2nd processed), 4 -> 3 (3rd),
  // 5 -> 4 (4th), 6 -> 5 (5th: startup rebuild, core 3), 7 -> repeat.
  EXPECT_EQ(log_.cores_created, 3);
}

TEST_F(GlueRunnerFakeTest, WorldPointsExposedAndZeroPaddingMapsToOrigin) {
  auto g = MakeRunner();
  PublishedFrame last;
  for (int k = 0; k < 9; ++k) last = Stage(g.get(), k);
  ASSERT_EQ(last.world_points.size(), 2u);
  EXPECT_EQ(last.world_points[0].id, 7u);
  EXPECT_FLOAT_EQ(last.world_points[0].position.x, 2.f);
  EXPECT_FLOAT_EQ(last.world_points[0].position.y, 4.f);
  EXPECT_FLOAT_EQ(last.world_points[0].position.z, -2.f);
  EXPECT_FLOAT_EQ(last.world_points[0].confidence, 0.5f);
  // Q10: the 0-padding entry (0, 0, 0) is published as (0, 2, 0).
  EXPECT_EQ(last.world_points[1].id, 0u);
  EXPECT_FLOAT_EQ(last.world_points[1].position.x, 0.f);
  EXPECT_FLOAT_EQ(last.world_points[1].position.y, 2.f);
  EXPECT_FLOAT_EQ(last.world_points[1].position.z, 0.f);
}

TEST_F(GlueRunnerFakeTest, InitializingHidesWorldPoints) {
  auto g = MakeRunner(false, TrackingReason::kInitializing);
  PublishedFrame last;
  for (int k = 0; k < 9; ++k) last = Stage(g.get(), k);
  EXPECT_EQ(last.reason, TrackingReason::kInitializing);
  EXPECT_TRUE(last.world_points.empty());
}

TEST_F(GlueRunnerFakeTest, AbsoluteScaleForcesOriginYOneAndScaleOne) {
  auto g = MakeRunner(true);
  std::vector<PublishedFrame> out;
  for (int k = 0; k < 9; ++k) out.push_back(Stage(g.get(), k));
  for (int k = 1; k <= 5; ++k) {
    EXPECT_EQ(out[k].position.y, 1.f) << k;  // startup overwrite uses origin.y = 1
  }
  EXPECT_NEAR(out[8].position.x, 0.1f, 1e-6f);   // scale 1
  EXPECT_NEAR(out[8].position.y, 1.2f, 1e-6f);
  EXPECT_NEAR(out[8].position.z, -0.3f, 1e-6f);
  ASSERT_EQ(out[8].world_points.size(), 2u);
  EXPECT_FLOAT_EQ(out[8].world_points[1].position.y, 1.f);
}

TEST_F(GlueRunnerFakeTest, PipelineOwnsPixels) {
  auto g = MakeRunner();
  // The caller reuses one pixel buffer; each staged frame must keep its own copy until tracked.
  for (int k = 0; k < 4; ++k) Stage(g.get(), 40 + k);
  ASSERT_EQ(log_.first_pixels.size(), 3u);
  EXPECT_EQ(log_.first_pixels[0], 40);
  EXPECT_EQ(log_.first_pixels[1], 41);
  EXPECT_EQ(log_.first_pixels[2], 42);
}

// S2 review MEDIUM-2: OwnPixels validates the strided views before copying rows.
using GlueRunnerFakeDeathTest = GlueRunnerFakeTest;

TEST_F(GlueRunnerFakeDeathTest, GrayStrideSmallerThanWidthAborts) {
  std::vector<uint8_t> px(64, 1);
  FrameInput in;
  in.gray = GrayImageView{px.data(), 8, 4, 7};
  EXPECT_DEATH(MakeRunner()->Publish(in), "gray view");
}

TEST_F(GlueRunnerFakeDeathTest, RgbaStrideSmallerThanFourTimesWidthAborts) {
  std::vector<uint8_t> px(256, 1);
  FrameInput in;
  in.rgba = RgbaImageView{px.data(), 8, 4, 31};
  EXPECT_DEATH(MakeRunner()->Publish(in), "rgba view");
}

TEST_F(GlueRunnerFakeDeathTest, NonPositiveGeometryAborts) {
  std::vector<uint8_t> px(64, 1);
  FrameInput gray;
  gray.gray = GrayImageView{px.data(), 8, 0, 8};
  EXPECT_DEATH(MakeRunner()->Publish(gray), "gray view");
  FrameInput rgba;
  rgba.rgba = RgbaImageView{px.data(), -1, 4, 32};
  EXPECT_DEATH(MakeRunner()->Publish(rgba), "rgba view");
  FrameInput atlas;
  atlas.pyramid = PyramidAtlasView{px.data(), 0, 4};
  EXPECT_DEATH(MakeRunner()->Publish(atlas), "injected pyramid size and rows");
}

TEST_F(GlueRunnerFakeTest, PaddedStrideIsCopiedTight) {
  auto g = MakeRunner();
  // 2 x 2 gray with stride 5: row 0 = {9, 8}, row 1 = {7, 6}; padding bytes never reach the core.
  std::vector<uint8_t> px = {9, 8, 0, 0, 0, 7, 6};
  FrameInput in;
  in.gray = GrayImageView{px.data(), 2, 2, 5};
  g->Publish(in);
  g->Publish(in);
  ASSERT_EQ(log_.grays.size(), 1u);
  EXPECT_EQ(log_.grays[0], (std::vector<uint8_t>{9, 8, 7, 6}));
  EXPECT_EQ(log_.gray_strides[0], 2);
}

}  // namespace
}  // namespace slam_refact
