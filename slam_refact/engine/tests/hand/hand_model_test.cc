// Hand tests that need the public MediaPipe models (label mlmodel; never copied into the repository).
// Synthetic and overlay-contaminated inputs only: properties of the graph, not accuracy.
#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "hand/hand_messages.h"
#include "hand/hand_models.h"
#include "hand/hand_tracker.h"

namespace slam_refact::hand {
namespace {

std::vector<uint8_t> Load(const char* rel) {
  std::vector<uint8_t> b;
  std::string err;
  EXPECT_TRUE(ReadFileBytes(std::string(SR_REPO_ROOT) + "/" + rel, &b, &err)) << err;
  return b;
}

Image3 Gray(int w, int h, uint8_t v) {
  Image3 img(w, h);
  std::fill(img.px.begin(), img.px.end(), v);
  return img;
}

Image3 Noise(int w, int h, uint32_t seed) {
  Image3 img(w, h);
  uint32_t s = seed;
  for (auto& p : img.px) {
    s = s * 1664525u + 1013904223u;
    p = static_cast<uint8_t>(s >> 24);
  }
  return img;
}

TEST(HandModels, FilesAreThePinnedMediaPipeModels) {
  EXPECT_EQ(Sha256Hex(Load(kPalmDetectionPath)), kPalmDetectionSha256);
  EXPECT_EQ(Sha256Hex(Load(kHandLandmarkPath)), kHandLandmarkSha256);
  EXPECT_EQ(Sha256Hex(Load(kHandLandmark3dPath)), kHandLandmark3dSha256);
  HandTracker t;
  std::string err;
  auto wrong = Load(kPalmDetectionPath);
  wrong[wrong.size() / 2] ^= 1;
  EXPECT_FALSE(t.Load(wrong, Load(kHandLandmark3dPath), LandmarkModel::k3d, true, &err));
  EXPECT_NE(err.find("sha256"), std::string::npos);
  EXPECT_FALSE(t.loaded());
}

TEST(HandModels, BothLandmarkModelsLoadAndRun) {
  for (auto kind : {LandmarkModel::k2d, LandmarkModel::k3d}) {
    HandTracker t;
    std::string err;
    ASSERT_TRUE(t.Load(Load(kPalmDetectionPath),
                       Load(kind == LandmarkModel::k3d ? kHandLandmark3dPath : kHandLandmarkPath), kind, true, &err))
        << err;
    HandResult r;
    ASSERT_TRUE(t.Process(Gray(300, 564, 128), &r, &err)) << err;
    EXPECT_TRUE(r.palm_ran);
    EXPECT_EQ(r.landmarks.size(), 21u);
    // 2D model: no z; 3D model: z present (may be small for a gray image).
    if (kind == LandmarkModel::k2d) {
      for (const auto& l : r.landmarks) EXPECT_EQ(l.z, 0.0f);
    }
  }
}

TEST(HandTrackerGraph, PalmRunsOnlyWhenThePreviousFrameHadNoHandAndResetForgets) {
  HandTracker t;
  std::string err;
  ASSERT_TRUE(t.Load(Load(kPalmDetectionPath), Load(kHandLandmark3dPath), LandmarkModel::k3d, true, &err)) << err;
  const Image3 img = Noise(300, 564, 5);
  HandResult a, b;
  ASSERT_TRUE(t.Process(img, &a, &err)) << err;
  EXPECT_TRUE(a.palm_ran) << "first frame: nothing looped back, the gate allows detection";
  // The decision of the NEXT frame depends only on this frame's presence (handflag > 0.1).
  ASSERT_TRUE(t.Process(img, &b, &err)) << err;
  EXPECT_EQ(b.palm_ran, !a.present);
  t.Reset();
  HandResult c;
  ASSERT_TRUE(t.Process(img, &c, &err)) << err;
  EXPECT_TRUE(c.palm_ran);
  // Determinism: same image after Reset gives the same landmarks as the first frame.
  ASSERT_EQ(a.landmarks.size(), c.landmarks.size());
  for (size_t i = 0; i < a.landmarks.size(); ++i) {
    EXPECT_EQ(a.landmarks[i].x, c.landmarks[i].x);
    EXPECT_EQ(a.landmarks[i].y, c.landmarks[i].y);
  }
}

TEST(HandTrackerGraph, UniformImageHasNoPalmAndTheLandmarkModelRunsOnTheWholeImage) {
  HandTracker t;
  std::string err;
  ASSERT_TRUE(t.Load(Load(kPalmDetectionPath), Load(kHandLandmark3dPath), LandmarkModel::k3d, true, &err)) << err;
  HandResult r;
  ASSERT_TRUE(t.Process(Gray(300, 564, 90), &r, &err)) << err;
  EXPECT_TRUE(r.palm_detections.empty());
  // No palm: the zero rect makes ImageCropping take the whole frame (GetCropSpecs default).
  EXPECT_EQ(r.rect_from_palm.width, 0.0f);
  EXPECT_EQ(r.hand_rect.width, 0.0f);
  EXPECT_FALSE(r.present);
}

TEST(HandEvents, FoundUpdatedLostAndTheAttachmentNames) {
  HandEventTracker ev;
  HandResult off, on;
  on.present = true;
  on.landmarks.assign(21, Landmark{0.5f, 0.5f, 0.0f});
  for (int i = 0; i < 21; ++i) on.landmarks[i].x = 0.01f * i;
  EXPECT_FALSE(ev.Update(off).has_value());
  auto f = ev.Update(on);
  ASSERT_TRUE(f.has_value());
  EXPECT_EQ(f->status, TrackingStatus::kFound);
  ASSERT_EQ(f->attachment_points.size(), 22u);
  EXPECT_EQ(f->attachment_points[0].name, AttachmentName::kWrist);
  EXPECT_EQ(f->attachment_points[4].name, AttachmentName::kThumbTip);
  EXPECT_EQ(f->attachment_points[8].name, AttachmentName::kIndexTip);
  EXPECT_EQ(f->attachment_points[20].name, AttachmentName::kPinkyTip);
  EXPECT_EQ(f->attachment_points[21].name, AttachmentName::kPalm);
  EXPECT_NEAR(f->attachment_points[21].position.x, (0.0f + 0.05f + 0.09f + 0.13f + 0.17f) / 5, 1e-6);
  EXPECT_EQ(ev.Update(on)->status, TrackingStatus::kUpdated);
  EXPECT_EQ(ev.Update(off)->status, TrackingStatus::kLost);
  EXPECT_FALSE(ev.Update(off).has_value());
}

}  // namespace
}  // namespace slam_refact::hand
