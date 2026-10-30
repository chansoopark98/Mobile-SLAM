// mlmodel tests (ctest label of RM-1, formerly ml-evidence) of the face pipeline with the deployed
// models (US-018/US-019; docs/spec/08-face-ear.md §5 T4, §8). The models are read from assets/references (gitignored,
// never copied into the repository); a missing model fails, it is not skipped. Inputs are synthetic
// (no real face or ear images exist locally: the public test images are git-lfs pointers), so
// these tests check the pipeline wiring, determinism and postprocessing properties, not accuracy.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "c8/pixels/pixel-buffer.h"
#include "face/face_controller.h"
#include "face/face_events.h"
#include "face/face_models.h"
#include "reality/engine/ears/ear-detector-local.h"
#include "reality/engine/ears/ear-types.h"

namespace slam_refact::face {
namespace {

std::string RepoRoot() {
  const char* env = std::getenv("SLAM_REFACT_ML_REPO_ROOT");
  return env != nullptr && *env != '\0' ? std::string(env) : std::string(SR_REPO_ROOT);
}

std::vector<uint8_t> ReadModel(const char* rel) {
  std::vector<uint8_t> bytes;
  std::string error;
  EXPECT_TRUE(ReadFileBytes(RepoRoot() + "/" + rel, &bytes, &error)) << error;
  return bytes;
}

// Deterministic textured gray frame (no face): sin/cos pattern plus LCG noise.
std::vector<uint8_t> SyntheticFrame(int w, int h, uint32_t seed) {
  std::vector<uint8_t> px(static_cast<size_t>(4 * w * h));
  uint32_t s = seed;
  for (int r = 0; r < h; ++r) {
    for (int c = 0; c < w; ++c) {
      s = s * 1664525u + 1013904223u;
      const double v = 128.0 + 70.0 * std::sin(0.05 * c + 0.01 * seed) * std::cos(0.04 * r) +
                       20.0 * (static_cast<double>(s >> 24) / 255.0 - 0.5);
      const uint8_t g = static_cast<uint8_t>(std::fmin(255.0, std::fmax(0.0, v)));
      uint8_t* p = &px[static_cast<size_t>(4 * (r * w + c))];
      p[0] = g;
      p[1] = g;
      p[2] = g;
      p[3] = 255;
    }
  }
  return px;
}

class FaceModelsEvidence : public ::testing::Test {
 protected:
  void SetUp() override {
    detect_ = ReadModel(kFaceDetectModelPath);
    mesh_ = ReadModel(kFaceMeshModelPath);
    ear_ = ReadModel(kFaceEarModelPath);
    ASSERT_FALSE(detect_.empty() || mesh_.empty() || ear_.empty())
        << "model files missing under " << RepoRoot();
  }

  std::unique_ptr<FaceController> MakeController(bool ears) {
    auto c = std::make_unique<FaceController>();
    FaceOptions o = DefaultFaceOptions();
    o.enable_ears = ears;
    o.has_debug = true;
    o.debug.detections = true;
    c->UpdateFaceOptions(o);
    FaceCameraEnvironment env;
    env.camera_width = 480;
    env.camera_height = 640;
    env.display_width = 480;
    env.display_height = 640;
    c->UpdateCameraEnvironment(env);
    c->GetFrameworkData();
    std::string error;
    EXPECT_TRUE(c->LoadFaceDetectModel(detect_, &error)) << error;
    EXPECT_TRUE(c->LoadFaceMeshModel(mesh_, &error)) << error;
    if (ears) {
      EXPECT_TRUE(c->LoadEarModel(ear_, &error)) << error;
    }
    return c;
  }

  std::vector<uint8_t> detect_;
  std::vector<uint8_t> mesh_;
  std::vector<uint8_t> ear_;
};

TEST_F(FaceModelsEvidence, DeployedModelsHaveThePinnedSha256) {
  EXPECT_EQ(kFaceDetectModelSha256, Sha256Hex(detect_));
  EXPECT_EQ(kFaceMeshModelSha256, Sha256Hex(mesh_));
  EXPECT_EQ(kFaceEarModelSha256, Sha256Hex(ear_));
}

std::vector<uint8_t> UniformFrame(int w, int h, uint8_t value) {
  std::vector<uint8_t> px(static_cast<size_t>(4 * w * h), value);
  for (size_t i = 3; i < px.size(); i += 4) {
    px[i] = 255;
  }
  return px;
}

// Without a face (uniform gray): the first ProcessStagedFrame has no response (the original returns
// null before the pipeline holds two frames), later responses carry no face, no global detection
// and no ROI; the only events are faceloading/facescanning from attach.
TEST_F(FaceModelsEvidence, UniformInputGivesEmptyResponsesAndNoFaceEvents) {
  auto controller = MakeController(true);
  FaceEventEmitter emitter;
  const auto attach = emitter.OnAttach(controller->GetFrameworkData());
  ASSERT_EQ(2u, attach.size());
  int responses = 0;
  for (int i = 0; i < 25; ++i) {
    const auto frame = UniformFrame(480, 640, static_cast<uint8_t>(100 + i));
    ASSERT_TRUE(controller->StageFrame({frame.data(), 480, 640, 4 * 480})) << "frame " << i;
    const FaceResponse* r = controller->ProcessStagedFrame();
    if (i == 0) {
      EXPECT_EQ(nullptr, r);
      continue;
    }
    ASSERT_NE(nullptr, r);
    ++responses;
    EXPECT_TRUE(r->faces.empty()) << "frame " << i;
    EXPECT_TRUE(r->debug.detections.faces.empty()) << "frame " << i;
    EXPECT_TRUE(r->debug.detections.meshes.empty()) << "frame " << i;
    EXPECT_FLOAT_EQ(-1.0f, r->camera.extrinsic_rotation.w);  // (1,0,0,0) made right-handed
    EXPECT_TRUE(emitter.OnProcessCpu(r, 33.3 * i).empty()) << "frame " << i;
  }
  EXPECT_EQ(24, responses);
  EXPECT_TRUE(controller->tracker().faceToUprightBBoxInRay().empty());
  EXPECT_TRUE(controller->face_renderer()->Result().faceMeshImages.empty());
}

// The textured synthetic pattern (no face) triggers model detections (observed 2026-10-19: BlazeFace
// scores 0.86-0.96, one mesh faceflag logit > 0.5). Whatever the models report, the tracker
// schedule and the output structure must follow the original: the global detector runs only on
// tracker frames with frameCounter % 20 in {0, 1} (face-tracker.cc GLOBAL_DETECTOR_INTERVAL), and a
// found face carries 478 vertices/normals/uvs, 29 attachment points and the first-frame events.
TEST_F(FaceModelsEvidence, TexturedSyntheticInputFollowsTheTrackerSchedule) {
  auto controller = MakeController(true);
  FaceEventEmitter emitter;
  int found = 0;
  for (int i = 0; i < 45; ++i) {
    const auto frame = SyntheticFrame(480, 640, static_cast<uint32_t>(i));
    ASSERT_TRUE(controller->StageFrame({frame.data(), 480, 640, 4 * 480})) << "frame " << i;
    const FaceResponse* r = controller->ProcessStagedFrame();
    if (r == nullptr) {
      continue;
    }
    const int tracker_frame = i - 1;  // processing of staged frame i-1
    if (!r->debug.detections.faces.empty()) {
      EXPECT_TRUE(tracker_frame % 20 == 0 || tracker_frame % 20 == 1) << "frame " << i;
    }
    const auto events = emitter.OnProcessCpu(r, 50.0 * i);
    for (const auto& f : r->faces) {
      if (f.status != FaceTrackingStatus::kFound) {
        continue;
      }
      ++found;
      EXPECT_EQ(478u, f.vertices.size());
      EXPECT_EQ(478u, f.normals.size());
      EXPECT_EQ(478u, f.uvs_in_camera_frame.size());
      EXPECT_EQ(29u, f.attachment_points.size());
      // Ear data needs ear crops drawn from the previous frame's ear ROIs, so the first found
      // frame has none (face-tracker.cc trackEars); later frames have 3 + 3.
      EXPECT_TRUE(f.ear_vertices.empty() || f.ear_vertices.size() == 6u);
      ASSERT_FALSE(events.empty());
      EXPECT_EQ("facecontroller.facefound", events[0].name);
    }
  }
  std::printf("[ textured synthetic ] facefound responses: %d\n", found);
  // RM-1d (review RM-7 MINOR 7): without a found face every structure check above is vacuous. The
  // textured pattern makes the models report one face (observed 2026-10-19/20, a model false
  // positive on a faceless texture, not a tracking result). Known limit (review r2 ML closure,
  // C-13): this guarantee rests on that one model false positive, so a legitimate change of the
  // model numerics could make it fail. That is a signal to find another input that yields a face
  // (no real face image exists locally), not a reason to relax the structure checks above.
  EXPECT_GT(found, 0) << "no face was found: the structure checks above did not run";
}

bool BitEqual(const Position32f& a, const Position32f& b) {
  return std::memcmp(&a, &b, sizeof(Position32f)) == 0;
}

bool BitEqual(const Quaternion32f& a, const Quaternion32f& b) {
  return std::memcmp(&a, &b, sizeof(Quaternion32f)) == 0;
}

bool BitEqual(float a, float b) { return std::memcmp(&a, &b, sizeof(float)) == 0; }

// Number of positions of `a` that equal the position of `b` at the same index, bit for bit.
size_t CountBitIdentical(const std::vector<Position32f>& a, const std::vector<Position32f>& b) {
  size_t same = 0;
  for (size_t i = 0; i < a.size() && i < b.size(); ++i) {
    if (BitEqual(a[i], b[i])) {
      ++same;
    }
  }
  return same;
}

// The same frames through two controllers give bit-identical responses.
//
// C-1 (review r2 ML closure): the earlier version of this test fed six frames (seeds 100-105) that
// produce no FaceMsg at all, so its vertex comparison loop never ran (0 responses with a face,
// counted with gdb in the review). This version uses the frame sequence of the textured test above
// (seeds 0-44), on which the models report a false-positive face, so the comparison below does run.
// It shows that two controllers fed the same frames give bit-identical responses, including the
// vertices, normals, transform, attachment points and uvs of that face. It is a determinism check
// only: the face is not a real one (the input has none) and nothing here measures accuracy. The
// checks after the loop fail the test when no found face was compared, when the number of
// bit-identical vertices differs from the number compared, or when the one-ULP negative control
// (a copy with one vertex nudged must count as different) did not run or did not see the nudge.
// Mutation runs of these guards: evidence/ML-closure/c1-b5-mutation/ (m1-m5).
TEST_F(FaceModelsEvidence, PipelineIsDeterministic) {
  auto a = MakeController(true);
  auto b = MakeController(true);
  int responses = 0;
  int faces_compared = 0;
  int found_faces_compared = 0;
  size_t vertices_compared = 0;
  size_t bit_identical_vertices = 0;
  size_t normals_compared = 0;
  size_t bit_identical_normals = 0;
  bool control_done = false;
  for (int i = 0; i < 45; ++i) {
    const auto frame = SyntheticFrame(480, 640, static_cast<uint32_t>(i));
    ASSERT_TRUE(a->StageFrame({frame.data(), 480, 640, 4 * 480})) << "frame " << i;
    ASSERT_TRUE(b->StageFrame({frame.data(), 480, 640, 4 * 480})) << "frame " << i;
    const FaceResponse* ra = a->ProcessStagedFrame();
    const FaceResponse* rb = b->ProcessStagedFrame();
    ASSERT_EQ(ra == nullptr, rb == nullptr) << "frame " << i;
    if (ra == nullptr) {
      continue;
    }
    ++responses;
    for (int k = 0; k < 16; ++k) {
      EXPECT_EQ(ra->camera.intrinsic[k], rb->camera.intrinsic[k]);
    }
    ASSERT_EQ(ra->faces.size(), rb->faces.size()) << "frame " << i;
    for (size_t f = 0; f < ra->faces.size(); ++f) {
      const FaceMsg& fa = ra->faces[f];
      const FaceMsg& fb = rb->faces[f];
      ++faces_compared;
      if (fa.status == FaceTrackingStatus::kFound) {
        ++found_faces_compared;
      }
      EXPECT_EQ(fa.id, fb.id);
      EXPECT_EQ(fa.status, fb.status);
      EXPECT_TRUE(BitEqual(fa.transform.position, fb.transform.position));
      EXPECT_TRUE(BitEqual(fa.transform.rotation, fb.transform.rotation));
      EXPECT_TRUE(BitEqual(fa.transform.scale, fb.transform.scale));
      ASSERT_EQ(fa.vertices.size(), fb.vertices.size());
      const size_t identical_vertices = CountBitIdentical(fa.vertices, fb.vertices);
      EXPECT_EQ(fa.vertices.size(), identical_vertices) << "frame " << i << " face " << f;
      vertices_compared += fa.vertices.size();
      bit_identical_vertices += identical_vertices;
      ASSERT_EQ(fa.normals.size(), fb.normals.size());
      const size_t identical_normals = CountBitIdentical(fa.normals, fb.normals);
      EXPECT_EQ(fa.normals.size(), identical_normals) << "frame " << i << " face " << f;
      normals_compared += fa.normals.size();
      bit_identical_normals += identical_normals;
      // Negative control (once): the same comparison must see a one-ULP change of one vertex, so a
      // comparison that is disabled or always true cannot pass this test.
      if (!control_done && fa.vertices.size() > 7) {
        std::vector<Position32f> nudged = fb.vertices;
        nudged[7].x = std::nextafter(nudged[7].x, 1.0e9f);
        EXPECT_EQ(fa.vertices.size() - 1, CountBitIdentical(fa.vertices, nudged));
        control_done = true;
      }
      ASSERT_EQ(fa.uvs_in_camera_frame.size(), fb.uvs_in_camera_frame.size());
      for (size_t v = 0; v < fa.uvs_in_camera_frame.size(); ++v) {
        EXPECT_TRUE(BitEqual(fa.uvs_in_camera_frame[v].u, fb.uvs_in_camera_frame[v].u));
        EXPECT_TRUE(BitEqual(fa.uvs_in_camera_frame[v].v, fb.uvs_in_camera_frame[v].v));
      }
      ASSERT_EQ(fa.attachment_points.size(), fb.attachment_points.size());
      for (size_t v = 0; v < fa.attachment_points.size(); ++v) {
        EXPECT_EQ(fa.attachment_points[v].name, fb.attachment_points[v].name);
        EXPECT_TRUE(BitEqual(fa.attachment_points[v].position, fb.attachment_points[v].position));
        EXPECT_TRUE(BitEqual(fa.attachment_points[v].rotation, fb.attachment_points[v].rotation));
      }
    }
  }
  std::printf(
      "[ determinism ] responses: %d, faces compared: %d (found: %d), vertices bit-identical: "
      "%zu of %zu, normals: %zu of %zu\n",
      responses, faces_compared, found_faces_compared, bit_identical_vertices, vertices_compared,
      bit_identical_normals, normals_compared);
  // Without a compared face every check above is vacuous (the earlier 0-response form).
  EXPECT_GT(found_faces_compared, 0) << "no found face was compared: the check was vacuous";
  EXPECT_GT(vertices_compared, 0u);
  EXPECT_TRUE(control_done) << "the one-ULP negative control did not run";
  EXPECT_EQ(vertices_compared, bit_identical_vertices);
  EXPECT_EQ(normals_compared, bit_identical_normals);
  const auto pa = a->face_renderer()->pixels();
  const auto pb = b->face_renderer()->pixels();
  for (int r = 0; r < pa.rows(); ++r) {
    for (int c = 0; c < 4 * pa.cols(); ++c) {
      ASSERT_EQ(pa.pixels()[r * pa.rowBytes() + c], pb.pixels()[r * pb.rowBytes() + c]);
    }
  }
}

// §5 T4: ear-detector-local-test.cc with the right input = the mirror of the left input, so both
// batch entries of the model input are equal after toLetterboxRGBFloat0To1FlipX.
TEST_F(FaceModelsEvidence, EarDetectorMirrorSymmetry) {
  constexpr int w = EAR_LANDMARK_DETECTION_INPUT_WIDTH;
  constexpr int h = EAR_LANDMARK_DETECTION_INPUT_HEIGHT;
  for (uint32_t seed : {1u, 2u, 3u}) {
    c8::RGBA8888PlanePixelBuffer left(h, w);
    c8::RGBA8888PlanePixelBuffer right(h, w);
    const auto src = SyntheticFrame(w, h, seed * 17u);
    for (int r = 0; r < h; ++r) {
      for (int c = 0; c < w; ++c) {
        for (int ch = 0; ch < 4; ++ch) {
          const uint8_t v = src[static_cast<size_t>(4 * (r * w + c) + ch)];
          left.pixels().pixels()[r * left.pixels().rowBytes() + 4 * c + ch] = v;
          right.pixels().pixels()[r * right.pixels().rowBytes() + 4 * (w - 1 - c) + ch] = v;
        }
      }
    }
    c8::EarDetectorLocal detector(ear_);
    c8::RenderedSubImage l{{0, 0, static_cast<float>(w), static_cast<float>(h)}, left.pixels(),
                           MakeImageRoi(c8::ImageRoi::Source::EAR_LEFT, 0, c8::HMatrixGen::i())};
    c8::RenderedSubImage rr{{0, 0, static_cast<float>(w), static_cast<float>(h)}, right.pixels(),
                            MakeImageRoi(c8::ImageRoi::Source::EAR_RIGHT, 0, c8::HMatrixGen::i())};
    const auto ears = detector.analyzeEars(l, rr, DefaultDeviceIntrinsics());
    ASSERT_EQ(2u, ears.size());
    for (int i = 0; i < EAR_LANDMARK_DETECTION_NUM_PER_EAR; ++i) {
      EXPECT_NEAR(ears[0].points[i].x() + ears[1].points[i].x(), 1.0f, 0.00005);
      EXPECT_NEAR(ears[0].points[i].y(), ears[1].points[i].y(), 0.01f);
      EXPECT_NEAR(ears[0].points[i].z(), ears[1].points[i].z(), 1e-6);
      // Visibility = RELU(MINIMUM(REDUCE_MAX(heatmap), 1)) in the model graph.
      EXPECT_GE(ears[0].points[i].z(), 0.0f);
      EXPECT_LE(ears[0].points[i].z(), 1.0f);
    }
    EXPECT_EQ(0, ears[0].detectedClass);
    EXPECT_EQ(1, ears[1].detectedClass);
  }
}

}  // namespace
}  // namespace slam_refact::face
