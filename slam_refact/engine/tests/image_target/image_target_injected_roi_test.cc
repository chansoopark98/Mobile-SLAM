// RM-4b (contract FULL-7): an injected pyramid atlas whose ROI slots the producer drew.
//
// The rendered synthetic sequence of the US-013 fixture (render_synthetic.py) runs twice through a
// GlueRunner with the target loaded and disableVio on (the G-ROI-C setting):
//   CPU path       the engine draws the ROI slots itself (frontend::DrawRoisIntoAtlas)
//   injected path  a producer outside the runner draws every frame with the ROIs the runner asked
//                  for (GlueRunner::NextDrawRois() converted back to c8::ImageRoi, as the web chunk
//                  gets them from getRequestedRois()) and hands the whole atlas in as FrameInput::
//                  pyramid
// The engine must use the injected ROI slots exactly as drawn: every published frame (pose, status,
// located images incl. the ROI-feature flag and inlier counts) is bit-identical to the CPU path.
// Mutation (review checklist): the same producer with the ROI slots cleared must change the result
// in at least one frame, which shows that the engine really reads the injected slots.
// Synthetic data: not a device accuracy (CLAUDE.md section 3).

#include <gtest/gtest.h>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "frontend/frontend.h"
#include "frontend/pyramid_source.h"
#include "frontend/roi_draw.h"
#include "image_target/image_target_spec.h"
#include "image_target/target_files.h"
#include "image_target/target_loader.h"
#include "reality/engine/imagedetection/image-tracking-state.h"
#include "slam_refact/glue_runner.h"
#include "../common/sha256_hex.h"

namespace slam_refact {
namespace {

constexpr char kName[] = "synth-flat";

struct Frame {
  int index = 0;
  bool visible = false;
  int64_t time_ns = 0;
};

struct Synth {
  std::string dir;
  std::vector<Frame> frames;
};

bool LoadSynth(Synth* s) {
  const char* d = std::getenv("SLAM_REFACT_IT_SYNTH_DIR");
  s->dir = d == nullptr ? std::string() : std::string(d);
  std::ifstream f(s->dir + "/gt.tsv");
  if (!f) return false;
  std::string line;
  while (std::getline(f, line)) {
    std::istringstream is(line);
    std::string head;
    is >> head;
    if (head == "camera_k" || head == "target_k") continue;
    Frame g;
    g.index = std::stoi(head);
    int vis = 0;
    is >> vis >> g.time_ns;
    g.visible = vis != 0;
    s->frames.push_back(g);
  }
  return !s->frames.empty();
}

// RM-3c (API-2 review MINOR-8): the fixture sr_image_target_synth_render re-renders the sequence
// with the tool's default trajectory before every ctest run, so a direct run of this binary uses
// whatever the last fixture left (the roi_feat_frames count was 109 on the first-run "narrow"
// trajectory and 107 on the registered one). The log therefore names the data it ran on.
std::string ReadAll(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

std::string DataIdentity(const std::string& dir) {
  const std::string gt_tsv = ReadAll(dir + "/gt.tsv");
  const std::string gt_json = ReadAll(dir + "/gt.json");
  // render_synthetic.py writes "trajectory" only for a non-narrow trajectory (the narrow gt.json
  // stays byte-identical to the first run).
  const size_t at = gt_json.find("\"trajectory\": \"");
  std::string trajectory = "narrow";
  if (at != std::string::npos) {
    const size_t b = at + std::strlen("\"trajectory\": \"");
    trajectory = gt_json.substr(b, gt_json.find('"', b) - b);
  }
  return "dir=" + dir + " trajectory=" + trajectory + " gt.tsv_sha256=" +
         testutil::Sha256Hex(gt_tsv) + " gt.json_sha256=" + testutil::Sha256Hex(gt_json);
}

enum class Mode { kCpu, kInjected, kInjectedSlotsCleared };

c8::ImageRoi ToImageRoi(const PyramidRoiRequest& r) {
  c8::ImageRoi roi;
  roi.source = static_cast<c8::ImageRoi::Source>(static_cast<int>(r.source));
  roi.faceId = r.face_id;
  roi.name = r.name;
  roi.warp = c8::HMatrix(r.warp, nullptr);
  return roi;
}

struct SeqResult {
  std::vector<PublishedFrame> published;
  size_t roi_frames = 0;  // frames whose ROI list was not empty
};

SeqResult RunSequence(const Synth& s, Mode mode) {
  EngineConfig e;
  e.capture_width = 480;
  e.capture_height = 640;
  e.disable_vio = true;
  GlueRunner glue(e, GlueConfig{});
  image_target::ImageTargetSpec spec;
  image_target::GrayImage lum;
  std::string error;
  const std::string json_path = s.dir + "/" + kName + ".json";
  EXPECT_TRUE(image_target::LoadImageTargetFiles(json_path, s.dir + "/" + kName + "_luminance.pgm",
                                                 &spec, &lum, &error))
      << error;
  std::string json;
  EXPECT_TRUE(image_target::ReadFileBytes(json_path, &json, &error)) << error;
  EXPECT_EQ(glue.SetImageTargets({{json, lum.view()}}), "");
  frontend::PyramidSource producer(480, 640, frontend::ShaderParams{}, e.pyramid_model);
  SeqResult run;
  for (const Frame& f : s.frames) {
    char name[32];
    std::snprintf(name, sizeof(name), "/frames/%04d.pgm", f.index);
    std::string bytes;
    image_target::GrayImage img;
    EXPECT_TRUE(image_target::ReadFileBytes(s.dir + name, &bytes, &error)) << error;
    EXPECT_TRUE(image_target::ParsePgm(bytes, &img, &error)) << error;
    FrameInput in;
    in.time_ns = f.time_ns;
    in.video_time_ns = f.time_ns;
    const std::vector<PyramidRoiRequest> requests = glue.NextDrawRois();
    if (!requests.empty()) ++run.roi_frames;
    if (mode == Mode::kCpu) {
      in.gray = img.view();
    } else {
      std::vector<c8::ImageRoi> rois;
      for (const PyramidRoiRequest& r : requests) rois.push_back(ToImageRoi(r));
      producer.GetWithRois(PyramidAtlasView{}, RgbaImageView{}, img.view(), rois);
      frontend::RgbaAtlas atlas = producer.last_cpu_atlas();  // copy: the producer reuses its buffer
      if (mode == Mode::kInjectedSlotsCleared) {
        for (const frontend::LevelLayout& slot : producer.layout().roi_slots) {
          for (int y = 0; y < slot.h && slot.r + y < atlas.rows; ++y) {
            std::memset(&atlas.rgba[(static_cast<size_t>(slot.r + y) * atlas.size + slot.c) * 4], 0,
                        static_cast<size_t>(slot.w) * 4);
          }
        }
      }
      in.pyramid = PyramidAtlasView{atlas.rgba.data(), atlas.size, atlas.rows};
      run.published.push_back(glue.Publish(in));  // Publish copies the atlas (OwnPixels)
      continue;
    }
    run.published.push_back(glue.Publish(in));
  }
  return run;
}

bool SameBits(float a, float b) { return std::memcmp(&a, &b, sizeof(float)) == 0; }

bool SameMatrix(const HMatrix4f& a, const HMatrix4f& b) {
  return std::memcmp(a.m, b.m, sizeof(a.m)) == 0;
}

// Frame indices where the two runs differ (bitwise pose, status, located images incl. the ROI-feats
// flag and the inlier counts of the FULL-6 debug fields).
std::vector<size_t> DifferingFrames(const SeqResult& a, const SeqResult& b) {
  std::vector<size_t> bad;
  EXPECT_EQ(a.published.size(), b.published.size());
  for (size_t i = 0; i < std::min(a.published.size(), b.published.size()); ++i) {
    const PublishedFrame& x = a.published[i];
    const PublishedFrame& y = b.published[i];
    bool same = x.repeated == y.repeated && x.status == y.status && x.reason == y.reason &&
                x.source_frame_index == y.source_frame_index &&
                SameBits(x.position.x, y.position.x) && SameBits(x.position.y, y.position.y) &&
                SameBits(x.position.z, y.position.z) && SameBits(x.rotation.w, y.rotation.w) &&
                SameBits(x.rotation.x, y.rotation.x) && SameBits(x.rotation.y, y.rotation.y) &&
                SameBits(x.rotation.z, y.rotation.z) &&
                x.detected_images.size() == y.detected_images.size() &&
                x.image_events.size() == y.image_events.size();
    if (same && x.step.has_value() != y.step.has_value()) same = false;
    if (same && x.step.has_value()) {
      const std::vector<LocatedImageOut>& lx = x.step->located_images;
      const std::vector<LocatedImageOut>& ly = y.step->located_images;
      same = lx.size() == ly.size();
      for (size_t k = 0; same && k < lx.size(); ++k) {
        same = lx[k].name == ly[k].name && SameMatrix(lx[k].pose, ly[k].pose) &&
               lx[k].status == ly[k].status && lx[k].last_seen == ly[k].last_seen &&
               lx[k].roi_feats == ly[k].roi_feats && lx[k].local_inliers == ly[k].local_inliers &&
               lx[k].global_inliers == ly[k].global_inliers;
      }
    }
    if (!same) bad.push_back(i);
  }
  return bad;
}

size_t RoiFeatFrames(const SeqResult& r) {
  size_t n = 0;
  for (const PublishedFrame& p : r.published) {
    if (p.repeated || !p.step.has_value()) continue;
    for (const LocatedImageOut& l : p.step->located_images) n += l.roi_feats ? 1 : 0;
  }
  return n;
}

struct ScratchKeeper {
  ScratchKeeper() : saved_(c8::ImageTrackingState::AGGRESSIVELY_FREE_MEMORY) {
    c8::ImageTrackingState::AGGRESSIVELY_FREE_MEMORY = false;
  }
  ~ScratchKeeper() { c8::ImageTrackingState::AGGRESSIVELY_FREE_MEMORY = saved_; }
  bool saved_;
};

class InjectedRoi : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { loaded_ = LoadSynth(&synth_); }
  static Synth synth_;
  static bool loaded_;
};
Synth InjectedRoi::synth_;
bool InjectedRoi::loaded_ = false;

TEST_F(InjectedRoi, InjectedAtlasGivesTheBitIdenticalSequenceAndTheSlotsAreRead) {
  ASSERT_TRUE(loaded_) << "SLAM_REFACT_IT_SYNTH_DIR missing";
  const ScratchKeeper keep;
  const SeqResult cpu = RunSequence(synth_, Mode::kCpu);
  const SeqResult injected = RunSequence(synth_, Mode::kInjected);
  const SeqResult cleared = RunSequence(synth_, Mode::kInjectedSlotsCleared);
  const std::vector<size_t> d_inj = DifferingFrames(cpu, injected);
  const std::vector<size_t> d_clr = DifferingFrames(cpu, cleared);
  std::printf("[FULL-7 injected ROI data] %s\n", DataIdentity(synth_.dir).c_str());
  std::printf("[FULL-7 injected ROI] frames=%zu roi_frames=%zu roi_feat_frames(cpu)=%zu "
              "injected_differing=%zu slots_cleared_differing=%zu\n",
              cpu.published.size(), cpu.roi_frames, RoiFeatFrames(cpu), d_inj.size(), d_clr.size());
  // Meaningful sequence: the ROIs were asked for and the local tracker used their features.
  EXPECT_GE(cpu.roi_frames, 100u);
  EXPECT_GE(RoiFeatFrames(cpu), 30u);
  EXPECT_EQ(injected.roi_frames, cpu.roi_frames);
  // The injected path equals the CPU path.
  EXPECT_TRUE(d_inj.empty()) << "first differing frame " << (d_inj.empty() ? 0 : d_inj.front());
  // Mutation: ROI slots cleared -> the engine's result changes (it reads the slots).
  EXPECT_GE(d_clr.size(), 1u);
}

}  // namespace
}  // namespace slam_refact
