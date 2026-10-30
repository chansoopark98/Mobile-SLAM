// RM-6 synthetic-sequence gates (build/slam-refact-full-20261019/preregistration/rm6-it-roi-gate.v2.md):
//   G-TRAJ  the rendered trajectory reaches the registered ranges (review M2)
//   G-K     target K and camera K of the renderer equal the engine's (review M2)
//   G-ROI-C local matching with the ROI feats succeeds in >= 90 % of the frames that use them
//           (review M1), plus C0 (the debug scratch switch does not change the result)
// The same G-ROI-C test runs in three mutant executables built from copies of roi_draw.cc with a
// vertical flip (MF), a scale error (MZ) or a shift (MS) in the slot draw (CMake, label mutation).
// With SLAM_REFACT_RM6_EXPECT_MUTANT=1 the test asserts the opposite of C1 and C2 (C1 or C2 must
// fail), so any other failure (missing data, crash) still fails the test.
// Synthetic data: not a device accuracy (CLAUDE.md section 3).

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
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
#include "image_target/image_target_spec.h"
#include "image_target/target_files.h"
#include "image_target/target_loader.h"
#include "reality/engine/imagedetection/image-tracking-state.h"
#include "slam_refact/glue_runner.h"

namespace slam_refact {
namespace {

constexpr char kName[] = "synth-flat";
constexpr double kPi = 3.14159265358979323846;

struct GtFrame {
  int index = 0;
  bool visible = false;
  int64_t time_ns = 0;
  Eigen::Matrix4d cam_to_tv = Eigen::Matrix4d::Identity();
};

struct Synth {
  std::string dir;
  std::vector<GtFrame> frames;
  double camera_k[4] = {0, 0, 0, 0};
  double target_k[4] = {0, 0, 0, 0};
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
    if (head == "camera_k" || head == "target_k") {
      double* k = head == "camera_k" ? s->camera_k : s->target_k;
      for (int i = 0; i < 4; ++i) is >> k[i];
      continue;
    }
    GtFrame g;
    g.index = std::stoi(head);
    int vis = 0;
    is >> vis >> g.time_ns;
    g.visible = vis != 0;
    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) is >> g.cam_to_tv(r, c);
    }
    s->frames.push_back(g);
  }
  return !s->frames.empty();
}

// ---------------------------------------------------------------------------------------------
// G-TRAJ: the same quantities as render_synthetic.py trajectory_quantities / check_trajectory_ranges.
struct Quantities {
  double x, y, distance, yaw, pitch, roll;
};

Quantities Measure(const Eigen::Matrix4d& pose) {
  const Eigen::Matrix3d r = pose.block<3, 3>(0, 0);
  const Eigen::Vector3d t = pose.block<3, 1>(0, 3);
  const Eigen::Vector3d z = r * Eigen::Vector3d(0, 0, 1);
  const Eigen::Vector3d x_cam = r * Eigen::Vector3d(1, 0, 0);
  Eigen::Vector3d x_ref = Eigen::Vector3d(0, 1, 0).cross(z);
  x_ref.normalize();
  Quantities q;
  q.x = t.x();
  q.y = t.y();
  q.distance = 1.0 - t.z();
  q.yaw = std::atan2(z.x(), z.z()) * 180.0 / kPi;
  q.pitch = std::atan2(z.y(), std::hypot(z.x(), z.z())) * 180.0 / kPi;
  q.roll = std::atan2(x_cam.cross(x_ref).dot(z), x_cam.dot(x_ref)) * 180.0 / kPi;
  return q;
}

// ---------------------------------------------------------------------------------------------
Eigen::Matrix4d FromHMatrix4f(const HMatrix4f& m) {
  Eigen::Matrix4d out;
  for (int c = 0; c < 4; ++c) {
    for (int r = 0; r < 4; ++r) out(r, c) = m.m[4 * c + r];
  }
  return out;
}

// Angle of a^T b as in image_target_synth_test.cc (quaternion form, numerically stable).
double AngleDeg(const Eigen::Matrix3d& a, const Eigen::Matrix3d& b) {
  const Eigen::Quaterniond q(a.transpose() * b);
  return 2.0 * std::atan2(q.vec().norm(), std::fabs(q.w())) * 180.0 / kPi;
}

struct SeqRun {
  std::vector<PublishedFrame> published;
};

SeqRun RunSequence(const Synth& s, bool disable_vio, int max_frames = 1 << 30) {
  EngineConfig e;
  e.capture_width = 480;
  e.capture_height = 640;
  e.disable_vio = disable_vio;
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
  SeqRun run;
  for (const GtFrame& f : s.frames) {
    if (f.index >= max_frames) break;
    char name[32];
    std::snprintf(name, sizeof(name), "/frames/%04d.pgm", f.index);
    std::string bytes;
    image_target::GrayImage img;
    EXPECT_TRUE(image_target::ReadFileBytes(s.dir + name, &bytes, &error)) << error;
    EXPECT_TRUE(image_target::ParsePgm(bytes, &img, &error)) << error;
    FrameInput in;
    in.time_ns = f.time_ns;
    in.video_time_ns = f.time_ns;
    in.gray = img.view();
    run.published.push_back(glue.Publish(in));
  }
  return run;
}

const LocatedImageOut* Located(const PublishedFrame& p) {
  if (!p.step.has_value()) return nullptr;
  for (const LocatedImageOut& l : p.step->located_images) {
    if (l.name == kName) return &l;
  }
  return nullptr;
}

FILE* EvidenceFile(const char* name) {
  const char* out = std::getenv("SLAM_REFACT_IT_SYNTH_OUT");
  if (out == nullptr) return nullptr;
  return std::fopen((std::string(out) + "/" + name).c_str(), "w");
}

class Rm6Synth : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { loaded_ = LoadSynth(&synth_); }
  static Synth synth_;
  static bool loaded_;
};
Synth Rm6Synth::synth_;
bool Rm6Synth::loaded_ = false;

// G-TRAJ + G-K.
TEST_F(Rm6Synth, TrajectoryAndKMatchRegistration) {
  ASSERT_TRUE(loaded_) << "SLAM_REFACT_IT_SYNTH_DIR missing";
  ASSERT_EQ(synth_.frames.size(), 132u);
  struct Range {
    const char* name;
    double centre, amp, hard;  // hard < 0: no symmetric bound
    double lo = 1e9, hi = -1e9;
  };
  Range ranges[6] = {{"x", 0.0, 0.3, 0.31}, {"y", 0.0, 0.3, 0.31}, {"distance", 2.1, 0.3, -1.0},
                     {"yaw", 0.0, 12.0, 14.0}, {"pitch", 0.0, 12.0, 14.0}, {"roll", 0.0, 8.0, 9.0}};
  for (const GtFrame& g : synth_.frames) {
    if (!g.visible) continue;
    const Quantities q = Measure(g.cam_to_tv);
    const double v[6] = {q.x, q.y, q.distance, q.yaw, q.pitch, q.roll};
    for (int i = 0; i < 6; ++i) {
      ranges[i].lo = std::min(ranges[i].lo, v[i]);
      ranges[i].hi = std::max(ranges[i].hi, v[i]);
    }
  }
  for (const Range& r : ranges) {
    std::printf("[RM6 G-TRAJ] %-8s min %+.4f max %+.4f (need <= %+.3f and >= %+.3f)\n", r.name, r.lo,
                r.hi, r.centre - 0.9 * r.amp, r.centre + 0.9 * r.amp);
    EXPECT_LE(r.lo, r.centre - 0.9 * r.amp) << r.name;
    EXPECT_GE(r.hi, r.centre + 0.9 * r.amp) << r.name;
    if (r.hard > 0.0) {
      EXPECT_GE(r.lo, r.centre - r.hard) << r.name;
      EXPECT_LE(r.hi, r.centre + r.hard) << r.name;
    } else {  // distance
      EXPECT_GE(r.lo, 1.79);
      EXPECT_LE(r.hi, 2.41);
    }
  }

  // G-K: target K of the engine loader (same JSON, PGM, device K) and the processing K of a step.
  EngineConfig e;
  e.capture_width = 480;
  e.capture_height = 640;
  image_target::ImageTargetSpec spec;
  image_target::GrayImage lum;
  std::string error;
  ASSERT_TRUE(image_target::LoadImageTargetFiles(synth_.dir + "/" + std::string(kName) + ".json",
                                                 synth_.dir + "/" + kName + "_luminance.pgm", &spec,
                                                 &lum, &error))
      << error;
  const c8::DetectionImage image = image_target::LoadDetectionImage(
      spec, lum.view(), frontend::FullIntrinsicsForDevice(e.manufacturer, e.model), e.pyramid_model);
  const c8_PixelPinholeCameraModel tk = image.framePoints().intrinsic();
  const double target[4] = {tk.focalLengthHorizontal, tk.focalLengthVertical, tk.centerPointX,
                            tk.centerPointY};
  const SeqRun run = RunSequence(synth_, true, 4);
  const PublishedFrame* first = nullptr;
  for (const PublishedFrame& p : run.published) {
    if (!p.repeated && p.step.has_value()) {
      first = &p;
      break;
    }
  }
  ASSERT_NE(first, nullptr);
  const PinholeIntrinsics& ck = first->step->intrinsics;
  const double camera[4] = {ck.fx, ck.fy, ck.cx, ck.cy};
  const char* comp[4] = {"fx", "fy", "cx", "cy"};
  for (int i = 0; i < 4; ++i) {
    std::printf("[RM6 G-K] %s target: renderer %.4f engine %.4f | camera: renderer %.4f engine %.4f\n",
                comp[i], synth_.target_k[i], target[i], synth_.camera_k[i], camera[i]);
    EXPECT_NEAR(synth_.target_k[i], target[i], 0.01) << "target K " << comp[i];
    EXPECT_NEAR(synth_.camera_k[i], camera[i], 0.01) << "camera K " << comp[i];
  }
}

// ---------------------------------------------------------------------------------------------
// G-ROI-C.
struct RoiPath {
  size_t eligible = 0;
  size_t local_ok = 0;
  size_t global_rescue = 0;  // eligible, found this frame, global match path
  size_t not_found = 0;      // eligible, not found this frame
  double ratio() const { return eligible == 0 ? 0.0 : static_cast<double>(local_ok) / eligible; }
};

RoiPath MeasureRoiPath(const Synth& s, const SeqRun& run, const char* evidence_name) {
  RoiPath out;
  FILE* log = evidence_name == nullptr ? nullptr : EvidenceFile(evidence_name);
  if (log != nullptr) {
    std::fprintf(log, "source\tvisible\troi_feats\tlast_seen\tlocal_inliers\tglobal_inliers\tlocal_ok\n");  // local_ok incl. the pose bound
  }
  for (const PublishedFrame& p : run.published) {
    if (p.repeated) continue;
    const GtFrame& gt = s.frames[static_cast<size_t>(p.source_frame_index)];
    const LocatedImageOut* l = Located(p);
    if (l == nullptr) continue;
    const bool eligible = gt.visible && l->roi_feats;
    bool pose_ok = false;  // IT-P1 per-frame maxima: rotation <= 3.0 deg, position <= 5 %
    if (eligible && p.step.has_value()) {
      const Eigen::Matrix4d rel = FromHMatrix4f(p.step->pose).inverse() * FromHMatrix4f(l->pose);
      const Eigen::Matrix3d r_gt = gt.cam_to_tv.block<3, 3>(0, 0);
      const Eigen::Vector3d t_gt = gt.cam_to_tv.block<3, 1>(0, 3);
      const Eigen::Vector3d rel_p_gt =
          static_cast<double>(l->scale) * (r_gt.transpose() * (Eigen::Vector3d(0, 0, 1) - t_gt));
      const double re = AngleDeg(rel.block<3, 3>(0, 0), r_gt.transpose());
      const double pe = (rel.block<3, 1>(0, 3) - rel_p_gt).norm() / rel_p_gt.norm();
      pose_ok = re <= 3.0 && pe <= 0.05;
    }
    const bool local_ok = eligible && l->last_seen == 0 && l->global_inliers == 0 &&
                          l->local_inliers > 0 && pose_ok;
    if (eligible) {
      ++out.eligible;
      if (local_ok) {
        ++out.local_ok;
      } else if (l->last_seen == 0) {
        ++out.global_rescue;
      } else {
        ++out.not_found;
      }
    }
    if (log != nullptr) {
      std::fprintf(log, "%lld\t%d\t%d\t%d\t%d\t%d\t%d\n", static_cast<long long>(p.source_frame_index),
                   gt.visible ? 1 : 0, l->roi_feats ? 1 : 0, l->last_seen, l->local_inliers,
                   l->global_inliers, local_ok ? 1 : 0);
    }
  }
  if (log != nullptr) std::fclose(log);
  return out;
}

// The scratch of the public debug API is cleared right after tracking unless this is false.
struct ScratchKeeper {
  explicit ScratchKeeper(bool keep) : saved_(c8::ImageTrackingState::AGGRESSIVELY_FREE_MEMORY) {
    c8::ImageTrackingState::AGGRESSIVELY_FREE_MEMORY = !keep;
  }
  ~ScratchKeeper() { c8::ImageTrackingState::AGGRESSIVELY_FREE_MEMORY = saved_; }
  bool saved_;
};

// C1 and C2 (also run by the mutant executables, which must fail them: EXPECT_MUTANT mode).
TEST_F(Rm6Synth, RoiLocalSuccessRatio) {
  ASSERT_TRUE(loaded_) << "SLAM_REFACT_IT_SYNTH_DIR missing";
  const ScratchKeeper keep(true);
  const SeqRun run = RunSequence(synth_, /*disable_vio=*/true);
  const RoiPath r = MeasureRoiPath(synth_, run, "roi-path.tsv");
  std::printf("[RM6 G-ROI-C] eligible %zu, local ROI success %zu (ratio %.4f), global rescue %zu, "
              "not found %zu\n",
              r.eligible, r.local_ok, r.ratio(), r.global_rescue, r.not_found);
  if (std::getenv("SLAM_REFACT_RM6_EXPECT_MUTANT") != nullptr) {
    EXPECT_TRUE(r.eligible < 60u || r.ratio() < 0.90) << "mutant not detected: C1 and C2 both hold";
    return;
  }
  EXPECT_GE(r.eligible, 60u) << "C1";
  EXPECT_GE(r.ratio(), 0.90) << "C2";
}

// C0: the same sequence with the default scratch clearing gives bit-identical located poses.
TEST_F(Rm6Synth, DebugScratchSwitchDoesNotChangeTheResult) {
  ASSERT_TRUE(loaded_) << "SLAM_REFACT_IT_SYNTH_DIR missing";
  SeqRun kept, cleared;
  {
    const ScratchKeeper keep(true);
    kept = RunSequence(synth_, true);
  }
  {
    const ScratchKeeper keep(false);
    cleared = RunSequence(synth_, true);
  }
  ASSERT_EQ(kept.published.size(), cleared.published.size());
  size_t compared = 0;
  for (size_t i = 0; i < kept.published.size(); ++i) {
    const LocatedImageOut* a = Located(kept.published[i]);
    const LocatedImageOut* b = Located(cleared.published[i]);
    ASSERT_EQ(a == nullptr, b == nullptr) << "response " << i;
    if (a == nullptr) continue;
    EXPECT_EQ(std::memcmp(a->pose.m, b->pose.m, sizeof(a->pose.m)), 0) << "response " << i;
    EXPECT_EQ(std::memcmp(a->target_view_pose.m, b->target_view_pose.m, sizeof(a->pose.m)), 0)
        << "response " << i;
    EXPECT_EQ(a->last_seen, b->last_seen);
    EXPECT_EQ(a->first_seen, b->first_seen);
    // The inlier counts are only kept by the first run (the default clears the scratch).
    EXPECT_EQ(b->local_inliers, 0);
    EXPECT_EQ(b->global_inliers, 0);
    ++compared;
  }
  std::printf("[RM6 G-ROI-C0] %zu located responses bit-identical with and without scratch clearing\n",
              compared);
  EXPECT_GT(compared, 80u);
}

}  // namespace
}  // namespace slam_refact
