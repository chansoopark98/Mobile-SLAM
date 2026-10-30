// US-013 synthetic render gate (docs/design/image-target.md §7.1-§7.3, tolerances fixed before
// the first run): a flat target rendered with known camera poses by
// slam_refact/tools/image_target/render_synthetic.py (ctest fixture sr_image_target_synth_render,
// directory in SLAM_REFACT_IT_SYNTH_DIR), run through GlueRunner::Publish.
//   image-target-only (disableVio): IT-T1..T5 (found / updated / lost), IT-P1..P3 (pose)
//   world on:                       IT-W1, IT-W2
// Synthetic data: the result is not a device accuracy (CLAUDE.md §3).

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "image_target/target_files.h"
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

std::string SynthDir() {
  const char* d = std::getenv("SLAM_REFACT_IT_SYNTH_DIR");
  return d == nullptr ? std::string() : std::string(d);
}

bool LoadSynth(Synth* s) {
  s->dir = SynthDir();
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

Eigen::Matrix4d FromHMatrix4f(const HMatrix4f& m) {
  Eigen::Matrix4d out;
  for (int c = 0; c < 4; ++c) {
    for (int r = 0; r < 4; ++r) out(r, c) = m.m[4 * c + r];
  }
  return out;
}

Eigen::Matrix3d FromQuat(const QuatWxyz& q) {
  return Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
}

// Angle of a^T b. The quaternion form 2 atan2(|v|, |w|) is used because acos((trace - 1) / 2) has
// a resolution floor of about 0.03 deg on float-precision rotations (trace noise 1e-7 ->
// sqrt(2e-7) rad), which is above the 0.01 deg IT-P3 bound (Addendum, image-target.md §7.5).
double AngleDeg(const Eigen::Matrix3d& a, const Eigen::Matrix3d& b) {
  const Eigen::Quaterniond q(a.transpose() * b);
  return 2.0 * std::atan2(q.vec().norm(), std::fabs(q.w())) * 180.0 / kPi;
}

// The registered first-run computation, kept for the report (Addendum, §7.5).
double AngleDegAcos(const Eigen::Matrix3d& a, const Eigen::Matrix3d& b) {
  const Eigen::Matrix3d d = a.transpose() * b;
  const double c = std::max(-1.0, std::min(1.0, (d.trace() - 1.0) / 2.0));
  return std::acos(c) * 180.0 / kPi;
}

double Percentile95(std::vector<double> v) {
  if (v.empty()) return 0.0;
  std::sort(v.begin(), v.end());
  const size_t rank = static_cast<size_t>(std::ceil(0.95 * static_cast<double>(v.size())));
  return v[std::max<size_t>(rank, 1) - 1];
}

double Max(const std::vector<double>& v) {
  return v.empty() ? 0.0 : *std::max_element(v.begin(), v.end());
}

struct SeqRun {
  std::vector<PublishedFrame> published;
  float glue_scale = 1.f;
};

SeqRun RunSequence(const Synth& s, bool disable_vio) {
  EngineConfig e;
  e.capture_width = 480;
  e.capture_height = 640;
  e.disable_vio = disable_vio;
  GlueConfig g;
  GlueRunner glue(e, g);
  SeqRun run;
  run.glue_scale = g.origin.y;  // responsive scale = origin.y
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
  for (const GtFrame& f : s.frames) {
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

const DetectedImageOut* Detected(const PublishedFrame& p) {
  for (const DetectedImageOut& d : p.detected_images) {
    if (d.name == kName) return &d;
  }
  return nullptr;
}

const LocatedImageOut* Located(const PublishedFrame& p) {
  if (!p.step.has_value()) return nullptr;
  for (const LocatedImageOut& l : p.step->located_images) {
    if (l.name == kName) return &l;
  }
  return nullptr;
}

struct EventLog {
  std::vector<int> found, lost, updated;  // source frame indices
};

EventLog Events(const SeqRun& run) {
  EventLog log;
  for (const PublishedFrame& p : run.published) {
    for (const ImageTargetEvent& ev : p.image_events) {
      const int src = static_cast<int>(p.source_frame_index);
      if (ev.kind == ImageTargetEventKind::kFound) log.found.push_back(src);
      if (ev.kind == ImageTargetEventKind::kLost) log.lost.push_back(src);
      if (ev.kind == ImageTargetEventKind::kUpdated) log.updated.push_back(src);
    }
  }
  return log;
}

int CountIn(const std::vector<int>& v, int lo, int hi) {
  return static_cast<int>(std::count_if(v.begin(), v.end(), [&](int x) { return x >= lo && x <= hi; }));
}

std::string Join(const std::vector<int>& v) {
  std::string s;
  for (int x : v) s += (s.empty() ? "" : ",") + std::to_string(x);
  return s;
}

FILE* EvidenceFile(const char* name) {
  const char* out = std::getenv("SLAM_REFACT_IT_SYNTH_OUT");
  if (out == nullptr) return nullptr;
  return std::fopen((std::string(out) + "/" + name).c_str(), "w");
}

class ImageTargetSynth : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    loaded_ = LoadSynth(&synth_);
  }
  static Synth synth_;
  static bool loaded_;
};
Synth ImageTargetSynth::synth_;
bool ImageTargetSynth::loaded_ = false;

TEST_F(ImageTargetSynth, ImageTargetOnly) {
  ASSERT_TRUE(loaded_) << "synthetic sequence missing in SLAM_REFACT_IT_SYNTH_DIR='" << SynthDir()
                       << "' (fixture sr_image_target_synth_render)";
  ASSERT_EQ(synth_.frames.size(), 132u);
  const SeqRun run = RunSequence(synth_, /*disable_vio=*/true);
  const EventLog ev = Events(run);
  FILE* log = EvidenceFile("image-target-only.tsv");
  if (log != nullptr) {
    std::fprintf(log, "video\tsource\trepeated\tvisible\tdetected\tlast_seen\tfirst_seen\tstatus\tscale"
                      "\trot_err_deg\tpos_err_rel\tevents\n");
  }

  // IT-T1, T4, T5 (source frame windows of §7.2).
  for (const PublishedFrame& p : run.published) {
    if (p.repeated) continue;
    if (p.source_frame_index < 12) {
      EXPECT_TRUE(p.detected_images.empty()) << "segment A frame " << p.source_frame_index;
      EXPECT_TRUE(p.image_events.empty()) << "segment A frame " << p.source_frame_index;
    }
  }
  EXPECT_EQ(CountIn(ev.found, 12, 71), 1) << "found " << Join(ev.found);
  EXPECT_EQ(CountIn(ev.found, 12, 22), 1) << "found " << Join(ev.found);
  EXPECT_EQ(CountIn(ev.lost, 72, 77), 1) << "lost " << Join(ev.lost);
  EXPECT_EQ(CountIn(ev.found, 96, 106), 1) << "found " << Join(ev.found);
  EXPECT_EQ(ev.found.size(), 2u) << "found " << Join(ev.found);
  EXPECT_EQ(ev.lost.size(), 1u) << "lost " << Join(ev.lost);
  // IT-T2: no lost inside B after the found frame.
  EXPECT_EQ(CountIn(ev.lost, 12, 71), 0) << "lost " << Join(ev.lost);

  // IT-T3: imageupdated exactly when the image of the same name changed; none on repeats.
  const DetectedImageOut* prev = nullptr;
  for (const PublishedFrame& p : run.published) {
    if (p.repeated) {
      EXPECT_TRUE(p.image_events.empty());
      continue;
    }
    const DetectedImageOut* cur = Detected(p);
    int updated = 0;
    for (const ImageTargetEvent& e : p.image_events) updated += e.kind == ImageTargetEventKind::kUpdated;
    bool changed = false;
    if (cur != nullptr && prev != nullptr) {
      const auto scale = [](const DetectedImageOut* d) {
        const float m = std::max(d->width_in_meters, d->height_in_meters);
        return m == 0.f ? 1.f : m;
      };
      changed = cur->position.x != prev->position.x || cur->position.y != prev->position.y ||
                cur->position.z != prev->position.z || cur->rotation.w != prev->rotation.w ||
                cur->rotation.x != prev->rotation.x || cur->rotation.y != prev->rotation.y ||
                cur->rotation.z != prev->rotation.z || scale(cur) != scale(prev) ||
                cur->type != prev->type;
    }
    EXPECT_EQ(updated, changed ? 1 : 0) << "source frame " << p.source_frame_index;
    prev = cur;
  }

  // IT-P1, P2, P3.
  std::vector<double> rot_err, pos_err, grace_rot, grace_pos;
  int p3_checked = 0;
  double p3_rot_max = 0.0, p3_rot_max_acos = 0.0;
  for (const PublishedFrame& p : run.published) {
    if (p.repeated) continue;
    const GtFrame& gt = synth_.frames[static_cast<size_t>(p.source_frame_index)];
    const DetectedImageOut* d = Detected(p);
    const LocatedImageOut* l = Located(p);
    double re = -1.0, pe = -1.0;
    if (d != nullptr) {
      EXPECT_NE(l, nullptr);
      if (l == nullptr) continue;
      EXPECT_EQ(l->scale, 0.5f) << "IT-P2 source frame " << p.source_frame_index;
      const Eigen::Matrix4d cam = FromHMatrix4f(p.step->pose);
      const Eigen::Matrix4d loc = FromHMatrix4f(l->pose);
      const Eigen::Matrix4d rel = cam.inverse() * loc;
      const Eigen::Matrix3d r_gt = gt.cam_to_tv.block<3, 3>(0, 0);
      const Eigen::Vector3d t_gt = gt.cam_to_tv.block<3, 1>(0, 3);
      const Eigen::Matrix3d rel_r_gt = r_gt.transpose();
      const Eigen::Vector3d rel_p_gt =
          static_cast<double>(l->scale) * (r_gt.transpose() * (Eigen::Vector3d(0, 0, 1) - t_gt));
      re = AngleDeg(rel.block<3, 3>(0, 0), rel_r_gt);
      pe = (rel.block<3, 1>(0, 3) - rel_p_gt).norm() / rel_p_gt.norm();
      if (l->last_seen == 0) {
        rot_err.push_back(re);
        pos_err.push_back(pe);
      } else {
        grace_rot.push_back(re);
        grace_pos.push_back(pe);
      }
      // IT-P3: published place relative to the published camera = right-handed (z flip) rel_int
      // with positions times the glue scale.
      const Eigen::Matrix3d rc = FromQuat(p.rotation);
      const Eigen::Matrix3d rp = FromQuat(d->rotation);
      const Eigen::Vector3d pc(p.position.x, p.position.y, p.position.z);
      const Eigen::Vector3d pp(d->position.x, d->position.y, d->position.z);
      const Eigen::Matrix3d flip = Eigen::Vector3d(1, 1, -1).asDiagonal();
      const Eigen::Matrix3d exp_r = flip * rel.block<3, 3>(0, 0) * flip;
      const Eigen::Vector3d exp_p = run.glue_scale * (flip * rel.block<3, 1>(0, 3));
      const Eigen::Vector3d got_p = rc.transpose() * (pp - pc);
      EXPECT_LE((got_p - exp_p).norm(), 1e-4 * std::max(1.0, exp_p.norm()))
          << "IT-P3 position, source frame " << p.source_frame_index;
      p3_rot_max = std::max(p3_rot_max, AngleDeg(rc.transpose() * rp, exp_r));
      p3_rot_max_acos = std::max(p3_rot_max_acos, AngleDegAcos(rc.transpose() * rp, exp_r));
      EXPECT_LE(AngleDeg(rc.transpose() * rp, exp_r), 0.01)
          << "IT-P3 rotation, source frame " << p.source_frame_index;
      EXPECT_NEAR(d->width_in_meters, l->width * run.glue_scale, 1e-6 * l->width * run.glue_scale);
      EXPECT_NEAR(d->height_in_meters, l->height * run.glue_scale, 1e-6 * l->height * run.glue_scale);
      ++p3_checked;
    }
    if (log != nullptr) {
      std::string events;
      for (const ImageTargetEvent& e : p.image_events) {
        events += e.kind == ImageTargetEventKind::kFound ? "found;" :
                  e.kind == ImageTargetEventKind::kUpdated ? "updated;" : "lost;";
      }
      std::fprintf(log, "%lld\t%lld\t%d\t%d\t%d\t%d\t%d\t%d\t%.4f\t%.5f\t%.6f\t%s\n",
                   static_cast<long long>(p.video_frame_index),
                   static_cast<long long>(p.source_frame_index), p.repeated ? 1 : 0,
                   gt.visible ? 1 : 0, d != nullptr ? 1 : 0, l ? l->last_seen : -99,
                   l ? l->first_seen : -99, l ? l->status : -99, l ? l->scale : 0.f, re, pe,
                   events.c_str());
    }
  }
  if (log != nullptr) std::fclose(log);
  const double rot_p95 = Percentile95(rot_err), rot_max = Max(rot_err);
  const double pos_p95 = Percentile95(pos_err), pos_max = Max(pos_err);
  std::printf("[IT image-target-only] found %s lost %s updated %zu | IT-P1 n %zu rot p95 %.4f max "
              "%.4f deg, pos p95 %.5f max %.5f | grace frames %zu rot max %.4f pos max %.5f | "
              "IT-P3 n %d rot max %.6f deg (acos form %.6f)\n",
              Join(ev.found).c_str(), Join(ev.lost).c_str(), ev.updated.size(), rot_err.size(),
              rot_p95, rot_max, pos_p95, pos_max, grace_rot.size(), Max(grace_rot), Max(grace_pos),
              p3_checked, p3_rot_max, p3_rot_max_acos);
  EXPECT_GE(rot_err.size(), 40u);
  EXPECT_LE(rot_p95, 1.0);
  EXPECT_LE(rot_max, 3.0);
  EXPECT_LE(pos_p95, 0.02);
  EXPECT_LE(pos_max, 0.05);
}

TEST_F(ImageTargetSynth, WorldOn) {
  ASSERT_TRUE(loaded_) << "synthetic sequence missing in SLAM_REFACT_IT_SYNTH_DIR='" << SynthDir()
                       << "' (fixture sr_image_target_synth_render)";
  const SeqRun run = RunSequence(synth_, /*disable_vio=*/false);
  const EventLog ev = Events(run);
  // IT-W1 = IT-T1 + IT-T4.
  for (const PublishedFrame& p : run.published) {
    if (!p.repeated && p.source_frame_index < 12) {
      EXPECT_TRUE(p.detected_images.empty() && p.image_events.empty())
          << "segment A frame " << p.source_frame_index;
    }
  }
  EXPECT_EQ(CountIn(ev.found, 12, 71), 1) << "found " << Join(ev.found);
  EXPECT_EQ(CountIn(ev.found, 12, 22), 1) << "found " << Join(ev.found);
  EXPECT_EQ(ev.lost.size(), 1u) << "lost " << Join(ev.lost);
  EXPECT_EQ(CountIn(ev.lost, 72, 77), 1) << "lost " << Join(ev.lost);
  // IT-W2: TrackedImage::pose (camera in the target view) of the frames that found the target.
  std::vector<double> rot_err, pos_err;
  FILE* log = EvidenceFile("world-on.tsv");
  if (log != nullptr) std::fprintf(log, "source\tdetected\tlast_seen\tscale\trot_err_deg\tpos_err_rel\n");
  for (const PublishedFrame& p : run.published) {
    if (p.repeated) continue;
    const GtFrame& gt = synth_.frames[static_cast<size_t>(p.source_frame_index)];
    const DetectedImageOut* d = Detected(p);
    const LocatedImageOut* l = Located(p);
    if (d == nullptr || l == nullptr || l->last_seen != 0) continue;
    const Eigen::Matrix4d tv = FromHMatrix4f(l->target_view_pose);
    const Eigen::Vector3d t_gt = gt.cam_to_tv.block<3, 1>(0, 3);
    const double re = AngleDeg(tv.block<3, 3>(0, 0), gt.cam_to_tv.block<3, 3>(0, 0));
    const double pe = (tv.block<3, 1>(0, 3) - t_gt).norm() / (t_gt - Eigen::Vector3d(0, 0, 1)).norm();
    rot_err.push_back(re);
    pos_err.push_back(pe);
    if (log != nullptr) {
      std::fprintf(log, "%lld\t1\t%d\t%.4f\t%.5f\t%.6f\n", static_cast<long long>(p.source_frame_index),
                   l->last_seen, l->scale, re, pe);
    }
  }
  if (log != nullptr) std::fclose(log);
  std::printf("[IT world on] found %s lost %s | IT-W2 n %zu rot p95 %.4f max %.4f deg, pos p95 %.5f "
              "max %.5f\n",
              Join(ev.found).c_str(), Join(ev.lost).c_str(), rot_err.size(), Percentile95(rot_err),
              Max(rot_err), Percentile95(pos_err), Max(pos_err));
  EXPECT_GE(rot_err.size(), 40u);
  EXPECT_LE(Percentile95(rot_err), 1.0);
  EXPECT_LE(Percentile95(pos_err), 0.02);
}

}  // namespace
}  // namespace slam_refact
