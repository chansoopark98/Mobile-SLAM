// US-014 curved target synthetic render gates (gate file preregistration/curvy-it-gate.v<N>.md, N =
// SLAM_REFACT_CURVY_GATE_VERSION; tolerances fixed before the first real run). A cylinder and a cone rendered with known camera poses by
// slam_refact/tools/image_target/render_synthetic.py --shape (ray casting, independent of the engine), run through
// GlueRunner::Publish.
//   premises (deterministic checks; the v1 result was known, v3 gate file section 2.1):
//            G-CGEO engine-derived geometry and the 9 CurvyGeometry fields against the physical model, G-CTRAJ, G-K
//   GATES    image-target-only (disableVio): IT-T1..T5, IT-P1..P3 in the curvy model frame; world on: IT-W1, IT-W2;
//            G-CROI-C local ROI success ratio (C1 eligible >= 60, C2 ratio >= 0.90); G-CGEO of the published images
//   REPORT   JSON parameter errors (G-CGEO must see them, the sequence result is reported)
// DATA SEPARATION (v3, review B-1): the gate tests of the ordinary executable run on the REGISTERED variant
// (derived from the coordinator seed) behind the HELD device (curvy_sr_held.h over support/sr_held.h: designation file, review record,
// pinned binary, input tree sha256, one-run marker per shape); the mutant executables and SLAM_REFACT_CURVY_VARIANT=dev
// use the DEV variant (the exposed data), and every other test (mutation, report) is dev-only. Each test asserts the
// variant recorded in gt.tsv, and a dev directory that is the registered data is refused.
// Synthetic data: the result is not a device accuracy (CLAUDE.md section 3).

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "curvy_sr_held.h"
#include "frontend/frontend.h"
#include "image_target/image_target_set.h"
#include "image_target/target_files.h"
#include "image_target/target_json.h"
#include "image_target/target_loader.h"
#include "reality/engine/imagedetection/image-tracking-state.h"
#include "slam_refact/glue_runner.h"

namespace slam_refact {
namespace {

constexpr double kPi = 3.14159265358979323846;

// Registered tolerances (gate file; same values as the flat gate IT-T/P/W of image-target.md section 7).
constexpr double kRotP95 = 1.0, kRotMax = 3.0, kPosP95 = 0.02, kPosMax = 0.05;
constexpr double kGeoRel = 1e-4;       // G-CGEO relative tolerance
constexpr size_t kMinSamples = 40;
constexpr size_t kMinEligible = 60;
constexpr double kMinRatio = 0.90;

struct GtFrame {
  int index = 0;
  bool visible = false;
  int64_t time_ns = 0;
  Eigen::Matrix4d cam_to_model = Eigen::Matrix4d::Identity();
};

struct Synth {
  std::string dir, shape, name, variant;
  std::vector<GtFrame> frames;
  double camera_k[4] = {0, 0, 0, 0};
  double target_k[4] = {0, 0, 0, 0};
  double rt = 0, rb = 0, left = 0, right = 0, w0 = 0, h0 = 0;
};

#ifndef SR_CURVY_CYL_DIR_DEV
#define SR_CURVY_CYL_DIR_DEV ""
#define SR_CURVY_CONE_DIR_DEV ""
#define SR_CURVY_CYL_DIR_REG ""
#define SR_CURVY_CONE_DIR_REG ""
#endif
// The data variant of a gate test: registered (default) for the ordinary executable, dev for the mutants (and for the
// ordinary executable when SLAM_REFACT_CURVY_VARIANT=dev, a control of the code that is never a gate result).
std::string GateVariant() {
#ifdef SR_CURVY_MUTANT_BUILD
  return "dev";
#else
  const char* v = std::getenv("SLAM_REFACT_CURVY_VARIANT");
  return v != nullptr && std::string(v) == "dev" ? "dev" : "registered";
#endif
}

// Directory of the data of (shape, variant): the CMake fixture; SLAM_REFACT_IT_CYL_DIR / _CONE_DIR may point a DEV run at
// another dev dataset. (The registered directory is also checked by tree sha256 in the HELD device.)
std::string DirFor(const std::string& shape, const std::string& variant) {
  const char* d = variant == "dev" ? std::getenv(shape == "cylinder" ? "SLAM_REFACT_IT_CYL_DIR" : "SLAM_REFACT_IT_CONE_DIR")
                                   : nullptr;
  if (d != nullptr) return d;
  if (variant == "dev") return shape == "cylinder" ? SR_CURVY_CYL_DIR_DEV : SR_CURVY_CONE_DIR_DEV;
  return shape == "cylinder" ? SR_CURVY_CYL_DIR_REG : SR_CURVY_CONE_DIR_REG;
}

// "" when the engine may run on `dir`: dev data must not be sealed gate data (checked when a gate is selected); registered data
// needs the full release of the shared HELD device (and claims the one-run marker of the shape). The gate id, the version and
// the preregistration directory come from the environment (curvy_sr_held.h; no default).
std::string DataGuard(const std::string& variant, const std::string& dir, const std::string& kind) {
  const curvy_gate_held::Selection sel = curvy_gate_held::SelectFromEnv();
  std::vector<std::string> log;
  const std::string why = variant == "dev" ? curvy_gate_held::DevGuard(sel, dir, &log)
                                           : curvy_gate_held::GateGuard(sel, kind, dir, &log);
  curvy_gate_held::Print(log);
  return why;
}

// Reads the sequence of (shape, variant); returns "" or the reason it cannot be used (missing, wrong variant, HELD).
std::string LoadSynth(const std::string& shape, const std::string& variant, Synth* s) {
  s->shape = shape;
  s->name = "synth-" + shape;
  s->dir = DirFor(shape, variant);
  std::ifstream f(s->dir + "/gt.tsv");
  if (!f) return "synthetic sequence missing in " + s->dir;
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
    if (head == "variant") {
      is >> s->variant;
      continue;
    }
    if (head == "shape") continue;
    if (head == "model") {
      is >> s->rt >> s->rb >> s->left >> s->right >> s->w0 >> s->h0;
      continue;
    }
    GtFrame g;
    g.index = std::stoi(head);
    int vis = 0;
    is >> vis >> g.time_ns;
    g.visible = vis != 0;
    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) is >> g.cam_to_model(r, c);
    }
    s->frames.push_back(g);
  }
  if (s->frames.empty()) return "no frames in " + s->dir + "/gt.tsv";
  if (s->variant != variant) return s->dir + " is the '" + s->variant + "' variant, this test needs '" + variant + "'";
  return DataGuard(variant, s->dir, shape);
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

// Angle of a^T b (quaternion form, numerically stable; see image_target_synth_test.cc).
double AngleDeg(const Eigen::Matrix3d& a, const Eigen::Matrix3d& b) {
  const Eigen::Quaterniond q(a.transpose() * b);
  return 2.0 * std::atan2(q.vec().norm(), std::fabs(q.w())) * 180.0 / kPi;
}

double Percentile95(std::vector<double> v) {
  if (v.empty()) return 0.0;
  std::sort(v.begin(), v.end());
  const size_t rank = static_cast<size_t>(std::ceil(0.95 * static_cast<double>(v.size())));
  return v[std::max<size_t>(rank, 1) - 1];
}

double Max(const std::vector<double>& v) { return v.empty() ? 0.0 : *std::max_element(v.begin(), v.end()); }

// ---------------------------------------------------------------------------------------------
// Number lookup in geometry.json ("key": number), no JSON parser needed.
double JsonNumber(const std::string& text, const std::string& key) {
  const std::string pat = "\"" + key + "\":";
  const size_t at = text.find(pat);
  if (at == std::string::npos) return std::nan("");
  return std::atof(text.c_str() + at + pat.size());
}

struct GeoCompare {
  double worst_rel = 0.0;
  std::string worst_key;
};

void Track(GeoCompare* g, const char* key, double actual, double expected) {
  const double rel = std::fabs(actual - expected) / std::max(std::fabs(expected), 1e-12);
  if (rel >= g->worst_rel) {
    g->worst_rel = rel;
    g->worst_key = key;
  }
}

// G-CGEO of one target JSON: radii, activation region and the 9 CurvyGeometry fields of the engine
// loader against geometry.json (physical model of the renderer).
GeoCompare CompareGeometry(const Synth& s, const std::string& json_path,
                           const std::string& geometry_file = "geometry.json") {
  GeoCompare g;
  image_target::ImageTargetSpec spec;
  image_target::GrayImage lum;
  std::string error;
  EXPECT_TRUE(image_target::LoadImageTargetFiles(json_path, s.dir + "/" + s.name + "_luminance.pgm", &spec,
                                                 &lum, &error))
      << error;
  EngineConfig e;
  e.capture_width = 480;
  e.capture_height = 640;
  const c8::DetectionImage image = image_target::LoadDetectionImage(
      spec, lum.view(), frontend::FullIntrinsicsForDevice(e.manufacturer, e.model), e.pyramid_model);
  std::string text;
  EXPECT_TRUE(image_target::ReadFileBytes(s.dir + "/" + geometry_file, &text, &error)) << error;
  EXPECT_EQ(static_cast<int>(image.getType()), static_cast<int>(c8::DetectionImageType::CURVY));
  const c8::CurvyImageGeometry geom = image.getGeometry();
  Track(&g, "radius", geom.radius, JsonNumber(text, "rt"));
  Track(&g, "radiusBottom", geom.radiusBottom, JsonNumber(text, "rb"));
  Track(&g, "activation.left", geom.activationRegion.left, JsonNumber(text, "left"));
  Track(&g, "activation.right", geom.activationRegion.right, JsonNumber(text, "right"));
  const CurvyGeometryOut c = image_target::CurvyGeometryOf(image);
  Track(&g, "curvyCircumferenceTop", c.curvy_circumference_top, JsonNumber(text, "curvyCircumferenceTop"));
  Track(&g, "curvyCircumferenceBottom", c.curvy_circumference_bottom, JsonNumber(text, "curvyCircumferenceBottom"));
  Track(&g, "curvySideLength", c.curvy_side_length, JsonNumber(text, "curvySideLength"));
  Track(&g, "targetCircumferenceTop", c.target_circumference_top, JsonNumber(text, "targetCircumferenceTop"));
  Track(&g, "height", c.height, JsonNumber(text, "height"));
  Track(&g, "topRadius", c.top_radius, JsonNumber(text, "topRadius"));
  Track(&g, "bottomRadius", c.bottom_radius, JsonNumber(text, "bottomRadius"));
  Track(&g, "arcLengthRadians", c.arc_length_radians, JsonNumber(text, "arcLengthRadians"));
  Track(&g, "arcStartRadians", c.arc_start_radians, JsonNumber(text, "arcStartRadians"));
  return g;
}

// ---------------------------------------------------------------------------------------------
struct SeqRun {
  std::vector<PublishedFrame> published;
  float glue_scale = 1.f;
};

SeqRun RunSequence(const Synth& s, bool disable_vio, const std::string& json_path, int max_frames = 1 << 30) {
  EngineConfig e;
  e.capture_width = 480;
  e.capture_height = 640;
  e.disable_vio = disable_vio;
  GlueConfig g;
  GlueRunner glue(e, g);
  SeqRun run;
  run.glue_scale = g.origin.y;
  image_target::ImageTargetSpec spec;
  image_target::GrayImage lum;
  std::string error;
  EXPECT_TRUE(image_target::LoadImageTargetFiles(json_path, s.dir + "/" + s.name + "_luminance.pgm", &spec,
                                                 &lum, &error))
      << error;
  std::string json;
  EXPECT_TRUE(image_target::ReadFileBytes(json_path, &json, &error)) << error;
  EXPECT_EQ(glue.SetImageTargets({{json, lum.view()}}), "");
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

const DetectedImageOut* Detected(const Synth& s, const PublishedFrame& p) {
  for (const DetectedImageOut& d : p.detected_images) {
    if (d.name == s.name) return &d;
  }
  return nullptr;
}

const LocatedImageOut* Located(const Synth& s, const PublishedFrame& p) {
  if (!p.step.has_value()) return nullptr;
  for (const LocatedImageOut& l : p.step->located_images) {
    if (l.name == s.name) return &l;
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

FILE* EvidenceFile(const std::string& name) {
  const char* out = std::getenv("SLAM_REFACT_IT_SYNTH_OUT");
  if (out == nullptr) return nullptr;
  return std::fopen((std::string(out) + "/" + name).c_str(), "w");
}

// Gates collect violations as strings: the normal tests expect none, the mutation test expects some.
struct Violations {
  std::vector<std::string> list;
  void Add(const std::string& s) { list.push_back(s); }
  bool Check(bool ok, const std::string& s) {
    if (!ok) list.push_back(s);
    return ok;
  }
};

std::string Fmt(const char* f, double a) {
  char b[160];
  std::snprintf(b, sizeof(b), f, a);
  return b;
}

// IT-T1..T5 (source frame windows of image-target.md section 7.2; same windows as the flat gate).
void CheckTransitions(const Synth& s, const SeqRun& run, const EventLog& ev, Violations* v) {
  for (const PublishedFrame& p : run.published) {
    if (p.repeated || p.source_frame_index >= 12) continue;
    v->Check(p.detected_images.empty(), "IT-T1 detection in segment A, frame " + std::to_string(p.source_frame_index));
    v->Check(p.image_events.empty(), "IT-T1 event in segment A, frame " + std::to_string(p.source_frame_index));
  }
  v->Check(CountIn(ev.found, 12, 71) == 1, "IT-T1 found in B != 1: " + Join(ev.found));
  v->Check(CountIn(ev.found, 12, 22) == 1, "IT-T1 found not in [12,22]: " + Join(ev.found));
  v->Check(CountIn(ev.lost, 72, 77) == 1, "IT-T4 lost not in [72,77]: " + Join(ev.lost));
  v->Check(CountIn(ev.found, 96, 106) == 1, "IT-T5 found not in [96,106]: " + Join(ev.found));
  v->Check(ev.found.size() == 2u, "IT-T5 found count != 2: " + Join(ev.found));
  v->Check(ev.lost.size() == 1u, "IT-T5 lost count != 1: " + Join(ev.lost));
  v->Check(CountIn(ev.lost, 12, 71) == 0, "IT-T2 lost inside B: " + Join(ev.lost));
  // IT-T3: imageupdated exactly when the image of the same name changed; none on repeats.
  const DetectedImageOut* prev = nullptr;
  for (const PublishedFrame& p : run.published) {
    if (p.repeated) {
      v->Check(p.image_events.empty(), "IT-T3 event on a repeated response");
      continue;
    }
    const DetectedImageOut* cur = Detected(s, p);
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
                cur->rotation.z != prev->rotation.z || scale(cur) != scale(prev) || cur->type != prev->type;
    }
    v->Check(updated == (changed ? 1 : 0), "IT-T3 updated=" + std::to_string(updated) + " changed=" +
                                                std::to_string(changed) + " at " + std::to_string(p.source_frame_index));
    prev = cur;
  }
}

struct PoseSummary {
  size_t n = 0;
  double rot_p95 = 0, rot_max = 0, pos_p95 = 0, pos_max = 0;
  size_t grace = 0;
  double grace_rot_max = 0, grace_pos_max = 0;
  int p3_checked = 0;
  // IT-P3 component maxima (audit F-10: only the verdicts used to be kept). pos: |got - expected| in model units, with the
  // worst value of the gate's own ratio |err| / (1e-4 * max(1, |expected|)); size: relative error of width and height.
  double p3_rot_max = 0, p3_pos_max = 0, p3_pos_ratio_max = 0, p3_width_rel_max = 0, p3_height_rel_max = 0;
  double geo_worst = 0;
  std::string geo_key;
  int geo_checked = 0;
};

// IT-P1..P3 (image-target-only) in the curvy model frame, plus G-CGEO on the published images.
void CheckPoseImageOnly(const Synth& s, const SeqRun& run, const std::string& evidence, Violations* v,
                        PoseSummary* sum) {
  std::vector<double> rot_err, pos_err, grace_rot, grace_pos;
  FILE* log = evidence.empty() ? nullptr : EvidenceFile(evidence);
  if (log != nullptr) {
    std::fprintf(log, "video\tsource\trepeated\tvisible\tdetected\tlast_seen\tfirst_seen\tstatus\tscale"
                      "\trot_err_deg\tpos_err_rel\tevents\n");
  }
  std::string geometry_text, error;
  EXPECT_TRUE(image_target::ReadFileBytes(s.dir + "/geometry.json", &geometry_text, &error)) << error;
  for (const PublishedFrame& p : run.published) {
    if (p.repeated) continue;
    const GtFrame& gt = s.frames[static_cast<size_t>(p.source_frame_index)];
    const DetectedImageOut* d = Detected(s, p);
    const LocatedImageOut* l = Located(s, p);
    double re = -1.0, pe = -1.0;
    if (d != nullptr && l != nullptr) {
      v->Check(l->scale == 0.5f, "IT-P2 scale " + Fmt("%.6f", l->scale) + " at " + std::to_string(p.source_frame_index));
      const Eigen::Matrix4d cam = FromHMatrix4f(p.step->pose);
      const Eigen::Matrix4d loc = FromHMatrix4f(l->pose);
      const Eigen::Matrix4d rel = cam.inverse() * loc;
      const Eigen::Matrix3d r_gt = gt.cam_to_model.block<3, 3>(0, 0);
      const Eigen::Vector3d t_gt = gt.cam_to_model.block<3, 1>(0, 3);
      const Eigen::Vector3d rel_p_gt = static_cast<double>(l->scale) * (r_gt.transpose() * (Eigen::Vector3d::Zero() - t_gt));
      re = AngleDeg(rel.block<3, 3>(0, 0), r_gt.transpose());
      pe = (rel.block<3, 1>(0, 3) - rel_p_gt).norm() / rel_p_gt.norm();
      if (l->last_seen == 0) {
        rot_err.push_back(re);
        pos_err.push_back(pe);
      } else {
        grace_rot.push_back(re);
        grace_pos.push_back(pe);
      }
      // IT-P3: published place relative to the published camera (z flip, glue scale).
      const Eigen::Matrix3d rc = FromQuat(p.rotation);
      const Eigen::Matrix3d rp = FromQuat(d->rotation);
      const Eigen::Vector3d pc(p.position.x, p.position.y, p.position.z);
      const Eigen::Vector3d pp(d->position.x, d->position.y, d->position.z);
      const Eigen::Matrix3d flip = Eigen::Vector3d(1, 1, -1).asDiagonal();
      const Eigen::Matrix3d exp_r = flip * rel.block<3, 3>(0, 0) * flip;
      const Eigen::Vector3d exp_p = run.glue_scale * (flip * rel.block<3, 1>(0, 3));
      const Eigen::Vector3d got_p = rc.transpose() * (pp - pc);
      v->Check((got_p - exp_p).norm() <= 1e-4 * std::max(1.0, exp_p.norm()),
               "IT-P3 position at " + std::to_string(p.source_frame_index));
      const double p3_pos_err = (got_p - exp_p).norm();
      sum->p3_pos_max = std::max(sum->p3_pos_max, p3_pos_err);
      sum->p3_pos_ratio_max = std::max(sum->p3_pos_ratio_max, p3_pos_err / (1e-4 * std::max(1.0, exp_p.norm())));
      const double p3 = AngleDeg(rc.transpose() * rp, exp_r);
      sum->p3_rot_max = std::max(sum->p3_rot_max, p3);
      v->Check(p3 <= 0.01, "IT-P3 rotation " + Fmt("%.6f", p3) + " at " + std::to_string(p.source_frame_index));
      v->Check(std::fabs(d->width_in_meters - l->width * run.glue_scale) <= 1e-6 * l->width * run.glue_scale,
               "IT-P3 width at " + std::to_string(p.source_frame_index));
      v->Check(std::fabs(d->height_in_meters - l->height * run.glue_scale) <= 1e-6 * l->height * run.glue_scale,
               "IT-P3 height at " + std::to_string(p.source_frame_index));
      sum->p3_width_rel_max =
          std::max(sum->p3_width_rel_max, static_cast<double>(std::fabs(d->width_in_meters - l->width * run.glue_scale) /
                                                              (l->width * run.glue_scale)));
      sum->p3_height_rel_max =
          std::max(sum->p3_height_rel_max, static_cast<double>(std::fabs(d->height_in_meters - l->height * run.glue_scale) /
                                                               (l->height * run.glue_scale)));
      ++sum->p3_checked;
      // G-CGEO on the published image: type and the 9 fields.
      v->Check(d->type == DetectedImageType::kCurvy, "G-CGEO published type at " + std::to_string(p.source_frame_index));
      GeoCompare g;
      const CurvyGeometryOut& c = d->curvy_geometry;
      Track(&g, "curvyCircumferenceTop", c.curvy_circumference_top, JsonNumber(geometry_text, "curvyCircumferenceTop"));
      Track(&g, "curvyCircumferenceBottom", c.curvy_circumference_bottom, JsonNumber(geometry_text, "curvyCircumferenceBottom"));
      Track(&g, "curvySideLength", c.curvy_side_length, JsonNumber(geometry_text, "curvySideLength"));
      Track(&g, "targetCircumferenceTop", c.target_circumference_top, JsonNumber(geometry_text, "targetCircumferenceTop"));
      Track(&g, "height", c.height, JsonNumber(geometry_text, "height"));
      Track(&g, "topRadius", c.top_radius, JsonNumber(geometry_text, "topRadius"));
      Track(&g, "bottomRadius", c.bottom_radius, JsonNumber(geometry_text, "bottomRadius"));
      Track(&g, "arcLengthRadians", c.arc_length_radians, JsonNumber(geometry_text, "arcLengthRadians"));
      Track(&g, "arcStartRadians", c.arc_start_radians, JsonNumber(geometry_text, "arcStartRadians"));
      if (g.worst_rel >= sum->geo_worst) {
        sum->geo_worst = g.worst_rel;
        sum->geo_key = g.worst_key;
      }
      ++sum->geo_checked;
    }
    if (log != nullptr) {
      std::string events;
      for (const ImageTargetEvent& e : p.image_events) {
        events += e.kind == ImageTargetEventKind::kFound ? "found;" : e.kind == ImageTargetEventKind::kUpdated ? "updated;" : "lost;";
      }
      std::fprintf(log, "%lld\t%lld\t%d\t%d\t%d\t%d\t%d\t%d\t%.4f\t%.5f\t%.6f\t%s\n",
                   static_cast<long long>(p.video_frame_index), static_cast<long long>(p.source_frame_index),
                   p.repeated ? 1 : 0, gt.visible ? 1 : 0, d != nullptr ? 1 : 0, l ? l->last_seen : -99,
                   l ? l->first_seen : -99, l ? l->status : -99, l ? l->scale : 0.f, re, pe, events.c_str());
    }
  }
  if (log != nullptr) std::fclose(log);
  sum->n = rot_err.size();
  sum->rot_p95 = Percentile95(rot_err);
  sum->rot_max = Max(rot_err);
  sum->pos_p95 = Percentile95(pos_err);
  sum->pos_max = Max(pos_err);
  sum->grace = grace_rot.size();
  sum->grace_rot_max = Max(grace_rot);
  sum->grace_pos_max = Max(grace_pos);
  v->Check(sum->n >= kMinSamples, "IT-P1 samples " + std::to_string(sum->n) + " < 40");
  v->Check(sum->rot_p95 <= kRotP95, "IT-P1 rotation p95 " + Fmt("%.4f", sum->rot_p95));
  v->Check(sum->rot_max <= kRotMax, "IT-P1 rotation max " + Fmt("%.4f", sum->rot_max));
  v->Check(sum->pos_p95 <= kPosP95, "IT-P1 position p95 " + Fmt("%.5f", sum->pos_p95));
  v->Check(sum->pos_max <= kPosMax, "IT-P1 position max " + Fmt("%.5f", sum->pos_max));
  v->Check(sum->geo_checked > 0 && sum->geo_worst <= kGeoRel,
           "G-CGEO published " + sum->geo_key + " " + Fmt("%.3e", sum->geo_worst));
}

class CurvySynth : public ::testing::TestWithParam<const char*> {
 protected:
  // Loads the data of the test's variant (guarded); on refusal the test fails with the reason before any engine output.
  bool Load(const std::string& variant) {
    const std::string why = LoadSynth(GetParam(), variant, &synth_);
    EXPECT_EQ(why, "") << why;
    if (!why.empty()) return false;
    EXPECT_EQ(synth_.frames.size(), 132u);
    return synth_.frames.size() == 132u;
  }
  std::string Json() const { return synth_.dir + "/" + synth_.name + ".json"; }
  Synth synth_;
};

#define LOAD_GATE() \
  if (!Load(GateVariant())) return
#define LOAD_DEV() \
  if (!Load("dev")) return

// G-CGEO: the engine's geometry of the target JSON against the physical model.
TEST_P(CurvySynth, GeometryAgainstPhysicalModel) {
  LOAD_GATE();
  const GeoCompare g = CompareGeometry(synth_, Json());
  std::printf("[CURVY G-CGEO %s] worst relative deviation %.3e at %s (tolerance %.0e)\n", GetParam(),
              g.worst_rel, g.worst_key.c_str(), kGeoRel);
  EXPECT_LE(g.worst_rel, kGeoRel) << g.worst_key;
  // API type: CURVY with |top - bottom| > 0.001 is CONICAL (tracking-controller.ts:86-96).
  const bool conical = std::fabs(synth_.rt - synth_.rb) > 0.001;
  EXPECT_EQ(conical, std::string(GetParam()) == "cone");
}

// G-CTRAJ + G-K.
TEST_P(CurvySynth, TrajectoryAndKMatchRegistration) {
  LOAD_GATE();
  struct Range {
    const char* name;
    double centre, amp, hard;  // hard < 0: distance
    double lo = 1e9, hi = -1e9;
  };
  Range ranges[6] = {{"azimuth", 0.0, 35.0, 37.0}, {"y", 0.0, 0.3, 0.31}, {"distance", 2.1, 0.3, -1.0},
                     {"yaw", 0.0, 4.5, 5.5}, {"pitch", 0.0, 4.5, 5.5}, {"roll", 0.0, 8.0, 9.0}};
  for (const GtFrame& g : synth_.frames) {
    if (!g.visible) continue;
    const Eigen::Matrix3d r = g.cam_to_model.block<3, 3>(0, 0);
    const Eigen::Vector3d t = g.cam_to_model.block<3, 1>(0, 3);
    // look-at-the-axis frame (render_synthetic.look_at): z toward the origin, x = up x z, y = z x x.
    Eigen::Vector3d z = -t;
    z.normalize();
    Eigen::Vector3d x = Eigen::Vector3d(0, 1, 0).cross(z);
    x.normalize();
    Eigen::Matrix3d look;
    look.col(0) = x;
    look.col(1) = z.cross(x);
    look.col(2) = z;
    const Eigen::Matrix3d rel = look.transpose() * r;
    const Eigen::Vector3d zc = rel * Eigen::Vector3d(0, 0, 1);
    const Eigen::Vector3d xc = rel * Eigen::Vector3d(1, 0, 0);
    Eigen::Vector3d xr = Eigen::Vector3d(0, 1, 0).cross(zc);
    xr.normalize();
    const double yaw = std::atan2(zc.x(), zc.z()) * 180.0 / kPi;
    const double pitch = std::atan2(zc.y(), std::hypot(zc.x(), zc.z())) * 180.0 / kPi;
    const double roll = std::atan2(xc.cross(xr).dot(zc), xc.dot(xr)) * 180.0 / kPi;
    const double v[6] = {std::atan2(t.x(), -t.z()) * 180.0 / kPi, t.y(), std::hypot(t.x(), t.z()), yaw, pitch, roll};
    for (int i = 0; i < 6; ++i) {
      ranges[i].lo = std::min(ranges[i].lo, v[i]);
      ranges[i].hi = std::max(ranges[i].hi, v[i]);
    }
  }
  for (const Range& r : ranges) {
    std::printf("[CURVY G-CTRAJ %s] %-8s min %+.4f max %+.4f (need <= %+.3f and >= %+.3f)\n", GetParam(), r.name,
                r.lo, r.hi, r.centre - 0.9 * r.amp, r.centre + 0.9 * r.amp);
    EXPECT_LE(r.lo, r.centre - 0.9 * r.amp) << r.name;
    EXPECT_GE(r.hi, r.centre + 0.9 * r.amp) << r.name;
    if (r.hard > 0.0) {
      EXPECT_GE(r.lo, r.centre - r.hard) << r.name;
      EXPECT_LE(r.hi, r.centre + r.hard) << r.name;
    } else {
      EXPECT_GE(r.lo, 1.79);
      EXPECT_LE(r.hi, 2.41);
    }
  }
  // G-K: the engine's processing K of a step equals the renderer's camera K.
  const SeqRun run = RunSequence(synth_, true, Json(), 4);
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
    std::printf("[CURVY G-K %s] %s camera: renderer %.4f engine %.4f\n", GetParam(), comp[i], synth_.camera_k[i], camera[i]);
    EXPECT_NEAR(synth_.camera_k[i], camera[i], 0.01) << "camera K " << comp[i];
  }
}

TEST_P(CurvySynth, ImageTargetOnly) {
  LOAD_GATE();
  const SeqRun run = RunSequence(synth_, /*disable_vio=*/true, Json());
  const EventLog ev = Events(run);
  Violations v;
  CheckTransitions(synth_, run, ev, &v);
  PoseSummary sum;
  CheckPoseImageOnly(synth_, run, std::string("curvy-") + GetParam() + "-image-target-only.tsv", &v, &sum);
  std::printf("[CURVY image-target-only %s] found %s lost %s updated %zu | IT-P1 n %zu rot p95 %.4f max %.4f deg, "
              "pos p95 %.5f max %.5f | grace frames %zu rot max %.4f pos max %.5f | IT-P3 n %d rot max %.6f deg (tol 0.01), pos max %.3e "
              "(gate ratio max %.3e, tol 1), width rel max %.3e, height rel max %.3e (tol 1e-6) | "
              "G-CGEO published (n %d) worst %.3e at %s\n",
              GetParam(), Join(ev.found).c_str(), Join(ev.lost).c_str(), ev.updated.size(), sum.n, sum.rot_p95,
              sum.rot_max, sum.pos_p95, sum.pos_max, sum.grace, sum.grace_rot_max, sum.grace_pos_max,
              sum.p3_checked, sum.p3_rot_max, sum.p3_pos_max, sum.p3_pos_ratio_max, sum.p3_width_rel_max,
              sum.p3_height_rel_max, sum.geo_checked, sum.geo_worst, sum.geo_key.c_str());
  for (const std::string& s : v.list) ADD_FAILURE() << s;
}

TEST_P(CurvySynth, WorldOn) {
  LOAD_GATE();
  const SeqRun run = RunSequence(synth_, /*disable_vio=*/false, Json());
  const EventLog ev = Events(run);
  Violations v;
  for (const PublishedFrame& p : run.published) {  // IT-W1 = IT-T1 + IT-T4
    if (!p.repeated && p.source_frame_index < 12) {
      v.Check(p.detected_images.empty() && p.image_events.empty(),
              "IT-W1 detection in segment A, frame " + std::to_string(p.source_frame_index));
    }
  }
  v.Check(CountIn(ev.found, 12, 71) == 1, "IT-W1 found in B != 1: " + Join(ev.found));
  v.Check(CountIn(ev.found, 12, 22) == 1, "IT-W1 found not in [12,22]: " + Join(ev.found));
  v.Check(ev.lost.size() == 1u, "IT-W1 lost count != 1: " + Join(ev.lost));
  v.Check(CountIn(ev.lost, 72, 77) == 1, "IT-W1 lost not in [72,77]: " + Join(ev.lost));
  std::vector<double> rot_err, pos_err;
  FILE* log = EvidenceFile(std::string("curvy-") + GetParam() + "-world-on.tsv");
  if (log != nullptr) std::fprintf(log, "source\tdetected\tlast_seen\tscale\trot_err_deg\tpos_err_rel\n");
  for (const PublishedFrame& p : run.published) {
    if (p.repeated) continue;
    const GtFrame& gt = synth_.frames[static_cast<size_t>(p.source_frame_index)];
    const DetectedImageOut* d = Detected(synth_, p);
    const LocatedImageOut* l = Located(synth_, p);
    if (d == nullptr || l == nullptr || l->last_seen != 0) continue;
    const Eigen::Matrix4d tv = FromHMatrix4f(l->target_view_pose);
    const Eigen::Vector3d t_gt = gt.cam_to_model.block<3, 1>(0, 3);
    const double re = AngleDeg(tv.block<3, 3>(0, 0), gt.cam_to_model.block<3, 3>(0, 0));
    const double pe = (tv.block<3, 1>(0, 3) - t_gt).norm() / t_gt.norm();
    rot_err.push_back(re);
    pos_err.push_back(pe);
    if (log != nullptr) {
      std::fprintf(log, "%lld\t1\t%d\t%.4f\t%.5f\t%.6f\n", static_cast<long long>(p.source_frame_index),
                   l->last_seen, l->scale, re, pe);
    }
  }
  if (log != nullptr) std::fclose(log);
  std::printf("[CURVY world on %s] found %s lost %s | IT-W2 n %zu rot p95 %.4f max %.4f deg, pos p95 %.5f max %.5f\n",
              GetParam(), Join(ev.found).c_str(), Join(ev.lost).c_str(), rot_err.size(), Percentile95(rot_err),
              Max(rot_err), Percentile95(pos_err), Max(pos_err));
  v.Check(rot_err.size() >= kMinSamples, "IT-W2 samples " + std::to_string(rot_err.size()) + " < 40");
  v.Check(Percentile95(rot_err) <= kRotP95, "IT-W2 rotation p95 " + Fmt("%.4f", Percentile95(rot_err)));
  v.Check(Percentile95(pos_err) <= kPosP95, "IT-W2 position p95 " + Fmt("%.5f", Percentile95(pos_err)));
  for (const std::string& s : v.list) ADD_FAILURE() << s;
}

// ---------------------------------------------------------------------------------------------
// G-CROI-C: local matching with the ROI feats succeeds in >= 90 % of the frames that use them.
struct RoiPath {
  size_t eligible = 0, local_ok = 0, global_rescue = 0, not_found = 0;
  double ratio() const { return eligible == 0 ? 0.0 : static_cast<double>(local_ok) / eligible; }
};

RoiPath MeasureRoiPath(const Synth& s, const SeqRun& run, const char* evidence_name) {
  RoiPath out;
  FILE* log = evidence_name == nullptr ? nullptr : EvidenceFile(evidence_name);
  if (log != nullptr) {
    std::fprintf(log, "source\tvisible\troi_feats\tlast_seen\tlocal_inliers\tglobal_inliers\tlocal_ok\n");
  }
  for (const PublishedFrame& p : run.published) {
    if (p.repeated) continue;
    const GtFrame& gt = s.frames[static_cast<size_t>(p.source_frame_index)];
    const LocatedImageOut* l = Located(s, p);
    if (l == nullptr) continue;
    const bool eligible = gt.visible && l->roi_feats;
    bool pose_ok = false;  // IT-P1 per-frame maxima: rotation <= 3.0 deg, position <= 5 %
    if (eligible && p.step.has_value()) {
      const Eigen::Matrix4d rel = FromHMatrix4f(p.step->pose).inverse() * FromHMatrix4f(l->pose);
      const Eigen::Matrix3d r_gt = gt.cam_to_model.block<3, 3>(0, 0);
      const Eigen::Vector3d t_gt = gt.cam_to_model.block<3, 1>(0, 3);
      const Eigen::Vector3d rel_p_gt = static_cast<double>(l->scale) * (r_gt.transpose() * (Eigen::Vector3d::Zero() - t_gt));
      const double re = AngleDeg(rel.block<3, 3>(0, 0), r_gt.transpose());
      const double pe = (rel.block<3, 1>(0, 3) - rel_p_gt).norm() / rel_p_gt.norm();
      pose_ok = re <= kRotMax && pe <= kPosMax;
    }
    const bool local_ok = eligible && l->last_seen == 0 && l->global_inliers == 0 && l->local_inliers > 0 && pose_ok;
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
                   gt.visible ? 1 : 0, l->roi_feats ? 1 : 0, l->last_seen, l->local_inliers, l->global_inliers,
                   local_ok ? 1 : 0);
    }
  }
  if (log != nullptr) std::fclose(log);
  return out;
}

struct ScratchKeeper {  // the debug inlier counts need the public tracker scratch kept
  explicit ScratchKeeper(bool keep) : saved_(c8::ImageTrackingState::AGGRESSIVELY_FREE_MEMORY) {
    c8::ImageTrackingState::AGGRESSIVELY_FREE_MEMORY = !keep;
  }
  ~ScratchKeeper() { c8::ImageTrackingState::AGGRESSIVELY_FREE_MEMORY = saved_; }
  bool saved_;
};

bool ExpectMutant() {
#ifdef SR_CURVY_MUTANT_BUILD
  return std::getenv("SLAM_REFACT_CURVY_EXPECT_MUTANT") != nullptr;
#else
  return false;
#endif
}

TEST_P(CurvySynth, RoiLocalSuccessRatio) {
  LOAD_GATE();
  const ScratchKeeper keep(true);
  const SeqRun run = RunSequence(synth_, /*disable_vio=*/true, Json());
  const RoiPath r = MeasureRoiPath(synth_, run, (std::string("curvy-") + GetParam() + "-roi-path.tsv").c_str());
  std::printf("[CURVY G-CROI-C %s] eligible %zu, local ROI success %zu (ratio %.4f), global rescue %zu, not found %zu\n",
              GetParam(), r.eligible, r.local_ok, r.ratio(), r.global_rescue, r.not_found);
  if (ExpectMutant()) {
    EXPECT_TRUE(r.eligible < kMinEligible || r.ratio() < kMinRatio) << "mutant not detected: C1 and C2 both hold";
    return;
  }
  EXPECT_GE(r.eligible, kMinEligible) << "C1";
  EXPECT_GE(r.ratio(), kMinRatio) << "C2";
}

// Parameter errors in the target JSON: G-CGEO must see every one (the sequence gates of the same run are
// reported, not asserted: a wrong curvature can still track).
TEST_P(CurvySynth, JsonParameterErrorsAreDetected) {
  LOAD_DEV();
  const std::vector<std::string> ids = std::string(GetParam()) == "cone"
                                            ? std::vector<std::string>{"circ13", "arc80", "swapTB", "cone2cyl"}
                                            : std::vector<std::string>{"circ13", "arc80", "cyl2cone"};
  for (const std::string& id : ids) {
    const std::string json = synth_.dir + "/" + synth_.name + ".mut-" + id + ".json";
    const GeoCompare g = CompareGeometry(synth_, json);
    std::printf("[CURVY mutation %s %s] G-CGEO worst relative deviation %.3e at %s (tolerance %.0e) %s\n",
                GetParam(), id.c_str(), g.worst_rel, g.worst_key.c_str(), kGeoRel,
                g.worst_rel > kGeoRel ? "DETECTED" : "NOT DETECTED");
    EXPECT_GT(g.worst_rel, kGeoRel) << id;
    if (std::getenv("SLAM_REFACT_CURVY_MUTATION_SEQUENCES") != nullptr) {
      const SeqRun run = RunSequence(synth_, true, json);
      const EventLog ev = Events(run);
      Violations v;
      CheckTransitions(synth_, run, ev, &v);
      PoseSummary sum;
      CheckPoseImageOnly(synth_, run, "", &v, &sum);
      std::printf("[CURVY mutation-sequence %s %s] found %s lost %s | n %zu rot p95 %.4f max %.4f pos p95 %.5f max %.5f | "
                  "sequence violations %zu (REPORT)\n",
                  GetParam(), id.c_str(), Join(ev.found).c_str(), Join(ev.lost).c_str(), sum.n, sum.rot_p95,
                  sum.rot_max, sum.pos_p95, sum.pos_max, v.list.size());
    }
  }
}

// G-CGEO-crop: a cropped cylinder target (original 800 x 900 px, crop 480 x 640 at (160, 120); geometry only,
// no sequence): the loader's crop branch (curvyForTarget) and the reported full-label geometry against the
// physical statement of the renderer (render_synthetic.curvy_crop_case).
TEST(CurvyCrop, GeometryAgainstPhysicalModel) {
  Synth s;
  const std::string why = LoadSynth("cylinder", GateVariant(), &s);
  ASSERT_EQ(why, "") << why;
  const GeoCompare g = CompareGeometry(s, s.dir + "/synth-cylinder-crop.json", "geometry-crop.json");
  std::printf("[CURVY G-CGEO-crop cylinder] worst relative deviation %.3e at %s (tolerance %.0e)\n",
              g.worst_rel, g.worst_key.c_str(), kGeoRel);
  EXPECT_LE(g.worst_rel, kGeoRel) << g.worst_key;
}

// Review L-2: the Dumela label of parameterized-geometry.h:44-48 (15.3 cm x 10 cm over the front 0.725 of the circumference):
// the engine's normalised radius is the object's radius in units of the CROP height, 3.35872 / 10 = 0.335872. The header
// comment goes on to "/ (0.9 - 0.1) = 0.41984", which is 0.335872 / 0.8 (one more division by the activation height) and is
// not a normalisation the engine uses (UNVERIFIED what the comment's second step stands for). No render data is needed.
TEST(CurvyKnownAnswer, DumelaLabelNormalisedRadius) {
  const std::string json =
      "{\"type\":\"CYLINDER\",\"name\":\"dumela\",\"imagePath\":\"dumela.png\",\"properties\":{\"left\":0,\"top\":0,"
      "\"width\":1530,\"height\":1000,\"isRotated\":false,\"originalWidth\":1530,\"originalHeight\":1000,"
      "\"cylinderCircumferenceTop\":21.1034483,\"cylinderCircumferenceBottom\":21.1034483,\"cylinderSideLength\":10,"
      "\"targetCircumferenceTop\":15.3}}";
  image_target::ImageTargetSpec spec;
  std::string error;
  ASSERT_TRUE(image_target::ParseImageTargetJson(json, &spec, &error)) << error;
  image_target::GrayImage lum;
  lum.width = 306;
  lum.height = 200;
  lum.pixels.resize(static_cast<size_t>(lum.width) * lum.height);
  uint32_t x = 12345u;
  for (uint8_t& p : lum.pixels) {
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    p = static_cast<uint8_t>(40 + (x % 180));
  }
  EngineConfig e;
  const c8::DetectionImage image = image_target::LoadDetectionImage(
      spec, lum.view(), frontend::FullIntrinsicsForDevice(e.manufacturer, e.model), e.pyramid_model);
  const c8::CurvyImageGeometry g = image.getGeometry();
  std::printf("[CURVY Dumela] engine radius %.6f (crop-height units); documented 3.35872 cm / 10 = 0.335872; the comment's 0.41984 = %.6f / 0.8\n",
              g.radius, g.radius);
  EXPECT_NEAR(g.radius, 0.335872, 1e-5);
  EXPECT_NEAR(g.radius * 10.0, 3.35872, 1e-4);   // back in cm
  EXPECT_NEAR(g.radius / 0.8, 0.41984, 1e-5);    // the comment's number
  EXPECT_EQ(g.radius, g.radiusBottom);
  EXPECT_FALSE(g.isCone);
}

INSTANTIATE_TEST_SUITE_P(Shapes, CurvySynth, ::testing::Values("cylinder", "cone"),
                         [](const ::testing::TestParamInfo<const char*>& info) { return std::string(info.param); });

}  // namespace
}  // namespace slam_refact
