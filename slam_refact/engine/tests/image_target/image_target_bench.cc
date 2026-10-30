// RM-6 (review m6): desktop native cost of the image target path. Not a gate and not registered in
// ctest; run by hand:
//   sr_bench_image_target <synth-dir> [out.tsv]
// Reports (1) pyramid with / without 4 ROI slots, (2) per-frame cost with / without a loaded target on
// the synthetic sequence (world SLAM on), (3) the synchronous load time of 32 targets.
// Desktop numbers: they are not a phone's.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "frontend/pyramid_source.h"
#include "image_target/image_roi_list.h"
#include "image_target/image_target_set.h"
#include "image_target/target_files.h"
#include "slam_refact/glue_runner.h"
#include "test_util.h"

namespace slam_refact {
namespace {

using Clock = std::chrono::steady_clock;
double MsSince(Clock::time_point t0) {
  return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

double Pct(std::vector<double> v, double p) {
  if (v.empty()) return 0.0;
  std::sort(v.begin(), v.end());
  const size_t rank = static_cast<size_t>(std::ceil(p / 100.0 * static_cast<double>(v.size())));
  return v[std::max<size_t>(rank, 1) - 1];
}

void Report(const char* what, const std::vector<double>& v) {
  std::printf("%-44s n %4zu  p50 %8.2f  p95 %8.2f  p99 %8.2f  max %8.2f ms\n", what, v.size(),
              Pct(v, 50), Pct(v, 95), Pct(v, 99), Pct(v, 100));
}

std::string TargetJson(const std::string& name, int w, int h) {
  return "{\"type\":\"PLANAR\",\"properties\":{\"left\":0,\"top\":0,\"width\":" + std::to_string(w) +
         ",\"height\":" + std::to_string(h) + ",\"isRotated\":false,\"originalWidth\":" +
         std::to_string(w) + ",\"originalHeight\":" + std::to_string(h) + "},\"imagePath\":\"" + name +
         ".png\",\"name\":\"" + name + "\"}";
}

void PyramidBench() {
  const image_target::GrayImage cam = image_target::testing::SyntheticTarget(480, 640, 3);
  frontend::PyramidSource plain(480, 640, frontend::ShaderParams{}, ShaderNumericModel::kIeee);
  frontend::PyramidSource with(480, 640, frontend::ShaderParams{}, ShaderNumericModel::kIeee);
  ImageRoiList rois;
  image_target::AddNextDrawHiResScans(PinholeIntrinsics{480, 640, 239.5f, 319.5f, 506.4104f, 506.4104f},
                                      &rois);
  std::vector<double> a, b;
  for (int i = 0; i < 60; ++i) {
    auto t0 = Clock::now();
    plain.Get(PyramidAtlasView{}, RgbaImageView{}, cam.view());
    if (i >= 5) a.push_back(MsSince(t0));
    t0 = Clock::now();
    with.GetWithRois(PyramidAtlasView{}, RgbaImageView{}, cam.view(), rois.rois);
    if (i >= 5) b.push_back(MsSince(t0));
  }
  Report("pyramid Get (no ROI)", a);
  Report("pyramid GetWithRois (4 HIRES slots)", b);
}

struct Gt {
  std::vector<std::string> frames;
  std::vector<int64_t> time_ns;
};

bool LoadGt(const std::string& dir, Gt* gt) {
  std::ifstream f(dir + "/gt.tsv");
  std::string line;
  while (std::getline(f, line)) {
    std::istringstream is(line);
    std::string head;
    is >> head;
    if (head == "camera_k" || head == "target_k") continue;
    int vis = 0;
    int64_t t = 0;
    is >> vis >> t;
    char name[32];
    std::snprintf(name, sizeof(name), "/frames/%04d.pgm", std::stoi(head));
    gt->frames.push_back(dir + name);
    gt->time_ns.push_back(t);
  }
  return !gt->frames.empty();
}

void SequenceBench(const std::string& dir, bool with_target, bool disable_vio, FILE* tsv) {
  Gt gt;
  if (!LoadGt(dir, &gt)) {
    std::printf("no sequence in %s\n", dir.c_str());
    return;
  }
  EngineConfig e;
  e.capture_width = 480;
  e.capture_height = 640;
  e.disable_vio = disable_vio;
  GlueRunner glue(e, GlueConfig{});
  if (with_target) {
    image_target::ImageTargetSpec spec;
    image_target::GrayImage lum;
    std::string error, json;
    image_target::LoadImageTargetFiles(dir + "/synth-flat.json", dir + "/synth-flat_luminance.pgm", &spec,
                                       &lum, &error);
    image_target::ReadFileBytes(dir + "/synth-flat.json", &json, &error);
    const std::string err = glue.SetImageTargets({{json, lum.view()}});
    if (!err.empty()) std::printf("SetImageTargets: %s\n", err.c_str());
  }
  std::vector<double> wall, frontend;
  for (size_t k = 0; k < gt.frames.size(); ++k) {
    std::string bytes, error;
    image_target::GrayImage img;
    image_target::ReadFileBytes(gt.frames[k], &bytes, &error);
    image_target::ParsePgm(bytes, &img, &error);
    FrameInput in;
    in.time_ns = gt.time_ns[k];
    in.video_time_ns = gt.time_ns[k];
    in.gray = img.view();
    const auto t0 = Clock::now();
    const PublishedFrame p = glue.Publish(in);
    const double ms = MsSince(t0);
    if (p.repeated || !p.step.has_value() || k < 20) continue;  // after the startup recenter
    wall.push_back(ms);
    frontend.push_back(p.step->timings.frontend_ms);
    if (tsv != nullptr) {
      std::fprintf(tsv, "%s\t%s\t%zu\t%.3f\t%.3f\t%.3f\t%.3f\n", with_target ? "target" : "none",
                   disable_vio ? "vio_off" : "vio_on", k, ms, p.step->timings.frontend_ms,
                   p.step->timings.map_tracker_ms, p.step->timings.map_builder_ms);
    }
  }
  std::string label = std::string("Publish wall, ") + (with_target ? "target loaded" : "no target") +
                      (disable_vio ? ", vio off" : ", vio on");
  Report(label.c_str(), wall);
  label = std::string("  step frontend_ms, ") + (with_target ? "target" : "none") +
          (disable_vio ? ", vio off" : ", vio on");
  Report(label.c_str(), frontend);
}

void LoadBench() {
  EngineConfig cfg;
  cfg.capture_width = 480;
  cfg.capture_height = 640;
  GlueRunner glue(cfg, GlueConfig{});
  std::vector<image_target::GrayImage> lums;
  std::vector<ImageTargetInput> in;
  for (int i = 0; i < 32; ++i) lums.push_back(image_target::testing::SyntheticTarget(480, 640, 100 + i));
  std::vector<std::string> jsons;
  for (int i = 0; i < 32; ++i) jsons.push_back(TargetJson("t" + std::to_string(i), 480, 640));
  for (int i = 0; i < 32; ++i) in.push_back({jsons[static_cast<size_t>(i)], lums[static_cast<size_t>(i)].view()});
  const auto t0 = Clock::now();
  const std::string err = glue.SetImageTargets(in);
  const double ms = MsSince(t0);
  std::printf("SetImageTargets 32 x 480x640 luminance: %.0f ms total, %.0f ms per target (error '%s')\n",
              ms, ms / 32.0, err.c_str());
  std::vector<ImageTargetInput> one(in.begin(), in.begin() + 1);
  GlueRunner single(cfg, GlueConfig{});
  const auto t1 = Clock::now();
  single.SetImageTargets(one);
  std::printf("SetImageTargets 1 x 480x640 luminance: %.0f ms\n", MsSince(t1));
}

}  // namespace
}  // namespace slam_refact

int main(int argc, char** argv) {
  const std::string dir = argc > 1 ? argv[1] : "";
  FILE* tsv = argc > 2 ? std::fopen(argv[2], "w") : nullptr;
  if (tsv != nullptr) std::fprintf(tsv, "targets\tvio\tframe\twall_ms\tfrontend_ms\tmap_tracker_ms\tmap_builder_ms\n");
  slam_refact::PyramidBench();
  slam_refact::LoadBench();
  if (!dir.empty()) {
    for (bool vio_off : {false, true}) {
      slam_refact::SequenceBench(dir, false, vio_off, tsv);
      slam_refact::SequenceBench(dir, true, vio_off, tsv);
    }
  }
  if (tsv != nullptr) std::fclose(tsv);
  return 0;
}
