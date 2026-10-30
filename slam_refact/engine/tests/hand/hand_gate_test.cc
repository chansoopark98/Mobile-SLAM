// Gate bodies of the US-021 hand preregistration (hand-us021.v1.md; the same measurements are registered again as v2 and
// later). Label handgate. docs/spec/10-hand.md §5.
//
//   HAND-G1  image operations (resize, FIT, rotated crop) against OpenCV (cv2) on fresh random cases
//   HAND-G2  calculator math against the numpy second implementation on fresh random cases
//   HAND-G3  the same post-processing on REAL palm model outputs (frames of recorded demo footage)
//
// Modes (environment), as sky_gate_test.cc:
//   gate (default)  HELD by the shared device (tests/support/sr_held.h, through hand_held.h): the registration is named by
//                   explicit inputs, none of them has a default: SR_HAND_PREREG_DIR, SR_HAND_GATE_ID,
//                   SR_HAND_GATE_VERSION, SR_HAND_GATE_KIND, SR_HAND_REVIEWER (the sha256 of the designation file) and
//                   SR_HAND_GATE_ROOT (the data root with g1, g2, g3, g3-tensors-1; its tree sha256 is registered in the
//                   manifest held.inputs[kind]). The binary is pinned through held.pins.
//   report          SR_HAND_GATE_MODE=report with SR_HAND_G1_DIR / SR_HAND_G2_DIR / SR_HAND_G3_DIR / SR_HAND_G3_TENSORS: runs
//                   the same measurements, writes the statistics to SR_HAND_REPORT_DIR (new directory), asserts nothing;
//                   everything printed is REPORT grade. Sealed gate data of any registration is refused.
#include <gtest/gtest.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "hand/hand_geometry.h"
#include "hand/hand_image.h"
#include "hand/hand_models.h"
#include "hand_held.h"
#include "hand_test_util.h"

namespace slam_refact::hand {
namespace {

namespace fs = std::filesystem;
using testutil::DifferingPixelFraction;
using testutil::MaxDiff;
using testutil::PlantedTensors;
using testutil::ReadRaw;
using testutil::TestImage;

// --- registered tolerances (hand-us021.v1.md §3; never changed after the first real run) --------------
constexpr int kLevelTol = 3;                    // G1 resize / FIT: max |difference| per byte, levels (resolution 1)
constexpr double kCropTol = 0.02;               // G1 crop: fraction of pixels that differ at all
constexpr double kGeoTol = 1e-5;                // G2, G3: float32 resolution <= 2.4e-7 for the values compared

std::string Env(const char* n) {
  const char* v = std::getenv(n);
  return v ? v : "";
}
struct Result {
  std::string gate;
  std::vector<std::string> lines;
  std::vector<std::string> failures;
  void Note(const std::string& s) { lines.push_back(s); }
  void Fail(const std::string& s) { failures.push_back(s); }
};

// ---- G1 ---------------------------------------------------------------------------------------------
void RunG1(const std::string& dir, Result* r) {
  r->gate = "HAND-G1";
  int worst_resize = 0, worst_fit = 0;
  double worst_crop = 0.0;
  const auto resize = testutil::ReadTsv(dir, "resize_cases.tsv");
  for (size_t i = 0; i < resize.size(); ++i) {
    const auto& c = resize[i];
    const int sw = std::stoi(c[0]), sh = std::stoi(c[1]), dw = std::stoi(c[2]), dh = std::stoi(c[3]);
    std::vector<uint8_t> expected;
    if (!ReadRaw(dir, "resize-" + std::to_string(i) + ".rgb", static_cast<size_t>(dw) * dh * 3, &expected)) {
      r->Fail("resize case " + std::to_string(i) + ": expected file missing or wrong size");
      continue;
    }
    const Image3 out = ResizeLinear(TestImage(sw, sh, static_cast<uint32_t>(std::stoul(c[4]))), dw, dh);
    worst_resize = std::max(worst_resize, MaxDiff(out, expected));
  }
  const auto fit = testutil::ReadTsv(dir, "fit_cases.tsv");
  int pad_mismatch = 0;
  for (size_t i = 0; i < fit.size(); ++i) {
    const auto& c = fit[i];
    const int sw = std::stoi(c[0]), sh = std::stoi(c[1]), ow = std::stoi(c[2]), oh = std::stoi(c[3]);
    std::vector<uint8_t> expected;
    if (!ReadRaw(dir, "fit-" + std::to_string(i) + ".rgb", static_cast<size_t>(ow) * oh * 3, &expected)) {
      r->Fail("fit case " + std::to_string(i) + ": expected file missing or wrong size");
      continue;
    }
    std::array<float, 4> pad{};
    const Image3 out = FitToSize(TestImage(sw, sh, static_cast<uint32_t>(std::stoul(c[4]))), ow, oh, &pad);
    worst_fit = std::max(worst_fit, MaxDiff(out, expected));
    for (int k = 0; k < 4; ++k) pad_mismatch += static_cast<double>(pad[k]) != std::stod(c[5 + k]);
  }
  const auto crop = testutil::ReadTsv(dir, "crop_cases.tsv");
  for (size_t i = 0; i < crop.size(); ++i) {
    const auto& c = crop[i];
    const int sw = std::stoi(c[0]), sh = std::stoi(c[1]), cx = std::stoi(c[2]), cy = std::stoi(c[3]);
    const int w = std::stoi(c[4]), h = std::stoi(c[5]);
    NormRect rect;
    rect.x_center = static_cast<float>(cx) / sw;
    rect.y_center = static_cast<float>(cy) / sh;
    rect.width = static_cast<float>(w) / sw;
    rect.height = static_cast<float>(h) / sh;
    rect.rotation = static_cast<float>(std::stod(c[6]));
    std::vector<uint8_t> expected;
    if (!ReadRaw(dir, "crop-" + std::to_string(i) + ".rgb", static_cast<size_t>(w) * h * 3, &expected)) {
      r->Fail("crop case " + std::to_string(i) + ": expected file missing or wrong size");
      continue;
    }
    const Image3 out = CropNormRect(TestImage(sw, sh, static_cast<uint32_t>(std::stoul(c[7]))), rect);
    if (out.width != w || out.height != h) {
      r->Fail("crop case " + std::to_string(i) + ": size differs");
      continue;
    }
    worst_crop = std::max(worst_crop, DifferingPixelFraction(out, expected));
  }
  char buf[256];
  std::snprintf(buf, sizeof(buf), "G1: resize cases %zu worst max level diff %d; FIT cases %zu worst %d, padding mismatches %d; crop cases %zu worst differing-pixel fraction %.6f",
                resize.size(), worst_resize, fit.size(), worst_fit, pad_mismatch, crop.size(), worst_crop);
  r->Note(buf);
  if (resize.empty() || fit.empty() || crop.empty()) r->Fail("G1 has no cases");
  if (worst_resize > kLevelTol) r->Fail("G1 resize differs from cv2 by more than 3 levels");
  if (worst_fit > kLevelTol) r->Fail("G1 FIT differs from cv2 by more than 3 levels");
  if (pad_mismatch != 0) r->Fail("G1 letterbox padding differs from the cv2-side computation");
  if (worst_crop > kCropTol) r->Fail("G1 crop differs from cv2 in more than 2% of the pixels");
}

// ---- G2 / G3 -----------------------------------------------------------------------------------------
double DetDiff(const Detection& g, const std::vector<std::string>& e) {
  double m = std::fabs(g.score - std::stod(e[1]));
  m = std::max(m, std::fabs(g.xmin - std::stod(e[2])));
  m = std::max(m, std::fabs(g.ymin - std::stod(e[3])));
  m = std::max(m, std::fabs(g.width - std::stod(e[4])));
  m = std::max(m, std::fabs(g.height - std::stod(e[5])));
  for (size_t k = 0; k < g.keypoints.size() && 7 + 2 * k < e.size(); ++k) {
    m = std::max(m, std::fabs(g.keypoints[k].x - std::stod(e[6 + 2 * k])));
    m = std::max(m, std::fabs(g.keypoints[k].y - std::stod(e[7 + 2 * k])));
  }
  return m;
}

void RunG2(const std::string& dir, Result* r) {
  r->gate = "HAND-G2";
  const auto anchors = GenerateAnchors(SsdAnchorsOptions());
  double worst_det = 0, worst_rect = 0, worst_lm = 0, worst_anchor = 0;
  size_t count_mismatch = 0, cases = 0, rect_cases = 0, landmark_cases = 0;
  {
    std::ifstream f(dir + "/anchors.f32", std::ios::binary);
    std::vector<float> exp(2944 * 4);
    f.read(reinterpret_cast<char*>(exp.data()), static_cast<std::streamsize>(exp.size() * 4));
    if (!f || anchors.size() != 2944) {
      r->Fail("anchors file unreadable or anchor count != 2944");
    } else {
      for (size_t i = 0; i < anchors.size(); ++i) {
        const double dx = std::fabs(static_cast<double>(anchors[i].x_center) - exp[4 * i]);
        const double dy = std::fabs(static_cast<double>(anchors[i].y_center) - exp[4 * i + 1]);
        const double dw = std::fabs(static_cast<double>(anchors[i].w) - exp[4 * i + 2]);
        const double dh = std::fabs(static_cast<double>(anchors[i].h) - exp[4 * i + 3]);
        worst_anchor = std::max({worst_anchor, dx, dy, dw, dh});
      }
    }
  }
  const auto det_rows = testutil::ReadTsv(dir, "detect_cases.tsv");
  for (size_t row = 0; row < det_rows.size();) {
    const auto& c = det_rows[row];
    if (c[0] != "case") {
      r->Fail("detect_cases.tsv malformed");
      break;
    }
    const uint32_t seed = static_cast<uint32_t>(std::stoul(c[1]));
    const size_t n_decoded = std::stoul(c[2]), n_final = std::stoul(c[3]);
    if (row + 1 + n_final > det_rows.size()) {
      r->Fail("detect_cases.tsv truncated or its retained count is inconsistent");
      break;
    }
    std::array<float, 4> pad;
    for (int i = 0; i < 4; ++i) pad[i] = std::stof(c[4 + i]);
    const int centers[3] = {std::stoi(c[8]), std::stoi(c[9]), std::stoi(c[10])};
    std::vector<float> boxes, scores;
    PlantedTensors(seed, centers, &boxes, &scores);
    const auto decoded = DecodePalmDetections(boxes.data(), scores.data(), anchors, PalmDecodeOptions());
    auto kept = WeightedNonMaxSuppression(decoded, 0.3f, 0.5f);
    RemoveLetterbox(&kept, pad);
    count_mismatch += decoded.size() != n_decoded || kept.size() != n_final;
    for (size_t d = 0; d < std::min(kept.size(), n_final); ++d) {
      worst_det = std::max(worst_det, DetDiff(kept[d], det_rows[row + 1 + d]));
    }
    row += 1 + n_final;
    ++cases;
  }
  for (const auto& e : testutil::ReadTsv(dir, "rect_cases.tsv")) {
    const int iw = std::stoi(e[0]), ih = std::stoi(e[1]);
    Detection d;
    d.xmin = std::stof(e[2]);
    d.ymin = std::stof(e[3]);
    d.width = std::stof(e[4]);
    d.height = std::stof(e[5]);
    for (int k = 0; k < 10; ++k) d.keypoints.push_back({std::stof(e[6 + 2 * k]), std::stof(e[7 + 2 * k])});
    const size_t b = 26;
    const NormRect rect = DetectionToNormRect(d, iw, ih, 0, 2, 90.0f);
    RectTransformOptions palm;
    palm.scale_x = palm.scale_y = 2.6f;
    palm.shift_y = -0.5f;
    palm.square_long = true;
    const NormRect t = TransformNormRect(rect, iw, ih, palm);
    RectTransformOptions next;
    next.scale_x = next.scale_y = 1.6f;
    next.square_long = true;
    const NormRect nt = TransformNormRect(DetectionToNormRect(d, iw, ih, 0, 9, 90.0f), iw, ih, next);
    const float got[15] = {rect.x_center, rect.y_center, rect.width, rect.height, rect.rotation, t.x_center, t.y_center, t.width,
                           t.height, t.rotation, nt.x_center, nt.y_center, nt.width, nt.height, nt.rotation};
    for (int k = 0; k < 15; ++k) worst_rect = std::max(worst_rect, std::fabs(got[k] - std::stod(e[b + k])));
    ++rect_cases;
  }
  for (const auto& e : testutil::ReadTsv(dir, "landmark_cases.tsv")) {
    std::array<float, 4> pad;
    for (int i = 0; i < 4; ++i) pad[i] = std::stof(e[i]);
    NormRect rect;
    rect.x_center = std::stof(e[4]);
    rect.y_center = std::stof(e[5]);
    rect.width = std::stof(e[6]);
    rect.height = std::stof(e[7]);
    rect.rotation = std::stof(e[8]);
    std::vector<Landmark> lms(21);
    for (int i = 0; i < 21; ++i) lms[i] = {std::stof(e[9 + 3 * i]), std::stof(e[10 + 3 * i]), std::stof(e[11 + 3 * i])};
    RemoveLetterbox(&lms, pad);
    const auto proj = ProjectLandmarks(lms, rect);
    const size_t b = 72;
    for (int i = 0; i < 21; ++i) {
      worst_lm = std::max({worst_lm, std::fabs(proj[i].x - std::stod(e[b + 3 * i])), std::fabs(proj[i].y - std::stod(e[b + 3 * i + 1])),
                           std::fabs(proj[i].z - std::stod(e[b + 3 * i + 2]))});
    }
    const Detection det = LandmarksToDetection(proj);
    const size_t c = b + 63;
    worst_lm = std::max({worst_lm, std::fabs(det.xmin - std::stod(e[c])), std::fabs(det.ymin - std::stod(e[c + 1])),
                         std::fabs(det.width - std::stod(e[c + 2])), std::fabs(det.height - std::stod(e[c + 3]))});
    ++landmark_cases;
  }
  char buf[400];
  std::snprintf(buf, sizeof(buf),
                "G2: anchors worst %.3g; detect cases %zu (count mismatches %zu) worst %.3g; rect cases %zu worst %.3g; landmark cases %zu worst %.3g",
                worst_anchor, cases, count_mismatch, worst_det, rect_cases, worst_rect, landmark_cases, worst_lm);
  r->Note(buf);
  if (cases == 0) r->Fail("G2 has no detect cases");
  if (worst_anchor > kGeoTol) r->Fail("G2 anchors differ by more than 1e-5");
  if (count_mismatch != 0) r->Fail("G2 decoded / retained detection count differs");
  if (worst_det > kGeoTol || worst_rect > kGeoTol || worst_lm > kGeoTol) r->Fail("G2 a value differs by more than 1e-5");
}

void RunG3(const std::string& dir, const std::string& tensor_dir, Result* r) {
  r->gate = "HAND-G3";
  const auto anchors = GenerateAnchors(SsdAnchorsOptions());
  const auto rows = testutil::ReadTsv(dir, "real_cases.tsv");
  double worst_det = 0, worst_rect[5] = {0, 0, 0, 0, 0};
  size_t frames = 0, count_mismatch = 0, dets = 0;
  for (size_t row = 0; row < rows.size();) {
    const auto& c = rows[row];
    if (c[0] != "frame") {
      r->Fail("real_cases.tsv malformed");
      break;
    }
    const int k = std::stoi(c[1]);
    const size_t n_decoded = std::stoul(c[2]), n_final = std::stoul(c[3]);
    if (row + 2 + n_final > rows.size() || rows[row + 1 + n_final][0] != "rect") {
      r->Fail("real_cases.tsv truncated or its retained count is inconsistent");
      break;
    }
    std::array<float, 4> pad;
    for (int i = 0; i < 4; ++i) pad[i] = std::stof(c[4 + i]);
    const int w = std::stoi(c[8]), h = std::stoi(c[9]);
    std::vector<uint8_t> braw, sraw;
    if (!ReadRaw(tensor_dir, "palm-boxes-" + std::to_string(k) + ".f32", 2944 * 18 * 4, &braw) ||
        !ReadRaw(tensor_dir, "palm-scores-" + std::to_string(k) + ".f32", 2944 * 4, &sraw)) {
      r->Fail("tensor files of frame " + std::to_string(k) + " missing");
      break;
    }
    std::vector<float> boxes(2944 * 18), scores(2944);
    std::copy(braw.begin(), braw.end(), reinterpret_cast<uint8_t*>(boxes.data()));
    std::copy(sraw.begin(), sraw.end(), reinterpret_cast<uint8_t*>(scores.data()));
    const auto decoded = DecodePalmDetections(boxes.data(), scores.data(), anchors, PalmDecodeOptions());
    auto kept = WeightedNonMaxSuppression(decoded, 0.3f, 0.5f);
    RemoveLetterbox(&kept, pad);
    count_mismatch += decoded.size() != n_decoded || kept.size() != n_final;
    for (size_t d = 0; d < std::min(kept.size(), n_final); ++d) {
      worst_det = std::max(worst_det, DetDiff(kept[d], rows[row + 1 + d]));
    }
    dets += n_final;
    // The palm rect: first detection -> rect -> RectTransformation(2.6, shift_y -0.5, square_long). The rect
    // stage is fed the EXPECTED first detection (the float32 values of the numpy side), not our own: the rotation
    // is an atan2 of two keypoints that can be a few pixels apart on weak detections, which amplifies the 2e-7
    // difference of the detection stage by orders of magnitude; the detection stage is compared on its own above.
    NormRect palm;
    if (n_final > 0) {
      const auto& e = rows[row + 1];
      Detection ed;
      ed.score = std::stof(e[1]);
      ed.xmin = std::stof(e[2]);
      ed.ymin = std::stof(e[3]);
      ed.width = std::stof(e[4]);
      ed.height = std::stof(e[5]);
      for (size_t q = 0; 6 + 2 * q + 1 < e.size(); ++q) ed.keypoints.push_back({std::stof(e[6 + 2 * q]), std::stof(e[7 + 2 * q])});
      palm = DetectionToNormRect(ed, w, h, 0, 2, 90.0f);
    }
    RectTransformOptions ro;
    ro.scale_x = ro.scale_y = 2.6f;
    ro.shift_y = -0.5f;
    ro.square_long = true;
    const NormRect t = TransformNormRect(palm, w, h, ro);
    const auto& rect_row = rows[row + 1 + n_final];
    const float got[5] = {t.x_center, t.y_center, t.width, t.height, t.rotation};
    for (int i = 0; i < 5; ++i) worst_rect[i] = std::max(worst_rect[i], std::fabs(got[i] - std::stod(rect_row[1 + i])));
    row += 2 + n_final;
    ++frames;
  }
  const double worst = std::max({worst_det, worst_rect[0], worst_rect[1], worst_rect[2], worst_rect[3], worst_rect[4]});
  char buf[320];
  std::snprintf(buf, sizeof(buf),
                "G3: real frames %zu, retained detections %zu, count mismatches %zu, worst difference %.3g (detections %.3g; palm rect x %.3g y %.3g w %.3g h %.3g rotation %.3g)",
                frames, dets, count_mismatch, worst, worst_det, worst_rect[0], worst_rect[1], worst_rect[2], worst_rect[3], worst_rect[4]);
  r->Note(buf);
  if (frames == 0) r->Fail("G3 has no frames");
  if (count_mismatch != 0) r->Fail("G3 decoded / retained detection count differs");
  if (worst > kGeoTol) r->Fail("G3 a value differs by more than 1e-5");
}
// --- the HELD glue itself, on synthetic registration directories (protocol 12.5, 12.6). The device checks are the
// business of tests/support/sr_held_conformance.cc; these tests show that THIS gate hands the right things to the device:
// the registration named by the caller (a gate id and version that no file here knows), its own binary, the data root,
// and that every refusal happens before the one-run marker.
struct SyntheticPrereg {
  static constexpr const char* kGateId = "synth-gate";  // not a real gate: nothing in the gate names a registration
  static constexpr int kVersion = 3;
  fs::path dir;
  fs::path data_root;
  std::string designation_sha;  // the value the environment must carry
  std::string self_sha;
  std::string self_name;
  explicit SyntheticPrereg(const std::string& tag) {
    dir = fs::temp_directory_path() / ("hand-held-" + tag + "-" + std::to_string(::getpid()));
    fs::remove_all(dir);
    fs::create_directories(dir);
    data_root = dir / "data";
    sr_held::Sha256File(sr_held::SelfExe(), &self_sha);
    self_name = fs::path(sr_held::SelfExe()).filename().string();
  }
  ~SyntheticPrereg() { std::error_code ec; fs::remove_all(dir, ec); }
  std::string Id() const { return std::string(kGateId) + ".v" + std::to_string(kVersion); }
  void Write(const fs::path& path, const std::string& text) const {
    fs::create_directories(path.parent_path());
    std::ofstream(path, std::ios::binary) << text;
  }
  // A tiny sealed set: two data files and the SHA256SUMS in the format of tools/hand/seal_gate_set.sh. Returns the
  // sha256 of the SHA256SUMS.
  std::string MakeSet(const std::string& name) const {
    const std::string a = "alpha of " + name, b = "beta of " + name;
    Write(data_root / name / "a.bin", a);
    Write(data_root / name / "b.bin", b);
    const std::string sums = HeldSha(a) + "  a.bin\n" + HeldSha(b) + "  b.bin\n";
    Write(data_root / name / "SHA256SUMS", sums);
    return HeldSha(sums);
  }
  GateRequest Request() const {
    GateRequest g;
    g.prereg_dir = dir.string();
    g.gate_id = kGateId;
    g.version = kVersion;
    g.kind = "main";
    g.reviewer = designation_sha;
    return g;
  }
  // The complete registration. `mutate` edits one thing for the m-th case (0 = none).
  void Build(int mutate) {
    const std::string md_sha = HeldSha("gate text");
    Write(dir / (Id() + ".md"), mutate == 7 ? "gate text changed after registration" : "gate text");
    Write(dir / (Id() + ".review.md"), "review text");
    std::string sealed_files;
    for (const std::string& name : HandDataSets()) sealed_files += (sealed_files.empty() ? "\"" : ", \"") + MakeSet(name) + "\"";
    std::string tree, code;
    EXPECT_TRUE(sr_held::TreeSha(data_root.string(), &tree, &code)) << code;
    if (mutate == 5) Write(data_root / "g2" / "a.bin", "changed after registration");
    if (mutate == 11) Write(data_root / "g3" / "unlisted.bin", "a file nobody registered");
    const std::string pins = mutate == 12 ? std::string() : "\"" + self_name + "\": \"" + (mutate == 6 ? std::string(64, '0') : self_sha) + "\"";
    const std::string inputs = mutate == 13 ? std::string() : "\"main\": \"" + tree + "\"";
    auto manifest = [&](const std::string& review_at, const std::string& designation_file_sha, const std::string& first_run,
                        const std::string& extra) {
      return "{\"gate_id\": \"" + std::string(kGateId) + "\", \"version\": " + std::to_string(kVersion) + ", \"file\": \"" + Id() +
             ".md\", \"sha256\": \"" + md_sha + "\", \"created_at\": \"2020-01-01T00:00:00Z\"" + extra +
             ", \"held\": {\"kinds\": [\"main\"], \"pins\": {" + pins + "}, \"inputs\": {" + inputs +
             "}, \"seed\": false, \"sealed_file_sha256\": [" + sealed_files + "]}, \"review\": {\"reviewer\": \"independent\", \"at\": " +
             review_at + ", \"file\": \"" + Id() + ".review.md\", \"sha256\": \"" + HeldSha("review text") +
             "\", \"designation_file\": \"" + Id() + ".review-designation.json\", \"designation_file_sha256\": \"" +
             designation_file_sha + "\"}, \"first_real_run_at\": " + first_run + "}";
    };
    const std::string at = mutate == 1 ? "null" : "\"2020-01-02T00:00:00Z\"";
    const std::string first_run = mutate == 8 ? "\"2020-01-03T00:00:00Z\"" : "null";
    // the pin is over the manifest without review / first_real_run_at, so it does not depend on the designation sha
    sr_held::Json parsed;
    std::string err, pin;
    const std::string draft = manifest(at, std::string(64, '0'), first_run, "");
    EXPECT_TRUE(sr_held::ParseJson(draft, &parsed, &err)) << err;
    EXPECT_TRUE(sr_held::CanonManifestSha(parsed, &pin));
    const std::string designation = "{\"gate\": \"" + Id() + "\", \"first_real_run_permitted\": " + (mutate == 4 ? "false" : "true") +
                                    ", \"pinned\": {\"gate_md_sha256\": \"" + md_sha + "\", \"review_file_sha256\": \"" + HeldSha("review text") +
                                    "\", \"manifest_before_review_sha256\": \"" + pin + "\"}}";
    Write(dir / (Id() + ".review-designation.json"), designation);
    const std::string dsha = HeldSha(designation);
    // a manifest edited after the designation (case 9): the tolerance is not in the pinned canonical form any more
    Write(dir / (Id() + ".manifest.json"), manifest(at, mutate == 2 ? std::string(64, '1') : dsha, first_run, mutate == 9 ? ", \"tolerance\": 0.5" : ""));
    designation_sha = mutate == 3 ? std::string(64, '2') : (mutate == 10 ? std::string() : dsha);
  }
  std::string Run(Log* log, const std::string& root = "") const {
    return Release(Request(), root.empty() ? data_root.string() : root, {sr_held::SelfExe()}, log);
  }
};

TEST(HandHeld, TheRegistrationIsNamedByTheCallerAndNeverDefaulted) {
  std::map<std::string, std::string> env = {{"SR_HAND_PREREG_DIR", "/p"}, {"SR_HAND_GATE_ID", "g"},   {"SR_HAND_GATE_VERSION", "12"},
                                            {"SR_HAND_GATE_KIND", "main"}, {"SR_HAND_REVIEWER", std::string(64, 'a')}};
  auto lookup = [&](const char* n) { return env.count(n) ? env[n] : std::string(); };
  GateRequest g;
  EXPECT_EQ(ReadGateRequest(lookup, &g), "");
  EXPECT_EQ(g.gate_id, "g");
  EXPECT_EQ(g.version, 12);
  EXPECT_EQ(g.kind, "main");
  EXPECT_EQ(g.prereg_dir, "/p");
  for (const auto& kv : env) {  // each one missing: refused, naming it
    auto without = [&](const char* n) { return std::string(n) == kv.first ? std::string() : lookup(n); };
    const std::string held = ReadGateRequest(without, &g);
    EXPECT_NE(held.find(kv.first), std::string::npos) << kv.first << ": " << held;
  }
  for (const char* bad : {"0", "01", "x", "1x", "-1", "1234567"}) {
    env["SR_HAND_GATE_VERSION"] = bad;
    EXPECT_NE(ReadGateRequest(lookup, &g).find("SR_HAND_GATE_VERSION"), std::string::npos) << bad;
  }
  env.clear();  // nothing set: not one default
  EXPECT_NE(ReadGateRequest(lookup, &g).find("SR_HAND_GATE_ID"), std::string::npos);
}

TEST(HandHeld, OnlyTheCompleteConsistentRegistrationIsReleased) {
  const char* const expect[] = {
      "",  // 0: complete
      "[REVIEW_MISSING]",
      "[DESIGNATION_SHA]",
      "[ENV_MISMATCH]",
      "[DESIGNATION_NOT_PERMITTED]",
      "[INPUT_MISMATCH]",    // 5: a data file changed after registration
      "[PIN_MISMATCH]",      // 6: this binary is not the registered one
      "[GATE_SHA_MISMATCH]",
      "[ALREADY_RUN]",
      "[DESIGNATION_PIN]",   // 9: manifest edited after the designation
      "[ENV_MISSING]",
      "[INPUT_MISMATCH]",    // 11: a file nobody registered
      "[PIN_UNREGISTERED]",  // 12: the manifest registers no pin for this binary
      "[INPUT_UNREGISTERED]",  // 13: data supplied, none registered
  };
  for (int m = 0; m < 14; ++m) {
    SyntheticPrereg p(std::to_string(m));
    p.Build(m);
    Log log;
    const std::string held = p.Run(&log);
    const std::string marker = (p.dir / (p.Id() + ".first-run.main.marker")).string();
    if (m == 0) {
      EXPECT_EQ(held, "") << "the complete registration must be released";
      EXPECT_TRUE(fs::exists(marker)) << "released run takes the marker of ITS registration and kind";
      EXPECT_FALSE(log.empty()) << "the checks are logged";
      bool binary_line = false, pin_line = false;
      for (const auto& l : log) {
        binary_line = binary_line || (l.find("gate binary ") != std::string::npos && l.find(p.self_sha) != std::string::npos);
        pin_line = pin_line || (l.find("pin " + p.self_name) != std::string::npos && l.find(p.self_sha) != std::string::npos);
      }
      EXPECT_TRUE(binary_line) << "the log carries the gate binary path and sha256";
      EXPECT_TRUE(pin_line) << "the log carries the pin comparison of the binary";
    } else {
      EXPECT_NE(held.find(expect[m]), std::string::npos) << "case " << m << ": " << held;
      EXPECT_FALSE(fs::exists(marker)) << "case " << m << ": a refusal is not a run";
    }
  }
}

TEST(HandHeld, AnotherGateIdOrVersionIsNotThisRegistration) {
  SyntheticPrereg p("other");
  p.Build(0);
  for (int variant = 0; variant < 3; ++variant) {
    GateRequest g = p.Request();
    if (variant == 0) g.gate_id = "hand-us021";   // the v1 name: nothing is registered under it here
    if (variant == 1) g.version = SyntheticPrereg::kVersion + 1;
    if (variant == 2) g.kind = "other";
    Log log;
    const std::string held = Release(g, p.data_root.string(), {sr_held::SelfExe()}, &log);
    EXPECT_NE(held.find("HELD ["), std::string::npos) << "variant " << variant << ": " << held;
  }
  for (const auto& e : fs::directory_iterator(p.dir)) {
    EXPECT_EQ(e.path().filename().string().find("first-run"), std::string::npos) << "no marker for another registration: " << e.path();
  }
}

TEST(HandHeld, TheFirstRunMarkerIsExclusiveAndANewRunIsRefused) {
  SyntheticPrereg p("marker");
  p.Build(0);
  Log log;
  ASSERT_EQ(p.Run(&log), "");
  struct stat st;
  const fs::path marker = p.dir / (p.Id() + ".first-run.main.marker");
  ASSERT_EQ(::stat(marker.c_str(), &st), 0);
  EXPECT_EQ(st.st_mode & 0777, 0444u) << "read-only marker";
  Log log2;
  const std::string again = p.Run(&log2);
  EXPECT_NE(again.find("MARKER_EXISTS"), std::string::npos) << again;
  EXPECT_NE(again.find("this kind was run before"), std::string::npos) << again;
}

TEST(HandHeld, TheRunRecordCarriesWhatTheMarkerDoesNot) {
  SyntheticPrereg p("record");
  p.Build(0);
  Log log;
  ASSERT_EQ(p.Run(&log), "");
  const fs::path report = p.dir / "report";
  fs::create_directories(report);
  WriteRunRecord(report.string(), p.Request(), log);
  const std::string rec = HeldReadAll((report / "held-run-record.txt").string());
  EXPECT_NE(rec.find("gate_binary " + sr_held::SelfExe() + " sha256 " + p.self_sha), std::string::npos) << rec;
  EXPECT_NE(rec.find("designation_sha256 " + p.designation_sha), std::string::npos) << rec;
  EXPECT_NE(rec.find("first-run.main.marker sha256 "), std::string::npos) << rec;
  EXPECT_NE(rec.find("[SR-HELD]"), std::string::npos) << "the device log lines are copied";
}

TEST(HandHeld, EverySealedSetIsVerifiedBeforeTheMarkerAndBeforeAnyMeasurement) {
  // A change in the LAST set, an extra file nobody listed, a tampered list and a missing file are found although the first
  // sets are fine (the tree sha256 of the data root); no marker is taken, so nothing was measured.
  for (int variant = 0; variant < 4; ++variant) {
    SyntheticPrereg p("seal" + std::to_string(variant));
    p.Build(0);
    const fs::path last = p.data_root / "g3-tensors-1";
    if (variant == 0) p.Write(last / "a.bin", "tampered data");
    if (variant == 1) p.Write(last / "extra.bin", "an unlisted file");
    if (variant == 2) p.Write(last / "SHA256SUMS", "00  a.bin\n");
    if (variant == 3) fs::remove(last / "b.bin");
    Log log;
    const std::string held = p.Run(&log);
    EXPECT_NE(held.find("[INPUT_MISMATCH]"), std::string::npos) << "variant " << variant << ": " << held;
    EXPECT_FALSE(fs::exists(p.dir / (p.Id() + ".first-run.main.marker"))) << "variant " << variant;
  }
}

TEST(HandHeld, TheDataRootMustHoldTheFourSetsBeforeTheMarker) {
  SyntheticPrereg p("layout");
  p.Build(0);
  EXPECT_EQ(CheckDataLayout(p.data_root.string()), "");
  fs::remove_all(p.data_root / "g2");
  EXPECT_NE(CheckDataLayout(p.data_root.string()).find("g2"), std::string::npos);
  EXPECT_NE(CheckDataLayout((p.dir / "missing").string()).find("HELD"), std::string::npos);
}

TEST(HandHeld, ReportModeRefusesSealedData) {
  SyntheticPrereg p("report");
  p.Build(0);
  std::string why;
  const std::string prereg = p.dir.string();
  const std::string repo = p.dir.string();
  // the registered data root (tree sha256) and a renamed copy of it
  EXPECT_TRUE(IsSealedHandData(prereg, repo, p.data_root.string(), &why)) << why;
  fs::copy(p.data_root, p.dir / "copy-of-root", fs::copy_options::recursive);
  EXPECT_TRUE(IsSealedHandData(prereg, repo, (p.dir / "copy-of-root").string(), &why)) << why;
  // one set: its SHA256SUMS has a registered sha256 (held.sealed_file_sha256), renamed or not
  fs::copy(p.data_root / "g2", p.dir / "copy-of-g2", fs::copy_options::recursive);
  EXPECT_TRUE(IsSealedHandData(prereg, repo, (p.dir / "copy-of-g2").string(), &why)) << why;
  EXPECT_NE(why.find("sealed"), std::string::npos) << why;
  // only the SHA256SUMS identical (a data file changed)
  fs::copy(p.data_root / "g3", p.dir / "copy2", fs::copy_options::recursive);
  p.Write(p.dir / "copy2" / "a.bin", "different");
  EXPECT_TRUE(IsSealedHandData(prereg, repo, (p.dir / "copy2").string(), &why)) << why;
  // development data is not sealed
  p.Write(p.dir / "dev-g1" / "x.bin", "a development set");
  EXPECT_FALSE(IsSealedHandData(prereg, repo, (p.dir / "dev-g1").string(), &why)) << why;
  // the decision does not depend on which gate id the registration has: a second registration of other data is consulted too
  {
    p.Write(p.dir / "second-data" / "x.bin", "data sealed by another registration");
    std::string tree, code;
    ASSERT_TRUE(sr_held::TreeSha((p.dir / "second-data").string(), &tree, &code)) << code;
    p.Write(p.dir / "another-gate.v9.manifest.json",
            "{\"gate_id\": \"another-gate\", \"version\": 9, \"held\": {\"inputs\": {\"main\": \"" + tree + "\"}}}");
    EXPECT_TRUE(IsSealedHandData(prereg, repo, (p.dir / "second-data").string(), &why)) << why;
    EXPECT_NE(why.find("another-gate.v9.manifest.json"), std::string::npos) << why;
    fs::remove(p.dir / "another-gate.v9.manifest.json");
  }
  // a registration that cannot be read: fail closed
  p.Write(p.dir / "broken.v1.manifest.json", "{not json");
  EXPECT_TRUE(IsSealedHandData(prereg, repo, (p.dir / "dev-g1").string(), &why)) << why;
  EXPECT_NE(why.find("fail closed"), std::string::npos) << why;
  fs::remove(p.dir / "broken.v1.manifest.json");
  // no registration at all: fail closed
  const fs::path empty = p.dir / "empty-prereg";
  fs::create_directories(empty);
  EXPECT_TRUE(IsSealedHandData(empty.string(), repo, (p.dir / "dev-g1").string(), &why)) << why;
  EXPECT_NE(why.find("fail closed"), std::string::npos) << why;
}

TEST(HandHeld, ReportModeStillRefusesDataSealedByAnOlderRegistrationList) {
  // v1 kept its sealed sets in <id>.v<N>.sealed.json (sums_sha256 per set, a root); those lists are still honoured.
  SyntheticPrereg p("legacy");
  fs::create_directories(p.dir);
  const std::string sums_sha = p.MakeSet("g1");
  p.Write(p.dir / "old-gate.v1.sealed.json",
          "{\"root\": \"" + p.data_root.string() + "\", \"sets\": [{\"name\": \"g1\", \"sums_sha256\": \"" + sums_sha + "\"}]}");
  std::string why;
  const std::string prereg = p.dir.string();
  EXPECT_TRUE(IsSealedHandData(prereg, prereg, (p.data_root / "g1").string(), &why)) << why;
  fs::copy(p.data_root / "g1", p.dir / "copy", fs::copy_options::recursive);
  fs::remove(p.dir / "copy" / "SHA256SUMS");
  EXPECT_TRUE(IsSealedHandData(prereg, prereg, (p.dir / "copy").string(), &why)) << why;  // by the content of the files
  p.Write(p.data_root / "other" / "x.bin", "x");
  EXPECT_TRUE(IsSealedHandData(prereg, prereg, (p.data_root / "other").string(), &why)) << why;
  EXPECT_NE(why.find("sealed root"), std::string::npos) << why;
  p.Write(p.dir / "dev-g1" / "x.bin", "a development set");
  EXPECT_FALSE(IsSealedHandData(prereg, prereg, (p.dir / "dev-g1").string(), &why)) << why;
  fs::remove(p.dir / "old-gate.v1.sealed.json");  // list gone: fail closed
  EXPECT_TRUE(IsSealedHandData(prereg, prereg, (p.dir / "dev-g1").string(), &why)) << why;
  EXPECT_NE(why.find("fail closed"), std::string::npos) << why;
}

TEST(HandGate, ConsistencyWithTheOpenCvAndNumpyReferences) {
  const bool report_mode = Env("SR_HAND_GATE_MODE") == "report";
  const std::string report_dir = Env("SR_HAND_REPORT_DIR");
  std::string g1, g2, g3, g3t;
  Log log;
  GateRequest gate;
  // The output directory is checked before the one-run marker is taken: a wrong argument must not consume the run.
  ASSERT_FALSE(report_dir.empty()) << "SR_HAND_REPORT_DIR not set";
  ASSERT_FALSE(fs::exists(report_dir)) << "refusing to overwrite " << report_dir;
  if (report_mode) {
    g1 = Env("SR_HAND_G1_DIR");
    g2 = Env("SR_HAND_G2_DIR");
    g3 = Env("SR_HAND_G3_DIR");
    g3t = Env("SR_HAND_G3_TENSORS");
    // Report mode is for development data only: sealed gate data is refused (by content and, for older lists, location),
    // whichever registration sealed it.
    for (const std::string* d : {&g1, &g2, &g3, &g3t}) {
      std::string why;
      ASSERT_FALSE(IsSealedHandData(SR_HAND_PREREG_DIR, SR_HAND_REPO_ROOT, *d, &why))
          << "HELD: report mode does not measure sealed gate data: " << why;
    }
  } else {
    const std::string request_error = ReadGateRequest(Env, &gate);
    ASSERT_TRUE(request_error.empty()) << request_error;
    const std::string root = Env("SR_HAND_GATE_ROOT");
    ASSERT_FALSE(root.empty()) << "SR_HAND_GATE_ROOT not set";
    const std::string layout = CheckDataLayout(root);
    ASSERT_TRUE(layout.empty()) << layout;
    const std::string held = Release(gate, root, {sr_held::SelfExe()}, &log);
    for (const auto& l : log) std::printf("%s\n", l.c_str());
    ASSERT_TRUE(held.empty()) << held;
    g1 = root + "/g1";
    g2 = root + "/g2";
    g3 = root + "/g3";
    g3t = root + "/g3-tensors-1";
  }
  ASSERT_FALSE(g1.empty() || g2.empty() || g3.empty() || g3t.empty());
  fs::create_directories(report_dir);
  if (!report_mode) WriteRunRecord(report_dir, gate, log);
  std::vector<Result> results(3);
  RunG1(g1, &results[0]);
  RunG2(g2, &results[1]);
  RunG3(g3, g3t, &results[2]);
  // The registered gate file says which gates decide the verdict and which are REPORT grade; this measurement function
  // keeps the v1 split (G1 against the OpenCV oracle is the GATE; G2 and G3, the numpy second implementation of the same
  // executor, are REPORT grade: T3 self-consistency) and the tolerances above.
  int failed = 0;
  std::ofstream out(report_dir + "/hand-gate-report.txt");
  for (const auto& r : results) {
    const bool is_gate = !report_mode && r.gate == "HAND-G1";
    std::printf("[HAND %s] %s: %s\n", is_gate ? "GATE" : "REPORT", r.gate.c_str(),
                r.failures.empty() ? "within the registered tolerances" : "FAIL");
    out << (is_gate ? "GATE " : "REPORT ") << r.gate << "\n";
    for (const auto& l : r.lines) {
      std::printf("  %s\n", l.c_str());
      out << "  " << l << "\n";
    }
    for (const auto& f : r.failures) {
      std::printf("  - %s\n", f.c_str());
      out << "  FAIL: " << f << "\n";
    }
    failed += (is_gate && !r.failures.empty()) ? 1 : 0;
  }
  if (!report_mode) {
    EXPECT_EQ(failed, 0) << "HAND-G1 failed; see " << report_dir;
  }
}

}  // namespace
}  // namespace slam_refact::hand
