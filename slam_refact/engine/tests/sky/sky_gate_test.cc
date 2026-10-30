// Gate bodies of the US-020 sky preregistration (sky-us020.v1.md; the same measurements are registered again as v2 and
// later): this pipeline against a recorded run of the ORIGINAL reference runtime sky runtime
// (tools/re/runtime/run-sky-reference.mjs). Label skygate. docs/spec/09-sky.md §6.
//
// Modes (environment):
//   gate (default)  HELD by the shared device (tests/support/sr_held.h, through bench/sky_held.h): the registration is named
//                   by explicit inputs, none of them has a default: SR_SKY_PREREG_DIR, SR_SKY_GATE_ID, SR_SKY_GATE_VERSION,
//                   SR_SKY_GATE_KIND, SR_SKY_REVIEWER (the sha256 of the designation file) and SR_SKY_REF_ROOT (the root of
//                   the recorded sets: its immediate subdirectories; its tree sha256 is registered in the manifest
//                   held.inputs[kind]). The binary and the model are pinned through held.pins (basenames
//                   sr_tests_sky_gate and semantics-model.tflite). The run record <report dir>/held-run-record.txt carries
//                   the sha256 of the binary.
//   report          SR_SKY_GATE_MODE=report: runs the same measurements on the sets named in
//                   SR_SKY_GATE_SETS (comma list of directory names under SR_SKY_REF_ROOT), writes the
//                   statistics to SR_SKY_REPORT_DIR (a new directory) and asserts nothing. Everything
//                   printed in this mode is REPORT grade, never a gate result. Sealed gate sets of any registration are
//                   refused.
// Both modes write <report dir>/<set>.json with the statistics of every rule.
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

#include "sky/bench/sky_held.h"
#include "sky/bench/sky_record.h"
#include "sky/bench/sky_replay_lib.h"
#include "sky/sky_model.h"
#include "sky/sky_preprocess.h"

namespace slam_refact::sky {
namespace {

namespace fs = std::filesystem;

// --- registered tolerances (sky-us020.v1.md §3; never changed after the first real run) -----------
constexpr int kReqImageMaxLevels = 3;            // R1: |request image - original invoke image| per level
constexpr double kInferMaxAbs = 1e-4;             // R2: max |ours - original| of the score plane
constexpr double kInferRms = 1e-5;                // R2: RMS
constexpr double kInferAmbigBand = 1e-4;          // R2: pixels within this of 0.4 may flip the percentage
constexpr int kTexLevels = 3;                     // R3: per-channel level tolerance
constexpr double kTexMinFraction = 0.95;          // R3: fraction of pixels (all channels) within it
constexpr double kTexMaxMeanAbs = 1.0;            // R3: mean |difference| over all channels
constexpr double kIntrinsicsRel = 1e-6;           // R4
constexpr double kPoseAbs = 1e-5;                 // R4 (rotation components up to sign, position)

std::string Env(const char* n) {
  const char* v = std::getenv(n);
  return v ? v : "";
}

struct Stats {
  // R1
  int req_count_ours = 0, req_count_orig = 0, rotation_mismatch = 0;
  int req_image_worst = 0;
  // R2
  double infer_max_abs = 0, infer_rms_worst = 0;
  int infer_pct_count_diff_worst = 0, infer_ambig_worst = 0;
  int infer_pct_violations = 0;
  // R3
  double tex_min_fraction = 1.0, tex_max_mean_abs = 0.0;
  int tex_frames = 0, tex_missing = 0, tex_dims_mismatch = 0, first_valid_ours = -1, first_valid_orig = -1;
  // R4
  double intr_worst_rel = 0, pose_worst = 0;
  // R5
  bool events_equal = false;
  std::string events_ours, events_orig;
  bool layerfound_pct_equal = true;
  std::vector<std::string> failures;
};

double QuatDiff(const QuaternionXyzw& a, const QuaternionXyzw& b) {
  double d1 = 0, d2 = 0;
  const double av[4] = {a.x, a.y, a.z, a.w}, bv[4] = {b.x, b.y, b.z, b.w};
  for (int i = 0; i < 4; ++i) {
    d1 = std::max(d1, std::fabs(av[i] - bv[i]));
    d2 = std::max(d2, std::fabs(av[i] + bv[i]));
  }
  return std::min(d1, d2);
}

bool Measure(const SkyRecord& rec, const std::vector<uint8_t>& model_bytes, const ReplayOptions& opt, Stats* st,
             std::string* error) {
  // ---- inject-worker replay: R1, R3, R4, R5, request counts --------------------------------------
  ReplayResult res;
  if (!RunReplay(rec, opt, nullptr, nullptr, &res, error)) {
    return false;
  }
  st->req_count_ours = static_cast<int>(res.requests.size());
  st->req_count_orig = static_cast<int>(rec.invokes.size());
  st->rotation_mismatch = res.rotation_mismatch;
  for (size_t n = 0; n < res.requests.size() && n < rec.invokes.size(); ++n) {
    Rgba8Image orig;
    if (!rec.ReadImage(rec.invokes[n].img, kModelInputWidth, kModelInputHeight, &orig)) {
      *error = "cannot read " + rec.invokes[n].img;
      return false;
    }
    int worst = 0;
    for (size_t i = 0; i < orig.px.size(); ++i) {
      worst = std::max(worst, std::abs(static_cast<int>(res.requests[n].img.px[i]) - orig.px[i]));
    }
    st->req_image_worst = std::max(st->req_image_worst, worst);
  }
  for (const auto& u : rec.updates) {
    if (u.k < 0 || static_cast<size_t>(u.k) >= res.frames.size()) {
      ++st->tex_missing;
      continue;
    }
    const ReplayFrame& f = res.frames[u.k];
    if (st->first_valid_orig < 0 || u.k < st->first_valid_orig) st->first_valid_orig = u.k;
    if (!f.valid) {
      ++st->tex_missing;
      continue;
    }
    if (st->first_valid_ours < 0 || u.k < st->first_valid_ours) st->first_valid_ours = u.k;
    Rgba8Image orig;
    if (!rec.ReadImage(u.file, u.tex_w, u.tex_h, &orig)) {
      *error = "cannot read " + u.file;
      return false;
    }
    if (f.out.sky.texture.width != orig.width || f.out.sky.texture.height != orig.height) {
      ++st->tex_dims_mismatch;
      continue;
    }
    size_t within = 0;
    double sum = 0;
    for (size_t p = 0; p < orig.px.size(); p += 4) {
      bool ok = true;
      for (int c = 0; c < 4; ++c) {
        const int d = std::abs(static_cast<int>(f.out.sky.texture.px[p + c]) - orig.px[p + c]);
        sum += d;
        ok = ok && d <= kTexLevels;
      }
      within += ok;
    }
    const double frac = static_cast<double>(within) / static_cast<double>(orig.px.size() / 4);
    const double mean = sum / static_cast<double>(orig.px.size());
    st->tex_min_fraction = std::min(st->tex_min_fraction, frac);
    st->tex_max_mean_abs = std::max(st->tex_max_mean_abs, mean);
    ++st->tex_frames;
    // R4: response camera.
    for (int i = 0; i < 16; ++i) {
      const double e = u.intrinsics[i], o = f.out.intrinsics[i];
      st->intr_worst_rel = std::max(st->intr_worst_rel, std::fabs(o - e) / std::max(1.0, std::fabs(e)));
    }
    st->pose_worst = std::max(st->pose_worst, QuatDiff(f.out.rotation, u.rot));
    st->pose_worst = std::max({st->pose_worst, std::fabs(f.out.position.x - u.pos.x),
                               std::fabs(f.out.position.y - u.pos.y),
                               std::fabs(f.out.position.z - u.pos.z)});
  }
  // R5: events (names in order, layerfound percentage).
  std::string ours, orig;
  for (const auto& e : res.events) ours += e.name + ";";
  for (const auto& e : rec.events) orig += e.name + ";";
  st->events_ours = ours;
  st->events_orig = orig;
  st->events_equal = ours == orig;
  {
    std::vector<std::string> ours_found;
    for (const auto& e : res.events) {
      if (e.name == "layerfound") ours_found.push_back(e.detail_json);
    }
    size_t i = 0;
    for (const auto& e : rec.events) {
      if (e.name != "layerfound") continue;
      char buf[64];
      std::snprintf(buf, sizeof(buf), "%.17g", e.percentage);
      const std::string want_prefix = "{\"name\":\"sky\",\"percentage\":";
      if (i >= ours_found.size()) {
        st->layerfound_pct_equal = false;
      } else {
        const double got = std::strtod(ours_found[i].substr(want_prefix.size()).c_str(), nullptr);
        st->layerfound_pct_equal = st->layerfound_pct_equal && got == e.percentage;
      }
      ++i;
    }
  }
  // ---- open loop of the model (R2): every recorded ORIGINAL worker input through our model ---------
  SkyModel model;
  if (!model.Load(model_bytes, true, error)) {
    return false;
  }
  for (const auto& iv : rec.invokes) {
    Rgba8Image img;
    std::vector<float> orig_data;
    if (!rec.ReadImage(iv.img, kModelInputWidth, kModelInputHeight, &img) ||
        !rec.ReadFloats(iv.data, &orig_data)) {
      *error = "cannot read invoke " + iv.img;
      return false;
    }
    SkyInference inf;
    if (!model.Invoke(img.px.data(), &inf) || inf.data.size() != orig_data.size()) {
      *error = "model failed on " + iv.img;
      return false;
    }
    double sq = 0, mx = 0;
    int ambig = 0, orig_count = 0, our_count = 0;
    for (size_t i = 0; i < orig_data.size(); ++i) {
      const double d = static_cast<double>(inf.data[i]) - orig_data[i];
      sq += d * d;
      mx = std::max(mx, std::fabs(d));
      ambig += std::fabs(static_cast<double>(orig_data[i]) - kModelThreshold) <= kInferAmbigBand;
      orig_count += static_cast<double>(orig_data[i]) > kModelThreshold;
      our_count += static_cast<double>(inf.data[i]) > kModelThreshold;
    }
    const double rms = std::sqrt(sq / static_cast<double>(orig_data.size()));
    st->infer_max_abs = std::max(st->infer_max_abs, mx);
    st->infer_rms_worst = std::max(st->infer_rms_worst, rms);
    const int cd = std::abs(our_count - orig_count);
    st->infer_pct_count_diff_worst = std::max(st->infer_pct_count_diff_worst, cd);
    st->infer_ambig_worst = std::max(st->infer_ambig_worst, ambig);
    st->infer_pct_violations += cd > ambig;
  }
  return true;
}

void Evaluate(Stats* st) {
  auto fail = [&](const std::string& m) { st->failures.push_back(m); };
  if (st->req_count_ours != st->req_count_orig) fail("R6 request count differs from the original invoke count");
  if (st->rotation_mismatch != 0) fail("R6 request rotation differs from the original invoke rotation");
  if (st->req_image_worst > kReqImageMaxLevels) fail("R1 request image differs by more than 3 levels");
  if (st->infer_max_abs > kInferMaxAbs) fail("R2 score plane max abs above 1e-4");
  if (st->infer_rms_worst > kInferRms) fail("R2 score plane RMS above 1e-5");
  if (st->infer_pct_violations != 0) fail("R2 percentage differs by more than the ambiguity band");
  if (st->tex_missing != 0) fail("R3 an original texture has no valid output of ours");
  if (st->tex_dims_mismatch != 0) fail("R3 texture size differs");
  if (st->first_valid_ours != st->first_valid_orig) fail("R3 first valid output frame differs");
  if (st->tex_min_fraction < kTexMinFraction) fail("R3 fraction of pixels within 3 levels below 0.95");
  if (st->tex_max_mean_abs > kTexMaxMeanAbs) fail("R3 mean abs difference above 1.0 level");
  if (st->intr_worst_rel > kIntrinsicsRel) fail("R4 response intrinsics differ by more than 1e-6 relative");
  if (st->pose_worst > kPoseAbs) fail("R4 response rotation or position differs by more than 1e-5");
  if (!st->events_equal) fail("R5 event name sequence differs");
  if (!st->layerfound_pct_equal) fail("R5 layerfound percentage differs");
}

std::string ToJson(const std::string& set, const Stats& s) {
  std::ostringstream o;
  o.precision(17);
  o << "{\n \"set\": \"" << set << "\",\n"
    << " \"R1_request_image_worst_level\": " << s.req_image_worst << ",\n"
    << " \"R2_infer_max_abs\": " << s.infer_max_abs << ",\n \"R2_infer_rms_worst\": " << s.infer_rms_worst
    << ",\n \"R2_pct_count_diff_worst\": " << s.infer_pct_count_diff_worst
    << ",\n \"R2_ambig_pixels_worst\": " << s.infer_ambig_worst
    << ",\n \"R2_pct_violations\": " << s.infer_pct_violations << ",\n"
    << " \"R3_frames\": " << s.tex_frames << ",\n \"R3_min_fraction_within_3_levels\": " << s.tex_min_fraction
    << ",\n \"R3_max_mean_abs\": " << s.tex_max_mean_abs << ",\n \"R3_missing\": " << s.tex_missing
    << ",\n \"R3_first_valid_ours\": " << s.first_valid_ours << ",\n \"R3_first_valid_orig\": " << s.first_valid_orig
    << ",\n"
    << " \"R4_intrinsics_worst_rel\": " << s.intr_worst_rel << ",\n \"R4_pose_worst_abs\": " << s.pose_worst << ",\n"
    << " \"R5_events_equal\": " << (s.events_equal ? "true" : "false") << ",\n \"R5_events_ours\": \""
    << s.events_ours << "\",\n \"R5_events_orig\": \"" << s.events_orig << "\",\n"
    << " \"R5_layerfound_pct_equal\": " << (s.layerfound_pct_equal ? "true" : "false") << ",\n"
    << " \"R6_requests_ours\": " << s.req_count_ours << ",\n \"R6_requests_orig\": " << s.req_count_orig
    << ",\n \"R6_rotation_mismatch\": " << s.rotation_mismatch << ",\n \"failures\": [";
  for (size_t i = 0; i < s.failures.size(); ++i) {
    o << (i ? ", " : "") << "\"" << s.failures[i] << "\"";
  }
  o << "]\n}\n";
  return o.str();
}

using Log = std::vector<std::string>;

// --- the HELD glue itself, on synthetic registration directories (protocol 12.5, 12.6). The device checks are the
// business of tests/support/sr_held_conformance.cc; these tests show that THIS gate hands the right things to the device:
// the registration named by the caller (a gate id and version that no file here knows), its own binary and the model, the
// data root, and that every refusal happens before the one-run marker.
struct SyntheticPrereg {
  static constexpr const char* kGateId = "synth-sky";  // not a real gate: nothing in the gate names a registration
  static constexpr int kVersion = 4;
  fs::path dir;
  fs::path ref_root;
  fs::path model;
  std::string designation_sha;  // the value the environment must carry
  std::string self_sha;
  std::string self_name;
  std::vector<std::string> sets = {"s0", "s1"};
  explicit SyntheticPrereg(const std::string& tag) {
    dir = fs::temp_directory_path() / ("sky-held-" + tag + "-" + std::to_string(::getpid()));
    fs::remove_all(dir);
    fs::create_directories(dir);
    ref_root = dir / "ref";
    model = dir / "model" / "semantics-model.tflite";
    sr_held::Sha256File(sr_held::SelfExe(), &self_sha);
    self_name = fs::path(sr_held::SelfExe()).filename().string();
  }
  ~SyntheticPrereg() { std::error_code ec; fs::remove_all(dir, ec); }
  std::string Id() const { return std::string(kGateId) + ".v" + std::to_string(kVersion); }
  void Write(const fs::path& path, const std::string& text) const {
    fs::create_directories(path.parent_path());
    std::ofstream(path, std::ios::binary) << text;
  }
  // A tiny recorded set: replay.txt, one data file and the SHA256SUMS that lists both (tools/sky/seal_sky_set.sh).
  // Returns {sha256 of replay.txt, sha256 of SHA256SUMS}.
  std::pair<std::string, std::string> MakeSet(const std::string& name) const {
    const std::string replay = "replay of " + name + "\n";
    const std::string data = "data of " + name;
    Write(ref_root / name / "replay.txt", replay);
    Write(ref_root / name / "cam" / "cam-0000.rgba", data);
    const std::string sums = HeldSha(replay) + "  replay.txt\n" + HeldSha(data) + "  cam/cam-0000.rgba\n";
    Write(ref_root / name / "SHA256SUMS", sums);
    return {HeldSha(replay), HeldSha(sums)};
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
    Write(model, "a model");
    const std::string model_sha = HeldSha("a model");
    std::string sealed_files;
    for (const std::string& name : sets) {
      const auto sh = MakeSet(name);
      sealed_files += (sealed_files.empty() ? "\"" : ", \"") + sh.first + "\", \"" + sh.second + "\"";
    }
    std::string tree, code;
    EXPECT_TRUE(sr_held::TreeSha(ref_root.string(), &tree, &code)) << code;
    if (mutate == 5) Write(ref_root / "s1" / "cam" / "cam-0000.rgba", "changed after registration");
    if (mutate == 11) Write(ref_root / "s0" / "unlisted.bin", "a file nobody registered");
    if (mutate == 14) Write(model, "another model");  // after its sha256 was registered
    std::string pins = mutate == 12 ? std::string() : "\"" + self_name + "\": \"" + (mutate == 6 ? std::string(64, '0') : self_sha) + "\", ";
    pins += "\"semantics-model.tflite\": \"" + model_sha + "\"";
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
  std::string Run(Log* log, std::vector<std::string>* out_sets) const {
    return Release(Request(), ref_root.string(), {sr_held::SelfExe(), model.string()}, log, out_sets);
  }
};

TEST(SkyHeld, TheRegistrationIsNamedByTheCallerAndNeverDefaulted) {
  std::map<std::string, std::string> env = {{"SR_SKY_PREREG_DIR", "/p"}, {"SR_SKY_GATE_ID", "g"},   {"SR_SKY_GATE_VERSION", "12"},
                                            {"SR_SKY_GATE_KIND", "main"}, {"SR_SKY_REVIEWER", std::string(64, 'a')}};
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
    env["SR_SKY_GATE_VERSION"] = bad;
    EXPECT_NE(ReadGateRequest(lookup, &g).find("SR_SKY_GATE_VERSION"), std::string::npos) << bad;
  }
  env.clear();  // nothing set: not one default
  EXPECT_NE(ReadGateRequest(lookup, &g).find("SR_SKY_GATE_ID"), std::string::npos);
}

TEST(SkyHeld, OnlyTheCompleteConsistentRegistrationIsReleased) {
  const char* const expect[] = {
      "",  // 0: complete
      "[REVIEW_MISSING]",
      "[DESIGNATION_SHA]",
      "[ENV_MISMATCH]",
      "[DESIGNATION_NOT_PERMITTED]",
      "[INPUT_MISMATCH]",      // 5: a data file changed after registration
      "[PIN_MISMATCH]",        // 6: this binary is not the registered one
      "[GATE_SHA_MISMATCH]",
      "[ALREADY_RUN]",
      "[DESIGNATION_PIN]",     // 9: manifest edited after the designation
      "[ENV_MISSING]",
      "[INPUT_MISMATCH]",      // 11: a file nobody registered
      "[PIN_UNREGISTERED]",    // 12: the manifest registers no pin for this binary
      "[INPUT_UNREGISTERED]",  // 13: data supplied, none registered
      "[PIN_MISMATCH]",        // 14: the model is not the registered one (checked before the marker)
  };
  for (int m = 0; m < 15; ++m) {
    SyntheticPrereg p(std::to_string(m));
    p.Build(m);
    Log log;
    std::vector<std::string> sets;
    const std::string held = p.Run(&log, &sets);
    const std::string marker = (p.dir / (p.Id() + ".first-run.main.marker")).string();
    if (m == 0) {
      EXPECT_EQ(held, "") << "the complete registration must be released";
      EXPECT_EQ(sets.size(), 2u);
      EXPECT_TRUE(fs::exists(marker)) << "released run takes the marker of ITS registration and kind";
      EXPECT_FALSE(log.empty()) << "the checks are logged";
      bool binary_line = false, pin_line = false;
      for (const auto& l : log) {
        binary_line = binary_line || (l.find("gate binary ") != std::string::npos && l.find(p.self_sha) != std::string::npos);
        pin_line = pin_line || (l.find("pin " + p.self_name) != std::string::npos && l.find(p.self_sha) != std::string::npos);
      }
      EXPECT_TRUE(binary_line) << "the log carries the gate binary path and sha256 (review F-10)";
      EXPECT_TRUE(pin_line) << "the log carries the pin comparison of the binary";
    } else {
      EXPECT_NE(held.find(expect[m]), std::string::npos) << "case " << m << ": " << held;
      EXPECT_FALSE(fs::exists(marker)) << "case " << m << ": a refusal is not a run";
    }
  }
}

TEST(SkyHeld, TheManifestIsNeverRequiredToChangeAfterTheReview) {
  // Review F-1 of the v1 run: a condition that asked for a line in the manifest after the review broke its pin. Here the
  // release leaves every registration file as it was (only the marker is new), and an unrelated NEW file next to the
  // manifest (a post-run note) changes nothing.
  SyntheticPrereg p("immutable");
  p.Build(0);
  std::map<std::string, std::string> before;
  for (const auto& e : fs::directory_iterator(p.dir)) {
    if (e.is_regular_file()) before[e.path().filename().string()] = HeldReadAll(e.path().string());
  }
  p.Write(p.dir / (p.Id() + ".post-run-notes.json"), "{\"note\": \"written after the run\"}");
  Log log;
  std::vector<std::string> sets;
  ASSERT_EQ(p.Run(&log, &sets), "");
  for (const auto& kv : before) {
    EXPECT_EQ(HeldReadAll((p.dir / kv.first).string()), kv.second) << kv.first << " was changed by the release";
  }
}

TEST(SkyHeld, AnotherGateIdOrVersionIsNotThisRegistration) {
  SyntheticPrereg p("other");
  p.Build(0);
  for (int variant = 0; variant < 3; ++variant) {
    GateRequest g = p.Request();
    if (variant == 0) g.gate_id = "sky-us020";  // the v1 name: nothing is registered under it here
    if (variant == 1) g.version = SyntheticPrereg::kVersion + 1;
    if (variant == 2) g.kind = "other";
    Log log;
    std::vector<std::string> sets;
    const std::string held = Release(g, p.ref_root.string(), {sr_held::SelfExe(), p.model.string()}, &log, &sets);
    EXPECT_NE(held.find("HELD ["), std::string::npos) << "variant " << variant << ": " << held;
  }
  for (const auto& e : fs::directory_iterator(p.dir)) {
    EXPECT_EQ(e.path().filename().string().find("first-run"), std::string::npos) << "no marker for another registration: " << e.path();
  }
}

TEST(SkyHeld, TheFirstRunMarkerIsExclusiveAndANewRunIsRefused) {
  SyntheticPrereg p("marker");
  p.Build(0);
  Log log;
  std::vector<std::string> sets;
  ASSERT_EQ(p.Run(&log, &sets), "");
  struct stat st;
  const fs::path marker = p.dir / (p.Id() + ".first-run.main.marker");
  ASSERT_EQ(::stat(marker.c_str(), &st), 0);
  EXPECT_EQ(st.st_mode & 0777, 0444u) << "read-only marker";
  Log log2;
  std::vector<std::string> sets2;
  const std::string again = p.Run(&log2, &sets2);
  EXPECT_NE(again.find("MARKER_EXISTS"), std::string::npos) << again;
  EXPECT_NE(again.find("this kind was run before"), std::string::npos) << again;
}

TEST(SkyHeld, TheRunRecordCarriesTheBinaryShaThatTheMarkerDoesNot) {
  SyntheticPrereg p("record");
  p.Build(0);
  Log log;
  std::vector<std::string> sets;
  ASSERT_EQ(p.Run(&log, &sets), "");
  const fs::path report = p.dir / "report";
  fs::create_directories(report);
  WriteRunRecord(report.string(), p.Request(), log);
  const std::string rec = HeldReadAll((report / "held-run-record.txt").string());
  EXPECT_NE(rec.find("gate_binary " + sr_held::SelfExe() + " sha256 " + p.self_sha), std::string::npos) << rec;
  EXPECT_NE(rec.find("designation_sha256 " + p.designation_sha), std::string::npos) << rec;
  EXPECT_NE(rec.find("first-run.main.marker sha256 "), std::string::npos) << rec;
  EXPECT_NE(rec.find("[SR-HELD]"), std::string::npos) << "the device log lines are copied";
  EXPECT_NE(rec.find("[SKY-HELD] set s1 replay.txt sha256="), std::string::npos) << "per set lines are copied";
}

TEST(SkyHeld, EverySealedSetIsVerifiedBeforeTheMarkerAndBeforeAnyMeasurement) {
  // A change in the SECOND set (a data file, replay.txt, the checksum list, an extra file) is found although the first set is
  // fine (the tree sha256 of the data root); no marker is taken, so nothing was measured. A directory that is not a recorded
  // set is refused by the structure check.
  for (int variant = 0; variant < 5; ++variant) {
    SyntheticPrereg p("seal" + std::to_string(variant));
    p.Build(0);
    const fs::path s1 = p.ref_root / "s1";
    if (variant == 0) p.Write(s1 / "cam" / "cam-0000.rgba", "tampered data");
    if (variant == 1) p.Write(s1 / "replay.txt", "tampered replay\n");
    if (variant == 2) p.Write(s1 / "SHA256SUMS", "00  replay.txt\n");
    if (variant == 3) p.Write(s1 / "extra.bin", "an unlisted file");
    if (variant == 4) p.Write(p.ref_root / "not-a-set" / "x.bin", "no replay.txt here");
    Log log;
    std::vector<std::string> sets;
    const std::string held = p.Run(&log, &sets);
    EXPECT_NE(held.find(variant == 4 ? "not a recorded set" : "[INPUT_MISMATCH]"), std::string::npos) << "variant " << variant << ": " << held;
    EXPECT_FALSE(fs::exists(p.dir / (p.Id() + ".first-run.main.marker"))) << "variant " << variant;
  }
}

TEST(SkyHeld, ReportModeAndTheReplayToolRefuseSealedSets) {
  SyntheticPrereg p("report");
  p.Build(0);
  std::string why;
  const std::string prereg = p.dir.string();
  const std::string repo = p.dir.string();
  // the registered data root (tree sha256) and a renamed copy of it
  EXPECT_TRUE(IsSealedGateSet(prereg, repo, "ref", p.ref_root.string(), &why)) << why;
  fs::copy(p.ref_root, p.dir / "copy-of-root", fs::copy_options::recursive);
  EXPECT_TRUE(IsSealedGateSet(prereg, repo, "copy-of-root", (p.dir / "copy-of-root").string(), &why)) << why;
  // one set, renamed: its replay.txt and SHA256SUMS have registered sha256 (held.sealed_file_sha256)
  fs::copy(p.ref_root / "s1", p.dir / "copy-of-s1", fs::copy_options::recursive);
  EXPECT_TRUE(IsSealedGateSet(prereg, repo, "renamed", (p.dir / "copy-of-s1").string(), &why)) << why;
  // only the checksum list identical (replay.txt changed)
  fs::copy(p.ref_root / "s1", p.dir / "copy2", fs::copy_options::recursive);
  p.Write(p.dir / "copy2" / "replay.txt", "different\n");
  EXPECT_TRUE(IsSealedGateSet(prereg, repo, "renamed2", (p.dir / "copy2").string(), &why)) << why;
  // only the replay.txt identical
  p.Write(p.dir / "copy3" / "replay.txt", "replay of s0\n");
  EXPECT_TRUE(IsSealedGateSet(prereg, repo, "renamed3", (p.dir / "copy3").string(), &why)) << why;
  // something unrelated is not sealed
  p.Write(p.dir / "dev-x" / "replay.txt", "a development set\n");
  EXPECT_FALSE(IsSealedGateSet(prereg, repo, "dev-x", (p.dir / "dev-x").string(), &why)) << why;
  // the decision does not depend on which gate id the registration has: a second registration of other data is consulted too
  {
    p.Write(p.dir / "second-data" / "x.bin", "data sealed by another registration");
    std::string tree, code;
    ASSERT_TRUE(sr_held::TreeSha((p.dir / "second-data").string(), &tree, &code)) << code;
    p.Write(p.dir / "another-gate.v9.manifest.json",
            "{\"gate_id\": \"another-gate\", \"version\": 9, \"held\": {\"inputs\": {\"main\": \"" + tree + "\"}}}");
    EXPECT_TRUE(IsSealedGateSet(prereg, repo, "second-data", (p.dir / "second-data").string(), &why)) << why;
    EXPECT_NE(why.find("another-gate.v9.manifest.json"), std::string::npos) << why;
    fs::remove(p.dir / "another-gate.v9.manifest.json");
  }
  // a registration that cannot be read: fail closed
  p.Write(p.dir / "broken.v1.manifest.json", "{not json");
  EXPECT_TRUE(IsSealedGateSet(prereg, repo, "dev-x", (p.dir / "dev-x").string(), &why)) << why;
  EXPECT_NE(why.find("fail closed"), std::string::npos) << why;
  fs::remove(p.dir / "broken.v1.manifest.json");
  // no registration at all: fail closed
  const fs::path empty = p.dir / "empty-prereg";
  fs::create_directories(empty);
  EXPECT_TRUE(IsSealedGateSet(empty.string(), repo, "dev-x", (p.dir / "dev-x").string(), &why)) << why;
  EXPECT_NE(why.find("fail closed"), std::string::npos) << why;
}

TEST(SkyHeld, ReportModeStillRefusesSetsSealedByAnOlderRegistrationList) {
  // v1 kept its sealed sets in <id>.v<N>.sealed.json (name, replay_sha256, sums_sha256, a root); those lists are still honoured.
  SyntheticPrereg p("legacy");
  fs::create_directories(p.dir);
  const auto sh = p.MakeSet("s0");
  p.Write(p.dir / "old-gate.v1.sealed.json",
          "{\"root\": \"" + p.ref_root.string() + "\", \"sets\": [{\"name\": \"s0\", \"replay_sha256\": \"" + sh.first +
              "\", \"sums_sha256\": \"" + sh.second + "\"}]}");
  std::string why;
  const std::string prereg = p.dir.string();
  EXPECT_TRUE(IsSealedGateSet(prereg, prereg, "s0", (p.ref_root / "s0").string(), &why)) << why;
  fs::copy(p.ref_root / "s0", p.dir / "copy-of-s0", fs::copy_options::recursive);
  EXPECT_TRUE(IsSealedGateSet(prereg, prereg, "renamed", (p.dir / "copy-of-s0").string(), &why)) << why;
  p.Write(p.ref_root / "zzz" / "replay.txt", "not a sealed name\n");
  EXPECT_TRUE(IsSealedGateSet(prereg, prereg, "zzz", (p.ref_root / "zzz").string(), &why)) << why;
  EXPECT_NE(why.find("sealed root"), std::string::npos) << why;
  p.Write(p.dir / "dev-x" / "replay.txt", "a development set\n");
  EXPECT_FALSE(IsSealedGateSet(prereg, prereg, "dev-x", (p.dir / "dev-x").string(), &why)) << why;
  fs::remove(p.dir / "old-gate.v1.sealed.json");  // list gone: fail closed
  EXPECT_TRUE(IsSealedGateSet(prereg, prereg, "dev-x", (p.dir / "dev-x").string(), &why)) << why;
  EXPECT_NE(why.find("fail closed"), std::string::npos) << why;
}

TEST(SkyGate, RecordedOriginalRunsVersusThisPipeline) {
  const std::string ref_root = Env("SR_SKY_REF_ROOT");
  const std::string report_dir = Env("SR_SKY_REPORT_DIR");
  const bool report_mode = Env("SR_SKY_GATE_MODE") == "report";
  std::vector<std::string> sets;
  Log log;
  GateRequest gate;
  ASSERT_FALSE(ref_root.empty()) << "SR_SKY_REF_ROOT (evidence reference directory) not set";
  ASSERT_FALSE(report_dir.empty()) << "SR_SKY_REPORT_DIR (new directory for the statistics) not set";
  ASSERT_FALSE(fs::exists(report_dir)) << "refusing to overwrite " << report_dir;
  // The model is read before the one-run marker is taken: a missing asset must not consume the run.
  std::vector<uint8_t> model_bytes;
  std::string err;
  const std::string model_path = std::string(SR_REPO_ROOT) + "/" + kSemanticsModelPath;
  ASSERT_TRUE(ReadFileBytes(model_path, &model_bytes, &err)) << err;
  if (report_mode) {
    // Report mode is for development sets only: a sealed gate set is refused (by content and, for older lists, name and
    // location), whichever registration sealed it.
    std::stringstream s(Env("SR_SKY_GATE_SETS"));
    std::string n;
    while (std::getline(s, n, ',')) {
      if (n.empty()) continue;
      std::string why;
      ASSERT_FALSE(IsSealedGateSet(SR_SKY_PREREG_DIR, SR_REPO_ROOT, n, ref_root + "/" + n, &why))
          << "HELD: report mode does not measure sealed gate sets: " << why;
      sets.push_back(n);
    }
    ASSERT_FALSE(sets.empty()) << "report mode needs SR_SKY_GATE_SETS";
  } else {
    const std::string request_error = ReadGateRequest(Env, &gate);
    ASSERT_TRUE(request_error.empty()) << request_error;
    // pins: this binary and the model (a model that differs from the registered one is refused before the marker)
    const std::string held = Release(gate, ref_root, {sr_held::SelfExe(), model_path}, &log, &sets);
    for (const auto& l : log) std::printf("%s\n", l.c_str());
    ASSERT_TRUE(held.empty()) << held;
  }
  fs::create_directories(report_dir);
  if (!report_mode) WriteRunRecord(report_dir, gate, log);

  // Mutation experiments (protocol 4) exist only in report mode: the replay of OUR pipeline is run with a changed
  // configuration or orientation, the recorded original is untouched. SR_SKY_MUTATION = beta=<deg> | invert=<0|1> |
  // edge=<value> | unseamless=1. The gate mode never reads this variable.
  ReplayOptions opt;
  opt.mode = ReplayMode::kInjectWorker;
  if (report_mode && !Env("SR_SKY_MUTATION").empty()) {
    const std::string m = Env("SR_SKY_MUTATION");
    const auto eq = m.find('=');
    ASSERT_NE(eq, std::string::npos) << "SR_SKY_MUTATION must be key=value";
    const std::string key = m.substr(0, eq);
    const double value = std::strtod(m.c_str() + eq + 1, nullptr);
    if (key == "beta") {
      opt.orientation_beta_offset_deg = value;
    } else if (key == "invert" || key == "edge") {
      opt.override_config = true;
      // keep the other field of the recorded configuration
      opt.invert = key == "invert" ? value != 0 : false;
      opt.edge_smoothness = key == "edge" ? value : 0.0;
    } else if (key == "unseamless") {
      opt.seamless = value == 0;
    } else {
      FAIL() << "unknown mutation " << key;
    }
  }
  int failed_sets = 0;
  for (const std::string& set_name : sets) {
    const std::string dir = ref_root + "/" + set_name;
    SkyRecord rec;
    ASSERT_TRUE(SkyRecord::Load(dir, &rec, &err)) << err;
    Stats st;
    ASSERT_TRUE(Measure(rec, model_bytes, opt, &st, &err)) << set_name << ": " << err;
    Evaluate(&st);
    const std::string json = ToJson(set_name, st);
    std::ofstream(report_dir + "/" + set_name + ".json") << json;
    std::printf("[SKY %s] %s: %s\n", report_mode ? "REPORT" : "GATE", set_name.c_str(),
                st.failures.empty() ? "all rules within the registered tolerances" : "FAIL");
    for (const auto& f : st.failures) std::printf("  - %s\n", f.c_str());
    failed_sets += st.failures.empty() ? 0 : 1;
  }
  if (!report_mode) {
    EXPECT_EQ(failed_sets, 0) << "see " << report_dir;
  }
}

}  // namespace
}  // namespace slam_refact::sky
